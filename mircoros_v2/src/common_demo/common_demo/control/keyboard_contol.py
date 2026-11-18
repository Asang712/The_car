#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Vector3

import sys, select, termios, tty
import time
import threading

# terminal settings, initialized in main() (stdin must be a TTY)
settings = None

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)


q/z : increase/decrease both speeds by 0.01  (linear: 0~2.0 m/s, angular: 0~5.0 rad/s)
w/x : increase/decrease only linear speed by 0.01
e/c : increase/decrease only angular speed by 0.01

PTZ Control:
   1/2 : PTZ x axis +/- (range -90 ~ 90)
   3/4 : PTZ y axis +/- (range -90 ~ 20)
   0   : reset PTZ to center (x:0, y:-45)

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings = {
    'q': (+0.01, +0.01),
    'z': (-0.01, -0.01),
    'w': (+0.01,  0.00),
    'x': (-0.01,  0.00),
    'e': ( 0.00, +0.01),
    'c': ( 0.00, -0.01),
}

# Speed limits and step size
SPEED_MIN, SPEED_MAX = 0.0, 2.0
TURN_MIN, TURN_MAX   = 0.0, 5.0

class KeyBoardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # Declare parameters with defaults
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('ptz_step', 2.0)   # degrees per key press
        self.declare_parameter('cmd_publish_period_s', 0.02)
        self.declare_parameter('servo_publish_period_s', 0.01)
        self.declare_parameter('key_timeout_s', 0.15)          # fast release — repeat rate is ~30+ Hz
        self.declare_parameter('key_timeout_first_s', 0.80)    # covers auto-repeat initial delay (~0.5s)
        
        # Get parameters
        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.ptz_step = self.get_parameter('ptz_step').value
        self.cmd_publish_period_s = self.get_parameter('cmd_publish_period_s').value
        self.servo_publish_period_s = self.get_parameter('servo_publish_period_s').value
        self.key_timeout_s = self.get_parameter('key_timeout_s').value
        self.key_timeout_first_s = self.get_parameter('key_timeout_first_s').value
        
        # PTZ limits and center position (deg)
        self.ptz_x_bounds = (-90.0, 90.0)
        self.ptz_y_bounds = (-90.0, 20.0)
        self.ptz_center = (0.0, -45.0)
        
        # Current PTZ angles, start from center position
        self.ptz_x = self.ptz_center[0]
        self.ptz_y = self.ptz_center[1]
        
        self._held_move = None        # current (x,y,z,th) or None
        self._last_key_time = None

        self._awaiting_first_repeat = False
        self._state_lock = threading.Lock()
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.cmd_ptz_angle_pub = self.create_publisher(Vector3, '/cmd_ptz_angle', 10)
        
        # Timers — continuous publish like joy_control
        self.create_timer(self.cmd_publish_period_s, self._cmd_timer_callback)
        self.create_timer(self.servo_publish_period_s, self._servo_timer_callback)
    
    def handle_key(self, key):
        """Process one key event from the reader thread."""
        now = time.monotonic()
        with self._state_lock:
            if key in moveBindings:
                new_move = moveBindings[key]
                if self._held_move is None or self._held_move != new_move:
                    # first event for this direction — enter long-timeout phase
                    self._awaiting_first_repeat = True
                else:
                    # same direction, confirmed repeat — switch to fast timeout
                    self._awaiting_first_repeat = False
                self._held_move = new_move
                self._last_key_time = now
            else:
                # any non-move key releases the current motion
                self._held_move = None
                self._awaiting_first_repeat = False
                self._last_key_time = now
        
        if key in speedBindings.keys():
            d_speed, d_turn = speedBindings[key]
            self.speed = max(SPEED_MIN, min(SPEED_MAX, round(self.speed + d_speed, 2)))
            self.turn  = max(TURN_MIN,  min(TURN_MAX,  round(self.turn + d_turn, 2)))
            print(vels(self.speed, self.turn))
        elif key == '1':
            # PTZ x axis increase
            self.adjust_ptz(self.ptz_step, 0.0)
        elif key == '2':
            # PTZ x axis decrease
            self.adjust_ptz(-self.ptz_step, 0.0)
        elif key == '3':
            # PTZ y axis increase
            self.adjust_ptz(0.0, self.ptz_step)
        elif key == '4':
            # PTZ y axis decrease
            self.adjust_ptz(0.0, -self.ptz_step)
        elif key == '0':
            # Reset PTZ to center position
            self.reset_ptz()
    
    def _held_move_active(self):

        if self._held_move is None or self._last_key_time is None:
            return False
        timeout = self.key_timeout_first_s if self._awaiting_first_repeat else self.key_timeout_s
        return (time.monotonic() - self._last_key_time) <= timeout
    
    def _cmd_timer_callback(self):
        """Publish /cmd_vel periodically; zero velocity when no key held."""
        with self._state_lock:
            move = self._held_move if self._held_move_active() else None
        
        twist = Twist()
        if move is not None:
            twist.linear.x = move[0] * self.speed
            twist.linear.y = move[1] * self.speed
            twist.linear.z = move[2] * self.speed
            twist.angular.z = move[3] * self.turn
        self.cmd_vel_pub.publish(twist)
    
    def _servo_timer_callback(self):
        """Publish current PTZ angle periodically like joy_control."""
        self.publish_ptz()
    
    def publish_ptz(self):
        """Publish current PTZ target angle (deg)"""
        msg = Vector3()
        msg.x = float(self.ptz_x)
        msg.y = float(self.ptz_y)
        msg.z = 0.0
        self.cmd_ptz_angle_pub.publish(msg)
    
    def adjust_ptz(self, dx, dy):
        """Adjust PTZ angle with bounds checking"""
        self.ptz_x = max(self.ptz_x_bounds[0], min(self.ptz_x_bounds[1], self.ptz_x + dx))
        self.ptz_y = max(self.ptz_y_bounds[0], min(self.ptz_y_bounds[1], self.ptz_y + dy))
        self.publish_ptz()
    
    def reset_ptz(self):
        """Reset PTZ to center position"""
        self.ptz_x, self.ptz_y = self.ptz_center
        self.publish_ptz()

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def key_reader_loop(node):

	while rclpy.ok():
		tty.setraw(sys.stdin.fileno())
		try:
			rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
			if rlist:
				key = sys.stdin.read(1)
			else:
				key = ''
		except Exception:
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
			break
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		if not key:
			continue
		if key == '\x03':  # CTRL-C
			rclpy.shutdown()
			break
		node.handle_key(key)

def vels(speed, turn):
	return "currently:\tspeed %.2f\tturn %.2f " % (speed, turn)

def main(args=None):
	global settings
	if args is None:
		args = sys.argv

	# keyboard control requires a real terminal (stdin as TTY)
	if not sys.stdin.isatty():
		print("[keyboard_control] stdin is not a TTY, keyboard control needs a real terminal.")
		print("[keyboard_control] Run it with: ros2 run common_demo keyboard_control")
		sys.exit(1)
	settings = termios.tcgetattr(sys.stdin)

	rclpy.init(args=args)
	keyboard_node = KeyBoardControlNode()
	
	try:
		print(msg)
		print(vels(keyboard_node.speed, keyboard_node.turn))
		# Move PTZ to center position on startup
		key_reader = threading.Thread(
			target=key_reader_loop, args=(keyboard_node,), daemon=True)
		key_reader.start()
		rclpy.spin(keyboard_node)

	except KeyboardInterrupt:
		pass

	except Exception as e:
		keyboard_node.get_logger().error(f"Error: {e}")

	finally:
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
		keyboard_node.cmd_vel_pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		
		keyboard_node.destroy_node()
		rclpy.try_shutdown()

