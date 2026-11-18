#!/usr/bin/env python3

from __future__ import annotations
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt16
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

AXIS_LY = 1        # Left Stick vertical   (up=-1, down=+1)
AXIS_RX = 2        # Right Stick horizontal (left=-1, right=+1)
AXIS_DPAD_H = 6    # D-Pad horizontal (left=-1, right=+1)
AXIS_DPAD_V = 7    # D-Pad vertical   (up=-1, down=+1)
BTN_A = 0
BTN_B = 1
BTN_X = 3
BTN_Y = 4
BTN_L1 = 6
BTN_R1 = 7
BTN_L2 = 8
BTN_R2 = 9
BTN_SELECT = 10
BTN_START = 11

# ------------------------------------------------------------------
# Fixed control constants (no longer ROS parameters)
# ------------------------------------------------------------------

STICK_DEADZONE = 0.15
LINEAR_CURVE_POWER = 2.0    # square curve for linear
ANGULAR_CURVE_POWER = 3.0   # cubic curve for angular
START_BEEP_UNITS = 3        # START hold beep value (3 x 10 ms)

# PTZ servo fixed limits (hardware-defined, matches firmware)
SERVO_S1_DEFAULT = 0.0
SERVO_S1_MIN = -90.0
SERVO_S1_MAX = 90.0
SERVO_S2_DEFAULT = -45.0
SERVO_S2_MIN = -90.0
SERVO_S2_MAX = 20.0
SERVO_S1_DIR_SIGN = -1.0
SERVO_S2_DIR_SIGN = -1.0

# 5-speed gear tables (matches firmware)
LINEAR_GEAR_TABLE = [0.20, 0.30, 0.50, 0.75, 1.00]   # m/s,   G1–G5
ANGULAR_GEAR_TABLE = [0.50, 1.50, 2.50, 3.75, 5.00]  # rad/s, G1–G5
DEFAULT_GEAR_INDEX = 1  # G2
MAX_LINEAR_SPEED = LINEAR_GEAR_TABLE[-1]
MAX_ANGULAR_SPEED = ANGULAR_GEAR_TABLE[-1]


class EdgeDetector:
    """Rising-edge detector for digital button levels."""

    def __init__(self) -> None:
        self._last_level = 0

    def update(self, level: int) -> bool:
        level = 1 if int(level) != 0 else 0
        rising = self._last_level == 0 and level == 1
        self._last_level = level
        return rising


@dataclass
class ServoAxisState:
    angle: float
    velocity_dps: float = 0.0


class SpeedBeepSequencer:
    """Timed beep pattern sequencer for speed-adjust feedback.

    Tick unit = beep publish period (20 ms).
    Single beep: 60 ms = 3 ticks.
    Double beep: 60 ms beep + 80 ms gap + 60 ms beep.
    """

    def __init__(self) -> None:
        self._pattern: list[tuple[int, int]] = []  # [(beep_ticks, gap_ticks), ...]
        self._phase_beep = 0    # remaining beep ticks in current phase
        self._phase_gap = 0     # remaining gap ticks in current phase
        self._in_gap = False

    def trigger_single(self) -> None:
        """Start a single 60 ms beep."""
        self._pattern.append((3, 0))

    def trigger_double(self) -> None:
        """Start a double beep (60 ms + 80 ms gap + 60 ms)."""
        self._pattern.append((3, 4))   # 60ms beep + 80ms gap
        self._pattern.append((3, 0))   # 60ms beep, no gap after

    @property
    def active(self) -> bool:
        return self._phase_beep > 0 or self._phase_gap > 0 or len(self._pattern) > 0

    def value(self) -> int:
        """Current beep_time value (in 10 ms units), or 0 if in gap."""
        if self._in_gap or self._phase_beep <= 0:
            return 0
        return 6  # 60 ms = 6 × 10 ms

    def tick(self) -> None:
        """Advance one 20 ms tick."""
        if self._in_gap:
            self._phase_gap -= 1
            if self._phase_gap <= 0:
                self._in_gap = False
                self._load_next()
            return

        if self._phase_beep > 0:
            self._phase_beep -= 1
            if self._phase_beep <= 0:
                # Beep phase ended — start gap if one is defined for this phase
                if self._phase_gap > 0:
                    self._in_gap = True
                else:
                    self._load_next()
            return

        # Nothing active — try loading next pattern
        self._load_next()

    def _load_next(self) -> None:
        if self._pattern:
            self._phase_beep, self._phase_gap = self._pattern.pop(0)
            self._in_gap = False
        else:
            self._phase_beep = 0
            self._phase_gap = 0
            self._in_gap = False

    def reset(self) -> None:
        """Cancel all pending beep patterns."""
        self._pattern.clear()
        self._phase_beep = 0
        self._phase_gap = 0
        self._in_gap = False


class MicroRosV2UsbJoyControl(Node):
    def __init__(self) -> None:
        super().__init__("microros_v2_usb_joy_control_node")

        # --- Parameters (periods / soft-stop / servo dynamics) ---
        self.declare_parameter("cmd_publish_period_s", 0.02)
        self.declare_parameter("servo_publish_period_s", 0.01)
        self.declare_parameter("beep_publish_period_s", 0.02)
        self.declare_parameter("control_timeout_s", 0.30)

        self.declare_parameter("soft_stop_linear", 0.80)    # m/s²
        self.declare_parameter("soft_stop_angular", 2.00)   # rad/s²
        self.declare_parameter("soft_stop_max_time", 0.70)  # s

        self.declare_parameter("servo_speed_dps", 180.0)
        self.declare_parameter("servo_accel_dps2", 360.0)

        self.cmd_publish_period_s = float(self.get_parameter("cmd_publish_period_s").value)
        self.servo_publish_period_s = float(self.get_parameter("servo_publish_period_s").value)
        self.beep_publish_period_s = float(self.get_parameter("beep_publish_period_s").value)
        self.control_timeout_s = float(self.get_parameter("control_timeout_s").value)

        self.soft_stop_linear = float(self.get_parameter("soft_stop_linear").value)
        self.soft_stop_angular = float(self.get_parameter("soft_stop_angular").value)
        self.soft_stop_max_time = float(self.get_parameter("soft_stop_max_time").value)

        self.servo_speed_dps = float(self.get_parameter("servo_speed_dps").value)
        self.servo_accel_dps2 = float(self.get_parameter("servo_accel_dps2").value)

        # --- Control state ---
        self._latest_joy = Joy()
        self._last_joy_time = self.get_clock().now()
        self._has_joy = False
        self._joy_control_enabled = True

        # --- Speed profile ---
        self._linear_gear_index = DEFAULT_GEAR_INDEX
        self._angular_gear_index = DEFAULT_GEAR_INDEX

        # --- Current output velocity (for soft-stop tracking) ---
        self._current_linear_vel = 0.0
        self._current_angular_vel = 0.0
        self._stop_elapsed = 0.0  # seconds in stop state

        # --- Edge detectors ---
        self._select_edge = EdgeDetector()
        self._linear_up_edge = EdgeDetector()
        self._linear_down_edge = EdgeDetector()
        self._angular_up_edge = EdgeDetector()
        self._angular_down_edge = EdgeDetector()

        # --- Servo state (fixed constants) ---
        self._servo_s1 = ServoAxisState(angle=SERVO_S1_DEFAULT)
        self._servo_s2 = ServoAxisState(angle=SERVO_S2_DEFAULT)

        # --- Beep state ---
        self._speed_beep = SpeedBeepSequencer()
        self._joy_toggle_beep_ticks = 0  # SELECT enable/disable beep feedback

        # --- Subscription ---
        self._joy_sub = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        # --- Publishers (BEST_EFFORT for firmware-facing topics) ---
        self.cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 5)
        self._servo_angle_pub = self.create_publisher(Vector3, "/cmd_ptz_angle", 5)
        self._beep_pub = self.create_publisher(UInt16, "/beep", 5)

        # --- Timers ---
        self._cmd_timer = self.create_timer(self.cmd_publish_period_s, self._cmd_timer_callback)
        self._servo_timer = self.create_timer(self.servo_publish_period_s, self._servo_timer_callback)
        self._beep_timer = self.create_timer(self.beep_publish_period_s, self._beep_timer_callback)



    # ------------------------------------------------------------------
    # Safe Joy accessors
    # ------------------------------------------------------------------

    @staticmethod
    def _axis(joy: Joy, index: int) -> float:
        return float(joy.axes[index]) if len(joy.axes) > index else 0.0

    @staticmethod
    def _button(joy: Joy, index: int) -> int:
        return int(joy.buttons[index]) if len(joy.buttons) > index else 0

    # ------------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(value, hi))

    @staticmethod
    def _move_toward(current: float, target: float, max_delta: float) -> float:
        if current < target:
            return min(current + max_delta, target)
        if current > target:
            return max(current - max_delta, target)
        return current

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        """Zero within +/-deadzone, else rescale so deadzone→0, +/-1→+/-1."""
        if abs(value) < deadzone or deadzone >= 1.0:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return (value - sign * deadzone) / (1.0 - deadzone)

    @staticmethod
    def _curve(value: float, power: float) -> float:
        """sign(x) x |x|^power.  Input must be deadzone-rescaled to +/-1."""
        if value == 0.0:
            return 0.0
        return (abs(value) ** power) * (1.0 if value > 0 else -1.0)

    # ------------------------------------------------------------------
    # Joy callback — cache + SELECT toggle + gear adjust
    # ------------------------------------------------------------------

    def _joy_callback(self, msg: Joy) -> None:
        self._latest_joy = msg
        self._last_joy_time = self.get_clock().now()
        self._has_joy = True

        # SELECT toggle works regardless of enable state
        if self._select_edge.update(self._button(msg, BTN_SELECT)):
            self._joy_control_enabled = not self._joy_control_enabled
            self.get_logger().info(f"joystick control enabled -> {self._joy_control_enabled}")
            if self._joy_control_enabled:
                self._joy_toggle_beep_ticks = 12   # enable:  12 x 20ms ≈  250ms
            else:
                self._joy_toggle_beep_ticks = 50   # disable: 50 x 20ms = 1000ms

        if not self._joy_control_enabled:
            return

        # --- Speed gear adjustment (edge-triggered, 5 fixed gears) ---
        if self._linear_up_edge.update(self._button(msg, BTN_L1)):
            if self._linear_gear_index < 4:
                self._linear_gear_index += 1
                self.get_logger().info(
                    f"linear gear -> G{self._linear_gear_index+1} "
                    f"({LINEAR_GEAR_TABLE[self._linear_gear_index]:.2f} m/s)"
                )
                self._speed_beep.trigger_single()
            else:
                self.get_logger().info(
                    f"linear gear at max G5 ({LINEAR_GEAR_TABLE[4]:.2f} m/s)"
                )
                self._speed_beep.trigger_double()

        if self._linear_down_edge.update(self._button(msg, BTN_L2)):
            if self._linear_gear_index > 0:
                self._linear_gear_index -= 1
                self.get_logger().info(
                    f"linear gear -> G{self._linear_gear_index+1} "
                    f"({LINEAR_GEAR_TABLE[self._linear_gear_index]:.2f} m/s)"
                )
                self._speed_beep.trigger_single()
            else:
                self.get_logger().info(
                    f"linear gear at min G1 ({LINEAR_GEAR_TABLE[0]:.2f} m/s)"
                )
                self._speed_beep.trigger_double()

        if self._angular_up_edge.update(self._button(msg, BTN_R1)):
            if self._angular_gear_index < 4:
                self._angular_gear_index += 1
                self.get_logger().info(
                    f"angular gear -> G{self._angular_gear_index+1} "
                    f"({ANGULAR_GEAR_TABLE[self._angular_gear_index]:.2f} rad/s)"
                )
                self._speed_beep.trigger_single()
            else:
                self.get_logger().info(
                    f"angular gear at max G5 ({ANGULAR_GEAR_TABLE[4]:.2f} rad/s)"
                )
                self._speed_beep.trigger_double()

        if self._angular_down_edge.update(self._button(msg, BTN_R2)):
            if self._angular_gear_index > 0:
                self._angular_gear_index -= 1
                self.get_logger().info(
                    f"angular gear -> G{self._angular_gear_index+1} "
                    f"({ANGULAR_GEAR_TABLE[self._angular_gear_index]:.2f} rad/s)"
                )
                self._speed_beep.trigger_single()
            else:
                self.get_logger().info(
                    f"angular gear at min G1 ({ANGULAR_GEAR_TABLE[0]:.2f} rad/s)"
                )
                self._speed_beep.trigger_double()

    # ------------------------------------------------------------------
    # Validity check — /joy freshness + enable state
    # ------------------------------------------------------------------

    def _joy_valid(self) -> bool:
        age_s = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        return self._has_joy and age_s <= self.control_timeout_s and self._joy_control_enabled

    # ------------------------------------------------------------------
    # Motion — stick (analog curve) with D-Pad digital fallback
    # ------------------------------------------------------------------

    def _compute_target_velocity(self, joy: Joy) -> tuple[float, float, bool]:
        """Returns (target_linear, target_angular, stop).

        Car convention: linear.x < 0 → FORWARD, angular.z < 0 → LEFT TURN.
        Joy convention: LY up=-1, RX left=-1 → signs pass through directly.
        """
        linear_speed = LINEAR_GEAR_TABLE[self._linear_gear_index]
        angular_speed = ANGULAR_GEAR_TABLE[self._angular_gear_index]

        # Linear: Left Stick LY, D-Pad V fallback
        ly_raw = self._axis(joy, AXIS_LY)
        if abs(ly_raw) > STICK_DEADZONE:
            ly = self._apply_deadzone(ly_raw, STICK_DEADZONE)
            target_linear = self._curve(ly, LINEAR_CURVE_POWER) * linear_speed
        else:
            dpad_v = self._axis(joy, AXIS_DPAD_V)
            if dpad_v < -0.5:      # D-Pad up → forward (linear.x < 0)
                target_linear = -linear_speed
            elif dpad_v > 0.5:     # D-Pad down → backward
                target_linear = linear_speed
            else:
                target_linear = 0.0

        # Angular: Right Stick RX, D-Pad H fallback
        rx_raw = self._axis(joy, AXIS_RX)
        if abs(rx_raw) > STICK_DEADZONE:
            rx = self._apply_deadzone(rx_raw, STICK_DEADZONE)
            target_angular = self._curve(rx, ANGULAR_CURVE_POWER) * angular_speed
        else:
            dpad_h = self._axis(joy, AXIS_DPAD_H)
            if dpad_h < -0.5:      # D-Pad left → left turn (angular.z < 0)
                target_angular = -angular_speed
            elif dpad_h > 0.5:     # D-Pad right → right turn
                target_angular = angular_speed
            else:
                target_angular = 0.0

        stop = target_linear == 0.0 and target_angular == 0.0
        return target_linear, target_angular, stop

    # ------------------------------------------------------------------
    # CmdVel timer — target velocity + soft-stop
    # ------------------------------------------------------------------

    def _cmd_timer_callback(self) -> None:
        if not self._joy_control_enabled:
            return

        dt = self.cmd_publish_period_s
        valid = self._joy_valid()

        if valid:
            target_linear, target_angular, stop = self._compute_target_velocity(self._latest_joy)
        else:
            target_linear, target_angular, stop = 0.0, 0.0, True

        if valid and not stop:
            # --- Motion active: snap to target (instant response) ---
            self._stop_elapsed = 0.0
            self._current_linear_vel = target_linear
            self._current_angular_vel = target_angular
        else:
            # --- Soft-stop ---
            self._stop_elapsed += dt
            if self._stop_elapsed > self.soft_stop_max_time:
                # Exceeded max stop time → hard stop
                self._current_linear_vel = 0.0
                self._current_angular_vel = 0.0
            else:
                self._current_linear_vel = self._move_toward(
                    self._current_linear_vel, 0.0, self.soft_stop_linear * dt
                )
                self._current_angular_vel = self._move_toward(
                    self._current_angular_vel, 0.0, self.soft_stop_angular * dt
                )

        twist = Twist()
        twist.linear.x = self._clamp(self._current_linear_vel, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        twist.linear.y = 0.0
        twist.angular.z = self._clamp(self._current_angular_vel, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        self._cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Servo timer — face buttons + acceleration-limited interpolation
    # ------------------------------------------------------------------

    def _compute_servo_direction(self, joy: Joy) -> tuple[int, int]:
        """X → S1 +1, B → S1 -1; Y → S2 -1, A → S2 +1; opposite pair cancels."""
        x_on = self._button(joy, BTN_X) == 1
        b_on = self._button(joy, BTN_B) == 1
        y_on = self._button(joy, BTN_Y) == 1
        a_on = self._button(joy, BTN_A) == 1

        s1 = 1 if (x_on and not b_on) else (-1 if (b_on and not x_on) else 0)
        s2 = -1 if (y_on and not a_on) else (1 if (a_on and not y_on) else 0)
        return s1, s2

    def _update_servo_axis(
        self, axis: ServoAxisState, direction: int,
        direction_sign: float, min_angle: float, max_angle: float, dt: float,
    ) -> None:
        if direction == 0:
            axis.velocity_dps = 0.0
            return

        target_velocity = float(direction) * direction_sign * self.servo_speed_dps
        velocity_delta = self.servo_accel_dps2 * dt
        axis.velocity_dps = self._move_toward(axis.velocity_dps, target_velocity, velocity_delta)
        axis.angle += axis.velocity_dps * dt
        axis.angle = self._clamp(axis.angle, min_angle, max_angle)

        if axis.angle <= min_angle or axis.angle >= max_angle:
            axis.velocity_dps = 0.0

    def _servo_timer_callback(self) -> None:
        if not self._joy_control_enabled:
            return

        if self._joy_valid():
            s1_dir, s2_dir = self._compute_servo_direction(self._latest_joy)
        else:
            s1_dir, s2_dir = 0, 0

        dt = self.servo_publish_period_s
        self._update_servo_axis(
            self._servo_s1, s1_dir, SERVO_S1_DIR_SIGN, SERVO_S1_MIN, SERVO_S1_MAX, dt,
        )
        self._update_servo_axis(
            self._servo_s2, s2_dir, SERVO_S2_DIR_SIGN, SERVO_S2_MIN, SERVO_S2_MAX, dt,
        )

        servo_msg = Vector3()
        servo_msg.x = float(self._servo_s1.angle)
        servo_msg.y = float(self._servo_s2.angle)
        servo_msg.z = 0.0
        self._servo_angle_pub.publish(servo_msg)

    # ------------------------------------------------------------------
    # Beeper timer
    # ------------------------------------------------------------------

    def _beep_timer_callback(self) -> None:
        beep_msg = UInt16()

        # SELECT toggle beep is independent of joystick enable state,
        # so we process it before the enabled guard.
        if self._joy_toggle_beep_ticks > 0:
            beep_msg.data = 50  # 50 units x 10ms = 500ms pulses
            self._joy_toggle_beep_ticks -= 1
            self._beep_pub.publish(beep_msg)
            if self._joy_toggle_beep_ticks == 0:
                # Send explicit stop after last tick to cut off firmware beep cleanly
                self._beep_pub.publish(UInt16(data=0))
            return

        if not self._joy_control_enabled:
            return

        # Priority: START continuous beep > speed-adjust feedback beep
        start_beep = (
            START_BEEP_UNITS
            if self._joy_valid() and self._button(self._latest_joy, BTN_START) == 1
            else 0
        )
        if start_beep > 0:
            beep_msg.data = start_beep
            self._speed_beep.reset()  # suppress pending speed beeps
        elif self._speed_beep.active:
            self._speed_beep.tick()
            beep_msg.data = self._speed_beep.value()
        else:
            beep_msg.data = 0

        self._beep_pub.publish(beep_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MicroRosV2UsbJoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
