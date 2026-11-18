# for patrol
# math
import math
from math import radians, copysign, sqrt, pow
from math import pi
import numpy as np

# rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# tf
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point

# msg
from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt16

# others
import PyKDL # type: ignore

RAD2DEG = 180 / math.pi

class YahboomCarPatrol(Node):
    def __init__(self, name):
        super().__init__(name)
        self.moving = True
        self.command_src = "finish"
        self.front_warning = 0
        self.SetLoop = False
        self.Linear = 0.5
        self.Angular = 1.0
        self.Length = 1.0  # 1.0
        self.Angle = 360.0
        self.LineScaling = 1.1
        self.RotationScaling = 1.0
        self.LineTolerance = 0.04
        self.RotationTolerance = 3.0
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.error = 0.0
        self.distance = 0.0
        self.last_angle = 0.0
        self.delta_angle = 0.0
        self.turn_angle = 0.0
        # create publisher
        self.pub_cmdVel = self.create_publisher(Twist, "/cmd_vel", 5)
        # create subscriber
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.LaserScanCallback, qos_profile_sensor_data
        )
        # 创建蜂鸣器发布者 / Create a publisher for the buzzer
        self.pub_beep = self.create_publisher(UInt16, "beep", 10)
        # create TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # laser -> base_link TF state for obstacle detection
        self.laser_frame = ""
        self.tf_ready = False
        self.last_tf_log_time = 0.0
        self.tf_log_interval = 3.0
        # declare param
        self.declare_parameter("odom_frame", "odom")
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.declare_parameter("base_frame", "base_footprint")
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.declare_parameter("circle_adjust", 2.0)
        self.circle_adjust = (
            self.get_parameter("circle_adjust").get_parameter_value().double_value
        )
        self.declare_parameter("Switch", False)
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value
        self.declare_parameter("Command", "Square")
        self.Command = self.get_parameter("Command").get_parameter_value().string_value
        self.declare_parameter("Set_loop", False)
        self.Set_loop = self.get_parameter("Set_loop").get_parameter_value().bool_value
        self.declare_parameter("ResponseDist", 0.30)
        self.ResponseDist = (
            self.get_parameter("ResponseDist").get_parameter_value().double_value
        )
        self.declare_parameter("LaserAngle", 60.0)
        self.LaserAngle = (
            self.get_parameter("LaserAngle").get_parameter_value().double_value
        )
        
        self.declare_parameter("Linear", 0.1)
        self.Linear = self.get_parameter("Linear").get_parameter_value().double_value
        self.declare_parameter("Angular", 0.4)
        self.Angular = self.get_parameter("Angular").get_parameter_value().double_value
        self.declare_parameter("Length", 1.0)
        self.Length = self.get_parameter("Length").get_parameter_value().double_value
        self.declare_parameter("LineTolerance", 0.04)
        self.LineTolerance = (
            self.get_parameter("LineTolerance").get_parameter_value().double_value
        )
        self.declare_parameter("RotationTolerance", 3.0)
        self.RotationTolerance = (
            self.get_parameter("RotationTolerance").get_parameter_value().double_value
        )
        self.declare_parameter("RotationScaling", 1.0)
        self.RotationScaling = (
            self.get_parameter("RotationScaling").get_parameter_value().double_value
        )

        # create a timer
        self.timer = self.create_timer(0.5, self.on_timer)
        self.index = 0

    def on_timer(self):
        # print("self.error: ",self.error)
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value
        self.Command = self.get_parameter("Command").get_parameter_value().string_value
        self.Set_loop = self.get_parameter("Set_loop").get_parameter_value().bool_value
        self.ResponseDist = (
            self.get_parameter("ResponseDist").get_parameter_value().double_value
        )
        self.Linear = self.get_parameter("Linear").get_parameter_value().double_value
        self.Angular = self.get_parameter("Angular").get_parameter_value().double_value
        self.Length = self.get_parameter("Length").get_parameter_value().double_value
        self.LaserAngle = (
            self.get_parameter("LaserAngle").get_parameter_value().double_value
        )
        self.LineTolerance = (
            self.get_parameter("LineTolerance").get_parameter_value().double_value
        )
        self.RotationTolerance = (
            self.get_parameter("RotationTolerance").get_parameter_value().double_value
        )
        self.RotationScaling = (
            self.get_parameter("RotationScaling").get_parameter_value().double_value
        )

        index = 0

        if self.Switch == True:
            index = 0
            self.get_logger().info("Switch True")
            if self.Command == "LengthTest":
                self.command_src = "LengthTest"
                self.get_logger().info("LengthTest")
                advancing = self.advancing(self.Length)
                if advancing == True:
                    self.Switch = rclpy.parameter.Parameter(
                        "Switch", rclpy.Parameter.Type.BOOL, False
                    )
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)
                    self.Command = rclpy.parameter.Parameter(
                        "Command", rclpy.Parameter.Type.STRING, "finish"
                    )
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)

            elif self.Command == "Circle":
                self.command_src = "Circle"
                spin = self.Spin(360)
                if spin == True:
                    self.get_logger().info("spin done")
                    # self.Command = "finish"
                    self.Switch = rclpy.parameter.Parameter(
                        "Switch", rclpy.Parameter.Type.BOOL, False
                    )
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)
                    self.Command = rclpy.parameter.Parameter(
                        "Command", rclpy.Parameter.Type.STRING, "finish"
                    )
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)

            elif self.Command == "Square":
                self.command_src = "Square"
                square = self.Square()
                if square == True:
                    self.Command = rclpy.parameter.Parameter(
                        "Command", rclpy.Parameter.Type.STRING, "finish"
                    )
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)

            elif self.Command == "Triangle":
                self.command_src = "Triangle"
                triangle = self.Triangle()
                if triangle == True:
                    self.Command = rclpy.parameter.Parameter(
                        "Command", rclpy.Parameter.Type.STRING, "finish"
                    )
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)

        else:
            self.pub_cmdVel.publish(Twist())
            self.pub_beep.publish(UInt16(data=0))            
            self.get_logger().info("Switch False")
            if self.Command == "finish":
                self.get_logger().info("finish")
                if self.Set_loop == True:
                    self.get_logger().info("Continute")
                    self.Command = rclpy.parameter.Parameter(
                        "Command", rclpy.Parameter.Type.STRING, self.command_src
                    )
                    all_new_parameters = [self.Command]
                    self.set_parameters(all_new_parameters)
                    self.Switch = rclpy.parameter.Parameter(
                        "Switch", rclpy.Parameter.Type.BOOL, True
                    )
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)
                else:
                    self.get_logger().info("Not loop")
                    self.Switch = rclpy.parameter.Parameter(
                        "Switch", rclpy.Parameter.Type.BOOL, False
                    )
                    all_new_parameters = [self.Switch]
                    self.set_parameters(all_new_parameters)

    def advancing(self, target_distance):
        self.position.x = self.get_position().transform.translation.x
        self.position.y = self.get_position().transform.translation.y
        move_cmd = Twist()
        self.distance = sqrt(
            pow((self.position.x - self.x_start), 2)
            + pow((self.position.y - self.y_start), 2)
        )
        self.distance *= self.LineScaling
        self.get_logger().info(f"distance: {self.distance}")
        self.error = self.distance - target_distance
        move_cmd.linear.x = self.Linear
        if abs(self.error) < self.LineTolerance:
            self.get_logger().info("stop")
            self.distance = 0.0
            self.pub_cmdVel.publish(Twist())
            self.x_start = self.position.x
            self.y_start = self.position.y
            self.Switch = rclpy.parameter.Parameter(
                "Switch", rclpy.Parameter.Type.BOOL, False
            )
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)
            return True
        else:
            if self.front_warning > 15:
                if self.moving == True:
                    self.pub_cmdVel.publish(Twist())
                    self.moving = False
                    self.get_logger().info("obstacles")
                    self.pub_beep.publish(UInt16(data=1))  
                                        
            else:
                # print("Go")
                self.pub_cmdVel.publish(move_cmd)
                self.pub_beep.publish(UInt16(data=0))
            self.moving = True
            return False

    def Spin(self, angle):
        self.target_angle = radians(angle)
        self.odom_angle = self.get_odom_angle()
        self.delta_angle = self.RotationScaling * self.normalize_angle(
            self.odom_angle - self.last_angle
        )
        self.turn_angle += self.delta_angle
        # print("turn_angle: ",self.turn_angle)
        self.get_logger().info(f"turn_angle: {self.turn_angle}")
        self.error = self.target_angle - self.turn_angle
        self.get_logger().info(f"error: {self.error}")
        self.last_angle = self.odom_angle
        move_cmd = Twist()
        # RotationTolerance is in degrees, convert to radians for comparison
        if abs(self.error) < radians(self.RotationTolerance) or self.Switch == False:
            self.pub_cmdVel.publish(Twist())
            self.turn_angle = 0.0
            """self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)"""
            return True
        if self.front_warning > 15:
            if self.moving == True:
                self.pub_cmdVel.publish(Twist())
                self.moving = False
                self.get_logger().info("obstacles")
                self.pub_beep.publish(UInt16(data=1))

        else:
            if self.Command == "Square" or self.Command == "Triangle":
                # move_cmd.linear.x = 0.2
                move_cmd.angular.z = copysign(self.Angular, self.error)
            elif self.Command == "Circle":
                length = self.Linear * self.circle_adjust / self.Length
                # print("length: ",length)
                move_cmd.linear.x = self.Linear
                move_cmd.angular.z = copysign(length, self.error)
                # print("angular: ",move_cmd.angular.z)
                """move_cmd.linear.x = 0.2
                move_cmd.angular.z = copysign(2, self.error)"""
            self.pub_cmdVel.publish(move_cmd)
            self.pub_beep.publish(UInt16(data=0))
        self.moving = True

    def Square(self):
        if self.index == 0:
            self.get_logger().info("Length")
            step1 = self.advancing(self.Length)
            # sleep(0.5)
            if step1 == True:
                # self.distance = 0.0
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 1:
            self.get_logger().info("Spin")
            step2 = self.Spin(90)
            # sleep(0.5)
            if step2 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 2:
            self.get_logger().info("Length")
            step3 = self.advancing(self.Length)
            # sleep(0.5)
            if step3 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 3:
            self.get_logger().info("Spin")
            step4 = self.Spin(90)
            # sleep(0.5)
            if step4 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 4:
            self.get_logger().info("Length")
            step5 = self.advancing(self.Length)
            # sleep(0.5)
            if step5 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 5:
            self.get_logger().info("Spin")
            step6 = self.Spin(90)
            # sleep(0.5)
            if step6 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 6:
            self.get_logger().info("Length")
            step7 = self.advancing(self.Length)
            # sleep(0.5)
            if step7 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 7:
            self.get_logger().info("Spin")
            step8 = self.Spin(90)
            # sleep(0.5)
            if step8 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 8:
            self.get_logger().info("Length")
            step9 = self.advancing(self.Length)
            # sleep(0.5)
            if step9 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)

        else:
            self.index = 0
            self.Switch = rclpy.parameter.Parameter(
                "Switch", rclpy.Parameter.Type.BOOL, False
            )
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)
            # self.Command == "finish"
            self.get_logger().info("Done!")
            return True

    def Triangle(self):
        if self.index == 0:
            self.get_logger().info("Length")
            step1 = self.advancing(self.Length)
            # sleep(0.5)
            if step1 == True:
                # self.distance = 0.0
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 1:
            self.get_logger().info("Spin")
            step2 = self.Spin(120)
            # sleep(0.5)
            if step2 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 2:
            self.get_logger().info("Length")
            step1 = self.advancing(self.Length)
            # sleep(0.5)
            if step1 == True:
                # self.distance = 0.0
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 3:
            self.get_logger().info("Spin")
            step4 = self.Spin(120)
            # sleep(0.5)
            if step4 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        elif self.index == 4:
            self.get_logger().info("Length")

            step5 = self.advancing(self.Length)
            # sleep(0.5)
            if step5 == True:
                # self.distance = 0.0
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)

        elif self.index == 5:
            self.get_logger().info("Spin")
            step6 = self.Spin(120)
            # sleep(0.5)
            if step6 == True:
                self.index = self.index + 1
                self.Switch = rclpy.parameter.Parameter(
                    "Switch", rclpy.Parameter.Type.BOOL, True
                )
                all_new_parameters = [self.Switch]
                self.set_parameters(all_new_parameters)
        else:
            self.index = 0
            self.Switch = rclpy.parameter.Parameter(
                "Switch", rclpy.Parameter.Type.BOOL, False
            )
            all_new_parameters = [self.Switch]
            self.set_parameters(all_new_parameters)
            self.get_logger().info("Done!")
            return True

    def get_odom_angle(self):
        try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, now)
            # print("oring_rot: ",rot.transform.rotation)
            cacl_rot = PyKDL.Rotation.Quaternion(
                rot.transform.rotation.x,
                rot.transform.rotation.y,
                rot.transform.rotation.z,
                rot.transform.rotation.w,
            )
            # print("cacl_rot: ",cacl_rot)
            angle_rot = cacl_rot.GetRPY()[2]

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info("transform not ready")
            return

        return angle_rot

    def get_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, now
            )
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info("transform not ready")
            raise

    def normalize_angle(self, angle):
        res = angle
        # print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def LaserScanCallback(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return

        # Auto-detect laser frame_id
        if not self.laser_frame:
            self.laser_frame = scan_data.header.frame_id
            self.get_logger().info(f"Detected laser frame: {self.laser_frame}")

        # Get transform from laser to base_link
        try:
            trans = self.tf_buffer.lookup_transform(
                "base_link",
                scan_data.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            if not self.tf_ready:
                self.tf_ready = True
                self.get_logger().info(f"TF ready: base_link <- {scan_data.header.frame_id}")
        except TransformException:
            # Throttle TF waiting logs; keep last warning state until TF is ready
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.last_tf_log_time) >= self.tf_log_interval:
                self.get_logger().warn("TF not available, skipping scan")
                self.last_tf_log_time = current_time
            return

        # Count obstacle points directly in front of base_link
        front_warning = 0
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            if ranges[i] == 0.0 or ranges[i] > scan_data.range_max:
                continue

            # Calculate laser point position in laser coordinate frame
            angle = scan_data.angle_min + scan_data.angle_increment * i
            point_in_laser = PointStamped()
            point_in_laser.header = scan_data.header
            point_in_laser.point.x = ranges[i] * math.cos(angle)
            point_in_laser.point.y = ranges[i] * math.sin(angle)
            point_in_laser.point.z = 0.0

            # Transform to base_link coordinate frame
            try:
                point_in_base = do_transform_point(point_in_laser, trans)
                x = point_in_base.point.x
                y = point_in_base.point.y

                # Only consider obstacles in front sector (within LaserAngle)
                dist = math.sqrt(x**2 + y**2)
                angle_to_base = math.atan2(y, x) * RAD2DEG
                if abs(angle_to_base) < self.LaserAngle * 0.5 and dist <= self.ResponseDist:
                    front_warning += 1
            except TransformException:
                continue

        self.front_warning = front_warning


def main():
    rclpy.init()
    class_patrol = YahboomCarPatrol("YahboomCarPatrol")
    print("create done")
    try:
        rclpy.spin(class_patrol)
    except KeyboardInterrupt:
        pass
    finally:
        class_patrol.pub_cmdVel.publish(Twist())
        rclpy.shutdown()
