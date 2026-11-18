
#pip library
import requests
from ament_index_python.packages import get_package_share_directory
import json
from colorama import Fore
import yaml
import os
import threading
from concurrent.futures import Future
import cv2

#ros library
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
import rclpy
import math

#interface
from geometry_msgs.msg import PoseStamped,Twist,Pose,TransformStamped,Vector3
from nav_msgs.msg import Odometry
from smart_tracker.srv import CancelTrack,GetTargetPose,StartTrack,GetBoxCenter
from smart_tracker.msg import TrackBox
from std_srvs.srv import Trigger
from std_msgs.msg import Bool,String
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from roadnet.action import RoadNetToGoal
from interfaces.action import MoveByOdom, PreciseAdjustPose
from interfaces.srv import GetRobotPose, SetTracking, GetPTZAngle
from multi_brains_pre.utils.robot_vision import RobotVisualPositioning,Visual_Result
# tools
from multi_brains_pre.utils.common_tools import get_yaw_from_quaternion,calculate_distance,calculate_angle_diff,normalize_angle,draw_grid_overlay,kill_process_tree

class ROSContext:
    """class to manage ROS context."""
    _node = None
    _started = False
    _spin_thread = None

    @classmethod
    def create_node(cls):
        """Get ActionController node object"""
        if cls._node is None:
            rclpy.init()
            cls._node = ActionController()
        return cls._node

    @classmethod
    def start_spin(cls):
        '''Start the ROS spinning in a separate thread'''
        if cls._started:
            return
        def spin():
            rclpy.spin(cls.create_node())
        cls._spin_thread = threading.Thread(target=spin, daemon=True)
        cls._spin_thread.start()
        cls._started = True

    @classmethod
    def shutdown(cls):
        '''Shutdown ROS context and cleanup resources'''
        if cls._node:
            cls._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

class ActionController(Node):
    '''core class of action controller'''
    def __init__(self):
        super().__init__("actioncontroller")
        self.init_param_config()  # Initialize parameter configuration
        self.init_ros_comunication()#  Initialize ROS communication
        test_connection = self.robot_vision.test_connection()
        if not test_connection[0]:
            self.get_logger().error(f"VLLM connection failed: {test_connection[1]}")
            
    def init_param_config(self):
        """
        Initialize parameter configuration
        """
        #file path
        self.declare_parameter("map_mapping_file", os.path.join(get_package_share_directory("microros_v2_bringup"),"config","map_mapping.yaml"))
        self.map_mapping_file = self.get_parameter("map_mapping_file").get_parameter_value().string_value
        self.declare_parameter("verify_image_path", "")
        self.declare_parameter("grid_images_path", "")
        self.verify_image_path = self.get_parameter("verify_image_path").get_parameter_value().string_value
        self.grid_images_path = self.get_parameter("grid_images_path").get_parameter_value().string_value


        self.declare_parameter("Vision_API_KEY", "")
        self.declare_parameter("Vision_BASE_URL", "")
        self.declare_parameter("Vision_MODEL", "")
        self.robot_vision = RobotVisualPositioning(
            api_key=self.get_parameter("Vision_API_KEY").get_parameter_value().string_value,
            base_url=self.get_parameter("Vision_BASE_URL").get_parameter_value().string_value,
            model=self.get_parameter("Vision_MODEL").get_parameter_value().string_value)

        # Navigation mode config
        self.declare_parameter("use_nav", False)
        self.declare_parameter("nav_mode", 'nav2')
        self.use_nav = self.get_parameter("use_nav").get_parameter_value().bool_value
        self.nav_mode = self.get_parameter("nav_mode").get_parameter_value().string_value
        self.image_path=None
                

    def init_ros_comunication(self):
        """
        Initialize creation of ROS communication objects and functions
        """
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 5)#Create velocity topic publisher
        self.start_track_client = self.create_client(StartTrack, "/start_track")
        self.cancel_track_client = self.create_client(CancelTrack, "/cancel_track")
        self.image_cllient = self.create_client(Trigger, '/save_image')
        self.box_center_client = self.create_client(GetBoxCenter, '/get_box_center')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')       
        self.roadnet_nav_client = ActionClient(self, RoadNetToGoal, 'compute_route_and_follow')
        self.move_by_odom_client = ActionClient(self, MoveByOdom, 'move_by_odom')
        self.precise_adjust_client = ActionClient(self, PreciseAdjustPose, 'precise_adjust_pose')
        self.get_robot_pose_client = self.create_client(GetRobotPose, 'get_robot_pose')
        self.set_tracking_client = self.create_client(SetTracking, '/visual_track/set_tracking')
        self.get_ptz_angle_client = self.create_client(GetPTZAngle, '/get_ptz_angle')
        self.cmd_ptz_angle_pub = self.create_publisher(Vector3, '/cmd_ptz_angle', 10)
        

#-----------------------------------------External access methods-----------------------------------------
    def move_by_odom(self, distance: float = 0.0, angle: float = 0.0, direction: str = 'forward') -> list[bool, str]:
        ''' Move the chassis by requesting the MoveByOdom action service.
        Supports: forward, backward, turn_left, turn_right.'''
        if not self.move_by_odom_client.wait_for_server(timeout_sec=10.0):
            return [False, "MoveByOdom action server not available"]
        goal = MoveByOdom.Goal()
        goal.distance = float(distance)
        goal.angle = float(angle)
        goal.direction = direction
        send_goal_future = self.move_by_odom_client.send_goal_async(goal)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.5)

        if not send_goal_future.done():
            return [False, "Failed to send MoveByOdom goal"]

        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.5)

        if not result_future.done():
            return [False, "MoveByOdom failed to get result"]

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f"MoveByOdom succeeded: {result.message}")
            return [True, result.message]
        else:
            self.get_logger().warn(f"MoveByOdom failed: {result.message}")
            return [False, result.message]

    def SeeWhat(self)->list[bool,str]:
        '''To capture the scene before robot eyes'''
        response:Trigger.Response=self.image_cllient.call(Trigger.Request())
        if response.success:
            result=[True,response.message]
            self.image_path=response.message
        else:
            self.image_path=None
            result=[False,response.message]
            self.get_logger().error(response.message)
        return result

    def AdjustCameraView(self,pitch:int=0,yaw:int=0)->list[bool,str]:
        '''quickly adjust the camera view'''
        if not self.get_ptz_angle_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GetPTZAngle service not available")
            return [False, "GetPTZAngle service not available"]
        cur: GetPTZAngle.Response = self.get_ptz_angle_client.call(GetPTZAngle.Request())
        if not cur.success:
            self.get_logger().error("Failed to get current PTZ angle")
            return [False, "Failed to get current PTZ angle"]
        target = Vector3()
        target.x = float(cur.x) + float(yaw)
        target.y = float(cur.y) + float(pitch)
        target.z = 0.0
        self.cmd_ptz_angle_pub.publish(target)
        return [True, f"Camera view adjusted to x={target.x:.1f}, y={target.y:.1f}"]

    def _publish_ptz(self, x: float, y: float):
        '''发布一次云台目标角度 (deg)'''
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.cmd_ptz_angle_pub.publish(msg)

    def _sweep_ptz(self, x_start: float, y_start: float, x_end: float, y_end: float,
                   step_deg: float = 2.0, rate_hz: float = 25.0):
        dx = x_end - x_start
        dy = y_end - y_start
        dist = max(abs(dx), abs(dy))
        steps = max(1, int(math.ceil(dist / step_deg)))
        dt = 1.0 / rate_hz
        for i in range(1, steps + 1):
            t = i / steps
            # smoothstep: 起步/停止平滑, 拼接多段时换向点速度为零
            s = t * t * (3.0 - 2.0 * t)
            self._publish_ptz(x_start + dx * s, y_start + dy * s)
            time.sleep(dt)

    def CameraActionGroup(self, action_name: str) -> list[bool, str]:
        if action_name == "init_camera_pose":
            self._publish_ptz(0.0, -45.0)
            return [True, "Camera pose initialized"]

        elif action_name == "nod":
            self._publish_ptz(0.0, -45.0)
            time.sleep(0.5)
            for _ in range(2):
                self._sweep_ptz(0.0, -45.0, 0.0, -90.0)
                self._sweep_ptz(0.0, -90.0, 0.0, -45.0)
            return [True, "Nod done"]

        elif action_name == "shake":
            self._publish_ptz(0.0, -45.0)
            time.sleep(0.5)
            x_from = 0.0
            for _ in range(2):
                self._sweep_ptz(x_from, -45.0, -45.0, -45.0,step_deg=5.0)
                self._sweep_ptz(-45.0, -45.0, 45.0, -45.0,step_deg=5.0)
                x_from = 45.0
            self._sweep_ptz(45.0, -45.0, 0.0, -45.0,step_deg=5.0)
            return [True, "Shake done"]
        else:
            self.get_logger().warn(Fore.YELLOW + f"Unknown camera action group: {action_name}" + Fore.RESET)
            return [False, f"Unknown camera action group: {action_name}"]


    def GetBbox(self,query:str)->list[bool,str]:
        """Get the bounding box of an object in the scene"""
        res=self.SeeWhat() #Automatic image refresh
        if not res[0]:
            return [False, f"Failed to capture image: {res[1]}"]

        img=draw_grid_overlay(image=cv2.imread(self.image_path),grid_size=30)
        cv2.imwrite(self.grid_images_path, img)

        result: Visual_Result = self.robot_vision.BoundingBoxDetection(self.grid_images_path, query)
        if result.error:
            return [False, result.error_message]
        if not result.found:
            return [False, result.error_message or "Target object not found in the image."]

        img = cv2.imread(self.image_path)
        cv2.rectangle(img, (result.x1, result.y1), (result.x2, result.y2), (0, 255, 0), 2)
        cv2.imwrite(self.verify_image_path, img)
        return [True, str({"x1": result.x1, "y1": result.y1, "x2": result.x2, "y2": result.y2})]


    def Navigation(self, location: str) -> list[bool,str]:
        '''Navigate to target location. Mode depends on use_nav and nav_mode parameters.'''
        if not self.use_nav:
            return [False, "Navigation is not enabled ,Please add the startup parameter `use_nav:=true` when starting the OpenClaw service.This step should only be performed by the user."]

        location = location.strip("'\"")
        try:
            with open(self.map_mapping_file, 'r', encoding='utf-8') as file:
                map_data = yaml.safe_load(file) or {}

            target_data = None
            for symbol, data in map_data.items():
                if symbol == location or (isinstance(data, dict) and data.get('name') == location):
                    target_data = data
                    break
            if target_data is None:
                error_info = f"The map mapping file has no coordinate record for '{location}'."
                self.get_logger().error(error_info)
                return [False,error_info]
        except Exception as e:
            self.get_logger().error(str(e))
            return [False,str(e)]

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        position = target_data.get('position', {})
        goal_msg.pose.position.x = float(position.get('x', 0.0))
        goal_msg.pose.position.y = float(position.get('y', 0.0))
        goal_msg.pose.position.z = float(position.get('z', 0.0))
        orientation = target_data.get('orientation', {})
        goal_msg.pose.orientation.x = float(orientation.get('x', 0.0))
        goal_msg.pose.orientation.y = float(orientation.get('y', 0.0))
        goal_msg.pose.orientation.z = float(orientation.get('z', 0.0))
        goal_msg.pose.orientation.w = float(orientation.get('w', 1.0))

        if self.nav_mode == 'nav2':
            return self._nav2_navigation(goal_msg, position, orientation, location)
        elif self.nav_mode == 'roadnet':
            return self._roadnet_navigation(goal_msg, location)
        else:
            error=f"Unknown nav_mode: '{self.nav_mode}'. Please check the nav_mode startup parameters."
            self.get_logger().error(error)
            return [False, error]

    def _nav2_navigation(self, goal_msg: PoseStamped, position: dict, orientation: dict, location: str) -> list[bool,str]:
        '''Navigate using nav2 NavigateToPose action'''
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            return [False, "Nav2 action server not available,Users should check if the navigation nodes are functioning correctly"]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(position.get('x', 0.0))
        goal.pose.pose.position.y = float(position.get('y', 0.0))
        goal.pose.pose.position.z = float(position.get('z', 0.0))
        goal.pose.pose.orientation.x = float(orientation.get('x', 0.0))
        goal.pose.pose.orientation.y = float(orientation.get('y', 0.0))
        goal.pose.pose.orientation.z = float(orientation.get('z', 0.0))
        goal.pose.pose.orientation.w = float(orientation.get('w', 1.0))

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.5)
        
        if not send_goal_future.done():
            return [False, "Failed to send navigation goal"]
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return [False, "Navigation goal rejected"]
        
        result_future = goal_handle.get_result_async()
        
        while rclpy.ok() and not result_future.done():
            time.sleep(0.5)
        
        if result_future.done():
            result = result_future.result().result
            self.PreciseAdjustPosition(goal_msg.pose)
            return [True, "Navigation succeeded"]
        else:
            return [False, "Navigation failed"]

    def _roadnet_navigation(self, goal_msg: PoseStamped, location: str) -> list[bool,str]:
        '''Navigate using roadnet action'''
        if not self.roadnet_nav_client.wait_for_server(timeout_sec=10.0):
            return [False, "RoadNet action server not available,Users should check if the navigation nodes are functioning correctly."]

        goal = RoadNetToGoal.Goal()
        goal.use_start = False
        goal.use_poses = True
        goal.goal = goal_msg

        self.get_logger().info(f"Sending roadnet navigation goal to '{location}'")

        send_goal_future = self.roadnet_nav_client.send_goal_async(goal)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.5)

        if not send_goal_future.done():
            return [False, "Failed to send roadnet navigation goal"]

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return [False, "Roadnet navigation goal rejected"]

        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            time.sleep(0.5)

        if result_future.done():
            result = result_future.result().result
            if result.success:
                self.PreciseAdjustPosition(goal_msg.pose)
                return [True, f"Roadnet navigation to '{location}' succeeded"]
            else:
                self.get_logger().warn(
                    Fore.YELLOW + f"Roadnet navigation to '{location}' failed: "f"error_code={result.error_code}, {result.error_msg}" + Fore.RESET)
                return [False, result.error_msg]
        else:
            return [False, "Roadnet navigation failed"]

    def PreciseAdjustPosition(self, target_pose: Pose) -> list[bool, str]:
        '''Request precise pose adjustment via PreciseAdjustPose action service'''
        if not self.precise_adjust_client.wait_for_server(timeout_sec=10.0):
            return [False, "PreciseAdjustPose action server not available"]

        goal = PreciseAdjustPose.Goal()
        goal.target_pose = target_pose

        send_goal_future = self.precise_adjust_client.send_goal_async(goal)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.5)

        if not send_goal_future.done():
            return [False, "Failed to send PreciseAdjustPose goal"]

        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.5)

        if not result_future.done():
            return [False, "PreciseAdjustPose result timeout"]

        result = result_future.result().result
        if result.success:
            return [True, result.message]
        return [False, result.message]
        
    def GetMapMapping(self) -> list[bool, str]:
        try:
            with open(self.map_mapping_file, 'r', encoding='utf-8') as file:
                existing_data = yaml.safe_load(file) or {}
            
            name_symbol_map = {}
            for symbol, data in existing_data.items():
                if isinstance(data, dict) and 'name' in data:
                    name_symbol_map[data['name']] = symbol
            return [True,str(name_symbol_map)]
        except Exception as e:
            self.get_logger().error(f"Failed to get map mapping: {str(e)}")
            return [False, str(e)]


    def RecordMapLocation(self, name: str, symbol: str) -> list[bool, str]:
        '''Record the current location to the map mapping file'''
        if not self.get_robot_pose_client.wait_for_service(timeout_sec=10.0):
            return [False, "GetRobotPose service not available, Please check if the base_driver node is alive?"]

        response: GetRobotPose.Response = self.get_robot_pose_client.call(GetRobotPose.Request())
        if not response.success:
            self.get_logger().error(Fore.RED + f"GetRobotPose failed: {response.message}" + Fore.RESET)
            return [False, response.message]

        pose = response.pose
        self.get_logger().info(f"Current pose: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
        
        try:
            with open(self.map_mapping_file, 'r', encoding='utf-8') as file:
                existing_data = yaml.safe_load(file) or {}
            if symbol in existing_data:
                self.get_logger().warn(f"Symbol '{symbol}' already exists, overwriting existing record")
        
            existing_data[symbol] = {
                'name': name,
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                }
            }
            
            with open(self.map_mapping_file, 'w', encoding='utf-8') as file:
                yaml.dump(existing_data, file, allow_unicode=True, default_flow_style=False, sort_keys=False)

            self.get_logger().info(f"Location recorded successfully: {symbol} - {name}")
            return [True, f"Location '{name}' (symbol: {symbol}) recorded to map mapping file"]
            
        except Exception as e:
            self.get_logger().error(f"Failed to record location: {str(e)}")
            return [False, str(e)]

    def TargetTrack(self, x1_or_cmd, y1:int=None, x2:int=None, y2:int=None):

        # Cancel tracking mode
        if isinstance(x1_or_cmd, str):
            if x1_or_cmd != "cancel":
                self.get_logger().warn(Fore.YELLOW+f"Invalid command for TargetTrack: Use 'cancel' to stop tracking{x1_or_cmd}"+Fore.RESET)
                return [False, "Invalid command for TargetTrack. Use 'cancel' to stop tracking."]

            if self.set_tracking_client.wait_for_service(timeout_sec=5.0):
                self.set_tracking_client.call(SetTracking.Request(enable=False))
            else:
                self.get_logger().warn("SetTracking service not available when cancelling")

            if not self.cancel_track_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("CancelTrack service not available")
                return [False, "CancelTrack service not available"]
            self.cancel_track_client.call(CancelTrack.Request())
            return [True, "Tracking cancelled and arm reset"]

        # Start tracking mode
        x1: int = x1_or_cmd
        if None in (y1, x2, y2):
            self.get_logger().error("TargetTrack: x1, y1, x2, y2 are all required to start tracking")
            return [False, "Missing bounding box parameters"]

        # Start tracking mode
        if not self.start_track_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("StartTrack service not available")
            return [False, "Tracking service not available"]
        result: StartTrack.Response = self.start_track_client.call(StartTrack.Request(x1=x1, y1=y1, x2=x2, y2=y2))
        if not result.success:
            self.get_logger().error("Failed to initialize tracking target")
            return [False, "Tracking failed"]

        # Request gimbal and chassis tracking services /visual_track/set_tracking
        if not self.set_tracking_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetTracking service not available")
            if self.cancel_track_client.wait_for_service(timeout_sec=5.0):
                self.cancel_track_client.call(CancelTrack.Request())
            return [False, "SetTracking service not available"]
        set_result: SetTracking.Response = self.set_tracking_client.call(SetTracking.Request(enable=True))
        if not set_result.success:
            self.get_logger().error("Failed to enable gimbal/chassis tracking")
            if self.cancel_track_client.wait_for_service(timeout_sec=5.0):
                self.cancel_track_client.call(CancelTrack.Request())
            return [False, "Failed to enable gimbal/chassis tracking"]

        return [True, "Tracking started"]

    def FollowLine(self, color: str) -> list[bool, str]:
        import subprocess
        color = color.strip("'\"").strip().lower()
        if color not in ('red', 'green', 'blue', 'yellow'):
            error_msg = f"Invalid color specified: '{color}'. Valid options are: red, green, blue, yellow."
            self.get_logger().error(Fore.RED+error_msg+Fore.RESET)
            return [False, error_msg]

        self._publish_ptz(x=0.0, y=-90.0)
        time.sleep(0.5)
        self.follow_line_process = None
        result = [True, "Follow line finished"]
        try:
            self.follow_line_process = subprocess.Popen(
                ['ros2', 'run', 'multi_brains_pre', 'follow_line',
                 '--ros-args', '-p', f'color:={color}'])
            self.get_logger().info(f"Start following the {color} line ...")
            self.follow_line_process.wait()
            return_code = self.follow_line_process.returncode
            if return_code == 2:
                self.get_logger().warn(Fore.YELLOW+"No line segment found, follow line aborted"+Fore.RESET)
                result = [False, "No line segment found"]
            elif return_code not in (0, None):
                result = [False, f"Follow line process exited with code {return_code}"]
        except Exception as e:
            self.get_logger().error(Fore.RED+f"FollowLine failed: {e}"+Fore.RESET)
            self._publish_ptz(x=0.0, y=-45.0)
            return [False, str(e)]
        finally:
            if self.follow_line_process is not None:
                kill_process_tree(self.follow_line_process.pid)
                self.follow_line_process = None
            self._publish_ptz(x=0.0, y=-45.0)
        return result
