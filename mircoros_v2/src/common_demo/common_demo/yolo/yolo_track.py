from typing import List
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterValue
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import time
from colorama import Fore
from collections import defaultdict
import numpy as np
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from interfaces.srv import ArmPosition
from ultralytics import YOLO

class YOLO_TRACKER(Node):
    def __init__(self,name):
        super().__init__(name)
        self.cv_bridge = CvBridge()
        self.count=0
        self.declare_parameter('init_pose', [90, 180, 0, 0, 90, 30])
        self.init_pose = self.get_parameter('init_pose').get_parameter_value().integer_array_value
        self.declare_parameter('model_path','')
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.set_arm_client = self.create_client(ArmPosition, 'set_arm_joints')
        self.declare_parameter('conf',0.1)
        self.declare_parameter('iou',0.3)
        self.declare_parameter('max_det',3)
        self.declare_parameter('track_id',9999)
        self.declare_parameter("max_draw_points",30)
        self.declare_parameter('keep_dist',1.0)
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)
        self.declare_parameter('Kp_1', -0.1)
        self.declare_parameter('Ki_1', 0.0)
        self.declare_parameter('Kd_1', -0.3)
        self.declare_parameter('max_integral', 1000.0)
        self.conf=self.get_parameter("conf").get_parameter_value().double_value
        self.iou=self.get_parameter("iou").get_parameter_value().double_value
        self.max_det=self.get_parameter("max_det").get_parameter_value().integer_value
        self.track_id=self.get_parameter("track_id").get_parameter_value().integer_value
        self.max_draw_points=self.get_parameter("max_draw_points").get_parameter_value().integer_value
        self.Kp_dist = self.get_parameter('Kp').value
        self.Ki_dist = self.get_parameter('Ki').value
        self.Kd_dist = self.get_parameter('Kd').value
        self.Kp_angular = self.get_parameter('Kp_1').value
        self.Ki_angular = self.get_parameter('Ki_1').value
        self.Kd_angular = self.get_parameter('Kd_1').value
        self.max_integral = self.get_parameter('max_integral').value
        self.keep_dist = self.get_parameter('keep_dist').value

        #create the subscriber
        self.depth_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.rgb_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub],queue_size=10,slop=1.0)
        self.ts.registerCallback(self.depth_img_Callback)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.frame_count=0
        self.pub_count=0
        self.prev_time = time.time()
        self.frame_count = 0
        self.fps = 0.0
        self.encoding = ['16UC1', '32FC1']
        self.track_history = defaultdict(lambda: [])
        self.dist_controller = PID_CONTROLLER(self.Kp_dist, self.Ki_dist, self.Kd_dist,self.max_integral)
        self.angular_controller = PID_CONTROLLER(self.Kp_angular, self.Ki_angular, self.Kd_angular,self.max_integral)
        self.yolo_model = YOLO(self.model_path, task='detect')
        self._init_arm_pose()
        self.get_logger().info(Fore.GREEN+"YOLO Model Load Success."+Fore.RESET)

#----------------------------------------callback Function-----------------------------------------
    def _init_arm_pose(self):
        if not self.set_arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Failed to connect to set_arm_joints server, skipping init arm pose")
            return
        req = ArmPosition.Request()
        req.request_joints = list(self.init_pose)
        future = self.set_arm_client.call_async(req)
        future.add_done_callback(self._init_arm_done_callback)

    def _init_arm_done_callback(self, future):
        result = future.result()
        if not result.success:
            self.get_logger().warn(f"Init arm pose failed: {list(result.result_joints)}, msg: {result.message}")

    def depth_img_Callback(self,rgb_msg:Image,depth_frame:Image):
        box=None
        current_pose=None
        # convert image
        rgb_image = self.cv_bridge .imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.cv_bridge.imgmsg_to_cv2(depth_frame, self.encoding[1])
        depth_image_info = depth_image.astype(np.float32) 
        infer_start = time.time()
        frame = rgb_image
        #yolo infer
        results = self.yolo_model.track(rgb_image, conf=self.conf, iou=self.iou, max_det=self.max_det, stream=True, persist=True,verbose=False)
        for res in results:
            frame = res.plot()
            if res.boxes is not None and res.boxes.id is not None:
                boxes = res.boxes.xywh.cpu()
                track_ids = res.boxes.id.int().cpu().tolist()
                # Find the box matching self.track_id
                box = None
                for tid, b in zip(track_ids, boxes):
                    if tid == self.track_id:
                        box = b
                        break
                if box is not None:
                    self.draw_trajectory_point(frame,box)
                    current_pose=self.get_target_dist(box,depth_image_info)
                    if current_pose is not None : self.move_control(current_pose[0],current_pose[1])
                    break  
        #handle result
        if current_pose is not None:
            cv2.putText(frame,f"dist: {current_pose[0]:.1f} m,x:{current_pose[1]},y:{current_pose[2]}",(10, 60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0, 255, 255),1)
        else:
            '''stop move'''
            self.pub_count+=1
            if self.pub_count>=3:
                self.pub_count=0
                self.cmd_vel_pub.publish(Twist())
        
        infer_end = time.time()
        self.frame_count += 1
        curr_time = time.time()
        time_diff = curr_time - self.prev_time
        if time_diff >= 1.0:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.prev_time = curr_time
        cv2.putText(frame,f"FPS: {self.fps:.2f}",(10, 25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0, 255, 0),1)
        infer_time = (infer_end - infer_start) * 1000
        cv2.putText(frame, f"Infer: {infer_time:.1f} ms", (10, 45),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.imshow("YOLO Track Panel", frame)
        cv2.waitKey(1)

    def parameters_callback(self, params:List[ParameterValue]):
        '''Update the parameter callback'''
        for param in params:
            if param.name == 'conf':
                self.conf = param.value
            elif param.name == 'iou':
                self.iou = param.value
            elif param.name == 'track_id':
                self.track_id = param.value
            elif param.name == 'Kp':
                self.Kp_dist = param.value
                self.dist_controller.set_param(kp=self.Kp_dist)
            elif param.name == 'Ki':
                self.Ki_dist = param.value
                self.dist_controller.set_param(ki=self.Ki_dist)
            elif param.name == 'Kd':
                self.Kd_dist = param.value
                self.dist_controller.set_param(kd=self.Kd_dist)
            elif param.name == 'Kp_1':
                self.Kp_angular = param.value
                self.angular_controller.set_param(kp=self.Kp_angular)
            elif param.name == 'Ki_1':
                self.Ki_angular = param.value
                self.angular_controller.set_param(ki=self.Ki_angular)
            elif param.name == 'Kd_1':
                self.Kd_angular = param.value
                self.angular_controller.set_param(kd=self.Kd_angular)
            elif param.name == 'max_integral':
                self.max_integral = param.value
                self.dist_controller.set_param(integral_limit=self.max_integral)
                self.angular_controller.set_param(integral_limit=self.max_integral)
            elif param.name == 'keep_dist':
                self.keep_dist = param.value
            elif param.name == 'max_det':
                self.max_det = param.value
            self.get_logger().info(f"{param.name} updated to {param.value}")
        return SetParametersResult(successful=True)


#-----------------------------------------Class Function-----------------------------------------
    def draw_trajectory_point(self,frame,box):
        '''Draw the trajectory and the center point'''
        x, y, w, h = box  
        center_x, center_y = int(x), int(y)
        cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 255, 0), thickness=-1 ) #draw center point

        track = self.track_history[self.track_id]  # get history of track
        track.append((float(x), float(y)))         # add to history
        if len(track) > self.max_draw_points:                        
            track.pop(0)
        points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [points], isClosed=False, color=(0, 0, 255), thickness=3)


    def get_target_dist(self,box,depth_image_info):
        '''Get the target distance'''
        x, y, w, h = box
        valid_depths = []
        center_x, center_y = int(x), int(y)
        points = [
            (int(center_y), int(center_x)), 
            (int(center_y -1), int(center_x -1)), 
            (int(center_y - 3), int(center_x - 3)) ]
        for y, x in points:
            depth_val = depth_image_info[y][x]                   #读取深度图对应坐标的深度值
            if depth_val != 0:  valid_depths.append(depth_val)
        
        if valid_depths:
            dist = (sum(valid_depths) / len(valid_depths))/1000  #convert to meter
            return dist,x, y
        return None

    def move_control(self,dist:float,point_x):
        '''计算PID控制移动'''
        speed=Twist()
        #calculate error
        dist_error = dist - self.keep_dist
        angular_error = point_x - 320
        #calculate PID
        if abs(dist_error) > 0.05:
            linear_x=self.dist_controller.pid_control(dist_error)
            linear_x=min(linear_x,0.35)
            linear_x=max(linear_x,-0.35)
            speed.linear.x = linear_x
        if abs(angular_error) >30:
            angular_z=self.angular_controller.pid_control(angular_error)
            angular_z=min(angular_z,0.3)
            angular_z=max(angular_z,-0.3)
            speed.angular.z = angular_z
        self.cmd_vel_pub.publish(speed)


class PID_CONTROLLER:
    """
    PID controller
    """
    def __init__(self, kp:float, ki:float, kd:float,max_integral:float=1000.0):
        #Initialize the PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    def pid_control(self, error):
        """
        PID计算输出PID calculation output
        """
        current_time = time.time()
        delta_time = current_time - self.last_time
        
        if delta_time <= 0:
            delta_time = 1e-6  
    
        proportional = self.kp * error
        
        self.integral += error * delta_time
        self.integral = min(self.integral, self.max_integral)
        integral_term = self.ki * self.integral
        
        derivative = (error - self.previous_error) / delta_time
        derivative_term = self.kd * derivative
        
        out = proportional + integral_term + derivative_term
        
        self.previous_error = error
        self.last_time = current_time
        return out
    
    def reset(self):
        """
        Reset the PID controller
        """
        self.previous_error = 0.0
        self.integral = 0.0

    def set_param(self, kp=None, ki=None, kd=None,integral_limit=None):
        """
        Set PID parameters
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if integral_limit is not None:
            self.max_integral = integral_limit


def main():
    rclpy.init()   
    yolov11_detect_node = YOLO_TRACKER('yolo_tracker_node')
    rclpy.spin(yolov11_detect_node)
    yolov11_detect_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
	main()        
