#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# author: Yahboom Technology ROS Robotics Team
# version: 202607

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from smart_tracker.msg import TrackBox
from interfaces.srv import SetTracking

class simplePID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.targetpoint = 0.0
        self.intergral = 0.0
        self.derivative = 0.0
        self.prevError = 0.0
    def compute(self, target, current):
        error = target - current
        self.intergral += error
        self.derivative = error - self.prevError
        self.targetpoint = self.kp * error + self.ki * self.intergral + self.kd * self.derivative
        self.prevError = error
        return self.targetpoint
    def reset(self):
        self.targetpoint = 0.0
        self.intergral = 0.0
        self.derivative = 0.0
        self.prevError = 0.0

class VisualTrack(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.declare_parameter("pixel_tolerance", 15)
        self.declare_parameter("max_step_xy", 5.0)
        self.declare_parameter("min_confidence", 0.01)
        self.declare_parameter("bbox_max_age", 0.4)

        # PID 参数 (作用于归一化误差 [-1, 1])
        self.declare_parameter("kp_x", 0.6)
        self.declare_parameter("kd_x", 0.15)
        self.declare_parameter("kp_y", 0.6)
        self.declare_parameter("kd_y", 0.15)

        self.declare_parameter("camera_type", '')
        self.declare_parameter("enable_body_follow", True)
        self.declare_parameter("body_kp", 0.03)
        self.declare_parameter("body_kd", 0.0)
        self.declare_parameter("body_tolerance_deg", 3.0) # 底盘停转阈值
        self.declare_parameter("servo1_engage_deg", 5.0) # 底盘旋转阈值
        self.declare_parameter("max_angular_z", 0.6)
        # 底盘转向翻转 (默认按 URDF 约定: 目标偏右则右转)
        self.declare_parameter("reverse_body_direction", False)
        # 舵机对车身旋转的前馈补偿增益 (VOR): 1.0 完全补偿, 0 关闭
        self.declare_parameter("body_ff_gain", 1.0)

        # no_ptz 模式 (无云台): 底盘直接依据水平像素误差旋转对准目标
        self.declare_parameter("no_ptz_yaw_kp", 0.9)
        self.declare_parameter("no_ptz_yaw_kd", 0.1)
        self.declare_parameter("no_ptz_engage_px", 30.0)   # 底盘起转的像素误差阈值
        self.declare_parameter("no_ptz_align_px", 80.0)    # 视为已对准(可前后移动)的像素误差阈值

        # 距离保持 (ptz / no_ptz 通用, 无雷达): 以刚捕获目标的前 N 帧 bbox 外接圆面积中位数为基准,
        # 按 "近大远小" 由面积比推算相对距离 (相对距离 ≈ sqrt(基准面积/当前面积))
        self.declare_parameter("bbox_ref_frames", 3)         # 基准采样帧数
        self.declare_parameter("bbox_area_smooth", 0.4)      # 当前面积 EMA 平滑系数 (0~1, 越大越灵敏)
        self.declare_parameter("bbox_ratio_tolerance", 0.08) # 相对距离死区 (|ratio-1| 小于该值不动)
        self.declare_parameter("bbox_dist_kp", 0.5)
        self.declare_parameter("bbox_dist_kd", 0.0)
        self.declare_parameter("bbox_ratio_max", 3.0)        # 相对距离比值限幅

        self.declare_parameter("enable_distance_keeping", True)
        # 前后距离控制的最低置信度: 低于该值只转向对准, 不做前后移动
        self.declare_parameter("distance_min_confidence", 0.6)
        # 最大线速度 (m/s)
        self.declare_parameter("max_linear_x", 0.18)
        self.declare_parameter("distance_align_deg", 40.0)
        # 线速度方向翻转 (不同机型 +x 方向可能相反)
        self.declare_parameter("reverse_linear", False)

        # ------------------------- 参数读取 -------------------------
        self.pixel_tolerance = float(self.get_parameter("pixel_tolerance").value)
        self.max_step_xy = float(self.get_parameter("max_step_xy").value)

        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.bbox_max_age = float(self.get_parameter("bbox_max_age").value)

        self.image_width = 640.0
        self.image_height = 480.0
        self.servo1_min = -90.0   
        self.servo1_max = 90.0    
        self.servo2_min = -90.0   
        self.servo2_max = 20.0    
        self.servo1_init = 0.0    
        self.servo2_init = -45.0  
        self.sign_x = 1.0         
        self.sign_y = -1.0 

        self.enable_body_follow = bool(self.get_parameter("enable_body_follow").value)
        self.body_kp = float(self.get_parameter("body_kp").value)
        self.body_kd = float(self.get_parameter("body_kd").value)
        self.body_tolerance_deg = float(self.get_parameter("body_tolerance_deg").value)
        self.servo1_engage_deg = float(self.get_parameter("servo1_engage_deg").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.body_ff_gain = float(self.get_parameter("body_ff_gain").value)

        # 追踪模式: camera_type == 'no_ptz' 时禁用云台, 仅用底盘随动; 其余(含 'ptz')为完整云台+底盘模式
        self.camera_type = str(self.get_parameter("camera_type").value)
        self.use_ptz = self.camera_type != 'no_ptz'
        self.no_ptz_yaw_kp = float(self.get_parameter("no_ptz_yaw_kp").value)
        self.no_ptz_yaw_kd = float(self.get_parameter("no_ptz_yaw_kd").value)
        self.no_ptz_engage_px = float(self.get_parameter("no_ptz_engage_px").value)
        self.no_ptz_align_px = float(self.get_parameter("no_ptz_align_px").value)
        self.bbox_ref_frames = max(1, int(self.get_parameter("bbox_ref_frames").value))
        self.bbox_area_smooth = self.clamp(float(self.get_parameter("bbox_area_smooth").value), 0.01, 1.0)
        self.bbox_ratio_tolerance = float(self.get_parameter("bbox_ratio_tolerance").value)
        self.bbox_dist_kp = float(self.get_parameter("bbox_dist_kp").value)
        self.bbox_dist_kd = float(self.get_parameter("bbox_dist_kd").value)
        self.bbox_ratio_max = max(1.1, float(self.get_parameter("bbox_ratio_max").value))

        self.enable_distance_keeping = bool(self.get_parameter("enable_distance_keeping").value)
        self.distance_min_confidence = float(self.get_parameter("distance_min_confidence").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.distance_align_deg = float(self.get_parameter("distance_align_deg").value)
        self.linear_dir = 1.0
        self.body_dir = -self.sign_x

        # 图像中心 (追踪目标点)
        self.target_x = self.image_width / 2.0
        self.target_y = self.image_height / 2.0

        # ------------------------- PID -------------------------
        self.pid_x = simplePID(float(self.get_parameter("kp_x").value), 0.0,float(self.get_parameter("kd_x").value))
        self.pid_y = simplePID(float(self.get_parameter("kp_y").value), 0.0,float(self.get_parameter("kd_y").value))

        self.current_x = self.servo1_init
        self.current_y = self.servo2_init
        self.target_lost = True

        # 底盘随动状态
        self.last_angular_z = 0.0       # 上一周期下发的底盘角速度 (用于 VOR 前馈)
        self.prev_servo_err = 0.0       # 上一周期 servo1 误差 (用于底盘 PID 微分)
        self.last_control_time = None   # 上一次控制时刻 (计算 dt)
        self.body_follow_armed = False  # 底盘随动是否已武装
        # 距离保持状态
        self.prev_dist_err = 0.0        # 上一周期距离误差 (用于距离 PID 微分)
        self.last_linear_x = 0.0        # 上一周期下发的线速度
        # bbox 测距状态
        self.bbox_ref_samples = []      # 捕获初期的外接圆面积样本
        self.bbox_ref_area = None       # 基准外接圆面积 (前 N 帧中位数)
        self.bbox_area_filt = None      # 当前外接圆面积 (EMA 平滑)
        self.bbox_dist_ratio = None     # 相对距离比 (1.0 = 基准距离, >1 更远)

        # 服务控制的跟随总开关 (默认停止, 由 /visual_track/set_tracking 开启)
        self.tracking_enabled = False
        # 最近一次目标置信度
        self.latest_confidence = 0.0
        self.latest_bbox = None         # (x1, y1, x2, y2), 图像像素

        self.ptz_angle_pub = self.create_publisher(Vector3, '/cmd_ptz_angle', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bbox_sub = self.create_subscription(TrackBox, '/tracking_bbox', self.bbox_callback, 1)
        self.set_tracking_srv = self.create_service(
            SetTracking, '/visual_track/set_tracking', self.set_tracking_callback)

        # init ptz to center (仅 ptz 模式需要云台)
        if self.use_ptz:
            self.publish_ptz(self.current_x, self.current_y)

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def publish_ptz(self, angle_x, angle_y):
        msg = Vector3()
        msg.x = float(angle_x)
        msg.y = float(angle_y)
        msg.z = 0.0
        self.ptz_angle_pub.publish(msg)

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def reset_tracking(self):
        self.pid_x.reset()
        self.pid_y.reset()

        self.body_follow_armed = False
        self.prev_servo_err = 0.0
        self.last_angular_z = 0.0
        self.prev_dist_err = 0.0
        self.last_linear_x = 0.0
        self.latest_bbox = None
        # bbox 测距: 丢失目标后重新捕获时重新标定基准
        self.bbox_ref_samples = []
        self.bbox_ref_area = None
        self.bbox_area_filt = None
        self.bbox_dist_ratio = None

    def _on_frame_dropped(self):
        self.publish_cmd_vel(0.0, 0.0)
        if not self.target_lost:
            self.target_lost = True
            self.reset_tracking()
            # self.get_logger().warn("Target lost (low confidence / stale bbox), stopping chassis.")

    def set_tracking_callback(self, request, response):
        if request.enable:
            if not self.tracking_enabled:
                self.tracking_enabled = True
                self.target_lost = True       # 等待重新捕获目标
                self.reset_tracking()
            response.message = "tracking started"
        else:
            self.tracking_enabled = False
            self.target_lost = True
            self.reset_tracking()
            self.publish_cmd_vel(0.0, 0.0)    # 立即停车
            response.message = "tracking stopped"
        self.get_logger().info(f"SetTracking: enable={bool(request.enable)}")
        response.success = True
        return response

    def bbox_callback(self, msg: TrackBox):
        if not self.tracking_enabled:
            return
        self.latest_confidence = float(msg.confidence)
        if msg.confidence < self.min_confidence:
            self._on_frame_dropped()
            return
        if not (msg.timestamp.sec == 0 and msg.timestamp.nanosec == 0):
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.timestamp)).nanoseconds * 1e-9
            if age > self.bbox_max_age:
                self._on_frame_dropped()
                return

        x1, y1, x2, y2 = msg.bbox[0], msg.bbox[1], msg.bbox[2], msg.bbox[3]
        cx = (float(x1) + float(x2)) / 2.0
        cy = (float(y1) + float(y2)) / 2.0

        if self.target_lost:
            self.pid_x.reset()
            self.pid_y.reset()

        self.target_lost = False
        self.latest_bbox = (int(x1), int(y1), int(x2), int(y2))
        self._update_bbox_distance(x1, y1, x2, y2)
        self.track(cx, cy)

    def _update_bbox_distance(self, x1, y1, x2, y2):
        """用 bbox 外接圆面积按近大远小估计相对距离 (ptz / no_ptz 通用)。

        刚捕获目标的前 bbox_ref_frames 帧面积中位数作为基准; 面积与距离平方
        成反比, 相对距离 = sqrt(基准面积/当前面积), >1 比基准远, <1 比基准近。
        """
        bw = max(1.0, float(x2) - float(x1))
        bh = max(1.0, float(y2) - float(y1))
        area = math.pi * (bw * bw + bh * bh) / 4.0   # 外接圆面积 = pi*(对角线/2)^2

        if self.bbox_ref_area is None:
            self.bbox_ref_samples.append(area)
            if len(self.bbox_ref_samples) >= self.bbox_ref_frames:
                self.bbox_ref_area = float(np.median(self.bbox_ref_samples))
                self.bbox_area_filt = self.bbox_ref_area
                self.bbox_dist_ratio = 1.0
            return

        # EMA 平滑当前面积, 抑制检测框抖动
        a = self.bbox_area_smooth
        self.bbox_area_filt = a * area + (1.0 - a) * self.bbox_area_filt
        ratio = math.sqrt(self.bbox_ref_area / max(self.bbox_area_filt, 1.0))
        self.bbox_dist_ratio = self.clamp(ratio, 1.0 / self.bbox_ratio_max, self.bbox_ratio_max)

    def track(self, cx, cy):
        now = time.monotonic()
        if self.last_control_time is None:
            dt = 1.0 / 30.0
        else:
            dt = self.clamp(now - self.last_control_time, 1e-3, 0.2)
        self.last_control_time = now

        half_w = max(1.0, self.image_width / 2.0)
        half_h = max(1.0, self.image_height / 2.0)

        error_x = cx - self.target_x          # >0: 目标偏右
        error_y = cy - self.target_y          # >0: 目标偏下

        # ---- no_ptz 模式: 无云台, 仅靠底盘旋转 + 前后移动跟随目标 ----
        if not self.use_ptz:
            self.update_chassis_no_ptz(error_x, half_w, dt)
            return


        # 车身偏航变化 Δyaw = last_angular_z * dt, 舵机需同步补偿 Δcurrent_x = sign_x * Δyaw
        if  abs(self.last_angular_z) > 1e-6:
            body_yaw_deg = math.degrees(self.last_angular_z * dt)
            self.current_x = self.clamp(
                self.current_x + self.body_ff_gain * self.sign_x * body_yaw_deg,
                self.servo1_min, self.servo1_max)

        aligned_x = abs(error_x) <= self.pixel_tolerance
        aligned_y = abs(error_y) <= self.pixel_tolerance

        # ---- 快环(相机): 像素反馈保持目标居中, 已对准的轴复位对应 PID ----
        if aligned_x:
            self.pid_x.reset()
            step_x = 0.0
        else:
            # 归一化误差 -> PID -> 限幅步进角
            norm_x = self.clamp(error_x / half_w, -1.0, 1.0)
            out_x = self.clamp(self.pid_x.compute(norm_x, 0.0), -1.0, 1.0)
            step_x = out_x * self.max_step_xy

        if aligned_y:
            self.pid_y.reset()
            step_y = 0.0
        else:
            norm_y = self.clamp(error_y / half_h, -1.0, 1.0)
            out_y = self.clamp(self.pid_y.compute(norm_y, 0.0), -1.0, 1.0)
            step_y = out_y * self.max_step_xy

        # 更新绝对角度并限制在舵机行程内
        self.current_x = self.clamp(self.current_x + self.sign_x * step_x, self.servo1_min, self.servo1_max)
        self.current_y = self.clamp(self.current_y + self.sign_y * step_y, self.servo2_min, self.servo2_max)

        self.publish_ptz(self.current_x, self.current_y)
        # ---- 慢环(底盘): 车身随动(正对目标) + 距离保持(x 线速度) ----
        self.update_chassis(dt)

    def update_chassis(self, dt):
        if self.enable_body_follow:
            angular_z = self._compute_body_follow(dt)
        else:
            self.body_follow_armed = False
            self.prev_servo_err = 0.0
            angular_z = 0.0

        # ---- 线速度: 距离保持 (车头基本对准后才前后移动) ----
        if self.enable_distance_keeping:
            aligned = abs(self.current_x) <= self.distance_align_deg
            linear_x = self._compute_distance_keeping_bbox(dt, aligned)
        else:
            self.prev_dist_err = 0.0
            linear_x = 0.0

        self.publish_cmd_vel(linear_x, angular_z)
        self.last_angular_z = angular_z
        self.last_linear_x = linear_x

    def update_chassis_no_ptz(self, error_x, half_w, dt):
        """no_ptz 模式底盘控制: 角速度由水平像素误差驱动, 线速度做距离保持。"""
        if self.enable_body_follow:
            angular_z = self._compute_body_follow_pixel(error_x, half_w, dt)
        else:
            self.body_follow_armed = False
            self.prev_servo_err = 0.0
            angular_z = 0.0

        if self.enable_distance_keeping:
            aligned = abs(error_x) <= self.no_ptz_align_px
            linear_x = self._compute_distance_keeping_bbox(dt, aligned)
        else:
            self.prev_dist_err = 0.0
            linear_x = 0.0

        self.publish_cmd_vel(linear_x, angular_z)
        self.last_angular_z = angular_z
        self.last_linear_x = linear_x

    def _compute_distance_keeping_bbox(self, dt, aligned):
        ratio = self.bbox_dist_ratio
        if ratio is None or not aligned:
            self.prev_dist_err = 0.0
            return 0.0
        if self.latest_confidence < self.distance_min_confidence:
            self.prev_dist_err = 0.0
            return 0.0

        error = ratio - 1.0                  # >0: 比基准远, 需前进
        if abs(error) < self.bbox_ratio_tolerance:
            self.prev_dist_err = 0.0
            return 0.0
        deriv = (error - self.prev_dist_err) / dt
        linear_x = self.bbox_dist_kp * error + self.bbox_dist_kd * deriv
        linear_x = self.clamp(linear_x, -self.max_linear_x, self.max_linear_x)
        self.prev_dist_err = error
        return self.linear_dir * linear_x

    def _compute_body_follow_pixel(self, error_x, half_w, dt):

        abs_err = abs(error_x)
        if abs_err >= self.no_ptz_engage_px:
            self.body_follow_armed = True
        elif abs_err <= self.pixel_tolerance:
            self.body_follow_armed = False

        if not self.body_follow_armed:
            self.prev_servo_err = 0.0
            return 0.0

        norm_x = self.clamp(error_x / half_w, -1.0, 1.0)
        deriv = (norm_x - self.prev_servo_err) / dt
        angular_z = self.body_dir * (self.no_ptz_yaw_kp * norm_x + self.no_ptz_yaw_kd * deriv)
        angular_z = self.clamp(angular_z, -self.max_angular_z, self.max_angular_z)
        self.prev_servo_err = norm_x
        return angular_z

    def _compute_body_follow(self, dt):

        if abs(self.current_x) >= self.servo1_engage_deg:
            self.body_follow_armed = True
        elif abs(self.current_x) <= self.body_tolerance_deg:
            self.body_follow_armed = False

        if not self.body_follow_armed:
            self.prev_servo_err = 0.0
            return 0.0

        servo_err = self.current_x
        deriv = (servo_err - self.prev_servo_err) / dt
        angular_z = self.body_dir * (self.body_kp * servo_err + self.body_kd * deriv)
        angular_z = self.clamp(angular_z, -self.max_angular_z, self.max_angular_z)
        self.prev_servo_err = servo_err
        return angular_z


def main():
    rclpy.init()
    node = VisualTrack('visual_track_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
