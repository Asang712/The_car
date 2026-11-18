#!/usr/bin/env python3
"""
相机内参验证节点

订阅 /camera/image_raw 与 /camera/camera_info，实时对图像做去畸变，
将原图与去畸变图左右并排显示，用于目视验证标定是否准确：
    - 若标定准确，画面中的直线（门框、桌沿、标定板边缘）去畸变后应变直；
    - 画面顶部叠加内参关键信息（fx, fy, cx, cy 及主点相对中心的偏移）供参考。

用法:
    ros2 run microros_v2_bringup calib_verify

可选参数:
    image_topic       图像话题        (默认 /camera/image_raw)
    camera_info_topic 内参话题        (默认 /camera/camera_info)
    show_window       是否弹窗显示     (默认 True，无显示器时设为 False)
    save_path         对比图保存路径   (默认 ''，非空时每帧覆盖保存一张)

棋盘格 PnP 定量验证参数:
    board_cols        棋盘格 *内角点* 列数 (默认 9，即每行黑白交叉点数)
    board_rows        棋盘格 *内角点* 行数 (默认 6)
    square_size       每个方格实际边长(米) (默认 0.025，即 25mm)
    enable_pnp        是否启用 PnP 测距    (默认 True)

PnP 验证原理:
    检测棋盘格内角点 -> solvePnP 解算相机到标定板的位姿 -> 输出距离 Z。
    将该距离与卷尺实测值比对，误差应在几个百分点内；同时输出角点重投影
    误差(RMS)，< 1px 说明内参精度良好。
"""
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CalibVerifyNode(Node):
    def __init__(self):
        super().__init__('calib_verify')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('show_window', True)
        self.declare_parameter('save_path', '')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value

        # 棋盘格 PnP 定量验证参数
        self.declare_parameter('board_cols', 9)
        self.declare_parameter('board_rows', 6)
        self.declare_parameter('square_size', 0.02)
        self.declare_parameter('enable_pnp', True)
        self.board_cols = self.get_parameter('board_cols').get_parameter_value().integer_value
        self.board_rows = self.get_parameter('board_rows').get_parameter_value().integer_value
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.enable_pnp = self.get_parameter('enable_pnp').get_parameter_value().bool_value

        # 预生成棋盘格在标定板坐标系下的 3D 角点 (Z=0 平面)
        self.board_size = (self.board_cols, self.board_rows)
        objp = np.zeros((self.board_rows * self.board_cols, 3), dtype=np.float32)
        objp[:, :2] = np.mgrid[0:self.board_cols, 0:self.board_rows].T.reshape(-1, 2)
        self.object_points = objp * self.square_size
        self._criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.width = 0
        self.height = 0
        self._map1 = None
        self._map2 = None
        self._info_logged = False

        # 用时间同步器对齐图像与内参
        self.image_sub = Subscriber(self, Image, self.image_topic)
        self.info_sub = Subscriber(self, CameraInfo, self.info_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub], queue_size=10, slop=0.5
        )
        self.sync.registerCallback(self._syncCallback)

        self.get_logger().info(
            f'calib_verify started. image: {self.image_topic}, '
            f'camera_info: {self.info_topic}, show_window: {self.show_window}'
        )
        if self.enable_pnp:
            self.get_logger().info(
                f'PnP 测距已启用: 棋盘格内角点 {self.board_cols}x{self.board_rows}, '
                f'方格边长 {self.square_size * 1000:.1f}mm'
            )

    def _updateCameraModel(self, info: CameraInfo):
        """从 CameraInfo 更新内参并预计算去畸变映射表"""
        K = np.array(info.k, dtype=np.float64).reshape(3, 3)
        D = np.array(info.d, dtype=np.float64)
        w, h = info.width, info.height

        changed = (
            self.K is None
            or not np.allclose(K, self.K)
            or not np.array_equal(D, self.D)
            or w != self.width
            or h != self.height
        )
        if not changed:
            return

        self.K, self.D = K, D
        self.width, self.height = w, h
        # 预计算去畸变+校正映射（alpha=1 保留全部像素，便于观察边缘畸变）
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=1, newImgSize=(w, h))
        self._new_K = new_K
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (w, h), cv2.CV_16SC2
        )

        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        self.get_logger().info(
            f'CameraInfo: {w}x{h} | fx={fx:.2f} fy={fy:.2f} '
            f'cx={cx:.2f} cy={cy:.2f} | '
            f'主点偏移中心: dx={cx - w / 2:+.1f}px dy={cy - h / 2:+.1f}px | '
            f'D={np.array2string(D, precision=4)}'
        )
        self._info_logged = True

    def _drawOverlay(self, img, title):
        """在图像上叠加标题与网格线，网格可帮助目视判断直线是否被拉直"""
        h, w = img.shape[:2]
        # 半透明标题条
        cv2.rectangle(img, (0, 0), (w, 24), (0, 0, 0), -1)
        cv2.putText(img, title, (8, 17), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1, cv2.LINE_AA)
        # 参考网格（去畸变后网格应保持直线）
        step = max(20, w // 10)
        for x in range(step, w, step):
            cv2.line(img, (x, 24), (x, h), (60, 60, 60), 1)
        for y in range(step + 24, h, step):
            cv2.line(img, (0, y), (w, y), (60, 60, 60), 1)
        return img

    def _runPnP(self, frame):
        """检测棋盘格并用 solvePnP 解算相机到标定板的距离。

        返回 (annotated_frame, result_dict)；未检测到棋盘格时 result_dict 为 None。
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, self.board_size, flags)
        if not found:
            return frame, None

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self._criteria)
        cv2.drawChessboardCorners(frame, self.board_size, corners, found)

        ok, rvec, tvec = cv2.solvePnP(
            self.object_points, corners, self.K, self.D,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return frame, None

        # 重投影误差 (RMS)，衡量内参/位姿拟合质量
        proj, _ = cv2.projectPoints(self.object_points, rvec, tvec, self.K, self.D)
        err = np.linalg.norm(corners.reshape(-1, 2) - proj.reshape(-1, 2), axis=1)
        rms = float(np.sqrt(np.mean(err ** 2)))

        # tvec 为相机坐标系下标定板原点位置 (米)；Z 即沿光轴距离
        x, y, z = float(tvec[0]), float(tvec[1]), float(tvec[2])
        distance = float(np.linalg.norm(tvec))  # 直线欧式距离
        return frame, {
            'x': x, 'y': y, 'z': z,
            'distance': distance,
            'rms': rms,
            'origin_px': tuple(corners[0].ravel().astype(int)),
        }

    def _drawPnPResult(self, img, result):
        """将 PnP 测距结果叠加到图像底部"""
        h, w = img.shape[:2]
        if result is None:
            cv2.rectangle(img, (0, h - 26), (w, h), (0, 0, 0), -1)
            cv2.putText(img, 'PnP: chessboard NOT found', (8, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)
            return img

        rms = result['rms']
        # 重投影误差颜色提示: 绿<1px, 黄<2px, 红>=2px
        color = (0, 255, 0) if rms < 1.0 else (0, 255, 255) if rms < 2.0 else (0, 0, 255)
        cv2.rectangle(img, (0, h - 44), (w, h), (0, 0, 0), -1)
        cv2.putText(
            img,
            f'PnP dist={result["distance"] * 100:.1f}cm  Z={result["z"] * 100:.1f}cm',
            (8, h - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA
        )
        cv2.putText(
            img, f'reproj RMS={rms:.3f}px', (8, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA
        )
        return img

    def _syncCallback(self, image_msg: Image, info_msg: CameraInfo):
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge convert failed: {e}')
            return

        self._updateCameraModel(info_msg)
        if self._map1 is None:
            return

        # 图像尺寸与内参声明尺寸不一致则跳过（避免 remap 出错）
        h, w = frame.shape[:2]
        if (w, h) != (self.width, self.height):
            self.get_logger().warn(
                f'图像尺寸 {w}x{h} 与 CameraInfo 尺寸 '
                f'{self.width}x{self.height} 不一致，标定分辨率不匹配！',
                throttle_duration_sec=5.0,
            )
            return

        undistorted = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)

        # 棋盘格 PnP 定量验证（在原图上检测，使用含畸变的 K/D）
        pnp_result = None
        if self.enable_pnp:
            frame, pnp_result = self._runPnP(frame)
            if pnp_result is not None:
                self.get_logger().info(
                    f'PnP: distance={pnp_result["distance"] * 100:.2f}cm '
                    f'(X={pnp_result["x"] * 100:+.2f} Y={pnp_result["y"] * 100:+.2f} '
                    f'Z={pnp_result["z"] * 100:+.2f}cm) | reproj RMS={pnp_result["rms"]:.3f}px',
                    throttle_duration_sec=1.0,
                )

        left = self._drawOverlay(frame.copy(), 'RAW (distorted)')
        right = self._drawOverlay(undistorted.copy(), 'UNDISTORTED')
        if self.enable_pnp:
            left = self._drawPnPResult(left, pnp_result)
        # 中间加分隔线
        sep = np.full((h, 4, 3), (0, 0, 255), dtype=np.uint8)
        compare = np.hstack([left, sep, right])

        if self.save_path:
            try:
                cv2.imwrite(self.save_path, compare)
            except Exception as e:
                self.get_logger().warn(f'save compare image failed: {e}',
                                       throttle_duration_sec=5.0)

        if self.show_window:
            cv2.imshow('calib verify (raw | undistorted)', compare)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibVerifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
