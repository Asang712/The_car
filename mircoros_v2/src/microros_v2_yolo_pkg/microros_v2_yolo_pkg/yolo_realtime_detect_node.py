#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from .image_message_utils import bgr_to_image_msg, image_msg_to_bgr

from .yolo_common import (
    OUTPUT_VIDEO_DIR,
    resolve_model_path,
    build_output_path,
    check_file_exists,
    ensure_dir,
    import_yolo,
    as_bool,
)


class YoloRealtimeDetectNode(Node):
    def __init__(self):
        super().__init__("yolo_realtime_detect_node")

        self.declare_parameter("model_name", "yolo11n")
        self.declare_parameter("model_path", "")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/yolo/detect/image")

        # Optimized according to your tested yolo track command.
        self.declare_parameter("conf", 0.05)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_det", 1000)

        # Detection is the default; tracking remains available when requested.
        self.declare_parameter("use_tracking", False)
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("persist", True)

        # Match vid_stride=1 idea in realtime topic mode:
        # process every image frame by default.
        self.declare_parameter("process_every_n_frames", 1)

        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "YOLO Realtime Detection")
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 540)

        self.declare_parameter("save_video", False)
        self.declare_parameter("output_dir", str(OUTPUT_VIDEO_DIR))
        self.declare_parameter("output_video_name", "realtime_yolo_detect.mp4")
        self.declare_parameter("save_video_fps", 15.0)
        self.declare_parameter("fourcc", "mp4v")

        self.model = None

        self.frame_index = 0
        self.processed_count = 0
        self.start_time = time.time()

        self.video_writer = None
        self.video_output_path = None

        self._load_parameters()
        self._load_model()
        self._setup_ros_io()

    def _load_parameters(self):
        self.model_name = self.get_parameter("model_name").value
        self.model_path_param = self.get_parameter("model_path").value

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)

        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.max_det = int(self.get_parameter("max_det").value)

        self.use_tracking = as_bool(self.get_parameter("use_tracking").value)
        self.tracker = str(self.get_parameter("tracker").value).strip()
        self.persist = as_bool(self.get_parameter("persist").value)

        if not self.tracker:
            self.tracker = "bytetrack.yaml"

        self.process_every_n_frames = max(
            1,
            int(self.get_parameter("process_every_n_frames").value),
        )

        self.show_window = as_bool(self.get_parameter("show_window").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)

        self.save_video = as_bool(self.get_parameter("save_video").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.output_video_name = str(self.get_parameter("output_video_name").value)
        self.save_video_fps = float(self.get_parameter("save_video_fps").value)
        self.fourcc_text = str(self.get_parameter("fourcc").value)

        if len(self.fourcc_text) < 4:
            self.fourcc_text = "mp4v"

        self.model_path = resolve_model_path(self.model_name, self.model_path_param)

    def _load_model(self):
        check_file_exists(self.model_path, "YOLO model")

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        YOLO = import_yolo()
        self.model = YOLO(str(self.model_path))

        self.get_logger().info(
            f"YOLO realtime node ready. "
            f"mode={'track' if self.use_tracking else 'detect'}, "
            f"tracker={self.tracker if self.use_tracking else 'none'}, "
            f"persist={self.persist}, device={self.device}"
        )
        self.get_logger().info(
            f"YOLO params: imgsz={self.imgsz}, conf={self.conf}, iou={self.iou}, "
            f"max_det={self.max_det}, process_every_n_frames={self.process_every_n_frames}"
        )

    def _setup_ros_io(self):
        self.publisher = self.create_publisher(Image, self.output_topic, 10)

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Subscribed image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing result image topic: {self.output_topic}")

    def _run_yolo(self, frame):
        if self.use_tracking:
            return self.model.track(
                source=frame,
                persist=self.persist,
                tracker=self.tracker,
                conf=self.conf,
                iou=self.iou,
                imgsz=self.imgsz,
                device=self.device,
                max_det=self.max_det,
                verbose=False,
            )

        return self.model.predict(
            source=frame,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            max_det=self.max_det,
            verbose=False,
        )

    def _init_video_writer_if_needed(self, frame):
        if not self.save_video:
            return

        if self.video_writer is not None:
            return

        height, width = frame.shape[:2]
        ensure_dir(Path(self.output_dir))

        output_candidate = Path(self.output_dir) / self.output_video_name
        task_suffix = "_track" if self.use_tracking else "_detect"

        self.video_output_path = build_output_path(
            input_path=output_candidate,
            output_dir=self.output_dir,
            model_path=self.model_path,
            task_suffix=task_suffix,
            output_path="",
            force_ext=".mp4",
        )

        fourcc = cv2.VideoWriter_fourcc(*self.fourcc_text[:4])
        self.video_writer = cv2.VideoWriter(
            str(self.video_output_path),
            fourcc,
            self.save_video_fps,
            (width, height),
        )

        if not self.video_writer.isOpened():
            self.get_logger().error(f"Failed to open video writer: {self.video_output_path}")
            self.video_writer = None
            self.save_video = False
            return

        self.get_logger().info(f"Saving realtime output video to: {self.video_output_path}")

    def image_callback(self, msg: Image):
        self.frame_index += 1

        if self.frame_index % self.process_every_n_frames != 0:
            return

        try:
            frame = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"ROS image conversion failed: {exc}")
            return

        try:
            results = self._run_yolo(frame)

            if results:
                annotated = results[0].plot()
            else:
                annotated = frame

        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        try:
            out_msg = bgr_to_image_msg(annotated, msg.header)
            self.publisher.publish(out_msg)
        except Exception as exc:
            self.get_logger().error(f"publish result image failed: {exc}")

        self._init_video_writer_if_needed(annotated)

        if self.video_writer is not None:
            self.video_writer.write(annotated)

        if self.show_window:
            if self.display_width > 0 and self.display_height > 0:
                show_img = cv2.resize(annotated, (self.display_width, self.display_height))
            else:
                show_img = annotated

            cv2.imshow(self.window_name, show_img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                self.get_logger().info("q/ESC pressed, shutting down.")
                os._exit(0)  # pylint: disable=protected-access

        self.processed_count += 1

        if self.processed_count % 30 == 0:
            elapsed = max(1e-6, time.time() - self.start_time)
            self.get_logger().info(
                f"received_frames={self.frame_index}, "
                f"processed_frames={self.processed_count}, "
                f"processed_fps={self.processed_count / elapsed:.2f}"
            )

    def destroy_node(self):
        """ROS 2 cleanup — release OpenCV resources then call super."""
        self.cleanup()
        super().destroy_node()

    def cleanup(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

            if self.video_output_path is not None:
                self.get_logger().info(f"Realtime video saved to: {self.video_output_path}")

        if self.show_window:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = YoloRealtimeDetectNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc))

    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
