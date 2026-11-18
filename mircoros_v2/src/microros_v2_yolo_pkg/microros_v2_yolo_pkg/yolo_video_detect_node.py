#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
from importlib.util import find_spec

import cv2
import rclpy
from rclpy.node import Node

from .yolo_common import (
    RAW_VIDEO_DIR,
    OUTPUT_VIDEO_DIR,
    resolve_model_path,
    resolve_input_path,
    build_output_path,
    check_file_exists,
    ensure_dir,
    import_yolo,
    as_bool,
)


class YoloVideoDetectNode(Node):
    def __init__(self):
        super().__init__("yolo_video_detect_node")

        self.declare_parameter("model_name", "yolo11n")
        self.declare_parameter("model_path", "")

        self.declare_parameter("input_dir", str(RAW_VIDEO_DIR))
        self.declare_parameter("video_name", "street.mp4")
        self.declare_parameter("input_video", "")

        self.declare_parameter("output_dir", str(OUTPUT_VIDEO_DIR))
        self.declare_parameter("output_video", "")

        # Optimized according to your tested yolo track command.
        self.declare_parameter("conf", 0.05)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_det", 1000)

        self.declare_parameter("vid_stride", 1)
        self.declare_parameter("max_frames", 0)

        # Detection is the default; tracking remains available when requested.
        self.declare_parameter("use_tracking", False)
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("persist", True)

        self.declare_parameter("show_window", False)
        self.declare_parameter("window_name", "YOLO Video Detection")
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 540)

        self.declare_parameter("fourcc", "mp4v")
        self.declare_parameter("output_fps", 0.0)

    def _run_yolo(self, model, frame, use_tracking, tracker, persist,
                  conf, iou, imgsz, device, max_det):
        if use_tracking:
            return model.track(
                source=frame,
                persist=persist,
                tracker=tracker,
                conf=conf,
                iou=iou,
                imgsz=imgsz,
                device=device,
                max_det=max_det,
                verbose=False,
            )

        return model.predict(
            source=frame,
            conf=conf,
            iou=iou,
            imgsz=imgsz,
            device=device,
            max_det=max_det,
            verbose=False,
        )

    def run(self) -> int:
        model_name = self.get_parameter("model_name").value
        model_path_param = self.get_parameter("model_path").value

        input_dir = self.get_parameter("input_dir").value
        video_name = self.get_parameter("video_name").value
        input_video = self.get_parameter("input_video").value

        output_dir = self.get_parameter("output_dir").value
        output_video = self.get_parameter("output_video").value

        conf = float(self.get_parameter("conf").value)
        iou = float(self.get_parameter("iou").value)
        imgsz = int(self.get_parameter("imgsz").value)
        device = str(self.get_parameter("device").value)
        max_det = int(self.get_parameter("max_det").value)

        vid_stride = max(1, int(self.get_parameter("vid_stride").value))
        max_frames = int(self.get_parameter("max_frames").value)

        use_tracking = as_bool(self.get_parameter("use_tracking").value)
        tracker = str(self.get_parameter("tracker").value).strip()
        persist = as_bool(self.get_parameter("persist").value)

        show_window = as_bool(self.get_parameter("show_window").value)
        window_name = str(self.get_parameter("window_name").value)
        display_width = int(self.get_parameter("display_width").value)
        display_height = int(self.get_parameter("display_height").value)

        fourcc_text = str(self.get_parameter("fourcc").value)
        if len(fourcc_text) < 4:
            fourcc_text = "mp4v"

        output_fps_param = float(self.get_parameter("output_fps").value)

        if not tracker:
            tracker = "bytetrack.yaml"

        if use_tracking and find_spec("lap") is None:
            self.get_logger().warning(
                "Tracking requires the optional dependency 'lap>=0.5.12', "
                "which is not installed. Falling back to detection mode."
            )
            use_tracking = False

        model_path = resolve_model_path(model_name, model_path_param)
        video_path = resolve_input_path(
            input_video,
            input_dir,
            video_name,
            "street.mp4",
        )

        task_suffix = "_track" if use_tracking else "_detect"

        output_path = build_output_path(
            input_path=video_path,
            output_dir=output_dir,
            model_path=model_path,
            task_suffix=task_suffix,
            output_path=output_video,
            force_ext=".mp4",
        )

        cap = None
        writer = None

        try:
            check_file_exists(model_path, "YOLO model")
            check_file_exists(video_path, "input video")
            ensure_dir(output_path.parent)

            self.get_logger().info(f"Loading YOLO model: {model_path}")
            YOLO = import_yolo()
            model = YOLO(str(model_path))

            self.get_logger().info(f"Opening video: {video_path}")
            cap = cv2.VideoCapture(str(video_path))

            if not cap.isOpened():
                self.get_logger().error(f"Failed to open input video: {video_path}")
                return 1

            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            source_fps = float(cap.get(cv2.CAP_PROP_FPS))

            if frame_width <= 0 or frame_height <= 0:
                self.get_logger().error("Invalid video frame size.")
                return 1

            if source_fps <= 0:
                source_fps = 25.0

            output_fps = output_fps_param if output_fps_param > 0.0 else source_fps

            fourcc = cv2.VideoWriter_fourcc(*fourcc_text[:4])
            writer = cv2.VideoWriter(
                str(output_path),
                fourcc,
                output_fps,
                (frame_width, frame_height),
            )

            if not writer.isOpened():
                self.get_logger().error(f"Failed to open output video writer: {output_path}")
                return 1

            self.get_logger().info(
                f"Mode={'track' if use_tracking else 'detect'}, "
                f"tracker={tracker if use_tracking else 'none'}, "
                f"persist={persist}, device={device}"
            )
            self.get_logger().info(
                f"YOLO params: imgsz={imgsz}, conf={conf}, iou={iou}, "
                f"max_det={max_det}, vid_stride={vid_stride}"
            )
            self.get_logger().info(
                f"Video info: width={frame_width}, height={frame_height}, "
                f"source_fps={source_fps:.2f}, output_fps={output_fps:.2f}"
            )
            self.get_logger().info(f"Output video: {output_path}")

            frame_index = 0
            processed_count = 0
            start_time = time.time()

            while rclpy.ok() and cap.isOpened():
                success, frame = cap.read()
                if not success:
                    break

                frame_index += 1

                if max_frames > 0 and frame_index > max_frames:
                    break

                if frame_index % vid_stride == 0:
                    results = self._run_yolo(
                        model=model,
                        frame=frame,
                        use_tracking=use_tracking,
                        tracker=tracker,
                        persist=persist,
                        conf=conf,
                        iou=iou,
                        imgsz=imgsz,
                        device=device,
                        max_det=max_det,
                    )

                    if results:
                        annotated = results[0].plot()
                    else:
                        annotated = frame

                    processed_count += 1
                else:
                    annotated = frame

                writer.write(annotated)

                if show_window:
                    if display_width > 0 and display_height > 0:
                        show_img = cv2.resize(annotated, (display_width, display_height))
                    else:
                        show_img = annotated

                    cv2.imshow(window_name, show_img)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.get_logger().info("User pressed q, stopping video processing.")
                        break

                if frame_index % 30 == 0:
                    elapsed = max(1e-6, time.time() - start_time)
                    self.get_logger().info(
                        f"frames={frame_index}, yolo_processed={processed_count}, "
                        f"avg_loop_fps={frame_index / elapsed:.2f}"
                    )

            elapsed = max(1e-6, time.time() - start_time)
            self.get_logger().info(
                f"Finished. total_frames={frame_index}, "
                f"yolo_processed_frames={processed_count}, "
                f"elapsed={elapsed:.2f}s, avg_loop_fps={frame_index / elapsed:.2f}"
            )
            self.get_logger().info(f"Output video saved to: {output_path}")

            return 0

        except Exception as exc:
            self.get_logger().error(str(exc))
            return 1

        finally:
            if cap is not None:
                cap.release()

            if writer is not None:
                writer.release()

            if show_window:
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YoloVideoDetectNode()

    try:
        ret = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(ret)


if __name__ == "__main__":
    main()
