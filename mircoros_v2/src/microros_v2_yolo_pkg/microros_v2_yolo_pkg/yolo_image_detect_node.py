#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import cv2
import rclpy
from rclpy.node import Node

from .yolo_common import (
    RAW_IMAGE_DIR,
    OUTPUT_IMAGE_DIR,
    resolve_model_path,
    resolve_input_path,
    build_output_path,
    check_file_exists,
    ensure_dir,
    import_yolo,
)


class YoloImageDetectNode(Node):
    def __init__(self):
        super().__init__("yolo_image_detect_node")

        self.declare_parameter("model_name", "yolo11n")
        self.declare_parameter("model_path", "")

        self.declare_parameter("input_dir", str(RAW_IMAGE_DIR))
        self.declare_parameter("image_name", "bus.jpg")
        self.declare_parameter("input_image", "")

        self.declare_parameter("output_dir", str(OUTPUT_IMAGE_DIR))
        self.declare_parameter("output_image", "")

        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_det", 300)

        self.declare_parameter("show_window", False)
        self.declare_parameter("window_name", "YOLO Image Detection")

    def run(self) -> int:
        model_name = self.get_parameter("model_name").value
        model_path_param = self.get_parameter("model_path").value

        input_dir = self.get_parameter("input_dir").value
        image_name = self.get_parameter("image_name").value
        input_image = self.get_parameter("input_image").value

        output_dir = self.get_parameter("output_dir").value
        output_image = self.get_parameter("output_image").value

        conf = float(self.get_parameter("conf").value)
        iou = float(self.get_parameter("iou").value)
        imgsz = int(self.get_parameter("imgsz").value)
        device = str(self.get_parameter("device").value)
        max_det = int(self.get_parameter("max_det").value)

        show_window = bool(self.get_parameter("show_window").value)
        window_name = str(self.get_parameter("window_name").value)

        model_path = resolve_model_path(model_name, model_path_param)
        image_path = resolve_input_path(input_image, input_dir, image_name, "bus.jpg")
        output_path = build_output_path(
            input_path=image_path,
            output_dir=output_dir,
            model_path=model_path,
            task_suffix="_detect",
            output_path=output_image,
            force_ext=".jpg",
        )

        try:
            check_file_exists(model_path, "YOLO model")
            check_file_exists(image_path, "input image")
            ensure_dir(output_path.parent)

            self.get_logger().info(f"Loading YOLO model: {model_path}")
            YOLO = import_yolo()
            model = YOLO(str(model_path))

            self.get_logger().info(f"Running image detection: {image_path}")
            results = model.predict(
                source=str(image_path),
                conf=conf,
                iou=iou,
                imgsz=imgsz,
                device=device,
                max_det=max_det,
                verbose=False,
            )

            if not results:
                self.get_logger().error("YOLO returned empty result.")
                return 1

            annotated = results[0].plot()

            ok = cv2.imwrite(str(output_path), annotated)
            if not ok:
                self.get_logger().error(f"Failed to save output image: {output_path}")
                return 1

            boxes = results[0].boxes
            box_count = 0 if boxes is None else len(boxes)

            self.get_logger().info(f"Detected objects: {box_count}")
            self.get_logger().info(f"Output image saved to: {output_path}")

            if show_window:
                cv2.imshow(window_name, annotated)
                self.get_logger().info("Press any key in the image window to close.")
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            return 0

        except Exception as exc:
            self.get_logger().error(str(exc))
            return 1


def main(args=None):
    rclpy.init(args=args)
    node = YoloImageDetectNode()

    try:
        ret = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(ret)


if __name__ == "__main__":
    main()