# yolo_onnx_follow_pkg

Standalone YOLOv11 ONNX visual-follow demo.

It subscribes to `/camera/image_raw`, runs ONNX inference with OpenCV DNN,
draws boxes in an OpenCV window, and follows the clicked target with PTZ pan/tilt
plus chassis yaw. It does not publish forward/backward velocity.
