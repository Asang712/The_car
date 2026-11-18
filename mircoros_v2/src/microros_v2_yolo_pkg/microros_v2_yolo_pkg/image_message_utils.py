#!/usr/bin/env python3

from __future__ import annotations

import cv2
import numpy as np
from sensor_msgs.msg import Image


def image_msg_to_bgr(message: Image) -> np.ndarray:
    """Convert common ROS Image encodings to an OpenCV BGR array."""
    encoding = str(message.encoding).strip().lower()
    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid image dimensions: {width}x{height}")

    channel_map = {
        "mono8": 1,
        "8uc1": 1,
        "bgr8": 3,
        "8uc3": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
    }
    channels = channel_map.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported ROS image encoding: {message.encoding}")

    row_bytes = width * channels
    step = int(message.step) or row_bytes
    if step < row_bytes:
        raise ValueError(f"Image step {step} is smaller than row width {row_bytes}")

    data = np.frombuffer(message.data, dtype=np.uint8)
    required = height * step
    if data.size < required:
        raise ValueError(f"Image data has {data.size} bytes; expected at least {required}")

    rows = data[:required].reshape(height, step)[:, :row_bytes]
    if channels == 1:
        return cv2.cvtColor(rows.reshape(height, width), cv2.COLOR_GRAY2BGR)

    image = rows.reshape(height, width, channels)
    if encoding in ("bgr8", "8uc3"):
        return image.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)


def bgr_to_image_msg(image: np.ndarray, header) -> Image:
    """Convert an OpenCV BGR array to a ROS Image message."""
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected a color BGR image with shape (height, width, 3)")

    image = np.ascontiguousarray(image, dtype=np.uint8)
    message = Image()
    message.header = header
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(image.shape[1] * 3)
    message.data = image.tobytes()
    return message
