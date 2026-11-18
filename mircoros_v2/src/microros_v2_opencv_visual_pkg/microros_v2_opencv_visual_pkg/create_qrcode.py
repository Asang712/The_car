#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
create_qrcode.py



English Description:
    ROS2 Humble QR code creation node.
    The QR generation method follows the original program:
    qrcode.QRCode + PIL logo overlay + save/show image.

    Changes:
    default path uses microros_v2_opencv_visual_pkg/config, qr_text can be passed by ROS parameter,
    and the node exits after the QR image is generated.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import qrcode
import rclpy
from ament_index_python.packages import get_package_share_directory
from PIL import Image
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# Default node name.
DEFAULT_NODE_NAME = "create_qrcode_node"


def get_default_config_directory() -> str:

    return os.path.join(get_package_share_directory(VISUAL_PACKAGE_NAME), "config")


def convert_value_to_boolean(input_value, default_boolean_value: bool) -> bool:
    """
    Safely convert value to bool.
    """
    if isinstance(input_value, bool):
        return input_value
    if isinstance(input_value, str):
        normalized_text_value = input_value.strip().lower()
        if normalized_text_value in ("true", "1", "yes", "y", "on"):
            return True
        if normalized_text_value in ("false", "0", "no", "n", "off"):
            return False
    if isinstance(input_value, (int, float)):
        return bool(input_value)
    return default_boolean_value


class QRCodeNode(Node):
    """QR code creation node."""

    def __init__(self) -> None:
        """Initialize node and create QR code."""
        super().__init__(DEFAULT_NODE_NAME)

        default_config_directory = get_default_config_directory()

        self.declare_parameter("file_path", default_config_directory)
        self.declare_parameter("logo_filename", "yahboom_logo.png")
        self.declare_parameter("output_filename", "yahboom_logo_qr.jpg")
        self.declare_parameter(
            "qr_text",
            "",
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("fill_color", "green")
        self.declare_parameter("back_color", "white")
        self.declare_parameter("show_image", True)
        self.declare_parameter("qr_version", 1)
        self.declare_parameter("box_size", 10)
        self.declare_parameter("border", 4)
        self.declare_parameter("logo_scale_factor", 6)

        self.file_path = os.path.expanduser(str(self.get_parameter("file_path").value))
        self.logo_filename = str(self.get_parameter("logo_filename").value)
        self.output_filename = str(self.get_parameter("output_filename").value)
        raw_qr_text_value = self.get_parameter("qr_text").value
        self.qr_text = "" if raw_qr_text_value is None else str(raw_qr_text_value)
        self.fill_color = str(self.get_parameter("fill_color").value)
        self.back_color = str(self.get_parameter("back_color").value)
        self.show_image = convert_value_to_boolean(self.get_parameter("show_image").value, True)
        self.qr_version = max(1, min(int(self.get_parameter("qr_version").value), 40))
        self.box_size = max(1, int(self.get_parameter("box_size").value))
        self.border = max(0, int(self.get_parameter("border").value))
        self.logo_scale_factor = max(1, int(self.get_parameter("logo_scale_factor").value))

        if not self.qr_text:
            self.qr_text = input("Enter the content to be printed by the QR code scan:")

        if not self.qr_text:
            self.get_logger().error("QR text is empty. No QR code was created.")
            return

        output_file_path = os.path.join(self.file_path, self.output_filename)
        logo_file_path = os.path.join(self.file_path, self.logo_filename)

        self.create_qrcode(
            self.qr_text,
            output_file_path,
            logo_file_path,
        )

    def add_logo(self, img, logo_path: str):
        """
        Add logo to the QR code center.

        Args:
            QR code image.
            Logo image path.

        Returns:
            QR code image with logo.
        """
        # Add logo, open logo image
        icon = Image.open(logo_path)
        img_w, img_h = img.size

        # Set the size of the logo
        factor = max(1, int(self.logo_scale_factor))
        size_w = int(img_w / factor)
        size_h = int(img_h / factor)
        icon_w, icon_h = icon.size

        if icon_w > size_w:
            icon_w = size_w
        if icon_h > size_h:
            icon_h = size_h

        # Resize the logo
        try:
            icon = icon.resize((icon_w, icon_h), Image.Resampling.LANCZOS)
        except AttributeError:
            icon = icon.resize((icon_w, icon_h), Image.LANCZOS)

        # Center the logo
        w = int((img_w - icon_w) / 2)
        h = int((img_h - icon_h) / 2)

        # Paste the logo
        img.paste(icon, (w, h), mask=None)

        return img

    def create_qrcode(self, data: str, file_name: str, logo_path: Optional[str] = None):
        """
        Create QR code.

            version=1
            error_correction=qrcode.constants.ERROR_CORRECT_H
            box_size=10
            border=4

        Args:
            QR code payload.
            Output image path.
            Logo image path.

        Returns:
            Generated QR image.
        """
        output_path = Path(os.path.expanduser(file_name))
        output_path.parent.mkdir(parents=True, exist_ok=True)

        qr = qrcode.QRCode(
            version=int(self.qr_version),
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=int(self.box_size),
            border=int(self.border),
        )

        # Add data to the QR code
        qr.add_data(data)
        qr.make(fit=True)
        img = qr.make_image(fill_color=self.fill_color, back_color=self.back_color)

        # Add logo if file exists
        if logo_path and Path(os.path.expanduser(logo_path)).is_file():
            img = self.add_logo(img, os.path.expanduser(logo_path))
            self.get_logger().info(f"Logo added: {os.path.expanduser(logo_path)}")
        else:
            self.get_logger().warn(f"Logo file not found, QR code will be generated without logo: {logo_path}")

        # Save and optionally show the image
        img.save(str(output_path))
        self.get_logger().info(f"QR code saved: {output_path}")

        if self.show_image:
            try:
                img.show()
            except Exception as exception_error:  # pylint: disable=broad-except
                self.get_logger().warn(f"Show QR image failed: {exception_error}")

        return img


def main(args=None) -> None:
    """
    Node entry function.
    """
    rclpy.init(args=args)
    node = QRCodeNode()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
