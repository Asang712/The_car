#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Utilities for reading/writing ROS 2 parameter YAML files used by microros_v2_patrol_node.

This version supports independent angular calibration for left/right turns:
- odom_angular_left_scale_correction:  positive yaw / left turn
- odom_angular_right_scale_correction: negative yaw / right turn

It also keeps backward compatibility with the legacy single parameter:
- odom_angular_scale_correction
"""

import os
from typing import Any, Optional

import yaml


def _normalize_node_name(node_name: str) -> str:
    return str(node_name).strip().lstrip("/") or "microros_v2_patrol_node"


def read_ros_parameter_yaml(
    yaml_path: str,
    target_node_name: str,
) -> dict:
    yaml_path = os.path.abspath(os.path.expanduser(str(yaml_path)))
    if not os.path.exists(yaml_path):
        return {}

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        return {}

    node_key = _normalize_node_name(target_node_name)
    node_block = data.get(node_key, {})
    if not isinstance(node_block, dict):
        return {}

    params = node_block.get("ros__parameters", {})
    return params if isinstance(params, dict) else {}


def read_yaml_value(
    yaml_path: str,
    target_node_name: str,
    parameter_name: str,
    default_value: Any,
) -> Any:
    params = read_ros_parameter_yaml(yaml_path, target_node_name)
    return params.get(parameter_name, default_value)


def update_odom_calibration_yaml(
    yaml_path: str,
    target_node_name: str = "microros_v2_patrol_node",
    linear_scale: Optional[float] = None,

    # Legacy angular parameter. If used, it updates the legacy key only.
    angular_scale: Optional[float] = None,

    # New independent angular parameters.
    angular_left_scale: Optional[float] = None,
    angular_right_scale: Optional[float] = None,

    enable_odom_calibration: Optional[bool] = None,
    scale_odom_covariance: Optional[bool] = None,
) -> None:
    """Create or update one ROS 2 parameter YAML file.

    Example angular YAML:
        microros_v2_patrol_node:
          ros__parameters:
            odom_angular_left_scale_correction: 1.02
            odom_angular_right_scale_correction: 1.05
    """
    yaml_path = os.path.abspath(os.path.expanduser(str(yaml_path)))
    parent = os.path.dirname(yaml_path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    data = {}
    if os.path.exists(yaml_path):
        with open(yaml_path, "r", encoding="utf-8") as f:
            loaded = yaml.safe_load(f)
            if isinstance(loaded, dict):
                data = loaded

    node_key = _normalize_node_name(target_node_name)
    node_block = data.setdefault(node_key, {})
    params = node_block.setdefault("ros__parameters", {})

    if enable_odom_calibration is not None:
        params["enable_odom_calibration"] = bool(enable_odom_calibration)

    if scale_odom_covariance is not None:
        params["scale_odom_covariance"] = bool(scale_odom_covariance)

    if linear_scale is not None:
        params["odom_linear_scale_correction"] = float(linear_scale)

    if angular_scale is not None:
        params["odom_angular_scale_correction"] = float(angular_scale)

    if angular_left_scale is not None:
        params["odom_angular_left_scale_correction"] = float(angular_left_scale)

    if angular_right_scale is not None:
        params["odom_angular_right_scale_correction"] = float(angular_right_scale)

    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
