# -*- coding: utf-8 -*-
"""Pure helpers for action classification and human-readable descriptions.

These helpers have no ROS or node dependencies, so the action dispatch layer
and the decision-plan formatter share one source of truth for "what kind of
action is this?" and "how do I phrase it for the terminal?".
"""

import re

from .feedback_protocol import is_finish_action

#: Re-exported so callers can import every classifier from one place.
is_finishtask = is_finish_action


def _action_root(action_str):
    """Lower-cased action name without parentheses/arguments."""
    return (action_str or "").strip().lower().split("(", maxsplit=1)[0]


def is_follow_line(action_str) -> bool:
    """Return whether the action launches visual line following."""
    return _action_root(action_str).startswith("follow_line")


def is_stop_action(action_str) -> bool:
    """Return whether the action stops motion."""
    return _action_root(action_str).startswith("stop")


def is_visual_follow(action_str) -> bool:
    """Return whether the action is a visual track/follow (manages its own PTZ)."""
    return _action_root(action_str).startswith("track_kcf")


def is_servo_action(action_str) -> bool:
    """Return whether an action directly controls a camera servo."""
    return _action_root(action_str) in {
        "servo1_move",
        "servo2_move",
        "servo_init",
        "servo_shake",
        "servo_nod",
    }


def describe_action(action: str) -> str:
    """Turn an action function string into a Chinese description for the plan.

    Unrecognized actions fall back to the raw string so any action can print.
    """
    action = (action or "").strip()
    match = re.fullmatch(r"(\w+)\((.*)\)", action)
    if match:
        name, args_str = match.groups()
    else:
        name, args_str = action, ""
    args = [a.strip().strip("'\"") for a in args_str.split(",") if a.strip()]
    arg_str = ", ".join(args) if args else ""

    descriptions = {
        "navigation": lambda: f"导航去{args[0]}点" if args else "导航",
        "follow_line": lambda: f"{args[0]}色巡线前进" if args else "红色巡线前进",
        "track_kcf": lambda: (
            "取消KCF追踪"
            if args and args[0].lower() == "cancel"
            else (f"KCF追踪目标框({arg_str})" if args else "KCF目标追踪")
        ),
        "seewhat": lambda: "拍摄当前画面并反馈",
        "observe_surroundings": lambda: "拍摄周围多方向画面并反馈",
        "move_forward": lambda: (
            f"前进(速度{args[0]}，{args[1]}秒)" if len(args) >= 2 else "前进"
        ),
        "move_forward_distance": lambda: (
            f"前进{args[0]}米" if args else "按里程计前进"
        ),
        "move_backward_distance": lambda: (
            f"后退{args[0]}米" if args else "按里程计后退"
        ),
        "move_backward": lambda: (
            f"后退(速度{args[0]}，{args[1]}秒)" if len(args) >= 2 else "后退"
        ),
        "turn_left": lambda: f"左转{args[0]}度" if args else "左转",
        "turn_right": lambda: f"右转{args[0]}度" if args else "右转",
        "stop": lambda: "停止运动",
        "push_object": lambda: "低速向前推进触碰目标",
        "push_contact": lambda: "持续推进直到顶住目标",
        "servo1_move": lambda: f"1号舵机转到{args[0]}度" if args else "移动1号舵机",
        "servo2_move": lambda: f"2号舵机转到{args[0]}度" if args else "移动2号舵机",
        "servo_init": lambda: "云台舵机复位",
        "servo_shake": lambda: "云台摇头",
        "servo_nod": lambda: "云台点头",
        "get_current_pose": lambda: "获取当前位置坐标",
        "finishtask": lambda: "任务完成",
    }

    builder = descriptions.get(name)
    if builder:
        try:
            return builder()
        except Exception:
            return action
    return action

