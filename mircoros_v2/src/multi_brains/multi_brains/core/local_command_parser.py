# -*- coding: utf-8 -*-
"""离线中文固定指令解析：把常用指令转换为现有动作调用。"""

import math
import re
import threading


NUMBER_PATTERN = r"(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百半]+(?:点[零〇一二两三四五六七八九]+)?)"
CHINESE_DIGITS = {
    "零": 0,
    "〇": 0,
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}
CHINESE_UNITS = {"十": 10, "百": 100}
COLOR_KEYWORDS = (
    ("红", "red"),
    ("橙", "orange"),
    ("黄", "yellow"),
    ("绿", "green"),
    ("蓝", "blue"),
    ("紫", "purple"),
    ("red", "red"),
    ("orange", "orange"),
    ("yellow", "yellow"),
    ("green", "green"),
    ("blue", "blue"),
    ("purple", "purple"),
)
REPEAT_COMMANDS = {
    "重复",
    "重复一次",
    "重复上一次",
    "重复上一次指令",
    "重复上一条",
    "重复上一条指令",
    "再来一次",
    "再执行一次",
    "再执行一遍",
    "再做一次",
    "again",
}
STOP_COMMANDS = {
    "停止",
    "停止运动",
    "停下",
    "停下来",
    "停车",
    "急停",
    "别动",
    "别动了",
    "stop",
}
PUSH_COMMANDS = {"推一下", "碰一下", "顶一下"}
SUPPORTED_EXAMPLES = (
    "前进1米、后退0.5米、左转90度、停止、重复上一次指令、找红色物体、推一下"
)


def _parse_chinese_integer(text: str) -> float:
    result = 0
    current = 0
    for char in text:
        if char in CHINESE_DIGITS:
            current = CHINESE_DIGITS[char]
        elif char in CHINESE_UNITS:
            unit = CHINESE_UNITS[char]
            result += (current or 1) * unit
            current = 0
        else:
            raise ValueError(f"无法识别数字: {text}")
    return float(result + current)


def parse_number(text: str) -> float:
    """解析阿拉伯数字及常见中文数字，如一米、九十度、零点五米。"""
    text = str(text).strip()
    try:
        value = float(text)
    except ValueError:
        if text == "半":
            value = 0.5
        elif "点" in text:
            integer_text, fraction_text = text.split("点", maxsplit=1)
            integer = _parse_chinese_integer(integer_text) if integer_text else 0.0
            fraction = "".join(str(CHINESE_DIGITS[char]) for char in fraction_text)
            value = float(f"{int(integer)}.{fraction}")
        else:
            value = _parse_chinese_integer(text)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("数值必须是大于0的有限数字")
    return value


def format_number(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".")


class LocalCommandParser:
    """无 ROS、无网络依赖的固定指令解析器。"""

    def __init__(self, config=None):
        config = config or {}
        motion = config.get("MOTION", {})
        self.linear_speed = float(motion.get("default_linear_speed", 0.2))
        self.angular_speed = float(motion.get("default_angular_speed", 0.6))
        self.max_duration = float(motion.get("max_duration_sec", 120.0))
        self.max_distance = float(motion.get("max_distance_m", 5.0))
        self.max_turn_angle = float(motion.get("max_turn_angle_deg", 720.0))
        self._last_actions = []
        self._history_lock = threading.Lock()

    def reset_history(self):
        with self._history_lock:
            self._last_actions = []

    def parse(self, text: str) -> list:
        """返回动作字符串列表；无法识别时抛出带中文提示的 ValueError。"""
        normalized = re.sub(r"[ \t　]+", "", str(text or "").strip().lower())
        normalized = re.sub(r"^(?:请你?|麻烦你?|帮我|帮忙|给我)+", "", normalized)
        normalized = normalized.rstrip("。！!")

        if normalized in REPEAT_COMMANDS:
            with self._history_lock:
                if not self._last_actions:
                    raise ValueError("还没有可重复的上一条有效指令")
                return list(self._last_actions)

        segments = [
            segment
            for segment in re.split(
                r"(?:然后|接着|之后|随后|，|,|；|;|\n|"
                r"再(?=(?:向|往)?(?:前|后|左|右)|找|寻找|停止|推|碰|顶))",
                normalized,
            )
            if segment
        ]
        if not segments:
            raise ValueError(f"请输入指令，例如：{SUPPORTED_EXAMPLES}")

        actions = []
        for segment in segments:
            actions.extend(self._parse_segment(segment))

        with self._history_lock:
            self._last_actions = list(actions)
        return actions

    def _parse_segment(self, segment: str) -> list:
        command = re.sub(r"^(?:立即|马上|立刻|现在)", "", segment)
        if command in STOP_COMMANDS:
            return ["stop()"]

        if command in PUSH_COMMANDS:
            return ["push_object()"]

        if "找" in command or "搜索" in command:
            for keyword, color in COLOR_KEYWORDS:
                if keyword in command:
                    return [f"find_object({color})"]
            raise ValueError("本地找物需要指定颜色，例如：找红色物体；支持红、橙、黄、绿、蓝、紫")

        motion = self._parse_linear_motion(command)
        if motion is not None:
            return [motion]

        turn = self._parse_turn(command)
        if turn is not None:
            return [turn]

        raise ValueError(f"无法识别“{segment}”。本地模式支持：{SUPPORTED_EXAMPLES}")

    def _parse_linear_motion(self, command: str):
        match = re.fullmatch(
            rf"(?:向|往)?(?P<direction>前|后)(?:进|退|行|走|移动)?"
            rf"(?:(?P<value>{NUMBER_PATTERN})(?P<unit>厘米|cm|米|m|秒|s))?",
            command,
        )
        if not match:
            return None

        is_forward = match.group("direction") == "前"
        action_name = "move_forward" if is_forward else "move_backward"
        value_text = match.group("value")
        unit = match.group("unit")
        if value_text is None:
            self._validate_duration(1.0)
            return f"{action_name}(1)"

        value = parse_number(value_text)
        if unit in ("秒", "s"):
            self._validate_duration(value)
            return f"{action_name}({format_number(value)})"

        distance_m = value / 100.0 if unit in ("厘米", "cm") else value
        self._validate_distance(distance_m)
        distance_action = (
            "move_forward_distance" if is_forward else "move_backward_distance"
        )
        return f"{distance_action}({format_number(distance_m)})"

    def _parse_turn(self, command: str):
        match = re.fullmatch(
            rf"(?:向|往)?(?P<direction>左|右)(?:转弯|转|转动)?"
            rf"(?:(?P<value>{NUMBER_PATTERN})(?P<unit>度|°|弧度|rad))?",
            command,
        )
        if not match:
            return None

        value = parse_number(match.group("value")) if match.group("value") else 90.0
        unit = match.group("unit") or "度"
        angle_deg = math.degrees(value) if unit in ("弧度", "rad") else value
        if angle_deg > self.max_turn_angle:
            raise ValueError(f"转向角度超过安全上限{format_number(self.max_turn_angle)}度")
        angle_rad = math.radians(angle_deg)
        duration = angle_rad / self.angular_speed
        self._validate_duration(duration)
        action_name = "turn_left" if match.group("direction") == "左" else "turn_right"
        return f"{action_name}({format_number(duration)})"

    def _validate_distance(self, distance: float):
        if not math.isfinite(distance) or distance <= 0.0:
            raise ValueError("运动距离必须大于0")
        if distance > self.max_distance:
            raise ValueError(
                f"运动距离超过安全上限{format_number(self.max_distance)}米"
            )

    def _validate_duration(self, duration: float):
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("运动时长必须大于0")
        if duration > self.max_duration:
            raise ValueError(f"运动时长超过安全上限{format_number(self.max_duration)}秒")
