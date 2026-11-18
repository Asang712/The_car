import os
import json

_ROBOT_INFO_PATH = os.path.join(
    os.path.expanduser("~"), "microros_ws", ".system_cfg", "robot_info.json")

_LANGUAGE_KEY = "language"
DEFAULT_LANGUAGE = "zh"

# Language code -> display name shown in the language selector.
LANGUAGES = {
    "zh": "中文",
    "en": "English",
}

TRANSLATIONS = {
    # ------------------------------------------------------------------ zh
    "zh": {
        # App
        "app.subtitle": "MicroRosV2 机器人管理器",
        "tab.info": "信息",
        "tab.host": "主机设置",
        "tab.board": "控制板",

        # Info panel
        "info.title": "机器人信息",
        "info.lang_label": "语言 Language",
        "info.refresh_btn": "刷新",
        "info.banner": "==================  MicroRosV2 机器人信息  ==================",
        "info.f_model": "机器人型号",
        "info.f_domain": "ROS_DOMAIN_ID",
        "info.f_cartype": "CAR_TYPE",
        "info.f_ip": "本机 IP",
        "info.f_camera": "摄像头 IP",
        "info.f_apikey": "Vision API Key",
        "info.f_baseurl": "Vision Base URL",
        "info.f_vmodel": "视觉模型",
        "info.f_sysver": "系统镜像版本",
        "info.f_dify_apikey": "Dify API Key",
        "info.f_dify_baseurl": "Dify Base URL",
        "info.not_set": "未设置",
        "info.status_title": "系统状态",
        "info.st_domain": "环境变量 ROS_DOMAIN_ID",
        "info.st_cartype": "环境变量 CAR_TYPE",
        "info.st_json": "robot_info.json",
        "info.st_board": "控制板串口 ({port})",
        "info.ok": "OK",
        "info.missing": "缺失",
        "info.board_present": "已检测到",
        "info.board_absent": "未检测到",

        # Host panel
        "host.title": "主机端参数设置",
        "host.group_env": "环境变量",
        "host.group_vision": "视觉配置",
        "host.group_dify": "Dify 配置",
        "host.ph_cartype": "例如 ptz",
        "host.save_btn": "保存",
        "host.reload_btn": "重新加载",
        "host.loaded": "已加载当前配置。",
        "host.save_ok": "保存成功。环境变量需重新打开终端后生效。",
        "host.save_fail": "保存失败: {err}",

        # Board panel
        "board.title": "控制板参数配置",
        "board.port_label": "串口 Port",
        "board.group_wifi": "WiFi",
        "board.group_agent": "Agent",
        "board.group_ros": "ROS",
        "board.group_servo": "舵机偏移",
        "board.group_motor_pid": "电机 PID (P, I, D)",
        "board.group_imu_pid": "IMU Yaw PID (P, I, D)",
        "board.group_odom": "里程计校准",
        "board.linear_scale": "线速度系数",
        "board.angular_scale": "角速度系数",
        "board.ph_namespace": "可为空",
        "board.read_btn": "读取板参数",
        "board.apply_btn": "应用配置",
        "board.verify_btn": "验证",
        "board.reset_btn": "恢复默认配置",
        "board.reset_ok_saved": "已恢复默认配置并保存到 robot_info.json，点击“应用配置”写入控制板。",
        "board.reset_ok_unsaved": "已恢复默认配置，但保存失败: {err}",
        "board.log_title": "日志",
        "board.loaded_last": "已加载上次/默认控制板配置。",
        "board.invalid": "参数校验失败: {err}",
        "board.connecting_apply": "正在连接控制板并写入参数...",
        "board.connecting_read": "正在连接控制板并读取参数...",
        "board.connecting_verify": "正在读取控制板参数以进行比对...",
        "board.log_connect": "连接: {port}",
        "board.log_entered": "已进入配置窗口",
        "board.log_entered_read": "已进入配置窗口，读取中...",
        "board.log_done": "配置写入完成",
        "board.log_saved": "已保存本次配置到 robot_info.json",
        "board.log_save_fail": "配置保存失败: {err}",
        "board.applied": "配置已应用并保存。",
        "board.apply_fail": "应用失败: {err}",
        "board.log_error": "错误: {err}",
        "board.read_done": "读取完成。",
        "board.read_fail": "读取失败: {err}",
        "board.diff": "差异: {msg}",
        "board.verify_diff": "验证完成，存在 {n} 处差异。",
        "board.verify_ok": "验证通过，参数一致。",
        "board.cmp": "{name} 期望={exp} 实际={act}",

        # backend: validation messages
        "val.ip_parts": "IP 需为 4 段，例如 192.168.12.28",
        "val.ip_digit": "IP 各段需为数字",
        "val.ip_range": "IP 各段需在 0-255 之间",
        "val.port_int": "端口需为整数",
        "val.port_range": "端口需在 1-65535 之间",
        "val.domain_int": "DOMAIN_ID 需为整数",
        "val.domain_range": "DOMAIN_ID 需在 0-232 之间",
        "val.domain_board": "控制板 DOMAIN_ID 需在 0-100 之间",
        "val.servo_int": "舵机偏移需为整数",
        "val.servo_range": "舵机偏移需在 -128 到 127 之间",
        "val.pid_num": "PID 需为数字",
        "val.pid_range": "PID 需在 -327.68 到 327.67 之间",
        "val.vscale_num": "速度系数需为数字",
        "val.vscale_range": "速度系数需在 {min} 到 {max} 之间",
        "val.nonempty": "{label}不能为空",
        "val.default_label": "该项",

        # backend: serial / apply messages
        "be.err_no_port": "串口设备不存在: {port}",
        "be.err_open": "打开串口失败 ({port}): {err}",
        "be.err_window": "固件配置窗口无响应，请检查串口连接后重试",
        "be.err_not_connected": "未连接控制板",
        "be.log_wifi": "写入 WiFi 配置...",
        "be.log_cam_skip": "当前 CAR_TYPE 为 basic_car（无相机），跳过相机配置写入与 IP 查询",
        "be.log_cam_wifi": "写入相机 WiFi 配置...",
        "be.log_cam_auto": "启用相机 WiFi 自动配置开关...",
        "be.err_cam_auto": "相机自动配置开关校验失败: 期望=1 实际={actual}",
        "be.log_cam_apply": "立即下发相机 WiFi 凭据...",
        "be.log_cam_ip": "查询相机 IP...",
        "be.log_cam_ip_ok": "相机 IP: {ip}，已写入 robot_info.json",
        "be.warn_cam_ip": "警告: 相机 IP 查询失败（已重试 5 次）",
        "be.log_agent": "写入 Agent 配置...",
        "be.log_domain": "写入 ROS DOMAIN_ID...",
        "be.log_ns": "写入 ROS namespace...",
        "be.log_servo": "写入舵机偏移...",
        "be.log_motor": "写入电机 PID...",
        "be.log_imu": "写入 IMU Yaw PID...",
        "be.log_vscale": "写入里程计速度系数...",
        "be.log_finish": "结束配置 (CONFIG_DONE)...",
    },

    # ------------------------------------------------------------------ en
    "en": {
        # App
        "app.subtitle": "MicroRosV2 Robot Manager",
        "tab.info": "Info",
        "tab.host": "Host Settings",
        "tab.board": "Board Config",

        # Info panel
        "info.title": "Robot Info",
        "info.lang_label": "Language",
        "info.refresh_btn": "Refresh",
        "info.banner": "==================  MicroRosV2 Robot Info  ==================",
        "info.f_model": "Robot Model",
        "info.f_domain": "ROS_DOMAIN_ID",
        "info.f_cartype": "CAR_TYPE",
        "info.f_ip": "Current IP",
        "info.f_camera": "Camera IP",
        "info.f_apikey": "Vision API Key",
        "info.f_baseurl": "Vision Base URL",
        "info.f_vmodel": "Vision Model",
        "info.f_sysver": "System Image Version",
        "info.f_dify_apikey": "Dify API Key",
        "info.f_dify_baseurl": "Dify Base URL",
        "info.not_set": "not set",
        "info.status_title": "System Status",
        "info.st_domain": "Env ROS_DOMAIN_ID",
        "info.st_cartype": "Env CAR_TYPE",
        "info.st_json": "robot_info.json",
        "info.st_board": "Board serial ({port})",
        "info.ok": "OK",
        "info.missing": "Missing",
        "info.board_present": "Detected",
        "info.board_absent": "Not detected",

        # Host panel
        "host.title": "Host Settings",
        "host.group_env": "Environment variables",
        "host.group_vision": "Vision config",
        "host.group_dify": "Dify config",
        "host.ph_cartype": "e.g. ptz",
        "host.save_btn": "Save",
        "host.reload_btn": "Reload",
        "host.loaded": "Current settings loaded.",
        "host.save_ok": "Saved. Env vars take effect in newly opened terminals.",
        "host.save_fail": "Save failed: {err}",

        # Board panel
        "board.title": "Board Configuration",
        "board.port_label": "Serial Port",
        "board.group_wifi": "WiFi",
        "board.group_agent": "Agent",
        "board.group_ros": "ROS",
        "board.group_servo": "Servo Offset",
        "board.group_motor_pid": "Motor PID (P, I, D)",
        "board.group_imu_pid": "IMU Yaw PID (P, I, D)",
        "board.group_odom": "Odometry calibration",
        "board.linear_scale": "linear_scale",
        "board.angular_scale": "angular_scale",
        "board.ph_namespace": "optional",
        "board.read_btn": "Read Board",
        "board.apply_btn": "Apply",
        "board.verify_btn": "Verify",
        "board.reset_btn": "Reset Defaults",
        "board.reset_ok_saved": "Defaults restored and saved to robot_info.json; press Apply to write to the board.",
        "board.reset_ok_unsaved": "Defaults restored, but save failed: {err}",
        "board.log_title": "Log",
        "board.loaded_last": "Loaded last-used/default board config.",
        "board.invalid": "Validation failed: {err}",
        "board.connecting_apply": "Connecting to board and writing parameters...",
        "board.connecting_read": "Connecting to board and reading parameters...",
        "board.connecting_verify": "Reading board parameters for comparison...",
        "board.log_connect": "Connecting: {port}",
        "board.log_entered": "Entered config window",
        "board.log_entered_read": "Entered config window, reading...",
        "board.log_done": "Configuration written",
        "board.log_saved": "Saved this config to robot_info.json",
        "board.log_save_fail": "Failed to save config: {err}",
        "board.applied": "Configuration applied and saved.",
        "board.apply_fail": "Apply failed: {err}",
        "board.log_error": "Error: {err}",
        "board.read_done": "Read complete.",
        "board.read_fail": "Read failed: {err}",
        "board.diff": "Mismatch: {msg}",
        "board.verify_diff": "Verification finished with {n} mismatch(es).",
        "board.verify_ok": "Verification passed, parameters match.",
        "board.cmp": "{name} expected={exp} actual={act}",

        # backend: validation messages
        "val.ip_parts": "IP must have 4 parts, e.g. 192.168.12.28",
        "val.ip_digit": "Each IP part must be numeric",
        "val.ip_range": "Each IP part must be within 0-255",
        "val.port_int": "Port must be an integer",
        "val.port_range": "Port must be within 1-65535",
        "val.domain_int": "DOMAIN_ID must be an integer",
        "val.domain_range": "DOMAIN_ID must be within 0-232",
        "val.domain_board": "Board DOMAIN_ID must be within 0-100",
        "val.servo_int": "Servo offset must be an integer",
        "val.servo_range": "Servo offset must be within -128 to 127",
        "val.pid_num": "PID must be a number",
        "val.pid_range": "PID must be within -327.68 to 327.67",
        "val.vscale_num": "Velocity scale must be a number",
        "val.vscale_range": "Velocity scale must be within {min} to {max}",
        "val.nonempty": "{label} must not be empty",
        "val.default_label": "This field",

        # backend: serial / apply messages
        "be.err_no_port": "Serial device not found: {port}",
        "be.err_open": "Failed to open serial port ({port}): {err}",
        "be.err_window": "Firmware config window not responding; "
                         "check the serial connection and retry",
        "be.err_not_connected": "Board not connected",
        "be.log_wifi": "Writing WiFi config...",
        "be.log_cam_skip": "CAR_TYPE is basic_car (no camera), skipping "
                           "camera config write and IP query",
        "be.log_cam_wifi": "Writing camera WiFi config...",
        "be.log_cam_auto": "Enabling camera WiFi auto-config switch...",
        "be.err_cam_auto": "Camera auto-config verification failed: "
                           "expected=1 actual={actual}",
        "be.log_cam_apply": "Applying WiFi credentials to camera...",
        "be.log_cam_ip": "Querying camera IP...",
        "be.log_cam_ip_ok": "Camera IP: {ip}, saved to robot_info.json",
        "be.warn_cam_ip": "Warning: camera IP query failed after 5 retries",
        "be.log_agent": "Writing Agent config...",
        "be.log_domain": "Writing ROS DOMAIN_ID...",
        "be.log_ns": "Writing ROS namespace...",
        "be.log_servo": "Writing servo offsets...",
        "be.log_motor": "Writing motor PID...",
        "be.log_imu": "Writing IMU Yaw PID...",
        "be.log_vscale": "Writing odometry velocity scales...",
        "be.log_finish": "Finishing config (CONFIG_DONE)...",
    },
}


# ---------------------------------------------------------------------------
# Language state (persisted in robot_info.json)
# ---------------------------------------------------------------------------
def _read_info_file():
    try:
        with open(_ROBOT_INFO_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _load_language():
    code = _read_info_file().get(_LANGUAGE_KEY)
    return code if code in TRANSLATIONS else DEFAULT_LANGUAGE


_current_language = _load_language()


def get_language():
    """Return the current language code (e.g. 'zh')."""
    return _current_language


def set_language(code):
    """Switch language and persist the choice into robot_info.json."""
    global _current_language
    if code not in TRANSLATIONS:
        raise ValueError("Unsupported language: %s" % code)
    _current_language = code
    data = _read_info_file()
    data[_LANGUAGE_KEY] = code
    os.makedirs(os.path.dirname(_ROBOT_INFO_PATH), exist_ok=True)
    with open(_ROBOT_INFO_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


def tr(key, **kwargs):
    """Translate `key` in the current language, formatting with kwargs.

    Falls back to the default language, then to the key itself, so a
    missing entry never crashes the UI.
    """
    table = TRANSLATIONS.get(_current_language, {})
    text = table.get(key)
    if text is None:
        text = TRANSLATIONS[DEFAULT_LANGUAGE].get(key, key)
    if kwargs:
        try:
            text = text.format(**kwargs)
        except (KeyError, IndexError):
            pass
    return text
