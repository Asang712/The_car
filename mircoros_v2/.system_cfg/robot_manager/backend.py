import os
import re
import json
import socket
from i18n import tr

# ---------------------------------------------------------------------------
# Common paths
# ---------------------------------------------------------------------------
_HOME = os.path.expanduser("~")
_WS_ROOT = os.path.join(_HOME, "microros_ws")
_BASHRC_PATH = os.path.join(_HOME, ".bashrc")
_ROBOT_INFO_PATH = os.path.join(_WS_ROOT, ".system_cfg", "robot_info.json")

ROBOT_MODEL = "MicroRosV2"
DEFAULT_BOARD_PORT = "/dev/microros_board"

# Allowed CAR_TYPE values shown in the host panel dropdown.
CAR_TYPES = ("ptz", "no_ptz", "basic_car")

# Key under which the control-board parameters live inside robot_info.json.
_BOARD_CONFIG_KEY = "board_config"

DEFAULT_BOARD_CONFIG = {
    "port": DEFAULT_BOARD_PORT,
    "wifi_ssid": "example_wifi",
    "wifi_passwd": "12345678",
    "agent_ip_addr": [192, 168, 12, 28],
    "agent_ip_port": 8090,
    "ros_domain_id": 40,
    "ros_namespace": "",
    "servo_1_offset": -5,
    "servo_2_offset": 0,
    "motor_pid": [1.8, 0.12, 0.0],
    "imu_yaw_pid": [2.4, 0.0, 0.16],
    "linear_velocity_scale": 1.0,
    "angular_velocity_scale": 1.0,
}


# ---------------------------------------------------------------------------
# Host env vars (~/.bashrc)
# ---------------------------------------------------------------------------
def read_env_from_bashrc(name, default=None):
    if os.path.isfile(_BASHRC_PATH):
        pattern = re.compile(r"^\s*export\s+%s=(.*)$" % re.escape(name))
        result = None
        with open(_BASHRC_PATH, "r", encoding="utf-8") as f:
            for line in f:
                m = pattern.match(line)
                if m:
                    # Keep the last definition and strip quotes/inline comments.
                    result = _strip_bash_value(m.group(1))
        if result is not None:
            return result

    env_val = os.getenv(name)
    if env_val is not None:
        return env_val
    return default


def _strip_bash_value(raw):
    """Normalize a raw bash rvalue: drop trailing comment, surrounding quotes."""
    value = raw.strip()
    # Remove trailing inline comment only when it is not inside quotes.
    if value and value[0] not in ("'", '"'):
        hash_idx = value.find("#")
        if hash_idx != -1:
            value = value[:hash_idx].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        value = value[1:-1]
    return value


def set_env_in_bashrc(name, value):
    """Set/replace `export NAME=value` in ~/.bashrc.

    Replaces every existing definition in place; appends one if none exist.
    Note: only affects newly opened shells, not the current process.
    """
    line_text = "export %s=%s" % (name, value)

    if not os.path.isfile(_BASHRC_PATH):
        with open(_BASHRC_PATH, "w", encoding="utf-8") as f:
            f.write(line_text + "\n")
        return

    with open(_BASHRC_PATH, "r", encoding="utf-8") as f:
        lines = f.readlines()

    pattern = re.compile(r"^\s*export\s+%s=" % re.escape(name))
    replaced = False
    new_lines = []
    for line in lines:
        if pattern.match(line):
            if not replaced:
                # Preserve the original line ending style.
                ending = "\n" if line.endswith("\n") else ""
                new_lines.append(line_text + ending if ending else line_text)
                replaced = True
            # Drop any additional duplicate definitions.
            continue
        new_lines.append(line)

    if not replaced:
        if new_lines and not new_lines[-1].endswith("\n"):
            new_lines[-1] = new_lines[-1] + "\n"
        new_lines.append(line_text + "\n")

    with open(_BASHRC_PATH, "w", encoding="utf-8") as f:
        f.writelines(new_lines)


# ---------------------------------------------------------------------------
# robot_info.json
# ---------------------------------------------------------------------------
def get_robot_info(key=None):
    """Read robot_info.json. Returns full dict when key is None."""
    try:
        with open(_ROBOT_INFO_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        data = {}
    if key is None:
        return data
    return data.get(key)


def set_robot_info(key, value):
    """Update a single key in robot_info.json, preserving other keys."""
    try:
        with open(_ROBOT_INFO_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        data = {}
    data[key] = value
    os.makedirs(os.path.dirname(_ROBOT_INFO_PATH), exist_ok=True)
    with open(_ROBOT_INFO_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


def robot_info_path_exists():
    return os.path.isfile(_ROBOT_INFO_PATH)


# ---------------------------------------------------------------------------
# Local IP
# ---------------------------------------------------------------------------
def get_local_ip():
    """Best-effort local IP detection (same approach as robot_tool.GetLocalIP)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "N/A"


def board_port_present(port=DEFAULT_BOARD_PORT):
    """Passive check whether the serial device node exists (no connection)."""
    return os.path.exists(port)


# ---------------------------------------------------------------------------
# Validation helpers  ->  (ok: bool, message_or_parsed)
# ---------------------------------------------------------------------------
def valid_ip(text):
    text = (text or "").strip()
    parts = text.split(".")
    if len(parts) != 4:
        return False, tr("val.ip_parts")
    octets = []
    for p in parts:
        if not p.isdigit():
            return False, tr("val.ip_digit")
        v = int(p)
        if v < 0 or v > 255:
            return False, tr("val.ip_range")
        octets.append(v)
    return True, octets


def valid_port(text):
    text = (text or "").strip()
    if not text.lstrip("-").isdigit():
        return False, tr("val.port_int")
    v = int(text)
    if v < 1 or v > 65535:
        return False, tr("val.port_range")
    return True, v


def valid_domain_id(text, for_board=False):
    text = (text or "").strip()
    if not text.lstrip("-").isdigit():
        return False, tr("val.domain_int")
    v = int(text)
    if v < 0 or v > 232:
        return False, tr("val.domain_range")
    if for_board and v > 100:
        return False, tr("val.domain_board")
    return True, v


def valid_servo(text):
    text = (text or "").strip()
    if not text.lstrip("-").isdigit():
        return False, tr("val.servo_int")
    v = int(text)
    if v < -128 or v > 127:
        return False, tr("val.servo_range")
    return True, v


def valid_pid_float(text):
    text = (text or "").strip()
    try:
        v = float(text)
    except ValueError:
        return False, tr("val.pid_num")
    # Packed as int(v * 100) into a signed short.
    if v < -327.68 or v > 327.67:
        return False, tr("val.pid_range")
    return True, v


def valid_velocity_scale(text):
    text = (text or "").strip()
    try:
        v = float(text)
    except ValueError:
        return False, tr("val.vscale_num")
    from config_robot import VELOCITY_SCALE_MAX, VELOCITY_SCALE_MIN
    if v < VELOCITY_SCALE_MIN or v > VELOCITY_SCALE_MAX:
        return False, tr("val.vscale_range",
                         min=VELOCITY_SCALE_MIN, max=VELOCITY_SCALE_MAX)
    return True, v


def valid_nonempty(text, label=None):
    if text is None or str(text).strip() == "":
        return False, tr("val.nonempty",
                         label=label or tr("val.default_label"))
    return True, str(text)

def get_board_config():
    """Return the board config from robot_info.json merged over defaults.

    Missing keys fall back to DEFAULT_BOARD_CONFIG so the panel always has a
    complete, valid set of values to pre-fill the inputs with.
    """
    config = dict(DEFAULT_BOARD_CONFIG)
    saved = get_robot_info(_BOARD_CONFIG_KEY)
    if isinstance(saved, dict):
        for key in DEFAULT_BOARD_CONFIG:
            if key in saved and saved[key] is not None:
                config[key] = saved[key]
    return config


def save_board_config(params):
    """Persist the last-applied board parameters into robot_info.json.

    Only the known board keys are stored, dropping any transient entries.
    """
    to_save = {}
    for key in DEFAULT_BOARD_CONFIG:
        to_save[key] = params.get(key, DEFAULT_BOARD_CONFIG[key])
    set_robot_info(_BOARD_CONFIG_KEY, to_save)
    return _ROBOT_INFO_PATH

class BoardConfigurator:
    """Thin wrapper around config_robot.MicroROS_Robot.

    Connects only on demand (apply/read). All methods raise readable
    RuntimeError on failure so the UI can show messages without crashing.
    """

    def __init__(self, port=DEFAULT_BOARD_PORT):
        self.port = port
        self._robot = None

    def _new_robot(self):

        import sys
        _here = os.path.dirname(os.path.abspath(__file__))
        if _here not in sys.path:
            sys.path.insert(0, _here)
        from config_robot import MicroROS_Robot
        return MicroROS_Robot(port=self.port, debug=False)

    def connect_and_enter(self, ready_timeout=10.0):
        """Open the port and enter the firmware config window."""
        if not os.path.exists(self.port):
            raise RuntimeError(tr("be.err_no_port", port=self.port))
        try:
            self._robot = self._new_robot()
        except Exception as exc:
            raise RuntimeError(
                tr("be.err_open", port=self.port, err=exc)) from exc

        import time
        time.sleep(0.5)
        try:
            self._robot._MicroROS_Robot__ser.flushInput()
            self._robot._MicroROS_Robot__ser.flushOutput()
        except Exception:
            pass

        if not self._robot.auto_enter_config_window(ready_timeout=ready_timeout,
                                                     interval=0.2):
            self.close()
            raise RuntimeError(tr("be.err_window"))

    def apply_all(self, params, logger=None, skip_camera=False):
        """Write all parameters, then close the config window (0x22).

        `params` must be the validated dict from BoardPanel.collect_params().
        `logger` is an optional callable(str) for progress output.
        `skip_camera` skips every camera step (no camera on basic_car).
        """
        def log(msg):
            if logger:
                logger(msg)

        if self._robot is None:
            raise RuntimeError(tr("be.err_not_connected"))

        import time
        r = self._robot
        log(tr("be.log_wifi"))
        r.set_wifi_config(params["wifi_ssid"], params["wifi_passwd"])
        if skip_camera:
            log(tr("be.log_cam_skip"))
        else:
            log(tr("be.log_cam_wifi"))
            r.set_camera_wifi_config(params["wifi_ssid"], params["wifi_passwd"])
            log(tr("be.log_cam_auto"))
            r.set_camera_auto_config(True)
            time.sleep(0.30)
            actual_auto = r.read_camera_auto_config()
            if actual_auto != 1:
                raise RuntimeError(tr("be.err_cam_auto", actual=actual_auto))
            log(tr("be.log_cam_apply"))
            r.apply_camera_wifi_config_now()
            log(tr("be.log_cam_ip"))
            # Wait at least 1s after applying camera Wi-Fi, then query IP.
            time.sleep(2)
            camera_ip = None
            for i in range(5):
                camera_ip = r.query_camera_ip_now(wait_timeout=4.0)
                if camera_ip:
                    break
                if i < 4:
                    time.sleep(2)
            if camera_ip:
                set_robot_info("camera_ip", camera_ip)
                log(tr("be.log_cam_ip_ok", ip=camera_ip))
            else:
                log(tr("be.warn_cam_ip"))
        log(tr("be.log_agent"))
        r.set_udp_config(params["agent_ip_addr"], params["agent_ip_port"])
        log(tr("be.log_domain"))
        r.set_ros_domain_id(params["ros_domain_id"])
        log(tr("be.log_ns"))
        r.set_ros_namespace(params["ros_namespace"])
        log(tr("be.log_servo"))
        r.set_pwm_servo_offset(1, params["servo_1_offset"])
        r.set_pwm_servo_offset(2, params["servo_2_offset"])
        log(tr("be.log_motor"))
        mp = params["motor_pid"]
        r.set_motor_pid_parm(mp[0], mp[1], mp[2])
        log(tr("be.log_imu"))
        ip = params["imu_yaw_pid"]
        r.set_imu_yaw_pid_parm(ip[0], ip[1], ip[2])
        log(tr("be.log_vscale"))
        r.set_velocity_scale(params["linear_velocity_scale"],
                             params["angular_velocity_scale"])
        log(tr("be.log_finish"))
        r._MicroROS_Robot__send_payload(0x22)

    def read_all(self):
        """Read back all board parameters as a display-friendly dict."""
        if self._robot is None:
            raise RuntimeError(tr("be.err_not_connected"))
        r = self._robot
        servo = r.read_pwm_servo_offset()
        s1, s2 = "", ""
        if servo:
            parts = servo.split(", ")
            s1 = parts[0] if len(parts) > 0 else ""
            s2 = parts[1] if len(parts) > 1 else ""
        vscale = r.read_velocity_scale()
        linear_scale, angular_scale = ("", "")
        if vscale is not None:
            linear_scale = "%.4g" % vscale[0]
            angular_scale = "%.4g" % vscale[1]
        return {
            "firmware_version": r.read_version() or "",
            "wifi_ssid": r.read_wifi_ssid() or "",
            "wifi_passwd": r.read_wifi_passwd() or "",
            "agent_ip_addr": r.read_agent_ip_addr() or "",
            "agent_ip_port": r.read_agent_ip_port() or "",
            "ros_domain_id": r.read_ros_domain_id() or "",
            "ros_namespace": r.read_ros_namespace() or "",
            "servo_1_offset": s1,
            "servo_2_offset": s2,
            "motor_pid": r.read_motor_pid_parm() or "",
            "imu_yaw_pid": r.read_imu_yaw_pid_parm() or "",
            "linear_velocity_scale": linear_scale,
            "angular_velocity_scale": angular_scale,
        }

    def close(self):
        """Close the serial port if open."""
        if self._robot is not None:
            try:
                self._robot._MicroROS_Robot__ser.close()
            except Exception:
                pass
            self._robot = None
