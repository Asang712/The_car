"""UI panels for robot_manager: InfoPanel, HostPanel, BoardPanel."""

from textual import work
from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import Button, Input, Label, RichLog, Select, Static
from rich.cells import cell_len

import backend
import i18n
from i18n import tr


def _pad(text, width):
    """Pad `text` to `width` terminal cells (CJK glyphs count as two).
    """
    return text + " " * max(width - cell_len(text), 0)


def _aligned_lines(rows, indent="", bold_name=True):
    """Render (name, value) rows with the value column left-aligned."""
    names = [name + ":" for name, _ in rows]
    width = max((cell_len(name) for name in names), default=0) + 1
    name_fmt = "[b]%s[/]" if bold_name else "%s"
    return ["%s%s%s" % (indent, name_fmt % _pad(name, width), value)
            for name, (_, value) in zip(names, rows)]


def _row(label_text, input_id, value="", password=False, placeholder=""):
    """Build a labeled input row (Label + Input)."""
    with Horizontal(classes="field-row"):
        yield Label(label_text, classes="field-label")
        yield Input(
            value=str(value) if value is not None else "",
            id=input_id,
            password=password,
            placeholder=placeholder,
            classes="field-input",
        )


class InfoPanel(VerticalScroll):
    """Displays robot info and live system/connection status."""

    def compose(self) -> ComposeResult:
        yield Static(tr("info.title"), classes="section-title")
        with Horizontal(classes="field-row"):
            yield Label(tr("info.lang_label"), classes="field-label")
            yield Select(
                [(name, code) for code, name in i18n.LANGUAGES.items()],
                value=i18n.get_language(),
                allow_blank=False,
                id="info-lang",
            )
        yield Static("", id="info-body", classes="info-body")
        yield Static("", id="info-status", classes="info-status")
        with Horizontal(classes="button-row"):
            yield Button(tr("info.refresh_btn"), id="info-refresh",
                         variant="primary")

    def on_mount(self) -> None:
        self.refresh_info()
        # Realtime refresh of local IP / status every 3 seconds.
        self.set_interval(3.0, self.refresh_info)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "info-refresh":
            self.refresh_info()

    def on_select_changed(self, event: Select.Changed) -> None:
        if event.select.id != "info-lang":
            return
        code = event.value
        if code in i18n.LANGUAGES and code != i18n.get_language():
            i18n.set_language(code)
            # Rebuild the whole UI so every tab/panel picks up the new
            # language (inputs are re-filled from the persisted config).
            self.app.refresh(recompose=True)

    def refresh_info(self) -> None:
        info = backend.get_robot_info()
        not_set = tr("info.not_set")
        domain_id = backend.read_env_from_bashrc("ROS_DOMAIN_ID",
                                                 default=not_set)
        car_type = backend.read_env_from_bashrc("CAR_TYPE", default=not_set)
        local_ip = backend.get_local_ip()

        def show(v):
            return v if v not in (None, "") else "N/A"

        fields = [
            (tr("info.f_model"), "[green]%s[/]" % backend.ROBOT_MODEL),
            (tr("info.f_domain"), show(domain_id)),
            (tr("info.f_cartype"), show(car_type)),
            (tr("info.f_ip"), show(local_ip)),
            (tr("info.f_camera"), show(info.get("camera_ip"))),
            (tr("info.f_apikey"), show(info.get("Vision_API_KEY"))),
            (tr("info.f_baseurl"), show(info.get("Vision_BASE_URL"))),
            (tr("info.f_vmodel"), show(info.get("Vision_MODEL"))),
            (tr("info.f_dify_apikey"), show(info.get("DIFY_API_KEY"))),
            (tr("info.f_dify_baseurl"), show(info.get("DIFY_BASE_URL"))),
            (tr("info.f_sysver"), show(info.get("System_Image_Version"))),
        ]
        self.query_one("#info-body", Static).update(
            "\n".join(_aligned_lines(fields)))

        # Passive connection / status checks (never opens the serial port).
        domain_ok = domain_id not in (None, "", not_set)
        car_ok = car_type not in (None, "", not_set)
        json_ok = backend.robot_info_path_exists()
        board_present = backend.board_port_present(backend.DEFAULT_BOARD_PORT)

        def mark(ok):
            return ("[green]%s[/]" % tr("info.ok") if ok
                    else "[red]%s[/]" % tr("info.missing"))

        board_txt = ("[green]%s[/]" % tr("info.board_present")
                     if board_present
                     else "[yellow]%s[/]" % tr("info.board_absent"))
        status_lines = [
            (tr("info.st_domain"), mark(domain_ok)),
            (tr("info.st_cartype"), mark(car_ok)),
            (tr("info.st_json"), mark(json_ok)),
            (tr("info.st_board", port=backend.DEFAULT_BOARD_PORT), board_txt),
        ]
        self.query_one("#info-status", Static).update(
            "\n".join(_aligned_lines(status_lines, indent="  ",
                                     bold_name=False)))


class HostPanel(VerticalScroll):
    """Host-side parameter settings (~/.bashrc env + robot_info.json)."""

    def compose(self) -> ComposeResult:
        yield Static(tr("host.title"), classes="section-title")

        yield Static(tr("host.group_env"), classes="group-title")
        yield from _row("ROS_DOMAIN_ID", "host-domain",
                        placeholder="0-232")
        with Horizontal(classes="field-row"):
            yield Label("CAR_TYPE", classes="field-label")
            yield Select(
                [(t, t) for t in backend.CAR_TYPES],
                value=backend.CAR_TYPES[0],
                allow_blank=False,
                id="host-cartype",
                classes="field-input",
            )

        yield Static(tr("host.group_vision"), classes="group-title")
        yield from _row("camera_ip", "host-camera-ip",
                        placeholder="192.168.12.32")
        yield from _row("Vision_API_KEY", "host-api-key")
        yield from _row("Vision_BASE_URL", "host-base-url")
        yield from _row("Vision_MODEL", "host-model")

        yield Static(tr("host.group_dify"), classes="group-title")
        yield from _row("DIFY_API_KEY", "host-dify-api-key")
        yield from _row("DIFY_BASE_URL", "host-dify-base-url",
                        placeholder="http://localhost/v1")

        with Horizontal(classes="button-row"):
            yield Button(tr("host.save_btn"), id="host-save", variant="success")
            yield Button(tr("host.reload_btn"), id="host-reload")
        yield Static("", id="host-status", classes="status-line")

    def on_mount(self) -> None:
        self.load_values()

    def load_values(self) -> None:
        info = backend.get_robot_info()
        self.query_one("#host-domain", Input).value = str(
            backend.read_env_from_bashrc("ROS_DOMAIN_ID", default="") or "")
        car_type = str(
            backend.read_env_from_bashrc("CAR_TYPE", default="") or "")
        if car_type in backend.CAR_TYPES:
            self.query_one("#host-cartype", Select).value = car_type
        self.query_one("#host-camera-ip", Input).value = str(
            info.get("camera_ip", "") or "")
        self.query_one("#host-api-key", Input).value = str(
            info.get("Vision_API_KEY", "") or "")
        self.query_one("#host-base-url", Input).value = str(
            info.get("Vision_BASE_URL", "") or "")
        self.query_one("#host-model", Input).value = str(
            info.get("Vision_MODEL", "") or "")
        self.query_one("#host-dify-api-key", Input).value = str(
            info.get("DIFY_API_KEY", "") or "")
        self.query_one("#host-dify-base-url", Input).value = str(
            info.get("DIFY_BASE_URL", "") or "")
        self._set_status(tr("host.loaded"), ok=True)

    def _set_status(self, msg, ok=True):
        status = self.query_one("#host-status", Static)
        status.remove_class("status-ok", "status-error")
        status.add_class("status-ok" if ok else "status-error")
        status.update(msg)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "host-reload":
            self.load_values()
        elif event.button.id == "host-save":
            self.save_values()

    def save_values(self) -> None:
        domain_raw = self.query_one("#host-domain", Input).value
        car_type = self.query_one("#host-cartype", Select).value
        camera_ip = self.query_one("#host-camera-ip", Input).value
        api_key = self.query_one("#host-api-key", Input).value.strip()
        base_url = self.query_one("#host-base-url", Input).value.strip()
        model = self.query_one("#host-model", Input).value.strip()
        dify_api_key = self.query_one("#host-dify-api-key", Input).value.strip()
        dify_base_url = self.query_one("#host-dify-base-url",
                                       Input).value.strip()

        # Validate before writing anything.
        ok, res = backend.valid_domain_id(domain_raw)
        if not ok:
            self._set_status("ROS_DOMAIN_ID: " + res, ok=False)
            return
        domain_id = res

        ok, res = backend.valid_nonempty(car_type, "CAR_TYPE")
        if not ok:
            self._set_status(res, ok=False)
            return

        ok, res = backend.valid_ip(camera_ip)
        if not ok:
            self._set_status("camera_ip: " + res, ok=False)
            return

        for label, val in (("Vision_API_KEY", api_key),
                           ("Vision_BASE_URL", base_url),
                           ("Vision_MODEL", model),
                           ("DIFY_API_KEY", dify_api_key),
                           ("DIFY_BASE_URL", dify_base_url)):
            ok, res = backend.valid_nonempty(val, label)
            if not ok:
                self._set_status(res, ok=False)
                return

        try:
            backend.set_env_in_bashrc("ROS_DOMAIN_ID", domain_id)
            backend.set_env_in_bashrc("CAR_TYPE", car_type)
            backend.set_robot_info("camera_ip", camera_ip.strip())
            backend.set_robot_info("Vision_API_KEY", api_key)
            backend.set_robot_info("Vision_BASE_URL", base_url)
            backend.set_robot_info("Vision_MODEL", model)
            backend.set_robot_info("DIFY_API_KEY", dify_api_key)
            backend.set_robot_info("DIFY_BASE_URL", dify_base_url)
        except Exception as exc:
            self._set_status(tr("host.save_fail", err=exc), ok=False)
            return

        self._set_status(tr("host.save_ok"), ok=True)


# ===========================================================================
# BoardPanel
# ===========================================================================
class BoardPanel(VerticalScroll):
    """Microros control-board serial configuration."""

    # Keys restored to DEFAULT_BOARD_CONFIG by the reset button.
    _RESET_KEYS = (
        "motor_pid", "imu_yaw_pid", "servo_1_offset", "servo_2_offset",
        "angular_velocity_scale", "linear_velocity_scale", "port",
        "agent_ip_port", "ros_domain_id", "ros_namespace",
    )

    def compose(self) -> ComposeResult:
        yield Static(tr("board.title"), classes="section-title")

        yield from _row(tr("board.port_label"), "board-port",
                        value=backend.DEFAULT_BOARD_PORT)

        yield Static(tr("board.group_wifi"), classes="group-title")
        yield from _row("wifi_ssid", "board-wifi-ssid")
        yield from _row("wifi_passwd", "board-wifi-passwd")

        yield Static(tr("board.group_agent"), classes="group-title")
        yield from _row("agent_ip_addr", "board-agent-ip",
                        placeholder="192.168.12.28")
        yield from _row("agent_ip_port", "board-agent-port",
                        placeholder="8090")

        yield Static(tr("board.group_ros"), classes="group-title")
        yield from _row("ros_domain_id", "board-domain", placeholder="0-100")
        yield from _row("ros_namespace", "board-namespace",
                        placeholder=tr("board.ph_namespace"))

        yield Static(tr("board.group_servo"), classes="group-title")
        yield from _row("servo_1_offset", "board-servo1", placeholder="-128~127")
        yield from _row("servo_2_offset", "board-servo2", placeholder="-128~127")

        yield Static(tr("board.group_motor_pid"), classes="group-title")
        with Horizontal(classes="field-row"):
            yield Label("motor_pid", classes="field-label")
            yield Input(id="board-motor-p", placeholder="P", classes="pid-input")
            yield Input(id="board-motor-i", placeholder="I", classes="pid-input")
            yield Input(id="board-motor-d", placeholder="D", classes="pid-input")

        yield Static(tr("board.group_imu_pid"), classes="group-title")
        with Horizontal(classes="field-row"):
            yield Label("imu_yaw_pid", classes="field-label")
            yield Input(id="board-imu-p", placeholder="P", classes="pid-input")
            yield Input(id="board-imu-i", placeholder="I", classes="pid-input")
            yield Input(id="board-imu-d", placeholder="D", classes="pid-input")

        yield Static(tr("board.group_odom"), classes="group-title")
        yield from _row(tr("board.linear_scale"), "board-linear-scale",
                        placeholder="1.0")
        yield from _row(tr("board.angular_scale"), "board-angular-scale",
                        placeholder="1.0")

        with Horizontal(classes="button-row"):
            yield Button(tr("board.read_btn"), id="board-read", variant="primary")
            yield Button(tr("board.apply_btn"), id="board-apply", variant="success")
            yield Button(tr("board.verify_btn"), id="board-verify")
            yield Button(tr("board.reset_btn"), id="board-reset",
                         variant="warning")

        yield Static("", id="board-status", classes="status-line")
        yield Static(tr("board.log_title"), classes="group-title")
        yield RichLog(id="board-log", highlight=False, markup=True,
                      classes="board-log")

    def on_mount(self) -> None:
        self._load_from_json()

    def on_show(self) -> None:
        self._load_from_json()

    def _load_from_json(self) -> None:
        config = backend.get_board_config()
        self._apply_profile_to_fields(config)
        self._refresh_agent_ip()
        self._set_status(tr("board.loaded_last"), ok=True)

    def _refresh_agent_ip(self) -> None:
        """Auto-fill agent_ip_addr with the current local IP so the user does
        not have to retype it. agent_ip_port is also filled from the default
        when empty."""
        local_ip = backend.get_local_ip()
        if local_ip and local_ip != "N/A":
            self._set("board-agent-ip", local_ip)
        if not self._get("board-agent-port").strip():
            self._set("board-agent-port",
                      backend.DEFAULT_BOARD_CONFIG["agent_ip_port"])

    def reset_defaults(self) -> None:
        """Restore the resettable board parameters to their default values.

        WiFi credentials are left untouched and agent_ip_addr is refreshed
        to the current local IP. Values are saved to robot_info.json and
        take effect on the board after Apply.
        """
        config = backend.get_board_config()
        for key in self._RESET_KEYS:
            config[key] = backend.DEFAULT_BOARD_CONFIG[key]
        local_ip = backend.get_local_ip()
        if local_ip and local_ip != "N/A":
            config["agent_ip_addr"] = [int(x) for x in local_ip.split(".")]
        self._apply_profile_to_fields(config)
        try:
            backend.save_board_config(config)
            self._set_status(tr("board.reset_ok_saved"), ok=True)
        except Exception as exc:
            self._set_status(tr("board.reset_ok_unsaved", err=exc), ok=False)

    # ---- helpers ----------------------------------------------------------
    def _set_status(self, msg, ok=True):
        status = self.query_one("#board-status", Static)
        status.remove_class("status-ok", "status-error")
        status.add_class("status-ok" if ok else "status-error")
        status.update(msg)

    def _log(self, msg):
        self.query_one("#board-log", RichLog).write(msg)

    def _get(self, wid):
        return self.query_one("#" + wid, Input).value

    def _set(self, wid, value):
        self.query_one("#" + wid, Input).value = "" if value is None else str(value)

    def collect_params(self):
        """Validate all inputs. Returns (True, params) or (False, err_msg)."""
        ok, ssid = backend.valid_nonempty(self._get("board-wifi-ssid"), "wifi_ssid")
        if not ok:
            return False, ssid
        wifi_passwd = self._get("board-wifi-passwd")  # may be empty

        ok, agent_ip = backend.valid_ip(self._get("board-agent-ip"))
        if not ok:
            return False, "agent_ip_addr: " + agent_ip
        ok, agent_port = backend.valid_port(self._get("board-agent-port"))
        if not ok:
            return False, "agent_ip_port: " + agent_port

        ok, domain_id = backend.valid_domain_id(self._get("board-domain"),
                                                for_board=True)
        if not ok:
            return False, "ros_domain_id: " + domain_id
        namespace = self._get("board-namespace")  # may be empty

        ok, servo1 = backend.valid_servo(self._get("board-servo1"))
        if not ok:
            return False, "servo_1_offset: " + servo1
        ok, servo2 = backend.valid_servo(self._get("board-servo2"))
        if not ok:
            return False, "servo_2_offset: " + servo2

        motor = []
        for wid, name in (("board-motor-p", "motor P"),
                          ("board-motor-i", "motor I"),
                          ("board-motor-d", "motor D")):
            ok, v = backend.valid_pid_float(self._get(wid))
            if not ok:
                return False, "%s: %s" % (name, v)
            motor.append(v)

        imu = []
        for wid, name in (("board-imu-p", "imu P"),
                          ("board-imu-i", "imu I"),
                          ("board-imu-d", "imu D")):
            ok, v = backend.valid_pid_float(self._get(wid))
            if not ok:
                return False, "%s: %s" % (name, v)
            imu.append(v)

        ok, linear_scale = backend.valid_velocity_scale(
            self._get("board-linear-scale"))
        if not ok:
            return False, "linear_velocity_scale: " + str(linear_scale)
        ok, angular_scale = backend.valid_velocity_scale(
            self._get("board-angular-scale"))
        if not ok:
            return False, "angular_velocity_scale: " + str(angular_scale)

        params = {
            "port": self._get("board-port").strip() or backend.DEFAULT_BOARD_PORT,
            "wifi_ssid": ssid,
            "wifi_passwd": wifi_passwd,
            "agent_ip_addr": agent_ip,
            "agent_ip_port": agent_port,
            "ros_domain_id": domain_id,
            "ros_namespace": namespace,
            "servo_1_offset": servo1,
            "servo_2_offset": servo2,
            "motor_pid": motor,
            "imu_yaw_pid": imu,
            "linear_velocity_scale": linear_scale,
            "angular_velocity_scale": angular_scale,
        }
        return True, params

    def _apply_profile_to_fields(self, p):
        self._set("board-port", p.get("port", backend.DEFAULT_BOARD_PORT))
        self._set("board-wifi-ssid", p.get("wifi_ssid", ""))
        self._set("board-wifi-passwd", p.get("wifi_passwd", ""))
        agent_ip = p.get("agent_ip_addr", "")
        if isinstance(agent_ip, list):
            agent_ip = ".".join(str(x) for x in agent_ip)
        self._set("board-agent-ip", agent_ip)
        self._set("board-agent-port", p.get("agent_ip_port", ""))
        self._set("board-domain", p.get("ros_domain_id", ""))
        self._set("board-namespace", p.get("ros_namespace", ""))
        self._set("board-servo1", p.get("servo_1_offset", ""))
        self._set("board-servo2", p.get("servo_2_offset", ""))
        motor = p.get("motor_pid", ["", "", ""]) or ["", "", ""]
        self._set("board-motor-p", motor[0] if len(motor) > 0 else "")
        self._set("board-motor-i", motor[1] if len(motor) > 1 else "")
        self._set("board-motor-d", motor[2] if len(motor) > 2 else "")
        imu = p.get("imu_yaw_pid", ["", "", ""]) or ["", "", ""]
        self._set("board-imu-p", imu[0] if len(imu) > 0 else "")
        self._set("board-imu-i", imu[1] if len(imu) > 1 else "")
        self._set("board-imu-d", imu[2] if len(imu) > 2 else "")
        self._set("board-linear-scale", p.get("linear_velocity_scale", ""))
        self._set("board-angular-scale", p.get("angular_velocity_scale", ""))

    # ---- button handling --------------------------------------------------
    def on_button_pressed(self, event: Button.Pressed) -> None:
        bid = event.button.id
        if bid == "board-apply":
            self.apply_action()
        elif bid == "board-read":
            self.read_action()
        elif bid == "board-verify":
            self.verify_action()
        elif bid == "board-reset":
            self.reset_defaults()

    def apply_action(self):
        ok, res = self.collect_params()
        if not ok:
            self._set_status(tr("board.invalid", err=res), ok=False)
            return
        self._set_buttons_disabled(True)
        self._set_status(tr("board.connecting_apply"), ok=True)
        self._apply_worker(res)

    def read_action(self):
        port = self._get("board-port").strip() or backend.DEFAULT_BOARD_PORT
        self._set_buttons_disabled(True)
        self._set_status(tr("board.connecting_read"), ok=True)
        self._read_worker(port, verify=False)

    def verify_action(self):
        ok, res = self.collect_params()
        if not ok:
            self._set_status(tr("board.invalid", err=res), ok=False)
            return
        self._set_buttons_disabled(True)
        self._set_status(tr("board.connecting_verify"), ok=True)
        self._read_worker(res["port"], verify=True, expected=res)

    def _set_buttons_disabled(self, disabled):
        for bid in ("board-read", "board-apply", "board-verify",
                    "board-reset"):
            self.query_one("#" + bid, Button).disabled = disabled

    # ---- workers (run in threads; serial I/O is blocking) -----------------
    @work(thread=True, exclusive=True)
    def _apply_worker(self, params):
        app = self.app
        cfg = backend.BoardConfigurator(port=params["port"])
        try:
            app.call_from_thread(
                self._log,
                "[cyan]%s[/]" % tr("board.log_connect", port=params["port"]))
            cfg.connect_and_enter()
            app.call_from_thread(
                self._log, "[green]%s[/]" % tr("board.log_entered"))
            # basic_car has no camera: skip camera config write/IP query.
            car_type = backend.read_env_from_bashrc("CAR_TYPE", default="")
            cfg.apply_all(params,
                          skip_camera=(car_type == "basic_car"),
                          logger=lambda m: app.call_from_thread(self._log, m))
            app.call_from_thread(
                self._log, "[b green]%s[/]" % tr("board.log_done"))
            # Persist the just-applied parameters as the new defaults.
            try:
                backend.save_board_config(params)
                app.call_from_thread(
                    self._log, "[green]%s[/]" % tr("board.log_saved"))
            except Exception as exc:
                app.call_from_thread(
                    self._log,
                    "[yellow]%s[/]" % tr("board.log_save_fail", err=exc))
            app.call_from_thread(self._set_status, tr("board.applied"), True)
        except Exception as exc:
            app.call_from_thread(
                self._log, "[red]%s[/]" % tr("board.log_error", err=exc))
            app.call_from_thread(
                self._set_status, tr("board.apply_fail", err=exc), False)
        finally:
            cfg.close()
            app.call_from_thread(self._set_buttons_disabled, False)

    @work(thread=True, exclusive=True)
    def _read_worker(self, port, verify=False, expected=None):
        app = self.app
        cfg = backend.BoardConfigurator(port=port)
        try:
            app.call_from_thread(
                self._log, "[cyan]%s[/]" % tr("board.log_connect", port=port))
            cfg.connect_and_enter()
            app.call_from_thread(
                self._log, "[green]%s[/]" % tr("board.log_entered_read"))
            data = cfg.read_all()
            app.call_from_thread(self._fill_from_readback, data)
            for k, v in data.items():
                app.call_from_thread(self._log, "  %s = %s" % (k, v))
            if verify and expected is not None:
                mismatches = self._compare(expected, data)
                if mismatches:
                    for m in mismatches:
                        app.call_from_thread(
                            self._log,
                            "[yellow]%s[/]" % tr("board.diff", msg=m))
                    app.call_from_thread(
                        self._set_status,
                        tr("board.verify_diff", n=len(mismatches)), False)
                else:
                    app.call_from_thread(
                        self._set_status, tr("board.verify_ok"), True)
            else:
                app.call_from_thread(self._set_status,
                                     tr("board.read_done"), True)
        except Exception as exc:
            app.call_from_thread(
                self._log, "[red]%s[/]" % tr("board.log_error", err=exc))
            app.call_from_thread(
                self._set_status, tr("board.read_fail", err=exc), False)
        finally:
            cfg.close()
            app.call_from_thread(self._set_buttons_disabled, False)

    def _fill_from_readback(self, data):
        """Populate input fields from board readback values."""
        self._set("board-wifi-ssid", data.get("wifi_ssid", ""))
        self._set("board-wifi-passwd", data.get("wifi_passwd", ""))
        self._set("board-agent-ip", data.get("agent_ip_addr", ""))
        self._set("board-agent-port", data.get("agent_ip_port", ""))
        self._set("board-domain", data.get("ros_domain_id", ""))
        self._set("board-namespace", data.get("ros_namespace", ""))
        self._set("board-servo1", data.get("servo_1_offset", ""))
        self._set("board-servo2", data.get("servo_2_offset", ""))
        motor = (data.get("motor_pid", "") or "").split(", ")
        self._set("board-motor-p", motor[0] if len(motor) > 0 else "")
        self._set("board-motor-i", motor[1] if len(motor) > 1 else "")
        self._set("board-motor-d", motor[2] if len(motor) > 2 else "")
        imu = (data.get("imu_yaw_pid", "") or "").split(", ")
        self._set("board-imu-p", imu[0] if len(imu) > 0 else "")
        self._set("board-imu-i", imu[1] if len(imu) > 1 else "")
        self._set("board-imu-d", imu[2] if len(imu) > 2 else "")
        self._set("board-linear-scale", data.get("linear_velocity_scale", ""))
        self._set("board-angular-scale",
                  data.get("angular_velocity_scale", ""))

    @staticmethod
    def _compare(expected, actual):
        """Return a list of human-readable mismatch descriptions."""
        mismatches = []

        def cmp(name, exp, act):
            if str(exp) != str(act):
                mismatches.append(tr("board.cmp", name=name, exp=exp, act=act))

        cmp("wifi_ssid", expected["wifi_ssid"], actual.get("wifi_ssid"))
        exp_ip = ".".join(str(x) for x in expected["agent_ip_addr"])
        cmp("agent_ip_addr", exp_ip, actual.get("agent_ip_addr"))
        cmp("agent_ip_port", expected["agent_ip_port"],
            actual.get("agent_ip_port"))
        cmp("ros_domain_id", expected["ros_domain_id"],
            actual.get("ros_domain_id"))
        cmp("ros_namespace", expected["ros_namespace"],
            actual.get("ros_namespace"))
        cmp("servo_1_offset", expected["servo_1_offset"],
            actual.get("servo_1_offset"))
        cmp("servo_2_offset", expected["servo_2_offset"],
            actual.get("servo_2_offset"))
        exp_motor = "%.2f, %.2f, %.2f" % tuple(expected["motor_pid"])
        cmp("motor_pid", exp_motor, actual.get("motor_pid"))
        exp_imu = "%.2f, %.2f, %.2f" % tuple(expected["imu_yaw_pid"])
        cmp("imu_yaw_pid", exp_imu, actual.get("imu_yaw_pid"))
        cmp("linear_velocity_scale", "%.4g" % expected["linear_velocity_scale"],
            actual.get("linear_velocity_scale"))
        cmp("angular_velocity_scale",
            "%.4g" % expected["angular_velocity_scale"],
            actual.get("angular_velocity_scale"))
        return mismatches
