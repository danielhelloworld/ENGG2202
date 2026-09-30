"""Tkinter GUI for safe real-time control of a WHEELTEC F32C gimbal."""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from f32c_protocol import F32CError, F32CTimeout, MODE_MULTI_T
from safe_f32c_driver import (
    DEFAULT_Y_MAX_DEG,
    DEFAULT_Y_MIN_DEG,
    F32CLimitError,
    SafeF32CGimbal,
)


X_UI_MIN_DEG = -360.0
X_UI_MAX_DEG = 360.0
STATUS_PERIOD_S = 0.10
COMMAND_PERIOD_S = 0.02


def list_serial_ports():
    """Return available serial device names without opening them."""
    from serial.tools import list_ports

    return [port.device for port in list_ports.comports()]


def pyserial_install_command():
    """Return an install command for the exact interpreter running the GUI."""
    return '"%s" -m pip install pyserial' % sys.executable


class GimbalWorker(threading.Thread):
    """Own the UART so commands and feedback requests never race."""

    def __init__(
        self, port, speed_rpm, acceleration, events, demo_mode=False,
        x_id=1, y_id=2,
    ):
        super().__init__(daemon=True)
        self.port = port
        self.speed_rpm = speed_rpm
        self.acceleration = acceleration
        self.events = events
        self.demo_mode = demo_mode
        self.x_id = x_id
        self.y_id = y_id
        self.stop_event = threading.Event()
        self.target_lock = threading.Lock()
        self.pending_target = None
        self.control_queue = queue.Queue()
        self.transport = None
        self.gimbal = None
        self.motor_enabled = False

    def set_target(self, x_deg, y_deg):
        if not self.motor_enabled:
            raise F32CError("电机当前失能；位置仍在读取，但不能下发运动命令")
        if not math.isfinite(x_deg) or not math.isfinite(y_deg):
            raise F32CLimitError("X/Y 目标角度必须是有限数字")
        if not X_UI_MIN_DEG <= x_deg <= X_UI_MAX_DEG:
            raise F32CLimitError("X 目标角度必须在 -360°～+360°")
        if not DEFAULT_Y_MIN_DEG <= y_deg <= DEFAULT_Y_MAX_DEG:
            raise F32CLimitError(
                "Y target must remain in %.1f..%.1f deg"
                % (DEFAULT_Y_MIN_DEG, DEFAULT_Y_MAX_DEG)
            )
        with self.target_lock:
            self.pending_target = (float(x_deg), float(y_deg))

    def set_enabled(self, enabled):
        self.control_queue.put(("enabled", bool(enabled)))

    def set_motion_parameters(self, speed_rpm, acceleration):
        if not 1 <= speed_rpm <= 1000:
            raise ValueError("位置速度必须为 1～1000 RPM")
        if not 0 <= acceleration <= 65535:
            raise ValueError("加速度必须为 0～65535")
        self.control_queue.put(("motion", (int(speed_rpm), int(acceleration))))

    def set_speed_pid(self, axis, kp, ki, save=False):
        if axis not in ("X", "Y"):
            raise ValueError("PID 轴必须是 X 或 Y")
        if not 0 <= kp <= 256 or not 0 <= ki <= 256:
            raise ValueError("PID 参数必须为 0～256")
        self.control_queue.put(("pid", (axis, int(kp), int(ki), bool(save))))

    def request_stop(self):
        self.stop_event.set()

    def _open_transport(self):
        if self.demo_mode:
            from demo import DryRunTransport

            return DryRunTransport()
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "当前 Python 未安装 pyserial。\n"
                "Python: %s\n"
                "请运行：%s" % (sys.executable, pyserial_install_command())
            ) from exc
        return serial.Serial(
            port=self.port,
            baudrate=115200,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.01,
            write_timeout=0.2,
        )

    def _pop_target(self):
        with self.target_lock:
            target = self.pending_target
            self.pending_target = None
        return target

    def _clear_target(self):
        with self.target_lock:
            self.pending_target = None

    def _process_control_commands(self):
        disabled = False
        while True:
            try:
                action, payload = self.control_queue.get_nowait()
            except queue.Empty:
                return disabled

            if action == "enabled":
                if payload:
                    if not self.gimbal.referenced:
                        self.gimbal.capture_software_zero()
                        self.gimbal.configure_position_mode(
                            MODE_MULTI_T, self.speed_rpm, self.acceleration
                        )
                        self.events.put(("referenced", self.gimbal.get_software_zero()))
                    self.gimbal.enable_at_current_position()
                else:
                    self.gimbal.disable()
                    self._clear_target()
                    disabled = True
                self.motor_enabled = self.gimbal.enabled
                self.events.put(("motor_enabled", self.motor_enabled))
            elif action == "motion":
                speed_rpm, acceleration = payload
                self.gimbal.set_position_speed(speed_rpm)
                self.gimbal.set_acceleration(acceleration)
                self.speed_rpm, self.acceleration = payload
                self.events.put(("motion", payload))
            elif action == "pid":
                axis, kp, ki, save = payload
                motor_id = self.gimbal.x_id if axis == "X" else self.gimbal.y_id
                self.gimbal.set_speed_pid(motor_id, kp, ki)
                if save:
                    self.gimbal.save_parameters(motor_id)
                self.events.put(("pid", payload))

    def run(self):
        try:
            self.events.put(("state", "正在打开串口并初始化电机…"))
            self.transport = self._open_transport()
            self.gimbal = SafeF32CGimbal(
                self.transport,
                x_id=self.x_id,
                y_id=self.y_id,
                y_min_deg=DEFAULT_Y_MIN_DEG,
                y_max_deg=DEFAULT_Y_MAX_DEG,
                feedback_timeout_s=0.35,
                feedback_retries=3,
            )
            startup_issue = None
            try:
                self.gimbal.start(
                    speed_rpm=self.speed_rpm,
                    mode=MODE_MULTI_T,
                    acceleration=self.acceleration,
                    power_on_delay_s=0.0 if self.demo_mode else 1.5,
                    require_feedback=True,
                )
            except F32CTimeout as exc:
                # Keep the COM port open for diagnosis and automatic read-only
                # retries. A silent bus must never be treated as motor ready.
                startup_issue = str(exc)
            else:
                self.motor_enabled = True
                self.gimbal.return_to_zero()
            zero_deg = (
                self.gimbal.get_software_zero()
                if self.gimbal.referenced else None
            )
            self.events.put(
                ("connected", (self.port or "模拟串口", zero_deg, self.motor_enabled))
            )
            if startup_issue is not None:
                self.events.put(("feedback_issue", (startup_issue, False)))

            next_status_s = 0.0
            next_command_s = 0.0
            last_target = (0.0, 0.0)
            target_dirty = False
            feedback_missing = not self.gimbal.referenced
            while not self.stop_event.is_set():
                try:
                    if self._process_control_commands():
                        target_dirty = False
                except F32CError as exc:
                    self.events.put(("command_error", str(exc)))

                now = time.monotonic()
                target = self._pop_target()
                if target is not None:
                    last_target = target
                    target_dirty = True
                if target_dirty and self.motor_enabled and now >= next_command_s:
                    self.gimbal.set_relative_angles(*last_target)
                    next_command_s = time.monotonic() + COMMAND_PERIOD_S
                    self.events.put(("target", last_target))
                    target_dirty = False

                # Feedback requests remain active even while torque is disabled.
                if now >= next_status_s:
                    try:
                        if not self.gimbal.referenced:
                            zero_deg = self.gimbal.capture_software_zero()
                            self.gimbal.configure_position_mode(
                                MODE_MULTI_T, self.speed_rpm, self.acceleration
                            )
                            self.events.put(("referenced", zero_deg))
                        x_deg, y_deg = self.gimbal.read_relative_angles()
                    except F32CTimeout as exc:
                        attempted_disable = self.motor_enabled
                        if self.motor_enabled:
                            self.gimbal.stop()
                            self.motor_enabled = False
                            self.events.put(("motor_enabled", False))
                        if not feedback_missing:
                            self.events.put(
                                ("feedback_issue", (str(exc), attempted_disable))
                            )
                        feedback_missing = True
                        next_status_s = time.monotonic() + 1.0
                    else:
                        if feedback_missing:
                            self.events.put(("feedback_restored", None))
                        feedback_missing = False
                        self.events.put(("position", (x_deg, y_deg)))
                        next_status_s = time.monotonic() + STATUS_PERIOD_S
                time.sleep(0.003)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.motor_enabled = False
            if self.gimbal is not None:
                try:
                    self.gimbal.stop()
                except Exception:
                    pass
            if self.transport is not None:
                close = getattr(self.transport, "close", None)
                if close:
                    try:
                        close()
                    except Exception:
                        pass
            self.events.put(("disconnected", None))


class GimbalGUI(tk.Tk):
    def __init__(self, demo_mode=False):
        super().__init__()
        self.demo_mode = demo_mode
        self.worker = None
        self.events = queue.Queue()
        self.send_after_id = None
        self.motor_enabled = False
        self.connected_port = ""

        self.title("WHEELTEC F32C 双轴云台控制")
        self.geometry("880x730")
        self.minsize(820, 690)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.port_var = tk.StringVar()
        self.x_id_var = tk.IntVar(value=1)
        self.y_id_var = tk.IntVar(value=2)
        self.speed_var = tk.IntVar(value=20)
        self.acceleration_var = tk.IntVar(value=100)
        self.state_var = tk.StringVar(value="未连接")
        self.current_x_var = tk.StringVar(value="--.-°")
        self.current_y_var = tk.StringVar(value="--.-°")
        self.zero_var = tk.StringVar(value="尚未读取电机零点")
        self.target_x_var = tk.DoubleVar(value=0.0)
        self.target_y_var = tk.DoubleVar(value=0.0)
        self.entry_x_var = tk.StringVar(value="0.0")
        self.entry_y_var = tk.StringVar(value="0.0")
        self.pid_axis_var = tk.StringVar(value="X")
        self.pid_kp_var = tk.IntVar(value=10)
        self.pid_ki_var = tk.IntVar(value=10)
        self.feedback_var = tk.StringVar(value="等待连接")
        self.diagnostic_var = tk.StringVar(value="")

        self._build_ui()
        self.refresh_ports()
        self.after(50, self.process_events)

    def _build_ui(self):
        root = ttk.Frame(self, padding=16)
        root.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(root, text="连接与运动参数", padding=12)
        connection.pack(fill="x")
        ttk.Label(connection, text="串口").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(
            connection, textvariable=self.port_var, width=14, state="normal"
        )
        self.port_box.grid(row=0, column=1, padx=(8, 8))
        ttk.Button(
            connection,
            text="刷新",
            command=lambda: self.refresh_ports(show_error=True),
        ).grid(
            row=0, column=2, padx=(0, 12)
        )
        ttk.Label(connection, text="速度 RPM").grid(row=0, column=3)
        self.speed_spin = ttk.Spinbox(
            connection, from_=1, to=1000, textvariable=self.speed_var, width=7
        )
        self.speed_spin.grid(row=0, column=4, padx=6)
        ttk.Label(connection, text="加速度").grid(row=0, column=5)
        self.acceleration_spin = ttk.Spinbox(
            connection,
            from_=0,
            to=65535,
            textvariable=self.acceleration_var,
            width=7,
        )
        self.acceleration_spin.grid(row=0, column=6, padx=6)
        self.motion_button = ttk.Button(
            connection,
            text="应用运动参数",
            command=self.apply_motion_parameters,
            state="disabled",
        )
        self.motion_button.grid(row=0, column=7, padx=5)

        self.connect_button = ttk.Button(
            connection, text="连接并使能", command=self.connect
        )
        self.connect_button.grid(row=1, column=0, columnspan=2, pady=(10, 0))
        self.enable_button = ttk.Button(
            connection,
            text="电机失能",
            command=self.toggle_motor_enabled,
            state="disabled",
        )
        self.enable_button.grid(row=1, column=2, columnspan=2, pady=(10, 0))
        self.disconnect_button = ttk.Button(
            connection, text="断开串口", command=self.disconnect, state="disabled"
        )
        self.disconnect_button.grid(row=1, column=4, columnspan=2, pady=(10, 0))
        for column, label, variable in (
            (6, "X ID", self.x_id_var), (7, "Y ID", self.y_id_var)
        ):
            id_box = ttk.Frame(connection)
            id_box.grid(row=1, column=column, pady=(10, 0))
            ttk.Label(id_box, text=label).pack(side="left")
            ttk.Spinbox(id_box, from_=1, to=255, textvariable=variable, width=4).pack(
                side="left", padx=3
            )
        ttk.Label(connection, textvariable=self.state_var).grid(
            row=2, column=0, columnspan=8, sticky="w", pady=(10, 0)
        )
        ttk.Label(connection, textvariable=self.zero_var).grid(
            row=3, column=0, columnspan=8, sticky="w", pady=(4, 0)
        )

        position = ttk.LabelFrame(root, text="实时反馈位置（失能时继续读取）", padding=12)
        position.pack(fill="x", pady=10)
        ttk.Label(position, text="X / yaw", font=("Segoe UI", 11)).grid(row=0, column=0)
        ttk.Label(
            position, textvariable=self.current_x_var, font=("Consolas", 26, "bold")
        ).grid(row=1, column=0, padx=50)
        ttk.Separator(position, orient="vertical").grid(
            row=0, column=1, rowspan=2, sticky="ns", padx=30
        )
        ttk.Label(position, text="Y / pitch", font=("Segoe UI", 11)).grid(row=0, column=2)
        ttk.Label(
            position, textvariable=self.current_y_var, font=("Consolas", 26, "bold")
        ).grid(row=1, column=2, padx=50)
        position.columnconfigure(0, weight=1)
        position.columnconfigure(2, weight=1)

        controls = ttk.LabelFrame(root, text="目标位置（多圈 T 型轨迹）", padding=12)
        controls.pack(fill="x")
        self._axis_controls(
            controls,
            row=0,
            label="X / yaw",
            variable=self.target_x_var,
            entry_variable=self.entry_x_var,
            minimum=X_UI_MIN_DEG,
            maximum=X_UI_MAX_DEG,
        )
        self._axis_controls(
            controls,
            row=2,
            label="Y / pitch",
            variable=self.target_y_var,
            entry_variable=self.entry_y_var,
            minimum=DEFAULT_Y_MIN_DEG,
            maximum=DEFAULT_Y_MAX_DEG,
        )
        ttk.Label(
            controls,
            text="X 指令范围 -360°～+360°；Y 驱动硬限制 -100°～+100°。",
            foreground="#a33",
        ).grid(row=4, column=0, columnspan=4, sticky="w", pady=(10, 2))

        tuning = ttk.LabelFrame(root, text="速度环 PID（厂家范围 0～256）", padding=10)
        tuning.pack(fill="x", pady=10)
        ttk.Label(tuning, text="电机轴").grid(row=0, column=0)
        self.pid_axis_box = ttk.Combobox(
            tuning,
            textvariable=self.pid_axis_var,
            values=("X", "Y"),
            width=5,
            state="readonly",
        )
        self.pid_axis_box.grid(row=0, column=1, padx=(6, 16))
        ttk.Label(tuning, text="KP").grid(row=0, column=2)
        ttk.Spinbox(
            tuning, from_=0, to=256, textvariable=self.pid_kp_var, width=7
        ).grid(row=0, column=3, padx=(6, 16))
        ttk.Label(tuning, text="KI").grid(row=0, column=4)
        ttk.Spinbox(
            tuning, from_=0, to=256, textvariable=self.pid_ki_var, width=7
        ).grid(row=0, column=5, padx=(6, 16))
        self.pid_apply_button = ttk.Button(
            tuning,
            text="临时应用",
            command=lambda: self.apply_pid(False),
            state="disabled",
        )
        self.pid_apply_button.grid(row=0, column=6, padx=5)
        self.pid_save_button = ttk.Button(
            tuning,
            text="应用并掉电保存",
            command=lambda: self.apply_pid(True),
            state="disabled",
        )
        self.pid_save_button.grid(row=0, column=7, padx=5)
        ttk.Label(
            tuning,
            text="建议先临时应用并从小到大调；过大会导致空载高频振动。",
        ).grid(row=1, column=0, columnspan=8, sticky="w", pady=(8, 0))

        actions = ttk.Frame(root)
        actions.pack(fill="x", pady=(2, 0))
        ttk.Button(
            actions, text="返回零点 (0°, 0°)", command=self.return_to_zero
        ).pack(side="left")
        ttk.Label(actions, textvariable=self.feedback_var).pack(side="right")
        ttk.Label(
            root,
            textvariable=self.diagnostic_var,
            wraplength=820,
            foreground="#a33",
        ).pack(fill="x", pady=(6, 0))

    def _axis_controls(
        self, parent, row, label, variable, entry_variable, minimum, maximum
    ):
        ttk.Label(parent, text=label, width=10).grid(row=row, column=0, sticky="w")
        scale = ttk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            variable=variable,
            command=lambda _value: self.on_slider(),
        )
        scale.grid(row=row, column=1, sticky="ew", padx=10)
        ttk.Label(parent, textvariable=variable, width=10).grid(row=row, column=2)
        entry = ttk.Entry(parent, textvariable=entry_variable, width=12)
        entry.grid(row=row + 1, column=1, sticky="e", padx=10, pady=(4, 8))
        entry.bind("<Return>", lambda _event: self.apply_entries())
        ttk.Button(parent, text="应用输入", command=self.apply_entries).grid(
            row=row + 1, column=2, pady=(4, 8)
        )
        parent.columnconfigure(1, weight=1)

    def refresh_ports(self, show_error=False):
        if self.demo_mode:
            self.port_box["values"] = ["DEMO"]
            self.port_var.set("DEMO")
            return
        try:
            ports = list_serial_ports()
        except ImportError:
            ports = []
            detail = (
                "当前 Python 未安装 pyserial，无法自动枚举或打开串口。\n\n"
                "Python: %s\n\n请运行：\n%s"
                % (sys.executable, pyserial_install_command())
            )
            self.state_var.set("串口组件缺失；请安装 pyserial")
            self.feedback_var.set(pyserial_install_command())
            if show_error:
                messagebox.showerror("无法枚举串口", detail)
        except Exception as exc:
            ports = []
            detail = "串口枚举失败：%s" % exc
            self.state_var.set(detail)
            self.feedback_var.set("可手动输入 COM4 后尝试连接")
            if show_error:
                messagebox.showerror("无法枚举串口", detail)
        self.port_box["values"] = ports
        current = self.port_var.get().strip()
        if ports and not current:
            self.port_var.set(ports[0])
        elif not ports and not current and not self.demo_mode:
            self.port_var.set("COM4")
            if "pyserial" not in self.state_var.get():
                self.state_var.set("未自动发现串口；可手动输入端口名后连接")

    def _read_motion_values(self):
        speed = int(self.speed_var.get())
        acceleration = int(self.acceleration_var.get())
        if not 1 <= speed <= 1000:
            raise ValueError("位置速度必须为 1～1000 RPM")
        if not 0 <= acceleration <= 65535:
            raise ValueError("加速度必须为 0～65535")
        return speed, acceleration

    def connect(self):
        if self.worker is not None and self.worker.is_alive():
            return
        port = self.port_var.get().strip()
        if not self.demo_mode and not port:
            messagebox.showerror("未选择串口", "请先选择 USB-TTL 对应的 COM 口。")
            return
        try:
            speed, acceleration = self._read_motion_values()
            x_id, y_id = int(self.x_id_var.get()), int(self.y_id_var.get())
            if not 1 <= x_id <= 255 or not 1 <= y_id <= 255 or x_id == y_id:
                raise ValueError("X/Y 电机 ID 必须在 1～255，且不能相同")
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("运动参数错误", str(exc))
            return
        self.connect_button.config(state="disabled")
        self.state_var.set("正在连接；请保持相机垂直地面作为 0°…")
        self.worker = GimbalWorker(
            port, speed, acceleration, self.events, self.demo_mode,
            x_id=x_id, y_id=y_id,
        )
        self.worker.start()

    def disconnect(self):
        if self.worker is not None:
            self.state_var.set("正在失能并断开串口…")
            self.worker.request_stop()

    def toggle_motor_enabled(self):
        if self.worker is not None and self.worker.is_alive():
            self.worker.set_enabled(not self.motor_enabled)

    def apply_motion_parameters(self):
        if self.worker is None or not self.worker.is_alive():
            return
        try:
            speed, acceleration = self._read_motion_values()
            self.worker.set_motion_parameters(speed, acceleration)
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("运动参数错误", str(exc))

    def apply_pid(self, save):
        if self.worker is None or not self.worker.is_alive():
            return
        try:
            axis = self.pid_axis_var.get()
            kp = int(self.pid_kp_var.get())
            ki = int(self.pid_ki_var.get())
            if not 0 <= kp <= 256 or not 0 <= ki <= 256:
                raise ValueError("KP/KI 必须为 0～256")
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("PID 参数错误", str(exc))
            return
        if save and not messagebox.askyesno(
            "确认保存 PID",
            "保存后断电仍生效。是否将 %s 轴速度环 KP=%d、KI=%d 写入电机？"
            % (axis, kp, ki),
        ):
            return
        self.worker.set_speed_pid(axis, kp, ki, save)

    def on_slider(self):
        self.entry_x_var.set("%.1f" % self.target_x_var.get())
        self.entry_y_var.set("%.1f" % self.target_y_var.get())
        self.schedule_target()

    def apply_entries(self):
        try:
            x_deg = float(self.entry_x_var.get())
            y_deg = float(self.entry_y_var.get())
        except ValueError:
            messagebox.showerror("输入错误", "角度必须是有效数字。")
            return
        if not math.isfinite(x_deg) or not math.isfinite(y_deg):
            messagebox.showerror("输入错误", "角度必须是有限数字。")
            return
        if not X_UI_MIN_DEG <= x_deg <= X_UI_MAX_DEG:
            messagebox.showerror("X 超出界面范围", "X 输入范围为 -360°～+360°。")
            return
        if not DEFAULT_Y_MIN_DEG <= y_deg <= DEFAULT_Y_MAX_DEG:
            messagebox.showerror("Y 硬限位", "Y 轴只允许 -100°～+100°。")
            return
        self.target_x_var.set(x_deg)
        self.target_y_var.set(y_deg)
        self.schedule_target(immediate=True)

    def schedule_target(self, immediate=False):
        if self.send_after_id is not None:
            self.after_cancel(self.send_after_id)
        self.send_after_id = self.after(0 if immediate else 50, self.send_target)

    def send_target(self):
        self.send_after_id = None
        if self.worker is None or not self.worker.is_alive():
            return
        x_deg = float(self.target_x_var.get())
        y_deg = float(self.target_y_var.get())
        try:
            self.worker.set_target(x_deg, y_deg)
            self.feedback_var.set("目标：X %.1f° / Y %.1f°" % (x_deg, y_deg))
        except (F32CLimitError, F32CError) as exc:
            self.feedback_var.set(str(exc))

    def return_to_zero(self):
        self.target_x_var.set(0.0)
        self.target_y_var.set(0.0)
        self.entry_x_var.set("0.0")
        self.entry_y_var.set("0.0")
        self.schedule_target(immediate=True)

    def _set_connected_controls(self, connected):
        self.connect_button.config(state="disabled" if connected else "normal")
        self.disconnect_button.config(state="normal" if connected else "disabled")
        self.enable_button.config(state="normal" if connected else "disabled")
        self.motion_button.config(state="normal" if connected else "disabled")
        self.pid_apply_button.config(state="normal" if connected else "disabled")
        self.pid_save_button.config(state="normal" if connected else "disabled")
        self.port_box.config(state="disabled" if connected else "normal")

    def process_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "state":
                    self.state_var.set(payload)
                elif kind == "connected":
                    port, zero_deg, enabled = payload
                    self.connected_port = port
                    self.motor_enabled = enabled
                    self.state_var.set(
                        "已连接并使能：%s" % port if enabled
                        else "串口已连接，电机失能：%s" % port
                    )
                    if zero_deg is None:
                        self.zero_var.set("尚未收到双轴有效反馈；正在重试")
                    else:
                        self.zero_var.set(
                            "软件零点（电机总角度）：X %+.1f° / Y %+.1f°"
                            % zero_deg
                        )
                    self.enable_button.config(
                        text="电机失能" if enabled else "电机使能"
                    )
                    self._set_connected_controls(True)
                    if zero_deg is None:
                        self.enable_button.config(state="disabled")
                elif kind == "referenced":
                    self.zero_var.set(
                        "软件零点（电机总角度）：X %+.1f° / Y %+.1f°"
                        % payload
                    )
                    self.enable_button.config(state="normal")
                    self.state_var.set(
                        "串口已连接，反馈正常，电机失能：%s" % self.connected_port
                    )
                elif kind == "feedback_issue":
                    detail, attempted_disable = payload
                    self.state_var.set(
                        (
                            "反馈异常；已尝试失能，实际状态未确认：%s"
                            if attempted_disable else
                            "串口已连接，无有效反馈，本程序未使能：%s"
                        ) % self.connected_port
                    )
                    self.feedback_var.set("反馈超时；检查电源、共地、TX/RX 和 ID")
                    self.diagnostic_var.set(detail)
                    self.current_x_var.set("--.-°")
                    self.current_y_var.set("--.-°")
                    self.enable_button.config(state="disabled")
                elif kind == "feedback_restored":
                    self.feedback_var.set("双轴反馈已恢复；可手动使能")
                    self.diagnostic_var.set("")
                    self.enable_button.config(state="normal")
                    if not self.motor_enabled:
                        self.state_var.set(
                            "串口已连接，反馈正常，电机失能：%s"
                            % self.connected_port
                        )
                elif kind == "command_error":
                    self.feedback_var.set("命令未执行：%s" % payload)
                elif kind == "motor_enabled":
                    self.motor_enabled = payload
                    if payload:
                        self.state_var.set("已连接并使能：%s" % self.connected_port)
                        self.enable_button.config(text="电机失能")
                    else:
                        self.state_var.set(
                            "已连接，电机失能；实时位置继续读取：%s"
                            % self.connected_port
                        )
                        self.enable_button.config(text="电机使能")
                elif kind == "position":
                    self.current_x_var.set("%+.1f°" % payload[0])
                    self.current_y_var.set("%+.1f°" % payload[1])
                elif kind == "target":
                    self.feedback_var.set(
                        "已下发：X %.1f° / Y %.1f°" % (payload[0], payload[1])
                    )
                elif kind == "motion":
                    self.feedback_var.set(
                        "运动参数已应用：%d RPM / 加速度 %d" % payload
                    )
                elif kind == "pid":
                    axis, kp, ki, saved = payload
                    self.feedback_var.set(
                        "%s 轴速度 PID 已应用：KP=%d / KI=%d%s"
                        % (axis, kp, ki, "（已保存）" if saved else "（临时）")
                    )
                elif kind == "error":
                    self.state_var.set("错误：%s" % payload)
                    messagebox.showerror("云台通信错误", payload)
                elif kind == "disconnected":
                    self.motor_enabled = False
                    self.connected_port = ""
                    self.state_var.set("已失能并断开串口")
                    self.zero_var.set("尚未读取电机零点")
                    self.diagnostic_var.set("")
                    self.enable_button.config(text="电机失能")
                    self._set_connected_controls(False)
                    self.worker = None
        except queue.Empty:
            pass
        self.after(50, self.process_events)

    def on_close(self):
        if self.worker is not None:
            self.worker.request_stop()
            self.worker.join(timeout=2.5)
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo", action="store_true", help="simulate motors without opening a COM port"
    )
    args = parser.parse_args()
    app = GimbalGUI(demo_mode=args.demo)
    app.mainloop()


if __name__ == "__main__":
    main()
