"""Tkinter GUI for safe real-time control of a WHEELTEC F32C gimbal."""

from __future__ import annotations

import argparse
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from safe_f32c_driver import (
    DEFAULT_Y_MAX_DEG,
    DEFAULT_Y_MIN_DEG,
    F32CLimitError,
    SafeF32CGimbal,
)


X_UI_MIN_DEG = -180.0
X_UI_MAX_DEG = 180.0
STATUS_PERIOD_S = 0.10
COMMAND_PERIOD_S = 0.02


def list_serial_ports():
    """Return available serial device names without opening them."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [port.device for port in list_ports.comports()]


class GimbalWorker(threading.Thread):
    """Own the UART so command and feedback operations never race."""

    def __init__(self, port, speed_rpm, events, demo_mode=False):
        super().__init__(daemon=True)
        self.port = port
        self.speed_rpm = speed_rpm
        self.events = events
        self.demo_mode = demo_mode
        self.stop_event = threading.Event()
        self.target_lock = threading.Lock()
        self.pending_target = None
        self.transport = None
        self.gimbal = None

    def set_target(self, x_deg, y_deg):
        # Validate again before crossing the GUI/worker boundary.
        if not DEFAULT_Y_MIN_DEG <= y_deg <= DEFAULT_Y_MAX_DEG:
            raise F32CLimitError(
                "Y target must remain in %.1f..%.1f deg"
                % (DEFAULT_Y_MIN_DEG, DEFAULT_Y_MAX_DEG)
            )
        with self.target_lock:
            self.pending_target = (float(x_deg), float(y_deg))

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
                "pyserial is not installed; run: python -m pip install pyserial"
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

    def run(self):
        try:
            self.events.put(("state", "正在打开串口并初始化电机…"))
            self.transport = self._open_transport()
            self.gimbal = SafeF32CGimbal(
                self.transport,
                y_min_deg=DEFAULT_Y_MIN_DEG,
                y_max_deg=DEFAULT_Y_MAX_DEG,
            )
            self.gimbal.start(
                speed_rpm=self.speed_rpm,
                power_on_delay_s=0.0 if self.demo_mode else 1.5,
                require_feedback=True,
            )
            # start() captures the physical startup pose as software zero.
            self.gimbal.set_relative_angles(0.0, 0.0)
            self.events.put(("connected", self.port or "模拟串口"))

            next_status_s = 0.0
            next_command_s = 0.0
            last_target = (0.0, 0.0)
            while not self.stop_event.is_set():
                now = time.monotonic()
                target = self._pop_target()
                if target is not None:
                    last_target = target
                if target is not None and now >= next_command_s:
                    # The safety driver validates Y before writing either axis.
                    self.gimbal.set_relative_angles(*target)
                    next_command_s = time.monotonic() + COMMAND_PERIOD_S
                    self.events.put(("target", last_target))

                if now >= next_status_s:
                    x_deg, y_deg = self.gimbal.read_relative_angles()
                    self.events.put(("position", (x_deg, y_deg)))
                    next_status_s = time.monotonic() + STATUS_PERIOD_S
                time.sleep(0.003)
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
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

        self.title("WHEELTEC F32C 双轴云台控制")
        self.geometry("780x560")
        self.minsize(720, 520)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.port_var = tk.StringVar()
        self.speed_var = tk.IntVar(value=20)
        self.state_var = tk.StringVar(value="未连接")
        self.current_x_var = tk.StringVar(value="--.-°")
        self.current_y_var = tk.StringVar(value="--.-°")
        self.target_x_var = tk.DoubleVar(value=0.0)
        self.target_y_var = tk.DoubleVar(value=0.0)
        self.entry_x_var = tk.StringVar(value="0.0")
        self.entry_y_var = tk.StringVar(value="0.0")
        self.feedback_var = tk.StringVar(value="等待连接")

        self._build_ui()
        self.refresh_ports()
        self.after(50, self.process_events)

    def _build_ui(self):
        root = ttk.Frame(self, padding=16)
        root.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(root, text="连接", padding=12)
        connection.pack(fill="x")
        ttk.Label(connection, text="串口").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(
            connection, textvariable=self.port_var, width=18, state="readonly"
        )
        self.port_box.grid(row=0, column=1, padx=(8, 14))
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=(0, 18)
        )
        ttk.Label(connection, text="位置速度 RPM").grid(row=0, column=3)
        self.speed_spin = ttk.Spinbox(
            connection, from_=1, to=1000, textvariable=self.speed_var, width=8
        )
        self.speed_spin.grid(row=0, column=4, padx=8)
        self.connect_button = ttk.Button(
            connection, text="连接并使能", command=self.connect
        )
        self.connect_button.grid(row=0, column=5, padx=6)
        self.disconnect_button = ttk.Button(
            connection, text="失能并断开", command=self.disconnect, state="disabled"
        )
        self.disconnect_button.grid(row=0, column=6, padx=6)

        ttk.Label(connection, textvariable=self.state_var).grid(
            row=1, column=0, columnspan=7, sticky="w", pady=(10, 0)
        )

        position = ttk.LabelFrame(root, text="实时反馈位置（相对软件零点）", padding=12)
        position.pack(fill="x", pady=12)
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

        controls = ttk.LabelFrame(root, text="目标位置", padding=12)
        controls.pack(fill="both", expand=True)
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
            text="驱动硬限制：Y 轴仅允许 -100°～+100°；区间外命令在串口发送前被拒绝。",
            foreground="#a33",
        ).grid(row=4, column=0, columnspan=4, sticky="w", pady=(14, 4))

        actions = ttk.Frame(root)
        actions.pack(fill="x", pady=(12, 0))
        ttk.Button(actions, text="回软件零点 (0°, 0°)", command=self.home).pack(
            side="left"
        )
        ttk.Label(actions, textvariable=self.feedback_var).pack(side="right")

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
        entry.grid(row=row + 1, column=1, sticky="e", padx=10, pady=(6, 12))
        entry.bind("<Return>", lambda _event: self.apply_entries())
        ttk.Button(parent, text="应用输入", command=self.apply_entries).grid(
            row=row + 1, column=2, pady=(6, 12)
        )
        parent.columnconfigure(1, weight=1)

    def refresh_ports(self):
        ports = list_serial_ports()
        if self.demo_mode:
            ports = ["DEMO"]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def connect(self):
        if self.worker is not None and self.worker.is_alive():
            return
        port = self.port_var.get()
        if not self.demo_mode and not port:
            messagebox.showerror("未选择串口", "请先选择 USB-TTL 对应的 COM 口。")
            return
        try:
            speed = int(self.speed_var.get())
            if not 1 <= speed <= 1000:
                raise ValueError
        except ValueError:
            messagebox.showerror("速度错误", "位置速度必须为 1～1000 RPM。")
            return
        self.connect_button.config(state="disabled")
        self.state_var.set("正在连接；请保持相机垂直地面作为 0°…")
        self.worker = GimbalWorker(port, speed, self.events, self.demo_mode)
        self.worker.start()

    def disconnect(self):
        if self.worker is not None:
            self.state_var.set("正在失能并断开…")
            self.worker.request_stop()

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
        if not X_UI_MIN_DEG <= x_deg <= X_UI_MAX_DEG:
            messagebox.showerror("X 超出界面范围", "X 输入范围为 -180°～+180°。")
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
        except F32CLimitError as exc:
            messagebox.showerror("驱动硬限位", str(exc))

    def home(self):
        self.target_x_var.set(0.0)
        self.target_y_var.set(0.0)
        self.entry_x_var.set("0.0")
        self.entry_y_var.set("0.0")
        self.schedule_target(immediate=True)

    def process_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "state":
                    self.state_var.set(payload)
                elif kind == "connected":
                    self.state_var.set("已连接并使能：%s" % payload)
                    self.connect_button.config(state="disabled")
                    self.disconnect_button.config(state="normal")
                    self.port_box.config(state="disabled")
                    self.speed_spin.config(state="disabled")
                elif kind == "position":
                    self.current_x_var.set("%+.1f°" % payload[0])
                    self.current_y_var.set("%+.1f°" % payload[1])
                elif kind == "target":
                    self.feedback_var.set(
                        "已下发：X %.1f° / Y %.1f°" % (payload[0], payload[1])
                    )
                elif kind == "error":
                    self.state_var.set("错误：%s" % payload)
                    messagebox.showerror("云台通信错误", payload)
                elif kind == "disconnected":
                    self.state_var.set("已失能并断开")
                    self.connect_button.config(state="normal")
                    self.disconnect_button.config(state="disabled")
                    self.port_box.config(state="readonly")
                    self.speed_spin.config(state="normal")
                    self.worker = None
        except queue.Empty:
            pass
        self.after(50, self.process_events)

    def on_close(self):
        if self.worker is not None:
            self.worker.request_stop()
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

