#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ST3215 双轴云台控制 - 完整 PID 版（0~180°）
新增功能：
1. 每个轴独立 PID 控制器
2. GUI 中加入 Kp / Ki / Kd 调节滑块（实时生效）
3. 每个轴独立 “启用 PID” 开关
4. 实时读取当前角度并显示
5. PID 模式下：滑块/输入框设置的是【目标角度】，PID 自动驱动电机到达目标
6. 手动模式下：直接控制（和之前一样）
"""

import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox

# ================== 添加 SDK 路径 ==================
sys.path.append("..")
from scservo_sdk import *

# ================== 配置（请修改这里） ==================
SERVO_ID_1 = 2          # ← 改成你的实际电机 ID
SERVO_ID_2 = 3
BAUDRATE   = 115200
DEVICENAME = 'COM5'     # ← 改成你的实际 COM 口

MOVING_SPEED = 2000
MOVING_ACC   = 30

MIN_ANGLE = 0
MAX_ANGLE = 180
DEFAULT_ANGLE = 90

# ================== PID 控制器类 ==================
class PIDController:
    def __init__(self, kp=1.2, ki=0.02, kd=0.4):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt=0.05):
        """计算 PID 输出（角度增量）"""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output   # 返回角度增量（度）

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

def angle_to_pos(angle):
    return int((angle % 360) * 4096 / 360)

def pos_to_angle(pos):
    return round((pos % 4096) * 360 / 4096, 1)

# ================== 初始化串口 ==================
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    messagebox.showerror("错误", f"无法打开串口 {DEVICENAME}")
    sys.exit(1)
if not portHandler.setBaudRate(BAUDRATE):
    messagebox.showerror("错误", "无法设置波特率")
    portHandler.closePort()
    sys.exit(1)

print("串口初始化成功")

# ================== PID 实例 ==================
pid1 = PIDController(kp=1.2, ki=0.02, kd=0.4)
pid2 = PIDController(kp=1.2, ki=0.02, kd=0.4)

# ================== 目标角度 & 防抖 ==================
target_angle = {SERVO_ID_1: DEFAULT_ANGLE, SERVO_ID_2: DEFAULT_ANGLE}
last_sent_pos = {SERVO_ID_1: angle_to_pos(DEFAULT_ANGLE), SERVO_ID_2: angle_to_pos(DEFAULT_ANGLE)}

def read_current_angle(servo_id):
    """读取电机当前位置"""
    try:
        pos, speed, comm_result, error = packetHandler.ReadPosSpeed(servo_id)
        if comm_result == COMM_SUCCESS and error == 0:
            return pos_to_angle(pos)
        return None
    except:
        return None

def set_servo_position(servo_id, position):
    """直接发送位置（防抖）"""
    global last_sent_pos
    if abs(position - last_sent_pos.get(servo_id, 0)) >= 5:
        comm_result, error = packetHandler.WritePosEx(servo_id, position, MOVING_SPEED, MOVING_ACC)
        if comm_result == COMM_SUCCESS and error == 0:
            last_sent_pos[servo_id] = position
            # print(f"Servo {servo_id} → {pos_to_angle(position):.1f}°")
        time.sleep(0.05)

# ================== GUI ==================
class ServoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ST3215 双轴云台 + PID 控制（0~180°）")
        self.root.geometry("820x680")

        # 每个轴的 PID 启用状态
        self.pid_enabled = {SERVO_ID_1: tk.BooleanVar(value=False),
                            SERVO_ID_2: tk.BooleanVar(value=False)}

        # PID 参数变量
        self.pid_vars = {
            SERVO_ID_1: {
                "kp": tk.DoubleVar(value=1.2),
                "ki": tk.DoubleVar(value=0.02),
                "kd": tk.DoubleVar(value=0.4)
            },
            SERVO_ID_2: {
                "kp": tk.DoubleVar(value=1.2),
                "ki": tk.DoubleVar(value=0.02),
                "kd": tk.DoubleVar(value=0.4)
            }
        }

        self.create_servo_panel(SERVO_ID_1, "Servo 1 - 俯仰轴", 0)
        self.create_servo_panel(SERVO_ID_2, "Servo 2 - 横滚轴", 1)

        # 底部全局按钮
        btn_frame = ttk.Frame(root)
        btn_frame.grid(row=2, column=0, columnspan=2, pady=15)
        ttk.Button(btn_frame, text="两轴回到中位 90°", command=self.go_to_middle).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="重置两个 PID", command=self.reset_pids).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="关闭程序", command=self.on_closing).pack(side=tk.LEFT, padx=10)

        # 启动 PID 循环（每 50ms 执行一次）
        self.pid_loop()

        self.go_to_middle()

    def create_servo_panel(self, servo_id, title, row):
        frame = ttk.LabelFrame(self.root, text=title, padding=15)
        frame.grid(row=row, column=0, columnspan=2, padx=20, pady=15, sticky="ew")

        # === 目标角度控制 ===
        ttk.Label(frame, text="目标角度 (0° ~ 180°):").grid(row=0, column=0, sticky="w", pady=5)
        self.target_scale = ttk.Scale(frame, from_=MIN_ANGLE, to=MAX_ANGLE, orient=tk.HORIZONTAL,
                                      length=500, command=lambda v, sid=servo_id: self.on_target_change(sid, float(v)))
        self.target_scale.grid(row=1, column=0, columnspan=2, pady=5, sticky="ew")
        self.target_scale.set(DEFAULT_ANGLE)

        self.target_entry = ttk.Entry(frame, width=12, font=("Arial", 11))
        self.target_entry.grid(row=1, column=2, padx=10)
        self.target_entry.insert(0, str(DEFAULT_ANGLE))
        self.target_entry.bind("<Return>", lambda e, sid=servo_id: self.on_target_entry(sid))

        ttk.Button(frame, text="设置目标", command=lambda sid=servo_id: self.on_target_entry(sid)).grid(row=1, column=3, padx=5)

        # 当前角度显示
        ttk.Label(frame, text="当前角度:").grid(row=2, column=0, sticky="w", pady=8)
        self.current_label = ttk.Label(frame, text="--- °", font=("Arial", 12, "bold"), foreground="blue")
        self.current_label.grid(row=2, column=1, sticky="w", pady=8)

        # === PID 调节区 ===
        pid_frame = ttk.LabelFrame(frame, text="PID 参数调节", padding=10)
        pid_frame.grid(row=3, column=0, columnspan=4, pady=15, sticky="ew")

        ttk.Label(pid_frame, text="Kp:").grid(row=0, column=0, sticky="w")
        kp_scale = ttk.Scale(pid_frame, from_=0.0, to=5.0, orient=tk.HORIZONTAL, length=180,
                             variable=self.pid_vars[servo_id]["kp"],
                             command=lambda v, sid=servo_id: self.update_pid_params(sid))
        kp_scale.grid(row=0, column=1, padx=5)
        self.kp_val = ttk.Label(pid_frame, text="1.20")
        self.kp_val.grid(row=0, column=2)

        ttk.Label(pid_frame, text="Ki:").grid(row=1, column=0, sticky="w")
        ki_scale = ttk.Scale(pid_frame, from_=0.0, to=0.5, orient=tk.HORIZONTAL, length=180,
                             variable=self.pid_vars[servo_id]["ki"],
                             command=lambda v, sid=servo_id: self.update_pid_params(sid))
        ki_scale.grid(row=1, column=1, padx=5)
        self.ki_val = ttk.Label(pid_frame, text="0.02")
        self.ki_val.grid(row=1, column=2)

        ttk.Label(pid_frame, text="Kd:").grid(row=2, column=0, sticky="w")
        kd_scale = ttk.Scale(pid_frame, from_=0.0, to=3.0, orient=tk.HORIZONTAL, length=180,
                             variable=self.pid_vars[servo_id]["kd"],
                             command=lambda v, sid=servo_id: self.update_pid_params(sid))
        kd_scale.grid(row=2, column=1, padx=5)
        self.kd_val = ttk.Label(pid_frame, text="0.40")
        self.kd_val.grid(row=2, column=2)

        # 启用 PID 开关
        ttk.Checkbutton(pid_frame, text="启用 PID 控制（自动跟踪目标）", variable=self.pid_enabled[servo_id],
                        command=lambda sid=servo_id: self.toggle_pid(sid)).grid(row=3, column=0, columnspan=3, pady=8)

        # 保存引用
        if servo_id == SERVO_ID_1:
            self.panel1 = frame
            self.current_label1 = self.current_label
            self.target_scale1 = self.target_scale
            self.target_entry1 = self.target_entry
        else:
            self.panel2 = frame
            self.current_label2 = self.current_label
            self.target_scale2 = self.target_scale
            self.target_entry2 = self.target_entry

    def update_pid_params(self, servo_id):
        """滑块改变时实时更新 PID 参数"""
        p = self.pid_vars[servo_id]
        if servo_id == SERVO_ID_1:
            pid1.kp = p["kp"].get()
            pid1.ki = p["ki"].get()
            pid1.kd = p["kd"].get()
            # 更新显示值
            self.kp_val.config(text=f"{pid1.kp:.2f}")
            self.ki_val.config(text=f"{pid1.ki:.3f}")
            self.kd_val.config(text=f"{pid1.kd:.2f}")
        else:
            pid2.kp = p["kp"].get()
            pid2.ki = p["ki"].get()
            pid2.kd = p["kd"].get()
            self.kp_val.config(text=f"{pid2.kp:.2f}")
            self.ki_val.config(text=f"{pid2.ki:.3f}")
            self.kd_val.config(text=f"{pid2.kd:.2f}")

    def toggle_pid(self, servo_id):
        """切换 PID 开关时重置积分"""
        if servo_id == SERVO_ID_1:
            pid1.reset()
        else:
            pid2.reset()

    def on_target_change(self, servo_id, angle):
        """目标滑块移动"""
        target_angle[servo_id] = angle
        # 更新输入框
        if servo_id == SERVO_ID_1:
            self.target_entry1.delete(0, tk.END)
            self.target_entry1.insert(0, f"{angle:.1f}")
        else:
            self.target_entry2.delete(0, tk.END)
            self.target_entry2.insert(0, f"{angle:.1f}")

    def on_target_entry(self, servo_id):
        """输入框设置目标"""
        try:
            angle = float(self.target_entry1.get() if servo_id == SERVO_ID_1 else self.target_entry2.get())
            if MIN_ANGLE <= angle <= MAX_ANGLE:
                target_angle[servo_id] = angle
                if servo_id == SERVO_ID_1:
                    self.target_scale1.set(angle)
                else:
                    self.target_scale2.set(angle)
            else:
                messagebox.showwarning("范围错误", f"角度必须在 {MIN_ANGLE}~{MAX_ANGLE} 之间")
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字")

    def pid_loop(self):
        """每 50ms 执行一次 PID 计算 + 读取当前角度"""
        for sid in [SERVO_ID_1, SERVO_ID_2]:
            current = read_current_angle(sid)
            if current is not None:
                # 更新界面当前角度显示
                if sid == SERVO_ID_1:
                    self.current_label1.config(text=f"{current:.1f}°")
                else:
                    self.current_label2.config(text=f"{current:.1f}°")

                # 如果启用 PID，则进行控制
                if self.pid_enabled[sid].get():
                    error = target_angle[sid] - current
                    if sid == SERVO_ID_1:
                        delta = pid1.compute(error)
                        new_angle = current + delta
                    else:
                        delta = pid2.compute(error)
                        new_angle = current + delta

                    # 限制范围
                    new_angle = max(MIN_ANGLE, min(MAX_ANGLE, new_angle))
                    pos = angle_to_pos(new_angle)
                    set_servo_position(sid, pos)

        # 定时循环
        self.root.after(50, self.pid_loop)

    def go_to_middle(self):
        for sid in [SERVO_ID_1, SERVO_ID_2]:
            target_angle[sid] = DEFAULT_ANGLE
            if sid == SERVO_ID_1:
                self.target_scale1.set(DEFAULT_ANGLE)
                self.target_entry1.delete(0, tk.END)
                self.target_entry1.insert(0, str(DEFAULT_ANGLE))
            else:
                self.target_scale2.set(DEFAULT_ANGLE)
                self.target_entry2.delete(0, tk.END)
                self.target_entry2.insert(0, str(DEFAULT_ANGLE))
            set_servo_position(sid, angle_to_pos(DEFAULT_ANGLE))

    def reset_pids(self):
        pid1.reset()
        pid2.reset()
        messagebox.showinfo("PID 重置", "两个 PID 积分与历史误差已清零")

    def on_closing(self):
        if messagebox.askokcancel("退出", "确定关闭程序并关闭串口？"):
            portHandler.closePort()
            self.root.destroy()

# ================== 启动 ==================
if __name__ == "__main__":
    root = tk.Tk()
    app = ServoGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
    portHandler.closePort()
    print("串口已关闭")