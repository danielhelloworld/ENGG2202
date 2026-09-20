import cv2
import numpy as np
import time
import sys
from object_detection import ObjectDetection
from scservo_sdk import *

# ================= 配置区 =================
VIDEO_SOURCE = 6
CONF_THRESHOLD = 0.5
TRACKER_TYPE = "botsort"

# --- 电机配置 (基于你的串口代码) ---
DEVICENAME = 'COM5'
BAUDRATE = 115200
SERVO_ID_PITCH = 1  # 俯仰轴
SERVO_ID_ROLL = 2   # 横滚轴
MIN_POS = 0
MAX_POS = 4095
MOVING_SPEED = 2400 
MOVING_ACC = 50

# --- 跟踪控制参数 ---
GAIN_X = 0.12       # 横向灵敏度 (P系数)
GAIN_Y = 0.12       # 纵向灵敏度
DEADZONE = 20       # 像素死区 (偏差小于此值电机不动，防止震动)
MAX_STEP = 150      # 单帧最大步进 (限制速度，防止过冲)

# --- 💡 方向修正开关 ---
# 如果电机往反方向转，请将对应的 True 改为 False，或反之
REVERSE_ROLL = False  
REVERSE_PITCH = True 

# ================= 系统初始化 =================

# 1. 初始化电机串口
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)
if not portHandler.openPort():
    print("❌ 无法打开串口 COM5"); sys.exit()
if not portHandler.setBaudRate(BAUDRATE):
    print("❌ 设置波特率失败"); sys.exit()

# 获取电机当前位置作为起点
def get_current_pos(sid):
    pos, _, res, err = packetHandler.ReadPosSpeed(sid)
    return pos if res == COMM_SUCCESS else 2048

curr_roll_pos = get_current_pos(SERVO_ID_ROLL)
curr_pitch_pos = get_current_pos(SERVO_ID_PITCH)

# 2. 初始化视觉识别
od = ObjectDetection()
cap = cv2.VideoCapture(VIDEO_SOURCE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

tracking_state = {"selected_id": None}
current_frame_boxes = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        for b in current_frame_boxes:
            x1, y1, x2, y2 = b["box"]
            if x1 <= x <= x2 and y1 <= y <= y2:
                tracking_state["selected_id"] = b["tid"]
                print(f"🎯 锁定目标 ID: {b['tid']}")

cv2.namedWindow("Gimbal Tracker", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Gimbal Tracker", mouse_callback)

# ================= 主程序循环 =================

print("🚀 系统启动：点击画面目标开始跟踪 | 按 'c' 释放 | 按 'q' 退出")

while True:
    ret, frame = cap.read()
    if not ret: break
    h, w = frame.shape[:2]
    img_center = (w // 2, h // 2)

    # 运行 YOLO 跟踪
    results = od.model.track(frame, persist=True, conf=CONF_THRESHOLD, verbose=False)[0]
    
    current_frame_boxes.clear()
    target_center = None

    if results.boxes.id is not None:
        boxes = results.boxes.xyxy.cpu().numpy()
        ids = results.boxes.id.cpu().numpy().astype(int)
        
        for box, tid in zip(boxes, ids):
            x1, y1, x2, y2 = box.astype(int)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            current_frame_boxes.append({"box": (x1, y1, x2, y2), "tid": tid})

            # 绘制所有检测到的目标
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)
            
            # 判断是否为选中目标
            if tid == tracking_state["selected_id"]:
                target_center = (cx, cy)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3) # 黄框锁定
                cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

    # 🤖 云台控制逻辑
    if target_center:
        # 计算偏移量 (目标坐标 - 中心坐标)
        error_x = target_center[0] - img_center[0]
        error_y = target_center[1] - img_center[1]

        # --- 横滚轴 (Roll/X) 处理 ---
        if abs(error_x) > DEADZONE:
            step_x = int(error_x * GAIN_X)
            step_x = np.clip(step_x, -MAX_STEP, MAX_STEP)
            
            if REVERSE_ROLL:
                curr_roll_pos -= step_x
            else:
                curr_roll_pos += step_x
            
            curr_roll_pos = np.clip(curr_roll_pos, MIN_POS, MAX_POS)

        # --- 俯仰轴 (Pitch/Y) 处理 ---
        if abs(error_y) > DEADZONE:
            step_y = int(error_y * GAIN_Y)
            step_y = np.clip(step_y, -MAX_STEP, MAX_STEP)
            
            if REVERSE_PITCH:
                curr_pitch_pos -= step_y
            else:
                curr_pitch_pos += step_y
            
            curr_pitch_pos = np.clip(curr_pitch_pos, MIN_POS, MAX_POS)

        # 发送指令到电机
        packetHandler.WritePosEx(SERVO_ID_ROLL, int(curr_roll_pos), MOVING_SPEED, MOVING_ACC)
        packetHandler.WritePosEx(SERVO_ID_PITCH, int(curr_pitch_pos), MOVING_SPEED, MOVING_ACC)

    # 绘制 UI 界面
    cv2.line(frame, (img_center[0], 0), (img_center[0], h), (200, 200, 200), 1)
    cv2.line(frame, (0, img_center[1]), (w, img_center[1]), (200, 200, 200), 1)
    status_text = f"ID:{tracking_state['selected_id']} TRACKING" if target_center else "SEARCHING / CLICK TO LOCK"
    cv2.putText(frame, status_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Gimbal Tracker", frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): 
        break
    elif key == ord('c'): 
        tracking_state["selected_id"] = None
        print("🧹 已清除锁定")

# 释放资源
cap.release()
portHandler.closePort()
cv2.destroyAllWindows()