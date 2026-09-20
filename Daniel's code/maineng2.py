import cv2
import numpy as np
import time
import sys
from collections import defaultdict, deque
from object_detection import ObjectDetection
from filterpy.kalman import KalmanFilter
from scservo_sdk import *

# ================= 配置区 =================
VIDEO_SOURCE = 6
CONF_THRESHOLD = 0.4
LOST_TIMEOUT = 2.0
FIXED_HEIGHT = 2.5          # 测速参考高度（米）
FX_APPROX = 2000             # 相机焦距近似值（像素）

# 电机
DEVICENAME = 'COM5'
BAUDRATE = 115200
SERVO_ID_PITCH = 1
SERVO_ID_ROLL = 2
MIN_POS = 0
MAX_POS = 4095
CENTER_POS = 1024           # 🔑 默认中位 1024 = 90°
MOVING_SPEED = 2400
MOVING_ACC = 45

# 控制参数
GAIN_X = 0.12
GAIN_Y = 0.12
DEADZONE = 20
MAX_STEP = 150
REVERSE_ROLL = False
REVERSE_PITCH = True
# ========================================

# ================= 初始化 =================
# 1. 串口初始化
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)
if not portHandler.openPort():
    print("❌ 串口打开失败"); sys.exit()
if not portHandler.setBaudRate(BAUDRATE):
    print("❌ 波特率失败"); sys.exit()

def get_pos(sid):
    pos, _, res, _ = packetHandler.ReadPosSpeed(sid)
    return pos if res == COMM_SUCCESS else CENTER_POS

curr_roll = get_pos(SERVO_ID_ROLL)
curr_pitch = get_pos(SERVO_ID_PITCH)

def move_motor():
    packetHandler.WritePosEx(SERVO_ID_ROLL, int(curr_roll), MOVING_SPEED, MOVING_ACC)
    packetHandler.WritePosEx(SERVO_ID_PITCH, int(curr_pitch), MOVING_SPEED, MOVING_ACC)

def reset_gimbal():
    global curr_roll, curr_pitch
    curr_roll = CENTER_POS
    curr_pitch = CENTER_POS
    move_motor()

# 2. 视觉初始化
od = ObjectDetection()
cap = cv2.VideoCapture(VIDEO_SOURCE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cv2.namedWindow("Tracker", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Tracker", 1280, 720)

# 3. 状态与FPS队列
tracking = {
    "selected_id": None,
    "last_seen": None,
    "mode": "WIDE"
}
current_boxes = []
fps_window = deque(maxlen=20)

def make_hist():
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0.]])
    kf.F = np.array([[1.]])
    kf.H = np.array([[1.]])
    kf.P *= 10; kf.R = 0.5; kf.Q = 0.01
    return {"px":None,"py":None,"pt":None,"kf":kf,"buf":deque(maxlen=4)}
history = defaultdict(make_hist)

# ================= 鼠标 =================
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        for b in current_boxes:
            x1,y1,x2,y2 = b["box"]
            if x1 <= x <= x2 and y1 <= y <= y2:
                tracking["selected_id"] = b["id"]
                tracking["mode"] = "TRACK"
                tracking["last_seen"] = time.perf_counter()
                print(f"🎯 锁定 ID {b['id']}")
cv2.setMouseCallback("Tracker", mouse_callback)

print("🚀 启动：默认广域模式 | 点击进入跟踪 | 'c' 清除 | 'q' 退出")

# ================= 主循环 =================
while True:
    t0 = time.perf_counter()
    ret, frame = cap.read()
    if not ret: break
    
    h, w = frame.shape[:2]
    center = (w//2, h//2)

    try:
        res = od.model.track(frame, persist=True, conf=CONF_THRESHOLD, iou=0.5, verbose=False, imgsz=416)[0]
    except: continue

    current_boxes.clear()
    detected_ids = set()

    if res.boxes.id is not None:
        boxes = res.boxes.xyxy.cpu().numpy()
        ids = res.boxes.id.cpu().numpy().astype(int)

        for box, tid in zip(boxes, ids):
            x1,y1,x2,y2 = box.astype(int)
            cx, cy = (x1+x2)//2, (y1+y2)//2
            current_boxes.append({"box":(x1,y1,x2,y2), "id":tid, "cx":cx, "cy":cy})
            detected_ids.add(tid)

    # ================= 丢失逻辑 =================
    if tracking["mode"] == "TRACK":
        if tracking["selected_id"] in detected_ids:
            tracking["last_seen"] = t0
        elif t0 - tracking["last_seen"] > LOST_TIMEOUT:
            print("❌ 丢失目标 -> 回广域")
            tracking["mode"] = "WIDE"
            tracking["selected_id"] = None
            reset_gimbal()

    target_center = None
    locked_speed = None

    # 🔑 计算当前云台相对于1024(90°)的偏差角（弧度）
    roll_dev_deg = (curr_roll - CENTER_POS) * 360.0 / 4096.0
    pitch_dev_deg = (curr_pitch - CENTER_POS) * 360.0 / 4096.0
    roll_rad = np.deg2rad(roll_dev_deg)
    pitch_rad = np.deg2rad(pitch_dev_deg)

    # ================= 遍历目标 =================
    if res.boxes.id is not None:
        boxes = res.boxes.xyxy.cpu().numpy()
        ids = res.boxes.id.cpu().numpy().astype(int)

        for box, tid in zip(boxes, ids):
            x1,y1,x2,y2 = box.astype(int)
            cx, cy = (x1+x2)//2, (y1+y2)//2
            hist = history[tid]

            # ===== 速度计算（几何透视补偿） =====
            dt = t0 - hist["pt"] if hist["pt"] else 0
            speed = None
            if 0 < dt < 0.3 and hist["px"] is not None:
                dx = cx - hist["px"]
                dy = cy - hist["py"]
                
                # 1. 横滚旋转补偿
                cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)
                dx_rot = dx * cos_r - dy * sin_r
                dy_rot = dx * sin_r + dy * cos_r
                
                # 2. 俯仰透视缩放（以1024偏差角为基准）
                cos_p = np.clip(np.cos(pitch_rad), 0.1, 1.0)
                scale_x = FIXED_HEIGHT / (FX_APPROX * cos_p)
                scale_y = FIXED_HEIGHT / (FX_APPROX * cos_p * cos_p)
                
                dist_meter = np.hypot(dx_rot * scale_x, dy_rot * scale_y)
                raw_speed = dist_meter / dt
                
                hist["buf"].append(raw_speed)
                avg = np.mean(hist["buf"])
                hist["kf"].predict()
                hist["kf"].update([[avg]])
                speed = abs(hist["kf"].x[0,0])
            hist["px"], hist["py"], hist["pt"] = cx, cy, t0

            # ===== 绘制 =====
            if tracking["mode"] == "TRACK" and tid == tracking["selected_id"]:
                target_center = (cx, cy)
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,255),3)
                cv2.drawMarker(frame,(cx,cy),(0,0,255),cv2.MARKER_CROSS,20,2)
                locked_speed = speed
            else:
                cv2.rectangle(frame,(x1,y1),(x2,y2),(255,0,0),1)

            if speed and tracking["mode"] == "WIDE":
                cv2.putText(frame,f"{speed:.1f}m/s",(x1,y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),2)

    # ================= 云台控制 =================
    if tracking["mode"] == "TRACK" and target_center:
        error_x = target_center[0] - center[0]
        error_y = target_center[1] - center[1]

        if abs(error_x) > DEADZONE:
            step = int(np.clip(error_x * GAIN_X, -MAX_STEP, MAX_STEP))
            curr_roll += (-step if REVERSE_ROLL else step)

        if abs(error_y) > DEADZONE:
            step = int(np.clip(error_y * GAIN_Y, -MAX_STEP, MAX_STEP))
            curr_pitch += (-step if REVERSE_PITCH else step)

        curr_roll = np.clip(curr_roll, MIN_POS, MAX_POS)
        curr_pitch = np.clip(curr_pitch, MIN_POS, MAX_POS)
        move_motor()

    # ================= 右上角黄色速度显示 =================
    if tracking["mode"] == "TRACK" and locked_speed is not None:
        cv2.putText(frame, f"{locked_speed:.2f} m/s", (w-260, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2) # BGR黄色

    # ================= HUD（保持原版风格） =================
    frame_dt = time.perf_counter() - t0
    fps_window.append(1.0 / frame_dt if frame_dt > 0 else 0.0)
    fps_avg = sum(fps_window) / len(fps_window)

    panel_x, panel_y = 10, 10
    panel_w, panel_h = 640, 160
    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)

    speed_txt = f"{locked_speed:.2f}" if locked_speed is not None else "N/A"

    hud_lines = [
        f"Mode: {tracking['mode']} | Target ID: {tracking['selected_id'] if tracking['selected_id'] else 'None'}",
        f"Dev Angle: Roll={roll_dev_deg:+.1f}° Pitch={pitch_dev_deg:+.1f}° | Speed: {speed_txt} m/s",
        f"Scale: 0.02000 m/px | FPS: {fps_avg:.1f}",
        f"DeadZone: <{DEADZONE}px | Gains X:{GAIN_X} Y:{GAIN_Y}",
        "Keys: [c] clear lock | [q] quit"
    ]

    for i, text in enumerate(hud_lines):
        color = (0, 255, 255) if i == 0 else (200, 200, 200)
        cv2.putText(frame, text,
                    (panel_x + 15, panel_y + 30 + i * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # 中心十字线
    cv2.line(frame,(center[0],0),(center[0],h),(200,200,200),1)
    cv2.line(frame,(0,center[1]),(w,center[1]),(200,200,200),1)
    cv2.imshow("Tracker", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        tracking["mode"] = "WIDE"
        tracking["selected_id"] = None
        reset_gimbal()

# ================= 退出 =================
print("🛑 退出 -> 云台回中")
reset_gimbal()
cap.release()
portHandler.closePort()
cv2.destroyAllWindows()