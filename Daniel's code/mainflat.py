import cv2
import numpy as np
import time
import sys
import os
import threading
from collections import defaultdict, deque
from object_detection import ObjectDetection
from filterpy.kalman import KalmanFilter
from height_sensor import HeightSensor

# ================= 树莓派极致性能优化 =================
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["MKL_NUM_THREADS"] = "2"
os.environ["OPENBLAS_NUM_THREADS"] = "2"
import torch
torch.set_num_threads(2)
torch.set_grad_enabled(False)
# =====================================================

# ================= 尝试导入电机 SDK =================
SDK_AVAILABLE = False
try:
    sys.path.append("..")
    from scservo_sdk import *
    SDK_AVAILABLE = True
except ImportError:
    print("Warning: scservo_sdk not found, running in simulation mode")
# =====================================================

# ================= 全局配置 =================
VIDEO_SOURCE = 6
IMG_W, IMG_H = 1920, 1080
YOLO_IMGSZ = 316          # Pi 甜点分辨率（兼顾精度与帧率）
FX_APPROX = 800
CONF_THRESHOLD = 0.4
TRACKER_TYPE = "bytetrack" # ByteTrack 比 BoT-SORT 快 2~3 倍
LOST_TIMEOUT = 2.0
DEVICENAME = 'COM5'
BAUDRATE = 115200
SERVO_PITCH = 1
SERVO_OTHER = 2
MIN_ANGLE, MAX_ANGLE = 0, 180
DEFAULT_ANGLE = 90
GIMBAL_MODE = "PITCH_ROLL"

# 🔑 防抖专用参数（增大死区 + 降低增益 + 增强平滑 + 限速）
DEADZONE_X = 65           # 🔺 横向死区：误差<65px 电机静止
DEADZONE_Y = 35           # 🔺 纵向死区：误差<35px 电机静止
GAIN_X  = 0.65          # 🔻 横向增益：响应更温和
GAIN_Y  = 0.4            # 🔻 纵向增益：克服重力但不过冲
EMA_X   = 0.08            # 🔻 横向平滑：重阻尼滤高频
EMA_Y   = 0.10            # 🔻 纵向平滑
STEP_X  = 2             # 🔻 横向单帧最大转角（度）
STEP_Y  = 2             # 🔻 纵向单帧最大转角（度）
DIR_X = 1; DIR_Y = -1
HEIGHT_INTERVAL = 0.2     # 5Hz 节流
# ============================================

# ================= 摄像头多线程 =================
class CameraStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
        self.ret, self.frame = self.cap.read()
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret, self.frame = ret, frame

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy()

    def release(self):
        self.running = False
        self.cap.release()

# ================= 异步电机控制器 =================
class AsyncMotorController:
    def __init__(self, active, port_handler, packet_handler, p_id, o_id):
        self.active = active
        self.ph = port_handler
        self.pkt = packet_handler
        self.p_id, self.o_id = p_id, o_id
        self.target_pitch = DEFAULT_ANGLE
        self.target_other = DEFAULT_ANGLE
        self.cmd_queue = []
        self.lock = threading.Lock()
        if active:
            threading.Thread(target=self._worker, daemon=True).start()

    def _worker(self):
        while True:
            with self.lock:
                if self.cmd_queue:
                    sid, pos = self.cmd_queue.pop(0)
            if 'sid' in locals():
                try:
                    self.pkt.WritePosEx(sid, pos, 2500, 60)
                except: pass
            time.sleep(0.005)

    def set_target_angles(self, pitch, other):
        self.target_pitch = np.clip(pitch, MIN_ANGLE, MAX_ANGLE)
        self.target_other = np.clip(other, MIN_ANGLE, MAX_ANGLE)
        if self.active:
            with self.lock:
                self.cmd_queue.append((self.p_id, int((self.target_pitch % 360) * 4096 / 360)))
                self.cmd_queue.append((self.o_id, int((self.target_other % 360) * 4096 / 360)))

    def reset(self):
        self.set_target_angles(DEFAULT_ANGLE, DEFAULT_ANGLE)

    def get_pitch_rad(self): return np.deg2rad(self.target_pitch - DEFAULT_ANGLE)
    def get_roll_rad(self): return np.deg2rad(self.target_other - DEFAULT_ANGLE) if GIMBAL_MODE=="PITCH_ROLL" else 0.0

# ================= 初始化 =================
od = ObjectDetection()
od.model.eval()
cam = CameraStream(VIDEO_SOURCE)
height_sensor = HeightSensor(offset_cm=9.3, default_height_m=30.0)

gimbal = None
if SDK_AVAILABLE:
    try:
        ph = PortHandler(DEVICENAME)
        pkt = sms_sts(ph)
        if ph.openPort() and ph.setBaudRate(BAUDRATE):
            print(f"Serial {DEVICENAME} OK")
            gimbal = AsyncMotorController(True, ph, pkt, SERVO_PITCH, SERVO_OTHER)
    except Exception as e:
        print(f"Serial init failed: {e}")
if gimbal is None:
    gimbal = AsyncMotorController(False, None, None, SERVO_PITCH, SERVO_OTHER)

cv2.namedWindow("UAV Tracker", cv2.WINDOW_NORMAL)
cv2.resizeWindow("UAV Tracker", 1280, 720)

tracking = {"selected_id": None, "last_seen": None}
current_boxes = []
fps_window = deque(maxlen=20)
real_height = 30.0
last_height_t = time.perf_counter()

smooth = {"x": DEFAULT_ANGLE, "y": DEFAULT_ANGLE, "prev_x": DEFAULT_ANGLE, "prev_y": DEFAULT_ANGLE}

def make_history():
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0.]]); kf.F = np.array([[1.]]); kf.H = np.array([[1.]])
    kf.P *= 10.; kf.R = 0.5; kf.Q = 0.01
    return {"px": None, "py": None, "pt": None, "kf": kf, "trail": deque(maxlen=40), "buf": deque(maxlen=4)}
track_history = defaultdict(make_history)

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and current_boxes:
        for b in current_boxes:
            if b["box"][0] <= x <= b["box"][2] and b["box"][1] <= y <= b["box"][3]:
                tracking["selected_id"] = b["id"]
                tracking["last_seen"] = time.perf_counter()
                smooth["x"] = smooth["prev_x"] = DEFAULT_ANGLE
                smooth["y"] = smooth["prev_y"] = DEFAULT_ANGLE
                print(f"Locked ID: {b['id']}")
                return
cv2.setMouseCallback("UAV Tracker", mouse_callback)
print("System started | Anti-jitter params | Target: 12+ FPS")

# ================= 主循环 =================
while True:
    t0 = time.perf_counter()
    
    # 节流读取高度
    if t0 - last_height_t > HEIGHT_INTERVAL:
        real_height = 40   #height_sensor.get_height_m()
        last_height_t = t0

    ret, frame = cam.read()
    if not ret: break
    
    h, w = frame.shape[:2]
    center = (w // 2, h // 2)

    try:
        res = od.model.track(frame, persist=True, conf=CONF_THRESHOLD, iou=0.5,
                             tracker=f"{TRACKER_TYPE}.yaml", verbose=False, imgsz=YOLO_IMGSZ)[0]
    except: continue

    current_boxes.clear()
    detected_ids = set()
    if res.boxes.id is not None:
        boxes = res.boxes.xyxy.cpu().numpy()
        ids = res.boxes.id.cpu().numpy().astype(int)
        detected_ids = set(ids)
        for b, tid in zip(boxes, ids):
            x1,y1,x2,y2 = b.astype(int)
            current_boxes.append({"box":(x1,y1,x2,y2), "id":tid, "cx":(x1+x2)//2, "cy":(y1+y2)//2})

    # 丢失超时检测
    if tracking["selected_id"] is not None:
        if tracking["selected_id"] in detected_ids:
            tracking["last_seen"] = t0
        elif tracking["last_seen"] and (t0 - tracking["last_seen"] > LOST_TIMEOUT):
            tracking["selected_id"] = None; tracking["last_seen"] = None
            gimbal.reset()

    locked_speed = None
    if res.boxes.id is not None:
        boxes = res.boxes.xyxy.cpu().numpy()
        ids = res.boxes.id.cpu().numpy().astype(int)
        cls = res.boxes.cls.cpu().numpy().astype(int)

        for b, tid, cid in zip(boxes, ids, cls):
            x1,y1,x2,y2 = b.astype(int)
            cx, cy = (x1+x2)//2, (y1+y2)//2
            if tracking["selected_id"] and tid != tracking["selected_id"]: continue

            hist = track_history[tid]
            if tracking["selected_id"] == tid:
                hist["trail"].append((cx,cy))
                for i in range(1, len(hist["trail"])):
                    cv2.line(frame, hist["trail"][i-1], hist["trail"][i], (255,100,0), 1)

            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 1)
            cv2.putText(frame, f"ID:{tid} {od.classes[cid]}", (x1,y1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            # 几何测速（含俯仰/横滚实时补偿）
            dt = t0 - hist["pt"] if hist["pt"] else 0.0
            speed = None
            if 0 < dt < 0.3 and hist["px"] is not None:
                dx_pixel = cx - hist["px"]
                dy_pixel = cy - hist["py"]
                pitch_rad = gimbal.get_pitch_rad()
                roll_rad = gimbal.get_roll_rad()
                
                cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)
                dx_rot = dx_pixel * cos_r - dy_pixel * sin_r
                dy_rot = dx_pixel * sin_r + dy_pixel * cos_r
                
                cos_p = np.clip(np.cos(pitch_rad), 0.1, 1.0)
                scale_x = real_height / (FX_APPROX * cos_p)
                scale_y = real_height / (FX_APPROX * cos_p * cos_p)
                
                dist_meter = np.hypot(dx_rot * scale_x, dy_rot * scale_y)
                raw_speed = dist_meter / dt
                
                hist["buf"].append(raw_speed)
                avg = np.mean(hist["buf"])
                hist["kf"].predict(); hist["kf"].update([[avg]])
                speed = abs(hist["kf"].x[0,0])
            
            hist["px"], hist["py"], hist["pt"] = cx, cy, t0

            # 速度显示逻辑
            if speed is not None:
                if tracking["selected_id"] is None:
                    cv2.putText(frame, f"{speed:.1f}m/s", (x1, y1-28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                elif tid == tracking["selected_id"]:
                    locked_speed = speed

            # 🔑 防抖控制逻辑（长方形死区 + 解耦参数）
            if tracking["selected_id"] is not None:
                err_x = cx - center[0]
                err_y = cy - center[1]
                in_deadzone = (abs(err_x) < DEADZONE_X) and (abs(err_y) < DEADZONE_Y)

                if in_deadzone:
                    final_x = smooth["prev_x"]
                    final_y = smooth["prev_y"]
                    status = "LOCKED"
                    color = (0, 255, 0)
                else:
                    target_x = DEFAULT_ANGLE + err_x * DIR_X * GAIN_X
                    target_y = DEFAULT_ANGLE + err_y * DIR_Y * GAIN_Y
                    
                    smooth["x"] = smooth["x"] * (1 - EMA_X) + target_x * EMA_X
                    smooth["y"] = smooth["y"] * (1 - EMA_Y) + target_y * EMA_Y
                    
                    dx = np.clip(smooth["x"] - smooth["prev_x"], -STEP_X, STEP_X)
                    dy = np.clip(smooth["y"] - smooth["prev_y"], -STEP_Y, STEP_Y)
                    final_x = smooth["prev_x"] + dx
                    final_y = smooth["prev_y"] + dy
                    smooth["prev_x"], smooth["prev_y"] = final_x, final_y
                    status = "TRACKING"
                    color = (0, 255, 255)

                gimbal.set_target_angles(final_y, final_x)

                cv2.drawMarker(frame, center, (255,0,255), cv2.MARKER_CROSS, 20, 1)
                cv2.circle(frame, (cx,cy), 4, (0,0,255), -1)
                cv2.arrowedLine(frame, (cx,cy), center, (0,255,255), 2, tipLength=0.2)

    # 锁定模式：右上角固定显示速度
    if tracking["selected_id"] is not None and locked_speed is not None:
        cv2.putText(frame, f"Speed: {locked_speed:.2f} m/s", (w-260, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,255), 2)

    # 计算实时 FPS
    frame_dt = time.perf_counter() - t0
    fps_window.append(1.0/frame_dt if frame_dt>0 else 0.0)
    fps_avg = sum(fps_window)/len(fps_window)

    # ================= HUD（原版高可读性风格） =================
    panel_x, panel_y = 10, 10
    panel_w, panel_h = 680, 180
    
    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)
    
    pitch_geo = gimbal.target_pitch - DEFAULT_ANGLE
    cos_p = np.clip(np.cos(np.deg2rad(pitch_geo)), 0.1, 1.0)
    scale_display = real_height / (FX_APPROX * cos_p)
    
    hud_lines = [
        f"Mode: {GIMBAL_MODE} | Target ID: {tracking['selected_id'] if tracking['selected_id'] else 'None'}",
        f"Height: {real_height:.2f}m | Geo Pitch: {pitch_geo:+.1f}deg | Roll: {gimbal.target_other-DEFAULT_ANGLE:+.1f}deg",
        f"Scale: {scale_display:.5f} m/px | FPS: {fps_avg:.1f}",
        f"DeadZone: X<{DEADZONE_X} Y<{DEADZONE_Y} | Gains X:{GAIN_X} Y:{GAIN_Y}",
        "Keys: [m] switch mode | [c] clear lock | [q] quit"
    ]
    
    for i, text in enumerate(hud_lines):
        color = (0, 255, 255) if i == 0 else (200, 200, 200)
        cv2.putText(frame, text,
                    (panel_x + 15, panel_y + 30 + i * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    cv2.imshow("UAV Tracker", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break
    if key == ord('c'):
        tracking["selected_id"]=None; tracking["last_seen"]=None
        gimbal.reset(); smooth["x"]=smooth["prev_x"]=DEFAULT_ANGLE; smooth["y"]=smooth["prev_y"]=DEFAULT_ANGLE
    if key == ord('m'):
        GIMBAL_MODE = "YAW_PITCH" if GIMBAL_MODE=="PITCH_ROLL" else "PITCH_ROLL"
        gimbal.reset()

cam.release(); cv2.destroyAllWindows()
print("Exited")