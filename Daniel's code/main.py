import cv2
import numpy as np
import time
from object_detection import ObjectDetection
from filterpy.kalman import KalmanFilter
from collections import defaultdict, deque

# ================= 配置 =================
VIDEO_SOURCE = 6
FIXED_DISTANCE_M = 25
FIXED_PITCH_DEG = 60
FX_APPROX = 800
CONF_THRESHOLD = 0.5
TRACKER_TYPE = "botsort"
TRAIL_LENGTH = 90
SPEED_WINDOW = 5
MAX_DT = 0.3
LOST_TIMEOUT = 1.0  # 🔑 目标丢失超时时间（秒）
# ==========================================

# ---------- 跟踪状态管理 ----------
tracking_state = {
    "selected_id": None,
    "last_seen_time": None
}
current_frame_boxes = []  # 供鼠标点击命中检测

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if not current_frame_boxes:
            return
        for b in current_frame_boxes:
            x1, y1, x2, y2 = b["box"]
            if x1 <= x <= x2 and y1 <= y <= y2:
                tracking_state["selected_id"] = b["tid"]
                tracking_state["last_seen_time"] = time.perf_counter()  # 重置丢失计时
                print(f"🎯 单击选中目标 ID: {b['tid']}")
                return

# ---------- 初始化 ----------
od = ObjectDetection()
cap = cv2.VideoCapture(VIDEO_SOURCE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cv2.namedWindow("UAV Speed Tracker", cv2.WINDOW_NORMAL)
cv2.resizeWindow("UAV Speed Tracker", 1920, 1080)
cv2.setMouseCallback("UAV Speed Tracker", mouse_callback)

pitch_rad = np.deg2rad(FIXED_PITCH_DEG)
fps_times = deque(maxlen=30)

def create_speed_kf():
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0.]])
    kf.F = np.array([[1.]])
    kf.H = np.array([[1.]])
    kf.P *= 10.
    kf.R = 0.5
    kf.Q = 0.01
    return kf

track_history = defaultdict(lambda: {
    "prev_x": None, "prev_y": None, "prev_time": None,
    "kf": create_speed_kf(),
    "trail": deque(maxlen=TRAIL_LENGTH),
    "speed_buffer": deque(maxlen=SPEED_WINDOW)
})

print(f"🚀 系统启动 | 1920x1080 | 丢失超时: {LOST_TIMEOUT}s | 单击锁定 | 'c'清除 | 'q'退出")

# ================= 主循环 =================
while True:
    t0 = time.perf_counter()
    dt = 0.0  # 防止 HUD 报 NameError
    ret, frame = cap.read()
    if not ret:
        break
    
    h, w = frame.shape[:2]
    img_center = (w // 2, h // 2)
    
    fps_times.append(time.time())
    fps = (len(fps_times) - 1) / (fps_times[-1] - fps_times[0]) if len(fps_times) > 1 else 0.0

    Z = FIXED_DISTANCE_M
    pixel_size = (Z * np.cos(pitch_rad)) / FX_APPROX

    try:
        results = od.model.track(
            frame, persist=True, conf=CONF_THRESHOLD, iou=0.5,
            tracker=f"{TRACKER_TYPE}.yaml", verbose=False
        )[0]
    except Exception as e:
        print(f"⚠️ 跟踪异常: {e}")
        continue

    current_frame_boxes.clear()
    detected_ids = set()
    
    if results.boxes.id is not None:
        boxes_xyxy = results.boxes.xyxy.cpu().numpy()
        track_ids = results.boxes.id.cpu().numpy().astype(int)
        class_ids = results.boxes.cls.cpu().numpy().astype(int)
        detected_ids = set(track_ids)

        for box_xyxy, tid, cls in zip(boxes_xyxy, track_ids, class_ids):
            x1, y1, x2, y2 = box_xyxy.astype(int)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            current_frame_boxes.append({"box": (x1, y1, x2, y2), "tid": tid, "cx": cx, "cy": cy})

    # 🔑 核心逻辑：丢失超时检测 & 自动恢复广域模式
    if tracking_state["selected_id"] is not None:
        if tracking_state["selected_id"] in detected_ids:
            tracking_state["last_seen_time"] = t0  # 目标可见，刷新时间
        elif tracking_state["last_seen_time"] is not None:
            lost_duration = t0 + 1 - tracking_state["last_seen_time"]
            if lost_duration > LOST_TIMEOUT:
                print(f"⏱️ 目标 ID {tracking_state['selected_id']} 丢失超过 {LOST_TIMEOUT}s，恢复广域识别模式")
                tracking_state["selected_id"] = None
                tracking_state["last_seen_time"] = None

    # ================= 绘制 =================
    if results.boxes.id is not None:
        boxes_xyxy = results.boxes.xyxy.cpu().numpy()
        track_ids = results.boxes.id.cpu().numpy().astype(int)
        class_ids = results.boxes.cls.cpu().numpy().astype(int)

        for box_xyxy, tid, cls in zip(boxes_xyxy, track_ids, class_ids):
            x1, y1, x2, y2 = box_xyxy.astype(int)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # 过滤逻辑
            if tracking_state["selected_id"] is not None and tid != tracking_state["selected_id"]:
                continue

            history = track_history[tid]
            kf = history["kf"]

            history["trail"].append((cx, cy))
            for i in range(1, len(history["trail"])):
                cv2.line(frame, history["trail"][i-1], history["trail"][i], (255, 100, 0), 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"ID:{tid} {od.classes[cls]}"
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 测速
            dt = t0 - history["prev_time"] if history["prev_time"] is not None else 0.0
            if 0 < dt < MAX_DT and history["prev_x"] is not None:
                dx = cx - history["prev_x"]
                dy = cy - history["prev_y"]
                dist_pixel = np.hypot(dx, dy)
                raw_speed = (dist_pixel * pixel_size) / dt
                
                history["speed_buffer"].append(raw_speed)
                avg_speed = np.mean(history["speed_buffer"])
                kf.predict()
                kf.update(np.array([[avg_speed]]))
                speed = abs(kf.x[0, 0])
                cv2.putText(frame, f"{speed:.2f} m/s", (x1, y1-40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 3)

            history["prev_x"] = cx
            history["prev_y"] = cy
            history["prev_time"] = t0

            # 🎯 调试箭头 & 状态提示
            if tracking_state["selected_id"] is not None:
                cv2.drawMarker(frame, img_center, (255, 0, 255), cv2.MARKER_CROSS, 30, 2)
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.arrowedLine(frame, (cx, cy), img_center, (0, 255, 255), 3, tipLength=0.3)
                
                # 丢失期间显示倒计时提示
                if tracking_state["last_seen_time"] is not None:
                    lost_sec = LOST_TIMEOUT - (t0 - tracking_state["last_seen_time"])
                    if lost_sec > 0:
                        cv2.putText(frame, f"LOST! {lost_sec:.1f}s", (x1, y1-70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    else:
                        cv2.putText(frame, "RECOVERING", (x1, y1-70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                else:
                    cv2.putText(frame, "TRACKING", (x1, y1-70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    # ================= HUD =================
    panel_x, panel_y = 10, 10
    panel_w, panel_h = 480, 170
    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (panel_x+panel_w, panel_y+panel_h), (0,0,0), -1)
    frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)

    # 动态状态文本
    if tracking_state["selected_id"] is None:
        status_txt = "👁️ 广域识别模式"
        status_color = (200, 200, 200)
    else:
        lost_duration = t0 - tracking_state["last_seen_time"] if tracking_state["last_seen_time"] else 0
        if lost_duration > LOST_TIMEOUT:
            status_txt = "🔄 已自动恢复广域模式"
            status_color = (0, 255, 0)
        elif tracking_state["last_seen_time"] and (t0 - tracking_state["last_seen_time"] > 0.5):
            status_txt = f"🔍 目标丢失，尝试重捕 ({LOST_TIMEOUT - lost_duration:.1f}s)"
            status_color = (0, 165, 255)
        else:
            status_txt = f"🎯 跟踪中: ID {tracking_state['selected_id']}"
            status_color = (0, 255, 255)

    hud_lines = [
        status_txt,
        f"RES: {w}x{h} | FPS: {fps:.1f} | DT: {dt:.3f}s",
        f"Z: {Z}m | Pitch: {FIXED_PITCH_DEG}° | 比例: {pixel_size:.5f} m/px",
        "['c'] 清除选择  |  ['q'] 退出"
    ]
    for i, txt in enumerate(hud_lines):
        color = status_color if i == 0 else (200, 200, 200)
        cv2.putText(frame, txt, (panel_x+15, panel_y+30+i*28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    cv2.imshow("UAV Speed Tracker", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        tracking_state["selected_id"] = None
        tracking_state["last_seen_time"] = None
        print("🧹 已手动清除锁定，恢复全局显示")

cap.release()
cv2.destroyAllWindows()