from ultralytics import YOLO
import torch

class ObjectDetection:
    def __init__(self, model_path="yolov8n.pt"):  # 🔑 修复：必须是 __init__
        print("Loading YOLOv8 with Ultralytics")
        self.model = YOLO(model_path)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {device.upper()}")
        self.model.to(device)

    @property
    def classes(self):
        return self.model.names

    def detect(self, frame):
        results = self.model(frame, conf=0.5, device=self.model.device)[0]
        boxes = results.boxes.xyxy.cpu().numpy().astype(int)
        class_ids = results.boxes.cls.cpu().numpy().astype(int)
        scores = results.boxes.conf.cpu().numpy()
        
        formatted_boxes = []
        for box in boxes:
            x1, y1, x2, y2 = box
            formatted_boxes.append((x1, y1, x2 - x1, y2 - y1))
        return class_ids, scores, formatted_boxes