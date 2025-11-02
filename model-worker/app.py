from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import base64
import cv2
import numpy as np
from ultralytics import YOLO
import torch
import importlib
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()


# Định nghĩa các model có sẵn
AVAILABLE_MODELS = {
    "yolov8n": lambda: YOLO("yolov8n.pt"),
    "yolov5s": lambda: torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True),
    # Thêm các model khác nếu muốn
    # "yolov7": lambda: ...
    # "pose": lambda: ...
    # "object_tracking": lambda: ...
}

# Cache model đã load
model_cache = {}

def get_model(model_name):
    if model_name not in AVAILABLE_MODELS:
        raise ValueError(f"Model {model_name} not supported")
    if model_name not in model_cache:
        model_cache[model_name] = AVAILABLE_MODELS[model_name]()
    return model_cache[model_name]

class FrameRequest(BaseModel):
    frame_b64: str

class Detection(BaseModel):
    id: int
    cls: str
    conf: float
    bbox: list
    distance_m: float

class InferenceResponse(BaseModel):
    detections: list[Detection]
    stats: dict

@app.post("/infer/{model_name}", response_model=InferenceResponse)
async def infer(model_name: str, request: FrameRequest):
    try:
        start_time = time.time()
        
        # Decode base64 frame
        frame_data = base64.b64decode(request.frame_b64)
        nparr = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if frame is None:
            raise ValueError("Invalid frame data")
        
        # Lấy model theo tên
        model = get_model(model_name)
        detections = []
        # YOLOv8 inference
        if model_name.startswith("yolov8"):
            results = model(frame, conf=0.5)
            for idx, result in enumerate(results):
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = box.conf[0].item()
                    cls_id = int(box.cls[0].item())
                    cls_name = model.names[cls_id]
                    bbox_height = y2 - y1
                    distance_m = 1000 / (bbox_height + 1) if bbox_height > 0 else 100
                    detections.append(Detection(
                        id=idx,
                        cls=cls_name,
                        conf=conf,
                        bbox=[x1, y1, x2 - x1, y2 - y1],
                        distance_m=distance_m
                    ))
        # YOLOv5 inference
        elif model_name.startswith("yolov5"):
            results = model(frame)
            for idx, det in enumerate(results.xyxy[0]):
                x1, y1, x2, y2, conf, cls_id = det[:6].tolist()
                cls_name = model.names[int(cls_id)]
                bbox_height = y2 - y1
                distance_m = 1000 / (bbox_height + 1) if bbox_height > 0 else 100
                detections.append(Detection(
                    id=idx,
                    cls=cls_name,
                    conf=conf,
                    bbox=[x1, y1, x2 - x1, y2 - y1],
                    distance_m=distance_m
                ))
        # TODO: Thêm các model khác (pose, tracking, ...)
        
        infer_time = (time.time() - start_time) * 1000
        return InferenceResponse(
            detections=detections,
            stats={"infer_ms": int(infer_time), "model": model_name}
        )
    
    except Exception as e:
        logger.error(f"Inference error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health():
    return {"status": "ok"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
