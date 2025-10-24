from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import base64
import cv2
import numpy as np
from ultralytics import YOLO
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()

# Load YOLOv8 model
model = YOLO("yolov8n.pt")

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

@app.post("/infer", response_model=InferenceResponse)
async def infer(request: FrameRequest):
    try:
        start_time = time.time()
        
        # Decode base64 frame
        frame_data = base64.b64decode(request.frame_b64)
        nparr = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if frame is None:
            raise ValueError("Invalid frame data")
        
        # Run YOLOv8 inference
        results = model(frame, conf=0.5)
        
        detections = []
        for idx, result in enumerate(results):
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = box.conf[0].item()
                cls_id = int(box.cls[0].item())
                cls_name = model.names[cls_id]
                
                # Calculate distance heuristic
                bbox_height = y2 - y1
                distance_m = 1000 / (bbox_height + 1) if bbox_height > 0 else 100
                
                detections.append(Detection(
                    id=idx,
                    cls=cls_name,
                    conf=conf,
                    bbox=[x1, y1, x2 - x1, y2 - y1],
                    distance_m=distance_m
                ))
        
        infer_time = (time.time() - start_time) * 1000
        
        return InferenceResponse(
            detections=detections,
            stats={"infer_ms": int(infer_time)}
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
