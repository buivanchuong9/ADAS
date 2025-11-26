"""
WebSocket API for Real-time Inference
Webcam → Base64 frames → Model inference → Detection results
"""
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict, Any
import json
import base64
import cv2
import numpy as np
from pathlib import Path
from ultralytics import YOLO
import time

router = APIRouter(prefix="/ws", tags=["WebSocket"])

# Model cache
models_cache: Dict[str, Any] = {}
MODELS_DIR = Path("ai_models/weights")


def load_model(model_id: str):
    """Load model vào cache với error handling"""
    if model_id in models_cache:
        return models_cache[model_id]
    
    model_file = None
    if model_id.startswith("yolov8"):
        model_file = MODELS_DIR / f"{model_id}.pt"
    elif model_id == "yolop":
        model_file = MODELS_DIR / "yolop.pt"
    elif model_id == "midas_small":
        model_file = MODELS_DIR / "midas_small.pt"
    
    if model_file and model_file.exists():
        if model_id.startswith("yolov8"):
            try:
                print(f"Loading model: {model_id} from {model_file}")
                models_cache[model_id] = YOLO(str(model_file))
                print(f"✅ Model {model_id} loaded successfully")
            except Exception as e:
                print(f"❌ Error loading {model_id}: {e}")
                return None
        # TODO: Add YOLOP and MiDaS loading
        return models_cache[model_id]
    
    print(f"❌ Model file not found: {model_file}")
    return None


@router.websocket("/infer/{model_id}")
async def websocket_inference(websocket: WebSocket, model_id: str):
    """
    WebSocket endpoint cho real-time inference
    
    Client gửi: {"frame": "base64_encoded_image"}
    Server trả: {"detections": [...], "fps": 30, "latency_ms": 15}
    """
    await websocket.accept()
    
    try:
        # Load model
        model = load_model(model_id)
        if model is None:
            await websocket.send_json({
                "error": f"Model {model_id} not found or not downloaded",
                "success": False
            })
            await websocket.close()
            return
        
        await websocket.send_json({
            "success": True,
            "message": f"Model {model_id} loaded successfully",
            "ready": True
        })
        
        frame_count = 0
        
        while True:
            # Nhận frame từ client
            data = await websocket.receive_text()
            request = json.loads(data)
            
            if "frame" not in request:
                await websocket.send_json({"error": "No frame provided"})
                continue
            
            # Decode base64 image
            frame_data = base64.b64decode(request["frame"].split(",")[1] if "," in request["frame"] else request["frame"])
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                await websocket.send_json({"error": "Invalid frame data"})
                continue
            
            # Run inference
            import time
            start_time = time.time()
            
            if model_id.startswith("yolov8"):
                # Giảm conf threshold để nhạy hơn, imgsz nhỏ hơn để nhanh hơn
                results = model(frame, conf=0.15, iou=0.4, imgsz=416, verbose=False)[0]
                
                detections = []
                for box in results.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = results.names[cls]
                    
                    detections.append({
                        "bbox": [float(x1), float(y1), float(x2), float(y2)],
                        "confidence": conf,
                        "class": class_name,
                        "class_id": cls
                    })
                
                latency_ms = (time.time() - start_time) * 1000
                frame_count += 1
                
                # Send results
                await websocket.send_json({
                    "success": True,
                    "detections": detections,
                    "frame_count": frame_count,
                    "latency_ms": round(latency_ms, 2),
                    "model_id": model_id
                })
            
    except WebSocketDisconnect:
        print(f"WebSocket disconnected for model {model_id}")
    except Exception as e:
        print(f"WebSocket error: {str(e)}")
        try:
            await websocket.send_json({"error": str(e), "success": False})
        except:
            pass


@router.websocket("/adas-unified")
async def websocket_adas_unified(websocket: WebSocket):
    """
    ADAS Unified endpoint - Tất cả tính năng trong 1 model
    Client gửi: {"frame": "base64..."}
    Server trả: {detections, lanes, alerts, stats}
    """
    await websocket.accept()
    
    try:
        # Import ADAS Unified Model
        import sys
        from pathlib import Path
        sys.path.append(str(Path(__file__).parent.parent))
        from ai_models.adas_unified import ADASUnifiedModel
        
        # Load model
        print("Loading ADAS Unified Model...")
        adas_model = ADASUnifiedModel()
        print("✅ ADAS Unified Model ready")
        
        await websocket.send_json({
            "success": True,
            "message": "ADAS Unified Model loaded",
            "features": ["detection", "lanes", "distance", "ttc", "alerts"]
        })
        
        prev_detections = None
        prev_time = time.time()
        
        while True:
            data = await websocket.receive_text()
            request = json.loads(data)
            
            if "frame" not in request:
                continue
            
            # Decode frame
            frame_data = base64.b64decode(request["frame"].split(",")[1] if "," in request["frame"] else request["frame"])
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                continue
            
            # Run unified inference
            current_time = time.time()
            delta_time = current_time - prev_time
            
            result = adas_model.run_inference(frame, prev_detections, delta_time)
            
            prev_detections = result['detections']
            prev_time = current_time
            
            # Send results
            await websocket.send_json({
                "success": True,
                "detections": result['detections'],
                "lanes": result['lanes'],
                "alerts": result['alerts'],
                "stats": result['stats'],
                "model": "ADAS Unified"
            })
            
    except WebSocketDisconnect:
        print("ADAS Unified WebSocket disconnected")
    except Exception as e:
        print(f"ADAS Unified error: {e}")
        import traceback
        traceback.print_exc()


@router.websocket("/stream")
async def websocket_stream(websocket: WebSocket):
    """
    Generic WebSocket endpoint cho multiple models
    Client gửi: {"model_id": "yolov8n", "frame": "base64..."}
    """
    await websocket.accept()
    
    try:
        await websocket.send_json({
            "success": True,
            "message": "WebSocket connected",
            "supported_models": ["yolov8n", "yolov8s", "yolov8m", "yolop", "midas_small"]
        })
        
        current_model = None
        current_model_id = None
        
        while True:
            data = await websocket.receive_text()
            request = json.loads(data)
            
            model_id = request.get("model_id", "yolov8n")
            
            # Reload model if changed
            if model_id != current_model_id:
                current_model = load_model(model_id)
                current_model_id = model_id
                
                if current_model is None:
                    await websocket.send_json({
                        "error": f"Model {model_id} not available",
                        "success": False
                    })
                    continue
            
            if "frame" not in request:
                continue
            
            # Decode frame
            frame_data = base64.b64decode(request["frame"].split(",")[1] if "," in request["frame"] else request["frame"])
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                continue
            
            # Run inference
            import time
            start_time = time.time()
            
            if model_id.startswith("yolov8"):
                # Giảm conf threshold và imgsz để nhanh + nhạy hơn
                results = current_model(frame, conf=0.15, iou=0.4, imgsz=416, verbose=False)[0]
                
                detections = []
                for box in results.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    
                    detections.append({
                        "bbox": [float(x1), float(y1), float(x2), float(y2)],
                        "confidence": conf,
                        "class": results.names[cls],
                        "class_id": cls
                    })
                
                latency_ms = (time.time() - start_time) * 1000
                
                await websocket.send_json({
                    "success": True,
                    "detections": detections,
                    "latency_ms": round(latency_ms, 2),
                    "model_id": model_id
                })
            
    except WebSocketDisconnect:
        print("WebSocket stream disconnected")
    except Exception as e:
        print(f"WebSocket stream error: {str(e)}")
