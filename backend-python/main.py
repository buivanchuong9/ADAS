# ADAS Backend - Python FastAPI
# SQL Server backend for Windows Server deployment

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
import uvicorn
from typing import List
import json
import base64
from datetime import datetime

from database import get_db, engine
from models import Base, Camera, Driver, Trip, Event, Detection, DriverStatus, Analytics, AIModel, VideoDataset, Label, Alert
from schemas import (
    CameraCreate, CameraResponse, 
    DriverCreate, DriverResponse,
    TripCreate, TripResponse,
    EventCreate, EventResponse,
    AIModelResponse, AIModelCreate,
    AnalyticsResponse
)
from services import ModelService, EventService, CameraService, TripService, DriverService, AnalyticsService
import config

# Import API routers
from api.upload.router import router as upload_router
from api.inference.router import router as inference_router
from api.training.router import router as training_router
from api.dataset.router import router as dataset_router
from api.alerts.router import router as alerts_router

# Create tables
Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="ADAS Backend API",
    description="Advanced Driver Assistance System - Backend API with AI Training",
    version="3.0.0"
)

# CORS Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(upload_router)
app.include_router(inference_router)
app.include_router(training_router)
app.include_router(dataset_router)
app.include_router(alerts_router)

# Initialize services
model_service = ModelService()

# ============ Camera Endpoints ============
@app.get("/api/cameras/list", response_model=List[CameraResponse])
async def get_cameras(db: Session = Depends(get_db)):
    """Get all cameras"""
    camera_service = CameraService(db)
    return camera_service.get_all()

@app.get("/api/cameras/{camera_id}", response_model=CameraResponse)
async def get_camera(camera_id: int, db: Session = Depends(get_db)):
    """Get camera by ID"""
    camera_service = CameraService(db)
    camera = camera_service.get_by_id(camera_id)
    if not camera:
        raise HTTPException(status_code=404, detail="Camera not found")
    return camera

@app.post("/api/cameras", response_model=CameraResponse)
async def create_camera(camera: CameraCreate, db: Session = Depends(get_db)):
    """Create new camera"""
    camera_service = CameraService(db)
    return camera_service.create(camera)

@app.put("/api/cameras/{camera_id}", response_model=CameraResponse)
async def update_camera(camera_id: int, camera: CameraCreate, db: Session = Depends(get_db)):
    """Update camera"""
    camera_service = CameraService(db)
    updated = camera_service.update(camera_id, camera)
    if not updated:
        raise HTTPException(status_code=404, detail="Camera not found")
    return updated

@app.delete("/api/cameras/{camera_id}")
async def delete_camera(camera_id: int, db: Session = Depends(get_db)):
    """Delete camera"""
    camera_service = CameraService(db)
    camera_service.delete(camera_id)
    return {"message": "Camera deleted"}

# ============ AI Models Endpoints ============
@app.get("/api/models/list", response_model=List[AIModelResponse])
async def get_models(db: Session = Depends(get_db)):
    """Get all AI models"""
    models = db.query(AIModel).all()
    return models

@app.get("/api/models/{model_id}", response_model=AIModelResponse)
async def get_model(model_id: int, db: Session = Depends(get_db)):
    """Get model by ID"""
    model = db.query(AIModel).filter(AIModel.id == model_id).first()
    if not model:
        raise HTTPException(status_code=404, detail="Model not found")
    return model

@app.post("/api/models/{model_id}/download")
async def download_model(model_id: int, db: Session = Depends(get_db)):
    """Download AI model"""
    model = db.query(AIModel).filter(AIModel.id == model_id).first()
    if not model:
        raise HTTPException(status_code=404, detail="Model not found")
    
    # TODO: Implement actual download logic
    model.downloaded = True
    model.last_used_at = datetime.utcnow()
    db.commit()
    
    return {"success": True, "message": f"Model {model.name} downloaded"}

@app.post("/api/models/{model_id}/activate")
async def activate_model(model_id: int, db: Session = Depends(get_db)):
    """Activate AI model"""
    # Deactivate all
    db.query(AIModel).update({"is_active": False})
    
    # Activate this one
    model = db.query(AIModel).filter(AIModel.id == model_id).first()
    if not model:
        raise HTTPException(status_code=404, detail="Model not found")
    
    model.is_active = True
    model.last_used_at = datetime.utcnow()
    db.commit()
    
    return {"success": True, "message": f"Model {model.name} activated"}

# ============ Trips Endpoints ============
@app.get("/api/trips/list", response_model=List[TripResponse])
async def get_trips(db: Session = Depends(get_db)):
    """Get all trips"""
    trip_service = TripService(db)
    return trip_service.get_all()

@app.get("/api/trips/{trip_id}", response_model=TripResponse)
async def get_trip(trip_id: int, db: Session = Depends(get_db)):
    """Get trip by ID"""
    trip_service = TripService(db)
    trip = trip_service.get_by_id(trip_id)
    if not trip:
        raise HTTPException(status_code=404, detail="Trip not found")
    return trip

@app.post("/api/trips", response_model=TripResponse)
async def create_trip(trip: TripCreate, db: Session = Depends(get_db)):
    """Create new trip"""
    trip_service = TripService(db)
    return trip_service.create(trip)

@app.post("/api/trips/{trip_id}/end")
async def end_trip(trip_id: int, db: Session = Depends(get_db)):
    """End trip"""
    trip_service = TripService(db)
    trip = trip_service.end_trip(trip_id)
    if not trip:
        raise HTTPException(status_code=404, detail="Trip not found")
    return trip

# ============ Events Endpoints ============
@app.get("/api/events/list", response_model=List[EventResponse])
async def get_events(db: Session = Depends(get_db)):
    """Get all events"""
    event_service = EventService(db)
    return event_service.get_all()

@app.post("/api/events", response_model=EventResponse)
async def create_event(event: EventCreate, db: Session = Depends(get_db)):
    """Log new event"""
    event_service = EventService(db)
    return event_service.create(event)

# ============ Drivers Endpoints ============
@app.get("/api/drivers/list", response_model=List[DriverResponse])
async def get_drivers(db: Session = Depends(get_db)):
    """Get all drivers"""
    driver_service = DriverService(db)
    return driver_service.get_all()

@app.post("/api/drivers", response_model=DriverResponse)
async def create_driver(driver: DriverCreate, db: Session = Depends(get_db)):
    """Create new driver"""
    driver_service = DriverService(db)
    return driver_service.create(driver)

# ============ Analytics Endpoints ============
@app.get("/api/analytics/dashboard")
async def get_dashboard_stats(db: Session = Depends(get_db)):
    """Get dashboard statistics"""
    analytics_service = AnalyticsService(db)
    return analytics_service.get_dashboard_stats()

# ============ Detection Endpoints ============
@app.post("/infer/{model_name}")
async def mock_inference(model_name: str, data: dict):
    """Mock inference endpoint for testing (returns fake detections)"""
    import random
    
    # Generate random detections for testing
    mock_classes = ["car", "person", "truck", "motorcycle", "bus", "bicycle"]
    num_detections = random.randint(1, 5)
    
    detections = []
    for i in range(num_detections):
        cls = random.choice(mock_classes)
        detections.append({
            "id": i,
            "cls": cls,
            "conf": round(random.uniform(0.7, 0.98), 2),
            "bbox": [
                random.randint(50, 400),   # x
                random.randint(50, 300),   # y
                random.randint(80, 200),   # width
                random.randint(80, 180)    # height
            ],
            "distance_m": round(random.uniform(2.0, 30.0), 1)
        })
    
    return {
        "detections": detections,
        "stats": {
            "infer_ms": round(random.uniform(15, 35), 1),
            "model": model_name
        }
    }

@app.post("/api/detections/save")
async def save_detections(data: dict, db: Session = Depends(get_db)):
    """Save detections to database"""
    try:
        detections_data = data.get("detections", [])
        trip_id = data.get("trip_id")
        camera_id = data.get("camera_id", 1)  # Default camera ID
        
        saved_detections = []
        
        for det in detections_data:
            bbox = det.get("bbox", [0, 0, 0, 0])
            detection = Detection(
                class_name=det.get("cls", "unknown"),
                confidence=det.get("conf", 0.0),
                bounding_box=json.dumps(bbox),  # Store as JSON string
                distance_meters=det.get("distance_m", 0.0),
                timestamp=datetime.utcnow(),
                trip_id=trip_id,
                camera_id=camera_id
            )
            db.add(detection)
            saved_detections.append(detection)
        
        db.commit()
        
        return {
            "success": True,
            "saved": len(saved_detections),
            "message": f"Saved {len(saved_detections)} detections to database"
        }
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Error saving detections: {str(e)}")

@app.get("/api/detections/recent")
async def get_recent_detections(limit: int = 50, db: Session = Depends(get_db)):
    """Get recent detections from database"""
    detections = db.query(Detection).order_by(Detection.timestamp.desc()).limit(limit).all()
    
    return {
        "detections": [
            {
                "id": d.id,
                "cls": d.class_name,
                "conf": d.confidence,
                "bbox": json.loads(d.bounding_box) if d.bounding_box else [0, 0, 0, 0],
                "distance_m": d.distance_meters,
                "timestamp": d.timestamp.isoformat() if d.timestamp else None,
                "camera_id": d.camera_id
            }
            for d in detections
        ],
        "total": len(detections)
    }

# ============ WebSocket Inference ============
@app.websocket("/ws/infer")
async def websocket_inference(websocket: WebSocket, db: Session = Depends(get_db)):
    """WebSocket endpoint for real-time inference"""
    await websocket.accept()
    event_service = EventService(db)
    
    try:
        while True:
            # Receive frame data
            data = await websocket.receive_text()
            request = json.loads(data)
            
            frame_b64 = request.get("frameB64") or request.get("FrameB64")
            
            if frame_b64:
                # Call model worker for inference
                inference_result = await model_service.infer(frame_b64)
                
                # Calculate TTC
                ttc = calculate_ttc(inference_result.get("detections", []))
                
                # Log collision warning
                if ttc < 1.5:
                    event = EventCreate(
                        event_type="collision_warning",
                        description=f"TTC: {ttc:.2f}s",
                        severity="high",
                        timestamp=datetime.utcnow()
                    )
                    event_service.create(event)
                
                # Send response
                response = {
                    "detections": inference_result.get("detections", []),
                    "ttc": ttc,
                    "stats": inference_result.get("stats", {})
                }
                
                await websocket.send_json(response)
                
    except WebSocketDisconnect:
        print("WebSocket disconnected")
    except Exception as e:
        print(f"WebSocket error: {e}")
        await websocket.close()

def calculate_ttc(detections):
    """Calculate Time To Collision"""
    cars = [d for d in detections if d.get("className") in ["car", "truck"]]
    if not cars:
        return float('inf')
    
    closest = min(cars, key=lambda x: x.get("distanceMeters", float('inf')))
    distance = closest.get("distanceMeters", float('inf'))
    
    if distance == float('inf'):
        return float('inf')
    
    # Assume constant velocity 60 km/h = 16.67 m/s
    velocity = 16.67
    return distance / velocity

# ============ Health Check ============
@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "ADAS Backend",
        "version": "2.0.0",
        "timestamp": datetime.utcnow().isoformat()
    }

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "ADAS Backend API",
        "version": "2.0.0",
        "endpoints": {
            "cameras": "/api/cameras/list",
            "models": "/api/models/list",
            "trips": "/api/trips/list",
            "events": "/api/events/list",
            "drivers": "/api/drivers/list",
            "analytics": "/api/analytics/dashboard",
            "websocket": "ws://localhost:8000/ws/infer"
        }
    }

# ============ Data Collection Module ============
from dataset_api import register_dataset_routes
register_dataset_routes(app)

if __name__ == "__main__":
    print("=" * 50)
    print("🚀 ADAS Backend Server Starting...")
    print("=" * 50)
    print(f"📊 API Docs: http://localhost:8000/docs")
    print(f"🔌 WebSocket: ws://localhost:8000/ws/infer")
    print(f"💚 Health: http://localhost:8000/health")
    print(f"📦 Dataset: http://localhost:8000/api/dataset")
    print("=" * 50)
    
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
