# Business logic services for ADAS Backend

from sqlalchemy.orm import Session
from sqlalchemy import func
from typing import List, Optional
from datetime import datetime
import httpx
import os

from models import Camera, Driver, Trip, Event, Detection, DriverStatus, Analytics, AIModel
from schemas import (
    CameraCreate, DriverCreate, TripCreate, EventCreate,
    DetectionCreate, AnalyticsCreate
)

# ============ Camera Service ============
class CameraService:
    def __init__(self, db: Session):
        self.db = db
    
    def get_all(self) -> List[Camera]:
        return self.db.query(Camera).all()
    
    def get_by_id(self, camera_id: int) -> Optional[Camera]:
        return self.db.query(Camera).filter(Camera.id == camera_id).first()
    
    def create(self, camera: CameraCreate) -> Camera:
        db_camera = Camera(**camera.dict())
        db_camera.created_at = datetime.utcnow()
        self.db.add(db_camera)
        self.db.commit()
        self.db.refresh(db_camera)
        return db_camera
    
    def update(self, camera_id: int, camera: CameraCreate) -> Optional[Camera]:
        db_camera = self.get_by_id(camera_id)
        if not db_camera:
            return None
        
        for key, value in camera.dict().items():
            setattr(db_camera, key, value)
        
        self.db.commit()
        self.db.refresh(db_camera)
        return db_camera
    
    def delete(self, camera_id: int) -> bool:
        db_camera = self.get_by_id(camera_id)
        if not db_camera:
            return False
        
        self.db.delete(db_camera)
        self.db.commit()
        return True
    
    def update_status(self, camera_id: int, status: str) -> Optional[Camera]:
        db_camera = self.get_by_id(camera_id)
        if not db_camera:
            return None
        
        db_camera.status = status
        db_camera.last_active_at = datetime.utcnow()
        self.db.commit()
        self.db.refresh(db_camera)
        return db_camera

# ============ Driver Service ============
class DriverService:
    def __init__(self, db: Session):
        self.db = db
    
    def get_all(self) -> List[Driver]:
        return self.db.query(Driver).all()
    
    def get_by_id(self, driver_id: int) -> Optional[Driver]:
        return self.db.query(Driver).filter(Driver.id == driver_id).first()
    
    def create(self, driver: DriverCreate) -> Driver:
        db_driver = Driver(**driver.dict())
        db_driver.created_at = datetime.utcnow()
        db_driver.status = "active"
        db_driver.safety_score = 100
        self.db.add(db_driver)
        self.db.commit()
        self.db.refresh(db_driver)
        return db_driver
    
    def update_safety_score(self, driver_id: int, score: int) -> Optional[Driver]:
        db_driver = self.get_by_id(driver_id)
        if not db_driver:
            return None
        
        db_driver.safety_score = score
        db_driver.last_active_at = datetime.utcnow()
        self.db.commit()
        self.db.refresh(db_driver)
        return db_driver

# ============ Trip Service ============
class TripService:
    def __init__(self, db: Session):
        self.db = db
    
    def get_all(self) -> List[Trip]:
        return self.db.query(Trip).order_by(Trip.start_time.desc()).all()
    
    def get_by_id(self, trip_id: int) -> Optional[Trip]:
        return self.db.query(Trip).filter(Trip.id == trip_id).first()
    
    def create(self, trip: TripCreate) -> Trip:
        db_trip = Trip(**trip.dict())
        db_trip.start_time = datetime.utcnow()
        db_trip.status = "active"
        self.db.add(db_trip)
        self.db.commit()
        self.db.refresh(db_trip)
        return db_trip
    
    def end_trip(self, trip_id: int) -> Optional[Trip]:
        db_trip = self.get_by_id(trip_id)
        if not db_trip:
            return None
        
        db_trip.end_time = datetime.utcnow()
        db_trip.status = "completed"
        
        if db_trip.start_time:
            duration = (db_trip.end_time - db_trip.start_time).total_seconds() / 60
            db_trip.duration_minutes = duration
        
        self.db.commit()
        self.db.refresh(db_trip)
        return db_trip
    
    def get_active_trip(self) -> Optional[Trip]:
        return self.db.query(Trip).filter(Trip.status == "active").first()

# ============ Event Service ============
class EventService:
    def __init__(self, db: Session):
        self.db = db
    
    def get_all(self) -> List[Event]:
        return self.db.query(Event).order_by(Event.timestamp.desc()).all()
    
    def get_by_id(self, event_id: int) -> Optional[Event]:
        return self.db.query(Event).filter(Event.id == event_id).first()
    
    def create(self, event: EventCreate) -> Event:
        db_event = Event(**event.dict())
        if not db_event.timestamp:
            db_event.timestamp = datetime.utcnow()
        self.db.add(db_event)
        self.db.commit()
        self.db.refresh(db_event)
        return db_event
    
    def get_by_trip(self, trip_id: int) -> List[Event]:
        return self.db.query(Event).filter(Event.trip_id == trip_id).order_by(Event.timestamp.desc()).all()
    
    def get_by_severity(self, severity: str) -> List[Event]:
        return self.db.query(Event).filter(Event.severity == severity).order_by(Event.timestamp.desc()).all()

# ============ Analytics Service ============
class AnalyticsService:
    def __init__(self, db: Session):
        self.db = db
    
    def get_dashboard_stats(self) -> dict:
        """Get dashboard statistics"""
        stats = {
            "totalTrips": self.db.query(Trip).count(),
            "activeTrips": self.db.query(Trip).filter(Trip.status == "active").count(),
            "totalEvents": self.db.query(Event).count(),
            "criticalEvents": self.db.query(Event).filter(Event.severity == "critical").count(),
            "totalDrivers": self.db.query(Driver).count(),
            "totalDetections": self.db.query(Detection).count(),
            "avgSafetyScore": self.db.query(func.avg(Driver.safety_score)).scalar() or 0
        }
        return stats
    
    def create(self, analytics: AnalyticsCreate) -> Analytics:
        db_analytics = Analytics(**analytics.dict())
        db_analytics.timestamp = datetime.utcnow()
        self.db.add(db_analytics)
        self.db.commit()
        self.db.refresh(db_analytics)
        return db_analytics

# ============ Model Service ============
class ModelService:
    def __init__(self):
        self.model_worker_url = os.getenv("MODEL_WORKER_URL", "http://localhost:8000")
    
    async def infer(self, frame_b64: str) -> dict:
        """Call model worker for inference"""
        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    f"{self.model_worker_url}/infer",
                    json={"frame_b64": frame_b64}
                )
                response.raise_for_status()
                return response.json()
        except Exception as e:
            print(f"Model inference error: {e}")
            return {"detections": [], "stats": {}}
