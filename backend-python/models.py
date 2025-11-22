# Database Models for ADAS System
# SQLAlchemy models for SQL Server

from sqlalchemy import Column, Integer, String, Float, Boolean, DateTime, ForeignKey, Text
from sqlalchemy.orm import relationship
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime

Base = declarative_base()

class Camera(Base):
    __tablename__ = "cameras"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(100), nullable=False)
    type = Column(String(50), nullable=False)  # webcam, smartphone, ip-camera, stream
    url = Column(String(500))
    status = Column(String(50), default="disconnected")  # ready, disconnected, active
    resolution = Column(String(100))
    frame_rate = Column(Integer)
    instructions = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    last_active_at = Column(DateTime)
    
    # Relationships
    detections = relationship("Detection", back_populates="camera")
    events = relationship("Event", back_populates="camera")
    trips = relationship("Trip", back_populates="camera")

class Driver(Base):
    __tablename__ = "drivers"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(100), nullable=False)
    license_number = Column(String(50))
    email = Column(String(100))
    phone = Column(String(20))
    date_of_birth = Column(DateTime)
    address = Column(String(200))
    total_trips = Column(Integer, default=0)
    total_distance_km = Column(Float, default=0.0)
    safety_score = Column(Integer, default=100)
    status = Column(String(50), default="active")  # active, inactive, suspended
    created_at = Column(DateTime, default=datetime.utcnow)
    last_active_at = Column(DateTime)
    
    # Relationships
    trips = relationship("Trip", back_populates="driver")
    events = relationship("Event", back_populates="driver")
    driver_statuses = relationship("DriverStatus", back_populates="driver")

class Trip(Base):
    __tablename__ = "trips"
    
    id = Column(Integer, primary_key=True, index=True)
    start_time = Column(DateTime, nullable=False, default=datetime.utcnow)
    end_time = Column(DateTime)
    distance_km = Column(Float)
    duration_minutes = Column(Float)
    start_location = Column(String(200))
    end_location = Column(String(200))
    average_speed = Column(Integer)
    max_speed = Column(Integer)
    status = Column(String(50), default="active")  # active, completed, cancelled
    total_events = Column(Integer, default=0)
    critical_events = Column(Integer, default=0)
    route_data = Column(Text)  # JSON
    
    driver_id = Column(Integer, ForeignKey("drivers.id"))
    camera_id = Column(Integer, ForeignKey("cameras.id"))
    
    # Relationships
    driver = relationship("Driver", back_populates="trips")
    camera = relationship("Camera", back_populates="trips")
    detections = relationship("Detection", back_populates="trip")
    events = relationship("Event", back_populates="trip")
    analytics = relationship("Analytics", back_populates="trip")

class Event(Base):
    __tablename__ = "events"
    
    id = Column(Integer, primary_key=True, index=True)
    event_type = Column(String(100), nullable=False, index=True)
    description = Column(String(500))
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow, index=True)
    severity = Column(String(50))  # low, medium, high, critical
    location = Column(String(200))
    event_metadata = Column("metadata", Text)  # JSON - renamed to avoid SQLAlchemy conflict
    
    trip_id = Column(Integer, ForeignKey("trips.id"))
    camera_id = Column(Integer, ForeignKey("cameras.id"))
    driver_id = Column(Integer, ForeignKey("drivers.id"))
    
    # Relationships
    trip = relationship("Trip", back_populates="events")
    camera = relationship("Camera", back_populates="events")
    driver = relationship("Driver", back_populates="events")

class Detection(Base):
    __tablename__ = "detections"
    
    id = Column(Integer, primary_key=True, index=True)
    class_name = Column(String(100), nullable=False, index=True)
    confidence = Column(Float, nullable=False)
    bounding_box = Column(Text, nullable=False)  # JSON [x1,y1,x2,y2]
    distance_meters = Column(Float)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow, index=True)
    
    trip_id = Column(Integer, ForeignKey("trips.id"))
    camera_id = Column(Integer, ForeignKey("cameras.id"))
    
    # Relationships
    trip = relationship("Trip", back_populates="detections")
    camera = relationship("Camera", back_populates="detections")

class DriverStatus(Base):
    __tablename__ = "driver_statuses"
    
    id = Column(Integer, primary_key=True, index=True)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow, index=True)
    fatigue_level = Column(String(50))  # alert, drowsy, very-drowsy, asleep
    distraction_level = Column(String(50))  # focused, distracted, very-distracted
    eye_closure_duration = Column(Float)
    head_pose_yaw = Column(Integer)
    head_pose_pitch = Column(Integer)
    is_yawning = Column(Boolean)
    is_using_phone = Column(Boolean)
    alert_count = Column(Integer, default=0)
    
    driver_id = Column(Integer, ForeignKey("drivers.id"))
    trip_id = Column(Integer, ForeignKey("trips.id"))
    
    # Relationships
    driver = relationship("Driver", back_populates="driver_statuses")

class Analytics(Base):
    __tablename__ = "analytics"
    
    id = Column(Integer, primary_key=True, index=True)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow, index=True)
    metric_type = Column(String(100), nullable=False, index=True)
    value = Column(Float, nullable=False)
    unit = Column(String(50))
    category = Column(String(100))
    analytics_metadata = Column("metadata", Text)  # JSON - renamed to avoid SQLAlchemy conflict
    
    trip_id = Column(Integer, ForeignKey("trips.id"))
    driver_id = Column(Integer, ForeignKey("drivers.id"))
    
    # Relationships
    trip = relationship("Trip", back_populates="analytics")

class AIModel(Base):
    __tablename__ = "ai_models"
    
    id = Column(Integer, primary_key=True, index=True)
    model_id = Column(String(100), unique=True, nullable=False, index=True)
    name = Column(String(200), nullable=False)
    size = Column(String(50))
    downloaded = Column(Boolean, default=False)
    accuracy = Column(Float)
    url = Column(String(500))
    file_path = Column(String(1000))
    version = Column(String(50))
    description = Column(String(500))
    created_at = Column(DateTime, default=datetime.utcnow)
    last_used_at = Column(DateTime)
    is_active = Column(Boolean, default=False, index=True)
