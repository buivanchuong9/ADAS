# Seed initial data for ADAS database

from sqlalchemy.orm import Session
from database import SessionLocal, engine
from models import Base, Camera, AIModel, Driver
from datetime import datetime

def seed_cameras(db: Session):
    """Seed initial cameras"""
    cameras = [
        Camera(
            name="Web Camera (máy tính)",
            type="webcam",
            status="ready",
            resolution="1280x720",
            frame_rate=30,
            instructions="Sử dụng webcam tích hợp hoặc USB camera của máy tính",
            created_at=datetime.utcnow()
        ),
        Camera(
            name="Smartphone (RTMP Stream)",
            type="smartphone",
            status="disconnected",
            url="rtmp://localhost:1935/live",
            instructions="1. Cài app IP Camera trên điện thoại\n2. Nhập URL: rtmp://[YOUR_PC_IP]:1935/live\n3. Bắt đầu phát",
            created_at=datetime.utcnow()
        ),
        Camera(
            name="Smartphone (WebRTC)",
            type="smartphone",
            status="disconnected",
            url="ws://localhost:8000/ws/camera",
            instructions="1. Cài app WebRTC cho điện thoại\n2. Kết nối tới: ws://[YOUR_PC_IP]:8000/ws/camera",
            created_at=datetime.utcnow()
        ),
        Camera(
            name="IP Camera",
            type="ip-camera",
            status="disconnected",
            url="rtsp://192.168.1.100:554/stream",
            instructions="1. Nhập URL RTSP/RTMP của camera IP\n2. Ví dụ: rtsp://192.168.1.100:554/stream",
            created_at=datetime.utcnow()
        )
    ]
    
    db.bulk_save_objects(cameras)
    db.commit()
    print("✅ Seeded cameras")

def seed_ai_models(db: Session):
    """Seed AI models"""
    models = [
        AIModel(
            model_id="yolov8n",
            name="YOLOv8 Nano",
            size="6.3 MB",
            downloaded=True,
            accuracy=80.4,
            url="https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt",
            version="8.0",
            description="Model nhẹ nhất, phù hợp cho CPU",
            is_active=True,
            created_at=datetime.utcnow()
        ),
        AIModel(
            model_id="yolov8s",
            name="YOLOv8 Small",
            size="22.5 MB",
            downloaded=False,
            accuracy=86.6,
            url="https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt",
            version="8.0",
            description="Cân bằng giữa tốc độ và độ chính xác",
            is_active=False,
            created_at=datetime.utcnow()
        ),
        AIModel(
            model_id="yolov8m",
            name="YOLOv8 Medium",
            size="49.2 MB",
            downloaded=False,
            accuracy=88.3,
            url="https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.pt",
            version="8.0",
            description="Độ chính xác cao hơn, yêu cầu GPU",
            is_active=False,
            created_at=datetime.utcnow()
        ),
        AIModel(
            model_id="yolov5s",
            name="YOLOv5 Small",
            size="27.6 MB",
            downloaded=False,
            accuracy=85.2,
            url="https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt",
            version="7.0",
            description="YOLOv5 - phiên bản ổn định",
            is_active=False,
            created_at=datetime.utcnow()
        )
    ]
    
    db.bulk_save_objects(models)
    db.commit()
    print("✅ Seeded AI models")

def seed_demo_driver(db: Session):
    """Seed demo driver"""
    driver = Driver(
        name="Demo Driver",
        email="demo@adas.com",
        phone="0123456789",
        license_number="DL-001",
        safety_score=85,
        status="active",
        total_trips=0,
        total_distance_km=0,
        created_at=datetime.utcnow()
    )
    
    db.add(driver)
    db.commit()
    print("✅ Seeded demo driver")

def seed_all():
    """Seed all initial data"""
    print("=" * 50)
    print("🌱 Seeding database...")
    print("=" * 50)
    
    # Create tables
    Base.metadata.create_all(bind=engine)
    print("✅ Created tables")
    
    # Create session
    db = SessionLocal()
    
    try:
        # Check if already seeded
        if db.query(Camera).count() > 0:
            print("⚠️  Database already seeded. Skipping...")
            return
        
        # Seed data
        seed_cameras(db)
        seed_ai_models(db)
        seed_demo_driver(db)
        
        print("=" * 50)
        print("✅ Database seeded successfully!")
        print("=" * 50)
        
    except Exception as e:
        print(f"❌ Seeding failed: {e}")
        db.rollback()
    finally:
        db.close()

if __name__ == "__main__":
    seed_all()
