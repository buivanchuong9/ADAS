"""
Training API Router
Chạy training YOLO với dataset đã label
"""
from fastapi import APIRouter, Depends, HTTPException, BackgroundTasks
from sqlalchemy.orm import Session
from typing import Dict, Any, Optional
from datetime import datetime
import os
from pathlib import Path

from database import get_db
from schemas import TrainingRequest, TrainingResponse

router = APIRouter(prefix="/api/training", tags=["Training"])

TRAINING_STATUS = {}  # In-memory storage cho training status


@router.post("/start", response_model=TrainingResponse)
async def start_training(
    background_tasks: BackgroundTasks,
    request: TrainingRequest,
    db: Session = Depends(get_db)
) -> Dict[str, Any]:
    """
    🔥 Bắt đầu training YOLO model
    
    Args:
        request: {
            "model_name": str,  # Tên model mới
            "base_model": str,  # yolov8n, yolov8s, yolov8m...
            "epochs": int,      # Số epochs (default: 50)
            "batch_size": int,  # Batch size (default: 16)
            "img_size": int,    # Image size (default: 640)
            "dataset_id": int   # ID của dataset (optional)
        }
    
    Returns:
        {
            "training_id": str,
            "status": "started",
            "message": str,
            "model_name": str
        }
    """
    
    # Kiểm tra dataset
    from models import VideoDataset
    
    if request.dataset_id:
        dataset = db.query(VideoDataset).filter(VideoDataset.id == request.dataset_id).first()
        if not dataset:
            raise HTTPException(status_code=404, detail="Dataset không tồn tại")
        
        if dataset.status != "labeled":
            raise HTTPException(status_code=400, detail="Dataset chưa được label")
    
    # Tạo training ID
    training_id = f"train_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    # Khởi tạo status
    TRAINING_STATUS[training_id] = {
        "status": "preparing",
        "progress": 0,
        "current_epoch": 0,
        "total_epochs": request.epochs,
        "message": "Đang chuẩn bị dataset...",
        "started_at": datetime.now().isoformat()
    }
    
    # Chạy training trong background
    background_tasks.add_task(
        run_training_task,
        training_id=training_id,
        request=request,
        db=db
    )
    
    return {
        "training_id": training_id,
        "status": "started",
        "message": f"Đã bắt đầu training model {request.model_name}",
        "model_name": request.model_name,
        "base_model": request.base_model,
        "epochs": request.epochs
    }


async def run_training_task(training_id: str, request: TrainingRequest, db: Session):
    """
    Background task: Chạy training YOLO
    """
    try:
        from ai_models.yolo_trainer import YOLOTrainer
        from models import AIModel
        
        # Update status
        TRAINING_STATUS[training_id]["status"] = "preparing"
        TRAINING_STATUS[training_id]["message"] = "Đang chuẩn bị dataset..."
        
        # 1. Chuẩn bị dataset
        trainer = YOLOTrainer(
            model_name=request.model_name,
            base_model=request.base_model
        )
        
        dataset_path = trainer.prepare_dataset(
            dataset_id=request.dataset_id,
            db=db
        )
        
        # 2. Bắt đầu training
        TRAINING_STATUS[training_id]["status"] = "training"
        TRAINING_STATUS[training_id]["message"] = "Đang training model..."
        
        def update_progress(epoch, total_epochs, metrics):
            """Callback để update progress"""
            TRAINING_STATUS[training_id]["current_epoch"] = epoch
            TRAINING_STATUS[training_id]["progress"] = (epoch / total_epochs) * 100
            TRAINING_STATUS[training_id]["metrics"] = metrics
        
        # Train model
        model_path, metrics = trainer.train(
            epochs=request.epochs,
            batch_size=request.batch_size,
            img_size=request.img_size,
            callback=update_progress
        )
        
        # 3. Lưu model vào DB
        TRAINING_STATUS[training_id]["status"] = "saving"
        TRAINING_STATUS[training_id]["message"] = "Đang lưu model..."
        
        ai_model = AIModel(
            name=request.model_name,
            model_type="yolov8",
            version=f"v{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            file_path=model_path,
            accuracy=metrics.get("map50", 0.0),
            config={
                "base_model": request.base_model,
                "epochs": request.epochs,
                "batch_size": request.batch_size,
                "img_size": request.img_size,
                "dataset_id": request.dataset_id,
                "metrics": metrics
            },
            is_active=False,  # Chưa active, cần test trước
            created_at=datetime.now()
        )
        db.add(ai_model)
        db.commit()
        
        # 4. Hoàn thành
        TRAINING_STATUS[training_id]["status"] = "completed"
        TRAINING_STATUS[training_id]["progress"] = 100
        TRAINING_STATUS[training_id]["message"] = "Training hoàn thành!"
        TRAINING_STATUS[training_id]["model_id"] = ai_model.id
        TRAINING_STATUS[training_id]["model_path"] = model_path
        TRAINING_STATUS[training_id]["metrics"] = metrics
        TRAINING_STATUS[training_id]["completed_at"] = datetime.now().isoformat()
        
    except Exception as e:
        TRAINING_STATUS[training_id]["status"] = "failed"
        TRAINING_STATUS[training_id]["message"] = f"Lỗi: {str(e)}"
        TRAINING_STATUS[training_id]["error"] = str(e)


@router.get("/status/{training_id}")
async def get_training_status(training_id: str):
    """
    Kiểm tra trạng thái training
    """
    if training_id not in TRAINING_STATUS:
        raise HTTPException(status_code=404, detail="Training ID không tồn tại")
    
    return TRAINING_STATUS[training_id]


@router.get("/list")
async def list_trainings():
    """
    Danh sách tất cả training sessions
    """
    return {
        "trainings": [
            {"training_id": tid, **status}
            for tid, status in TRAINING_STATUS.items()
        ]
    }


@router.post("/activate/{model_id}")
async def activate_model(model_id: int, db: Session = Depends(get_db)):
    """
    Kích hoạt model để sử dụng cho inference
    """
    from models import AIModel
    
    # Deactivate tất cả models cũ
    db.query(AIModel).filter(AIModel.model_type == "yolov8").update({"is_active": False})
    
    # Activate model mới
    model = db.query(AIModel).filter(AIModel.id == model_id).first()
    if not model:
        raise HTTPException(status_code=404, detail="Model không tồn tại")
    
    model.is_active = True
    db.commit()
    
    return {
        "success": True,
        "message": f"Model {model.name} đã được kích hoạt",
        "model_id": model_id
    }
