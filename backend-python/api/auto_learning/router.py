"""
Auto Learning API Router
Incremental training với auto-collected data
"""
from fastapi import APIRouter, Depends, HTTPException, BackgroundTasks
from sqlalchemy.orm import Session
from typing import Dict, Any, Optional
from datetime import datetime
from pathlib import Path
import json

from database import get_db

router = APIRouter(prefix="/api/auto-learning", tags=["Auto Learning"])

# Global training status
INCREMENTAL_TRAINING_STATUS = {}


@router.get("/stats")
async def get_collection_stats() -> Dict[str, Any]:
    """
    📊 Lấy thống kê auto-collection
    
    Returns:
        {
            "total_images": int,
            "total_labels": int,
            "unique_classes": int,
            "class_distribution": {...},
            "collection_stats": {...},
            "ready_for_training": bool
        }
    """
    try:
        collection_dir = Path("dataset/auto_collected")
        
        if not collection_dir.exists():
            return {
                "total_images": 0,
                "total_labels": 0,
                "unique_classes": 0,
                "class_distribution": {},
                "ready_for_training": False
            }
        
        # Count images and labels
        images_dir = collection_dir / "images"
        labels_dir = collection_dir / "labels"
        
        total_images = len(list(images_dir.glob("*.jpg"))) if images_dir.exists() else 0
        total_labels = len(list(labels_dir.glob("*.txt"))) if labels_dir.exists() else 0
        
        # Load object memory
        memory_file = collection_dir / "object_memory.json"
        object_memory = {}
        if memory_file.exists():
            with open(memory_file, 'r') as f:
                object_memory = json.load(f)
        
        # Class distribution
        class_distribution = {
            cls: data.get('count', 0) 
            for cls, data in object_memory.items()
        }
        
        return {
            "total_images": total_images,
            "total_labels": total_labels,
            "unique_classes": len(object_memory),
            "class_distribution": class_distribution,
            "object_memory": object_memory,
            "ready_for_training": total_images >= 10  # Need at least 10 samples
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting stats: {str(e)}")


@router.post("/train-incremental")
async def start_incremental_training(
    background_tasks: BackgroundTasks,
    base_model: str = "yolov8n.pt",
    epochs: int = 20,
    batch_size: int = 8,
    db: Session = Depends(get_db)
) -> Dict[str, Any]:
    """
    🚀 Bắt đầu incremental training với auto-collected data
    
    Trains model on newly collected data to learn new objects
    
    Args:
        base_model: Base model to fine-tune (default: yolov8n.pt)
        epochs: Training epochs (default: 20 for incremental)
        batch_size: Batch size (default: 8)
    
    Returns:
        {
            "training_id": str,
            "status": "started",
            "message": str,
            "total_samples": int
        }
    """
    try:
        # Check if we have enough data
        stats = await get_collection_stats()
        
        if not stats["ready_for_training"]:
            raise HTTPException(
                status_code=400, 
                detail=f"Not enough training data. Need at least 10 samples, have {stats['total_images']}"
            )
        
        # Create training ID
        training_id = f"incremental_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Initialize status
        INCREMENTAL_TRAINING_STATUS[training_id] = {
            "status": "preparing",
            "progress": 0,
            "current_epoch": 0,
            "total_epochs": epochs,
            "message": "Đang chuẩn bị incremental training...",
            "started_at": datetime.now().isoformat(),
            "total_samples": stats["total_images"],
            "new_classes": stats["unique_classes"]
        }
        
        # Run training in background
        background_tasks.add_task(
            run_incremental_training,
            training_id=training_id,
            base_model=base_model,
            epochs=epochs,
            batch_size=batch_size,
            db=db
        )
        
        return {
            "training_id": training_id,
            "status": "started",
            "message": f"Started incremental training with {stats['total_images']} samples",
            "total_samples": stats["total_images"],
            "new_classes": stats["unique_classes"]
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting training: {str(e)}")


@router.get("/training-status/{training_id}")
async def get_training_status(training_id: str) -> Dict[str, Any]:
    """
    📈 Lấy trạng thái incremental training
    
    Args:
        training_id: ID của training job
    
    Returns:
        Training status with progress
    """
    if training_id not in INCREMENTAL_TRAINING_STATUS:
        raise HTTPException(status_code=404, detail="Training job not found")
    
    return INCREMENTAL_TRAINING_STATUS[training_id]


async def run_incremental_training(
    training_id: str,
    base_model: str,
    epochs: int,
    batch_size: int,
    db: Session
):
    """
    Background task: Chạy incremental training
    """
    try:
        from ai_models.yolo_trainer import YOLOTrainer
        from models import AIModel
        
        # Update status
        INCREMENTAL_TRAINING_STATUS[training_id]["status"] = "preparing"
        INCREMENTAL_TRAINING_STATUS[training_id]["message"] = "Preparing auto-collected dataset..."
        
        # 1. Prepare dataset from auto-collected data
        collection_dir = Path("dataset/auto_collected")
        
        # Create YOLO dataset structure
        dataset_dir = Path("dataset/incremental_training")
        dataset_dir.mkdir(parents=True, exist_ok=True)
        
        # Create train/val split (80/20)
        import shutil
        from sklearn.model_selection import train_test_split
        
        images = list((collection_dir / "images").glob("*.jpg"))
        train_imgs, val_imgs = train_test_split(images, test_size=0.2, random_state=42)
        
        # Copy to train/val directories
        for split, img_list in [("train", train_imgs), ("val", val_imgs)]:
            (dataset_dir / split / "images").mkdir(parents=True, exist_ok=True)
            (dataset_dir / split / "labels").mkdir(parents=True, exist_ok=True)
            
            for img_path in img_list:
                # Copy image
                shutil.copy(img_path, dataset_dir / split / "images" / img_path.name)
                
                # Copy label
                label_path = collection_dir / "labels" / f"{img_path.stem}.txt"
                if label_path.exists():
                    shutil.copy(label_path, dataset_dir / split / "labels" / f"{img_path.stem}.txt")
        
        # 2. Create data.yaml
        data_yaml = dataset_dir / "data.yaml"
        with open(data_yaml, 'w') as f:
            f.write(f"""# Auto-collected ADAS Dataset
path: {dataset_dir.absolute()}
train: train/images
val: val/images

# Classes (COCO 80 classes)
nc: 80
names: ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
        'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
        'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
        'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
        'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
        'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
        'scissors', 'teddy bear', 'hair drier', 'toothbrush']
""")
        
        # 3. Start incremental training
        INCREMENTAL_TRAINING_STATUS[training_id]["status"] = "training"
        INCREMENTAL_TRAINING_STATUS[training_id]["message"] = "Training model with new data..."
        
        trainer = YOLOTrainer(
            model_name=f"incremental_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            base_model=base_model
        )
        
        def update_progress(epoch, total_epochs, metrics):
            """Callback to update progress"""
            INCREMENTAL_TRAINING_STATUS[training_id]["current_epoch"] = epoch
            INCREMENTAL_TRAINING_STATUS[training_id]["progress"] = (epoch / total_epochs) * 100
            INCREMENTAL_TRAINING_STATUS[training_id]["metrics"] = metrics
        
        # Train with incremental learning settings
        model_path, metrics = trainer.train(
            data_yaml=str(data_yaml),
            epochs=epochs,
            batch_size=batch_size,
            img_size=640,
            callback=update_progress,
            patience=10  # Early stopping
        )
        
        # 4. Save model to DB
        INCREMENTAL_TRAINING_STATUS[training_id]["status"] = "completed"
        INCREMENTAL_TRAINING_STATUS[training_id]["message"] = "Incremental training completed!"
        INCREMENTAL_TRAINING_STATUS[training_id]["model_path"] = model_path
        INCREMENTAL_TRAINING_STATUS[training_id]["metrics"] = metrics
        
        ai_model = AIModel(
            name=trainer.model_name,
            model_type="yolov8_incremental",
            version=f"v{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            file_path=model_path,
            accuracy=metrics.get("metrics/mAP50(B)", 0.0),
            config={
                "base_model": base_model,
                "epochs": epochs,
                "batch_size": batch_size,
                "training_type": "incremental",
                "metrics": metrics
            },
            is_active=False,
            created_at=datetime.now()
        )
        
        db.add(ai_model)
        db.commit()
        
        print(f"✅ Incremental training completed: {model_path}")
        
    except Exception as e:
        INCREMENTAL_TRAINING_STATUS[training_id]["status"] = "failed"
        INCREMENTAL_TRAINING_STATUS[training_id]["message"] = f"Training failed: {str(e)}"
        INCREMENTAL_TRAINING_STATUS[training_id]["error"] = str(e)
        print(f"❌ Incremental training failed: {e}")


@router.post("/clear-collection")
async def clear_collection() -> Dict[str, Any]:
    """
    🗑️ Clear auto-collected data (after successful training)
    
    Returns:
        Success message
    """
    try:
        import shutil
        
        collection_dir = Path("dataset/auto_collected")
        
        if collection_dir.exists():
            shutil.rmtree(collection_dir)
            collection_dir.mkdir(parents=True, exist_ok=True)
            (collection_dir / "images").mkdir(exist_ok=True)
            (collection_dir / "labels").mkdir(exist_ok=True)
        
        return {
            "status": "success",
            "message": "Auto-collected data cleared successfully"
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error clearing data: {str(e)}")
