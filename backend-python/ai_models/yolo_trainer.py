"""
YOLO Trainer
Training YOLO models với custom dataset
"""
import torch
from ultralytics import YOLO
from pathlib import Path
import yaml
import shutil
from typing import Dict, Any, Optional, Callable
from datetime import datetime
from sqlalchemy.orm import Session


class YOLOTrainer:
    """
    YOLO Model Trainer
    """
    
    def __init__(self, model_name: str, base_model: str = "yolo11n.pt"):
        """
        Args:
            model_name: Tên model cần train
            base_model: Base model (yolo11n, yolo11s, yolo11m, yolo11l, yolo11x)
        """
        self.model_name = model_name
        self.base_model = base_model
        
        # Directories
        self.dataset_dir = Path("dataset/training")
        self.models_dir = Path("models/trained")
        self.models_dir.mkdir(parents=True, exist_ok=True)
        
        # Load base model
        self.model = YOLO(base_model)
        
        print(f"YOLO Trainer initialized")
        print(f"Base model: {base_model}")
        print(f"Target model: {model_name}")
    
    def prepare_dataset(self, dataset_id: Optional[int], db: Session) -> str:
        """
        Chuẩn bị dataset cho training
        
        Tạo cấu trúc:
        dataset/training/
          ├── images/
          │   ├── train/
          │   └── val/
          ├── labels/
          │   ├── train/
          │   └── val/
          └── data.yaml
        
        Returns:
            Path to data.yaml
        """
        from models import VideoDataset, Label
        import json
        
        # Tạo directories
        train_images = self.dataset_dir / "images" / "train"
        val_images = self.dataset_dir / "images" / "val"
        train_labels = self.dataset_dir / "labels" / "train"
        val_labels = self.dataset_dir / "labels" / "val"
        
        for d in [train_images, val_images, train_labels, val_labels]:
            d.mkdir(parents=True, exist_ok=True)
        
        # Query dataset
        if dataset_id:
            videos = db.query(VideoDataset).filter(
                VideoDataset.id == dataset_id,
                VideoDataset.status == "labeled"
            ).all()
        else:
            videos = db.query(VideoDataset).filter(
                VideoDataset.status == "labeled"
            ).all()
        
        if not videos:
            raise ValueError("Không tìm thấy dataset đã label")
        
        # Copy images và labels
        frame_count = 0
        
        for video in videos:
            labels = db.query(Label).filter(Label.video_id == video.id).all()
            
            for label in labels:
                # Đường dẫn frame
                frame_filename = f"frame_{video.id}_{label.frame_number}.jpg"
                frame_path = Path("dataset/raw/frames") / frame_filename
                
                if not frame_path.exists():
                    continue
                
                # Split train/val (80/20)
                is_train = frame_count % 5 != 0
                
                if is_train:
                    img_dest = train_images / frame_filename
                    label_dest = train_labels / f"{frame_filename.replace('.jpg', '.txt')}"
                else:
                    img_dest = val_images / frame_filename
                    label_dest = val_labels / f"{frame_filename.replace('.jpg', '.txt')}"
                
                # Copy image
                shutil.copy(frame_path, img_dest)
                
                # Write label (YOLO format)
                label_data = json.loads(label.label_data)
                
                with open(label_dest, 'w') as f:
                    for obj in label_data:
                        # YOLO format: class_id x_center y_center width height
                        class_id = obj['class_id']
                        bbox = obj['bbox']
                        f.write(f"{class_id} {bbox[0]} {bbox[1]} {bbox[2]} {bbox[3]}\n")
                
                frame_count += 1
        
        print(f"Prepared {frame_count} frames for training")
        
        # Tạo data.yaml
        data_yaml = self.dataset_dir / "data.yaml"
        
        yaml_content = {
            'path': str(self.dataset_dir.absolute()),
            'train': 'images/train',
            'val': 'images/val',
            'nc': 4,  # Number of classes
            'names': ['car', 'motorcycle', 'bus', 'truck']
        }
        
        with open(data_yaml, 'w') as f:
            yaml.dump(yaml_content, f)
        
        return str(data_yaml)
    
    def train(
        self,
        epochs: int = 50,
        batch_size: int = 16,
        img_size: int = 640,
        callback: Optional[Callable] = None,
        data_yaml: Optional[str] = None,
        patience: int = 50
    ) -> tuple:
        """
        Train model
        
        Args:
            epochs: Số epochs
            batch_size: Batch size
            img_size: Image size
            callback: Callback function(epoch, total_epochs, metrics)
            data_yaml: Path to data.yaml (optional, defaults to dataset/training/data.yaml)
            patience: Early stopping patience
        
        Returns:
            (model_path, metrics)
        """
        if data_yaml is None:
            data_yaml = str(self.dataset_dir / "data.yaml")
        
        data_yaml_path = Path(data_yaml)
        if not data_yaml_path.exists():
            raise ValueError(f"Dataset config not found: {data_yaml_path}")
        
        # Training với callback
        class TrainingCallback:
            def __init__(self, callback_fn):
                self.callback_fn = callback_fn
            
            def on_train_epoch_end(self, trainer):
                if self.callback_fn:
                    epoch = trainer.epoch + 1
                    metrics = {
                        "loss": float(trainer.loss.item()) if hasattr(trainer, 'loss') else 0,
                        "map50": float(trainer.metrics.get('metrics/mAP50(B)', 0)),
                        "map50_95": float(trainer.metrics.get('metrics/mAP50-95(B)', 0)),
                    }
                    self.callback_fn(epoch, trainer.epochs, metrics)
        
        # Train
        results = self.model.train(
            data=str(data_yaml_path),
            epochs=epochs,
            batch=batch_size,
            imgsz=img_size,
            project=str(self.models_dir),
            name=self.model_name,
            exist_ok=True,
            verbose=True,
            patience=patience,
            callbacks=[TrainingCallback(callback)] if callback else None
        )
        
        # Lưu best model
        best_model_path = self.models_dir / self.model_name / "weights" / "best.pt"
        
        # Copy to models directory
        final_model_path = self.models_dir / f"{self.model_name}.pt"
        shutil.copy(best_model_path, final_model_path)
        
        # Extract metrics
        metrics = {
            "map50": float(results.results_dict.get('metrics/mAP50(B)', 0)),
            "map50_95": float(results.results_dict.get('metrics/mAP50-95(B)', 0)),
            "precision": float(results.results_dict.get('metrics/precision(B)', 0)),
            "recall": float(results.results_dict.get('metrics/recall(B)', 0)),
        }
        
        print(f"Training completed!")
        print(f"Model saved: {final_model_path}")
        print(f"Metrics: {metrics}")
        
        return str(final_model_path), metrics
