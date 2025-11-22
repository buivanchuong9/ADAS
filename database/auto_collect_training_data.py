"""
Auto Training Data Collector
Automatically collect high-quality training data from live detections
"""

import os
import json
import cv2
import numpy as np
from datetime import datetime, timedelta
from sqlalchemy import create_engine, text
from sqlalchemy.orm import sessionmaker
import hashlib
from typing import List, Dict, Tuple

class TrainingDataCollector:
    """Collect and prepare training data from live system"""
    
    def __init__(self, db_url: str, output_dir: str = "./training_data"):
        self.engine = create_engine(db_url)
        self.Session = sessionmaker(bind=self.engine)
        self.output_dir = output_dir
        
        # Create output directories
        for split in ['train', 'val', 'test']:
            os.makedirs(f"{output_dir}/{split}/images", exist_ok=True)
            os.makedirs(f"{output_dir}/{split}/labels", exist_ok=True)
    
    def get_high_confidence_detections(self, 
                                       min_confidence: float = 0.90,
                                       days_back: int = 7,
                                       limit: int = 1000) -> List[Dict]:
        """Get high-confidence detections for auto-labeling"""
        
        query = text("""
            SELECT 
                d.Id,
                d.ClassName,
                d.Confidence,
                d.BoundingBox,
                d.Timestamp,
                c.Name AS CameraName
            FROM Detections d
            LEFT JOIN Cameras c ON d.CameraId = c.Id
            WHERE d.Confidence >= :min_conf
              AND d.Timestamp > :cutoff_date
            ORDER BY d.Confidence DESC
            LIMIT :limit
        """)
        
        cutoff_date = datetime.utcnow() - timedelta(days=days_back)
        
        with self.Session() as session:
            result = session.execute(query, {
                'min_conf': min_confidence,
                'cutoff_date': cutoff_date,
                'limit': limit
            })
            
            return [dict(row) for row in result]
    
    def calculate_image_quality(self, image: np.ndarray) -> Dict[str, float]:
        """Calculate image quality metrics"""
        
        # Convert to grayscale for blur detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        
        # Laplacian variance (blur detection)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        blur_score = min(100, laplacian_var / 10)  # Normalize to 0-100
        
        # Brightness
        brightness = np.mean(gray)
        brightness_score = 100 - abs(brightness - 127) / 127 * 100
        
        return {
            'blur_score': blur_score,
            'brightness_score': brightness_score,
            'quality_score': (blur_score + brightness_score) / 2
        }
    
    def save_training_image(self, 
                           image: np.ndarray,
                           annotations: List[Dict],
                           class_mapping: Dict[str, int],
                           split: str = 'train') -> Tuple[str, str]:
        """
        Save image and YOLO format annotations
        
        YOLO format: <class_id> <x_center> <y_center> <width> <height>
        All coordinates normalized to 0-1
        """
        
        # Generate unique filename using hash
        image_bytes = cv2.imencode('.jpg', image)[1].tobytes()
        image_hash = hashlib.sha256(image_bytes).hexdigest()[:16]
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{image_hash}"
        
        img_path = f"{self.output_dir}/{split}/images/{filename}.jpg"
        label_path = f"{self.output_dir}/{split}/labels/{filename}.txt"
        
        # Save image
        cv2.imwrite(img_path, image)
        
        # Save annotations in YOLO format
        height, width = image.shape[:2]
        
        with open(label_path, 'w') as f:
            for ann in annotations:
                class_id = class_mapping.get(ann['class_name'], 0)
                
                # Parse bounding box [x, y, w, h]
                bbox = json.loads(ann['bounding_box']) if isinstance(ann['bounding_box'], str) else ann['bounding_box']
                x, y, w, h = bbox
                
                # Convert to YOLO format (normalized center coordinates)
                x_center = (x + w / 2) / width
                y_center = (y + h / 2) / height
                norm_width = w / width
                norm_height = h / height
                
                # Write YOLO annotation
                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {norm_width:.6f} {norm_height:.6f}\n")
        
        return img_path, label_path
    
    def save_to_database(self, 
                        image_path: str,
                        width: int,
                        height: int,
                        quality_metrics: Dict,
                        annotations: List[Dict],
                        split: str):
        """Save training image and annotations to database"""
        
        with self.Session() as session:
            # Insert training image
            insert_image = text("""
                INSERT INTO TrainingImages 
                (ImagePath, ImageHash, Width, Height, FileSize, Format, 
                 SourceType, BlurScore, BrightnessScore, DatasetSplit, IsLabeled, LabeledAt)
                VALUES 
                (:path, :hash, :width, :height, :size, :format,
                 :source, :blur, :brightness, :split, 1, :labeled_at)
                RETURNING Id
            """)
            
            file_size = os.path.getsize(image_path)
            image_hash = hashlib.sha256(open(image_path, 'rb').read()).hexdigest()
            
            result = session.execute(insert_image, {
                'path': image_path,
                'hash': image_hash,
                'width': width,
                'height': height,
                'size': file_size,
                'format': 'jpg',
                'source': 'auto-labeled',
                'blur': quality_metrics['blur_score'],
                'brightness': quality_metrics['brightness_score'],
                'split': split,
                'labeled_at': datetime.utcnow()
            })
            
            image_id = result.scalar()
            
            # Insert annotations
            insert_ann = text("""
                INSERT INTO TrainingAnnotations
                (ImageId, ClassName, BoundingBox, Confidence, AnnotationSource, AnnotatedBy, CreatedAt)
                VALUES
                (:image_id, :class_name, :bbox, :confidence, 'auto', 'system', :created_at)
            """)
            
            for ann in annotations:
                session.execute(insert_ann, {
                    'image_id': image_id,
                    'class_name': ann['class_name'],
                    'bbox': ann['bounding_box'],
                    'confidence': ann['confidence'],
                    'created_at': datetime.utcnow()
                })
            
            session.commit()
    
    def collect_and_save(self, 
                        min_confidence: float = 0.90,
                        min_quality_score: float = 60.0,
                        max_samples: int = 1000):
        """Main collection pipeline"""
        
        print("=" * 60)
        print("🤖 Auto Training Data Collector")
        print("=" * 60)
        print(f"📊 Min Confidence: {min_confidence}")
        print(f"📊 Min Quality: {min_quality_score}")
        print(f"📦 Max Samples: {max_samples}")
        print("=" * 60)
        
        # Get detections
        print("\n📥 Fetching high-confidence detections...")
        detections = self.get_high_confidence_detections(
            min_confidence=min_confidence,
            limit=max_samples
        )
        print(f"✅ Found {len(detections)} detections")
        
        # Group by class for balancing
        class_counts = {}
        saved_count = 0
        
        # Define class mapping (update based on your classes)
        class_mapping = {
            'person': 0,
            'bicycle': 1,
            'car': 2,
            'motorcycle': 3,
            'bus': 4,
            'truck': 5,
            'traffic light': 6,
            'stop sign': 7
        }
        
        for det in detections:
            class_name = det['ClassName'].lower()
            
            # Balance dataset - limit samples per class
            if class_counts.get(class_name, 0) >= max_samples // len(class_mapping):
                continue
            
            # Here you would normally capture the actual frame from the camera
            # For demo, we'll create a placeholder
            # In production, you'd get the actual frame from camera stream
            
            # Placeholder: Create dummy image (replace with actual camera frame)
            dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            
            # Calculate quality
            quality = self.calculate_image_quality(dummy_image)
            
            if quality['quality_score'] < min_quality_score:
                continue
            
            # Determine train/val/test split (80/15/5)
            rand = np.random.random()
            if rand < 0.8:
                split = 'train'
            elif rand < 0.95:
                split = 'val'
            else:
                split = 'test'
            
            # Prepare annotation
            annotation = {
                'class_name': class_name,
                'bounding_box': det['BoundingBox'],
                'confidence': det['Confidence']
            }
            
            # Save to disk (YOLO format)
            try:
                img_path, label_path = self.save_training_image(
                    image=dummy_image,
                    annotations=[annotation],
                    class_mapping=class_mapping,
                    split=split
                )
                
                # Save to database
                self.save_to_database(
                    image_path=img_path,
                    width=dummy_image.shape[1],
                    height=dummy_image.shape[0],
                    quality_metrics=quality,
                    annotations=[annotation],
                    split=split
                )
                
                class_counts[class_name] = class_counts.get(class_name, 0) + 1
                saved_count += 1
                
                if saved_count % 100 == 0:
                    print(f"  ⏳ Saved {saved_count} images...")
                    
            except Exception as e:
                print(f"❌ Error saving {det['Id']}: {e}")
        
        print("\n" + "=" * 60)
        print("📊 Collection Summary")
        print("=" * 60)
        print(f"✅ Total images saved: {saved_count}")
        print("\n📈 Class distribution:")
        for class_name, count in sorted(class_counts.items(), key=lambda x: x[1], reverse=True):
            print(f"  {class_name}: {count}")
        print("=" * 60)
        
        # Generate data.yaml for YOLO training
        self.generate_yaml_config(class_mapping)
    
    def generate_yaml_config(self, class_mapping: Dict[str, int]):
        """Generate data.yaml for YOLO training"""
        
        config = f"""# ADAS Training Dataset Configuration
# Auto-generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

path: {os.path.abspath(self.output_dir)}
train: train/images
val: val/images
test: test/images

# Classes
nc: {len(class_mapping)}
names: {[name for name, _ in sorted(class_mapping.items(), key=lambda x: x[1])]}

# Training settings (recommended)
# epochs: 100
# batch: 16
# imgsz: 640
# device: 0  # GPU
"""
        
        yaml_path = f"{self.output_dir}/data.yaml"
        with open(yaml_path, 'w') as f:
            f.write(config)
        
        print(f"\n✅ Generated YOLO config: {yaml_path}")


if __name__ == "__main__":
    # Configuration
    DB_URL = "sqlite:///./adas_test.db"  # Change to SQL Server for production
    OUTPUT_DIR = "./training_data"
    
    # Initialize collector
    collector = TrainingDataCollector(db_url=DB_URL, output_dir=OUTPUT_DIR)
    
    # Collect training data
    collector.collect_and_save(
        min_confidence=0.90,
        min_quality_score=60.0,
        max_samples=1000
    )
    
    print("\n🎯 Next steps:")
    print("1. Review collected data in ./training_data")
    print("2. Train model: yolo train data=training_data/data.yaml model=yolov8n.pt epochs=100")
    print("3. Evaluate: yolo val model=runs/detect/train/weights/best.pt data=training_data/data.yaml")
