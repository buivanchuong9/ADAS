"""
ADAS Unified Model - Tất cả tính năng trong 1 model với auto-learning
- YOLOv8 detection (ALL 80 COCO classes: vehicles, pedestrians, trees, animals, etc.)
- Lane detection
- Depth estimation
- TTC computation
- Voice alerts
- Auto-collection & Continuous Learning
"""
import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path
import time
import json
from datetime import datetime
import os

def convert_to_python_type(obj):
    """Convert numpy types to Python native types for JSON serialization"""
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {key: convert_to_python_type(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_python_type(item) for item in obj]
    return obj

class ADASUnifiedModel:
    def __init__(self, weights_dir="ai_models/weights", enable_auto_collection=True):
        self.weights_dir = Path(weights_dir)
        self.enable_auto_collection = enable_auto_collection
        
        # Load YOLOv8n - fastest and most reliable
        yolo_path = self.weights_dir / "yolov8n.pt"
        if yolo_path.exists():
            print("Loading YOLOv8n for ALL object detection...")
            self.yolo = YOLO(str(yolo_path))
            print("✅ YOLOv8n loaded - Can detect all 80 COCO classes")
        else:
            raise FileNotFoundError(f"YOLOv8n not found at {yolo_path}")
        
        # All 80 COCO classes - detect everything!
        self.coco_classes = {
            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
            5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
            10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird',
            15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
            20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
            25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
            30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat',
            35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle',
            40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon',
            45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange',
            50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut',
            55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed',
            60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse',
            65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven',
            70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock',
            75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
        }
        
        # Danger classes for TTC alerts (vehicles, pedestrians)
        self.danger_classes = {
            'person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
            'traffic light', 'stop sign', 'dog', 'cat', 'horse', 'cow'
        }
        
        # TTC thresholds
        self.ttc_critical = 2.0  # seconds
        self.ttc_warning = 3.5   # seconds
        
        # Auto-collection settings
        self.collection_dir = Path("dataset/auto_collected")
        self.collection_dir.mkdir(parents=True, exist_ok=True)
        (self.collection_dir / "images").mkdir(exist_ok=True)
        (self.collection_dir / "labels").mkdir(exist_ok=True)
        
        # Memory of seen objects for learning
        self.seen_objects = self._load_seen_objects()
        self.collection_stats = {
            'total_collected': 0,
            'new_objects_learned': 0,
            'last_collection': None
        }
        
        # Memory of seen objects for learning
        self.seen_objects = self._load_seen_objects()
        self.collection_stats = {
            'total_collected': 0,
            'new_objects_learned': 0,
            'last_collection': None
        }
    
    def _load_seen_objects(self):
        """Load memory of previously seen objects"""
        memory_file = self.collection_dir / "object_memory.json"
        if memory_file.exists():
            with open(memory_file, 'r') as f:
                return json.load(f)
        return {}
    
    def _save_seen_objects(self):
        """Save memory of seen objects"""
        memory_file = self.collection_dir / "object_memory.json"
        with open(memory_file, 'w') as f:
            json.dump(self.seen_objects, f, indent=2)
    
    def _is_new_object(self, class_name, bbox, conf):
        """Check if this is a new/unique object worth collecting"""
        # Always collect high confidence detections of new object types
        if class_name not in self.seen_objects:
            return True, "new_class"
        
        # Collect high confidence variations
        if conf > 0.85:
            obj_data = self.seen_objects.get(class_name, {})
            count = obj_data.get('count', 0)
            
            # Collect first 50 high-quality samples of each class
            if count < 50:
                return True, "high_quality"
        
        return False, None
    
    def _save_training_data(self, frame, detections, frame_id):
        """Save frame and annotations in YOLO format for training"""
        if not self.enable_auto_collection:
            return
        
        try:
            h, w = frame.shape[:2]
            
            # Save image
            img_name = f"{frame_id}.jpg"
            img_path = self.collection_dir / "images" / img_name
            cv2.imwrite(str(img_path), frame)
            
            # Save YOLO format labels
            label_name = f"{frame_id}.txt"
            label_path = self.collection_dir / "labels" / label_name
            
            with open(label_path, 'w') as f:
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    # Convert to YOLO format (normalized center x, y, width, height)
                    x_center = ((x1 + x2) / 2) / w
                    y_center = ((y1 + y2) / 2) / h
                    width = (x2 - x1) / w
                    height = (y2 - y1) / h
                    
                    class_id = det['class_id']
                    f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
            
            # Update memory
            for det in detections:
                class_name = det['class']
                if class_name not in self.seen_objects:
                    self.seen_objects[class_name] = {
                        'count': 0,
                        'first_seen': datetime.now().isoformat(),
                        'avg_confidence': 0.0
                    }
                
                obj_data = self.seen_objects[class_name]
                obj_data['count'] += 1
                obj_data['last_seen'] = datetime.now().isoformat()
                
                # Update avg confidence
                old_avg = obj_data.get('avg_confidence', 0.0)
                obj_data['avg_confidence'] = (old_avg * (obj_data['count'] - 1) + det['conf']) / obj_data['count']
            
            self._save_seen_objects()
            self.collection_stats['total_collected'] += 1
            self.collection_stats['last_collection'] = datetime.now().isoformat()
            
            print(f"📦 Saved training data: {img_name} with {len(detections)} objects")
            
        except Exception as e:
            print(f"⚠️ Error saving training data: {e}")
    
    def estimate_distance(self, bbox, frame_height):
        """Ước tính khoảng cách dựa trên kích thước bbox"""
        x1, y1, x2, y2 = bbox
        height = y2 - y1
        width = x2 - x1
        
        # Baseline: Object chiếm 50% frame height ≈ 5 meters
        # Simple inverse proportion
        if height > 0:
            estimated_distance = (frame_height * 5.0) / (height * 2)
            return max(0.5, min(estimated_distance, 50.0))  # Clamp 0.5-50m
        return 10.0
    
    def estimate_speed(self, prev_distance, curr_distance, delta_time):
        """Ước tính tốc độ tương đối (m/s)"""
        if delta_time > 0 and prev_distance is not None:
            speed = (prev_distance - curr_distance) / delta_time
            return speed
        return 0.0
    
    def compute_ttc(self, distance, speed):
        """Time To Collision"""
        if speed > 0.1:  # Approaching
            ttc = distance / speed
            return ttc
        return float('inf')
    
    def detect_lanes(self, frame):
        """Simple lane detection using edge detection"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Region of interest (bottom half)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        roi_vertices = np.array([[
            (0, height),
            (width // 2 - 50, height // 2),
            (width // 2 + 50, height // 2),
            (width, height)
        ]], dtype=np.int32)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Hough lines
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, 
                                minLineLength=50, maxLineGap=150)
        
        lane_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Convert to Python native int for JSON serialization
                lane_lines.append([(int(x1), int(y1)), (int(x2), int(y2))])
        
        return lane_lines
    
    def run_inference(self, frame, prev_detections=None, delta_time=0.1, collect_data=True):
        """
        Chạy tất cả inference một lần với auto-learning
        Returns: {
            'detections': [...],  # ALL detected objects (80 COCO classes)
            'lanes': [...],
            'alerts': [...],
            'stats': {...},
            'collection_stats': {...}  # Auto-collection statistics
        }
        """
        start_time = time.time()
        h, w = frame.shape[:2]
        
        # 1. YOLO Detection - Detect ALL 80 COCO classes!
        # Lower conf threshold to catch more objects including trees, animals, etc.
        results = self.yolo(frame, conf=0.25, iou=0.45, imgsz=640, verbose=False)[0]
        
        # Print detected objects with class names for debugging
        if len(results.boxes) > 0:
            detected_classes = [results.names[int(box.cls[0])] for box in results.boxes]
            class_summary = ', '.join([f"{cls}" for cls in set(detected_classes)])
            print(f"🔍 Detected {len(results.boxes)} objects → [{class_summary}]", end="")
        else:
            print(f"🔍 Detected 0 objects", end="")
        
        detections = []
        alerts = []
        new_objects = []
        
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = results.names[cls_id]
            
            # Check if this is a new/unique object
            is_new, reason = self._is_new_object(cls_name, [x1, y1, x2, y2], conf)
            if is_new:
                new_objects.append({'class': cls_name, 'reason': reason})
            
            # Estimate distance
            distance = self.estimate_distance([x1, y1, x2, y2], h)
            
            # Check previous detection for speed
            speed = 0.0
            prev_dist = None
            if prev_detections:
                for prev in prev_detections:
                    if prev['class'] == cls_name and prev['class_id'] == cls_id:
                        prev_dist = prev.get('distance', None)
                        break
            
            if prev_dist:
                speed = self.estimate_speed(prev_dist, distance, delta_time)
            
            # Compute TTC
            ttc = self.compute_ttc(distance, speed)
            
            # Danger assessment
            is_danger = cls_name in self.danger_classes and distance < 10.0
            
            detection = {
                'bbox': [float(x1), float(y1), float(x2), float(y2)],
                'conf': float(conf),
                'cls': str(cls_name),
                'class': str(cls_name),
                'class_id': int(cls_id),
                'distance_m': float(distance),
                'speed': float(speed),
                'ttc': float(ttc) if ttc != float('inf') else None,
                'danger': bool(is_danger),
                'is_new': bool(is_new)  # Flag for newly discovered objects
            }
            detections.append(detection)
            
            # Generate alerts only for danger classes
            if is_danger:
                if ttc < self.ttc_critical:
                    alerts.append({
                        'level': 'critical',
                        'message': f'⚠️ CRITICAL: {cls_name} collision in {ttc:.1f}s!',
                        'class': str(cls_name),
                        'distance': float(distance),
                        'ttc': float(ttc)
                    })
                elif ttc < self.ttc_warning:
                    alerts.append({
                        'level': 'warning',
                        'message': f'⚡ WARNING: {cls_name} ahead at {distance:.1f}m',
                        'class': str(cls_name),
                        'distance': float(distance),
                        'ttc': float(ttc)
                    })
        
        # 2. Auto-collect training data if enabled and new objects detected
        if collect_data and self.enable_auto_collection and len(new_objects) > 0:
            frame_id = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
            self._save_training_data(frame, detections, frame_id)
            self.collection_stats['new_objects_learned'] += len(new_objects)
        
        # 3. Lane Detection (lightweight)
        lanes = self.detect_lanes(frame)
        
        # 4. Stats
        inference_time = (time.time() - start_time) * 1000
        fps = int(1000 / inference_time) if inference_time > 0 else 0
        
        # Get unique object classes detected
        unique_classes = list(set([d['class'] for d in detections]))
        
        result = {
            'detections': detections,
            'lanes': lanes,
            'alerts': alerts,
            'stats': {
                'fps': fps,
                'inference_time': inference_time,
                'total_objects': len(detections),
                'unique_classes': unique_classes,
                'new_objects_count': len(new_objects)
            },
            'collection_stats': self.collection_stats.copy(),
            'new_objects': new_objects
        }
        
        print(f" → {len(detections)} objects ({len(unique_classes)} types), {len(new_objects)} new, {len(lanes)} lanes, {len(alerts)} alerts, {fps} FPS")
        
        # Convert all numpy types to Python native types
        return convert_to_python_type(result)
