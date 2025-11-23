# ADAS AI Training & Inference APIs

## 🎯 Tổng Quan

Hệ thống ADAS với khả năng:
- ✅ Upload video + **Auto-labeling tự động** (YOLO + YOLOP + MiDaS)
- ✅ Inference real-time (phân tích video)
- ✅ Training model YOLO với dataset tự động
- ✅ Dataset management

---

## 📁 Cấu Trúc Module

```
backend-python/
├── api/
│   ├── upload/          # Upload video + auto-label
│   ├── inference/       # Inference real-time
│   ├── training/        # Training YOLO models
│   └── dataset/         # Dataset management
├── models/              # AI Models (YOLO, YOLOP, MiDaS)
│   ├── yolo_detector.py
│   ├── yolop_detector.py
│   ├── depth_estimator.py
│   └── yolo_trainer.py
├── dataset/             # Dataset storage
│   ├── raw/
│   │   ├── videos/
│   │   └── frames/
│   ├── labels/
│   └── training/
└── scripts/             # Utility scripts
```

---

## 🔥 API Endpoints

### 1. Upload Video + Auto-Label

**POST** `/api/upload/video`

Upload video và tự động label bằng YOLO + YOLOP + MiDaS.

**Request:**
```bash
curl -X POST "http://localhost:8000/api/upload/video" \
  -F "file=@video.mp4" \
  -F "description=Test video" \
  -F "auto_label=true"
```

**Response:**
```json
{
  "video_id": 1,
  "filename": "20241122_153045_video.mp4",
  "total_frames": 1200,
  "fps": 30.0,
  "duration": 40.0,
  "status": "processing",
  "message": "Video đang được xử lý và auto-label"
}
```

**Chức năng:**
1. YOLO detect vehicles
2. YOLOP detect lanes
3. MiDaS estimate depth
4. Tự động tạo labels (bounding box + distance)
5. Lưu vào database

---

### 2. Check Upload Status

**GET** `/api/upload/video/{video_id}/status`

Kiểm tra tiến độ xử lý video.

**Response:**
```json
{
  "video_id": 1,
  "filename": "20241122_153045_video.mp4",
  "status": "labeled",
  "total_frames": 1200,
  "labeled_frames": 240,
  "progress": 100.0
}
```

---

### 3. Inference Video

**POST** `/api/inference/video`

Phân tích video real-time với YOLO + YOLOP + MiDaS.

**Request:**
```bash
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@test.mp4" \
  -F "detect_vehicles=true" \
  -F "detect_lanes=true" \
  -F "estimate_depth=true" \
  -F "warning_distance=5.0"
```

**Response:**
```json
{
  "success": true,
  "frames": [
    {
      "frame_number": 0,
      "timestamp": 0.0,
      "vehicles": [
        {
          "class_name": "car",
          "confidence": 0.92,
          "bbox": [0.5, 0.4, 0.2, 0.3],
          "distance": 12.5
        }
      ],
      "lanes": {
        "left_lane": [...],
        "right_lane": [...],
        "lane_departure": false
      },
      "warnings": [
        {
          "type": "collision_warning",
          "message": "Cảnh báo: Xe phía trước cách 4.2m!",
          "severity": "medium"
        }
      ]
    }
  ],
  "summary": {
    "total_frames": 300,
    "total_vehicles_detected": 45,
    "total_warnings": 3,
    "duration": 10.0
  }
}
```

---

### 4. Inference Image

**POST** `/api/inference/image`

Phân tích 1 ảnh.

**Request:**
```bash
curl -X POST "http://localhost:8000/api/inference/image" \
  -F "file=@image.jpg"
```

---

### 5. Start Training

**POST** `/api/training/start`

Bắt đầu training YOLO model với dataset đã label.

**Request:**
```json
{
  "model_name": "adas_yolov8_v1",
  "base_model": "yolov8n.pt",
  "epochs": 50,
  "batch_size": 16,
  "img_size": 640,
  "dataset_id": 1
}
```

**Response:**
```json
{
  "training_id": "train_20241122_153045",
  "status": "started",
  "message": "Đã bắt đầu training model adas_yolov8_v1",
  "model_name": "adas_yolov8_v1",
  "base_model": "yolov8n.pt",
  "epochs": 50
}
```

---

### 6. Check Training Status

**GET** `/api/training/status/{training_id}`

Kiểm tra tiến độ training.

**Response:**
```json
{
  "status": "training",
  "progress": 45.0,
  "current_epoch": 23,
  "total_epochs": 50,
  "message": "Đang training model...",
  "metrics": {
    "loss": 0.045,
    "map50": 0.85,
    "map50_95": 0.72
  }
}
```

---

### 7. Activate Model

**POST** `/api/training/activate/{model_id}`

Kích hoạt model để dùng cho inference.

**Response:**
```json
{
  "success": true,
  "message": "Model adas_yolov8_v1 đã được kích hoạt",
  "model_id": 5
}
```

---

### 8. Dataset Management

**GET** `/api/dataset/videos` - Danh sách videos

**GET** `/api/dataset/videos/{video_id}` - Chi tiết video

**GET** `/api/dataset/videos/{video_id}/labels` - Labels của video

**DELETE** `/api/dataset/videos/{video_id}` - Xóa video

**GET** `/api/dataset/stats` - Thống kê dataset

**Response (stats):**
```json
{
  "total_videos": 10,
  "labeled_videos": 8,
  "total_frames": 12000,
  "total_labels": 2400,
  "labels_with_vehicle": 1850,
  "labels_with_lane": 2100,
  "auto_labeled_percentage": 77.08
}
```

---

## 🚀 Installation

### 1. Install Dependencies

```bash
cd backend-python
pip install -r requirements.txt
```

**Key Dependencies:**
- `torch` - PyTorch
- `ultralytics` - YOLOv8
- `opencv-python` - Image processing
- `timm` - PyTorch Image Models (MiDaS)

### 2. Download Models

Models sẽ tự động download khi chạy lần đầu:
- **YOLOv8**: `yolov8n.pt` (6MB)
- **MiDaS**: `DPT_Small` (~100MB, torch hub)

### 3. Run Server

```bash
python3 main.py
```

Server chạy tại: `http://localhost:8000`

---

## 💡 Use Cases

### Use Case 1: Upload & Auto-Label Dataset

```python
import requests

# 1. Upload video
with open('dashcam_video.mp4', 'rb') as f:
    response = requests.post(
        'http://localhost:8000/api/upload/video',
        files={'file': f},
        data={
            'description': 'Highway driving video',
            'auto_label': True
        }
    )

video_id = response.json()['video_id']

# 2. Check progress
status = requests.get(f'http://localhost:8000/api/upload/video/{video_id}/status')
print(status.json())
# => {"status": "labeled", "progress": 100.0}
```

### Use Case 2: Train Custom Model

```python
# 1. Start training
response = requests.post(
    'http://localhost:8000/api/training/start',
    json={
        'model_name': 'adas_highway_v1',
        'base_model': 'yolov8s.pt',
        'epochs': 100,
        'batch_size': 16,
        'img_size': 640
    }
)

training_id = response.json()['training_id']

# 2. Monitor training
import time
while True:
    status = requests.get(f'http://localhost:8000/api/training/status/{training_id}')
    data = status.json()
    print(f"Epoch {data['current_epoch']}/{data['total_epochs']} - {data['progress']:.1f}%")
    
    if data['status'] == 'completed':
        break
    
    time.sleep(10)

# 3. Activate model
model_id = data['model_id']
requests.post(f'http://localhost:8000/api/training/activate/{model_id}')
```

### Use Case 3: Real-time Inference

```python
# Analyze video
with open('test_drive.mp4', 'rb') as f:
    response = requests.post(
        'http://localhost:8000/api/inference/video',
        files={'file': f},
        data={
            'detect_vehicles': True,
            'detect_lanes': True,
            'estimate_depth': True,
            'warning_distance': 5.0
        }
    )

result = response.json()

# Process results
for frame in result['frames']:
    if frame['warnings']:
        print(f"⚠️  Frame {frame['frame_number']}: {frame['warnings']}")
```

---

## 🎓 Demo cho Ban Giám Khảo

### Kịch bản DEMO:

1. **Upload Video Dashboard Cam** 
   - Show auto-labeling tự động
   - YOLO detect xe
   - YOLOP detect làn đường
   - MiDaS tính khoảng cách

2. **Dataset Statistics**
   - Show số lượng frames đã label
   - Tỷ lệ auto-label
   - Chất lượng dataset

3. **Training Model**
   - Start training với dataset tự động
   - Monitor real-time progress
   - Show metrics (mAP, precision, recall)

4. **Inference Real-time**
   - Load video mới
   - Detect vehicles + lanes
   - Warning khi xe gần
   - Warning khi lệch làn

### Điểm Mạnh Để Trình Bày:

✅ **Auto-labeling hoàn toàn tự động** - không cần label thủ công

✅ **Training pipeline hoàn chỉnh** - từ data → train → deploy

✅ **Multi-model integration** - YOLO + YOLOP + MiDaS

✅ **Real-time inference** - phân tích video với cảnh báo

✅ **Production-ready** - API đầy đủ, database, monitoring

---

## 📊 Database Schema

### VideoDatasets
```sql
- id, filename, file_path
- fps, total_frames, labeled_frames
- status (uploaded, processing, labeled, error)
- created_at, processed_at
```

### Labels
```sql
- id, video_id, frame_number
- label_data (JSON: bbox, class, distance)
- has_vehicle, has_lane
- auto_labeled, verified
```

### AIModels
```sql
- id, name, model_type, version
- file_path, accuracy, config
- is_active
```

---

## 🔧 Configuration

Tạo file `.env`:

```env
# Database
DATABASE_URL=sqlite:///./adas.db

# Model paths
YOLO_MODEL_PATH=models/trained/
DATASET_PATH=dataset/

# Training
DEFAULT_EPOCHS=50
DEFAULT_BATCH_SIZE=16
DEFAULT_IMG_SIZE=640

# Inference
WARNING_DISTANCE=5.0
CONFIDENCE_THRESHOLD=0.5
```

---

## 📝 Notes

- Models tự động download lần đầu chạy
- Training yêu cầu GPU (recommended) hoặc CPU (chậm hơn)
- Dataset tự động split 80/20 train/val
- Auto-label xử lý mỗi 5 frames để tối ưu tốc độ

---

## 🆘 Troubleshooting

### Lỗi "torch not found"
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### Lỗi MiDaS download
```bash
# MiDaS sẽ tự động download từ torch hub
# Nếu lỗi, có thể dùng simple depth estimation (fallback)
```

### Training chậm
- Giảm `batch_size`
- Giảm `img_size` (640 → 416)
- Dùng GPU nếu có

---

## 🎉 SHOW BAN GIÁM KHẢO CỰC CHẤT!

Hệ thống này cho thấy:
1. ✅ Hiểu sâu về AI/ML pipeline
2. ✅ Tích hợp multiple models
3. ✅ Auto-labeling thông minh
4. ✅ Production-ready code
5. ✅ Full-stack capabilities

**Điểm độc đáo:** Tự động label dataset bằng AI → train model mới → improve accuracy → iterate!
