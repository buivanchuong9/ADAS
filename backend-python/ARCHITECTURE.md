# 🏗️ ADAS Platform - Architecture Overview

## 📁 Project Structure

```
adas-platform/
├── backend-python/              # Python Backend (FastAPI)
│   ├── api/                     # 🔥 NEW: API Modules
│   │   ├── upload/             # Upload video + Auto-label
│   │   │   └── router.py
│   │   ├── inference/          # Real-time inference
│   │   │   └── router.py
│   │   ├── training/           # Model training
│   │   │   └── router.py
│   │   └── dataset/            # Dataset management
│   │       └── router.py
│   │
│   ├── models/                  # 🔥 NEW: AI Models
│   │   ├── yolo_detector.py    # YOLOv8 wrapper
│   │   ├── yolop_detector.py   # YOLOP lane detection
│   │   ├── depth_estimator.py  # MiDaS depth
│   │   └── yolo_trainer.py     # Training pipeline
│   │
│   ├── dataset/                 # 🔥 NEW: Dataset storage
│   │   ├── raw/
│   │   │   ├── videos/         # Uploaded videos
│   │   │   └── frames/         # Extracted frames
│   │   ├── labels/             # Auto-generated labels
│   │   └── training/           # Training dataset
│   │       ├── images/
│   │       │   ├── train/
│   │       │   └── val/
│   │       └── labels/
│   │           ├── train/
│   │           └── val/
│   │
│   ├── scripts/                 # 🔥 NEW: Utility scripts
│   │   └── test_apis.py
│   │
│   ├── main.py                  # Main FastAPI app
│   ├── models.py                # Database models (updated)
│   ├── schemas.py               # Pydantic schemas (updated)
│   ├── database.py              # Database connection
│   ├── services.py              # Business logic
│   ├── config.py                # Configuration
│   └── requirements.txt         # 🔥 UPDATED: Added torch, ultralytics
│
├── app/                         # Next.js Frontend
├── components/                  # React components
└── public/                      # Static files
```

---

## 🎯 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ADAS Platform v3.0                        │
└─────────────────────────────────────────────────────────────────┘

┌──────────────┐     ┌──────────────────────────────────────────┐
│   Frontend   │────▶│           Backend APIs                    │
│  (Next.js)   │     │                                           │
│              │     │  /api/upload     - Upload + Auto-label   │
│  - Dashboard │     │  /api/inference  - Real-time analysis    │
│  - Analytics │     │  /api/training   - Model training        │
│  - Training  │     │  /api/dataset    - Dataset management    │
└──────────────┘     └──────────────────────────────────────────┘
                                      │
                     ┌────────────────┼────────────────┐
                     │                │                │
                     ▼                ▼                ▼
            ┌─────────────┐  ┌──────────────┐  ┌──────────────┐
            │   YOLO      │  │   YOLOP      │  │    MiDaS     │
            │  Detector   │  │ Lane Detect  │  │ Depth Est.   │
            │             │  │              │  │              │
            │ - Vehicle   │  │ - Lanes      │  │ - Distance   │
            │ - Bbox      │  │ - Departure  │  │ - 3D info    │
            └─────────────┘  └──────────────┘  └──────────────┘
                     │                │                │
                     └────────────────┼────────────────┘
                                      ▼
                            ┌──────────────────┐
                            │    Database      │
                            │  (SQL Server)    │
                            │                  │
                            │ - Videos         │
                            │ - Labels         │
                            │ - Models         │
                            │ - Metrics        │
                            └──────────────────┘
```

---

## 🔄 Data Flow

### 1. Upload & Auto-Label Flow

```
User uploads video
       │
       ▼
┌──────────────────┐
│  Upload API      │  1. Save video to disk
│  /api/upload/    │  2. Extract metadata (fps, frames)
└──────────────────┘  3. Create DB record
       │              4. Start background task
       ▼
┌──────────────────┐
│  Auto-Labeling   │  For each frame (every 5 frames):
│  Background Task │    - YOLO: Detect vehicles
└──────────────────┘    - YOLOP: Detect lanes
       │                - MiDaS: Estimate depth
       │                - Save frame + label to DB
       ▼
┌──────────────────┐
│   Database       │  Labels: {
│   VideoDatasets  │    class_id, bbox, confidence,
│   Labels         │    distance, has_vehicle, has_lane
└──────────────────┘  }
```

### 2. Training Flow

```
User starts training
       │
       ▼
┌──────────────────┐
│  Training API    │  1. Get labeled dataset from DB
│  /api/training/  │  2. Prepare YOLO format
└──────────────────┘  3. Split train/val
       │              4. Start background training
       ▼
┌──────────────────┐
│  YOLO Trainer    │  - Load base model (yolov8n.pt)
│                  │  - Train with custom dataset
│                  │  - Save checkpoints
└──────────────────┘  - Calculate metrics (mAP)
       │
       ▼
┌──────────────────┐
│  Trained Model   │  - Save .pt file
│  AIModels (DB)   │  - Store metadata
└──────────────────┘  - Set as active
```

### 3. Inference Flow

```
User uploads test video
       │
       ▼
┌──────────────────┐
│  Inference API   │  For each frame:
│  /api/inference/ │    1. YOLO: Detect vehicles
└──────────────────┘    2. YOLOP: Detect lanes
       │                3. MiDaS: Get depth
       │                4. Calculate warnings
       ▼
┌──────────────────┐
│  Warning System  │  - Collision warning (distance < 5m)
│                  │  - Lane departure warning
└──────────────────┘  - Speed warnings
       │
       ▼
┌──────────────────┐
│  Response        │  {
│  to Frontend     │    frames: [...],
└──────────────────┘    warnings: [...],
                        summary: {...}
                      }
```

---

## 🗄️ Database Schema

### New Tables

```sql
-- VideoDatasets: Uploaded videos
CREATE TABLE VideoDatasets (
    Id INT PRIMARY KEY,
    Filename NVARCHAR(200),
    FilePath NVARCHAR(1000),
    Fps FLOAT,
    TotalFrames INT,
    LabeledFrames INT,
    Status NVARCHAR(50),  -- uploaded, processing, labeled
    CreatedAt DATETIME,
    ProcessedAt DATETIME
);

-- Labels: Auto-generated labels
CREATE TABLE Labels (
    Id INT PRIMARY KEY,
    VideoId INT FOREIGN KEY,
    FrameNumber INT,
    LabelData TEXT,  -- JSON: [{bbox, class, distance}]
    HasVehicle BIT,
    HasLane BIT,
    AutoLabeled BIT,
    Verified BIT,
    CreatedAt DATETIME
);

-- AIModels: Trained models
CREATE TABLE AIModels (
    Id INT PRIMARY KEY,
    Name NVARCHAR(200),
    ModelType NVARCHAR(50),  -- yolov8, yolop, midas
    Version NVARCHAR(50),
    FilePath NVARCHAR(1000),
    Accuracy FLOAT,
    Config TEXT,  -- JSON training config
    IsActive BIT,
    CreatedAt DATETIME
);
```

---

## 🔌 API Endpoints Summary

### Upload Module
- `POST /api/upload/video` - Upload + auto-label
- `GET /api/upload/video/{id}/status` - Check progress

### Inference Module
- `POST /api/inference/video` - Analyze video
- `POST /api/inference/image` - Analyze image

### Training Module
- `POST /api/training/start` - Start training
- `GET /api/training/status/{id}` - Monitor progress
- `GET /api/training/list` - List all trainings
- `POST /api/training/activate/{model_id}` - Activate model

### Dataset Module
- `GET /api/dataset/videos` - List videos
- `GET /api/dataset/videos/{id}` - Video details
- `GET /api/dataset/videos/{id}/labels` - Get labels
- `DELETE /api/dataset/videos/{id}` - Delete video
- `GET /api/dataset/stats` - Statistics

---

## 🧠 AI Models Integration

### YOLOv8 (Vehicle Detection)
- **Purpose**: Detect cars, trucks, motorcycles, buses
- **Input**: Video frame (BGR)
- **Output**: Bounding boxes + confidence
- **Model**: yolov8n.pt (6MB) → custom trained

### YOLOP (Lane Detection)
- **Purpose**: Detect lane lines, lane departure
- **Input**: Video frame
- **Output**: Lane coordinates, departure flag
- **Fallback**: OpenCV Canny + Hough Transform

### MiDaS (Depth Estimation)
- **Purpose**: Estimate distance to objects
- **Input**: Video frame
- **Output**: Depth map (inverse depth)
- **Model**: DPT_Small (~100MB)

---

## 🔧 Configuration

### Environment Variables
```env
# Database
DATABASE_URL=sqlite:///./adas.db

# Paths
DATASET_PATH=dataset/
MODEL_PATH=models/trained/

# Training
DEFAULT_EPOCHS=50
DEFAULT_BATCH_SIZE=16
DEFAULT_IMG_SIZE=640

# Inference
WARNING_DISTANCE=5.0
CONFIDENCE_THRESHOLD=0.5
```

---

## 📊 Performance Metrics

### Auto-Labeling
- Speed: ~5 FPS (CPU), ~15 FPS (GPU)
- Accuracy: 85-95% (depends on video quality)
- Process: 1 frame every 5 frames

### Training
- Time: ~10 mins (50 epochs, 1000 images, GPU)
- mAP50: 0.80-0.90 (good dataset)
- Model size: ~12MB (yolov8n)

### Inference
- Speed: ~10 FPS (CPU), ~30 FPS (GPU)
- Latency: <100ms per frame
- Memory: ~2GB (with all models loaded)

---

## 🚀 Deployment

### Development
```bash
cd backend-python
pip install -r requirements.txt
python3 main.py
```

### Production
```bash
# With GPU
pip install torch --index-url https://download.pytorch.org/whl/cu118

# Run with gunicorn
gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker
```

---

## 🎓 Key Features for Demo

1. ✅ **Auto-Labeling**: Upload video → tự động label
2. ✅ **Training Pipeline**: Dataset → Train → Model
3. ✅ **Multi-Model**: YOLO + YOLOP + MiDaS
4. ✅ **Real-time Inference**: Video analysis + warnings
5. ✅ **Production Ready**: APIs, DB, monitoring

---

## 📚 Documentation

- [QUICKSTART_TRAINING.md](QUICKSTART_TRAINING.md) - Quick setup guide
- [API_TRAINING_README.md](API_TRAINING_README.md) - Full API docs
- [API Swagger](http://localhost:8000/docs) - Interactive docs

---

**Built with ❤️ for ADAS Platform v3.0**
