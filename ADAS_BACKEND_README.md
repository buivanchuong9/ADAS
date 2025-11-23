# 🚗 ADAS FastAPI Backend - Complete Documentation

## 📋 Overview

Production-ready FastAPI backend for Advanced Driver Assistance System (ADAS) with:
- Multi-model AI pipeline (YOLOv8, YOLOP, LaneNet, MiDaS, DeepSort, Traffic Sign Detection)
- Video processing and frame extraction
- SQL Server database integration
- Automatic training pipeline
- Real-time detection results

## 🏗️ Architecture

```
┌──────────────┐
│   Video      │
│   Upload     │
└──────┬───────┘
       │
       ▼
┌──────────────────────────────────────┐
│   FastAPI Backend                    │
│   ┌────────────────────────────┐    │
│   │  Video Processor           │    │
│   │  - Extract frames (0.5s)   │    │
│   │  - Run AI pipeline         │    │
│   │  - Save frames + labels    │    │
│   └────────────────────────────┘    │
│                                      │
│   ┌────────────────────────────┐    │
│   │  Model Manager             │    │
│   │  - YOLOv8n (fast)          │    │
│   │  - YOLOv8m (accurate)      │    │
│   │  - Lane detection          │    │
│   │  - Distance estimation     │    │
│   │  - Traffic signs           │    │
│   └────────────────────────────┘    │
│                                      │
│   ┌────────────────────────────┐    │
│   │  Training Manager          │    │
│   │  - Background training     │    │
│   │  - Auto model reload       │    │
│   └────────────────────────────┘    │
└──────────────────────────────────────┘
       │                      │
       ▼                      ▼
┌──────────────┐      ┌──────────────┐
│  SQL Server  │      │   Dataset    │
│  (Events DB) │      │  (Training)  │
└──────────────┘      └──────────────┘
```

## 📁 Folder Structure

On Windows Server:
```
C:\ADAS\
├── backend\           # Backend code (auto-created)
├── models\            # AI models
│   ├── yolov8n.pt    # Fast vehicle detection
│   ├── yolov8m.pt    # High accuracy detection
│   ├── yolop.pt      # Lane + vehicle detection
│   ├── lanenet.pt    # Lane segmentation
│   ├── midas.pt      # Depth estimation
│   ├── deepsort.pt   # Vehicle tracking
│   └── traffic_sign.pt # Traffic sign detection
├── dataset\
│   ├── images\       # Extracted frames
│   ├── labels\       # YOLO format annotations
│   └── videos\       # Uploaded videos
├── logs\             # Application logs
└── temp\             # Temporary files
```

On Mac/Linux (development):
```
<project>/ADAS_DATA/
├── models/
├── dataset/
│   ├── images/
│   ├── labels/
│   └── videos/
├── logs/
└── temp/
```

## 🚀 Quick Start

### 1. Install Dependencies

```bash
# Windows
pip install -r requirements_adas.txt

# Mac/Linux
pip install -r requirements_adas.txt
```

### 2. Setup Database

**Windows (SQL Server):**
```sql
-- Create database
CREATE DATABASE ADAS_DB;
```

**Mac/Linux (SQLite):**
- No setup needed, database is created automatically

### 3. Run Backend

```bash
python adas_backend.py
```

Server will start on: **http://localhost:8000**

## 📡 API Endpoints

### 1. Upload Video

**Endpoint:** `POST /upload_video`

**Description:** Upload video for AI processing

**Request:**
```bash
curl -X POST "http://localhost:8000/upload_video" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@dashcam.mp4"
```

**Response:**
```json
{
  "vehicle_count": 15,
  "lane_status": "on_lane",
  "distance_front_car": 12.5,
  "traffic_signs": ["speed_limit_50", "stop"],
  "vehicles": [
    {
      "vehicle_count": 3,
      "lane_status": "on_lane",
      "distance_front_car": 12.5,
      "traffic_signs": [],
      "vehicles": [...]
    }
  ],
  "processing_time": 5.23,
  "frame_count": 20,
  "video_name": "20251122_143025_dashcam.mp4"
}
```

**Process:**
1. Video saved to `dataset/videos/`
2. Frames extracted every 0.5 seconds
3. Each frame processed through AI pipeline:
   - Vehicle detection (YOLOv8)
   - Lane detection
   - Distance estimation
   - Traffic sign detection
4. Results saved to SQL Server database
5. Frames + labels saved for training

### 2. Get System Status

**Endpoint:** `GET /status`

**Description:** Check model loading status and dataset info

**Request:**
```bash
curl http://localhost:8000/status
```

**Response:**
```json
{
  "yolov8n_loaded": true,
  "yolov8m_loaded": true,
  "yolop_loaded": false,
  "lanenet_loaded": false,
  "midas_loaded": false,
  "deepsort_loaded": false,
  "traffic_sign_loaded": false,
  "last_training": "2025-11-22T14:30:25",
  "total_training_images": 250
}
```

### 3. Start Training

**Endpoint:** `POST /train`

**Description:** Start model training on collected dataset

**Request:**
```bash
curl -X POST http://localhost:8000/train
```

**Response:**
```json
{
  "status": "started",
  "message": "Training started in background"
}
```

**Process:**
1. Uses all images + labels in `dataset/`
2. Trains YOLOv8 models
3. Saves new models to `models/`
4. Automatically reloads models (no restart needed)

### 4. Get Training Status

**Endpoint:** `GET /train/status`

**Description:** Check current training progress

**Request:**
```bash
curl http://localhost:8000/train/status
```

**Response:**
```json
{
  "status": "training",
  "message": "Training YOLOv8n...",
  "epoch": 25,
  "total_epochs": 50,
  "current_loss": 0.045
}
```

### 5. Health Check

**Endpoint:** `GET /health`

**Request:**
```bash
curl http://localhost:8000/health
```

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-11-22T14:30:25.123456",
  "models_loaded": 2,
  "database_connected": true
}
```

## 🗄️ Database Schema

**Table: Events**

```sql
CREATE TABLE events (
    id INT PRIMARY KEY IDENTITY(1,1),
    vehicle_type VARCHAR(50),
    confidence FLOAT,
    distance FLOAT,
    lane_status VARCHAR(50),
    traffic_sign VARCHAR(100),
    timestamp DATETIME DEFAULT GETUTCDATE(),
    frame_path VARCHAR(500),
    video_name VARCHAR(200)
);
```

**Sample Query:**
```sql
-- Get recent detections
SELECT TOP 10 * FROM events ORDER BY timestamp DESC;

-- Count vehicles by type
SELECT vehicle_type, COUNT(*) as count 
FROM events 
GROUP BY vehicle_type;

-- Get dangerous situations (close distance)
SELECT * FROM events 
WHERE distance < 10 
ORDER BY timestamp DESC;
```

## 🎓 Training Pipeline

### Automatic Data Collection

Every video upload:
1. Frames saved to `dataset/images/`
2. Labels saved to `dataset/labels/` in YOLO format:
   ```
   0 0.5 0.6 0.2 0.3
   # class_id x_center y_center width height (all normalized 0-1)
   ```

### Training Process

```python
# Trigger training via API
curl -X POST http://localhost:8000/train

# Or programmatically
import requests
response = requests.post("http://localhost:8000/train")
```

**What happens:**
1. System checks dataset (needs minimum 10 images)
2. Creates YOLO data.yaml configuration
3. Trains YOLOv8n model (50 epochs default)
4. Saves best model weights
5. Replaces old model with new trained model
6. Reloads model in memory (no restart needed)

### Configuration

Edit in `adas_backend.py`:
```python
class Config:
    TRAINING_EPOCHS = 50      # Number of training epochs
    BATCH_SIZE = 16           # Batch size for training
    IMG_SIZE = 640            # Image size for training
    FRAME_EXTRACT_INTERVAL = 0.5  # Extract frame every 0.5s
```

## 🔧 Customization

### Add Custom Models

1. **Place model file in `models/` folder:**
   ```
   C:\ADAS\models\my_custom_model.pt
   ```

2. **Register in ModelManager:**
   ```python
   self.model_files = {
       # ... existing models ...
       "my_custom": "my_custom_model.pt"
   }
   ```

3. **Add loading logic:**
   ```python
   def _load_custom_model(self, model_name: str):
       model_path = Config.MODELS_DIR / self.model_files[model_name]
       if model_path.exists():
           self.models[model_name] = torch.load(str(model_path))
   ```

### Change Database

**Switch to PostgreSQL:**
```python
# In Config class
DATABASE_URL = "postgresql://user:pass@localhost/adas_db"
```

**Switch to MySQL:**
```python
DATABASE_URL = "mysql+pymysql://user:pass@localhost/adas_db"
```

## 🐛 Troubleshooting

### Models not loading

**Issue:** Models fail to load on startup

**Solution:**
1. Check if model files exist in `C:\ADAS\models\`
2. First run will auto-download YOLOv8 models
3. Check logs in `C:\ADAS\logs\`

### Database connection failed

**Windows SQL Server:**
```bash
# Check SQL Server is running
Get-Service MSSQL*

# Test connection
sqlcmd -S localhost -E -Q "SELECT @@VERSION"
```

**Fix:** Update connection string in code:
```python
# For named instance
DATABASE_URL = "mssql+pyodbc://localhost\SQLEXPRESS/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server&trusted_connection=yes"
```

### Training fails

**Issue:** Not enough training data

**Solution:**
- Upload at least 10 videos
- Check `dataset/images/` has images
- Check `dataset/labels/` has corresponding .txt files

### CUDA out of memory

**Issue:** GPU runs out of memory during training

**Solution:**
```python
# Reduce batch size in Config
BATCH_SIZE = 8  # or 4
```

## 📊 Performance Tips

### Speed up processing

1. **Use GPU:**
   - Install CUDA-enabled PyTorch
   - Backend auto-detects GPU

2. **Reduce frame extraction:**
   ```python
   FRAME_EXTRACT_INTERVAL = 1.0  # Process 1 frame/second
   ```

3. **Use faster model:**
   - YOLOv8n is faster than YOLOv8m
   - Trade-off: speed vs accuracy

### Optimize database

```sql
-- Add indexes
CREATE INDEX idx_timestamp ON events(timestamp);
CREATE INDEX idx_vehicle_type ON events(vehicle_type);

-- Partition large tables
-- Archive old data
```

## 🔒 Security

### Production Deployment

1. **Restrict CORS:**
   ```python
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["https://yourdomain.com"],
       ...
   )
   ```

2. **Add authentication:**
   ```python
   from fastapi.security import HTTPBearer
   
   security = HTTPBearer()
   
   @app.post("/upload_video")
   async def upload_video(credentials: HTTPBearer = Depends(security)):
       # Verify token
       ...
   ```

3. **Use HTTPS:**
   - Deploy behind nginx/IIS
   - Use SSL certificates

## 📝 API Documentation

Interactive API docs available at:
- **Swagger UI:** http://localhost:8000/docs
- **ReDoc:** http://localhost:8000/redoc

## 🎉 Features

- ✅ Multi-model AI pipeline
- ✅ Real-time video processing
- ✅ Automatic frame extraction
- ✅ SQL Server database integration
- ✅ Automatic training data collection
- ✅ Background model training
- ✅ Hot model reload (no restart)
- ✅ YOLO format dataset generation
- ✅ Distance estimation
- ✅ Lane detection
- ✅ Traffic sign detection
- ✅ Comprehensive logging
- ✅ REST API with async support
- ✅ Cross-platform (Windows/Mac/Linux)

## 📞 Support

For issues or questions:
1. Check logs: `C:\ADAS\logs\`
2. Review API docs: http://localhost:8000/docs
3. Check database: Query `events` table

## 📄 License

MIT License
