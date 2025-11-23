# 🚀 HOW TO RUN - ADAS Training System

## Quick Start (3 bước)

### Bước 1: Install
```bash
cd backend-python
./install.sh       # macOS/Linux
# hoặc
install.bat        # Windows
```

### Bước 2: Run
```bash
python3 main.py
```

### Bước 3: Test
```bash
# Mở browser
open http://localhost:8000/docs

# Hoặc test tự động
python3 scripts/test_apis.py
```

---

## 📋 Chi Tiết

### 1. Kiểm tra Python
```bash
python3 --version
# Cần Python 3.8+
```

### 2. Cài đặt Dependencies

**Cách 1: Tự động (Recommended)**
```bash
./install.sh
```

**Cách 2: Manual**
```bash
# Tạo virtual environment
python3 -m venv venv
source venv/bin/activate  # macOS/Linux
# hoặc
venv\Scripts\activate.bat  # Windows

# Install
pip install -r requirements.txt

# Download YOLO
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

### 3. Chạy Server
```bash
python3 main.py

# Server sẽ chạy tại:
# http://localhost:8000
```

### 4. Verify
```bash
# Test API health
curl http://localhost:8000/docs

# Hoặc
python3 scripts/test_apis.py
```

---

## 🧪 Test APIs

### 1. Dataset Stats
```bash
curl http://localhost:8000/api/dataset/stats
```

**Expected Response:**
```json
{
  "total_videos": 0,
  "labeled_videos": 0,
  "total_frames": 0,
  "total_labels": 0
}
```

### 2. Upload Video (cần file video)
```bash
curl -X POST "http://localhost:8000/api/upload/video" \
  -F "file=@your_video.mp4" \
  -F "auto_label=true"
```

### 3. Start Training (cần dataset)
```bash
curl -X POST "http://localhost:8000/api/training/start" \
  -H "Content-Type: application/json" \
  -d '{
    "model_name": "test_model",
    "epochs": 2,
    "batch_size": 8
  }'
```

### 4. Inference Video (cần file video)
```bash
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@test_video.mp4"
```

---

## 🎯 Demo Workflow

### Scenario: Upload → Train → Inference

#### 1. Upload Video
```bash
# Upload dashcam video
curl -X POST "http://localhost:8000/api/upload/video" \
  -F "file=@dashcam.mp4" \
  -F "description=Highway driving" \
  -F "auto_label=true"

# Response: {"video_id": 1, "status": "processing"}
```

#### 2. Check Progress
```bash
# Đợi vài phút cho auto-labeling
curl http://localhost:8000/api/upload/video/1/status

# Response: {"status": "labeled", "progress": 100}
```

#### 3. View Dataset Stats
```bash
curl http://localhost:8000/api/dataset/stats

# Response:
# {
#   "total_videos": 1,
#   "labeled_videos": 1,
#   "total_frames": 1200,
#   "total_labels": 240
# }
```

#### 4. Train Model
```bash
curl -X POST "http://localhost:8000/api/training/start" \
  -H "Content-Type: application/json" \
  -d '{
    "model_name": "adas_model_v1",
    "base_model": "yolov8n.pt",
    "epochs": 50,
    "batch_size": 16
  }'

# Response: {"training_id": "train_20241122_153045"}
```

#### 5. Monitor Training
```bash
# Check progress
curl http://localhost:8000/api/training/status/train_20241122_153045

# Response:
# {
#   "status": "training",
#   "progress": 45.0,
#   "current_epoch": 23,
#   "total_epochs": 50
# }
```

#### 6. Inference New Video
```bash
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@new_video.mp4" \
  -F "warning_distance=5.0"

# Response: Full analysis with detections & warnings
```

---

## 📱 Frontend (Optional)

### Setup Frontend
```bash
# Về root directory
cd ..

# Install
npm install

# Run
npm run dev
```

**Frontend:** http://localhost:3000

---

## 🔧 Troubleshooting

### Port Already in Use
```bash
# Đổi port
uvicorn main:app --port 8001
```

### PyTorch Installation Error
```bash
# CPU only
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# CUDA (GPU)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### Database Error
```bash
# Reset database
rm adas.db
python3 main.py  # Tạo lại
```

### Import Error
```bash
# Verify installation
python3 -c "import torch; import cv2; from ultralytics import YOLO; print('OK')"
```

---

## 📊 System Requirements

### Minimum
- Python 3.8+
- 4GB RAM
- 5GB disk space

### Recommended
- Python 3.10+
- 8GB RAM
- GPU (CUDA)
- 10GB disk space

---

## 🎓 Demo cho Ban Giám Khảo

### Chuẩn Bị (5 phút)
```bash
# 1. Install
./install.sh

# 2. Run server
python3 main.py

# 3. Chuẩn bị 2-3 video test
# - dashcam.mp4 (để upload)
# - test.mp4 (để inference)
```

### Demo Flow (10-15 phút)

**1. Show Code (2 phút)**
```bash
# Show structure
tree -L 2

# Show modules
ls -la api/
ls -la models/
```

**2. Upload + Auto-Label (3 phút)**
```bash
# Upload video
curl -X POST "http://localhost:8000/api/upload/video" \
  -F "file=@dashcam.mp4" \
  -F "auto_label=true"

# Show progress
curl http://localhost:8000/api/upload/video/1/status

# Show stats
curl http://localhost:8000/api/dataset/stats
```

**3. Training (5 phút)**
```bash
# Start training
curl -X POST "http://localhost:8000/api/training/start" \
  -d '{"model_name": "demo_model", "epochs": 2}'

# Monitor (F5 vài lần)
curl http://localhost:8000/api/training/status/train_xxx
```

**4. Inference (3 phút)**
```bash
# Test video
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@test.mp4"

# Show detections + warnings
```

**5. Q&A (2 phút)**

---

## 🎊 READY!

**Files to Run:**
```
./install.sh          → Install
python3 main.py       → Run server
python3 scripts/test_apis.py  → Test
```

**URLs:**
- Backend: http://localhost:8000
- API Docs: http://localhost:8000/docs
- Frontend: http://localhost:3000 (optional)

**Docs:**
- QUICKSTART_TRAINING.md
- API_TRAINING_README.md
- ARCHITECTURE.md

---

**SHOW TIME! 🚀**
