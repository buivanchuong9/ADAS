# 🚀 QUICK START - ADAS Training System

## Setup Nhanh (5 phút)

### 1. Install Dependencies

```bash
cd backend-python
pip3 install -r requirements.txt
```

### 2. Chạy Backend

```bash
python3 main.py
```

Server chạy tại: http://localhost:8000

### 3. Test APIs

```bash
# Test cơ bản
python3 scripts/test_apis.py

# Hoặc mở browser
open http://localhost:8000/docs
```

---

## 🎯 Demo Workflow

### Bước 1: Upload Video + Auto-Label

```bash
curl -X POST "http://localhost:8000/api/upload/video" \
  -F "file=@your_video.mp4" \
  -F "auto_label=true"
```

**Output:**
```json
{
  "video_id": 1,
  "status": "processing"
}
```

### Bước 2: Check Dataset Stats

```bash
curl http://localhost:8000/api/dataset/stats
```

**Output:**
```json
{
  "total_videos": 1,
  "labeled_videos": 1,
  "total_frames": 1200,
  "total_labels": 240,
  "auto_labeled_percentage": 100.0
}
```

### Bước 3: Train Model

```bash
curl -X POST "http://localhost:8000/api/training/start" \
  -H "Content-Type: application/json" \
  -d '{
    "model_name": "my_adas_model",
    "base_model": "yolov8n.pt",
    "epochs": 50,
    "batch_size": 16
  }'
```

**Output:**
```json
{
  "training_id": "train_20241122_153045",
  "status": "started"
}
```

### Bước 4: Monitor Training

```bash
curl http://localhost:8000/api/training/status/train_20241122_153045
```

**Output:**
```json
{
  "status": "training",
  "progress": 45.0,
  "current_epoch": 23,
  "total_epochs": 50
}
```

### Bước 5: Inference

```bash
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@test_video.mp4" \
  -F "warning_distance=5.0"
```

**Output:**
```json
{
  "frames": [...],
  "summary": {
    "total_vehicles_detected": 45,
    "total_warnings": 3
  }
}
```

---

## 📊 Cấu Trúc API

```
/api/upload/video          # Upload + auto-label
/api/upload/video/{id}/status

/api/inference/video       # Analyze video
/api/inference/image       # Analyze image

/api/training/start        # Start training
/api/training/status/{id}  # Check progress
/api/training/activate/{model_id}

/api/dataset/videos        # List videos
/api/dataset/stats         # Statistics
```

---

## 💡 Tips

### Tăng Tốc Training
- Dùng GPU: `pip install torch --index-url https://download.pytorch.org/whl/cu118`
- Giảm batch_size nếu out of memory
- Dùng model nhỏ hơn: `yolov8n` < `yolov8s` < `yolov8m`

### Tăng Accuracy
- Upload nhiều video (diverse dataset)
- Train nhiều epochs (50-100)
- Verify labels sau auto-labeling

---

## 🔧 Troubleshooting

### Port đã được sử dụng
```bash
# Đổi port trong config.py hoặc:
uvicorn main:app --port 8001
```

### Model không download được
```bash
# Download thủ công
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
mv yolov8n.pt models/
```

### Database error
```bash
# Reset database
rm adas.db
python3 main.py  # Sẽ tự tạo lại
```

---

## 📱 Frontend Integration

```javascript
// Upload video
const formData = new FormData();
formData.append('file', videoFile);
formData.append('auto_label', true);

const response = await fetch('http://localhost:8000/api/upload/video', {
  method: 'POST',
  body: formData
});

const { video_id } = await response.json();

// Check progress
const checkProgress = async () => {
  const res = await fetch(`http://localhost:8000/api/upload/video/${video_id}/status`);
  const { progress, status } = await res.json();
  
  if (status === 'labeled') {
    console.log('✅ Auto-labeling completed!');
  }
};
```

---

## 🎓 Demo Scenario

1. ✅ Upload 3-5 videos dashboard cam
2. ✅ Show auto-labeling progress
3. ✅ Show dataset statistics
4. ✅ Start training (2-5 epochs for demo)
5. ✅ Monitor training metrics
6. ✅ Test inference on new video
7. ✅ Show warnings (collision, lane departure)

**Thời gian demo: 10-15 phút**

---

## 📚 Resources

- API Docs: http://localhost:8000/docs
- Full Guide: [API_TRAINING_README.md](API_TRAINING_README.md)
- YOLOv8: https://docs.ultralytics.com/
- MiDaS: https://github.com/isl-org/MiDaS

---

**READY TO IMPRESS! 🎉**
