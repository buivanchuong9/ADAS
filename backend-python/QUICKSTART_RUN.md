# 🚀 ADAS Backend - Quick Start

## Chạy Server Bằng 1 Lệnh (Cross-Platform)

### ✅ Windows, macOS, Linux - Tất cả dùng chung 1 lệnh:

```bash
cd backend-python
python run.py
```

Hoặc:

```bash
cd backend-python
python3 run.py
```

**Script sẽ tự động:**
- ✅ Kiểm tra Python 3.8+
- ✅ Cài đặt dependencies (nếu chưa có)
- ✅ Dừng server cũ (nếu đang chạy)
- ✅ Khởi động server mới

---

## 📋 Các Cách Chạy Khác

### 1. **Universal Script (Khuyên dùng)** - Windows + macOS + Linux
```bash
python run.py
```

### 2. **Bash Script** - macOS + Linux only
```bash
./start.sh
```

### 3. **Windows BAT** - Windows only
```cmd
deploy-windows.bat
```

### 4. **Manual** - Chạy trực tiếp
```bash
pip install -r requirements.txt
python main.py
```

---

## 🌐 Sau Khi Chạy

Server sẽ chạy tại: **http://localhost:8000**

**Endpoints quan trọng:**
- 📖 **API Docs**: http://localhost:8000/docs
- 💚 **Health Check**: http://localhost:8000/health
- 🎯 **Alerts**: http://localhost:8000/api/alerts/latest
- 📦 **Dataset**: http://localhost:8000/api/dataset/stats
- 🔍 **Inference**: http://localhost:8000/api/inference/video

---

## 🧪 Test Server

```bash
# Health check
curl http://localhost:8000/health

# Alerts stats
curl http://localhost:8000/api/alerts/stats

# Dataset stats
curl http://localhost:8000/api/dataset/stats
```

---

## 🛑 Dừng Server

### Nếu chạy foreground:
```
Ctrl + C
```

### Nếu chạy background:
**Windows:**
```powershell
Get-Process -Name python* | Where-Object {$_.Path -like '*main.py*'} | Stop-Process
```

**macOS/Linux:**
```bash
pkill -f "python3 main.py"
# Hoặc kill theo port
lsof -ti:8000 | xargs kill
```

---

## 📦 Requirements

- Python 3.8+
- Các dependencies trong `requirements.txt` (tự động cài)

---

## 🔧 Troubleshooting

### Lỗi: "Address already in use"
```bash
# Windows
powershell -Command "(Get-NetTCPConnection -LocalPort 8000).OwningProcess | Stop-Process -Force"

# macOS/Linux
lsof -ti:8000 | xargs kill -9
```

### Lỗi: "ModuleNotFoundError"
```bash
pip install -r requirements.txt
```

### Lỗi: "Permission denied"
```bash
# macOS/Linux - Cấp quyền execute
chmod +x run.py
chmod +x start.sh
```

---

## 🎯 Phase 1 Features (TTC + Voice Alerts)

Xem chi tiết tại: [PHASE1_TTC_ALERTS.md](PHASE1_TTC_ALERTS.md)

**Test TTC computation:**
```bash
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@test.mp4" \
  -F "compute_ttc=true" \
  -F "create_voice_alerts=true"
```

---

## 📚 Documentation

- **PHASE1_TTC_ALERTS.md** - TTC computation & Voice alerts
- **API_REFERENCE.md** - API endpoints chi tiết
- **QUICKSTART.md** - Hướng dẫn deploy
- **TESTING.md** - Test cases

---

## ✨ Features

✅ **AI Models:**
- YOLOv8 - Vehicle detection
- YOLOP - Lane detection  
- MiDaS - Depth estimation
- TTC Computer - Time-to-collision
- Voice Alerts - pyttsx3 TTS

✅ **APIs:**
- Upload & Auto-labeling
- Inference (real-time video analysis)
- Training (YOLO model training)
- Dataset management
- Alerts (TTC warnings)

✅ **Database:**
- SQLite (development)
- SQL Server (production - Windows)
