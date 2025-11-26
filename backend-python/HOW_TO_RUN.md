# 🎯 ADAS Backend - Cách Chạy Server

## ⚡ Khuyên Dùng: 1 LỆNH DUY NHẤT

### Windows 🪟
```cmd
cd backend-python
START.bat
```
Hoặc **double-click** file `START.bat`

### macOS / Linux 🍎🐧
```bash
cd backend-python
./START.sh
```

**✨ Tự động:**
- Cài đặt TẤT CẢ thư viện
- Kill server cũ
- Khởi động server mới

---

## 📁 Các File Chạy Server

| File | Platform | Mô tả |
|------|----------|-------|
| `START.bat` | Windows | Double-click hoặc chạy trong CMD |
| `START.sh` | macOS/Linux | Bash script với colors |
| `start.py` | Cross-platform | Python script universal |
| `run.py` | Cross-platform | Python script với kill process |

---

## 🚀 Tất Cả Cách Chạy

### 1️⃣ START Script (Tự động cài thư viện)
**Windows:**
```cmd
START.bat
```

**macOS/Linux:**
```bash
chmod +x START.sh
./START.sh
```

### 2️⃣ Python Universal Script
```bash
python start.py
# hoặc
python3 start.py
```

### 3️⃣ Run Script (Kill + Start)
```bash
python run.py
# hoặc  
python3 run.py
```

### 4️⃣ Manual (Không tự động cài)
```bash
# Cài thư viện trước
pip install -r requirements.txt

# Chạy server
python main.py
```

---

## 📊 Server URLs

Sau khi chạy, truy cập:

- 📖 **API Documentation**: http://localhost:8000/docs
- 💚 **Health Check**: http://localhost:8000/health
- 🎯 **Alerts API**: http://localhost:8000/api/alerts/latest
- 📦 **Dataset Stats**: http://localhost:8000/api/dataset/stats
- 🔍 **Inference**: http://localhost:8000/api/inference/video

---

## ✅ Test Server

```bash
curl http://localhost:8000/health
```

Kết quả:
```json
{"status":"healthy","service":"ADAS Backend","version":"2.0.0"}
```

---

## 🛑 Dừng Server

### Foreground (đang chạy ở terminal):
```
Ctrl + C
```

### Background:

**Windows CMD/PowerShell:**
```powershell
# Kill tất cả Python
taskkill /F /IM python.exe

# Hoặc kill theo port 8000
netstat -ano | findstr :8000
taskkill /PID <PID> /F
```

**macOS/Linux:**
```bash
# Kill process main.py
pkill -f "python3 main.py"

# Hoặc kill theo port
lsof -ti:8000 | xargs kill -9
```

---

## 🔧 Troubleshooting

### Lỗi: "Python not found"
**Windows:**
1. Tải Python: https://python.org
2. Tick "Add Python to PATH" khi cài
3. Restart terminal

**macOS:**
```bash
brew install python3
```

**Linux:**
```bash
sudo apt update
sudo apt install python3 python3-pip
```

### Lỗi: "Address already in use"
Port 8000 đang bị chiếm. Script `START.sh` / `START.bat` sẽ tự động kill.

Hoặc kill thủ công:
```bash
# macOS/Linux
lsof -ti:8000 | xargs kill -9

# Windows
netstat -ano | findstr :8000
taskkill /PID <PID> /F
```

### Lỗi: Cài thư viện thất bại
```bash
# Upgrade pip
python -m pip install --upgrade pip --user

# Cài lại
pip install -r requirements.txt
```

### Lỗi: "Permission denied" (macOS/Linux)
```bash
chmod +x START.sh
chmod +x start.py
chmod +x run.py
```

---

## 📦 Thư Viện Được Cài

Từ `requirements.txt`:

**API Framework:**
- FastAPI 0.104.1
- Uvicorn 0.24.0
- SQLAlchemy 2.0.36+

**AI/ML:**
- PyTorch 2.0.0+
- TorchVision 0.15.0+
- Ultralytics YOLOv8
- OpenCV 4.8.0+
- NumPy 1.24.0+

**ADAS Features:**
- pyttsx3 2.90+ (Text-to-Speech)
- MiDaS timm (Depth estimation)

---

## 🎯 Features

✅ Phase 1 (Completed):
- TTC Computation
- Voice Alerts (Vietnamese)
- Real-time Inference
- Auto-labeling

Xem chi tiết: [PHASE1_TTC_ALERTS.md](PHASE1_TTC_ALERTS.md)

---

## 📝 So Sánh Các Cách Chạy

| Cách | Tự động cài lib | Tự động kill | Platform | Khuyên dùng |
|------|----------------|--------------|----------|-------------|
| `START.bat` | ✅ | ✅ | Windows | ⭐⭐⭐ |
| `START.sh` | ✅ | ✅ | macOS/Linux | ⭐⭐⭐ |
| `start.py` | ✅ | ❌ | All | ⭐⭐ |
| `run.py` | ✅ | ✅ | All | ⭐⭐ |
| `python main.py` | ❌ | ❌ | All | ⭐ |

**Khuyên dùng:** `START.bat` (Windows) hoặc `START.sh` (macOS/Linux)

---

## 🚀 Quick Start (TL;DR)

**Windows:**
```
Double-click: START.bat
```

**macOS/Linux:**
```bash
./START.sh
```

**Done!** Server chạy tại http://localhost:8000/docs
