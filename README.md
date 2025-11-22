# 🚗 ADAS Platform - Advanced Driver Assistance System

> Hệ thống hỗ trợ lái xe tiên tiến với AI real-time detection, driver monitoring, và analytics dashboard

[![FastAPI](https://img.shields.io/badge/FastAPI-0.104-green)](https://fastapi.tiangolo.com)
[![Next.js](https://img.shields.io/badge/Next.js-14-black)](https://nextjs.org)
[![Python](https://img.shields.io/badge/Python-3.10+-blue)](https://python.org)
[![SQL Server](https://img.shields.io/badge/SQL%20Server-2019+-red)](https://microsoft.com/sql-server)

---

## 📦 Project Structure

```
adas-platform/
├── app/                    # Next.js 14 Frontend (React)
│   ├── adas/              # ADAS detection page
│   ├── dashboard/         # Main dashboard
│   ├── driver-monitor/    # Driver monitoring
│   └── api/               # API routes (unused, moved to backend)
│
├── backend-python/         # FastAPI Backend (Python)
│   ├── main.py            # API server
│   ├── models.py          # SQLAlchemy models
│   ├── services.py        # Business logic
│   └── docs/              # Complete documentation
│
├── adas_system/           # ROS2/Python ADAS modules
│   ├── perception/        # Camera, LiDAR, BEV
│   ├── tracking/          # DeepSORT tracker
│   ├── prediction/        # Trajectory prediction
│   ├── control/           # Vehicle controller
│   └── decision/          # Safety state machine
│
├── model-worker/          # YOLO Inference Service
│   └── app.py             # FastAPI inference API
│
└── components/            # React UI components
```

---

## 🚀 Quick Start

### Option 1: Docker (Recommended)

```bash
# Clone repository
git clone https://github.com/buivanchuong9/ADAS.git
cd adas-platform

# Start full stack
docker-compose up -d

# Access
# Frontend: http://localhost:3000
# Backend API: http://localhost:8000/docs
```

### Option 2: Manual Setup

#### 1. Backend Setup (Python + SQL Server)

```bash
cd backend-python

# Windows
setup.bat

# Linux/Mac
./setup.sh

# Configure .env
cp .env.example .env
# Edit .env with your SQL Server credentials

# Create database
python seed.py

# Run server
python main.py
```

#### 2. Frontend Setup (Next.js)

```bash
# At root directory
npm install
# or
pnpm install

# Configure environment
# Update API base URL in app/api/* if needed

# Run dev server
npm run dev
```

#### 3. Model Worker (Optional)

```bash
cd model-worker
pip install -r requirements.txt
python app.py
```

---

## 📱 Application URLs

| Service | URL | Description |
|---------|-----|-------------|
| **Frontend** | http://localhost:3000 | Main web application |
| **Dashboard** | http://localhost:3000/dashboard | Analytics & statistics |
| **ADAS Page** | http://localhost:3000/adas | Real-time detection |
| **Driver Monitor** | http://localhost:3000/driver-monitor | Fatigue detection |
| **AI Assistant** | http://localhost:3000/ai-assistant | Chat assistant |
| **Backend API** | http://localhost:8000/docs | FastAPI Swagger UI |
| **API Health** | http://localhost:8000/health | Health check |

---

## 🏗️ System Architecture

```
┌─────────────────────┐      ┌──────────────────────┐      ┌─────────────────────┐
│  Frontend (Next.js) │ ───▶ │ Backend (FastAPI)    │ ───▶ │ Model Worker (YOLO) │
│  Port: 3000         │      │ Port: 8000           │      │ Port: 8001          │
│  React + Tailwind   │      │ Python + SQLAlchemy  │      │ YOLOv8 Inference    │
└─────────────────────┘      └──────────────────────┘      └─────────────────────┘
                                      │
                                      ▼
                             ┌──────────────────┐
                             │  SQL Server      │
                             │  Database        │
                             │  Port: 1433      │
                             └──────────────────┘
```

**Stack:**
- **Frontend**: Next.js 14 + React 19 + TypeScript + TailwindCSS
- **Backend**: FastAPI + SQLAlchemy + pyodbc
- **Database**: Microsoft SQL Server 2019+
- **ML**: YOLOv8/v5 + OpenCV + PyTorch
- **Communication**: REST API + WebSocket

---

## 📋 System Requirements

| Component | Requirement |
|-----------|-------------|
| **Node.js** | 18.0+ |
| **Python** | 3.10+ |
| **SQL Server** | 2019+ (Express/Developer/Standard) |
| **ODBC Driver** | 17 for SQL Server |
| **RAM** | 8GB minimum, 16GB recommended |
| **Disk** | 10GB free space |
| **Ports** | 3000, 8000, 8001, 1433 |

---

## 💡 Features

### ✅ Core Features
- **Real-time Object Detection** - YOLOv8/v5 models (15+ variants)
- **Driver Monitoring** - Fatigue & distraction detection
- **Multi-camera Support** - Webcam, IP camera, smartphone
- **Trip Management** - Track journeys with events
- **Analytics Dashboard** - Real-time statistics & charts
- **Event System** - Safety alerts & notifications
- **AI Assistant** - Chat-based help system
- **WebSocket Streaming** - Low-latency video inference

### 📊 Detection Capabilities
- Vehicle detection (cars, trucks, buses)
- Pedestrian detection
- Traffic sign recognition
- Lane detection
- Driver pose estimation
- Facial recognition
- License plate detection (optional)

---

## 🔧 Configuration

### Backend Configuration

Edit `backend-python/.env`:

```env
# SQL Server
SQL_SERVER=localhost
SQL_DATABASE=ADAS_DB
SQL_USERNAME=sa
SQL_PASSWORD=YourPassword

# Model Worker
MODEL_WORKER_URL=http://localhost:8001

# CORS
ALLOWED_ORIGINS=http://localhost:3000

# Server
PORT=8000
```

### Frontend Configuration

Update API endpoints in `app/api/*/route.ts` if needed:

```typescript
const API_BASE = "http://localhost:8000";
```

---

## 🧪 Testing

### Backend Tests
```bash
cd backend-python
pytest -v
```

### API Testing
```bash
# Health check
curl http://localhost:8000/health

# Get cameras
curl http://localhost:8000/api/cameras/list

# API docs
open http://localhost:8000/docs
```

---

## 📚 Documentation

- **Backend**: [backend-python/README.md](backend-python/README.md)
- **Migration Guide**: [backend-python/MIGRATION_GUIDE.md](backend-python/MIGRATION_GUIDE.md)
- **Windows Service**: [backend-python/WINDOWS_SERVICE.md](backend-python/WINDOWS_SERVICE.md)
- **Docker**: [backend-python/DOCKER.md](backend-python/DOCKER.md)
- **Testing**: [backend-python/TESTING.md](backend-python/TESTING.md)
- **Quick Start**: [backend-python/QUICKSTART.md](backend-python/QUICKSTART.md)

---

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| **Port already in use** | Kill process: `lsof -i :8000` then `kill -9 [PID]` |
| **SQL Server connection failed** | 1. Check SQL Server running<br>2. Verify credentials in `.env`<br>3. Enable TCP/IP in SQL Config Manager |
| **ODBC Driver not found** | Install: https://aka.ms/downloadmsodbcsql |
| **Module not found (Python)** | `pip install -r requirements.txt` |
| **npm install fails** | `npm install --legacy-peer-deps` |
| **Database not exist** | Run `python backend-python/seed.py` |

---
---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📝 License

This project is for educational and research purposes.

---

## 👤 Author

**Bui Van Chuong**

- GitHub: [@buivanchuong9](https://github.com/buivanchuong9)
- Repository: [ADAS Platform](https://github.com/buivanchuong9/ADAS)

---

## 🙏 Acknowledgments

- **YOLOv8** - Ultralytics
- **FastAPI** - Sebastián Ramírez
- **Next.js** - Vercel
- **SQL Server** - Microsoft
- **React** - Meta

---

**⭐ Star this repo if you find it useful!**

*Built with ❤️ for safer driving*

- ⚡ **Cần tốc độ cao** (live stream): YOLOv8 Nano
- 🎯 **Cần độ chính xác cao**: YOLOv8 Large hoặc Faster RCNN
- 👤 **Phát hiện tài xế mệt mỏi**: YOLOv8 Pose
- 🚙 **Biển số xe**: License Plate Recognition
- 🛑 **Biển báo**: Traffic Sign Detector

### 2. Live Detection (Phát hiện thời gian thực)
- Mở: http://localhost:3000/live
- Cho phép truy cập camera
- Đặt vật thể trước camera
- Xem detection boxes realtime

### 3. Dashboard (Analytics)
- Mở: http://localhost:3000/dashboard
- Xem recent events
- Xem inference results
- Data tự sync từ Firebase

### 4. AI Assistant (Trợ lý AI)
- Mở: http://localhost:3000/ai-assistant
- Chat về lái xe an toàn
- Nhận tư vấn từ AI

### 5. Driver Monitor (Giám sát tài xế)
- Mở: http://localhost:3000/driver-monitor
- Phát hiện mệt mỏi
- Cảnh báo an toàn

---

## 🧪 Kiểm tra setup

Chạy verification script:
```bash
# Windows
check.bat

# macOS/Linux
bash check.sh
```

Kết quả mong đợi:
```
✓ Node.js: v18.x.x
✓ .NET SDK: 8.x.x
✓ Python: 3.11.x
✓ npm packages installed
✓ Python packages installed
✓ Firebase service account found
```

---

## 🎯 Tips & Tricks

💡 **Monitor logs**: Check terminal output khi services đang chạy
💡 **Clear cache**: `rm -rf .next && npm install --legacy-peer-deps`
💡 **Stop service**: Close that terminal window
💡 **Different port**: Edit uvicorn/dotnet/npm commands
💡 **Debug mode**: Add `--debug` flag khi chạy services

---

## 📝 Các lệnh hữu ích

```bash
# Cài lại dependencies
npm install --legacy-peer-deps --force
cd model-worker && pip install -r requirements.txt --force-reinstall

# Xóa cache
rm -rf .next node_modules __pycache__ bin obj
npm install

# Check ports
lsof -i :3000
lsof -i :5000
lsof -i :8000

# Kill process on port
kill -9 $(lsof -ti:3000)
```

---

## 🚀 Next Steps

1. ✅ Run `setup.sh` or `setup.bat`
2. ✅ Add `backend/firebase-service-account.json`
3. ✅ Run `run.sh` or `run.ps1`
4. ✅ Open http://localhost:3000/dashboard
5. 🎉 Enjoy!

---

**Questions?** Check terminal logs or verify Firebase credentials.

**Ready to deploy?** Project is production-ready with Firebase backend!

Made with ❤️ for ADAS Platform
