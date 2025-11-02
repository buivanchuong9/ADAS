# ADAS - Advanced Driver Assistance System Platform

## 🚀 Tính năng
- **AI Detection**: YOLOv8, YOLOv5, pose estimation với mô hình đa chiều
- **Realtime Processing**: WebSocket streaming video/ảnh realtime
- **Firebase Integration**: Lưu trữ tập trung, phân tích realtime
- **Dashboard Phân tích**: Xem sự kiện, cảnh báo, thống kê trực tiếp
- **Multi-Model Support**: Hỗ trợ chuyển đổi nhanh giữa các model AI

## 📋 Yêu cầu hệ thống

### Windows
- **Node.js** 16+ (https://nodejs.org/)
- **.NET SDK** 8.0+ (https://dotnet.microsoft.com/en-us/download)
- **Python** 3.9+ (https://www.python.org/downloads/)
- **Git** (https://git-scm.com/)

### macOS/Linux
```bash
# macOS (using Homebrew)
brew install node dotnet python@3.11 git

# Ubuntu/Debian
sudo apt-get install nodejs dotnet-sdk-8.0 python3.11 git
```

## 🛠️ Cài đặt

### Windows (Tự động)
```bash
# 1. Mở cmd hoặc PowerShell ở thư mục dự án
# 2. Chạy script cài đặt
install.bat
```

### Windows (Thủ công)
```bash
# Terminal 1: Frontend
npm install --legacy-peer-deps
npm run dev

# Terminal 2: Backend
cd backend
dotnet restore
dotnet run

# Terminal 3: Model Worker
cd model-worker
pip install -r requirements.txt
uvicorn app:app --host 0.0.0.0 --port 8000
```

### macOS/Linux
```bash
# Terminal 1: Frontend
npm install --legacy-peer-deps
npm run dev

# Terminal 2: Backend
cd backend
dotnet restore
dotnet run

# Terminal 3: Model Worker
cd model-worker
pip install -r requirements.txt
uvicorn app:app --host 0.0.0.0 --port 8000
```

## 🚀 Chạy ứng dụng

### Windows (Tự động - PowerShell)
```powershell
# Mở PowerShell ở thư mục dự án
PowerShell -ExecutionPolicy Bypass -File run.ps1
```

### Chạy từng thành phần riêng
```bash
# Terminal 1: Model Worker (Port 8000)
cd model-worker
uvicorn app:app --host 0.0.0.0 --port 8000

# Terminal 2: Backend (Port 5000)
cd backend
dotnet run

# Terminal 3: Frontend (Port 3000)
npm run dev
```

## 📱 Truy cập ứng dụng
- **Dashboard**: http://localhost:3000/dashboard
- **Live Detection**: http://localhost:3000/live
- **AI Assistant**: http://localhost:3000/ai-assistant
- **Model Worker Health**: http://localhost:8000/health

## 🏗️ Kiến trúc hệ thống
```
Frontend (Next.js/React) 
    ↓ (WebSocket/HTTP)
Backend (.NET)
    ↓ (HTTP)
Model Worker (FastAPI/Python)
    ↓
Firebase Firestore
```

## 📊 Luồng dữ liệu
1. **Frontend gửi frame** (camera/video) qua WebSocket → Backend
2. **Backend nhận frame** → gọi Model Worker (AI inference)
3. **Model Worker chạy AI** (YOLOv8, YOLOv5...) → trả kết quả
4. **Backend xử lý kết quả** → lưu sự kiện, cảnh báo
5. **Lưu Firebase Firestore** → Dashboard realtime cập nhật

## 🔧 Cấu hình

### Firebase Service Account
- Đặt file service account JSON tại: `backend/firebase-service-account.json`
- Tải từ: Firebase Console → Project Settings → Service Accounts

### Model Worker Configuration
- File: `model-worker/app.py`
- Hỗ trợ model: `yolov8n`, `yolov5s`, (mở rộng thêm)
- Thêm model mới: cập nhật `AVAILABLE_MODELS` dict

## 🚨 Gặp sự cố?

### npm error: ERESOLVE
```bash
npm install --legacy-peer-deps
```

### Python package error
```bash
pip install --upgrade pip
pip install -r model-worker/requirements.txt
```

### .NET build error
```bash
cd backend
dotnet clean
dotnet restore
dotnet build
```

### Port đã bị sử dụng
```bash
# Tìm process sử dụng port (ví dụ port 3000)
# Windows: netstat -ano | findstr :3000
# macOS/Linux: lsof -i :3000
```

## 📦 Dependency chính
- **Frontend**: Next.js 16, React 19, TailwindCSS, Firebase SDK
- **Backend**: .NET 8, Entity Framework, SignalR/WebSocket
- **Model Worker**: FastAPI, PyTorch, Ultralytics YOLOv8/v5
- **Database**: Firebase Firestore

## 🔐 Bảo mật
- Service account JSON được thêm vào `.gitignore`
- Firebase credentials được quản lý an toàn
- API communication qua HTTPS (production)

## 📄 License
MIT

## 👨‍💻 Tác giả
ADAS Platform Development Team
