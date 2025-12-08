# 🚗 HƯỚNG DẪN CÀI ĐẶT VÀ SỬ DỤNG HỆ THỐNG ADAS

## 📋 Mục Lục
1. [Giới Thiệu](#giới-thiệu)
2. [Yêu Cầu Hệ Thống](#yêu-cầu-hệ-thống)
3. [Cài Đặt](#cài-đặt)
4. [Chạy Dự Án](#chạy-dự-án)
5. [Cấu Trúc Dự Án](#cấu-trúc-dự-án)
6. [Tính Năng](#tính-năng)
7. [API Documentation](#api-documentation)
8. [Khắc Phục Sự Cố](#khắc-phục-sự-cố)

---

## 🎯 Giới Thiệu

**Hệ Thống Hỗ Trợ Lái Xe Nâng Cao (ADAS)** là một nền tảng AI sử dụng YOLOv11 để:

- ✅ Phát hiện phương tiện, người đi bộ, biển báo giao thông
- ✅ Cảnh báo va chạm thời gian thực
- ✅ Phát hiện làn đường
- ✅ Giám sát hành vi lái xe
- ✅ Thu thập và huấn luyện dữ liệu tự động
- ✅ WebSocket streaming cho camera trực tiếp

**Công nghệ sử dụng:**
- 🧠 **Backend**: FastAPI + Python 3.11 + YOLOv11
- 🎨 **Frontend**: Next.js 14 + TypeScript + Tailwind CSS
- 🐳 **Deploy**: Docker + Docker Compose
- 💾 **Database**: SQLite / SQL Server

---

## 💻 Yêu Cầu Hệ Thống

### Phần Cứng Tối Thiểu
- **CPU**: Intel Core i5 hoặc tương đương
- **RAM**: 8GB (khuyến nghị 16GB)
- **Ổ cứng**: 10GB trống
- **GPU** (tùy chọn): NVIDIA GPU với CUDA để tăng tốc

### Phần Mềm
- **Docker Desktop** (macOS, Windows, Linux)
  - macOS: [Tải Docker Desktop](https://www.docker.com/products/docker-desktop/)
  - Windows: [Tải Docker Desktop](https://www.docker.com/products/docker-desktop/)
  - Linux: [Cài đặt Docker Engine](https://docs.docker.com/engine/install/)

**HOẶC** nếu không dùng Docker:

- **Node.js** 18+ và npm/pnpm
- **Python** 3.11+
- **Git**

---

## 🚀 Cài Đặt

### Phương Án 1: Sử Dụng Docker (Khuyến Nghị) ⭐

Docker đảm bảo dự án chạy giống hệt nhau trên mọi máy tính.

#### Bước 1: Clone Repository

```bash
git clone https://github.com/buivanchuong9/ADAS.git
cd adas-platform
```

#### Bước 2: Chạy Backend với Docker

```bash
cd backend-python

# Build Docker image (chỉ cần làm 1 lần)
docker compose -f docker-compose.cross-platform.yml build

# Khởi động backend
docker compose -f docker-compose.cross-platform.yml up
```

**Hoặc dùng script nhanh:**

```bash
# macOS/Linux
./start-docker-dev.sh

# Windows
docker-start.bat
```

Backend sẽ chạy tại: `http://localhost:8080`

#### Bước 3: Chạy Frontend

Mở terminal mới:

```bash
# Quay lại thư mục gốc
cd ..

# Cài đặt dependencies
npm install
# hoặc
pnpm install

# Chạy frontend
npm run dev
```

Frontend sẽ chạy tại: `http://localhost:3000`

---

### Phương Án 2: Cài Đặt Thủ Công (Không Dùng Docker)

#### Bước 1: Clone Repository

```bash
git clone https://github.com/buivanchuong9/ADAS.git
cd adas-platform
```

#### Bước 2: Cài Đặt Backend

```bash
cd backend-python

# Tạo môi trường ảo Python
python3 -m venv venv

# Kích hoạt môi trường ảo
# macOS/Linux:
source venv/bin/activate
# Windows:
venv\Scripts\activate

# Cài đặt thư viện Python
pip install -r requirements.txt

# Tải model YOLOv11
bash download_yolo11.sh
# hoặc Windows:
# python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"

# Khởi động backend
python main.py
```

Backend sẽ chạy tại: `http://localhost:8080`

#### Bước 3: Cài Đặt Frontend

Mở terminal mới:

```bash
# Quay lại thư mục gốc
cd ..

# Cài đặt dependencies
npm install
# hoặc
pnpm install

# Chạy frontend
npm run dev
```

Frontend sẽ chạy tại: `http://localhost:3000`

---

## 🎮 Chạy Dự Án

### Chạy Cả Backend và Frontend Cùng Lúc

```bash
# Từ thư mục gốc
npm run dev
```

Lệnh này sẽ chạy đồng thời:
- Backend Python (port 8080)
- Frontend Next.js (port 3000)

### Chạy Riêng Lẻ

```bash
# Chỉ chạy Frontend
npm run dev:fe

# Chỉ chạy Backend
npm run dev:be
```

### Dừng Server

**Docker:**
```bash
# Dừng backend
cd backend-python
docker compose -f docker-compose.cross-platform.yml down

# hoặc dùng script
./stop-docker.sh  # macOS/Linux
stop-docker.bat   # Windows
```

**Thủ công:**
- Nhấn `Ctrl + C` trong terminal đang chạy

---

## 📁 Cấu Trúc Dự Án

```
adas-platform/
├── 📱 Frontend (Next.js)
│   ├── app/                      # Next.js App Router
│   │   ├── page.tsx              # Trang chủ
│   │   ├── adas/                 # Trang ADAS detection
│   │   ├── dashboard/            # Dashboard tổng quan
│   │   ├── analytics/            # Phân tích dữ liệu
│   │   └── api/                  # API routes
│   │
│   ├── components/               # React components
│   │   ├── ui/                   # UI components (shadcn)
│   │   ├── adas/                 # ADAS-specific components
│   │   ├── sidebar.tsx           # Navigation sidebar
│   │   └── ...
│   │
│   ├── lib/                      # Utilities
│   │   ├── api-client.ts         # API client
│   │   ├── translations.ts       # Đa ngôn ngữ
│   │   └── utils.ts              # Helpers
│   │
│   └── styles/                   # CSS/Tailwind
│
├── 🧠 Backend (FastAPI)
│   ├── backend-python/
│   │   ├── main.py               # FastAPI entry point
│   │   ├── adas_backend.py       # ADAS core logic
│   │   ├── database.py           # Database models
│   │   ├── models.py             # SQLAlchemy models
│   │   ├── schemas.py            # Pydantic schemas
│   │   │
│   │   ├── api/                  # API endpoints
│   │   │   ├── websocket_inference.py  # WebSocket streaming
│   │   │   ├── detections/       # Detection API
│   │   │   ├── upload/           # File upload
│   │   │   ├── training/         # Model training
│   │   │   └── ...
│   │   │
│   │   ├── core/                 # Core infrastructure
│   │   │   ├── config.py         # Configuration
│   │   │   ├── logging_config.py # Logging setup
│   │   │   └── responses.py      # API responses
│   │   │
│   │   ├── services/             # Business logic
│   │   │   ├── enhanced_services.py
│   │   │   └── realtime_aggregator.py
│   │   │
│   │   ├── ai_models/            # AI models
│   │   │   ├── yolo11_detector.py    # YOLOv11 wrapper
│   │   │   ├── adas_unified.py       # Unified ADAS model
│   │   │   ├── yolo_trainer.py       # Training logic
│   │   │   └── weights/              # Model files (.pt)
│   │   │
│   │   ├── dataset/              # Training data
│   │   │   ├── raw/              # Raw images
│   │   │   ├── labels/           # YOLO labels
│   │   │   └── auto_collected/   # Auto-collected
│   │   │
│   │   └── logs/                 # Application logs
│
├── 🐳 Docker
│   ├── Dockerfile                # Frontend Dockerfile
│   ├── docker-compose.yml        # Full stack compose
│   └── backend-python/
│       ├── Dockerfile.cross-platform  # Backend Dockerfile
│       └── docker-compose.cross-platform.yml
│
├── 📚 Documentation
│   ├── HUONG_DAN.md             # File này (Tiếng Việt)
│   ├── README.md                # English README
│   ├── QUICK_START.md           # Quick start guide
│   └── docs/deprecated/         # Tài liệu cũ
│
└── ⚙️ Configuration
    ├── package.json             # Node.js dependencies
    ├── tsconfig.json            # TypeScript config
    ├── next.config.mjs          # Next.js config
    ├── tailwind.config.js       # Tailwind CSS
    └── .env.local.example       # Environment variables
```

---

## ✨ Tính Năng

### 1. 🎥 Phát Hiện Thời Gian Thực (WebSocket)

- Kết nối camera/webcam
- Stream video qua WebSocket
- Phát hiện: xe, người, biển báo, làn đường
- Hiển thị bounding boxes và confidence scores
- FPS counter

**Sử dụng:**
1. Vào trang `/adas` hoặc `/models-webcam`
2. Cho phép truy cập camera
3. Nhấn "Bắt đầu phát hiện"

### 2. 📊 Dashboard & Analytics

- Tổng quan thống kê
- Biểu đồ phát hiện theo thời gian
- Top đối tượng phát hiện nhiều nhất
- Lịch sử cảnh báo

**Truy cập:** `http://localhost:3000/dashboard`

### 3. 🚨 Hệ Thống Cảnh Báo

- Cảnh báo va chạm phía trước
- Phát hiện xe vượt làn
- Cảnh báo người đi bộ
- Âm thanh cảnh báo

### 4. 📁 Quản Lý Dataset

- Upload ảnh/video training
- Tự động gán nhãn (auto-labeling)
- Thu thập dữ liệu tự động
- Export dataset định dạng YOLO

**Truy cập:** `http://localhost:3000/data-collection`

### 5. 🧠 Huấn Luyện Model

- Train YOLOv11 với dữ liệu tùy chỉnh
- Monitor training progress
- Auto-reload model sau training

**API Endpoint:**
```bash
POST http://localhost:8080/train
```

### 6. 👤 Giám Sát Lái Xe

- Phát hiện buồn ngủ (yawning, eye closure)
- Cảnh báo mất tập trung
- Ghi nhận hành vi lái xe

**Truy cập:** `http://localhost:3000/driver-monitor`

---

## 🔌 API Documentation

### Base URL
```
http://localhost:8080
```

### WebSocket Endpoints

#### 1. Real-time Inference
```
ws://localhost:8080/ws/inference
```

**Gửi frame (Client → Server):**
```json
{
  "frame": "base64_encoded_image_data"
}
```

**Nhận kết quả (Server → Client):**
```json
{
  "detections": [
    {
      "class": "car",
      "confidence": 0.95,
      "bbox": [100, 150, 300, 400]
    }
  ],
  "fps": 30.5,
  "processing_time": 0.033
}
```

#### 2. Driver Monitoring
```
ws://localhost:8080/ws/monitor
```

### REST API Endpoints

#### Health Check
```http
GET /health
```

**Response:**
```json
{
  "status": "healthy",
  "message": "ADAS Backend API is running"
}
```

#### Get All Detections
```http
GET /api/detections
```

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "id": 1,
      "timestamp": "2025-12-01T10:30:00",
      "class_name": "car",
      "confidence": 0.95,
      "bbox_x": 100,
      "bbox_y": 150,
      "bbox_w": 200,
      "bbox_h": 250
    }
  ]
}
```

#### Upload Video/Image
```http
POST /api/upload
Content-Type: multipart/form-data

file: <binary>
```

#### Start Training
```http
POST /train
```

**Response:**
```json
{
  "status": "training",
  "message": "Training YOLOv11n...",
  "epoch": 5,
  "total_epochs": 50
}
```

#### Get Model Status
```http
GET /models/status
```

**Response:**
```json
{
  "yolo11n_loaded": true,
  "yolo11m_loaded": true,
  "total_training_images": 1500
}
```

---

## 🛠️ Cấu Hình

### Environment Variables

Tạo file `.env.local` từ `.env.local.example`:

```bash
cp .env.local.example .env.local
```

**Frontend (.env.local):**
```bash
# Backend API URL
NEXT_PUBLIC_API_URL=http://localhost:8080

# WebSocket URL
NEXT_PUBLIC_WS_URL=ws://localhost:8080
```

**Backend (.env):**
```bash
# Server
HOST=0.0.0.0
PORT=8080
DEBUG=true

# Database
DATABASE_URL=sqlite:///./adas.db

# Model Paths
WEIGHTS_DIR=/app/ai_models/weights
DATASET_DIR=/app/dataset
LOG_DIR=/app/logs
```

---

## 🐛 Khắc Phục Sự Cố

### 1. Port đã được sử dụng

**Lỗi:**
```
Error: Port 8080 already in use
```

**Giải pháp:**
```bash
# Tìm process đang dùng port
# macOS/Linux:
lsof -i :8080

# Windows:
netstat -ano | findstr :8080

# Kill process hoặc đổi port trong docker-compose
```

### 2. Docker container không start

**Kiểm tra logs:**
```bash
cd backend-python
docker compose -f docker-compose.cross-platform.yml logs
```

**Rebuild image:**
```bash
docker compose -f docker-compose.cross-platform.yml build --no-cache
docker compose -f docker-compose.cross-platform.yml up
```

### 3. WebSocket không kết nối

**Kiểm tra:**
1. Backend đã chạy: `curl http://localhost:8080/health`
2. Firewall không chặn port 8080
3. CORS settings trong backend

**Test WebSocket:**
```bash
# Dùng websocat (cài đặt: brew install websocat)
websocat ws://localhost:8080/ws/inference
```

### 4. Model không load được

**Tải lại YOLOv11:**
```bash
cd backend-python

# Thủ công
python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"

# Hoặc dùng script
bash download_yolo11.sh
```

### 5. Frontend không kết nối backend

**Kiểm tra:**
1. `.env.local` có đúng URL không
2. Backend đã chạy chưa
3. CORS enabled trong backend

**Test API:**
```bash
curl http://localhost:8080/health
```

### 6. Hot reload không hoạt động (Windows)

**Đảm bảo trong docker-compose.cross-platform.yml:**
```yaml
environment:
  - WATCHFILES_FORCE_POLLING=true
```

---

## 📖 Tài Liệu Thêm

- **API Full Documentation:** `http://localhost:8080/docs` (Swagger UI)
- **ReDoc:** `http://localhost:8080/redoc`
- **Tài liệu cũ:** `docs/deprecated/`

---

## 🤝 Đóng Góp

1. Fork repository
2. Tạo branch: `git checkout -b feature/tinh-nang-moi`
3. Commit: `git commit -m 'Thêm tính năng mới'`
4. Push: `git push origin feature/tinh-nang-moi`
5. Tạo Pull Request

---

## 📝 License

MIT License - Xem file `LICENSE` để biết thêm chi tiết.

---

## 📧 Liên Hệ

- **Repository:** https://github.com/buivanchuong9/ADAS
- **Issues:** https://github.com/buivanchuong9/ADAS/issues

---

## 🎓 Credits

- **YOLOv11:** [Ultralytics](https://github.com/ultralytics/ultralytics)
- **FastAPI:** [FastAPI](https://fastapi.tiangolo.com/)
- **Next.js:** [Next.js](https://nextjs.org/)
- **Shadcn UI:** [Shadcn UI](https://ui.shadcn.com/)

---

**Chúc bạn sử dụng thành công! 🚀**
