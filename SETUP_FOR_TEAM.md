# 🚀 Hướng Dẫn Setup Cho Team Members

> **Dành cho**: Thành viên mới clone project từ GitHub lần đầu

## ⚠️ Vấn Đề Thường Gặp

Khi clone project từ GitHub và chạy `docker compose up --build`, bạn sẽ gặp lỗi vì **thiếu các file và thư mục** bị `.gitignore` loại trừ:

- ❌ File `.env` (backend)
- ❌ File `.env.local` (frontend)
- ❌ Thư mục `ai_models/weights/` (model files)
- ❌ Database `adas.db`
- ❌ Thư mục `logs/`, `dataset/`

---

## 📋 Checklist Setup (Làm Theo Thứ Tự)

### ✅ Bước 1: Kiểm Tra Yêu Cầu Hệ Thống

```bash
# 1. Docker Desktop phải được cài đặt và ĐANG CHẠY
docker --version
# Expected: Docker version 20.x.x trở lên

# 2. Node.js 18+ 
node --version
# Expected: v18.x.x hoặc cao hơn

# 3. pnpm (hoặc npm)
pnpm --version
# Nếu chưa có: npm install -g pnpm
```

### ✅ Bước 2: Clone Project

```bash
git clone <repository-url>
cd adas-platform
```

### ✅ Bước 3: Tạo File Environment (QUAN TRỌNG!)

#### 3.1. Backend Environment

```bash
cd backend-python

# Copy file mẫu
cp .env.example .env

# Hoặc tạo file .env với nội dung sau:
cat > .env << 'EOF'
# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=*

# Database
DATABASE_URL=sqlite:///./adas.db

# Model Weights Directory
WEIGHTS_DIR=/app/ai_models/weights

# Server Configuration
HOST=0.0.0.0
PORT=8000
DEBUG=False

# CORS Origins
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:5173
EOF

cd ..
```

#### 3.2. Frontend Environment

```bash
# Tạo file .env.local ở thư mục root
cat > .env.local << 'EOF'
NEXT_PUBLIC_API_URL=http://localhost:8000
EOF
```

### ✅ Bước 4: Tạo Thư Mục Cần Thiết

```bash
# Tạo các thư mục cho backend
cd backend-python

mkdir -p ai_models/weights
mkdir -p dataset/raw
mkdir -p dataset/labels
mkdir -p dataset/auto_collected
mkdir -p logs/alerts
mkdir -p adas_core/tests/unit
mkdir -p adas_core/tests/integration
mkdir -p adas_core/tests/scenarios

cd ..
```

### ✅ Bước 5: Cài Dependencies Frontend

```bash
# Ở thư mục root
pnpm install
# Hoặc: npm install
```

### ✅ Bước 6: Download YOLO Model (Tùy Chọn)

```bash
cd backend-python

# Nếu có script download
bash download_yolo11.sh

# Hoặc download thủ công từ:
# https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
# Đặt file vào: backend-python/ai_models/weights/

cd ..
```

### ✅ Bước 7: Khởi Động Docker Backend

```bash
cd backend-python

# Mở Docker Desktop trước!
docker compose up --build

# Đợi đến khi thấy:
# ✅ "Application startup complete"
# ✅ "Uvicorn running on http://0.0.0.0:8000"
```

**Lưu ý**: Lần đầu build sẽ mất 5-10 phút để download dependencies.

### ✅ Bước 8: Khởi Động Frontend (Terminal Mới)

```bash
# Mở terminal mới, ở thư mục root
pnpm run dev
# Hoặc: npm run dev

# Đợi đến khi thấy:
# ✅ "Ready in Xms"
# ✅ "Local: http://localhost:3000"
```

### ✅ Bước 9: Kiểm Tra Hệ Thống

```bash
# Test 1: Backend Health Check
curl http://localhost:8000/health
# Expected: {"status":"healthy","version":"3.0.0"}

# Test 2: Frontend
# Mở browser: http://localhost:3000
# Phải thấy giao diện ADAS Dashboard

# Test 3: API Docs
# Mở browser: http://localhost:8000/docs
# Phải thấy Swagger UI
```

---

## 🔧 Troubleshooting - Xử Lý Lỗi Thường Gặp

### ❌ Lỗi 1: "docker compose: command not found"

**Nguyên nhân**: Docker Desktop chưa cài hoặc chưa chạy

**Giải pháp**:
```bash
# macOS
open -a Docker

# Windows: Mở Docker Desktop từ Start Menu

# Đợi 30 giây rồi thử lại
docker compose up --build
```

### ❌ Lỗi 2: "Error: ENOENT: no such file or directory, open '.env.local'"

**Nguyên nhân**: Thiếu file `.env.local`

**Giải pháp**:
```bash
# Tạo file .env.local ở thư mục root
echo "NEXT_PUBLIC_API_URL=http://localhost:8000" > .env.local
```

### ❌ Lỗi 3: Docker build failed - "requirements.txt not found"

**Nguyên nhân**: Đang ở sai thư mục

**Giải pháp**:
```bash
# Phải cd vào backend-python trước
cd backend-python
docker compose up --build
```

### ❌ Lỗi 4: "Port 8000 already in use"

**Nguyên nhân**: Port 8000 đã được process khác sử dụng

**Giải pháp**:
```bash
# macOS/Linux
lsof -i :8000
kill -9 <PID>

# Windows
netstat -ano | findstr :8000
taskkill /PID <PID> /F

# Sau đó chạy lại
docker compose up --build
```

### ❌ Lỗi 5: "Cannot connect to Docker daemon"

**Nguyên nhân**: Docker Desktop chưa khởi động xong

**Giải pháp**:
```bash
# Đợi Docker Desktop khởi động hoàn toàn (icon không còn animation)
# Thử lại sau 1 phút
docker compose up --build
```

### ❌ Lỗi 6: Frontend không kết nối được Backend

**Nguyên nhân**: Sai URL trong `.env.local`

**Giải pháp**:
```bash
# Kiểm tra file .env.local
cat .env.local
# Phải có: NEXT_PUBLIC_API_URL=http://localhost:8000

# Nếu sai, sửa lại:
echo "NEXT_PUBLIC_API_URL=http://localhost:8000" > .env.local

# Restart frontend
# Ctrl+C rồi chạy lại: pnpm run dev
```

### ❌ Lỗi 7: "Module not found" khi chạy frontend

**Nguyên nhân**: Chưa cài dependencies

**Giải pháp**:
```bash
# Xóa node_modules và cài lại
rm -rf node_modules
pnpm install
# Hoặc: npm install

pnpm run dev
```

---

## 🎯 Script Tự Động (Recommended)

Thay vì làm thủ công, dùng script này:

```bash
# Tạo file setup-first-time.sh
cat > setup-first-time.sh << 'SCRIPT'
#!/bin/bash

echo "🚀 ADAS Platform - First Time Setup"
echo "===================================="

# 1. Check Docker
if ! docker --version &> /dev/null; then
    echo "❌ Docker chưa cài đặt. Vui lòng cài Docker Desktop!"
    exit 1
fi

# 2. Check Node.js
if ! node --version &> /dev/null; then
    echo "❌ Node.js chưa cài đặt. Vui lòng cài Node.js 18+!"
    exit 1
fi

echo "✅ Docker và Node.js đã cài đặt"

# 3. Create backend .env
echo "📝 Tạo backend .env..."
cd backend-python
if [ ! -f .env ]; then
    cp .env.example .env 2>/dev/null || cat > .env << 'EOF'
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=*
DATABASE_URL=sqlite:///./adas.db
WEIGHTS_DIR=/app/ai_models/weights
EOF
    echo "✅ Đã tạo backend-python/.env"
else
    echo "⚠️  backend-python/.env đã tồn tại"
fi

# 4. Create directories
echo "📁 Tạo thư mục cần thiết..."
mkdir -p ai_models/weights
mkdir -p dataset/{raw,labels,auto_collected}
mkdir -p logs/alerts
mkdir -p adas_core/tests/{unit,integration,scenarios}
echo "✅ Đã tạo các thư mục"

cd ..

# 5. Create frontend .env.local
echo "📝 Tạo frontend .env.local..."
if [ ! -f .env.local ]; then
    echo "NEXT_PUBLIC_API_URL=http://localhost:8000" > .env.local
    echo "✅ Đã tạo .env.local"
else
    echo "⚠️  .env.local đã tồn tại"
fi

# 6. Install frontend dependencies
echo "📦 Cài đặt frontend dependencies..."
if command -v pnpm &> /dev/null; then
    pnpm install
else
    npm install
fi

echo ""
echo "✅ SETUP HOÀN TẤT!"
echo ""
echo "🚀 Bước tiếp theo:"
echo "1. Mở Docker Desktop"
echo "2. cd backend-python && docker compose up --build"
echo "3. Terminal mới: pnpm run dev"
echo "4. Mở http://localhost:3000"
SCRIPT

# Cho phép execute
chmod +x setup-first-time.sh

# Chạy script
./setup-first-time.sh
```

---

## 📊 Kiểm Tra Sau Khi Setup

### ✅ Checklist Cuối Cùng

- [ ] Docker Desktop đang chạy
- [ ] Backend container đang chạy: `docker ps` (thấy `adas-backend`)
- [ ] Backend health check OK: `curl http://localhost:8000/health`
- [ ] Frontend đang chạy: http://localhost:3000
- [ ] API Docs accessible: http://localhost:8000/docs
- [ ] WebSocket test: Mở /adas page, bật camera

### 📁 Cấu Trúc Thư Mục Sau Setup

```
adas-platform/
├── .env.local                    ✅ (phải có)
├── backend-python/
│   ├── .env                      ✅ (phải có)
│   ├── ai_models/
│   │   └── weights/              ✅ (thư mục trống OK)
│   ├── dataset/                  ✅ (phải có)
│   ├── logs/                     ✅ (phải có)
│   ├── docker-compose.yml
│   ├── Dockerfile
│   └── requirements.txt
├── node_modules/                 ✅ (sau pnpm install)
└── package.json
```

---

## 🆘 Cần Trợ Giúp?

### Các Lệnh Debug Hữu Ích

```bash
# Xem Docker logs
cd backend-python
docker compose logs -f backend

# Xem container đang chạy
docker ps

# Restart Docker container
docker compose restart

# Rebuild từ đầu (nếu có lỗi cache)
docker compose down
docker compose build --no-cache
docker compose up

# Kiểm tra port đang sử dụng
lsof -i :8000
lsof -i :3000
```

### Liên Hệ

- Nếu vẫn gặp lỗi, gửi screenshot lỗi + output của:
  ```bash
  docker compose logs backend
  ```

---

**Last Updated**: 2025-11-30  
**Version**: 1.0  
**Tested on**: macOS, Windows 11, Ubuntu 22.04
