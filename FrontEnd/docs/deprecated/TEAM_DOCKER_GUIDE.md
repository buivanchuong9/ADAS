# 🚀 ADAS Platform - Hướng Dẫn Docker Cho Team

## 📋 Mục Lục
1. [Giới Thiệu](#giới-thiệu)
2. [Yêu Cầu Hệ Thống](#yêu-cầu-hệ-thống)
3. [Cài Đặt Lần Đầu](#cài-đặt-lần-đầu)
4. [Chạy Development Mode](#chạy-development-mode)
5. [Chạy Production Mode](#chạy-production-mode)
6. [Các Lệnh Hữu Ích](#các-lệnh-hữu-ích)
7. [Troubleshooting](#troubleshooting)
8. [Best Practices](#best-practices)

---

## 🎯 Giới Thiệu

ADAS Platform giờ đã được **tối ưu hóa** và **tách riêng** thành 2 container:
- 🎨 **Frontend**: Next.js 16 (Port 3000)
- 🧠 **Backend**: FastAPI + YOLOv11 (Port 8000)

### ✅ Ưu Điểm Của Kiến Trúc Mới
- ⚡ **Build nhanh hơn** với layer caching
- 🔥 **Hot reload** cho cả frontend và backend
- 🔒 **Isolation** - service riêng biệt, dễ debug
- 📦 **Easy deployment** - pull và chạy ngay
- 👥 **Team collaboration** - ai cũng có môi trường giống nhau

---

## 💻 Yêu Cầu Hệ Thống

### Bắt Buộc
- **Docker Desktop**: >= 24.0
- **Docker Compose**: >= 2.20
- **RAM**: >= 8GB (khuyến nghị 16GB)
- **Disk**: >= 20GB trống

### Khuyến Nghị (cho Production)
- **NVIDIA GPU**: GTX 1060 trở lên
- **CUDA**: >= 11.8
- **NVIDIA Docker Runtime**: Installed

### Kiểm Tra Docker
```bash
# Kiểm tra Docker version
docker --version
docker-compose --version

# Kiểm tra Docker đang chạy
docker info

# Kiểm tra GPU (nếu có)
nvidia-smi
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

## 🛠️ Cài Đặt Lần Đầu

### 1. Clone Repository
```bash
git clone https://github.com/buivanchuong9/ADAS.git
cd ADAS
```

### 2. Kiểm Tra File Cấu Trúc
Đảm bảo các file sau tồn tại:
```
✅ frontend.Dockerfile
✅ backend-python/backend.Dockerfile
✅ docker-compose.dev.yml
✅ docker-compose.prod.yml
✅ .dockerignore
✅ backend-python/.dockerignore
```

### 3. Cấu Hình Environment (Optional)
```bash
# Tạo file .env nếu cần
cp backend-python/.env.example backend-python/.env
# Chỉnh sửa các biến môi trường nếu cần
```

---

## 🔥 Chạy Development Mode

### Quick Start (Recommended)
```bash
# Chạy 1 lệnh - done!
./dev-start.sh
```

### Manual Start
```bash
# Build và start containers
docker-compose -f docker-compose.dev.yml up --build -d

# Xem logs
docker-compose -f docker-compose.dev.yml logs -f
```

### Access URLs
- 🌐 Frontend: http://localhost:3000
- 🔌 Backend API: http://localhost:8000
- 📚 API Docs: http://localhost:8000/docs

### Hot Reload 🔥
- **Frontend**: Edit files trong `app/`, `components/`, `lib/` → Auto refresh
- **Backend**: Edit files trong `backend-python/` → Auto reload

### Các Lệnh Development
```bash
# Xem logs tất cả services
./dev-logs.sh

# Xem logs của 1 service
./dev-logs.sh frontend
./dev-logs.sh backend

# Restart services
./dev-restart.sh              # Restart all
./dev-restart.sh frontend     # Restart frontend only

# Stop services
./dev-stop.sh

# Clean up (xóa volumes, data)
./dev-clean.sh
```

---

## 🚀 Chạy Production Mode

### Quick Start
```bash
# Yêu cầu GPU!
./prod-start.sh
```

### Manual Start
```bash
docker-compose -f docker-compose.prod.yml up --build -d
```

### Các Lệnh Production
```bash
# Xem logs
./prod-logs.sh

# Restart
./prod-restart.sh

# Stop
./prod-stop.sh
```

---

## 🔧 Các Lệnh Hữu Ích

### Docker Compose Basics
```bash
# Xem status của containers
docker-compose -f docker-compose.dev.yml ps

# Stop và xóa containers
docker-compose -f docker-compose.dev.yml down

# Stop, xóa containers + volumes
docker-compose -f docker-compose.dev.yml down -v

# Rebuild 1 service
docker-compose -f docker-compose.dev.yml build frontend
docker-compose -f docker-compose.dev.yml up -d frontend

# Exec vào container
docker exec -it adas-frontend-dev sh
docker exec -it adas-backend-dev bash
```

### Debug Commands
```bash
# Xem logs realtime
docker logs -f adas-frontend-dev
docker logs -f adas-backend-dev

# Xem resource usage
docker stats

# Inspect container
docker inspect adas-backend-dev

# Xem networks
docker network ls
docker network inspect adas-network
```

### Database Commands
```bash
# Access SQLite database
docker exec -it adas-backend-dev sqlite3 /app/data/adas.db

# Backup database
docker cp adas-backend-dev:/app/data/adas.db ./backup-adas.db

# Restore database
docker cp ./backup-adas.db adas-backend-dev:/app/data/adas.db
```

### Clean Up
```bash
# Xóa tất cả containers đã stop
docker container prune -f

# Xóa tất cả images không dùng
docker image prune -a -f

# Xóa tất cả volumes không dùng
docker volume prune -f

# Xóa tất cả (CAREFUL!)
docker system prune -a --volumes -f
```

---

## 🐛 Troubleshooting

### 1. Container không start được

**Lỗi: Port already in use**
```bash
# Tìm process đang dùng port
lsof -i :3000
lsof -i :8000

# Kill process
kill -9 <PID>

# Hoặc đổi port trong docker-compose.yml
ports:
  - "3001:3000"  # Frontend
  - "8001:8000"  # Backend
```

**Lỗi: Cannot connect to Docker daemon**
```bash
# Khởi động Docker Desktop
# macOS: Mở Docker Desktop app
# Linux: sudo systemctl start docker
```

### 2. Build quá lâu

**Giải pháp:**
```bash
# Sử dụng BuildKit (nhanh hơn)
export DOCKER_BUILDKIT=1
docker-compose -f docker-compose.dev.yml build

# Build không cache (khi cần)
docker-compose -f docker-compose.dev.yml build --no-cache

# Build parallel
docker-compose -f docker-compose.dev.yml build --parallel
```

### 3. GPU không detect được

**Kiểm tra:**
```bash
# Trong container
docker exec -it adas-backend-dev nvidia-smi

# Nếu lỗi, cài NVIDIA Container Toolkit
# Ubuntu/Debian:
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### 4. Frontend không kết nối được Backend

**Kiểm tra:**
```bash
# Ping backend từ frontend container
docker exec -it adas-frontend-dev ping backend

# Check network
docker network inspect adas-network

# Xem biến môi trường
docker exec -it adas-frontend-dev env | grep API_URL
```

**Fix:**
Đảm bảo `NEXT_PUBLIC_API_URL=http://backend:8000` trong docker-compose

### 5. Hot Reload không hoạt động

**Frontend:**
```bash
# Kiểm tra volumes đã mount đúng chưa
docker inspect adas-frontend-dev | grep -A 10 Mounts

# Restart container
./dev-restart.sh frontend
```

**Backend:**
```bash
# Kiểm tra uvicorn có flag --reload
docker logs adas-backend-dev | grep reload

# Restart
./dev-restart.sh backend
```

### 6. Out of Memory

**Giải pháp:**
```bash
# Tăng memory cho Docker Desktop
# macOS: Preferences → Resources → Memory (set to 8GB+)

# Hoặc giảm resource limits trong docker-compose
deploy:
  resources:
    limits:
      memory: 4G
```

### 7. Model weights không load được

**Giải pháp:**
```bash
# Download YOLO weights thủ công
cd backend-python/ai_models/weights
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Hoặc mount từ host
volumes:
  - ./backend-python/ai_models/weights:/app/ai_models/weights
```

---

## 📚 Best Practices

### 1. Development Workflow
```bash
# 1. Pull latest code
git pull origin main

# 2. Rebuild nếu có thay đổi dependencies
./dev-start.sh

# 3. Code và test với hot reload

# 4. Commit code
git add .
git commit -m "Your message"
git push origin your-branch
```

### 2. Khi Add Dependencies

**Frontend:**
```bash
# Add package
docker exec -it adas-frontend-dev npm install <package>

# Hoặc edit package.json rồi rebuild
./dev-restart.sh frontend
```

**Backend:**
```bash
# Add to requirements.txt
echo "new-package==1.0.0" >> backend-python/requirements.txt

# Rebuild
docker-compose -f docker-compose.dev.yml build backend
docker-compose -f docker-compose.dev.yml up -d backend
```

### 3. Database Migrations
```bash
# Run migrations
docker exec -it adas-backend-dev alembic upgrade head

# Create new migration
docker exec -it adas-backend-dev alembic revision --autogenerate -m "description"
```

### 4. Testing
```bash
# Frontend tests
docker exec -it adas-frontend-dev npm test

# Backend tests
docker exec -it adas-backend-dev pytest
```

### 5. Monitoring
```bash
# Resource usage
docker stats

# Logs với timestamps
docker-compose -f docker-compose.dev.yml logs -f -t

# Export logs
docker logs adas-backend-dev > backend.log 2>&1
```

---

## 🎓 Các Câu Hỏi Thường Gặp

**Q: Tôi nên dùng dev hay prod?**
- Development: Khi coding, testing, debugging
- Production: Khi deploy lên server, demo cho khách hàng

**Q: Build lần đầu mất bao lâu?**
- Khoảng 5-10 phút tùy tốc độ mạng (download images, packages)
- Lần sau chỉ mất 1-2 phút nhờ caching

**Q: Tôi có cần GPU không?**
- Dev: Không bắt buộc (dùng CPU cũng được)
- Prod: Khuyến nghị có GPU để inference nhanh

**Q: Làm sao share code với team?**
```bash
# Push code lên Git
git push origin main

# Team pull về và chạy
git pull origin main
./dev-start.sh
```

**Q: Database data có mất khi restart không?**
- Không! Data được lưu trong Docker volumes
- Chỉ mất khi chạy `./dev-clean.sh` hoặc `docker-compose down -v`

---

## 📞 Support

Nếu gặp vấn đề:
1. Check [Troubleshooting](#troubleshooting) section
2. Xem logs: `./dev-logs.sh`
3. Google error message
4. Hỏi team lead hoặc tạo issue trên GitHub

---

## 🎉 Happy Coding!

Bây giờ bạn đã sẵn sàng! Chỉ cần:
```bash
./dev-start.sh
```

Và bắt đầu code! 🚀
