# 🐳 ADAS Backend - Docker Setup Guide

## 🚀 Quick Start (One Command)

Team members chỉ cần chạy **MỘT LỆNH Duy nhất**:

```bash
cd backend-python
docker compose up --build
```

Hoặc sử dụng script tự động:

```bash
cd backend-python
./docker-start.sh
```

## 📋 Yêu cầu

- ✅ Docker Desktop đã cài đặt
- ✅ Docker Desktop đang chạy
- ❌ **KHÔNG** cần cài Python
- ❌ **KHÔNG** cần cài pip/virtualenv
- ❌ **KHÔNG** cần cài bất kỳ dependencies nào

## 🏗️ Kiến trúc Docker

### 1. **Dockerfile**
```dockerfile
FROM python:3.11-slim
WORKDIR /app

# Install OpenCV dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code
COPY . .

# Expose port
EXPOSE 8000

# Run FastAPI with WebSocket support
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--ws", "websockets"]
```

### 2. **docker-compose.yml**
```yaml
services:
  backend:
    build: .
    container_name: adas-backend
    ports:
      - "8000:8000"
    volumes:
      - .:/app  # Hot-reload for development
      - ./ai_models/weights:/app/ai_models/weights
      - ./adas.db:/app/adas.db
    restart: always
```

## 🔧 Các lệnh Docker

### Khởi động Backend
```bash
# Lần đầu tiên (build image)
docker compose up --build

# Lần sau (dùng image đã build)
docker compose up

# Chạy background
docker compose up -d
```

### Dừng Backend
```bash
# Dừng containers
docker compose down

# Dừng và xóa volumes
docker compose down -v
```

### Xem logs
```bash
# Xem logs realtime
docker compose logs -f backend

# Xem logs 100 dòng cuối
docker compose logs --tail=100 backend
```

### Vào shell container
```bash
# Vào bash trong container
docker compose exec backend bash

# Chạy lệnh Python
docker compose exec backend python -c "import torch; print(torch.__version__)"
```

### Rebuild image
```bash
# Rebuild khi thay đổi requirements.txt
docker compose build --no-cache
docker compose up
```

## 🌐 WebSocket Support

WebSocket hoạt động hoàn toàn bình thường trong Docker:

**Từ Frontend:**
```javascript
const ws = new WebSocket('ws://localhost:8000/ws/inference')
ws.onopen = () => console.log('✅ WebSocket connected')
```

**Từ bên ngoài Docker:**
```python
import websockets
async with websockets.connect('ws://localhost:8000/ws/inference') as ws:
    await ws.send(json.dumps({"model_id": "yolo11n", "frame": "..."}))
```

## 📁 Volumes (Dữ liệu được persist)

Docker ánh xạ các thư mục sau:

```
Host                          →  Container
./ai_models/weights           →  /app/ai_models/weights  (Models)
./adas.db                     →  /app/adas.db            (Database)
./logs                        →  /app/logs               (Logs)
./dataset                     →  /app/dataset            (Datasets)
.                             →  /app                    (Source - hot reload)
```

## 🔥 Hot Reload (Development Mode)

Code changes được tự động reload nhờ volume mapping:

1. Sửa file Python trên host
2. Container tự động reload ⚡
3. Không cần rebuild!

**Lưu ý:** Nếu thay đổi `requirements.txt`, cần rebuild:
```bash
docker compose up --build
```

## 🏥 Health Check

Docker tự động kiểm tra health:

```bash
# Xem trạng thái
docker compose ps

# Output:
# NAME           STATUS                    PORTS
# adas-backend   Up 5 minutes (healthy)    0.0.0.0:8000->8000/tcp
```

Endpoint health check: `http://localhost:8000/health`

## 🐛 Troubleshooting

### 1. Port 8000 đã được sử dụng
```bash
# Tìm process đang dùng port 8000
lsof -i :8000
# hoặc
netstat -ano | grep 8000

# Kill process
kill -9 <PID>
```

### 2. Container không start
```bash
# Xem logs lỗi
docker compose logs backend

# Kiểm tra Dockerfile syntax
docker compose config
```

### 3. WebSocket không kết nối
```bash
# Kiểm tra container có chạy không
docker compose ps

# Kiểm tra port mapping
docker compose port backend 8000

# Test WebSocket từ trong container
docker compose exec backend curl -i -N \
  -H "Connection: Upgrade" \
  -H "Upgrade: websocket" \
  http://localhost:8000/ws/inference
```

### 4. Thiếu model weights
```bash
# Copy model vào container
docker compose exec backend bash
cd ai_models/weights
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
```

### 5. Permission denied
```bash
# Thêm quyền cho thư mục
chmod -R 755 backend-python/

# Rebuild với quyền mới
docker compose up --build
```

## 📊 Performance

### Resource Usage (Container)
```bash
# Xem CPU/Memory usage
docker stats adas-backend

# Giới hạn resources (optional)
# Thêm vào docker-compose.yml:
services:
  backend:
    deploy:
      resources:
        limits:
          cpus: '2'
          memory: 4G
```

### Build Time
- **First build**: ~5-10 phút (download base image + dependencies)
- **Rebuild**: ~1-2 phút (chỉ update code)
- **No changes**: <10 giây (start container)

## 🔐 Production Deployment

### 1. Tối ưu Dockerfile cho production
```dockerfile
# Multi-stage build
FROM python:3.11-slim as builder
WORKDIR /app
COPY requirements.txt .
RUN pip install --user --no-cache-dir -r requirements.txt

FROM python:3.11-slim
WORKDIR /app
COPY --from=builder /root/.local /root/.local
COPY . .
ENV PATH=/root/.local/bin:$PATH
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "4"]
```

### 2. Environment variables
```bash
# Tạo file .env
API_HOST=0.0.0.0
API_PORT=8000
DATABASE_URL=sqlite:///./adas.db
CORS_ORIGINS=https://yourdomain.com
```

### 3. HTTPS + Reverse Proxy
```yaml
# docker-compose.prod.yml
services:
  nginx:
    image: nginx:alpine
    ports:
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - backend

  backend:
    build: .
    expose:
      - "8000"
```

## ✅ Checklist cho Team Members

Khi clone project lần đầu:

- [ ] Cài Docker Desktop
- [ ] Start Docker Desktop
- [ ] Clone repo: `git clone <repo-url>`
- [ ] Vào thư mục: `cd backend-python`
- [ ] Chạy lệnh: `docker compose up --build`
- [ ] Đợi 5-10 phút (lần đầu)
- [ ] Mở browser: `http://localhost:8000/docs`
- [ ] Test WebSocket: `ws://localhost:8000/ws/inference`
- [ ] ✅ Done! Không cần cài gì thêm!

## 🎯 Best Practices

1. **Luôn dùng Docker** cho development để đảm bảo môi trường đồng nhất
2. **Commit .dockerignore** để giảm kích thước image
3. **Không commit .env** (dùng .env.example)
4. **Persist volumes** cho data quan trọng (DB, models, logs)
5. **Health checks** để đảm bảo service luôn sẵn sàng
6. **Hot reload** cho development, **workers** cho production

## 📚 Tài liệu tham khảo

- Docker Compose: https://docs.docker.com/compose/
- FastAPI in Docker: https://fastapi.tiangolo.com/deployment/docker/
- WebSocket with Docker: https://www.docker.com/blog/how-to-use-the-official-nginx-docker-image/

---

**Version**: 1.0.0  
**Last Updated**: 2025-11-30  
**Docker Compose Version**: 3.8  
**Python Version**: 3.11
