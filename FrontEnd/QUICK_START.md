# 🚀 ADAS - Quick Start Guide

## 🎯 Chạy hệ thống siêu nhanh (Recommended)

### Lần đầu tiên (Build Backend):
```bash
# Bước 1: Build backend image (chỉ cần 1 lần)
npm run backend:build

# Hoặc dùng docker compose trực tiếp
docker compose build backend
```

### Từ lần 2 trở đi (Chỉ 2 lệnh):
```bash
# Terminal 1: Start Backend (tự động mở Docker)
npm run backend

# Terminal 2: Start Frontend  
npm run dev
```

**✨ Script tự động:**
- ✅ Tự động mở Docker Desktop nếu chưa chạy (macOS/Linux/Windows)
- ✅ Tự động start backend container
- ✅ Không cần cd vào thư mục nào
- ✅ Hoạt động trên mọi hệ điều hành

### Các lệnh hữu ích:
```bash
npm run backend        # Start backend (auto-start Docker)
npm run backend:build  # Build backend image
npm run backend:stop   # Stop backend
npm run backend:logs   # Xem logs realtime
npm run dev            # Start frontend
```

## 📊 Sau khi khởi động

### Backend (Docker - Port 8000)
- 🌐 API: http://localhost:8000
- 📚 Docs: http://localhost:8000/docs
- 🔌 WebSocket: ws://localhost:8000/ws/inference
- 🏥 Health: http://localhost:8000/health

### Frontend (Next.js - Port 3000)
- 🎨 UI: http://localhost:3000
- 🚗 ADAS: http://localhost:3000/adas
- 👁️ Driver Monitor: http://localhost:3000/driver-monitor

## 🔧 Kiểm tra kết nối FE-BE

### Test 1: Health Check
```bash
curl http://localhost:8000/health
# Expected: {"status":"healthy","version":"3.0.0"}
```

### Test 2: WebSocket từ Frontend
```javascript
// Mở Console trong browser (F12)
const ws = new WebSocket('ws://localhost:8000/ws/inference')
ws.onopen = () => console.log('✅ Connected!')
ws.onmessage = (e) => console.log('Message:', e.data)
```

### Test 3: API Call từ Frontend
```javascript
// Trong browser console
fetch('http://localhost:8000/api/status')
  .then(r => r.json())
  .then(d => console.log('✅ API:', d))
```

## ⚠️ Troubleshooting

### 1. Backend không start được
```bash
# Check Docker đang chạy chưa
docker info

# Mở Docker Desktop thủ công
# macOS: open -a Docker
# Windows: Mở từ Start Menu
# Linux: sudo systemctl start docker

# Thử lại
npm run backend
```

### 2. Port 8000 đã được sử dụng
```bash
# macOS/Linux: Tìm và kill process
lsof -i :8000
kill -9 <PID>

# Windows: Tìm và kill process
netstat -ano | findstr :8000
taskkill /PID <PID> /F

# Hoặc stop container cũ
npm run backend:stop
```

### 3. Docker không chạy
```bash
# macOS
open -a Docker
sleep 30
npm run backend

# Linux
sudo systemctl start docker
npm run backend

# Windows
# Mở Docker Desktop từ Start Menu
# Đợi 30s rồi chạy: npm run backend
```

### 4. Frontend không kết nối được Backend
```bash
# Check .env.local
cat .env.local
# Phải có: NEXT_PUBLIC_API_URL=http://localhost:8000

# Restart Frontend
npm run dev
```

### 4. WebSocket bị disconnect
```bash
# Check Docker logs
npm run backend:logs

# Restart Backend
npm run backend:stop
npm run backend
```

## 📝 Environment Variables

### Backend (.env trong backend-python/)
```env
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=*
```

### Frontend (.env.local trong root/)
```env
NEXT_PUBLIC_API_URL=http://localhost:8000
```

## 🛑 Dừng hệ thống

```bash
# Dừng Backend
npm run backend:stop

# Dừng Frontend: Ctrl+C trong terminal đang chạy npm run dev
```

## 🐳 Docker Commands (Nâng cao)

```bash
# Xem logs realtime
npm run backend:logs

# Rebuild image
npm run backend:build

# Stop containers
npm run backend:stop

# Vào shell container
docker compose exec backend bash

# Xem container status
docker compose ps
```

## ✅ Checklist Deployment

### Lần đầu clone project:
- [ ] Docker Desktop đã cài và đang chạy
- [ ] Node.js 18+ đã cài
- [ ] Clone project: `git clone <repo>`
- [ ] Cài dependencies: `npm install`
- [ ] Tạo .env.local: `NEXT_PUBLIC_API_URL=http://localhost:8000`
- [ ] Build backend: `npm run backend:build`
- [ ] Start backend: `npm run backend`
- [ ] Terminal mới: `npm run dev`
- [ ] Mở http://localhost:3000
- [ ] Test WebSocket tại /adas
- [ ] ✅ Done!

### Mỗi lần dev:
- [ ] Terminal 1: `npm run backend` (tự động mở Docker)
- [ ] Terminal 2: `npm run dev`
- [ ] Mở http://localhost:3000

### Script Files:
- **macOS/Linux**: `start-backend.sh` (tự động chạy qua npm)
- **Windows**: `start-backend.bat` (tự động chạy qua npm)
- **Cross-platform**: `npm run backend` hoạt động trên mọi OS

## 🎯 Port Summary

| Service | Port | URL | Protocol |
|---------|------|-----|----------|
| Backend API | 8000 | http://localhost:8000 | HTTP |
| WebSocket | 8000 | ws://localhost:8000/ws/* | WS |
| Frontend | 3000 | http://localhost:3000 | HTTP |

**⚠️ LƯU Ý: Backend đã chuyển từ port 8080 → 8000 để khớp với Docker!**

---

**Last Updated**: 2025-12-06  
**Docker**: ✅ Ready  
**Auto-Start**: ✅ Enabled (macOS/Linux/Windows)
**WebSocket**: ✅ Working  
**Hot Reload**: ✅ Enabled
