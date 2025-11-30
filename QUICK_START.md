# 🚀 ADAS - Quick Start Guide

## 🎯 Chạy toàn bộ hệ thống với 1 lệnh

### Option 1: Tự động (Recommended)
```bash
./start-fullstack.sh
```

### Option 2: Từng bước

#### Bước 1: Start Backend (Docker)
```bash
cd backend-python
docker compose up --build
```

#### Bước 2: Start Frontend (Terminal mới)
```bash
npm run dev
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

### 1. Port 8000 đã được sử dụng
```bash
# Tìm process
lsof -i :8000

# Kill process
kill -9 <PID>
```

### 2. Docker không chạy
```bash
# Mở Docker Desktop
open -a Docker

# Đợi 30s rồi chạy lại
docker compose up --build
```

### 3. Frontend không kết nối được Backend
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
docker compose logs -f backend

# Restart Docker container
docker compose restart
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

### Dừng Backend
```bash
cd backend-python
docker compose down
```

### Dừng Frontend
```bash
# Press Ctrl+C trong terminal đang chạy npm run dev
```

### Dừng tất cả
```bash
# Backend
cd backend-python && docker compose down

# Frontend (Ctrl+C trong terminal)
```

## 🐳 Docker Commands

```bash
# Xem logs realtime
docker compose logs -f backend

# Restart container
docker compose restart

# Vào shell container
docker compose exec backend bash

# Rebuild image
docker compose build --no-cache
docker compose up
```

## ✅ Checklist Deployment

### Lần đầu clone project:
- [ ] Docker Desktop đã cài và đang chạy
- [ ] Node.js 18+ đã cài
- [ ] Clone project: `git clone <repo>`
- [ ] Cài dependencies: `npm install`
- [ ] Tạo .env.local: `NEXT_PUBLIC_API_URL=http://localhost:8000`
- [ ] Chạy: `./start-fullstack.sh`
- [ ] Mở http://localhost:3000
- [ ] Test WebSocket tại /adas
- [ ] ✅ Done!

### Mỗi lần dev:
- [ ] Mở Docker Desktop
- [ ] Chạy: `cd backend-python && docker compose up`
- [ ] Terminal mới: `npm run dev`
- [ ] Mở http://localhost:3000

## 🎯 Port Summary

| Service | Port | URL | Protocol |
|---------|------|-----|----------|
| Backend API | 8000 | http://localhost:8000 | HTTP |
| WebSocket | 8000 | ws://localhost:8000/ws/* | WS |
| Frontend | 3000 | http://localhost:3000 | HTTP |

**⚠️ LƯU Ý: Backend đã chuyển từ port 8080 → 8000 để khớp với Docker!**

---

**Last Updated**: 2025-11-30  
**Docker**: ✅ Ready  
**WebSocket**: ✅ Working  
**Hot Reload**: ✅ Enabled
