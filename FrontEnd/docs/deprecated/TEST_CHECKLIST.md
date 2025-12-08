# ✅ ADAS - Test Checklist

## 🎯 Mục tiêu
Kiểm tra hệ thống ADAS hoàn chỉnh sau khi:
- ✅ Migrate YOLOv11
- ✅ Fix port configuration (8000)
- ✅ Setup Docker environment
- ✅ Cleanup unnecessary files

---

## 📋 Pre-Test Checklist

### 1. Docker Desktop
- [ ] Docker Desktop đã mở
- [ ] Docker daemon đang chạy
- [ ] Command: `docker info` (không báo lỗi)

### 2. Environment Files
- [ ] `.env.local` có: `NEXT_PUBLIC_API_URL=http://localhost:8000`
- [ ] Backend sử dụng port 8000
- [ ] Frontend mong đợi port 8000

---

## 🐳 Backend Tests (Docker)

### Build & Start
```bash
cd backend-python
docker compose up -d --build
```

**Expected:**
- [ ] Build successful (không có error)
- [ ] Container name: `adas-backend`
- [ ] Container status: `Up` (healthy)
- [ ] Port mapping: `0.0.0.0:8000->8000/tcp`

### Health Checks
```bash
# 1. Health endpoint
curl http://localhost:8000/health
# Expected: {"success":true,"message":"Service is healthy",...}

# 2. API docs
curl http://localhost:8000/docs
# Expected: HTML page (Swagger UI)

# 3. Status endpoint
curl http://localhost:8000/api/status
# Expected: {"success":true,"message":"All systems operational",...}
```

**Results:**
- [ ] `/health` returns 200 OK
- [ ] `/docs` accessible
- [ ] `/api/status` shows database connected
- [ ] Container logs không có ERROR

### WebSocket Test
```bash
# Install wscat (nếu chưa có)
npm install -g wscat

# Connect to WebSocket
wscat -c ws://localhost:8000/ws/inference
```

**Expected:**
- [ ] Connection established
- [ ] Nhận được welcome message
- [ ] Có thể gửi/nhận JSON messages

---

## ⚛️ Frontend Tests (Next.js)

### Start Frontend
```bash
# Từ root directory
npm run dev
```

**Expected:**
- [ ] Next.js starts on `http://localhost:3000`
- [ ] No compilation errors
- [ ] No port conflict errors

### Browser Tests

#### 1. Homepage
```
URL: http://localhost:3000
```
- [ ] Page loads successfully
- [ ] Navigation menu visible
- [ ] No console errors

#### 2. ADAS Page
```
URL: http://localhost:3000/adas
```
- [ ] Page loads
- [ ] WebSocket connection indicator shows "Connected" 
- [ ] Camera/upload UI visible
- [ ] Detection controls visible
- [ ] No CORS errors in console

**WebSocket Check:**
- [ ] Browser console: `ws://localhost:8000/ws/inference` connected
- [ ] Green "Connected" indicator
- [ ] Can start/stop detection

#### 3. Driver Monitor
```
URL: http://localhost:3000/driver-monitor
```
- [ ] Page loads
- [ ] Camera access prompt appears
- [ ] After allowing camera: WebSocket connects
- [ ] Driver state metrics visible
- [ ] No errors in console

**WebSocket Check:**
- [ ] URL: `ws://localhost:8000/api/driver/ws/monitor`
- [ ] Status: Connected
- [ ] Metrics updating in real-time

#### 4. Models Page
```
URL: http://localhost:3000/models-webcam
```
- [ ] Page loads
- [ ] Model list loads from API
- [ ] Can select YOLOv11 models
- [ ] Camera feed works

---

## 🔌 API Integration Tests

### 1. Model API
```bash
curl http://localhost:8000/api/models/available
```
**Expected:**
- [ ] Returns YOLOv11 models (yolo11n, yolo11s, yolo11m)
- [ ] Each model has: name, version, size, parameters

### 2. Detection API
```bash
curl http://localhost:8000/api/detections/stats
```
**Expected:**
- [ ] Returns statistics object
- [ ] No database errors

### 3. Camera API
```bash
curl http://localhost:8000/api/cameras/list
```
**Expected:**
- [ ] Returns array of cameras
- [ ] Status code 200

---

## 🎥 End-to-End Tests

### Test 1: ADAS Detection via Webcam
1. [ ] Mở http://localhost:3000/adas
2. [ ] Click "Start Webcam"
3. [ ] Allow camera access
4. [ ] Select model: YOLOv11n
5. [ ] Click "Start Detection"
6. [ ] **Expected:**
   - [ ] Video stream hiển thị
   - [ ] Bounding boxes xuất hiện khi có object
   - [ ] FPS counter updates
   - [ ] Detection log updates

### Test 2: Driver Monitoring
1. [ ] Mở http://localhost:3000/driver-monitor
2. [ ] Allow camera access
3. [ ] **Expected:**
   - [ ] Driver state hiển thị (ALERT/DROWSY/...)
   - [ ] Attention score updates (0-100%)
   - [ ] Eye Aspect Ratio (EAR) updates
   - [ ] PERCLOS percentage updates
   - [ ] Head pose (pitch/yaw/roll) updates
   - [ ] Alert sounds nếu buồn ngủ

### Test 3: Video Upload
1. [ ] Mở http://localhost:3000/adas
2. [ ] Upload một video file (.mp4)
3. [ ] **Expected:**
   - [ ] Upload progress bar
   - [ ] Server processes video
   - [ ] Results returned (detections, stats)

---

## 📊 Performance Tests

### Backend Performance
```bash
# Check Docker resource usage
docker stats adas-backend
```
**Monitor:**
- [ ] CPU < 50% khi idle
- [ ] Memory < 2GB khi idle
- [ ] CPU spikes khi inference (normal)

### Frontend Performance
**Browser DevTools (F12) → Performance:**
- [ ] First Contentful Paint < 2s
- [ ] Time to Interactive < 3s
- [ ] No memory leaks (check over 5 minutes)

---

## 🐛 Error Scenarios

### Test Network Errors
1. [ ] Stop backend: `docker compose down`
2. [ ] Frontend shows "Disconnected"
3. [ ] Start backend: `docker compose up -d`
4. [ ] Frontend reconnects automatically

### Test Invalid Input
1. [ ] Upload invalid file (e.g., .txt)
2. [ ] **Expected:** Error message, không crash

### Test CORS
1. [ ] Open frontend from different port (e.g., :3001)
2. [ ] **Expected:** Backend accepts (CORS=*)

---

## ✅ Success Criteria

### Minimum Requirements
- [x] Backend starts in Docker (1 command)
- [x] Frontend connects to backend
- [x] WebSocket working (ADAS + Driver Monitor)
- [x] YOLOv11 models available
- [x] No critical errors in logs

### Full Pass
- [ ] All health checks pass
- [ ] All API endpoints respond
- [ ] Both WebSocket endpoints work
- [ ] Webcam detection works
- [ ] Driver monitoring works
- [ ] Upload feature works
- [ ] No console errors
- [ ] Performance acceptable

---

## 📝 Test Results Log

### Date: ___________
### Tester: ___________

| Test Category | Status | Notes |
|--------------|--------|-------|
| Docker Build | ⏳ In Progress | Building... |
| Backend Health | ⬜ Not Started | |
| Frontend Start | ⬜ Not Started | |
| WebSocket | ⬜ Not Started | |
| ADAS Detection | ⬜ Not Started | |
| Driver Monitor | ⬜ Not Started | |
| API Integration | ⬜ Not Started | |

**Legend:**
- ⬜ Not Started
- ⏳ In Progress
- ✅ Pass
- ❌ Fail

---

## 🔧 Troubleshooting Quick Reference

### Issue: Backend không start
```bash
docker compose logs backend
# Check for Python errors, missing dependencies
```

### Issue: Frontend không connect
```bash
# Check .env.local
cat .env.local
# Should have: NEXT_PUBLIC_API_URL=http://localhost:8000
```

### Issue: WebSocket disconnect
```bash
# Check backend logs
docker compose logs -f backend | grep -i websocket

# Check browser console (F12)
# Look for WebSocket errors
```

### Issue: Port conflict
```bash
# Find process using port 8000
lsof -i :8000

# Kill process
kill -9 <PID>
```

---

**Last Updated:** 2025-11-30
**Version:** 1.0
**Status:** Ready for Testing
