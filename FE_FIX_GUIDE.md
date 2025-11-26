# 🔧 Frontend-Backend Integration - Quick Fix Guide

## ✅ Đã Fix

### 1. Next.js API Routes - Proxy đúng endpoints

**Fixed Files:**
- `/app/api/detection/route.ts` - Gọi `/api/detections/recent` & `/api/detections/save`
- `/app/api/detection/stats/route.ts` - Stats endpoint (NEW)
- `/app/api/alerts/route.ts` - Latest alerts (NEW)
- `/app/api/alerts/stats/route.ts` - Alert stats (NEW)

### 2. Backend Endpoints Hoạt Động

```bash
✅ http://localhost:8000/api/detections/recent?limit=20
✅ http://localhost:8000/api/detections/save
✅ http://localhost:8000/api/detections/stats
✅ http://localhost:8000/api/alerts/latest
✅ http://localhost:8000/api/alerts/stats
✅ http://localhost:8000/api/analytics/dashboard
```

---

## 🎯 Cách Test Frontend

### Test trong Browser Console (F12)

```javascript
// Test detections API
fetch('http://localhost:8000/api/detections/recent?limit=5')
  .then(r => r.json())
  .then(d => console.log('Detections:', d))

// Test analytics dashboard
fetch('http://localhost:8000/api/analytics/dashboard')
  .then(r => r.json())
  .then(d => console.log('Dashboard:', d))

// Test alerts
fetch('http://localhost:8000/api/alerts/latest?limit=10')
  .then(r => r.json())
  .then(d => console.log('Alerts:', d))
```

---

## 📱 Pages Cần Reload

### 1. ADAS Page - http://localhost:3000/adas
**Đang fetch:**
- `http://localhost:8000/api/detections/recent` ✅
- `http://localhost:8000/health` ✅

**Nếu lỗi CORS:**
- Mở DevTools (F12)
- Check Console errors
- Verify backend CORS cho phép `localhost:3000`

### 2. Dashboard - http://localhost:3000/dashboard
**Đang fetch:**
- `http://localhost:8000/api/analytics/dashboard` ✅

### 3. Data Collection - http://localhost:3000/data-collection
**Đang fetch:**
- `/api/dataset` (Next.js route) ✅ → proxies to backend

---

## 🚨 Common Issues & Fixes

### Issue 1: "Failed to fetch" hoặc CORS error

**Solution:**
Backend CORS đã allow all origins. Nếu vẫn lỗi:

1. Check backend đang chạy:
```bash
curl http://localhost:8000/health
```

2. Restart backend:
```bash
cd backend-python
python run.py
```

### Issue 2: Next.js API routes return 404

**Solution:**
Next.js cần compile routes mới. Đợi vài giây hoặc reload page (Cmd+R).

### Issue 3: Data rỗng hoặc undefined

**Problem:** Backend có data nhưng frontend không hiển thị.

**Solution:**
Check response format trong Console:

```javascript
// Expected format for detections
{
  "success": true,
  "detections": [
    {
      "id": 123,
      "class_name": "car",
      "confidence": 0.95,
      "bbox": [100, 100, 200, 200],
      "timestamp": "2025-11-26T14:00:00",
      "distance_meters": 15.5
    }
  ],
  "total": 1
}
```

### Issue 4: Models không load

**Problem:** Frontend gọi `/api/models/list` nhưng backend chưa có data.

**Solution:**
Seed database hoặc tạo models:

```bash
cd backend-python
python seed.py  # Nếu có seed script
```

---

## 🔄 Reload Frontend

Sau khi fix Next.js API routes, reload browser:

1. **Cmd + R** (macOS) hoặc **Ctrl + R** (Windows)
2. **Hard reload**: Cmd + Shift + R hoặc Ctrl + Shift + R
3. **Clear cache**: DevTools → Network → ✓ Disable cache

---

## ✅ Test Checklist

Mở http://localhost:3000/adas và check:

- [ ] Page loads không lỗi console
- [ ] Model Worker status = ✅ (xanh)
- [ ] Recent detections hiển thị (bảng bên phải)
- [ ] Webcam button hoạt động
- [ ] No CORS errors in console

---

## 🎯 Expected Data

**Backend có:**
- 8,843 detections (bicycle, car, truck, bus, person, motorcycle)
- Classes: 6 types
- Distance & confidence data

**Frontend nên hiển thị:**
- Recent detections list (refresh every 3s)
- Class names + confidence
- Timestamps
- Bounding boxes (nếu có canvas)

---

## 🚀 Next Steps

1. ✅ Reload browser: http://localhost:3000/adas
2. ✅ Check Console (F12) - không có lỗi đỏ
3. ✅ Verify "Model Worker" = green
4. ✅ Xem Recent Detections bảng bên phải có data

**Nếu vẫn lỗi:**
- Screenshot console errors
- Check Network tab (F12)
- Xem response của failed requests
