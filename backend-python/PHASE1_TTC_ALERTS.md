# 🔥 Phase 1 COMPLETED - TTC & Voice Alerts

## ✅ Đã Hoàn Thành

### 1. TTC (Time-to-Collision) Computation
**File:** `models/ttc_computer.py`

**Tính năng:**
- Tính TTC = distance / relative_speed
- Severity levels:
  - **Critical**: TTC < 1s → Nguy hiểm!
  - **High**: 1s ≤ TTC < 3s → Cảnh báo!
  - **Medium**: 3s ≤ TTC < 6s → Chú ý
  - **Low**: TTC ≥ 6s → An toàn

**Methods:**
```python
ttc_comp = TTCComputer()

# Compute single vehicle TTC
result = ttc_comp.compute_ttc(
    distance=15.5,  # meters
    relative_speed=5.0,  # m/s approaching
    previous_distance=16.0,
    time_delta=0.1
)
# Returns: {ttc, severity, distance, relative_speed, warning, safe}

# Compute batch TTC
vehicles_with_ttc = ttc_comp.compute_batch_ttc(vehicles, tracking_history)

# Get most critical vehicle
critical = ttc_comp.get_most_critical(vehicles_with_ttc)
```

---

### 2. Voice Alert System
**File:** `models/voice_alert.py`

**Tính năng:**
- Text-to-Speech offline (pyttsx3)
- Tạo audio alerts cho collision & lane departure
- Save audio files to `logs/alerts/`
- Async/non-blocking alerts

**Methods:**
```python
voice = VoiceAlertSystem()

# Create collision alert
audio_path = voice.create_collision_alert(distance=4.2, ttc=0.8)
# → "Nguy hiểm! Va chạm sau 0.8 giây! Phanh gấp!"

# Create lane departure alert
audio_path = voice.create_lane_departure_alert()
# → "Cảnh báo! Xe đang lệch làn đường!"

# Custom alert
audio_path = voice.create_alert("Chú ý xe phía trước", severity="high")
```

---

### 3. Updated Inference API
**Endpoint:** `POST /api/inference/video`

**New Parameters:**
```bash
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@test.mp4" \
  -F "compute_ttc=true" \
  -F "create_voice_alerts=true" \
  -F "warning_distance=5.0"
```

**Response (Enhanced):**
```json
{
  "success": true,
  "frames": [
    {
      "frame_number": 42,
      "timestamp": 1.4,
      "vehicles": [
        {
          "class_name": "car",
          "distance": 12.5,
          "ttc": 2.5,
          "severity": "high",
          "relative_speed": 5.0,
          "warning": "⚠️ Cảnh báo! Xe phía trước...",
          "safe": false
        }
      ],
      "ttc_info": {
        "min_ttc": 2.5,
        "distance": 12.5,
        "severity": "high"
      },
      "warnings": [
        {
          "type": "collision_warning",
          "ttc": 2.5,
          "severity": "high",
          "message": "..."
        }
      ]
    }
  ],
  "summary": {
    "min_ttc": 2.5,
    "critical_alerts": 3,
    "total_warnings": 8
  },
  "alerts": [
    {
      "ttc": 0.8,
      "severity": "critical",
      "audio_path": "logs/alerts/alert_critical_20241122_153045.wav",
      "message": "Nguy hiểm! Va chạm sau 0.8s!"
    }
  ]
}
```

---

### 4. Alerts API
**New Router:** `api/alerts/router.py`

#### Endpoints:

**GET /api/alerts/latest**
Lấy alerts mới nhất (cho FE polling)
```bash
curl "http://localhost:8000/api/alerts/latest?limit=10&severity=critical&unplayed_only=true"
```

**GET /api/alerts/audio/{alert_id}**
Download alert audio file
```bash
curl "http://localhost:8000/api/alerts/audio/123" --output alert.wav
```

**GET /api/alerts/stats**
Thống kê alerts
```bash
curl "http://localhost:8000/api/alerts/stats?hours=24"
```

**Response:**
```json
{
  "total_alerts": 45,
  "severity_breakdown": {
    "critical": 5,
    "high": 12,
    "medium": 18,
    "low": 10
  },
  "type_breakdown": {
    "collision_warning": 17,
    "lane_departure": 28
  },
  "min_ttc": 0.5,
  "avg_critical_distance": 8.3
}
```

**POST /api/alerts/mark-played/{alert_id}**
Đánh dấu alert đã play

**DELETE /api/alerts/clear-old**
Xóa alerts cũ (cleanup)

---

### 5. Database Models
**New Table:** `Alert`

```sql
CREATE TABLE Alerts (
    Id INT PRIMARY KEY,
    EventId INT,
    TTC FLOAT,
    Distance FLOAT,
    RelativeSpeed FLOAT,
    Severity VARCHAR(50),  -- critical, high, medium, low
    AlertType VARCHAR(100),  -- collision_warning, lane_departure
    Message TEXT,
    AudioPath VARCHAR(1000),
    Played BIT DEFAULT 0,
    CreatedAt DATETIME
);
```

---

## 🚀 How to Use

### 1. Install Dependencies
```bash
cd backend-python
pip3 install pyttsx3
```

### 2. Test TTC Computation
```python
from models.ttc_computer import TTCComputer

ttc = TTCComputer()
result = ttc.compute_ttc(distance=10, relative_speed=5)
print(result)
# {'ttc': 2.0, 'severity': 'high', ...}
```

### 3. Test Voice Alerts
```python
from models.voice_alert import VoiceAlertSystem

voice = VoiceAlertSystem()
voice.test_voice()  # Test TTS
audio = voice.create_collision_alert(distance=5, ttc=1)
```

### 4. Run Inference with TTC
```bash
curl -X POST "http://localhost:8000/api/inference/video" \
  -F "file=@dashcam.mp4" \
  -F "compute_ttc=true" \
  -F "create_voice_alerts=true"
```

### 5. Frontend: Poll for Alerts
```javascript
// Poll every 3 seconds
setInterval(async () => {
  const res = await fetch('/api/alerts/latest?unplayed_only=true');
  const alerts = await res.json();
  
  for (const alert of alerts) {
    if (alert.audio_url && alert.severity === 'critical') {
      // Play audio
      const audio = new Audio(alert.audio_url);
      audio.play();
      
      // Mark as played
      await fetch(`/api/alerts/mark-played/${alert.id}`, {method: 'POST'});
    }
  }
}, 3000);
```

---

## 📊 TTC Computation Logic

### Formula:
```
TTC = distance / relative_speed

Where:
- distance: meters (from MiDaS depth estimation)
- relative_speed: m/s (from tracking frame-to-frame)
```

### Relative Speed Estimation:
```python
# Frame N-1: distance = 20m
# Frame N:   distance = 18m
# Time delta: 0.1s (10 FPS)

relative_speed = (20 - 18) / 0.1 = 20 m/s (approaching)
```

### Severity Mapping:
```
TTC < 1s    → CRITICAL  → "Nguy hiểm! Phanh gấp!"
1s ≤ TTC < 3s  → HIGH      → "Cảnh báo! Giảm tốc độ!"
3s ≤ TTC < 6s  → MEDIUM    → "Chú ý xe phía trước"
TTC ≥ 6s       → LOW       → "An toàn"
```

---

## 🎯 Features Summary

### ✅ Completed:
1. **TTC Computation** - Real-time collision time estimation
2. **Severity Classification** - 4 levels (critical/high/medium/low)
3. **Voice Alerts** - Offline TTS with pyttsx3
4. **Audio File Generation** - Save alerts to disk
5. **Alerts API** - RESTful endpoints for FE
6. **Database Integration** - Alert storage & retrieval
7. **Frontend-Ready** - Polling mechanism, audio URLs

### 🔥 Key Advantages:
- **Real-time**: TTC computed per frame
- **Offline**: No internet required (pyttsx3)
- **Scalable**: Batch processing for multiple vehicles
- **Actionable**: Severity-based alerts
- **User-friendly**: Audio + visual warnings

---

## 📝 Next Steps (Phase 2)

Có thể làm tiếp:
1. **Auto-labeling feedback system** - User corrections
2. **Hazard map** - GPS-based danger zones
3. **Dataset approval workflow** - Review & approve labels
4. **Training automation** - Background training jobs

---

## 🎊 PHASE 1 DONE!

**Files Created:**
- `models/ttc_computer.py` ✅
- `models/voice_alert.py` ✅
- `api/alerts/router.py` ✅
- Updated `api/inference/router.py` ✅
- Updated `models.py` (Alert table) ✅
- Updated `main.py` (alerts router) ✅
- Updated `requirements.txt` (pyttsx3) ✅

**Ready to test! 🚀**
