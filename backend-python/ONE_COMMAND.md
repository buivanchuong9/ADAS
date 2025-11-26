# 🚀 ADAS Backend - One Command Setup & Run

## Chạy Server Bằng 1 Lệnh Duy Nhất

Tự động cài tất cả thư viện và chạy server chỉ với **1 lệnh**!

---

## Windows 🪟

### Cách 1: Double-click (Đơn giản nhất)
```
Double-click file: START.bat
```

### Cách 2: Command Prompt
```cmd
cd backend-python
START.bat
```

### Cách 3: Python script
```cmd
cd backend-python
python start.py
```

---

## macOS / Linux 🍎🐧

### Cách 1: Bash script (Khuyên dùng)
```bash
cd backend-python
chmod +x START.sh
./START.sh
```

### Cách 2: Python script
```bash
cd backend-python
python3 start.py
```

---

## ✨ Script Sẽ Tự Động:

1. ✅ Kiểm tra Python 3.8+
2. ✅ Upgrade pip
3. ✅ **Cài đặt TẤT CẢ thư viện** từ `requirements.txt`
4. ✅ Dừng server cũ (nếu đang chạy)
5. ✅ Khởi động server mới
6. ✅ Hiển thị URLs để truy cập

**Lần đầu chạy:** Sẽ tải và cài đặt thư viện (2-5 phút)  
**Lần sau:** Chỉ mất vài giây khởi động

---

## 📊 Sau Khi Chạy

Server tại: **http://localhost:8000**

**URLs quan trọng:**
- 📖 API Docs: http://localhost:8000/docs
- 💚 Health: http://localhost:8000/health
- 🎯 Alerts: http://localhost:8000/api/alerts/latest
- 📦 Dataset: http://localhost:8000/api/dataset/stats

---

## 🧪 Test Server

```bash
# Windows PowerShell hoặc macOS/Linux Terminal
curl http://localhost:8000/health
```

Kết quả:
```json
{"status":"healthy","service":"ADAS Backend","version":"2.0.0"}
```

---

## 🛑 Dừng Server

### Foreground (Terminal đang chạy):
```
Nhấn Ctrl+C
```

### Background:
**Windows:**
```powershell
taskkill /F /IM python.exe
```

**macOS/Linux:**
```bash
pkill -f "python3 main.py"
```

---

## 📦 Thư Viện Được Cài

Script tự động cài từ `requirements.txt`:

**Core:**
- FastAPI + Uvicorn (API server)
- SQLAlchemy (Database ORM)
- Pydantic (Data validation)

**AI/ML:**
- PyTorch + TorchVision
- Ultralytics YOLOv8
- OpenCV (cv2)
- NumPy, Pillow

**ADAS Features:**
- pyttsx3 (Text-to-Speech)
- MiDaS (Depth estimation)

---

## 🔧 Troubleshooting

### Lỗi: "Python not found"
**Windows:** Tải Python từ https://python.org (tick "Add to PATH")  
**macOS:** `brew install python3`  
**Linux:** `sudo apt install python3 python3-pip`

### Lỗi: Cài thư viện thất bại
```bash
# Thử cài thủ công
cd backend-python
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Lỗi: "Address already in use"
Port 8000 đang bị chiếm. Chạy lại script, nó sẽ tự động kill process cũ.

---

## 🎯 Phase 1 Features

✅ TTC Computation (Time-to-Collision)  
✅ Voice Alerts (Vietnamese TTS)  
✅ Real-time Inference  
✅ Auto-labeling Dataset  

Xem: [PHASE1_TTC_ALERTS.md](PHASE1_TTC_ALERTS.md)

---

## 📝 Tóm Tắt 1 Lệnh

| Platform | Command |
|----------|---------|
| **Windows** | `START.bat` |
| **macOS/Linux** | `./START.sh` |
| **Universal** | `python start.py` |

**Tất cả đều tự động cài thư viện!** 🚀
