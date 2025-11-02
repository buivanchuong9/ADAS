# ADAS - Advanced Driver Assistance System

Hệ thống hỗ trợ lái xe tiên tiến với phát hiện vật thể thời gian thực, giám sát tài xế, và trợ lý AI.

---

## 🚀 QUICK START (3 Bước)

### Step 1: Cài đặt tự động
```bash
# Windows
setup.bat

# macOS/Linux
bash setup.sh
```
Chờ 2-5 phút để cài đặt tất cả dependencies.

### Step 2: Thêm Firebase (QUAN TRỌNG!)
1. Go to [Firebase Console](https://console.firebase.google.com/)
2. Project Settings → Service Accounts → Generate new private key
3. Save JSON file as: `backend/firebase-service-account.json`

### Step 3: Chạy hệ thống
```bash
# Windows
PowerShell -ExecutionPolicy Bypass -File run.ps1

# macOS/Linux
bash run.sh
```

Mở browser: **http://localhost:3000/dashboard**

---

## 📱 Truy cập ứng dụng

| Tính năng | URL |
|----------|-----|
| **Dashboard** | http://localhost:3000/dashboard |
| **Live Detection** | http://localhost:3000/live |
| **AI Assistant** | http://localhost:3000/ai-assistant |
| **Driver Monitor** | http://localhost:3000/driver-monitor |
| **Analytics** | http://localhost:3000/analytics |

---

## 🏗️ Kiến trúc hệ thống

```
Frontend (Next.js)           Backend (.NET)           Model Worker (FastAPI)
http://localhost:3000        http://localhost:5000    http://localhost:8000
                                    ↓
                              Firebase Firestore
                        (Real-time Data Storage)
```

**Thành phần:**
- **Frontend**: Next.js + React 19 + TailwindCSS
- **Backend**: ASP.NET Core 8 + Entity Framework + WebSocket
- **Model**: Python FastAPI + YOLOv8/YOLOv5
- **Database**: Firebase Firestore (real-time)

---

## 📋 Yêu cầu hệ thống

- **Node.js 18+**: https://nodejs.org/
- **.NET SDK 8**: https://dotnet.microsoft.com/
- **Python 3.11+**: https://www.python.org/
- **Ports**: 3000, 5000, 8000 (phải rảnh)
- **Disk**: ~5GB free space

---

## 🔧 Cài đặt thủ công (nếu auto install thất bại)

### 1. Install dependencies
```bash
npm install --legacy-peer-deps
cd model-worker && pip install -r requirements.txt && cd ..
cd backend && dotnet restore && cd ..
```

### 2. Chạy từng service riêng

**Terminal 1 - Model Worker:**
```bash
cd model-worker
uvicorn app:app --host 0.0.0.0 --port 8000
```

**Terminal 2 - Backend:**
```bash
cd backend
dotnet run
```

**Terminal 3 - Frontend:**
```bash
npm run dev
```

---

## 🆘 Khắc phục sự cố

| Lỗi | Giải pháp |
|-----|----------|
| **Port already in use** | `lsof -i :3000` → `kill -9 [PID]` |
| **npm install fails** | `npm install --legacy-peer-deps --force` |
| **Python errors** | `pip install -r requirements.txt --force-reinstall` |
| **Firebase not found** | Add `backend/firebase-service-account.json` |
| **.NET build fails** | `cd backend && dotnet clean && dotnet build` |
| **Permission denied** (Linux/Mac) | `chmod +x setup.sh run.sh` |

---

## 📁 Project Structure

```
adas-platform/
├── README.md                    ← You are here
├── setup.sh / setup.bat         ← Auto installer
├── run.sh / run.ps1             ← Service launcher
├── next.config.mjs
├── tsconfig.json
├── package.json
│
├── app/                         ← Next.js pages
│   ├── page.tsx
│   ├── dashboard/page.tsx
│   ├── live/page.tsx
│   └── ...
│
├── components/                  ← React components
│   └── ui/                      ← Shadcn UI components
│
├── backend/                     ← .NET Core API
│   ├── ADAS.csproj
│   ├── Program.cs
│   ├── Controllers/
│   ├── Services/
│   │   ├── FirebaseDataService.cs
│   │   ├── ModelService.cs
│   │   └── EventService.cs
│   └── Models/
│
├── model-worker/                ← FastAPI Server
│   ├── app.py
│   ├── requirements.txt
│   └── yolov8n.pt
│
└── lib/
    └── utils.ts
```

---

## 💡 Tính năng chính

✅ **Phát hiện vật thể real-time** - Sử dụng YOLOv8/v5
✅ **Giám sát tài xế** - Phát hiện mệt mỏi, chuyên tâm
✅ **Trợ lý AI** - Chat với Perplexity API
✅ **Dashboard real-time** - Xem analytics qua Firebase
✅ **WebSocket streaming** - Dữ liệu real-time
✅ **Multi-model AI** - Hỗ trợ nhiều model

---

## 🔐 Firebase Setup Chi Tiết

1. **Tạo project** (nếu chưa có):
   - https://console.firebase.google.com/ → Create Project

2. **Tạo Service Account**:
   - Project Settings (⚙️) → Service Accounts
   - Click "Generate New Private Key"
   - Sẽ download 1 JSON file

3. **Lưu file**:
   ```bash
   # Copy JSON file to:
   backend/firebase-service-account.json
   ```

4. **Xác nhận**:
   - File phải có `"type": "service_account"` ở đầu
   - Backend sẽ tự kết nối khi khởi động

---

## 📊 Luồng dữ liệu

```
Camera/Sensor
    ↓
Frontend (Live Detection)
    ↓
WebSocket to Backend
    ↓
Model Worker (AI Inference)
    ↓
Backend (Process & Log)
    ↓
Firebase Firestore ← Lưu dữ liệu thực
    ↓
Dashboard (Real-time Display)
```

---

## ✨ Sử dụng các tính năng

### 1. Live Detection (Phát hiện thời gian thực)
- Mở: http://localhost:3000/live
- Cho phép truy cập camera
- Đặt vật thể trước camera
- Xem detection boxes realtime

### 2. Dashboard (Analytics)
- Mở: http://localhost:3000/dashboard
- Xem recent events
- Xem inference results
- Data tự sync từ Firebase

### 3. AI Assistant (Trợ lý AI)
- Mở: http://localhost:3000/ai-assistant
- Chat về lái xe an toàn
- Nhận tư vấn từ AI

### 4. Driver Monitor (Giám sát tài xế)
- Mở: http://localhost:3000/driver-monitor
- Phát hiện mệt mỏi
- Cảnh báo an toàn

---

## 🧪 Kiểm tra setup

Chạy verification script:
```bash
# Windows
check.bat

# macOS/Linux
bash check.sh
```

Kết quả mong đợi:
```
✓ Node.js: v18.x.x
✓ .NET SDK: 8.x.x
✓ Python: 3.11.x
✓ npm packages installed
✓ Python packages installed
✓ Firebase service account found
```

---

## 🎯 Tips & Tricks

💡 **Monitor logs**: Check terminal output khi services đang chạy
💡 **Clear cache**: `rm -rf .next && npm install --legacy-peer-deps`
💡 **Stop service**: Close that terminal window
💡 **Different port**: Edit uvicorn/dotnet/npm commands
💡 **Debug mode**: Add `--debug` flag khi chạy services

---

## 📝 Các lệnh hữu ích

```bash
# Cài lại dependencies
npm install --legacy-peer-deps --force
cd model-worker && pip install -r requirements.txt --force-reinstall

# Xóa cache
rm -rf .next node_modules __pycache__ bin obj
npm install

# Check ports
lsof -i :3000
lsof -i :5000
lsof -i :8000

# Kill process on port
kill -9 $(lsof -ti:3000)
```

---

## 🚀 Next Steps

1. ✅ Run `setup.sh` or `setup.bat`
2. ✅ Add `backend/firebase-service-account.json`
3. ✅ Run `run.sh` or `run.ps1`
4. ✅ Open http://localhost:3000/dashboard
5. 🎉 Enjoy!

---

**Questions?** Check terminal logs or verify Firebase credentials.

**Ready to deploy?** Project is production-ready with Firebase backend!

Made with ❤️ for ADAS Platform
