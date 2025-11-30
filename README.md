# 🚗 ADAS Platform - Advanced Driver Assistance System

> **Real-time AI-powered driver safety monitoring system**

## 🆕 For Team Members - First Time Setup

**Nếu bạn mới clone project từ GitHub**, vui lòng đọc hướng dẫn setup đầy đủ:

📖 **[SETUP_FOR_TEAM.md](./SETUP_FOR_TEAM.md)** - Hướng dẫn chi tiết cho team members

Hoặc chạy script tự động:
```bash
./setup-first-time.sh
```

---
AI-powered Advanced Driver Assistance System với real-time object detection, auto data collection, incremental training, và remote access capabilities.

## ✨ Features

- 🎯 **Real-time Object Detection** - YOLOv8 detection cho vehicles, pedestrians, traffic signs
- 📊 **Auto Data Collection** - Tự động thu thập và label dữ liệu training
- 🔄 **Incremental Training** - Tự động train model mới khi có đủ data
- 🔔 **Alert System** - Cảnh báo nguy hiểm với voice notifications
- 📹 **WebSocket Live Feed** - Streaming video với real-time inference
- 🌐 **Remote Access** - Cloudflare Tunnel để access API từ bất kỳ đâu
- 📈 **Analytics Dashboard** - Visualization và statistics
- 🗄️ **Dataset Management** - Quản lý videos, labels, training data

## 🚀 Quick Start

### Prerequisites

- Python 3.8+
- Node.js 16+ (cho frontend)
- SQL Server (hoặc SQLite cho development)
- Homebrew (macOS) hoặc apt (Linux) - cho Cloudflare Tunnel

### 1. Cài đặt Backend

```bash
cd backend-python
pip install -r requirements.txt
```

### 2. Cài đặt Frontend

```bash
npm install
# hoặc
pnpm install
```

### 3. Chạy hệ thống

**Option A: Local Only (cùng máy)**

```bash
# Terminal 1 - Backend
cd backend-python
python main.py

# Terminal 2 - Frontend
npm run dev
```

Access:
- Backend API: http://localhost:8080/docs
- Frontend: http://localhost:3000

**Option B: Remote Access (khác máy, khác mạng)**

```bash
# Terminal 1 - Backend
cd backend-python
python main.py

# Terminal 2 - Tunnel
./start-tunnel.sh
# Copy URL được print ra (VD: https://abc-123.trycloudflare.com)

# Cập nhật frontend config
echo "NEXT_PUBLIC_API_URL=https://abc-123.trycloudflare.com" > .env.local

# Terminal 3 - Frontend
npm run dev
```

Giờ frontend có thể chạy ở **BẤT KỲ MÁY NÀO, BẤT KỲ ĐÂU!** 🌍

## 📖 Documentation

- **[QUICKSTART.md](./QUICKSTART.md)** - Hướng dẫn nhanh setup và chạy
- **[TUNNEL_SETUP.md](./TUNNEL_SETUP.md)** - Chi tiết về remote access với Cloudflare Tunnel
- **API Docs** - http://localhost:8080/docs (interactive Swagger UI)

## 🛠️ Available Scripts

### Backend Scripts

```bash
./setup-tunnel.sh        # Cài đặt Cloudflare Tunnel
./start-tunnel.sh        # Khởi động tunnel, expose backend ra internet
./test-api.sh           # Test tất cả API endpoints
./test-api.sh <URL>     # Test remote API endpoints
```

### Frontend Scripts

```bash
npm run dev             # Chạy development server
npm run build           # Build production
npm run start           # Start production server
npm run lint            # Lint check
```

## 🌐 API Endpoints Overview

### Core
- `GET /` - API information
- `GET /health` - Health check
- `GET /api/status` - Detailed status với DB check
- `GET /docs` - Interactive API documentation

### ADAS Features
- `/api/cameras/*` - Camera management
- `/api/models/*` - AI model management
- `/api/detections/*` - Detection results
- `/api/alerts/*` - Alert/warning system
- `/api/dataset/*` - Dataset management
- `/api/training/*` - Model training
- `/api/auto-learning/*` - Auto incremental learning
- `/api/inference/*` - Image/video inference
- `/ws/infer` - WebSocket live inference

### Data Management
- `/api/trips/*` - Trip tracking
- `/api/events/*` - Event logging
- `/api/drivers/*` - Driver management
- `/api/analytics/*` - Analytics & statistics

**Xem danh sách đầy đủ tại [QUICKSTART.md](./QUICKSTART.md)**

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Frontend (Next.js)                  │
│                                                         │
│  ┌─────────┐  ┌──────────┐  ┌────────┐  ┌──────────┐ │
│  │Dashboard│  │Detection │  │Training│  │Analytics │ │
│  └─────────┘  └──────────┘  └────────┘  └──────────┘ │
└────────────────────┬────────────────────────────────────┘
                     │ HTTP/WebSocket
                     ▼
┌─────────────────────────────────────────────────────────┐
│            Cloudflare Tunnel (Optional)                 │
│         https://xyz.trycloudflare.com → localhost:8080  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Backend (FastAPI Python)                   │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│  │Detection │  │Training  │  │Dataset   │            │
│  │Engine    │  │Pipeline  │  │Manager   │            │
│  └──────────┘  └──────────┘  └──────────┘            │
│                                                         │
│  ┌──────────────────────────────────────┐             │
│  │      AI Models (YOLOv8)              │             │
│  │  - Object Detection                  │             │
│  │  - Auto Labeling                     │             │
│  │  - Incremental Training              │             │
│  └──────────────────────────────────────┘             │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Database (SQL Server/SQLite)               │
│  - Detections, Alerts, Trips, Events                   │
│  - Models, Datasets, Labels                            │
│  - Analytics, Statistics                               │
└─────────────────────────────────────────────────────────┘
```

## 🔧 Configuration

### Backend Config

File: `backend-python/config.py`
- Database connection
- Model paths
- Training parameters
- Alert thresholds

### Frontend Config

File: `lib/api-config.ts`
- API base URL
- All endpoint paths
- WebSocket URLs

Environment: `.env.local`
```bash
NEXT_PUBLIC_API_URL=https://your-tunnel-url.trycloudflare.com
```

## 🧪 Testing

### Test Local API
```bash
./test-api.sh
```

### Test Remote API
```bash
./test-api.sh https://your-tunnel-url.trycloudflare.com
```

### Test specific endpoint
```bash
curl http://localhost:8080/api/status | jq
curl http://localhost:8080/api/detections/stats | jq
```

## 📊 Tech Stack

### Backend
- **Framework**: FastAPI (Python)
- **AI/ML**: YOLOv8, PyTorch, Ultralytics
- **Database**: SQL Server / SQLite
- **ORM**: SQLAlchemy
- **Real-time**: WebSocket
- **Deployment**: Uvicorn

### Frontend
- **Framework**: Next.js 14 (React)
- **Language**: TypeScript
- **Styling**: Tailwind CSS
- **UI Components**: shadcn/ui
- **Charts**: Recharts
- **State**: React Hooks

### DevOps
- **Tunnel**: Cloudflare Tunnel (cloudflared)
- **API Testing**: curl, custom scripts
- **Documentation**: Swagger/OpenAPI

## 🚀 Remote Access Setup

1. **Install cloudflared:**
   ```bash
   ./setup-tunnel.sh
   ```

2. **Start backend:**
   ```bash
   cd backend-python && python main.py
   ```

3. **Start tunnel:**
   ```bash
   ./start-tunnel.sh
   ```
   Copy the URL printed (e.g., `https://abc-123.trycloudflare.com`)

4. **Update frontend:**
   ```bash
   echo "NEXT_PUBLIC_API_URL=https://abc-123.trycloudflare.com" > .env.local
   npm run dev
   ```

5. **Access from anywhere:**
   - API: `https://abc-123.trycloudflare.com/docs`
   - Frontend: Any machine running the frontend code

**Chi tiết:** [TUNNEL_SETUP.md](./TUNNEL_SETUP.md)

## 📱 Use Cases

1. **Development Team** - Multiple developers ở khác địa điểm cùng test API
2. **Mobile Testing** - Test từ điện thoại/tablet qua 4G/5G
3. **Demo/Presentation** - Show clients từ xa mà không cần deploy
4. **Cross-device Testing** - Test trên nhiều devices khác nhau
5. **Remote Debugging** - Debug issues từ xa với production-like setup

## 🔒 Security Notes

- Cloudflare Tunnel URL là **public** - bất kỳ ai có link đều truy cập được
- Chỉ dùng cho **development/testing**, không dùng cho production
- Thêm authentication/authorization nếu cần bảo mật
- Có thể dùng Cloudflare Access để protect tunnel
- Production deployment nên dùng proper hosting với SSL

## 🐛 Troubleshooting

### Backend không chạy
```bash
cd backend-python
pip install -r requirements.txt
python main.py
```

### Port conflict
```bash
# Kill process on port 8080
lsof -ti:8080 | xargs kill -9
```

### Tunnel issues
```bash
# Reinstall cloudflared
./setup-tunnel.sh

# Check tunnel status
tail -f tunnel.log
```

### Frontend không connect
1. Check `.env.local` có đúng URL
2. Restart Next.js: `npm run dev`
3. Clear browser cache
4. Check browser console for errors

### CORS errors
- Backend đã config `allow_origins=["*"]`
- Restart backend nếu vẫn lỗi
- Check Network tab trong browser DevTools

## 📈 Performance

- **Detection Speed**: ~30-60 FPS (depending on GPU)
- **WebSocket Latency**: <100ms (local), ~200-500ms (via tunnel)
- **Training Time**: ~1-5 hours (depending on dataset size)
- **Auto Learning**: Incremental training mỗi 100-500 new samples

## 🗺️ Roadmap

- [ ] Multi-camera support
- [ ] Advanced driver monitoring (drowsiness, distraction)
- [ ] GPS integration for location-based alerts
- [ ] Cloud deployment guide
- [ ] Mobile app (React Native)
- [ ] Advanced analytics dashboard
- [ ] Model comparison tools
- [ ] Distributed training support

## 📄 License

This project is for educational and research purposes.

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repo
2. Create feature branch
3. Make changes
4. Submit PR

## 📧 Support

Issues? Questions?
- Check [QUICKSTART.md](./QUICKSTART.md)
- Check [TUNNEL_SETUP.md](./TUNNEL_SETUP.md)
- Check `/docs` endpoint
- Open GitHub issue

---

**Made with ❤️ for safer driving**
