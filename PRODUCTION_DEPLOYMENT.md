# 🚀 ADAS Production Deployment Guide

## Overview

Hệ thống ADAS real-time production-ready với:
- ✅ FastAPI backend với WebSocket
- ✅ YOLO11 real-time detection
- ✅ Camera support (webcam/USB/RTSP)
- ✅ Next.js frontend responsive
- ✅ Docker deployment cross-platform
- ✅ Database persistence (SQL Server/SQLite)

## 📁 Architecture

```
┌─────────────────┐     WebSocket      ┌──────────────────┐
│                 │ ←───────────────→  │                  │
│  Next.js        │   Camera Stream     │  FastAPI         │
│  Frontend       │   ADAS Results      │  Backend         │
│                 │                     │                  │
└─────────────────┘                     └──────────────────┘
        ↓                                        ↓
    Browser Camera                        ┌──────────────┐
    getUserMedia()                        │  ADAS Engine │
                                         │  - TSR       │
                                         │  - FCW       │
                                         │  - LDW       │
                                         └──────────────┘
                                                ↓
                                         ┌──────────────┐
                                         │  YOLO11      │
                                         │  Detection   │
                                         └──────────────┘
                                                ↓
                                         ┌──────────────┐
                                         │  Database    │
                                         │  Events/Stats│
                                         └──────────────┘
```

## 🏗️ System Requirements

### Minimum
- CPU: 4 cores
- RAM: 8GB
- Storage: 20GB
- OS: Linux/macOS/Windows
- Docker & Docker Compose
- Camera device (webcam/USB)

### Recommended
- CPU: 8 cores
- RAM: 16GB
- GPU: NVIDIA with CUDA support
- Storage: 50GB SSD
- Network: Gigabit ethernet
- Multiple cameras support

## 📦 Installation

### 1. Clone Repository
```bash
git clone <repository-url>
cd ADAS
```

### 2. Backend Setup

```bash
cd backend-python

# Create .env file
cp .env.example .env

# Edit .env với configuration của bạn
nano .env
```

**.env configuration:**
```env
# Server
HOST=0.0.0.0
PORT=8000
DEBUG=False

# Database (SQL Server)
SQL_SERVER=localhost
SQL_DATABASE=ADAS_DB
SQL_USERNAME=sa
SQL_PASSWORD=YourSecurePassword
SQL_DRIVER=ODBC Driver 17 for SQL Server

# Or SQLite (for development)
DATABASE_URL=sqlite:///./adas.db

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://your-frontend.com

# ADAS Config
ADAS_ENABLE_TSR=true
ADAS_ENABLE_FCW=true
ADAS_ENABLE_LDW=true
```

### 3. Download YOLO11 Model
```bash
chmod +x download_adas_models.sh
./download_adas_models.sh
```

### 4. Frontend Setup
```bash
cd ../FrontEnd

# Install dependencies
npm install
# or
pnpm install

# Create .env.local
cp .env.example .env.local

# Edit .env.local
nano .env.local
```

**.env.local:**
```env
NEXT_PUBLIC_API_URL=http://localhost:8000
NEXT_PUBLIC_WS_URL=ws://localhost:8000
```

## 🐳 Docker Deployment

### Option 1: Docker Compose (Recommended)

```bash
# Build and start all services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

### Option 2: Separate Containers

**Backend:**
```bash
cd backend-python

# Build image
docker build -t adas-backend .

# Run container
docker run -d \
  --name adas-backend \
  -p 8000:8000 \
  -v $(pwd)/ai_models/weights:/app/ai_models/weights \
  -v $(pwd)/adas:/app/adas \
  --device /dev/video0:/dev/video0 \
  adas-backend
```

**Frontend:**
```bash
cd FrontEnd

# Build image
docker build -t adas-frontend .

# Run container
docker run -d \
  --name adas-frontend \
  -p 3000:3000 \
  -e NEXT_PUBLIC_API_URL=http://backend-ip:8000 \
  adas-frontend
```

## 🚀 Running in Production

### 1. Start Backend
```bash
cd backend-python

# With Docker
docker-compose up -d adas-api

# Or without Docker
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

### 2. Start Frontend
```bash
cd FrontEnd

# Production build
npm run build
npm start

# Or with Docker
docker-compose up -d adas-frontend
```

### 3. Access Application

- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- API Docs: http://localhost:8000/docs
- ADAS Page: http://localhost:3000/adas

## 📸 Camera Setup

### Linux
```bash
# Check available cameras
ls /dev/video*

# Test camera
ffmpeg -f v4l2 -list_formats all -i /dev/video0

# Give permissions
sudo usermod -a -G video $USER
sudo chmod 666 /dev/video0
```

### macOS
```bash
# Camera permissions handled by browser
# Grant camera access when prompted
```

### Windows
```bash
# Camera permissions in Settings > Privacy > Camera
# Enable camera access for browsers
```

### Docker Camera Access

**Linux:**
```yaml
# docker-compose.yml
services:
  adas-api:
    devices:
      - /dev/video0:/dev/video0
```

**macOS/Windows:**
```yaml
# Camera streaming from browser to backend
# No direct device access needed
```

## 🔧 Configuration

### ADAS Thresholds

Edit `backend-python/adas/config.py`:

```python
# Traffic Sign Recognition
TSR_CONF_THRESHOLD = 0.45

# Forward Collision Warning
FCW_DANGER_DISTANCE = 15.0   # meters
FCW_WARNING_DISTANCE = 30.0  # meters

# Lane Departure Warning
DEPARTURE_THRESHOLD = 0.15
```

### Camera Settings

Edit frontend camera resolution:

```typescript
// FrontEnd/app/adas/page.tsx
const stream = await navigator.mediaDevices.getUserMedia({
  video: {
    width: { ideal: 1280 },
    height: { ideal: 720 },
    frameRate: { ideal: 30 }
  }
})
```

## 📊 Database Setup

### SQL Server (Production)

```sql
-- Create database
CREATE DATABASE ADAS_DB;
GO

USE ADAS_DB;
GO

-- Tables will be created automatically by SQLAlchemy
```

### SQLite (Development)

```bash
# Auto-created on first run
# File: backend-python/adas.db
```

### Migrations

```bash
cd backend-python

# Generate migration
alembic revision --autogenerate -m "Add ADAS tables"

# Apply migration
alembic upgrade head
```

## 🧪 Testing

### Backend API Test
```bash
# Health check
curl http://localhost:8000/health

# ADAS health
curl http://localhost:8000/api/adas/health

# WebSocket test (use wscat)
npm install -g wscat
wscat -c ws://localhost:8000/ws/adas/stream
```

### Frontend Test
```bash
# Open browser
open http://localhost:3000/adas

# Grant camera permission
# Click "Start Camera"
# Click "Connect ADAS"
# Click "Start Streaming"
```

### Full Integration Test
```bash
cd backend-python
python test_adas.py
```

## 📈 Monitoring

### Logs

**Backend logs:**
```bash
# Docker
docker logs -f adas-backend

# Direct
tail -f logs/adas_backend.log
```

**Frontend logs:**
```bash
# Docker
docker logs -f adas-frontend

# Direct
npm run dev  # Development mode with logs
```

### Metrics

Access API metrics:
```bash
curl http://localhost:8000/api/adas/stats
```

View in dashboard:
```
http://localhost:3000/analytics
```

## 🔒 Security

### Production Checklist

- [ ] Change default passwords
- [ ] Enable HTTPS/WSS
- [ ] Configure CORS properly
- [ ] Use environment variables
- [ ] Enable authentication
- [ ] Rate limiting on WebSocket
- [ ] Input validation
- [ ] SQL injection prevention
- [ ] XSS protection

### HTTPS Setup

**Backend (Nginx reverse proxy):**
```nginx
server {
    listen 443 ssl;
    server_name api.yourdomain.com;
    
    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;
    
    location / {
        proxy_pass http://localhost:8000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

**Frontend:**
```bash
# Use Vercel/Netlify for automatic HTTPS
# Or configure Nginx for Next.js
```

## 🐛 Troubleshooting

### Camera not detected
```bash
# Check browser permissions
# Check /dev/video* exists
# Try different camera index
# Check Docker device mapping
```

### WebSocket connection fails
```bash
# Check backend is running
# Check firewall/ports
# Verify CORS settings
# Check WebSocket URL in frontend
```

### Low FPS
```bash
# Reduce camera resolution
# Use smaller YOLO model (yolo11n)
# Enable GPU acceleration
# Reduce frame send rate
```

### Model not found
```bash
# Run download script
./download_adas_models.sh

# Or manual download
cd ai_models/weights
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
```

## 📱 Multi-Platform Support

### Linux (Ubuntu/Debian)
```bash
# Install dependencies
sudo apt update
sudo apt install -y python3-opencv python3-pip docker.io

# Run
docker-compose up -d
```

### macOS
```bash
# Install dependencies
brew install python opencv docker

# Run
docker-compose up -d
```

### Windows
```powershell
# Install Docker Desktop
# Install Python 3.11+
# Install CUDA (for GPU)

# Run
docker-compose up -d
```

## 🌐 Cloud Deployment

### AWS EC2
```bash
# t3.medium or larger
# Open ports 8000, 3000
# Install Docker
# Run docker-compose
```

### Google Cloud
```bash
# e2-medium or larger
# Enable firewall rules
# Deploy containers
```

### Azure
```bash
# B2s or larger
# Configure network
# Deploy with Azure Container Instances
```

## 📞 Support

Issues: https://github.com/your-repo/issues
Docs: https://your-docs-site.com
Email: support@yourdomain.com

---

**🚗 Production ADAS System - Ready to deploy!**
