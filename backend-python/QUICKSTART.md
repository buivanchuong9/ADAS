# 🚀 Quick Reference - ADAS Backend Python

## ⚡ Cheat Sheet

### Cài đặt nhanh (Windows)
```bash
cd backend-python
setup.bat
# Sửa .env với SQL credentials
python seed.py
run.bat
```

### Cài đặt nhanh (Linux/Mac)
```bash
cd backend-python
chmod +x setup.sh run.sh
./setup.sh
# Sửa .env với SQL credentials
python seed.py
./run.sh
```

### Docker (quickest)
```bash
cd backend-python
docker-compose up -d
```

---

## 📋 Commands Cheat Sheet

| Task | Command |
|------|---------|
| **Setup** | `setup.bat` (Windows) hoặc `./setup.sh` (Unix) |
| **Run server** | `run.bat` hoặc `./run.sh` hoặc `python main.py` |
| **Seed DB** | `python seed.py` |
| **Test connection** | `python database.py` |
| **Install deps** | `pip install -r requirements.txt` |
| **Activate venv** | `venv\Scripts\activate` (Win) / `source venv/bin/activate` (Unix) |
| **Run tests** | `pytest -v` |
| **Docker build** | `docker build -t adas-backend .` |
| **Docker run** | `docker-compose up -d` |

---

## 🌐 URLs Cheat Sheet

| Service | URL |
|---------|-----|
| **API Base** | http://localhost:8000 |
| **Swagger Docs** | http://localhost:8000/docs |
| **ReDoc** | http://localhost:8000/redoc |
| **Health Check** | http://localhost:8000/health |
| **WebSocket** | ws://localhost:8000/ws/infer |

---

## 🔌 API Endpoints Quick Reference

### Cameras
```bash
GET    /api/cameras/list
GET    /api/cameras/{id}
POST   /api/cameras
PUT    /api/cameras/{id}
DELETE /api/cameras/{id}
PUT    /api/cameras/{id}/status
```

### Models
```bash
GET    /api/models/list
GET    /api/models/{id}
POST   /api/models/{id}/download
POST   /api/models/{id}/activate
DELETE /api/models/{id}
```

### Trips
```bash
GET    /api/trips/list
POST   /api/trips
PUT    /api/trips/{id}
POST   /api/trips/{id}/end
```

### Events
```bash
GET    /api/events/list
POST   /api/events
DELETE /api/events/{id}
```

### Drivers
```bash
GET    /api/drivers/list
GET    /api/drivers/{id}
POST   /api/drivers
PUT    /api/drivers/{id}
```

### Analytics
```bash
GET    /api/analytics/dashboard
```

---

## 🧪 Test Commands

```bash
# Health check
curl http://localhost:8000/health

# Get all cameras
curl http://localhost:8000/api/cameras/list

# Create camera
curl -X POST http://localhost:8000/api/cameras \
  -H "Content-Type: application/json" \
  -d '{"name":"Test Camera","type":"webcam","status":"ready"}'

# Get camera by ID
curl http://localhost:8000/api/cameras/1

# Update camera
curl -X PUT http://localhost:8000/api/cameras/1 \
  -H "Content-Type: application/json" \
  -d '{"name":"Updated Camera","type":"ip","status":"active"}'

# Delete camera
curl -X DELETE http://localhost:8000/api/cameras/1

# Get analytics
curl http://localhost:8000/api/analytics/dashboard

# WebSocket test (wscat)
npm install -g wscat
wscat -c ws://localhost:8000/ws/infer
```

---

## 🔧 Environment Variables

```env
SQL_SERVER=localhost                    # SQL Server host
SQL_DATABASE=ADAS_DB                   # Database name
SQL_USERNAME=sa                         # Username
SQL_PASSWORD=YourPassword              # Password
SQL_DRIVER=ODBC Driver 17 for SQL Server
MODEL_WORKER_URL=http://localhost:8000
ALLOWED_ORIGINS=http://localhost:3000
PORT=8000
```

---

## 🐛 Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| **Cannot connect to SQL** | 1. Check SQL Server running<br>2. Enable TCP/IP<br>3. Check firewall<br>4. Verify credentials |
| **ODBC Driver not found** | Install: https://aka.ms/downloadmsodbcsql |
| **Module not found** | `pip install -r requirements.txt` |
| **Port 8000 in use** | `netstat -ano \| findstr :8000`<br>Kill process or change PORT in .env |
| **Database not exist** | Run `python seed.py` |

---

## 📦 Files Overview

| File | Purpose |
|------|---------|
| `main.py` | FastAPI app + all routes |
| `models.py` | SQLAlchemy database models |
| `schemas.py` | Pydantic validation schemas |
| `services.py` | Business logic layer |
| `database.py` | Database connection |
| `config.py` | Configuration management |
| `seed.py` | Database initialization |
| `setup.bat/sh` | Installation script |
| `run.bat/sh` | Run script |

---

## 🔥 Production Deployment (Windows Service)

```powershell
# Install NSSM
choco install nssm

# Create service
nssm install AdasBackend "C:\Python310\python.exe" "C:\adas\backend-python\main.py"
nssm set AdasBackend AppDirectory "C:\adas\backend-python"
nssm start AdasBackend

# Manage service
nssm status AdasBackend
nssm restart AdasBackend
nssm stop AdasBackend
```

---

## 📊 Database Schema Quick View

```
8 tables:
├── cameras (6 columns)
├── drivers (6 columns)
├── trips (6 columns)
├── events (7 columns)
├── detections (5 columns)
├── driver_statuses (6 columns)
├── analytics (4 columns)
└── ai_models (8 columns)
```

---

## 🎯 One-Liners

```bash
# Fresh install + run
setup.bat && python seed.py && run.bat

# Docker everything
docker-compose up --build -d && docker-compose logs -f

# Test all endpoints
curl http://localhost:8000/docs

# Check logs (if service)
Get-Content logs\service.log -Tail 50 -Wait

# Backup database
sqlcmd -S localhost -U sa -P password -Q "BACKUP DATABASE ADAS_DB TO DISK='C:\backup\adas.bak'"
```

---

## 🎓 Learning Resources

- FastAPI: https://fastapi.tiangolo.com
- SQLAlchemy: https://docs.sqlalchemy.org
- pyodbc: https://github.com/mkleehammer/pyodbc
- Pydantic: https://docs.pydantic.dev

---

**Print this page for quick reference! 📄**
