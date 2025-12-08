# 🎯 Quick Reference - Cross-Platform Docker Commands

## 🚀 Most Common Commands

### Start Development Server
```bash
# Method 1: Helper script (easiest)
./start-docker-dev.sh                                    # macOS/Linux
docker-start.bat                                         # Windows

# Method 2: Direct Docker Compose
docker compose -f docker-compose.cross-platform.yml up
```

### Stop Server
```bash
# Method 1: Helper script
./stop-docker.sh                                         # macOS/Linux  
stop-docker.bat                                          # Windows

# Method 2: Direct Docker Compose
docker compose -f docker-compose.cross-platform.yml down

# Method 3: Ctrl+C in the terminal (if running in foreground)
```

### First Time Setup
```bash
cd backend-python
docker compose -f docker-compose.cross-platform.yml build
```

### View Logs
```bash
# Real-time logs
docker compose -f docker-compose.cross-platform.yml logs -f

# Last 100 lines
docker compose -f docker-compose.cross-platform.yml logs --tail=100
```

### Rebuild After Changes
```bash
# Rebuild and start
docker compose -f docker-compose.cross-platform.yml up --build

# Force rebuild (no cache)
docker compose -f docker-compose.cross-platform.yml build --no-cache
docker compose -f docker-compose.cross-platform.yml up
```

---

## 📊 File Locations

| File | Purpose | Use When |
|------|---------|----------|
| `Dockerfile.cross-platform` | Multi-stage build config | Changing dependencies or system packages |
| `docker-compose.cross-platform.yml` | Service configuration | Changing ports, volumes, environment vars |
| `start-docker-dev.sh` | Quick start script (Unix) | Starting server on macOS/Linux |
| `docker-start.bat` | Quick start script (Windows) | Starting server on Windows |
| `DOCKER_CROSS_PLATFORM_GUIDE.md` | Complete documentation | Learning or troubleshooting |
| `DOCKER_IMPLEMENTATION_SUMMARY.md` | This summary | Quick overview |

---

## 🌐 Connection Endpoints

| Type | URL | Purpose |
|------|-----|---------|
| **WebSocket** | `ws://localhost:8080/ws/inference` | Real-time detection streaming |
| **REST API** | `http://localhost:8080/api/*` | Standard API calls |
| **Health Check** | `http://localhost:8080/health` | Verify server is running |
| **Driver Monitor** | `ws://localhost:8080/ws/monitor` | Driver monitoring WebSocket |

---

## ⚙️ Configuration

### Change Port
Edit `docker-compose.cross-platform.yml`:
```yaml
ports:
  - "3000:8080"  # Host:Container (change 3000 to your desired port)
```

### Enable Debug Mode
Edit `docker-compose.cross-platform.yml`:
```yaml
environment:
  - DEBUG=true  # or false for production
```

### Change Database
Edit `docker-compose.cross-platform.yml`:
```yaml
environment:
  - DATABASE_URL=sqlite:///./adas.db  # or your SQL Server connection
```

---

## 🧹 Cleanup Commands

```bash
# Stop and remove containers
docker compose -f docker-compose.cross-platform.yml down

# Also remove volumes (database, logs)
docker compose -f docker-compose.cross-platform.yml down -v

# Remove Docker images
docker rmi backend-python-adas-backend-dev
docker rmi backend-python-adas-backend-prod

# Clean all Docker resources (use with caution!)
docker system prune -a --volumes
```

---

## 🐛 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| Port already in use | Change port in `docker-compose.cross-platform.yml` |
| Container won't start | Check logs: `docker compose -f docker-compose.cross-platform.yml logs` |
| Hot reload not working | Rebuild: `docker compose -f docker-compose.cross-platform.yml build --no-cache` |
| Cannot connect to WebSocket | Verify server running: `curl http://localhost:8080/health` |
| Docker not running | Start Docker Desktop |
| Out of disk space | Run: `docker system prune` |

---

## 📦 Production Deployment

```bash
# Build production image
docker compose -f docker-compose.cross-platform.yml build adas-backend-prod

# Start production server
docker compose -f docker-compose.cross-platform.yml --profile production up -d adas-backend-prod

# Check status
docker compose -f docker-compose.cross-platform.yml ps

# View production logs
docker compose -f docker-compose.cross-platform.yml logs -f adas-backend-prod
```

---

## ✅ Verification Checklist

After starting the server, verify:

```bash
# 1. Container is running
docker compose -f docker-compose.cross-platform.yml ps
# Expected: adas-websocket-dev running

# 2. Health check passes
curl http://localhost:8080/health
# Expected: {"status": "healthy", ...}

# 3. No errors in logs
docker compose -f docker-compose.cross-platform.yml logs --tail=50
# Expected: "Application startup complete"

# 4. WebSocket is accessible
# Open browser console and run:
# ws = new WebSocket('ws://localhost:8080/ws/inference')
# Expected: Connection established
```

---

## 💡 Tips

- **Start in background**: Add `-d` flag → `docker compose -f docker-compose.cross-platform.yml up -d`
- **View specific service logs**: `docker compose -f docker-compose.cross-platform.yml logs -f adas-backend-dev`
- **Enter container shell**: `docker compose -f docker-compose.cross-platform.yml exec adas-backend-dev bash`
- **Run Python inside container**: `docker compose -f docker-compose.cross-platform.yml exec adas-backend-dev python seed.py`

---

## 🆘 Need Help?

1. Check `DOCKER_CROSS_PLATFORM_GUIDE.md` for detailed troubleshooting
2. View logs: `docker compose -f docker-compose.cross-platform.yml logs -f`
3. Verify Docker is running: `docker info`
4. Check container status: `docker compose -f docker-compose.cross-platform.yml ps`

---

**For complete documentation, see:** `DOCKER_CROSS_PLATFORM_GUIDE.md`
