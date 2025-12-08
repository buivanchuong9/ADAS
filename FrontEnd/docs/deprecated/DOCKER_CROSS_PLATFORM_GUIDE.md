# 🐳 ADAS WebSocket Backend - Cross-Platform Docker Setup

## 🎯 Overview

This guide shows you how to run the ADAS WebSocket backend in Docker **identically** on macOS, Windows, and Linux.

✅ **Build once, run anywhere**  
✅ **No local Python, OpenCV, or dependencies needed**  
✅ **Hot reload works on all platforms**  
✅ **WebSocket server behaves 100% identically**

---

## 📋 Prerequisites

- **Docker Desktop** installed and running
  - macOS: [Download Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/)
  - Windows: [Download Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
  - Linux: [Install Docker Engine](https://docs.docker.com/engine/install/)

- **Docker Compose** (included with Docker Desktop)

---

## 🚀 Quick Start

### 1️⃣ Navigate to Backend Directory

```bash
cd backend-python
```

### 2️⃣ Build the Docker Image (ONE TIME ONLY)

```bash
docker compose -f docker-compose.cross-platform.yml build
```

**Expected output:**
```
[+] Building 45.2s (15/15) FINISHED
 => [internal] load build definition from Dockerfile.cross-platform
 => => transferring dockerfile: 2.31kB
 => [internal] load .dockerignore
 => => transferring context: 891B
 ...
 => => naming to docker.io/library/backend-python-adas-backend-dev
```

### 3️⃣ Start the WebSocket Server

**Development Mode (with hot reload):**
```bash
docker compose -f docker-compose.cross-platform.yml up
```

**Production Mode:**
```bash
docker compose -f docker-compose.cross-platform.yml --profile production up adas-backend-prod
```

**Background Mode (detached):**
```bash
docker compose -f docker-compose.cross-platform.yml up -d
```

### 4️⃣ Verify Server is Running

**Check logs:**
```bash
docker compose -f docker-compose.cross-platform.yml logs -f
```

**Expected output:**
```
adas-websocket-dev  | INFO:     Uvicorn running on http://0.0.0.0:8080 (Press CTRL+C to quit)
adas-websocket-dev  | INFO:     Started reloader process
adas-websocket-dev  | ✅ Database tables created/verified
adas-websocket-dev  | ✅ Directories verified
adas-websocket-dev  | ✅ Startup complete!
```

**Test WebSocket connection:**
```bash
curl http://localhost:8080/health
```

**Expected response:**
```json
{"status": "healthy", "message": "ADAS Backend API is running"}
```

---

## 🌐 Connect from Frontend

Your frontend can connect to the WebSocket server at:

```javascript
// Development
const ws = new WebSocket('ws://localhost:8080/ws/inference');

// Or use the API
fetch('http://localhost:8080/api/detections')
  .then(res => res.json())
  .then(data => console.log(data));
```

---

## 🔄 Hot Reload

**Changes are automatically detected** when you edit files:

1. Edit any `.py` file in `backend-python/`
2. Save the file
3. Docker automatically reloads the server

**Example:**
```
adas-websocket-dev  | INFO:     Detected changes in 'api/websocket_inference.py'
adas-websocket-dev  | INFO:     Reloading...
adas-websocket-dev  | INFO:     Application startup complete
```

**Note for Windows users:**  
Hot reload uses `WATCHFILES_FORCE_POLLING=true` to ensure file changes are detected on Windows filesystems.

---

## 🛠️ Common Commands

### View Running Containers
```bash
docker compose -f docker-compose.cross-platform.yml ps
```

### Stop the Server
```bash
docker compose -f docker-compose.cross-platform.yml down
```

### Restart the Server
```bash
docker compose -f docker-compose.cross-platform.yml restart
```

### View Logs (Real-time)
```bash
docker compose -f docker-compose.cross-platform.yml logs -f
```

### Rebuild After Dependency Changes
```bash
docker compose -f docker-compose.cross-platform.yml build --no-cache
docker compose -f docker-compose.cross-platform.yml up
```

### Execute Commands Inside Container
```bash
# Open shell
docker compose -f docker-compose.cross-platform.yml exec adas-backend-dev bash

# Run Python script
docker compose -f docker-compose.cross-platform.yml exec adas-backend-dev python seed.py

# Check Python version
docker compose -f docker-compose.cross-platform.yml exec adas-backend-dev python --version
```

### Clean Up Everything
```bash
# Stop and remove containers, networks
docker compose -f docker-compose.cross-platform.yml down

# Also remove volumes (database, logs)
docker compose -f docker-compose.cross-platform.yml down -v

# Remove Docker images
docker rmi backend-python-adas-backend-dev
docker rmi backend-python-adas-backend-prod
```

---

## 📁 Project Structure

```
backend-python/
├── Dockerfile.cross-platform          # Multi-stage Docker build
├── docker-compose.cross-platform.yml  # Docker Compose config
├── .dockerignore                      # Optimize build performance
├── main.py                            # FastAPI app entry point
├── requirements.txt                   # Python dependencies
├── api/
│   ├── websocket_inference.py         # WebSocket endpoints
│   └── ...                            # Other API routes
├── core/
│   ├── config.py                      # Configuration
│   └── ...
├── services/                          # Business logic
├── ai_models/
│   └── weights/                       # YOLO model files (mounted)
├── dataset/                           # Training data (mounted)
└── logs/                              # Application logs (mounted)
```

---

## 🔧 Configuration

### Environment Variables

Edit `docker-compose.cross-platform.yml` to customize:

```yaml
environment:
  - HOST=0.0.0.0                    # Server host
  - PORT=8080                       # WebSocket/API port
  - DEBUG=true                      # Enable debug mode
  - CORS_ORIGINS=*                  # CORS settings
  - DATABASE_URL=sqlite:///./adas.db # Database connection
  - WEIGHTS_DIR=/app/ai_models/weights
```

### Ports

- **8080**: WebSocket + REST API (exposed to host)

To change the port, edit `docker-compose.cross-platform.yml`:

```yaml
ports:
  - "3000:8080"  # Access at localhost:3000, container uses 8080
```

### Volumes (Data Persistence)

These directories are mounted from your host machine:

- `./ai_models/weights` → Model files (read-only in production)
- `./dataset` → Training data
- `./logs` → Application logs
- Named volume for database

**Your data persists** even when containers are removed.

---

## 🐛 Troubleshooting

### Port Already in Use

**Error:**
```
Error starting userland proxy: listen tcp 0.0.0.0:8080: bind: address already in use
```

**Solutions:**
```bash
# Find process using port 8080
# macOS/Linux:
lsof -i :8080
# Windows PowerShell:
netstat -ano | findstr :8080

# Kill the process or change port in docker-compose.cross-platform.yml
ports:
  - "8081:8080"  # Use 8081 instead
```

### Hot Reload Not Working on Windows

Make sure `WATCHFILES_FORCE_POLLING=true` is set in `docker-compose.cross-platform.yml`.

If still not working:
```bash
# Rebuild with no cache
docker compose -f docker-compose.cross-platform.yml build --no-cache
docker compose -f docker-compose.cross-platform.yml up
```

### Container Keeps Restarting

**Check logs:**
```bash
docker compose -f docker-compose.cross-platform.yml logs
```

**Common issues:**
- Missing dependencies → Rebuild: `docker compose -f docker-compose.cross-platform.yml build --no-cache`
- Port conflict → Change port in docker-compose.cross-platform.yml
- Database permission issue → Check volume permissions

### Cannot Connect to WebSocket

1. **Verify server is running:**
   ```bash
   docker compose -f docker-compose.cross-platform.yml ps
   ```

2. **Check health:**
   ```bash
   curl http://localhost:8080/health
   ```

3. **Check firewall settings** (Windows/macOS)

4. **Test WebSocket connection:**
   ```bash
   # Install websocat: https://github.com/vi/websocat
   websocat ws://localhost:8080/ws/inference
   ```

### Out of Disk Space

**Clean up Docker resources:**
```bash
# Remove unused containers, images, networks
docker system prune

# Remove everything including volumes
docker system prune -a --volumes
```

---

## ✅ Verification Checklist

After starting the server, verify:

- [ ] Container is running: `docker compose -f docker-compose.cross-platform.yml ps`
- [ ] Health check passes: `curl http://localhost:8080/health`
- [ ] Logs show no errors: `docker compose -f docker-compose.cross-platform.yml logs`
- [ ] Hot reload works: Edit a `.py` file and check logs
- [ ] Frontend can connect: Test WebSocket connection from browser
- [ ] API endpoints work: `curl http://localhost:8080/api/detections`

---

## 🎓 Understanding the Setup

### Multi-Stage Dockerfile

```dockerfile
# Stage 1: Base dependencies (shared)
FROM python:3.11-slim AS base

# Stage 2: Install Python packages
FROM base AS dependencies

# Stage 3: Development (hot reload)
FROM dependencies AS development

# Stage 4: Production (optimized)
FROM dependencies AS production
```

**Benefits:**
- Smaller production images
- Faster builds (layer caching)
- Separate dev and prod configurations

### Docker Compose Profiles

```yaml
# Development (default)
docker compose up

# Production (explicit)
docker compose --profile production up adas-backend-prod
```

### Volume Mounts Explained

```yaml
volumes:
  # Source code (hot reload)
  - ./:/app
  
  # Exclude these (prevent conflicts)
  - /app/__pycache__
  - /app/venv
  
  # Data directories (persistent)
  - ./ai_models/weights:/app/ai_models/weights
  - ./dataset:/app/dataset
  - ./logs:/app/logs
```

---

## 🚢 Deploying to Production

For production deployment:

```bash
# Build production image
docker compose -f docker-compose.cross-platform.yml build adas-backend-prod

# Run in production mode
docker compose -f docker-compose.cross-platform.yml --profile production up -d adas-backend-prod

# Check status
docker compose -f docker-compose.cross-platform.yml ps

# View logs
docker compose -f docker-compose.cross-platform.yml logs -f adas-backend-prod
```

**Production features:**
- No source code mounted (immutable)
- Multiple workers (2x CPU cores recommended)
- Optimized image size
- Auto-restart on failure

---

## 📚 Additional Resources

- [Docker Documentation](https://docs.docker.com/)
- [Docker Compose Documentation](https://docs.docker.com/compose/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [WebSocket Documentation](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)

---

## 💡 Team Onboarding

New team members only need:

1. **Install Docker Desktop**
2. **Clone the repository**
3. **Run these commands:**
   ```bash
   cd backend-python
   docker compose -f docker-compose.cross-platform.yml build
   docker compose -f docker-compose.cross-platform.yml up
   ```

**No Python installation required!**  
**No dependency version conflicts!**  
**Works the same on Windows, macOS, and Linux!**

---

## 🎉 Success!

Your WebSocket backend is now running in Docker!

**Access points:**
- WebSocket: `ws://localhost:8080/ws/inference`
- REST API: `http://localhost:8080/api/*`
- Health Check: `http://localhost:8080/health`

**Next steps:**
1. Connect your frontend to `ws://localhost:8080`
2. Test WebSocket streaming with webcam frames
3. Monitor logs for detections: `docker compose -f docker-compose.cross-platform.yml logs -f`

---

**Questions or issues?** Check the troubleshooting section above or open an issue in the repository.
