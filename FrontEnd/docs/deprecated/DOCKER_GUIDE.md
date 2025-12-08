# 🐳 Docker Deployment Guide

## Quick Start

```bash
# Build and start all services
docker compose up --build

# Run in background
docker compose up -d --build

# View logs
docker compose logs -f

# Stop all services
docker compose down
```

## Services

### Backend (FastAPI)
- **Port**: 8000
- **Health Check**: http://localhost:8000/health
- **API Docs**: http://localhost:8000/docs
- **WebSocket**: ws://localhost:8000/ws/inference

### Frontend (Next.js)
- **Port**: 3000
- **URL**: http://localhost:3000

## Environment Variables

### Backend (.env in backend-python/)
```env
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=*
DATABASE_URL=sqlite:///./adas.db
WEIGHTS_DIR=/app/ai_models/weights
```

### Frontend (.env.local in root/)
```env
NEXT_PUBLIC_API_URL=http://localhost:8000
```

## Development Workflow

### Hot Reload
Both services support hot reload:
- **Backend**: Python code changes auto-reload
- **Frontend**: Next.js fast refresh

### Rebuild After Changes
```bash
# Rebuild specific service
docker compose up --build backend
docker compose up --build frontend

# Rebuild all
docker compose up --build
```

### View Logs
```bash
# All services
docker compose logs -f

# Specific service
docker compose logs -f backend
docker compose logs -f frontend
```

## Troubleshooting

### Port Already in Use
```bash
# Find process using port
lsof -i :8000
lsof -i :3000

# Kill process
kill -9 <PID>
```

### Clear Docker Cache
```bash
# Remove containers and volumes
docker compose down -v

# Remove images
docker compose down --rmi all

# Rebuild from scratch
docker compose build --no-cache
docker compose up
```

### Database Issues
```bash
# Access backend container
docker compose exec backend bash

# Test database connection
python -c "from database import test_connection; test_connection()"
```

### WebSocket Connection Failed
1. Check backend is running: `curl http://localhost:8000/health`
2. Check CORS settings in backend
3. Verify frontend env: `NEXT_PUBLIC_API_URL=http://localhost:8000`
4. Check browser console for errors

## Production Deployment

### Build Production Images
```bash
# Build optimized images
docker compose -f docker-compose.yml build

# Tag for registry
docker tag adas-backend:latest your-registry/adas-backend:v3.0
docker tag adas-frontend:latest your-registry/adas-frontend:v3.0

# Push to registry
docker push your-registry/adas-backend:v3.0
docker push your-registry/adas-frontend:v3.0
```

### Environment-Specific Configs
```bash
# Production
docker compose -f docker-compose.prod.yml up -d

# Staging
docker compose -f docker-compose.staging.yml up -d
```

## Health Checks

Both services have built-in health checks:

```bash
# Check service health
docker compose ps

# Backend health
curl http://localhost:8000/health

# Frontend health
curl http://localhost:3000
```

## Resource Management

### View Resource Usage
```bash
docker stats
```

### Limit Resources (docker-compose.yml)
```yaml
services:
  backend:
    deploy:
      resources:
        limits:
          cpus: '2'
          memory: 4G
```

## Backup & Restore

### Backup Database
```bash
docker compose exec backend cp /app/adas.db /app/adas.db.backup
docker cp adas-backend:/app/adas.db.backup ./backup/
```

### Restore Database
```bash
docker cp ./backup/adas.db.backup adas-backend:/app/adas.db
docker compose restart backend
```

---

**Last Updated**: 2025-11-30  
**Docker Compose Version**: 3.8  
**Services**: Backend (FastAPI) + Frontend (Next.js)
