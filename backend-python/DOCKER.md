# Docker Deployment Guide

## Quick Start với Docker

### 1. Build và chạy toàn bộ stack

```bash
# Tạo .env file trước
cp .env.example .env

# Build và start tất cả services
docker-compose up -d

# Xem logs
docker-compose logs -f backend
```

### 2. Chỉ chạy backend (SQL Server đã có sẵn)

```bash
# Build image
docker build -t adas-backend .

# Run container
docker run -d \
  --name adas-backend \
  -p 8000:8000 \
  -e SQL_SERVER=host.docker.internal \
  -e SQL_DATABASE=ADAS_DB \
  -e SQL_USERNAME=sa \
  -e SQL_PASSWORD=YourPassword \
  adas-backend
```

### 3. Seed database

```bash
# Exec vào container
docker exec -it adas-backend bash

# Chạy seed
python seed.py
```

## Docker Compose Services

- `backend` - FastAPI application (port 8000)
- `sqlserver` - SQL Server 2019 Express (port 1433)
- `model-worker` - YOLO inference service (port 8001)

## Environment Variables

Tạo file `.env` trong thư mục `backend-python`:

```env
SQL_SERVER=sqlserver
SQL_DATABASE=ADAS_DB
SQL_USERNAME=sa
SQL_PASSWORD=YourStrong@Password123
MODEL_WORKER_URL=http://model-worker:8000
ALLOWED_ORIGINS=http://localhost:3000
```

## Production Deployment

### Docker Swarm

```bash
# Init swarm
docker swarm init

# Create secret cho password
echo "YourStrongPassword" | docker secret create sql_password -

# Deploy stack
docker stack deploy -c docker-compose.yml adas
```

### Kubernetes

Tạo file `k8s-deployment.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: adas-backend
spec:
  replicas: 3
  selector:
    matchLabels:
      app: adas-backend
  template:
    metadata:
      labels:
        app: adas-backend
    spec:
      containers:
      - name: backend
        image: adas-backend:latest
        ports:
        - containerPort: 8000
        env:
        - name: SQL_SERVER
          value: "sqlserver-service"
        - name: SQL_PASSWORD
          valueFrom:
            secretKeyRef:
              name: sql-secret
              key: password
```

## Health Checks

```bash
# Check container health
docker inspect --format='{{.State.Health.Status}}' adas-backend

# Manual health check
curl http://localhost:8000/health
```

## Logs

```bash
# View logs
docker logs adas-backend -f

# Export logs
docker logs adas-backend > backend.log 2>&1
```

## Troubleshooting

### Container không start

```bash
# Check logs
docker logs adas-backend

# Check if SQL Server is ready
docker exec adas-sqlserver /opt/mssql-tools/bin/sqlcmd -S localhost -U sa -P "$SQL_PASSWORD" -Q "SELECT 1"
```

### Connection refused

```bash
# Check network
docker network inspect adas-network

# Test connection từ backend
docker exec adas-backend ping sqlserver
```

## Clean Up

```bash
# Stop all services
docker-compose down

# Remove volumes (warning: deletes data)
docker-compose down -v

# Remove images
docker rmi adas-backend
```
