#!/bin/bash
# ============================================
# ADAS Backend - Docker Quick Start Script
# ============================================

set -e

echo "🐳 ADAS Backend - Docker Setup"
echo "================================"
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running!"
    echo "Please start Docker Desktop and try again."
    exit 1
fi

echo "✅ Docker is running"
echo ""

# Stop existing containers
echo "🛑 Stopping existing containers..."
docker compose down 2>/dev/null || true

# Build and start
echo "🔨 Building Docker image..."
echo "This may take a few minutes on first run..."
echo ""

docker compose up --build -d

echo ""
echo "⏳ Waiting for backend to start..."
sleep 5

# Check health
echo "🏥 Checking backend health..."
for i in {1..30}; do
    if curl -f http://localhost:8000/health > /dev/null 2>&1; then
        echo ""
        echo "✅ Backend is healthy and ready!"
        echo ""
        echo "📊 Backend Info:"
        echo "   - URL: http://localhost:8000"
        echo "   - Docs: http://localhost:8000/docs"
        echo "   - WebSocket: ws://localhost:8000/ws/inference"
        echo ""
        echo "📝 Useful commands:"
        echo "   - View logs: docker compose logs -f backend"
        echo "   - Stop: docker compose down"
        echo "   - Restart: docker compose restart"
        echo "   - Shell: docker compose exec backend bash"
        echo ""
        exit 0
    fi
    echo -n "."
    sleep 1
done

echo ""
echo "⚠️  Backend did not start successfully"
echo "Check logs: docker compose logs backend"
exit 1
