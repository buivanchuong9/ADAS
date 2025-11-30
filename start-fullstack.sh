#!/bin/bash
# ============================================
# ADAS Full Stack - One Command Start
# Backend (Docker) + Frontend (Next.js)
# ============================================

set -e

echo "🚀 ADAS Full Stack - Starting..."
echo "=================================="
echo ""

# Check Docker
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker không chạy!"
    echo "Hãy mở Docker Desktop và chạy lại script này."
    exit 1
fi

echo "✅ Docker đang chạy"
echo ""

# Start Backend in Docker
echo "🐳 Starting Backend (Docker)..."
cd backend-python
docker compose down 2>/dev/null || true
docker compose up -d --build

echo "⏳ Đợi Backend khởi động..."
sleep 5

# Check Backend health
for i in {1..20}; do
    if curl -f http://localhost:8000/health > /dev/null 2>&1; then
        echo ""
        echo "✅ Backend sẵn sàng! (http://localhost:8000)"
        break
    fi
    echo -n "."
    sleep 1
done

echo ""
echo ""

# Start Frontend
echo "⚛️  Starting Frontend (Next.js)..."
cd ..
npm run dev &
FRONTEND_PID=$!

echo ""
echo "⏳ Đợi Frontend khởi động..."
sleep 5

echo ""
echo "=================================="
echo "✅ Full Stack đã khởi động!"
echo "=================================="
echo ""
echo "📊 Services:"
echo "   🐳 Backend:  http://localhost:8000"
echo "   📚 API Docs: http://localhost:8000/docs"
echo "   🔌 WebSocket: ws://localhost:8000/ws/inference"
echo "   ⚛️  Frontend: http://localhost:3000"
echo ""
echo "📝 Useful commands:"
echo "   - Backend logs: docker compose logs -f backend"
echo "   - Stop all: docker compose down && kill $FRONTEND_PID"
echo ""
echo "🎉 Hệ thống đã sẵn sàng!"
