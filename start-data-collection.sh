#!/bin/bash
# Quick Start Script for ADAS Data Collection

echo "🚀 Starting ADAS Data Collection System..."
echo ""

# Kill existing processes
echo "🔄 Cleaning up existing processes..."
lsof -ti:3000 | xargs kill -9 2>/dev/null
lsof -ti:8000 | xargs kill -9 2>/dev/null
sleep 1

# Start Backend
echo "📡 Starting Backend (port 8000)..."
cd backend-python
python3 main.py &
BACKEND_PID=$!
cd ..
sleep 3

# Start Frontend
echo "🌐 Starting Frontend (port 3000)..."
npm run dev &
FRONTEND_PID=$!
sleep 5

echo ""
echo "======================================"
echo "✅ ADAS Data Collection System Running"
echo "======================================"
echo ""
echo "🔗 Access Points:"
echo "   Frontend:     http://localhost:3000/data-collection"
echo "   API Docs:     http://localhost:8000/docs"
echo "   Backend API:  http://localhost:8000/api/dataset"
echo ""
echo "📖 Full Guide:   HUONG_DAN_DATA_COLLECTION.md"
echo ""
echo "⚠️  Press Ctrl+C to stop both services"
echo "======================================"
echo ""

# Wait for user interrupt
trap "echo ''; echo '🛑 Stopping services...'; kill $BACKEND_PID $FRONTEND_PID 2>/dev/null; exit 0" INT

# Keep script running
wait
