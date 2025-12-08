#!/bin/bash
# ADAS Quick Start - Docker Mode (Cross-Platform)
# Supports: macOS, Linux, Windows (WSL/Git Bash)

set -e

echo "=============================================="
echo "🚀 ADAS Production - Docker Mode"
echo "=============================================="
echo ""

# Check if running from correct directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "📁 Working directory: $SCRIPT_DIR"
echo ""

# Check Docker
echo "=============================================="
echo "🐳 Checking Docker..."
echo "=============================================="

if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found! Please install Docker Desktop"
    echo "   Download: https://www.docker.com/products/docker-desktop"
    exit 1
fi

if ! docker info &> /dev/null 2>&1; then
    echo "❌ Docker is not running! Please start Docker Desktop"
    exit 1
fi

echo "✅ Docker is ready"
echo ""

# Setup directories
echo "=============================================="
echo "📁 Creating directories..."
echo "=============================================="

cd backend-python
mkdir -p ai_models/weights dataset/{raw,labels,auto_collected} logs/alerts
echo "✅ Directories created"
echo ""

# Stop existing containers
echo "=============================================="
echo "🛑 Stopping old containers..."
echo "=============================================="

docker-compose down 2>/dev/null || true
echo "✅ Cleaned up"
echo ""

# Build and start
echo "=============================================="
echo "🔨 Building Docker image..."
echo "=============================================="

docker-compose build
echo "✅ Image built"
echo ""

echo "=============================================="
echo "🚀 Starting Backend (Docker)..."
echo "=============================================="

docker-compose up -d
echo "✅ Backend container started"
echo ""

# Wait for backend
echo "⏳ Waiting for backend to be ready..."
for i in {1..30}; do
    if curl -s http://localhost:8000/health > /dev/null 2>&1; then
        echo "✅ Backend is healthy!"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "⚠️  Backend taking longer than expected. Check logs:"
        echo "   docker-compose logs -f"
    fi
    sleep 2
done

cd ..
echo ""
echo ""

# Frontend setup
echo "=============================================="
echo "🎨 Setting up Frontend..."
echo "=============================================="

cd "$SCRIPT_DIR/FrontEnd"

if ! command -v node &> /dev/null; then
    echo "❌ Node.js not found! Install from: https://nodejs.org"
    exit 1
fi

echo "✅ Node.js: $(node --version)"

if [ ! -d "node_modules" ]; then
    echo "📦 Installing dependencies..."
    if command -v pnpm &> /dev/null; then
        pnpm install
    else
        npm install
    fi
fi

echo "✅ Frontend ready"
echo ""

# Environment files
if [ ! -f ".env.local" ]; then
    cat > .env.local << 'EOF'
NEXT_PUBLIC_API_URL=http://localhost:8000
NEXT_PUBLIC_WS_URL=ws://localhost:8000
EOF
    echo "✅ Created .env.local"
fi

cd "$SCRIPT_DIR"

# Start Frontend
echo "=============================================="
echo "🚀 Starting Frontend..."
echo "=============================================="

cd "$SCRIPT_DIR/FrontEnd"
npm run dev &
FRONTEND_PID=$!
cd "$SCRIPT_DIR"

echo "✅ Frontend starting (PID: $FRONTEND_PID)"
echo ""
        echo "   kill $BACKEND_PID $FRONTEND_PID"
        ;;
        
    3)
        echo ""
        echo "⏭️  Skipping auto-start"
        echo ""
        echo "To start manually:"
        echo ""
        echo "Backend:"
        echo "  cd backend-python"
        echo "  source venv/bin/activate"
        echo "  python3 -m uvicorn main:app --host 0.0.0.0 --port 8000"
        echo ""
        echo "Frontend:"
        echo "  cd FrontEnd"
        echo "  npm run dev"
        ;;
        
    *)
        echo "❌ Invalid choice"
        exit 1
        ;;
esac

# Final summary
echo ""
echo "=============================================="
echo "✅ ADAS System Setup Complete!"
echo "=============================================="
echo ""
echo "📡 Services:"
echo "   Backend API:  http://localhost:8000"
echo "   API Docs:     http://localhost:8000/docs"
echo "   Frontend:     http://localhost:3000"
echo "   ADAS Page:    http://localhost:3000/adas"
echo ""
echo "🧪 Quick Test:"
echo "   curl http://localhost:8000/health"
echo "   curl http://localhost:8000/api/adas/health"
echo ""
echo "📖 Documentation:"
echo "   Production Guide: PRODUCTION_DEPLOYMENT.md"
echo "   Integration Info: PRODUCTION_INTEGRATION_SUMMARY.md"
echo "   ADAS README:      backend-python/ADAS_README.md"
echo ""
echo "🎯 Next Steps:"
echo "   1. Open http://localhost:3000/adas in browser"
echo "   2. Grant camera permission"
echo "   3. Click 'Start Camera'"
echo "   4. Click 'Connect ADAS'"
# Summary
echo "=============================================="
echo "✅ ADAS System Running!"
echo "=============================================="
echo ""
echo "📡 Services:"
echo "   • Backend:    http://localhost:8000"
echo "   • Frontend:   http://localhost:3000"
echo "   • ADAS Page:  http://localhost:3000/adas"
echo "   • API Docs:   http://localhost:8000/docs"
echo ""
echo "🔧 Commands:"
echo "   • View logs:      docker-compose -f backend-python/docker-compose.yml logs -f"
echo "   • Stop backend:   docker-compose -f backend-python/docker-compose.yml down"
echo "   • Restart:        ./quick-start.sh"
echo ""
echo "🚀 Next Steps:"
echo "   1. Open: http://localhost:3000/adas"
echo "   2. Click 'Start Camera'"
echo "   3. Click 'Connect ADAS'"
echo "   4. Click 'Start Streaming'"
echo ""
echo "🐛 Debug WebSocket:"
echo "   • Check browser console (F12)"
echo "   • Network tab → WS filter"
echo "   • Backend logs: docker-compose -f backend-python/docker-compose.yml logs -f"
echo ""
echo "=============================================="
echo "🚗 Ready! Happy driving! 💨"
echo "=============================================="
