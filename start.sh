#!/bin/bash
# ADAS Platform - Quick Start Script

echo "========================================"
echo "🚗 ADAS Platform - Quick Start"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Choose deployment method:${NC}"
echo "1. Docker (Recommended - Full stack)"
echo "2. Local Development (Manual setup)"
echo ""
read -p "Enter choice (1 or 2): " choice

if [ "$choice" == "1" ]; then
    echo ""
    echo -e "${GREEN}[Docker Deployment]${NC}"
    echo "Starting all services with docker-compose..."
    echo ""
    
    # Check if .env exists
    if [ ! -f .env.docker ]; then
        echo -e "${YELLOW}Warning: .env.docker not found, using defaults${NC}"
    fi
    
    # Start services
    docker-compose up -d
    
    echo ""
    echo -e "${GREEN}✅ Services started!${NC}"
    echo ""
    echo "Access:"
    echo "  Frontend: http://localhost:3000"
    echo "  Backend API: http://localhost:8000/docs"
    echo "  Model Worker: http://localhost:8001"
    echo ""
    echo "View logs:"
    echo "  docker-compose logs -f"
    echo ""
    echo "Stop services:"
    echo "  docker-compose down"
    echo ""
    
elif [ "$choice" == "2" ]; then
    echo ""
    echo -e "${GREEN}[Local Development]${NC}"
    echo ""
    
    # Check dependencies
    echo "Checking dependencies..."
    
    if ! command -v node &> /dev/null; then
        echo "❌ Node.js not found. Install from https://nodejs.org"
        exit 1
    fi
    
    if ! command -v python3 &> /dev/null; then
        echo "❌ Python not found. Install from https://python.org"
        exit 1
    fi
    
    echo -e "${GREEN}✅ Dependencies OK${NC}"
    echo ""
    
    # Setup backend
    echo "Setting up backend..."
    cd backend-python
    
    if [ ! -d "venv" ]; then
        echo "Creating virtual environment..."
        python3 -m venv venv
    fi
    
    source venv/bin/activate
    
    if [ ! -f ".env" ]; then
        echo "Creating .env from template..."
        cp .env.example .env
        echo -e "${YELLOW}⚠️  Please update .env with your SQL Server credentials${NC}"
    fi
    
    echo "Installing Python dependencies..."
    pip install -q -r requirements.txt
    
    cd ..
    
    # Setup frontend
    echo "Setting up frontend..."
    if [ ! -d "node_modules" ]; then
        echo "Installing npm dependencies..."
        npm install --silent
    fi
    
    echo ""
    echo -e "${GREEN}✅ Setup complete!${NC}"
    echo ""
    echo "To start services:"
    echo ""
    echo "Terminal 1 - Backend:"
    echo "  cd backend-python"
    echo "  source venv/bin/activate"
    echo "  python main.py"
    echo ""
    echo "Terminal 2 - Frontend:"
    echo "  npm run dev"
    echo ""
    echo "Terminal 3 - Model Worker (optional):"
    echo "  cd model-worker"
    echo "  python app.py"
    echo ""
    
else
    echo "Invalid choice"
    exit 1
fi
