#!/bin/bash
# ============================================
# ADAS Backend - Quick Start Script
# One-command server startup for macOS/Linux
# ============================================

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "=================================================="
echo "🚀 ADAS Backend Server - Quick Start"
echo "=================================================="
echo -e "${NC}"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check Python
echo -e "${YELLOW}[1/4] Checking Python...${NC}"
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 not found! Please install Python 3.8+${NC}"
    exit 1
fi
PYTHON_VERSION=$(python3 --version)
echo -e "${GREEN}✅ $PYTHON_VERSION${NC}"

# Check/Install dependencies
echo -e "${YELLOW}[2/4] Checking dependencies...${NC}"
if ! python3 -c "import fastapi" 2>/dev/null; then
    echo -e "${YELLOW}Installing requirements...${NC}"
    pip3 install -r requirements.txt --quiet
    echo -e "${GREEN}✅ Dependencies installed${NC}"
else
    echo -e "${GREEN}✅ Dependencies OK${NC}"
fi

# Kill existing server
echo -e "${YELLOW}[3/4] Stopping old server (if any)...${NC}"
# Kill by process name
pkill -9 -f "python3 main.py" 2>/dev/null || true
pkill -9 -f "python3.*uvicorn" 2>/dev/null || true
# Kill by port
lsof -ti:8000 | xargs kill -9 2>/dev/null || true
sleep 2
echo -e "${GREEN}✅ Ready${NC}"

# Start server
echo -e "${YELLOW}[4/4] Starting server...${NC}"
echo -e "${BLUE}"
echo "=================================================="
echo "📊 API Docs: http://localhost:8000/docs"
echo "💚 Health: http://localhost:8000/health"
echo "🎯 Alerts: http://localhost:8000/api/alerts/latest"
echo "📦 Dataset: http://localhost:8000/api/dataset/stats"
echo "=================================================="
echo -e "${NC}"
echo -e "${GREEN}Press Ctrl+C to stop${NC}"
echo ""

python3 main.py
