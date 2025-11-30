#!/bin/bash

# ============================================
# ADAS Platform - First Time Setup Script
# For team members cloning from GitHub
# ============================================

set -e  # Exit on error

echo "🚀 ADAS Platform - First Time Setup"
echo "===================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. Check Docker
echo "🔍 Checking Docker..."
if ! docker --version &> /dev/null; then
    echo -e "${RED}❌ Docker chưa cài đặt!${NC}"
    echo "   Vui lòng cài Docker Desktop từ: https://www.docker.com/products/docker-desktop"
    exit 1
fi

if ! docker ps &> /dev/null; then
    echo -e "${YELLOW}⚠️  Docker Desktop chưa chạy!${NC}"
    echo "   Đang mở Docker Desktop..."
    open -a Docker 2>/dev/null || echo "   Vui lòng mở Docker Desktop thủ công"
    echo "   Đợi 30 giây..."
    sleep 30
fi

echo -e "${GREEN}✅ Docker OK${NC}"

# 2. Check Node.js
echo "🔍 Checking Node.js..."
if ! node --version &> /dev/null; then
    echo -e "${RED}❌ Node.js chưa cài đặt!${NC}"
    echo "   Vui lòng cài Node.js 18+ từ: https://nodejs.org"
    exit 1
fi

NODE_VERSION=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
if [ "$NODE_VERSION" -lt 18 ]; then
    echo -e "${YELLOW}⚠️  Node.js version < 18. Recommended: 18+${NC}"
fi

echo -e "${GREEN}✅ Node.js $(node --version)${NC}"

# 3. Create backend .env
echo ""
echo "📝 Tạo backend environment file..."
cd backend-python

if [ -f .env ]; then
    echo -e "${YELLOW}⚠️  backend-python/.env đã tồn tại, bỏ qua${NC}"
else
    if [ -f .env.example ]; then
        cp .env.example .env
        echo -e "${GREEN}✅ Đã copy .env.example → .env${NC}"
    else
        # Create default .env
        cat > .env << 'EOF'
# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=*

# Database
DATABASE_URL=sqlite:///./adas.db

# Model Weights Directory
WEIGHTS_DIR=/app/ai_models/weights

# Server Configuration
HOST=0.0.0.0
PORT=8000
DEBUG=False

# CORS Origins
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:5173
EOF
        echo -e "${GREEN}✅ Đã tạo backend-python/.env${NC}"
    fi
fi

# 4. Create necessary directories
echo ""
echo "📁 Tạo thư mục cần thiết..."

mkdir -p ai_models/weights
mkdir -p dataset/raw
mkdir -p dataset/labels
mkdir -p dataset/auto_collected
mkdir -p logs/alerts
mkdir -p adas_core/tests/unit
mkdir -p adas_core/tests/integration
mkdir -p adas_core/tests/scenarios

echo -e "${GREEN}✅ Đã tạo các thư mục:${NC}"
echo "   - ai_models/weights/"
echo "   - dataset/"
echo "   - logs/"
echo "   - adas_core/tests/"

cd ..

# 5. Create frontend .env.local
echo ""
echo "📝 Tạo frontend environment file..."

if [ -f .env.local ]; then
    echo -e "${YELLOW}⚠️  .env.local đã tồn tại, bỏ qua${NC}"
else
    echo "NEXT_PUBLIC_API_URL=http://localhost:8000" > .env.local
    echo -e "${GREEN}✅ Đã tạo .env.local${NC}"
fi

# 6. Install frontend dependencies
echo ""
echo "📦 Cài đặt frontend dependencies..."

if command -v pnpm &> /dev/null; then
    echo "   Sử dụng pnpm..."
    pnpm install
elif command -v npm &> /dev/null; then
    echo "   Sử dụng npm..."
    npm install
else
    echo -e "${RED}❌ Không tìm thấy npm hoặc pnpm!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Đã cài đặt dependencies${NC}"

# 7. Summary
echo ""
echo "=========================================="
echo -e "${GREEN}✅ SETUP HOÀN TẤT!${NC}"
echo "=========================================="
echo ""
echo "📋 Cấu trúc đã tạo:"
echo "   ✅ backend-python/.env"
echo "   ✅ .env.local"
echo "   ✅ ai_models/weights/"
echo "   ✅ dataset/, logs/"
echo "   ✅ node_modules/"
echo ""
echo "🚀 Bước tiếp theo:"
echo ""
echo "   1️⃣  Khởi động Backend (Docker):"
echo "       cd backend-python"
echo "       docker compose up --build"
echo ""
echo "   2️⃣  Khởi động Frontend (Terminal mới):"
echo "       pnpm run dev"
echo "       # hoặc: npm run dev"
echo ""
echo "   3️⃣  Mở trình duyệt:"
echo "       http://localhost:3000"
echo ""
echo "📚 Xem thêm: SETUP_FOR_TEAM.md"
echo ""
