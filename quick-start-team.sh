#!/bin/bash

echo "🚀 ADAS Platform - Quick Start for Team"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}❌ Docker không chạy! Vui lòng mở Docker Desktop${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Docker đang chạy${NC}"
echo ""

# Check if images exist
if docker images | grep -q "adas-platform1-backend" && docker images | grep -q "adas-platform1-frontend"; then
    echo -e "${GREEN}✅ Images đã có sẵn, starting services...${NC}"
    docker compose up
else
    echo -e "${YELLOW}⚠️  Images chưa có, có 2 lựa chọn:${NC}"
    echo ""
    echo "1. Build từ source (10-12 phút)"
    echo "2. Download pre-built images từ Drive (2 phút)"
    echo ""
    read -p "Chọn (1/2): " choice
    
    if [ "$choice" = "1" ]; then
        echo -e "${YELLOW}☕ Building... Đi uống cafe 10 phút nhé!${NC}"
        docker compose up --build
    elif [ "$choice" = "2" ]; then
        echo ""
        echo -e "${YELLOW}📥 Download 2 files từ Google Drive:${NC}"
        echo "- backend.tar (~2GB)"
        echo "- frontend.tar (~200MB)"
        echo ""
        echo "Link: [THÊM LINK DRIVE CỦA TEAM]"
        echo ""
        read -p "Đã tải xong và đặt trong thư mục này? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            if [ -f "backend.tar" ] && [ -f "frontend.tar" ]; then
                echo -e "${GREEN}Loading backend image...${NC}"
                docker load -i backend.tar
                echo -e "${GREEN}Loading frontend image...${NC}"
                docker load -i frontend.tar
                echo -e "${GREEN}✅ Images loaded! Starting services...${NC}"
                docker compose up
            else
                echo -e "${RED}❌ Không tìm thấy backend.tar hoặc frontend.tar${NC}"
                exit 1
            fi
        fi
    else
        echo -e "${RED}❌ Lựa chọn không hợp lệ${NC}"
        exit 1
    fi
fi
