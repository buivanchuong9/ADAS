#!/bin/bash

# Quick Install Script for Cloudflare Tunnel
# This script checks and installs cloudflared if needed

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}Cloudflare Tunnel Setup for ADAS Backend${NC}"
echo ""

# Check if cloudflared is already installed
if command -v cloudflared &> /dev/null; then
    VERSION=$(cloudflared --version 2>&1 | head -1)
    echo -e "${GREEN}✅ cloudflared is already installed!${NC}"
    echo -e "${BLUE}   Version: ${VERSION}${NC}"
    echo ""
    echo -e "${GREEN}You're ready to go! Run:${NC}"
    echo -e "${YELLOW}   ./start-tunnel.sh${NC}"
    exit 0
fi

echo -e "${YELLOW}⚠️  cloudflared not found. Installing...${NC}"
echo ""

# Detect OS and install
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo -e "${BLUE}Detected: macOS${NC}"
    
    if command -v brew &> /dev/null; then
        echo -e "${BLUE}Installing via Homebrew...${NC}"
        brew install cloudflared
        echo -e "${GREEN}✅ Installation complete!${NC}"
    else
        echo -e "${RED}❌ Homebrew not found.${NC}"
        echo ""
        echo -e "${YELLOW}Option 1: Install Homebrew first${NC}"
        echo -e "   /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
        echo ""
        echo -e "${YELLOW}Option 2: Download cloudflared manually${NC}"
        echo -e "   Visit: ${BLUE}https://developers.cloudflare.com/cloudflare-one/connections/connect-apps/install-and-setup/installation/${NC}"
        exit 1
    fi
    
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo -e "${BLUE}Detected: Linux${NC}"
    
    # Check architecture
    ARCH=$(uname -m)
    if [[ "$ARCH" == "x86_64" ]]; then
        echo -e "${BLUE}Downloading cloudflared for Linux (amd64)...${NC}"
        wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64.deb
        sudo dpkg -i cloudflared-linux-amd64.deb
        rm cloudflared-linux-amd64.deb
        echo -e "${GREEN}✅ Installation complete!${NC}"
    elif [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == "arm64" ]]; then
        echo -e "${BLUE}Downloading cloudflared for Linux (arm64)...${NC}"
        wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64.deb
        sudo dpkg -i cloudflared-linux-arm64.deb
        rm cloudflared-linux-arm64.deb
        echo -e "${GREEN}✅ Installation complete!${NC}"
    else
        echo -e "${RED}❌ Unsupported architecture: $ARCH${NC}"
        echo -e "${YELLOW}Please install cloudflared manually from:${NC}"
        echo -e "${BLUE}https://developers.cloudflare.com/cloudflare-one/connections/connect-apps/install-and-setup/installation/${NC}"
        exit 1
    fi
    
else
    echo -e "${RED}❌ Unsupported OS: $OSTYPE${NC}"
    echo -e "${YELLOW}Please install cloudflared manually from:${NC}"
    echo -e "${BLUE}https://developers.cloudflare.com/cloudflare-one/connections/connect-apps/install-and-setup/installation/${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║         ✅ Setup Complete! ✅                          ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Next steps:${NC}"
echo -e "  1. Start your backend: ${YELLOW}cd backend-python && python main.py${NC}"
echo -e "  2. Start the tunnel:   ${YELLOW}./start-tunnel.sh${NC}"
echo ""
