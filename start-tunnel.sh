#!/bin/bash

# ADAS Backend Cloudflare Tunnel Starter
# This script exposes your local backend (http://localhost:8080) to the internet
# so that any device on any network can access your API

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Log file
LOG_FILE="tunnel.log"
URL_FILE=".tunnel-url"

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     ADAS Backend Cloudflare Tunnel Starter            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if cloudflared is installed
if ! command -v cloudflared &> /dev/null; then
    echo -e "${YELLOW}⚠️  cloudflared not found. Installing...${NC}"
    
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        if command -v brew &> /dev/null; then
            echo -e "${BLUE}Installing via Homebrew...${NC}"
            brew install cloudflared
        else
            echo -e "${RED}❌ Homebrew not found. Please install Homebrew first or install cloudflared manually:${NC}"
            echo -e "${YELLOW}Visit: https://developers.cloudflare.com/cloudflare-one/connections/connect-apps/install-and-setup/installation/${NC}"
            exit 1
        fi
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux
        echo -e "${BLUE}Installing cloudflared for Linux...${NC}"
        wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64.deb
        sudo dpkg -i cloudflared-linux-amd64.deb
        rm cloudflared-linux-amd64.deb
    else
        echo -e "${RED}❌ Unsupported OS. Please install cloudflared manually:${NC}"
        echo -e "${YELLOW}Visit: https://developers.cloudflare.com/cloudflare-one/connections/connect-apps/install-and-setup/installation/${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ cloudflared installed successfully!${NC}"
    echo ""
fi

# Check if backend is running
echo -e "${BLUE}🔍 Checking if backend is running on http://localhost:8080...${NC}"
if ! curl -s http://localhost:8080 > /dev/null 2>&1 && ! curl -s http://localhost:8080/docs > /dev/null 2>&1; then
    echo -e "${YELLOW}⚠️  Backend doesn't appear to be running on port 8080${NC}"
    echo -e "${YELLOW}   Please start your backend first with:${NC}"
    echo -e "${YELLOW}   cd backend-python && python main.py${NC}"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "${GREEN}✅ Backend is running!${NC}"
fi

echo ""
echo -e "${BLUE}🚀 Starting Cloudflare Tunnel...${NC}"
echo -e "${YELLOW}📝 Logs will be written to: ${LOG_FILE}${NC}"
echo ""

# Clean up old URL file
rm -f "$URL_FILE"

# Start cloudflared tunnel in background and capture output
cloudflared tunnel --url http://localhost:8080 --loglevel info 2>&1 | tee "$LOG_FILE" | while IFS= read -r line; do
    echo "$line"
    
    # Extract the public URL from cloudflared output
    if echo "$line" | grep -q "https://.*trycloudflare.com"; then
        TUNNEL_URL=$(echo "$line" | grep -oP 'https://[^[:space:]]+\.trycloudflare\.com' | head -1)
        
        if [ ! -z "$TUNNEL_URL" ]; then
            # Save URL to file
            echo "$TUNNEL_URL" > "$URL_FILE"
            
            echo ""
            echo -e "${GREEN}╔════════════════════════════════════════════════════════╗${NC}"
            echo -e "${GREEN}║              🎉 TUNNEL IS READY! 🎉                    ║${NC}"
            echo -e "${GREEN}╚════════════════════════════════════════════════════════╝${NC}"
            echo ""
            echo -e "${YELLOW}📡 Public URL:${NC} ${GREEN}${TUNNEL_URL}${NC}"
            echo ""
            echo -e "${BLUE}Example API endpoints:${NC}"
            echo -e "  • ${TUNNEL_URL}/docs"
            echo -e "  • ${TUNNEL_URL}/api/detection/status"
            echo -e "  • ${TUNNEL_URL}/api/analytics/summary"
            echo ""
            echo -e "${YELLOW}💡 Update your frontend API config to:${NC}"
            echo -e "   ${GREEN}export const API_BASE_URL = '${TUNNEL_URL}';${NC}"
            echo ""
            
            # Copy to clipboard
            if command -v pbcopy &> /dev/null; then
                # macOS
                echo "$TUNNEL_URL" | pbcopy
                echo -e "${GREEN}✅ URL copied to clipboard!${NC}"
            elif command -v xclip &> /dev/null; then
                # Linux with xclip
                echo "$TUNNEL_URL" | xclip -selection clipboard
                echo -e "${GREEN}✅ URL copied to clipboard!${NC}"
            elif command -v xsel &> /dev/null; then
                # Linux with xsel
                echo "$TUNNEL_URL" | xsel --clipboard
                echo -e "${GREEN}✅ URL copied to clipboard!${NC}"
            else
                echo -e "${YELLOW}⚠️  Clipboard utility not found. Please copy the URL manually.${NC}"
            fi
            
            echo ""
            echo -e "${BLUE}Press Ctrl+C to stop the tunnel${NC}"
            echo ""
        fi
    fi
done

# Cleanup on exit
trap 'echo -e "\n${YELLOW}🛑 Stopping tunnel...${NC}"; rm -f "$URL_FILE"; exit 0' INT TERM
