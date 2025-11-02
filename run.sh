#!/bin/bash
# ADAS Platform - Multi-Service Launcher
# Support: Linux, macOS
# Usage: bash run.sh

set -e

echo ""
echo "============================================"
echo "  ADAS Platform - Starting Services"
echo "============================================"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check Firebase configuration
if [ ! -f "backend/firebase-service-account.json" ]; then
    echo "⚠️  WARNING: Firebase service account file not found!"
    echo "Location: backend/firebase-service-account.json"
    echo ""
    echo "The backend will not work without this file."
    echo "Download from: Firebase Console > Project Settings > Service Accounts"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""

# Detect OS
OS="$(uname -s)"

# Function to open terminal based on OS
open_terminal() {
    local title=$1
    local dir=$2
    local cmd=$3
    
    if [[ "$OS" == "Darwin" ]]; then
        # macOS
        osascript <<EOF
tell application "Terminal"
    do script "cd '$SCRIPT_DIR/$dir' && $cmd"
    activate
end tell
EOF
    elif [[ "$OS" == "Linux" ]]; then
        # Linux - try different terminal emulators
        if command -v gnome-terminal &> /dev/null; then
            gnome-terminal --title="$title" -- bash -c "cd '$SCRIPT_DIR/$dir' && $cmd; bash"
        elif command -v konsole &> /dev/null; then
            konsole --title="$title" -e bash -c "cd '$SCRIPT_DIR/$dir' && $cmd; bash"
        elif command -v xterm &> /dev/null; then
            xterm -title "$title" -e bash -c "cd '$SCRIPT_DIR/$dir' && $cmd; bash" &
        else
            echo "No terminal emulator found. Running in background..."
            bash -c "cd '$SCRIPT_DIR/$dir' && $cmd" &
        fi
    fi
    
    sleep 1
}

# Start services
echo "Starting services..."
echo ""

open_terminal "Model Worker (FastAPI)" "model-worker" "uvicorn app:app --host 0.0.0.0 --port 8000"
echo "✓ Model Worker started (Port 8000)"

open_terminal "Backend API (.NET)" "backend" "dotnet run"
echo "✓ Backend started (Port 5000)"

open_terminal "Frontend (Next.js)" "." "npm run dev"
echo "✓ Frontend started (Port 3000)"

echo ""
echo "============================================"
echo "  All Services Started!"
echo "============================================"
echo ""
echo "Access the application:"
echo ""
echo "  Dashboard:        http://localhost:3000/dashboard"
echo "  Live Detection:   http://localhost:3000/live"
echo "  Model Worker API: http://localhost:8000/health"
echo ""
echo "Close any terminal window to stop that service."
echo ""
