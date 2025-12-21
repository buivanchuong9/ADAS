#!/bin/bash

# ADAS Frontend - Quick Start Script
# Khởi động frontend với production API

cd "$(dirname "$0")/FrontEnd"

echo "🚀 ADAS Frontend - Starting..."
echo "======================================"
echo ""
echo "📡 API Server: https://adas-api.aiotlab.edu.vn"
echo "🌐 Frontend: http://localhost:3000"
echo ""
echo "Installing dependencies..."

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo "Installing packages..."
    pnpm install
fi

echo ""
echo "Starting development server..."
echo ""

pnpm dev
