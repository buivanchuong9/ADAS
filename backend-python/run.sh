#!/bin/bash
# ADAS Backend Python - Run Script

echo "Starting ADAS Backend..."
echo ""

# Check venv
if [ ! -d "venv" ]; then
    echo "ERROR: Virtual environment not found"
    echo "Please run setup.sh first"
    exit 1
fi

# Activate venv
source venv/bin/activate

# Check .env
if [ ! -f .env ]; then
    echo "WARNING: .env file not found"
    echo "Using .env.example instead - this may not work!"
    echo ""
fi

# Run server
echo "Server starting at http://localhost:8000"
echo "Docs at http://localhost:8000/docs"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python main.py
