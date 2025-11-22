#!/bin/bash
# ADAS Backend Python - Setup Script

echo "========================================"
echo "ADAS Backend Python - Setup"
echo "========================================"
echo ""

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 is not installed"
    exit 1
fi

echo "[1/5] Python version:"
python3 --version
echo ""

echo "[2/5] Creating virtual environment..."
python3 -m venv venv
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to create virtual environment"
    exit 1
fi
echo ""

echo "[3/5] Activating virtual environment..."
source venv/bin/activate
echo ""

echo "[4/5] Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to install dependencies"
    exit 1
fi
echo ""

echo "[5/5] Setting up environment..."
if [ ! -f .env ]; then
    cp .env.example .env
    echo "Created .env file - Please update with your SQL Server credentials"
else
    echo ".env file already exists"
fi
echo ""

echo "========================================"
echo "Setup completed successfully!"
echo "========================================"
echo ""
echo "IMPORTANT: Update .env file with your SQL Server credentials"
echo ""
echo "Then run:"
echo "  1. source venv/bin/activate"
echo "  2. python seed.py (to create database tables)"
echo "  3. python main.py (to start server)"
echo ""
echo "Server will be available at: http://localhost:8000"
echo "API docs: http://localhost:8000/docs"
echo ""
