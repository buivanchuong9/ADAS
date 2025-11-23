#!/bin/bash

# ADAS Backend - Install Dependencies
# Hướng dẫn cài đặt cho macOS/Linux

echo "🚀 ADAS Backend - Installing Dependencies"
echo "=========================================="

# Check Python version
echo ""
echo "📌 Checking Python version..."
python3 --version

if [ $? -ne 0 ]; then
    echo "❌ Python 3 is not installed!"
    echo "Please install Python 3.8+ first"
    exit 1
fi

# Create virtual environment (recommended)
echo ""
echo "📦 Creating virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "✅ Virtual environment created"
else
    echo "⚠️  Virtual environment already exists"
fi

# Activate virtual environment
echo ""
echo "🔌 Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo ""
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo ""
echo "📥 Installing dependencies from requirements.txt..."
pip install -r requirements.txt

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ Installation failed!"
    echo ""
    echo "💡 Common issues:"
    echo "   1. PyTorch installation - try:"
    echo "      pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu"
    echo ""
    echo "   2. If using GPU (CUDA):"
    echo "      pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118"
    echo ""
    exit 1
fi

echo ""
echo "✅ All dependencies installed successfully!"

# Download YOLO model
echo ""
echo "📥 Downloading YOLOv8 model..."
if [ ! -f "yolov8n.pt" ]; then
    python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt')"
    echo "✅ YOLOv8 model downloaded"
else
    echo "⚠️  YOLOv8 model already exists"
fi

# Create necessary directories
echo ""
echo "📁 Creating directories..."
mkdir -p dataset/raw/videos
mkdir -p dataset/raw/frames
mkdir -p dataset/labels
mkdir -p dataset/training/images/train
mkdir -p dataset/training/images/val
mkdir -p dataset/training/labels/train
mkdir -p dataset/training/labels/val
mkdir -p models/trained
echo "✅ Directories created"

# Test imports
echo ""
echo "🧪 Testing imports..."
python3 -c "
import torch
import cv2
from ultralytics import YOLO
print('✅ All imports successful')
print(f'   - PyTorch: {torch.__version__}')
print(f'   - CUDA available: {torch.cuda.is_available()}')
"

if [ $? -ne 0 ]; then
    echo "❌ Import test failed!"
    exit 1
fi

echo ""
echo "=========================================="
echo "🎉 Installation completed successfully!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Run the server:"
echo "     python3 main.py"
echo ""
echo "  2. Or test the APIs:"
echo "     python3 scripts/test_apis.py"
echo ""
echo "  3. Open API docs:"
echo "     http://localhost:8000/docs"
echo ""
echo "📚 Documentation:"
echo "   - QUICKSTART_TRAINING.md"
echo "   - API_TRAINING_README.md"
echo "   - ARCHITECTURE.md"
echo ""
