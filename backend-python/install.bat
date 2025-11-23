@echo off
REM ADAS Backend - Install Dependencies (Windows)

echo ========================================
echo ADAS Backend - Installing Dependencies
echo ========================================

echo.
echo Checking Python version...
python --version
if errorlevel 1 (
    echo Python is not installed!
    echo Please install Python 3.8+ first
    pause
    exit /b 1
)

echo.
echo Creating virtual environment...
if not exist "venv" (
    python -m venv venv
    echo Virtual environment created
) else (
    echo Virtual environment already exists
)

echo.
echo Activating virtual environment...
call venv\Scripts\activate.bat

echo.
echo Upgrading pip...
python -m pip install --upgrade pip

echo.
echo Installing dependencies...
pip install -r requirements.txt

if errorlevel 1 (
    echo.
    echo Installation failed!
    echo.
    echo Common issues:
    echo   1. PyTorch installation - try:
    echo      pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
    echo.
    echo   2. If using GPU (CUDA):
    echo      pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
    echo.
    pause
    exit /b 1
)

echo.
echo All dependencies installed successfully!

echo.
echo Downloading YOLOv8 model...
if not exist "yolov8n.pt" (
    python -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt')"
    echo YOLOv8 model downloaded
) else (
    echo YOLOv8 model already exists
)

echo.
echo Creating directories...
mkdir dataset\raw\videos 2>nul
mkdir dataset\raw\frames 2>nul
mkdir dataset\labels 2>nul
mkdir dataset\training\images\train 2>nul
mkdir dataset\training\images\val 2>nul
mkdir dataset\training\labels\train 2>nul
mkdir dataset\training\labels\val 2>nul
mkdir models\trained 2>nul
echo Directories created

echo.
echo Testing imports...
python -c "import torch; import cv2; from ultralytics import YOLO; print('All imports successful'); print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"

if errorlevel 1 (
    echo Import test failed!
    pause
    exit /b 1
)

echo.
echo ========================================
echo Installation completed successfully!
echo ========================================
echo.
echo Next steps:
echo   1. Run the server:
echo      python main.py
echo.
echo   2. Or test the APIs:
echo      python scripts\test_apis.py
echo.
echo   3. Open API docs:
echo      http://localhost:8000/docs
echo.
echo Documentation:
echo   - QUICKSTART_TRAINING.md
echo   - API_TRAINING_README.md
echo   - ARCHITECTURE.md
echo.

pause
