@echo off
REM ADAS Platform - Quick Start Script

echo ========================================
echo   ADAS Platform - Quick Start
echo ========================================
echo.

echo Choose deployment method:
echo 1. Docker (Recommended - Full stack)
echo 2. Local Development (Manual setup)
echo.
set /p choice="Enter choice (1 or 2): "

if "%choice%"=="1" goto docker
if "%choice%"=="2" goto local
echo Invalid choice
exit /b 1

:docker
echo.
echo [Docker Deployment]
echo Starting all services with docker-compose...
echo.

REM Check if .env.docker exists
if not exist .env.docker (
    echo Warning: .env.docker not found, using defaults
)

REM Start services
docker-compose up -d

echo.
echo Services started!
echo.
echo Access:
echo   Frontend: http://localhost:3000
echo   Backend API: http://localhost:8000/docs
echo   Model Worker: http://localhost:8001
echo.
echo View logs:
echo   docker-compose logs -f
echo.
echo Stop services:
echo   docker-compose down
echo.
pause
exit /b 0

:local
echo.
echo [Local Development]
echo.

REM Check dependencies
echo Checking dependencies...

where node >nul 2>&1
if errorlevel 1 (
    echo ERROR: Node.js not found. Install from https://nodejs.org
    pause
    exit /b 1
)

where python >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found. Install from https://python.org
    pause
    exit /b 1
)

echo Dependencies OK
echo.

REM Setup backend
echo Setting up backend...
cd backend-python

if not exist venv\ (
    echo Creating virtual environment...
    python -m venv venv
)

call venv\Scripts\activate.bat

if not exist .env (
    echo Creating .env from template...
    copy .env.example .env
    echo WARNING: Please update .env with your SQL Server credentials
)

echo Installing Python dependencies...
pip install -q -r requirements.txt

cd ..

REM Setup frontend
echo Setting up frontend...
if not exist node_modules\ (
    echo Installing npm dependencies...
    npm install --silent
)

echo.
echo Setup complete!
echo.
echo To start services:
echo.
echo Terminal 1 - Backend:
echo   cd backend-python
echo   venv\Scripts\activate
echo   python main.py
echo.
echo Terminal 2 - Frontend:
echo   npm run dev
echo.
echo Terminal 3 - Model Worker (optional):
echo   cd model-worker
echo   python app.py
echo.
pause
exit /b 0
