@echo off
REM ===================================================
REM ADAS Backend - Windows Server Deployment Script
REM Run this script to deploy ADAS backend on Windows Server
REM ===================================================

echo.
echo =====================================================
echo    ADAS Backend Deployment for Windows Server
echo =====================================================
echo.

REM Check Python installation
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed!
    echo Please install Python 3.9+ from https://www.python.org/downloads/
    pause
    exit /b 1
)

echo [1/7] Python detected...
python --version

REM Check SQL Server connectivity
echo.
echo [2/7] Checking SQL Server connection...
sqlcmd -S localhost -U sa -P "YourPassword123!" -Q "SELECT @@VERSION" >nul 2>&1
if %errorlevel% neq 0 (
    echo [WARNING] SQL Server not accessible with default credentials
    echo Please update DATABASE_URL in .env file with correct credentials
    echo.
    set /p CONTINUE="Continue anyway? (Y/N): "
    if /i not "%CONTINUE%"=="Y" exit /b 1
) else (
    echo [OK] SQL Server connection successful
)

REM Create virtual environment
echo.
echo [3/7] Creating Python virtual environment...
if not exist venv (
    python -m venv venv
    echo [OK] Virtual environment created
) else (
    echo [OK] Virtual environment already exists
)

REM Activate virtual environment and install dependencies
echo.
echo [4/7] Installing Python dependencies...
call venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt

REM Check .env configuration
echo.
echo [5/7] Checking configuration...
if not exist .env (
    echo [WARNING] .env file not found!
    echo Creating .env from template...
    copy .env.example .env >nul 2>&1
    echo [OK] Please update .env with your SQL Server credentials
    notepad .env
    pause
)

REM Initialize database
echo.
echo [6/7] Initializing database...
python -c "from database import engine, test_connection; from models import Base; Base.metadata.create_all(bind=engine); test_connection()"
if %errorlevel% neq 0 (
    echo [ERROR] Database initialization failed!
    echo Please check your SQL Server connection settings in .env
    pause
    exit /b 1
)
echo [OK] Database initialized

REM Start backend server
echo.
echo [7/7] Starting ADAS Backend Server...
echo.
echo =====================================================
echo    Backend running on http://localhost:8000
echo    API Docs: http://localhost:8000/docs
echo    Health Check: http://localhost:8000/health
echo =====================================================
echo.
echo Press Ctrl+C to stop the server
echo.

python main.py

pause
