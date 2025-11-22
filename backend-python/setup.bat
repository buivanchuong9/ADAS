@echo off
REM ADAS Backend Python - Setup Script for Windows

echo ========================================
echo ADAS Backend Python - Setup
echo ========================================
echo.

REM Check Python installation
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.10+ from https://www.python.org/
    pause
    exit /b 1
)

echo [1/5] Python version:
python --version
echo.

echo [2/5] Creating virtual environment...
python -m venv venv
if errorlevel 1 (
    echo ERROR: Failed to create virtual environment
    pause
    exit /b 1
)
echo.

echo [3/5] Activating virtual environment...
call venv\Scripts\activate.bat
echo.

echo [4/5] Installing dependencies...
pip install --upgrade pip
pip install -r requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies
    pause
    exit /b 1
)
echo.

echo [5/5] Setting up environment...
if not exist .env (
    copy .env.example .env
    echo Created .env file - Please update with your SQL Server credentials
) else (
    echo .env file already exists
)
echo.

echo ========================================
echo Setup completed successfully!
echo ========================================
echo.
echo IMPORTANT: Update .env file with your SQL Server credentials:
echo   - SQL_SERVER
echo   - SQL_DATABASE
echo   - SQL_USERNAME
echo   - SQL_PASSWORD
echo.
echo Then run:
echo   1. venv\Scripts\activate
echo   2. python seed.py (to create database tables)
echo   3. python main.py (to start server)
echo.
echo Server will be available at: http://localhost:8000
echo API docs: http://localhost:8000/docs
echo.
pause
