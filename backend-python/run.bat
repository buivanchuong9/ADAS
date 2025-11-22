@echo off
REM ADAS Backend Python - Run Script

echo Starting ADAS Backend...
echo.

REM Check if venv exists
if not exist venv\ (
    echo ERROR: Virtual environment not found
    echo Please run setup.bat first
    pause
    exit /b 1
)

REM Activate venv
call venv\Scripts\activate.bat

REM Check .env
if not exist .env (
    echo WARNING: .env file not found
    echo Using .env.example instead - this may not work!
    echo.
)

REM Run server
echo Server starting at http://localhost:8000
echo Docs at http://localhost:8000/docs
echo.
echo Press Ctrl+C to stop
echo.

python main.py
