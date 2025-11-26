@echo off
REM ================================================================
REM ADAS Backend - One-Command Start (Windows)
REM Tu dong cai thu vien va chay server
REM ================================================================

echo.
echo ================================================================
echo    ADAS Backend - Automatic Setup ^& Start
echo ================================================================
echo.

cd /d "%~dp0"

REM Check Python
python --version >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERROR] Python khong tim thay!
    echo Vui long cai Python 3.8+ tu python.org
    pause
    exit /b 1
)

REM Run start.py
python start.py

pause
