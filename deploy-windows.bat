@echo off
REM ============================================
REM ADAS Platform - Windows Server Deployment
REM One-command deployment script
REM ============================================

echo.
echo ========================================
echo    ADAS Platform Deployment
echo    Windows Server Edition
echo ========================================
echo.

REM Check if running as Administrator
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERROR] Please run as Administrator!
    pause
    exit /b 1
)

echo [1/6] Checking Python installation...
python --version >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERROR] Python 3.8+ is required! Please install from python.org
    pause
    exit /b 1
)
echo [OK] Python found

echo.
echo [2/6] Installing Python dependencies...
cd backend-python
python -m pip install --upgrade pip
pip install -r requirements.txt
if %errorLevel% neq 0 (
    echo [ERROR] Failed to install dependencies
    pause
    exit /b 1
)
echo [OK] Dependencies installed

echo.
echo [3/6] Checking SQL Server...
sqlcmd -S localhost -E -Q "SELECT @@VERSION" >nul 2>&1
if %errorLevel% neq 0 (
    echo [WARNING] SQL Server not detected or not accessible
    echo Please ensure SQL Server is installed and running
    echo You can download SQL Server Express from microsoft.com
    pause
)
echo [OK] SQL Server ready

echo.
echo [4/6] Creating database...
sqlcmd -S localhost -E -Q "IF NOT EXISTS (SELECT * FROM sys.databases WHERE name = 'ADAS_DB') CREATE DATABASE ADAS_DB"
if %errorLevel% neq 0 (
    echo [WARNING] Could not create database, trying with SQL Auth...
    set /p SA_PASSWORD="Enter SQL Server SA password (or press Enter to skip): "
    if not "!SA_PASSWORD!"=="" (
        sqlcmd -S localhost -U sa -P !SA_PASSWORD! -Q "IF NOT EXISTS (SELECT * FROM sys.databases WHERE name = 'ADAS_DB') CREATE DATABASE ADAS_DB"
    )
)
echo [OK] Database ready

echo.
echo [5/6] Creating database schema...
if exist "..\database\sql-server-schema.sql" (
    sqlcmd -S localhost -E -d ADAS_DB -i ..\database\sql-server-schema.sql
    if %errorLevel% neq 0 (
        echo [WARNING] Schema creation failed, continuing anyway...
    ) else (
        echo [OK] Schema created
    )
) else (
    echo [SKIP] Schema file not found
)

echo.
echo [6/6] Starting ADAS Backend Server...
echo.
echo ========================================
echo    Server Starting...
echo ========================================
echo.
echo API Documentation: http://localhost:8000/docs
echo Health Check: http://localhost:8000/health
echo.
echo Press Ctrl+C to stop the server
echo.

REM Start the server
python main.py

echo.
echo Server stopped.
pause
