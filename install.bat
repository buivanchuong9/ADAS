@echo off
REM Script cài đặt ADAS Platform cho Windows

echo ============================================
echo ADAS Platform - Installation Script (Windows)
echo ============================================
echo.

REM Kiểm tra Node.js
echo [1/6] Kiểm tra Node.js...
node --version >nul 2>&1
if errorlevel 1 (
    echo Node.js chưa được cài đặt!
    echo Tải từ: https://nodejs.org/
    exit /b 1
)
echo Node.js: %NODE_VERSION%

REM Kiểm tra .NET SDK
echo [2/6] Kiểm tra .NET SDK...
dotnet --version >nul 2>&1
if errorlevel 1 (
    echo .NET SDK chưa được cài đặt!
    echo Tải từ: https://dotnet.microsoft.com/en-us/download
    exit /b 1
)
echo .NET SDK cài đặt thành công

REM Kiểm tra Python
echo [3/6] Kiểm tra Python...
python --version >nul 2>&1
if errorlevel 1 (
    python3 --version >nul 2>&1
    if errorlevel 1 (
        echo Python chưa được cài đặt!
        echo Tải từ: https://www.python.org/downloads/
        exit /b 1
    )
)
echo Python cài đặt thành công

REM Cài npm packages cho frontend
echo [4/6] Cài npm packages...
call npm install --legacy-peer-deps
if errorlevel 1 (
    echo Lỗi cài npm packages
    exit /b 1
)
echo npm packages cài đặt thành công

REM Cài Python dependencies cho model worker
echo [5/6] Cài Python dependencies...
cd model-worker
pip install -r requirements.txt
if errorlevel 1 (
    echo Lỗi cài Python dependencies
    exit /b 1
)
cd ..
echo Python dependencies cài đặt thành công

REM Restore .NET packages
echo [6/6] Restore .NET packages...
cd backend
dotnet restore
if errorlevel 1 (
    echo Lỗi restore .NET packages
    exit /b 1
)
cd ..
echo .NET packages restore thành công

echo.
echo ============================================
echo Cài đặt hoàn tất!
echo ============================================
echo.
echo Bước tiếp theo - Chạy ứng dụng:
echo.
echo Terminal 1 - Model Worker (Python):
echo   cd model-worker
echo   uvicorn app:app --host 0.0.0.0 --port 8000
echo.
echo Terminal 2 - Backend (.NET):
echo   cd backend
echo   dotnet run
echo.
echo Terminal 3 - Frontend (Next.js):
echo   npm run dev
echo.
echo Truy cập: http://localhost:3000
echo Dashboard: http://localhost:3000/dashboard
echo Live Detection: http://localhost:3000/live
echo.
pause
