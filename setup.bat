@echo off
REM ADAS Platform - Automatic Installation (Windows)
REM This script installs all dependencies and sets up the project

setlocal enabledelayedexpansion

echo.
echo ============================================
echo   ADAS Platform - Automatic Installation
echo ============================================
echo.

REM Check for Administrator privileges
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo This script requires Administrator privileges.
    echo Right-click this file and select "Run as administrator"
    pause
    exit /b 1
)

REM Check Node.js
echo [1/6] Checking Node.js...
node --version >nul 2>&1
if errorlevel 1 (
    echo Node.js not found. Installing...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "& {Invoke-WebRequest -Uri 'https://nodejs.org/dist/v20.11.0/node-v20.11.0-x64.msi' -OutFile 'node-installer.msi'; Start-Process 'node-installer.msi' -ArgumentList '/quiet' -Wait; del 'node-installer.msi'}"
    setx PATH "%PATH%;C:\Program Files\nodejs"
) else (
    for /f "tokens=*" %%i in ('node --version') do echo. Node.js: %%i
)

REM Check .NET SDK
echo [2/6] Checking .NET SDK...
dotnet --version >nul 2>&1
if errorlevel 1 (
    echo .NET SDK not found. Installing...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "& {Invoke-WebRequest -Uri 'https://dot.net/v1/dotnet-install.ps1' -OutFile 'dotnet-install.ps1'; powershell -ExecutionPolicy Bypass .\dotnet-install.ps1; del 'dotnet-install.ps1'}"
) else (
    for /f "tokens=*" %%i in ('dotnet --version') do echo. .NET SDK: %%i
)

REM Check Python
echo [3/6] Checking Python...
python --version >nul 2>&1
if errorlevel 1 (
    echo Python not found. Installing...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "& {Invoke-WebRequest -Uri 'https://www.python.org/ftp/python/3.11.7/python-3.11.7-amd64.exe' -OutFile 'python-installer.exe'; Start-Process 'python-installer.exe' -ArgumentList '/quiet InstallAllUsers=1 PrependPath=1' -Wait; del 'python-installer.exe'}"
) else (
    for /f "tokens=*" %%i in ('python --version') do echo. Python: %%i
)

REM Refresh PATH
setx PATH "%PATH%"

REM Install npm packages
echo.
echo [4/6] Installing npm packages...
call npm install --legacy-peer-deps
if errorlevel 1 (
    echo Failed to install npm packages
    pause
    exit /b 1
)
echo. npm packages installed successfully

REM Install Python dependencies
echo.
echo [5/6] Installing Python dependencies...
cd model-worker
call pip install -r requirements.txt
if errorlevel 1 (
    echo Failed to install Python dependencies
    pause
    exit /b 1
)
cd ..
echo. Python dependencies installed successfully

REM Restore .NET packages
echo.
echo [6/6] Restoring .NET packages...
cd backend
call dotnet restore
if errorlevel 1 (
    echo Failed to restore .NET packages
    pause
    exit /b 1
)
cd ..
echo. .NET packages restored successfully

REM Check Firebase
echo.
echo Checking Firebase configuration...
if not exist "backend\firebase-service-account.json" (
    echo.
    echo WARNING: Firebase service account file not found!
    echo Please download it from Firebase Console:
    echo   1. Go to Firebase Console
    echo   2. Project Settings ^> Service Accounts
    echo   3. Generate new private key
    echo   4. Save as: backend\firebase-service-account.json
    echo.
    echo The backend will not work without this file!
) else (
    echo. Firebase service account found
)

echo.
echo ============================================
echo   Installation Complete! ✅
echo ============================================
echo.
echo Next Steps:
echo.
echo   1. Make sure you have firebase-service-account.json in backend folder
echo.
echo   2. Run the project with run.bat:
echo      run.bat
echo.
echo   Or run services manually in separate terminals:
echo.
echo   Terminal 1 - Model Worker:
echo     cd model-worker
echo     uvicorn app:app --host 0.0.0.0 --port 8000
echo.
echo   Terminal 2 - Backend:
echo     cd backend
echo     dotnet run
echo.
echo   Terminal 3 - Frontend:
echo     npm run dev
echo.
echo   3. Access the application:
echo      Dashboard: http://localhost:3000/dashboard
echo      Live Detection: http://localhost:3000/live
echo.
echo ============================================
echo.

pause
