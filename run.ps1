# Script chạy ADAS Platform trên Windows
# Sử dụng: PowerShell -ExecutionPolicy Bypass -File run.ps1

Write-Host "============================================" -ForegroundColor Green
Write-Host "ADAS Platform - Multi-Terminal Runner" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host ""

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition

# Hàm mở terminal mới
function Start-ProcessInNewWindow {
    param(
        [string]$WorkingDirectory,
        [string]$Command,
        [string]$Title
    )
    
    Write-Host "Khởi động: $Title" -ForegroundColor Yellow
    Start-Process -WorkingDirectory $WorkingDirectory -NoNewWindow -ArgumentList "/c $Command" -UseNewEnvironment
}

# Terminal 1: Model Worker (Python)
Write-Host "1. Khởi động Model Worker (FastAPI, Port 8000)..." -ForegroundColor Cyan
$modelWorkerCmd = "cmd /c cd model-worker && uvicorn app:app --host 0.0.0.0 --port 8000 && pause"
Start-Process "cmd" -ArgumentList "/c $modelWorkerCmd" -WorkingDirectory $scriptDir

Start-Sleep -Seconds 2

# Terminal 2: Backend (.NET)
Write-Host "2. Khởi động Backend (.NET, Port 5000)..." -ForegroundColor Cyan
$backendCmd = "cmd /c cd backend && dotnet run && pause"
Start-Process "cmd" -ArgumentList "/c $backendCmd" -WorkingDirectory $scriptDir

Start-Sleep -Seconds 2

# Terminal 3: Frontend (Next.js)
Write-Host "3. Khởi động Frontend (Next.js, Port 3000)..." -ForegroundColor Cyan
$frontendCmd = "cmd /c npm run dev && pause"
Start-Process "cmd" -ArgumentList "/c $frontendCmd" -WorkingDirectory $scriptDir

Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "Tất cả thành phần đã khởi động!" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host ""
Write-Host "Truy cập:" -ForegroundColor Yellow
Write-Host "  Dashboard: http://localhost:3000/dashboard" -ForegroundColor White
Write-Host "  Live Detection: http://localhost:3000/live" -ForegroundColor White
Write-Host "  Model Worker Health: http://localhost:8000/health" -ForegroundColor White
Write-Host ""
Write-Host "Nhấn bất kỳ phím nào để thoát script này..." -ForegroundColor Yellow
Read-Host
