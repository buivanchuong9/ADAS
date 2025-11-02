# ADAS Platform - Multi-Service Launcher (Windows PowerShell)
# Usage: PowerShell -ExecutionPolicy Bypass -File run.ps1

Write-Host ""
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  ADAS Platform - Starting Services" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

# Function to open service in new window
function Start-Service {
    param(
        [string]$Title,
        [string]$WorkingDir,
        [string]$Command,
        [string]$Color = "Green"
    )
    
    Write-Host "Starting $Title..." -ForegroundColor $Color
    $FullPath = (Get-Item -Path $WorkingDir).FullName
    
    $scriptBlock = @"
Set-Location "$FullPath"
Write-Host "============================================"
Write-Host "$Title"
Write-Host "============================================"
Write-Host ""
$Command
if (`$LASTEXITCODE -ne 0) {
    Write-Host "Service stopped or encountered an error." -ForegroundColor Red
    Read-Host "Press Enter to close"
}
"@
    
    Start-Process powershell -ArgumentList "-NoExit", "-Command", $scriptBlock
    Start-Sleep -Seconds 1
}

# Get script directory
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $scriptDir

# Verify Firebase configuration
if (-not (Test-Path "backend/firebase-service-account.json")) {
    Write-Host ""
    Write-Host "WARNING: Firebase service account file not found!" -ForegroundColor Yellow
    Write-Host "Location: backend/firebase-service-account.json" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "The backend will not work without this file." -ForegroundColor Yellow
    Write-Host "Download from: Firebase Console > Project Settings > Service Accounts" -ForegroundColor Yellow
    Write-Host ""
    $proceed = Read-Host "Continue anyway? (y/n)"
    if ($proceed -ne "y") {
        exit
    }
}

Write-Host ""

# Start Model Worker
Start-Service `
    -Title "Model Worker (FastAPI)" `
    -WorkingDir "./model-worker" `
    -Command "uvicorn app:app --host 0.0.0.0 --port 8000" `
    -Color "Yellow"

# Start Backend
Start-Service `
    -Title "Backend API (.NET)" `
    -WorkingDir "./backend" `
    -Command "dotnet run" `
    -Color "Blue"

# Start Frontend
Start-Service `
    -Title "Frontend (Next.js)" `
    -WorkingDir "." `
    -Command "npm run dev" `
    -Color "Green"

Write-Host ""
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  All Services Started!" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Access the application:" -ForegroundColor Green
Write-Host ""
Write-Host "  Dashboard:        http://localhost:3000/dashboard" -ForegroundColor White
Write-Host "  Live Detection:   http://localhost:3000/live" -ForegroundColor White
Write-Host "  Model Worker API: http://localhost:8000/health" -ForegroundColor White
Write-Host ""
Write-Host "Close any window to stop that service." -ForegroundColor Yellow
Write-Host ""

# Keep script running
while ($true) {
    Start-Sleep -Seconds 10
}
Write-Host ""
Write-Host "Nhấn bất kỳ phím nào để thoát script này..." -ForegroundColor Yellow
Read-Host
