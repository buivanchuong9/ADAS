# ADAS Platform Quick Start
# PowerShell Script for Windows

Write-Host "🚀 ADAS Platform Quick Start" -ForegroundColor Green
Write-Host "================================" -ForegroundColor Green
Write-Host ""

# Check Docker
Write-Host "Checking Docker..." -ForegroundColor Cyan
if (!(Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Host "❌ Docker chưa cài đặt!" -ForegroundColor Red
    Write-Host ""
    Write-Host "Vui lòng cài Docker Desktop:" -ForegroundColor Yellow
    Write-Host "https://www.docker.com/products/docker-desktop/" -ForegroundColor Cyan
    Write-Host ""
    pause
    exit 1
}

# Check if Docker is running
try {
    docker info | Out-Null
    Write-Host "✅ Docker đang chạy" -ForegroundColor Green
} catch {
    Write-Host "❌ Docker Desktop chưa mở!" -ForegroundColor Red
    Write-Host "Vui lòng mở Docker Desktop và chờ nó khởi động" -ForegroundColor Yellow
    Write-Host ""
    pause
    exit 1
}

Write-Host ""

# Check if images exist
Write-Host "Checking images..." -ForegroundColor Cyan
$backendExists = docker images | Select-String "adas-platform1-backend"
$frontendExists = docker images | Select-String "adas-platform1-frontend"

if ($backendExists -and $frontendExists) {
    Write-Host "✅ Images đã có sẵn" -ForegroundColor Green
    Write-Host ""
    Write-Host "Starting services..." -ForegroundColor Cyan
    docker compose up
} else {
    Write-Host "⚠️  Images chưa có" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Chọn phương án setup:" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "1. Build từ source (10-12 phút - lần đầu)" -ForegroundColor White
    Write-Host "2. Load pre-built images từ Drive (2 phút - nhanh)" -ForegroundColor White
    Write-Host ""
    
    $choice = Read-Host "Nhập lựa chọn (1/2)"
    
    if ($choice -eq "1") {
        Write-Host ""
        Write-Host "☕ Building từ source..." -ForegroundColor Yellow
        Write-Host "Thời gian dự kiến: 10-12 phút" -ForegroundColor Yellow
        Write-Host "Đi uống cafe hoặc làm việc khác nhé!" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Đang build..." -ForegroundColor Cyan
        docker compose up --build
        
    } elseif ($choice -eq "2") {
        Write-Host ""
        Write-Host "📥 Hướng dẫn download pre-built images:" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Bước 1: Download 2 files từ Google Drive" -ForegroundColor Cyan
        Write-Host "  - backend.tar (~2GB)" -ForegroundColor White
        Write-Host "  - frontend.tar (~200MB)" -ForegroundColor White
        Write-Host ""
        Write-Host "Bước 2: Đặt 2 files vào thư mục này:" -ForegroundColor Cyan
        Write-Host "  $PWD" -ForegroundColor White
        Write-Host ""
        Write-Host "Link Google Drive: [HỎI TEAM LEAD ĐỂ LẤY LINK]" -ForegroundColor Yellow
        Write-Host ""
        
        $ready = Read-Host "Đã download và đặt file xong? (y/n)"
        
        if ($ready -eq "y" -or $ready -eq "Y") {
            Write-Host ""
            Write-Host "Checking files..." -ForegroundColor Cyan
            
            if ((Test-Path "backend.tar") -and (Test-Path "frontend.tar")) {
                Write-Host "✅ Tìm thấy cả 2 files" -ForegroundColor Green
                Write-Host ""
                Write-Host "Loading backend image..." -ForegroundColor Cyan
                docker load -i backend.tar
                
                Write-Host "Loading frontend image..." -ForegroundColor Cyan
                docker load -i frontend.tar
                
                Write-Host ""
                Write-Host "✅ Images loaded thành công!" -ForegroundColor Green
                Write-Host ""
                Write-Host "Starting services..." -ForegroundColor Cyan
                docker compose up
                
            } else {
                Write-Host ""
                Write-Host "❌ Không tìm thấy backend.tar hoặc frontend.tar" -ForegroundColor Red
                Write-Host ""
                Write-Host "Vui lòng:" -ForegroundColor Yellow
                Write-Host "1. Download 2 files từ Drive" -ForegroundColor White
                Write-Host "2. Đặt vào thư mục: $PWD" -ForegroundColor White
                Write-Host "3. Chạy lại script này" -ForegroundColor White
                Write-Host ""
                pause
                exit 1
            }
        } else {
            Write-Host ""
            Write-Host "Hủy bỏ. Chạy lại script khi đã download xong." -ForegroundColor Yellow
            Write-Host ""
            pause
            exit 0
        }
        
    } else {
        Write-Host ""
        Write-Host "❌ Lựa chọn không hợp lệ. Vui lòng chọn 1 hoặc 2" -ForegroundColor Red
        Write-Host ""
        pause
        exit 1
    }
}

Write-Host ""
Write-Host "================================" -ForegroundColor Green
Write-Host "🎉 Services đã start!" -ForegroundColor Green
Write-Host ""
Write-Host "Truy cập ứng dụng tại:" -ForegroundColor Cyan
Write-Host "  - Homepage: http://localhost:3000" -ForegroundColor White
Write-Host "  - ADAS Detection: http://localhost:3000/adas" -ForegroundColor White
Write-Host "  - Backend API: http://localhost:8000/docs" -ForegroundColor White
Write-Host ""
Write-Host "Nhấn Ctrl+C để dừng services" -ForegroundColor Yellow
Write-Host "================================" -ForegroundColor Green
