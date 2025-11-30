# 🪟 Hướng Dẫn Setup ADAS Platform Trên Windows

## 📋 Yêu Cầu Hệ Thống

- **Windows 10/11** (64-bit)
- **RAM**: Tối thiểu 8GB (khuyến nghị 16GB)
- **Disk**: 20GB trống
- **Internet**: Tốc độ tốt để download packages

---

## 🔧 Bước 1: Cài Đặt Docker Desktop

### 1.1. Download Docker Desktop

Truy cập: https://www.docker.com/products/docker-desktop/

Hoặc link trực tiếp: https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe

### 1.2. Cài Đặt

1. Chạy file `Docker Desktop Installer.exe`
2. Chọn **"Use WSL 2 instead of Hyper-V"** (khuyến nghị)
3. Click **Install**
4. Khởi động lại máy khi được yêu cầu

### 1.3. Cấu Hình Docker

Sau khi khởi động lại:

1. Mở **Docker Desktop**
2. Click biểu tượng ⚙️ (Settings)
3. Vào **Resources** → **Advanced**
4. Cấu hình:
   - **CPUs**: 4 cores
   - **Memory**: 6GB (hoặc 4GB nếu RAM máy < 16GB)
   - **Disk**: 20GB
5. Click **Apply & Restart**

---

## 📥 Bước 2: Download Source Code

### 2.1. Cài Git (nếu chưa có)

Download: https://git-scm.com/download/win

### 2.2. Clone Repository

Mở **PowerShell** hoặc **Git Bash**:

```powershell
# Di chuyển đến thư mục muốn lưu project
cd C:\Users\YourName\Desktop

# Clone repo
git clone <repository-url>
cd adas-platform
```

---

## 🚀 Bước 3: Chạy Project

### Option 1: Build Từ Source (Lần Đầu - 10-12 phút)

Mở **PowerShell** trong thư mục project:

```powershell
# Build và start tất cả services
docker compose up --build
```

**Chờ 10-12 phút** ☕ (Docker sẽ download và build):
- Backend: PyTorch, OpenCV, scipy (~500MB)
- Frontend: Node packages, Next.js build (~200MB)

Khi thấy:
```
✔ Container adas-backend   Healthy
✔ Container adas-frontend  Started
```

→ **Thành công!** Truy cập http://localhost:3000

---

### Option 2: Sử Dụng Pre-built Images (Nhanh - 2 phút)

**Nếu team đã share images:**

#### Bước 1: Download Images

Tải 2 files từ Google Drive:
- `backend.tar` (~2GB)
- `frontend.tar` (~200MB)

Đặt vào thư mục project `adas-platform`

#### Bước 2: Load Images

Mở **PowerShell** trong thư mục project:

```powershell
# Load backend image
docker load -i backend.tar

# Load frontend image
docker load -i frontend.tar
```

#### Bước 3: Start Services

```powershell
docker compose up
```

**Chỉ mất 10-20 giây!** 🚀

---

## 🌐 Bước 4: Truy Cập Ứng Dụng

Mở trình duyệt (Chrome/Edge khuyến nghị):

- **Homepage**: http://localhost:3000
- **ADAS Detection**: http://localhost:3000/adas
- **Driver Monitor**: http://localhost:3000/driver-monitor
- **Backend API**: http://localhost:8000/docs

---

## 🧪 Bước 5: Test Detection

1. Truy cập: http://localhost:3000/adas
2. Click **"Bật Camera"**
3. Cho phép truy cập camera khi trình duyệt hỏi
4. Click **"Bắt Đầu Phát Hiện"**
5. Quan sát:
   - ✅ Bounding boxes màu xanh/đỏ/cam
   - ✅ Labels tiếng Việt: "Xe", "Người", etc.
   - ✅ FPS hiển thị (mục tiêu: 30-40 FPS)

---

## 🛑 Dừng Services

Trong PowerShell đang chạy Docker:

```powershell
# Nhấn Ctrl + C

# Hoặc dừng hoàn toàn
docker compose down
```

---

## 🔄 Chạy Lại Lần Sau

```powershell
# Vào thư mục project
cd C:\Users\YourName\Desktop\adas-platform

# Start (nhanh - 10 giây)
docker compose up
```

---

## 🐛 Xử Lý Lỗi Thường Gặp

### Lỗi 1: "Docker daemon is not running"

**Nguyên nhân**: Docker Desktop chưa mở

**Giải pháp**:
1. Mở **Docker Desktop**
2. Đợi biểu tượng Docker ở system tray chuyển sang màu xanh
3. Chạy lại lệnh

---

### Lỗi 2: "Port 3000 is already in use"

**Nguyên nhân**: Có app khác đang dùng port 3000

**Giải pháp**:

```powershell
# Tìm process đang dùng port 3000
netstat -ano | findstr :3000

# Kill process (thay <PID> bằng số ở cột cuối)
taskkill /PID <PID> /F

# Hoặc đổi port trong docker-compose.yml
# frontend:
#   ports:
#     - "3001:3000"  # Đổi 3000 thành 3001
```

---

### Lỗi 3: Build Quá Lâu (>20 phút)

**Nguyên nhân**: 
- Mạng chậm
- RAM/CPU không đủ

**Giải pháp**:

1. **Tăng Docker Resources**:
   - Docker Desktop → Settings → Resources
   - RAM: 6GB, CPU: 4 cores

2. **Sử dụng Pre-built Images** (khuyến nghị):
   - Download từ Drive
   - Load bằng `docker load -i`

3. **Check mạng**:
   ```powershell
   # Test download speed
   curl -o test.zip https://speed.hetzner.de/100MB.bin
   ```

---

### Lỗi 4: "Cannot connect to backend"

**Kiểm tra backend đang chạy**:

```powershell
# Check containers
docker compose ps

# Xem logs backend
docker compose logs backend

# Restart backend
docker compose restart backend
```

**Test backend API**:

Mở trình duyệt: http://localhost:8000/health

Nên thấy:
```json
{
  "status": "success",
  "data": {
    "status": "ok"
  }
}
```

---

### Lỗi 5: Camera Không Hoạt Động

**Nguyên nhân**: Trình duyệt chặn camera

**Giải pháp**:

1. Click biểu tượng 🔒 hoặc 🎥 trên thanh địa chỉ
2. Chọn **"Allow"** cho Camera
3. Refresh trang (F5)
4. Click "Bật Camera" lại

**Nếu vẫn không được**:
- Thử trình duyệt khác (Chrome/Edge)
- Check camera đang được dùng bởi app khác (Zoom, Teams)

---

## 📊 Kiểm Tra Hệ Thống

### Check Docker Version

```powershell
docker --version
# Nên thấy: Docker version 24.x.x hoặc mới hơn

docker compose version
# Nên thấy: Docker Compose version v2.x.x
```

### Check Containers

```powershell
# List running containers
docker compose ps

# Nên thấy:
# NAME            STATUS
# adas-backend    Up (healthy)
# adas-frontend   Up
```

### Check Logs

```powershell
# Xem logs tất cả services
docker compose logs

# Chỉ xem backend
docker compose logs backend

# Chỉ xem frontend
docker compose logs frontend

# Follow logs real-time
docker compose logs -f
```

---

## 🎯 Quick Commands Cheat Sheet

```powershell
# Start services
docker compose up

# Start in background
docker compose up -d

# Stop services
docker compose down

# Restart một service
docker compose restart frontend

# Rebuild một service
docker compose up -d --build frontend

# Xem logs
docker compose logs -f

# Xóa tất cả (cẩn thận!)
docker compose down -v
docker system prune -a
```

---

## 💾 Backup & Share Images (Cho Team Lead)

### Tạo Images Để Share

```powershell
# Build images
docker compose build

# Save images
docker save adas-platform1-backend -o backend.tar
docker save adas-platform1-frontend -o frontend.tar

# Upload lên Google Drive
# Chia sẻ link với team
```

### Team Members Load Images

```powershell
# Download backend.tar và frontend.tar từ Drive
# Đặt vào thư mục project

# Load images
docker load -i backend.tar
docker load -i frontend.tar

# Start
docker compose up
```

---

## 📱 Sử Dụng PowerShell Script Tự Động

### Tạo File `quick-start.ps1`

```powershell
# Tạo file quick-start.ps1 với nội dung:

Write-Host "🚀 ADAS Platform Quick Start" -ForegroundColor Green
Write-Host "================================" -ForegroundColor Green
Write-Host ""

# Check Docker
if (!(Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Host "❌ Docker chưa cài đặt!" -ForegroundColor Red
    Write-Host "Download tại: https://www.docker.com/products/docker-desktop/" -ForegroundColor Yellow
    exit 1
}

Write-Host "✅ Docker đã cài đặt" -ForegroundColor Green

# Check if images exist
$backendExists = docker images | Select-String "adas-platform1-backend"
$frontendExists = docker images | Select-String "adas-platform1-frontend"

if ($backendExists -and $frontendExists) {
    Write-Host "✅ Images đã có, starting services..." -ForegroundColor Green
    docker compose up
} else {
    Write-Host "⚠️  Images chưa có" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Chọn phương án:" -ForegroundColor Cyan
    Write-Host "1. Build từ source (10-12 phút)"
    Write-Host "2. Load pre-built images từ Drive (2 phút)"
    Write-Host ""
    
    $choice = Read-Host "Nhập lựa chọn (1/2)"
    
    if ($choice -eq "1") {
        Write-Host "☕ Building... Đi uống cafe 10 phút!" -ForegroundColor Yellow
        docker compose up --build
    } elseif ($choice -eq "2") {
        Write-Host ""
        Write-Host "📥 Download 2 files từ Google Drive:" -ForegroundColor Yellow
        Write-Host "- backend.tar (~2GB)"
        Write-Host "- frontend.tar (~200MB)"
        Write-Host ""
        Write-Host "Link: [THÊM LINK DRIVE]" -ForegroundColor Cyan
        Write-Host ""
        
        $ready = Read-Host "Đã tải xong? (y/n)"
        
        if ($ready -eq "y") {
            if ((Test-Path "backend.tar") -and (Test-Path "frontend.tar")) {
                Write-Host "Loading images..." -ForegroundColor Green
                docker load -i backend.tar
                docker load -i frontend.tar
                Write-Host "✅ Done! Starting services..." -ForegroundColor Green
                docker compose up
            } else {
                Write-Host "❌ Không tìm thấy backend.tar hoặc frontend.tar" -ForegroundColor Red
                exit 1
            }
        }
    }
}
```

### Chạy Script

```powershell
# Cho phép chạy script (chỉ cần 1 lần)
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

# Chạy script
.\quick-start.ps1
```

---

## 📞 Hỗ Trợ

Nếu gặp vấn đề:

1. **Check logs**: `docker compose logs`
2. **Restart Docker Desktop**
3. **Ping team lead** với screenshot lỗi
4. **Check DOCKER_BUILD_OPTIMIZATION.md** để biết thêm tips

---

## ✅ Checklist Setup Thành Công

- [ ] Docker Desktop đã cài và chạy
- [ ] Docker RAM ≥ 4GB, CPU ≥ 4 cores
- [ ] Clone repo thành công
- [ ] `docker compose up` chạy không lỗi
- [ ] Truy cập http://localhost:3000 thấy UI
- [ ] Backend http://localhost:8000/health trả về OK
- [ ] Camera hoạt động tại /adas page
- [ ] Detection labels hiển thị tiếng Việt

**Nếu tất cả ✅ → Chúc mừng! Bạn đã setup thành công!** 🎉

---

**Cập nhật**: 2025-11-30  
**Version**: v3.1 - Windows Edition
