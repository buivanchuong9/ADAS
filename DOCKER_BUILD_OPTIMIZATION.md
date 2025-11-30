# 🚀 Hướng Dẫn Build Docker Nhanh Cho Team

## ⏱️ Thời Gian Build Dự Kiến

### Lần Đầu (Cold Build)
- **Backend**: ~8-10 phút
  - Download Python packages: PyTorch (104MB), OpenCV (46MB), scipy (34MB), polars (38MB)
  - Tổng ~500MB packages
- **Frontend**: ~30-40 giây
  - Download Node packages: ~200MB
  - Build Next.js: ~20 giây
- **Tổng**: ~10-12 phút

### Lần Sau (Cached Build)
- **Backend**: ~10 giây (nếu không đổi requirements.txt)
- **Frontend**: ~30 giây (nếu không đổi package.json)
- **Tổng**: ~40 giây

---

## 🎯 Cách Build Nhanh Nhất

### 1. Sử dụng Docker Layer Caching

Docker đã tự động cache layers. Chỉ rebuild khi có thay đổi:

```bash
# Build lần đầu (lâu)
docker compose up --build

# Lần sau chỉ cần (nhanh)
docker compose up
```

### 2. Build Riêng Từng Service

Nếu chỉ sửa frontend, không cần rebuild backend:

```bash
# Chỉ rebuild frontend (30 giây)
docker compose up -d --build frontend

# Chỉ rebuild backend (nếu cần)
docker compose up -d --build backend
```

### 3. Sử dụng Pre-built Images (Khuyến Nghị)

**Tạo images một lần, share cho team:**

```bash
# Người đầu tiên build và save
docker compose build
docker save adas-platform1-backend -o backend.tar
docker save adas-platform1-frontend -o frontend.tar

# Upload lên Google Drive/Dropbox

# Team khác chỉ cần load (2 phút)
docker load -i backend.tar
docker load -i frontend.tar
docker compose up
```

---

## 💡 Tips Tối Ưu

### 1. Tăng Docker Resources

**Mac/Windows Docker Desktop:**
- Settings → Resources
- RAM: Tăng lên 4-6GB (mặc định 2GB)
- CPU: Tăng lên 4 cores
- Disk: 20GB+

### 2. Sử dụng BuildKit (Nhanh hơn 2x)

```bash
# Thêm vào ~/.zshrc hoặc ~/.bashrc
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

# Reload
source ~/.zshrc
```

### 3. Cleanup Để Tăng Tốc

```bash
# Xóa old images/containers (chạy 1 tháng/lần)
docker system prune -a --volumes
```

---

## 🔧 Troubleshooting

### Build Quá Lâu (>20 phút)

**Nguyên nhân:**
- Mạng chậm khi download packages
- RAM/CPU không đủ
- Disk đầy

**Giải pháp:**
```bash
# 1. Check Docker resources
docker info | grep -E "CPUs|Total Memory"

# 2. Check disk space
df -h

# 3. Sử dụng mirror nhanh hơn (cho backend)
# Thêm vào backend-python/Dockerfile:
RUN pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

### Build Bị Stuck

```bash
# Stop all
docker compose down

# Clear cache
docker builder prune -a

# Rebuild
docker compose up --build
```

---

## 📊 So Sánh Thời Gian

| Phương Pháp | Lần Đầu | Lần Sau | Khuyến Nghị |
|-------------|---------|---------|-------------|
| `docker compose up --build` | 10-12 phút | 40 giây | ✅ Tốt nhất |
| Load pre-built images | 2 phút | 10 giây | ⭐ Nhanh nhất cho team |
| Build từ source mỗi lần | 10-12 phút | 10-12 phút | ❌ Không nên |

---

## 🎓 Best Practices Cho Team

### 1. Người Đầu Tiên Setup

```bash
# Clone repo
git clone <repo-url>
cd adas-platform

# Build lần đầu (10-12 phút - chờ uống cafe ☕)
docker compose up --build

# Save images để share
docker save adas-platform1-backend -o backend.tar
docker save adas-platform1-frontend -o frontend.tar

# Upload lên Drive
# Link: https://drive.google.com/...
```

### 2. Team Members Tiếp Theo

```bash
# Clone repo
git clone <repo-url>
cd adas-platform

# Download images từ Drive (2 phút)
# Giải nén và load
docker load -i backend.tar
docker load -i frontend.tar

# Start ngay (10 giây)
docker compose up
```

### 3. Khi Có Code Mới

```bash
# Pull code mới
git pull

# Chỉ rebuild service bị thay đổi
docker compose up -d --build frontend  # Nếu sửa FE
docker compose up -d --build backend   # Nếu sửa BE
```

---

## 🚀 Quick Start Script Cho Team

Tạo file `quick-start.sh`:

```bash
#!/bin/bash

echo "🚀 ADAS Platform Quick Start"
echo ""

# Check if images exist
if docker images | grep -q "adas-platform1-backend"; then
    echo "✅ Images đã có, starting services..."
    docker compose up
else
    echo "⚠️  Images chưa có, downloading..."
    echo "Tải backend.tar và frontend.tar từ Drive"
    echo "Link: https://drive.google.com/..."
    echo ""
    read -p "Đã tải xong? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker load -i backend.tar
        docker load -i frontend.tar
        docker compose up
    fi
fi
```

Sử dụng:
```bash
chmod +x quick-start.sh
./quick-start.sh
```

---

## 📝 Checklist Cho Team Mới

- [ ] Cài Docker Desktop
- [ ] Tăng RAM lên 4-6GB trong Docker Settings
- [ ] Download pre-built images từ Drive (nếu có)
- [ ] Clone repo
- [ ] Chạy `docker compose up` (hoặc `./quick-start.sh`)
- [ ] Truy cập http://localhost:3000
- [ ] Test ADAS detection tại /adas

---

## 🎯 Tóm Tắt

**Lần đầu build:**
- ☕ Uống cafe 10-12 phút
- Hoặc download pre-built images (2 phút)

**Lần sau:**
- ⚡ Chỉ 10-40 giây
- Docker cache tự động

**Nếu vẫn lâu:**
- Tăng Docker RAM/CPU
- Sử dụng pre-built images
- Check mạng internet

---

**Liên hệ:** Nếu vẫn gặp vấn đề, ping team lead! 💬
