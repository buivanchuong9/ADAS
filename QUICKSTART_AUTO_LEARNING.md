# 🚀 Quick Start - ADAS Auto-Learning System

## ⚡ Bắt Đầu Nhanh (5 phút)

### Bước 1: Start Backend
```bash
cd backend-python
python start.py
```

### Bước 2: Start Frontend
```bash
cd ..
npm run dev
# hoặc
pnpm dev
```

### Bước 3: Mở Trình Duyệt
```
http://localhost:3000/adas
```

### Bước 4: Bật Camera
1. Click **"Bật Camera"**
2. Cho phép quyền truy cập camera
3. Click **"Bắt Đầu Phát Hiện"**

### Bước 5: Xem Phép Màu! ✨
- **Đỏ**: Nguy hiểm (TTC < 2s)
- **Cam**: Cảnh báo (TTC < 3.5s)
- **Xanh**: An toàn (phương tiện)
- **Cyan**: Đối tượng thường (cây, vật)
- **Tím + 🆕**: Đối tượng MỚI đang học!

---

## 🎯 Test Auto-Learning

### 1. Di chuyển camera qua nhiều đối tượng
```
- Xe hơi ✅
- Người ✅
- Cây cối ✅
- Chó mèo ✅
- Ghế, bàn ✅
- Bất cứ thứ gì! ✅
```

### 2. Quan sát thống kê
Xem các số liệu:
- **Loại đối tượng**: Tăng khi phát hiện loại mới
- **Đã thu thập**: Số frame đã lưu
- **Đối tượng mới học**: Số loại mới
- **Mới (frame hiện tại)**: Đối tượng mới trong frame

### 3. Khi có badge 🆕 TÍM
= Hệ thống đang HỌC đối tượng này!

---

## 🔥 Training Với Dữ Liệu Mới

### Check Stats
```bash
curl http://localhost:8000/api/auto-learning/stats
```

### Start Training (khi có ≥10 samples)
```bash
curl -X POST http://localhost:8000/api/auto-learning/train-incremental \
  -H "Content-Type: application/json" \
  -d '{
    "epochs": 20,
    "batch_size": 8
  }'
```

### Check Training Progress
```bash
curl http://localhost:8000/api/auto-learning/training-status/{training_id}
```

---

## 🎨 Hiểu Màu Sắc

| Màu | Nghĩa | Khi Nào Xuất Hiện |
|-----|-------|-------------------|
| 🔴 **Đỏ** | NGUY HIỂM | Xe/người tiến gần (TTC < 2s) |
| 🟠 **Cam** | CẢNH BÁO | Xe/người gần (TTC < 3.5s) |
| 🟢 **Xanh** | AN TOÀN | Xe/người xa |
| 🔵 **Cyan** | TRUNG LẬP | Cây, vật thể, động vật |
| 🟣 **Tím** | MỚI HỌC | Lần đầu gặp! |

---

## 📊 80 Loại Đối Tượng Có Thể Phát Hiện

### Phương Tiện (8)
🚗 car, 🏍️ motorcycle, 🚌 bus, 🚚 truck, 🚲 bicycle, 🚂 train, ⛵ boat, ✈️ airplane

### Người & Động Vật (12)
👤 person, 🐕 dog, 🐈 cat, 🐴 horse, 🐑 sheep, 🐄 cow, 🐘 elephant, 🐻 bear, 🦓 zebra, 🦒 giraffe, 🐦 bird

### Giao Thông (4)
🚦 traffic light, 🛑 stop sign, 🚒 fire hydrant, 🅿️ parking meter

### Vật Thể & Nội Thất (20+)
🪑 chair, 🛋️ couch, 🌱 potted plant, 🪴 vase, 📚 book, ⏰ clock, 💻 laptop, 📱 cell phone, ...

### Thức Ăn (15+)
🍌 banana, 🍎 apple, 🍊 orange, 🥕 carrot, 🍕 pizza, 🍰 cake, ...

### Và 20+ loại khác!

---

## 💡 Tips

### Để Thu Thập Dữ Liệu Tốt
1. ✅ Ánh sáng đủ (không quá tối/sáng)
2. ✅ Góc nhìn rõ ràng
3. ✅ Đối tượng không bị che khuất
4. ✅ Di chuyển chậm, ổn định

### Để Model Học Tốt
1. ✅ Nhiều góc độ khác nhau
2. ✅ Nhiều điều kiện ánh sáng
3. ✅ Nhiều khoảng cách khác nhau
4. ✅ Thu thập ít nhất 10-50 mẫu/loại

### Tối Ưu Performance
1. ✅ Close các app khác
2. ✅ Sử dụng camera HD
3. ✅ Stable internet (nếu cần)
4. ✅ Đủ RAM (8GB+)

---

## 🐛 Sửa Lỗi Nhanh

### Không phát hiện được gì?
```bash
# Kiểm tra backend
curl http://localhost:8000/

# Kiểm tra camera
# Đảm bảo cho phép quyền camera trong browser
```

### FPS quá thấp?
```bash
# Giảm độ phân giải camera
# Hoặc tắt các app nặng khác
```

### Không thu thập dữ liệu?
```bash
# Check logs
tail -f backend-python/logs/adas.log

# Kiểm tra folder
ls dataset/auto_collected/images/
```

### Training lỗi?
```bash
# Cần ít nhất 10 samples
curl http://localhost:8000/api/auto-learning/stats
```

---

## 📚 Tài Liệu Đầy Đủ

- 📖 **AUTO_LEARNING_GUIDE.md**: Hướng dẫn chi tiết
- 🎨 **COLOR_GUIDE.md**: Giải thích màu sắc
- 📋 **CHANGES_SUMMARY.md**: Tổng hợp thay đổi

---

## 🎯 What's Next?

### Sau 5 phút đầu:
1. ✅ Đã biết cách dùng camera
2. ✅ Hiểu màu sắc bounding box
3. ✅ Thấy hệ thống phát hiện đối tượng

### Sau 30 phút:
1. ✅ Thu thập được 50-100 samples
2. ✅ Phát hiện 10-20 loại đối tượng
3. ✅ Sẵn sàng training lần đầu

### Sau 1 giờ:
1. ✅ Chạy incremental training
2. ✅ Model cải thiện 5-10%
3. ✅ Hiểu toàn bộ workflow

---

## 🚀 Bắt Đầu Ngay!

```bash
# Terminal 1: Backend
cd backend-python && python start.py

# Terminal 2: Frontend  
npm run dev

# Browser
open http://localhost:3000/adas
```

---

## 🎉 Kết Quả Mong Đợi

Sau vài phút sử dụng:
- ✅ Phát hiện TẤT CẢ loại đối tượng (80 classes)
- ✅ Tự động thu thập high-quality samples
- ✅ Nhận diện đối tượng mới chưa từng gặp
- ✅ Hiển thị thống kê real-time
- ✅ Sẵn sàng training để cải thiện

**HỆ THỐNG TỰ HỌC - KHÔNG CẦN LABEL THỦ CÔNG!** 🤖✨

---

## 📞 Hỗ Trợ

Gặp vấn đề? Check:
1. Backend logs: `backend-python/logs/`
2. Collection stats: `GET /api/auto-learning/stats`
3. Documentation: `AUTO_LEARNING_GUIDE.md`

**Happy Learning! 🚀🎓**
