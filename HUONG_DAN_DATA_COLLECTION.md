# 📸 Hướng Dẫn Thu Thập Dữ Liệu ADAS

## 🚀 Khởi Động Hệ Thống

### 1. Khởi động Backend (Port 8000)
```bash
cd backend-python
python3 main.py
```

Kiểm tra: http://localhost:8000/docs

### 2. Khởi động Frontend (Port 3000)
```bash
npm run dev
```

Truy cập: http://localhost:3000/data-collection

---

## 📝 Cách Thu Thập Dữ Liệu

### Bước 1: Upload File
- Click nút **"Choose File"**
- Chọn ảnh hoặc video (tối đa 200MB)
- Hỗ trợ định dạng: JPG, PNG, MP4, AVI

### Bước 2: Chọn Loại Đối Tượng (Object Types)
Tích chọn các đối tượng có trong ảnh:
- ✅ **car** - Ô tô
- ✅ **motorcycle** - Xe máy
- ✅ **pedestrian** - Người đi bộ
- ✅ **bicycle** - Xe đạp
- ✅ **traffic_light** - Đèn giao thông
- ✅ **traffic_sign** - Biển báo
- ✅ **truck** - Xe tải
- ✅ **bus** - Xe buýt

### Bước 3: Vẽ Bounding Box
1. **Chọn loại đối tượng** từ dropdown (phía dưới canvas)
2. **Click và kéo chuột** trên ảnh để vẽ hộp
3. Thả chuột để hoàn thành
4. Hộp sẽ tự động có màu và nhãn
5. Lặp lại để vẽ nhiều hộp

**Lưu ý:**
- Phải chọn loại đối tượng trước khi vẽ
- Mỗi hộp có màu khác nhau
- Có thể xóa hộp bằng nút 🗑️

### Bước 4: Chọn Điều Kiện Thời Tiết (Weather)
- ☀️ **sunny** - Nắng
- 🌧️ **rainy** - Mưa
- 🌫️ **foggy** - Sương mù
- 🌙 **night** - Ban đêm

### Bước 5: Chọn Loại Đường (Road Type)
- 🏙️ **urban** - Đô thị
- 🛣️ **highway** - Cao tốc
- 🌾 **rural** - Nông thôn

### Bước 6: Mô Tả (Tùy Chọn)
Nhập mô tả chi tiết về ảnh/video (không bắt buộc)

### Bước 7: Submit
Click nút **"Submit Dataset"** để lưu

---

## 📊 Xem Danh Sách Dataset

### Cách 1: Trong UI
- Click nút **"View Dataset List"**
- Xem danh sách các item đã thu thập
- Click 🗑️ để xóa item

### Cách 2: API
```bash
# Xem tất cả
curl http://localhost:8000/api/dataset

# Xóa item
curl -X DELETE http://localhost:8000/api/dataset/{id}
```

---

## 📁 Cấu Trúc Thư Mục Dataset

```
backend-python/
└── dataset/
    ├── raw/           # File gốc (ảnh/video)
    ├── images/        # Ảnh đã xử lý (YOLO format)
    ├── labels/        # File nhãn YOLO (.txt)
    └── data.yaml      # Config cho training
```

---

## 🎯 Format YOLO Label

Mỗi file `.txt` trong `labels/` chứa:
```
class_id x_center y_center width height
```

**Ví dụ:**
```
0 0.512 0.345 0.234 0.156
1 0.678 0.567 0.123 0.089
```

- **class_id**: 0=car, 1=motorcycle, 2=pedestrian, ...
- **Tất cả giá trị đã normalize** (0-1)

---

## 🔧 API Endpoints

### POST /api/dataset
Tạo item mới
```bash
curl -X POST http://localhost:8000/api/dataset \
  -F "file=@image.jpg" \
  -F 'metadata={"labels":["car","pedestrian"],"boundingBoxes":[...],"weather":"sunny","roadType":"urban"}'
```

### GET /api/dataset
Lấy tất cả items
```bash
curl http://localhost:8000/api/dataset
```

### DELETE /api/dataset/{id}
Xóa item
```bash
curl -X DELETE http://localhost:8000/api/dataset/abc-123
```

### POST /api/dataset/export-yolo
Export dataset ra YOLO format
```bash
curl -X POST http://localhost:8000/api/dataset/export-yolo
```

### GET /api/dataset/stats
Xem thống kê dataset
```bash
curl http://localhost:8000/api/dataset/stats
```

---

## 🏋️ Training với YOLO

### Bước 1: Export Dataset
```bash
curl -X POST http://localhost:8000/api/dataset/export-yolo
```

### Bước 2: Train YOLOv8
```python
from ultralytics import YOLO

# Load pretrained model
model = YOLO('yolov8n.pt')

# Train on custom dataset
results = model.train(
    data='backend-python/dataset/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name='adas_custom'
)
```

### Bước 3: Validate
```python
metrics = model.val()
print(f"mAP50: {metrics.box.map50}")
```

### Bước 4: Inference
```python
results = model.predict(
    source='test.jpg',
    conf=0.25,
    save=True
)
```

---

## ✅ Checklist Thu Thập Dữ Liệu Chất Lượng

### Đa Dạng Điều Kiện
- [ ] Nắng, mưa, sương mù, đêm
- [ ] Đô thị, cao tốc, nông thôn
- [ ] Nhiều góc camera khác nhau
- [ ] Nhiều khoảng cách khác nhau

### Chất Lượng Bounding Box
- [ ] Khít đối tượng (không quá rộng/hẹp)
- [ ] Label chính xác
- [ ] Không thiếu đối tượng quan trọng
- [ ] Không trùng lặp box

### Số Lượng
- [ ] Mỗi class ≥ 100 ảnh
- [ ] Tổng cộng ≥ 1000 ảnh
- [ ] Train/Val/Test = 70/20/10

---

## 🐛 Troubleshooting

### Lỗi: "No module named 'Pillow'"
```bash
pip3 install Pillow
```

### Lỗi: "Failed to upload dataset"
- Kiểm tra backend đang chạy: http://localhost:8000/docs
- Kiểm tra file size < 200MB
- Xem console log trong browser (F12)

### Lỗi: "Canvas not drawing"
- Chọn loại đối tượng trước khi vẽ
- Refresh page và thử lại

### Dataset folder không tạo
Backend tự động tạo khi upload file đầu tiên

---

## 📌 Tips & Best Practices

### 1. Vẽ Bounding Box Chuẩn
- Box khít đối tượng
- Không cắt mất phần quan trọng
- Nhiều box nhỏ > 1 box to (nếu nhiều đối tượng)

### 2. Label Chính Xác
- Xe máy ≠ Xe đạp
- Xe tải ≠ Xe buýt
- Đèn giao thông ≠ Biển báo

### 3. Đa Dạng Dữ Liệu
- Thu thập ở nhiều địa điểm
- Nhiều thời điểm trong ngày
- Nhiều điều kiện thời tiết

### 4. Kiểm Tra Định Kỳ
```bash
# Xem số lượng ảnh/label
ls backend-python/dataset/images/*.jpg | wc -l
ls backend-python/dataset/labels/*.txt | wc -l

# Xem thống kê
curl http://localhost:8000/api/dataset/stats
```

---

## 🎓 Video Hướng Dẫn

1. **Upload File**: Chọn ảnh từ máy
2. **Select Labels**: Tích chọn các đối tượng
3. **Draw Boxes**: Chọn loại → Vẽ hộp
4. **Set Conditions**: Chọn weather + road type
5. **Submit**: Click nút submit

---

## 📞 Support

- **API Documentation**: http://localhost:8000/docs
- **Frontend**: http://localhost:3000/data-collection
- **Dataset Stats**: http://localhost:8000/api/dataset/stats

---

## 🔄 Workflow Tổng Thể

```
1. Upload ảnh/video
   ↓
2. Chọn object types (car, pedestrian...)
   ↓
3. Vẽ bounding boxes
   ↓
4. Chọn weather & road type
   ↓
5. Submit → Backend lưu vào dataset/
   ↓
6. Export YOLO format
   ↓
7. Training YOLOv8
   ↓
8. Deploy model mới
```

---

**✨ Chúc bạn thu thập dữ liệu thành công!**
