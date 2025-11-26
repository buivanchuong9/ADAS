# 🎨 ADAS Detection Color Guide - Bảng Màu Phát Hiện

## 📌 Màu Sắc Bounding Box

### 🔴 ĐỎ (Red) - NGUY HIỂM NGHIÊM TRỌNG
- **Điều kiện**: TTC < 2 giây
- **Ý nghĩa**: Va chạm sắp xảy ra!
- **Hành động**: Phanh gấp, tránh ngay
- **Áp dụng cho**: Xe, người, động vật nguy hiểm

### 🟠 CAM (Orange) - CẢNH BÁO
- **Điều kiện**: TTC < 3.5 giây
- **Ý nghĩa**: Đối tượng đang tiến gần
- **Hành động**: Giảm tốc độ, chuẩn bị phanh
- **Áp dụng cho**: Xe, người, động vật cảnh báo

### 🟢 XANH LÁ (Green) - AN TOÀN
- **Điều kiện**: TTC > 3.5 giây hoặc xa
- **Ý nghĩa**: Đối tượng an toàn
- **Hành động**: Theo dõi bình thường
- **Áp dụng cho**: Xe, người xa

### 🔵 XANH DƯƠNG (Cyan) - ĐỐI TƯỢNG THƯỜNG
- **Điều kiện**: Không phải nguy hiểm
- **Ý nghĩa**: Đối tượng môi trường (cây, vật thể, v.v.)
- **Hành động**: Ghi nhận, không cảnh báo
- **Áp dụng cho**: Cây, vật thể tĩnh, động vật không nguy hiểm

### 🟣 TÍM (Magenta) + 🆕 - ĐỐI TƯỢNG MỚI HỌC
- **Điều kiện**: Lần đầu phát hiện hoặc chất lượng cao
- **Ý nghĩa**: Hệ thống đang học đối tượng này!
- **Hành động**: Tự động lưu vào training dataset
- **Áp dụng cho**: Bất kỳ đối tượng mới nào

---

## 🎯 Danh Sách 80 Loại Đối Tượng COCO

### 🚗 Phương Tiện (8 loại)
1. car (xe hơi)
2. motorcycle (xe máy)
3. bus (xe buýt)
4. truck (xe tải)
5. bicycle (xe đạp)
6. train (tàu hỏa)
7. boat (thuyền)
8. airplane (máy bay)

### 👥 Con Người (1 loại)
9. person (người)

### 🐕 Động Vật (11 loại)
10. bird (chim)
11. cat (mèo)
12. dog (chó)
13. horse (ngựa)
14. sheep (cừu)
15. cow (bò)
16. elephant (voi)
17. bear (gấu)
18. zebra (ngựa vằn)
19. giraffe (hươu cao cổ)

### 🚦 Giao Thông (4 loại)
20. traffic light (đèn giao thông)
21. fire hydrant (vòi cứu hỏa)
22. stop sign (biển báo dừng)
23. parking meter (máy đỗ xe)

### 🪑 Nội Thất (10 loại)
24. bench (ghế băng)
25. chair (ghế)
26. couch (sofa)
27. potted plant (cây trong chậu)
28. bed (giường)
29. dining table (bàn ăn)
30. toilet (toilet)
31. tv (TV)
32. laptop (laptop)

### 🎒 Đồ Vật Cá Nhân (7 loại)
33. backpack (ba lô)
34. umbrella (ô)
35. handbag (túi xách)
36. tie (cà vạt)
37. suitcase (vali)

### ⚽ Thể Thao (8 loại)
38. frisbee (đĩa bay)
39. skis (ván trượt tuyết)
40. snowboard (ván trượt tuyết)
41. sports ball (bóng thể thao)
42. kite (diều)
43. baseball bat (gậy bóng chày)
44. baseball glove (găng bóng chày)
45. skateboard (ván trượt)
46. surfboard (ván lướt sóng)
47. tennis racket (vợt tennis)

### 🍽️ Đồ Ăn & Nhà Bếp (21 loại)
48. bottle (chai)
49. wine glass (ly rượu)
50. cup (cốc)
51. fork (nĩa)
52. knife (dao)
53. spoon (thìa)
54. bowl (bát)
55. banana (chuối)
56. apple (táo)
57. sandwich (bánh mì)
58. orange (cam)
59. broccoli (súp lơ)
60. carrot (cà rốt)
61. hot dog (xúc xích)
62. pizza (pizza)
63. donut (bánh donut)
64. cake (bánh ngọt)

### 📱 Điện Tử (5 loại)
65. mouse (chuột máy tính)
66. remote (điều khiển)
67. keyboard (bàn phím)
68. cell phone (điện thoại)

### 🏠 Thiết Bị Nhà (4 loại)
69. microwave (lò vi sóng)
70. oven (lò nướng)
71. toaster (máy nướng bánh)
72. sink (bồn rửa)
73. refrigerator (tủ lạnh)

### 📚 Khác (7 loại)
74. book (sách)
75. clock (đồng hồ)
76. vase (lọ hoa)
77. scissors (kéo)
78. teddy bear (gấu bông)
79. hair drier (máy sấy tóc)
80. toothbrush (bàn chải đánh răng)

---

## 🔍 Thông Tin Hiển Thị

### Label Format
```
[Class Name] [Confidence%] [Distance]m [TTC:X.Xs] [🆕]
```

### Ví Dụ
- `car 89% 5.2m TTC:2.1s` - Xe hơi, 89% tin cậy, cách 5.2m, TTC 2.1s
- `tree 76% 12.4m 🆕` - Cây (mới học), 76% tin cậy, cách 12.4m
- `person 92% 3.5m TTC:3.2s` - Người, 92% tin cậy, cách 3.5m

---

## 💡 Mẹo Sử Dụng

### Khi Lái Xe
1. **Chú ý màu ĐỎ** - Nguy hiểm ngay lập tức
2. **Theo dõi màu CAM** - Chuẩn bị hành động
3. **Quan sát màu XANH** - Kiểm soát tình huống
4. **Ghi nhận màu TÍM** - Hệ thống đang học!

### Tối Ưu Hóa Học
1. Đưa camera vào nhiều tình huống khác nhau
2. Cho hệ thống "nhìn" nhiều đối tượng đa dạng
3. Di chuyển qua các môi trường khác nhau (thành phố, nông thôn, v.v.)
4. Các điều kiện ánh sáng khác nhau (sáng, tối, mưa, v.v.)

---

## 📊 Statistics Dashboard

### Metrics Hiển Thị
- **FPS**: Tốc độ xử lý (8-12 FPS)
- **Detections**: Số đối tượng phát hiện
- **Inference Time**: Thời gian xử lý (80-120ms)
- **Loại đối tượng**: Số loại khác nhau trong frame
- **Đã thu thập**: Tổng frame đã lưu cho học
- **Đối tượng mới học**: Số loại mới đã học được
- **Mới (frame hiện tại)**: Số đối tượng mới trong frame này

---

## ⚡ Quick Reference

| Màu | Tình Huống | Hành Động |
|-----|-----------|-----------|
| 🔴 Red | TTC < 2s | PHANH GẤP |
| 🟠 Orange | TTC < 3.5s | GIẢM TỐC |
| 🟢 Green | Xa, an toàn | THEO DÕI |
| 🔵 Cyan | Vật thể thường | GHI NHẬN |
| 🟣 Magenta | Mới học | TỰ ĐỘNG LƯU |

---

**Lưu ý**: Hệ thống chỉ cảnh báo cho các đối tượng NGUY HIỂM (xe, người, động vật lớn). 
Các đối tượng khác (cây, vật tĩnh) được phát hiện nhưng không tạo alert. 🚀
