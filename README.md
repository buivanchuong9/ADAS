# ADAS - Advanced Driver Assistance System

Hệ thống hỗ trợ lái xe tiên tiến với phát hiện vật thể thời gian thực, giám sát tài xế, và trợ lý AI.

## 🏗️ Kiến trúc hệ thống

- **Frontend**: React + TailwindCSS + WebSocket
- **Backend**: ASP.NET Core 8 + Entity Framework Core
- **Model Worker**: Python FastAPI + YOLOv8
- **Database**: SQL Server 2022
- **AI**: Perplexity API

## 📋 Yêu cầu

- Docker & Docker Compose
- Perplexity API Key (https://www.perplexity.ai/api)

## 🚀 Cài đặt & Chạy

### 1. Clone repository
\`\`\`bash
git clone <repo-url>
cd adas-platform
\`\`\`

### 2. Cấu hình .env
\`\`\`bash
cp .env.example .env
# Chỉnh sửa .env với Perplexity API key của bạn
\`\`\`

### 3. Khởi động hệ thống
\`\`\`bash
docker-compose up --build
\`\`\`

### 4. Truy cập ứng dụng
- Frontend: http://localhost:3000
- Backend API: http://localhost:5000
- Model Worker: http://localhost:8000

## 📱 Sử dụng

1. Mở http://localhost:3000
2. Nhấn "Nhận diện trực tiếp" để bắt đầu camera
3. Cho phép truy cập camera
4. Xem phát hiện vật thể thời gian thực
5. Kiểm tra "Giám sát tài xế" để phát hiện mệt mỏi
6. Hỏi "Trợ lý AI" về lái xe an toàn

## 🔧 Cấu trúc thư mục

\`\`\`
adas-platform/
├── frontend/          # React app
├── backend/           # ASP.NET Core API
├── model-worker/      # Python YOLOv8 service
├── docker-compose.yml
├── .env.example
└── README.md
\`\`\`

## 📝 Ghi chú

- Tất cả dữ liệu được lưu vào SQL Server
- Sự kiện va chạm được ghi lại tự động
- Cảnh báo mệt mỏi kích hoạt khi mắt đóng > 2 giây
- Perplexity API cung cấp lời khuyên lái xe an toàn

## 🐛 Troubleshooting

**Lỗi kết nối camera**: Cho phép quyền truy cập camera trong trình duyệt
**Lỗi WebSocket**: Kiểm tra backend đang chạy trên port 5000
**Lỗi Model Worker**: Đảm bảo Docker có đủ bộ nhớ (4GB)

---

**English Version**

# ADAS - Advanced Driver Assistance System

Real-time object detection, driver monitoring, and AI assistant for safe driving.

## 🏗️ Architecture

- **Frontend**: React + TailwindCSS + WebSocket
- **Backend**: ASP.NET Core 8 + Entity Framework Core
- **Model Worker**: Python FastAPI + YOLOv8
- **Database**: SQL Server 2022
- **AI**: Perplexity API

## 📋 Requirements

- Docker & Docker Compose
- Perplexity API Key

## 🚀 Installation & Run

\`\`\`bash
cp .env.example .env
# Edit .env with your Perplexity API key
docker-compose up --build
\`\`\`

\`\`\`

## 🛠️ Lệnh cài đặt (cài hết thư viện)

Phần này liệt kê các lệnh cần thiết để cài đặt tất cả phụ thuộc cho frontend (Node), backend (.NET) và model worker (Python). Chạy từng phần riêng tùy theo môi trường phát triển của bạn.

1) Cài Node (frontend)

Trong thư mục gốc của repository, nếu bạn dùng pnpm:

\`\`\`bash
# cài dependencies (dùng pnpm nếu có)
pnpm install

# hoặc nếu dùng npm
npm install
\`\`\`

Lưu ý: nếu cài một số gói native có build scripts (ví dụ sharp), pnpm có thể yêu cầu xác nhận; chạy `pnpm approve-builds` nếu cần.

2) Cài Python (model-worker)

\`\`\`bash
# chuyển vào thư mục model-worker
cd model-worker

# tạo virtualenv (macOS / Linux)
python3 -m venv .venv

# kích hoạt virtualenv
. .venv/bin/activate

# cập nhật pip và cài tất cả packages
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

# để chạy service inference (dev)
uvicorn app:app --host 0.0.0.0 --port 8000 --reload
\`\`\`

Lưu ý: package `ultralytics` sẽ cài torch/torchvision và các wheel lớn — thời gian và băng thông có thể mất nhiều phút.

3) Backend (.NET)

Hiện trong thư mục `backend` chưa có file dự án `.csproj` trong repository. Nếu bạn đã có `.csproj` hoặc solution, chạy trong thư mục chứa file `.csproj`:

\`\`\`bash
# restore packages
dotnet restore

# build và chạy
dotnet build
dotnet run
\`\`\`

Nếu bạn muốn tôi tạo một `backend.csproj` tối giản để có thể build/run local, báo tôi sẽ scaffold nhanh.

4) Tùy chọn: chạy bằng Docker Compose (recommended)

\`\`\`bash
# từ thư mục gốc
docker-compose up --build
\`\`\`

5) Kiểm tra sức khỏe (smoke checks)

\`\`\`bash
# model worker
curl http://localhost:8000/health

# backend (nếu chạy)
curl http://localhost:5000/  # hoặc endpoint health nếu có

# frontend
open http://localhost:3000
\`\`\`

Nếu cần, tôi có thể thêm script tự động `scripts/setup.sh` để thực thi tất cả bước trên (với flag để bỏ qua backend nếu không có .csproj). Hãy cho biết bạn muốn tôi tạo script tự động hay chỉ cần hướng dẫn như trên.
# ADAS
# ADAS
