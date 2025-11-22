# ADAS Backend - Python FastAPI

Backend API cho hệ thống ADAS, viết bằng Python FastAPI, kết nối SQL Server, chạy trên Windows Server.

## 📋 Yêu cầu

- **Python 3.10+** 
- **SQL Server** (Express/Developer/Standard) hoặc Azure SQL
- **ODBC Driver 17 for SQL Server** (hoặc mới hơn)
- **Windows Server** 2016+ hoặc Windows 10/11

## 🚀 Cài đặt

### 1. Cài đặt ODBC Driver cho SQL Server

Tải và cài đặt từ:
```
https://learn.microsoft.com/en-us/sql/connect/odbc/download-odbc-driver-for-sql-server
```

Hoặc dùng PowerShell (admin):
```powershell
# Download và install ODBC Driver 17
Invoke-WebRequest -Uri "https://go.microsoft.com/fwlink/?linkid=2249004" -OutFile "msodbcsql.msi"
msiexec /i msodbcsql.msi /quiet IACCEPTMSODBCSQLLICENSETERMS=YES
```

### 2. Tạo môi trường ảo và cài dependencies

```bash
# Tạo virtual environment
python -m venv venv

# Kích hoạt (Windows)
venv\Scripts\activate

# Cài packages
pip install -r requirements.txt
```

### 3. Cấu hình môi trường

Copy `.env.example` thành `.env` và cập nhật:

```bash
copy .env.example .env
```

Chỉnh sửa `.env`:
```env
SQL_SERVER=localhost
SQL_DATABASE=ADAS_DB
SQL_USERNAME=sa
SQL_PASSWORD=YourStrongPassword123!
SQL_DRIVER=ODBC Driver 17 for SQL Server

MODEL_WORKER_URL=http://localhost:8000
PORT=8000
```

### 4. Tạo database và seed dữ liệu

```bash
# Tạo tables
python seed.py
```

Hoặc tạo database thủ công bằng SQL:
```sql
CREATE DATABASE ADAS_DB;
```

### 5. Chạy server

```bash
# Development mode (auto-reload)
python main.py

# hoặc dùng uvicorn trực tiếp
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 6. Production mode

```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --workers 4
```

## 📊 API Endpoints

### Swagger Documentation
```
http://localhost:8000/docs
```

### Health Check
```
GET http://localhost:8000/health
```

### Main Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/cameras/list` | GET | Lấy danh sách cameras |
| `/api/cameras/{id}` | GET | Lấy camera theo ID |
| `/api/cameras` | POST | Tạo camera mới |
| `/api/cameras/{id}` | PUT | Cập nhật camera |
| `/api/cameras/{id}` | DELETE | Xóa camera |
| `/api/models/list` | GET | Lấy danh sách AI models |
| `/api/models/{id}/download` | POST | Tải model |
| `/api/models/{id}/activate` | POST | Kích hoạt model |
| `/api/trips/list` | GET | Lấy danh sách trips |
| `/api/trips` | POST | Tạo trip mới |
| `/api/trips/{id}/end` | POST | Kết thúc trip |
| `/api/events/list` | GET | Lấy danh sách events |
| `/api/events` | POST | Tạo event |
| `/api/drivers/list` | GET | Lấy danh sách drivers |
| `/api/drivers` | POST | Tạo driver |
| `/api/analytics/dashboard` | GET | Thống kê dashboard |
| `ws://localhost:8000/ws/infer` | WebSocket | Real-time inference |

## 🔧 Cấu hình SQL Server

### Connection String Formats

**SQL Authentication:**
```
mssql+pyodbc://sa:password@localhost/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server&TrustServerCertificate=yes
```

**Windows Authentication:**
```
mssql+pyodbc://localhost/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server&trusted_connection=yes
```

### Enable TCP/IP (nếu cần remote connection)

1. Mở **SQL Server Configuration Manager**
2. Expand **SQL Server Network Configuration**
3. Click **Protocols for MSSQLSERVER**
4. Enable **TCP/IP**
5. Restart SQL Server service

### Firewall Rules

```powershell
# Mở port 1433 cho SQL Server
New-NetFirewallRule -DisplayName "SQL Server" -Direction Inbound -LocalPort 1433 -Protocol TCP -Action Allow

# Mở port 8000 cho Backend
New-NetFirewallRule -DisplayName "ADAS Backend" -Direction Inbound -LocalPort 8000 -Protocol TCP -Action Allow
```

## 🐳 Deploy như Windows Service

### Sử dụng NSSM (Non-Sucking Service Manager)

1. Tải NSSM: https://nssm.cc/download

2. Cài đặt service:
```powershell
# Install service
nssm install AdasBackend "C:\Path\To\Python\python.exe" "C:\Path\To\backend-python\main.py"

# Set working directory
nssm set AdasBackend AppDirectory "C:\Path\To\backend-python"

# Set environment variables
nssm set AdasBackend AppEnvironmentExtra SQL_PASSWORD=YourPassword

# Start service
nssm start AdasBackend
```

3. Quản lý service:
```powershell
# Xem status
nssm status AdasBackend

# Stop service
nssm stop AdasBackend

# Remove service
nssm remove AdasBackend confirm
```

## 🧪 Testing

### Test Database Connection

```bash
python database.py
```

### Test API với curl

```bash
# Get cameras
curl http://localhost:8000/api/cameras/list

# Create camera
curl -X POST http://localhost:8000/api/cameras \
  -H "Content-Type: application/json" \
  -d '{"name":"Test Camera","type":"webcam","status":"ready"}'

# Health check
curl http://localhost:8000/health
```

### Test WebSocket

Sử dụng `wscat`:
```bash
npm install -g wscat
wscat -c ws://localhost:8000/ws/infer
```

Gửi message:
```json
{"frameB64":"<base64_encoded_image>"}
```

## 📁 Project Structure

```
backend-python/
├── main.py              # FastAPI app & routes
├── models.py            # SQLAlchemy models
├── schemas.py           # Pydantic schemas
├── database.py          # Database connection
├── services.py          # Business logic
├── config.py            # Configuration
├── seed.py              # Database seeding
├── requirements.txt     # Dependencies
├── .env.example         # Environment template
└── README.md           # This file
```

## 🔐 Security

- **Không commit** file `.env` vào Git
- Sử dụng **strong passwords** cho SQL Server
- Cấu hình **CORS** origins cụ thể trong production
- Sử dụng **HTTPS** cho production deployment
- Thiết lập **backup** cho database

## 🐛 Troubleshooting

### Lỗi: "pyodbc.InterfaceError: ('28000', ...)"
→ Kiểm tra username/password trong `.env`

### Lỗi: "Can't open lib 'ODBC Driver 17 for SQL Server'"
→ Cài đặt ODBC Driver

### Lỗi: "Login failed for user"
→ Kiểm tra SQL Server Authentication mode (Mixed Mode)

### Database connection timeout
→ Kiểm tra SQL Server đang chạy và firewall rules

## 📚 Documentation

- FastAPI Docs: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc
- OpenAPI JSON: http://localhost:8000/openapi.json

## 🎯 Production Checklist

- [ ] Cài ODBC Driver 17
- [ ] Cấu hình SQL Server (TCP/IP, Mixed Mode)
- [ ] Tạo database và user riêng (không dùng sa)
- [ ] Cập nhật `.env` với production credentials
- [ ] Cấu hình firewall rules
- [ ] Setup Windows Service với NSSM
- [ ] Thiết lập backup database
- [ ] Cấu hình CORS origins
- [ ] Setup reverse proxy (nginx/IIS) nếu cần
- [ ] Enable HTTPS/TLS
- [ ] Setup monitoring và logging

---

**Ready to use!** 🚀
