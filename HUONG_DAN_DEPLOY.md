# 🚀 HƯỚNG DẪN DEPLOY HỆ THỐNG ADAS LÊN WINDOWS SERVER
## Dành cho người mới - Làm theo từng bước

---

## 📋 BƯỚC 1: CHUẨN BỊ

### Những thứ cần cài đặt (KHÔNG CẦN DOCKER):

1. **Python 3.8 trở lên**
   - Tải về: https://www.python.org/downloads/
   - ✅ Khi cài, NHỚ TÍCH VÀO Ô "Add Python to PATH"
   - Cài xong, mở Command Prompt gõ `python --version` để kiểm tra

2. **SQL Server Express (MIỄN PHÍ)**
   - Tải về: https://go.microsoft.com/fwlink/?linkid=866658
   - Chọn "Basic" và cài đặt
   - Ghi nhớ mật khẩu SA nếu được hỏi

3. **ODBC Driver 17 cho SQL Server**
   - Tải về: https://go.microsoft.com/fwlink/?linkid=2249006
   - Cài đặt bình thường

---

## 🎯 BƯỚC 2: DEPLOY NHANH (1 LỆNH DUY NHẤT)

### Cách 1: PowerShell (KHUYÊN DÙNG)

1. Mở **PowerShell** với quyền **Administrator** (click phải → Run as Administrator)
2. Di chuyển vào thư mục dự án:
   ```powershell
   cd "C:\đường\dẫn\tới\adas-platform (1)"
   ```
3. Chạy lệnh này:
   ```powershell
   .\setup-windows.ps1
   ```
4. Làm theo hướng dẫn trên màn hình
5. **XONG!** Server sẽ tự động chạy

### Cách 2: Command Prompt

1. Mở **Command Prompt** với quyền **Administrator**
2. Di chuyển vào thư mục dự án:
   ```cmd
   cd "C:\đường\dẫn\tới\adas-platform (1)"
   ```
3. Chạy lệnh này:
   ```cmd
   deploy-windows.bat
   ```
4. **XONG!** Server sẽ tự động chạy

---

## 🔧 BƯỚC 3: CÀI ĐẶT THỦ CÔNG (NẾU LỆNH TỰ ĐỘNG KHÔNG CHẠY)

### 3.1. Cài đặt thư viện Python

```cmd
cd backend-python
pip install -r requirements.txt
```

### 3.2. Tạo Database

**Mở SQL Server Management Studio hoặc Command Prompt:**

```cmd
sqlcmd -S localhost -E -Q "CREATE DATABASE ADAS_DB"
```

**Nếu lỗi, thử với SA password:**
```cmd
sqlcmd -S localhost -U sa -P "MatKhauCuaBan" -Q "CREATE DATABASE ADAS_DB"
```

### 3.3. Tạo bảng trong Database

```cmd
cd ..
sqlcmd -S localhost -E -d ADAS_DB -i database\sql-server-schema.sql
```

### 3.4. Cấu hình kết nối Database

Mở file `backend-python\.env` và chỉnh sửa:

**Nếu dùng Windows Authentication (không cần mật khẩu):**
```
DATABASE_URL=mssql+pyodbc://localhost/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server&trusted_connection=yes
```

**Nếu dùng SQL Authentication (cần username/password):**
```
DATABASE_URL=mssql+pyodbc://sa:MatKhauSA@localhost/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server
```

**Nếu KHÔNG CÓ SQL Server (dùng SQLite):**
```
DATABASE_URL=sqlite:///./adas.db
```

### 3.5. Chạy Server

```cmd
cd backend-python
python main.py
```

✅ **Mở trình duyệt và truy cập:**
- API Docs: http://localhost:8000/docs
- Health Check: http://localhost:8000/health

---

## 🔒 BƯỚC 4: MỞ FIREWALL (NẾU KHÔNG TRUY CẬP ĐƯỢC)

Mở PowerShell với quyền Administrator:

```powershell
New-NetFirewallRule -DisplayName "ADAS Backend API" -Direction Inbound -Protocol TCP -LocalPort 8000 -Action Allow
```

---

## 🚀 BƯỚC 5: CHẠY TỰ ĐỘNG KHI KHỞI ĐỘNG WINDOWS (TÙY CHỌN)

### Cách 1: Tạo Windows Service với NSSM

1. **Tải NSSM**: https://nssm.cc/download
2. **Giải nén** vào `C:\nssm\`
3. **Mở Command Prompt với quyền Administrator:**

```cmd
cd C:\nssm
nssm install ADASBackend
```

4. **Một cửa sổ hiện ra, điền thông tin:**
   - Path: `C:\Python3x\python.exe` (đường dẫn Python của bạn)
   - Startup directory: `C:\đường\dẫn\tới\backend-python`
   - Arguments: `main.py`

5. **Khởi động Service:**

```cmd
nssm start ADASBackend
```

### Quản lý Service:

```cmd
REM Khởi động
nssm start ADASBackend

REM Dừng
nssm stop ADASBackend

REM Restart
nssm restart ADASBackend

REM Xóa service
nssm remove ADASBackend confirm
```

### Cách 2: Tạo Task Scheduler

1. Mở **Task Scheduler** (gõ trong Start Menu)
2. Click **Create Task**
3. **General tab:**
   - Name: `ADAS Backend`
   - Chọn: `Run whether user is logged on or not`
   - Chọn: `Run with highest privileges`
4. **Triggers tab:**
   - Click **New**
   - Begin: `At startup`
   - Click **OK**
5. **Actions tab:**
   - Click **New**
   - Action: `Start a program`
   - Program: `C:\Python3x\python.exe`
   - Arguments: `main.py`
   - Start in: `C:\đường\dẫn\tới\backend-python`
   - Click **OK**
6. Click **OK** để lưu

---

## 📱 BƯỚC 6: DEPLOY FRONTEND (NEXT.JS)

### 6.1. Cài đặt Node.js

Tải về: https://nodejs.org/ (chọn bản LTS)

### 6.2. Cài đặt dependencies

```cmd
cd "C:\đường\dẫn\tới\adas-platform (1)"
npm install
```

### 6.3. Chạy Frontend

**Development mode:**
```cmd
npm run dev
```

**Production mode:**
```cmd
npm run build
npm start
```

✅ Mở trình duyệt: http://localhost:3000

---

## ❓ XỬ LÝ LỖI THƯỜNG GẶP

### Lỗi 1: "Python không được nhận diện"

**Nguyên nhân:** Chưa add Python vào PATH

**Giải pháp:**
1. Gỡ cài đặt Python
2. Cài lại và NHỚ TÍCH "Add Python to PATH"

### Lỗi 2: "Cannot connect to SQL Server"

**Giải pháp:**

1. **Kiểm tra SQL Server có chạy không:**
```cmd
sqlcmd -S localhost -E -Q "SELECT @@VERSION"
```

2. **Khởi động lại SQL Server:**
```powershell
Restart-Service -Name "MSSQL$SQLEXPRESS"
```

3. **Kiểm tra tên instance:**
```cmd
sqlcmd -S localhost\SQLEXPRESS -E -Q "SELECT @@VERSION"
```

Nếu lệnh trên chạy được, sửa `.env`:
```
DATABASE_URL=mssql+pyodbc://localhost\SQLEXPRESS/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server&trusted_connection=yes
```

### Lỗi 3: "ODBC Driver not found"

**Giải pháp:**

1. **Kiểm tra driver đã cài:**
```powershell
Get-OdbcDriver
```

2. **Nếu không có, tải về cài lại:**
   - ODBC 17: https://go.microsoft.com/fwlink/?linkid=2249006
   - ODBC 18: https://go.microsoft.com/fwlink/?linkid=2282284

3. **Sửa `.env` để dùng driver đúng:**
```
# Nếu cài ODBC 18
DATABASE_URL=mssql+pyodbc://localhost/ADAS_DB?driver=ODBC+Driver+18+for+SQL+Server&trusted_connection=yes
```

### Lỗi 4: "Port 8000 already in use"

**Giải pháp:**

1. **Tìm process đang dùng port 8000:**
```cmd
netstat -ano | findstr :8000
```

2. **Kết thúc process (thay PID bằng số từ lệnh trên):**
```cmd
taskkill /PID <số_PID> /F
```

3. **Hoặc đổi port khác trong `.env`:**
```
PORT=8080
```

### Lỗi 5: Không truy cập được từ máy khác

**Giải pháp:**

1. **Mở Firewall:**
```powershell
New-NetFirewallRule -DisplayName "ADAS Backend" -Direction Inbound -Protocol TCP -LocalPort 8000 -Action Allow
```

2. **Lấy địa chỉ IP máy chủ:**
```cmd
ipconfig
```

3. **Truy cập từ máy khác:**
```
http://<IP_máy_chủ>:8000/docs
```

---

## 🧪 KIỂM TRA HỆ THỐNG

### Test Backend

```cmd
cd backend-python
python -c "from database import engine; engine.connect(); print('✅ Database OK!')"
```

### Test API

Mở trình duyệt:
- http://localhost:8000/health
- http://localhost:8000/docs

### Test từ xa

Từ máy khác, mở trình duyệt:
- http://<IP_Server>:8000/health

---

## 📊 MỞ SQL SERVER MANAGEMENT STUDIO

1. Tải về: https://aka.ms/ssmsfullsetup
2. Cài đặt
3. Mở SSMS
4. Kết nối:
   - Server name: `localhost` hoặc `localhost\SQLEXPRESS`
   - Authentication: `Windows Authentication`
5. Xem database `ADAS_DB` → Tables

---

## 🔑 THÔNG TIN QUAN TRỌNG

### URL Truy Cập

- **Backend API:** http://localhost:8000
- **API Documentation:** http://localhost:8000/docs
- **Health Check:** http://localhost:8000/health
- **Frontend:** http://localhost:3000

### File Cấu Hình

- **Database:** `backend-python\.env`
- **Server Log:** `backend-python\server.log`

### Lệnh Hay Dùng

```cmd
REM Kiểm tra Python
python --version

REM Kiểm tra pip
pip --version

REM Kiểm tra SQL Server
sqlcmd -S localhost -E -Q "SELECT @@VERSION"

REM Khởi động backend
cd backend-python
python main.py

REM Khởi động frontend
npm run dev
```

---

## 📞 HỖ TRỢ

Nếu gặp lỗi:

1. Kiểm tra log: `backend-python\server.log`
2. Test database: 
   ```cmd
   cd backend-python
   python -c "from database import engine; engine.connect()"
   ```
3. Kiểm tra SQL Server:
   ```cmd
   sqlcmd -S localhost -E -Q "SELECT @@VERSION"
   ```

---

## ✅ CHECKLIST HOÀN TẤT

- [ ] Python đã cài và chạy được lệnh `python --version`
- [ ] SQL Server đã cài và chạy được `sqlcmd`
- [ ] ODBC Driver đã cài
- [ ] Database ADAS_DB đã tạo
- [ ] Bảng trong database đã tạo
- [ ] File `.env` đã cấu hình đúng
- [ ] Python packages đã cài (`pip install -r requirements.txt`)
- [ ] Backend chạy được (`python main.py`)
- [ ] Truy cập được http://localhost:8000/docs
- [ ] Frontend chạy được (`npm run dev`)
- [ ] Firewall đã mở port 8000
- [ ] (Tùy chọn) Windows Service đã tạo

**🎉 CHÚC MỪNG! HỆ THỐNG ĐÃ SẴN SÀNG!**
