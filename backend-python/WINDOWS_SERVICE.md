# ADAS Backend - Windows Service Setup

## Cài đặt NSSM (Non-Sucking Service Manager)

### Tải NSSM
Download từ: https://nssm.cc/download

Hoặc dùng Chocolatey:
```powershell
choco install nssm
```

## Cài đặt service (PowerShell as Admin)

### 1. Tạo script wrapper

Tạo file `start_service.bat` trong thư mục `backend-python`:

```batch
@echo off
cd /d "C:\Path\To\backend-python"
call venv\Scripts\activate.bat
python main.py
```

### 2. Install service với NSSM

```powershell
# Thay đổi đường dẫn phù hợp
$BACKEND_PATH = "C:\Path\To\backend-python"
$PYTHON_EXE = "$BACKEND_PATH\venv\Scripts\python.exe"
$MAIN_PY = "$BACKEND_PATH\main.py"

# Install service
nssm install AdasBackend "$PYTHON_EXE" "$MAIN_PY"

# Set working directory
nssm set AdasBackend AppDirectory "$BACKEND_PATH"

# Set environment variables (nếu cần)
nssm set AdasBackend AppEnvironmentExtra SQL_PASSWORD=YourPassword

# Set startup type
nssm set AdasBackend Start SERVICE_AUTO_START

# Set description
nssm set AdasBackend Description "ADAS Backend API Service"

# Rotate logs
nssm set AdasBackend AppStdout "$BACKEND_PATH\logs\service.log"
nssm set AdasBackend AppStderr "$BACKEND_PATH\logs\service_error.log"
nssm set AdasBackend AppRotateFiles 1
nssm set AdasBackend AppRotateSeconds 86400
nssm set AdasBackend AppRotateBytes 10485760
```

### 3. Quản lý service

```powershell
# Start service
nssm start AdasBackend
# hoặc
net start AdasBackend

# Stop service
nssm stop AdasBackend
# hoặc
net stop AdasBackend

# Restart service
nssm restart AdasBackend

# Check status
nssm status AdasBackend

# Remove service
nssm remove AdasBackend confirm
```

### 4. Xem logs

```powershell
# Tạo thư mục logs trước
New-Item -ItemType Directory -Force -Path "$BACKEND_PATH\logs"

# Xem logs
Get-Content "$BACKEND_PATH\logs\service.log" -Tail 50 -Wait
```

## Cách 2: Sử dụng pythonservice (win32serviceutil)

### 1. Cài đặt dependencies

```bash
pip install pywin32
```

### 2. Tạo file `windows_service.py`

```python
import win32serviceutil
import win32service
import win32event
import servicemanager
import socket
import sys
import os
from pathlib import Path

# Add current directory to path
sys.path.append(str(Path(__file__).parent))

class AdasBackendService(win32serviceutil.ServiceFramework):
    _svc_name_ = "AdasBackend"
    _svc_display_name_ = "ADAS Backend API Service"
    _svc_description_ = "FastAPI backend for ADAS platform"

    def __init__(self, args):
        win32serviceutil.ServiceFramework.__init__(self, args)
        self.hWaitStop = win32event.CreateEvent(None, 0, 0, None)
        socket.setdefaulttimeout(60)
        self.is_alive = True

    def SvcStop(self):
        self.ReportServiceStatus(win32service.SERVICE_STOP_PENDING)
        win32event.SetEvent(self.hWaitStop)
        self.is_alive = False

    def SvcDoRun(self):
        servicemanager.LogMsg(
            servicemanager.EVENTLOG_INFORMATION_TYPE,
            servicemanager.PYS_SERVICE_STARTED,
            (self._svc_name_, '')
        )
        self.main()

    def main(self):
        import uvicorn
        from main import app
        
        config = uvicorn.Config(
            app,
            host="0.0.0.0",
            port=8000,
            log_level="info"
        )
        server = uvicorn.Server(config)
        server.run()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        servicemanager.Initialize()
        servicemanager.PrepareToHostSingle(AdasBackendService)
        servicemanager.StartServiceCtrlDispatcher()
    else:
        win32serviceutil.HandleCommandLine(AdasBackendService)
```

### 3. Install và quản lý

```powershell
# Install service
python windows_service.py install

# Start service
python windows_service.py start

# Stop service
python windows_service.py stop

# Remove service
python windows_service.py remove

# Debug mode
python windows_service.py debug
```

## Auto-start on Windows Boot

Nếu dùng NSSM, service sẽ tự động start nếu đã set:
```powershell
nssm set AdasBackend Start SERVICE_AUTO_START
```

## Task Scheduler (Alternative)

### 1. Tạo task

```powershell
$action = New-ScheduledTaskAction -Execute "$BACKEND_PATH\run.bat"
$trigger = New-ScheduledTaskTrigger -AtStartup
$settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
$principal = New-ScheduledTaskPrincipal -UserId "SYSTEM" -LogonType ServiceAccount -RunLevel Highest

Register-ScheduledTask -TaskName "AdasBackend" -Action $action -Trigger $trigger -Settings $settings -Principal $principal
```

### 2. Quản lý

```powershell
# Start
Start-ScheduledTask -TaskName "AdasBackend"

# Stop
Stop-ScheduledTask -TaskName "AdasBackend"

# Remove
Unregister-ScheduledTask -TaskName "AdasBackend" -Confirm:$false
```

## Troubleshooting

### Service không start

1. Check Event Viewer:
   - Windows Logs → Application
   - Look for AdasBackend errors

2. Check logs:
   ```powershell
   Get-Content logs\service_error.log
   ```

3. Test manually:
   ```powershell
   cd C:\Path\To\backend-python
   venv\Scripts\activate
   python main.py
   ```

### Service crashes

- Kiểm tra `.env` file tồn tại và đúng format
- Kiểm tra SQL Server đang chạy
- Kiểm tra permissions (service user có access SQL Server)

## Best Practices

1. **Run as specific user** (không nên dùng SYSTEM):
   ```powershell
   nssm set AdasBackend ObjectName ".\ServiceAccount" "password"
   ```

2. **Set dependencies**:
   ```powershell
   # Service chờ SQL Server start trước
   nssm set AdasBackend DependOnService MSSQLSERVER
   ```

3. **Monitoring**:
   - Setup alerts cho service failure
   - Regular log rotation
   - Health check endpoint

---

**Recommended: NSSM** - Đơn giản, dễ quản lý, logs tốt hơn
