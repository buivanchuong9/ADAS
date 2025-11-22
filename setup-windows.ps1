# ============================================
# ADAS Platform - Windows Server Setup
# PowerShell deployment script
# ============================================

Write-Host ""
Write-Host "========================================"
Write-Host "   ADAS Platform Setup & Deployment"
Write-Host "   Windows Server Edition"
Write-Host "========================================"
Write-Host ""

# Check Administrator privileges
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "[ERROR] Please run as Administrator!" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

# Step 1: Check Python
Write-Host "[1/8] Checking Python installation..." -ForegroundColor Cyan
try {
    $pythonVersion = python --version 2>&1
    Write-Host "[OK] $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "[ERROR] Python 3.8+ required! Download from python.org" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

# Step 2: Install Python packages
Write-Host ""
Write-Host "[2/8] Installing Python dependencies..." -ForegroundColor Cyan
Set-Location backend-python
python -m pip install --upgrade pip --quiet
pip install -r requirements.txt --quiet
if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Failed to install dependencies" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}
Write-Host "[OK] Dependencies installed" -ForegroundColor Green

# Step 3: Check SQL Server
Write-Host ""
Write-Host "[3/8] Checking SQL Server..." -ForegroundColor Cyan
$sqlServerFound = $false
try {
    $null = sqlcmd -S localhost -E -Q "SELECT @@VERSION" 2>&1
    if ($LASTEXITCODE -eq 0) {
        $sqlServerFound = $true
        Write-Host "[OK] SQL Server detected (Windows Authentication)" -ForegroundColor Green
    }
} catch {}

if (-not $sqlServerFound) {
    try {
        $null = sqlcmd -S (hostname) -E -Q "SELECT @@VERSION" 2>&1
        if ($LASTEXITCODE -eq 0) {
            $sqlServerFound = $true
            Write-Host "[OK] SQL Server detected on $(hostname)" -ForegroundColor Green
        }
    } catch {}
}

if (-not $sqlServerFound) {
    Write-Host "[WARNING] SQL Server not accessible" -ForegroundColor Yellow
    Write-Host "Install SQL Server Express: https://go.microsoft.com/fwlink/?linkid=866658" -ForegroundColor Yellow
    $continue = Read-Host "Continue without SQL Server? (SQLite will be used) [Y/n]"
    if ($continue -eq "n" -or $continue -eq "N") {
        exit 1
    }
}

# Step 4: Configure database connection
Write-Host ""
Write-Host "[4/8] Configuring database..." -ForegroundColor Cyan

if ($sqlServerFound) {
    # Try to create database
    $dbCreated = $false
    
    # Try Windows Authentication first
    try {
        sqlcmd -S localhost -E -Q "IF NOT EXISTS (SELECT * FROM sys.databases WHERE name = 'ADAS_DB') CREATE DATABASE ADAS_DB" 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) {
            $dbCreated = $true
            Write-Host "[OK] Database ADAS_DB created/verified" -ForegroundColor Green
            
            # Set connection string in .env
            $envContent = @"
# Database Configuration
DATABASE_URL=mssql+pyodbc://localhost/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server&trusted_connection=yes

# Server Configuration
PORT=8000
HOST=0.0.0.0
DEBUG=False

# CORS
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001

# Model Worker
MODEL_WORKER_URL=http://localhost:5000
"@
            $envContent | Out-File -FilePath ".env" -Encoding UTF8
        }
    } catch {}
    
    # If Windows Auth failed, try SQL Auth
    if (-not $dbCreated) {
        Write-Host "[INFO] Windows Authentication failed, trying SQL Authentication..." -ForegroundColor Yellow
        $saPassword = Read-Host "Enter SQL Server SA password (or press Enter to use SQLite)"
        
        if ($saPassword) {
            try {
                sqlcmd -S localhost -U sa -P $saPassword -Q "IF NOT EXISTS (SELECT * FROM sys.databases WHERE name = 'ADAS_DB') CREATE DATABASE ADAS_DB" 2>&1 | Out-Null
                if ($LASTEXITCODE -eq 0) {
                    $dbCreated = $true
                    Write-Host "[OK] Database ADAS_DB created with SQL Auth" -ForegroundColor Green
                    
                    # Set connection string with SQL Auth
                    $envContent = @"
# Database Configuration
DATABASE_URL=mssql+pyodbc://sa:$saPassword@localhost/ADAS_DB?driver=ODBC+Driver+17+for+SQL+Server

# Server Configuration
PORT=8000
HOST=0.0.0.0
DEBUG=False

# CORS
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001

# Model Worker
MODEL_WORKER_URL=http://localhost:5000
"@
                    $envContent | Out-File -FilePath ".env" -Encoding UTF8
                }
            } catch {}
        }
    }
}

# Fallback to SQLite
if (-not $dbCreated) {
    Write-Host "[INFO] Using SQLite database" -ForegroundColor Yellow
    $envContent = @"
# Database Configuration
DATABASE_URL=sqlite:///./adas.db

# Server Configuration
PORT=8000
HOST=0.0.0.0
DEBUG=False

# CORS
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001

# Model Worker
MODEL_WORKER_URL=http://localhost:5000
"@
    $envContent | Out-File -FilePath ".env" -Encoding UTF8
    Write-Host "[OK] SQLite configured" -ForegroundColor Green
}

# Step 5: Apply database schema (if SQL Server)
if ($dbCreated -and (Test-Path "..\database\sql-server-schema.sql")) {
    Write-Host ""
    Write-Host "[5/8] Creating database schema..." -ForegroundColor Cyan
    
    try {
        sqlcmd -S localhost -E -d ADAS_DB -i ..\database\sql-server-schema.sql 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "[OK] Database schema created" -ForegroundColor Green
        } else {
            Write-Host "[WARNING] Schema creation had warnings (continuing)" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "[WARNING] Could not apply schema" -ForegroundColor Yellow
    }
} else {
    Write-Host ""
    Write-Host "[5/8] Schema creation skipped" -ForegroundColor Yellow
}

# Step 6: Create Windows Service (optional)
Write-Host ""
Write-Host "[6/8] Windows Service setup..." -ForegroundColor Cyan
$createService = Read-Host "Create Windows Service for auto-start? [Y/n]"

if ($createService -ne "n" -and $createService -ne "N") {
    $serviceName = "ADASBackend"
    $serviceExists = Get-Service -Name $serviceName -ErrorAction SilentlyContinue
    
    if ($serviceExists) {
        Write-Host "[INFO] Removing existing service..." -ForegroundColor Yellow
        Stop-Service -Name $serviceName -Force -ErrorAction SilentlyContinue
        sc.exe delete $serviceName | Out-Null
    }
    
    $pythonPath = (Get-Command python).Source
    $scriptPath = (Get-Location).Path + "\main.py"
    
    # Create NSSM service wrapper
    $nssmPath = "C:\nssm\nssm.exe"
    if (Test-Path $nssmPath) {
        & $nssmPath install $serviceName $pythonPath $scriptPath
        & $nssmPath set $serviceName AppDirectory (Get-Location).Path
        & $nssmPath set $serviceName DisplayName "ADAS Backend Service"
        & $nssmPath set $serviceName Description "ADAS Platform FastAPI Backend"
        & $nssmPath set $serviceName Start SERVICE_AUTO_START
        Write-Host "[OK] Windows Service created: $serviceName" -ForegroundColor Green
    } else {
        Write-Host "[INFO] NSSM not found. Download from nssm.cc for service creation" -ForegroundColor Yellow
        Write-Host "[SKIP] Service creation skipped" -ForegroundColor Yellow
    }
} else {
    Write-Host "[SKIP] Service creation skipped" -ForegroundColor Yellow
}

# Step 7: Configure Firewall
Write-Host ""
Write-Host "[7/8] Configuring Windows Firewall..." -ForegroundColor Cyan
try {
    $ruleName = "ADAS Backend API"
    $existingRule = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue
    
    if ($existingRule) {
        Remove-NetFirewallRule -DisplayName $ruleName
    }
    
    New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Protocol TCP -LocalPort 8000 -Action Allow | Out-Null
    Write-Host "[OK] Firewall rule created for port 8000" -ForegroundColor Green
} catch {
    Write-Host "[WARNING] Could not configure firewall (run as Administrator)" -ForegroundColor Yellow
}

# Step 8: Test database connection
Write-Host ""
Write-Host "[8/8] Testing database connection..." -ForegroundColor Cyan
try {
    $testResult = python -c "from database import engine; engine.connect(); print('OK')" 2>&1
    if ($testResult -like "*OK*") {
        Write-Host "[OK] Database connection successful" -ForegroundColor Green
    } else {
        Write-Host "[WARNING] Database connection test failed" -ForegroundColor Yellow
    }
} catch {
    Write-Host "[WARNING] Could not test connection" -ForegroundColor Yellow
}

# Summary
Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "   Setup Complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "Start the server:" -ForegroundColor Cyan
Write-Host "  python main.py" -ForegroundColor White
Write-Host ""
Write-Host "Or run as service:" -ForegroundColor Cyan
Write-Host "  Start-Service ADASBackend" -ForegroundColor White
Write-Host ""
Write-Host "Access points:" -ForegroundColor Cyan
Write-Host "  API Docs:     http://localhost:8000/docs" -ForegroundColor White
Write-Host "  Health Check: http://localhost:8000/health" -ForegroundColor White
Write-Host ""

# Ask to start now
$startNow = Read-Host "Start server now? [Y/n]"
if ($startNow -ne "n" -and $startNow -ne "N") {
    Write-Host ""
    Write-Host "Starting ADAS Backend Server..." -ForegroundColor Green
    Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow
    Write-Host ""
    python main.py
}
