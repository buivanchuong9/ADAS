#!/bin/bash
# ADAS Platform - Unified Installation Script
# Support: Windows (Git Bash), macOS, Linux
# Usage: bash setup.sh

set -e

echo "============================================"
echo "ADAS Platform - Automatic Installation"
echo "============================================"
echo ""

# Detect OS
OS="$(uname -s)"
case "${OS}" in
    Linux*)     OS_TYPE=LINUX;;
    Darwin*)    OS_TYPE=MACOS;;
    MINGW*)     OS_TYPE=WINDOWS;;
    *)          OS_TYPE="UNKNOWN";;
esac

echo "Detected OS: $OS_TYPE"
echo ""

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to install dependencies based on OS
install_system_deps() {
    if [ "$OS_TYPE" = "MACOS" ]; then
        echo "Installing macOS dependencies..."
        if ! command_exists brew; then
            echo "Installing Homebrew..."
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        fi
        brew install node dotnet python@3.11
    elif [ "$OS_TYPE" = "LINUX" ]; then
        echo "Installing Linux dependencies..."
        sudo apt-get update
        sudo apt-get install -y nodejs npm dotnet-sdk-8.0 python3.11 python3-pip
    elif [ "$OS_TYPE" = "WINDOWS" ]; then
        echo "Windows detected. Please install manually:"
        echo "1. Node.js: https://nodejs.org/"
        echo "2. .NET SDK: https://dotnet.microsoft.com/en-us/download"
        echo "3. Python: https://www.python.org/downloads/"
        echo ""
        echo "After installation, run this script again."
        exit 1
    fi
}

# Check and install dependencies
echo "[1/6] Checking system dependencies..."

if ! command_exists node; then
    echo "Node.js not found. Installing..."
    install_system_deps
else
    echo "✓ Node.js: $(node --version)"
fi

if ! command_exists dotnet; then
    echo ".NET SDK not found. Installing..."
    install_system_deps
else
    echo "✓ .NET SDK: $(dotnet --version)"
fi

if ! command_exists python3; then
    echo "Python not found. Installing..."
    install_system_deps
else
    echo "✓ Python: $(python3 --version)"
fi

# Install npm dependencies
echo ""
echo "[2/6] Installing npm packages..."
npm install --legacy-peer-deps
echo "✓ npm packages installed"

# Install Python dependencies
echo ""
echo "[3/6] Installing Python dependencies..."
cd model-worker
pip install -r requirements.txt
cd ..
echo "✓ Python dependencies installed"

# Restore .NET packages
echo ""
echo "[4/6] Restoring .NET packages..."
cd backend
dotnet restore
echo "✓ .NET packages restored"
cd ..

# Setup Firebase (check if service account exists)
echo ""
echo "[5/6] Checking Firebase configuration..."
if [ ! -f "backend/firebase-service-account.json" ]; then
    echo "⚠️  Firebase service account file not found!"
    echo "Please add: backend/firebase-service-account.json"
    echo "Download from Firebase Console → Project Settings → Service Accounts"
else
    echo "✓ Firebase service account found"
fi

# Create run scripts
echo ""
echo "[6/6] Creating run scripts..."

# Create run.sh for Unix systems
if [ "$OS_TYPE" != "WINDOWS" ]; then
    cat > run.sh << 'EOF'
#!/bin/bash
# Run ADAS Platform - Multi-terminal launcher

echo "Starting ADAS Platform..."
echo ""

# Open terminals based on OS
if command -v gnome-terminal >/dev/null; then
    # Linux with GNOME
    gnome-terminal -- bash -c 'cd model-worker && uvicorn app:app --host 0.0.0.0 --port 8000; bash'
    sleep 1
    gnome-terminal -- bash -c 'cd backend && dotnet run; bash'
    sleep 1
    gnome-terminal -- bash -c 'npm run dev; bash'
elif command -v osascript >/dev/null; then
    # macOS
    osascript <<OSASCRIPT
tell application "Terminal"
    do script "cd model-worker && uvicorn app:app --host 0.0.0.0 --port 8000"
    delay 1
    do script "cd backend && dotnet run"
    delay 1
    do script "npm run dev"
end tell
OSASCRIPT
fi

echo ""
echo "============================================"
echo "All services started!"
echo "============================================"
echo ""
echo "Access:"
echo "  Dashboard: http://localhost:3000/dashboard"
echo "  Live Detection: http://localhost:3000/live"
echo "  Model Worker: http://localhost:8000/health"
EOF
    chmod +x run.sh
    echo "✓ Created run.sh"
fi

echo ""
echo "============================================"
echo "Installation Complete! ✅"
echo "============================================"
echo ""
echo "Next steps:"
echo ""
echo "1. On macOS/Linux, run:"
echo "   bash run.sh"
echo ""
echo "2. Or run services manually:"
echo ""
echo "   Terminal 1 - Model Worker:"
echo "   cd model-worker"
echo "   uvicorn app:app --host 0.0.0.0 --port 8000"
echo ""
echo "   Terminal 2 - Backend:"
echo "   cd backend"
echo "   dotnet run"
echo ""
echo "   Terminal 3 - Frontend:"
echo "   npm run dev"
echo ""
echo "3. Access:"
echo "   Dashboard: http://localhost:3000/dashboard"
echo "   Live Detection: http://localhost:3000/live"
echo ""
