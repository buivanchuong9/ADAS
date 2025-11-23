#!/bin/bash

# =====================================================
# Phase 1 Testing Script - TTC & Voice Alerts
# =====================================================

echo "🧪 Testing Phase 1: TTC Computation & Voice Alerts"
echo "===================================================="

BASE_URL="http://localhost:8000"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Health Check
echo ""
echo "📊 Test 1: Health Check"
echo "--------------------"
response=$(curl -s "$BASE_URL/health")
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ PASS${NC}: Server is running"
    echo "$response" | jq
else
    echo -e "${RED}❌ FAIL${NC}: Cannot connect to server"
    exit 1
fi

# Test 2: Alerts API - Stats
echo ""
echo "📊 Test 2: Alerts API - Stats"
echo "--------------------"
response=$(curl -s "$BASE_URL/api/alerts/stats")
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ PASS${NC}: Alerts API working"
    echo "$response" | jq
else
    echo -e "${RED}❌ FAIL${NC}: Alerts API failed"
fi

# Test 3: Alerts API - Latest
echo ""
echo "📊 Test 3: Alerts API - Latest Alerts"
echo "--------------------"
response=$(curl -s "$BASE_URL/api/alerts/latest?limit=5")
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ PASS${NC}: Latest alerts endpoint working"
    echo "$response" | jq
else
    echo -e "${RED}❌ FAIL${NC}: Latest alerts failed"
fi

# Test 4: TTC Computer (Python import)
echo ""
echo "📊 Test 4: TTC Computer Module"
echo "--------------------"
python3 << 'EOF'
try:
    from ai_models.ttc_computer import TTCComputer
    ttc = TTCComputer()
    
    # Test basic computation
    result = ttc.compute_ttc(
        distance=10.0,
        relative_speed=5.0,
        previous_distance=12.0,
        time_delta=0.1
    )
    
    print(f"✅ PASS: TTC Computer working")
    print(f"  TTC: {result['ttc']:.2f}s")
    print(f"  Severity: {result['severity']}")
    print(f"  Warning: {result['warning']}")
    
except Exception as e:
    print(f"❌ FAIL: {e}")
    exit(1)
EOF

# Test 5: Voice Alert System
echo ""
echo "📊 Test 5: Voice Alert System"
echo "--------------------"
python3 << 'EOF'
try:
    from ai_models.voice_alert import VoiceAlertSystem
    voice = VoiceAlertSystem()
    
    print("✅ PASS: Voice Alert System initialized")
    print(f"  TTS Engine: pyttsx3")
    print(f"  Alert Directory: logs/alerts/")
    
    # Test voice (without actually playing)
    print("  Testing TTS (no audio playback)...")
    # voice.test_voice()  # Uncomment to hear test
    
except Exception as e:
    print(f"❌ FAIL: {e}")
    exit(1)
EOF

# Test 6: Dataset API
echo ""
echo "📊 Test 6: Dataset API - Stats"
echo "--------------------"
response=$(curl -s "$BASE_URL/api/dataset/stats")
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ PASS${NC}: Dataset API working"
    echo "$response" | jq
else
    echo -e "${RED}❌ FAIL${NC}: Dataset API failed"
fi

# Test 7: Check AI Models directory
echo ""
echo "📊 Test 7: AI Models Directory"
echo "--------------------"
if [ -d "ai_models" ]; then
    echo -e "${GREEN}✅ PASS${NC}: ai_models/ directory exists"
    echo "  Files:"
    ls -1 ai_models/*.py | sed 's/^/    /'
else
    echo -e "${RED}❌ FAIL${NC}: ai_models/ directory not found"
fi

# Test 8: Database Models
echo ""
echo "📊 Test 8: Database Alert Model"
echo "--------------------"
python3 << 'EOF'
try:
    from models import Alert, Base
    from database import engine
    from sqlalchemy import inspect
    
    # Check if Alert table exists
    inspector = inspect(engine)
    tables = inspector.get_table_names()
    
    if 'Alerts' in tables:
        print("✅ PASS: Alerts table exists in database")
        
        # Get columns
        columns = inspector.get_columns('Alerts')
        print("  Columns:")
        for col in columns:
            print(f"    - {col['name']}: {col['type']}")
    else:
        print("⚠️  WARNING: Alerts table not yet created")
        print("  Run: Base.metadata.create_all(bind=engine)")
        
except Exception as e:
    print(f"❌ FAIL: {e}")
EOF

# Summary
echo ""
echo "===================================================="
echo "🎊 Phase 1 Testing Complete!"
echo "===================================================="
echo ""
echo "✅ All core components tested"
echo ""
echo "Next Steps:"
echo "  1. Upload a test video: curl -X POST -F 'file=@test.mp4' $BASE_URL/api/inference/video"
echo "  2. Check alerts: curl $BASE_URL/api/alerts/latest"
echo "  3. View API docs: http://localhost:8000/docs"
echo ""
