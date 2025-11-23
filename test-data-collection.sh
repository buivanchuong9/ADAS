#!/bin/bash
# Test Data Collection System

echo "======================================"
echo "🧪 Testing ADAS Data Collection"
echo "======================================"

# Test 1: Check Backend Health
echo ""
echo "1️⃣ Testing Backend Health..."
HEALTH=$(curl -s http://localhost:8000/health)
if [ $? -eq 0 ]; then
    echo "✅ Backend is running"
    echo "   Response: $HEALTH"
else
    echo "❌ Backend is NOT running"
    echo "   Please start: cd backend-python && python3 main.py"
    exit 1
fi

# Test 2: Check Dataset Endpoint
echo ""
echo "2️⃣ Testing Dataset Endpoint..."
DATASET=$(curl -s http://localhost:8000/api/dataset)
if [ $? -eq 0 ]; then
    echo "✅ Dataset endpoint working"
    echo "   Current items: $(echo $DATASET | grep -o '\[' | wc -l)"
else
    echo "❌ Dataset endpoint failed"
    exit 1
fi

# Test 3: Check Dataset Stats
echo ""
echo "3️⃣ Testing Dataset Stats..."
STATS=$(curl -s http://localhost:8000/api/dataset/stats)
if [ $? -eq 0 ]; then
    echo "✅ Stats endpoint working"
    echo "$STATS" | python3 -m json.tool 2>/dev/null || echo "$STATS"
else
    echo "❌ Stats endpoint failed"
fi

# Test 4: Check API Docs
echo ""
echo "4️⃣ Checking API Documentation..."
DOCS=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8000/docs)
if [ "$DOCS" = "200" ]; then
    echo "✅ API docs available at: http://localhost:8000/docs"
else
    echo "⚠️  API docs may not be accessible"
fi

# Test 5: Check Frontend (if running)
echo ""
echo "5️⃣ Checking Frontend..."
FRONTEND=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:3000)
if [ "$FRONTEND" = "200" ]; then
    echo "✅ Frontend is running at: http://localhost:3000"
    echo "   Data Collection: http://localhost:3000/data-collection"
else
    echo "⚠️  Frontend is NOT running"
    echo "   Start with: npm run dev"
fi

# Test 6: Check Dataset Folder Structure
echo ""
echo "6️⃣ Checking Dataset Folder..."
DATASET_DIR="backend-python/dataset"
if [ -d "$DATASET_DIR" ]; then
    echo "✅ Dataset folder exists"
    echo "   📁 Images: $(ls -1 $DATASET_DIR/images/*.jpg 2>/dev/null | wc -l | tr -d ' ') files"
    echo "   📄 Labels: $(ls -1 $DATASET_DIR/labels/*.txt 2>/dev/null | wc -l | tr -d ' ') files"
    echo "   📦 Raw: $(ls -1 $DATASET_DIR/raw/* 2>/dev/null | wc -l | tr -d ' ') files"
else
    echo "⚠️  Dataset folder will be created on first upload"
fi

echo ""
echo "======================================"
echo "📊 Test Summary"
echo "======================================"
echo "Backend: ✅ Running on port 8000"
echo "Frontend: Check above"
echo "Dataset Endpoint: ✅ Working"
echo ""
echo "🎯 Next Steps:"
echo "1. Access: http://localhost:3000/data-collection"
echo "2. Upload an image"
echo "3. Draw bounding boxes"
echo "4. Submit dataset"
echo ""
echo "📖 Full guide: HUONG_DAN_DATA_COLLECTION.md"
echo "======================================"
