#!/bin/bash

# ADAS System - Production API Test Script
# Test connectivity với production server

echo "🧪 ADAS Production API Test"
echo "======================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

API_URL="https://adas-api.aiotlab.edu.vn"

echo "📡 Testing API: $API_URL"
echo ""

# Test 1: Health Check
echo "1️⃣  Testing Health Endpoint..."
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$API_URL/health")
if [ "$HTTP_CODE" == "200" ]; then
    echo -e "${GREEN}✅ Health check passed (HTTP $HTTP_CODE)${NC}"
    curl -s "$API_URL/health" | python3 -m json.tool
else
    echo -e "${RED}❌ Health check failed (HTTP $HTTP_CODE)${NC}"
fi
echo ""

# Test 2: API Documentation
echo "2️⃣  Testing API Documentation..."
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$API_URL/docs")
if [ "$HTTP_CODE" == "200" ]; then
    echo -e "${GREEN}✅ API docs available at: $API_URL/docs${NC}"
else
    echo -e "${YELLOW}⚠️  API docs returned HTTP $HTTP_CODE${NC}"
fi
echo ""

# Test 3: Admin Overview
echo "3️⃣  Testing Admin Overview..."
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$API_URL/admin/overview")
if [ "$HTTP_CODE" == "200" ]; then
    echo -e "${GREEN}✅ Admin overview accessible${NC}"
    curl -s "$API_URL/admin/overview" | python3 -m json.tool
else
    echo -e "${RED}❌ Admin overview failed (HTTP $HTTP_CODE)${NC}"
fi
echo ""

# Test 4: Admin Statistics
echo "4️⃣  Testing Admin Statistics..."
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "$API_URL/admin/statistics")
if [ "$HTTP_CODE" == "200" ]; then
    echo -e "${GREEN}✅ Admin statistics accessible${NC}"
    curl -s "$API_URL/admin/statistics" | python3 -m json.tool
else
    echo -e "${RED}❌ Admin statistics failed (HTTP $HTTP_CODE)${NC}"
fi
echo ""

# Summary
echo "======================================"
echo "🏁 Test Summary"
echo "======================================"
echo "API URL: $API_URL"
echo "Port: 52000"
echo "Protocol: HTTPS"
echo ""
echo "Next steps:"
echo "1. Start frontend: cd ADAS/FrontEnd && pnpm dev"
echo "2. Open browser: http://localhost:3000"
echo "3. Test upload video functionality"
echo ""
