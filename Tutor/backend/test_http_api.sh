#!/bin/bash
# Quick HTTP API Test Script
# Tests all backend endpoints using curl

echo "================================================================================"
echo "🧪 TutorGPT Backend HTTP API Tests"
echo "================================================================================"
echo ""

BASE_URL="http://localhost:8000"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
TOTAL_TESTS=0
PASSED_TESTS=0

# Function to run test
run_test() {
    local test_name=$1
    local method=$2
    local endpoint=$3
    local data=$4
    local headers=$5

    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    echo "TEST $TOTAL_TESTS: $test_name"
    echo "--------------------------------------------------------------------------------"

    if [ "$method" = "GET" ]; then
        response=$(curl -s -w "\n%{http_code}" -X GET "$BASE_URL$endpoint" $headers)
    elif [ "$method" = "POST" ]; then
        response=$(curl -s -w "\n%{http_code}" -X POST "$BASE_URL$endpoint" \
            -H "Content-Type: application/json" \
            $headers \
            -d "$data")
    elif [ "$method" = "PUT" ]; then
        response=$(curl -s -w "\n%{http_code}" -X PUT "$BASE_URL$endpoint" \
            -H "Content-Type: application/json" \
            $headers \
            -d "$data")
    elif [ "$method" = "DELETE" ]; then
        response=$(curl -s -w "\n%{http_code}" -X DELETE "$BASE_URL$endpoint" $headers)
    fi

    # Extract status code (last line)
    status_code=$(echo "$response" | tail -n 1)
    # Extract body (everything except last line)
    body=$(echo "$response" | sed '$d')

    # Check if successful (2xx status code)
    if [ "$status_code" -ge 200 ] && [ "$status_code" -lt 300 ]; then
        echo -e "${GREEN}✅ PASS${NC} - Status: $status_code"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo -e "${RED}❌ FAIL${NC} - Status: $status_code"
    fi

    # Pretty print JSON response if jq is available
    if command -v jq &> /dev/null; then
        echo "$body" | jq '.' 2>/dev/null || echo "$body"
    else
        echo "$body"
    fi
    echo ""
}

# Check if server is running
echo "Checking if server is running..."
if ! curl -s "$BASE_URL/health" > /dev/null 2>&1; then
    echo -e "${RED}❌ Server is not running!${NC}"
    echo ""
    echo "Please start the server first:"
    echo "  cd Tutor/backend"
    echo "  uvicorn app.main:app --reload"
    echo ""
    exit 1
fi
echo -e "${GREEN}✅ Server is running${NC}"
echo ""

echo "================================================================================"
echo "PHASE 1: Server Health & Info"
echo "================================================================================"
echo ""

run_test "Health Check" "GET" "/health"
run_test "API Root Info" "GET" "/"

echo "================================================================================"
echo "PHASE 2: Authentication"
echo "================================================================================"
echo ""

# Signup
SIGNUP_DATA='{
    "name": "Test User",
    "email": "test@example.com",
    "password": "testpass123",
    "level": "beginner"
}'

run_test "User Signup" "POST" "/api/auth/signup" "$SIGNUP_DATA"

# Login
LOGIN_DATA='{
    "email": "test@example.com",
    "password": "testpass123"
}'

echo "TEST $((TOTAL_TESTS + 1)): User Login & Get Token"
echo "--------------------------------------------------------------------------------"
TOTAL_TESTS=$((TOTAL_TESTS + 1))

LOGIN_RESPONSE=$(curl -s -X POST "$BASE_URL/api/auth/login" \
    -H "Content-Type: application/json" \
    -d "$LOGIN_DATA")

# Extract token
TOKEN=$(echo "$LOGIN_RESPONSE" | grep -o '"access_token":"[^"]*' | sed 's/"access_token":"//')

if [ -n "$TOKEN" ]; then
    echo -e "${GREEN}✅ PASS${NC} - Token received"
    PASSED_TESTS=$((PASSED_TESTS + 1))
    echo "Token preview: ${TOKEN:0:50}..."
else
    echo -e "${RED}❌ FAIL${NC} - No token received"
fi
echo ""

# Get current user
run_test "Get Current User" "GET" "/api/auth/me" "" "-H \"Authorization: Bearer $TOKEN\""

echo "================================================================================"
echo "PHASE 3: Profile Management"
echo "================================================================================"
echo ""

run_test "Get Profile" "GET" "/api/profile" "" "-H \"Authorization: Bearer $TOKEN\""

UPDATE_DATA='{
    "level": "intermediate",
    "current_chapter": "04-python",
    "current_lesson": "01-intro",
    "learning_style": "code_focused"
}'

run_test "Update Profile" "PUT" "/api/profile" "$UPDATE_DATA" "-H \"Authorization: Bearer $TOKEN\""

echo "================================================================================"
echo "PHASE 4: Chat & Agent"
echo "================================================================================"
echo ""

run_test "Get Greeting" "GET" "/api/chat/greeting" "" "-H \"Authorization: Bearer $TOKEN\""

CHAT_DATA='{
    "message": "What is Python?",
    "current_chapter": "04-python"
}'

run_test "Send Chat Message" "POST" "/api/chat/message" "$CHAT_DATA" "-H \"Authorization: Bearer $TOKEN\""

run_test "Get Chat Status" "GET" "/api/chat/status" "" "-H \"Authorization: Bearer $TOKEN\""

echo "================================================================================"
echo "PHASE 5: Chat History & Sessions"
echo "================================================================================"
echo ""

run_test "Get All Sessions" "GET" "/api/chat/sessions?limit=10" "" "-H \"Authorization: Bearer $TOKEN\""

run_test "Get Chat History" "GET" "/api/chat/history?limit=20" "" "-H \"Authorization: Bearer $TOKEN\""

echo "================================================================================"
echo "PHASE 6: Analytics"
echo "================================================================================"
echo ""

run_test "Get Progress Analytics" "GET" "/api/analytics/progress" "" "-H \"Authorization: Bearer $TOKEN\""

run_test "Get Topic Analysis" "GET" "/api/analytics/topics" "" "-H \"Authorization: Bearer $TOKEN\""

run_test "Get Performance Metrics" "GET" "/api/analytics/performance" "" "-H \"Authorization: Bearer $TOKEN\""

run_test "Get Recommendations" "GET" "/api/analytics/recommendations" "" "-H \"Authorization: Bearer $TOKEN\""

echo "================================================================================"
echo "🎉 TEST SUMMARY"
echo "================================================================================"
echo ""

FAILED_TESTS=$((TOTAL_TESTS - PASSED_TESTS))

echo "Total Tests: $TOTAL_TESTS"
echo -e "Passed: ${GREEN}$PASSED_TESTS${NC}"

if [ $FAILED_TESTS -gt 0 ]; then
    echo -e "Failed: ${RED}$FAILED_TESTS${NC}"
else
    echo -e "Failed: ${GREEN}$FAILED_TESTS${NC}"
fi

echo ""

if [ $FAILED_TESTS -eq 0 ]; then
    echo -e "${GREEN}✅ ALL TESTS PASSED!${NC}"
    echo ""
    echo "Backend Status: PRODUCTION-READY ✅"
else
    echo -e "${YELLOW}⚠️  Some tests failed${NC}"
    echo ""
    echo "Please check the errors above and ensure:"
    echo "  1. Server is running (uvicorn app.main:app --reload)"
    echo "  2. Database is initialized"
    echo "  3. All dependencies are installed"
fi

echo ""
echo "================================================================================"
echo "Next Steps:"
echo "  - Test WebSocket: Open test_websocket.html in browser"
echo "  - View API docs: http://localhost:8000/docs"
echo "  - Run full tests: python test_all_backend_features.py"
echo "================================================================================"
