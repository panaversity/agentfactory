#!/bin/bash
# Feature 017: Usage Data Collection System
# Connection Testing Script
# Version: 1.0.0

set -e

echo "========================================"
echo "Telemetry Stack Connection Test"
echo "========================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test functions
test_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

test_fail() {
    echo -e "${RED}✗${NC} $1"
}

test_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

# Load environment variables
if [ -f .env ]; then
    export $(cat .env | grep -v '^#' | xargs)
fi

CLICKHOUSE_USER=${CLICKHOUSE_USER:-telemetry_user}
CLICKHOUSE_PASSWORD=${CLICKHOUSE_PASSWORD:-change_me_in_production}

# 1. Test ClickHouse HTTP interface
echo "1. Testing ClickHouse HTTP interface (port 8123)..."
if curl -s --max-time 5 http://localhost:8123/ping > /dev/null 2>&1; then
    test_pass "ClickHouse HTTP interface is responding"

    # Test authentication
    RESPONSE=$(curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
        "http://localhost:8123/?query=SELECT%20version()")

    if [ -n "$RESPONSE" ]; then
        test_pass "Authentication successful (ClickHouse version: $RESPONSE)"
    else
        test_fail "Authentication failed. Check credentials in .env"
    fi
else
    test_fail "ClickHouse HTTP interface not accessible. Is the service running?"
    exit 1
fi

# 2. Test ClickHouse database
echo ""
echo "2. Testing telemetry database..."
DB_CHECK=$(curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
    "http://localhost:8123/?query=SHOW%20DATABASES%20LIKE%20'telemetry'" || echo "")

if echo "$DB_CHECK" | grep -q "telemetry"; then
    test_pass "Telemetry database exists"

    # Check tables
    TABLE_COUNT=$(curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
        "http://localhost:8123/?query=SELECT%20count()%20FROM%20system.tables%20WHERE%20database%3D'telemetry'" || echo "0")

    test_info "Found $TABLE_COUNT tables in telemetry database"
else
    test_fail "Telemetry database not found. Schema may not have been initialized."
fi

# 3. Test telemetry_events table
echo ""
echo "3. Testing telemetry_events table..."
TABLE_CHECK=$(curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
    "http://localhost:8123/?query=EXISTS%20TABLE%20telemetry.telemetry_events" || echo "0")

if [ "$TABLE_CHECK" = "1" ]; then
    test_pass "telemetry_events table exists"

    # Check row count
    ROW_COUNT=$(curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
        "http://localhost:8123/?query=SELECT%20count()%20FROM%20telemetry.telemetry_events" || echo "0")

    test_info "Current event count: $ROW_COUNT"
else
    test_fail "telemetry_events table not found"
fi

# 4. Test OTLP Collector gRPC endpoint
echo ""
echo "4. Testing OTLP Collector gRPC endpoint (port 4317)..."
if nc -z localhost 4317 2>/dev/null || (echo > /dev/tcp/localhost/4317) 2>/dev/null; then
    test_pass "OTLP Collector gRPC port is open"
else
    test_fail "OTLP Collector gRPC port (4317) is not accessible"
fi

# 5. Test OTLP Collector HTTP endpoint
echo ""
echo "5. Testing OTLP Collector HTTP endpoint (port 4318)..."
if nc -z localhost 4318 2>/dev/null || (echo > /dev/tcp/localhost/4318) 2>/dev/null; then
    test_pass "OTLP Collector HTTP port is open"
else
    test_fail "OTLP Collector HTTP port (4318) is not accessible"
fi

# 6. Test OTLP Collector health endpoint
echo ""
echo "6. Testing OTLP Collector health check (port 13133)..."
HEALTH_CHECK=$(curl -s --max-time 5 http://localhost:13133/ 2>/dev/null || echo "")

if [ -n "$HEALTH_CHECK" ]; then
    test_pass "OTLP Collector health endpoint is responding"
else
    test_fail "OTLP Collector health endpoint not accessible"
fi

# 7. Test Prometheus metrics endpoint
echo ""
echo "7. Testing Prometheus metrics endpoint (port 8888)..."
METRICS=$(curl -s --max-time 5 http://localhost:8888/metrics 2>/dev/null | head -n 5)

if [ -n "$METRICS" ]; then
    test_pass "Prometheus metrics endpoint is responding"
else
    test_fail "Prometheus metrics endpoint not accessible"
fi

# 8. Send test event
echo ""
echo "8. Sending test telemetry event..."

TEST_EVENT=$(cat <<EOF
{
  "event_type": "user_prompt",
  "session_id": "$(uuidgen)",
  "user_id": "${USER}",
  "timestamp": "$(date -u +"%Y-%m-%d %H:%M:%S")",
  "prompt_text_hash": "$(echo 'test prompt' | shasum -a 256 | cut -d' ' -f1)",
  "workflow_step": "test",
  "feature_name": "setup-verification",
  "model_name": "claude-sonnet-4-5-20250929",
  "event_data": "{\"test\": true}"
}
EOF
)

INSERT_QUERY="INSERT INTO telemetry.telemetry_events FORMAT JSONEachRow $TEST_EVENT"
INSERT_RESULT=$(curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
    -X POST "http://localhost:8123/" \
    -d "$INSERT_QUERY" 2>&1)

if [ -z "$INSERT_RESULT" ]; then
    test_pass "Test event inserted successfully"

    # Verify insertion
    sleep 1
    NEW_COUNT=$(curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
        "http://localhost:8123/?query=SELECT%20count()%20FROM%20telemetry.telemetry_events%20WHERE%20feature_name%3D'setup-verification'" || echo "0")

    if [ "$NEW_COUNT" -gt "0" ]; then
        test_pass "Test event verified in database (count: $NEW_COUNT)"
    fi
else
    test_fail "Failed to insert test event: $INSERT_RESULT"
fi

# 9. Test query performance
echo ""
echo "9. Testing query performance..."
START_TIME=$(date +%s%N)
curl -s -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
    "http://localhost:8123/?query=SELECT%20count()%2C%20event_type%20FROM%20telemetry.telemetry_events%20GROUP%20BY%20event_type" > /dev/null
END_TIME=$(date +%s%N)
QUERY_TIME_MS=$(( (END_TIME - START_TIME) / 1000000 ))

if [ "$QUERY_TIME_MS" -lt 1000 ]; then
    test_pass "Query performance is good (${QUERY_TIME_MS}ms)"
else
    test_info "Query took ${QUERY_TIME_MS}ms (acceptable for small dataset)"
fi

# Summary
echo ""
echo "========================================"
echo "Connection Test Complete"
echo "========================================"
echo ""

# Check Docker container status
echo "Docker container status:"
docker-compose ps

echo ""
echo "To view live logs:"
echo "  docker-compose logs -f"
echo ""
echo "To stop the stack:"
echo "  docker-compose down"
echo ""
echo "To restart the stack:"
echo "  docker-compose restart"
echo ""
echo "For query examples, see: queries/README.md"
echo ""
