#!/bin/bash
# Feature 017: Usage Data Collection System
# Setup Verification Script
# Version: 1.0.0

set -e

echo "========================================"
echo "Telemetry Setup Verification"
echo "========================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check functions
check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    exit 1
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# 1. Check Docker installation
echo "1. Checking Docker installation..."
if command -v docker &> /dev/null; then
    DOCKER_VERSION=$(docker --version | cut -d ' ' -f3 | cut -d ',' -f1)
    check_pass "Docker installed (version: $DOCKER_VERSION)"
else
    check_fail "Docker not found. Please install Docker Engine 20.10+"
fi

# 2. Check Docker Compose installation
echo ""
echo "2. Checking Docker Compose installation..."
if command -v docker-compose &> /dev/null || docker compose version &> /dev/null 2>&1; then
    if command -v docker-compose &> /dev/null; then
        COMPOSE_VERSION=$(docker-compose --version | cut -d ' ' -f4 | cut -d ',' -f1)
    else
        COMPOSE_VERSION=$(docker compose version --short)
    fi
    check_pass "Docker Compose installed (version: $COMPOSE_VERSION)"
else
    check_fail "Docker Compose not found. Please install Docker Compose 2.0+"
fi

# 3. Check if Docker daemon is running
echo ""
echo "3. Checking Docker daemon status..."
if docker info &> /dev/null; then
    check_pass "Docker daemon is running"
else
    check_fail "Docker daemon is not running. Please start Docker."
fi

# 4. Check directory structure
echo ""
echo "4. Checking directory structure..."
REQUIRED_DIRS=("clickhouse" "otel-collector" "scripts" "queries" "docs")
for dir in "${REQUIRED_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        check_pass "Directory exists: $dir/"
    else
        check_warn "Directory missing: $dir/ (will be created)"
        mkdir -p "$dir"
    fi
done

# 5. Check required files
echo ""
echo "5. Checking required configuration files..."
REQUIRED_FILES=(
    "docker-compose.yml"
    "clickhouse/schema.sql"
    "clickhouse/config.xml"
    "otel-collector/config.yaml"
    ".env.template"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        check_pass "File exists: $file"
    else
        check_fail "File missing: $file"
    fi
done

# 6. Check environment file
echo ""
echo "6. Checking environment configuration..."
if [ -f ".env" ]; then
    check_pass ".env file exists"

    # Check for default passwords
    if grep -q "change_me_in_production" .env 2>/dev/null; then
        check_warn "Default password detected in .env. Change CLICKHOUSE_PASSWORD for production use."
    fi
else
    check_warn ".env file not found. Copying from .env.template..."
    cp .env.template .env
    check_pass "Created .env from template. Please review and customize."
fi

# 7. Check available disk space
echo ""
echo "7. Checking available disk space..."
AVAILABLE_SPACE=$(df -h . | awk 'NR==2 {print $4}')
AVAILABLE_SPACE_GB=$(df -BG . | awk 'NR==2 {print $4}' | sed 's/G//')

if [ "$AVAILABLE_SPACE_GB" -ge 10 ]; then
    check_pass "Sufficient disk space available ($AVAILABLE_SPACE)"
else
    check_warn "Low disk space ($AVAILABLE_SPACE). Recommended: 10GB+ for telemetry data."
fi

# 8. Check available memory
echo ""
echo "8. Checking available memory..."
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    TOTAL_MEM_GB=$(sysctl -n hw.memsize | awk '{print int($1/1024/1024/1024)}')
else
    # Linux
    TOTAL_MEM_GB=$(free -g | awk '/^Mem:/ {print $2}')
fi

if [ "$TOTAL_MEM_GB" -ge 4 ]; then
    check_pass "Sufficient memory available (${TOTAL_MEM_GB}GB total)"
else
    check_warn "Low memory (${TOTAL_MEM_GB}GB). Recommended: 4GB+ RAM for telemetry stack."
fi

# 9. Check port availability
echo ""
echo "9. Checking port availability..."
REQUIRED_PORTS=(4317 4318 8123 9000 8888 13133)
for port in "${REQUIRED_PORTS[@]}"; do
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1 || netstat -an 2>/dev/null | grep -q ":$port.*LISTEN"; then
        check_warn "Port $port is in use. Docker may fail to bind."
    else
        check_pass "Port $port is available"
    fi
done

# 10. Check .gitignore configuration
echo ""
echo "10. Checking .gitignore configuration..."
if [ -f "../../.gitignore" ]; then
    if grep -q "telemetry-server/clickhouse_data/" ../../.gitignore 2>/dev/null; then
        check_pass "Telemetry data exclusions configured in .gitignore"
    else
        check_warn "Telemetry data exclusions not found in .gitignore"
    fi
else
    check_warn ".gitignore not found in repository root"
fi

# Summary
echo ""
echo "========================================"
echo "Verification Complete"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. Review and customize .env file"
echo "2. Start the telemetry stack: docker-compose up -d"
echo "3. Verify services: docker-compose ps"
echo "4. Check logs: docker-compose logs -f"
echo "5. Test connection: ./scripts/test-connection.sh"
echo ""
echo "For detailed setup instructions, see: docs/quickstart.md"
echo ""
