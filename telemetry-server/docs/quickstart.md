# Telemetry Server - Quick Start Guide

**Feature 017**: Usage Data Collection System  
**Estimated Setup Time**: 10-15 minutes  
**Target**: Book development team members

## Overview

This guide walks you through setting up the telemetry infrastructure on your local machine. After completion, Claude Code will automatically export usage data to your local ClickHouse database for analysis.

## Prerequisites

### Required Software

- **Docker Engine 20.10+** ([Install Docker](https://docs.docker.com/get-docker/))
- **Docker Compose 2.0+** (included with Docker Desktop)
- **Claude Code** (your existing installation)

### System Requirements

- **RAM**: 4GB+ available (2GB for ClickHouse, 512MB for OTLP Collector)
- **Disk**: 10GB+ free space (telemetry data grows over time)
- **OS**: macOS, Linux, or Windows with WSL2

### Check Your Setup

```bash
# Verify Docker
docker --version
# Expected: Docker version 20.10.0 or higher

# Verify Docker Compose
docker compose version
# Expected: Docker Compose version v2.0.0 or higher

# Verify Docker is running
docker info
# Should show server information (not an error)
```

## Setup Steps

### Phase 1: Initial Setup (5 minutes)

#### Step 1: Navigate to Telemetry Directory

```bash
cd telemetry-server
```

#### Step 2: Run Pre-Flight Checks

```bash
./scripts/verify-setup.sh
```

**Expected Output**:
```
✓ Docker installed (version: 24.0.5)
✓ Docker Compose installed (version: v2.20.2)
✓ Docker daemon is running
✓ Directory exists: clickhouse/
✓ Directory exists: otel-collector/
✓ File exists: docker-compose.yml
✓ File exists: clickhouse/schema.sql
✓ Port 4317 is available
✓ Port 8123 is available
```

**If any checks fail**, see [Troubleshooting](#troubleshooting) section below.

#### Step 3: Configure Environment

```bash
# Copy environment template
cp .env.template .env

# Edit configuration (use your preferred editor)
nano .env
```

**IMPORTANT**: Change these values in `.env`:

```bash
# BEFORE (insecure defaults)
CLICKHOUSE_PASSWORD=change_me_in_production
TELEMETRY_READER_PASSWORD=readonly_password

# AFTER (your secure passwords)
CLICKHOUSE_PASSWORD=your_secure_password_here_min_12_chars
TELEMETRY_READER_PASSWORD=another_secure_password_here
```

**Recommended Changes**:
```bash
# Personalize user identification
USER_ID=your_name  # e.g., USER_ID=mjs

# Adjust data retention (default is 90 days)
TELEMETRY_DATA_RETENTION_DAYS=90
```

Save and exit (`Ctrl+X`, then `Y`, then `Enter` in nano).

### Phase 2: Start Services (3 minutes)

#### Step 4: Launch Telemetry Stack

```bash
# Start services in background
docker-compose up -d
```

**Expected Output**:
```
[+] Running 3/3
 ✔ Network telemetry-network      Created
 ✔ Container telemetry-clickhouse Started
 ✔ Container telemetry-otel-collector Started
```

#### Step 5: Verify Services Started

```bash
docker-compose ps
```

**Expected Output**:
```
NAME                      STATUS    PORTS
telemetry-clickhouse      Up        0.0.0.0:8123->8123/tcp, 0.0.0.0:9000->9000/tcp
telemetry-otel-collector  Up        0.0.0.0:4317-4318->4317-4318/tcp
```

**Status should be "Up"**. If status is "Restarting" or "Exited", see [Troubleshooting](#service-wont-start).

#### Step 6: Test Connections

```bash
./scripts/test-connection.sh
```

**Expected Output** (abbreviated):
```
✓ ClickHouse HTTP interface is responding
✓ Authentication successful (ClickHouse version: 24.3.1.2)
✓ Telemetry database exists
✓ telemetry_events table exists
✓ OTLP Collector gRPC port is open
✓ OTLP Collector health endpoint is responding
✓ Test event inserted successfully
✓ Test event verified in database (count: 1)
```

**If any checks fail**, see [Troubleshooting](#connection-issues).

### Phase 3: Enable Claude Code Telemetry (2 minutes)

#### Step 7: Configure Claude Code

**Option A: Temporary (Single Session)**

```bash
# Start Claude Code with telemetry enabled
claude-code --env ../.claude/env-config/telemetry-enabled.env
```

**Option B: Permanent (All Sessions)**

Add to your shell profile (`~/.bashrc`, `~/.zshrc`, etc.):

```bash
# Enable Claude Code telemetry
export CLAUDE_ENV_CONFIG=/path/to/ai-native-software-development/.claude/env-config/telemetry-enabled.env
```

Then reload your shell:
```bash
source ~/.zshrc  # or ~/.bashrc
```

#### Step 8: Verify Telemetry Collection

```bash
# Use Claude Code normally for a few minutes
claude-code

# In another terminal, check event count
docker exec -it telemetry-clickhouse clickhouse-client \
  --query "SELECT count() FROM telemetry.telemetry_events"
```

**Expected**: Event count should increase as you use Claude Code.

### Phase 4: First Query (2 minutes)

#### Step 9: View Your First Session

```bash
# Connect to ClickHouse
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user \
  --password your_password_here

# Run query
USE telemetry;

SELECT 
    session_id,
    user_id,
    count() AS events,
    min(timestamp) AS start_time,
    max(timestamp) AS end_time
FROM telemetry_events
GROUP BY session_id, user_id
ORDER BY start_time DESC
LIMIT 10;
```

**Exit ClickHouse client**: Type `exit` and press Enter.

## What's Next?

### Explore Query Examples

```bash
cd queries/
ls
# error-analysis.sql      - Andrew Ng's error analysis methodology
# session-analysis.sql    - Productivity and workflow analysis
# tool-usage.sql          - Tool performance and usage patterns
```

Run example queries:
```bash
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password your_password \
  --database telemetry --multiquery < queries/session-analysis.sql
```

### View Live Logs

```bash
# All services
docker-compose logs -f

# Specific service
docker-compose logs -f clickhouse
docker-compose logs -f otel-collector
```

### Access Prometheus Metrics

- **Collector metrics**: http://localhost:8888/metrics
- **Health check**: http://localhost:13133/

### Daily Workflow

1. **Start day**: `docker-compose start` (if stopped)
2. **Use Claude Code**: Work normally with telemetry enabled
3. **Analyze data**: Run queries in `queries/` directory
4. **End day**: `docker-compose stop` (optional, can leave running)

## Common Operations

### Stop Telemetry Stack

```bash
docker-compose stop
```

Data is preserved. Restart with `docker-compose start`.

### Restart Services

```bash
docker-compose restart
```

Useful if configuration changes are made.

### View Current Status

```bash
docker-compose ps
```

### Check Resource Usage

```bash
docker stats telemetry-clickhouse telemetry-otel-collector
```

**Expected**:
- ClickHouse: ~200-500MB RAM (grows with data)
- OTLP Collector: ~50-100MB RAM

### Export Data

```bash
# Export last 7 days to CSV
docker exec -it telemetry-clickhouse clickhouse-client \
  --query "SELECT * FROM telemetry.telemetry_events WHERE timestamp >= now() - INTERVAL 7 DAY" \
  --format CSV > telemetry-export-$(date +%Y%m%d).csv
```

### Backup Database

```bash
# Create backup directory
mkdir -p backups

# Backup ClickHouse data
docker exec telemetry-clickhouse clickhouse-client \
  --query "BACKUP TABLE telemetry.telemetry_events TO Disk('default', 'backup-$(date +%Y%m%d).zip')"
```

### Reset Everything (⚠️ Deletes All Data)

```bash
# Stop and remove containers + volumes
docker-compose down -v

# Remove local data
rm -rf clickhouse_data/ otel_logs/

# Start fresh
docker-compose up -d
```

## Troubleshooting

### Service Won't Start

**Symptom**: `docker-compose ps` shows "Exited" or "Restarting"

**Solution**:
```bash
# Check logs for errors
docker-compose logs clickhouse
docker-compose logs otel-collector

# Common issue: Port already in use
lsof -i :8123  # Check if ClickHouse port is used
lsof -i :4317  # Check if OTLP port is used

# Kill conflicting process or change ports in docker-compose.yml
```

### Connection Issues

**Symptom**: `./scripts/test-connection.sh` fails with "Connection refused"

**Solution**:
```bash
# Verify services are running
docker-compose ps

# Check Docker network
docker network ls | grep telemetry

# Restart services
docker-compose restart

# Wait 30 seconds for startup
sleep 30
./scripts/test-connection.sh
```

### Authentication Errors

**Symptom**: "Authentication failed" or "Access denied"

**Solution**:
```bash
# Verify credentials in .env
cat .env | grep CLICKHOUSE_PASSWORD

# Restart with new credentials
docker-compose down
docker-compose up -d

# Wait for initialization
sleep 30
./scripts/test-connection.sh
```

### No Data Appearing

**Symptom**: Claude Code running but event count stays at 0

**Solution**:
```bash
# Verify environment is loaded
env | grep OTEL_EXPORTER

# If empty, reload telemetry config
export CLAUDE_ENV_CONFIG=/path/to/.claude/env-config/telemetry-enabled.env

# Restart Claude Code
claude-code --env $CLAUDE_ENV_CONFIG

# Check OTLP Collector is receiving data
docker-compose logs otel-collector | grep -i "batch"
```

### Disk Space Issues

**Symptom**: "No space left on device"

**Solution**:
```bash
# Check disk usage
df -h .

# Check ClickHouse data size
du -sh clickhouse_data/

# Reduce retention period (edit .env)
TELEMETRY_DATA_RETENTION_DAYS=30  # Reduce from 90

# Restart to apply
docker-compose restart

# Manually delete old data
docker exec -it telemetry-clickhouse clickhouse-client \
  --query "ALTER TABLE telemetry.telemetry_events DELETE WHERE timestamp < now() - INTERVAL 30 DAY"
```

### Performance Issues

**Symptom**: Slow queries or high CPU usage

**Solution**:
```bash
# Check resource limits
docker stats

# Increase memory limits in docker-compose.yml
# Change from 2G to 4G for ClickHouse

# Optimize table (defragmentation)
docker exec -it telemetry-clickhouse clickhouse-client \
  --query "OPTIMIZE TABLE telemetry.telemetry_events FINAL"
```

## Getting Help

### Documentation

- **Troubleshooting Guide**: `docs/troubleshooting.md`
- **Query Examples**: `queries/README.md`
- **Feature Spec**: `../specs/017-usage-data-collection/spec.md`

### Log Analysis

```bash
# View last 100 lines
docker-compose logs --tail=100

# Follow live logs
docker-compose logs -f

# Export logs for sharing
docker-compose logs > telemetry-logs-$(date +%Y%m%d).txt
```

### Contact

- **Team**: Book development team
- **Spec Owner**: Check `../specs/017-usage-data-collection/spec.md`

## Success Criteria

You've successfully completed setup when:

- ✅ `docker-compose ps` shows both services "Up"
- ✅ `./scripts/test-connection.sh` passes all checks
- ✅ Event count increases when using Claude Code
- ✅ Queries return results in < 1 second
- ✅ Total setup time < 15 minutes

## Next Steps

1. **Run error analysis** (Andrew Ng's methodology):
   ```bash
   docker exec -it telemetry-clickhouse clickhouse-client \
     --database telemetry --multiquery < queries/error-analysis.sql
   ```

2. **Create custom queries** for your specific analysis needs

3. **Set up automated reports** (Phase 2 roadmap)

4. **Share findings** with team for continuous improvement

---

**Ready to collect insights!** 🚀

For advanced configuration and troubleshooting, see `docs/troubleshooting.md`
