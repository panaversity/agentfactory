# Telemetry Server - Troubleshooting Guide

**Feature 017**: Usage Data Collection System  
**Version**: 1.0.0

This guide covers common issues, their diagnoses, and solutions.

## Table of Contents

1. [Service Issues](#service-issues)
2. [Connection Issues](#connection-issues)
3. [Data Collection Issues](#data-collection-issues)
4. [Performance Issues](#performance-issues)
5. [Docker Issues](#docker-issues)
6. [Query Issues](#query-issues)
7. [Security Issues](#security-issues)
8. [Advanced Debugging](#advanced-debugging)

---

## Service Issues

### Issue: Services Won't Start

**Symptoms**:
- `docker-compose up -d` completes but `docker-compose ps` shows "Exited"
- Services show "Restarting" status
- ClickHouse or OTLP Collector containers keep restarting

**Diagnosis**:
```bash
# Check logs for errors
docker-compose logs clickhouse
docker-compose logs otel-collector

# Check container status
docker-compose ps -a
```

**Common Causes & Solutions**:

#### Cause 1: Port Already in Use

**Error in logs**: "address already in use" or "bind: address already in use"

**Solution**:
```bash
# Find what's using the port
lsof -i :8123   # ClickHouse HTTP
lsof -i :9000   # ClickHouse Native
lsof -i :4317   # OTLP gRPC
lsof -i :4318   # OTLP HTTP

# Kill the process (if safe)
kill -9 <PID>

# Or change port in docker-compose.yml
# Example: Change "8123:8123" to "8124:8123"
```

#### Cause 2: Insufficient Memory

**Error in logs**: "Cannot allocate memory" or "OOM"

**Solution**:
```bash
# Check available memory
free -h  # Linux
vm_stat | perl -ne '/page size of (\d+)/ and $size=$1; /Pages (free|active|inactive|speculative|throttled|wired down): (\d+)/ and printf("%-16s % 16.2f Mi\n", "$1:", $2 * $size / 1048576);'  # macOS

# Reduce resource limits in docker-compose.yml
# ClickHouse: Change from 2G to 1G
# OTLP Collector: Change from 512M to 256M

# Restart
docker-compose down
docker-compose up -d
```

#### Cause 3: Invalid Configuration

**Error in logs**: "config error" or "parsing error"

**Solution**:
```bash
# Validate docker-compose.yml
docker-compose config

# Validate ClickHouse config
docker run --rm -v $(pwd)/clickhouse/config.xml:/config.xml clickhouse/clickhouse-server:latest clickhouse-server --config-file=/config.xml --dry-run

# Validate OTLP config
docker run --rm -v $(pwd)/otel-collector/config.yaml:/config.yaml otel/opentelemetry-collector-contrib:latest otelcol validate --config=/config.yaml

# Fix any reported errors
```

#### Cause 4: Missing .env File

**Error in logs**: "variable is not set" or "invalid interpolation"

**Solution**:
```bash
# Check if .env exists
ls -la .env

# If missing, create from template
cp .env.template .env

# Edit required values
nano .env

# Restart
docker-compose down
docker-compose up -d
```

### Issue: Service Starts Then Crashes

**Symptoms**:
- Service starts successfully
- After a few seconds/minutes, status changes to "Exited"

**Diagnosis**:
```bash
# Watch logs in real-time
docker-compose logs -f clickhouse

# Check exit code
docker-compose ps
# Look for "Exited (1)" or other non-zero codes
```

**Solution**:
```bash
# Check disk space
df -h .

# Check for corrupted data
rm -rf clickhouse_data/
docker-compose up -d

# Check for permission issues
ls -la clickhouse_data/
sudo chown -R $USER clickhouse_data/  # Linux
```

---

## Connection Issues

### Issue: Cannot Connect to ClickHouse

**Symptoms**:
- `curl http://localhost:8123/ping` returns "Connection refused"
- Test connection script fails

**Diagnosis**:
```bash
# Check if service is running
docker-compose ps clickhouse

# Check if port is exposed
docker port telemetry-clickhouse

# Check if listening on correct interface
docker exec telemetry-clickhouse netstat -tlnp | grep 8123
```

**Solutions**:

#### Solution 1: Service Not Running
```bash
docker-compose start clickhouse
sleep 10  # Wait for startup
./scripts/test-connection.sh
```

#### Solution 2: Firewall Blocking
```bash
# macOS
sudo pfctl -d  # Disable firewall temporarily for testing
# If this works, add firewall rule

# Linux (ufw)
sudo ufw allow 8123/tcp
sudo ufw allow 4317/tcp

# Linux (iptables)
sudo iptables -A INPUT -p tcp --dport 8123 -j ACCEPT
sudo iptables -A INPUT -p tcp --dport 4317 -j ACCEPT
```

#### Solution 3: Docker Network Issue
```bash
# Recreate network
docker-compose down
docker network prune
docker-compose up -d
```

### Issue: Authentication Failed

**Symptoms**:
- "Access denied" error
- "Invalid user or password"
- 401/403 HTTP errors

**Diagnosis**:
```bash
# Check environment variables
cat .env | grep CLICKHOUSE_

# Test with default credentials
curl -u "default:" http://localhost:8123/?query=SELECT%201
```

**Solutions**:

#### Solution 1: Wrong Credentials
```bash
# Reset to known credentials
docker-compose down -v
# Edit .env with new passwords
docker-compose up -d

# Wait for initialization
sleep 30

# Test with new credentials
curl -u "telemetry_user:your_new_password" http://localhost:8123/ping
```

#### Solution 2: User Not Created
```bash
# Check if users exist
docker exec telemetry-clickhouse clickhouse-client \
  --query "SELECT name FROM system.users"

# If telemetry_user missing, recreate
docker-compose down -v
docker-compose up -d
```

---

## Data Collection Issues

### Issue: No Telemetry Data Appearing

**Symptoms**:
- Claude Code running but `SELECT count() FROM telemetry_events` returns 0
- OTLP Collector shows no incoming data in logs

**Diagnosis**:
```bash
# Check if Claude Code is configured
env | grep OTEL_

# Check OTLP Collector logs
docker-compose logs otel-collector | grep -i "receiver"

# Test manual event insertion
curl -X POST http://localhost:8123/ \
  -u "telemetry_user:password" \
  --data "INSERT INTO telemetry.telemetry_events (event_type, session_id, user_id) VALUES ('test', generateUUIDv4(), 'test_user')"

# Verify insertion
curl -u "telemetry_user:password" \
  "http://localhost:8123/?query=SELECT%20count()%20FROM%20telemetry.telemetry_events"
```

**Solutions**:

#### Solution 1: Environment Not Loaded
```bash
# Load telemetry config
export CLAUDE_ENV_CONFIG=/full/path/to/.claude/env-config/telemetry-enabled.env

# Verify it's set
echo $CLAUDE_ENV_CONFIG

# Restart Claude Code
claude-code --env $CLAUDE_ENV_CONFIG
```

#### Solution 2: Wrong OTLP Endpoint
```bash
# Check endpoint in .claude/env-config/telemetry-enabled.env
cat .claude/env-config/telemetry-enabled.env | grep OTEL_EXPORTER_OTLP_ENDPOINT

# Should be: http://localhost:4317

# If wrong, fix and restart Claude Code
```

#### Solution 3: OTLP Collector Not Receiving
```bash
# Test OTLP endpoint manually
# Install grpcurl: https://github.com/fullstorydev/grpcurl
grpcurl -plaintext localhost:4317 list

# If this fails, OTLP Collector has issues
docker-compose restart otel-collector
```

### Issue: Data Collection Stops After Some Time

**Symptoms**:
- Initially data appears
- After hours/days, no new data despite usage

**Diagnosis**:
```bash
# Check when last data was inserted
docker exec telemetry-clickhouse clickhouse-client \
  --query "SELECT max(timestamp) FROM telemetry.telemetry_events"

# Check OTLP Collector health
curl http://localhost:13133/

# Check for disk space issues
df -h .
```

**Solutions**:

#### Solution 1: Disk Full
```bash
# Check disk usage
du -sh clickhouse_data/

# Clear old data
docker exec telemetry-clickhouse clickhouse-client \
  --query "ALTER TABLE telemetry.telemetry_events DELETE WHERE timestamp < now() - INTERVAL 30 DAY"

# Reduce retention in schema.sql (change TTL)
```

#### Solution 2: OTLP Queue Full
```bash
# Check OTLP Collector metrics
curl http://localhost:8888/metrics | grep queue

# Increase queue size in otel-collector/config.yaml
# Change queue_size from 5000 to 10000

docker-compose restart otel-collector
```

---

## Performance Issues

### Issue: Slow Queries

**Symptoms**:
- Queries take > 5 seconds
- High CPU usage on ClickHouse

**Diagnosis**:
```bash
# Check query performance
docker exec telemetry-clickhouse clickhouse-client \
  --query "SELECT query, elapsed, memory_usage FROM system.query_log ORDER BY elapsed DESC LIMIT 10"

# Check table size
docker exec telemetry-clickhouse clickhouse-client \
  --query "SELECT formatReadableSize(sum(bytes)) AS size FROM system.parts WHERE table = 'telemetry_events'"
```

**Solutions**:

#### Solution 1: Large Dataset
```bash
# Use time-based filtering
# BAD (slow): SELECT * FROM telemetry_events
# GOOD (fast): SELECT * FROM telemetry_events WHERE timestamp >= now() - INTERVAL 7 DAY

# Use LIMIT
SELECT * FROM telemetry_events LIMIT 1000

# Use materialized views
SELECT * FROM session_summaries  # Pre-aggregated
```

#### Solution 2: Fragmented Table
```bash
# Optimize table (defragmentation)
docker exec telemetry-clickhouse clickhouse-client \
  --query "OPTIMIZE TABLE telemetry.telemetry_events FINAL"

# This can take several minutes for large tables
```

#### Solution 3: Insufficient Resources
```bash
# Increase memory limit in docker-compose.yml
# Change from 2G to 4G

# Restart
docker-compose down
docker-compose up -d
```

### Issue: High Memory Usage

**Symptoms**:
- Docker shows ClickHouse using > 2GB RAM
- System becomes sluggish

**Diagnosis**:
```bash
# Check memory usage
docker stats telemetry-clickhouse

# Check ClickHouse internals
docker exec telemetry-clickhouse clickhouse-client \
  --query "SELECT * FROM system.metrics WHERE metric LIKE '%Memory%'"
```

**Solutions**:

#### Solution 1: Reduce Resource Limits
```bash
# Edit docker-compose.yml
# Reduce memory limit from 2G to 1G

# Reduce max_server_memory_usage in clickhouse/config.xml
<max_server_memory_usage_to_ram_ratio>0.5</max_server_memory_usage_to_ram_ratio>
```

#### Solution 2: Clear Query Cache
```bash
docker exec telemetry-clickhouse clickhouse-client \
  --query "SYSTEM DROP QUERY CACHE"
```

---

## Docker Issues

### Issue: Docker Daemon Not Running

**Error**: "Cannot connect to the Docker daemon"

**Solution**:
```bash
# macOS
open -a Docker

# Linux (systemd)
sudo systemctl start docker

# Linux (service)
sudo service docker start

# Verify
docker info
```

### Issue: Docker Compose Version Mismatch

**Error**: "unsupported Compose file version"

**Solution**:
```bash
# Check version
docker compose version

# If < 2.0, upgrade Docker Desktop or install standalone
# https://docs.docker.com/compose/install/

# Temporarily downgrade compose file version
# Change "version: '3.8'" to "version: '3.3'" in docker-compose.yml
```

### Issue: Permission Denied

**Error**: "permission denied while trying to connect"

**Solution**:
```bash
# Linux: Add user to docker group
sudo usermod -aG docker $USER

# Log out and log back in, or run:
newgrp docker

# Verify
docker ps
```

---

## Query Issues

### Issue: Syntax Error in Query

**Error**: "Syntax error" or "Unknown identifier"

**Diagnosis**:
```bash
# Check ClickHouse version (syntax varies by version)
docker exec telemetry-clickhouse clickhouse-client --version

# Test query with EXPLAIN
docker exec telemetry-clickhouse clickhouse-client \
  --query "EXPLAIN SELECT * FROM telemetry_events LIMIT 1"
```

**Solutions**:

#### Solution 1: Wrong SQL Dialect
ClickHouse SQL is different from PostgreSQL/MySQL:

```sql
-- BAD (PostgreSQL syntax)
SELECT * FROM telemetry_events OFFSET 100 LIMIT 10;

-- GOOD (ClickHouse syntax)
SELECT * FROM telemetry_events LIMIT 10 OFFSET 100;
```

#### Solution 2: Table Name Not Fully Qualified
```sql
-- BAD
SELECT * FROM telemetry_events;

-- GOOD
SELECT * FROM telemetry.telemetry_events;
```

### Issue: Empty Result Set

**Symptoms**:
- Query runs successfully but returns 0 rows
- Expected data but got nothing

**Diagnosis**:
```bash
# Check if table has data
docker exec telemetry-clickhouse clickhouse-client \
  --query "SELECT count() FROM telemetry.telemetry_events"

# Check time range
docker exec telemetry-clickhouse clickhouse-client \
  --query "SELECT min(timestamp), max(timestamp) FROM telemetry.telemetry_events"

# Check your filter conditions
```

**Solutions**:

#### Solution 1: Time Filter Too Restrictive
```sql
-- If no recent data, expand time range
SELECT * FROM telemetry_events 
WHERE timestamp >= now() - INTERVAL 30 DAY  -- Instead of 1 DAY
```

#### Solution 2: NULL Values in Filter
```sql
-- Check for NULLs
SELECT count() FROM telemetry_events WHERE feature_name IS NULL;

-- Include NULLs in filter
SELECT * FROM telemetry_events 
WHERE feature_name IS NULL OR feature_name = 'your_feature';
```

---

## Security Issues

### Issue: Exposed Credentials in Logs

**Risk**: Passwords visible in logs or command history

**Mitigation**:
```bash
# Clear bash history
history -c

# Use password file instead of inline
echo "your_password" > ~/.clickhouse_password
chmod 600 ~/.clickhouse_password

# Connect using password file
clickhouse-client --password "$(cat ~/.clickhouse_password)"

# Or use environment variable
export CLICKHOUSE_PASSWORD="your_password"
clickhouse-client --password "$CLICKHOUSE_PASSWORD"
```

### Issue: Default Passwords Not Changed

**Risk**: Using "change_me_in_production" in production

**Solution**:
```bash
# Generate strong password
openssl rand -base64 32

# Update .env
nano .env  # Change CLICKHOUSE_PASSWORD

# Restart services
docker-compose down
docker-compose up -d
```

---

## Advanced Debugging

### Enable Debug Logging

#### ClickHouse
```bash
# Edit clickhouse/config.xml
# Change <level>information</level> to <level>debug</level>

# Restart
docker-compose restart clickhouse

# View debug logs
docker-compose logs -f clickhouse | grep -i debug
```

#### OTLP Collector
```bash
# Edit otel-collector/config.yaml
# Change level: info to level: debug

# Restart
docker-compose restart otel-collector

# View debug logs
docker-compose logs -f otel-collector | grep -i debug
```

### Inspect Docker Network

```bash
# List networks
docker network ls

# Inspect telemetry network
docker network inspect telemetry-network

# Check connectivity between containers
docker exec telemetry-otel-collector ping clickhouse
```

### Check File Permissions

```bash
# List files with permissions
ls -la clickhouse_data/

# Fix ownership (Linux)
sudo chown -R $USER:$USER clickhouse_data/

# Fix permissions
chmod -R 755 clickhouse_data/
```

### Monitor System Resources

```bash
# Real-time resource monitoring
docker stats

# Disk I/O
iostat -x 1

# Network traffic
iftop -i docker0
```

### Export Logs for Support

```bash
# Export all logs
docker-compose logs > telemetry-logs-full.txt

# Export with timestamps
docker-compose logs --timestamps > telemetry-logs-timestamped.txt

# Export last 1000 lines
docker-compose logs --tail=1000 > telemetry-logs-recent.txt
```

---

## Getting Help

### Before Asking for Help

Collect this information:

1. **System Information**:
   ```bash
   uname -a  # OS version
   docker --version  # Docker version
   docker compose version  # Compose version
   ```

2. **Service Status**:
   ```bash
   docker-compose ps
   ```

3. **Recent Logs**:
   ```bash
   docker-compose logs --tail=100 > logs.txt
   ```

4. **Configuration** (sanitized):
   ```bash
   # Remove passwords before sharing
   cat .env | sed 's/PASSWORD=.*/PASSWORD=***REDACTED***/'
   ```

### Contact Team

- **Spec**: `../specs/017-usage-data-collection/spec.md`
- **GitHub Issues**: (if applicable)
- **Team Chat**: Contact book development team

---

## Common Error Messages

### "Code: 60. DB::Exception: Table telemetry.telemetry_events doesn't exist"

**Cause**: Schema not initialized

**Solution**:
```bash
docker-compose down -v
docker-compose up -d
```

### "OTLP receiver failed to start"

**Cause**: Port 4317 in use

**Solution**:
```bash
lsof -i :4317
kill -9 <PID>
docker-compose restart otel-collector
```

### "Memory limit exceeded"

**Cause**: Query too large or insufficient memory

**Solution**:
```bash
# Add LIMIT to query
SELECT * FROM telemetry_events LIMIT 1000;

# Or increase memory in docker-compose.yml
```

---

**Still having issues?** See `docs/quickstart.md` for setup verification or contact the team.
