# Quick Start Guide: Telemetry Collection Setup (Feature 017)

**Target Time**: 15 minutes | **Audience**: Book development team members | **Date**: 2025-11-10

## Overview

This guide enables you to collect Claude Code usage data locally. After setup, all your Claude Code interactions (prompts, tool calls, API requests, errors) are automatically exported to a local ClickHouse database where you can run analysis queries.

**What You Get**:
- Automatic telemetry collection during all Claude Code sessions
- Local database (no cloud dependencies, data stays private)
- Query access for cost analysis, error patterns, and workflow insights
- Example queries ready to copy/paste

**Time Breakdown**:
- Prerequisites check: 2 minutes
- Docker setup: 5 minutes
- Claude Code configuration: 4 minutes
- Verification: 2 minutes
- First query: 2 minutes

**Total: ~15 minutes**

---

## Step 0: Prerequisites (2 minutes)

Verify you have:

### Docker Desktop/Engine Installed
```bash
docker --version
# Expected: Docker version 20.10+ (any recent version)

docker-compose --version
# Expected: Docker Compose version 2.0+
```

If missing, install:
- **macOS**: [Docker Desktop](https://www.docker.com/products/docker-desktop)
- **Linux**: `sudo apt install docker.io docker-compose`
- **Windows**: [Docker Desktop for WSL2](https://www.docker.com/products/docker-desktop)

### System Resources
- RAM available: At least 2GB free (4GB recommended)
- Disk space: 10GB free for telemetry data growth
- Network: Not required (telemetry runs locally)

### Git Repository Access
```bash
cd /path/to/ai-native-software-development  # Your repo directory
git status  # Verify you're in the repo root
```

---

## Step 1: Start Telemetry Infrastructure (5 minutes)

### 1a. Navigate to telemetry-server directory

```bash
cd /path/to/ai-native-software-development
ls telemetry-server/  # Verify directory exists

# You should see:
# docker-compose.yml
# .env.template
# collector-config.yaml
# clickhouse/
# queries/
```

### 1b. Initialize environment configuration

```bash
cd telemetry-server

# Copy template to actual config
cp .env.template .env

# Verify .env was created
ls -la .env
```

**Note**: `.env` is excluded from git (.gitignore). Your configuration stays private.

### 1c. Start Docker containers

```bash
# Start OTLP collector and ClickHouse
docker-compose up -d

# Watch startup logs (optional, press Ctrl+C to exit)
docker-compose logs -f

# Verify containers are running
docker-compose ps

# Expected output:
# NAME                 STATUS
# otel-collector       Up 2 seconds
# clickhouse           Up 3 seconds
```

**Troubleshooting**: If containers fail to start, see [Troubleshooting](#troubleshooting) section below.

### 1d. Verify ClickHouse database is ready

```bash
# Wait 10 seconds for ClickHouse to initialize
sleep 10

# Test connection
curl -s 'http://localhost:8123/?query=SELECT%201' 

# Expected output: 1
```

If you see "Connection refused", wait another 10 seconds and retry.

---

## Step 2: Configure Claude Code Telemetry (4 minutes)

### 2a. Locate Claude Code environment setup

```bash
# Go back to repo root
cd /path/to/ai-native-software-development

# Check Claude environment templates
ls -la .claude/env-config/

# You should see:
# telemetry-enabled.env
# telemetry-disabled.env
```

### 2b. Copy telemetry environment template to your shell

Choose ONE of these options:

**Option A: Add to your shell profile (.zshrc, .bashrc, etc.)**

```bash
# Identify your shell
echo $SHELL
# Output: /bin/zsh or /bin/bash

# Open your shell config file
# For zsh: nano ~/.zshrc
# For bash: nano ~/.bashrc

# Add these lines at the end:
export CLAUDE_CODE_ENABLE_TELEMETRY=true
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317
export OTEL_EXPORTER_OTLP_PROTOCOL=grpc
export OTEL_SESSION_ID="$(uuidgen)"  # Generate unique session ID
export OTEL_USER_ID="$(git config user.email)"  # Your email from git config
export OTEL_ORGANIZATION_ID="panaversity-book-project"

# Save and exit (Ctrl+X, then Y, then Enter for nano)

# Reload shell config
source ~/.zshrc  # or source ~/.bashrc
```

**Option B: Set variables for this session only (temporary)**

```bash
export CLAUDE_CODE_ENABLE_TELEMETRY=true
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317
export OTEL_EXPORTER_OTLP_PROTOCOL=grpc
export OTEL_SESSION_ID="$(uuidgen)"
export OTEL_USER_ID="$(git config user.email)"
export OTEL_ORGANIZATION_ID="panaversity-book-project"

# Note: These settings disappear when you close the terminal
```

### 2c. Verify environment variables are set

```bash
# Check each variable
echo "Telemetry enabled: $CLAUDE_CODE_ENABLE_TELEMETRY"
echo "OTLP endpoint: $OTEL_EXPORTER_OTLP_ENDPOINT"
echo "Session ID: $OTEL_SESSION_ID"
echo "User ID: $OTEL_USER_ID"

# All should show values (not empty)
```

---

## Step 3: Verify Telemetry Collection (2 minutes)

### 3a. Generate sample telemetry data

```bash
# Run a simple Claude Code command to generate telemetry
cd /path/to/ai-native-software-development

# Example: Run a simple prompt (replace with your own)
# This will generate: 1 user_prompt event, 1 api_request event, possibly 1 tool_call event

# Make sure your telemetry environment is loaded, then:
# (Use your actual Claude Code CLI command here)
```

Alternatively, **skip this step** and telemetry will be collected automatically on your next Claude Code session.

### 3b. Query the database to verify data was collected

```bash
# Connect to ClickHouse and check for events
curl -s 'http://localhost:8123/?query=SELECT%20COUNT%28%29%20FROM%20telemetry_events'

# Expected output: 1 (or higher if you've run multiple prompts)
# If output is 0, data collection just hasn't started yet (that's OK)
```

### 3c. View raw events (optional)

```bash
# See the most recent events
curl -s 'http://localhost:8123/?query=SELECT%20event_type,%20session_id,%20user_id,%20timestamp%20FROM%20telemetry_events%20ORDER%20BY%20timestamp%20DESC%20LIMIT%2010'

# Expected output: CSV format with columns: event_type, session_id, user_id, timestamp
```

---

## Step 4: Run Your First Analysis Query (2 minutes)

Now that telemetry is running, you can query aggregated data.

### 4a. Cost analysis (how much have you spent?)

```bash
# Total cost by date
curl -s 'http://localhost:8123/?query=SELECT%20toDate%28timestamp%29%20as%20date,%20SUM%28cost_usd%29%20as%20daily_cost%20FROM%20telemetry_events%20WHERE%20event_type%20=%20%27api_request%27%20GROUP%20BY%20date%20ORDER%20BY%20date%20DESC'

# Expected output: dates and costs
```

### 4b. Error patterns (what went wrong?)

```bash
# Most common errors
curl -s 'http://localhost:8123/?query=SELECT%20error_type,%20COUNT%28*%29%20as%20count%20FROM%20telemetry_events%20WHERE%20event_type%20=%20%27api_error%27%20GROUP%20BY%20error_type%20ORDER%20BY%20count%20DESC'

# Expected output: error types and counts
```

### 4c. Token usage (which sessions used the most tokens?)

```bash
# Token usage by session
curl -s 'http://localhost:8123/?query=SELECT%20session_id,%20SUM%28tokens_input%29%20as%20input_tokens,%20SUM%28tokens_output%29%20as%20output_tokens%20FROM%20telemetry_events%20WHERE%20event_type%20=%20%27api_request%27%20GROUP%20BY%20session_id%20ORDER%20BY%20input_tokens%20DESC%20LIMIT%2010'

# Expected output: top 10 sessions by token usage
```

---

## Example Queries for Common Analysis

All queries use `curl` (available on Mac/Linux/WSL) to talk to ClickHouse HTTP API.

### Query Template

```bash
# Basic template (substitute YOUR_QUERY)
curl -s 'http://localhost:8123/?query=SELECT%20*%20FROM%20table'

# Pro tip: For complex queries with special characters, 
# use a query file instead:
cat > query.sql << 'EOF'
SELECT event_type, COUNT(*) as count
FROM telemetry_events
GROUP BY event_type
ORDER BY count DESC
EOF

# Execute query file
cat query.sql | curl -s -X POST --data-binary @- 'http://localhost:8123/'
```

### Query 1: Total Cost by User

```bash
curl -s 'http://localhost:8123/?query=SELECT%20user_id,%20SUM%28cost_usd%29%20as%20total_cost%20FROM%20telemetry_events%20WHERE%20event_type%20=%20%27api_request%27%20GROUP%20BY%20user_id%20ORDER%20BY%20total_cost%20DESC'
```

### Query 2: Cost by Chapter

```bash
curl -s 'http://localhost:8123/?query=SELECT%20chapter_number,%20COUNT%28DISTINCT%20session_id%29%20as%20sessions,%20SUM%28cost_usd%29%20as%20total_cost%20FROM%20telemetry_events%20WHERE%20event_type%20=%20%27api_request%27%20AND%20chapter_number%20IS%20NOT%20NULL%20GROUP%20BY%20chapter_number%20ORDER%20BY%20total_cost%20DESC'
```

### Query 3: Error Rate by Feature

```bash
curl -s 'http://localhost:8123/?query=SELECT%20feature_id,%20COUNT%28*%29%20as%20total_events,%20SUM%28IF%28event_type%20=%20%27api_error%27,%201,0%29%29%20as%20error_count,%20ROUND%28SUM%28IF%28event_type%20=%20%27api_error%27,%201,0%29%29%20/%20COUNT%28*%29%20*%20100,%202%29%20as%20error_rate_percent%20FROM%20telemetry_events%20GROUP%20BY%20feature_id%20ORDER%20BY%20error_count%20DESC'
```

### Query 4: Top 10 Most Common Errors (Last 7 Days)

```bash
curl -s 'http://localhost:8123/?query=SELECT%20error_type,%20COUNT%28*%29%20as%20occurrences%20FROM%20telemetry_events%20WHERE%20event_type%20=%20%27api_error%27%20AND%20timestamp%20>%20now%28%29%20-%20INTERVAL%207%20DAY%20GROUP%20BY%20error_type%20ORDER%20BY%20occurrences%20DESC%20LIMIT%2010'
```

### Query 5: Session Workflow Trace (Debug a Specific Session)

```bash
# Replace SESSION_ID with actual UUID from your events
SESSION_ID="550e8400-e29b-41d4-a716-446655440000"

curl -s "http://localhost:8123/?query=SELECT%20timestamp,%20event_type,%20CASE%20WHEN%20event_type=%27api_request%27%20THEN%20CONCAT%28%27tokens:%27,%20tokens_input,%27→%27,%20tokens_output%29%20WHEN%20event_type=%27api_error%27%20THEN%20error_type%20WHEN%20event_type=%27tool_call%27%20THEN%20tool_name%20ELSE%20%27%27%20END%20as%20detail%20FROM%20telemetry_events%20WHERE%20session_id%20=%20%27${SESSION_ID}%27%20ORDER%20BY%20timestamp"
```

---

## Verification Checklist

- [ ] Docker containers running: `docker-compose ps` shows 2 healthy containers
- [ ] ClickHouse database responding: `curl -s 'http://localhost:8123/?query=SELECT%201'` returns `1`
- [ ] Environment variables set: `echo $CLAUDE_CODE_ENABLE_TELEMETRY` returns `true`
- [ ] First query successful: At least one query above returns results
- [ ] Data growing: Run same query twice with interval; row count increases

If all above are true, **you're ready to go!** Telemetry will now automatically collect all Claude Code activity.

---

## Troubleshooting

### Problem: `docker-compose: command not found`

**Solution**: Docker Compose not installed or not in PATH
```bash
# Check Docker Compose version
docker compose version  # (note: no hyphen in newer Docker versions)

# If that works, use:
docker compose up -d   # Instead of docker-compose up -d

# Or install Docker Compose v1 separately (deprecated but works):
sudo pip install docker-compose
```

### Problem: Containers fail to start (status `Exited` or `Restarting`)

**Solution**: Check logs and resource availability
```bash
# View error logs
docker-compose logs

# Common causes:
# 1. Port 8123 or 4317 already in use
lsof -i :8123  # See what's using port 8123
lsof -i :4317  # See what's using port 4317

# 2. Insufficient disk space
df -h  # Check available disk

# 3. Insufficient RAM
docker stats  # Check memory usage

# Fix: Stop and remove everything, then restart
docker-compose down -v  # -v removes volumes (deletes data)
docker-compose up -d
```

### Problem: `curl: (7) Failed to connect to localhost port 8123`

**Solution**: ClickHouse not ready yet or container crashed
```bash
# Wait longer (ClickHouse takes time to start)
sleep 30
curl -s 'http://localhost:8123/?query=SELECT%201'

# Or check if container is running
docker-compose ps
# If ClickHouse shows "Exited", restart it:
docker-compose restart clickhouse
```

### Problem: `CLAUSE_CODE_ENABLE_TELEMETRY not found` error in Claude Code

**Solution**: Environment variables not loaded
```bash
# Verify variables are set
echo $CLAUDE_CODE_ENABLE_TELEMETRY
# If empty, you haven't set them yet

# Set them for this terminal session:
export CLAUDE_CODE_ENABLE_TELEMETRY=true
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317
export OTEL_EXPORTER_OTLP_PROTOCOL=grpc

# Then run Claude Code in same terminal
```

### Problem: No data appearing in database after running Claude Code

**Solution**: Telemetry export might be disabled or misconfigured
```bash
# Check if Claude Code is reporting telemetry
# Look for log output from Claude Code that mentions "OTLP" or "telemetry"

# Verify ClickHouse is receiving data from collector
# Check collector logs:
docker-compose logs otel-collector | tail -20

# If no "Export" messages, telemetry export isn't working
# Verify OTLP endpoint is correct:
echo $OTEL_EXPORTER_OTLP_ENDPOINT
# Should be: http://localhost:4317
```

### Problem: ClickHouse port 8123 already in use

**Solution**: Another service is using that port
```bash
# Find what's using port 8123
lsof -i :8123

# If it's ClickHouse from a previous run:
docker-compose down
docker-compose up -d

# If it's something else, either:
# 1. Stop that service, or
# 2. Change ClickHouse port in docker-compose.yml (line ~XX):
#    Change "8123:8123" to "8124:8123"
```

### Problem: Data queries return no results

**Solution**: Data hasn't been collected yet or table is empty
```bash
# Check if any data exists
curl -s 'http://localhost:8123/?query=SELECT%20COUNT%28*%29%20FROM%20telemetry_events'
# Returns: 0

# This is OK! It means:
# 1. You haven't run Claude Code with telemetry enabled yet, OR
# 2. Data is still in flight from collector to database

# Try running a Claude Code command to generate data:
# Then wait 10 seconds for export

# Check table exists
curl -s 'http://localhost:8123/?query=SHOW%20TABLES'
# Should include: telemetry_events
```

---

## Next Steps

Now that telemetry is running:

### For Daily Use
- Telemetry now auto-collects all Claude Code activity
- No action needed; it "just works"
- To disable: `unset CLAUDE_CODE_ENABLE_TELEMETRY` or set to `false`

### For Analysis
- Run example queries above whenever you want insights
- Queries are documented in `telemetry-server/queries/README.md`
- Create your own queries (SQL dialect is ClickHouse SQL)

### For Team Coordination
- Share your daily cost reports with project leads
- Identify high-error patterns for improvement
- Use workflow traces to debug AI output quality

### If You Want More Details
- See `data-model.md` for schema documentation
- See `ARCHITECTURE.md` for data flow diagrams
- See `queries/README.md` for more example queries
- See `TROUBLESHOOTING.md` for deeper diagnostics

---

## FAQ

**Q: Is my data secure? Will it leave my computer?**  
A: Yes. All data stays local in your telemetry-server directory. No cloud uploads. Data is private to this project.

**Q: Can I delete collected data?**  
A: Yes. Raw events auto-delete after 90 days. To manually delete:
```bash
docker-compose exec clickhouse clickhouse-client -q "DELETE FROM telemetry_events WHERE 1"
```

**Q: What if I want to stop collecting telemetry?**  
A: Unset the environment variable:
```bash
unset CLAUDE_CODE_ENABLE_TELEMETRY
```
Or set to false:
```bash
export CLAUDE_CODE_ENABLE_TELEMETRY=false
```

**Q: Does telemetry slow down Claude Code?**  
A: No. Telemetry export happens asynchronously (buffered). Claude Code continues without waiting for export.

**Q: Can I share query results with teammates?**  
A: Yes. Query results are CSV (copy-paste friendly). Share the CSV or the query itself. See `telemetry-server/queries/README.md` for shareable query templates.

**Q: How much disk space will telemetry use?**  
A: ~1-5 MB/week for a single developer (excellent compression). Budget 100 MB/year per developer. Adjust retention policy in `.env` if needed.

---

## Support

If you encounter issues not covered above:

1. Check the [Troubleshooting](#troubleshooting) section
2. Review `ARCHITECTURE.md` for system design details
3. Check `TROUBLESHOOTING.md` for detailed diagnostics
4. Ask team lead or open an issue in the repo

---

**You're all set!** Your telemetry infrastructure is running. Start using Claude Code normally, and data will be collected automatically.
