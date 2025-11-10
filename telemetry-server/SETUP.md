# Local Telemetry Setup - Quick Guide

**Time Required**: 10-15 minutes

## Prerequisites

You need:
- **Docker Desktop** installed and running ([download here](https://www.docker.com/products/docker-desktop))
- **10GB free disk space**
- **4GB available RAM**

### Check Docker Installation

```bash
# Check Docker is installed
docker --version
# Expected: Docker version 20.10.0 or higher

# Check Docker is running
docker ps
# Should show empty list (not an error)
```

If Docker commands don't work, start Docker Desktop application first.

---

## Setup Steps

### 1. Navigate to Telemetry Directory

```bash
cd telemetry-server
```

### 2. Run Pre-Flight Checks

```bash
./scripts/verify-setup.sh
```

**Expected**: All checks pass with ✓ green checkmarks.

**If you see errors**: Most common issue is Docker not running. Start Docker Desktop and try again.

### 3. Create Environment File

```bash
# Copy the template
cp .env.template .env

# Edit the file
nano .env   # or use: code .env, vim .env, etc.
```

**IMPORTANT - Change these two lines**:

```bash
# BEFORE (line 7):
CLICKHOUSE_PASSWORD=change_me_in_production

# AFTER:
CLICKHOUSE_PASSWORD=MySecurePassword123!

# BEFORE (line 11):
TELEMETRY_READER_PASSWORD=readonly_password

# AFTER:
TELEMETRY_READER_PASSWORD=AnotherSecurePass456!
```

Save and exit:
- **nano**: Press `Ctrl+X`, then `Y`, then `Enter`
- **vim**: Press `Esc`, type `:wq`, press `Enter`
- **VS Code**: Just save the file normally

### 4. Start Services

```bash
# Start ClickHouse and OTLP Collector
docker-compose up -d
```

**Expected output**:
```
[+] Running 3/3
 ✔ Network telemetry-network      Created
 ✔ Container telemetry-clickhouse Started
 ✔ Container telemetry-otel-collector Started
```

**Wait 30 seconds** for services to initialize.

### 5. Verify Everything Works

```bash
./scripts/test-connection.sh
```

**Expected**: All checks pass with ✓ green checkmarks.

**If you see "Connection refused"**: Wait another 30 seconds and try again. Services take time to start.

---

## Enable Telemetry in Claude Code

### Option A: One-Time Use

```bash
# Start Claude Code with telemetry enabled
claude-code --env ./.claude/env-config/telemetry-enabled.env
```

### Option B: Always Enabled (Recommended)

Add this line to your shell config file:

**For zsh** (macOS default):
```bash
echo 'export CLAUDE_ENV_CONFIG="$HOME/Documents/code/panaversity-official/tutorgpt-build/ai-native-software-development/.claude/env-config/telemetry-enabled.env"' >> ~/.zshrc
source ~/.zshrc
```

**For bash** (Linux default):
```bash
echo 'export CLAUDE_ENV_CONFIG="$HOME/path/to/ai-native-software-development/.claude/env-config/telemetry-enabled.env"' >> ~/.bashrc
source ~/.bashrc
```

**Note**: Replace the path with your actual repository location.

---

## Verify Data Collection

### 1. Use Claude Code Normally

Just work as you normally would. Telemetry collects automatically in the background.

### 2. Check Data is Being Collected

```bash
# Connect to database
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user \
  --password YOUR_PASSWORD_HERE

# Count events (should increase as you use Claude Code)
SELECT count() FROM telemetry.telemetry_events;

# View recent events
SELECT 
    timestamp,
    event_type,
    tool_name,
    workflow_step
FROM telemetry.telemetry_events
ORDER BY timestamp DESC
LIMIT 10;

# Exit
exit
```

---

## Daily Usage

### Start Services (if stopped)
```bash
cd telemetry-server
docker-compose start
```

### Stop Services (optional, saves resources)
```bash
cd telemetry-server
docker-compose stop
```

### View Live Logs
```bash
cd telemetry-server
docker-compose logs -f
```

Press `Ctrl+C` to stop viewing logs.

### Check Service Status
```bash
cd telemetry-server
docker-compose ps
```

Both services should show status "Up".

---

## Running Analysis Queries

### Example: View Your Recent Sessions

```bash
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user \
  --password YOUR_PASSWORD_HERE \
  --database telemetry \
  --multiquery < queries/session-analysis.sql
```

### Example: Analyze Errors (Andrew Ng's Methodology)

```bash
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user \
  --password YOUR_PASSWORD_HERE \
  --database telemetry \
  --multiquery < queries/error-analysis.sql
```

### Example: Check Tool Usage

```bash
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user \
  --password YOUR_PASSWORD_HERE \
  --database telemetry \
  --multiquery < queries/tool-usage.sql
```

---

## Troubleshooting

### Services Won't Start

```bash
# Check what's wrong
docker-compose logs

# Most common: port conflict
# Check if something is using port 8123 or 4317
lsof -i :8123
lsof -i :4317

# Kill the conflicting process or restart Docker Desktop
```

### No Data Appearing

```bash
# Check environment variable is set
echo $CLAUDE_ENV_CONFIG

# If empty, set it manually
export CLAUDE_ENV_CONFIG="/full/path/to/.claude/env-config/telemetry-enabled.env"

# Restart Claude Code
```

### Authentication Failed

```bash
# Make sure you're using the password from .env file
cat .env | grep CLICKHOUSE_PASSWORD

# Use that exact password in your commands
```

### Reset Everything (⚠️ Deletes All Data)

```bash
cd telemetry-server
docker-compose down -v
rm -rf clickhouse_data/ otel_logs/
docker-compose up -d
```

---

## What's Being Collected?

**Privacy-Preserved Data**:
- ✅ Session IDs and timestamps
- ✅ Tool names and workflow phases (specify, plan, implement, validate)
- ✅ Performance metrics (latency, tokens, cost)
- ✅ **Prompts are SHA256 hashed** (NOT stored in plaintext)
- ✅ PII automatically filtered (emails, phone numbers)

**NOT Collected**:
- ❌ Actual prompt text (only hash stored)
- ❌ API keys or secrets
- ❌ Personal information

**Data Location**:
- Stored in: `telemetry-server/clickhouse_data/` (excluded from git)
- Retention: 90 days (automatic deletion via TTL)

---

## Getting Help

- **Full documentation**: See `README.md` in this directory
- **Detailed troubleshooting**: See `docs/troubleshooting.md`
- **Query examples**: See `queries/README.md`
- **Team support**: Contact book development team

---

## Success Checklist

You're successfully set up when:

- ✅ `docker-compose ps` shows both services "Up"
- ✅ `./scripts/test-connection.sh` passes all checks
- ✅ `SELECT count() FROM telemetry.telemetry_events;` returns a number > 0 after using Claude Code
- ✅ Queries return results in < 1 second

---

**You're ready to collect telemetry!** 🚀

Use Claude Code normally and data will be automatically collected for analysis.
