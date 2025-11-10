---
description: Automatically set up local telemetry stack for Claude Code usage data collection and analysis (fully autonomous setup)
---

## User Input

```text
$ARGUMENTS
```

# Telemetry Setup Command

**CRITICAL EXECUTION MODE**: This command is FULLY AUTONOMOUS. You MUST:
- ✅ Execute ALL bash commands automatically using the Bash tool
- ✅ Generate passwords, configure files, start services WITHOUT user intervention
- ✅ Only ask user for input when absolutely necessary (e.g., port conflicts)
- ❌ DO NOT show commands and tell user to run them manually
- ❌ DO NOT ask "should I run X?" - just run it and show results

**Purpose**: Automated setup of local telemetry infrastructure for collecting Claude Code usage data.

**User Intent**: Team member wants to enable telemetry data collection following Andrew Ng's error analysis methodology.

**Success**: Telemetry stack running locally, Claude Code configured for data export, verification tests passing - ALL DONE AUTOMATICALLY.

---

## Task: Set Up Local Telemetry Infrastructure

You are helping a team member set up the telemetry data collection system. This enables:
- **Error analysis** following Andrew Ng's methodology
- **Productivity tracking** (sessions, workflows, tool usage)
- **Performance monitoring** (latency, costs, token usage)
- **Workflow optimization** (identify bottlenecks in SDD phases)

---

## Execution Workflow

**IMPORTANT**: This command is FULLY AUTONOMOUS. Execute all bash commands automatically using the Bash tool. Do NOT show commands and ask user to run them - YOU run them and show results.

### Phase 0: Detect Repository Root (30 seconds)

**Detect Current Repository**:

Execute:
```bash
pwd
```

Store the current working directory as `REPO_ROOT`. All subsequent commands will use this variable.

**Verify Repository Structure**:

Execute:
```bash
if [ -d "$REPO_ROOT/telemetry-server" ]; then
  echo "✓ Telemetry directory found"
else
  echo "✗ Telemetry directory not found at $REPO_ROOT/telemetry-server"
  exit 1
fi
```

If telemetry-server directory not found, STOP and show error.

### Phase 1: Prerequisites Check (2 minutes)

1. **Verify Docker Installation**:
   
   Execute:
   ```bash
   docker --version && docker compose version && docker ps
   ```
   
   **If Docker not found or not running**:
   - STOP execution
   - Show error: "❌ Docker is not installed or not running"
   - Provide instructions: "Please install Docker Desktop from https://docs.docker.com/get-docker/ and start it, then run /sp.telemetry-setup again"
   - EXIT

2. **Check System Resources**:
   
   Execute (using detected $REPO_ROOT):
   ```bash
   cd "$REPO_ROOT"
   df -h . | tail -1
   ```
   
   Parse output and verify:
   - 10GB+ free disk space
   
   If insufficient: WARN user but continue

3. **Navigate to Telemetry Directory**:
   
   Execute (using detected $REPO_ROOT):
   ```bash
   cd "$REPO_ROOT/telemetry-server" && pwd
   ```

### Phase 2: Pre-Flight Verification (2 minutes)

4. **Run Automated Checks**:
   
   Execute (using $REPO_ROOT):
   ```bash
   cd "$REPO_ROOT/telemetry-server"
   ./scripts/verify-setup.sh
   ```
   
   Parse output and check for failures.
   
   **If port conflicts detected**:
   - Execute: `lsof -i :4317 && lsof -i :4318 && lsof -i :8123 && lsof -i :9000`
   - Show conflicting processes
   - Ask user: "Ports are in use. Kill conflicting processes? (yes/no)"
   - If yes: Kill processes automatically
   - If no: STOP and instruct user to resolve manually

### Phase 3: Environment Configuration (3 minutes)

5. **Create Environment File**:
   
   Execute (using $REPO_ROOT):
   ```bash
   cd "$REPO_ROOT/telemetry-server"
   cp .env.template .env
   ```

6. **Generate Secure Passwords**:
   
   Execute:
   ```bash
   openssl rand -base64 24
   openssl rand -base64 24
   ```
   
   Store both passwords in variables.

7. **Edit Configuration Automatically**:
   
   Execute (using $REPO_ROOT):
   ```bash
   cd "$REPO_ROOT/telemetry-server"
   
   # Replace passwords with generated ones
   sed -i.bak "s/CLICKHOUSE_PASSWORD=change_me_in_production/CLICKHOUSE_PASSWORD=<PASSWORD_1>/" .env
   sed -i.bak "s/TELEMETRY_READER_PASSWORD=readonly_password/TELEMETRY_READER_PASSWORD=<PASSWORD_2>/" .env
   
   # Set user ID
   sed -i.bak "s/USER_ID=\${USER}/USER_ID=$(whoami)/" .env
   
   # Remove backup
   rm -f .env.bak
   ```
   
   Replace `<PASSWORD_1>` and `<PASSWORD_2>` with the actual generated passwords.
   
   Show user: "✓ Generated secure passwords and configured .env"

### Phase 4: Service Startup (3 minutes)

8. **Start Docker Services**:
   
   Execute (using $REPO_ROOT):
   ```bash
   cd "$REPO_ROOT/telemetry-server"
   docker-compose up -d
   ```
   
   Check output contains "Started" for both containers.
   
   Show user: "✓ Starting ClickHouse and OTLP Collector..."

9. **Wait for Initialization**:
   
   Execute:
   ```bash
   sleep 30
   ```
   
   Show user: "⏳ Waiting 30 seconds for services to initialize..."

10. **Verify Service Status**:
    
    Execute (using $REPO_ROOT):
    ```bash
    cd "$REPO_ROOT/telemetry-server"
    docker-compose ps
    ```
    
    Parse output and verify both services show "Up" status.
    
    **If "Restarting" or "Exited"**:
    - Execute: `docker-compose logs clickhouse && docker-compose logs otel-collector`
    - Show error logs
    - Offer to restart: "Services failed to start. Try restarting? (yes/no)"
    - If yes: `docker-compose restart && sleep 30`
    - If still failing: STOP and show troubleshooting guide

### Phase 5: Connection Testing (2 minutes)

11. **Run Connection Tests**:
    
    Execute (using $REPO_ROOT):
    ```bash
    cd "$REPO_ROOT/telemetry-server"
    ./scripts/test-connection.sh
    ```
    
    Parse output and verify all checks pass.
    
    Show user: "✓ Running 9 connection tests..."
    
    **If any test fails**:
    - Show which test failed
    - Execute diagnostics based on failure type
    - Offer to retry or show troubleshooting steps

12. **Verify Database Schema**:
    
    Execute (use the actual generated password from step 6):
    ```bash
    docker exec telemetry-clickhouse clickhouse-client \
      --user telemetry_user \
      --password <ACTUAL_GENERATED_PASSWORD> \
      --query "SHOW TABLES FROM telemetry"
    ```
    
    Verify output contains all 4 tables:
    - error_patterns
    - session_summaries  
    - telemetry_events
    - tool_usage_patterns
    
    Show user: "✓ Database schema verified (4 tables created)"

### Phase 6: Claude Code Integration (3 minutes)

13. **Detect Shell Type**:
    
    Execute:
    ```bash
    echo $SHELL
    ```
    
    Determine if zsh or bash.

14. **Enable Telemetry Permanently**:
    
    Execute (based on shell type):
    
    **For zsh**:
    ```bash
    # Check if already configured
    if ! grep -q "CLAUDE_CODE_ENABLE_TELEMETRY" ~/.zshrc 2>/dev/null; then
        echo "" >> ~/.zshrc
        echo "# Claude Code Telemetry (Feature 017)" >> ~/.zshrc
        echo "export CLAUDE_CODE_ENABLE_TELEMETRY=1" >> ~/.zshrc
        echo "export OTEL_METRICS_EXPORTER=otlp" >> ~/.zshrc
        echo "export OTEL_LOGS_EXPORTER=otlp" >> ~/.zshrc
        echo "export OTEL_EXPORTER_OTLP_PROTOCOL=grpc" >> ~/.zshrc
        echo "export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317" >> ~/.zshrc
        echo "export OTEL_METRIC_EXPORT_INTERVAL=10000" >> ~/.zshrc
        echo "export OTEL_LOGS_EXPORT_INTERVAL=5000" >> ~/.zshrc
        echo "export OTEL_LOG_USER_PROMPTS=1" >> ~/.zshrc
        echo "✓ Telemetry variables added to ~/.zshrc"
    else
        echo "✓ Telemetry already configured in ~/.zshrc"
    fi
    ```
    
    **For bash**:
    ```bash
    # Check if already configured
    if ! grep -q "CLAUDE_CODE_ENABLE_TELEMETRY" ~/.bashrc 2>/dev/null; then
        echo "" >> ~/.bashrc
        echo "# Claude Code Telemetry (Feature 017)" >> ~/.bashrc
        echo "export CLAUDE_CODE_ENABLE_TELEMETRY=1" >> ~/.bashrc
        echo "export OTEL_METRICS_EXPORTER=otlp" >> ~/.bashrc
        echo "export OTEL_LOGS_EXPORTER=otlp" >> ~/.bashrc
        echo "export OTEL_EXPORTER_OTLP_PROTOCOL=grpc" >> ~/.bashrc
        echo "export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317" >> ~/.bashrc
        echo "export OTEL_METRIC_EXPORT_INTERVAL=10000" >> ~/.bashrc
        echo "export OTEL_LOGS_EXPORT_INTERVAL=5000" >> ~/.bashrc
        echo "export OTEL_LOG_USER_PROMPTS=1" >> ~/.bashrc
        echo "✓ Telemetry variables added to ~/.bashrc"
    else
        echo "✓ Telemetry already configured in ~/.bashrc"
    fi
    ```

15. **Load Configuration in Current Session**:
    
    Execute:
    ```bash
    export CLAUDE_CODE_ENABLE_TELEMETRY=1
    export OTEL_METRICS_EXPORTER=otlp
    export OTEL_LOGS_EXPORTER=otlp
    export OTEL_EXPORTER_OTLP_PROTOCOL=grpc
    export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317
    export OTEL_METRIC_EXPORT_INTERVAL=10000
    export OTEL_LOGS_EXPORT_INTERVAL=5000
    export OTEL_LOG_USER_PROMPTS=1
    ```

16. **Verify Environment Variables**:
    
    Execute:
    ```bash
    env | grep -E "(CLAUDE_CODE_ENABLE_TELEMETRY|OTEL_)" | head -5
    ```
    
    Verify telemetry variables are set.
    
    Show user: "✓ Claude Code telemetry enabled for current and future sessions"

### Phase 7: Data Collection Verification (3 minutes)

17. **Explain to User**:
    
    Show message:
    ```
    ✓ Setup complete! Telemetry will collect automatically.
    
    Note: Telemetry activates on NEXT Claude Code restart.
    For this session, I'll verify the system works with test data.
    ```

18. **Check Initial Data (from test-connection.sh)**:
    
    Execute (use actual generated password):
    ```bash
    docker exec telemetry-clickhouse clickhouse-client \
      --user telemetry_user \
      --password <ACTUAL_GENERATED_PASSWORD> \
      --query "SELECT count() FROM telemetry.telemetry_events"
    ```
    
    Should show at least 1 (test event from connection test).
    
    Show user: "✓ Test event verified in database"

19. **View Sample Data**:
    
    Execute (use actual generated password):
    ```bash
    docker exec telemetry-clickhouse clickhouse-client \
      --user telemetry_user \
      --password <ACTUAL_GENERATED_PASSWORD> \
      --database telemetry \
      --query "SELECT timestamp, event_type, tool_name, workflow_step FROM telemetry_events ORDER BY timestamp DESC LIMIT 5 FORMAT Pretty"
    ```
    
    Show output to user.

### Phase 8: Summary & Next Steps (2 minutes)

20. **Create Quick Reference File**:
    
    Execute (use actual generated password and $REPO_ROOT):
    ```bash
    cd "$REPO_ROOT/telemetry-server"
    
    cat > CREDENTIALS.txt <<EOF
# Telemetry Credentials
# Generated: $(date)
# KEEP THIS FILE SECURE - DO NOT COMMIT TO GIT

ClickHouse User: telemetry_user
ClickHouse Password: <ACTUAL_GENERATED_PASSWORD>

Quick Commands:
--------------

# Check event count:
docker exec telemetry-clickhouse clickhouse-client \\
  --user telemetry_user --password <ACTUAL_GENERATED_PASSWORD> \\
  --query "SELECT count() FROM telemetry.telemetry_events"

# Run error analysis:
docker exec telemetry-clickhouse clickhouse-client \\
  --user telemetry_user --password <ACTUAL_GENERATED_PASSWORD> \\
  --database telemetry --multiquery < queries/error-analysis.sql

# Run session analysis:
docker exec telemetry-clickhouse clickhouse-client \\
  --user telemetry_user --password <ACTUAL_GENERATED_PASSWORD> \\
  --database telemetry --multiquery < queries/session-analysis.sql

# Start services:
cd telemetry-server && docker-compose start

# Stop services:
cd telemetry-server && docker-compose stop
EOF
    
    chmod 600 CREDENTIALS.txt
    ```
    
    Replace `<ACTUAL_GENERATED_PASSWORD>` with real password.

21. **Display Final Summary**:
    
    Show user a comprehensive success message with all details (see Output Style section below).

---

## Success Criteria Checklist

Verify all criteria before completing:

- ✅ **Docker verified**: `docker ps` succeeds
- ✅ **Pre-flight checks passed**: All ✓ in `verify-setup.sh`
- ✅ **Environment configured**: Passwords changed in `.env`
- ✅ **Services running**: `docker-compose ps` shows both "Up"
- ✅ **Connection tests passed**: All ✓ in `test-connection.sh`
- ✅ **Database initialized**: Tables exist (telemetry_events, etc.)
- ✅ **Claude Code configured**: `$CLAUDE_ENV_CONFIG` set
- ✅ **Data collecting**: Event count > 0 after usage
- ✅ **Queries working**: Sample query returns results
- ✅ **Setup time**: < 15 minutes total

---

## Daily Usage Commands

Provide user with these commands for daily operations:

```bash
# Start telemetry stack (if stopped)
cd telemetry-server && docker-compose start

# Stop telemetry stack (optional, saves resources)
cd telemetry-server && docker-compose stop

# View live logs
cd telemetry-server && docker-compose logs -f

# Check service status
cd telemetry-server && docker-compose ps

# View event count
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password <PASSWORD> \
  --query "SELECT count() FROM telemetry.telemetry_events"

# Run error analysis
cd telemetry-server && docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password <PASSWORD> \
  --database telemetry --multiquery < queries/error-analysis.sql

# Run session analysis
cd telemetry-server && docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password <PASSWORD> \
  --database telemetry --multiquery < queries/session-analysis.sql
```

---

## Troubleshooting Guide

### Issue: Services Won't Start

**Diagnosis**:
```bash
docker-compose logs
```

**Common Causes**:
1. **Port conflict**: Another process using 4317, 4318, 8123, or 9000
   - Check: `lsof -i :8123` (repeat for other ports)
   - Fix: Kill process or change port in docker-compose.yml

2. **Insufficient memory**: Docker doesn't have enough RAM allocated
   - Fix: Increase Docker Desktop memory limit (Preferences → Resources)

3. **Invalid configuration**: Syntax error in docker-compose.yml or config files
   - Check: `docker-compose config`

### Issue: No Data Appearing

**Diagnosis**:
```bash
# Check environment variable
echo $CLAUDE_ENV_CONFIG

# Check OTLP Collector is receiving data
docker-compose logs otel-collector | grep -i "receiver"
```

**Solutions**:
1. **Environment not loaded**:
   ```bash
   export CLAUDE_ENV_CONFIG="/full/path/to/.claude/env-config/telemetry-enabled.env"
   ```

2. **Wrong OTLP endpoint**:
   - Check `.claude/env-config/telemetry-enabled.env` has `OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317`

3. **Firewall blocking**:
   - Temporarily disable firewall for testing
   - Add firewall rules for ports 4317, 8123

### Issue: Authentication Failed

**Diagnosis**:
```bash
# Check password in .env
cat .env | grep CLICKHOUSE_PASSWORD
```

**Solution**:
```bash
# Reset with known passwords
docker-compose down -v
# Edit .env with new passwords
docker-compose up -d
sleep 30
./scripts/test-connection.sh
```

### Issue: Slow Queries

**Solution**:
```bash
# Use time-based filtering
SELECT * FROM telemetry_events WHERE timestamp >= now() - INTERVAL 7 DAY LIMIT 1000;

# Use materialized views (pre-aggregated)
SELECT * FROM session_summaries WHERE session_date >= today() - INTERVAL 7 DAY;

# Optimize table (defragmentation)
docker exec -it telemetry-clickhouse clickhouse-client \
  --query "OPTIMIZE TABLE telemetry.telemetry_events FINAL"
```

---

## Privacy & Security Notes

**Explain to user**:

1. **Privacy-Preserving Design**:
   - ✅ Prompts are SHA256 hashed (NOT stored in plaintext)
   - ✅ Tool parameters are hashed
   - ✅ PII (emails, phone numbers) automatically filtered
   - ✅ API keys never logged

2. **Data Location**:
   - Stored in: `telemetry-server/clickhouse_data/` (git-ignored)
   - Retention: 90 days (automatic deletion via TTL)
   - Backup: Manual (Phase 2 will add automated backups)

3. **Access Control**:
   - Two users: `telemetry_user` (read/write), `telemetry_reader` (read-only)
   - Passwords in `.env` file (never commit to git)

4. **What's Collected**:
   - ✅ Session IDs, timestamps, event types
   - ✅ Tool names, workflow phases (specify/plan/implement/validate)
   - ✅ Performance metrics (latency, tokens, cost)
   - ✅ Error codes and messages (sanitized)
   - ❌ NOT collected: Raw prompts, API keys, personal information

---

## Output Style

Use this format for user communication:

```
🚀 TELEMETRY SETUP STARTING
═══════════════════════════════════════════════════════════════

Phase 1: Prerequisites Check
  ✓ Docker version 24.0.5 detected
  ✓ Docker Compose version v2.20.2 detected  
  ✓ Docker daemon is running
  ✓ 45GB free disk space available

Phase 2: Pre-Flight Verification
  ✓ Running automated checks...
  ✓ All 10 checks passed

Phase 3: Environment Configuration
  ✓ Generated secure password (24 chars)
  ✓ Configured .env file automatically
  ✓ Set user ID: <username>

Phase 4: Service Startup
  ✓ Starting ClickHouse database...
  ✓ Starting OTLP Collector...
  ⏳ Waiting 30 seconds for initialization...
  ✓ Both services running

Phase 5: Connection Testing
  ✓ Running 9 connection tests...
  ✓ ClickHouse HTTP interface: OK
  ✓ Authentication: OK
  ✓ Database schema: 4 tables created
  ✓ OTLP endpoints: OK
  ✓ Test event inserted: OK

Phase 6: Claude Code Integration
  ✓ Detected shell: zsh
  ✓ Added to ~/.zshrc
  ✓ Environment variable configured
  ✓ Telemetry enabled (activates on next restart)

Phase 7: Verification
  ✓ Test event verified in database
  ✓ Query performance: <100ms

Phase 8: Summary
  ✓ Created CREDENTIALS.txt with quick commands
  ✓ Saved database password securely

═══════════════════════════════════════════════════════════════
✅ TELEMETRY SETUP COMPLETE (Total time: <X> minutes)
═══════════════════════════════════════════════════════════════

📊 What's Ready:
• Local telemetry stack running (ClickHouse + OTLP Collector)
• Privacy-preserving data collection (SHA256 hashing, PII filtering)
• Andrew Ng's error analysis queries available
• 90-day automatic data retention
• Credentials saved in: telemetry-server/CREDENTIALS.txt

🔐 Your Credentials:
• Username: telemetry_user
• Password: <DISPLAYED_ONCE_HERE>
• (Also saved in: CREDENTIALS.txt - keep secure!)

📈 Available Analyses:
1. Error Analysis (Andrew Ng methodology)
   → 10 queries for pattern identification and prioritization

2. Session Analysis (Productivity tracking)
   → 12 queries for workflow and performance insights

3. Tool Usage Analysis (Performance monitoring)
   → 13 queries for tool efficiency and reliability

🚀 Next Steps:

1. RESTART Claude Code to activate telemetry collection
   (Current session won't collect data - that's normal)

2. Check your data after using Claude Code:
   cd telemetry-server
   docker exec telemetry-clickhouse clickhouse-client \
     --user telemetry_user --password <PASSWORD> \
     --query "SELECT count() FROM telemetry.telemetry_events"

3. Run your first analysis (after collecting some data):
   docker exec telemetry-clickhouse clickhouse-client \
     --user telemetry_user --password <PASSWORD> \
     --database telemetry --multiquery < queries/session-analysis.sql

📖 Documentation:
• Quick reference: telemetry-server/START_HERE.md
• Full guide: telemetry-server/README.md
• Troubleshooting: telemetry-server/docs/troubleshooting.md
• Query examples: telemetry-server/queries/README.md

💡 Daily Commands:
• Start: cd telemetry-server && docker-compose start
• Stop: cd telemetry-server && docker-compose stop
• Status: cd telemetry-server && docker-compose ps
• Logs: cd telemetry-server && docker-compose logs -f

🔒 Privacy Notes:
✅ Prompts are SHA256 hashed (NOT plaintext)
✅ PII automatically filtered
✅ Data stays local (not sent anywhere)
✅ 90-day automatic deletion
✅ Excluded from git

═══════════════════════════════════════════════════════════════
You're all set! Restart Claude Code to start collecting data. 🎉
═══════════════════════════════════════════════════════════════
```

---

## Error Handling

If any step fails:

1. **Stop execution** and show detailed error
2. **Provide diagnosis commands** specific to the failure
3. **Offer solutions** from troubleshooting guide
4. **Suggest rollback** if needed: `docker-compose down -v`
5. **Direct to documentation**: `docs/troubleshooting.md` for detailed help

---

## Post-Setup Validation

After successful setup, create a validation report:

```bash
# Create report file
cat > telemetry-server/SETUP_REPORT.txt <<EOF
Telemetry Setup Report
======================
Date: $(date)
User: $(whoami)
Host: $(hostname)

Services Status:
$(docker-compose ps)

Event Count:
$(docker exec -it telemetry-clickhouse clickhouse-client --user telemetry_user --password <PASSWORD> --query "SELECT count() FROM telemetry.telemetry_events")

Recent Events:
$(docker exec -it telemetry-clickhouse clickhouse-client --user telemetry_user --password <PASSWORD> --query "SELECT timestamp, event_type, tool_name FROM telemetry.telemetry_events ORDER BY timestamp DESC LIMIT 5")

Setup Time: <ACTUAL_TIME_TAKEN>

All success criteria met: YES/NO
EOF
```

**End of command. Execute all phases sequentially, verify success criteria, and provide user with daily usage commands.**
