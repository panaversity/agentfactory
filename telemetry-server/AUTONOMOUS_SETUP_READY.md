# ✅ Autonomous Telemetry Setup - Ready to Use!

## 🎯 For Team Members

Just run this ONE command in Claude Code:

```
/sp.telemetry-setup
```

**Everything happens automatically!** You don't need to:
- ❌ Run any commands manually
- ❌ Edit any files
- ❌ Generate passwords yourself
- ❌ Configure anything

The command will:
1. ✅ Check Docker is running
2. ✅ Generate secure passwords automatically
3. ✅ Configure .env file
4. ✅ Start ClickHouse and OTLP Collector services
5. ✅ Verify all connections work
6. ✅ Enable Claude Code telemetry
7. ✅ Create CREDENTIALS.txt with your password and quick commands
8. ✅ Show you a complete success summary

**Time**: 10-15 minutes (fully automated)

---

## What the Command Does

### Phase 1: Prerequisites (1 min)
- Checks Docker is installed and running
- Verifies disk space available
- Navigates to telemetry directory

### Phase 2: Verification (1 min)
- Runs automated pre-flight checks
- Detects port conflicts (asks if you want to kill them)

### Phase 3: Configuration (2 min)
- Generates TWO secure 24-character passwords
- Creates .env file from template
- Replaces default passwords with secure ones
- Sets your user ID automatically

### Phase 4: Service Startup (3 min)
- Starts ClickHouse database
- Starts OTLP Collector
- Waits 30 seconds for initialization
- Verifies both services are running

### Phase 5: Connection Testing (2 min)
- Tests ClickHouse HTTP interface
- Tests authentication
- Verifies database schema (4 tables)
- Tests OTLP endpoints
- Inserts test event

### Phase 6: Claude Code Integration (2 min)
- Detects your shell (zsh or bash)
- Adds environment variable to ~/.zshrc or ~/.bashrc
- Configures telemetry to activate on next Claude Code restart

### Phase 7: Verification (1 min)
- Verifies test data in database
- Tests query performance

### Phase 8: Summary (1 min)
- Creates CREDENTIALS.txt with your password and quick commands
- Shows comprehensive success message
- Displays next steps

---

## After Setup

### Your credentials will be in:
```
telemetry-server/CREDENTIALS.txt
```

**Keep this file secure!** It contains your database password.

### To start using telemetry:

1. **Restart Claude Code** (telemetry activates on restart)

2. **Use Claude Code normally** - data collects automatically

3. **Check your data**:
   ```bash
   cd telemetry-server
   docker exec telemetry-clickhouse clickhouse-client \
     --user telemetry_user --password YOUR_PASSWORD \
     --query "SELECT count() FROM telemetry.telemetry_events"
   ```

4. **Run analysis queries**:
   ```bash
   # Error analysis (Andrew Ng methodology)
   docker exec telemetry-clickhouse clickhouse-client \
     --user telemetry_user --password YOUR_PASSWORD \
     --database telemetry --multiquery < queries/error-analysis.sql
   
   # Session analysis (productivity tracking)
   docker exec telemetry-clickhouse clickhouse-client \
     --user telemetry_user --password YOUR_PASSWORD \
     --database telemetry --multiquery < queries/session-analysis.sql
   ```

---

## What Data is Collected?

### ✅ Privacy-Preserved Data:
- Session IDs and timestamps
- Tool names and workflow phases (specify, plan, implement, validate)
- Performance metrics (latency, tokens, cost)
- **Prompts are SHA256 hashed** (NOT stored in plaintext)
- PII automatically filtered (emails, phone numbers)

### ❌ NOT Collected:
- Actual prompt text (only hash stored)
- API keys or secrets
- Personal information
- File contents

### 🔒 Privacy Features:
- ✅ SHA256 hashing for sensitive data
- ✅ PII filtering (emails, phone numbers removed)
- ✅ API key filtering (prevents accidental logging)
- ✅ Local storage only (data never leaves your machine)
- ✅ 90-day automatic deletion (TTL)
- ✅ Excluded from git (in .gitignore)

---

## Daily Commands

All these commands will be in your `CREDENTIALS.txt` file:

```bash
# Start telemetry (if stopped)
cd telemetry-server && docker-compose start

# Stop telemetry (saves resources)
cd telemetry-server && docker-compose stop

# Check status
cd telemetry-server && docker-compose ps

# View logs
cd telemetry-server && docker-compose logs -f

# Check event count
docker exec telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password YOUR_PASSWORD \
  --query "SELECT count() FROM telemetry.telemetry_events"
```

---

## Available Analyses

### 1. Error Analysis (Andrew Ng's Methodology)
10 queries for:
- Pattern identification
- Error categorization
- Context analysis
- Prioritization
- Recovery time measurement

### 2. Session Analysis (Productivity Tracking)
12 queries for:
- Session duration distribution
- Daily productivity metrics
- Peak usage hours
- Workflow completeness
- User productivity comparison

### 3. Tool Usage Analysis (Performance Monitoring)
13 queries for:
- Most used tools
- Tool performance (latency percentiles)
- Tool reliability ranking
- Tool combination patterns
- Efficiency scoring

---

## Troubleshooting

### If setup fails:

1. **Check Docker is running**:
   ```bash
   docker ps
   ```
   If error: Start Docker Desktop app

2. **Check the logs**:
   ```bash
   cd telemetry-server
   docker-compose logs
   ```

3. **Try the manual verification**:
   ```bash
   cd telemetry-server
   ./scripts/verify-setup.sh
   ```

4. **See detailed troubleshooting**:
   ```bash
   cat docs/troubleshooting.md
   ```

5. **Reset and try again**:
   ```bash
   cd telemetry-server
   docker-compose down -v
   # Then run /sp.telemetry-setup again
   ```

---

## Documentation

- **START_HERE.md** - This file
- **QUICK_START.md** - Quick reference
- **SETUP.md** - Manual setup (if needed)
- **README.md** - Full documentation
- **docs/quickstart.md** - Detailed guide
- **docs/troubleshooting.md** - 25+ issues solved
- **queries/README.md** - Query examples

---

## Support

- **Run the command**: `/sp.telemetry-setup`
- **Check documentation**: See files listed above
- **Contact team**: Book development team

---

## Success Criteria

Setup is successful when:
- ✅ Both services show "Up" status
- ✅ All connection tests pass
- ✅ Database has 4 tables
- ✅ Test event exists in database
- ✅ CREDENTIALS.txt file created
- ✅ Environment variable added to shell config
- ✅ Total setup time < 15 minutes

---

**Ready to start! Just run `/sp.telemetry-setup` in Claude Code! 🚀**

**Remember**: The command does EVERYTHING automatically. You just watch it work!
