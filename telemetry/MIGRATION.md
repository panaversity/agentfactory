# Migration from Docker-based to File-based Telemetry

**Date**: 2025-01-12
**Reason**: Simplify infrastructure, remove Docker dependencies, focus on core data collection

## What Changed

### Old Approach (v1.0) - Docker-based
```
┌─────────────────────┐
│   Claude Code       │
│ (OTLP exporter)     │
└──────────┬──────────┘
           │
           │ gRPC/HTTP
           ▼
┌─────────────────────┐
│ OTLP Collector      │
│ (Docker container)  │
└──────────┬──────────┘
           │
           │ ClickHouse Protocol
           ▼
┌─────────────────────┐
│   ClickHouse DB     │
│ (Docker container)  │
└──────────┬──────────┘
           │
           │ SQL queries
           ▼
┌─────────────────────┐
│   Analysis          │
└─────────────────────┘
```

**Components**:
- `telemetry-server/docker-compose.yml`
- `telemetry-server/otel-collector/config.yaml`
- `telemetry-server/clickhouse/schema.sql`
- Multiple SQL query files

**Setup complexity**: ~20 steps, Docker required, port management, database credentials

---

### New Approach (v2.0) - File-based
```
┌─────────────────────┐
│   Claude Code       │
│ (console exporter)  │
└──────────┬──────────┘
           │
           │ stdout/stderr
           ▼
┌─────────────────────┐
│  Session Log File   │
│  (plain text)       │
└──────────┬──────────┘
           │
           │ parser.py
           ▼
┌─────────────────────┐
│  Structured JSON    │
└──────────┬──────────┘
           │
           │ analyze.py
           ▼
┌─────────────────────┐
│   Analysis Report   │
└─────────────────────┘
```

**Components**:
- `telemetry/enable-telemetry.sh` (setup)
- `telemetry/parser.py` (log → JSON)
- `telemetry/analyze.py` (JSON → insights)
- `~/.claude-code-telemetry/` (data directory)

**Setup complexity**: 1 command, no dependencies beyond Python 3.8+

## Benefits of New Approach

### 1. Simplicity
- ❌ **Old**: Docker, OTLP collector, ClickHouse database, SQL queries
- ✅ **New**: Bash scripts, Python parsing, JSON files

### 2. Portability
- ❌ **Old**: Docker required, port 4317 and 8123, database credentials
- ✅ **New**: Works anywhere with Bash + Python

### 3. Debugging
- ❌ **Old**: Check Docker logs, OTLP collector, ClickHouse connection
- ✅ **New**: Read plain text log files directly

### 4. Setup Time
- ❌ **Old**: ~10 minutes (Docker setup, start containers, verify connections)
- ✅ **New**: ~30 seconds (run one script)

### 5. Team Onboarding
- ❌ **Old**: Requires Docker knowledge, database familiarity, SQL skills
- ✅ **New**: Run script, share JSON files

### 6. Privacy
- ❌ **Old**: Data sent over network (localhost, but still networked)
- ✅ **New**: All data in local files

## Migration Path

### For Existing Users

If you set up the Docker-based system:

1. **Stop the old infrastructure** (optional, can keep both):
   ```bash
   cd telemetry-server
   docker-compose down
   ```

2. **Enable new file-based telemetry**:
   ```bash
   bash telemetry/enable-telemetry.sh
   ```

3. **Start using the wrapper**:
   ```bash
   ~/.claude-code-telemetry/claude-with-telemetry.sh
   ```

4. **No data migration needed** - old and new systems are independent

### For New Users

Just follow the Quick Start:
```bash
bash telemetry/enable-telemetry.sh
~/.claude-code-telemetry/claude-with-telemetry.sh
# Use Claude Code normally
python telemetry/parser.py
python telemetry/analyze.py
```

## What's Preserved

The core data collection strategy remains the same:

- ✅ Andrew Ng's methodology (break silos, error analysis, evals-first)
- ✅ Same telemetry events (prompts, tools, API calls, errors)
- ✅ Same metrics (tokens, costs, sessions)
- ✅ Same analysis patterns (cost analysis, error identification)

## What's Removed

Infrastructure complexity:

- ❌ Docker containers
- ❌ OTLP collector configuration
- ❌ ClickHouse database
- ❌ SQL query files
- ❌ Network protocols
- ❌ Port management
- ❌ Database credentials

## Files Changed

### New Files
```
telemetry/
├── enable-telemetry.sh       # Setup script
├── parser.py                 # Log parser
├── analyze.py                # Analysis engine
├── README.md                 # Full documentation
├── QUICKSTART.md             # 5-minute guide
└── MIGRATION.md              # This file
```

### Updated Files
```
specs/017-usage-data-collection/
└── spec.md                   # Updated with v2.0 approach
```

### Deprecated Files (kept for reference)
```
telemetry-server/             # Old Docker-based setup
├── docker-compose.yml
├── otel-collector/
├── clickhouse/
└── queries/
```

## Rationale

**Quote from user request**:
> "Now the data strategy is same but you will use claude code monitoring https://code.claude.com/docs/en/monitoring-usage and no docker nothing keep it simple and get it right"

**Decision**: Use Claude Code's built-in console exporter instead of OTLP
- Simpler setup (environment variables only)
- No external dependencies (Docker, databases)
- Same data, simpler collection
- Better for individual contributors
- Easier team adoption

## Next Steps

1. Test the new system with a real session
2. Verify parser extracts data correctly
3. Confirm analysis reports are useful
4. Gather team feedback
5. Iterate based on actual usage

## Rollback Plan

If file-based approach has issues:

1. Old Docker setup still exists in `telemetry-server/`
2. Can re-enable OTLP exporter:
   ```bash
   export OTEL_METRICS_EXPORTER=otlp
   export OTEL_LOGS_EXPORTER=otlp
   export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317
   ```
3. Restart Docker containers:
   ```bash
   cd telemetry-server
   docker-compose up -d
   ```

## Questions?

- Check `telemetry/README.md` for full documentation
- Check `telemetry/QUICKSTART.md` for 5-minute setup
- Review actual log files in `~/.claude-code-telemetry/logs/`
- Inspect parsed JSON in `~/.claude-code-telemetry/data/`
