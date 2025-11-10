# Telemetry Server - Usage Data Collection System

**Feature 017**: Usage Data Collection System  
**Version**: 1.0.0  
**Status**: MVP (Phase 1 - Local Collection)

## Overview

This directory contains the local telemetry infrastructure for collecting Claude Code usage data following Andrew Ng's error analysis methodology. The system enables:

- **Real-time telemetry collection** via OpenTelemetry Protocol (OTLP)
- **Time-series storage** in ClickHouse database
- **Error pattern analysis** for improving agentic AI performance
- **Privacy-preserving data sanitization** (SHA256 hashing, PII filtering)

## Architecture

```
Claude Code (client)
    ↓ OTLP/gRPC (port 4317)
OTLP Collector
    ↓ Batch processing + data sanitization
ClickHouse Database
    ↓ SQL queries
Analysis & Insights
```

## Quick Start

### Automated Setup (Recommended) 🚀

Use the slash command for guided setup:

```
/sp.telemetry-setup
```

This command will walk you through all steps automatically. **Time: 10-15 minutes**

### Manual Setup

### 1. Prerequisites

- Docker Engine 20.10+
- Docker Compose 2.0+
- 4GB+ RAM available
- 10GB+ disk space

### 2. Verify Setup

```bash
cd telemetry-server
./scripts/verify-setup.sh
```

This checks:
- ✅ Docker installation
- ✅ Required files exist
- ✅ Ports available
- ✅ Disk space sufficient

### 3. Configure Environment

```bash
# Copy environment template
cp .env.template .env

# Edit credentials (IMPORTANT: change default passwords!)
nano .env
```

**Minimum required changes**:
```bash
CLICKHOUSE_PASSWORD=your_secure_password_here
TELEMETRY_READER_PASSWORD=another_secure_password
```

### 4. Start Services

```bash
docker-compose up -d
```

Services started:
- **ClickHouse** (localhost:8123) - Database
- **OTLP Collector** (localhost:4317) - Telemetry ingestion

### 5. Verify Connection

```bash
./scripts/test-connection.sh
```

Expected output:
```
✓ ClickHouse HTTP interface is responding
✓ Authentication successful
✓ Telemetry database exists
✓ OTLP Collector gRPC port is open
✓ Test event inserted successfully
```

### 6. Enable Claude Code Telemetry

```bash
# Start Claude Code with telemetry enabled
claude-code --env ../.claude/env-config/telemetry-enabled.env
```

Or set permanently in your shell profile:
```bash
export CLAUDE_ENV_CONFIG=/path/to/.claude/env-config/telemetry-enabled.env
```

## Directory Structure

```
telemetry-server/
├── docker-compose.yml          # Service orchestration
├── .env.template               # Environment configuration template
├── .env                        # Your local configuration (git-ignored)
├── README.md                   # This file
│
├── clickhouse/                 # ClickHouse database configuration
│   ├── schema.sql              # Database schema (auto-initialized)
│   └── config.xml              # Performance tuning
│
├── otel-collector/             # OTLP Collector configuration
│   └── config.yaml             # Receivers, processors, exporters
│
├── scripts/                    # Automation scripts
│   ├── verify-setup.sh         # Pre-flight checks
│   ├── test-connection.sh      # Connection testing
│   └── backup-data.sh          # Data backup (Phase 2)
│
├── queries/                    # Example SQL queries
│   ├── README.md               # Query documentation
│   ├── session-analysis.sql    # Session performance queries
│   ├── error-analysis.sql      # Error pattern queries
│   └── tool-usage.sql          # Tool usage analytics
│
├── docs/                       # Extended documentation
│   ├── quickstart.md           # Detailed setup guide
│   ├── troubleshooting.md      # Common issues
│   └── query-examples.md       # Advanced query patterns
│
├── clickhouse_data/            # Database storage (git-ignored)
└── otel_logs/                  # Collector logs (git-ignored)
```

## Common Operations

### View Logs

```bash
# All services
docker-compose logs -f

# Specific service
docker-compose logs -f clickhouse
docker-compose logs -f otel-collector
```

### Check Service Status

```bash
docker-compose ps
```

Expected output:
```
NAME                      STATUS    PORTS
telemetry-clickhouse      Up        0.0.0.0:8123->8123/tcp, 0.0.0.0:9000->9000/tcp
telemetry-otel-collector  Up        0.0.0.0:4317-4318->4317-4318/tcp
```

### Restart Services

```bash
docker-compose restart
```

### Stop Services

```bash
docker-compose down
```

### Reset Database (⚠️ Deletes all data)

```bash
docker-compose down -v
rm -rf clickhouse_data/
docker-compose up -d
```

## Query Examples

### Event Count by Type

```bash
curl -u "telemetry_user:your_password" \
  "http://localhost:8123/?query=SELECT%20event_type%2C%20count()%20FROM%20telemetry.telemetry_events%20GROUP%20BY%20event_type"
```

### Recent Sessions

```sql
SELECT 
    session_id,
    user_id,
    count() AS event_count,
    min(timestamp) AS session_start,
    max(timestamp) AS session_end,
    dateDiff('minute', min(timestamp), max(timestamp)) AS duration_minutes
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 1 DAY
GROUP BY session_id, user_id
ORDER BY session_start DESC
LIMIT 10;
```

### Error Analysis (Andrew Ng's methodology)

```sql
SELECT 
    api_error_code,
    tool_name,
    workflow_step,
    count() AS error_count,
    uniqExact(session_id) AS affected_sessions,
    anyLast(api_error_message) AS sample_error
FROM telemetry.telemetry_events
WHERE event_type = 'api_error'
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY api_error_code, tool_name, workflow_step
ORDER BY error_count DESC
LIMIT 20;
```

For more examples, see `queries/README.md`.

## Data Privacy & Security

### Privacy Controls (Enabled by Default)

- ✅ **Prompt hashing**: User prompts are SHA256 hashed, not stored in plaintext
- ✅ **PII filtering**: Email addresses, phone numbers automatically removed
- ✅ **Parameter sanitization**: Tool parameters hashed for privacy
- ✅ **API key filtering**: Prevents accidental logging of secrets

### Data Retention

- **Default**: 90 days (configurable via `TTL` in schema.sql)
- **Location**: `clickhouse_data/` (excluded from git)
- **Backup**: Manual (Phase 2 will add automated backups)

### Access Control

Two database users:
1. **telemetry_user**: Read/write access (for OTLP Collector)
2. **telemetry_reader**: Read-only access (for queries)

Change passwords in `.env` file.

## Performance Tuning

### Resource Limits

Current defaults (adjust in `docker-compose.yml`):
- **ClickHouse**: 2GB RAM, 1 CPU
- **OTLP Collector**: 512MB RAM, 0.5 CPU

### Expected Performance

- **Ingestion**: 1,000+ events/second
- **Storage**: ~20x compression ratio (ClickHouse)
- **Query**: <100ms for most analytical queries

### Monitoring

- **Prometheus metrics**: http://localhost:8888/metrics
- **Health check**: http://localhost:13133/
- **ClickHouse system tables**: `system.query_log`, `system.metrics`

## Troubleshooting

### Services won't start

```bash
# Check Docker daemon
docker info

# Check port conflicts
lsof -i :4317,4318,8123,9000

# Check logs
docker-compose logs
```

### Connection refused

```bash
# Verify services are running
docker-compose ps

# Test ClickHouse directly
curl http://localhost:8123/ping

# Test OTLP Collector health
curl http://localhost:13133/
```

### No data appearing

```bash
# Check Claude Code is using telemetry config
env | grep OTEL_

# Verify OTLP Collector is receiving data
docker-compose logs otel-collector | grep "Traces"

# Check ClickHouse for events
curl -u "telemetry_user:password" \
  "http://localhost:8123/?query=SELECT%20count()%20FROM%20telemetry.telemetry_events"
```

### Authentication errors

```bash
# Verify credentials in .env match docker-compose
cat .env | grep CLICKHOUSE_PASSWORD

# Reset credentials
docker-compose down -v
# Edit .env
docker-compose up -d
```

For more troubleshooting, see `docs/troubleshooting.md`.

## Roadmap

### ✅ Phase 1: Local Collection (CURRENT)

- [x] Docker Compose setup
- [x] ClickHouse database
- [x] OTLP Collector
- [x] Basic queries
- [x] Privacy controls

### 🔄 Phase 2: Cloud Migration (OPTIONAL)

- [ ] Centralized cloud endpoint
- [ ] Team access controls
- [ ] Automated backups
- [ ] Data export utilities

### 📊 Phase 3: Analytics Dashboard (OPTIONAL)

- [ ] Pre-built dashboards
- [ ] Automated reports
- [ ] Anomaly detection
- [ ] Cost analysis

## Support

- **Documentation**: `docs/quickstart.md`
- **Spec**: `../specs/017-usage-data-collection/spec.md`
- **Issues**: Check `docs/troubleshooting.md` first
- **Team**: Contact book development team

## License

**CC BY-NC-ND 4.0** - See LICENSE in repository root  
**Collected data**: Project-private, not included in repository

---

**Ready to collect telemetry!** 🚀

For detailed setup instructions, see `docs/quickstart.md`
