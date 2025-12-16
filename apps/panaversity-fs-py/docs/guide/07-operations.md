# Operations Guide

> Deploy, monitor, and maintain PanaversityFS in production

**Spec Reference**: [Feature 039: PanaversityFS Production Hardening](../../../../specs/039-panaversity-fs-hardening/spec.md)

This guide covers operational requirements for achieving the success criteria (SC-001 to SC-010) defined in the authoritative specification.

## Deployment Options

### Option 1: Docker Container

```dockerfile
FROM python:3.13-slim

# System dependencies
RUN apt-get update && apt-get install -y libmagic1 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .

# Install uv and dependencies
RUN pip install uv && uv sync --frozen

# Run migrations and start server
CMD ["sh", "-c", "uv run alembic upgrade head && uv run python -m panaversity_fs.server"]
```

Build and run:

```bash
docker build -t panaversity-fs .
docker run -p 8000:8000 \
  -e PANAVERSITY_STORAGE_BACKEND=s3 \
  -e PANAVERSITY_S3_BUCKET=my-bucket \
  -e PANAVERSITY_S3_ENDPOINT=https://... \
  -e PANAVERSITY_S3_ACCESS_KEY_ID=... \
  -e PANAVERSITY_S3_SECRET_ACCESS_KEY=... \
  -e DATABASE_URL=postgresql+asyncpg://... \
  panaversity-fs
```

### Option 2: Fly.io

```toml
# fly.toml
app = "panaversity-fs"
primary_region = "sjc"

[build]
  dockerfile = "Dockerfile"

[env]
  PANAVERSITY_SERVER_HOST = "0.0.0.0"
  PANAVERSITY_SERVER_PORT = "8080"

[http_service]
  internal_port = 8080
  force_https = true

[[services]]
  internal_port = 8080
  protocol = "tcp"

  [[services.ports]]
    port = 443
    handlers = ["tls", "http"]
```

Deploy:

```bash
fly launch
fly secrets set PANAVERSITY_S3_ACCESS_KEY_ID=...
fly secrets set PANAVERSITY_S3_SECRET_ACCESS_KEY=...
fly secrets set DATABASE_URL=...
fly deploy
```

### Option 3: Google Cloud Run

```bash
# Build and push
gcloud builds submit --tag gcr.io/PROJECT/panaversity-fs

# Deploy
gcloud run deploy panaversity-fs \
  --image gcr.io/PROJECT/panaversity-fs \
  --platform managed \
  --region us-central1 \
  --set-env-vars "PANAVERSITY_STORAGE_BACKEND=s3" \
  --set-secrets "PANAVERSITY_S3_ACCESS_KEY_ID=s3-key:latest"
```

## Database Management

### Production Database Setup

Use PostgreSQL for production (SQLite is for development only):

```bash
# Create database
createdb panaversityfs

# Set connection URL
export DATABASE_URL=postgresql+asyncpg://user:password@host:5432/panaversityfs

# Run migrations
uv run alembic upgrade head
```

### Migration Workflow

```bash
# Check current migration state
uv run alembic current

# View history
uv run alembic history

# Upgrade to latest
uv run alembic upgrade head

# Downgrade one step (use with caution!)
uv run alembic downgrade -1
```

### Backup Strategy

```bash
# PostgreSQL backup
pg_dump panaversityfs > backup_$(date +%Y%m%d).sql

# Restore
psql panaversityfs < backup_20250101.sql
```

## Monitoring

### Prometheus Metrics (from Spec Instrumentation Requirements)

PanaversityFS exposes metrics at `/metrics` for verifying success criteria:

| Metric | Type | Labels | Success Criteria |
|--------|------|--------|------------------|
| `panaversityfs_write_total` | Counter | book_id, status, mode | SC-002, SC-003 |
| `panaversityfs_write_duration_seconds` | Histogram | book_id | - |
| `panaversityfs_archive_total` | Counter | book_id, scope, status | SC-001 |
| `panaversityfs_archive_duration_seconds` | Histogram | scope | SC-001 (<60s) |
| `panaversityfs_archive_memory_bytes` | Gauge | - | SC-001 (<64MB) |
| `panaversityfs_journal_entries_total` | Gauge | book_id | - |
| `panaversityfs_audit_chain_valid` | Gauge | - | SC-004 (R6 integrity) |
| `panaversityfs_delta_files_count` | Histogram | book_id | SC-007 |

### Prometheus Configuration

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'panaversity-fs'
    static_configs:
      - targets: ['localhost:8000']
    metrics_path: '/metrics'
```

### Grafana Dashboard

Key panels:

1. **Request Rate**: `rate(panaversityfs_write_total[5m])`
2. **Error Rate**: `rate(panaversityfs_write_total{status="error"}[5m])`
3. **Latency p99**: `histogram_quantile(0.99, panaversityfs_write_duration_seconds_bucket)`
4. **Archive Memory**: `panaversityfs_archive_memory_bytes`

### Health Checks

```bash
# Basic health check
curl http://localhost:8000/health

# MCP protocol check
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
```

## Logging

### Log Levels

```bash
# Set log level
export PANAVERSITY_LOG_LEVEL=INFO  # DEBUG, INFO, WARNING, ERROR
```

### Log Format

```
2025-12-04T10:30:00Z INFO  [panaversity_fs.tools.content] write_content book_id=ai-python path=content/01-Part/01-lesson.md size=2345
2025-12-04T10:30:00Z INFO  [panaversity_fs.audit] logged operation=WRITE agent_id=agent-123 path=content/01-Part/01-lesson.md
```

### Structured Logging

Logs are JSON-formatted for easy parsing:

```json
{
  "timestamp": "2025-12-04T10:30:00Z",
  "level": "INFO",
  "logger": "panaversity_fs.tools.content",
  "message": "write_content",
  "book_id": "ai-python",
  "path": "content/01-Part/01-lesson.md",
  "size": 2345,
  "duration_ms": 45
}
```

## Troubleshooting

### Common Issues

#### 1. "Connection refused" on startup

```bash
# Check if port is in use
lsof -i :8000

# Use different port
export PANAVERSITY_SERVER_PORT=8080
```

#### 2. "Database migration failed"

```bash
# Check current state
uv run alembic current

# Force sync (destructive!)
uv run alembic stamp head
uv run alembic upgrade head
```

#### 3. "S3 access denied"

```bash
# Verify credentials
aws s3 ls --endpoint-url=$PANAVERSITY_S3_ENDPOINT s3://$PANAVERSITY_S3_BUCKET

# Check IAM permissions
# Needs: s3:GetObject, s3:PutObject, s3:DeleteObject, s3:ListBucket
```

#### 4. "Archive generation timeout"

```bash
# Check book size
find /path/to/book -type f | wc -l
du -sh /path/to/book

# Increase timeout
export PANAVERSITY_ARCHIVE_TIMEOUT=120
```

#### 5. "Hash chain broken in audit log"

```sql
-- Find broken entries
SELECT a.*
FROM audit_log a
LEFT JOIN audit_log b ON a.prev_hash = b.entry_hash
WHERE a.prev_hash IS NOT NULL AND b.id IS NULL;
```

### Debug Mode

```bash
# Enable debug logging
export PANAVERSITY_LOG_LEVEL=DEBUG

# Run with verbose output
uv run python -m panaversity_fs.server 2>&1 | tee server.log
```

## Scaling

### Horizontal Scaling

PanaversityFS is stateless (except for database):

```
                    ┌─────────────────┐
                    │  Load Balancer  │
                    └────────┬────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
    ┌────▼────┐         ┌────▼────┐         ┌────▼────┐
    │ Server  │         │ Server  │         │ Server  │
    │   #1    │         │   #2    │         │   #3    │
    └────┬────┘         └────┬────┘         └────┬────┘
         │                   │                   │
         └───────────────────┴───────────────────┘
                             │
                    ┌────────▼────────┐
                    │   PostgreSQL    │
                    │   (shared DB)   │
                    └─────────────────┘
```

### Connection Pooling

```python
# In config.py or environment
DATABASE_POOL_SIZE=20
DATABASE_MAX_OVERFLOW=10
DATABASE_POOL_TIMEOUT=30
```

### Caching

Consider adding Redis for:
- Session caching
- Frequently accessed content
- Rate limiting

## Security Checklist

- [ ] JWT authentication enabled in production
- [ ] HTTPS enforced (TLS termination at load balancer)
- [ ] Database credentials in secrets manager
- [ ] S3/R2 credentials with minimal permissions
- [ ] Audit logging enabled
- [ ] Rate limiting configured
- [ ] CORS configured for allowed origins
- [ ] Input validation on all tools (Pydantic)
- [ ] Path traversal protection verified
- [ ] Error messages don't leak internals

## Success Criteria Verification

Regularly verify that success criteria from the spec are being met:

| Criteria | Metric/Check | Target |
|----------|--------------|--------|
| SC-001 | `panaversityfs_archive_duration_seconds` | p99 < 60s |
| SC-001 | `panaversityfs_archive_memory_bytes` | max < 64MB |
| SC-002 | `storage_rollback_count` metric | 0 orphans |
| SC-003 | `panaversityfs_write_total{status="conflict"}` | 100% detection |
| SC-004 | Audit chain health check | all valid |
| SC-005 | `panaversityfs_write_total{status="schema_violation"}` | 0 in prod |
| SC-006 | Overlay latency histogram | p95 < 10ms |
| SC-009 | Audit log query: `agent_id != 'system'` | 100% |
| SC-010 | R2 operation success rate | 99.9%+ |

## Maintenance Tasks

### Daily
- Check error rate in metrics (SC-003, SC-010)
- Review audit log for anomalies (SC-009)
- Verify backup completion

### Weekly
- Verify audit hash chain integrity (SC-004, R6)
- Review slow query logs
- Check disk space on database
- Update dependency security patches

### Monthly
- Full SC-001 to SC-010 validation
- Rotate credentials
- Review access permissions
- Capacity planning review
- Dependency updates (`uv update`)
