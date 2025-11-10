# Telemetry Collection Workflow: Local-First → Optional Cloud

**Feature**: 017-usage-data-collection  
**Date**: 2025-01-10  
**Architecture**: Phased deployment (local individual → optional centralized)

---

## Overview

This document describes the **phased approach** to telemetry collection for the AI Native Software Development book project:

1. **Phase 1 (NOW)**: Each team member runs local telemetry collection
2. **Phase 2 (LATER)**: Optional centralization to shared cloud endpoint
3. **Phase 3 (OPTIONAL)**: Data aggregation and team-wide analysis

**Key Principle**: Start local, centralize when satisfied.

---

## Phase 1: Local Collection (Current Approach)

### Architecture

Each team member runs their own telemetry stack **locally on their machine**:

```
Team Member A (Laptop)              Team Member B (Laptop)
┌─────────────────────────┐        ┌─────────────────────────┐
│ Claude Code             │        │ Claude Code             │
│   ↓ OTLP export         │        │   ↓ OTLP export         │
│ OTLP Collector          │        │ OTLP Collector          │
│   ↓ store events        │        │   ↓ store events        │
│ ClickHouse (Docker)     │        │ ClickHouse (Docker)     │
│   localhost:8123        │        │   localhost:8123        │
└─────────────────────────┘        └─────────────────────────┘
         ↓                                  ↓
    Query own data                     Query own data
    (personal analytics)               (personal analytics)
```

### Setup (Per Team Member)

**1. Start local telemetry server**:
```bash
cd telemetry-server/
docker-compose up -d
```

**2. Configure Claude Code**:
```bash
# In your shell profile (~/.bashrc, ~/.zshrc, etc.)
export CLAUDE_CODE_ENABLE_TELEMETRY=1
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317
```

**3. Verify collection**:
```bash
# Run a Claude Code session
claude-code

# Check that events are being collected
curl -G 'http://localhost:8123' --data-urlencode "query=SELECT count() FROM telemetry_events FORMAT Pretty"
```

### Data Storage (Local)

Each team member's data is stored in **local Docker volumes**:

```
telemetry-server/
├── docker-compose.yml
└── data/                    # NOT in git (in .gitignore)
    ├── clickhouse/          # ClickHouse database files
    │   └── *.db
    └── otel-logs/           # OTLP Collector logs
        └── *.log
```

**Important**: This data is NOT committed to git and NOT shared with other team members automatically.

### Advantages of Local-First

✅ **Privacy**: Your data stays on your machine  
✅ **No dependencies**: Works offline, no shared infrastructure needed  
✅ **Fast iteration**: Immediate feedback, no network latency  
✅ **Safe testing**: Experiment without affecting team data  
✅ **Low cost**: No cloud hosting costs during development  

### Querying Your Own Data

Each team member can analyze their own usage:

```bash
# Cost analysis (your sessions only)
curl -G 'http://localhost:8123' --data-urlencode "query=
SELECT
    toDate(timestamp) as date,
    sum(cost_usd) as total_cost,
    count() as request_count
FROM telemetry_events
WHERE event_type = 'api_request'
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY date
ORDER BY date DESC
FORMAT Pretty"
```

---

## Phase 2: Optional Centralization (Future)

### When to Centralize

Move to centralized collection when:

✅ Team wants cross-member analysis (compare productivity patterns)  
✅ Project coordinator needs cost tracking across all contributors  
✅ Error analysis requires identifying patterns across multiple users  
✅ You're satisfied with local collection stability  

**Don't centralize if**:
❌ Still testing/debugging the telemetry setup  
❌ Team members want data privacy  
❌ No need for cross-team analytics yet  

### Architecture (Centralized)

```
Team Member A              Team Member B              Team Member C
┌──────────────┐          ┌──────────────┐          ┌──────────────┐
│ Claude Code  │          │ Claude Code  │          │ Claude Code  │
│   ↓ OTLP     │          │   ↓ OTLP     │          │   ↓ OTLP     │
└──────────────┘          └──────────────┘          └──────────────┘
         │                         │                         │
         └─────────────────────────┴─────────────────────────┘
                                   ↓
                    Shared OTLP Collector (Cloud)
                         https://telemetry.yourproject.com:4317
                                   ↓
                    Centralized ClickHouse Database
                                   ↓
              ┌────────────────────┴────────────────────┐
              ↓                                         ↓
    Team Coordinator                          Individual Contributors
    (All data, aggregated)                    (Own data only)
```

### Setup (Centralized)

**Option A: Cloud VM (Recommended for teams)**

Deploy telemetry infrastructure to a cloud provider:

```bash
# 1. Provision a VM (e.g., AWS EC2, DigitalOcean Droplet, Azure VM)
#    Recommended: 2 CPU, 4GB RAM, 50GB disk

# 2. Install Docker and Docker Compose on the VM

# 3. Clone telemetry-server/ to the VM
scp -r telemetry-server/ user@telemetry-vm:/opt/

# 4. Configure firewall to allow port 4317 (OTLP) and 8123 (HTTP queries)

# 5. Start services
ssh user@telemetry-vm
cd /opt/telemetry-server/
docker-compose up -d

# 6. Update team members' .env to point to cloud endpoint
export OTEL_EXPORTER_OTLP_ENDPOINT=https://telemetry.yourproject.com:4317
```

**Option B: Docker Compose on shared server**

If you have a shared development server:

```bash
# Run on the shared server
cd /shared/telemetry-server/
docker-compose up -d

# Team members configure their Claude Code
export OTEL_EXPORTER_OTLP_ENDPOINT=http://dev-server.local:4317
```

### Data Sharing Workflow

**How team members transition from local → cloud**:

1. **Export local data** (optional, if you want to preserve history):
```bash
# Dump local data to CSV
curl -G 'http://localhost:8123' --data-urlencode "query=
SELECT * FROM telemetry_events FORMAT CSVWithNames" > my-local-data.csv

# Import to centralized database (on cloud VM)
cat my-local-data.csv | ssh user@telemetry-vm "clickhouse-client --query='INSERT INTO telemetry_events FORMAT CSVWithNames'"
```

2. **Reconfigure Claude Code** to point to cloud endpoint:
```bash
# Update .bashrc / .zshrc
export OTEL_EXPORTER_OTLP_ENDPOINT=https://telemetry.yourproject.com:4317

# Reload shell
source ~/.bashrc
```

3. **Stop local Docker services** (optional, saves resources):
```bash
cd telemetry-server/
docker-compose down
```

4. **Verify cloud collection**:
```bash
# Query centralized database
curl -G 'https://telemetry.yourproject.com:8123' --data-urlencode "query=
SELECT user_id, count() as event_count FROM telemetry_events GROUP BY user_id FORMAT Pretty"
```

### Access Control (Centralized)

When using centralized collection, implement role-based access:

| Role | Can View | Can Modify | Can Delete |
|------|----------|------------|------------|
| **Individual Contributor** | Own data only | No | Own data only (via request) |
| **Project Coordinator** | All aggregated data | No | Via retention policy |
| **Administrator** | All data | Schema only | Via retention policy |

**Implementation**:
- Use ClickHouse user accounts with row-level security
- Filter queries by `user_id` for contributors
- Allow unrestricted queries for coordinators

---

## Phase 3: Data Aggregation & Analysis (Optional)

### Cross-Team Analytics

Once centralized, enable team-wide insights:

**Cost Analysis**:
```sql
-- Total cost by team member (last 30 days)
SELECT
    user_id,
    sum(cost_usd) as total_cost,
    count() as request_count,
    avg(cost_usd) as avg_cost_per_request
FROM telemetry_events
WHERE event_type = 'api_request'
  AND timestamp >= now() - INTERVAL 30 DAY
GROUP BY user_id
ORDER BY total_cost DESC
```

**Error Pattern Analysis**:
```sql
-- Most common errors across team
SELECT
    error_type,
    count() as occurrence_count,
    uniq(user_id) as affected_users
FROM telemetry_events
WHERE event_type = 'api_error'
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY error_type
ORDER BY occurrence_count DESC
LIMIT 10
```

**Chapter Complexity Analysis**:
```sql
-- Which chapters require most AI assistance (by token usage)?
SELECT
    chapter_context,
    sum(token_count) as total_tokens,
    count(DISTINCT user_id) as contributors,
    avg(token_count) as avg_tokens_per_request
FROM telemetry_events
WHERE event_type = 'api_request'
  AND chapter_context != ''
GROUP BY chapter_context
ORDER BY total_tokens DESC
```

### Automated Reports

Generate weekly/monthly reports for project coordinator:

```bash
#!/bin/bash
# scripts/generate-weekly-report.sh

echo "Telemetry Report: $(date +%Y-%m-%d)"
echo "=================================="

# Total cost
echo "Total Cost (Last 7 Days):"
curl -G 'https://telemetry.yourproject.com:8123' --data-urlencode "query=
SELECT sum(cost_usd) FROM telemetry_events WHERE timestamp >= now() - INTERVAL 7 DAY FORMAT Pretty"

# Top contributors
echo "Top Contributors (by activity):"
curl -G 'https://telemetry.yourproject.com:8123' --data-urlencode "query=
SELECT user_id, count() as events FROM telemetry_events WHERE timestamp >= now() - INTERVAL 7 DAY GROUP BY user_id ORDER BY events DESC LIMIT 5 FORMAT Pretty"

# Error rate
echo "Error Rate:"
curl -G 'https://telemetry.yourproject.com:8123' --data-urlencode "query=
SELECT count() as errors, (SELECT count() FROM telemetry_events WHERE timestamp >= now() - INTERVAL 7 DAY) as total, (errors / total) * 100 as error_percentage FROM telemetry_events WHERE event_type = 'api_error' AND timestamp >= now() - INTERVAL 7 DAY FORMAT Pretty"
```

---

## Data Retention & Cleanup

Regardless of deployment (local or cloud), apply retention policies:

**Automatic Cleanup** (configured in ClickHouse schema):
- Raw events: 90 days (TTL)
- Session summaries: 1 year
- Aggregated metrics: Indefinite

**Manual Export** (before cleanup):
```bash
# Archive old data before automatic deletion
curl -G 'http://localhost:8123' --data-urlencode "query=
SELECT * FROM telemetry_events 
WHERE timestamp < now() - INTERVAL 85 DAY
FORMAT Parquet" > archive-2025-01.parquet
```

---

## Decision Tree: Should I Centralize?

Use this decision tree to determine if you should move from local to centralized collection:

```
Start: Using local telemetry collection
  ↓
Q1: Do you need cross-team analytics?
  ├─ NO → Stay local (Phase 1)
  └─ YES → Q2: Is local collection working reliably?
      ├─ NO → Fix local issues first, stay in Phase 1
      └─ YES → Q3: Do team members consent to data sharing?
          ├─ NO → Stay local, respect privacy
          └─ YES → Q4: Can you provision cloud infrastructure?
              ├─ NO → Stay local until budget/resources available
              └─ YES → Proceed to Phase 2 (Centralization)
```

---

## Security Considerations

### Local Collection (Phase 1)
- Data never leaves your machine
- Accessible only via localhost
- Protected by OS-level user permissions

### Centralized Collection (Phase 2)
- **Transport**: Use HTTPS/TLS for OTLP endpoint (not plain HTTP)
- **Authentication**: Implement OTLP authentication (API keys or OAuth)
- **Firewall**: Restrict port 4317 to team IP addresses
- **Access control**: ClickHouse user accounts with row-level security
- **Encryption at rest**: Enable ClickHouse encryption for sensitive fields

**Example: OTLP with TLS**:
```yaml
# docker-compose.yml (cloud deployment)
services:
  otel-collector:
    environment:
      - OTEL_EXPORTER_OTLP_ENDPOINT=https://0.0.0.0:4317
      - OTEL_EXPORTER_OTLP_CERTIFICATE=/certs/server.crt
      - OTEL_EXPORTER_OTLP_KEY=/certs/server.key
    volumes:
      - ./certs:/certs
```

---

## Cost Estimation

### Local Collection (Phase 1)
- **Infrastructure cost**: $0 (uses your local machine)
- **Disk space**: ~100MB for 30 days of typical usage
- **Performance impact**: Negligible (<1% CPU, <100MB RAM)

### Centralized Collection (Phase 2)
- **Cloud VM**: $10-30/month (2 CPU, 4GB RAM, 50GB disk)
  - AWS EC2 t3.medium: ~$30/month
  - DigitalOcean Droplet: ~$18/month
  - Azure B2s: ~$30/month
- **Network egress**: Minimal (<1GB/month for telemetry data)
- **Disk space**: ~1GB per team member per month (with compression)

**Scaling Example**:
- 10 team members × 100MB/month = 1GB total
- 90-day retention × 1GB/month = 3GB storage needed
- Recommended: 50GB disk = ~15 months of data before scaling

---

## Migration Checklist

### Moving from Local → Cloud

- [ ] Provision cloud VM or shared server
- [ ] Install Docker and Docker Compose
- [ ] Copy `telemetry-server/` to cloud VM
- [ ] Configure firewall rules (ports 4317, 8123)
- [ ] Set up TLS certificates (Let's Encrypt recommended)
- [ ] Configure OTLP authentication (API keys or OAuth)
- [ ] Test connection from one team member's machine
- [ ] Export local data (optional, if preserving history)
- [ ] Update team members' environment variables
- [ ] Verify centralized collection for all team members
- [ ] Configure automated backups
- [ ] Set up monitoring (Prometheus, Grafana)
- [ ] Document access procedures for team

---

## Rollback Plan

If centralized collection has issues, revert to local:

```bash
# 1. Stop centralized services
ssh user@telemetry-vm "cd /opt/telemetry-server/ && docker-compose down"

# 2. Team members reconfigure to local
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317

# 3. Restart local services
cd telemetry-server/
docker-compose up -d

# 4. Optionally export centralized data to local
curl -G 'https://telemetry.yourproject.com:8123' --data-urlencode "query=
SELECT * FROM telemetry_events WHERE user_id = 'YOUR_USER_ID' FORMAT CSVWithNames" > my-cloud-data.csv

# Import to local ClickHouse
cat my-cloud-data.csv | docker exec -i $(docker ps -qf "name=clickhouse") \
  clickhouse-client --query="INSERT INTO telemetry_events FORMAT CSVWithNames"
```

---

## Frequently Asked Questions

### Can I use both local AND cloud collection simultaneously?
**Yes**, but not recommended. OTLP supports multiple exporters, so you could configure:
```bash
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317,https://telemetry.yourproject.com:4317
```
However, this duplicates data and may cause confusion. Better to choose one.

### How do I share my local data with the team before centralizing?
**Option 1**: Export to CSV and share via Slack/email:
```bash
curl -G 'http://localhost:8123' --data-urlencode "query=
SELECT * FROM telemetry_events FORMAT CSVWithNames" > my-data.csv
```

**Option 2**: Export aggregated report (no raw data):
```bash
curl -G 'http://localhost:8123' --data-urlencode "query=
SELECT
    toDate(timestamp) as date,
    count() as events,
    sum(cost_usd) as cost
FROM telemetry_events
GROUP BY date
FORMAT Pretty" > my-summary.txt
```

### What if I want to keep some data local and share other data?
Implement **data filtering** in OTLP collector config:
```yaml
# collector-config.yaml
processors:
  filter/sensitive:
    logs:
      exclude:
        match_type: regexp
        record_attributes:
          - key: event_type
            value: "(user_prompt|api_error)"  # Keep prompts local
exporters:
  otlp/local:
    endpoint: localhost:4317
  otlp/cloud:
    endpoint: telemetry.yourproject.com:4317

service:
  pipelines:
    logs:
      receivers: [otlp]
      processors: []
      exporters: [otlp/local]  # All data to local
    logs/filtered:
      receivers: [otlp]
      processors: [filter/sensitive]
      exporters: [otlp/cloud]  # Filtered data to cloud
```

### How do I delete my data from centralized storage?
**Individual contributors**:
```sql
-- Request deletion from project coordinator
-- They will run:
ALTER TABLE telemetry_events DELETE WHERE user_id = 'YOUR_USER_ID';
```

**Coordinators**: See `DATA-USAGE-POLICY` for deletion procedures and timelines (30-day SLA).

---

## Summary

| Phase | When | Architecture | Access | Cost |
|-------|------|--------------|--------|------|
| **Phase 1: Local** | NOW (default) | Each member runs own stack | Own data only | $0 |
| **Phase 2: Cloud** | When satisfied + need cross-team analytics | Shared cloud endpoint | Role-based | $10-30/month |
| **Phase 3: Analytics** | After Phase 2 stable | Automated reports, dashboards | Coordinator-led | Included in Phase 2 |

**Recommendation**: Start with Phase 1 (local), validate stability for 2-4 weeks, then decide if Phase 2 (cloud) is needed.

---

**Last Updated**: 2025-01-10  
**Panaversity Project**
