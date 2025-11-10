# Research: Usage Data Collection System (Feature 017)

**Phase**: 0 - Research | **Date**: 2025-11-10 | **Status**: Complete

## Executive Summary

This research validates technology choices for Feature 017 (usage data collection). We recommend:
- **OTLP Collector** (OpenTelemetry Community) for telemetry ingestion
- **ClickHouse** for time-series storage and analysis
- **Docker Compose** for local infrastructure orchestration

This combination provides production-grade telemetry infrastructure that runs entirely locally, requires minimal operational overhead, and scales from single developer to 10+ team members without architectural changes.

---

## 1. OTLP Collector Comparison

### OpenTelemetry Collector (Recommended)

**Project**: https://github.com/open-telemetry/opentelemetry-collector  
**License**: Apache 2.0 (permissive)  
**Maturity**: CNCF Graduated Project (stable)  
**Active Development**: Yes (monthly releases)

**Architecture**:
- Receiver (accepts telemetry in OTLP, JAEGER, Prometheus formats)
- Processor (transforms, filters, samples events)
- Exporter (sends to time-series databases, logging systems)
- Configuration: YAML-based, Hot-reloadable

**Strengths**:
- Standard OTLP protocol ensures Claude Code compatibility
- Flexible receiver/processor/exporter pipelines
- Lightweight distribution available (50MB minimal image)
- Excellent documentation and examples
- CNCF backing ensures long-term support
- Can export to ClickHouse, Prometheus, Jaeger, cloud services simultaneously
- Sampling support (probabilistic, head-based) for high-volume scenarios

**Weaknesses**:
- Requires configuration file (though simple for basic setup)
- Minimal UI (metrics exported to separate visualization tools)
- Moderate resource footprint (150-300MB with Go runtime)
- Learning curve for processor customization

**Cloud/Local Trade-off**:
- Local: ✅ Fully self-contained, no cloud dependency, data ownership
- Local resources: 150-300MB memory, 50-100MB disk for collector

**Recommendation**: PRIMARY CHOICE
- Proven at scale (Anthropic, OpenAI, Google use OpenTelemetry)
- Aligns with OTLP standard mentioned in spec
- Flexible exporter architecture (support Prometheus later if needed)

---

### Jaeger All-in-One (Alternative)

**Project**: https://www.jaegertracing.io/  
**License**: Apache 2.0  
**Maturity**: CNCF Graduated Project (stable)  
**Active Development**: Yes

**Architecture**:
- All components in single binary (agent, collector, database, UI)
- UI dashboard included (distributed trace visualization)
- Supports multiple backends (Elasticsearch, Cassandra, Badger)

**Strengths**:
- Zero-configuration all-in-one deployment
- Beautiful UI for exploring traces
- Excellent for understanding request flows
- Single docker command: `docker run jaegertracing/all-in-one`
- Lower operational complexity for trace visualization

**Weaknesses**:
- Optimized for traces (request tracing), not events or metrics
- Heavier footprint (~1.5GB Docker image)
- Less flexible for custom event types or aggregations
- Limited for cost analysis (events != traces)

**When to Use**:
- Teams that prioritize trace visualization
- Smaller teams (<5 people) who want minimal config
- Research-phase exploration of what telemetry looks like

**Not Recommended for This Project Because**:
- Feature 017 requires event analysis (prompts, tool calls, costs) not trace visualization
- Error analysis (SR-3 in spec) needs aggregation queries across events, not trace following
- ClickHouse + Collector better suits cost/token analysis use cases

---

### Grafana Tempo (Alternative)

**Project**: https://grafana.com/oss/tempo/  
**License**: AGPL v3.0 + Grafana Labs proprietary  
**Maturity**: Production-ready (Grafana product)  
**Active Development**: Yes

**Strengths**:
- Integrates seamlessly with Grafana dashboards
- Scales to high-volume tracing (cloud-native)
- No index (simpler operations)
- Cost-effective at scale

**Weaknesses**:
- AGPL licensing requires careful review for commercial deployments
- More complex local setup (multiple components)
- Designed for cloud deployment, not local development
- Heavier footprint
- Still trace-focused, not event-analysis focused

**When to Use**:
- Teams already using Grafana + Prometheus stack
- Enterprise deployments with cloud infrastructure
- Later phase if dashboards required (Part 10+)

**Not Recommended for Feature 017** Because local team setup is too complex

---

## 2. Time-Series Database Comparison

### ClickHouse (Recommended)

**Project**: https://github.com/ClickHouse/ClickHouse  
**License**: BSL 1.1 (Business Source License) + community edition, SSPL-like  
**Maturity**: Yandex/Company production system (15+ years)  
**Active Development**: Yes (weekly releases)

**Architecture**:
- Column-oriented OLAP database
- Distributed INSERT performance (1M+ rows/sec single node)
- SQL dialect (familiar to most developers)
- Compression: 10-100x typical rate depending on data

**Strengths**:
- Exceptional INSERT performance for telemetry ingestion
- Powerful aggregation queries (GROUP BY, time-bucketing)
- Schema-flexible (can evolve event types without migrations)
- Excellent compression (telemetry data compresses 10-20x)
- Docker image small-ish (1.5GB)
- Perfect for "structured logs" use case (events with attributes)
- Can handle 1M+ events/day per team easily
- Supports both PromQL and native SQL

**Weaknesses**:
- BSL licensing requires clarity (OK for internal use, not for SaaS)
- Steeper learning curve (SQL dialect slightly different from PostgreSQL)
- Overkill for small teams (<5 people, <1k events/day)
- Memory footprint larger than lightweight alternatives

**Licensing Analysis**:
- BSL 1.1: Free for community/internal use; commercial restrictions for SaaS/services
- This project (internal telemetry): ✅ FULLY COMPLIANT
- Example: Cannot sell "ClickHouse-as-a-Service"; can use ClickHouse internally
- SSPL concern: Mitigated by providing alternative (Prometheus) if needed

**Data Volumes**:
- 5 developers, 100-500 events/session, 50 sessions/week = 5k-10k events/week
- Annual: ~260-520k events
- Compressed in ClickHouse: ~1-5 MB (vs 10-50 MB uncompressed)
- Well within single-node capacity (tested to 100M+ events/node)

**Cost Analysis** (why ClickHouse shines for this project):
```sql
-- Cost by chapter (aggregation query)
SELECT chapter_id, SUM(cost_usd) as total_cost, COUNT(*) as event_count
FROM telemetry_events
WHERE timestamp > now() - INTERVAL 30 day
GROUP BY chapter_id
ORDER BY total_cost DESC

-- Error patterns (event analysis)
SELECT error_type, COUNT(*) as occurrences, 
       arrayMap(x -> x.session_id, groupArray(session_id)) as affected_sessions
FROM telemetry_events
WHERE event_type = 'api_error'
GROUP BY error_type
ORDER BY occurrences DESC LIMIT 10
```

**Recommendation**: PRIMARY CHOICE
- Proven at scale (used by Yandex for 100B+ events/day)
- Perfect data structure for telemetry events
- Cost and error analysis queries are natural in ClickHouse

---

### Prometheus (Alternative)

**Project**: https://prometheus.io/  
**License**: Apache 2.0  
**Maturity**: CNCF Graduated Project  
**Active Development**: Yes

**Architecture**:
- Pull-based metrics system (targets expose HTTP endpoints)
- Time-series storage (optimized for metrics, not events)
- PromQL query language (powerful for aggregations)
- Retention policies (time-based pruning)

**Strengths**:
- Lightweight (~150MB), minimal resources
- Excellent for metrics (avg response time, error rate, throughput)
- Industry standard (used by Kubernetes, all major cloud providers)
- Simple to get running
- Integrates with Grafana dashboards
- Apache 2.0 license (unambiguous)

**Weaknesses**:
- Not designed for event data (prompts, error messages, contexts)
- Limited dimension cardinality (high-cardinality attributes cause memory bloat)
- No support for individual event exploration (only aggregated metrics)
- Cannot store prompt text, error details, or rich event metadata
- Pull-based model (awkward for push scenarios like agent telemetry)

**When to Use**:
- Teams that already run Prometheus
- Metrics-only approach (cost/token aggregates only, no error analysis)
- Lighter resource footprint required
- Smaller teams

**Not Recommended for Feature 017** Because:
- Error analysis (User Story 3) requires event-level drill-down
- Cannot store prompt text or detailed error context
- Workflow trace review (spec requirement) needs event-level data

**Can Revisit**: If feature scope shrinks to "metrics only," Prometheus is viable. Otherwise, ClickHouse is better.

---

### TimescaleDB (Alternative)

**Project**: https://www.timescale.com/  
**License**: Timescale License + Apache 2.0 community edition  
**Maturity**: Production-ready (Timescale Inc. backed)  
**Active Development**: Yes

**Architecture**:
- PostgreSQL extension (hypertables for time-series)
- Familiar SQL interface
- ACID guarantees
- Compression plugin available

**Strengths**:
- Familiar PostgreSQL environment
- ACID transactions (good for correctness)
- Reasonable resource footprint
- Compression available (10x+)
- Good for event-based data

**Weaknesses**:
- Slower INSERT throughput than ClickHouse (100k-300k/sec vs 1M+/sec)
- Smaller ecosystem (fewer tools, less community knowledge)
- Heavier than Prometheus but less optimized than ClickHouse
- License complexity (community vs enterprise)
- Licensing cost if advanced features needed

**When to Use**:
- Teams comfortable with PostgreSQL
- ACID transaction requirements
- Smaller data volumes (<100k events/week)

**Not Recommended for Feature 017** Because ClickHouse outperforms for telemetry use case.

**Can Consider**: As alternative if team wants PostgreSQL ecosystem; requires different schema and queries than ClickHouse.

---

### MongoDB Time-Series Collections (Not Recommended)

**Project**: https://www.mongodb.com/  
**License**: SSPL  
**Maturity**: Production-ready

**Why Not**:
- SSPL licensing (SaaS concerns, similar to ClickHouse)
- Higher storage overhead (JSON documents less dense than columnar)
- Slower aggregation queries than ClickHouse
- Less specialized for time-series operations
- No advantage over ClickHouse for this use case

---

## 3. Docker Orchestration Patterns

### Docker Compose (Recommended)

**Tool**: Docker Compose (v2.0+, officially supported)  
**License**: Apache 2.0  
**Best For**: Single-host multi-container coordination

**Example**: `docker-compose.yml`
```yaml
version: '3.8'
services:
  otel-collector:
    image: otel/opentelemetry-collector:0.95.0
    ports:
      - "4317:4317"  # OTLP gRPC
      - "4318:4318"  # OTLP HTTP
    volumes:
      - ./collector-config.yaml:/etc/otel-collector-config.yaml
    environment:
      - OTEL_CONFIG_YAML=/etc/otel-collector-config.yaml
    command: ["--config=/etc/otel-collector-config.yaml"]

  clickhouse:
    image: clickhouse/clickhouse-server:latest
    ports:
      - "8123:8123"  # HTTP interface
      - "9000:9000"  # Native protocol
    volumes:
      - ./clickhouse/schema.sql:/docker-entrypoint-initdb.d/schema.sql
      - clickhouse_data:/var/lib/clickhouse
    environment:
      - CLICKHOUSE_DB=telemetry

volumes:
  clickhouse_data:
```

**Strengths**:
- Single command to start infrastructure: `docker-compose up -d`
- All configuration in one file
- Volume management for persistent data
- Environment variable support
- Works on Mac, Linux, WSL
- Officially supported (Docker Inc. maintained)
- Perfect for local development

**Weaknesses**:
- Single-host only (doesn't scale to multiple servers)
- No orchestration logic (manual failover, rolling updates)
- Not for production clusters (though fine for local team telemetry)

**Recommendation**: PRIMARY CHOICE for this project
- Local team infrastructure (5-10 developers)
- No multi-host requirements
- Simplest operational model

---

### Kubernetes (Future Consideration)

**When to Use**: Parts 10-13 when teaching production deployment

**For Feature 017**: Not needed yet
- Local team scale doesn't justify Kubernetes
- Can be added later if team grows to 50+ developers
- Current docker-compose can be converted to Helm charts if needed

---

### Podman Compose (Alternative)

**Tool**: Podman Compose (drop-in Docker Compose replacement)  
**License**: Apache 2.0  
**Best For**: Kubernetes-first organizations, systemd integration

**Strengths**:
- Rootless containers (security benefit)
- Daemonless (no Podman daemon required)
- Kubernetes-compatible (Podman pods map to K8s pods)
- Drop-in replacement for Docker Compose

**Weaknesses**:
- Smaller ecosystem
- Fewer examples and community resources
- Slightly slower adoption

**When to Use**: Teams using Podman instead of Docker Engine

**Recommendation**: Support as alternative
- Document both Docker Compose and Podman Compose in quickstart.md
- Provide compatibility notes

---

## 4. OpenTelemetry Best Practices

### Semantic Conventions

**Reference**: https://opentelemetry.io/docs/specs/semconv/

**Application to Feature 017**:
- Use standard attribute names (e.g., `messaging.message_id` → `session_id`)
- Define custom attributes for project-specific data (e.g., `chapter_id`, `feature_number`)
- Document all custom attributes in data-model.md

**Example Schema**:
```json
{
  "name": "user_prompt",
  "attributes": {
    "session.id": "uuid",                      // Session ID
    "user.id": "string",                       // User identifier
    "organization.id": "string",               // Project/organization
    "code.version": "string",                  // Claude Code version
    "prompt.length": "int",                    // Character count
    "prompt.tokens_estimated": "int",          // Token estimate
    "git.branch": "string",                    // Active git branch
    "feature.id": "string",                    // Feature spec directory
    "chapter.number": "int",                   // Chapter being worked on
    "timestamp": "timestamp"                   // ISO 8601 timestamp
  }
}
```

### Sampling Strategies

**Use Case**: Teams with 5-10 developers
- No sampling needed (1M events/year fits easily in ClickHouse)
- Can add sampling later if volume exceeds 100M events/year

**Sampling Implementation** (if needed):
- **Probabilistic**: Sample 10% of events (good for high-volume services)
- **Head-based**: Sample based on session start (good for trace continuity)

**Configuration Example**:
```yaml
processors:
  probabilistic_sampler:
    sampling_percentage: 100  # Start at 100%, reduce if needed
```

### Context Propagation

**Goal**: All events within a session share session ID, user ID, and chapter context

**Implementation Approach**:
- Environment variables: `OTEL_SESSION_ID`, `OTEL_USER_ID`, `OTEL_CHAPTER`
- Config file: Automatically populated when Claude Code CLI starts
- Benefits: Events are automatically grouped for analysis

**Example**:
```bash
export OTEL_SESSION_ID="$(uuidgen)"
export OTEL_USER_ID="alice@example.com"
export OTEL_CHAPTER="12"  # Chapter number or feature number
```

### Graceful Degradation

**Requirement** (from spec FR-010): "Buffer telemetry data locally if backend is unreachable"

**Implementation Approaches**:

1. **File-based buffering** (recommended):
   - Export to local JSON Lines file if network unavailable
   - Periodically retry export to backend
   - On success, delete buffer file
   - Advantages: Resilient, simple, visible (team can see buffered events)

2. **Memory buffering**:
   - Accumulate events in memory
   - Export batch on interval or threshold
   - Advantages: Fast, minimal disk I/O
   - Disadvantages: Loss of data if process crashes

3. **Ring buffer**:
   - Fixed-size circular buffer
   - Oldest events overwritten if backend unavailable
   - Advantages: Bounded memory, recent events prioritized
   - Disadvantages: Lose old data

**Recommendation**: Hybrid approach
- Memory buffer for current session (bounded, <10k events)
- File buffer for persistence (if memory buffer exceeds capacity or network down)
- Retry with exponential backoff (1s, 2s, 4s, 8s, stop after 5 retries)
- Log buffer status to console (transparency for users)

---

## 5. Data Sanitization Patterns for Open Source Projects

### Sensitive Data Types to Filter

**API Keys & Credentials**:
- Pattern: `api_key=[a-z0-9]{32,}`, `token=...`, `Authorization: Bearer ...`
- Action: Replace with `[REDACTED_API_KEY]`

**Personally Identifiable Information (PII)**:
- Email addresses: `[a-z0-9+.-]+@[a-z0-9.-]+\.[a-z]{2,}`
- Phone numbers: `\d{3}-\d{3}-\d{4}`
- Names: Optional (can be extracted from git config; decide on policy)
- Action: Hash or remove before export

**Proprietary Code**:
- File paths with project-specific info
- Secret variable names or comments
- Action: Configurable patterns (team defines what to exclude)

**Example Sanitization Policy**:
```yaml
# Data Sanitization Configuration
filters:
  - type: regex
    pattern: '(api_key|token|Authorization)=[^ ]+'
    replacement: '[REDACTED_API_KEY]'
  - type: regex
    pattern: '[a-z0-9+.-]+@[a-z0-9.-]+\.[a-z]{2,}'
    replacement: '[REDACTED_EMAIL]'
  - type: pii_hash
    fields: [user.email, user.name]
    hash_algorithm: sha256
  - type: path_filter
    patterns: ['*/proprietary/*', '*/secret/*']
    action: exclude
```

### Implementation Points

**1. At Collector Level** (recommended):
- OpenTelemetry Collector processors can filter attributes
- Clean data before it leaves the agent
- Example processor: `attributes` processor with regex patterns

**2. At Export Level**:
- Filter in the exporter (before ClickHouse insert)
- Can catch patterns specific to the target backend
- Fine-grained control per exporter

**3. At Database Level**:
- Filter on INSERT (views, triggers)
- Last-line defense
- Can audit what was filtered

**Recommendation**: Combine #1 (collector processor) + #2 (exporter filter)
- Defense in depth
- Transparent (teams can see what's being filtered)

---

## 6. Cross-Platform Testing Strategy

### Test Matrix

| Platform | OS | Docker | Testing |
|----------|----|---------|----|
| macOS Intel | Monterey+ | Docker Desktop 4.1+ | Primary |
| macOS Apple Silicon | Monterey+ | Docker Desktop 4.1+ | Primary |
| Linux | Ubuntu 22.04+ | Docker Engine 20.10+ | Primary |
| WSL 2 | Windows 11+ | Docker Desktop WSL2 | Primary |
| WSL 1 | Windows 10+ | Limited support | Secondary |

### Network Scenarios

**Corporate Environment**:
- Issue: Proxy requirements, firewall blocking localhost:4317
- Mitigation: Document proxy configuration in quickstart.md
- Test: Behind proxy (if available)

**Offline Scenario**:
- Issue: Backend unreachable (no network)
- Mitigation: File-based buffering (see "Graceful Degradation" above)
- Test: Manually kill collector, verify buffering works

**High-Volume Scenario**:
- Issue: 1000+ events in single session
- Mitigation: Batch export, sampling, compression
- Test: Scripted generation of high-volume session data

### Docker Resource Limits

**Minimum Requirements**:
- RAM: 4GB total system
- Disk: 10GB free space (buffer for data growth)
- CPU: 2 cores (shared)

**Docker Compose Resource Settings** (example):
```yaml
services:
  otel-collector:
    resources:
      limits:
        cpus: '0.5'
        memory: 256M
      reservations:
        cpus: '0.25'
        memory: 128M

  clickhouse:
    resources:
      limits:
        cpus: '1.0'
        memory: 1G
      reservations:
        cpus: '0.5'
        memory: 512M
```

---

## 7. Implementation Feasibility & Timeline

### Phase 0 (Research) - COMPLETE
- Technology selection: Done (this document)
- Trade-study validation: Complete
- Licensing review: Complete
- Cross-platform strategy: Defined

### Phase 1 (Design) - READY
- Estimated effort: 8-16 hours
- Deliverables:
  - `data-model.md` (ClickHouse schemas, aggregation queries)
  - `quickstart.md` (15-minute setup guide)
  - `contracts/` (API specs, database DDL)
- Dependencies: None (research complete)

### Phase 2 (Implementation) - READY FOR TASKING
- Estimated effort: 20-40 hours
- Major components:
  - docker-compose.yml + collector-config.yaml (4h)
  - ClickHouse schema + initialization (6h)
  - Claude Code environment configuration (.env templates) (4h)
  - Verification scripts (setup verification, sample data) (6h)
  - Documentation (quickstart, troubleshooting, queries) (8h)
  - Testing (multi-platform docker setup, sample data collection) (6h)
- Dependencies: Phase 1 complete

### Phase 3 (Validation) - POST-IMPLEMENTATION
- Estimated effort: 6-10 hours
- Activities:
  - Cross-platform testing (Mac, Linux, WSL)
  - High-volume data testing (1000+ events)
  - Offline buffering validation
  - Error recovery testing
  - Team feedback and documentation refinement

**Total Estimated Effort**: 34-66 hours across Phases 0-3

---

## 8. Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| ClickHouse SSPL licensing concern | Medium | Medium | Document licensing explicitly; provide Prometheus alternative |
| Docker image sizes excessive | Low | Medium | Provide resource limits in docker-compose.yml |
| Team hesitant to enable telemetry | Medium | High | Strong DATA-USAGE-POLICY explaining data ownership and benefits |
| Query language learning curve | Medium | Low | Provide example queries as shell scripts (copy-paste friendly) |
| High-volume session data loss | Low | High | Implement file-based buffering with retry logic |
| Cross-platform compatibility issues | Medium | Low | Test on Mac/Linux/WSL; provide troubleshooting docs |

---

## 9. Recommended Reading

**OpenTelemetry**:
- https://opentelemetry.io/docs/
- https://opentelemetry.io/docs/specs/semconv/

**ClickHouse**:
- https://clickhouse.com/docs/en/intro
- https://clickhouse.com/docs/en/engines/table-engines/mergetree-family/mergetree (understanding table design)

**Docker Compose**:
- https://docs.docker.com/compose/

**OSS Data & Privacy**:
- https://en.wikipedia.org/wiki/Business_Source_License (ClickHouse licensing)
- https://opensource.org/licenses/ (standard OSS licenses)

---

## Conclusion

**Technology Recommendation**:
- OTLP Collector + ClickHouse + Docker Compose is the optimal choice for Feature 017
- Proven technologies with mature ecosystems
- Minimal operational overhead for local team use
- Scales from 1 developer to 10+ without architectural changes
- Clear licensing and data ownership story for OSS distribution

**Next Steps**:
1. Proceed to Phase 1 (Design) with full confidence in technology choices
2. Create data-model.md, quickstart.md, and contracts/ artifacts
3. Generate Phase 2 implementation tasks via `/sp.tasks` command
