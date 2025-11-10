# Telemetry Query Examples

This directory contains SQL queries for analyzing Claude Code usage data. All queries are written for ClickHouse SQL dialect.

## Running Queries

### Via cURL (HTTP Interface)

```bash
# Set credentials
export CLICKHOUSE_USER="telemetry_user"
export CLICKHOUSE_PASSWORD="your_password"

# Execute query
curl -u "${CLICKHOUSE_USER}:${CLICKHOUSE_PASSWORD}" \
  "http://localhost:8123/" \
  --data-binary @queries/session-analysis.sql
```

### Via ClickHouse Client

```bash
# Install client (if needed)
# macOS: brew install clickhouse
# Linux: apt-get install clickhouse-client

# Connect
clickhouse-client --host localhost --port 9000 \
  --user telemetry_user --password your_password

# Run query
USE telemetry;
\i queries/session-analysis.sql
```

### Via Docker

```bash
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password your_password \
  --database telemetry \
  --multiquery < queries/session-analysis.sql
```

## Query Categories

### 1. Session Analysis (`session-analysis.sql`)

Analyze work sessions: duration, event counts, costs, performance patterns.

**Use cases**:
- How long are typical sessions?
- What features consume most resources?
- Which workflows are most expensive?

### 2. Error Analysis (`error-analysis.sql`)

Following Andrew Ng's error analysis methodology: identify patterns, categorize, prioritize fixes.

**Use cases**:
- What errors occur most frequently?
- Which tools have highest failure rates?
- What workflow steps are error-prone?

### 3. Tool Usage (`tool-usage.sql`)

Understand which tools are used, how often, and their performance characteristics.

**Use cases**:
- Most frequently used tools?
- Which tools are slowest?
- Tool usage by workflow phase?

### 4. Cost Analysis (`cost-analysis.sql`)

Track API costs and token usage.

**Use cases**:
- Daily/weekly/monthly cost trends?
- Most expensive features?
- Cost optimization opportunities?

### 5. Workflow Traces (`workflow-traces.sql`)

Reconstruct complete workflow executions (Specify → Plan → Implement → Validate).

**Use cases**:
- How long does SDD workflow take?
- Where are bottlenecks?
- Success rates by phase?

## Common Query Patterns

### Time-Based Filtering

```sql
-- Last 24 hours
WHERE timestamp >= now() - INTERVAL 1 DAY

-- Last 7 days
WHERE timestamp >= now() - INTERVAL 7 DAY

-- Specific date range
WHERE timestamp BETWEEN '2025-01-01' AND '2025-01-31'

-- By hour of day (identify peak usage)
WHERE toHour(timestamp) BETWEEN 9 AND 17
```

### Aggregation Functions

```sql
-- Event counts
count() AS event_count

-- Unique users
uniqExact(user_id) AS unique_users

-- Average/percentiles
avg(latency_ms) AS avg_latency,
quantile(0.95)(latency_ms) AS p95_latency

-- First/last events
min(timestamp) AS first_event,
max(timestamp) AS last_event,
anyLast(event_data) AS sample_event
```

### Grouping and Ordering

```sql
-- By event type
GROUP BY event_type
ORDER BY count() DESC

-- By time buckets
GROUP BY toStartOfHour(timestamp)
ORDER BY toStartOfHour(timestamp)

-- By multiple dimensions
GROUP BY workflow_step, tool_name
ORDER BY count() DESC
```

### Materialized Views (Pre-Aggregated)

```sql
-- Session summaries (updated automatically)
SELECT * FROM session_summaries
WHERE session_date >= today() - INTERVAL 7 DAY
ORDER BY session_date DESC;

-- Error patterns (optimized for error analysis)
SELECT * FROM error_patterns
WHERE error_date >= today() - INTERVAL 30 DAY
ORDER BY error_occurrences DESC;

-- Tool usage patterns
SELECT * FROM tool_usage_patterns
WHERE usage_date >= today() - INTERVAL 7 DAY
ORDER BY usage_count DESC;
```

## Example Queries

### 1. Daily Active Users

```sql
SELECT 
    toDate(timestamp) AS date,
    uniqExact(user_id) AS active_users,
    count() AS total_events
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 30 DAY
GROUP BY date
ORDER BY date DESC;
```

### 2. Top 10 Most Used Tools

```sql
SELECT 
    tool_name,
    count() AS usage_count,
    avg(latency_ms) AS avg_latency_ms,
    countIf(api_status = 'success') AS success_count,
    countIf(api_status = 'error') AS error_count,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY tool_name
ORDER BY usage_count DESC
LIMIT 10;
```

### 3. Session Performance Metrics

```sql
SELECT 
    user_id,
    session_id,
    count() AS events,
    min(timestamp) AS start_time,
    max(timestamp) AS end_time,
    dateDiff('minute', min(timestamp), max(timestamp)) AS duration_minutes,
    sum(tokens_input) AS total_input_tokens,
    sum(tokens_output) AS total_output_tokens,
    sum(cost_usd) AS total_cost_usd,
    countIf(event_type = 'api_error') AS error_count
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 1 DAY
GROUP BY user_id, session_id
ORDER BY start_time DESC
LIMIT 20;
```

### 4. Error Rate by Workflow Phase

```sql
SELECT 
    workflow_step,
    count() AS total_events,
    countIf(event_type = 'api_error') AS errors,
    round(countIf(event_type = 'api_error') / count() * 100, 2) AS error_rate_pct
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 7 DAY
  AND workflow_step IS NOT NULL
GROUP BY workflow_step
ORDER BY error_rate_pct DESC;
```

### 5. Cost Analysis by Feature

```sql
SELECT 
    feature_name,
    count() AS event_count,
    sum(cost_usd) AS total_cost_usd,
    avg(cost_usd) AS avg_cost_per_event,
    sum(tokens_input + tokens_output) AS total_tokens
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 30 DAY
  AND feature_name IS NOT NULL
  AND cost_usd IS NOT NULL
GROUP BY feature_name
ORDER BY total_cost_usd DESC
LIMIT 20;
```

### 6. Agent Performance Comparison

```sql
SELECT 
    agent_type,
    count() AS usage_count,
    avg(latency_ms) AS avg_latency_ms,
    quantile(0.50)(latency_ms) AS p50_latency,
    quantile(0.95)(latency_ms) AS p95_latency,
    avg(tokens_output) AS avg_output_tokens,
    countIf(api_status = 'error') AS error_count
FROM telemetry.telemetry_events
WHERE agent_type IS NOT NULL
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY agent_type
ORDER BY usage_count DESC;
```

### 7. Hourly Usage Heatmap

```sql
SELECT 
    toHour(timestamp) AS hour_of_day,
    toDayOfWeek(timestamp) AS day_of_week,
    count() AS event_count,
    uniqExact(user_id) AS active_users
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 30 DAY
GROUP BY hour_of_day, day_of_week
ORDER BY day_of_week, hour_of_day;
```

### 8. Longest Running Sessions

```sql
SELECT 
    session_id,
    user_id,
    feature_name,
    min(timestamp) AS session_start,
    max(timestamp) AS session_end,
    dateDiff('minute', min(timestamp), max(timestamp)) AS duration_minutes,
    count() AS event_count,
    sum(cost_usd) AS total_cost_usd
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 7 DAY
GROUP BY session_id, user_id, feature_name
HAVING duration_minutes > 30
ORDER BY duration_minutes DESC
LIMIT 10;
```

## Performance Optimization Tips

### 1. Use Time Partitioning

```sql
-- Fast (uses partition pruning)
WHERE timestamp >= now() - INTERVAL 7 DAY

-- Slow (scans all partitions)
WHERE toDate(timestamp) = today()
```

### 2. Leverage Materialized Views

```sql
-- Instead of this (expensive)
SELECT ... FROM telemetry_events GROUP BY ...

-- Use this (pre-aggregated)
SELECT ... FROM session_summaries
```

### 3. Use Indexes

```sql
-- Bloom filter indexes are already created for:
-- - session_id
-- - user_id
-- - feature_name
-- These columns are fast for filtering
WHERE session_id = 'xxx'  -- Fast
WHERE user_id = 'yyy'      -- Fast
```

### 4. Limit Result Sets

```sql
-- Always use LIMIT for exploratory queries
SELECT ... LIMIT 100

-- Use OFFSET for pagination
SELECT ... LIMIT 100 OFFSET 200
```

## Data Export

### Export to CSV

```bash
clickhouse-client --query "SELECT * FROM telemetry.telemetry_events WHERE timestamp >= now() - INTERVAL 1 DAY" \
  --format CSV > events.csv
```

### Export to JSON

```bash
clickhouse-client --query "SELECT * FROM telemetry.telemetry_events LIMIT 100" \
  --format JSONEachRow > events.jsonl
```

### Export to Parquet (for analysis in Python/pandas)

```bash
clickhouse-client --query "SELECT * FROM telemetry.telemetry_events WHERE timestamp >= now() - INTERVAL 7 DAY" \
  --format Parquet > events.parquet
```

## Next Steps

1. **Customize queries**: Modify examples for your specific analysis needs
2. **Create dashboards**: Connect BI tools (Grafana, Metabase, Superset)
3. **Automate reports**: Schedule queries with cron
4. **Build alerts**: Query for anomalies and send notifications

## Reference

- [ClickHouse SQL Reference](https://clickhouse.com/docs/en/sql-reference/)
- [ClickHouse Functions](https://clickhouse.com/docs/en/sql-reference/functions/)
- [Time Series Analysis](https://clickhouse.com/docs/en/guides/time-series/)

---

**Happy querying!** 📊
