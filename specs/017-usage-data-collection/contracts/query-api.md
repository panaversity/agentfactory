# Query API: Telemetry Data Access Interface (Feature 017)

**Status**: Reference Documentation | **Date**: 2025-11-10

## Overview

This document describes the HTTP API for querying telemetry data from the local ClickHouse database. All queries are executed via HTTP GET/POST to the ClickHouse native HTTP interface.

---

## Base Endpoint

```
http://localhost:8123/
```

**Port**: 8123 (HTTP interface)  
**Authentication**: None (local-only access)  
**Protocol**: HTTP/1.1  
**Default Format**: CSV

---

## Query Methods

### Method 1: HTTP GET with URL-encoded Query

**Usage**: Simple queries, easy to test in browser

```bash
curl -s 'http://localhost:8123/?query=SELECT+COUNT%28*%29+FROM+telemetry_events'
```

**Limitations**:
- URL length limits (2000-8000 chars depending on client)
- Special characters must be URL-encoded
- Difficult for complex multi-line queries

---

### Method 2: HTTP POST with Query in Body

**Usage**: Complex queries, recommended for scripts

```bash
curl -s -X POST --data-binary @query.sql 'http://localhost:8123/'
```

**Advantages**:
- No URL length limits
- Readable multi-line queries
- Better for shell scripts and automation

**Example**:
```bash
cat > query.sql << 'EOF'
SELECT 
  event_type,
  COUNT(*) as count
FROM telemetry_events
GROUP BY event_type
ORDER BY count DESC
EOF

curl -s -X POST --data-binary @query.sql 'http://localhost:8123/'
```

---

### Method 3: Formatted Output with `format` Parameter

**Syntax**:
```bash
curl -s 'http://localhost:8123/?query=SELECT+*+FROM+telemetry_events+LIMIT+1&format=JSON'
```

**Supported Formats**:
- `CSV` (default) — Comma-separated values
- `JSON` — JSON objects
- `JSONCompact` — Compact JSON
- `TabSeparated` — Tab-separated values
- `Pretty` — Human-readable (good for testing)

**Example with Pretty Format**:
```bash
curl -s 'http://localhost:8123/?query=SELECT+user_id,+SUM%28cost_usd%29+FROM+telemetry_events+WHERE+event_type=%27api_request%27+GROUP+BY+user_id&format=Pretty'
```

---

## Standard Query Patterns

### Pattern 1: Count Events by Type

**Purpose**: Understand data distribution

```sql
SELECT 
  event_type,
  COUNT(*) as count
FROM telemetry_events
GROUP BY event_type
ORDER BY count DESC
```

**Expected Columns**: event_type (String), count (UInt64)

---

### Pattern 2: Time-Series Aggregation

**Purpose**: Analyze trends over time

```sql
SELECT 
  toDate(timestamp) as date,
  event_type,
  COUNT(*) as count
FROM telemetry_events
WHERE timestamp > now() - INTERVAL 7 day
GROUP BY date, event_type
ORDER BY date DESC, event_type
```

**Expected Columns**: date (Date), event_type (String), count (UInt64)

---

### Pattern 3: Cost Analysis

**Purpose**: Track spending by dimension

```sql
SELECT 
  user_id,
  feature_id,
  SUM(cost_usd) as total_cost,
  COUNT(*) as request_count,
  ROUND(SUM(cost_usd) / COUNT(*), 6) as avg_cost_per_request
FROM telemetry_events
WHERE event_type = 'api_request' AND cost_usd IS NOT NULL
GROUP BY user_id, feature_id
ORDER BY total_cost DESC
LIMIT 20
```

**Expected Columns**: user_id (String), feature_id (String), total_cost (Decimal), request_count (UInt64), avg_cost_per_request (Decimal)

---

### Pattern 4: Error Analysis

**Purpose**: Identify failure patterns

```sql
SELECT 
  error_type,
  COUNT(*) as occurrences,
  COUNT(DISTINCT session_id) as affected_sessions,
  COUNT(DISTINCT user_id) as affected_users,
  MAX(timestamp) as last_occurred
FROM telemetry_events
WHERE event_type = 'api_error'
GROUP BY error_type
ORDER BY occurrences DESC
LIMIT 20
```

**Expected Columns**: error_type (String), occurrences (UInt64), affected_sessions (UInt64), affected_users (UInt64), last_occurred (DateTime)

---

### Pattern 5: Session Workflow Trace

**Purpose**: Debug a specific session

```sql
SELECT 
  timestamp,
  event_type,
  CASE 
    WHEN event_type = 'user_prompt' 
      THEN CONCAT('prompt length: ', toString(prompt_length_chars), ' tokens: ', toString(prompt_tokens_estimated))
    WHEN event_type = 'api_request' 
      THEN CONCAT('tokens: ', toString(tokens_input), '→', toString(tokens_output), ' cost: $', toString(cost_usd))
    WHEN event_type = 'api_error' 
      THEN CONCAT(error_type, ' (', toString(error_http_status), ')')
    WHEN event_type = 'tool_call' 
      THEN CONCAT(tool_name, ': ', tool_status, ' (', toString(tool_duration_ms), 'ms)')
    WHEN event_type = 'validation_result' 
      THEN CONCAT(output_type, ' → ', validation_outcome)
    ELSE ''
  END as detail
FROM telemetry_events
WHERE session_id = 'YOUR_SESSION_ID_HERE'
ORDER BY timestamp ASC
```

**Replace**: `YOUR_SESSION_ID_HERE` with actual UUID from your data

**Expected Columns**: timestamp (DateTime), event_type (String), detail (String)

---

### Pattern 6: Token Efficiency by Feature

**Purpose**: Identify features with best token ROI

```sql
SELECT 
  feature_id,
  chapter_number,
  COUNT(DISTINCT session_id) as sessions,
  SUM(tokens_input) as total_input,
  SUM(tokens_output) as total_output,
  ROUND(SUM(tokens_output) / SUM(tokens_input), 3) as output_input_ratio,
  SUM(cost_usd) as total_cost
FROM telemetry_events
WHERE event_type = 'api_request' 
  AND tokens_input > 0
  AND cost_usd IS NOT NULL
GROUP BY feature_id, chapter_number
ORDER BY total_cost DESC
```

**Expected Columns**: feature_id (String), chapter_number (UInt16), sessions (UInt64), total_input (UInt64), total_output (UInt64), output_input_ratio (Float), total_cost (Decimal)

---

### Pattern 7: Validation Success Rate

**Purpose**: Measure AI output quality

```sql
SELECT 
  feature_id,
  COUNT(*) as total_validations,
  SUM(IF(validation_outcome = 'approved', 1, 0)) as approved,
  SUM(IF(validation_outcome = 'rejected', 1, 0)) as rejected,
  SUM(IF(validation_outcome = 'modified', 1, 0)) as modified,
  ROUND(SUM(IF(validation_outcome = 'approved', 1, 0)) / COUNT(*) * 100, 2) as approval_rate_percent
FROM telemetry_events
WHERE event_type = 'validation_result'
GROUP BY feature_id
ORDER BY approval_rate_percent DESC
```

**Expected Columns**: feature_id (String), total_validations (UInt64), approved (UInt64), rejected (UInt64), modified (UInt64), approval_rate_percent (Decimal)

---

## Response Format

### CSV Response (Default)

```
event_type,count
user_prompt,143
api_request,289
api_error,12
tool_call,256
validation_result,89
workflow_checkpoint,34
```

### JSON Response (with &format=JSON)

```json
[
  {"event_type": "user_prompt", "count": "143"},
  {"event_type": "api_request", "count": "289"},
  {"event_type": "api_error", "count": "12"}
]
```

### Pretty Response (with &format=Pretty)

```
┌─event_type──┬──count─┐
│ user_prompt │    143 │
│ api_request │    289 │
│ api_error   │     12 │
└─────────────┴────────┘
```

---

## Common Query Examples with Curl

### Example 1: Get Total Event Count

```bash
curl -s 'http://localhost:8123/?query=SELECT%20COUNT%28*%29%20as%20total_events%20FROM%20telemetry_events'

# Output: 732
```

---

### Example 2: Cost Report (Last 7 Days)

```bash
curl -s -X POST 'http://localhost:8123/?format=CSV' << 'EOF'
SELECT 
  toDate(timestamp) as date,
  user_id,
  SUM(cost_usd) as daily_cost
FROM telemetry_events
WHERE event_type = 'api_request'
  AND timestamp > now() - INTERVAL 7 day
GROUP BY date, user_id
ORDER BY date DESC, daily_cost DESC
EOF

# Output:
# date,user_id,daily_cost
# 2025-01-10,alice@example.com,15.32
# 2025-01-10,bob@example.com,8.21
# 2025-01-09,alice@example.com,12.45
```

---

### Example 3: Error Summary

```bash
curl -s -X POST 'http://localhost:8123/?format=Pretty' << 'EOF'
SELECT 
  error_type,
  COUNT(*) as occurrences
FROM telemetry_events
WHERE event_type = 'api_error'
GROUP BY error_type
ORDER BY occurrences DESC
LIMIT 10
EOF

# Output:
# ┌─error_type─────────┬─occurrences─┐
# │ rate_limit_error   │           7 │
# │ timeout_error      │           3 │
# │ network_error      │           2 │
# └────────────────────┴─────────────┘
```

---

## Error Handling

### Invalid Query

**Request**:
```bash
curl -s 'http://localhost:8123/?query=SELECT%20*%20FROM%20invalid_table'
```

**Response**:
```
Code: 60. DB::Exception: Table default.invalid_table doesn't exist.
```

### Connection Refused

**Request**:
```bash
curl -s 'http://localhost:8124/?query=SELECT%201'  # Wrong port
```

**Response**:
```
curl: (7) Failed to connect to localhost port 8124: Connection refused
```

**Solution**: Verify ClickHouse is running: `docker-compose ps`

---

## Performance Considerations

### Query Optimization Tips

1. **Use Indexes**: Filter by `session_id`, `user_id`, `timestamp` first
   ```sql
   -- Good (uses index on session_id)
   SELECT * FROM telemetry_events WHERE session_id = 'uuid' LIMIT 100

   -- Less efficient (full table scan)
   SELECT * FROM telemetry_events WHERE prompt_length_chars > 500
   ```

2. **Limit Rows**: Always use `LIMIT` for exploration
   ```sql
   -- Good
   SELECT * FROM telemetry_events LIMIT 10

   -- Bad (potentially slow)
   SELECT * FROM telemetry_events
   ```

3. **Use Aggregations**: Pre-aggregate data when possible
   ```sql
   -- Fast (uses aggregation function)
   SELECT COUNT(*) FROM telemetry_events

   -- Slower (full table scan)
   SELECT * FROM telemetry_events
   ```

4. **Time-Range Filtering**: Use `timestamp` index for historical queries
   ```sql
   -- Fast (uses timestamp index)
   SELECT * FROM telemetry_events WHERE timestamp > now() - INTERVAL 7 day

   -- Slower (might scan all data)
   SELECT * FROM telemetry_events WHERE feature_id = '010'
   ```

---

## Scripting with Queries

### Bash Script: Generate Monthly Cost Report

```bash
#!/bin/bash
# monthly-cost-report.sh

YEAR_MONTH=$(date +%Y-%m)
OUTPUT_FILE="cost-report-${YEAR_MONTH}.csv"

cat > query.sql << EOF
SELECT 
  toDate(timestamp) as date,
  user_id,
  feature_id,
  COUNT(DISTINCT session_id) as sessions,
  SUM(tokens_input) as tokens_in,
  SUM(tokens_output) as tokens_out,
  SUM(cost_usd) as cost
FROM telemetry_events
WHERE event_type = 'api_request'
  AND toYYYYMM(timestamp) = toYYYYMM(now())
GROUP BY date, user_id, feature_id
ORDER BY date DESC, cost DESC
EOF

curl -s -X POST 'http://localhost:8123/?format=CSV' \
  --data-binary @query.sql > "$OUTPUT_FILE"

echo "Cost report saved to $OUTPUT_FILE"
cat "$OUTPUT_FILE"
```

---

### Python Script: Query and Process Results

```python
#!/usr/bin/env python3
"""Query telemetry data and generate insights."""

import urllib.request
import csv
import json
from datetime import datetime, timedelta

def query_clickhouse(sql_query, format='CSV'):
    """Execute query against ClickHouse HTTP API."""
    url = 'http://localhost:8123/'
    params = f'?format={format}'
    
    req = urllib.request.Request(
        url + params,
        data=sql_query.encode('utf-8'),
        method='POST'
    )
    
    with urllib.request.urlopen(req) as response:
        return response.read().decode('utf-8')

# Example: Get cost by user
query = """
SELECT 
  user_id,
  SUM(cost_usd) as total_cost,
  COUNT(DISTINCT session_id) as sessions
FROM telemetry_events
WHERE event_type = 'api_request'
GROUP BY user_id
ORDER BY total_cost DESC
"""

result = query_clickhouse(query)
print(result)

# Parse CSV result
reader = csv.DictReader(result.strip().split('\n'))
for row in reader:
    print(f"{row['user_id']}: ${row['total_cost']} ({row['sessions']} sessions)")
```

---

## Monitoring & Health Checks

### Health Check Query

```bash
curl -s 'http://localhost:8123/?query=SELECT%201'

# Expected output: 1 (ClickHouse is healthy)
```

### Table Statistics

```bash
curl -s 'http://localhost:8123/?query=SELECT%20table,%20sum%28bytes%29%20as%20size_bytes%20FROM%20system.parts%20WHERE%20database=%27default%27%20GROUP%20BY%20table%20ORDER%20BY%20size_bytes%20DESC&format=Pretty'
```

---

## Limitations

1. **Local-Only**: Queries only work from `localhost` (can be changed via collector config)
2. **No Authentication**: No user/password required (by design for local development)
3. **Timeout**: Queries must complete within server timeout (default: 30 seconds)
4. **Memory**: Large result sets load entirely into memory (limit with `LIMIT`)

---

## Next Steps

- See `quickstart.md` for example queries to copy/paste
- See `data-model.md` for complete schema documentation
- See repository `telemetry-server/queries/` directory for runnable query scripts
