# Data Model: Usage Data Collection System (Feature 017)

**Phase**: 1 - Design | **Date**: 2025-11-10 | **Status**: Complete

## Overview

This document defines the telemetry data model for Feature 017, including:
- Event types and attributes (OpenTelemetry schemas)
- Session and user entities
- ClickHouse table definitions and schemas
- Aggregation models for reporting and analysis
- Data retention and archival strategies

The model is designed to support all four user stories (P1-P4) in the specification:
1. **P1**: Enable usage data collection (raw events)
2. **P2**: Centralized data storage (sessions, queries)
3. **P3**: Error analysis workflow (pattern detection)
4. **P4**: Team documentation & onboarding (example queries)

---

## 1. OpenTelemetry Event Schema

### Event Type: `user_prompt`

**When**: User types a prompt into Claude Code CLI

**Attributes**:
```json
{
  "event_type": "user_prompt",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",    // UUID v4
  "user_id": "alice@panaversity.com",                      // Team member identifier
  "organization_id": "panaversity-book-project",           // Project identifier
  "timestamp": "2025-01-10T14:23:45.123Z",                 // ISO 8601 UTC
  "code.version": "claude-code-1.2.3",                      // Claude Code version
  "terminal.type": "zsh",                                  // Shell type
  "git.branch": "010-chapter-specification",               // Active branch
  "git.commit": "a1b2c3d4e5f6g7h8i9j0",                    // Commit hash
  "feature.id": "010",                                     // Feature/chapter number
  "chapter.number": 10,                                    // Chapter number (if applicable)
  "prompt.length_chars": 1247,                             // Character count
  "prompt.tokens_estimated": 380,                          // Estimated token count
  "prompt.text_hash": "sha256:abc123...",                  // Hash of prompt text (privacy)
  "model_id": "claude-3.5-sonnet",                         // Model used
  "temperature": 0.7,                                      // Temperature setting
  "max_tokens": 4096                                       // Max tokens requested
}
```

**Retention**: 90 days raw (detailed prompt text rarely needed after initial analysis)

---

### Event Type: `tool_call`

**When**: Claude Code invokes a tool (file edit, bash, browser, etc.)

**Attributes**:
```json
{
  "event_type": "tool_call",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "alice@panaversity.com",
  "organization_id": "panaversity-book-project",
  "timestamp": "2025-01-10T14:23:47.456Z",
  "tool_name": "file_edit",                                // Tool type: file_edit, bash, browser, etc.
  "tool_call_id": "tool_123456",                           // Unique identifier for this call
  "tool_status": "success",                                // success | failure | timeout
  "tool_duration_ms": 234,                                 // Execution time
  "file_path": "/path/to/file.py",                         // For file operations
  "file_lines_changed": 15,                                // Insertions + deletions
  "bash_command_hash": "sha256:def456...",                 // Hash of command (privacy)
  "bash_exit_code": 0,                                     // Exit code
  "tool_output_length_chars": 512,                         // Tool output size
  "error_message": null,                                   // Error if failed
  "git.branch": "010-chapter-specification",
  "feature.id": "010"
}
```

**Retention**: 90 days raw

---

### Event Type: `api_request`

**When**: API call made (to Claude, external services, etc.)

**Attributes**:
```json
{
  "event_type": "api_request",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "alice@panaversity.com",
  "organization_id": "panaversity-book-project",
  "timestamp": "2025-01-10T14:23:50.789Z",
  "api_name": "anthropic.messages",                        // API endpoint
  "api_status": "success",                                 // success | rate_limited | error
  "api_http_status": 200,                                  // HTTP status code
  "api_duration_ms": 3200,                                 // Time to response
  "tokens_input": 850,                                     // Input tokens used
  "tokens_output": 420,                                    // Output tokens generated
  "tokens_cache_read": 0,                                  // Prompt caching (if applicable)
  "cost_usd": 0.042,                                       // Calculated cost (input + output)
  "model_id": "claude-3.5-sonnet",                         // Model used
  "stop_reason": "end_turn",                               // Why response stopped
  "git.branch": "010-chapter-specification",
  "feature.id": "010"
}
```

**Retention**: Indefinite (cost analysis requires historical data)

---

### Event Type: `api_error`

**When**: API call fails

**Attributes**:
```json
{
  "event_type": "api_error",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "alice@panaversity.com",
  "organization_id": "panaversity-book-project",
  "timestamp": "2025-01-10T14:23:52.100Z",
  "error_type": "rate_limit_error",                        // Error classification
  "error_message_hash": "sha256:ghi789...",                // Error message hash (privacy)
  "http_status": 429,                                      // HTTP status
  "api_name": "anthropic.messages",
  "retry_attempt": 2,                                      // Which retry this is
  "retry_after_seconds": 60,                               // Backoff guidance
  "git.branch": "010-chapter-specification",
  "feature.id": "010"
}
```

**Retention**: 90 days raw

---

### Event Type: `validation_result`

**When**: User validates AI output (approves, rejects, modifies)

**Attributes**:
```json
{
  "event_type": "validation_result",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "alice@panaversity.com",
  "organization_id": "panaversity-book-project",
  "timestamp": "2025-01-10T14:25:00.123Z",
  "output_type": "code",                                   // code | markdown | analysis | etc
  "validation_outcome": "approved",                        // approved | rejected | modified | pending_review
  "quality_score": 8,                                      // 1-10 user rating (optional)
  "feedback_text_hash": "sha256:jkl012...",                // Hash of feedback (privacy)
  "feedback_text_length_chars": 245,                       // Length of feedback
  "time_to_validate_seconds": 45,                          // How long review took
  "git.branch": "010-chapter-specification",
  "feature.id": "010"
}
```

**Retention**: 90 days raw (pattern analysis needs detail)

---

### Event Type: `workflow_checkpoint`

**When**: Major workflow milestone reached (spec → plan → implement → validate)

**Attributes**:
```json
{
  "event_type": "workflow_checkpoint",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "alice@panaversity.com",
  "organization_id": "panaversity-book-project",
  "timestamp": "2025-01-10T14:30:00.000Z",
  "checkpoint_type": "spec_completed",                     // spec_completed | plan_completed | implement_started | validation_passed | etc
  "chapter_number": 10,
  "feature_id": "010",
  "elapsed_time_minutes": 45,                              // Since session start
  "artifacts_created": 2,                                  // Number of files/outputs
  "git.branch": "010-chapter-specification"
}
```

**Retention**: Indefinite (tracks project progress)

---

## 2. Core Entities

### Session

Represents a single Claude Code CLI session.

**Attributes**:
```sql
CREATE TABLE sessions (
  session_id UUID PRIMARY KEY,
  user_id String,
  organization_id String,
  start_timestamp DateTime,
  end_timestamp DateTime,
  duration_seconds UInt32,
  code_version String,
  terminal_type String,
  git_branch String,
  git_commit String,
  feature_id String,
  chapter_number Nullable(UInt16),
  event_count UInt32,
  total_tokens_input UInt64,
  total_tokens_output UInt64,
  total_cost_usd Decimal(10, 6),
  validation_failures UInt16,
  api_errors UInt16
) ENGINE = MergeTree()
  ORDER BY (start_timestamp, user_id)
```

**Lifecycle**:
- Created: When Claude Code session starts
- Updated: At session end (aggregate metrics)
- Retained: Indefinite (workflow history)

### User

Represents a team member using Claude Code.

**Attributes**:
```sql
CREATE TABLE users (
  user_id String PRIMARY KEY,
  user_email String,
  organization_id String,
  full_name_hash String,  -- Hashed for privacy
  first_session DateTime,
  last_session DateTime,
  total_sessions UInt32,
  role String  -- author | reviewer | coordinator
) ENGINE = ReplacingMergeTree(last_session)
  ORDER BY (user_id)
```

**Lifecycle**:
- Created: At first session
- Updated: After each session
- Retained: Indefinite

---

## 3. ClickHouse Table Definitions

### Primary Table: `telemetry_events`

Raw event stream (insert-optimized).

**DDL**:
```sql
CREATE TABLE telemetry_events (
  event_id UUID DEFAULT generateUUIDv4(),
  event_type String,                           -- user_prompt | tool_call | api_request | api_error | validation_result | workflow_checkpoint
  session_id UUID,
  user_id String,
  organization_id String,
  timestamp DateTime,
  
  -- Common attributes
  code_version String,
  terminal_type Nullable(String),
  git_branch String,
  git_commit Nullable(String),
  feature_id String,
  chapter_number Nullable(UInt16),
  
  -- Event-specific attributes (sparse columns)
  prompt_length_chars Nullable(UInt32),
  prompt_tokens_estimated Nullable(UInt16),
  prompt_text_hash Nullable(String),
  tool_name Nullable(String),
  tool_status Nullable(String),
  tool_duration_ms Nullable(UInt32),
  file_path Nullable(String),
  file_lines_changed Nullable(Int32),
  bash_command_hash Nullable(String),
  bash_exit_code Nullable(Int32),
  api_name Nullable(String),
  api_status Nullable(String),
  api_http_status Nullable(UInt16),
  api_duration_ms Nullable(UInt32),
  tokens_input Nullable(UInt32),
  tokens_output Nullable(UInt32),
  tokens_cache_read Nullable(UInt32),
  cost_usd Nullable(Decimal(10, 6)),
  model_id Nullable(String),
  error_type Nullable(String),
  error_message_hash Nullable(String),
  validation_outcome Nullable(String),
  quality_score Nullable(UInt8),
  checkpoint_type Nullable(String),
  
  -- Metadata
  collected_at DateTime DEFAULT now(),
  INDEX idx_session_id session_id TYPE hash,
  INDEX idx_user_id user_id TYPE hash,
  INDEX idx_timestamp timestamp TYPE minmax,
  INDEX idx_event_type event_type TYPE set(10),
  INDEX idx_feature_id feature_id TYPE hash
) ENGINE = MergeTree()
  ORDER BY (timestamp, session_id)
  TTL timestamp + INTERVAL 90 day DELETE  -- Auto-delete old raw events
  SETTINGS storage_policy = 'default'
```

**Compression**: zstd (excellent for telemetry data, 10-20x compression)

**Storage Optimization**:
- Timestamp ordering enables range queries and time-based pruning
- Sparse columns (nullable) for event-specific data
- TTL for automatic cleanup per retention policy

---

### Session Summary Table: `sessions`

Pre-aggregated session metrics.

**DDL**:
```sql
CREATE TABLE sessions (
  session_id UUID,
  user_id String,
  organization_id String,
  start_timestamp DateTime,
  end_timestamp DateTime,
  duration_seconds UInt32,
  code_version String,
  terminal_type String,
  git_branch String,
  git_commit String,
  feature_id String,
  chapter_number Nullable(UInt16),
  
  -- Aggregated metrics
  event_count UInt32,
  prompt_count UInt16,
  tool_call_count UInt16,
  api_request_count UInt16,
  api_error_count UInt16,
  validation_count UInt16,
  
  total_tokens_input UInt64,
  total_tokens_output UInt64,
  total_cost_usd Decimal(10, 6),
  
  validation_approved_count UInt16,
  validation_rejected_count UInt16,
  validation_modified_count UInt16,
  
  error_rate_percent Decimal(5, 2),
  avg_api_duration_ms UInt32,
  
  INDEX idx_user_id user_id TYPE hash,
  INDEX idx_feature_id feature_id TYPE hash,
  INDEX idx_timestamp start_timestamp TYPE minmax
) ENGINE = MergeTree()
  ORDER BY (start_timestamp, user_id)
  PRIMARY KEY (session_id)
```

**Population**: Materialized view from telemetry_events (automatic aggregation on insert)

---

### Cost Analysis Table: `daily_cost_metrics`

Pre-aggregated daily costs for fast reporting.

**DDL**:
```sql
CREATE TABLE daily_cost_metrics (
  metric_date Date,
  user_id String,
  organization_id String,
  feature_id String,
  chapter_number Nullable(UInt16),
  
  session_count UInt32,
  event_count UInt32,
  tokens_input UInt64,
  tokens_output UInt64,
  total_cost_usd Decimal(10, 6),
  avg_cost_per_session Decimal(10, 6),
  
  api_errors UInt32,
  validation_failures UInt32
) ENGINE = SummingMergeTree()
  ORDER BY (metric_date, user_id, feature_id)
  SETTINGS allow_experimental_primary_key_in_columns_only = 1
```

**Population**: Materialized view aggregating daily from telemetry_events

**Query Performance**: <100ms for 1 year of data (pre-aggregated)

---

## 4. Materialized Views for Reporting

### View: `cost_by_chapter_daily`

Enables quick "cost by chapter" analysis (User Story 2).

```sql
CREATE MATERIALIZED VIEW cost_by_chapter_daily AS
SELECT 
  toDate(timestamp) as metric_date,
  feature_id,
  chapter_number,
  COUNT(DISTINCT session_id) as session_count,
  COUNT(*) as event_count,
  SUM(tokens_input) as total_tokens_input,
  SUM(tokens_output) as total_tokens_output,
  SUM(cost_usd) as total_cost_usd,
  SUM(cost_usd) / COUNT(DISTINCT session_id) as avg_cost_per_session
FROM telemetry_events
WHERE event_type = 'api_request' AND cost_usd IS NOT NULL
GROUP BY metric_date, feature_id, chapter_number
ENGINE = SummingMergeTree()
ORDER BY (metric_date, feature_id)
```

**Query Example**:
```sql
-- Cost by chapter (last 30 days)
SELECT 
  chapter_number,
  SUM(total_cost_usd) as total_cost,
  SUM(session_count) as sessions,
  ROUND(total_cost_usd / session_count, 4) as cost_per_session
FROM cost_by_chapter_daily
WHERE metric_date >= today() - 30
GROUP BY chapter_number
ORDER BY total_cost DESC
```

---

### View: `error_patterns`

Enables error analysis workflow (User Story 3).

```sql
CREATE MATERIALIZED VIEW error_patterns AS
SELECT 
  error_type,
  COUNT(*) as occurrences,
  COUNT(DISTINCT session_id) as affected_sessions,
  COUNT(DISTINCT user_id) as affected_users,
  groupArray(DISTINCT session_id) as session_ids,
  toDate(MAX(timestamp)) as last_occurred,
  toDate(MIN(timestamp)) as first_occurred
FROM telemetry_events
WHERE event_type = 'api_error'
GROUP BY error_type
ENGINE = SummingMergeTree()
ORDER BY occurrences DESC
```

**Query Example**:
```sql
-- High-impact error patterns
SELECT 
  error_type,
  occurrences,
  affected_users,
  last_occurred,
  multiIf(
    occurrences > 100, 'CRITICAL',
    occurrences > 50, 'HIGH',
    occurrences > 10, 'MEDIUM',
    'LOW'
  ) as severity
FROM error_patterns
WHERE last_occurred >= today() - 7
ORDER BY occurrences DESC
```

---

## 5. Data Aggregation Models

### Cost Analysis Aggregation

**Dimensions**: user_id, feature_id, chapter_number, date
**Metrics**: total_cost_usd, tokens_input, tokens_output, session_count

**Queries**:

```sql
-- Total cost by user (all time)
SELECT 
  user_id,
  SUM(total_cost_usd) as total_spent,
  COUNT(DISTINCT session_id) as sessions
FROM sessions
GROUP BY user_id
ORDER BY total_spent DESC

-- Cost trend (weekly)
SELECT 
  toStartOfWeek(start_timestamp) as week,
  user_id,
  SUM(total_cost_usd) as weekly_cost
FROM sessions
GROUP BY week, user_id
ORDER BY week DESC, weekly_cost DESC

-- Cost by chapter (identify expensive chapters)
SELECT 
  chapter_number,
  COUNT(DISTINCT session_id) as sessions,
  SUM(total_cost_usd) as total_cost,
  ROUND(SUM(total_cost_usd) / COUNT(DISTINCT session_id), 4) as avg_cost_per_session
FROM sessions
WHERE chapter_number IS NOT NULL
GROUP BY chapter_number
ORDER BY total_cost DESC
```

### Token Efficiency Analysis

**Dimensions**: user_id, feature_id, event_type
**Metrics**: tokens_input, tokens_output, input_output_ratio

```sql
-- Token efficiency by session
SELECT 
  session_id,
  user_id,
  SUM(tokens_input) as total_input_tokens,
  SUM(tokens_output) as total_output_tokens,
  ROUND(SUM(tokens_output) / SUM(tokens_input), 3) as output_input_ratio,
  COUNT(*) as api_calls
FROM telemetry_events
WHERE event_type = 'api_request' AND tokens_input > 0
GROUP BY session_id, user_id
ORDER BY session_id DESC
LIMIT 100
```

### Error Rate Analysis

**Dimensions**: feature_id, error_type, user_id
**Metrics**: error_count, error_rate_percent, affected_sessions

```sql
-- Error rates by feature
SELECT 
  feature_id,
  COUNT(*) as total_events,
  SUM(IF(event_type = 'api_error', 1, 0)) as error_count,
  ROUND(SUM(IF(event_type = 'api_error', 1, 0)) / COUNT(*) * 100, 2) as error_rate_percent
FROM telemetry_events
GROUP BY feature_id
ORDER BY error_count DESC

-- Most common errors
SELECT 
  error_type,
  COUNT(*) as occurrences,
  COUNT(DISTINCT session_id) as sessions,
  COUNT(DISTINCT user_id) as users
FROM telemetry_events
WHERE event_type = 'api_error'
GROUP BY error_type
ORDER BY occurrences DESC
LIMIT 10
```

---

## 6. Workflow Trace Model

**Goal**: Enable "workflow trace review" from User Story 3 (error analysis)

**Trace Definition**: Sequence of related events within a session

**Example Trace**:
```
User starts session on 010-chapter-specification branch
  ↓
User submits prompt: "Create specification for feature X"
  ↓
API call to Claude (tokens: 850→420)
  ↓
Tool call: file_edit (spec.md, +50 lines)
  ↓
Tool call: bash (run validation script)
  ↓
API error: rate_limit (retry after 60s)
  ↓
Validation: User reviews and modifies spec
  ↓
Workflow checkpoint: spec_completed
```

**Query for Workflow Traces**:
```sql
-- Get complete trace for a session
SELECT 
  event_id,
  event_type,
  timestamp,
  CASE 
    WHEN event_type = 'user_prompt' THEN prompt_length_chars
    WHEN event_type = 'tool_call' THEN tool_status
    WHEN event_type = 'api_request' THEN CONCAT('tokens:', tokens_input, '→', tokens_output)
    ELSE ''
  END as event_detail
FROM telemetry_events
WHERE session_id = '550e8400-e29b-41d4-a716-446655440000'
ORDER BY timestamp ASC
```

---

## 7. Data Retention Policy

**Raw Events** (detailed data):
- Retention: 90 days
- Action: Automatic deletion via TTL
- Rationale: Error analysis needs recent detail; older data less useful for improvement

**Sessions** (aggregated):
- Retention: 1 year
- Action: Manual archive (optional)
- Rationale: Workflow history, cost reconciliation

**Aggregated Metrics** (daily_cost_metrics, error_patterns):
- Retention: Indefinite
- Action: Keep indefinitely
- Rationale: Trend analysis, year-over-year comparison

**Example Retention Configuration**:
```yaml
# telemetry-server/.env
TELEMETRY_RAW_EVENT_TTL_DAYS=90
TELEMETRY_SESSION_TTL_DAYS=365
TELEMETRY_METRICS_TTL_DAYS=0  # Never delete
TELEMETRY_ARCHIVE_PATH=/archive/telemetry  # Optional

# Automated cleanup script (run nightly)
# DELETE FROM telemetry_events WHERE timestamp < now() - INTERVAL 90 day
# Archive older sessions before deletion
```

---

## 8. Data Sanitization Schemas

**Goal**: Filter sensitive information before export

**Sensitive Patterns**:

```json
{
  "filters": [
    {
      "name": "api_keys",
      "type": "regex",
      "pattern": "(api_key|token|sk_|pk_)[a-zA-Z0-9_\\-]{20,}",
      "replacement": "[REDACTED_API_KEY]",
      "apply_to": ["prompt_text_hash", "error_message_hash", "bash_command_hash"]
    },
    {
      "name": "emails",
      "type": "regex",
      "pattern": "[a-z0-9+.-]+@[a-z0-9.-]+\\.[a-z]{2,}",
      "replacement": "[REDACTED_EMAIL]",
      "apply_to": ["user_id"]
    },
    {
      "name": "phone_numbers",
      "type": "regex",
      "pattern": "\\d{3}-?\\d{3}-?\\d{4}",
      "replacement": "[REDACTED_PHONE]",
      "apply_to": ["error_message_hash"]
    },
    {
      "name": "proprietary_paths",
      "type": "path_filter",
      "patterns": ["*/proprietary/*", "*/secret/*", "*/internal/*"],
      "action": "exclude_event"
    }
  ]
}
```

**Implementation**: Applied in OpenTelemetry Collector before ClickHouse insert

---

## Conclusion

This data model supports:
- ✅ User Story 1 (P1): Raw event collection with rich attributes
- ✅ User Story 2 (P2): Centralized queryable storage with aggregations
- ✅ User Story 3 (P3): Error analysis via workflow traces and pattern detection
- ✅ User Story 4 (P4): Example queries for common analysis patterns

The schema is optimized for:
- **Insertion performance**: 1M+ events/day per node
- **Query speed**: <3s for 30-day aggregations
- **Storage efficiency**: 10-20x compression via ClickHouse
- **Privacy**: Data sanitization before export
- **Compliance**: TTL-based retention policies
