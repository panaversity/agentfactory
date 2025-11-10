-- Feature 017: Usage Data Collection System
-- ClickHouse Schema for Telemetry Events
-- Database: telemetry
-- Version: 1.0.0

-- Create database if not exists
CREATE DATABASE IF NOT EXISTS telemetry;

USE telemetry;

-- Main telemetry events table
CREATE TABLE IF NOT EXISTS telemetry_events (
  -- Core identification
  event_id UUID DEFAULT generateUUIDv4(),
  event_type String,  -- user_prompt | tool_call | api_request | api_error | validation_result | session_start | session_end
  session_id UUID,
  user_id String,
  organization_id String DEFAULT 'panaversity',
  timestamp DateTime DEFAULT now(),

  -- Event-specific attributes (sparse columns)
  prompt_text_hash Nullable(String),       -- SHA256 hash of user prompt (privacy-preserving)
  tool_name Nullable(String),               -- Tool invoked (Read, Write, Bash, etc.)
  tool_parameters_hash Nullable(String),    -- SHA256 hash of tool parameters
  api_status Nullable(String),              -- success | error | timeout
  api_error_code Nullable(String),          -- Error code if applicable
  api_error_message Nullable(String),       -- Error message (sanitized)

  -- Performance metrics
  tokens_input Nullable(UInt32),
  tokens_output Nullable(UInt32),
  latency_ms Nullable(UInt32),
  cost_usd Nullable(Decimal(10, 6)),

  -- Workflow context
  workflow_step Nullable(String),           -- Which phase: specify | plan | implement | validate
  feature_name Nullable(String),            -- Feature being worked on
  file_path Nullable(String),               -- File being modified

  -- Metadata
  claude_code_version Nullable(String),
  model_name Nullable(String),              -- claude-sonnet-4-5-20250929, etc.
  agent_type Nullable(String),              -- chapter-planner | lesson-writer | technical-reviewer

  -- Raw event data (JSON)
  event_data String DEFAULT ''              -- Full event payload as JSON

) ENGINE = MergeTree()
PARTITION BY toYYYYMM(timestamp)
ORDER BY (timestamp, session_id, event_type)
TTL timestamp + INTERVAL 90 DAY
SETTINGS index_granularity = 8192;

-- Indexes for common queries
ALTER TABLE telemetry_events ADD INDEX IF NOT EXISTS idx_session_id session_id TYPE bloom_filter GRANULARITY 4;
ALTER TABLE telemetry_events ADD INDEX IF NOT EXISTS idx_user_id user_id TYPE bloom_filter GRANULARITY 4;
ALTER TABLE telemetry_events ADD INDEX IF NOT EXISTS idx_event_type event_type TYPE set(0) GRANULARITY 1;
ALTER TABLE telemetry_events ADD INDEX IF NOT EXISTS idx_feature_name feature_name TYPE bloom_filter GRANULARITY 4;

-- Materialized view for session summaries (pre-aggregated analytics)
CREATE MATERIALIZED VIEW IF NOT EXISTS session_summaries
ENGINE = SummingMergeTree()
PARTITION BY toYYYYMM(session_date)
ORDER BY (session_date, session_id, user_id)
AS SELECT
  toDate(timestamp) AS session_date,
  session_id,
  user_id,
  organization_id,
  feature_name,
  count() AS event_count,
  countIf(event_type = 'api_error') AS error_count,
  sum(tokens_input) AS total_tokens_input,
  sum(tokens_output) AS total_tokens_output,
  sum(cost_usd) AS total_cost_usd,
  avg(latency_ms) AS avg_latency_ms,
  min(timestamp) AS session_start,
  max(timestamp) AS session_end,
  dateDiff('minute', min(timestamp), max(timestamp)) AS session_duration_minutes
FROM telemetry_events
GROUP BY session_date, session_id, user_id, organization_id, feature_name;

-- Materialized view for error patterns (Andrew Ng's error analysis workflow)
CREATE MATERIALIZED VIEW IF NOT EXISTS error_patterns
ENGINE = AggregatingMergeTree()
PARTITION BY toYYYYMM(error_date)
ORDER BY (error_date, api_error_code, tool_name)
AS SELECT
  toDate(timestamp) AS error_date,
  api_error_code,
  tool_name,
  workflow_step,
  feature_name,
  count() AS error_occurrences,
  uniqExact(session_id) AS affected_sessions,
  uniqExact(user_id) AS affected_users,
  anyLast(api_error_message) AS sample_error_message
FROM telemetry_events
WHERE event_type = 'api_error'
GROUP BY error_date, api_error_code, tool_name, workflow_step, feature_name;

-- Materialized view for tool usage patterns
CREATE MATERIALIZED VIEW IF NOT EXISTS tool_usage_patterns
ENGINE = SummingMergeTree()
PARTITION BY toYYYYMM(usage_date)
ORDER BY (usage_date, tool_name, workflow_step)
AS SELECT
  toDate(timestamp) AS usage_date,
  tool_name,
  workflow_step,
  agent_type,
  count() AS usage_count,
  avg(latency_ms) AS avg_latency_ms,
  countIf(api_status = 'success') AS success_count,
  countIf(api_status = 'error') AS error_count
FROM telemetry_events
WHERE event_type = 'tool_call'
GROUP BY usage_date, tool_name, workflow_step, agent_type;

-- Create read-only user for query access (optional, for team sharing)
CREATE USER IF NOT EXISTS telemetry_reader IDENTIFIED BY '${TELEMETRY_READER_PASSWORD:-readonly_password}';
GRANT SELECT ON telemetry.* TO telemetry_reader;

-- Verification query
SELECT 'Schema initialized successfully' AS status,
       count() AS table_count
FROM system.tables
WHERE database = 'telemetry';
