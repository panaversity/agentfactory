-- ClickHouse Schema for Telemetry Storage (Feature 017)
-- This file defines the complete database schema for the usage data collection system
-- See data-model.md for detailed documentation

-- ============================================================================
-- PRIMARY TABLE: telemetry_events (Raw event stream - insert optimized)
-- ============================================================================

CREATE TABLE IF NOT EXISTS telemetry_events (
  event_id UUID DEFAULT generateUUIDv4(),
  event_type String,                           -- user_prompt | tool_call | api_request | api_error | validation_result | workflow_checkpoint
  session_id UUID,
  user_id String,
  organization_id String,
  timestamp DateTime,

  -- Common attributes for all events
  code_version String,
  terminal_type Nullable(String),
  git_branch String,
  git_commit Nullable(String),
  feature_id String,
  chapter_number Nullable(UInt16),

  -- Event-specific attributes (sparse columns - most will be NULL for non-matching event types)
  prompt_length_chars Nullable(UInt32),
  prompt_tokens_estimated Nullable(UInt16),
  prompt_text_hash Nullable(String),
  prompt_model_id Nullable(String),
  prompt_temperature Nullable(Float32),
  prompt_max_tokens Nullable(UInt32),

  tool_name Nullable(String),
  tool_call_id Nullable(String),
  tool_status Nullable(String),
  tool_duration_ms Nullable(UInt32),
  file_path Nullable(String),
  file_lines_changed Nullable(Int32),
  bash_command_hash Nullable(String),
  bash_exit_code Nullable(Int32),
  tool_output_length_chars Nullable(UInt32),

  api_name Nullable(String),
  api_status Nullable(String),
  api_http_status Nullable(UInt16),
  api_duration_ms Nullable(UInt32),
  tokens_input Nullable(UInt32),
  tokens_output Nullable(UInt32),
  tokens_cache_read Nullable(UInt32),
  cost_usd Nullable(Decimal(10, 6)),
  api_model_id Nullable(String),
  stop_reason Nullable(String),

  error_type Nullable(String),
  error_message_hash Nullable(String),
  error_http_status Nullable(UInt16),
  retry_attempt Nullable(UInt8),
  retry_after_seconds Nullable(UInt16),

  validation_outcome Nullable(String),
  output_type Nullable(String),
  quality_score Nullable(UInt8),
  feedback_text_hash Nullable(String),
  feedback_text_length_chars Nullable(UInt32),
  time_to_validate_seconds Nullable(UInt32),

  checkpoint_type Nullable(String),
  elapsed_time_minutes Nullable(UInt32),
  artifacts_created Nullable(UInt16),

  -- Metadata
  collected_at DateTime DEFAULT now(),

  -- Indexes for common query patterns
  INDEX idx_session_id session_id TYPE hash,
  INDEX idx_user_id user_id TYPE hash,
  INDEX idx_timestamp timestamp TYPE minmax,
  INDEX idx_event_type event_type TYPE set(10),
  INDEX idx_feature_id feature_id TYPE hash

) ENGINE = MergeTree()
  ORDER BY (timestamp, session_id)
  TTL timestamp + INTERVAL 90 day DELETE
  PARTITION BY toYYYYMM(timestamp)
  SETTINGS
    storage_policy = 'default',
    codec = 'zstd(19)',
    allow_nullable_key = 1;

-- ============================================================================
-- SESSION SUMMARY TABLE: sessions (Pre-aggregated session metrics)
-- ============================================================================

CREATE TABLE IF NOT EXISTS sessions (
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

  -- Metadata
  created_at DateTime DEFAULT now(),

  -- Indexes
  INDEX idx_user_id user_id TYPE hash,
  INDEX idx_feature_id feature_id TYPE hash,
  INDEX idx_start_timestamp start_timestamp TYPE minmax

) ENGINE = MergeTree()
  ORDER BY (start_timestamp DESC, user_id)
  PRIMARY KEY (session_id)
  PARTITION BY toYYYYMM(start_timestamp);

-- ============================================================================
-- USER DIRECTORY: users (Team member metadata)
-- ============================================================================

CREATE TABLE IF NOT EXISTS users (
  user_id String,
  user_email String,
  organization_id String,
  full_name_hash String,
  first_session DateTime,
  last_session DateTime,
  total_sessions UInt32,
  role String,  -- author | reviewer | coordinator

  updated_at DateTime DEFAULT now()

) ENGINE = ReplacingMergeTree(updated_at)
  ORDER BY (user_id)
  PRIMARY KEY (user_id);

-- ============================================================================
-- COST METRICS TABLE: daily_cost_metrics (Pre-aggregated for reporting)
-- ============================================================================

CREATE TABLE IF NOT EXISTS daily_cost_metrics (
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
  ORDER BY (metric_date DESC, user_id, feature_id)
  PARTITION BY toYYYYMM(metric_date);

-- ============================================================================
-- MATERIALIZED VIEW: cost_by_chapter_daily (Fast cost analysis)
-- ============================================================================

CREATE MATERIALIZED VIEW IF NOT EXISTS cost_by_chapter_daily TO daily_cost_metrics AS
SELECT
  toDate(timestamp) as metric_date,
  user_id,
  organization_id,
  feature_id,
  chapter_number,

  COUNT(DISTINCT session_id) as session_count,
  COUNT(*) as event_count,
  SUM(tokens_input) as tokens_input,
  SUM(tokens_output) as tokens_output,
  SUM(cost_usd) as total_cost_usd,
  SUM(cost_usd) / COUNT(DISTINCT session_id) as avg_cost_per_session,

  SUM(IF(event_type = 'api_error', 1, 0)) as api_errors,
  SUM(IF(event_type = 'validation_result' AND validation_outcome = 'rejected', 1, 0)) as validation_failures

FROM telemetry_events
WHERE event_type IN ('api_request', 'api_error', 'validation_result')
  AND cost_usd IS NOT NULL
GROUP BY metric_date, user_id, organization_id, feature_id, chapter_number;

-- ============================================================================
-- MATERIALIZED VIEW: error_patterns (Error analysis)
-- ============================================================================

CREATE TABLE IF NOT EXISTS error_patterns (
  error_type String,
  occurrences UInt32,
  affected_sessions UInt32,
  affected_users UInt32,
  last_occurred DateTime,
  first_occurred DateTime

) ENGINE = SummingMergeTree()
  ORDER BY (occurrences DESC, error_type);

CREATE MATERIALIZED VIEW IF NOT EXISTS error_patterns_mv TO error_patterns AS
SELECT
  error_type,
  COUNT(*) as occurrences,
  COUNT(DISTINCT session_id) as affected_sessions,
  COUNT(DISTINCT user_id) as affected_users,
  MAX(timestamp) as last_occurred,
  MIN(timestamp) as first_occurred
FROM telemetry_events
WHERE event_type = 'api_error' AND error_type IS NOT NULL
GROUP BY error_type;

-- ============================================================================
-- VIEW: cost_analysis (Cost by user, chapter, date)
-- ============================================================================

CREATE VIEW IF NOT EXISTS v_cost_by_user AS
SELECT
  user_id,
  SUM(cost_usd) as total_spent,
  COUNT(DISTINCT session_id) as sessions,
  ROUND(SUM(cost_usd) / COUNT(DISTINCT session_id), 6) as avg_cost_per_session
FROM telemetry_events
WHERE event_type = 'api_request' AND cost_usd IS NOT NULL
GROUP BY user_id
ORDER BY total_spent DESC;

CREATE VIEW IF NOT EXISTS v_cost_by_chapter AS
SELECT
  chapter_number,
  COUNT(DISTINCT session_id) as sessions,
  SUM(cost_usd) as total_cost,
  ROUND(SUM(cost_usd) / COUNT(DISTINCT session_id), 6) as avg_cost_per_session
FROM telemetry_events
WHERE event_type = 'api_request'
  AND cost_usd IS NOT NULL
  AND chapter_number IS NOT NULL
GROUP BY chapter_number
ORDER BY total_cost DESC;

CREATE VIEW IF NOT EXISTS v_cost_by_feature AS
SELECT
  feature_id,
  COUNT(DISTINCT session_id) as sessions,
  SUM(cost_usd) as total_cost,
  ROUND(SUM(cost_usd) / COUNT(DISTINCT session_id), 6) as avg_cost_per_session
FROM telemetry_events
WHERE event_type = 'api_request' AND cost_usd IS NOT NULL
GROUP BY feature_id
ORDER BY total_cost DESC;

-- ============================================================================
-- VIEW: error_analysis (Error rate trends)
-- ============================================================================

CREATE VIEW IF NOT EXISTS v_error_rate_by_feature AS
SELECT
  feature_id,
  COUNT(*) as total_events,
  SUM(IF(event_type = 'api_error', 1, 0)) as error_count,
  ROUND(SUM(IF(event_type = 'api_error', 1, 0)) / COUNT(*) * 100, 2) as error_rate_percent,
  COUNT(DISTINCT user_id) as users_affected
FROM telemetry_events
GROUP BY feature_id
ORDER BY error_count DESC;

CREATE VIEW IF NOT EXISTS v_most_common_errors AS
SELECT
  error_type,
  COUNT(*) as occurrences,
  COUNT(DISTINCT session_id) as sessions,
  COUNT(DISTINCT user_id) as users,
  MAX(timestamp) as last_occurred
FROM telemetry_events
WHERE event_type = 'api_error' AND error_type IS NOT NULL
GROUP BY error_type
ORDER BY occurrences DESC;

-- ============================================================================
-- VIEW: token_efficiency (Token usage analysis)
-- ============================================================================

CREATE VIEW IF NOT EXISTS v_token_efficiency_by_session AS
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
ORDER BY session_id DESC;

CREATE VIEW IF NOT EXISTS v_token_efficiency_by_feature AS
SELECT
  feature_id,
  COUNT(DISTINCT session_id) as sessions,
  SUM(tokens_input) as total_input_tokens,
  SUM(tokens_output) as total_output_tokens,
  ROUND(SUM(tokens_output) / SUM(tokens_input), 3) as output_input_ratio
FROM telemetry_events
WHERE event_type = 'api_request' AND tokens_input > 0
GROUP BY feature_id
ORDER BY total_input_tokens DESC;

-- ============================================================================
-- QUERY HELPER: workflow_trace (Get complete trace for a session)
-- ============================================================================

-- NOTE: This is a template query, not a materialized view
-- Use in queries to drill down into individual sessions:
--
-- SELECT
--   timestamp,
--   event_type,
--   CASE
--     WHEN event_type = 'user_prompt' THEN CONCAT('length:', prompt_length_chars, ' tokens:', prompt_tokens_estimated)
--     WHEN event_type = 'api_request' THEN CONCAT('tokens:', tokens_input, '→', tokens_output, ' cost:', cost_usd)
--     WHEN event_type = 'api_error' THEN CONCAT(error_type, ' (', error_http_status, ')')
--     WHEN event_type = 'tool_call' THEN CONCAT(tool_name, ': ', tool_status)
--     WHEN event_type = 'validation_result' THEN CONCAT(output_type, ' ', validation_outcome)
--     ELSE ''
--   END as detail
-- FROM telemetry_events
-- WHERE session_id = 'YOUR_SESSION_ID'
-- ORDER BY timestamp ASC;

-- ============================================================================
-- MONITORING: Data retention and cleanup
-- ============================================================================

-- Raw events have TTL of 90 days (defined in table ENGINE)
-- To manually trigger deletion of old data:
-- ALTER TABLE telemetry_events DELETE WHERE timestamp < today() - 90;

-- To check table sizes:
-- SELECT
--   table,
--   sum(bytes) as size_bytes,
--   formatReadableSize(size_bytes) as size_readable
-- FROM system.parts
-- WHERE database = 'default'
-- GROUP BY table
-- ORDER BY size_bytes DESC;

-- ============================================================================
-- END OF SCHEMA
-- ============================================================================
