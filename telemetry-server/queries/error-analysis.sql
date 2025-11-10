-- Error Analysis Queries
-- Following Andrew Ng's Error Analysis Methodology
-- https://www.deeplearning.ai/the-batch/improve-agentic-performance-with-evals-and-error-analysis-part-2/

-- ============================================
-- 1. ERROR PATTERN IDENTIFICATION
-- ============================================
-- Goal: Identify most common error patterns to prioritize fixes

SELECT
    api_error_code,
    tool_name,
    workflow_step,
    count() AS error_occurrences,
    uniqExact(session_id) AS affected_sessions,
    uniqExact(user_id) AS affected_users,
    round(count() / (SELECT count() FROM telemetry.telemetry_events WHERE event_type = 'api_error') * 100, 2) AS error_percentage,
    anyLast(api_error_message) AS sample_error_message,
    min(timestamp) AS first_occurrence,
    max(timestamp) AS last_occurrence
FROM telemetry.telemetry_events
WHERE event_type = 'api_error'
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY api_error_code, tool_name, workflow_step
ORDER BY error_occurrences DESC
LIMIT 20;

-- ============================================
-- 2. ERROR RATE BY WORKFLOW PHASE
-- ============================================
-- Goal: Identify which workflow phases have highest error rates

SELECT
    workflow_step,
    count() AS total_operations,
    countIf(api_status = 'error') AS error_count,
    countIf(api_status = 'success') AS success_count,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct,
    avg(latency_ms) AS avg_latency_ms
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 7 DAY
  AND workflow_step IS NOT NULL
  AND api_status IS NOT NULL
GROUP BY workflow_step
ORDER BY error_rate_pct DESC;

-- ============================================
-- 3. TOOL RELIABILITY ANALYSIS
-- ============================================
-- Goal: Which tools are most/least reliable?

SELECT
    tool_name,
    count() AS total_invocations,
    countIf(api_status = 'success') AS success_count,
    countIf(api_status = 'error') AS error_count,
    countIf(api_status = 'timeout') AS timeout_count,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct,
    round(countIf(api_status = 'timeout') / count() * 100, 2) AS timeout_rate_pct,
    avg(latency_ms) AS avg_latency_ms,
    quantile(0.95)(latency_ms) AS p95_latency_ms
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IS NOT NULL
GROUP BY tool_name
ORDER BY error_rate_pct DESC;

-- ============================================
-- 4. ERROR TREND OVER TIME
-- ============================================
-- Goal: Are errors increasing or decreasing?

SELECT
    toDate(timestamp) AS date,
    count() AS total_errors,
    uniqExact(session_id) AS affected_sessions,
    uniqExact(api_error_code) AS unique_error_types
FROM telemetry.telemetry_events
WHERE event_type = 'api_error'
  AND timestamp >= now() - INTERVAL 30 DAY
GROUP BY date
ORDER BY date DESC;

-- ============================================
-- 5. SESSION FAILURE ANALYSIS
-- ============================================
-- Goal: Identify sessions with high error rates

SELECT
    session_id,
    user_id,
    feature_name,
    count() AS total_events,
    countIf(event_type = 'api_error') AS error_count,
    round(countIf(event_type = 'api_error') / count() * 100, 2) AS error_rate_pct,
    min(timestamp) AS session_start,
    max(timestamp) AS session_end,
    dateDiff('minute', min(timestamp), max(timestamp)) AS duration_minutes,
    groupArray(DISTINCT api_error_code) AS error_codes
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 1 DAY
GROUP BY session_id, user_id, feature_name
HAVING error_count > 0
ORDER BY error_rate_pct DESC, error_count DESC
LIMIT 20;

-- ============================================
-- 6. ERROR CATEGORIZATION (Andrew Ng's Method)
-- ============================================
-- Goal: Group errors by category for prioritization

WITH error_categories AS (
    SELECT
        api_error_code,
        CASE
            WHEN api_error_code LIKE '%timeout%' OR api_error_code LIKE '%429%' THEN 'Infrastructure'
            WHEN api_error_code LIKE '%auth%' OR api_error_code LIKE '%401%' OR api_error_code LIKE '%403%' THEN 'Authentication'
            WHEN api_error_code LIKE '%validation%' OR api_error_code LIKE '%400%' THEN 'Input Validation'
            WHEN api_error_code LIKE '%500%' OR api_error_code LIKE '%502%' OR api_error_code LIKE '%503%' THEN 'Server Error'
            WHEN api_error_code LIKE '%not_found%' OR api_error_code LIKE '%404%' THEN 'Resource Not Found'
            ELSE 'Other'
        END AS error_category,
        count() AS occurrences,
        uniqExact(session_id) AS affected_sessions
    FROM telemetry.telemetry_events
    WHERE event_type = 'api_error'
      AND timestamp >= now() - INTERVAL 7 DAY
    GROUP BY api_error_code
)
SELECT
    error_category,
    count() AS error_types,
    sum(occurrences) AS total_occurrences,
    sum(affected_sessions) AS total_affected_sessions,
    round(sum(occurrences) / (SELECT sum(occurrences) FROM error_categories) * 100, 2) AS percentage_of_all_errors
FROM error_categories
GROUP BY error_category
ORDER BY total_occurrences DESC;

-- ============================================
-- 7. REPEATED ERRORS IN SAME SESSION
-- ============================================
-- Goal: Identify errors that repeatedly occur (potential workflow bugs)

SELECT
    session_id,
    user_id,
    api_error_code,
    count() AS error_repetitions,
    min(timestamp) AS first_error,
    max(timestamp) AS last_error,
    dateDiff('minute', min(timestamp), max(timestamp)) AS error_duration_minutes,
    anyLast(api_error_message) AS sample_message
FROM telemetry.telemetry_events
WHERE event_type = 'api_error'
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY session_id, user_id, api_error_code
HAVING error_repetitions >= 3
ORDER BY error_repetitions DESC, error_duration_minutes DESC
LIMIT 20;

-- ============================================
-- 8. ERROR CONTEXT ANALYSIS
-- ============================================
-- Goal: What events happen before errors?

WITH error_events AS (
    SELECT
        session_id,
        timestamp AS error_timestamp,
        tool_name AS error_tool,
        api_error_code
    FROM telemetry.telemetry_events
    WHERE event_type = 'api_error'
      AND timestamp >= now() - INTERVAL 1 DAY
)
SELECT
    e.api_error_code,
    t.event_type AS preceding_event_type,
    t.tool_name AS preceding_tool,
    count() AS occurrences,
    avg(dateDiff('second', t.timestamp, e.error_timestamp)) AS avg_seconds_before_error
FROM error_events e
JOIN telemetry.telemetry_events t
    ON e.session_id = t.session_id
    AND t.timestamp < e.error_timestamp
    AND t.timestamp >= e.error_timestamp - INTERVAL 5 MINUTE
GROUP BY e.api_error_code, t.event_type, t.tool_name
ORDER BY occurrences DESC
LIMIT 30;

-- ============================================
-- 9. AGENT-SPECIFIC ERROR RATES
-- ============================================
-- Goal: Which agents (chapter-planner, lesson-writer, etc.) have highest error rates?

SELECT
    agent_type,
    count() AS total_operations,
    countIf(api_status = 'error') AS error_count,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct,
    groupArray(DISTINCT api_error_code) AS error_types
FROM telemetry.telemetry_events
WHERE agent_type IS NOT NULL
  AND timestamp >= now() - INTERVAL 7 DAY
GROUP BY agent_type
ORDER BY error_rate_pct DESC;

-- ============================================
-- 10. ERROR RECOVERY TIME ANALYSIS
-- ============================================
-- Goal: How long until successful operation after error?

WITH error_recovery AS (
    SELECT
        session_id,
        tool_name,
        timestamp AS error_time,
        any(timestamp) AS next_success_time
    FROM telemetry.telemetry_events
    WHERE event_type = 'tool_call'
      AND timestamp >= now() - INTERVAL 7 DAY
    GROUP BY session_id, tool_name, timestamp
    HAVING any(api_status = 'error')
)
SELECT
    tool_name,
    count() AS error_occurrences,
    avg(dateDiff('second', error_time, next_success_time)) AS avg_recovery_time_seconds,
    quantile(0.50)(dateDiff('second', error_time, next_success_time)) AS median_recovery_time,
    max(dateDiff('second', error_time, next_success_time)) AS max_recovery_time
FROM error_recovery
WHERE next_success_time IS NOT NULL
GROUP BY tool_name
ORDER BY avg_recovery_time_seconds DESC;

-- ============================================
-- ANDREW NG'S RECOMMENDED WORKFLOW
-- ============================================
-- 1. Run query #1 (ERROR PATTERN IDENTIFICATION) to find top 5 error types
-- 2. For each top error, run query #8 (ERROR CONTEXT ANALYSIS) to understand cause
-- 3. Use query #5 (SESSION FAILURE ANALYSIS) to examine specific failed sessions
-- 4. Categorize errors using query #6 (ERROR CATEGORIZATION)
-- 5. Prioritize fixes based on:
--    - Error frequency (query #1)
--    - Business impact (query #2 - which workflow phases?)
--    - Recovery difficulty (query #10)
--
-- Goal: Reduce error rate by 50% in top 3 categories over next sprint
-- ============================================
