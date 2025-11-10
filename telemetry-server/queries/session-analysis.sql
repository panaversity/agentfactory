-- Session Analysis Queries
-- Analyze work sessions, productivity patterns, and resource usage

-- ============================================
-- 1. RECENT SESSIONS OVERVIEW
-- ============================================
-- Goal: Quick overview of recent work sessions

SELECT
    session_id,
    user_id,
    feature_name,
    count() AS event_count,
    min(timestamp) AS session_start,
    max(timestamp) AS session_end,
    dateDiff('minute', min(timestamp), max(timestamp)) AS duration_minutes,
    sum(tokens_input) AS total_input_tokens,
    sum(tokens_output) AS total_output_tokens,
    sum(cost_usd) AS total_cost_usd,
    countIf(event_type = 'api_error') AS error_count,
    round(countIf(event_type = 'api_error') / count() * 100, 2) AS error_rate_pct
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 24 HOUR
GROUP BY session_id, user_id, feature_name
ORDER BY session_start DESC
LIMIT 20;

-- ============================================
-- 2. SESSION DURATION DISTRIBUTION
-- ============================================
-- Goal: Understand typical session lengths

SELECT
    CASE
        WHEN duration_minutes < 5 THEN '0-5 min'
        WHEN duration_minutes < 15 THEN '5-15 min'
        WHEN duration_minutes < 30 THEN '15-30 min'
        WHEN duration_minutes < 60 THEN '30-60 min'
        WHEN duration_minutes < 120 THEN '1-2 hours'
        WHEN duration_minutes < 240 THEN '2-4 hours'
        ELSE '4+ hours'
    END AS duration_bucket,
    count() AS session_count,
    round(avg(event_count), 0) AS avg_events_per_session,
    round(avg(cost_usd), 4) AS avg_cost_per_session
FROM (
    SELECT
        session_id,
        dateDiff('minute', min(timestamp), max(timestamp)) AS duration_minutes,
        count() AS event_count,
        sum(cost_usd) AS cost_usd
    FROM telemetry.telemetry_events
    WHERE timestamp >= now() - INTERVAL 7 DAY
    GROUP BY session_id
)
GROUP BY duration_bucket
ORDER BY
    CASE duration_bucket
        WHEN '0-5 min' THEN 1
        WHEN '5-15 min' THEN 2
        WHEN '15-30 min' THEN 3
        WHEN '30-60 min' THEN 4
        WHEN '1-2 hours' THEN 5
        WHEN '2-4 hours' THEN 6
        ELSE 7
    END;

-- ============================================
-- 3. DAILY PRODUCTIVITY METRICS
-- ============================================
-- Goal: Track daily activity and productivity

SELECT
    toDate(timestamp) AS date,
    uniqExact(session_id) AS total_sessions,
    uniqExact(user_id) AS active_users,
    count() AS total_events,
    countIf(event_type = 'user_prompt') AS prompts_count,
    countIf(event_type = 'tool_call') AS tool_calls,
    sum(tokens_input + tokens_output) AS total_tokens,
    sum(cost_usd) AS total_cost_usd,
    round(avg(latency_ms), 0) AS avg_latency_ms
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 30 DAY
GROUP BY date
ORDER BY date DESC;

-- ============================================
-- 4. PEAK USAGE HOURS
-- ============================================
-- Goal: Identify when team is most active

SELECT
    toHour(timestamp) AS hour_of_day,
    count() AS event_count,
    uniqExact(session_id) AS active_sessions,
    uniqExact(user_id) AS active_users,
    round(avg(latency_ms), 0) AS avg_latency_ms
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 7 DAY
GROUP BY hour_of_day
ORDER BY hour_of_day;

-- ============================================
-- 5. SESSION WORKFLOW COMPLETENESS
-- ============================================
-- Goal: How many sessions complete full SDD workflow?

WITH session_workflows AS (
    SELECT
        session_id,
        user_id,
        feature_name,
        groupUniqArray(workflow_step) AS completed_steps,
        count() AS event_count,
        min(timestamp) AS session_start,
        max(timestamp) AS session_end
    FROM telemetry.telemetry_events
    WHERE workflow_step IS NOT NULL
      AND timestamp >= now() - INTERVAL 7 DAY
    GROUP BY session_id, user_id, feature_name
)
SELECT
    feature_name,
    count() AS total_sessions,
    countIf(has(completed_steps, 'specify')) AS has_specify,
    countIf(has(completed_steps, 'plan')) AS has_plan,
    countIf(has(completed_steps, 'implement')) AS has_implement,
    countIf(has(completed_steps, 'validate')) AS has_validate,
    countIf(
        has(completed_steps, 'specify') AND
        has(completed_steps, 'plan') AND
        has(completed_steps, 'implement') AND
        has(completed_steps, 'validate')
    ) AS complete_workflow_count,
    round(
        countIf(
            has(completed_steps, 'specify') AND
            has(completed_steps, 'plan') AND
            has(completed_steps, 'implement') AND
            has(completed_steps, 'validate')
        ) / count() * 100, 2
    ) AS complete_workflow_pct
FROM session_workflows
GROUP BY feature_name
ORDER BY total_sessions DESC;

-- ============================================
-- 6. USER PRODUCTIVITY COMPARISON
-- ============================================
-- Goal: Compare productivity across team members

SELECT
    user_id,
    uniqExact(session_id) AS total_sessions,
    count() AS total_events,
    round(count() / uniqExact(session_id), 0) AS avg_events_per_session,
    sum(tokens_input + tokens_output) AS total_tokens,
    sum(cost_usd) AS total_cost_usd,
    round(avg(latency_ms), 0) AS avg_latency_ms,
    countIf(event_type = 'api_error') AS error_count,
    round(countIf(event_type = 'api_error') / count() * 100, 2) AS error_rate_pct,
    uniqExact(feature_name) AS features_worked_on
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 7 DAY
GROUP BY user_id
ORDER BY total_sessions DESC;

-- ============================================
-- 7. FEATURE DEVELOPMENT TIME ANALYSIS
-- ============================================
-- Goal: How long does it take to complete features?

SELECT
    feature_name,
    count(DISTINCT session_id) AS sessions_count,
    count(DISTINCT user_id) AS contributors,
    min(timestamp) AS feature_start,
    max(timestamp) AS feature_end,
    dateDiff('day', min(timestamp), max(timestamp)) AS development_days,
    count() AS total_events,
    sum(cost_usd) AS total_cost_usd,
    round(avg(latency_ms), 0) AS avg_latency_ms
FROM telemetry.telemetry_events
WHERE feature_name IS NOT NULL
  AND timestamp >= now() - INTERVAL 30 DAY
GROUP BY feature_name
ORDER BY development_days DESC, total_events DESC;

-- ============================================
-- 8. SESSION INTENSITY ANALYSIS
-- ============================================
-- Goal: Events per minute (high intensity = lots of activity)

SELECT
    session_id,
    user_id,
    feature_name,
    count() AS event_count,
    dateDiff('minute', min(timestamp), max(timestamp)) AS duration_minutes,
    CASE
        WHEN dateDiff('minute', min(timestamp), max(timestamp)) > 0
        THEN round(count() / dateDiff('minute', min(timestamp), max(timestamp)), 2)
        ELSE 0
    END AS events_per_minute,
    sum(cost_usd) AS total_cost_usd,
    countIf(event_type = 'api_error') AS error_count
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 7 DAY
GROUP BY session_id, user_id, feature_name
HAVING duration_minutes > 5
ORDER BY events_per_minute DESC
LIMIT 20;

-- ============================================
-- 9. LONG-RUNNING SESSIONS
-- ============================================
-- Goal: Identify sessions that might need optimization

SELECT
    session_id,
    user_id,
    feature_name,
    min(timestamp) AS session_start,
    max(timestamp) AS session_end,
    dateDiff('hour', min(timestamp), max(timestamp)) AS duration_hours,
    count() AS event_count,
    sum(tokens_input + tokens_output) AS total_tokens,
    sum(cost_usd) AS total_cost_usd,
    countIf(event_type = 'api_error') AS error_count,
    groupArray(DISTINCT workflow_step) AS workflow_steps_completed
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 7 DAY
GROUP BY session_id, user_id, feature_name
HAVING duration_hours >= 2
ORDER BY duration_hours DESC
LIMIT 20;

-- ============================================
-- 10. SESSION SUCCESS RATE
-- ============================================
-- Goal: What percentage of sessions complete without errors?

WITH session_metrics AS (
    SELECT
        session_id,
        count() AS event_count,
        countIf(event_type = 'api_error') AS error_count,
        CASE WHEN countIf(event_type = 'api_error') = 0 THEN 1 ELSE 0 END AS is_error_free
    FROM telemetry.telemetry_events
    WHERE timestamp >= now() - INTERVAL 7 DAY
    GROUP BY session_id
)
SELECT
    count() AS total_sessions,
    sum(is_error_free) AS error_free_sessions,
    count() - sum(is_error_free) AS sessions_with_errors,
    round(sum(is_error_free) / count() * 100, 2) AS error_free_pct,
    round(avg(event_count), 0) AS avg_events_per_session,
    round(avg(error_count), 2) AS avg_errors_per_session
FROM session_metrics;

-- ============================================
-- 11. WEEKLY TREND ANALYSIS
-- ============================================
-- Goal: Week-over-week growth and trends

SELECT
    toStartOfWeek(timestamp) AS week_start,
    uniqExact(session_id) AS sessions,
    uniqExact(user_id) AS active_users,
    count() AS total_events,
    sum(cost_usd) AS total_cost_usd,
    round(avg(latency_ms), 0) AS avg_latency_ms,
    countIf(event_type = 'api_error') AS errors,
    round(countIf(event_type = 'api_error') / count() * 100, 2) AS error_rate_pct
FROM telemetry.telemetry_events
WHERE timestamp >= now() - INTERVAL 12 WEEK
GROUP BY week_start
ORDER BY week_start DESC;

-- ============================================
-- 12. SESSION RECONSTRUCTION (Full Trace)
-- ============================================
-- Goal: Reconstruct complete event sequence for a specific session

-- USAGE: Replace 'YOUR_SESSION_ID' with actual session_id
-- SELECT
--     timestamp,
--     event_type,
--     workflow_step,
--     tool_name,
--     agent_type,
--     api_status,
--     latency_ms,
--     tokens_input,
--     tokens_output,
--     cost_usd,
--     api_error_message,
--     file_path
-- FROM telemetry.telemetry_events
-- WHERE session_id = 'YOUR_SESSION_ID'
-- ORDER BY timestamp ASC;

-- ============================================
-- INSIGHTS & RECOMMENDATIONS
-- ============================================
-- 1. Sessions < 5 minutes may indicate setup/configuration issues
-- 2. Sessions > 4 hours may need workflow optimization
-- 3. Error-free percentage should be > 95% for production
-- 4. Events per minute > 10 may indicate inefficient retry loops
-- 5. Features taking > 7 days may need scope reduction
-- ============================================
