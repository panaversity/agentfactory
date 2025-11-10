-- Tool Usage Analysis Queries
-- Understand which tools are used, how often, and their performance

-- ============================================
-- 1. TOP 20 MOST USED TOOLS
-- ============================================
-- Goal: Identify most frequently invoked tools

SELECT
    tool_name,
    count() AS usage_count,
    uniqExact(session_id) AS used_in_sessions,
    uniqExact(user_id) AS used_by_users,
    round(avg(latency_ms), 0) AS avg_latency_ms,
    quantile(0.50)(latency_ms) AS p50_latency_ms,
    quantile(0.95)(latency_ms) AS p95_latency_ms,
    countIf(api_status = 'success') AS success_count,
    countIf(api_status = 'error') AS error_count,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IS NOT NULL
GROUP BY tool_name
ORDER BY usage_count DESC
LIMIT 20;

-- ============================================
-- 2. TOOL PERFORMANCE COMPARISON
-- ============================================
-- Goal: Compare latency across different tools

SELECT
    tool_name,
    count() AS invocations,
    round(avg(latency_ms), 0) AS avg_latency,
    quantile(0.50)(latency_ms) AS p50_latency,
    quantile(0.95)(latency_ms) AS p95_latency,
    quantile(0.99)(latency_ms) AS p99_latency,
    max(latency_ms) AS max_latency,
    CASE
        WHEN avg(latency_ms) < 100 THEN 'Fast'
        WHEN avg(latency_ms) < 500 THEN 'Moderate'
        WHEN avg(latency_ms) < 2000 THEN 'Slow'
        ELSE 'Very Slow'
    END AS performance_category
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IS NOT NULL
  AND latency_ms IS NOT NULL
GROUP BY tool_name
ORDER BY avg_latency DESC;

-- ============================================
-- 3. TOOL USAGE BY WORKFLOW PHASE
-- ============================================
-- Goal: Which tools are used in which workflow phases?

SELECT
    workflow_step,
    tool_name,
    count() AS usage_count,
    round(avg(latency_ms), 0) AS avg_latency_ms,
    countIf(api_status = 'error') AS error_count,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IS NOT NULL
  AND workflow_step IS NOT NULL
GROUP BY workflow_step, tool_name
ORDER BY workflow_step, usage_count DESC;

-- ============================================
-- 4. TOOL RELIABILITY RANKING
-- ============================================
-- Goal: Most reliable tools (lowest error rate)

SELECT
    tool_name,
    count() AS total_invocations,
    countIf(api_status = 'success') AS success_count,
    countIf(api_status = 'error') AS error_count,
    round(countIf(api_status = 'success') / count() * 100, 2) AS success_rate_pct,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct,
    CASE
        WHEN countIf(api_status = 'success') / count() >= 0.99 THEN '⭐⭐⭐⭐⭐ Excellent'
        WHEN countIf(api_status = 'success') / count() >= 0.95 THEN '⭐⭐⭐⭐ Good'
        WHEN countIf(api_status = 'success') / count() >= 0.90 THEN '⭐⭐⭐ Fair'
        WHEN countIf(api_status = 'success') / count() >= 0.80 THEN '⭐⭐ Poor'
        ELSE '⭐ Critical'
    END AS reliability_rating
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IS NOT NULL
  AND api_status IS NOT NULL
GROUP BY tool_name
HAVING total_invocations >= 10  -- Minimum sample size
ORDER BY success_rate_pct DESC, total_invocations DESC;

-- ============================================
-- 5. TOOL USAGE TRENDS OVER TIME
-- ============================================
-- Goal: Daily tool usage trends

SELECT
    toDate(timestamp) AS date,
    tool_name,
    count() AS usage_count,
    round(avg(latency_ms), 0) AS avg_latency_ms,
    countIf(api_status = 'error') AS error_count
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 30 DAY
  AND tool_name IN (
      -- Top 5 most used tools
      SELECT tool_name
      FROM telemetry.telemetry_events
      WHERE event_type = 'tool_call'
      GROUP BY tool_name
      ORDER BY count() DESC
      LIMIT 5
  )
GROUP BY date, tool_name
ORDER BY date DESC, usage_count DESC;

-- ============================================
-- 6. TOOL COMBINATION PATTERNS
-- ============================================
-- Goal: Which tools are frequently used together in same session?

WITH tool_sessions AS (
    SELECT
        session_id,
        groupArray(DISTINCT tool_name) AS tools_used
    FROM telemetry.telemetry_events
    WHERE event_type = 'tool_call'
      AND timestamp >= now() - INTERVAL 7 DAY
      AND tool_name IS NOT NULL
    GROUP BY session_id
)
SELECT
    arraySort(tools_used) AS tool_combination,
    count() AS session_count,
    length(tools_used) AS tools_in_combination
FROM tool_sessions
WHERE length(tools_used) >= 2 AND length(tools_used) <= 5
GROUP BY tools_used
ORDER BY session_count DESC
LIMIT 20;

-- ============================================
-- 7. SLOWEST TOOL INVOCATIONS
-- ============================================
-- Goal: Identify outlier slow invocations

SELECT
    timestamp,
    session_id,
    user_id,
    tool_name,
    latency_ms,
    api_status,
    workflow_step,
    feature_name
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND latency_ms IS NOT NULL
ORDER BY latency_ms DESC
LIMIT 50;

-- ============================================
-- 8. TOOL USAGE BY AGENT TYPE
-- ============================================
-- Goal: Which agents use which tools?

SELECT
    agent_type,
    tool_name,
    count() AS usage_count,
    round(avg(latency_ms), 0) AS avg_latency_ms,
    countIf(api_status = 'error') AS error_count
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND agent_type IS NOT NULL
  AND tool_name IS NOT NULL
GROUP BY agent_type, tool_name
ORDER BY agent_type, usage_count DESC;

-- ============================================
-- 9. HOURLY TOOL USAGE HEATMAP
-- ============================================
-- Goal: When are specific tools most used?

SELECT
    tool_name,
    toHour(timestamp) AS hour_of_day,
    count() AS usage_count,
    round(avg(latency_ms), 0) AS avg_latency_ms
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IN (
      SELECT tool_name
      FROM telemetry.telemetry_events
      WHERE event_type = 'tool_call'
      GROUP BY tool_name
      ORDER BY count() DESC
      LIMIT 10
  )
GROUP BY tool_name, hour_of_day
ORDER BY tool_name, hour_of_day;

-- ============================================
-- 10. TOOL ADOPTION RATE
-- ============================================
-- Goal: Are new tools being adopted?

WITH first_use AS (
    SELECT
        tool_name,
        min(timestamp) AS first_used_date,
        count(DISTINCT user_id) AS total_users
    FROM telemetry.telemetry_events
    WHERE event_type = 'tool_call'
      AND tool_name IS NOT NULL
    GROUP BY tool_name
)
SELECT
    tool_name,
    first_used_date,
    dateDiff('day', first_used_date, now()) AS days_since_first_use,
    total_users AS users_adopted,
    CASE
        WHEN dateDiff('day', first_used_date, now()) <= 7 THEN 'New (< 1 week)'
        WHEN dateDiff('day', first_used_date, now()) <= 30 THEN 'Recent (< 1 month)'
        WHEN dateDiff('day', first_used_date, now()) <= 90 THEN 'Established (< 3 months)'
        ELSE 'Mature (> 3 months)'
    END AS maturity_level
FROM first_use
ORDER BY first_used_date DESC
LIMIT 20;

-- ============================================
-- 11. TOOL ERROR HOTSPOTS
-- ============================================
-- Goal: Which tool + workflow combinations have highest errors?

SELECT
    tool_name,
    workflow_step,
    count() AS invocations,
    countIf(api_status = 'error') AS errors,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct,
    groupArray(DISTINCT api_error_code) AS error_types
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IS NOT NULL
GROUP BY tool_name, workflow_step
HAVING errors > 0
ORDER BY error_rate_pct DESC, errors DESC
LIMIT 20;

-- ============================================
-- 12. TOOL EFFICIENCY SCORE
-- ============================================
-- Goal: Composite score based on speed, reliability, and usage

WITH tool_metrics AS (
    SELECT
        tool_name,
        count() AS usage_count,
        avg(latency_ms) AS avg_latency,
        countIf(api_status = 'success') / count() AS success_rate
    FROM telemetry.telemetry_events
    WHERE event_type = 'tool_call'
      AND timestamp >= now() - INTERVAL 7 DAY
      AND tool_name IS NOT NULL
    GROUP BY tool_name
    HAVING usage_count >= 10
)
SELECT
    tool_name,
    usage_count,
    round(avg_latency, 0) AS avg_latency_ms,
    round(success_rate * 100, 2) AS success_rate_pct,
    -- Efficiency score: 40% success rate, 40% speed (inverted latency), 20% popularity
    round(
        (success_rate * 0.4 +
         (1 - least(avg_latency / 5000, 1)) * 0.4 +
         (least(usage_count / 1000, 1)) * 0.2) * 100,
        1
    ) AS efficiency_score
FROM tool_metrics
ORDER BY efficiency_score DESC;

-- ============================================
-- 13. TOOL PARAMETER ANALYSIS
-- ============================================
-- Goal: Are certain parameter patterns causing errors?

SELECT
    tool_name,
    tool_parameters_hash,
    count() AS usage_count,
    countIf(api_status = 'success') AS success_count,
    countIf(api_status = 'error') AS error_count,
    round(countIf(api_status = 'error') / count() * 100, 2) AS error_rate_pct,
    round(avg(latency_ms), 0) AS avg_latency_ms
FROM telemetry.telemetry_events
WHERE event_type = 'tool_call'
  AND timestamp >= now() - INTERVAL 7 DAY
  AND tool_name IS NOT NULL
  AND tool_parameters_hash IS NOT NULL
GROUP BY tool_name, tool_parameters_hash
HAVING usage_count >= 5
ORDER BY error_rate_pct DESC, usage_count DESC
LIMIT 30;

-- ============================================
-- INSIGHTS & RECOMMENDATIONS
-- ============================================
-- 1. Tools with error_rate > 5% need investigation
-- 2. Tools with p95 latency > 2000ms need optimization
-- 3. Tools used in < 2 sessions may need better documentation
-- 4. Tool combinations with > 100 sessions indicate common workflows
-- 5. New tools (< 7 days) should have adoption tracking
-- ============================================
