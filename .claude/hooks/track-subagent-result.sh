#!/usr/bin/env bash
# Track Task tool results (subagent completion)

# Read JSON input from stdin
INPUT=$(cat)

# Extract tool name - only process Task tool
TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty')
[ "$TOOL" != "Task" ] && exit 0

# Extract result info
SUBAGENT_TYPE=$(echo "$INPUT" | jq -r '.tool_input.subagent_type // "unknown"')
AGENT_ID=$(echo "$INPUT" | jq -r '.tool_result.agent_id // ""')
# Check if result contains error indicators
RESULT_TEXT=$(echo "$INPUT" | jq -r '.tool_result // ""')

# Determine status
if echo "$RESULT_TEXT" | grep -qi "error\|failed\|exception"; then
    STATUS="error"
else
    STATUS="completed"
fi

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"')
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs

# Write subagent completion event
jq -nc \
  --arg ts "$TIMESTAMP" \
  --arg sid "$SESSION_ID" \
  --arg agent "$SUBAGENT_TYPE" \
  --arg aid "$AGENT_ID" \
  --arg status "$STATUS" \
  '{timestamp: $ts, session_id: $sid, subagent: $agent, agent_id: $aid, status: $status, event: "complete"}' \
  >> .claude/activity-logs/subagent-usage.jsonl

exit 0
