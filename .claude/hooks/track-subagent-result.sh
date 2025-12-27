#!/usr/bin/env bash
# Track Task tool results (subagent completion)
# Silently exit on any error to avoid blocking Claude Code

# Read JSON input from stdin
INPUT=$(cat 2>/dev/null) || exit 0

# Validate input is JSON
echo "$INPUT" | jq -e . >/dev/null 2>&1 || exit 0

# Extract tool name - only process Task tool
TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty' 2>/dev/null) || exit 0
[ "$TOOL" != "Task" ] && exit 0

# Extract result info
SUBAGENT_TYPE=$(echo "$INPUT" | jq -r '.tool_input.subagent_type // "unknown"' 2>/dev/null) || exit 0
AGENT_ID=$(echo "$INPUT" | jq -r '.tool_result.agent_id // ""' 2>/dev/null) || AGENT_ID=""
RESULT_TEXT=$(echo "$INPUT" | jq -r '.tool_result // ""' 2>/dev/null) || RESULT_TEXT=""

# Determine status
if echo "$RESULT_TEXT" | grep -qi "error\|failed\|exception" 2>/dev/null; then
    STATUS="error"
else
    STATUS="completed"
fi

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"' 2>/dev/null) || exit 0
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs 2>/dev/null || exit 0

# Write subagent completion event
jq -nc \
  --arg ts "$TIMESTAMP" \
  --arg sid "$SESSION_ID" \
  --arg agent "$SUBAGENT_TYPE" \
  --arg aid "$AGENT_ID" \
  --arg status "$STATUS" \
  '{timestamp: $ts, session_id: $sid, subagent: $agent, agent_id: $aid, status: $status, event: "complete"}' \
  >> .claude/activity-logs/subagent-usage.jsonl 2>/dev/null

# Silent - no stdout to avoid hook errors
exit 0
