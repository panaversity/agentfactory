#!/usr/bin/env bash
# Track Task tool invocations (subagent spawning)
# Silently exit on any error to avoid blocking Claude Code

# Read JSON input from stdin
INPUT=$(cat 2>/dev/null) || exit 0

# Validate input is JSON
echo "$INPUT" | jq -e . >/dev/null 2>&1 || exit 0

# Extract tool name - only process Task tool
TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty' 2>/dev/null) || exit 0
[ "$TOOL" != "Task" ] && exit 0

# Extract subagent details
SUBAGENT_TYPE=$(echo "$INPUT" | jq -r '.tool_input.subagent_type // "unknown"' 2>/dev/null) || exit 0
DESCRIPTION=$(echo "$INPUT" | jq -r '.tool_input.description // ""' 2>/dev/null) || DESCRIPTION=""
RUN_IN_BACKGROUND=$(echo "$INPUT" | jq -r '.tool_input.run_in_background // false' 2>/dev/null) || RUN_IN_BACKGROUND="false"
MODEL=$(echo "$INPUT" | jq -r '.tool_input.model // "inherit"' 2>/dev/null) || MODEL="inherit"

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"' 2>/dev/null) || exit 0
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs 2>/dev/null || exit 0

# Write subagent spawn event
jq -nc \
  --arg ts "$TIMESTAMP" \
  --arg sid "$SESSION_ID" \
  --arg agent "$SUBAGENT_TYPE" \
  --arg desc "$DESCRIPTION" \
  --arg bg "$RUN_IN_BACKGROUND" \
  --arg model "$MODEL" \
  '{timestamp: $ts, session_id: $sid, subagent: $agent, description: $desc, background: ($bg == "true"), model: $model, event: "spawn"}' \
  >> .claude/activity-logs/subagent-usage.jsonl 2>/dev/null

# Silent - no stdout to avoid hook errors
exit 0
