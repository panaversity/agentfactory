#!/usr/bin/env bash
# Track Task tool invocations (subagent spawning)

# Read JSON input from stdin
INPUT=$(cat)

# Extract tool name - only process Task tool
TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty')
[ "$TOOL" != "Task" ] && exit 0

# Extract subagent details
SUBAGENT_TYPE=$(echo "$INPUT" | jq -r '.tool_input.subagent_type // "unknown"')
DESCRIPTION=$(echo "$INPUT" | jq -r '.tool_input.description // ""')
RUN_IN_BACKGROUND=$(echo "$INPUT" | jq -r '.tool_input.run_in_background // false')
MODEL=$(echo "$INPUT" | jq -r '.tool_input.model // "inherit"')

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"')
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs

# Write subagent spawn event
jq -nc \
  --arg ts "$TIMESTAMP" \
  --arg sid "$SESSION_ID" \
  --arg agent "$SUBAGENT_TYPE" \
  --arg desc "$DESCRIPTION" \
  --arg bg "$RUN_IN_BACKGROUND" \
  --arg model "$MODEL" \
  '{timestamp: $ts, session_id: $sid, subagent: $agent, description: $desc, background: ($bg == "true"), model: $model, event: "spawn"}' \
  >> .claude/activity-logs/subagent-usage.jsonl

exit 0
