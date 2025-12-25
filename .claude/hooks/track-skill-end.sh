#!/usr/bin/env bash
# Track skill verification results (sync - fast operation)

# Read JSON input from stdin
INPUT=$(cat)

# Extract command and exit code
COMMAND=$(echo "$INPUT" | jq -r '.tool_input.command // empty')
EXIT_CODE=$(echo "$INPUT" | jq -r '.tool_result.exit_code // empty')

# Check if command is running verify.py
if [[ "$COMMAND" =~ skills/([^/]+)/scripts/verify\.py ]]; then
    SKILL_NAME="${BASH_REMATCH[1]}"
elif [[ "$COMMAND" =~ python.*verify\.py ]] && [[ "$COMMAND" =~ skills/([^/]+)/ ]]; then
    SKILL_NAME="${BASH_REMATCH[1]}"
else
    exit 0
fi

# Determine status from exit code
if [ "$EXIT_CODE" = "0" ]; then
    STATUS="success"
else
    STATUS="failure"
fi

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"')
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs

# Write verify event using jq for proper JSON (compact for JSONL)
jq -nc --arg ts "$TIMESTAMP" --arg sid "$SESSION_ID" --arg skill "$SKILL_NAME" --arg status "$STATUS" \
  '{timestamp: $ts, session_id: $sid, skill: $skill, event: "verify", status: $status}' >> .claude/activity-logs/skill-usage.jsonl

exit 0
