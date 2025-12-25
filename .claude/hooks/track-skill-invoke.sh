#!/usr/bin/env bash
# Track Skill tool invocations (when user runs /skill-name or agent uses Skill tool)

# Read JSON input from stdin
INPUT=$(cat)

# Extract tool name - only process Skill tool
TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty')
[ "$TOOL" != "Skill" ] && exit 0

# Extract skill name from tool input
SKILL_NAME=$(echo "$INPUT" | jq -r '.tool_input.skill // empty')
[ -z "$SKILL_NAME" ] && exit 0

# Get optional args
ARGS=$(echo "$INPUT" | jq -r '.tool_input.args // ""')

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"')
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs

# Write invocation event
jq -nc --arg ts "$TIMESTAMP" --arg sid "$SESSION_ID" --arg skill "$SKILL_NAME" --arg args "$ARGS" \
  '{timestamp: $ts, session_id: $sid, skill: $skill, args: $args, event: "invoke", source: "Skill_tool"}' >> .claude/activity-logs/skill-usage.jsonl

exit 0
