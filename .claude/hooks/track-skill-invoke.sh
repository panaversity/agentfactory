#!/usr/bin/env bash
# Track Skill tool invocations (when user runs /skill-name or agent uses Skill tool)
# Silently exit on any error to avoid blocking Claude Code

# Read JSON input from stdin
INPUT=$(cat 2>/dev/null) || exit 0

# Validate input is JSON
echo "$INPUT" | jq -e . >/dev/null 2>&1 || exit 0

# Extract tool name - only process Skill tool
TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty' 2>/dev/null) || exit 0
[ "$TOOL" != "Skill" ] && exit 0

# Extract skill name from tool input
SKILL_NAME=$(echo "$INPUT" | jq -r '.tool_input.skill // empty' 2>/dev/null) || exit 0
[ -z "$SKILL_NAME" ] && exit 0

# Get optional args
ARGS=$(echo "$INPUT" | jq -r '.tool_input.args // ""' 2>/dev/null) || ARGS=""

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"' 2>/dev/null) || exit 0
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs 2>/dev/null || exit 0

# Write invocation event
jq -nc --arg ts "$TIMESTAMP" --arg sid "$SESSION_ID" --arg skill "$SKILL_NAME" --arg args "$ARGS" \
  '{timestamp: $ts, session_id: $sid, skill: $skill, args: $args, event: "invoke", source: "Skill_tool"}' >> .claude/activity-logs/skill-usage.jsonl 2>/dev/null

# Silent - no stdout to avoid hook errors
exit 0
