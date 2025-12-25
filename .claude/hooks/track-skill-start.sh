#!/usr/bin/env bash
# Track skill activations via SKILL.md reads (sync - fast operation)

# Read JSON input from stdin
INPUT=$(cat)

# Extract tool and file path - ONLY Read tool, not Bash
TOOL=$(echo "$INPUT" | jq -r '.tool_name // empty')
[ "$TOOL" != "Read" ] && exit 0

FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

# Only match SKILL.md reads in .claude/skills/[name]/
# Must be: .claude/skills/[valid-skill-name]/SKILL.md
if [[ "$FILE_PATH" =~ \.claude/skills/([a-z][a-z0-9-]*)/SKILL\.md$ ]]; then
    SKILL_NAME="${BASH_REMATCH[1]}"
else
    exit 0
fi

# Validate skill exists (has SKILL.md)
SKILL_DIR=".claude/skills/$SKILL_NAME"
[ ! -f "$SKILL_DIR/SKILL.md" ] && exit 0

SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"')
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Ensure log directory exists
mkdir -p .claude/activity-logs

# Write start event
jq -nc --arg ts "$TIMESTAMP" --arg sid "$SESSION_ID" --arg skill "$SKILL_NAME" \
  '{timestamp: $ts, session_id: $sid, skill: $skill, event: "start"}' >> .claude/activity-logs/skill-usage.jsonl

exit 0
