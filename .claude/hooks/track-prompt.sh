#!/usr/bin/env bash
# Track user prompt submissions (sync - fast operation)

# Read JSON input from stdin
INPUT=$(cat)

# Extract fields using jq
PROMPT=$(echo "$INPUT" | jq -r '.prompt // empty')
SESSION_ID=$(echo "$INPUT" | jq -r '.session_id // "unknown"')
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")

# Skip if no prompt
[ -z "$PROMPT" ] && exit 0

# Ensure log directory exists
mkdir -p .claude/activity-logs

# Write log entry using jq for proper JSON (compact for JSONL)
jq -nc --arg ts "$TIMESTAMP" --arg sid "$SESSION_ID" --arg prompt "$PROMPT"   '{timestamp: $ts, session_id: $sid, prompt: $prompt}' >> .claude/activity-logs/prompts.jsonl

exit 0
