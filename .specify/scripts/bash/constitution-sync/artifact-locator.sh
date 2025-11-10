#!/usr/bin/env bash
# artifact-locator.sh - Find all artifacts for a chapter
# Usage: ./artifact-locator.sh 14

set -euo pipefail

CHAPTER_NUM="${1:-}"

if [[ -z "$CHAPTER_NUM" ]]; then
    echo "Usage: $0 <chapter-number>" >&2
    exit 1
fi

# Find PHR
PHR_PATH=$(find history/phr -type f -name "*chapter-${CHAPTER_NUM}*" 2>/dev/null | head -1 || echo "")

# Find spec directory
SPEC_DIR=$(find specs -type d -name "*chapter-${CHAPTER_NUM}" 2>/dev/null | head -1 || echo "")

# Find spec.md
if [[ -n "$SPEC_DIR" ]]; then
    SPEC_FILE="$SPEC_DIR/spec.md"
    PLAN_FILE="$SPEC_DIR/plan.md"
    TASKS_FILE="$SPEC_DIR/tasks.md"
else
    SPEC_FILE=""
    PLAN_FILE=""
    TASKS_FILE=""
fi

# Find lesson directory
LESSON_DIR=$(find book-source/docs -type d -path "*/${CHAPTER_NUM}-*" 2>/dev/null | head -1 || echo "")

# Find lesson files
if [[ -n "$LESSON_DIR" ]]; then
    LESSON_FILES=$(find "$LESSON_DIR" -type f -name "*.md" ! -name "readme.md" 2>/dev/null | sort || echo "")
else
    LESSON_FILES=""
fi

# Find validation report
VALIDATION_REPORT=$(find . -maxdepth 3 -type f -name "*CHAPTER*${CHAPTER_NUM}*.md" -o -name "*chapter-${CHAPTER_NUM}*.md" 2>/dev/null | grep -i validation | head -1 || echo "")

# Determine status
if [[ -n "$SPEC_FILE" ]] && [[ -n "$LESSON_FILES" ]]; then
    STATUS="complete"
elif [[ -n "$SPEC_FILE" ]]; then
    STATUS="spec_only"
elif [[ -n "$LESSON_FILES" ]]; then
    STATUS="lessons_only"
else
    STATUS="not_found"
fi

# Output JSON
cat <<EOF
{
  "chapter": $CHAPTER_NUM,
  "artifacts": {
    "phr": "$PHR_PATH",
    "spec": "$SPEC_FILE",
    "plan": "$PLAN_FILE",
    "tasks": "$TASKS_FILE",
    "lessons": [
EOF

# Output lesson files as JSON array
if [[ -n "$LESSON_FILES" ]]; then
    LESSON_ARRAY=()
    while IFS= read -r lesson; do
        LESSON_ARRAY+=("\"$lesson\"")
    done <<< "$LESSON_FILES"

    for i in "${!LESSON_ARRAY[@]}"; do
        if (( i == ${#LESSON_ARRAY[@]} - 1 )); then
            echo "      ${LESSON_ARRAY[$i]}"
        else
            echo "      ${LESSON_ARRAY[$i]},"
        fi
    done
fi

cat <<EOF
    ],
    "validation_report": "$VALIDATION_REPORT"
  },
  "status": "$STATUS"
}
EOF
