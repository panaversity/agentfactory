#!/usr/bin/env bash
# artifact-locator.sh - Locate all artifacts for a given chapter
# Usage: ./artifact-locator.sh 14

set -euo pipefail

CHAPTER_NUM="${1:-}"

if [[ -z "$CHAPTER_NUM" ]]; then
    echo "Usage: $0 <chapter-number>" >&2
    exit 1
fi

# Initialize result JSON
cat <<EOF
{
  "chapter": $CHAPTER_NUM,
  "artifacts": {
EOF

# Find PHR
PHR_PATH=$(find history/phr -type f -name "*chapter-${CHAPTER_NUM}*" 2>/dev/null | head -1 || echo "")
if [[ -n "$PHR_PATH" ]]; then
    echo "    \"phr\": \"$PHR_PATH\","
else
    echo "    \"phr\": null,"
fi

# Find spec directory (pattern: specs/part-X-chapter-N/)
SPEC_DIR=$(find specs -type d -name "*chapter-${CHAPTER_NUM}" 2>/dev/null | head -1 || echo "")
if [[ -n "$SPEC_DIR" ]]; then
    echo "    \"spec\": \"$SPEC_DIR/spec.md\","
    echo "    \"plan\": \"$SPEC_DIR/plan.md\","
    echo "    \"tasks\": \"$SPEC_DIR/tasks.md\","
else
    echo "    \"spec\": null,"
    echo "    \"plan\": null,"
    echo "    \"tasks\": null,"
fi

# Find lesson files (pattern: apps/learn-app/docs/XX-Part-X-Name/N-*/*.md)
LESSONS=$(find book-source/docs -type f -path "*/${CHAPTER_NUM}-*/*.md" 2>/dev/null | grep -v "/readme.md" | sort || echo "")
if [[ -n "$LESSONS" ]]; then
    echo "    \"lessons\": ["
    echo "$LESSONS" | while IFS= read -r lesson; do
        # Check if this is the last line
        if [[ "$lesson" == "$(echo "$LESSONS" | tail -1)" ]]; then
            echo "      \"$lesson\""
        else
            echo "      \"$lesson\","
        fi
    done
    echo "    ],"
else
    echo "    \"lessons\": [],"
fi

# Find validation report
VALIDATION_REPORT=$(find . -type f -name "VALIDATION_REPORT_CHAPTER_${CHAPTER_NUM}.md" 2>/dev/null | head -1 || echo "")
if [[ -n "$VALIDATION_REPORT" ]]; then
    echo "    \"validation_report\": \"$VALIDATION_REPORT\""
else
    echo "    \"validation_report\": null"
fi

# Status determination
if [[ -n "$LESSONS" ]]; then
    STATUS="complete"
elif [[ -n "$SPEC_DIR" ]]; then
    STATUS="spec_only"
else
    STATUS="not_found"
fi

cat <<EOF
  },
  "status": "$STATUS"
}
EOF
