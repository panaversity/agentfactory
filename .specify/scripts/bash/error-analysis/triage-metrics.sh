#!/usr/bin/env bash
# triage-metrics.sh - Quick quantitative metrics for chapter triage
# Usage: ./triage-metrics.sh 14

set -euo pipefail

CHAPTER_NUM="${1:-}"

if [[ -z "$CHAPTER_NUM" ]]; then
    echo "Usage: $0 <chapter-number>" >&2
    exit 1
fi

# Find lesson files
LESSON_DIR=$(find book-source/docs -type d -path "*/${CHAPTER_NUM}-*" 2>/dev/null | head -1)
if [[ -z "$LESSON_DIR" ]]; then
    echo "{\"error\": \"No lessons found for chapter $CHAPTER_NUM\"}" >&2
    exit 1
fi

LESSON_FILES=("$LESSON_DIR"/*.md)
LESSON_COUNT=${#LESSON_FILES[@]}

# Count colearning elements across all lessons
PROMPT_COUNT=$(grep -roh "#### 💬" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')
COMMENTARY_COUNT=$(grep -roh "#### 🎓" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')
CHALLENGE_COUNT=$(grep -roh "#### 🚀" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')
TIP_COUNT=$(grep -roh "#### ✨" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')

# Check for lesson closure violations (any ## after "Try With AI")
CLOSURE_VIOLATIONS=()
for lesson in "${LESSON_FILES[@]}"; do
    if grep -q "## Try With AI" "$lesson" 2>/dev/null; then
        # Get line number of "Try With AI"
        TRY_LINE=$(grep -n "## Try With AI" "$lesson" | cut -d: -f1 | head -1)
        # Check if any ## headers exist after that line
        TOTAL_LINES=$(wc -l < "$lesson")
        AFTER_TRY=$(tail -n +"$((TRY_LINE + 1))" "$lesson" | grep -n "^##" | head -1 || echo "")
        if [[ -n "$AFTER_TRY" ]]; then
            VIOLATION_LINE=$((TRY_LINE + $(echo "$AFTER_TRY" | cut -d: -f1)))
            CLOSURE_VIOLATIONS+=("$(basename "$lesson") has ## after Try With AI (line $VIOLATION_LINE)")
        fi
    fi
done

# Detect potential forward references (common patterns)
FORWARD_REF_FLAGS=()
for lesson in "${LESSON_FILES[@]}"; do
    # Look for common forward reference patterns
    if grep -qE '\.upper\(\)|\.lower\(\)|\.strip\(\)|\.split\(\)' "$lesson" 2>/dev/null; then
        LINE=$(grep -nE '\.upper\(\)|\.lower\(\)|\.strip\(\)|\.split\(\)' "$lesson" | head -1 | cut -d: -f1)
        FORWARD_REF_FLAGS+=("$(basename "$lesson"):$LINE - String methods (may be taught later)")
    fi
    if grep -qE '\bisinstance\(' "$lesson" 2>/dev/null; then
        LINE=$(grep -nE '\bisinstance\(' "$lesson" | head -1 | cut -d: -f1)
        FORWARD_REF_FLAGS+=("$(basename "$lesson"):$LINE - isinstance() (may be taught later)")
    fi
    if grep -qE '\blen\(' "$lesson" 2>/dev/null; then
        LINE=$(grep -nE '\blen\(' "$lesson" | head -1 | cut -d: -f1)
        FORWARD_REF_FLAGS+=("$(basename "$lesson"):$LINE - len() (check if introduced)")
    fi
done

# Count code blocks
CODE_BLOCK_COUNT=$(grep -roh '^```python' "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')

# Estimate concepts per lesson (rough heuristic: count ### headers)
HEADER_COUNT=$(grep -roh '^### ' "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')
AVG_CONCEPTS=$(echo "scale=1; $HEADER_COUNT / $LESSON_COUNT" | bc)

# Suggested analysis depth
FORWARD_REF_COUNT=${#FORWARD_REF_FLAGS[@]}
CLOSURE_VIOLATION_COUNT=${#CLOSURE_VIOLATIONS[@]}

if (( FORWARD_REF_COUNT > 2 || CLOSURE_VIOLATION_COUNT > 0 )); then
    SUGGESTED_DEPTH="full"
elif (( PROMPT_COUNT < 10 || COMMENTARY_COUNT < 10 )); then
    SUGGESTED_DEPTH="full"
else
    SUGGESTED_DEPTH="quick"
fi

# Output JSON
cat <<EOF
{
  "chapter": $CHAPTER_NUM,
  "metrics": {
    "colearning_elements": {
      "💬_prompts": $PROMPT_COUNT,
      "🎓_commentaries": $COMMENTARY_COUNT,
      "🚀_challenges": $CHALLENGE_COUNT,
      "✨_tips": $TIP_COUNT
    },
    "lesson_closure_violations": [
EOF

# Add closure violations
for i in "${!CLOSURE_VIOLATIONS[@]}"; do
    if (( i == ${#CLOSURE_VIOLATIONS[@]} - 1 )); then
        echo "      \"${CLOSURE_VIOLATIONS[$i]}\""
    else
        echo "      \"${CLOSURE_VIOLATIONS[$i]}\","
    fi
done

if (( ${#CLOSURE_VIOLATIONS[@]} == 0 )); then
    echo ""
fi

cat <<EOF
    ],
    "pedagogical_ordering_flags": [
EOF

# Add forward reference flags
for i in "${!FORWARD_REF_FLAGS[@]}"; do
    if (( i == ${#FORWARD_REF_FLAGS[@]} - 1 )); then
        echo "      \"${FORWARD_REF_FLAGS[$i]}\""
    else
        echo "      \"${FORWARD_REF_FLAGS[$i]}\","
    fi
done

if (( ${#FORWARD_REF_FLAGS[@]} == 0 )); then
    echo ""
fi

cat <<EOF
    ],
    "code_block_count": $CODE_BLOCK_COUNT,
    "lesson_count": $LESSON_COUNT,
    "avg_concepts_per_lesson": $AVG_CONCEPTS,
    "forward_references_found": $FORWARD_REF_COUNT
  },
  "suggested_analysis_depth": "$SUGGESTED_DEPTH"
}
EOF
