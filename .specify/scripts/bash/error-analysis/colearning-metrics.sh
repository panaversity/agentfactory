#!/usr/bin/env bash
# colearning-metrics.sh - Detailed colearning element analysis per lesson
# Usage: ./colearning-metrics.sh 14

set -euo pipefail

CHAPTER_NUM="${1:-}"

if [[ -z "$CHAPTER_NUM" ]]; then
    echo "Usage: $0 <chapter-number>" >&2
    exit 1
fi

# Find lesson directory
LESSON_DIR=$(find book-source/docs -type d -path "*/${CHAPTER_NUM}-*" 2>/dev/null | head -1)
if [[ -z "$LESSON_DIR" ]]; then
    echo "{\"error\": \"No lessons found for chapter $CHAPTER_NUM\"}" >&2
    exit 1
fi

LESSON_FILES=("$LESSON_DIR"/*.md)

# Start JSON output
cat <<EOF
{
  "chapter": $CHAPTER_NUM,
  "lessons": {
EOF

# Process each lesson
for i in "${!LESSON_FILES[@]}"; do
    LESSON="${LESSON_FILES[$i]}"
    BASENAME=$(basename "$LESSON" .md)

    # Count elements
    PROMPT_COUNT=$(grep -c "#### 💬" "$LESSON" 2>/dev/null || echo "0")
    COMMENTARY_COUNT=$(grep -c "#### 🎓" "$LESSON" 2>/dev/null || echo "0")
    CHALLENGE_COUNT=$(grep -c "#### 🚀" "$LESSON" 2>/dev/null || echo "0")
    TIP_COUNT=$(grep -c "#### ✨" "$LESSON" 2>/dev/null || echo "0")

    # Get prompt locations (line numbers)
    PROMPT_LINES=$(grep -n "#### 💬" "$LESSON" 2>/dev/null | cut -d: -f1 | tr '\n' ',' | sed 's/,$//' || echo "")

    # Extract first 3 prompt snippets (next line after "#### 💬")
    PROMPT_SNIPPETS=()
    while IFS= read -r line_num; do
        if [[ -n "$line_num" ]]; then
            SNIPPET=$(sed -n "$((line_num + 1))p" "$LESSON" | sed 's/^[*"]*//; s/[*"]*$//; s/"/\\"/g')
            PROMPT_SNIPPETS+=("$SNIPPET")
        fi
    done < <(grep -n "#### 💬" "$LESSON" 2>/dev/null | cut -d: -f1 | head -3)

    # Output lesson JSON
    echo "    \"$BASENAME\": {"
    echo "      \"💬_prompts\": $PROMPT_COUNT,"
    echo "      \"🎓_commentaries\": $COMMENTARY_COUNT,"
    echo "      \"🚀_challenges\": $CHALLENGE_COUNT,"
    echo "      \"✨_tips\": $TIP_COUNT,"
    if [[ -n "$PROMPT_LINES" ]]; then
        echo "      \"prompt_locations\": [$PROMPT_LINES],"
    else
        echo "      \"prompt_locations\": [],"
    fi
    echo "      \"prompt_snippets\": ["
    for j in "${!PROMPT_SNIPPETS[@]}"; do
        if (( j == ${#PROMPT_SNIPPETS[@]} - 1 )); then
            echo "        \"${PROMPT_SNIPPETS[$j]}\""
        else
            echo "        \"${PROMPT_SNIPPETS[$j]}\","
        fi
    done
    if (( ${#PROMPT_SNIPPETS[@]} == 0 )); then
        echo ""
    fi
    echo "      ]"

    if (( i == ${#LESSON_FILES[@]} - 1 )); then
        echo "    }"
    else
        echo "    },"
    fi
done

# Calculate totals
TOTAL_PROMPTS=$(grep -roh "#### 💬" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')
TOTAL_COMMENTARIES=$(grep -roh "#### 🎓" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')
TOTAL_CHALLENGES=$(grep -roh "#### 🚀" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')
TOTAL_TIPS=$(grep -roh "#### ✨" "$LESSON_DIR" 2>/dev/null | wc -l | tr -d ' ')

LESSON_COUNT=${#LESSON_FILES[@]}

cat <<EOF
  },
  "totals": {
    "💬_prompts": $TOTAL_PROMPTS,
    "🎓_commentaries": $TOTAL_COMMENTARIES,
    "🚀_challenges": $TOTAL_CHALLENGES,
    "✨_tips": $TOTAL_TIPS
  },
  "expected_ranges": {
    "💬_prompts": "$((LESSON_COUNT * 1))-$((LESSON_COUNT * 4)) (1-4 per lesson × $LESSON_COUNT lessons)",
    "🎓_commentaries": "$((LESSON_COUNT * 2))-$((LESSON_COUNT * 4)) (2-4 per lesson)",
    "🚀_challenges": "$((LESSON_COUNT * 1))-$((LESSON_COUNT * 4)) (1-4 per lesson)",
    "✨_tips": "$((LESSON_COUNT * 1))-$((LESSON_COUNT * 3)) (1-3 per lesson)"
  }
}
EOF
