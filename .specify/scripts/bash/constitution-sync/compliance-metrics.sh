#!/usr/bin/env bash
# compliance-metrics.sh - Quantitative compliance metrics per lesson
# Usage: ./compliance-metrics.sh 14 book-source/docs/04-Part-4/14-data-types/01-intro.md

set -euo pipefail

CHAPTER_NUM="${1:-}"
LESSON_FILE="${2:-}"

if [[ -z "$CHAPTER_NUM" ]] || [[ -z "$LESSON_FILE" ]]; then
    echo "Usage: $0 <chapter-number> <lesson-file-path>" >&2
    exit 1
fi

if [[ ! -f "$LESSON_FILE" ]]; then
    echo "{\"error\": \"Lesson file not found: $LESSON_FILE\"}" >&2
    exit 1
fi

LESSON_BASENAME=$(basename "$LESSON_FILE")

# Count CoLearning elements
PROMPT_COUNT=$(grep -c "#### 💬" "$LESSON_FILE" 2>/dev/null || echo "0")
COMMENTARY_COUNT=$(grep -c "#### 🎓" "$LESSON_FILE" 2>/dev/null || echo "0")
CHALLENGE_COUNT=$(grep -c "#### 🚀" "$LESSON_FILE" 2>/dev/null || echo "0")
TIP_COUNT=$(grep -c "#### ✨" "$LESSON_FILE" 2>/dev/null || echo "0")

# Check lesson closure pattern
TRY_WITH_AI_EXISTS=false
POST_SECTIONS=()

if grep -q "## Try With AI" "$LESSON_FILE" 2>/dev/null; then
    TRY_WITH_AI_EXISTS=true

    # Get line number of "Try With AI"
    TRY_LINE=$(grep -n "## Try With AI" "$LESSON_FILE" | cut -d: -f1 | head -1)

    # Check if any ## headers exist after that line
    AFTER_TRY=$(tail -n +"$((TRY_LINE + 1))" "$LESSON_FILE" | grep -n "^##" 2>/dev/null || echo "")

    if [[ -n "$AFTER_TRY" ]]; then
        # Extract post-section titles
        while IFS= read -r line; do
            SECTION_TITLE=$(echo "$line" | sed 's/^[0-9]*:##* *//')
            POST_SECTIONS+=("\"$SECTION_TITLE\"")
        done <<< "$AFTER_TRY"
    fi
fi

# Check for forward references (common patterns)
FORWARD_REF_COUNT=0
FLAGGED_LINES=()

# String methods
if grep -qE '\.(upper|lower|strip|split|replace|startswith|endswith)\(' "$LESSON_FILE" 2>/dev/null; then
    LINES=$(grep -nE '\.(upper|lower|strip|split|replace|startswith|endswith)\(' "$LESSON_FILE" | cut -d: -f1)
    while IFS= read -r line; do
        FLAGGED_LINES+=("$line")
        ((FORWARD_REF_COUNT++))
    done <<< "$LINES"
fi

# Built-in functions
if grep -qE '\b(isinstance|len|type|range)\(' "$LESSON_FILE" 2>/dev/null; then
    LINES=$(grep -nE '\b(isinstance|len|type|range)\(' "$LESSON_FILE" | cut -d: -f1)
    while IFS= read -r line; do
        FLAGGED_LINES+=("$line")
        ((FORWARD_REF_COUNT++))
    done <<< "$LINES"
fi

# Advanced syntax
if grep -qE '\b(def |class |import |lambda )\b' "$LESSON_FILE" 2>/dev/null; then
    LINES=$(grep -nE '\b(def |class |import |lambda )\b' "$LESSON_FILE" | cut -d: -f1)
    while IFS= read -r line; do
        FLAGGED_LINES+=("$line")
        ((FORWARD_REF_COUNT++))
    done <<< "$LINES"
fi

# Tone analysis (heuristic - conversational vs documentation)
# Count conversational markers
CONVERSATIONAL_MARKERS=$(grep -ciE "\b(you|your|let's|we'll|you'll|imagine|think about|what if|how does|why does)\b" "$LESSON_FILE" || echo "0")
TOTAL_LINES=$(wc -l < "$LESSON_FILE")
CONVERSATIONAL_SCORE=$((CONVERSATIONAL_MARKERS * 10 / TOTAL_LINES))
if (( CONVERSATIONAL_SCORE > 10 )); then
    CONVERSATIONAL_SCORE=10
fi

# Check for exploration language
EXPLORATION_LANGUAGE="absent"
if grep -qiE "(explore|discover|experiment|try|what happens|let's see)" "$LESSON_FILE" 2>/dev/null; then
    EXPLORATION_LANGUAGE="present"
fi

# Check for AI partnership language
AI_PARTNERSHIP="absent"
if grep -qiE "(your AI|AI co-teacher|AI pair-programmer|with AI|ask your AI)" "$LESSON_FILE" 2>/dev/null; then
    AI_PARTNERSHIP="mentioned"
    # Check if it's embedded throughout (not just "Try With AI" section)
    PRE_TRY_CONTENT=$(head -n "${TRY_LINE:-999999}" "$LESSON_FILE" 2>/dev/null || cat "$LESSON_FILE")
    if echo "$PRE_TRY_CONTENT" | grep -qiE "(your AI|AI co-teacher|ask your AI)" 2>/dev/null; then
        AI_PARTNERSHIP="embedded"
    fi
fi

# Code quality checks (Python 3.13+, type hints)
PYTHON_VERSION="unknown"
TYPE_HINTS="unknown"
SECURITY="unknown"

if grep -q "```python" "$LESSON_FILE" 2>/dev/null; then
    # Check for modern Python syntax
    if grep -qE "match |case " "$LESSON_FILE" 2>/dev/null; then
        PYTHON_VERSION="3.10+"
    fi
    if grep -qE "\| None|\| int|\| str" "$LESSON_FILE" 2>/dev/null; then
        PYTHON_VERSION="3.10+"
    fi

    # Check for type hints
    if grep -qE ":\s*(str|int|float|bool|list\[|dict\[|tuple\[|set\[)" "$LESSON_FILE" 2>/dev/null; then
        TYPE_HINTS="present"
    else
        TYPE_HINTS="absent"
    fi

    # Security check (hardcoded secrets)
    if grep -qiE "(password\s*=\s*['\"]|api_key\s*=\s*['\"]|token\s*=\s*['\"])" "$LESSON_FILE" 2>/dev/null; then
        SECURITY="hardcoded_secrets"
    else
        SECURITY="no_issues"
    fi
fi

# Output JSON
cat <<EOF
{
  "chapter": $CHAPTER_NUM,
  "lesson": "$LESSON_BASENAME",
  "colearning_elements": {
    "💬_prompts": $PROMPT_COUNT,
    "🎓_commentaries": $COMMENTARY_COUNT,
    "🚀_challenges": $CHALLENGE_COUNT,
    "✨_tips": $TIP_COUNT
  },
  "lesson_closure": {
    "try_with_ai_exists": $TRY_WITH_AI_EXISTS,
    "post_sections": [
EOF

# Output post-sections array
for i in "${!POST_SECTIONS[@]}"; do
    if (( i == ${#POST_SECTIONS[@]} - 1 )); then
        echo "      ${POST_SECTIONS[$i]}"
    else
        echo "      ${POST_SECTIONS[$i]},"
    fi
done

if (( ${#POST_SECTIONS[@]} == 0 )); then
    echo ""
fi

cat <<EOF
    ]
  },
  "pedagogical_ordering": {
    "forward_references_found": $FORWARD_REF_COUNT,
    "flagged_lines": [
EOF

# Output flagged lines array
for i in "${!FLAGGED_LINES[@]}"; do
    if (( i == ${#FLAGGED_LINES[@]} - 1 )); then
        echo "      ${FLAGGED_LINES[$i]}"
    else
        echo "      ${FLAGGED_LINES[$i]},"
    fi
done

if (( ${#FLAGGED_LINES[@]} == 0 )); then
    echo ""
fi

cat <<EOF
    ]
  },
  "tone_analysis": {
    "conversational_score": $CONVERSATIONAL_SCORE,
    "exploration_language": "$EXPLORATION_LANGUAGE",
    "ai_partnership": "$AI_PARTNERSHIP"
  },
  "code_quality": {
    "python_version": "$PYTHON_VERSION",
    "type_hints": "$TYPE_HINTS",
    "security": "$SECURITY"
  }
}
EOF
