#!/usr/bin/env bash
# detect-forward-references.sh - Detect potential pedagogical ordering violations
# Usage: ./detect-forward-references.sh 14

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

# Define patterns to detect (common forward references)
# These are heuristics - AI will read context to confirm
declare -A PATTERNS=(
    [".upper()|.lower()|.strip()|.split()|.replace()|.startswith()|.endswith()"]="String method not introduced"
    ["isinstance("]="Function isinstance() not introduced"
    ["len("]="Built-in function len() not introduced"
    ["type("]="Built-in function type() not introduced"
    ["range("]="Built-in function range() not introduced"
    ["def "]="Function definition (def) not introduced"
    ["class "]="Class definition not introduced"
    ["import "]="Module import not introduced"
    ["lambda "]="Lambda function not introduced"
    ["list(|dict(|set(|tuple("]="Type constructor not introduced"
)

# Start JSON output
cat <<EOF
{
  "chapter": $CHAPTER_NUM,
  "violations_detected": [
EOF

VIOLATIONS=()

# Scan each lesson for patterns
for LESSON in "${LESSON_FILES[@]}"; do
    LESSON_NAME=$(basename "$LESSON")

    # Extract code blocks only (to avoid false positives in explanatory text)
    CODE_BLOCKS=$(awk '/^```python$/,/^```$/' "$LESSON" | grep -v '^```' || echo "")

    # Check each pattern
    for PATTERN in "${!PATTERNS[@]}"; do
        ISSUE="${PATTERNS[$PATTERN]}"

        # Search for pattern in code blocks
        if echo "$CODE_BLOCKS" | grep -qE "$PATTERN" 2>/dev/null; then
            # Get line number and context
            LINE_NUM=$(grep -nE "$PATTERN" "$LESSON" | head -1 | cut -d: -f1 || echo "0")
            CODE_SNIPPET=$(grep -oE "[a-zA-Z_][a-zA-Z0-9_]*\(.*\)|[a-zA-Z_][a-zA-Z0-9_]*\.[a-zA-Z_][a-zA-Z0-9_]*\(.*\)" "$LESSON" | grep -E "$PATTERN" | head -1 || echo "")

            # Get surrounding context (3 lines before and after)
            if [[ $LINE_NUM -gt 0 ]]; then
                CONTEXT=$(sed -n "$((LINE_NUM - 3)),$((LINE_NUM + 3))p" "$LESSON" | sed 's/"/\\"/g' | tr '\n' ' ' | sed 's/  */ /g')
            else
                CONTEXT="Pattern found in code block"
            fi

            # Determine severity flag (heuristic)
            SEVERITY="MAJOR"
            if [[ "$PATTERN" =~ "def " ]] || [[ "$PATTERN" =~ "class " ]] || [[ "$PATTERN" =~ "isinstance" ]]; then
                SEVERITY="CRITICAL"
            elif [[ "$PATTERN" =~ "len\(|type\(" ]]; then
                SEVERITY="MAJOR"
            fi

            VIOLATIONS+=("{\"lesson\":\"$LESSON_NAME\",\"line\":$LINE_NUM,\"code_snippet\":\"$CODE_SNIPPET\",\"issue\":\"$ISSUE\",\"severity_flag\":\"$SEVERITY\",\"context\":\"$CONTEXT\"}")
        fi
    done
done

# Output violations
for i in "${!VIOLATIONS[@]}"; do
    if (( i == ${#VIOLATIONS[@]} - 1 )); then
        echo "    ${VIOLATIONS[$i]}"
    else
        echo "    ${VIOLATIONS[$i]},"
    fi
done

if (( ${#VIOLATIONS[@]} == 0 )); then
    echo ""
fi

# Find clean lessons
CLEAN_LESSONS=()
for LESSON in "${LESSON_FILES[@]}"; do
    IS_CLEAN=true
    for VIOLATION in "${VIOLATIONS[@]}"; do
        if echo "$VIOLATION" | grep -q "$(basename "$LESSON")"; then
            IS_CLEAN=false
            break
        fi
    done
    if $IS_CLEAN; then
        CLEAN_LESSONS+=("\"$(basename "$LESSON")\"")
    fi
done

cat <<EOF
  ],
  "clean_lessons": [
EOF

for i in "${!CLEAN_LESSONS[@]}"; do
    if (( i == ${#CLEAN_LESSONS[@]} - 1 )); then
        echo "    ${CLEAN_LESSONS[$i]}"
    else
        echo "    ${CLEAN_LESSONS[$i]},"
    fi
done

if (( ${#CLEAN_LESSONS[@]} == 0 )); then
    echo ""
fi

cat <<EOF
  ],
  "violation_count": ${#VIOLATIONS[@]}
}
EOF
