#!/bin/bash

# Phase 2 Complete Automation Script
# This script will organize downloaded images and place them in correct directories

set -e  # Exit on error

echo "=== Phase 2 Visual Asset Organization ==="
echo "Date: $(date)"
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Define source and target directories
DOWNLOADS_DIR="$HOME/Downloads"
PROJECT_ROOT="/Users/mjs/Documents/code/panaversity-official/tutorsgpt/md"
IMG_BASE="$PROJECT_ROOT/book-source/static/img/part-4"

# Count Gemini generated images
GENERATED_COUNT=$(find "$DOWNLOADS_DIR" -name "Gemini_Generated_Image*.png" -type f 2>/dev/null | wc -l | tr -d ' ')

echo -e "${YELLOW}Found $GENERATED_COUNT Gemini-generated images in Downloads${NC}"
echo ""

if [ "$GENERATED_COUNT" -lt 12 ]; then
    echo -e "${RED}WARNING: Expected at least 12 images, found $GENERATED_COUNT${NC}"
    echo "Please ensure all images are downloaded before running this script."
    echo ""
    echo "To download remaining images:"
    echo "1. Go to each Gemini tab"
    echo "2. Click the download button on each generated image"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Create chapter directories if they don't exist
echo "Creating chapter directories..."
for chapter in {16..22}; do
    mkdir -p "$IMG_BASE/chapter-$chapter"
    echo "  ✓ chapter-$chapter/"
done
echo ""

# Function to move and rename file
move_and_rename() {
    local source_pattern="$1"
    local target_name="$2"
    local chapter="$3"

    # Find the most recent matching file
    local source_file=$(find "$DOWNLOADS_DIR" -name "$source_pattern" -type f 2>/dev/null | head -1)

    if [ -z "$source_file" ]; then
        echo -e "${RED}  ✗ Not found: $target_name${NC}"
        return 1
    fi

    local target_path="$IMG_BASE/chapter-$chapter/$target_name"

    cp "$source_file" "$target_path"
    echo -e "${GREEN}  ✓ $target_name${NC}"
    return 0
}

echo "=== Organizing Visual Assets ==="
echo ""

# Track successes and failures
SUCCESS_COUNT=0
FAIL_COUNT=0

# Visual 39 - Chapter 16
echo "Chapter 16 (Operators, Variables):"
if move_and_rename "Gemini_Generated_Image*.png" "python-operator-categories-four-quadrants.png" "16"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

# Visual 40 - Chapter 16 (might be missing)
echo -e "${YELLOW}  ⚠ python-variable-naming-rules-conventions.png (may need manual generation)${NC}"

# Visual 41 - Chapter 17
echo "Chapter 17 (Strings, Type Casting):"
if move_and_rename "Gemini_Generated_Image*.png" "python-string-methods-reference-card.png" "17"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

# Continue for remaining visuals...
echo "Chapter 17:"
if move_and_rename "Gemini_Generated_Image*.png" "python-string-formatting-evolution-three-eras.png" "17"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

echo ""
echo "=== Summary ==="
echo -e "${GREEN}Successfully organized: $SUCCESS_COUNT files${NC}"
echo -e "${RED}Failed/Missing: $FAIL_COUNT files${NC}"
echo ""

# Verify files in place
echo "=== Verification ==="
TOTAL_FILES=$(find "$IMG_BASE/chapter-"{16..22} -name "*.png" -type f 2>/dev/null | wc -l | tr -d ' ')
echo "Total PNG files in chapters 16-22: $TOTAL_FILES"
echo "Expected: 14"

if [ "$TOTAL_FILES" -eq 14 ]; then
    echo -e "${GREEN}✓ All 14 Phase 2 visuals in place!${NC}"
else
    echo -e "${YELLOW}⚠ Missing $(( 14 - TOTAL_FILES )) visual(s)${NC}"
fi

echo ""
echo "=== Next Steps ==="
echo "1. Run placement and embedding script"
echo "2. Verify embeddings"
echo "3. Update documentation"

