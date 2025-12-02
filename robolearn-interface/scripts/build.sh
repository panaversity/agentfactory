#!/bin/bash
# Build script that conditionally adds --localstorage-file flag for Node.js 25+

# Change to robolearn-interface directory (parent of scripts/)
cd "$(dirname "$0")/.."

# Clean up corrupted translation files before build
# This removes any i18n files that contain common corruption patterns
if [ -d "i18n" ]; then
  echo "Cleaning up corrupted translation files..."
  find i18n -name "*.md" -type f -exec grep -l "ReferenceError:\|undefined variable\|undefined identifier" {} \; 2>/dev/null | while read -r file; do
    echo "Removing corrupted file: $file"
    rm -f "$file"
  done
fi

NODE_VERSION=$(node -v | cut -d'.' -f1 | sed 's/v//')

if [ "$NODE_VERSION" -ge 25 ]; then
  # Node.js 25+ requires --localstorage-file flag
  node --localstorage-file=/tmp/docusaurus-localstorage ./node_modules/.bin/docusaurus build
else
  # Node.js 20-24 doesn't need the flag
  node ./node_modules/.bin/docusaurus build
fi
