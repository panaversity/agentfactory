#!/bin/bash
# Build script that conditionally adds --localstorage-file flag for Node.js 25+

# Change to book-source directory (parent of scripts/)
cd "$(dirname "$0")/.."

NODE_VERSION=$(node -v | cut -d'.' -f1 | sed 's/v//')

if [ "$NODE_VERSION" -ge 25 ]; then
  # Node.js 25+ requires --localstorage-file flag
  node --localstorage-file=/tmp/docusaurus-localstorage ./node_modules/.bin/docusaurus build
else
  # Node.js 20-24 doesn't need the flag
  node ./node_modules/.bin/docusaurus build
fi
