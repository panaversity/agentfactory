#!/bin/bash
# Build script that conditionally adds --localstorage-file flag for Node.js 25+

# Change to learn-app directory (parent of scripts/)
cd "$(dirname "$0")/.."

NODE_VERSION=$(node -v | cut -d'.' -f1 | sed 's/v//')

if [ "$NODE_VERSION" -ge 25 ]; then
  # Node.js 25+ requires --localstorage-file flag
  # Use npx to properly resolve the docusaurus binary
  NODE_OPTIONS="--localstorage-file=/tmp/docusaurus-localstorage" npx docusaurus build
else
  # Node.js 20-24 doesn't need the flag
  npx docusaurus build
fi
