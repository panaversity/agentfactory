# Quickstart Guide

> Get PanaversityFS running in 5 minutes

**Spec Reference**: This guide implements the setup portion of [Feature 039: PanaversityFS Production Hardening](../../../../specs/039-panaversity-fs-hardening/spec.md).

## Prerequisites

- Python 3.13+
- [uv](https://docs.astral.sh/uv/) package manager
- (Optional) Docker for containerized deployment
- (Optional) PostgreSQL for production (FR-028, FR-029 - defaults to SQLite for development)

## Installation

```bash
# Clone and enter directory
cd panaversity-fs

# Install dependencies
uv sync

# Verify installation
uv run python -c "from panaversity_fs import __version__; print(f'PanaversityFS ready')"
```

## Quick Test

```bash
# Set up local filesystem backend
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-test

# Run tests to verify everything works
uv run pytest tests/ -q
# Expected: 301 passed
```

## Start the Server

```bash
# Start MCP server on port 8000
uv run python -m panaversity_fs.server
```

Server runs at: `http://0.0.0.0:8000/mcp`

## Create Test Content

```bash
# Create a test book structure
mkdir -p /tmp/panaversity-test/books/my-book/content/01-Part/01-Chapter

# Create a lesson
cat > /tmp/panaversity-test/books/my-book/content/01-Part/01-Chapter/01-intro.md << 'EOF'
---
title: Introduction
---

# Welcome to My Book

This is the first lesson.
EOF
```

## Test with curl

```bash
# List available books
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"list_books","arguments":{}}}'

# Read content
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"read_content","arguments":{"book_id":"my-book","path":"content/01-Part/01-Chapter/01-intro.md"}}}'
```

## Test with MCP Inspector

```bash
# Install MCP Inspector
npm install -g @modelcontextprotocol/inspector

# Open inspector UI
npx @modelcontextprotocol/inspector http://localhost:8000/mcp
```

## Available MCP Tools

| Tool | Purpose | Key Spec Requirements |
|------|---------|----------------------|
| `list_books` | Discover available books | - |
| `read_content` | Read lessons/summaries | FR-016 (overlay fallback) |
| `write_content` | Create/update content | FR-002-005 (conflict detection) |
| `delete_content` | Remove content | R3 (idempotent) |
| `upload_asset` | Upload images/videos | FR-008 (asset schema) |
| `get_asset` | Download assets | - |
| `list_assets` | List book assets | - |
| `glob_search` | Find files by pattern | - |
| `grep_search` | Search content by regex | - |
| `get_book_archive` | Download book as ZIP | FR-011-014, R4 (<60s, <64MB) |
| `validate_book` | Check book structure | FR-010, R1 (schema enforcement) |
| `delta_build` | Get changed files since timestamp | FR-025-027 (manifest hash) |

## Next Steps

1. **Understand the architecture**: [02-architecture.md](./02-architecture.md)
2. **Learn the full API**: [03-tools-reference.md](./03-tools-reference.md)
3. **Explore the codebase**: [04-codebase-map.md](./04-codebase-map.md)

## Common Issues

### "Module not found" errors
```bash
# Ensure you're using uv's environment
uv run python -m panaversity_fs.server
# Not: python -m panaversity_fs.server
```

### Permission denied on /tmp
```bash
# Use a directory you own
export PANAVERSITY_STORAGE_ROOT=$HOME/panaversity-data
mkdir -p $HOME/panaversity-data
```

### Tests failing
```bash
# Check if pytest-asyncio is installed
uv run pip list | grep asyncio

# Run with verbose output to diagnose
uv run pytest tests/ -v --tb=short
```
