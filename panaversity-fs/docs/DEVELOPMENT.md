# PanaversityFS Development Workflow

> Guide for developing with PanaversityFS as a standalone MCP server

**ADR-0018**: [Docusaurus-Aligned Storage Structure](../../history/adr/0018-panaversityfs-docusaurus-aligned-structure.md)

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Local Development](#local-development)
3. [Content Authoring Workflow](#content-authoring-workflow)
4. [Docusaurus Integration](#docusaurus-integration)
5. [Separate Repository Setup](#separate-repository-setup)
6. [CI/CD Pipeline](#cicd-pipeline)

---

## Architecture Overview

### Content Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    Content Authors                          │
│         (AI Agents, Human Writers via MCP)                  │
└─────────────────────────┬───────────────────────────────────┘
                          │ MCP Protocol (read/write/search)
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                  PanaversityFS MCP Server                   │
│              (Cloud deployment or local)                    │
└─────────────────────────┬───────────────────────────────────┘
                          │ Storage API
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              Cloud Storage (R2 or Supabase)                 │
│                   books/ai-native-dev/                      │
│                   ├── content/   (markdown)                 │
│                   └── static/    (assets)                   │
└─────────────────────────┬───────────────────────────────────┘
                          │ Sync (on-demand or scheduled)
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                  Docusaurus Build Pipeline                  │
│    1. Fetch from PanaversityFS (get_book_archive)           │
│    2. Build static site                                     │
│    3. Deploy to GitHub Pages                                │
└─────────────────────────────────────────────────────────────┘
```

### Source of Truth

After migration:
- **PanaversityFS** is the source of truth for content
- **Docusaurus** renders content for web display
- **AI Agents** read/write via MCP protocol

---

## Local Development

### Prerequisites

```bash
# Python 3.13+ with uv
python --version  # 3.13+
uv --version

# Clone and install
cd panaversity-fs
uv sync --all-extras
```

### Running Locally

```bash
# Option 1: Use migrated data
export PANAVERSITY_STORAGE_ROOT=$(pwd)/data
export PANAVERSITY_STORAGE_BACKEND=fs
.venv/bin/python -m panaversity_fs.server

# Option 2: Connect to cloud storage
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=panaversity-books
# ... (see SETUP.md for full config)
.venv/bin/python -m panaversity_fs.server
```

### Running Tests

```bash
# All tests (45 total)
.venv/bin/pytest tests/ -v

# By category
.venv/bin/pytest tests/unit/ -v          # Unit tests
.venv/bin/pytest tests/integration/ -v   # Integration tests
.venv/bin/pytest tests/e2e/ -v           # End-to-end tests
.venv/bin/pytest tests/edge_cases/ -v    # Production scenarios
```

---

## Content Authoring Workflow

### Using MCP Tools

AI agents connect to PanaversityFS and use these tools:

#### 1. Read Content
```python
# Read a lesson
await read_content({
    "book_id": "ai-native-dev",
    "path": "content/01-Part/01-Chapter/01-intro.md"
})

# Read a summary (ADR-0018: sibling file convention)
await read_content({
    "book_id": "ai-native-dev",
    "path": "content/01-Part/01-Chapter/01-intro.summary.md"
})
```

#### 2. Write Content with Conflict Detection
```python
# Read first to get hash
result = await read_content({
    "book_id": "ai-native-dev",
    "path": "content/01-Part/01-Chapter/01-intro.md"
})
current_hash = result["file_hash_sha256"]

# Write with conflict detection
await write_content({
    "book_id": "ai-native-dev",
    "path": "content/01-Part/01-Chapter/01-intro.md",
    "content": "# Updated Content\n\nNew content here...",
    "file_hash": current_hash  # Prevents concurrent write conflicts
})
```

#### 3. Search Content
```python
# Find all markdown files
await glob_search({
    "book_id": "ai-native-dev",
    "pattern": "**/*.md"
})

# Search content by regex
await grep_search({
    "book_id": "ai-native-dev",
    "pattern": "Python",
    "max_results": 50
})
```

### Content Structure (ADR-0018)

```
books/{book-id}/
├── content/                            # Maps to Docusaurus docs/
│   ├── 01-Introducing-AI-Driven-Development/
│   │   ├── README.md                   # Part intro
│   │   ├── 01-Chapter/
│   │   │   ├── README.md               # Chapter intro
│   │   │   ├── 01-lesson.md            # Lesson content
│   │   │   └── 01-lesson.summary.md    # Lesson summary (sibling)
│   │   └── 02-Chapter/
│   │       └── ...
│   └── 02-AI-Tool-Landscape/
│       └── ...
└── static/                             # Maps to Docusaurus static/
    ├── images/
    │   └── diagram.png
    └── slides/
        └── presentation.pdf
```

---

## Docusaurus Integration

### Option 1: Build-Time Sync

Add a pre-build script to fetch content:

```bash
#!/bin/bash
# scripts/sync-content.sh

# Fetch book archive
curl -X POST https://panaversityfs.example.com/mcp \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $API_KEY" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"get_book_archive","arguments":{"book_id":"ai-native-dev"}}}' \
  | jq -r '.result' > archive_response.json

# Extract URL and download
ARCHIVE_URL=$(jq -r '.archive_url' archive_response.json)
curl -o book.zip "$ARCHIVE_URL"

# Extract to Docusaurus directories
unzip -o book.zip -d temp/
cp -r temp/content/* docs/
cp -r temp/static/* static/

# Cleanup
rm -rf temp book.zip archive_response.json
```

### Option 2: Docusaurus Plugin

Create a plugin that fetches during build:

```javascript
// plugins/panaversityfs-plugin/index.js
module.exports = function(context, options) {
  return {
    name: 'panaversityfs-plugin',
    async loadContent() {
      const response = await fetch(`${options.serverUrl}/mcp`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${options.apiKey}`
        },
        body: JSON.stringify({
          jsonrpc: '2.0',
          id: 1,
          method: 'tools/call',
          params: {
            name: 'get_book_archive',
            arguments: { book_id: options.bookId }
          }
        })
      });
      // ... extract and return content
    }
  };
};
```

### Option 3: GitHub Actions Sync

```yaml
# .github/workflows/sync-content.yml
name: Sync Content from PanaversityFS

on:
  schedule:
    - cron: '0 */6 * * *'  # Every 6 hours
  workflow_dispatch:       # Manual trigger

jobs:
  sync:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Fetch content from PanaversityFS
        run: |
          ./scripts/sync-content.sh
        env:
          API_KEY: ${{ secrets.PANAVERSITYFS_API_KEY }}

      - name: Commit changes
        run: |
          git config user.name "github-actions[bot]"
          git config user.email "github-actions[bot]@users.noreply.github.com"
          git add docs/ static/
          git diff --staged --quiet || git commit -m "chore: sync content from PanaversityFS"
          git push
```

---

## Separate Repository Setup

### Step 1: Create New Repository

```bash
# Create repo on GitHub
gh repo create panaversity/panaversity-fs --public --description "Agent-Native Multi-Book Storage MCP Server"

# Clone and copy files
git clone git@github.com:panaversity/panaversity-fs.git /path/to/new/panaversity-fs
cp -r /current/panaversity-fs/* /path/to/new/panaversity-fs/

# Remove data directory (will be in cloud storage)
rm -rf /path/to/new/panaversity-fs/data
```

### Step 2: Update References

Update relative paths in documentation:
- `../../history/adr/` → Link to ADR in main repo or copy ADR
- `../../specs/` → Link to spec in main repo or copy spec

### Step 3: Initialize and Push

```bash
cd /path/to/new/panaversity-fs
git init
git add .
git commit -m "Initial commit: PanaversityFS MCP server (ADR-0018)"
git remote add origin git@github.com:panaversity/panaversity-fs.git
git push -u origin main
```

### Repository Structure

```
panaversity-fs/
├── src/panaversity_fs/      # Python package
│   ├── __init__.py
│   ├── server.py
│   ├── config.py
│   ├── models.py
│   ├── storage.py
│   ├── audit.py
│   ├── errors.py
│   └── tools/
│       ├── content.py
│       ├── assets.py
│       ├── search.py
│       ├── registry.py
│       └── bulk.py
├── tests/                   # Test suite
├── scripts/                 # Migration and utility scripts
├── docs/                    # Documentation
│   ├── ARCHITECTURE.md
│   ├── MCP-TOOLS.md
│   ├── SETUP.md
│   ├── MIGRATION.md
│   └── DEVELOPMENT.md
├── pyproject.toml
├── uv.lock
├── Dockerfile
├── .env.example
└── README.md
```

---

## CI/CD Pipeline

### GitHub Actions for Testing

```yaml
# .github/workflows/test.yml
name: Test

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install uv
        uses: astral-sh/setup-uv@v4

      - name: Set up Python
        run: uv python install 3.13

      - name: Install dependencies
        run: uv sync --all-extras

      - name: Run tests
        run: |
          uv run pytest tests/ -v --cov=panaversity_fs --cov-report=xml
        env:
          PANAVERSITY_STORAGE_BACKEND: fs
          PANAVERSITY_STORAGE_ROOT: /tmp/panaversity-test

      - name: Upload coverage
        uses: codecov/codecov-action@v4
        with:
          file: ./coverage.xml
```

### GitHub Actions for Deployment

```yaml
# .github/workflows/deploy.yml
name: Deploy

on:
  push:
    tags:
      - 'v*'

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3

      - name: Login to Container Registry
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Build and push
        uses: docker/build-push-action@v5
        with:
          context: .
          push: true
          tags: ghcr.io/panaversity/panaversity-fs:${{ github.ref_name }}

      - name: Deploy to Fly.io
        uses: superfly/flyctl-actions/setup-flyctl@master
      - run: flyctl deploy --remote-only
        env:
          FLY_API_TOKEN: ${{ secrets.FLY_API_TOKEN }}
```

### PyPI Publishing (Optional)

```yaml
# .github/workflows/publish.yml
name: Publish to PyPI

on:
  release:
    types: [published]

jobs:
  publish:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install uv
        uses: astral-sh/setup-uv@v4

      - name: Build package
        run: uv build

      - name: Publish to PyPI
        run: uv publish
        env:
          UV_PUBLISH_TOKEN: ${{ secrets.PYPI_TOKEN }}
```

---

## Environment Variables Reference

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `PANAVERSITY_STORAGE_BACKEND` | Yes | `fs` | Storage backend: `fs`, `s3`, `supabase` |
| `PANAVERSITY_STORAGE_ROOT` | For `fs` | - | Local filesystem root path |
| `PANAVERSITY_S3_BUCKET` | For `s3` | - | S3/R2 bucket name |
| `PANAVERSITY_S3_ENDPOINT` | For `s3` | - | S3/R2 endpoint URL |
| `PANAVERSITY_S3_ACCESS_KEY_ID` | For `s3` | - | S3/R2 access key |
| `PANAVERSITY_S3_SECRET_ACCESS_KEY` | For `s3` | - | S3/R2 secret key |
| `PANAVERSITY_S3_REGION` | For `s3` | `auto` | S3/R2 region |
| `PANAVERSITY_SUPABASE_URL` | For `supabase` | - | Supabase project URL |
| `PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY` | For `supabase` | - | Supabase service key |
| `PANAVERSITY_SUPABASE_BUCKET` | For `supabase` | - | Supabase bucket name |
| `PANAVERSITY_CDN_BASE_URL` | No | - | CDN base URL for assets |
| `PANAVERSITY_SERVER_HOST` | No | `0.0.0.0` | Server bind host |
| `PANAVERSITY_SERVER_PORT` | No | `8000` | Server bind port |

---

## Related Documentation

- **[Architecture](./ARCHITECTURE.md)**: System design
- **[MCP Tools](./MCP-TOOLS.md)**: Tool API reference
- **[Setup Guide](./SETUP.md)**: Backend configuration
- **[Migration Guide](./MIGRATION.md)**: Production deployment
