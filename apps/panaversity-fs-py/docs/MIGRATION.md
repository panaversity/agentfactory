# PanaversityFS Migration & Production Guide

> Complete guide for migrating book-source content to PanaversityFS and deploying to production

**ADR-0018**: [Docusaurus-Aligned Storage Structure](../../history/adr/0018-panaversityfs-docusaurus-aligned-structure.md)

## Table of Contents

1. [Overview](#overview)
2. [Phase 1: Local Filesystem Testing](#phase-1-local-filesystem-testing)
3. [Phase 2: Cloud Storage Migration](#phase-2-cloud-storage-migration)
4. [Phase 3: Production Deployment](#phase-3-production-deployment)
5. [Development Workflow](#development-workflow)
6. [Separate Repository Setup](#separate-repository-setup)

---

## Overview

### Current State

```
ai-native-software-development/
├── book-source/              # Docusaurus site (299 markdown files)
│   ├── docs/                 # Content (13 parts, 33 chapters)
│   └── static/               # Assets (202 files: images, slides)
└── panaversity-fs/           # MCP server (9 tools, 45 tests)
```

### Target State

```
panaversity-fs/               # Standalone MCP server
└── data/                     # OR cloud storage (R2/Supabase)
    └── books/                # Books discovered dynamically by list_books
        └── ai-native-dev/
            ├── content/      # 299 markdown files
            └── static/       # 202 asset files
```

### Migration Statistics

- **Content files**: 299 markdown files
- **Static assets**: 202 files (images, slides)
- **Total size**: ~50MB estimated
- **MCP tools**: 9 (read/write/delete content, assets, search, registry, bulk)

---

## Phase 1: Local Filesystem Testing

### Step 1.1: Run Migration (Dry Run)

```bash
cd panaversity-fs

# Preview migration without making changes
uv run python scripts/migrate_book_source.py --dry-run --verbose
```

Expected output:

```
============================================================
PanaversityFS Migration (ADR-0018)
============================================================
Source:     .../book-source
Target:     .../panaversity-fs/data
Book ID:    ai-native-dev
Dry Run:    True
============================================================

Migrating content (docs/ → content/)...
  ✓ 299 files, 47 directories

Migrating static assets (static/ → static/)...
  ✓ 202 files, 3 directories

Migration Preview!
Total files:       501
Total directories: 50
```

### Step 1.2: Execute Migration

```bash
# Run actual migration
uv run python scripts/migrate_book_source.py --target ./data

# Verify structure
ls -la data/
ls -la data/books/ai-native-dev/
ls -la data/books/ai-native-dev/content/ | head -20
```

### Step 1.3: Start MCP Server

```bash
# Configure environment
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=$(pwd)/data
export PANAVERSITY_SERVER_HOST=0.0.0.0
export PANAVERSITY_SERVER_PORT=8000

# Start server
uv run python -m panaversity_fs.server
```

### Step 1.4: Test MCP Tools

```bash
# Test list_books
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"list_books","arguments":{}}}'

# Test read_content
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"read_content","arguments":{"book_id":"ai-native-dev","path":"content/01-Introducing-AI-Driven-Development/README.md"}}}'

# Test glob_search
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"glob_search","arguments":{"book_id":"ai-native-dev","pattern":"**/*.md"}}}'

# Test grep_search
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"grep_search","arguments":{"book_id":"ai-native-dev","pattern":"Python","max_results":10}}}'
```

### Step 1.5: Run Integration Tests

```bash
# Run all tests against local storage
PANAVERSITY_STORAGE_ROOT=$(pwd)/data uv run pytest tests/ -v

# Expected: 45 passed
```

---

## Phase 2: Cloud Storage Migration

Choose ONE of the following backends:

### Option A: Cloudflare R2

**Advantages**: S3-compatible, zero egress fees, global CDN

#### A1. Create R2 Bucket

```bash
# Via Cloudflare Dashboard or Wrangler CLI
wrangler r2 bucket create panaversity-books
```

#### A2. Get Credentials

1. Go to Cloudflare Dashboard → R2 → Manage R2 API Tokens
2. Create token with "Admin Read & Write" permissions
3. Note: Account ID, Access Key ID, Secret Access Key

#### A3. Configure Environment

```bash
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=panaversity-books
export PANAVERSITY_S3_ENDPOINT=https://<ACCOUNT_ID>.r2.cloudflarestorage.com
export PANAVERSITY_S3_ACCESS_KEY_ID=<ACCESS_KEY>
export PANAVERSITY_S3_SECRET_ACCESS_KEY=<SECRET_KEY>
export PANAVERSITY_S3_REGION=auto
export PANAVERSITY_CDN_BASE_URL=https://cdn.yourdomain.com
```

#### A4. Upload Content to R2

```bash
# Use rclone for bulk upload
rclone config
# Configure R2 as 'cloudflare-r2' remote

# Sync local data to R2
rclone sync ./data cloudflare-r2:panaversity-books/

# Verify
rclone ls cloudflare-r2:panaversity-books/
```

#### A5. Configure R2 Public Access (Optional)

For CDN URLs to work:

1. R2 Dashboard → Bucket → Settings → Public Access
2. Enable and configure custom domain

---

### Option B: Supabase Storage

**Advantages**: PostgreSQL integration, Row Level Security, Dashboard UI

#### B1. Create Supabase Project

1. Go to [supabase.com](https://supabase.com)
2. Create new project
3. Note: Project URL, Service Role Key

#### B2. Create Storage Bucket

```sql
-- In Supabase SQL Editor
INSERT INTO storage.buckets (id, name, public)
VALUES ('panaversity-books', 'panaversity-books', true);
```

#### B3. Configure Environment

```bash
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://<PROJECT_REF>.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY=<SERVICE_KEY>
export PANAVERSITY_SUPABASE_BUCKET=panaversity-books
export PANAVERSITY_CDN_BASE_URL=https://<PROJECT_REF>.supabase.co/storage/v1/object/public/panaversity-books
```

#### B4. Upload Content to Supabase

```python
# Use Supabase Python client for bulk upload
# scripts/upload_to_supabase.py

import os
from pathlib import Path
from supabase import create_client

supabase = create_client(
    os.environ["PANAVERSITY_SUPABASE_URL"],
    os.environ["PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY"]
)

def upload_directory(local_path: Path, bucket_path: str):
    for file_path in local_path.rglob("*"):
        if file_path.is_file():
            rel_path = file_path.relative_to(local_path)
            remote_path = f"{bucket_path}/{rel_path}"

            with open(file_path, "rb") as f:
                supabase.storage.from_("panaversity-books").upload(
                    remote_path,
                    f.read(),
                    {"content-type": "application/octet-stream"}
                )
            print(f"Uploaded: {remote_path}")

upload_directory(Path("./data"), "")
```

---

## Phase 3: Production Deployment

### Deployment Options

#### Option 1: Docker (Recommended)

```dockerfile
# Dockerfile
FROM python:3.13-slim

# Install system dependencies
RUN apt-get update && \
    apt-get install -y libmagic1 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install uv
RUN pip install uv

# Copy project
COPY pyproject.toml uv.lock ./
COPY src/ ./src/

# Install dependencies
RUN uv sync --frozen --no-dev

# Expose port
EXPOSE 8000

# Run server
CMD ["uv", "run", "python", "-m", "panaversity_fs.server"]
```

```yaml
# docker-compose.yml
version: "3.8"
services:
  panaversity-fs:
    build: .
    ports:
      - "8000:8000"
    environment:
      - PANAVERSITY_STORAGE_BACKEND=s3
      - PANAVERSITY_S3_BUCKET=${S3_BUCKET}
      - PANAVERSITY_S3_ENDPOINT=${S3_ENDPOINT}
      - PANAVERSITY_S3_ACCESS_KEY_ID=${S3_ACCESS_KEY}
      - PANAVERSITY_S3_SECRET_ACCESS_KEY=${S3_SECRET_KEY}
      - PANAVERSITY_API_KEY=${API_KEY}
    restart: unless-stopped
```

#### Option 2: Fly.io

```toml
# fly.toml
app = "panaversity-fs"
primary_region = "ord"

[build]
  dockerfile = "Dockerfile"

[http_service]
  internal_port = 8000
  force_https = true
  auto_stop_machines = true
  auto_start_machines = true

[env]
  PANAVERSITY_SERVER_HOST = "0.0.0.0"
  PANAVERSITY_SERVER_PORT = "8000"
```

```bash
# Deploy
fly launch
fly secrets set PANAVERSITY_S3_BUCKET=...
fly secrets set PANAVERSITY_S3_ACCESS_KEY_ID=...
fly secrets set PANAVERSITY_S3_SECRET_ACCESS_KEY=...
fly deploy
```

#### Option 3: Railway

```bash
# railway.json
{
  "build": {
    "builder": "DOCKERFILE"
  },
  "deploy": {
    "startCommand": "uv run python -m panaversity_fs.server"
  }
}
```

### Health Check Endpoint

The MCP server exposes `/health` for monitoring:

```bash
curl https://your-deployment.com/health
# {"status": "healthy", "storage_backend": "s3"}
```

---

## Development Workflow

### After Migration: How Development Continues

```
┌─────────────────────────────────────────────────────────────┐
│                    Content Authors                          │
│         (AI Agents, Human Writers via MCP)                  │
└─────────────────────────┬───────────────────────────────────┘
                          │ MCP Protocol
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                  PanaversityFS MCP Server                   │
│              (Running in Cloud/Container)                   │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              Cloud Storage (R2 or Supabase)                 │
│                   books/ai-native-dev/                      │
│                   ├── content/                              │
│                   └── static/                               │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                  Docusaurus Build Pipeline                  │
│    1. Sync from PanaversityFS (get_book_archive)            │
│    2. Build static site                                     │
│    3. Deploy to GitHub Pages                                │
└─────────────────────────────────────────────────────────────┘
```

### Content Editing Workflow

1. **AI Agent** (Claude Code, custom agents) connects to MCP server
2. Uses `read_content` to fetch current lesson
3. Makes edits locally
4. Uses `write_content` with `file_hash` for conflict detection
5. Changes persisted to cloud storage immediately
6. Docusaurus build triggered (via webhook or schedule)

### Docusaurus Sync Script

```bash
#!/bin/bash
# scripts/sync_from_panaversityfs.sh

# Download book archive
curl -X POST https://panaversityfs.example.com/mcp \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $API_KEY" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"get_book_archive","arguments":{"book_id":"ai-native-dev"}}}' \
  | jq -r '.result.archive_url' \
  | xargs curl -o book.zip

# Extract to Docusaurus
unzip -o book.zip -d book-source/
mv book-source/content/* apps/learn-app/docs/
mv book-source/static/* book-source/static/

# Build Docusaurus
cd book-source && npm run build
```

---

## Separate Repository Setup

### Repository Structure for panaversity-fs

```
github.com/panaversity/panaversity-fs/
├── src/panaversity_fs/
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
├── tests/
├── scripts/
│   └── migrate_book_source.py
├── docs/
│   ├── ARCHITECTURE.md
│   ├── MCP-TOOLS.md
│   ├── SETUP.md
│   └── MIGRATION.md
├── pyproject.toml
├── uv.lock
├── Dockerfile
├── docker-compose.yml
├── .env.example
└── README.md
```

### Steps to Create Separate Repo

```bash
# 1. Create new repo
gh repo create panaversity/panaversity-fs --public

# 2. Copy files (from current location)
cp -r panaversity-fs/* /path/to/new/panaversity-fs/

# 3. Initialize and push
cd /path/to/new/panaversity-fs
git init
git add .
git commit -m "Initial commit: PanaversityFS MCP server (ADR-0018)"
git remote add origin git@github.com:panaversity/panaversity-fs.git
git push -u origin main

# 4. Set up CI/CD
# .github/workflows/test.yml for automated testing
# .github/workflows/deploy.yml for deployment
```

### PyPI Publishing (Optional)

```toml
# pyproject.toml additions
[project]
name = "panaversity-fs"
version = "1.0.0"
description = "Agent-Native Multi-Book Storage MCP Server"
authors = [{name = "Panaversity Team"}]
license = "MIT"
readme = "README.md"
requires-python = ">=3.11"
classifiers = [
    "Development Status :: 4 - Beta",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: MIT License",
    "Programming Language :: Python :: 3.11",
    "Programming Language :: Python :: 3.12",
    "Programming Language :: Python :: 3.13",
]

[project.scripts]
panaversity-fs = "panaversity_fs.server:main"

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

```bash
# Publish to PyPI
uv build
uv publish
```

---

## Checklist

### Phase 1: Local Testing

- [ ] Run migration dry-run
- [ ] Execute migration
- [ ] Start MCP server locally
- [ ] Test all 9 MCP tools
- [ ] Run all 45 tests

### Phase 2: Cloud Storage

- [ ] Choose backend (R2 or Supabase)
- [ ] Create bucket/storage
- [ ] Configure credentials
- [ ] Upload content
- [ ] Verify via MCP tools

### Phase 3: Production

- [ ] Build Docker image
- [ ] Deploy to platform (Fly.io/Railway)
- [ ] Configure secrets
- [ ] Test health endpoint
- [ ] Set up monitoring

### Development Workflow

- [ ] Configure Docusaurus sync
- [ ] Test end-to-end content editing
- [ ] Document for team

### Separate Repository

- [ ] Create new repo
- [ ] Set up CI/CD
- [ ] Update documentation links
- [ ] Publish to PyPI (optional)
