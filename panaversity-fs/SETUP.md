# PanaversityFS Setup Guide

Complete setup instructions for testing with all three storage backends.

## Prerequisites

```bash
# Install dependencies
cd panaversity-fs
uv sync

# Verify installation
uv run python -c "import opendal; print('OpenDAL version:', opendal.__version__)"
```

---

## 1. Local Filesystem (fs) - Easiest Setup

### Configuration

```bash
# Set environment variables
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-fs-data

# Optional: Set CDN URL
export PANAVERSITY_CDN_BASE_URL=http://localhost:8000
```

### Create Test Data

```bash
# Create directory structure
mkdir -p /tmp/panaversity-fs-data

# Create registry
cat > /tmp/panaversity-fs-data/registry.yaml << 'EOF'
books:
  - book_id: ai-native-python
    title: AI-Native Python Development
    storage_backend: fs
    created_at: "2025-01-01T00:00:00Z"
    status: active
EOF

# Create sample book structure
mkdir -p /tmp/panaversity-fs-data/books/ai-native-python/{lessons/part-1/chapter-01,chapters/chapter-01,assets/images}

# Create sample lesson
cat > /tmp/panaversity-fs-data/books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md << 'EOF'
---
title: Introduction to Python
---

# Lesson 1: Introduction to Python

Python is a high-level programming language.

```python
def hello():
    print("Hello, World!")
```
EOF

# Create sample summary
cat > /tmp/panaversity-fs-data/books/ai-native-python/chapters/chapter-01/.summary.md << 'EOF'
# Chapter 1 Summary

Key concepts covered in this chapter.
EOF
```

### Start Server

```bash
# Start the MCP server
uv run python -m panaversity_fs.server
```

Server will run at: **http://0.0.0.0:8000/mcp**

### Run Tests

```bash
# In a new terminal
uv run python test_all_tools.py
```

---

## 2. Cloudflare R2 (S3-Compatible)

### Prerequisites

1. **Cloudflare Account**: Sign up at https://dash.cloudflare.com
2. **R2 Enabled**: Go to R2 section in dashboard
3. **Create Bucket**: Create a bucket named `panaversity-books`

### Get Credentials

#### Step 1: Create R2 API Token

1. Go to **R2 → Manage R2 API Tokens**
2. Click **Create API token**
3. Configure:
   - **Token Name**: `panaversity-fs-token`
   - **Permissions**: Object Read & Write
   - **Apply to specific buckets**: Select `panaversity-books`
4. Click **Create API token**
5. **Save these values** (shown only once):
   - Access Key ID
   - Secret Access Key
   - Endpoint URL (format: `https://<account-id>.r2.cloudflarestorage.com`)

#### Step 2: Get Account ID

1. Go to **R2 → Overview**
2. Copy your **Account ID** (shown in the sidebar)

### Configuration

```bash
# Set environment variables
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=panaversity-books
export PANAVERSITY_S3_REGION=auto
export PANAVERSITY_S3_ACCESS_KEY_ID=<your-access-key-id>
export PANAVERSITY_S3_SECRET_ACCESS_KEY=<your-secret-access-key>
export PANAVERSITY_S3_ENDPOINT=https://<account-id>.r2.cloudflarestorage.com

# Optional: Set public CDN URL (if using R2 public buckets)
export PANAVERSITY_CDN_BASE_URL=https://pub-<hash>.r2.dev
```

### Create Test Data in R2

```bash
# Method 1: Use the MCP server to create content
uv run python -m panaversity_fs.server

# In another terminal, use test script
uv run python << 'EOF'
import asyncio
import os
from panaversity_fs.tools.content import write_content
from panaversity_fs.models import WriteContentInput

async def setup_r2():
    # Create registry
    registry = """books:
  - book_id: ai-native-python
    title: AI-Native Python Development
    storage_backend: s3
    created_at: "2025-01-01T00:00:00Z"
    status: active
"""

    await write_content(WriteContentInput(
        book_id="",  # Root level
        path="registry.yaml",
        content=registry
    ))

    # Create sample lesson
    lesson = """---
title: Introduction to Python
---

# Lesson 1: Introduction to Python

Python programming basics.
"""

    await write_content(WriteContentInput(
        book_id="ai-native-python",
        path="lessons/part-1/chapter-01/lesson-01.md",
        content=lesson
    ))

    print("✅ R2 test data created")

asyncio.run(setup_r2())
EOF
```

### Run Tests with R2

```bash
uv run python test_all_tools.py
```

### Verify Data in R2 Dashboard

1. Go to **R2 → Your Bucket**
2. You should see:
   ```
   registry.yaml
   books/
   └── ai-native-python/
       └── lessons/
           └── part-1/
               └── chapter-01/
                   └── lesson-01.md
   ```

---

## 3. Supabase Storage

### Prerequisites

1. **Supabase Account**: Sign up at https://supabase.com
2. **Create Project**: Create a new project

### Get Credentials

#### Step 1: Create Storage Bucket

1. Go to **Storage** in sidebar
2. Click **New bucket**
3. Configure:
   - **Name**: `books`
   - **Public**: No (keep private)
   - **File size limit**: 50MB
4. Click **Create bucket**

#### Step 2: Get API Credentials

1. Go to **Settings → API**
2. Copy these values:
   - **Project URL**: `https://<project-ref>.supabase.co`
   - **anon public key**: (for client-side, not needed for server)
   - **service_role key**: (⚠️ Keep secret! Server-side only)

### Configuration

```bash
# Set environment variables
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://<project-ref>.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_KEY=<your-service-role-key>
export PANAVERSITY_SUPABASE_BUCKET=books

# Optional: Set public CDN URL
export PANAVERSITY_CDN_BASE_URL=https://<project-ref>.supabase.co/storage/v1/object/public/books
```

### Create Test Data in Supabase

```bash
# Start server
uv run python -m panaversity_fs.server

# In another terminal, create test data
uv run python << 'EOF'
import asyncio
import os
from panaversity_fs.tools.content import write_content
from panaversity_fs.models import WriteContentInput

async def setup_supabase():
    # Create registry
    registry = """books:
  - book_id: ai-native-python
    title: AI-Native Python Development
    storage_backend: supabase
    created_at: "2025-01-01T00:00:00Z"
    status: active
"""

    await write_content(WriteContentInput(
        book_id="",
        path="registry.yaml",
        content=registry
    ))

    # Create sample lesson
    lesson = """---
title: Introduction to Python
---

# Lesson 1: Introduction to Python

Learn Python basics with Supabase storage.
"""

    await write_content(WriteContentInput(
        book_id="ai-native-python",
        path="lessons/part-1/chapter-01/lesson-01.md",
        content=lesson
    ))

    print("✅ Supabase test data created")

asyncio.run(setup_supabase())
EOF
```

### Configure Storage Policies (Optional for Public Access)

If you want public read access:

1. Go to **Storage → Policies**
2. Click **New Policy** for `books` bucket
3. Create policy:
   - **Policy name**: `Public read access`
   - **Allowed operation**: SELECT
   - **Policy definition**: `true` (allow all)
4. Click **Review → Save policy**

### Run Tests with Supabase

```bash
uv run python test_all_tools.py
```

### Verify Data in Supabase Dashboard

1. Go to **Storage → books**
2. You should see your folder structure

---

## Testing with MCP Inspector

### Install MCP Inspector

```bash
npm install -g @modelcontextprotocol/inspector
```

### Test Each Backend

#### Local Filesystem

```bash
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-fs-data
uv run python -m panaversity_fs.server &

# In another terminal
npx @modelcontextprotocol/inspector http://localhost:8000/mcp
```

#### Cloudflare R2

```bash
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=panaversity-books
export PANAVERSITY_S3_REGION=auto
export PANAVERSITY_S3_ACCESS_KEY_ID=<your-key>
export PANAVERSITY_S3_SECRET_ACCESS_KEY=<your-secret>
export PANAVERSITY_S3_ENDPOINT=https://<account-id>.r2.cloudflarestorage.com
uv run python -m panaversity_fs.server &

npx @modelcontextprotocol/inspector http://localhost:8000/mcp
```

#### Supabase

```bash
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://<project-ref>.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_KEY=<your-service-key>
export PANAVERSITY_SUPABASE_BUCKET=books
uv run python -m panaversity_fs.server &

npx @modelcontextprotocol/inspector http://localhost:8000/mcp
```

### MCP Inspector Usage

1. Browser opens to inspector UI
2. You'll see all 14 tools listed
3. Click a tool to test it
4. Example: **read_content**
   ```json
   {
     "book_id": "ai-native-python",
     "path": "lessons/part-1/chapter-01/lesson-01.md"
   }
   ```
5. Click **Execute** to test

---

## Quick Test Script for All Backends

Save this as `test_backends.sh`:

```bash
#!/bin/bash

echo "================================"
echo "Testing PanaversityFS Backends"
echo "================================"

# Test 1: Local Filesystem
echo -e "\n📁 Testing Local Filesystem..."
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-fs-data
uv run python test_all_tools.py
echo "✅ Local filesystem test complete"

# Test 2: Cloudflare R2 (if credentials set)
if [ ! -z "$CF_R2_ACCESS_KEY" ]; then
    echo -e "\n☁️  Testing Cloudflare R2..."
    export PANAVERSITY_STORAGE_BACKEND=s3
    export PANAVERSITY_S3_BUCKET=panaversity-books
    export PANAVERSITY_S3_REGION=auto
    export PANAVERSITY_S3_ACCESS_KEY_ID=$CF_R2_ACCESS_KEY
    export PANAVERSITY_S3_SECRET_ACCESS_KEY=$CF_R2_SECRET_KEY
    export PANAVERSITY_S3_ENDPOINT=$CF_R2_ENDPOINT
    uv run python test_all_tools.py
    echo "✅ Cloudflare R2 test complete"
else
    echo "⏭️  Skipping R2 (set CF_R2_ACCESS_KEY to enable)"
fi

# Test 3: Supabase (if credentials set)
if [ ! -z "$SUPABASE_URL" ]; then
    echo -e "\n🗄️  Testing Supabase..."
    export PANAVERSITY_STORAGE_BACKEND=supabase
    export PANAVERSITY_SUPABASE_URL=$SUPABASE_URL
    export PANAVERSITY_SUPABASE_SERVICE_KEY=$SUPABASE_KEY
    export PANAVERSITY_SUPABASE_BUCKET=books
    uv run python test_all_tools.py
    echo "✅ Supabase test complete"
else
    echo "⏭️  Skipping Supabase (set SUPABASE_URL to enable)"
fi

echo -e "\n================================"
echo "✅ All backend tests complete!"
echo "================================"
```

Make it executable and run:

```bash
chmod +x test_backends.sh

# Set credentials (optional for R2 and Supabase)
export CF_R2_ACCESS_KEY=<your-r2-access-key>
export CF_R2_SECRET_KEY=<your-r2-secret>
export CF_R2_ENDPOINT=https://<account-id>.r2.cloudflarestorage.com

export SUPABASE_URL=https://<project-ref>.supabase.co
export SUPABASE_KEY=<your-service-role-key>

# Run tests
./test_backends.sh
```

---

## Troubleshooting

### Local Filesystem

**Issue**: Permission denied on /tmp/panaversity-fs-data
```bash
# Fix: Use a directory you own
export PANAVERSITY_STORAGE_ROOT=$HOME/panaversity-fs-data
mkdir -p $HOME/panaversity-fs-data
```

### Cloudflare R2

**Issue**: "Invalid credentials"
```bash
# Verify credentials
aws s3 ls --endpoint-url=$PANAVERSITY_S3_ENDPOINT

# If that fails, regenerate API token in Cloudflare dashboard
```

**Issue**: "Bucket not found"
```bash
# Create bucket via Cloudflare dashboard or AWS CLI
aws s3 mb s3://panaversity-books --endpoint-url=$PANAVERSITY_S3_ENDPOINT
```

### Supabase

**Issue**: "Authentication failed"
```bash
# Verify you're using service_role key (not anon key)
# Get it from: Settings → API → service_role key

# Test connection
curl https://<project-ref>.supabase.co/rest/v1/ \
  -H "apikey: $PANAVERSITY_SUPABASE_SERVICE_KEY" \
  -H "Authorization: Bearer $PANAVERSITY_SUPABASE_SERVICE_KEY"
```

**Issue**: "Bucket not found"
```bash
# Create bucket in Supabase dashboard:
# Storage → New Bucket → Name: books
```

---

## Next Steps

After testing all backends:

1. ✅ Verify all 14 tools work with each backend
2. ✅ Test conflict detection (try concurrent writes)
3. ✅ Verify audit logs are created
4. ✅ Test archive generation performance
5. ✅ Test search operations with real content

For production deployment, see `DEPLOYMENT.md`.
