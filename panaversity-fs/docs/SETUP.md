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

## JWT Authentication (Optional)

PanaversityFS supports OAuth 2.1 compliant JWT authentication. Authentication is **optional** - if not configured, the server runs in development mode without auth.

### Enable Authentication

Set the `PANAVERSITY_JWT_SECRET` environment variable to enable JWT authentication:

```bash
# Required: Secret key for HS256 JWT verification
export PANAVERSITY_JWT_SECRET=your-secret-key-min-32-chars

# Optional: JWT algorithm (default: HS256)
export PANAVERSITY_JWT_ALGORITHM=HS256

# Optional: JWT issuer URL validation
export PANAVERSITY_AUTH_ISSUER=https://auth.example.com

# Optional: JWT audience validation
export PANAVERSITY_AUTH_AUDIENCE=panaversity-fs

# Optional: Required scopes (comma-separated, default: read,write)
export PANAVERSITY_REQUIRED_SCOPES_STR=read,write

# Optional: Server URL for RFC 9728 metadata
export PANAVERSITY_RESOURCE_SERVER_URL=https://api.panaversity.com
```

### Authentication Modes

| Mode | `JWT_SECRET` | Behavior |
|------|--------------|----------|
| **Dev Mode** | Not set | No authentication required |
| **Production** | Set | All requests require valid JWT |

### Making Authenticated Requests

When authentication is enabled, include a JWT in the `Authorization` header:

```bash
# Get a test token (for development)
TOKEN=$(uv run python -c "
from panaversity_fs.auth import create_test_token
print(create_test_token())
")

# Make authenticated request
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $TOKEN" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
```

### JWT Token Requirements

Tokens must include:

| Claim | Required | Description |
|-------|----------|-------------|
| `sub` | Yes | Subject (client identifier) |
| `scopes` | Yes | List or comma-separated string of scopes |
| `exp` | Yes | Expiration timestamp |
| `iat` | Yes | Issued-at timestamp |
| `iss` | If configured | Issuer URL (validated against `AUTH_ISSUER`) |
| `aud` | If configured | Audience (validated against `AUTH_AUDIENCE`) |

Example token payload:
```json
{
  "sub": "agent-123",
  "scopes": ["read", "write"],
  "iat": 1700000000,
  "exp": 1700003600,
  "iss": "https://auth.example.com"
}
```

### Generate Test Tokens

```python
from panaversity_fs.auth import create_test_token

# Default token (read, write scopes, 1 hour expiry)
token = create_test_token()

# Custom scopes
admin_token = create_test_token(scopes=["read", "write", "admin"])

# Custom expiry (5 minutes)
short_token = create_test_token(expires_in_seconds=300)

# Custom subject
agent_token = create_test_token(subject="my-agent-id")
```

### Server Startup Messages

```bash
# Without auth (dev mode)
[PanaversityFS] Running in dev mode (no authentication)

# With auth enabled
[PanaversityFS] JWT authentication enabled
[PanaversityFS] Issuer: https://auth.example.com
[PanaversityFS] Required scopes: ['read', 'write']
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

# Create sample book structure (ADR-0018: Docusaurus-aligned)
mkdir -p /tmp/panaversity-fs-data/books/ai-native-python/{content/01-Part/01-Chapter,static/images}

# Create sample lesson
cat > /tmp/panaversity-fs-data/books/ai-native-python/content/01-Part/01-Chapter/01-intro.md << 'EOF'
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

# Create sample summary (ADR-0018: sibling file with .summary.md suffix)
cat > /tmp/panaversity-fs-data/books/ai-native-python/content/01-Part/01-Chapter/01-intro.summary.md << 'EOF'
# Lesson 1 Summary

Key concepts covered in this lesson.
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
# Run the test suite
uv run pytest tests/ -v
```

---

## 2. Cloudflare R2 (S3-Compatible)

### Prerequisites

1. **Cloudflare Account**: Sign up at https://dash.cloudflare.com
2. **R2 Enabled**: Go to R2 section in dashboard

### Step 1: Create R2 Bucket

1. Go to **Cloudflare Dashboard** → **R2 Object Storage**
2. Click **Create bucket**
3. Configure:
   - **Bucket name**: `panaversity-books`
   - **Location hint**: (choose closest region, or leave default)
4. Click **Create bucket**

### Step 2: Enable Public Access (for CDN URLs)

After creating the bucket:

1. Click on your bucket (`panaversity-books`)
2. Go to **Settings** tab
3. Under **Public access**, click **Allow Access**
4. Choose **R2.dev subdomain** (easiest option)
5. Click **Allow Access** to confirm
6. Copy the public URL shown (e.g., `https://pub-abc123xyz.r2.dev`)

This gives you a public CDN URL like:
```
https://pub-abc123xyz.r2.dev/books/ai-native-dev/static/images/...
```

### Step 3: Create R2 API Token

1. Go to **R2** → **Manage R2 API Tokens** (top right)
2. Click **Create API token**
3. Configure:
   - **Token name**: `panaversity-fs-token`
   - **Permissions**: **Object Read & Write**
   - **Specify bucket(s)**: Select `panaversity-books`
   - **TTL**: (optional, leave blank for no expiry)
4. Click **Create API token**
5. **IMPORTANT - Save these values** (shown only once):
   ```
   Access Key ID:     xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
   Secret Access Key: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

   Endpoint:          https://<account-id>.r2.cloudflarestorage.com
   ```

### Step 4: Get Your Account ID

Your Account ID is shown in:
- The endpoint URL from Step 3
- Or go to **R2 Overview** → look in the right sidebar

### Configuration

Create/update your `.env` file:

```bash
# Cloudflare R2 Configuration
PANAVERSITY_STORAGE_BACKEND=s3
PANAVERSITY_S3_BUCKET=panaversity-books
PANAVERSITY_S3_REGION=auto
PANAVERSITY_S3_ENDPOINT=https://<account-id>.r2.cloudflarestorage.com
PANAVERSITY_S3_ACCESS_KEY_ID=<your-access-key-id>
PANAVERSITY_S3_SECRET_ACCESS_KEY=<your-secret-access-key>

# Public CDN URL (from Step 2)
PANAVERSITY_CDN_BASE_URL=https://pub-<hash>.r2.dev
```

Or export as environment variables:

```bash
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=panaversity-books
export PANAVERSITY_S3_REGION=auto
export PANAVERSITY_S3_ACCESS_KEY_ID=<your-access-key-id>
export PANAVERSITY_S3_SECRET_ACCESS_KEY=<your-secret-access-key>
export PANAVERSITY_S3_ENDPOINT=https://<account-id>.r2.cloudflarestorage.com
export PANAVERSITY_CDN_BASE_URL=https://pub-<hash>.r2.dev
```

### Step 5: Migrate Content

```bash
# Dry run first
uv run python scripts/migrate_book_source.py --dry-run

# Full migration with URL rewriting
uv run python scripts/migrate_book_source.py --rewrite-urls

# Or migrate in parts
uv run python scripts/migrate_book_source.py --assets-only      # Assets first
uv run python scripts/migrate_book_source.py --content-only --rewrite-urls  # Then content
```

### Step 6: Verify Upload

1. Go to **R2** → **panaversity-books** → **Objects**
2. You should see:
   ```
   books/
   └── ai-native-dev/
       ├── content/
       │   └── 01-Part/
       │       └── 01-Chapter/
       │           └── 01-lesson.md
       └── static/
           ├── images/
           │   └── *.png
           └── slides/
               └── *.pdf
   ```

3. Test a public URL:
   ```bash
   curl -I "https://pub-<hash>.r2.dev/books/ai-native-dev/content/01-Part/README.md"
   # Should return 200 OK
   ```

### Custom Domain (Optional)

Instead of `pub-<hash>.r2.dev`, you can use a custom domain:

1. In bucket **Settings** → **Public access** → **Custom Domains**
2. Click **Connect Domain**
3. Enter your domain (e.g., `cdn.panaversity.com`)
4. Follow DNS configuration instructions
5. Update `.env`:
   ```bash
   PANAVERSITY_CDN_BASE_URL=https://cdn.panaversity.com
   ```

### R2 Pricing (as of 2024)

| Resource | Free Tier | After Free Tier |
|----------|-----------|-----------------|
| Storage | 10 GB/month | $0.015/GB/month |
| Class A ops (writes) | 1M/month | $4.50/M |
| Class B ops (reads) | 10M/month | $0.36/M |
| Egress | Free | Free (no bandwidth charges!) |

**Note**: R2 has no egress fees, making it ideal for CDN use cases.

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
        path="content/01-Part/01-Chapter/01-intro.md",
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
2. You should see (ADR-0018 structure):
   ```
   registry.yaml
   books/
   └── ai-native-python/
       └── content/
           └── 01-Part/
               └── 01-Chapter/
                   └── 01-intro.md
   ```

---

## 3. Supabase Storage

### Prerequisites

1. **Supabase Account**: Sign up at https://supabase.com
2. **Create Project**: Create a new project

### Step 1: Create Storage Bucket

1. Go to **Storage** in sidebar
2. Click **New bucket**
3. Configure:
   - **Name**: `panaversity-books`
   - **Public bucket**: **ON** (toggle this for public CDN access)
   - **File size limit**: 50MB (or higher for large PDFs)
4. Click **Create bucket**

**Important**: Toggle **Public bucket** ON if you want public CDN URLs. Otherwise you'll get 404 errors.

### Step 2: Get API Credentials

1. Go to **Project Settings** (gear icon) → **API**
2. Copy these values:
   - **Project URL**: `https://<project-ref>.supabase.co`
   - **service_role key**: Under "Project API keys" section (⚠️ Keep secret!)

**Note**: Use `service_role` key (not `anon` key) for server-side uploads.

### Step 3: Configure Environment

Create/update your `.env` file:

```bash
# Supabase Storage Configuration
PANAVERSITY_STORAGE_BACKEND=supabase
PANAVERSITY_SUPABASE_URL=https://<project-ref>.supabase.co
PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
PANAVERSITY_SUPABASE_BUCKET=panaversity-books

# CDN URL (auto-generated from above, but can be explicit)
PANAVERSITY_CDN_BASE_URL=https://<project-ref>.supabase.co/storage/v1/object/public/panaversity-books
```

Or export as environment variables:

```bash
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://<project-ref>.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY=<your-service-role-key>
export PANAVERSITY_SUPABASE_BUCKET=panaversity-books
```

### Step 4: Migrate Content

```bash
# Dry run first
uv run python scripts/migrate_book_source.py --dry-run

# Full migration with URL rewriting
uv run python scripts/migrate_book_source.py --rewrite-urls
```

### Step 5: Verify Upload

1. Go to **Storage** → **panaversity-books**
2. You should see:
   ```
   books/
   └── ai-native-dev/
       ├── content/
       └── static/
           ├── images/
           └── slides/
   ```

3. Test a public URL:
   ```bash
   curl -I "https://<project-ref>.supabase.co/storage/v1/object/public/panaversity-books/books/ai-native-dev/content/01-Part/README.md"
   # Should return 200 OK
   ```

### Making Existing Bucket Public

If you already created a private bucket:

1. Go to **Storage** → click on your bucket
2. Click **Settings** (gear icon in bucket view)
3. Toggle **Public bucket** to **ON**
4. Confirm the change

### Supabase Pricing (as of 2024)

| Resource | Free Tier | Pro ($25/mo) |
|----------|-----------|--------------|
| Storage | 1 GB | 100 GB |
| Bandwidth | 2 GB/month | 250 GB/month |
| File uploads | 50 MB max | 5 GB max |

**Note**: Free tier is sufficient for development and small books.

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
        path="content/01-Part/01-Chapter/01-intro.md",
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
2. You'll see all 9 tools (ADR-0018) listed
3. Click a tool to test it
4. Example: **read_content**
   ```json
   {
     "book_id": "ai-native-python",
     "path": "content/01-Part/01-Chapter/01-intro.md"
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

1. ✅ Verify all 9 tools (ADR-0018) work with each backend
2. ✅ Test conflict detection (try concurrent writes)
3. ✅ Verify audit logs are created
4. ✅ Test archive generation performance
5. ✅ Test search operations with real content

For production deployment, see `DEPLOYMENT.md`.
