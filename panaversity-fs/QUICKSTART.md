# PanaversityFS Quick Start

## 1️⃣ Local Filesystem (Fastest - 2 minutes)

```bash
# Setup
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-fs-data

# Create test data
uv run python setup_test_data.py

# Start server
uv run python -m panaversity_fs.server &

# Run tests
uv run python test_all_tools.py

# Test with MCP Inspector
npx @modelcontextprotocol/inspector http://localhost:8000/mcp
```

**✅ Done!** Server running with local storage.

---

## 2️⃣ Cloudflare R2 (10 minutes)

### Get Credentials (5 min)

1. Go to https://dash.cloudflare.com → **R2**
2. **Create bucket**: `panaversity-books`
3. **Manage R2 API Tokens** → **Create API token**
   - Permissions: Object Read & Write
   - Buckets: `panaversity-books`
4. **Copy**: Access Key ID, Secret Access Key, Endpoint URL

### Setup & Test

```bash
# Configure
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=panaversity-books
export PANAVERSITY_S3_REGION=auto
export PANAVERSITY_S3_ACCESS_KEY_ID=<your-access-key>
export PANAVERSITY_S3_SECRET_ACCESS_KEY=<your-secret-key>
export PANAVERSITY_S3_ENDPOINT=https://<account-id>.r2.cloudflarestorage.com

# Create test data
uv run python setup_test_data.py

# Start server
uv run python -m panaversity_fs.server &

# Run tests
uv run python test_all_tools.py
```

**Verify in Dashboard**: Go to R2 → `panaversity-books` → See files

---

## 3️⃣ Supabase Storage (10 minutes)

### Get Credentials (5 min)

1. Go to https://supabase.com → **New Project**
2. **Storage** → **New bucket** → Name: `books` (private)
3. **Settings → API**:
   - Copy **Project URL**: `https://<project-ref>.supabase.co`
   - Copy **service_role key** (⚠️ Keep secret!)

### Setup & Test

```bash
# Configure
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://<project-ref>.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_KEY=<your-service-role-key>
export PANAVERSITY_SUPABASE_BUCKET=books

# Create test data
uv run python setup_test_data.py

# Start server
uv run python -m panaversity_fs.server &

# Run tests
uv run python test_all_tools.py
```

**Verify in Dashboard**: Go to Storage → `books` → See files

---

## 🧪 Testing All 14 Tools

### Automated Tests

```bash
# Test all tools
uv run python test_all_tools.py

# Expected output:
# ✅ ALL TESTS PASSED - 14/14 tools working
```

### Manual Testing with MCP Inspector

```bash
# 1. Start server (if not running)
uv run python -m panaversity_fs.server &

# 2. Open inspector
npx @modelcontextprotocol/inspector http://localhost:8000/mcp

# 3. Test tools in browser UI
```

### Test Individual Tools

```python
# Test read_content
uv run python -c "
import asyncio
from panaversity_fs.tools.content import read_content
from panaversity_fs.models import ReadContentInput

async def test():
    result = await read_content(ReadContentInput(
        book_id='ai-native-python',
        path='lessons/part-1/chapter-01/lesson-01.md'
    ))
    print(result)

asyncio.run(test())
"
```

---

## 📋 Tool Testing Checklist

### Content Tools (3)
```bash
# read_content
{"book_id": "ai-native-python", "path": "lessons/part-1/chapter-01/lesson-01.md"}

# write_content
{"book_id": "ai-native-python", "path": "lessons/test.md", "content": "# Test"}

# delete_content
{"book_id": "ai-native-python", "path": "lessons/test.md"}
```

### Summary Tools (4)
```bash
# get_summary
{"book_id": "ai-native-python", "chapter_id": "chapter-01"}

# update_summary
{"book_id": "ai-native-python", "chapter_id": "chapter-01", "content": "# Updated"}

# list_summaries
{"book_id": "ai-native-python"}
```

### Registry Tools (1)
```bash
# list_books
{}
```

### Search Tools (2)
```bash
# glob_search
{"book_id": "ai-native-python", "pattern": "**/*.md"}

# grep_search
{"book_id": "ai-native-python", "pattern": "Python", "max_results": 10}
```

### Bulk Tools (1)
```bash
# get_book_archive
{"book_id": "ai-native-python"}
```

### Asset Tools (3)
```bash
# upload_asset (requires base64 binary data)
{"book_id": "ai-native-python", "asset_type": "images", "filename": "test.png", "binary_data": "<base64>"}

# get_asset
{"book_id": "ai-native-python", "asset_type": "images", "filename": "test.png"}

# list_assets
{"book_id": "ai-native-python", "asset_type": "images"}
```

---

## 🔍 Verification Commands

### Check Data Exists

**Local Filesystem:**
```bash
ls -R /tmp/panaversity-fs-data/
cat /tmp/panaversity-fs-data/registry.yaml
```

**Cloudflare R2:**
```bash
# Use AWS CLI with R2 endpoint
aws s3 ls s3://panaversity-books/ --recursive --endpoint-url=$PANAVERSITY_S3_ENDPOINT
```

**Supabase:**
```bash
# Check via dashboard: Storage → books
# Or use Supabase CLI
supabase storage list books
```

### Check Audit Logs

```bash
# Local filesystem
cat /tmp/panaversity-fs-data/.audit/$(date +%Y-%m-%d).jsonl | jq

# Other backends
# Logs are stored in .audit/ directory in the storage backend
```

---

## 🚨 Troubleshooting

### Server won't start
```bash
# Check configuration
uv run python -c "
from panaversity_fs.config import get_config
config = get_config()
print(f'Backend: {config.storage_backend}')
config.validate_backend_config()
"

# Check imports
uv run python -c "
from panaversity_fs.server import mcp
from panaversity_fs.tools import content
print('✅ Imports successful')
"
```

### Tests fail
```bash
# Verify test data exists
uv run python -c "
import asyncio
from panaversity_fs.storage import get_operator

async def check():
    op = get_operator()
    try:
        content = await op.read('registry.yaml')
        print('✅ registry.yaml exists')
    except:
        print('❌ registry.yaml not found')
        print('Run: uv run python setup_test_data.py')

asyncio.run(check())
"
```

### Storage backend errors

**R2**: Invalid credentials
```bash
# Regenerate API token in Cloudflare dashboard
# Verify endpoint format: https://<account-id>.r2.cloudflarestorage.com
```

**Supabase**: Authentication failed
```bash
# Verify you're using service_role key (not anon key)
# Check: Settings → API → service_role key
```

---

## 📚 Next Steps

1. ✅ Test all 14 tools with your preferred backend
2. ✅ Verify audit logs are created
3. ✅ Test conflict detection (concurrent writes)
4. ✅ Benchmark performance with your data
5. 📖 Read full documentation: `SETUP.md`, `TESTING.md`

---

## 🔗 Resources

- **Full Setup Guide**: `SETUP.md`
- **Testing Guide**: `TESTING.md`
- **API Docs**: See tool docstrings in `src/panaversity_fs/tools/`
- **Draft PR**: https://github.com/panaversity/ai-native-software-development/pull/299

---

**Happy Testing! 🎉**
