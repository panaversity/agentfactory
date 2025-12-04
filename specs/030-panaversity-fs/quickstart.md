# Quickstart Guide: PanaversityFS

**Feature**: 030-panaversity-fs
**Date**: 2025-11-24

## Purpose

Get PanaversityFS running locally in under 10 minutes. This guide covers local development setup, authentication configuration, first MCP tool invocation, and Docusaurus hydration integration.

---

## Prerequisites

- **Node.js**: v20.0.0+ (for OpenDAL bindings and MCP server)
- **npm**: v10.0.0+
- **Wrangler CLI**: v4.10.0+ (for Cloudflare Workers local development)
- **Git**: For cloning repository

**Optional** (for production deployment):
- Cloudflare account with Workers plan
- R2 storage bucket configured

---

## Step 1: Local Development Setup

### Clone Repository

```bash
git clone https://github.com/panaversity/panaversity-fs.git
cd panaversity-fs
```

### Install Dependencies

```bash
npm install
```

**Key dependencies installed**:
- `opendal` (v0.49.1+): Storage abstraction library
- `@modelcontextprotocol/sdk` (v1.20.2+): MCP server framework
- `zod` (v3.24.2+): Schema validation
- `archiver` (latest): ZIP generation for book archives
- `wrangler` (v4.10.0+): Cloudflare Workers CLI

### Configure Storage Backend

**For Local Development** (filesystem backend):

Create `.env` file in project root:

```env
# Storage Backend Configuration
STORAGE_BACKEND=local
LOCAL_STORAGE_PATH=./panaversity-storage

# MCP Server Configuration
MCP_SERVER_PORT=3000
OAUTH_DISABLED=true  # Skip OAuth for local dev

# Logging
LOG_LEVEL=debug
```

Create storage directory structure:

```bash
mkdir -p panaversity-storage/books
mkdir -p panaversity-storage/.audit
touch panaversity-storage/registry.yaml
```

Initialize empty registry:

```bash
cat > panaversity-storage/registry.yaml << EOF
version: "1.0"
created_at: "$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
last_updated: "$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
books: []
EOF
```

### Start MCP Server (Local Development)

```bash
npm run dev
```

**Expected output**:

```
[INFO] PanaversityFS MCP Server v0.1.0
[INFO] Storage backend: local (./panaversity-storage)
[INFO] OAuth: disabled (local dev mode)
[INFO] MCP server listening on http://localhost:3000
[INFO] 15 MCP tools registered
```

Server is now ready to accept MCP tool calls!

---

## Step 2: Authentication Configuration

### Local Development (No OAuth)

In local mode (`OAUTH_DISABLED=true`), all requests accepted without authentication. Useful for testing and development.

### Production (OAuth 2.0)

**For Cloudflare Workers deployment**:

1. **Create OAuth Provider** (using Cloudflare Workers OAuth):

```bash
wrangler secret put OAUTH_CLIENT_ID
wrangler secret put OAUTH_CLIENT_SECRET
wrangler secret put OAUTH_TOKEN_URL
```

2. **Configure Environment Variables** in `wrangler.toml`:

```toml
[env.production]
STORAGE_BACKEND = "r2"
R2_BUCKET_NAME = "panaversity-books"
OAUTH_DISABLED = "false"
OAUTH_ISSUER = "https://auth.panaversity.com"
```

3. **Obtain OAuth Token** (for MCP clients):

```bash
# Use OAuth client credentials flow
curl -X POST https://auth.panaversity.com/oauth/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=client_credentials" \
  -d "client_id=YOUR_CLIENT_ID" \
  -d "client_secret=YOUR_CLIENT_SECRET" \
  -d "scope=panaversity:books:read panaversity:books:write"
```

**Response**:

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh_token_here"
}
```

Store `access_token` in environment variable:

```bash
export PANAVERSITY_OAUTH_TOKEN="eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9..."
```

---

## Step 3: First MCP Tool Invocation

### Using MCP Client (TypeScript)

Install MCP client SDK:

```bash
npm install @modelcontextprotocol/sdk
```

**Example: Add Book to Registry**

```typescript
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StdioClientTransport } from '@modelcontextprotocol/sdk/client/stdio.js';

// Connect to MCP server
const transport = new StdioClientTransport({
  command: 'node',
  args: ['./dist/server.js'], // Path to compiled MCP server
});

const client = new Client({
  name: 'panaversity-quickstart',
  version: '1.0.0',
}, {
  capabilities: {},
});

await client.connect(transport);

// Call add_book_to_registry tool
const result = await client.request({
  method: 'tools/call',
  params: {
    name: 'add_book_to_registry',
    arguments: {
      book_id: 'ai-native-software-development',
      title: 'AI Native Software Development',
      storage_backend: 'local'
    }
  }
});

console.log('Book added:', result);
```

**Expected Response**:

```json
{
  "success": true,
  "book_id": "ai-native-software-development",
  "book_yaml_path": "books/ai-native-software-development/book.yaml",
  "audit_entry_id": "2025-11-24T12:00:00.000Z"
}
```

### Using curl (Direct HTTP)

**Add Book**:

```bash
curl -X POST http://localhost:3000/tools/call \
  -H "Content-Type: application/json" \
  -d '{
    "name": "add_book_to_registry",
    "arguments": {
      "book_id": "ai-native-software-development",
      "title": "AI Native Software Development",
      "storage_backend": "local"
    }
  }'
```

**Add Lesson Content**:

```bash
curl -X POST http://localhost:3000/tools/call \
  -H "Content-Type: application/json" \
  -d '{
    "name": "add_content",
    "arguments": {
      "book_id": "ai-native-software-development",
      "path": "lessons/part-1/chapter-01/lesson-01.md",
      "content": "---\ntitle: Introduction\nlesson_id: lesson-01\n---\n\n# Introduction\n\nWelcome to the course!"
    }
  }'
```

**Read Lesson Content**:

```bash
curl -X POST http://localhost:3000/tools/call \
  -H "Content-Type: application/json" \
  -d '{
    "name": "read_content",
    "arguments": {
      "book_id": "ai-native-software-development",
      "path": "lessons/part-1/chapter-01/lesson-01.md"
    }
  }'
```

**List Books**:

```bash
curl -X POST http://localhost:3000/tools/call \
  -H "Content-Type: application/json" \
  -d '{
    "name": "list_books",
    "arguments": {}
  }'
```

---

## Step 4: Docusaurus Hydration Script Integration

### Create Hydration Script

**File**: `docusaurus-hydration.ts`

```typescript
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StdioClientTransport } from '@modelcontextprotocol/sdk/client/stdio.js';
import * as fs from 'fs';
import * as path from 'path';
import AdmZip from 'adm-zip';

const BOOK_ID = 'ai-native-software-development';
const OUTPUT_DIR = './book-source/docs';
const ASSETS_DIR = './book-source/static/assets';

async function hydrate() {
  console.log('[Hydration] Starting PanaversityFS content download...');

  // Connect to PanaversityFS MCP server
  const transport = new StdioClientTransport({
    command: 'node',
    args: ['./dist/server.js'],
  });

  const client = new Client({
    name: 'docusaurus-hydration',
    version: '1.0.0',
  }, {
    capabilities: {},
  });

  await client.connect(transport);

  // Strategy 1: Try bulk download (fast path)
  try {
    console.log('[Hydration] Attempting bulk download via get_book_archive...');

    const archiveResult = await client.request({
      method: 'tools/call',
      params: {
        name: 'get_book_archive',
        arguments: { book_id: BOOK_ID, format: 'zip' }
      }
    });

    // Download ZIP from presigned URL
    const response = await fetch(archiveResult.download_url);
    const arrayBuffer = await response.arrayBuffer();
    const buffer = Buffer.from(arrayBuffer);

    // Extract ZIP to output directories
    const zip = new AdmZip(buffer);
    zip.extractAllTo(OUTPUT_DIR, true);

    console.log(`[Hydration] Bulk download completed in ${archiveResult.generation_time_seconds}s`);
    console.log(`[Hydration] Archive size: ${archiveResult.estimated_size_mb}MB`);

  } catch (bulkError) {
    // Strategy 2: Fallback to individual file downloads
    console.warn('[Hydration] Bulk download failed, falling back to individual files...');
    console.warn(`[Hydration] Error: ${bulkError.message}`);

    // List all books to verify connection
    const booksResult = await client.request({
      method: 'tools/call',
      params: { name: 'list_books', arguments: {} }
    });

    console.log(`[Hydration] Found ${booksResult.books.length} books in registry`);

    // Find files via glob
    const filesResult = await client.request({
      method: 'tools/call',
      params: {
        name: 'glob_search',
        arguments: {
          book_id: BOOK_ID,
          pattern: 'lessons/**/*.md'
        }
      }
    });

    console.log(`[Hydration] Downloading ${filesResult.total_count} lessons individually...`);

    // Download each lesson
    for (const file of filesResult.matches) {
      const contentResult = await client.request({
        method: 'tools/call',
        params: {
          name: 'read_content',
          arguments: {
            book_id: BOOK_ID,
            path: file.file_path
          }
        }
      });

      // Write to output directory
      const outputPath = path.join(OUTPUT_DIR, file.file_path);
      fs.mkdirSync(path.dirname(outputPath), { recursive: true });
      fs.writeFileSync(outputPath, contentResult.content);
    }

    // Download assets
    const assetsResult = await client.request({
      method: 'tools/call',
      params: {
        name: 'list_assets',
        arguments: { book_id: BOOK_ID }
      }
    });

    console.log(`[Hydration] Downloading ${assetsResult.total_count} assets...`);

    for (const asset of assetsResult.assets) {
      // Download from CDN URL
      const response = await fetch(asset.cdn_url);
      const buffer = await response.arrayBuffer();

      const outputPath = path.join(ASSETS_DIR, asset.asset_type, asset.filename);
      fs.mkdirSync(path.dirname(outputPath), { recursive: true });
      fs.writeFileSync(outputPath, Buffer.from(buffer));
    }
  }

  console.log('[Hydration] Content hydration complete!');
  await client.close();
}

// Run hydration
hydrate().catch((error) => {
  console.error('[Hydration] FATAL ERROR:', error);
  process.exit(1);
});
```

### Integrate with Docusaurus Build

**File**: `docusaurus.config.js` (add plugin)

```javascript
module.exports = {
  // ... other config
  plugins: [
    function hydrationPlugin(context, options) {
      return {
        name: 'panaversity-hydration',
        async loadContent() {
          // Run hydration script before build
          const { execSync } = require('child_process');
          console.log('[Docusaurus] Running PanaversityFS hydration...');
          execSync('npx tsx docusaurus-hydration.ts', { stdio: 'inherit' });
        },
      };
    },
  ],
};
```

### Run Docusaurus Build with Hydration

```bash
npm run build
```

**Expected output**:

```
[Docusaurus] Running PanaversityFS hydration...
[Hydration] Starting PanaversityFS content download...
[Hydration] Attempting bulk download via get_book_archive...
[Hydration] Bulk download completed in 23.4s
[Hydration] Archive size: 142MB
[Hydration] Content hydration complete!
[Docusaurus] Building static site...
[Docusaurus] Build completed successfully!
```

**Production CI/CD** (GitHub Actions example):

```yaml
name: Deploy Docusaurus

on:
  push:
    branches: [main]

jobs:
  build-deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '20'

      - name: Install dependencies
        run: npm ci

      - name: Hydrate content from PanaversityFS
        env:
          PANAVERSITY_OAUTH_TOKEN: ${{ secrets.PANAVERSITY_OAUTH_TOKEN }}
        run: npx tsx docusaurus-hydration.ts

      - name: Build Docusaurus
        run: npm run build

      - name: Deploy to Cloudflare Pages
        run: npx wrangler pages publish ./build
```

---

## Verification & Testing

### Verify Local Setup

**Check 1: MCP Server Health**

```bash
curl http://localhost:3000/health
```

**Expected**:

```json
{
  "status": "healthy",
  "storage_backends": {
    "local": "up"
  },
  "last_operation_timestamp": "2025-11-24T12:34:56.789Z"
}
```

**Check 2: Storage Directory Structure**

```bash
tree panaversity-storage
```

**Expected**:

```
panaversity-storage/
├── registry.yaml
├── .audit/
└── books/
    └── ai-native-software-development/
        ├── book.yaml
        └── lessons/
            └── part-1/
                └── chapter-01/
                    └── lesson-01.md
```

**Check 3: Audit Log**

```bash
cat panaversity-storage/.audit/$(date +%Y-%m-%d).jsonl | jq
```

**Expected** (JSONL entries):

```json
{"timestamp":"2025-11-24T12:00:00.000Z","agent_id":"local-dev","operation":"add_book_to_registry","path":"books/ai-native-software-development","status":"success","execution_time_ms":45}
{"timestamp":"2025-11-24T12:01:00.000Z","agent_id":"local-dev","operation":"add_content","path":"books/ai-native-software-development/lessons/part-1/chapter-01/lesson-01.md","status":"success","execution_time_ms":120}
```

---

## Next Steps

1. **Migrate Existing Content**: Use migration CLI to import Git repository
   ```bash
   npm run migrate -- ./existing-repo --to local --book-id ai-native-software-development
   ```

2. **Configure Production Storage**: Set up Cloudflare R2 bucket and update `wrangler.toml`

3. **Deploy MCP Server**: Deploy to Cloudflare Workers
   ```bash
   wrangler deploy
   ```

4. **Test with AI Agents**: Connect Claude Code or custom agents via MCP SDK

5. **Monitor Operations**: Query audit logs and set up Sentry/Analytics Engine

---

## Troubleshooting

### Issue: "OpenDAL service not found"

**Solution**: Verify storage backend configuration in `.env`:

```bash
# For local development
STORAGE_BACKEND=local
LOCAL_STORAGE_PATH=./panaversity-storage

# For R2 (production)
STORAGE_BACKEND=r2
R2_BUCKET_NAME=panaversity-books
R2_ACCOUNT_ID=your_account_id
```

### Issue: "OAuth token invalid"

**Solution**: Refresh token using refresh token flow:

```bash
curl -X POST https://auth.panaversity.com/oauth/token \
  -d "grant_type=refresh_token" \
  -d "refresh_token=YOUR_REFRESH_TOKEN" \
  -d "client_id=YOUR_CLIENT_ID"
```

### Issue: "Archive generation timeout"

**Solution**: Use individual file downloads (fallback automatically handled in hydration script). For large books (>500 lessons), consider parallel downloads:

```typescript
const downloadPromises = filesResult.matches.map(file =>
  client.request({ method: 'tools/call', params: { name: 'read_content', arguments: { book_id: BOOK_ID, path: file.file_path } } })
);

await Promise.all(downloadPromises);
```

---

## Resources

- **OpenDAL Node.js Docs**: https://opendal.apache.org/docs/binding-nodejs/
- **MCP SDK Documentation**: https://github.com/modelcontextprotocol/sdk
- **Cloudflare Workers**: https://developers.cloudflare.com/workers/
- **Wrangler CLI Guide**: https://developers.cloudflare.com/workers/wrangler/
- **PanaversityFS GitHub**: https://github.com/panaversity/panaversity-fs
- **Issue Tracker**: https://github.com/panaversity/panaversity-fs/issues
