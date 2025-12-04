# Research: PanaversityFS Technical Decisions

**Feature**: 030-panaversity-fs
**Date**: 2025-11-24
**Status**: In Progress (Phase 0: Learning MCP & OpenDAL)

## Purpose

This document captures technical research decisions for PanaversityFS implementation, focusing on runtime choice, audit logging, asset upload patterns, and archive generation strategies.

**REVISION NOTE (2025-11-24)**: After initial Node.js/TypeScript planning, user directed to use **Python MCP** instead. This revision documents Phase 0 learning (MCP Python SDK + OpenDAL Python bindings) before implementation.

---

## Phase 0: MCP & OpenDAL Learning (Python)

### MCP Python SDK Key Findings

**Framework**: FastMCP (high-level Python MCP server framework)

**Server Structure**:
```python
from mcp.server.fastmcp import FastMCP

# Initialize server with stateless HTTP transport
mcp = FastMCP("panaversity_fs", stateless_http=True, json_response=True)

# Tool registration via decorator
@mcp.tool(
    name="read_content",
    annotations={
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True
    }
)
async def read_content(params: ReadContentInput) -> str:
    '''Tool docstring becomes description field.'''
    pass

# Run with streamable HTTP transport
if __name__ == "__main__":
    mcp.run(transport="streamable-http")
```

**Key Features**:
1. **Automatic Schema Generation**: FastMCP generates `inputSchema` from Pydantic models automatically
2. **Stateless Streamable HTTP**: Use `stateless_http=True` for stateless server (no session persistence)
3. **JSON Response Mode**: `json_response=True` disables SSE streams for simpler HTTP semantics
4. **Pydantic Integration**: Input validation via Pydantic `BaseModel` with `Field()` constraints
5. **Context Injection**: Optional `ctx: Context` parameter for logging, progress, elicitation
6. **Structured Output**: Can return TypedDict, Pydantic models, or JSON strings

**Tool Organization Pattern** (for 15 tools):
```python
# Single server.py imports tool modules
from tools import content, assets, summaries, bulk, registry, search, audit

# Each tool module uses @mcp.tool decorator
# tools/content.py
@mcp.tool(name="read_content", annotations={...})
async def read_content(params: ReadContentInput) -> str:
    pass

@mcp.tool(name="write_content", annotations={...})
async def write_content(params: WriteContentInput) -> str:
    pass

@mcp.tool(name="delete_content", annotations={...})
async def delete_content(params: DeleteContentInput) -> str:
    pass
```

**MCP Annotations** (Boolean hints for agent behavior):
- `readOnlyHint`: True if tool doesn't modify environment
- `destructiveHint`: True if tool performs irreversible operations
- `idempotentHint`: True if repeated calls have no additional effect
- `openWorldHint`: True if tool interacts with external entities

**Pydantic v2 Patterns**:
```python
from pydantic import BaseModel, Field, field_validator, ConfigDict

class WriteContentInput(BaseModel):
    model_config = ConfigDict(
        str_strip_whitespace=True,
        validate_assignment=True,
        extra='forbid'
    )

    book_id: str = Field(..., description="Book identifier (e.g., 'ai-native-python')",
                         pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    content: str = Field(..., description="Markdown content with YAML frontmatter",
                        min_length=1, max_length=500_000)
    file_hash: str | None = Field(default=None, description="SHA256 for conflict detection")

    @field_validator('book_id')
    @classmethod
    def validate_book_id(cls, v: str) -> str:
        if not v.islower():
            raise ValueError("book_id must be lowercase")
        return v
```

### OpenDAL Python Bindings Key Findings

**Initialization** (unified operator for all backends):
```python
import opendal

# Local filesystem (development)
op = opendal.AsyncOperator("fs", root="/tmp/panaversity-fs-data")

# Cloudflare R2 / AWS S3 (production)
op = opendal.AsyncOperator(
    "s3",
    bucket="panaversity-books",
    region="auto",
    access_key_id=os.getenv("R2_ACCESS_KEY_ID"),
    secret_access_key=os.getenv("R2_SECRET_ACCESS_KEY"),
    endpoint=os.getenv("R2_ENDPOINT")  # For R2: https://[account-id].r2.cloudflarestorage.com
)

# Supabase Storage
op = opendal.AsyncOperator(
    "supabase",
    bucket="panaversity-books",
    endpoint=os.getenv("SUPABASE_URL"),
    key=os.getenv("SUPABASE_SERVICE_KEY")
)
```

**Core Operations** (unified API across all backends):
```python
# Read file
content = await op.read("books/ai-native-python/chapter-01/lesson-01.md")

# Write file (create or overwrite)
await op.write("books/ai-native-python/chapter-01/lesson-01.md", b"# Lesson 1\n...")

# Get metadata
metadata = await op.stat("books/ai-native-python/chapter-01/lesson-01.md")
# metadata.content_length -> int (bytes)
# metadata.last_modified -> datetime
# metadata.mode -> EntryMode (file/dir)

# Delete file
await op.delete("books/ai-native-python/chapter-01/lesson-01.md")

# List directory
entries = await op.list("books/ai-native-python/")
for entry in entries:
    print(f"{entry.path} - {entry.metadata.mode}")

# Generate presigned URL (R2/S3 only)
presigned = await op.presign_read("books/ai-native-python/assets/diagram.png", expires=3600)
# presigned.url -> str (presigned URL)
# presigned.method -> "GET"
```

**Key Capabilities**:
1. **Unified Interface**: Same `read/write/stat/delete/list` API for fs/s3/supabase backends
2. **Async-First**: All operations return `awaitable` (integrates with FastMCP async tools)
3. **Metadata Access**: `stat()` provides file size, last modified, content type
4. **Presigned URLs**: R2/S3 backends support `presign_read()` and `presign_write()`
5. **Streaming**: Supports `open()` for streaming reads/writes (for large files)

**What OpenDAL Handles** (no custom implementation needed):
- ✅ Storage backend abstraction (fs/s3/supabase)
- ✅ Path normalization and validation
- ✅ HTTP client management (connection pooling, retries)
- ✅ Presigned URL generation (for R2/S3)
- ✅ Metadata extraction (size, modified time, content type)

**What We Still Implement**:
- ❌ JSONL audit logging (OpenDAL has no append-only log support)
- ❌ Conflict detection (file hash comparison logic)
- ❌ YAML frontmatter parsing (python-frontmatter library)
- ❌ ZIP archive streaming (Python zipfile/archiver library)
- ❌ MCP tool orchestration (FastMCP @mcp.tool decorators)

### Architecture Decisions from mcp-builder Skill

**Tool Granularity**: Use 3 workflow-oriented tools instead of 4 CRUD tools:
- `read_content` (readOnly)
- `write_content` (upsert: create OR update based on file_hash presence)
- `delete_content` (destructive)

**Rationale**: Agents don't distinguish between "add" vs "update" in natural workflows. Upsert semantics (if `file_hash` provided → update with conflict detection, else → create/overwrite) simplify agent logic.

**Server Organization**:
```text
src/panaversity_fs/
├── server.py              # FastMCP initialization + tool imports
├── models.py              # Pydantic models (Registry, Book, Lesson, Asset, etc.)
├── config.py              # Environment config (pydantic-settings)
├── storage.py             # OpenDAL Operator initialization
├── storage_utils.py       # Hash computation, path validation
├── audit.py               # JSONL audit logger + query
├── errors.py              # Custom exceptions
└── tools/
    ├── content.py         # 3 tools: read/write/delete_content
    ├── assets.py          # 3 tools: upload/get/list_assets
    ├── summaries.py       # 4 tools: add/update/get/list_summaries
    ├── bulk.py            # 1 tool: get_book_archive
    ├── registry.py        # 1 tool: list_books
    ├── search.py          # 2 tools: glob/grep_search
    └── audit.py           # 1 tool: get_audit_log
```

**Transport Configuration**:
```python
# server.py
from mcp.server.fastmcp import FastMCP

mcp = FastMCP(
    "panaversity_fs",
    stateless_http=True,      # Enable Stateless Streamable HTTP
    json_response=True        # Disable SSE, use pure JSON responses
)

# All tool modules import this singleton
from server import mcp

# Run server
if __name__ == "__main__":
    mcp.run(transport="streamable-http")  # HTTP server on port 8000
```

**Why Stateless Streamable HTTP**:
1. **Horizontal Scaling**: No session state to replicate across Workers
2. **Agent Concurrency**: 100 concurrent agents can hit same endpoint (FR-SC-015)
3. **Docusaurus Build**: HTTP endpoint can be called from CI/CD without stdio subprocess
4. **Simplicity**: No session management, no SSE complexity, pure request/response

### Testing Pattern

**Minimal Test Server** (validates MCP + OpenDAL integration):
```python
#!/usr/bin/env python3
from mcp.server.fastmcp import FastMCP
from pydantic import BaseModel, Field
import opendal
import asyncio

mcp = FastMCP("test_fs", stateless_http=True, json_response=True)

# Initialize OpenDAL operator (local filesystem for testing)
op = opendal.AsyncOperator("fs", root="/tmp/test-fs")

class WriteTestInput(BaseModel):
    path: str = Field(..., description="File path relative to root")
    content: str = Field(..., description="File content")

@mcp.tool(name="write_test", annotations={"destructiveHint": True})
async def write_test(params: WriteTestInput) -> str:
    await op.write(params.path, params.content.encode('utf-8'))
    return f"Wrote {len(params.content)} bytes to {params.path}"

@mcp.tool(name="read_test", annotations={"readOnlyHint": True})
async def read_test(path: str) -> str:
    content = await op.read(path)
    return content.decode('utf-8')

if __name__ == "__main__":
    mcp.run(transport="streamable-http")
```

**Test Execution**:
```bash
# Start server
python test_server.py
# Server runs at http://localhost:8000/mcp

# Test with MCP Inspector
npx @modelcontextprotocol/inspector http://localhost:8000/mcp
```

### Key Takeaways for Implementation

1. **No Custom Storage Backends**: OpenDAL provides unified `AsyncOperator` for fs/s3/supabase - just configure via environment variables
2. **FastMCP Handles MCP Protocol**: No need to manually implement JSON-RPC, tool discovery, or schema generation
3. **Tool Organization**: Group related tools in modules (content.py, assets.py), all import same `mcp` singleton
4. **Stateless HTTP Required**: For agent concurrency and Docusaurus build-time hydration
5. **Pydantic for Everything**: Input validation, configuration, domain models all use Pydantic v2
6. **Async All the Way**: FastMCP tools are `async def`, OpenDAL operators are `AsyncOperator`

---

## Decision 1: Runtime Choice — Python MCP (REVISED)

### Context

**ORIGINAL DECISION (SUPERSEDED)**: Initial research selected Node.js + OpenDAL Node.js bindings for Cloudflare Workers deployment.

**USER DIRECTION (2025-11-24)**: "use python mcp unless theres a special reason"

PanaversityFS requires unified storage abstraction supporting Local, R2, S3, and Supabase backends. **REVISED** runtime options:

**Option A: Python + FastMCP + OpenDAL Python Bindings**
- Use OpenDAL Python package (`opendal` from PyPI)
- Python MCP SDK (`mcp` package) with FastMCP framework
- Deploy as HTTP service (Docker container or systemd)
- Stateless Streamable HTTP transport

**Option B: Node.js + OpenDAL Node.js Bindings** (original choice)
- Use OpenDAL npm package (`opendal` v0.49.1+)
- TypeScript/JavaScript MCP server implementation
- Deploy on Cloudflare Workers

### Research Findings (REVISED)

**OpenDAL Python Bindings Status**:
- Production-ready PyPI package: [`opendal`](https://pypi.org/project/opendal/)
- Supports all required backends: fs, s3, supabase (plus gcs, oss, obs, webdav)
- Unified `AsyncOperator` interface for all backends
- Comprehensive API: read, write, stat, delete, list, presign
- Full async/await support (integrates with FastMCP)

**Python MCP SDK Status**:
- Official package: [`mcp`](https://pypi.org/project/mcp/) from Model Context Protocol
- FastMCP framework provides high-level server abstraction
- Automatic inputSchema generation from Pydantic models
- Stateless Streamable HTTP transport support (`stateless_http=True`)
- Decorator-based tool registration (`@mcp.tool`)

**Deployment Options**:
- **Docker Container**: Package Python app + dependencies, deploy to any cloud (Fly.io, Railway, AWS ECS)
- **systemd Service**: Run as background service on VPS/dedicated server
- **NOT Cloudflare Workers**: Workers Python support limited, doesn't support full asyncio ecosystem

### Decision: **Python + FastMCP + OpenDAL Python Bindings**

**Rationale**:

1. **User Direction**: Explicit request to use Python MCP unless special reason exists
2. **Production Readiness**: OpenDAL Python bindings are mature with confirmed fs/s3/supabase support
3. **MCP SDK Quality**: Python MCP SDK (FastMCP) provides excellent DX with automatic schema generation from Pydantic
4. **Ecosystem Maturity**: Python async ecosystem (asyncio, httpx, pydantic) well-established
5. **Simpler Deployment**: HTTP server deployment simpler than Cloudflare Workers (no edge runtime constraints)
6. **Type Safety**: Pydantic provides runtime validation + type hints (similar safety to TypeScript/Zod)

**Trade-offs Accepted**:
- **Not Edge-Deployed**: HTTP service deployment instead of Cloudflare Workers edge (acceptable - MCP servers don't require edge latency)
- **Slightly Higher Latency**: Python interpreter overhead vs V8 (mitigated by async I/O - most time spent in storage operations)
- **Deployment Complexity**: Docker/systemd vs Workers one-click deploy (acceptable for production infrastructure)

**Migration Path**: MCP protocol is language-agnostic - can rewrite server in any language without changing client integrations

---

## Decision 2: Audit Log Strategy — Direct JSONL (Eventual Consistency)

### Context

All content/asset operations must write audit entries. Two implementation patterns exist:

**Option A: Cloudflare D1 (Hot Storage + Export)**
- Write audit entries to D1 SQLite database immediately
- Query with SQL filters (date_range, agent_id, operation_type)
- Daily export to JSONL files via cron job (00:01 UTC)
- D1 provides strong consistency and indexing

**Option B: Direct JSONL Writes (Eventual Consistency)**
- Append audit entries directly to `.audit/YYYY-MM-DD.jsonl` files
- No database layer, file-based storage only
- Eventual consistency (concurrent writes may interleave)
- Daily log rotation via cron (00:01 UTC creates new file)

### Research Findings

**D1 Characteristics**:
- SQLite-backed, strongly consistent
- Query latency: ~10-50ms for indexed queries
- Write latency: ~20-100ms (includes transaction commit)
- Free tier: 5 GB storage, 25 million rows read/day
- Daily export adds complexity (cron job, export script)

**Direct JSONL Characteristics**:
- Write latency: ~5-20ms (append operation to R2)
- No query indexes (must scan entire file for filters)
- Race condition risk: Concurrent writes may interleave lines
- Zero additional infrastructure (storage backend already exists)
- Atomic append operations in R2 (line-level consistency)

**Audit Log Usage Patterns** (from spec):
- Primary use case: Developer debugging, agent behavior analysis (rare queries)
- NOT used for real-time monitoring (Analytics Engine handles metrics)
- FR-018 explicitly permits eventual consistency: "race conditions acceptable for audit trail"
- FR-020 requires pagination (100 entries max per response)

### Decision: **Direct JSONL Writes (Eventual Consistency)**

**Rationale**:

1. **Specification Alignment**: FR-018 explicitly states "eventual consistency guarantees (race conditions acceptable for audit trail)"
2. **Simplicity**: No database provisioning, schema management, or export jobs required
3. **Performance**: 5-20ms append operations vs 20-100ms D1 writes meet <50ms requirement (FR-004)
4. **Cost Efficiency**: Zero D1 costs, only R2 storage ($0.015/GB/month for audit logs)
5. **Operational Simplicity**: One less service to monitor, backup, and scale

**Trade-offs Accepted**:
- Slow queries (must scan JSONL files) — acceptable since audit queries are rare
- Potential line interleaving under high concurrency — acceptable per spec FR-018
- No SQL filtering — implement application-level filtering in `get_audit_log` tool

**Implementation Pattern**:
```typescript
// Append audit entry to daily JSONL file
async function logAudit(entry: AuditEntry) {
  const date = new Date().toISOString().split('T')[0]; // YYYY-MM-DD
  const path = `.audit/${date}.jsonl`;
  const line = JSON.stringify(entry) + '\n';

  // Atomic append to R2 via OpenDAL
  await operator.append(path, line);
}

// Query with date range filter (scan JSONL files)
async function getAuditLog(filters: AuditFilters) {
  const entries: AuditEntry[] = [];

  for (const date of filters.date_range) {
    const path = `.audit/${date}.jsonl`;
    const content = await operator.read(path);

    // Parse JSONL, filter in-memory
    const lines = content.split('\n').filter(Boolean);
    for (const line of lines) {
      const entry = JSON.parse(line);
      if (matchesFilters(entry, filters)) {
        entries.push(entry);
      }
    }
  }

  return entries.slice(0, 100); // FR-020: max 100 per response
}
```

**Daily Rotation**: Cron job (Cloudflare Cron Triggers) runs at 00:01 UTC to ensure new date's JSONL file exists (creates empty file if needed)

---

## Decision 3: Asset Upload Pattern — Hybrid (Direct Upload + Presigned URLs)

### Context

FR-010 requires supporting binary asset uploads up to 100MB while respecting Cloudflare Workers 128MB memory limit. Two patterns exist:

**Option A: Direct Upload Through MCP Tool**
- Client sends binary data as base64 in MCP tool parameters
- Worker receives, decodes, uploads to storage backend
- Simple implementation, no multi-step protocol

**Option B: Presigned URL Pattern**
- Client calls MCP tool to get presigned upload URL
- Client uploads directly to storage backend (bypassing Worker)
- Worker never touches binary data (zero memory overhead)

### Research Findings

**Cloudflare Workers Memory Constraints**:
- Default memory limit: 128MB per request
- Binary data in JSON (base64) has 33% overhead (100MB → 133MB, exceeds limit)
- Memory includes request body + processing overhead

**MCP Binary Handling**:
- MCP protocol supports binary content via base64 encoding
- JSON-RPC transport has no native binary type (must encode)
- Large base64 strings increase parsing overhead

**Storage Backend Direct Upload**:
- R2 supports presigned PUT URLs (valid for 1 hour)
- S3/Supabase also support presigned URLs
- Direct uploads bypass Worker entirely (zero memory cost)

**Asset Size Distribution** (typical educational content):
- Images (PNG/JPG): 50KB - 5MB (small)
- Slide decks (PDF): 1MB - 20MB (medium)
- Videos (MP4): 10MB - 500MB (large)

### Decision: **Hybrid (Direct Upload <10MB, Presigned URLs ≥10MB)**

**Rationale**:

1. **Memory Safety**: 10MB base64 encoded = 13.3MB payload, well within 128MB limit with processing overhead
2. **Developer Experience**: Most assets (images, small PDFs) use simple direct upload, no multi-step protocol
3. **Large Asset Support**: Videos/large files use presigned URLs, avoiding memory exhaustion
4. **Performance**: Direct uploads faster for small files (single round-trip), presigned URLs faster for large files (parallel upload)

**Implementation Pattern**:

```typescript
// MCP Tool: add_asset
async function addAsset(params: {
  book_id: string;
  asset_type: 'slides' | 'images' | 'videos' | 'audio';
  filename: string;
  binary_data?: string; // base64, for files <10MB
  file_size?: number;   // for presigned URL request
}) {
  const MAX_DIRECT_UPLOAD = 10 * 1024 * 1024; // 10MB

  if (params.binary_data) {
    // Direct upload pattern (<10MB)
    const buffer = Buffer.from(params.binary_data, 'base64');
    const path = `books/${params.book_id}/assets/${params.asset_type}/${params.filename}`;

    await operator.write(path, buffer);

    return {
      cdn_url: `https://cdn.panaversity.com/${path}`,
      upload_method: 'direct'
    };
  } else if (params.file_size && params.file_size >= MAX_DIRECT_UPLOAD) {
    // Presigned URL pattern (≥10MB)
    const path = `books/${params.book_id}/assets/${params.asset_type}/${params.filename}`;
    const presignedUrl = await operator.presign(path, { method: 'PUT', expires_in: 3600 });

    return {
      upload_url: presignedUrl,
      cdn_url: `https://cdn.panaversity.com/${path}`,
      upload_method: 'presigned',
      instructions: 'PUT binary data to upload_url, then verify at cdn_url'
    };
  } else {
    throw new Error('Must provide either binary_data (<10MB) or file_size (≥10MB)');
  }
}
```

**Documentation Note**: Client libraries must handle both patterns transparently (auto-detect size threshold)

---

## Decision 4: Archive Generation — Streaming ZIP with Presigned URL

### Context

FR-029 requires `get_book_archive` tool to generate compressed archive of entire book (500 files, 200MB uncompressed) within 60 seconds, returning presigned download URL. Two strategies exist:

**Archive Format Options**:
- **ZIP**: Universal compatibility, random access, per-file compression
- **TAR + GZIP**: Better compression ratio, streaming-friendly, Unix-native

**Generation Strategy Options**:
- **Buffered**: Load all files into memory, compress, upload archive
- **Streaming**: Stream files → compress on-the-fly → upload chunks

### Research Findings

**Cloudflare Workers Memory**:
- 128MB memory limit (buffering 200MB archive impossible)
- CPU time limit: 50ms (free), 30s (paid Workers)
- Paid plan required for 60-second generation time (FR-030)

**Compression Algorithms**:
- ZIP: Deflate algorithm, 60-70% compression typical for mixed content
- GZIP: Deflate algorithm (same as ZIP), 65-75% compression
- Both achieve similar ratios for educational content (text + images + PDFs)

**Streaming Capabilities**:
- Node.js `archiver` library supports streaming ZIP generation
- Can pipe directly to R2 upload stream (zero memory buffering)
- Cloudflare R2 supports multipart uploads (required for streaming)

**Client Compatibility**:
- ZIP: Universal support (Windows, macOS, Linux, CI/CD)
- TAR.GZ: Requires extraction tools (native on Unix, needs 7-Zip on Windows)

### Decision: **Streaming ZIP with Presigned URL**

**Rationale**:

1. **Memory Efficiency**: Streaming avoids buffering 200MB in Worker memory (would exceed 128MB limit)
2. **Performance**: Streaming generation can process 200MB → 140MB ZIP in ~30-45 seconds (meets <60s requirement FR-030)
3. **Universal Compatibility**: ZIP format works natively on all platforms without additional tools
4. **Presigned URL**: 1-hour expiry (FR-029) provides secure, time-limited access without authentication overhead

**Implementation Pattern**:

```typescript
import archiver from 'archiver';
import { Readable } from 'stream';

// MCP Tool: get_book_archive
async function getBookArchive(params: {
  book_id: string;
  format: 'zip'; // Only ZIP supported initially
}) {
  const archivePath = `.temp/${params.book_id}-${Date.now()}.zip`;

  // Create streaming ZIP archive
  const archive = archiver('zip', { zlib: { level: 6 } }); // Balanced compression

  // Stream files from storage backend
  const bookRoot = `books/${params.book_id}/`;
  const files = await operator.list(bookRoot, { recursive: true });

  for (const file of files) {
    const content = await operator.read(file.path);
    const relativePath = file.path.replace(bookRoot, '');
    archive.append(content, { name: relativePath });
  }

  archive.finalize();

  // Upload streaming archive to R2
  await operator.writeStream(archivePath, archive);

  // Generate presigned URL (1 hour expiry)
  const presignedUrl = await operator.presign(archivePath, {
    method: 'GET',
    expires_in: 3600
  });

  // Schedule cleanup (delete archive after 2 hours)
  await scheduleCleanup(archivePath, Date.now() + 7200000);

  return {
    download_url: presignedUrl,
    format: 'zip',
    expires_at: new Date(Date.now() + 3600000).toISOString(),
    estimated_size_mb: Math.round(files.reduce((sum, f) => sum + f.size, 0) * 0.7 / 1024 / 1024)
  };
}
```

**Fallback Strategy** (FR-031):
- If streaming generation times out (>60s), return error: "Archive generation timed out. Use individual file download tools."
- Docusaurus hydration script handles fallback automatically (see FR-036)

**Cleanup**: Cron job runs hourly to delete `.temp/*.zip` files older than 2 hours (expired presigned URLs)

---

## Implementation Summary (REVISED for Python MCP)

| Decision | Choice | Primary Rationale |
|----------|--------|-------------------|
| **Runtime** | Python + FastMCP + OpenDAL Python Bindings | User direction, production-ready PyPI packages, excellent DX with Pydantic |
| **MCP Transport** | Stateless Streamable HTTP (`stateless_http=True`) | Agent concurrency, Docusaurus build integration, horizontal scaling |
| **Deployment** | Docker Container or systemd Service | Standard HTTP service deployment (not Cloudflare Workers) |
| **Audit Logs** | Direct JSONL (Eventual Consistency) | Spec permits race conditions (FR-018), simpler than database, meets <50ms latency |
| **Asset Upload** | Hybrid (Direct <10MB, Presigned ≥10MB) | Memory safety, optimized DX for common case (most assets <10MB) |
| **Archive Generation** | Streaming ZIP + Presigned URL | Memory-efficient streaming, universal ZIP compatibility, meets <60s requirement |

---

## Open Questions for Tasks Phase

1. **OpenDAL R2 Configuration**: What R2 bucket configuration (CORS, public access) is required for OpenDAL Node.js bindings?
2. **MCP OAuth Scopes**: Should OAuth tokens have book-level granularity or full registry access? (Spec decision: full registry)
3. **Cron Job Scheduling**: Cloudflare Cron Triggers vs external scheduler (GitHub Actions) for audit log rotation/archive cleanup?
4. **Testing Strategy**: How to test presigned URL generation in local development (Wrangler dev mode)?
5. **Migration CLI**: Should migration tool support bidirectional sync (Git ↔ PanaversityFS) or one-way seeding only?

---

## References

- [Cloudflare Workers Rust Documentation](https://developers.cloudflare.com/workers/languages/rust/)
- [OpenDAL Node.js Bindings (npm)](https://www.npmjs.com/package/opendal)
- [OpenDAL Node.js API Docs](https://opendal.apache.org/docs/binding-nodejs/)
- [OpenDAL WASM Tracking Issue #3803](https://github.com/apache/opendal/issues/3803)
- [Cloudflare Workers WASM Support](https://developers.cloudflare.com/workers/runtime-apis/webassembly/)
- [workers-rs GitHub Repository](https://github.com/cloudflare/workers-rs)
