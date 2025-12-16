# Specification: Directory MCP Server (Production)

**Status**: Ready for Implementation
**Version**: 3.0 (Final Shipping Product)
**Stack**: Python 3.12+ | FastMCP | OpenDAL
**Timeline**: 8 Hours to Production
**Created**: 2025-11-21

---

## Executive Summary

**What**: Universal MCP server for accessing organized content (books, docs, knowledge bases)
**Why**: Decouple content from filesystem, enable agent access, support cloud storage
**How**: Python MCP server with OpenDAL storage abstraction
**Timeline**: 8 hours (same day deployment)
**LOC**: ~280 lines of Python

---

## Product Vision

### The Core Insight

**"Book" is just one type of organized directory.**

Any structured content can be a "directory":

- **📚 Book**: Parts → Chapters → Lessons
- **📖 Documentation**: Sections → Pages
- **🧠 Knowledge Base**: Topics → Articles
- **🎓 Course**: Weeks → Modules → Lessons
- **🤖 Agent Mind**: Skills → Prompts → Memory

**Universal Pattern**: Organized content with schema + metadata + storage backend

---

## The Problem

### Current State ❌

```
84 chapters locked in Git filesystem
  ↓
apps/learn-app/docs/01-Part/01-chapter/01-lesson.md
  ❌ Local filesystem only
  ❌ Agents clone entire repo to access
  ❌ Cannot deploy to R2/S3
  ❌ No queryable API
  ❌ Not reusable across projects
```

### Desired State ✅

```
┌─────────────────────────────────────────┐
│  Directory MCP Server                   │
│  - Serves organized content via MCP     │
│  - Storage backend abstraction          │
│  - Local (dev) / R2 (prod) switching    │
└─────────────────────────────────────────┘
              ↓
    ┌─────────┴─────────┐
    ↓                   ↓
┌─────────┐       ┌──────────┐
│  Local  │       │    R2    │
│  (Dev)  │       │  (Prod)  │
└─────────┘       └──────────┘
```

---

## System Architecture

### High-Level Design

```
┌───────────────────────────────────────────────────────┐
│  Consumers                                             │
│  ├─ Claude Code (MCP Client)                          │
│  ├─ Python Agents (MCP Client)                        │
│  └─ Docusaurus (Hydration Script)                     │
└───────────────────────────────────────────────────────┘
                        ↓
┌───────────────────────────────────────────────────────┐
│  Directory MCP Server (Python + FastMCP)              │
│  ┌─────────────────────────────────────────────────┐ │
│  │ MCP Tools (7 total)                             │ │
│  │  1. read_content(path)                          │ │
│  │  2. write_content(path, content, agent_id)      │ │
│  │  3. list_contents(prefix)                       │ │
│  │  4. delete_content(path, agent_id)              │ │
│  │  5. glob_search(pattern)                        │ │
│  │  6. grep_search(pattern, path, context)         │ │
│  │  7. get_audit_log(filters)                      │ │
│  └─────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────┘
                        ↓
┌───────────────────────────────────────────────────────┐
│  OpenDAL Storage Abstraction                          │
│  "One Layer, All Storage"                             │
│  ┌──────────────┐         ┌────────────────┐         │
│  │ Local (Dev)  │         │  R2 (Prod)     │         │
│  └──────────────┘         └────────────────┘         │
└───────────────────────────────────────────────────────┘
```

---

## Functional Requirements

### FR-001: Read Content from Storage

**Tool**: `read_content`

**Purpose**: Read file content from any storage backend

**Signature**:

```python
@mcp.tool()
async def read_content(path: str) -> str:
    """
    Read file content from storage

    Args:
        path: File path (e.g., "01-Part/05-chapter/02-lesson.md")

    Returns:
        File content as string
    """
```

**Example**:

```python
content = await read_content(path="01-Part/05-chapter/02-lesson.md")
# Returns: "# Lesson 2\n\nContent here..."
```

**Acceptance Criteria**:

- [ ] Reads from configured storage backend (local or R2)
- [ ] Returns UTF-8 decoded content
- [ ] Throws error if file doesn't exist
- [ ] Logs read operation to audit log

---

### FR-002: Write Content to Storage

**Tool**: `write_content`

**Purpose**: Write file content to storage with audit trail

**Signature**:

```python
@mcp.tool()
async def write_content(path: str, content: str, agent_id: str) -> str:
    """
    Write file content to storage

    Args:
        path: File path
        content: File content to write
        agent_id: ID of agent making the change

    Returns:
        Success message
    """
```

**Example**:

```python
result = await write_content(
    path="01-Part/05-chapter/02-lesson.md",
    content="# Updated Lesson 2\n\n...",
    agent_id="claude-code-123"
)
# Returns: "✓ Content written to 01-Part/05-chapter/02-lesson.md"
```

**Acceptance Criteria**:

- [ ] Writes to configured storage backend
- [ ] Creates parent directories if needed
- [ ] Logs write operation with agent_id to audit log
- [ ] Returns confirmation message

---

### FR-003: List Directory Contents

**Tool**: `list_contents`

**Purpose**: List all files under a directory prefix

**Signature**:

```python
@mcp.tool()
async def list_contents(prefix: str = "") -> list[str]:
    """
    List all files under a directory prefix

    Args:
        prefix: Directory prefix (e.g., "01-Part/") or "" for all files

    Returns:
        List of file paths
    """
```

**Example**:

```python
files = await list_contents(prefix="01-Part/05-chapter/")
# Returns: [
#   "01-Part/05-chapter/01-lesson.md",
#   "01-Part/05-chapter/02-lesson.md",
#   "01-Part/05-chapter/README.md"
# ]
```

**Acceptance Criteria**:

- [ ] Lists files recursively under prefix
- [ ] Returns sorted file paths
- [ ] Empty prefix lists all files
- [ ] Logs list operation to audit log

---

### FR-004: Delete Content

**Tool**: `delete_content`

**Purpose**: Delete file from storage

**Signature**:

```python
@mcp.tool()
async def delete_content(path: str, agent_id: str) -> str:
    """
    Delete file from storage

    Args:
        path: File path to delete
        agent_id: ID of agent making the change

    Returns:
        Success message
    """
```

**Example**:

```python
result = await delete_content(
    path="01-Part/05-chapter/old-lesson.md",
    agent_id="claude-code-123"
)
# Returns: "✓ Deleted 01-Part/05-chapter/old-lesson.md"
```

**Acceptance Criteria**:

- [ ] Deletes file from storage
- [ ] Throws error if file doesn't exist
- [ ] Logs delete operation with agent_id
- [ ] Returns confirmation message

---

### FR-005: Glob Pattern Search

**Tool**: `glob_search`

**Purpose**: Find files matching glob patterns (LangChain-inspired)

**Signature**:

```python
@mcp.tool()
async def glob_search(pattern: str) -> list[str]:
    """
    Find files matching glob pattern

    Args:
        pattern: Glob pattern (e.g., "**/*.md", "**/05-chapter/**")

    Returns:
        List of matching file paths
    """
```

**Examples**:

```python
# Find all markdown files
files = await glob_search(pattern="**/*.md")

# Find all files in chapter 5
files = await glob_search(pattern="**/05-chapter/**")

# Find all lesson files
files = await glob_search(pattern="**/lesson-*.md")
```

**Acceptance Criteria**:

- [ ] Supports glob patterns (\*, \*\*, ?, [])
- [ ] Returns sorted matching files
- [ ] Empty result if no matches
- [ ] Logs search operation

---

### FR-006: Content Search (Grep)

**Tool**: `grep_search`

**Purpose**: Search file contents for text/regex with context

**Signature**:

```python
@mcp.tool()
async def grep_search(
    pattern: str,
    path: str = "",
    context_lines: int = 2,
    is_regex: bool = False
) -> list[dict]:
    """
    Search file contents for pattern

    Args:
        pattern: Text or regex pattern to search for
        path: Directory to search in (default: all files)
        context_lines: Lines of context before/after match (default: 2)
        is_regex: Treat pattern as regex (default: False)

    Returns:
        List of matches with file, line number, content, context
    """
```

**Example**:

```python
matches = await grep_search(
    pattern="async/await",
    path="04-Python-Fundamentals/",
    context_lines=3
)
# Returns: [
#   {
#     "file": "04-Python-Fundamentals/29-asyncio/02-lesson.md",
#     "line_number": 45,
#     "line": "The `async/await` syntax provides...",
#     "context_before": ["", "## Understanding Async/Await", ""],
#     "context_after": ["a cleaner way...", "", "### Basic Example"]
#   }
# ]
```

**Acceptance Criteria**:

- [ ] Searches file contents (text and regex)
- [ ] Returns matches with context lines
- [ ] Searches all .md files by default
- [ ] Logs search operation

---

### FR-007: Audit Log Query

**Tool**: `get_audit_log`

**Purpose**: Query operation history for tracking and debugging

**Signature**:

```python
@mcp.tool()
async def get_audit_log(
    operation: Optional[str] = None,
    agent_id: Optional[str] = None,
    since: Optional[str] = None,
    limit: int = 100
) -> list[dict]:
    """
    Query audit log for operations

    Args:
        operation: Filter by operation type
        agent_id: Filter by agent ID
        since: Filter by timestamp (ISO 8601)
        limit: Max entries to return (default: 100)

    Returns:
        List of audit log entries
    """
```

**Example**:

```python
# Get all writes by claude-code
log = await get_audit_log(operation="write", agent_id="claude-code-123")

# Get recent operations
log = await get_audit_log(since="2025-11-21T00:00:00Z", limit=50)
```

**Acceptance Criteria**:

- [ ] Queries audit log with filters
- [ ] Returns sorted by timestamp (newest first)
- [ ] Supports pagination via limit
- [ ] Returns structured log entries

---

### FR-008: Storage Backend Abstraction

**Requirement**: Transparent switching between local and cloud storage

**Implementation**:

```python
from opendal import Operator

# Configuration-driven backend selection
if config["storage"]["backend"] == "local":
    storage = Operator(
        "fs",
        root=config["storage"]["path"]
    )
elif config["storage"]["backend"] == "r2":
    storage = Operator(
        "s3",
        bucket=config["storage"]["bucket"],
        endpoint=config["storage"]["endpoint"],
        access_key_id=config["storage"]["access_key_id"],
        secret_access_key=config["storage"]["secret_access_key"]
    )
```

**Configuration**:

```json
// Development (local)
{
  "storage": {
    "backend": "local",
    "path": "./content"
  }
}

// Production (R2)
{
  "storage": {
    "backend": "r2",
    "bucket": "panaversity-book",
    "endpoint": "https://xxx.r2.cloudflarestorage.com",
    "access_key_id": "${R2_ACCESS_KEY_ID}",
    "secret_access_key": "${R2_SECRET_ACCESS_KEY}"
  }
}
```

**Acceptance Criteria**:

- [ ] Switch backend via config (no code changes)
- [ ] All 7 tools work identically on both backends
- [ ] Environment variable substitution for credentials
- [ ] Graceful error handling if backend unavailable

---

## Non-Functional Requirements

### NFR-001: Performance

- **Read latency**: < 100ms (local), < 500ms (R2) at p95
- **Write latency**: < 200ms (local), < 1s (R2) at p95
- **Search**: < 2s for grep across 84 chapters
- **Concurrent ops**: 10 simultaneous operations

### NFR-002: Reliability

- **Error handling**: Graceful degradation if storage unavailable
- **Retry logic**: Exponential backoff for transient failures
- **Audit integrity**: All operations logged, no lost entries
- **Data consistency**: Atomic writes (all-or-nothing)

### NFR-003: Security

- **Credentials**: Stored in environment variables (not code)
- **Audit trail**: Every operation tracked with agent_id
- **Read-only mode**: Optional config to prevent writes
- **Input validation**: Sanitize paths to prevent traversal attacks

### NFR-004: Maintainability

- **Code simplicity**: < 300 LOC total
- **Type hints**: Full Python type annotations
- **Documentation**: Docstrings for all tools
- **Testing**: 80%+ code coverage

---

## User Stories

### US-001: Agent Reads Lesson Content

**As an** AI agent
**I want to** read lesson content from storage
**So that** I can reference examples or build new content

**Acceptance Criteria**:

- [ ] Agent calls `read_content(path="01-Part/05-chapter/02-lesson.md")`
- [ ] Returns lesson markdown content
- [ ] Works on both local and R2 storage
- [ ] Completes in < 500ms

**Example**:

```python
# Agent workflow
content = await mcp.call_tool("read_content", {
    "path": "01-Part/05-chapter/02-lesson.md"
})
# Uses content to generate new lesson
```

---

### US-002: Agent Searches for Topics

**As an** AI agent
**I want to** search for specific topics across chapters
**So that** I can find related content quickly

**Acceptance Criteria**:

- [ ] Agent calls `grep_search(pattern="async/await")`
- [ ] Returns all matches with context
- [ ] Searches only .md files
- [ ] Completes in < 2s for 84 chapters

**Example**:

```python
# Find all mentions of "async/await"
matches = await mcp.call_tool("grep_search", {
    "pattern": "async/await",
    "path": "04-Python-Fundamentals/",
    "context_lines": 3
})
# Agent reviews 5 matches found
```

---

### US-003: Deploy to Production (R2)

**As a** developer
**I want to** deploy content to R2
**So that** production agents access from cloud storage

**Acceptance Criteria**:

- [ ] Run migration script to upload 84 chapters to R2
- [ ] Update config.json to use R2 backend
- [ ] Restart MCP server
- [ ] Agents query same API, content comes from R2
- [ ] No code changes required

**Workflow**:

```bash
# 1. Migrate content
python scripts/migrate.py --source ./book-source/docs --target r2://panaversity-book/

# 2. Update config
vim config.json  # Change backend: "local" → "r2"

# 3. Restart server
python src/server.py

# ✅ Now serving from R2
```

---

## Technical Stack

### Languages & Frameworks

```python
Python 3.12+              # Core language
FastMCP 0.5+              # High-level MCP wrapper
opendal 0.46+             # Storage abstraction
fnmatch (stdlib)          # Glob pattern matching
re (stdlib)               # Regex for grep
json (stdlib)             # Audit log format
asyncio (stdlib)          # Async operations
```

### Dependencies

```txt
# requirements.txt
mcp==1.0.0          # Official MCP SDK
fastmcp==0.5.0      # High-level MCP wrapper
opendal==0.46.0     # Storage abstraction
python-dotenv==1.0  # Environment variables
```

**Total**: 4 external dependencies

### Why This Stack?

| Reason      | Benefit                                           |
| ----------- | ------------------------------------------------- |
| **Python**  | 60% less code than TypeScript (280 vs 700 LOC)    |
| **FastMCP** | Decorator-based API (simpler than raw MCP SDK)    |
| **OpenDAL** | Unified storage interface (local + R2 + S3 + ...) |
| **stdlib**  | fnmatch + re included (no extra deps)             |

---

## Implementation Structure

### File Organization

```
directory-mcp-server/
├── pyproject.toml           # Python project config
├── requirements.txt         # Dependencies
├── config.json              # Runtime configuration
├── .env.example             # Environment template
├── README.md
│
├── src/
│   ├── __init__.py
│   ├── server.py            # MCP server entry (70 LOC)
│   │
│   ├── storage/
│   │   ├── __init__.py
│   │   └── backend.py       # OpenDAL wrapper (30 LOC)
│   │
│   ├── search/
│   │   ├── __init__.py
│   │   ├── glob.py          # Glob search (20 LOC)
│   │   └── grep.py          # Grep search (40 LOC)
│   │
│   └── audit/
│       ├── __init__.py
│       └── logger.py        # Audit logging (30 LOC)
│
├── scripts/
│   ├── migrate.py           # Git → R2 migration (40 LOC)
│   └── hydrate.py           # R2 → Docusaurus (50 LOC)
│
└── tests/
    ├── test_storage.py
    ├── test_search.py
    ├── test_audit.py
    └── test_server.py
```

**Total LOC**: ~280 (vs 700 in TypeScript)

---

## Success Criteria

### Technical Success

- [ ] All 7 MCP tools functional
- [ ] Local + R2 backends working
- [ ] 84 chapters migrated to R2
- [ ] Glob + grep search returning accurate results
- [ ] Audit log capturing all operations
- [ ] < 500ms read latency from R2 (p95)

### Workflow Success

- [ ] Agent reads lesson content successfully
- [ ] Agent searches for "async/await" and finds 5+ matches
- [ ] Switch from local → R2 via config change only
- [ ] Docusaurus hydration script fetches from R2
- [ ] Website builds and deploys from R2 content

### Adoption Success

- [ ] Claude Code using MCP server for tutorsgpt
- [ ] Python agents querying content
- [ ] 100+ operations/day in audit log
- [ ] Zero production errors in first week

---

## Out of Scope (Phase 2)

**Deferred features**:

- ❌ Multi-directory support (register multiple books)
- ❌ Directory schemas (book vs docs vs knowledge)
- ❌ Vector search (LanceDB integration)
- ❌ Watcher agents (auto-summaries)
- ❌ REST API (FastAPI integration)
- ❌ WebSocket (real-time updates)

**Why deferred**: Focus on single directory (tutorsgpt) first, validate usage, then expand.

---

## Risk Analysis

| Risk                               | Impact | Probability | Mitigation                                |
| ---------------------------------- | ------ | ----------- | ----------------------------------------- |
| OpenDAL breaking changes (pre-1.0) | MEDIUM | MEDIUM      | Pin version, can swap to boto3 in 2 hours |
| R2 downtime                        | HIGH   | LOW         | Local fallback config                     |
| Performance issues                 | MEDIUM | LOW         | Implement caching in Phase 2              |
| Python async complexity            | LOW    | LOW         | FastMCP abstracts complexity              |

**Overall Risk**: LOW ✅

---

## Timeline to Production

### 8-Hour Implementation Plan

| Hours   | Phase      | Deliverable                                 |
| ------- | ---------- | ------------------------------------------- |
| 0-1.5   | Setup      | Project initialized, dependencies installed |
| 1.5-2.5 | Storage    | OpenDAL wrapper implemented, tested         |
| 2.5-3.5 | Audit      | Audit logger implemented, tested            |
| 3.5-5   | Search     | Glob + grep implemented, tested             |
| 5-6     | MCP Server | 7 tools implemented, tested                 |
| 6-8     | Scripts    | Migration + hydration scripts, deployed     |

**End of Hour 8**: ✅ **LIVE IN PRODUCTION**

---

## Deployment

### Docker Container

```dockerfile
FROM python:3.12-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY src/ ./src/
COPY config.json .

# Run server
CMD ["python", "src/server.py"]
```

**Image size**: ~100MB

---

## Configuration Example

### config.json (Development)

```json
{
  "storage": {
    "backend": "local",
    "path": "./content"
  },
  "audit": {
    "enabled": true,
    "log_file": "audit.jsonl"
  }
}
```

### config.json (Production)

```json
{
  "storage": {
    "backend": "r2",
    "bucket": "panaversity-book",
    "endpoint": "https://xxx.r2.cloudflarestorage.com",
    "access_key_id": "${R2_ACCESS_KEY_ID}",
    "secret_access_key": "${R2_SECRET_ACCESS_KEY}"
  },
  "audit": {
    "enabled": true,
    "log_file": "audit.jsonl"
  }
}
```

---

## Approval Checklist

Before implementation, confirm:

- [ ] Python stack approved (vs TypeScript)
- [ ] 8-hour timeline acceptable
- [ ] 7 MCP tools meet requirements
- [ ] OpenDAL usage approved (pre-1.0 risk accepted)
- [ ] Local + R2 backends sufficient (no S3 needed for MVP)
- [ ] Audit logging approach approved (JSONL file)

---

**Document Status**: ✅ **READY FOR IMPLEMENTATION**

**Next Action**: Approve spec → Create plan.md → Start implementation

**Estimated Completion**: 8 hours from start

---

**END OF SPECIFICATION**
