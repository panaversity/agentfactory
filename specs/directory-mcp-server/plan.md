# Implementation Plan: Directory MCP Server

**Status**: Ready for Execution
**Version**: 3.0 (Final Shipping Product)
**Stack**: Python 3.12+ | FastMCP | OpenDAL
**Timeline**: 8 Hours to Production
**Date**: 2025-11-24

---

## Overview

This is the **hour-by-hour implementation plan** for deploying the Directory MCP Server to production. By end of Hour 8, the system will be live with 84 chapters migrated to R2 and agents accessing content via MCP.

**Reference**: See `spec-v3-final.md` for complete functional requirements.

---

## Hour 0-1.5: Project Setup & Dependencies

### Tasks

**0.1 Create Project Structure** (15 min)
```bash
mkdir directory-mcp-server
cd directory-mcp-server

# Create directory structure
mkdir -p src/{storage,search,audit}
mkdir -p scripts
mkdir -p tests/{storage,search,audit}
mkdir -p .github/workflows
touch src/__init__.py
touch src/storage/__init__.py
touch src/search/__init__.py
touch src/audit/__init__.py
```

**0.2 Initialize Python Project** (15 min)
```bash
# Create pyproject.toml
cat > pyproject.toml << 'EOF'
[project]
name = "directory-mcp-server"
version = "1.0.0"
description = "MCP server for directory-based content access"
requires-python = ">=3.12"
dependencies = [
    "mcp>=0.9.0",
    "opendal>=0.46.0",
]

[project.optional-dependencies]
dev = [
    "pytest>=7.4.0",
    "pytest-asyncio>=0.21.0",
    "black>=23.0.0",
    "ruff>=0.1.0",
]
EOF

# Create uv.lock (if using uv) or requirements.txt
uv sync  # or: pip install -e ".[dev]"
```

**0.3 Create Configuration** (15 min)
```bash
# config.example.json (template)
cat > config.example.json << 'EOF'
{
  "storage": {
    "backend": "local",
    "local": {
      "root": "./content"
    },
    "r2": {
      "bucket": "panaversity-book",
      "endpoint": "https://<account-id>.r2.cloudflarestorage.com",
      "region": "auto",
      "access_key_id": "${R2_ACCESS_KEY_ID}",
      "secret_access_key": "${R2_SECRET_ACCESS_KEY}"
    }
  },
  "audit": {
    "log_path": "./audit.jsonl"
  }
}
EOF

# config.json (local dev)
cat > config.json << 'EOF'
{
  "storage": {
    "backend": "local",
    "local": {
      "root": "./content"
    }
  },
  "audit": {
    "log_path": "./audit.jsonl"
  }
}
EOF

# .env.example
cat > .env.example << 'EOF'
R2_ACCESS_KEY_ID=your_access_key_here
R2_SECRET_ACCESS_KEY=your_secret_key_here
EOF

# .gitignore
cat > .gitignore << 'EOF'
__pycache__/
*.pyc
.env
config.json
audit.jsonl
content/
.pytest_cache/
.ruff_cache/
dist/
build/
EOF
```

**0.4 Create README** (15 min)
```bash
cat > README.md << 'EOF'
# Directory MCP Server

Production MCP server for directory-based content access with pluggable storage backends.

## Quick Start

### Development (Local Storage)
```bash
# Install dependencies
uv sync

# Start server
python -m src.server

# Test with MCP inspector
mcp dev src/server.py
```

### Production (R2 Storage)
```bash
# Set environment variables
export R2_ACCESS_KEY_ID=xxx
export R2_SECRET_ACCESS_KEY=xxx

# Update config.json to use "r2" backend
# Start server
python -m src.server
```

## Tools

- `read_content(path)` - Read file
- `write_content(path, content, agent_id)` - Write file
- `list_contents(prefix)` - List directory
- `delete_content(path, agent_id)` - Delete file
- `glob_search(pattern)` - Find files by pattern
- `grep_search(pattern, path, context_lines, is_regex)` - Search contents
- `get_audit_log(operation, agent_id, since, limit)` - Query history

## Architecture

```
MCP Server (Python + FastMCP)
    ↓
OpenDAL Storage Abstraction
    ↓
Local (dev) / R2 (prod)
```

See `spec-v3-final.md` for complete specification.
EOF
```

**0.5 Acceptance Criteria**
- [ ] Directory structure created
- [ ] Python project initialized with dependencies
- [ ] Configuration files created
- [ ] README documents quick start
- [ ] Can run `uv sync` successfully

---

## Hour 1.5-2.5: Storage Abstraction Layer

### Tasks

**1.1 Create Storage Interface** (20 min)

File: `src/storage/backend.py`
```python
"""Storage backend abstraction using OpenDAL."""
import json
import os
from typing import Optional
from opendal import Operator


class StorageBackend:
    """Unified storage interface using OpenDAL."""

    def __init__(self, config: dict):
        """Initialize storage backend from config.

        Args:
            config: Configuration dict with storage.backend and storage.<backend> keys
        """
        self.config = config
        backend_type = config["storage"]["backend"]

        if backend_type == "local":
            self._init_local(config["storage"]["local"])
        elif backend_type == "r2":
            self._init_r2(config["storage"]["r2"])
        else:
            raise ValueError(f"Unknown backend: {backend_type}")

    def _init_local(self, config: dict):
        """Initialize local filesystem backend."""
        root = config["root"]
        os.makedirs(root, exist_ok=True)
        self.operator = Operator("fs", root=root)

    def _init_r2(self, config: dict):
        """Initialize Cloudflare R2 backend."""
        # Expand environment variables
        access_key = os.path.expandvars(config["access_key_id"])
        secret_key = os.path.expandvars(config["secret_access_key"])

        self.operator = Operator(
            "s3",
            bucket=config["bucket"],
            endpoint=config["endpoint"],
            region=config["region"],
            access_key_id=access_key,
            secret_access_key=secret_key,
        )

    async def read(self, path: str) -> str:
        """Read file content."""
        content_bytes = await self.operator.read(path)
        return content_bytes.decode("utf-8")

    async def write(self, path: str, content: str):
        """Write file content."""
        await self.operator.write(path, content.encode("utf-8"))

    async def list(self, prefix: str = "") -> list[str]:
        """List all files under prefix."""
        entries = await self.operator.list(prefix)
        return [entry.path for entry in entries if not entry.path.endswith("/")]

    async def exists(self, path: str) -> bool:
        """Check if file exists."""
        try:
            await self.operator.stat(path)
            return True
        except Exception:
            return False

    async def delete(self, path: str):
        """Delete file."""
        await self.operator.delete(path)


def load_config(config_path: str = "config.json") -> dict:
    """Load configuration from JSON file."""
    with open(config_path) as f:
        return json.load(f)
```

**1.2 Write Storage Tests** (20 min)

File: `tests/storage/test_backend.py`
```python
"""Tests for storage backend."""
import pytest
import tempfile
import os
from src.storage.backend import StorageBackend, load_config


@pytest.fixture
def local_config():
    """Create temporary local storage config."""
    tmpdir = tempfile.mkdtemp()
    return {
        "storage": {
            "backend": "local",
            "local": {"root": tmpdir}
        }
    }


@pytest.mark.asyncio
async def test_write_and_read(local_config):
    """Test writing and reading content."""
    storage = StorageBackend(local_config)

    await storage.write("test.txt", "Hello, World!")
    content = await storage.read("test.txt")

    assert content == "Hello, World!"


@pytest.mark.asyncio
async def test_list_files(local_config):
    """Test listing files."""
    storage = StorageBackend(local_config)

    await storage.write("file1.txt", "content1")
    await storage.write("dir/file2.txt", "content2")

    files = await storage.list("")
    assert "file1.txt" in files
    assert "dir/file2.txt" in files


@pytest.mark.asyncio
async def test_exists(local_config):
    """Test file existence check."""
    storage = StorageBackend(local_config)

    await storage.write("exists.txt", "content")

    assert await storage.exists("exists.txt")
    assert not await storage.exists("not-exists.txt")


@pytest.mark.asyncio
async def test_delete(local_config):
    """Test file deletion."""
    storage = StorageBackend(local_config)

    await storage.write("delete-me.txt", "content")
    assert await storage.exists("delete-me.txt")

    await storage.delete("delete-me.txt")
    assert not await storage.exists("delete-me.txt")
```

**1.3 Run Tests** (10 min)
```bash
pytest tests/storage/ -v
```

**1.4 Acceptance Criteria**
- [ ] StorageBackend class implements read/write/list/exists/delete
- [ ] Supports both local and R2 backends via config
- [ ] All tests pass
- [ ] Can switch backends by changing config.json only

---

## Hour 2.5-3.5: Audit Logger

### Tasks

**2.1 Create Audit Logger** (30 min)

File: `src/audit/logger.py`
```python
"""Audit logger for tracking all operations."""
import json
from datetime import datetime
from pathlib import Path
from typing import Optional


class AuditLogger:
    """Append-only JSONL audit log."""

    def __init__(self, log_path: str):
        """Initialize audit logger.

        Args:
            log_path: Path to JSONL log file
        """
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)

        # Create file if it doesn't exist
        if not self.log_path.exists():
            self.log_path.touch()

    def log(
        self,
        operation: str,
        path: str,
        agent_id: str,
        metadata: Optional[dict] = None
    ):
        """Log an operation.

        Args:
            operation: Operation type (read, write, delete, etc.)
            path: File path operated on
            agent_id: Agent performing operation
            metadata: Optional additional metadata
        """
        entry = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "operation": operation,
            "path": path,
            "agent_id": agent_id,
            "metadata": metadata or {}
        }

        with open(self.log_path, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def query(
        self,
        operation: Optional[str] = None,
        agent_id: Optional[str] = None,
        since: Optional[str] = None,
        limit: int = 100
    ) -> list[dict]:
        """Query audit log with filters.

        Args:
            operation: Filter by operation type
            agent_id: Filter by agent ID
            since: Filter by timestamp (ISO format)
            limit: Maximum number of entries to return

        Returns:
            List of matching log entries
        """
        if not self.log_path.exists():
            return []

        results = []

        with open(self.log_path) as f:
            for line in f:
                if not line.strip():
                    continue

                entry = json.loads(line)

                # Apply filters
                if operation and entry["operation"] != operation:
                    continue
                if agent_id and entry["agent_id"] != agent_id:
                    continue
                if since and entry["timestamp"] < since:
                    continue

                results.append(entry)

                if len(results) >= limit:
                    break

        # Return most recent first
        return list(reversed(results[-limit:]))
```

**2.2 Write Audit Tests** (20 min)

File: `tests/audit/test_logger.py`
```python
"""Tests for audit logger."""
import pytest
import tempfile
from pathlib import Path
from src.audit.logger import AuditLogger


@pytest.fixture
def logger():
    """Create temporary audit logger."""
    tmpfile = tempfile.NamedTemporaryFile(delete=False, suffix=".jsonl")
    return AuditLogger(tmpfile.name)


def test_log_operation(logger):
    """Test logging an operation."""
    logger.log("write", "test.txt", "claude", {"size": 100})

    entries = logger.query()
    assert len(entries) == 1
    assert entries[0]["operation"] == "write"
    assert entries[0]["path"] == "test.txt"
    assert entries[0]["agent_id"] == "claude"
    assert entries[0]["metadata"]["size"] == 100


def test_query_by_operation(logger):
    """Test filtering by operation."""
    logger.log("write", "file1.txt", "claude")
    logger.log("read", "file1.txt", "claude")
    logger.log("write", "file2.txt", "gpt")

    write_ops = logger.query(operation="write")
    assert len(write_ops) == 2
    assert all(e["operation"] == "write" for e in write_ops)


def test_query_by_agent(logger):
    """Test filtering by agent_id."""
    logger.log("write", "file1.txt", "claude")
    logger.log("write", "file2.txt", "gpt")
    logger.log("read", "file1.txt", "claude")

    claude_ops = logger.query(agent_id="claude")
    assert len(claude_ops) == 2
    assert all(e["agent_id"] == "claude" for e in claude_ops)


def test_query_limit(logger):
    """Test result limit."""
    for i in range(10):
        logger.log("write", f"file{i}.txt", "claude")

    entries = logger.query(limit=5)
    assert len(entries) == 5
```

**2.3 Run Tests** (10 min)
```bash
pytest tests/audit/ -v
```

**2.4 Acceptance Criteria**
- [ ] AuditLogger logs operations to JSONL
- [ ] Supports filtering by operation, agent_id, timestamp
- [ ] Returns most recent entries first
- [ ] All tests pass

---

## Hour 3.5-5: Search Tools (Glob + Grep)

### Tasks

**3.1 Implement Glob Search** (20 min)

File: `src/search/glob.py`
```python
"""Glob search for finding files by pattern."""
import fnmatch
from typing import List
from src.storage.backend import StorageBackend


class GlobSearcher:
    """Find files matching glob patterns."""

    def __init__(self, storage: StorageBackend):
        """Initialize glob searcher.

        Args:
            storage: Storage backend to search
        """
        self.storage = storage

    async def search(self, pattern: str) -> List[str]:
        """Find files matching glob pattern.

        Args:
            pattern: Glob pattern (e.g., "**/*.md", "**/chapter-5/**")

        Returns:
            List of matching file paths
        """
        all_files = await self.storage.list("")
        matched = [f for f in all_files if fnmatch.fnmatch(f, pattern)]
        return sorted(matched)
```

**3.2 Implement Grep Search** (40 min)

File: `src/search/grep.py`
```python
"""Grep search for searching file contents."""
import re
from typing import List, Optional
from src.storage.backend import StorageBackend


class GrepMatch:
    """Single grep match result."""

    def __init__(
        self,
        file: str,
        line_number: int,
        line: str,
        context_before: List[str],
        context_after: List[str]
    ):
        self.file = file
        self.line_number = line_number
        self.line = line
        self.context_before = context_before
        self.context_after = context_after

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "file": self.file,
            "lineNumber": self.line_number,
            "line": self.line,
            "contextBefore": self.context_before,
            "contextAfter": self.context_after
        }


class GrepSearcher:
    """Search file contents with context lines."""

    def __init__(self, storage: StorageBackend):
        """Initialize grep searcher.

        Args:
            storage: Storage backend to search
        """
        self.storage = storage

    async def search(
        self,
        pattern: str,
        path: str = "",
        context_lines: int = 2,
        is_regex: bool = False
    ) -> List[GrepMatch]:
        """Search file contents for pattern.

        Args:
            pattern: Text or regex pattern to search
            path: Directory prefix to search within
            context_lines: Number of lines before/after to include
            is_regex: Whether pattern is regex

        Returns:
            List of GrepMatch objects
        """
        results = []

        # Get files to search
        files = await self.storage.list(path)
        md_files = [f for f in files if f.endswith(".md")]

        # Compile regex
        if is_regex:
            regex = re.compile(pattern, re.IGNORECASE)
        else:
            # Escape special chars for literal match
            escaped = re.escape(pattern)
            regex = re.compile(escaped, re.IGNORECASE)

        # Search each file
        for file in md_files:
            try:
                content = await self.storage.read(file)
                lines = content.split("\n")

                for i, line in enumerate(lines):
                    if regex.search(line):
                        # Extract context
                        start = max(0, i - context_lines)
                        end = min(len(lines), i + 1 + context_lines)

                        context_before = lines[start:i]
                        context_after = lines[i+1:end]

                        match = GrepMatch(
                            file=file,
                            line_number=i + 1,
                            line=line.strip(),
                            context_before=context_before,
                            context_after=context_after
                        )
                        results.append(match)
            except Exception:
                # Skip files that can't be read
                continue

        return results
```

**3.3 Write Search Tests** (30 min)

File: `tests/search/test_glob.py`
```python
"""Tests for glob search."""
import pytest
from src.search.glob import GlobSearcher
from src.storage.backend import StorageBackend


@pytest.fixture
async def storage_with_files():
    """Create storage with test files."""
    config = {
        "storage": {
            "backend": "local",
            "local": {"root": "./test-content"}
        }
    }
    storage = StorageBackend(config)

    # Create test files
    await storage.write("01-Part/01-chapter/01-lesson.md", "content")
    await storage.write("01-Part/01-chapter/02-lesson.md", "content")
    await storage.write("01-Part/02-chapter/01-lesson.md", "content")
    await storage.write("02-Part/01-chapter/01-lesson.md", "content")
    await storage.write("README.md", "content")

    return storage


@pytest.mark.asyncio
async def test_glob_all_markdown(storage_with_files):
    """Test finding all markdown files."""
    searcher = GlobSearcher(storage_with_files)
    results = await searcher.search("**/*.md")
    assert len(results) == 5


@pytest.mark.asyncio
async def test_glob_specific_chapter(storage_with_files):
    """Test finding specific chapter."""
    searcher = GlobSearcher(storage_with_files)
    results = await searcher.search("**/01-chapter/**")
    assert len(results) == 3
    assert all("01-chapter" in r for r in results)
```

File: `tests/search/test_grep.py`
```python
"""Tests for grep search."""
import pytest
from src.search.grep import GrepSearcher
from src.storage.backend import StorageBackend


@pytest.fixture
async def storage_with_content():
    """Create storage with searchable content."""
    config = {
        "storage": {
            "backend": "local",
            "local": {"root": "./test-content"}
        }
    }
    storage = StorageBackend(config)

    # Create files with searchable content
    await storage.write("test1.md", """
# Chapter 1

This lesson covers async/await patterns.

## Example

Use async/await for better code.
""")

    await storage.write("test2.md", """
# Chapter 2

Learn about Python loops.
""")

    return storage


@pytest.mark.asyncio
async def test_grep_literal_search(storage_with_content):
    """Test literal text search."""
    searcher = GrepSearcher(storage_with_content)
    results = await searcher.search("async/await", "", 1)

    assert len(results) >= 1
    assert "async/await" in results[0].line.lower()
    assert len(results[0].context_before) <= 1
    assert len(results[0].context_after) <= 1


@pytest.mark.asyncio
async def test_grep_regex_search(storage_with_content):
    """Test regex search."""
    searcher = GrepSearcher(storage_with_content)
    results = await searcher.search("async.*await", "", 2, is_regex=True)

    assert len(results) >= 1
```

**3.4 Run Tests** (10 min)
```bash
pytest tests/search/ -v
```

**3.5 Acceptance Criteria**
- [ ] GlobSearcher finds files by pattern
- [ ] GrepSearcher searches contents with context
- [ ] Supports both literal and regex patterns
- [ ] All tests pass

---

## Hour 5-6: MCP Server Implementation

### Tasks

**4.1 Implement MCP Server** (45 min)

File: `src/server.py`
```python
"""MCP server for directory-based content access."""
import asyncio
from typing import Optional
from mcp.server import Server
from mcp.types import TextContent, Tool
from src.storage.backend import StorageBackend, load_config
from src.audit.logger import AuditLogger
from src.search.glob import GlobSearcher
from src.search.grep import GrepSearcher

# Initialize components
config = load_config()
storage = StorageBackend(config)
audit = AuditLogger(config["audit"]["log_path"])
glob_searcher = GlobSearcher(storage)
grep_searcher = GrepSearcher(storage)

# Create MCP server
app = Server("directory-mcp-server")


@app.list_tools()
async def list_tools() -> list[Tool]:
    """List available MCP tools."""
    return [
        Tool(
            name="read_content",
            description="Read file content from storage",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {"type": "string", "description": "File path to read"}
                },
                "required": ["path"]
            }
        ),
        Tool(
            name="write_content",
            description="Write file content to storage",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {"type": "string"},
                    "content": {"type": "string"},
                    "agent_id": {"type": "string"}
                },
                "required": ["path", "content", "agent_id"]
            }
        ),
        Tool(
            name="list_contents",
            description="List all files under a directory prefix",
            inputSchema={
                "type": "object",
                "properties": {
                    "prefix": {"type": "string", "default": ""}
                }
            }
        ),
        Tool(
            name="delete_content",
            description="Delete file from storage",
            inputSchema={
                "type": "object",
                "properties": {
                    "path": {"type": "string"},
                    "agent_id": {"type": "string"}
                },
                "required": ["path", "agent_id"]
            }
        ),
        Tool(
            name="glob_search",
            description="Find files matching glob pattern",
            inputSchema={
                "type": "object",
                "properties": {
                    "pattern": {"type": "string", "description": "Glob pattern (e.g., **/*.md)"}
                },
                "required": ["pattern"]
            }
        ),
        Tool(
            name="grep_search",
            description="Search file contents for text/regex with context",
            inputSchema={
                "type": "object",
                "properties": {
                    "pattern": {"type": "string"},
                    "path": {"type": "string", "default": ""},
                    "context_lines": {"type": "integer", "default": 2},
                    "is_regex": {"type": "boolean", "default": False}
                },
                "required": ["pattern"]
            }
        ),
        Tool(
            name="get_audit_log",
            description="Query operation history",
            inputSchema={
                "type": "object",
                "properties": {
                    "operation": {"type": "string"},
                    "agent_id": {"type": "string"},
                    "since": {"type": "string"},
                    "limit": {"type": "integer", "default": 100}
                }
            }
        ),
    ]


@app.call_tool()
async def call_tool(name: str, arguments: dict):
    """Handle tool calls."""

    if name == "read_content":
        path = arguments["path"]
        content = await storage.read(path)
        audit.log("read", path, "unknown")
        return [TextContent(type="text", text=content)]

    elif name == "write_content":
        path = arguments["path"]
        content = arguments["content"]
        agent_id = arguments["agent_id"]

        await storage.write(path, content)
        audit.log("write", path, agent_id, {"size": len(content)})
        return [TextContent(type="text", text=f"Written {len(content)} bytes to {path}")]

    elif name == "list_contents":
        prefix = arguments.get("prefix", "")
        files = await storage.list(prefix)
        result = "\n".join(files)
        return [TextContent(type="text", text=result)]

    elif name == "delete_content":
        path = arguments["path"]
        agent_id = arguments["agent_id"]

        await storage.delete(path)
        audit.log("delete", path, agent_id)
        return [TextContent(type="text", text=f"Deleted {path}")]

    elif name == "glob_search":
        pattern = arguments["pattern"]
        results = await glob_searcher.search(pattern)
        result = "\n".join(results)
        return [TextContent(type="text", text=result)]

    elif name == "grep_search":
        pattern = arguments["pattern"]
        path = arguments.get("path", "")
        context_lines = arguments.get("context_lines", 2)
        is_regex = arguments.get("is_regex", False)

        matches = await grep_searcher.search(pattern, path, context_lines, is_regex)

        # Format results
        result_lines = []
        for match in matches:
            result_lines.append(f"\n{match.file}:{match.line_number}")
            if match.context_before:
                result_lines.extend([f"  {line}" for line in match.context_before])
            result_lines.append(f"> {match.line}")
            if match.context_after:
                result_lines.extend([f"  {line}" for line in match.context_after])

        return [TextContent(type="text", text="\n".join(result_lines))]

    elif name == "get_audit_log":
        operation = arguments.get("operation")
        agent_id = arguments.get("agent_id")
        since = arguments.get("since")
        limit = arguments.get("limit", 100)

        entries = audit.query(operation, agent_id, since, limit)

        # Format as JSON lines
        import json
        result = "\n".join([json.dumps(e) for e in entries])
        return [TextContent(type="text", text=result)]

    else:
        raise ValueError(f"Unknown tool: {name}")


async def main():
    """Run the MCP server."""
    from mcp.server.stdio import stdio_server

    async with stdio_server() as (read_stream, write_stream):
        await app.run(
            read_stream,
            write_stream,
            app.create_initialization_options()
        )


if __name__ == "__main__":
    asyncio.run(main())
```

**4.2 Test MCP Server** (15 min)
```bash
# Start server
python -m src.server

# In another terminal, test with MCP inspector
mcp dev src/server.py

# Or test programmatically with MCP client
# (Create simple test client script)
```

**4.3 Acceptance Criteria**
- [ ] Server starts and responds to tool requests
- [ ] All 7 tools work correctly
- [ ] Audit logging captures operations
- [ ] Can test with MCP inspector

---

## Hour 6-7: Migration & Hydration Scripts

### Tasks

**5.1 Create Migration Script** (30 min)

File: `scripts/migrate.py`
```python
"""Migrate content from Git to R2."""
import asyncio
import sys
from pathlib import Path
from src.storage.backend import StorageBackend, load_config


async def migrate(
    source_dir: str,
    dry_run: bool = False
):
    """Migrate files from local directory to R2.

    Args:
        source_dir: Path to Git repository content (e.g., ./book-source/docs)
        dry_run: If True, only print what would be done
    """
    # Load R2 config
    config = load_config()
    if config["storage"]["backend"] != "r2":
        print("ERROR: config.json must use 'r2' backend for migration")
        sys.exit(1)

    storage = StorageBackend(config)

    # Find all .md files
    source_path = Path(source_dir)
    md_files = list(source_path.rglob("*.md"))

    print(f"Found {len(md_files)} markdown files in {source_dir}")

    if dry_run:
        print("\nDRY RUN - Would migrate:")
        for file in md_files[:10]:  # Show first 10
            relative = file.relative_to(source_path)
            print(f"  {relative}")
        if len(md_files) > 10:
            print(f"  ... and {len(md_files) - 10} more")
        return

    # Migrate each file
    print("\nMigrating...")
    for i, file in enumerate(md_files, 1):
        relative = file.relative_to(source_path)
        content = file.read_text()

        await storage.write(str(relative), content)
        print(f"  [{i}/{len(md_files)}] {relative}")

    print(f"\n✓ Migrated {len(md_files)} files to R2")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Migrate content to R2")
    parser.add_argument("source_dir", help="Source directory (e.g., ./book-source/docs)")
    parser.add_argument("--dry-run", action="store_true", help="Show what would be done")

    args = parser.parse_args()

    asyncio.run(migrate(args.source_dir, args.dry_run))
```

**5.2 Create Hydration Script** (30 min)

File: `scripts/hydrate.py`
```python
"""Hydrate Docusaurus temp directory from R2."""
import asyncio
import sys
from pathlib import Path
from src.storage.backend import StorageBackend, load_config


async def hydrate(
    target_dir: str = "./book-source/docs-temp"
):
    """Hydrate local directory from R2 for Docusaurus build.

    Args:
        target_dir: Target directory to write files
    """
    # Load R2 config
    config = load_config()
    if config["storage"]["backend"] != "r2":
        print("ERROR: config.json must use 'r2' backend for hydration")
        sys.exit(1)

    storage = StorageBackend(config)
    target = Path(target_dir)

    # Clear target directory
    if target.exists():
        import shutil
        shutil.rmtree(target)
    target.mkdir(parents=True)

    # List all files
    print("Fetching file list from R2...")
    files = await storage.list("")
    print(f"Found {len(files)} files")

    # Download each file
    print("\nHydrating...")
    for i, file_path in enumerate(files, 1):
        content = await storage.read(file_path)

        # Write to target
        local_path = target / file_path
        local_path.parent.mkdir(parents=True, exist_ok=True)
        local_path.write_text(content)

        print(f"  [{i}/{len(files)}] {file_path}")

    print(f"\n✓ Hydrated {len(files)} files to {target_dir}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Hydrate Docusaurus from R2")
    parser.add_argument("--target", default="./book-source/docs-temp", help="Target directory")

    args = parser.parse_args()

    asyncio.run(hydrate(args.target))
```

**5.3 Test Scripts** (10 min)
```bash
# Test migration (dry run)
python scripts/migrate.py ../book-source/docs --dry-run

# Test hydration (requires R2 credentials)
# python scripts/hydrate.py --target ./test-output
```

**5.4 Acceptance Criteria**
- [ ] Migration script uploads all .md files to R2
- [ ] Dry-run mode works correctly
- [ ] Hydration script downloads all files
- [ ] Preserves directory structure

---

## Hour 7-8: GitHub Actions & Deployment

### Tasks

**6.1 Create GitHub Actions Workflow** (20 min)

File: `.github/workflows/deploy.yml`
```yaml
name: Deploy Docusaurus

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  deploy:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.12'

      - name: Install MCP Server
        run: |
          cd directory-mcp-server
          pip install -e .

      - name: Configure R2 Storage
        run: |
          cd directory-mcp-server
          cat > config.json << EOF
          {
            "storage": {
              "backend": "r2",
              "r2": {
                "bucket": "panaversity-book",
                "endpoint": "https://${{ secrets.R2_ACCOUNT_ID }}.r2.cloudflarestorage.com",
                "region": "auto",
                "access_key_id": "${{ secrets.R2_ACCESS_KEY_ID }}",
                "secret_access_key": "${{ secrets.R2_SECRET_ACCESS_KEY }}"
              }
            },
            "audit": {
              "log_path": "./audit.jsonl"
            }
          }
          EOF

      - name: Hydrate Content from R2
        run: |
          cd directory-mcp-server
          python scripts/hydrate.py --target ../book-source/docs-temp

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '18'

      - name: Install Docusaurus Dependencies
        run: |
          cd book-source
          npm install

      - name: Build Docusaurus
        run: |
          cd book-source
          npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./book-source/build
```

**6.2 Create Deployment README** (10 min)

File: `DEPLOYMENT.md`
```markdown
# Deployment Guide

## Prerequisites

1. **Cloudflare R2 Bucket**:
   - Create bucket: `panaversity-book`
   - Generate R2 API tokens

2. **GitHub Secrets**:
   - `R2_ACCOUNT_ID`: Your Cloudflare account ID
   - `R2_ACCESS_KEY_ID`: R2 API token ID
   - `R2_SECRET_ACCESS_KEY`: R2 API token secret

## Initial Migration

### Step 1: Set Environment Variables
```bash
export R2_ACCESS_KEY_ID=xxx
export R2_SECRET_ACCESS_KEY=xxx
```

### Step 2: Update Config for R2
```bash
cd directory-mcp-server
cat > config.json << EOF
{
  "storage": {
    "backend": "r2",
    "r2": {
      "bucket": "panaversity-book",
      "endpoint": "https://YOUR_ACCOUNT_ID.r2.cloudflarestorage.com",
      "region": "auto",
      "access_key_id": "${R2_ACCESS_KEY_ID}",
      "secret_access_key": "${R2_SECRET_ACCESS_KEY}"
    }
  },
  "audit": {
    "log_path": "./audit.jsonl"
  }
}
EOF
```

### Step 3: Dry Run Migration
```bash
python scripts/migrate.py ../book-source/docs --dry-run
```

### Step 4: Execute Migration
```bash
python scripts/migrate.py ../book-source/docs
```

### Step 5: Verify
```bash
# Start MCP server
python -m src.server

# Test with MCP inspector or client
mcp dev src/server.py
```

## Continuous Deployment

Once GitHub Actions is configured, every push to `main` will:
1. Hydrate content from R2
2. Build Docusaurus
3. Deploy to GitHub Pages

## Rollback Plan

If issues occur:

1. **Revert to Git-based workflow**:
   ```bash
   cd book-source
   # Remove symlink
   rm docs
   # Restore original docs directory
   git checkout docs
   ```

2. **Content is safe**: Original Git repository remains unchanged

3. **R2 backup**: All content is in R2 with audit log

## Monitoring

Check audit log:
```bash
python -c "from src.audit.logger import AuditLogger; import json; logger = AuditLogger('audit.jsonl'); [print(json.dumps(e)) for e in logger.query(limit=10)]"
```
```

**6.3 Update Main README** (10 min)

Update `directory-mcp-server/README.md` with deployment section.

**6.4 Create Production Checklist** (10 min)

File: `PRODUCTION-CHECKLIST.md`
```markdown
# Production Deployment Checklist

## Pre-Deployment

- [ ] R2 bucket created (`panaversity-book`)
- [ ] R2 API tokens generated
- [ ] GitHub secrets configured
- [ ] Local migration tested (dry-run)
- [ ] Migration completed successfully
- [ ] MCP server tested with R2 backend
- [ ] Hydration script tested

## Deployment

- [ ] GitHub Actions workflow pushed to repo
- [ ] Workflow runs successfully
- [ ] Docusaurus site builds correctly
- [ ] GitHub Pages deployment successful
- [ ] Website accessible and renders correctly

## Post-Deployment

- [ ] All 84 chapters accessible
- [ ] Search functionality works
- [ ] No broken links
- [ ] Audit log captures operations
- [ ] MCP tools respond correctly

## Success Criteria

- [ ] MCP server responds in < 100ms
- [ ] All chapters migrated without data loss
- [ ] Docusaurus build completes successfully
- [ ] Website deploys and renders correctly
- [ ] Audit log captures all operations
- [ ] Agents can read/write via MCP
```

**6.5 Final Testing** (10 min)
```bash
# Run full test suite
pytest tests/ -v

# Verify all scripts work
python scripts/migrate.py --help
python scripts/hydrate.py --help

# Test MCP server
python -m src.server &
# (Test with MCP client)
```

**6.6 Acceptance Criteria**
- [ ] GitHub Actions workflow configured
- [ ] Deployment documentation complete
- [ ] Production checklist created
- [ ] All tests pass
- [ ] Ready to deploy

---

## Post-Hour 8: Production Deployment

### Deployment Steps

**Step 1: Configure GitHub Secrets** (5 min)
- Add `R2_ACCOUNT_ID`
- Add `R2_ACCESS_KEY_ID`
- Add `R2_SECRET_ACCESS_KEY`

**Step 2: Initial Migration** (15 min)
```bash
# Set environment variables
export R2_ACCESS_KEY_ID=xxx
export R2_SECRET_ACCESS_KEY=xxx

# Update config.json for R2
# Run migration
python scripts/migrate.py ../book-source/docs

# Verify
python -m src.server
```

**Step 3: Push to GitHub** (5 min)
```bash
git add .
git commit -m "Add Directory MCP Server"
git push origin main
```

**Step 4: Monitor Deployment** (5 min)
- Watch GitHub Actions workflow
- Verify Docusaurus build
- Check deployed site

**Step 5: Verify Production** (10 min)
- [ ] All chapters accessible
- [ ] No 404 errors
- [ ] Audit log working
- [ ] MCP server responding

**Total Deployment Time**: 40 minutes

---

## Summary

**Timeline**:
- **Hour 0-1.5**: Project setup ✓
- **Hour 1.5-2.5**: Storage layer ✓
- **Hour 2.5-3.5**: Audit logger ✓
- **Hour 3.5-5**: Search tools ✓
- **Hour 5-6**: MCP server ✓
- **Hour 6-7**: Scripts ✓
- **Hour 7-8**: GitHub Actions ✓
- **Post-Hour 8**: Production deployment (40 min)

**Total**: 8 hours + 40 min = **8.7 hours to live in production**

**Deliverables**:
- ✓ 7 MCP tools (read, write, list, delete, glob, grep, audit)
- ✓ Storage abstraction (local + R2)
- ✓ Migration and hydration scripts
- ✓ GitHub Actions integration
- ✓ Complete test suite
- ✓ Deployment documentation

**LOC**: ~280 lines of production code

**Risk**: LOW (proven tech stack, comprehensive tests, rollback plan)

---

## Next Steps

1. **Execute this plan**: Follow hour-by-hour breakdown
2. **Run tests frequently**: After each phase
3. **Document issues**: Note any deviations from plan
4. **Deploy to production**: Follow Post-Hour 8 steps
5. **Monitor**: Check audit logs and website

---

**Status**: ✅ READY FOR EXECUTION

**Start Time**: When approved by user

**Expected Completion**: 8 hours from start
