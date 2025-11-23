Perfect - now I have the full context. Before I dive into comprehensive research, let me clarify a few things to make this maximally useful:

1. **OpenDAL depth** - Should I focus heavily on OpenDAL patterns (Python binding specifics, R2 configuration, error handling, streaming large files) or just enough to implement?

2. **MCP server architecture** - Do you want research on building production MCP servers (auth, rate limiting, caching, multiple tools) or minimal viable implementation patterns?

3. **Docusaurus injection** - Should I research multiple approaches (plugin-based, CLI sync, runtime fetch) with trade-offs, or focus specifically on build-time sync from R2?

This will help me target the research toward building vs. understanding trade-offs.

Why do we need a tradational REST API when same thing is offered by MCP server with their Tools Functionaly

You're absolutely right. For an **AI-native system**, REST is redundant. Let me rethink:

## Who Actually Consumes the Content?

| Consumer | Needs REST? | Better Alternative |
|----------|-------------|-------------------|
| AI Agents (Claude, GPT, etc.) | ❌ | **MCP Tools** |
| Docusaurus (build time) | ❌ | **Direct OpenDAL sync** (CLI/script) |
| Other AI Agents | ❌ | **MCP Tools** |
| Future AI systems | ❌ | **MCP Tools** |

## Simplified Architecture (No REST)

```
┌─────────────────────────────────────────────────────────┐
│                     CONSUMERS                            │
├──────────────────────────┬──────────────────────────────┤
│      AI AGENTS           │         DOCUSAURUS           │
│   (Claude, GPT, etc.)    │        (Build Time)          │
│                          │                              │
│    ┌──────────────┐      │      ┌──────────────┐       │
│    │  MCP Client  │      │      │  CLI Script  │       │
│    └──────┬───────┘      │      └──────┬───────┘       │
│           │              │             │               │
└───────────┼──────────────┴─────────────┼───────────────┘
            │                            │
            ▼                            │
   ┌────────────────┐                    │
   │   MCP SERVER   │                    │
   │                │                    │
   │  Tools:        │                    │
   │  - read_lesson │                    │
   │  - write_lesson│                    │
   │  - list_lessons│                    │
   │  - search      │                    │
   └────────┬───────┘                    │
            │                            │
            └──────────────┬─────────────┘
                           │
                  ┌────────▼────────┐
                  │    OpenDAL      │
                  │  (Storage API)  │
                  └────────┬────────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
           Local          R2           S3
```

## What We Actually Need

```python
# 1. Storage Layer (OpenDAL) - shared by everything
class BookStorage:
    def __init__(self, backend="fs", **config):
        self.op = opendal.Operator(backend, **config)
    
    def read(self, path): return self.op.read(path)
    def write(self, path, content): self.op.write(path, content)
    def list(self, prefix): return list(self.op.scan(prefix))


# 2. MCP Server - for AI agents
from mcp import Server

server = Server("book-content")
storage = BookStorage(...)

@server.tool("read_lesson")
def read_lesson(book: str, part: str, chapter: str, lesson: str) -> str:
    path = f"books/{book}/{part}/{chapter}/{lesson}.md"
    return storage.read(path).decode()

@server.tool("write_lesson")
def write_lesson(book: str, part: str, chapter: str, lesson: str, content: str):
    path = f"books/{book}/{part}/{chapter}/{lesson}.md"
    storage.write(path, content.encode())

@server.tool("list_chapters")
def list_chapters(book: str, part: str) -> list[str]:
    prefix = f"books/{book}/{part}/"
    return [e.path for e in storage.list(prefix) if e.path.startswith("chapter-")]


# 3. CLI for Docusaurus sync - just uses storage directly
# sync.py
import click

@click.command()
@click.option("--to", default="./docs")
def sync(to: str):
    """Sync book content to local for Docusaurus build"""
    storage = BookStorage("s3", bucket="book-content", ...)
    
    for entry in storage.list("books/"):
        content = storage.read(entry.path)
        local_path = Path(to) / entry.path
        local_path.parent.mkdir(parents=True, exist_ok=True)
        local_path.write_bytes(content)
        print(f"Synced: {entry.path}")

# Usage: python sync.py --to ./docs
```

## When Would You Need REST?

Only if you have **non-AI consumers** that can't use MCP:

- Browser-based human editors (but you could use a CMS instead)
- Mobile apps (but do you need one?)
- Third-party webhooks (but MCP might support this soon)
- Legacy systems (unlikely for a new project)

## Final Simplified Stack

```
book-content-system/
├── src/
│   ├── storage.py          # OpenDAL wrapper (shared)
│   ├── mcp_server.py       # MCP for AI agents
│   └── cli.py              # CLI for Docusaurus sync
├── pyproject.toml
└── README.md
```

**That's it.** No REST. No GraphQL. Just:
1. **OpenDAL** for storage abstraction
2. **MCP Server** for AI agents  
3. **CLI** for build-time sync

You're thinking AI-native. REST is legacy thinking. 🎯

Want me to sketch the minimal implementation?

so what  is the proposal use opendal as core of AgentFS to get the best of both systems. This builds unified system where educational content flows seamlessly to AI agents, static documentation sites, and any consumer through a backend-agnostic API. building a truly reusable, extensible filesystem abstraction that works across local storage, R2, S3, and any cloud bucket while optimizing for AI agent consumption. First thing is for book that have parts, lessons and chapters and summar of lessons and it is updated and read

# Proposal: PanaversityFS - Agent-Native Book Storage

## Core Idea

**OpenDAL (storage abstraction) + AgentFS patterns (audit, agent-native) = PanaversityFS**

```
┌─────────────────────────────────────────────────────────────┐
│                      PanaversityFS                           │
│         "One Layer, All Storage, Agent-Native"               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   AgentFS Patterns          OpenDAL Core                     │
│   ┌─────────────────┐      ┌─────────────────┐              │
│   │ • Audit trails  │      │ • Rust speed    │              │
│   │ • Tool tracking │  +   │ • Python/TS/Rust│              │
│   │ • Agent-ready   │      │ • R2/S3/Local   │              │
│   │ • Queryable     │      │ • Battle-tested │              │
│   └─────────────────┘      └─────────────────┘              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Book Data Model

```
books/
└── colearning-python/
    ├── book.md                          # Book intro + metadata in frontmatter
    │
    ├── part-01-foundations/
    │   ├── part.md                      # Part intro
    │   │
    │   ├── chapter-01-introduction/
    │   │   ├── chapter.md               # Chapter intro
    │   │   ├── lesson-01-what-is-python.md
    │   │   ├── lesson-01-what-is-python.summary.md   # AI-generated summary
    │   │   ├── lesson-02-setup.md
    │   │   ├── lesson-02-setup.summary.md
    │   │   └── images/
    │   │       └── setup-screenshot.png
    │   │
    │   └── chapter-02-variables/
    │       ├── chapter.md
    │       ├── lesson-01-data-types.md
    │       ├── lesson-01-data-types.summary.md
    │       └── images/
    │
    └── part-02-intermediate/
        └── ...
```

## Lesson File Format

```markdown
<!-- lesson-01-what-is-python.md -->
---
title: What is Python?
order: 1
estimated_minutes: 15
difficulty: beginner
video: https://youtube.com/...
slides: https://docs.google.com/presentation/d/...
updated_at: 2025-01-15T10:30:00Z
updated_by: claude-agent
---

# What is Python?

Python is a versatile programming language...

![Python Logo](./images/python-logo.png)
```

```markdown
<!-- lesson-01-what-is-python.summary.md -->
---
generated_at: 2025-01-15T10:35:00Z
generated_by: claude-agent
source_hash: abc123...
---

Python is a high-level, interpreted language known for readability...
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                       CONSUMERS                              │
├────────────────────────┬────────────────────────────────────┤
│      AI AGENTS         │           DOCUSAURUS               │
│  (Read + Write)        │          (Read Only)               │
│                        │                                    │
│  • Write lessons       │  • Build static site               │
│  • Generate summaries  │  • Serve to humans                 │
│  • Update content      │                                    │
└───────────┬────────────┴──────────────────┬─────────────────┘
            │                               │
            ▼                               │
    ┌───────────────┐                       │
    │  MCP SERVER   │                       │
    │               │                       │
    │  Tools:       │                       │
    │  read_lesson  │                       │
    │  write_lesson │                       │
    │  get_summary  │                       │
    │  update_summary                       │
    │  list_structure                       │
    │  get_audit_log│                       │
    └───────┬───────┘                       │
            │                               │
            └───────────────┬───────────────┘
                            │
                   ┌────────▼────────┐
                   │  PanaversityFS  │
                   │                 │
                   │  • Book ops     │
                   │  • Audit trail  │
                   │  • Path logic   │
                   └────────┬────────┘
                            │
                   ┌────────▼────────┐
                   │    OpenDAL      │
                   │  (Rust Core)    │
                   └────────┬────────┘
                            │
            ┌───────────────┼───────────────┐
            │               │               │
         Local             R2              S3
          Dev            Prod           Backup
```

## Core Implementation

### 1. PanaversityFS - The Book-Aware Layer

```python
# panaversity_fs/core.py
import opendal
import json
from datetime import datetime
from pathlib import PurePosixPath
from typing import Optional
from dataclasses import dataclass

@dataclass
class AuditEntry:
    timestamp: str
    operation: str  # read, write, delete
    path: str
    agent_id: str
    details: Optional[dict] = None

class PanaversityFS:
    """Agent-native filesystem for educational books."""
    
    def __init__(self, backend: str = "fs", **config):
        self.op = opendal.Operator(backend, **config)
        self._audit_enabled = True
    
    # ============ BOOK OPERATIONS ============
    
    def read_lesson(self, book: str, part: str, chapter: str, lesson: str) -> str:
        """Read a lesson's markdown content."""
        path = f"books/{book}/{part}/{chapter}/{lesson}.md"
        content = self.op.read(path).decode('utf-8')
        self._audit("read", path, "system")
        return content
    
    def write_lesson(self, book: str, part: str, chapter: str, lesson: str, 
                     content: str, agent_id: str = "unknown") -> None:
        """Write/update a lesson with audit trail."""
        path = f"books/{book}/{part}/{chapter}/{lesson}.md"
        
        # Add/update frontmatter metadata
        content = self._update_frontmatter(content, {
            "updated_at": datetime.utcnow().isoformat() + "Z",
            "updated_by": agent_id
        })
        
        self.op.write(path, content.encode('utf-8'))
        self._audit("write", path, agent_id)
    
    def get_summary(self, book: str, part: str, chapter: str, lesson: str) -> Optional[str]:
        """Get lesson summary if it exists."""
        path = f"books/{book}/{part}/{chapter}/{lesson}.summary.md"
        try:
            return self.op.read(path).decode('utf-8')
        except:
            return None
    
    def write_summary(self, book: str, part: str, chapter: str, lesson: str,
                      summary: str, agent_id: str = "unknown") -> None:
        """Write/update lesson summary."""
        lesson_path = f"books/{book}/{part}/{chapter}/{lesson}.md"
        summary_path = f"books/{book}/{part}/{chapter}/{lesson}.summary.md"
        
        # Get source hash for tracking
        lesson_content = self.op.read(lesson_path)
        source_hash = self._hash(lesson_content)
        
        # Add metadata
        summary = self._update_frontmatter(summary, {
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "generated_by": agent_id,
            "source_hash": source_hash
        })
        
        self.op.write(summary_path, summary.encode('utf-8'))
        self._audit("write_summary", summary_path, agent_id, {"source_hash": source_hash})
    
    # ============ STRUCTURE OPERATIONS ============
    
    def get_book_structure(self, book: str) -> dict:
        """Get complete book structure (for Docusaurus sidebar)."""
        structure = {"book": book, "parts": []}
        
        for entry in self.op.list(f"books/{book}/"):
            if entry.path.startswith("part-") and entry.path.endswith("/"):
                part = self._parse_part(book, entry.path.rstrip("/"))
                structure["parts"].append(part)
        
        structure["parts"].sort(key=lambda x: x["order"])
        return structure
    
    def _parse_part(self, book: str, part: str) -> dict:
        """Parse a part's structure."""
        part_data = {"id": part, "order": self._extract_order(part), "chapters": []}
        
        for entry in self.op.list(f"books/{book}/{part}/"):
            if entry.path.startswith("chapter-") and entry.path.endswith("/"):
                chapter = self._parse_chapter(book, part, entry.path.rstrip("/"))
                part_data["chapters"].append(chapter)
        
        part_data["chapters"].sort(key=lambda x: x["order"])
        return part_data
    
    def _parse_chapter(self, book: str, part: str, chapter: str) -> dict:
        """Parse a chapter's lessons."""
        chapter_data = {"id": chapter, "order": self._extract_order(chapter), "lessons": []}
        
        for entry in self.op.list(f"books/{book}/{part}/{chapter}/"):
            if entry.path.startswith("lesson-") and entry.path.endswith(".md"):
                if not entry.path.endswith(".summary.md"):
                    lesson_id = entry.path.replace(".md", "")
                    has_summary = self._exists(f"books/{book}/{part}/{chapter}/{lesson_id}.summary.md")
                    chapter_data["lessons"].append({
                        "id": lesson_id,
                        "order": self._extract_order(lesson_id),
                        "has_summary": has_summary
                    })
        
        chapter_data["lessons"].sort(key=lambda x: x["order"])
        return chapter_data
    
    # ============ AUDIT OPERATIONS ============
    
    def _audit(self, operation: str, path: str, agent_id: str, details: dict = None):
        """Record operation to audit log."""
        if not self._audit_enabled:
            return
            
        entry = AuditEntry(
            timestamp=datetime.utcnow().isoformat() + "Z",
            operation=operation,
            path=path,
            agent_id=agent_id,
            details=details
        )
        
        # Append to daily audit log
        date = datetime.utcnow().strftime("%Y-%m-%d")
        audit_path = f".audit/{date}.jsonl"
        
        try:
            existing = self.op.read(audit_path).decode('utf-8')
        except:
            existing = ""
        
        line = json.dumps(entry.__dict__) + "\n"
        self.op.write(audit_path, (existing + line).encode('utf-8'))
    
    def get_audit_log(self, date: str = None, path_filter: str = None) -> list[AuditEntry]:
        """Query audit log."""
        if date is None:
            date = datetime.utcnow().strftime("%Y-%m-%d")
        
        audit_path = f".audit/{date}.jsonl"
        try:
            content = self.op.read(audit_path).decode('utf-8')
        except:
            return []
        
        entries = []
        for line in content.strip().split("\n"):
            if line:
                entry = AuditEntry(**json.loads(line))
                if path_filter is None or path_filter in entry.path:
                    entries.append(entry)
        return entries
    
    # ============ HELPERS ============
    
    def _extract_order(self, name: str) -> int:
        """Extract order number from 'part-01-name' format."""
        import re
        match = re.search(r'-(\d+)-', name)
        return int(match.group(1)) if match else 0
    
    def _update_frontmatter(self, content: str, updates: dict) -> str:
        """Update YAML frontmatter in markdown."""
        import yaml
        
        if content.startswith("---"):
            parts = content.split("---", 2)
            if len(parts) >= 3:
                frontmatter = yaml.safe_load(parts[1]) or {}
                frontmatter.update(updates)
                return f"---\n{yaml.dump(frontmatter)}---{parts[2]}"
        
        # No frontmatter, add it
        return f"---\n{yaml.dump(updates)}---\n\n{content}"
    
    def _hash(self, content: bytes) -> str:
        import hashlib
        return hashlib.sha256(content).hexdigest()[:12]
    
    def _exists(self, path: str) -> bool:
        try:
            self.op.stat(path)
            return True
        except:
            return False
```

### 2. MCP Server - AI Agent Interface

```python
# panaversity_fs/mcp_server.py
from mcp import Server, Tool
from panaversity_fs.core import PanaversityFS
import os

server = Server("panaversity-books")

# Initialize storage based on environment
fs = PanaversityFS(
    backend=os.getenv("STORAGE_BACKEND", "fs"),
    bucket=os.getenv("R2_BUCKET"),
    region="auto",
    endpoint=os.getenv("R2_ENDPOINT"),
    access_key_id=os.getenv("R2_ACCESS_KEY"),
    secret_access_key=os.getenv("R2_SECRET_KEY"),
    root=os.getenv("FS_ROOT", "./content")
)

# ============ READ TOOLS ============

@server.tool("read_lesson")
def read_lesson(book: str, part: str, chapter: str, lesson: str) -> str:
    """Read a lesson's full markdown content.
    
    Args:
        book: Book slug (e.g., 'colearning-python')
        part: Part slug (e.g., 'part-01-foundations')
        chapter: Chapter slug (e.g., 'chapter-01-introduction')
        lesson: Lesson slug (e.g., 'lesson-01-what-is-python')
    
    Returns:
        Full markdown content with frontmatter
    """
    return fs.read_lesson(book, part, chapter, lesson)

@server.tool("get_summary")
def get_summary(book: str, part: str, chapter: str, lesson: str) -> str:
    """Get lesson summary if it exists.
    
    Returns:
        Summary markdown or 'NO_SUMMARY' if not generated yet
    """
    summary = fs.get_summary(book, part, chapter, lesson)
    return summary if summary else "NO_SUMMARY"

@server.tool("get_book_structure")
def get_book_structure(book: str) -> dict:
    """Get complete book structure (parts, chapters, lessons).
    
    Useful for understanding book organization before reading/writing.
    """
    return fs.get_book_structure(book)

@server.tool("list_books")
def list_books() -> list[str]:
    """List all available books."""
    books = []
    for entry in fs.op.list("books/"):
        if entry.path.endswith("/"):
            books.append(entry.path.rstrip("/"))
    return books

# ============ WRITE TOOLS ============

@server.tool("write_lesson")
def write_lesson(book: str, part: str, chapter: str, lesson: str, 
                 content: str, agent_id: str = "claude") -> str:
    """Write or update a lesson.
    
    Args:
        book: Book slug
        part: Part slug  
        chapter: Chapter slug
        lesson: Lesson slug (will create lesson-XX-name.md)
        content: Full markdown content (frontmatter optional, will be added)
        agent_id: Identifier for the writing agent
    
    Returns:
        Confirmation message with path
    """
    fs.write_lesson(book, part, chapter, lesson, content, agent_id)
    return f"✓ Written: books/{book}/{part}/{chapter}/{lesson}.md"

@server.tool("write_summary")
def write_summary(book: str, part: str, chapter: str, lesson: str,
                  summary: str, agent_id: str = "claude") -> str:
    """Generate/update summary for a lesson.
    
    Args:
        book, part, chapter, lesson: Location identifiers
        summary: Summary markdown content
        agent_id: Identifier for the generating agent
    
    Returns:
        Confirmation with source hash for verification
    """
    fs.write_summary(book, part, chapter, lesson, summary, agent_id)
    return f"✓ Summary written for {lesson}"

# ============ AUDIT TOOLS ============

@server.tool("get_recent_changes")
def get_recent_changes(book: str = None, limit: int = 20) -> list[dict]:
    """Get recent changes to content (audit log).
    
    Args:
        book: Optional filter by book
        limit: Max entries to return
    
    Returns:
        List of recent operations with timestamps and agents
    """
    path_filter = f"books/{book}" if book else None
    entries = fs.get_audit_log(path_filter=path_filter)
    return [e.__dict__ for e in entries[-limit:]]

# Run server
if __name__ == "__main__":
    server.run()
```

### 3. CLI - Docusaurus Sync

```python
# panaversity_fs/cli.py
import click
from pathlib import Path
from panaversity_fs.core import PanaversityFS
import os

@click.group()
def cli():
    """PanaversityFS CLI - Sync book content for Docusaurus"""
    pass

@cli.command()
@click.option("--book", required=True, help="Book slug to sync")
@click.option("--to", "dest", default="./docs", help="Destination directory")
@click.option("--include-summaries/--no-summaries", default=True)
def sync(book: str, dest: str, include_summaries: bool):
    """Sync book content from R2 to local filesystem for Docusaurus build."""
    
    # Source: R2 (or local based on env)
    fs = PanaversityFS(
        backend=os.getenv("STORAGE_BACKEND", "s3"),
        bucket=os.getenv("R2_BUCKET"),
        region="auto",
        endpoint=os.getenv("R2_ENDPOINT"),
        access_key_id=os.getenv("R2_ACCESS_KEY"),
        secret_access_key=os.getenv("R2_SECRET_KEY")
    )
    
    dest_path = Path(dest)
    synced = 0
    
    click.echo(f"Syncing {book} to {dest}...")
    
    # Sync all files
    for entry in fs.op.scan(f"books/{book}/"):
        if entry.path.endswith("/"):
            continue
            
        # Skip summaries if not wanted
        if not include_summaries and entry.path.endswith(".summary.md"):
            continue
        
        # Read and write
        content = fs.op.read(entry.path)
        local_path = dest_path / entry.path
        local_path.parent.mkdir(parents=True, exist_ok=True)
        local_path.write_bytes(content)
        
        synced += 1
        click.echo(f"  ✓ {entry.path}")
    
    click.echo(f"\n✓ Synced {synced} files")

@cli.command()
@click.option("--book", required=True)
def structure(book: str):
    """Show book structure."""
    fs = PanaversityFS(
        backend=os.getenv("STORAGE_BACKEND", "fs"),
        root=os.getenv("FS_ROOT", "./content")
    )
    
    struct = fs.get_book_structure(book)
    
    click.echo(f"\n📚 {book}")
    for part in struct["parts"]:
        click.echo(f"  📁 {part['id']}")
        for chapter in part["chapters"]:
            click.echo(f"    📁 {chapter['id']}")
            for lesson in chapter["lessons"]:
                summary_mark = "✓" if lesson["has_summary"] else "○"
                click.echo(f"      📄 {lesson['id']} [{summary_mark}]")

@cli.command()
@click.option("--book", required=True)
def generate_sidebar(book: str):
    """Generate Docusaurus sidebar.js from book structure."""
    fs = PanaversityFS(...)
    struct = fs.get_book_structure(book)
    
    sidebar = {
        "type": "category",
        "label": book.replace("-", " ").title(),
        "items": []
    }
    
    for part in struct["parts"]:
        part_item = {
            "type": "category", 
            "label": part["id"].replace("-", " ").title(),
            "items": []
        }
        for chapter in part["chapters"]:
            chapter_item = {
                "type": "category",
                "label": chapter["id"].replace("-", " ").title(),
                "items": [f"books/{book}/{part['id']}/{chapter['id']}/{l['id']}" 
                         for l in chapter["lessons"]]
            }
            part_item["items"].append(chapter_item)
        sidebar["items"].append(part_item)
    
    import json
    click.echo(json.dumps(sidebar, indent=2))

if __name__ == "__main__":
    cli()
```

## Package Structure

```
panaversity-fs/
├── pyproject.toml
├── README.md
│
├── panaversity_fs/
│   ├── __init__.py
│   ├── core.py              # PanaversityFS class
│   ├── mcp_server.py        # MCP tools for AI agents
│   └── cli.py               # CLI for Docusaurus sync
│
└── tests/
    ├── test_core.py
    ├── test_mcp.py
    └── fixtures/
        └── sample-book/
```

## Usage Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    AI AGENT WORKFLOW                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  Human: "Write a lesson about Python variables"              │
│                                                              │
│  Claude:                                                     │
│    1. get_book_structure("colearning-python")               │
│    2. read_lesson(..., "lesson-01-data-types") # context    │
│    3. write_lesson(..., "lesson-02-variables", content)     │
│    4. write_summary(..., "lesson-02-variables", summary)    │
│                                                              │
│  → Content saved to R2 with audit trail                     │
│                                                              │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   DOCUSAURUS BUILD                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  $ panaversity-fs sync --book colearning-python --to ./docs │
│                                                              │
│    ✓ books/colearning-python/part-01.../chapter-01.../...  │
│    ✓ Synced 47 files                                        │
│                                                              │
│  $ npm run build                                             │
│                                                              │
│  → Static site generated with all content                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## What Makes This Agent-Native?

| Feature | Traditional FS | PanaversityFS |
|---------|---------------|---------------|
| **Audit trail** | ❌ None | ✅ Every read/write logged |
| **Agent tracking** | ❌ Unknown | ✅ `updated_by: claude-agent` |
| **Summary management** | ❌ Manual | ✅ `write_summary()` with source hash |
| **Structure awareness** | ❌ Just paths | ✅ `get_book_structure()` returns hierarchy |
| **Backend agnostic** | ❌ Hardcoded | ✅ OpenDAL: local/R2/S3/any |
| **MCP-first** | ❌ REST/HTTP | ✅ Native MCP tools |

## Next Steps

1. **Week 1:** Core `PanaversityFS` class + tests
2. **Week 2:** MCP server + test with Claude
3. **Week 3:** CLI + Docusaurus integration
4. **Week 4:** Deploy to R2 + CI/CD pipeline

Want me to start implementing the core class?