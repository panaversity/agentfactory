# Implementation Tasks: Incremental Build Integration

**Feature**: `040-incremental-build-integration`
**Plan**: `specs/040-incremental-build-integration/plan.md`
**Created**: 2025-12-08

---

## Task Overview

| Phase | Tasks | Priority | Status |
|-------|-------|----------|--------|
| Phase 1 | T-001 to T-005 | P0 | Pending |
| Phase 2 | T-101 to T-105 | P1 | Pending |
| Phase 3 | T-201 to T-203 | P2 | Pending |
| Phase 4 | T-301 to T-304 | P2 | Pending |
| Phase 5 | T-401 to T-403 | P3 | Pending |

---

## Phase 1: Hydration Script

### T-001: Create Common MCP Client Module

**File**: `panaversity-fs/scripts/common/mcp_client.py`

**Requirements**:
- [ ] Async HTTP client using httpx
- [ ] `MCPClient` class with `call_tool(tool_name: str, params: dict) -> dict`
- [ ] Support for `plan_build`, `read_content` tool calls
- [ ] Retry logic (3 attempts with exponential backoff)
- [ ] Configuration via `PANAVERSITY_MCP_URL` environment variable
- [ ] Proper error handling with custom exceptions

**Acceptance**:
```python
client = MCPClient()
result = await client.call_tool("plan_build", {"book_id": "test-book"})
assert "manifest_hash" in result
```

---

### T-002: Create Manifest Module

**File**: `panaversity-fs/scripts/hydrate/manifest.py`

**Requirements**:
- [ ] Pydantic model `ManifestFile`: `{book_id: str, manifest_hash: str, timestamp: str, file_count: int}`
- [ ] `load_manifest(path: Path) -> ManifestFile | None`
- [ ] `save_manifest(path: Path, data: ManifestFile) -> None`
- [ ] Create parent directories if not exist
- [ ] Return None on missing/corrupt file (don't raise)

**Acceptance**:
```python
manifest = load_manifest(Path(".panaversity/manifest.json"))
assert manifest is None  # First run

save_manifest(path, ManifestFile(book_id="test", manifest_hash="abc", timestamp="2025-12-08T00:00:00Z", file_count=10))
manifest = load_manifest(path)
assert manifest.manifest_hash == "abc"
```

---

### T-003: Create Downloader Module

**File**: `panaversity-fs/scripts/hydrate/downloader.py`

**Requirements**:
- [ ] `download_changed_files(client: MCPClient, files: list[dict], output_dir: Path) -> DownloadResult`
- [ ] Stream each file to disk (don't load all into memory)
- [ ] Create subdirectories as needed
- [ ] Track: files_downloaded, bytes_transferred, errors
- [ ] Support verbose logging option

**Acceptance**:
```python
files = [{"path": "content/01-Part/01-Chapter/01-lesson.md", "current_hash": "abc"}]
result = await download_changed_files(client, files, Path("output"))
assert result.files_downloaded == 1
assert Path("output/content/01-Part/01-Chapter/01-lesson.md").exists()
```

---

### T-004: Create Hydrate CLI

**File**: `panaversity-fs/scripts/hydrate/cli.py`

**Requirements**:
- [ ] Click command `hydrate`
- [ ] Options: `--book-id` (required), `--output-dir` (default: `.docusaurus/content`), `--manifest-file` (default: `.panaversity/manifest.json`), `--full-rebuild` (flag), `--verbose` (flag)
- [ ] Logic flow:
  1. Load manifest (if exists and not `--full-rebuild`)
  2. Call `plan_build` with manifest hash (or without for full)
  3. Download changed files
  4. Save new manifest
- [ ] Clear progress output: "Downloading 3/100 files..."
- [ ] Exit code 0 on success, non-zero on failure

**Acceptance**:
```bash
# First run - downloads all
python scripts/hydrate-book.py --book-id test-book
# Output: Downloaded 100 files (5.2MB)

# Second run - downloads delta
python scripts/hydrate-book.py --book-id test-book
# Output: Downloaded 0 files (no changes)
```

---

### T-005: Create Hydrate Entry Point

**File**: `panaversity-fs/scripts/hydrate-book.py`

**Requirements**:
- [ ] Shebang for direct execution: `#!/usr/bin/env python`
- [ ] Import and invoke CLI from `scripts/hydrate/cli.py`
- [ ] Handle KeyboardInterrupt gracefully

**Acceptance**:
```bash
python panaversity-fs/scripts/hydrate-book.py --help
# Shows help with all options
```

---

## Phase 2: Ingestion Pipeline

### T-101: Create Source Scanner

**File**: `panaversity-fs/scripts/ingest/source_scanner.py`

**Requirements**:
- [ ] `scan_source_directory(source_dir: Path) -> list[SourceFile]`
- [ ] `SourceFile` dataclass: `{path: Path, relative_path: str, sha256: str, content_type: str}`
- [ ] Identify markdown files (`.md`)
- [ ] Identify assets (images: `.png`, `.jpg`, `.gif`, `.svg`, `.webp`)
- [ ] Skip hidden files/directories (`.git`, `.DS_Store`)
- [ ] Compute SHA256 hash for each file

**Acceptance**:
```python
files = scan_source_directory(Path("book-source/docs"))
assert any(f.relative_path == "Part-01/Chapter-01/01-intro.md" for f in files)
assert all(len(f.sha256) == 64 for f in files)
```

---

### T-102: Create Path Mapper

**File**: `panaversity-fs/scripts/ingest/path_mapper.py`

**Requirements**:
- [ ] `map_source_to_storage(source_path: str) -> str | None`
- [ ] Mapping rules:
  - `Part-NN/Chapter-NN/NN-name.md` → `content/NN-Part/NN-Chapter/NN-name.md`
  - `Part-NN/Chapter-NN/img/*.png` → `static/img/*.png`
  - `Part-NN/Chapter-NN/*.summary.md` → `content/NN-Part/NN-Chapter/*.summary.md`
- [ ] Return None for non-conforming paths
- [ ] Validate output paths against FR-007/FR-008 patterns

**Acceptance**:
```python
assert map_source_to_storage("Part-01/Chapter-02/03-lesson.md") == "content/01-Part/02-Chapter/03-lesson.md"
assert map_source_to_storage("Part-01/Chapter-02/img/diagram.png") == "static/img/diagram.png"
assert map_source_to_storage("invalid/path.txt") is None
```

---

### T-103: Create Sync Engine

**File**: `panaversity-fs/scripts/ingest/sync_engine.py`

**Requirements**:
- [ ] `sync_to_storage(client: MCPClient, book_id: str, source_files: list[SourceFile], dry_run: bool) -> SyncResult`
- [ ] Query current storage state via `glob_search` + `read_content`
- [ ] Categorize files: `added`, `modified`, `deleted`, `unchanged`
- [ ] For adds: call `write_content` without expected_hash
- [ ] For updates: call `write_content` with expected_hash from storage
- [ ] For deletes: call `delete_content`
- [ ] `SyncResult` dataclass: `{added: int, modified: int, deleted: int, unchanged: int, errors: list}`
- [ ] Respect `dry_run` flag (log but don't write)

**Acceptance**:
```python
result = await sync_to_storage(client, "test-book", source_files, dry_run=False)
assert result.added + result.modified + result.deleted + result.unchanged == len(source_files)
```

---

### T-104: Create Ingest CLI

**File**: `panaversity-fs/scripts/ingest/cli.py`

**Requirements**:
- [ ] Click command `ingest`
- [ ] Options: `--book-id` (required), `--source-dir` (default: `book-source/docs/`), `--dry-run` (flag), `--verbose` (flag)
- [ ] Logic flow:
  1. Scan source directory
  2. Map paths to storage format
  3. Sync to storage
  4. Report summary
- [ ] Summary output: "Synced 50 files: 5 added, 3 modified, 2 deleted, 40 unchanged"
- [ ] Exit code 0 on success, non-zero on failure

**Acceptance**:
```bash
python scripts/ingest-book.py --book-id test-book --dry-run
# Output: Would sync 50 files: 5 added, 3 modified, 2 deleted, 40 unchanged
```

---

### T-105: Create Ingest Entry Point

**File**: `panaversity-fs/scripts/ingest-book.py`

**Requirements**:
- [ ] Shebang for direct execution: `#!/usr/bin/env python`
- [ ] Import and invoke CLI from `scripts/ingest/cli.py`
- [ ] Handle KeyboardInterrupt gracefully

**Acceptance**:
```bash
python panaversity-fs/scripts/ingest-book.py --help
# Shows help with all options
```

---

## Phase 3: Build Integration

### T-201: Create Makefile Targets

**File**: `panaversity-fs/Makefile` (update existing or create)

**Requirements**:
- [ ] Target `ingest`: Run ingest-book.py with env vars
- [ ] Target `hydrate`: Run hydrate-book.py with env vars
- [ ] Target `local-build`: hydrate + npm run build
- [ ] Target `full-rebuild`: hydrate with --full-rebuild + npm run build
- [ ] Target `ingest-dry`: Dry-run ingestion
- [ ] Use `$(BOOK_ID)` variable with default from env

**Acceptance**:
```bash
make hydrate BOOK_ID=test-book
# Runs hydration script
```

---

### T-202: Create Environment Configuration

**File**: `panaversity-fs/.env.example`

**Requirements**:
- [ ] Document all required environment variables
- [ ] Provide sensible defaults for local development
- [ ] Include comments explaining each variable

**Acceptance**:
```bash
cat panaversity-fs/.env.example
# PANAVERSITY_MCP_URL=http://localhost:8000/mcp
# PANAVERSITY_BOOK_ID=ai-native-python
# ...
```

---

### T-203: Update pyproject.toml with Script Dependencies

**File**: `panaversity-fs/pyproject.toml` (update)

**Requirements**:
- [ ] Add `click` to dependencies
- [ ] Add `httpx` to dependencies (if not present)
- [ ] Add script entry points:
  ```toml
  [project.scripts]
  hydrate-book = "scripts.hydrate.cli:main"
  ingest-book = "scripts.ingest.cli:main"
  ```

**Acceptance**:
```bash
uv run hydrate-book --help
# Shows CLI help
```

---

## Phase 4: Testing

### T-301: Create Test Fixtures

**Directory**: `panaversity-fs/tests/fixtures/book-source-sample/`

**Requirements**:
- [ ] Create mini book structure:
  ```
  Part-01/
    Chapter-01/
      01-intro.md
      02-concepts.md
      img/
        diagram.png
    README.md
  ```
- [ ] Include valid markdown content
- [ ] Include sample image (small PNG)

**Acceptance**:
```bash
ls panaversity-fs/tests/fixtures/book-source-sample/Part-01/Chapter-01/
# 01-intro.md  02-concepts.md  img/
```

---

### T-302: Create Hydration Integration Tests

**File**: `panaversity-fs/tests/integration/test_hydrate_incremental.py`

**Requirements**:
- [ ] Test first build (no manifest) downloads all files
- [ ] Test incremental build downloads only changed files
- [ ] Test `--full-rebuild` bypasses manifest
- [ ] Test manifest file is created/updated correctly
- [ ] Use pytest-asyncio for async tests
- [ ] Mock MCP client or use local server

**Acceptance**:
```bash
uv run pytest tests/integration/test_hydrate_incremental.py -v
# All tests pass
```

---

### T-303: Create Ingestion Integration Tests

**File**: `panaversity-fs/tests/integration/test_ingest_sync.py`

**Requirements**:
- [ ] Test new files are created in storage
- [ ] Test modified files are updated with correct hash
- [ ] Test deleted files are removed from storage
- [ ] Test dry-run doesn't modify storage
- [ ] Test path mapping edge cases
- [ ] Use pytest-asyncio for async tests

**Acceptance**:
```bash
uv run pytest tests/integration/test_ingest_sync.py -v
# All tests pass
```

---

### T-304: Create Path Mapper Unit Tests

**File**: `panaversity-fs/tests/unit/test_path_mapper.py`

**Requirements**:
- [ ] Test valid content path mappings
- [ ] Test valid asset path mappings
- [ ] Test summary file mappings
- [ ] Test invalid path rejection
- [ ] Test edge cases (special characters, long paths)

**Acceptance**:
```bash
uv run pytest tests/unit/test_path_mapper.py -v
# All tests pass
```

---

## Phase 5: CI/CD Integration

### T-401: Create GitHub Actions Workflow

**File**: `.github/workflows/build-book.yml`

**Requirements**:
- [ ] Trigger on push to main (book-source paths)
- [ ] Trigger on workflow_dispatch with full_rebuild input
- [ ] Job 1: Ingest content to PanaversityFS
- [ ] Job 2: Hydrate + build Docusaurus
- [ ] Cache manifest.json between runs
- [ ] Deploy to GitHub Pages

**Acceptance**:
```bash
# Push change to book-source/
# GitHub Action runs successfully
# Site is deployed with only changed content
```

---

### T-402: Configure GitHub Secrets

**Secrets** (document, not implement):
- [ ] `MCP_URL`: Production PanaversityFS MCP endpoint
- [ ] `BOOK_ID`: Book identifier for this repository

**Acceptance**:
```markdown
# Documentation in README explaining required secrets
```

---

### T-403: Create Local Development Guide

**File**: `panaversity-fs/docs/LOCAL-DEVELOPMENT.md`

**Requirements**:
- [ ] Step-by-step setup for local development
- [ ] How to run MCP server locally
- [ ] How to ingest test content
- [ ] How to hydrate and build
- [ ] Troubleshooting common issues

**Acceptance**:
```bash
# New developer can follow guide and run local build within 15 minutes
```

---

## Task Dependencies

```
T-001 (MCP Client)
    ↓
T-002 (Manifest) ──────┐
    ↓                  │
T-003 (Downloader) ────┤
    ↓                  │
T-004 (Hydrate CLI) ───┴──→ T-005 (Entry Point)
                              ↓
T-101 (Scanner) ────────────────┐
    ↓                           │
T-102 (Path Mapper) ────────────┤
    ↓                           │
T-103 (Sync Engine) ────────────┴──→ T-104 (Ingest CLI) ──→ T-105 (Entry Point)
                                            ↓
                              T-201 (Makefile) ─────────────────┐
                                     │                          │
                              T-202 (Env Config)                │
                                     │                          │
                              T-203 (pyproject.toml)            │
                                            ↓                   │
                              T-301 (Fixtures) ─────────────────┤
                                     │                          │
                              T-302 (Hydrate Tests)             │
                                     │                          │
                              T-303 (Ingest Tests)              │
                                     │                          │
                              T-304 (Path Mapper Tests)         │
                                            ↓                   │
                              T-401 (GitHub Actions) ───────────┘
                                     │
                              T-402 (Secrets Doc)
                                     │
                              T-403 (Dev Guide)
```

---

## Implementation Order (Recommended)

1. **T-001** - MCP Client (foundation for all other tasks)
2. **T-002** - Manifest Module
3. **T-003** - Downloader
4. **T-004** - Hydrate CLI
5. **T-005** - Hydrate Entry Point
6. **T-301** - Test Fixtures (needed for testing)
7. **T-302** - Hydrate Integration Tests (validate Phase 1)
8. **T-102** - Path Mapper (foundation for ingestion)
9. **T-304** - Path Mapper Unit Tests (validate early)
10. **T-101** - Source Scanner
11. **T-103** - Sync Engine
12. **T-104** - Ingest CLI
13. **T-105** - Ingest Entry Point
14. **T-303** - Ingest Integration Tests (validate Phase 2)
15. **T-201** - Makefile
16. **T-202** - Environment Config
17. **T-203** - pyproject.toml update
18. **T-401** - GitHub Actions
19. **T-402** - Secrets Documentation
20. **T-403** - Local Development Guide
