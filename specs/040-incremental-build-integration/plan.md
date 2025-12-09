# Implementation Plan: Incremental Build Integration

**Feature**: `040-incremental-build-integration`
**Spec**: `specs/040-incremental-build-integration/spec.md`
**Created**: 2025-12-08

---

## Phase Overview

| Phase | Focus | Deliverables | Dependencies |
|-------|-------|--------------|--------------|
| 1 | Hydration Script | `hydrate-book.py` CLI | MCP tools (existing) |
| 2 | Ingestion Pipeline | `ingest-book.py` CLI | hydrate-book.py |
| 3 | Makefile Integration | `make ingest`, `make hydrate` | Both scripts |
| 4 | Local Testing | Test suite + fixtures | All scripts |
| 5 | CI/CD Integration | GitHub Actions workflow | Tested scripts |

---

## Phase 1: Hydration Script (FR-101 to FR-107)

**Goal**: Create `scripts/hydrate-book.py` that fetches content from PanaversityFS using delta detection.

### Architecture

```
hydrate-book.py
├── cli.py              # Click CLI interface
├── mcp_client.py       # MCP tool invocation (plan_build, read_content)
├── manifest.py         # Manifest file read/write
└── downloader.py       # Selective file download logic
```

### Implementation Sequence

1. **MCP Client Module** (`panaversity-fs/scripts/hydrate/mcp_client.py`)
   - Connect to MCP server via HTTP/SSE
   - Implement `call_tool(tool_name, params)` wrapper
   - Implement `plan_build()` wrapper returning structured response
   - Implement `read_content()` for individual file downloads

2. **Manifest Module** (`panaversity-fs/scripts/hydrate/manifest.py`)
   - Define `ManifestFile` Pydantic model: `{book_id, manifest_hash, timestamp, file_count}`
   - Implement `load_manifest(path)` → `ManifestFile | None`
   - Implement `save_manifest(path, manifest)`
   - Handle missing/corrupt manifest gracefully

3. **Downloader Module** (`panaversity-fs/scripts/hydrate/downloader.py`)
   - Implement `download_changed_files(files: list, output_dir: Path)`
   - Stream downloads (don't load all into memory)
   - Handle subdirectory creation
   - Report progress (files downloaded, bytes transferred)

4. **CLI Module** (`panaversity-fs/scripts/hydrate/cli.py`)
   - Click command with options:
     - `--book-id` (required)
     - `--output-dir` (default: `.docusaurus/content`)
     - `--manifest-file` (default: `.panaversity/manifest.json`)
     - `--full-rebuild` (flag)
     - `--verbose` (flag)
   - Orchestrate: load manifest → plan_build → download delta → save manifest

5. **Entry Point** (`panaversity-fs/scripts/hydrate-book.py`)
   - Thin wrapper invoking CLI module

### Key Decisions

- **MCP Communication**: Use HTTP POST to MCP server endpoint (not stdio MCP)
- **Concurrency**: Download files sequentially (simple) or use asyncio.gather (fast)
- **Error Handling**: Retry 3x on network errors, fail build on persistent errors

---

## Phase 2: Ingestion Pipeline (FR-201 to FR-206)

**Goal**: Create `scripts/ingest-book.py` that syncs book-source to PanaversityFS.

### Architecture

```
ingest-book.py
├── cli.py              # Click CLI interface
├── source_scanner.py   # Walk book-source directory
├── path_mapper.py      # Source path → PanaversityFS path
├── sync_engine.py      # Determine add/update/delete operations
└── mcp_client.py       # Reuse from hydrate (shared module)
```

### Implementation Sequence

1. **Source Scanner** (`panaversity-fs/scripts/ingest/source_scanner.py`)
   - Walk `book-source/docs/` recursively
   - Identify markdown files and assets
   - Compute SHA256 for each file
   - Return `list[SourceFile]` with path, hash, content type

2. **Path Mapper** (`panaversity-fs/scripts/ingest/path_mapper.py`)
   - Convert source paths to PanaversityFS schema:
     - `Part-01/Chapter-02/03-lesson.md` → `content/01-Part/02-Chapter/03-lesson.md`
     - `Part-01/Chapter-02/img/diagram.png` → `static/img/diagram.png`
   - Validate mapped paths against FR-007/FR-008 patterns
   - Reject and log non-conforming paths

3. **Sync Engine** (`panaversity-fs/scripts/ingest/sync_engine.py`)
   - Query current PanaversityFS state (glob_search + read_content for hashes)
   - Compare source files to storage files
   - Categorize: `added`, `modified`, `deleted`, `unchanged`
   - For adds: `write_content` without expected_hash
   - For updates: `write_content` with expected_hash
   - For deletes: `delete_content`

4. **CLI Module** (`panaversity-fs/scripts/ingest/cli.py`)
   - Click command with options:
     - `--book-id` (required)
     - `--source-dir` (default: `book-source/docs/`)
     - `--dry-run` (flag)
     - `--verbose` (flag)
   - Orchestrate: scan source → map paths → sync to storage → report summary

5. **Entry Point** (`panaversity-fs/scripts/ingest-book.py`)
   - Thin wrapper invoking CLI module

### Key Decisions

- **Conflict Handling**: Fail fast on conflict (hash mismatch); user must resolve
- **Asset Handling**: Use `upload_asset` for binaries, `write_content` for markdown
- **Delete Policy**: Full sync (delete from storage if not in source) - NOT append-only

---

## Phase 3: Makefile Integration (FR-501)

**Goal**: Provide developer-friendly `make` targets.

### Makefile Targets

```makefile
# Book ingestion: sync source to PanaversityFS
ingest:
	python scripts/ingest-book.py --book-id $(BOOK_ID) --source-dir $(SOURCE_DIR)

# Book hydration: fetch from PanaversityFS for build
hydrate:
	python scripts/hydrate-book.py --book-id $(BOOK_ID) --output-dir .docusaurus/content

# Full local build
local-build: hydrate
	npm run build

# Force full rebuild (bypass manifest)
full-rebuild:
	python scripts/hydrate-book.py --book-id $(BOOK_ID) --full-rebuild
	npm run build

# Dry-run ingestion (see what would change)
ingest-dry:
	python scripts/ingest-book.py --book-id $(BOOK_ID) --dry-run
```

### Environment Configuration

```bash
# .env.example
PANAVERSITY_MCP_URL=http://localhost:8000/mcp
PANAVERSITY_BOOK_ID=ai-native-python
PANAVERSITY_STORAGE_BACKEND=fs
PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-data
```

---

## Phase 4: Testing (Local Validation)

**Goal**: Verify the pipeline works end-to-end locally.

### Test Structure

```
panaversity-fs/tests/
├── integration/
│   ├── test_hydrate_incremental.py    # Delta detection tests
│   ├── test_hydrate_full_rebuild.py   # Full rebuild tests
│   ├── test_ingest_sync.py            # Ingestion sync tests
│   └── test_ingest_path_mapping.py    # Path conversion tests
└── fixtures/
    ├── book-source-sample/            # Mini book for testing
    │   ├── Part-01/
    │   │   └── Chapter-01/
    │   │       ├── 01-intro.md
    │   │       └── img/
    │   │           └── diagram.png
    │   └── README.md
    └── expected-output/               # Expected PanaversityFS state
```

### Test Scenarios

1. **Hydrate - First Build**
   - No manifest exists
   - All files downloaded
   - Manifest created with current hash

2. **Hydrate - Incremental Build**
   - Manifest exists from previous build
   - Only changed files downloaded
   - Manifest updated with new hash

3. **Hydrate - Full Rebuild Flag**
   - Manifest exists but `--full-rebuild` passed
   - All files downloaded (bypasses manifest)
   - Manifest updated

4. **Ingest - New Files**
   - Source has files, storage empty
   - All files created in storage

5. **Ingest - Modified Files**
   - Source file changed (different hash)
   - Storage file updated with new content

6. **Ingest - Deleted Files**
   - Source file removed
   - Storage file deleted

7. **Ingest - Path Mapping**
   - Various source paths mapped correctly
   - Invalid paths rejected with clear error

---

## Phase 5: CI/CD Integration

**Goal**: Integrate into GitHub Actions for production builds.

### GitHub Actions Workflow

```yaml
# .github/workflows/build-book.yml
name: Build Book

on:
  push:
    branches: [main]
    paths:
      - 'book-source/**'
  workflow_dispatch:
    inputs:
      full_rebuild:
        description: 'Force full rebuild'
        type: boolean
        default: false

jobs:
  ingest:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.13'

      - name: Install dependencies
        run: pip install -e ./panaversity-fs[dev]

      - name: Ingest to PanaversityFS
        env:
          PANAVERSITY_MCP_URL: ${{ secrets.MCP_URL }}
          PANAVERSITY_BOOK_ID: ai-native-python
        run: python panaversity-fs/scripts/ingest-book.py

  build:
    needs: ingest
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Restore manifest cache
        uses: actions/cache@v4
        with:
          path: .panaversity/manifest.json
          key: manifest-${{ github.ref }}-${{ github.sha }}
          restore-keys: |
            manifest-${{ github.ref }}-
            manifest-main-

      - name: Hydrate content
        env:
          PANAVERSITY_MCP_URL: ${{ secrets.MCP_URL }}
          PANAVERSITY_BOOK_ID: ai-native-python
        run: |
          if [ "${{ inputs.full_rebuild }}" == "true" ]; then
            python panaversity-fs/scripts/hydrate-book.py --full-rebuild
          else
            python panaversity-fs/scripts/hydrate-book.py
          fi

      - name: Build Docusaurus
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v4
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### Cache Strategy

- **Manifest cache**: Keyed by branch + commit SHA
- **Restore keys**: Fall back to branch-level, then main branch
- **Full rebuild**: Skips cache restoration

---

## Risk Mitigations

| Risk | Mitigation in Plan |
|------|-------------------|
| MCP server unavailable | Scripts fail fast with clear error; no silent fallback |
| Large book timeout | Streaming downloads; 10-minute timeout for full builds |
| Manifest corruption | Validate JSON before use; fallback to full rebuild |
| Path mapping errors | Strict validation; reject and log non-conforming paths |

---

## File Structure (Final)

```
panaversity-fs/
├── scripts/
│   ├── hydrate-book.py           # Entry point
│   ├── ingest-book.py            # Entry point
│   ├── hydrate/
│   │   ├── __init__.py
│   │   ├── cli.py
│   │   ├── mcp_client.py
│   │   ├── manifest.py
│   │   └── downloader.py
│   ├── ingest/
│   │   ├── __init__.py
│   │   ├── cli.py
│   │   ├── source_scanner.py
│   │   ├── path_mapper.py
│   │   └── sync_engine.py
│   └── common/
│       └── mcp_client.py         # Shared MCP client
├── tests/
│   ├── integration/
│   │   ├── test_hydrate_incremental.py
│   │   ├── test_hydrate_full_rebuild.py
│   │   ├── test_ingest_sync.py
│   │   └── test_ingest_path_mapping.py
│   └── fixtures/
│       └── book-source-sample/
└── Makefile
```

---

## Estimated Complexity

| Component | Files | Complexity | Notes |
|-----------|-------|------------|-------|
| MCP Client | 1 | Low | HTTP POST wrapper |
| Manifest Module | 1 | Low | JSON read/write |
| Downloader | 1 | Medium | Streaming, progress |
| Hydrate CLI | 1 | Medium | Orchestration |
| Source Scanner | 1 | Low | Directory walk |
| Path Mapper | 1 | Medium | Regex validation |
| Sync Engine | 1 | Medium | Diff logic |
| Ingest CLI | 1 | Medium | Orchestration |
| Tests | 4+ | Medium | Integration tests |
| CI/CD | 1 | Low | YAML workflow |

**Total**: ~12 files, ~1000-1500 lines of code
