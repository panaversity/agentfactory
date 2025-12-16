# Feature Specification: Incremental Build Integration

**Feature Branch**: `incremental-build-integration`
**Created**: 2025-12-08
**Status**: Draft
**Input**: Enable incremental builds for 1000+ lesson books by integrating PanaversityFS delta tools with Docusaurus build pipeline
**Predecessor**: `specs/039-panaversity-fs-hardening/` (delta_build, plan_build tools)

## Executive Summary

The PanaversityFS MCP server has production-ready delta detection tools (`delta_build`, `plan_build`) per FR-025/026/027, but they're not integrated into the actual Docusaurus build pipeline. With 1000+ lessons per book, full builds are infeasible (~1GB download per build). This specification defines the integration that enables:

1. **Incremental downloads** - Only fetch changed files using `plan_build` manifest comparison
2. **Book source ingestion** - Auto-sync content from source to PanaversityFS storage
3. **Asset pipeline** - Handle static assets (images, slides) alongside content
4. **Local/staging testing** - Verify the pipeline before production deployment

---

## User Scenarios & Testing _(mandatory)_

### User Story 1 - Docusaurus Incremental Build (Priority: P0 - CRITICAL)

A CI/CD pipeline needs to build a 1000+ lesson book without downloading all content every time.

**Why this priority**: This is the blocking issue. Without incremental builds, shipping is impossible (502 timeouts, excessive bandwidth, 10+ minute builds).

**Independent Test**: Run two consecutive builds with only 1 file changed. Second build should download only that file, not 1000+.

**Acceptance Scenarios**:

1. **Given** a book with 1000 lessons exists, **When** `plan_build` is called with previous manifest hash, **Then** only changed files are returned (not all 1000)
2. **Given** first build (no manifest), **When** hydration script runs, **Then** it downloads all files AND stores manifest hash for next build
3. **Given** second build with stored manifest, **When** 3 lessons changed, **Then** only those 3 are downloaded (measured by bytes transferred)
4. **Given** build fails mid-download, **When** retried, **Then** it resumes using same manifest (idempotent)

---

### User Story 2 - Book Source Ingestion (Priority: P1)

Authors write content in `apps/learn-app/docs/` which needs to be synced to PanaversityFS before builds.

**Why this priority**: Without ingestion, there's no content in PanaversityFS to build from. The pipeline is: Author writes → Ingest → Build.

**Independent Test**: Add a lesson to `apps/learn-app/docs/`, run ingestion, verify it appears in PanaversityFS.

**Acceptance Scenarios**:

1. **Given** new lesson at `apps/learn-app/docs/Part-01/Chapter-01/01-intro.md`, **When** ingestion runs, **Then** file exists at `content/01-Part/01-Chapter/01-intro.md` in PanaversityFS
2. **Given** lesson modified in source, **When** ingestion runs, **Then** PanaversityFS file updated with new hash
3. **Given** lesson deleted from source, **When** ingestion runs, **Then** PanaversityFS file deleted (sync, not append-only)
4. **Given** image at `apps/learn-app/docs/Part-01/Chapter-01/img/diagram.png`, **When** ingestion runs, **Then** asset uploaded via `upload_asset`

---

### User Story 3 - Asset Handling in Builds (Priority: P2)

Build pipeline needs to include static assets (images, slides) with incremental detection.

**Why this priority**: Assets are often larger than content. Incremental asset handling prevents re-downloading 100MB of images every build.

**Independent Test**: Change one image, verify only that image is re-downloaded in next build.

**Acceptance Scenarios**:

1. **Given** book has 100 images, **When** `plan_build` returns changes, **Then** only changed images are listed
2. **Given** `scope=content` on archive, **When** building markdown-only output, **Then** assets not downloaded
3. **Given** `scope=all` on archive, **When** building full site, **Then** both content and assets included

---

### User Story 4 - Local Development Workflow (Priority: P2)

Developer needs to test the pipeline locally before committing.

**Why this priority**: Can't test 1000-lesson builds in production. Local workflow enables iteration.

**Independent Test**: Run full pipeline locally with 10 test lessons, verify output matches expected.

**Acceptance Scenarios**:

1. **Given** developer runs `make local-build`, **When** SQLite backend configured, **Then** full pipeline completes in <30s for 100 files
2. **Given** local build succeeds, **When** same content pushed to staging, **Then** staging build produces identical output
3. **Given** manifest mismatch between local and staging, **When** developer debugs, **Then** manifest diff tool shows which files differ

---

### Edge Cases

- **First build after schema migration**: All files have new paths → `plan_build` returns everything (expected, full rebuild)
- **Deleted manifest reference**: What if stored manifest hash doesn't exist in PanaversityFS? → Return warning, fallback to full build (FR-027 compliant)
- **Large single file**: What if one lesson is 50MB (embedded images)? → Stream it, don't load into memory
- **Concurrent ingestion and build**: What if ingestion runs while build is downloading? → Build uses snapshot from `plan_build` call time (manifest hash locks state)
- **Empty book**: What if book has no content yet? → Return empty archive with metadata, not error

---

## Requirements _(mandatory)_

### Functional Requirements

**Hydration Script (Docusaurus Integration)**

- **FR-101**: Provide `scripts/hydrate-book.py` that fetches content from PanaversityFS for Docusaurus builds
- **FR-102**: Hydration script MUST accept `--book-id`, `--output-dir`, and `--manifest-file` parameters
- **FR-103**: On first run (no manifest), script MUST call `plan_build(book_id)` to get all files and save manifest
- **FR-104**: On subsequent runs, script MUST call `plan_build(book_id, target_manifest_hash=<stored>)` to get delta
- **FR-105**: Script MUST download only files listed in `plan_build` response, not full archive
- **FR-106**: Script MUST update stored manifest hash after successful download
- **FR-107**: Script MUST support `--full-rebuild` flag to bypass manifest and force full download

**Ingestion Pipeline (Book Source → PanaversityFS)**

- **FR-201**: Provide `scripts/ingest-book.py` that syncs `apps/learn-app/docs/` to PanaversityFS
- **FR-202**: Ingestion MUST map source paths to PanaversityFS schema (e.g., `Part-01/Chapter-01/` → `content/01-Part/01-Chapter/`)
- **FR-203**: Ingestion MUST use `write_content` with proper hash-based conflict detection
- **FR-204**: Ingestion MUST handle asset files via `upload_asset` tool
- **FR-205**: Ingestion MUST support `--dry-run` flag to show what would change without writing
- **FR-206**: Ingestion MUST report summary: files added, updated, deleted, unchanged

**Manifest Management**

- **FR-301**: Manifest hash MUST be stored in `.panaversity/manifest.json` in build output directory
- **FR-302**: Manifest file format: `{"book_id": "...", "manifest_hash": "...", "timestamp": "...", "file_count": N}`
- **FR-303**: CI/CD pipelines MUST cache manifest file between builds for incremental detection
- **FR-304**: Manifest MUST be invalidated (deleted) when `--full-rebuild` requested

**Performance Requirements**

- **FR-401**: Incremental download of 10 changed files MUST complete in <10 seconds (excluding network latency)
- **FR-402**: Ingestion of 1000 files MUST complete in <60 seconds
- **FR-403**: Memory usage MUST stay below 256MB during hydration (streaming downloads)

**CLI Interface**

- **FR-501**: Provide `Makefile` targets: `make ingest`, `make hydrate`, `make local-build`
- **FR-502**: Scripts MUST read configuration from environment variables: `PANAVERSITY_MCP_URL`, `PANAVERSITY_BOOK_ID`
- **FR-503**: Scripts MUST provide `--verbose` flag for detailed logging
- **FR-504**: Scripts MUST exit with non-zero code on failure with clear error message

---

## Architecture

### Component Interaction

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Author Workflow                                 │
│                                                                      │
│  apps/learn-app/docs/     ──ingest-book.py──►    PanaversityFS MCP     │
│  (markdown + assets)         (write_content,      (storage)         │
│                               upload_asset)                          │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ plan_build (manifest comparison)
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Build Workflow                                  │
│                                                                      │
│  CI/CD Pipeline  ──hydrate-book.py──►  .docusaurus/content/         │
│  (GitHub Actions)     (selective          (downloaded files)         │
│                        download)                                     │
│                           │                                          │
│                           ▼                                          │
│                    docusaurus build                                  │
│                           │                                          │
│                           ▼                                          │
│                    dist/ (deployed site)                            │
└─────────────────────────────────────────────────────────────────────┘
```

### Manifest Hash Flow

```
Build N:                           Build N+1:
─────────                          ──────────
1. plan_build()                    1. Read manifest.json → hash "abc123"
   → all files                     2. plan_build(target="abc123")
   → manifest_hash: "abc123"          → only changed: [file1, file2]
                                      → new manifest_hash: "def456"
2. Download all files              3. Download only file1, file2
3. Save manifest.json              4. Update manifest.json
   {"hash": "abc123"}                 {"hash": "def456"}
```

---

## Success Criteria _(mandatory)_

### Measurable Outcomes

- **SC-101**: Incremental build with 5 changed files downloads <500KB (vs 50MB+ for full book)
- **SC-102**: Build time for 5-file delta is <30 seconds (vs 5+ minutes for full)
- **SC-103**: Manifest comparison accuracy is 100% (no false positives/negatives in delta)
- **SC-104**: Ingestion handles 1000 files without OOM or timeout
- **SC-105**: Pipeline runs identically on local SQLite and production PostgreSQL/R2

### Instrumentation

| Metric                      | Location        | Purpose                   |
| --------------------------- | --------------- | ------------------------- |
| `hydrate_files_downloaded`  | hydrate-book.py | Track incremental vs full |
| `hydrate_bytes_transferred` | hydrate-book.py | Measure bandwidth savings |
| `ingest_files_changed`      | ingest-book.py  | Track sync operations     |
| `build_total_seconds`       | CI/CD           | End-to-end build time     |

---

## Non-Goals _(explicit scope boundaries)_

- **Real-time sync**: No live watching of book-source. Ingestion is triggered, not continuous.
- **Multi-book builds**: One book per build. Cross-book builds are separate pipelines.
- **Conflict resolution UI**: Ingestion fails on conflict; manual resolution required.
- **CDN invalidation**: After build, CDN caching is external concern.
- **Docusaurus plugin**: We're using scripts, not a Docusaurus plugin architecture.

---

## Assumptions

- **MCP server is running**: `hydrate-book.py` assumes PanaversityFS MCP server is accessible at configured URL
- **book-source structure**: Source follows `Part-NN/Chapter-NN/NN-lesson.md` naming convention
- **Single book per repo**: Each repository has one book; book_id derived from config
- **GitHub Actions caching**: CI/CD will cache manifest.json between runs
- **Network reliability**: Scripts retry on transient network failures (3 attempts)

---

## Risks & Mitigations

**Risk 1: Manifest corruption**

- **Likelihood**: Low (simple JSON file)
- **Impact**: High (forces full rebuild)
- **Mitigation**: Validate manifest format before use; fallback to full rebuild on parse error

**Risk 2: Ingestion conflicts with concurrent edits**

- **Likelihood**: Medium (multiple authors)
- **Impact**: Medium (ingestion fails, requires retry)
- **Mitigation**: Ingestion uses hash-based conflict detection; clear error messages guide retry

**Risk 3: Large book exceeds download timeout**

- **Likelihood**: Low (incremental is small by design)
- **Impact**: High (build fails)
- **Mitigation**: Full rebuild has 10-minute timeout; incremental should never hit this

---

## Formal Verification (Alloy-Style Invariants)

### I1: Manifest Consistency

```alloy
sig Manifest {
  hash: String,
  file_count: Int
}

sig Build {
  input_manifest: lone Manifest,
  output_manifest: Manifest,
  downloaded_files: set File
}

pred incrementalBuild[b: Build] {
  some b.input_manifest implies {
    -- Delta: only download changed files
    #b.downloaded_files <= (b.output_manifest.file_count - unchanged_count)
  }
}

assert NoUnnecessaryDownloads {
  all b: Build | incrementalBuild[b]
}
check NoUnnecessaryDownloads for 5
```

**Small-scope test**: 5 builds with varying deltas
**Pytest assertion**: `assert downloaded_count <= changed_count`

### I2: Ingestion Idempotency

```alloy
pred ingestIdempotent[source: set File, storage: Storage] {
  let storage' = ingest(source, storage) |
  let storage'' = ingest(source, storage') |
  storage' = storage''
}

assert IngestIdempotent {
  all source: set File, storage: Storage | ingestIdempotent[source, storage]
}
check IngestIdempotent for 5
```

**Small-scope test**: Run ingestion twice with same source
**Pytest assertion**: `assert journal_state_after_first == journal_state_after_second`

---

## Dependencies

**Runtime Dependencies**:

- PanaversityFS MCP server (existing)
- httpx (async HTTP client for MCP calls)
- click (CLI framework for scripts)
- pydantic (configuration validation)

**CI/CD Integration**:

- GitHub Actions (build pipeline)
- Actions cache (manifest persistence)

**Development Tools**:

- pytest (testing)
- pytest-asyncio (async tests)
- make (build orchestration)

---

## Open Questions

1. **Q**: Should manifest be stored in repo or external cache?
   **A**: External cache (GitHub Actions cache) to avoid polluting git history with build artifacts.

2. **Q**: How to handle book-source → PanaversityFS path mapping edge cases?
   **A**: Define explicit mapping rules in FR-202; reject non-conforming paths with clear errors.

3. **Q**: What's the fallback if MCP server is unavailable during build?
   **A**: Build fails with clear error. No local fallback (content must come from authoritative source).
