---
id: 0004
title: adr-0018-docusaurus-aligned-structure
stage: refactor
date: 2025-11-25
surface: claude-code
model: claude-opus-4-5-20251101
feature: 030-panaversity-fs
branch: 030-panaversity-fs
user: mjs
command: direct-chat
labels: [refactoring, mcp-tools, docusaurus-alignment, adr]
links:
  spec: specs/030-panaversity-fs/spec.md
  ticket: null
  adr: history/adr/0018-panaversityfs-docusaurus-aligned-structure.md
  pr: "#299"
files:
  - history/adr/0018-panaversityfs-docusaurus-aligned-structure.md (NEW)
  - specs/030-panaversity-fs/spec.md (MODIFIED)
  - src/panaversity_fs/tools/summaries.py (DELETED)
  - src/panaversity_fs/server.py (MODIFIED)
  - src/panaversity_fs/models.py (MODIFIED)
  - src/panaversity_fs/tools/__init__.py (MODIFIED)
  - src/panaversity_fs/tools/content.py (MODIFIED)
  - tests/unit/test_summary_tools.py (DELETED)
  - tests/e2e/test_complete_book_workflow.py (MODIFIED)
  - tests/edge_cases/test_production_structure.py (MODIFIED)
  - tests/unit/test_content_tools.py (MODIFIED)
  - tests/unit/test_search_tools.py (MODIFIED)
  - tests/conftest.py (MODIFIED)
  - docs/ARCHITECTURE.md (MODIFIED)
  - docs/MCP-TOOLS.md (MODIFIED)
  - README.md (MODIFIED)
tests:
  - tests/unit/test_content_tools.py (10 tests)
  - tests/unit/test_search_tools.py (10 tests)
  - tests/unit/test_registry_bulk_tools.py (6 tests)
  - tests/integration/test_content_workflows.py (3 tests)
  - tests/e2e/test_complete_book_workflow.py (3 tests)
  - tests/edge_cases/test_production_structure.py (13 tests)
---

## Prompt

User requested storage structure alignment with Docusaurus and removal of summary tools:

> "Should this be like [Docusaurus-aligned structure]... This makes the structure more reliable in itself like here @book-source/docs/"

After clarifying that summaries should be at lesson level (not chapter level), user confirmed:

> "1. yes 2. yes 3. yes 4. yes 5. reflect in specs and record a new adr as well so we can track back source and see phr as well."

The user wanted:
1. Storage structure to mirror Docusaurus `docs/` convention (`content/` instead of `lessons/`)
2. Static assets to mirror Docusaurus `static/` convention
3. Summary tools removed (use content tools with `.summary.md` naming)
4. Summaries as sibling files to lessons (e.g., `01-intro.summary.md` for `01-intro.md`)

## Response snapshot

Created ADR-0018 and refactored PanaversityFS from 12 tools to 9 tools:

**Storage Structure Changes (ADR-0018):**

Before:
```
books/{book-id}/
├── lessons/
│   └── part-{n}/
│       └── chapter-{nn}/
│           └── lesson-{nn}.md
├── chapters/
│   └── chapter-{nn}/
│       └── .summary.md
└── assets/
    └── {type}/
```

After (Docusaurus-aligned):
```
books/{book-id}/
├── content/                   # Maps to Docusaurus docs/
│   └── {NN-Part}/
│       └── {NN-Chapter}/
│           ├── {NN-lesson}.md
│           └── {NN-lesson}.summary.md  # Sibling file
└── static/                    # Maps to Docusaurus static/
    └── {type}/
```

**Tool Changes:**
- Removed: `read_summary`, `write_summary`, `delete_summary`
- Content tools now handle summaries via `.summary.md` naming convention
- Total tools: 12 → 9

**Key design decisions:**
1. Summaries are sibling files to lessons (`.summary.md` suffix)
2. Mirrored Docusaurus directory structure for seamless integration
3. Single tool interface for all markdown content (lessons + summaries)
4. Simplified API surface without losing functionality

## Outcome

- Impact: API simplification (12 → 9 tools), Docusaurus alignment
- Tests: 45 passed (was 52, removed 7 summary-specific tests)
- Files: 16 files modified/created/deleted
- ADR: 0018-panaversityfs-docusaurus-aligned-structure.md
- Next prompts: Update PR comment with ADR-0018 summary
- Reflection: Aligning storage with target deployment platform (Docusaurus) simplifies integration and reduces cognitive overhead

## Evaluation notes (flywheel)

- Failure modes observed: Initial confusion about summary granularity (chapter vs lesson level) - clarified via user question
- Graders run and results (PASS/FAIL): pytest tests - PASS (45/45 tests)
- Prompt variant (if applicable): N/A
- Next experiment: Validate that Docusaurus plugin can directly sync from this structure without transformation
