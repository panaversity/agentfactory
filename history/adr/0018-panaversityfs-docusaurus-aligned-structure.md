# ADR 0018: PanaversityFS Docusaurus-Aligned Storage Structure

**Status**: Accepted
**Date**: 2025-11-25
**Deciders**: MJS, Claude Code
**Context**: PanaversityFS storage structure redesign for Docusaurus compatibility

---

## Context

### Problem Statement

The original PanaversityFS storage structure did not mirror the Docusaurus book source structure:

**Original PanaversityFS Structure**:
```
books/{book-id}/
├── lessons/
│   └── part-1/
│       └── chapter-01/
│           ├── lesson-01.md
│           └── lesson-02.md
├── chapters/
│   └── chapter-01/
│       └── .summary.md
└── assets/
    ├── images/
    └── slides/
```

**Actual Docusaurus Structure** (`book-source/docs/`):
```
01-Introducing-AI-Driven-Development/
├── README.md
├── 01-what-is-ai-native/
│   └── README.md
├── 02-why-it-matters/
│   └── README.md
└── ...
```

**Problems**:
1. Different naming conventions (`part-1` vs `01-Part-Name`)
2. Lessons separated from chapters (different directories)
3. Summaries at chapter level, not lesson level
4. Assets at book level, not mirroring `static/` structure
5. Extra summary tools (3 tools) for what is just a file naming convention

### Design Goals

1. **1:1 mapping** with Docusaurus structure for easy sync/migration
2. **Reduce tool count** by eliminating redundant summary-specific tools
3. **Self-describing paths** using actual part/chapter/lesson names
4. **Assets mirror** Docusaurus `static/` directory structure

---

## Decision

### 1. Adopt Docusaurus-Aligned Storage Structure

**New Structure**:
```
data/books/{book-id}/
│
├── content/                                    # Maps to docs/
│   ├── 01-Introducing-AI-Driven-Development/   # Part
│   │   ├── README.md                           # Part intro
│   │   ├── 01-what-is-ai-native/               # Chapter
│   │   │   ├── README.md                       # Chapter intro
│   │   │   ├── 01-lesson-one.md                # Lesson
│   │   │   ├── 01-lesson-one.summary.md        # Lesson summary (sibling)
│   │   │   ├── 02-lesson-two.md
│   │   │   └── 02-lesson-two.summary.md
│   │   └── 02-another-chapter/
│   │       └── ...
│   └── 02-AI-Tool-Landscape/
│       └── ...
│
├── static/                                     # Maps to static/
│   ├── img/                                    # Images organized by part
│   │   ├── 01-introducing/
│   │   │   └── diagram.png
│   │   └── 02-tool-landscape/
│   │       └── screenshot.png
│   └── slides/                                 # Slide decks
│
└── book.yaml                                   # Book metadata
```

### 2. Remove Summary Tools (12 → 9 tools)

**Before (12 tools)**:
- Content: `read_content`, `write_content`, `delete_content`
- Summary: `read_summary`, `write_summary`, `delete_summary` ← **REMOVE**
- Assets: `upload_asset`, `get_asset`, `list_assets`
- Search: `glob_search`, `grep_search`
- Registry: `list_books`
- Bulk: `get_book_archive`

**After (9 tools)**:
- Content: `read_content`, `write_content`, `delete_content`
- Assets: `upload_asset`, `get_asset`, `list_assets`
- Search: `glob_search`, `grep_search`
- Registry: `list_books`
- Bulk: `get_book_archive`

**Rationale**: Summaries are just markdown files with `.summary.md` suffix. The existing content tools handle them:
```python
# Write lesson
write_content(path="content/01-part/01-chapter/01-lesson.md", ...)

# Write its summary (same tool!)
write_content(path="content/01-part/01-chapter/01-lesson.summary.md", ...)
```

### 3. Summary Naming Convention

- **Format**: `{lesson-name}.summary.md` (sibling to lesson file)
- **Location**: Same directory as the lesson
- **Example**: `01-lesson-one.md` → `01-lesson-one.summary.md`
- **Visibility**: Not hidden (no leading dot) for easier discovery

---

## Consequences

### Positive

1. **Simplified sync**: Content structure matches Docusaurus exactly
2. **Fewer tools**: 9 tools instead of 12 (less code, simpler API)
3. **Self-documenting paths**: `/content/01-Introducing-AI-Driven-Development/01-what-is-ai-native/01-lesson.md` is human-readable
4. **Asset organization**: Mirrors `static/` structure from Docusaurus
5. **No special summary logic**: Just files with a naming convention

### Negative

1. **Breaking change**: Requires migration of any existing data
2. **Path length**: Longer paths than numeric-only structure
3. **Test updates**: All 52 tests need path updates

### Neutral

1. **Summary discovery**: Use `glob_search(pattern="**/*.summary.md")` to find all summaries
2. **Migration tool needed**: Must update `poc_migrate_chapter.py` for new structure

---

## Implementation Impact

### Files to Update

| File | Change |
|------|--------|
| `tools/content.py` | Update path construction: `content/{path}` |
| `tools/assets.py` | Update path: `static/{type}/{path}` |
| `tools/summaries.py` | **DELETE** entire file |
| `tools/search.py` | Update base paths |
| `tools/bulk.py` | Update archive paths |
| `models.py` | Remove `ReadSummaryInput`, `WriteSummaryInput`, `DeleteSummaryInput` |
| `server.py` | Remove summary tool imports |
| All tests | Update paths and remove summary tests |
| `docs/*.md` | Update documentation |
| `spec.md` | Update FR-005, remove FR-014/15/16 |

### Migration Path

1. Update spec (this ADR documents the decision)
2. Refactor tools to new paths
3. Remove summary tools
4. Update all tests
5. Update documentation
6. No data migration needed (fresh start, no production data yet)

---

## References

- **Docusaurus structure**: `book-source/docs/` and `book-source/static/`
- **Previous spec**: `specs/030-panaversity-fs/spec.md` (FR-014, FR-015, FR-016)
- **PHR**: `history/prompts/030-panaversity-fs/0004-docusaurus-aligned-structure.refactor.prompt.md`
