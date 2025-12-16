# Implementation Plan: Book Chapter Reorganization

**Branch**: `031-chapter-reorganization` | **Date**: 2025-11-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/031-chapter-reorganization/spec.md`

## Summary

Reorganize book filesystem to match updated chapter-index.md:

- Delete Chapter 15 (Capstone) from Part 4
- Renumber Part 5 Python chapters 16-33 → 15-32
- Update all assets (slides, images) and references (frontmatter, image paths)
- Commit as single atomic change

## Technical Context

**Language/Version**: Bash scripts, Markdown files, YAML frontmatter
**Primary Dependencies**: Git, Docusaurus 3.x (for build validation)
**Storage**: Filesystem (book-source directory)
**Testing**: Grep validation scripts, Docusaurus build
**Target Platform**: GitHub repository, Docusaurus static site
**Project Type**: Documentation/Content reorganization
**Performance Goals**: N/A (one-time operation)
**Constraints**: Single atomic commit, preserve git history where possible
**Scale/Scope**: ~200 file operations (18 dir renames, 18 slide renames, 15 image dir renames, ~150 file edits)

## Constitution Check

_GATE: This is an infrastructure task, not educational content. Constitution principles apply minimally._

| Principle                 | Applies | Status                                     |
| ------------------------- | ------- | ------------------------------------------ |
| Specification Primacy     | No      | N/A - file reorganization                  |
| Progressive Complexity    | No      | N/A - not teaching content                 |
| Factual Accuracy          | No      | N/A - structural changes only              |
| Coherent Structure        | Yes     | PASS - reorganization improves structure   |
| Intelligence Accumulation | No      | N/A - not creating reusable intelligence   |
| Anti-Convergence          | No      | N/A - not educational content              |
| Minimal Content           | Yes     | PASS - minimal changes, only what's needed |

**Gate Status**: PASS - Infrastructure task with minimal constitution overlap

## Project Structure

### Files Affected

```text
book-source/
├── docs/
│   ├── 04-SDD-RI-Fundamentals/
│   │   ├── 15-ai-product-business-intelligence-capstone/  # DELETE
│   │   └── README.md                                       # UPDATE refs
│   └── 05-Python-Fundamentals/
│       ├── 16-python-uv-package-manager/    → 15-python-uv-package-manager/
│       ├── 17-introduction-to-python/       → 16-introduction-to-python/
│       ├── 18-data-types/                   → 17-data-types/
│       ├── ... (15 more directories)
│       ├── 33-cpython-gil/                  → 32-cpython-gil/
│       └── README.md                        # UPDATE if needed
├── static/
│   ├── slides/
│   │   ├── chapter-16-slides.pdf → chapter-15-slides.pdf
│   │   ├── chapter-17-slides.pdf → chapter-16-slides.pdf
│   │   └── ... (18 total PDFs)
│   └── img/
│       └── part-5/
│           ├── chapter-16/ → chapter-15/
│           ├── chapter-18/ → chapter-17/
│           └── ... (15 existing directories)
```

### Operation Sequence

Operations must be performed in this order to avoid conflicts:

1. **Phase 1: Delete Chapter 15** (Part 4)

   - Remove `04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/`

2. **Phase 2: Rename Part 5 Directories** (reverse order to avoid conflicts)

   - Rename from highest to lowest: 33→32, 32→31, ..., 16→15
   - Must go in reverse to prevent `16→15` blocking `17→16`

3. **Phase 3: Update README Frontmatter**

   - Update `sidebar_position` in each renamed chapter
   - Update `title` with new chapter numbers
   - Update `slides.source` and `slides.title` references

4. **Phase 4: Rename Slide PDFs** (reverse order)

   - Rename from highest to lowest: chapter-33→32, ..., chapter-16→15

5. **Phase 5: Rename Image Directories** (reverse order)

   - Rename existing directories only (sparse - 15 of 18 chapters)

6. **Phase 6: Update Image References in Content**

   - Find-replace `/img/part-5/chapter-N/` → `/img/part-5/chapter-(N-1)/`
   - For each N from 16-33

7. **Phase 7: Update Part READMEs**

   - Part 4 README: Remove Chapter 15 references
   - Part 5 README: Update chapter range if mentioned

8. **Phase 8: Validation**

   - Grep for orphaned references
   - Verify file counts
   - Run Docusaurus build

9. **Phase 9: Commit**
   - Single atomic git commit

---

## Detailed Mapping

### Chapter Directory Renames (18 operations)

| Old Path                                                  | New Path                                                  | Old # | New # |
| --------------------------------------------------------- | --------------------------------------------------------- | ----- | ----- |
| `05-Python-Fundamentals/16-python-uv-package-manager/`    | `05-Python-Fundamentals/15-python-uv-package-manager/`    | 16    | 15    |
| `05-Python-Fundamentals/17-introduction-to-python/`       | `05-Python-Fundamentals/16-introduction-to-python/`       | 17    | 16    |
| `05-Python-Fundamentals/18-data-types/`                   | `05-Python-Fundamentals/17-data-types/`                   | 18    | 17    |
| `05-Python-Fundamentals/19-operators-keywords-variables/` | `05-Python-Fundamentals/18-operators-keywords-variables/` | 19    | 18    |
| `05-Python-Fundamentals/20-strings-type-casting/`         | `05-Python-Fundamentals/19-strings-type-casting/`         | 20    | 19    |
| `05-Python-Fundamentals/21-control-flow-loops/`           | `05-Python-Fundamentals/20-control-flow-loops/`           | 21    | 20    |
| `05-Python-Fundamentals/22-lists-tuples-dictionary/`      | `05-Python-Fundamentals/21-lists-tuples-dictionary/`      | 22    | 21    |
| `05-Python-Fundamentals/23-set-frozenset-gc/`             | `05-Python-Fundamentals/22-set-frozenset-gc/`             | 23    | 22    |
| `05-Python-Fundamentals/24-module-functions/`             | `05-Python-Fundamentals/23-module-functions/`             | 24    | 23    |
| `05-Python-Fundamentals/25-exception-handling/`           | `05-Python-Fundamentals/24-exception-handling/`           | 25    | 24    |
| `05-Python-Fundamentals/26-io-file-handling/`             | `05-Python-Fundamentals/25-io-file-handling/`             | 26    | 25    |
| `05-Python-Fundamentals/27-math-datetime-calendar/`       | `05-Python-Fundamentals/26-math-datetime-calendar/`       | 27    | 26    |
| `05-Python-Fundamentals/28-oop-part-1/`                   | `05-Python-Fundamentals/27-oop-part-1/`                   | 28    | 27    |
| `05-Python-Fundamentals/29-oop-part-2/`                   | `05-Python-Fundamentals/28-oop-part-2/`                   | 29    | 28    |
| `05-Python-Fundamentals/30-metaclasses-dataclasses/`      | `05-Python-Fundamentals/29-metaclasses-dataclasses/`      | 30    | 29    |
| `05-Python-Fundamentals/31-pydantic-generics/`            | `05-Python-Fundamentals/30-pydantic-generics/`            | 31    | 30    |
| `05-Python-Fundamentals/32-asyncio/`                      | `05-Python-Fundamentals/31-asyncio/`                      | 32    | 31    |
| `05-Python-Fundamentals/33-cpython-gil/`                  | `05-Python-Fundamentals/32-cpython-gil/`                  | 33    | 32    |

### Slide PDF Renames (18 operations)

| Old Filename            | New Filename            |
| ----------------------- | ----------------------- |
| `chapter-16-slides.pdf` | `chapter-15-slides.pdf` |
| `chapter-17-slides.pdf` | `chapter-16-slides.pdf` |
| `chapter-18-slides.pdf` | `chapter-17-slides.pdf` |
| `chapter-19-slides.pdf` | `chapter-18-slides.pdf` |
| `chapter-20-slides.pdf` | `chapter-19-slides.pdf` |
| `chapter-21-slides.pdf` | `chapter-20-slides.pdf` |
| `chapter-22-slides.pdf` | `chapter-21-slides.pdf` |
| `chapter-23-slides.pdf` | `chapter-22-slides.pdf` |
| `chapter-24-slides.pdf` | `chapter-23-slides.pdf` |
| `chapter-25-slides.pdf` | `chapter-24-slides.pdf` |
| `chapter-26-slides.pdf` | `chapter-25-slides.pdf` |
| `chapter-27-slides.pdf` | `chapter-26-slides.pdf` |
| `chapter-28-slides.pdf` | `chapter-27-slides.pdf` |
| `chapter-29-slides.pdf` | `chapter-28-slides.pdf` |
| `chapter-30-slides.pdf` | `chapter-29-slides.pdf` |
| `chapter-31-slides.pdf` | `chapter-30-slides.pdf` |
| `chapter-32-slides.pdf` | `chapter-31-slides.pdf` |
| `chapter-33-slides.pdf` | `chapter-32-slides.pdf` |

### Image Directory Renames (15 operations - sparse)

| Old Path                 | New Path                 | Exists |
| ------------------------ | ------------------------ | ------ |
| `img/part-5/chapter-16/` | `img/part-5/chapter-15/` | ✓      |
| `img/part-5/chapter-17/` | `img/part-5/chapter-16/` | ✗      |
| `img/part-5/chapter-18/` | `img/part-5/chapter-17/` | ✓      |
| `img/part-5/chapter-19/` | `img/part-5/chapter-18/` | ✓      |
| `img/part-5/chapter-20/` | `img/part-5/chapter-19/` | ✗      |
| `img/part-5/chapter-21/` | `img/part-5/chapter-20/` | ✓      |
| `img/part-5/chapter-22/` | `img/part-5/chapter-21/` | ✓      |
| `img/part-5/chapter-23/` | `img/part-5/chapter-22/` | ✓      |
| `img/part-5/chapter-24/` | `img/part-5/chapter-23/` | ✓      |
| `img/part-5/chapter-25/` | `img/part-5/chapter-24/` | ✗      |
| `img/part-5/chapter-26/` | `img/part-5/chapter-25/` | ✓      |
| `img/part-5/chapter-27/` | `img/part-5/chapter-26/` | ✓      |
| `img/part-5/chapter-28/` | `img/part-5/chapter-27/` | ✓      |
| `img/part-5/chapter-29/` | `img/part-5/chapter-28/` | ✓      |
| `img/part-5/chapter-30/` | `img/part-5/chapter-29/` | ✓      |
| `img/part-5/chapter-31/` | `img/part-5/chapter-30/` | ✓      |
| `img/part-5/chapter-32/` | `img/part-5/chapter-31/` | ✓      |
| `img/part-5/chapter-33/` | `img/part-5/chapter-32/` | ✓      |

### README Frontmatter Updates (18 files)

For each Part 5 chapter README.md:

```yaml
# Before (example for chapter 16→15)
---
sidebar_position: 16
title: "Chapter 16: Python UV — The Fastest Python Package Manager"
slides:
  source: "slides/chapter-16-slides.pdf"
  title: "Chapter 16: Python UV — The Fastest Python Package Manager"
---
# After
---
sidebar_position: 15
title: "Chapter 15: Python UV — The Fastest Python Package Manager"
slides:
  source: "slides/chapter-15-slides.pdf"
  title: "Chapter 15: Python UV — The Fastest Python Package Manager"
---
```

### Image Reference Updates (~30 files)

Pattern replacement in all Part 5 lesson files:

| Find Pattern              | Replace With              |
| ------------------------- | ------------------------- |
| `/img/part-5/chapter-16/` | `/img/part-5/chapter-15/` |
| `/img/part-5/chapter-17/` | `/img/part-5/chapter-16/` |
| `/img/part-5/chapter-18/` | `/img/part-5/chapter-17/` |
| ...                       | ...                       |
| `/img/part-5/chapter-33/` | `/img/part-5/chapter-32/` |

---

## Validation Scripts

### Pre-Implementation Verification

```bash
# Verify current state matches expectations
ls apps/learn-app/docs/04-SDD-RI-Fundamentals/ | grep "^1[345]-"
# Expected: 13-, 14-, 15-

ls apps/learn-app/docs/05-Python-Fundamentals/ | grep "^[0-9]" | wc -l
# Expected: 18

ls book-source/static/slides/ | grep "chapter-" | wc -l
# Expected: ~33 (chapters 1-33 minus gaps)
```

### Post-Implementation Verification

```bash
# SC-003: Part 5 directories numbered 15-32
ls apps/learn-app/docs/05-Python-Fundamentals/ | grep "^1[5-9]-\|^2[0-9]-\|^3[0-2]-" | wc -l
# Expected: 18

# SC-004: Slide PDFs exist
for i in $(seq 15 32); do
  ls book-source/static/slides/chapter-$i-slides.pdf 2>/dev/null || echo "MISSING: chapter-$i-slides.pdf"
done

# SC-005: No old chapter numbers in Part 5 READMEs
grep -r "Chapter 1[6-9]\|Chapter 2[0-9]\|Chapter 3[0-3]" apps/learn-app/docs/05-Python-Fundamentals/*/README.md
# Expected: 0 matches

# SC-006: No old slide references
grep -r "chapter-1[6-9]-slides\|chapter-2[0-9]-slides\|chapter-3[0-3]-slides" apps/learn-app/docs/05-Python-Fundamentals/
# Expected: 0 matches

# SC-007: Part 4 has exactly 2 chapters
ls apps/learn-app/docs/04-SDD-RI-Fundamentals/ | grep "^1[34]-" | wc -l
# Expected: 2

# SC-010: Image directories exist
ls book-source/static/img/part-5/ | grep "chapter-" | wc -l
# Expected: 15 (same count, different numbers)

# SC-011: No old image references
grep -r "/img/part-5/chapter-1[6-9]\|/img/part-5/chapter-2[0-9]\|/img/part-5/chapter-3[0-3]" apps/learn-app/docs/05-Python-Fundamentals/
# Expected: 0 matches

# SC-001: Docusaurus build
cd book-source && npm run build
# Expected: success
```

---

## Complexity Tracking

No complexity violations - this is a straightforward file reorganization task.

| Aspect          | Assessment                                     |
| --------------- | ---------------------------------------------- |
| Operation Count | ~200 operations (acceptable for one-time task) |
| Risk Level      | Medium (many files, but pattern-based)         |
| Rollback        | Easy (single commit can be reverted)           |
| Testing         | Comprehensive (grep + build validation)        |

---

## Next Steps

1. Run `/sp.tasks` to generate actionable task checklist
2. Execute tasks in specified order
3. Run validation scripts
4. Create single atomic commit
5. Run Docusaurus build to verify
