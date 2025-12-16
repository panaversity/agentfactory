# Feature Specification: Book Chapter Reorganization

**Feature Branch**: `031-chapter-reorganization`
**Created**: 2025-11-26
**Status**: Draft
**Input**: User description: "Reorganize book chapters: delete Chapter 15, renumber Python chapters 16-33 to 15-32, update all assets and references per chapter-index.md"

## Overview

This feature reorganizes the book's file structure to match the updated chapter-index.md. The chapter-index.md has already been updated to reflect the new structure; this task synchronizes the physical filesystem and all references to match.

### Current State Analysis

**Physical Directory Structure (filesystem):**

- Part 4 (04-SDD-RI-Fundamentals): Contains chapters 13, 14, 15
- Part 5 (05-Python-Fundamentals): Contains chapters 16-33 (18 chapters)
- Static slides: chapter-01 through chapter-33 PDFs exist (no chapter-15)
- Static images: `book-source/static/img/part-5/` contains chapter-16 through chapter-33 directories
- Image references: Lesson content contains paths like `/img/part-5/chapter-16/filename.png`

**Target State (per chapter-index.md - already updated):**

- Part 4: Only chapters 13-14 (Chapter 15 deleted)
- Part 5: Chapters 15-32 (renumbered from 16-33, now 18 chapters numbered 15-32)
- All slide PDFs renumbered accordingly
- All image directories renumbered (chapter-16 → chapter-15, etc.)
- All image references in lesson content updated

### Change Summary

| Change Type        | From                                         | To                           | Count       |
| ------------------ | -------------------------------------------- | ---------------------------- | ----------- |
| Delete directory   | 15-ai-product-business-intelligence-capstone | (deleted)                    | 1           |
| Rename directory   | 16-python-uv-package-manager                 | 15-python-uv-package-manager | 1           |
| Rename directory   | 17-introduction-to-python                    | 16-introduction-to-python    | 1           |
| ...                | ...                                          | ...                          | ...         |
| Rename directory   | 33-cpython-gil                               | 32-cpython-gil               | 1           |
| Rename slides      | chapter-16-slides.pdf                        | chapter-15-slides.pdf        | 18          |
| Rename image dirs  | img/part-5/chapter-16                        | img/part-5/chapter-15        | 15 dirs     |
| Update image refs  | /img/part-5/chapter-16/                      | /img/part-5/chapter-15/      | ~30 refs    |
| Update frontmatter | sidebar_position: 16                         | sidebar_position: 15         | 18 chapters |

---

## User Scenarios & Testing _(mandatory)_

### User Story 1 - Book Reader Navigation (Priority: P1)

A reader navigating the book sees consistent chapter numbers throughout - in the table of contents, chapter titles, sidebar navigation, and slide references. Numbers match the chapter-index.md specification without gaps or duplicates.

**Why this priority**: This is the core deliverable - consistent numbering is essential for reader experience and Docusaurus build success.

**Independent Test**: Build the Docusaurus site and verify all chapters appear in correct order with correct titles in sidebar navigation.

**Acceptance Scenarios**:

1. **Given** the book is built with Docusaurus, **When** a reader views the sidebar, **Then** Part 5 shows chapters 15-32 in sequential order
2. **Given** a reader opens Chapter 15, **When** they view the page title, **Then** it shows "Chapter 15: Python UV — The Fastest Python Package Manager"
3. **Given** a reader clicks on Chapter 16 slides, **When** the PDF loads, **Then** it is the "Introduction to Python" slides (not the old UV slides)

---

### User Story 2 - Developer File Operations (Priority: P1)

A developer working in the codebase can find chapters by their correct number. Directory names match chapter numbers. All file operations (git, search, navigation) work correctly after reorganization.

**Why this priority**: Developer experience directly affects content maintenance and future contributions.

**Independent Test**: Run `ls apps/learn-app/docs/05-Python-Fundamentals/` and verify directories are numbered 15-32 with no 16-33 directories remaining.

**Acceptance Scenarios**:

1. **Given** the reorganization is complete, **When** I list Part 5 directories, **Then** I see 15-python-uv-package-manager through 32-cpython-gil
2. **Given** the reorganization is complete, **When** I search for "Chapter 16", **Then** results show "Introduction to Python" content (not UV content)
3. **Given** git status after reorganization, **When** I review changes, **Then** all changes are in a single commit with clear message

---

### User Story 3 - Slide Asset Consistency (Priority: P2)

All slide PDFs are renamed to match their new chapter numbers. README frontmatter slide references point to correct files.

**Why this priority**: Slides are supporting assets; incorrect references cause 404 errors but don't break core navigation.

**Independent Test**: Verify each chapter's README.md `slides.source` field points to an existing PDF with matching chapter number.

**Acceptance Scenarios**:

1. **Given** Chapter 15 README, **When** I check slides.source, **Then** it references "slides/chapter-15-slides.pdf" and that file exists
2. **Given** the old chapter-16-slides.pdf, **When** I look for it, **Then** it has been renamed to chapter-15-slides.pdf
3. **Given** any chapter README in Part 5, **When** I check sidebar_position, **Then** it matches the directory number (15-32)

---

### User Story 4 - Image Asset Consistency (Priority: P2)

All image directories in `book-source/static/img/part-5/` are renamed to match new chapter numbers. All image references in lesson markdown files point to correct paths.

**Why this priority**: Broken images degrade reader experience but don't break navigation or build.

**Independent Test**: Grep for `/img/part-5/chapter-16` in lesson content - should return zero matches after reorganization.

**Acceptance Scenarios**:

1. **Given** lesson content in Chapter 15 (formerly 16), **When** I view images, **Then** they load correctly from `/img/part-5/chapter-15/`
2. **Given** the old `img/part-5/chapter-16/` directory, **When** I look for it, **Then** it has been renamed to `chapter-15/`
3. **Given** any lesson file in Part 5, **When** I check image references, **Then** all paths use updated chapter numbers (15-32)

---

### Edge Cases

- **Missing slides**: chapter-15-slides.pdf doesn't currently exist (Chapter 15 was Capstone with no slides). After deletion, no gap exists since Python UV becomes Chapter 15.
- **Cross-references**: Content that references "Chapter 16" by number (not by topic) needs updating to "Chapter 15".
- **Part 4 README**: References to "Chapters 13-15" must update to "Chapters 13-14".
- **Quiz file naming**: Quiz files like `08_chapter_15_quiz.md` inside deleted chapter must be removed, not renamed.
- **Sparse image directories**: Not all chapters have image directories (only 15 of 18 chapters have images). Rename only existing directories.
- **Image reference in content**: Lesson markdown files contain hardcoded paths like `/img/part-5/chapter-NN/`. These must be updated via find-replace.

---

## Requirements _(mandatory)_

### Functional Requirements

**Directory Operations:**

- **FR-001**: System MUST delete the directory `04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/` and all its contents
- **FR-002**: System MUST rename all Part 5 chapter directories from `NN-*` to `(NN-1)-*` for chapters 16-33 → 15-32
- **FR-003**: System MUST preserve all file contents within renamed directories unchanged (only directory names change)

**README Frontmatter Updates:**

- **FR-004**: System MUST update `sidebar_position` in each Part 5 chapter README from N to N-1 (16→15, 17→16, ..., 33→32)
- **FR-005**: System MUST update chapter titles in README from "Chapter N:" to "Chapter N-1:" (16→15, etc.)
- **FR-006**: System MUST update `slides.source` references from "chapter-N-slides.pdf" to "chapter-(N-1)-slides.pdf"
- **FR-007**: System MUST update `slides.title` references to match new chapter numbers

**Slide Asset Operations:**

- **FR-008**: System MUST rename slide PDFs in `book-source/static/slides/` from chapter-N to chapter-(N-1) for N=16-33
- **FR-009**: System MUST NOT create or modify chapter-14-slides.pdf (stays as-is)

**Image Asset Operations:**

- **FR-015**: System MUST rename image directories in `book-source/static/img/part-5/` from chapter-N to chapter-(N-1) for existing directories (N=16-33)
- **FR-016**: System MUST update all image references in Part 5 lesson markdown files from `/img/part-5/chapter-N/` to `/img/part-5/chapter-(N-1)/`
- **FR-017**: System MUST preserve all image files within renamed directories unchanged (only directory names change)

**Part README Updates:**

- **FR-010**: System MUST update Part 4 README to remove references to Chapter 15 (now only covers Chapters 13-14)
- **FR-011**: System MUST update Part 5 README if it contains chapter number references

**Validation:**

- **FR-012**: System MUST verify all renamed files exist after operation completes
- **FR-013**: System MUST verify no orphaned references exist (grep for old chapter numbers)
- **FR-014**: All changes MUST be committed as a single atomic git commit

### Key Entities

- **Chapter Directory**: Directory containing a chapter's lessons, README, and quiz. Named `NN-chapter-slug/`
- **Chapter README**: Frontmatter with `sidebar_position`, `title`, `slides` object. Located at `NN-chapter-slug/README.md`
- **Slide PDF**: Binary PDF file. Named `chapter-NN-slides.pdf` in `book-source/static/slides/`
- **Image Directory**: Directory containing chapter images. Named `chapter-NN/` in `book-source/static/img/part-N/`
- **Image Reference**: Markdown image path in lesson content. Format: `/img/part-5/chapter-NN/filename.png`
- **Chapter Index**: Source of truth for chapter numbering. Located at `specs/book/chapter-index.md`

---

## Success Criteria _(mandatory)_

### Measurable Outcomes

- **SC-001**: Docusaurus build completes successfully with zero errors
- **SC-002**: Part 5 sidebar shows exactly 18 chapters numbered 15-32 in sequential order
- **SC-003**: All 18 Part 5 chapter directories exist with correct numbering (15-32)
- **SC-004**: All 18 renamed slide PDFs exist (chapter-15 through chapter-32)
- **SC-005**: Zero grep matches for "Chapter 16" through "Chapter 33" in Part 5 READMEs (all updated to 15-32)
- **SC-006**: Zero grep matches for "chapter-16-slides" through "chapter-33-slides" in Part 5 READMEs (all updated)
- **SC-007**: Part 4 contains exactly 2 chapter directories (13 and 14)
- **SC-008**: Single git commit contains all changes with descriptive message
- **SC-009**: Zero dangling references to deleted Chapter 15 content
- **SC-010**: All Part 5 image directories renamed (chapter-15 through chapter-32 where applicable)
- **SC-011**: Zero grep matches for `/img/part-5/chapter-16` through `/img/part-5/chapter-33` in lesson content (all updated)
- **SC-012**: All images render correctly when Docusaurus site is built and viewed

---

## Constraints

- **Single Commit**: All changes must be atomic - no partial states that break the build
- **Minimal Content Changes**: Only structural changes (renames, frontmatter) and image path references. Lesson prose unchanged.
- **Preserve Git History**: Use git mv for directory renames to preserve file history
- **chapter-index.md Already Updated**: The index is the source of truth - we are synchronizing filesystem to match it

## Non-Goals

- Updating lesson prose content (only image path references are updated, not explanatory text)
- Creating new chapters or lessons
- Modifying chapter-index.md (already correct)
- Updating cross-references that mention chapter numbers in prose (e.g., "as we learned in Chapter 16" in body text)
- Renumbering Parts 1-4 or Parts 6-12 (only Part 5 chapters affected)
- Modifying image file contents (only directory names and path references change)

## Assumptions

- The chapter-index.md reflects the authoritative target state
- No external systems depend on the current chapter numbers (URLs may change)
- Slide PDFs are standalone files with no internal cross-references needing updates
- Git history preservation via `git mv` is preferred but not strictly required

## Risks

- **Build Breakage**: If any frontmatter is malformed, Docusaurus build fails. Mitigation: Validate YAML after each edit.
- **Missing References**: Cross-references in lesson content might use hard-coded numbers. Mitigation: Grep validation in SC-005/SC-006/SC-011.
- **Slide Mismatch**: If a slide title says "Chapter 16" internally, it won't match. Mitigation: Out of scope (PDF content not editable).
- **Broken Images**: If image path references are missed, images won't load. Mitigation: Comprehensive grep for all `/img/part-5/chapter-` patterns.
