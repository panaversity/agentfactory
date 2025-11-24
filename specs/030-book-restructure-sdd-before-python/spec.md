# Feature Specification: Book Restructure - SDD-RI Before Python

**Feature Branch**: `030-book-restructure-sdd-before-python`
**Created**: 2025-01-24
**Status**: Draft
**Input**: User description: "Restructure book to introduce SDD-RI BEFORE Python (Part 4: 3 chapters on SDD-RI fundamentals + business thinking, Part 6: 3 chapters on AI Product Leadership). Move current chapters 31-33 to new positions, create placeholders for new chapters, update chapter-index.md. This is Phase 1: Structure & Placeholders only - no content creation yet."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Update Book Structure Metadata (Priority: P1)

As a book maintainer, I need the chapter-index.md file updated to reflect the new structure so that all team members and tools understand the new chapter organization before any content work begins.

**Why this priority**: Foundation for all other work - without correct metadata, subsequent chapters and content cannot be properly organized.

**Independent Test**: Can verify by reading chapter-index.md and confirming all chapters are correctly numbered and positioned in their new parts.

**Acceptance Scenarios**:

1. **Given** the current chapter-index.md with Parts 4-5 structure, **When** I update it with the new structure, **Then** Part 4 contains chapters 13-15 (SDD-RI Fundamentals), Part 5 contains chapters 16-33 (Python), and Part 6 contains chapters 34-36 (AI Product Leadership)
2. **Given** the updated chapter-index.md, **When** I search for chapter 31, **Then** it no longer exists at that number; the content appears as Chapter 13 in Part 4
3. **Given** the updated chapter-index.md, **When** I review implementation status markers, **Then** existing content shows ✅ with correct new numbers, new placeholders show 📋 NEW

---

### User Story 2 - Move Existing Chapter Directories (Priority: P2)

As a book maintainer, I need existing chapter directories (31-33) moved to their new numerical positions (13, 14, 35) so that the file structure matches the updated chapter-index.md.

**Why this priority**: Ensures file system consistency with metadata before creating new content.

**Independent Test**: Can verify by listing directories under book-source/docs/ and confirming old chapter numbers no longer exist and new numbers contain the moved content.

**Acceptance Scenarios**:

1. **Given** current chapter 31 directory at `book-source/docs/05-Spec-Driven-Development/31-specification-driven-development-fundamentals/`, **When** I move it to position 13, **Then** it exists at `book-source/docs/04-SDD-RI-Fundamentals/13-specification-driven-development-fundamentals/` with all lesson files intact
2. **Given** current chapter 32 directory, **When** I move it to position 14, **Then** it exists in new location with README.md updated to reflect position 14
3. **Given** current chapter 33 directory (AI Orchestra), **When** I move it to position 35, **Then** it exists in Part 6 directory structure at `book-source/docs/06-AI-Product-Leadership/35-ai-orchestra-agent-teams-manager/`

---

### User Story 3 - Create Placeholder READMEs for New Chapters (Priority: P3)

As a book maintainer, I need placeholder README.md files for chapters 15, 34, and 36 so that the structure is complete and ready for future content implementation.

**Why this priority**: Completes the structural foundation, but doesn't block understanding of the reorganization.

**Independent Test**: Can verify by checking that each new chapter directory contains a README.md following the established placeholder pattern (similar to current Chapter 34).

**Acceptance Scenarios**:

1. **Given** Chapter 15 needs to be created (AI Product & Business Intelligence + Capstone), **When** I create the placeholder, **Then** `book-source/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/README.md` exists with correct YAML frontmatter (sidebar_position: 15, title, slides metadata)
2. **Given** Chapter 34 needs to be created (AI Product Development & Evaluation-First), **When** I create the placeholder, **Then** it exists in Part 6 directory with proper frontmatter and placeholder content explaining the chapter's purpose
3. **Given** Chapter 36 needs to be created (Product & Engineering Leadership), **When** I create the placeholder, **Then** it follows the same pattern as other placeholders with descriptive overview content

---

### User Story 4 - Rename All Slide PDFs and Images (Priority: P1)

As a book maintainer, I need **all 21 affected slide PDFs and ~18 image directories** renamed/moved to match new chapter numbers so that Docusaurus displays correct assets and all links remain valid.

**Why this priority**: Critical infrastructure change - broken asset links break site navigation and visual content. Must be done atomically with chapter moves.

**Independent Test**: Can verify by listing `book-source/static/slides/` and `book-source/static/img/` directories, confirming no files with old numbering remain, and validating YAML frontmatter references match actual files.

**Acceptance Scenarios**:

1. **Given** 18 Python slide PDFs (chapter-13 through chapter-30), **When** I rename them with +3 shift, **Then** they exist as chapter-16 through chapter-33 slides with git history preserved
2. **Given** SDD chapters 31-32 slide PDFs, **When** I rename them to 13-14, **Then** `book-source/static/slides/chapter-13-slides.pdf` and `chapter-14-slides.pdf` exist with git history
3. **Given** ~15 Python image directories in `part-4/chapter-*/`, **When** I move them to `part-5/` with +3 numbering shift, **Then** all images exist at new paths and old paths no longer exist
4. **Given** all 21 chapter READMEs with moved/renumbered content, **When** I update YAML frontmatter `slides.source`, **Then** every README references the correct chapter-N-slides.pdf that actually exists
5. **Given** lesson content files with image markdown `![](/img/part-4/chapter-X/)`, **When** I update references to new part/chapter paths, **Then** all images render correctly in Docusaurus preview

---

### User Story 5 - Update Part Folder Names (Priority: P2)

As a book maintainer, I need part folder names updated to reflect the new structure so that directories match the logical organization described in chapter-index.md.

**Why this priority**: Ensures directory structure semantic clarity and prevents confusion.

**Independent Test**: Can verify by listing book-source/docs/ directories and confirming folder names match part descriptions.

**Acceptance Scenarios**:

1. **Given** current `04-Python-Fundamentals/` folder, **When** I rename it, **Then** it becomes `05-Python-Fundamentals/` with all chapter subdirectories intact
2. **Given** current `05-Spec-Driven-Development/` folder, **When** I rename it, **Then** it becomes `04-SDD-RI-Fundamentals/`
3. **Given** no current Part 6 folder, **When** I create it, **Then** `06-AI-Product-Leadership/` exists and contains chapters 34, 35, 36

---

### Edge Cases

**Directory & Git Operations:**
- What happens if chapter directory moves fail mid-operation? (Solution: Document rollback procedure in tasks.md, use git operations which are atomic)
- How do we handle existing git history for moved directories? (Solution: Use `git mv` to preserve history)
- What if part folder renumbering conflicts with existing content? (Solution: Perform moves in correct sequence - rename parts first, then move chapters)
- What if Docusaurus build fails after restructuring? (Solution: Test build after each major change; rollback if necessary)

**Plugin & Build System:**
- **CRITICAL**: What happens to `remark-interactive-python` plugin with hardcoded path `/04-Python-Fundamentals/`? (Solution: Must update docusaurus.config.ts and plugin index.js to `/05-Python-Fundamentals/`)
- What if custom plugins extract chapter numbers from directory names? (Solution: Verify og-image-generator and structured-data plugins don't parse directory numbers)

**Content References:**
- What happens to 315+ cross-references in lesson files mentioning "Chapter 13-33"? (Solution: **MOVED TO PHASE 1** - automated grep-and-replace updates all narrative chapter mentions)
- What happens to internal markdown links like `[link](/docs/04-Python/.../)`? (Solution: Docusaurus uses relative links and `sidebar_position`, so YAML updates sufficient)

**Asset Hosting:**
- ~~What if slides are hosted on CDN/S3?~~ **RESOLVED**: All slides are local-only (confirmed by user)
- What happens to generated sitemaps and SEO metadata after URL changes? (Solution: Docusaurus regenerates automatically during build based on new structure)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST update specs/book/chapter-index.md with new 86-chapter structure (was 84 chapters, +2 from Part 4 expansion)
- **FR-002**: System MUST move `book-source/docs/05-Spec-Driven-Development/31-specification-driven-development-fundamentals/` to `book-source/docs/04-SDD-RI-Fundamentals/13-specification-driven-development-fundamentals/` using `git mv`
- **FR-003**: System MUST move `book-source/docs/05-Spec-Driven-Development/32-spec-kit-plus-hands-on/` to `book-source/docs/04-SDD-RI-Fundamentals/14-youtube-content-workflow-gemini/` using `git mv` (note: directory rename reflects future content pivot)
- **FR-004**: System MUST move `book-source/docs/05-Spec-Driven-Development/33-ai-orchestra-agent-teams-manager/` to `book-source/docs/06-AI-Product-Leadership/35-ai-orchestra-agent-teams-manager/` using `git mv`
- **FR-005**: System MUST rename `book-source/docs/04-Python-Fundamentals/` to `book-source/docs/05-Python-Fundamentals/` using `git mv`
- **FR-006**: System MUST rename `book-source/docs/05-Spec-Driven-Development/` to `book-source/docs/04-SDD-RI-Fundamentals/` using `git mv`
- **FR-007**: System MUST create `book-source/docs/06-AI-Product-Leadership/` directory structure
- **FR-008**: System MUST create placeholder README.md for Chapter 15 at `book-source/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/README.md`
- **FR-009**: System MUST create placeholder README.md for Chapter 34 at `book-source/docs/06-AI-Product-Leadership/34-ai-product-development-evaluation-first/README.md`
- **FR-010**: System MUST create placeholder README.md for Chapter 36 at `book-source/docs/06-AI-Product-Leadership/36-product-engineering-leadership-ai/README.md`
- **FR-011**: System MUST update YAML frontmatter in moved chapter README.md files to reflect new sidebar_position numbers (13, 14, 35)
- **FR-012**: System MUST preserve all existing lesson content files within moved chapter directories unchanged
- **FR-013**: System MUST use `git mv` command for all directory moves to preserve git history
- **FR-014**: System MUST NOT modify lesson content EXCEPT for updating asset paths (slides, images) to match new numbering
- **FR-015**: System MUST replace specs/book/chapter-index.md with updated content from specs/book/chapter-index-NEW.md showing complete 86-chapter structure:
  - Chapters 1-12: No change
  - Chapters 13-15: New SDD-RI part
  - Chapters 16-33: Python (was 13-30, +3 shift in metadata)
  - Chapters 34-36: New AI Product Leadership part
  - Chapters 37-86: All remaining parts (was 35-84, +2 shift in metadata only - no directory moves in Phase 1)
- **FR-016**: System MUST rename 18 Python slide PDFs with +3 shift using `git mv`: `chapter-13-slides.pdf` → `chapter-16-slides.pdf`, `chapter-14-slides.pdf` → `chapter-17-slides.pdf`, ..., `chapter-30-slides.pdf` → `chapter-33-slides.pdf`
- **FR-017**: System MUST rename SDD slide PDFs using `git mv`: `chapter-31-slides.pdf` → `chapter-13-slides.pdf`, `chapter-32-slides.pdf` → `chapter-14-slides.pdf`
- **FR-018**: System MUST rename AI Orchestra slide PDF using `git mv`: `chapter-33-slides.pdf` → `chapter-35-slides.pdf`
- **FR-019**: System MUST move 15+ Python image directories with +3 shift using `git mv`: `book-source/static/img/part-4/chapter-13/` → `book-source/static/img/part-5/chapter-16/`, etc.
- **FR-020**: System MUST move SDD image directories using `git mv`: `part-5/chapter-31/` → `part-4/chapter-13/`, `part-5/chapter-32/` → `part-4/chapter-14/`
- **FR-021**: System MUST move AI Orchestra image directory using `git mv`: `part-5/chapter-33/` → `part-6/chapter-35/`
- **FR-022**: System MUST update YAML frontmatter `slides.source` values in all 21 affected chapter READMEs to reference new slide filenames
- **FR-023**: System MUST update image references in all affected lesson content files to point to new part/chapter paths (grep for `/img/part-` references)
- **FR-024**: System MUST update hardcoded plugin path in `book-source/docusaurus.config.ts` from `includePaths: ['/04-Python-Fundamentals/']` to `includePaths: ['/05-Python-Fundamentals/']`
- **FR-025**: System MUST update hardcoded plugin path in `book-source/plugins/remark-interactive-python/index.js` from `includePaths = ['/04-Python-Fundamentals/']` to `includePaths = ['/05-Python-Fundamentals/']`
- **FR-026**: System SHOULD verify `docusaurus-plugin-og-image-generator` and `docusaurus-plugin-structured-data` plugins don't extract chapter numbers from directory names (investigation task)
- **FR-027**: System MUST update all narrative chapter references in lesson content files using automated search-and-replace for affected chapter mentions (e.g., "Chapter 13" → "Chapter 16", "Chapter 15" → "Chapter 18", ..., "Chapter 30" → "Chapter 33", "Chapter 31" → "Chapter 13", "Chapter 32" → "Chapter 14", "Chapter 33" → "Chapter 35")
- **FR-028**: System MUST preserve chapter references that are NOT affected by renumbering (Chapters 1-12, 34+ if mentioned in old content remain unchanged)

### Key Entities

- **Chapter Directory**: Represents a book chapter with structure `[number]-[name]/` containing README.md and lesson files
- **Part Directory**: Represents a book part with structure `[number]-[Part-Name]/` containing multiple chapter directories
- **Chapter Index**: Master metadata file (specs/book/chapter-index.md) defining all 86 chapters across 12 parts with implementation status
- **Chapter README**: Metadata file containing YAML frontmatter (sidebar_position, title, slides) and chapter overview content
- **Placeholder README**: Template-based README for chapters not yet implemented, following pattern from current Chapter 34
- **Slide PDF**: NotebookLM-generated presentation file stored in `book-source/static/slides/` with naming pattern `chapter-[number]-slides.pdf`

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 3 existing chapter directories (31, 32, 33) successfully moved to new positions (13, 14, 35) with git history preserved
- **SC-002**: All 3 new placeholder READMEs created (15, 34, 36) with valid YAML frontmatter and descriptive content
- **SC-003**: Chapter-index.md accurately reflects new 86-chapter structure with correct part assignments and renumbering
- **SC-004**: Directory structure validation passes: `book-source/docs/04-SDD-RI-Fundamentals/` contains 3 chapters (13-15), `book-source/docs/05-Python-Fundamentals/` contains 18 chapters (16-33), `book-source/docs/06-AI-Product-Leadership/` contains 3 chapters (34-36)
- **SC-005**: Zero content files lost during migration (verify count of *.md files before and after matches)
- **SC-006**: Docusaurus build system successfully generates site with new structure without errors
- **SC-007**: All moved chapter READMEs have correct sidebar_position values matching their new chapter numbers
- **SC-008**: All 21 affected slide PDF files renamed to match new chapter numbers with git history preserved (verify via `git log --follow book-source/static/slides/chapter-*.pdf`)
- **SC-009**: All ~18 affected image directories moved to new part/chapter paths with git history preserved
- **SC-010**: All YAML frontmatter `slides.source` references in 21 chapter READMEs updated to point to correct renamed slide files
- **SC-011**: All image markdown references in lesson content updated to point to new part/chapter paths (verify via `grep -r "img/part-" book-source/docs/` showing no references to old paths)
- **SC-012**: Interactive Python plugin updated to use `/05-Python-Fundamentals/` path (verify via `grep "includePaths.*04-Python" book-source` returns zero matches)
- **SC-013**: Docusaurus build succeeds AND interactive Python code blocks render correctly in new Part 5 chapters (manual test: open Chapter 16 in browser, verify code is interactive)
- **SC-014**: All narrative chapter references updated in lesson content (verify via `grep -r "Chapter (13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33)" book-source/docs` and manually check affected chapters point to correct new numbers)

## Constraints *(if applicable)*

### Technical Constraints

- MUST use `git mv` for all moves to maintain git history tracking
- MUST NOT modify lesson content files (Phase 1 is structure only)
- MUST preserve YAML frontmatter structure in all README files
- MUST follow existing naming conventions: chapter folders lowercase-with-hyphens, part folders Title-Case-With-Hyphens
- MUST create placeholder READMEs following exact pattern from existing Chapter 34 (Tessl)

### Process Constraints

- MUST complete part folder renames before moving chapters (avoids naming conflicts)
- MUST update chapter-index.md after all file system changes complete (ensures accuracy)
- MUST validate Docusaurus build after changes (early error detection)
- MUST document rollback procedure in tasks.md (recovery plan)

## Assumptions *(if applicable)*

- Docusaurus build configuration automatically picks up new sidebar_position values from YAML frontmatter
- Current Chapter 34 (Tessl) placeholder README represents the desired template format
- Git history preservation via `git mv` is sufficient (no need for specialized git filter-branch operations)
- Cross-references to old chapter numbers in lesson content will be addressed in Phase 2 (content refinement)
- Existing build scripts (package.json, docusaurus.config.js) do not hardcode chapter numbers
- Chapter numbering shift (+3 for chapters after Part 4) is consistently applied across all metadata
- Slides metadata in YAML frontmatter can reference files that don't yet exist (will be created later)

## Open Questions

None at specification phase. User explicitly scoped Phase 1 to "Structure & Placeholders only - no content creation yet."

## Dependencies *(if applicable)*

### Prerequisites

- Current chapter-index.md exists and is accurate for chapters 1-33
- All existing chapter directories (31-33) contain complete content
- Git repository is in clean state (no uncommitted changes in working directory)
- Docusaurus build system is functional and configured

### Downstream Impacts

- Phase 2 will handle content creation for new chapters (15, 34, 36)
- Phase 2 will handle content pivot for Chapter 14 (Python → YouTube/Gemini)
- Phase 2 will handle cross-reference updates in existing lessons
- Chapters 35+ in other parts may need updated references if they mention specific chapter numbers

## Non-Goals *(important)*

**Phase 1 does NOT include**:
- Writing lesson content for new chapters 15, 34, 36 (only placeholder READMEs)
- Pivoting Chapter 14 content from Python to YouTube/Gemini (structure + assets only, content pivot deferred)
- Generating new slide PDFs for placeholder chapters (15, 34, 36)
- Creating lesson files for placeholder chapters (only README.md)
- Updating any documentation outside of specs/book/ and book-source/docs/
- Updating GitHub issues, external documentation, or other repositories that reference chapter numbers

**Phase 1 DOES include** (clarification):
- ✅ Renumbering ALL Python chapter directories 13-30 → 16-33 (changed from original scope)
- ✅ Renaming ALL 21 affected slide PDFs to match new chapter numbers
- ✅ Moving ALL ~18 image directories to new part/chapter paths
- ✅ Updating asset references (slides.source in YAML, image paths in markdown)
- ✅ Updating ALL 315+ narrative chapter references in lesson content (e.g., "Chapter 15" text → "Chapter 18")
- ✅ Updating plugin configuration files (docusaurus.config.ts, remark-interactive-python)
- ✅ Updating chapter-index.md to show complete 86-chapter structure (including Part 7-12 shift by +2)

## References *(if applicable)*

- Current structure: specs/book/chapter-index.md (84 chapters, 12 parts)
- New structure: specs/book/chapter-index-NEW.md (86 chapters, 12 parts, already created in previous work)
- Placeholder template: book-source/docs/05-Spec-Driven-Development/34-tessl-framework-and-integration/README.md
- Constitution: .specify/memory/constitution.md (v6.0.1 - governs content quality, not structure)

## Additional Context *(if helpful)*

### Part 7-12 Numbering Strategy (Metadata vs Directories)

**Decision**: Phase 1 updates chapter-index.md to show chapters 37-86 (was 35-84), but does NOT move directory structures for these chapters.

**Rationale**:
- **Option chosen**: Update metadata only (Option C from spec-architect validation)
- **Why**: Clean separation of concerns - Phase 1 handles Parts 1-6 file operations + complete metadata, Phase 2+ handles remaining directory moves if needed
- **Impact**: chapter-index.md will be accurate (86 chapters correctly numbered 1-86), but Part 7-12 directories will temporarily use old numbering (this is acceptable as Docusaurus relies on `sidebar_position` in YAML frontmatter, not directory names)
- **Future**: If/when Part 7-12 chapters get implemented, directory renaming can happen then without breaking existing structure

### Rationale for Restructuring

This restructuring aligns with the book's constitutional principle that "specifications are the new syntax." The current structure teaches 18 chapters of Python (13-30) before introducing SDD concepts (31-34), which violates this principle. The new structure:

1. **Part 4 (Chapters 13-15)**: Introduces SDD-RI fundamentals and business thinking BEFORE Python
2. **Part 5 (Chapters 16-33)**: Python fundamentals with SDD mindset already established
3. **Part 6 (Chapters 34-36)**: Advanced AI product leadership and team orchestration

This allows students to learn "specification-first thinking" as a foundation before encountering implementation syntax.

### Migration Strategy Sequencing

**To avoid conflicts, perform operations in this order**:

### Phase A: Directory Structure Preparation
1. Rename Part 4 (Python) to Part 5: `04-Python-Fundamentals` → `05-Python-Fundamentals`
2. Rename old Part 5 to Part 4: `05-Spec-Driven-Development` → `04-SDD-RI-Fundamentals`
3. Create new Part 6 directory: `06-AI-Product-Leadership/`

### Phase B: Chapter Directory Moves (reverse order to avoid conflicts)
4. Move chapter 33 → 35: `04-SDD-RI-Fundamentals/33-*` → `06-AI-Product-Leadership/35-*` (temp location)
5. Move Python chapters 13-30 → 16-33 (**REVERSE ORDER 30→33 first, then 29→32, ..., 13→16 last**)
6. Move SDD chapters: 31→13, 32→14 to Part 4
7. Move chapter 35 from temp to final Part 6 location

### Phase C: Asset Migration (slides + images)
8. Rename Python slide PDFs (**REVERSE ORDER 30→33 first, then 29→32, ..., 13→16 last**)
9. Rename SDD slide PDFs: 31→13, 32→14
10. Rename AI Orchestra slide: 33→35
11. Move Python image directories (reverse order, same logic as slides)
12. Move SDD image directories: part-5/chapter-31 → part-4/chapter-13, etc.
13. Move AI Orchestra images: part-5/chapter-33 → part-6/chapter-35

### Phase D: Metadata & Content Updates
14. Update YAML frontmatter in all 21 affected chapter READMEs (sidebar_position + slides.source)
15. Update image markdown references in lesson content files (grep-and-replace `/img/part-4/chapter-X` → `/img/part-5/chapter-Y`)
16. Update plugin configuration files:
    - `book-source/docusaurus.config.ts`: `/04-Python-Fundamentals/` → `/05-Python-Fundamentals/`
    - `book-source/plugins/remark-interactive-python/index.js`: same update
17. Update narrative chapter references in lesson content (**REVERSE ORDER to prevent double-replacement**):
    - "Chapter 33" → "Chapter 35" (do this first)
    - "Chapter 32" → "Chapter 14"
    - "Chapter 31" → "Chapter 13"
    - "Chapter 30" → "Chapter 33", "Chapter 29" → "Chapter 32", ..., "Chapter 13" → "Chapter 16" (Python chapters)
18. Create placeholder READMEs for chapters 15, 34, 36
19. Update chapter-index.md with new 86-chapter structure

### Phase E: Validation
20. Run Docusaurus build to verify no broken links
21. Validate git history preservation for all `git mv` operations
22. Verify asset counts: 33 slide PDFs exist, ~30 image directories exist, no orphaned files
23. Manual browser test: Verify Chapter 16 (Python UV) has interactive code blocks working
24. Spot-check chapter reference updates: Verify 5 random "Chapter X" mentions in content point to correct new numbers

**Why Reverse Order?**
When renaming files with numeric sequences where source and target ranges overlap (e.g., 13-30 → 16-33), forward order causes conflicts:
- Forward: `chapter-13 → chapter-16` works, but then `chapter-16 → chapter-19` would overwrite the file we just created!
- Reverse: `chapter-30 → chapter-33` (safe, 33 doesn't exist), `chapter-29 → chapter-32` (safe), ..., `chapter-13 → chapter-16` (safe, 13 no longer needed)

This applies to:
- Chapter directories: 30→33, 29→32, ..., 13→16
- Slide PDFs: chapter-30-slides.pdf → chapter-33-slides.pdf, ..., chapter-13-slides.pdf → chapter-16-slides.pdf
- Image directories: part-4/chapter-30 → part-5/chapter-33, ..., part-4/chapter-13 → part-5/chapter-16

**Rollback plan** (if needed):
- All operations via `git mv` means `git reset --hard` restores original state
- Document original structure in tasks.md for reference
- Maintain backup of chapter-index.md before updates
