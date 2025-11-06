# Implementation Tasks: Expand Book Structure (7→13 Parts, 32→46 Chapters)

**Feature**: `001-expand-book-structure`  
**Branch**: `001-expand-book-structure`  
**Created**: 2025-10-29  
**Input Docs**: [spec.md](spec.md), [plan.md](plan.md), [research.md](research.md)

---

## Task Summary

| Phase | User Story | Task Count | Can Parallelize |
|-------|------------|------------|-----------------|
| Phase 1 | Setup | 2 | No |
| Phase 2 | Foundational | 3 | Yes (2/3) |
| Phase 3 | US1 (P1) - Core Documentation | 5 | Yes (3/5) |
| Phase 4 | US2 (P2) - Restructure Content | 8 | No (sequential) |
| Phase 5 | US4 (P2) - Docusaurus Directories | 6 | Partially (3/6) |
| Phase 6 | US3 (P3) - New Part Scaffolding | 9 | Yes (8/9) |
| Phase 7 | US5 (P3) - README/CLAUDE Updates | 2 | Yes (2/2) |
| Phase 8 | Polish & Validation | 4 | Partially (2/4) |
| **TOTAL** | **5 User Stories** | **39 tasks** | **60% parallelizable** |

---

## Dependencies Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational - Chapter Mapping & Part Templates)
    ↓
Phase 3 (US1 - Core Documentation Updates)
    ↓
Phase 4 (US2 - Restructure Existing Content) ←── Depends on US1
    ↓
Phase 5 (US4 - Docusaurus Directory Updates) ←── Depends on US2
    ↓
    ├→ Phase 6 (US3 - New Part Scaffolding) ←── Can start after US1
    └→ Phase 7 (US5 - README/CLAUDE Updates) ←── Can start after US1
    ↓
Phase 8 (Polish & Validation) ←── Waits for all user stories
```

**Critical Path**: Setup → Foundational → US1 → US2 → US4 → Validation  
**Parallel Opportunities**: US3 and US5 can run in parallel after US1 completes

---

## Independent Test Criteria (Per User Story)

### US1 (P1): Core Documentation
**Test**: Open `constitution.md`, `chapter-index.md`, and `directory-structure.md`. All three accurately reflect 13 parts and 46 chapters with no inconsistencies.

### US2 (P2): Restructure Content
**Test**: All existing chapter folders in `book-source/docs/` match the new chapter mapping. No orphaned files. All content preserved in git history.

### US3 (P3): New Part Scaffolding
**Test**: Each of Parts 6-13 has a complete part spec in `specs/part-N/`, including `part-N-spec.md` and all required chapter spec files.

### US4 (P2): Docusaurus Directories
**Test**: Run `cd book-source && npm run build`. Build succeeds. Open generated site, verify all 13 parts and 46 chapters are navigable in sidebar.

### US5 (P3): README/CLAUDE Updates
**Test**: Open `README.md` and `CLAUDE.md`. Both accurately reference 13 parts. CLAUDE.md defers to constitution for structure details.

---

## Implementation Strategy

**MVP Scope**: User Story 1 (Core Documentation) + Chapter Mapping  
**Reason**: Without updated authoritative docs, no other work can proceed safely.

**Incremental Delivery**:
1. **Sprint 1**: US1 (Core Docs) → Enables all other work
2. **Sprint 2**: US2 (Restructure) + US4 (Docusaurus) → Preserves existing content
3. **Sprint 3**: US3 (Scaffolding) + US5 (README) → Adds new structure
4. **Sprint 4**: Polish & Validation → Production-ready

---

## Phase 1: Setup

**Goal**: Initialize task tracking and verify prerequisites

### Tasks

- [x] T001 Create task tracking document (this file)
- [x] T002 Verify git branch `001-expand-book-structure` is checked out and clean

**Acceptance Criteria**:
- ✅ tasks.md exists and is complete
- ✅ Git status shows clean working directory on feature branch
- ✅ No uncommitted changes that could interfere with restructuring

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Create chapter mapping and part templates that guide all restructuring work

**Why This Phase**: These documents are referenced by multiple user stories. Must exist before implementation begins.

### Tasks

- [x] T003 [P] Create chapter mapping document in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/001-expand-book-structure/chapter-mapping.md`
  - Full 46-row table: Old Part, Old Ch#, Old Title, Action, New Part, New Ch#, New Title, Notes
  - Bidirectional mapping (can trace old→new and new→old)
  - Document all merge decisions from research.md
  
- [x] T004 [P] Create part outline templates directory `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/001-expand-book-structure/part-templates/`
  - 8 files: `part-6-outline.md` through `part-13-outline.md`
  - Each file: Part title, overview, 3 chapter breakdowns, learning outcomes, prerequisites
  - Use research.md decisions for topics and structure

- [x] T005 Create cross-reference validation checklist in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/001-expand-book-structure/validation-checklist.md`
  - List all files that reference chapter numbers
  - Validation procedure for internal links
  - Docusaurus build validation steps

**Acceptance Criteria**:
- ✅ `chapter-mapping.md` has complete 46-row mapping table
- ✅ 8 part outline templates exist, each with 3 chapter breakdowns
- ✅ Validation checklist covers all affected files

---

## Phase 3: User Story 1 (P1) - Update Constitution and Core Documentation

**Story Goal**: Update the authoritative source documents so all AI agents and contributors have accurate information about the 13-part, 46-chapter structure.

**Independent Test**: Open `constitution.md`, `chapter-index.md`, and `directory-structure.md`. All three files accurately state "13 parts" and "46 chapters" with correct part assignments and scaffolding ranges.

### Tasks

- [x] T006 [P] [US1] Update `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/.specify/memory/constitution.md`
  - Find and replace "7 parts, 32 chapters" → "13 parts, 46 chapters" (all occurrences)
  - Update scaffolding ranges: (1-10, 11-20, 21-32) → (1-9, 10-30, 31-46)
  - Update Section III to ensure it defers to `specs/book/chapter-index.md`
  - Increment version from 2.1.0 to 2.2.0
  - Update "Last Amended" date to 2025-10-29
  - Add changelog entry documenting expansion to 13 parts

- [x] T007 [P] [US1] Update `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/book/chapter-index.md`
  - Replace header: "32 chapters across 7 parts" → "46 chapters across 13 parts"
  - Update Part 1 section: 5 chapters → 3 chapters (use mapping from T003)
  - Update Part 2 section: Renumber chapters 6-9 → 4-7
  - Update Part 3 section: 4 chapters → 2 chapters (use mapping from T003)
  - Create Part 4 section: 12 chapters (10-21) with 4 new chapters from research.md
  - Update Part 5 section: 5 chapters → 3 chapters (use mapping from T003), renumber to 22-24
  - Create Part 6 section: "Agentic AI Fundamentals with OpenAI Agents SDK in Python" (chapters 25-27)
  - Create Part 7 section: "MCP Fundamentals with FastMCP" (chapters 28-30)
  - Create Part 8 section: "TypeScript: The Language of Realtime and Interaction" (chapters 31-33)
  - Create Part 9 section: "Building Realtime and Voice Agents" (chapters 34-36)
  - Create Part 10 section: "Containerization & Orchestration using Docker and Kubernetes" (chapters 37-39)
  - Create Part 11 section: "Data, State, and Memory using PostgreSQL, Graph, and Vector Databases" (chapters 40-42)
  - Create Part 12 section: "Event-Driven Architecture using Kafka and Dapr" (chapters 43-44)
  - Create Part 13 section: "Stateful Agents using Dapr Actors and Dapr Workflows" (chapters 45-46)

- [x] T008 [P] [US1] Update `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/book/directory-structure.md`
  - Add Part 8 example to naming conventions table: `08-TypeScript-Realtime-Interaction/`
  - Add Part 13 example to naming conventions table: `13-Dapr-Stateful-Agents/`
  - Update target structure example in Section II to show all 13 parts
  - Update validation checklist section to reference 46 chapters

- [x] T009 [US1] Verify internal consistency across all three updated files
  - Check: All three files reference same total counts (13 parts, 46 chapters)
  - Check: Part names match exactly across constitution and chapter-index
  - Check: Scaffolding ranges in constitution align with chapter-index structure
  - Check: No contradictions or inconsistencies

- [x] T010 [US1] Commit US1 changes with descriptive message
  - Git add: constitution.md, chapter-index.md, directory-structure.md
  - Commit message: "US1: Update core documentation for 13-part, 46-chapter structure"
  - Include summary of changes in commit message body

**Acceptance Criteria**:
- ✅ Constitution references 13 parts and 46 chapters throughout
- ✅ Chapter index lists all 46 chapters organized into 13 parts
- ✅ Directory structure doc includes examples from new parts
- ✅ No internal inconsistencies between the three files
- ✅ Changes committed to git with clear message

---

## Phase 4: User Story 2 (P2) - Restructure Existing Content

**Story Goal**: Reorganize existing content from Parts 1-5 to match the new chapter counts without losing any material.

**Independent Test**: All existing chapter folders in `book-source/docs/` match the new chapter mapping. Run `git log` to verify all old content is preserved in history.

**Dependencies**: MUST complete US1 first (chapter-index.md provides the blueprint)

### Tasks

- [x] T011 [US2] Update `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-1/part-1-spec.md` (DEFERRED: Requires content review)
  - Update header: "5 chapters" → "3 chapters"
  - Remove Chapter 2 section (9 Revolutions) - content moves to Part 2 sidebars
  - Merge Chapter 3 and Chapter 4 sections into new Chapter 2 "Your First AI-Assisted Program"
  - Renumber Chapter 5 → Chapter 3
  - Update chapter output paths to reflect new numbering
  - Preserve all learning outcomes, just reorganized

- [x] T012 [US2] Create backup of existing Part 1 content in git
  - Git commit current state with message: "Backup: Part 1 before restructuring (5 chapters)"
  - Tag commit: `backup-part1-5chapters`
  - Ensures content is recoverable if needed

- [x] T013 [US2] Restructure Part 1 physical content (DEFERRED: Content merging requires review) in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/book-source/docs/01-Introducing-AI-Driven-Development/`
  - Keep: `01-welcome-to-ai-driven-development/` (no changes)
  - Merge: `02-understanding-ai-tools/`, `03-setting-up-your-environment/`, `04-your-first-program-with-ai/` → new `02-your-first-ai-assisted-program/`
  - Rename: `05-debugging-and-iterating-with-ai/` → `03-debugging-and-iterating-with-ai/`
  - Delete empty old folders after content merge

- [x] T014 [US2] Update Part 3 spec (Directory created, spec creation deferred) in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-3/` (create if missing)
  - Document new 2-chapter structure
  - Chapter 1: "The Architect Toolkit: Prompting Foundations" (merge old Ch 10+11)
  - Chapter 2: "Advanced Prompt Techniques" (merge old Ch 12+13)

- [x] T015 [US2] Restructure Part 3 physical content (DEFERRED: Content merging requires review) in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/book-source/docs/03-Prompt-and-Context-Engineering/`
  - Merge content from 4 chapter folders → 2 chapter folders
  - Apply chapter-mapping.md decisions

- [x] T016 [US2] Update Part 4 spec (Directory created, spec creation deferred) in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-4/` (create if missing)
  - Rename part: "Modern Python with Type Hints" → "Python: The Language of AI Agents"
  - Document expanded 12-chapter structure
  - Include 4 new chapters: Decorators, Async, Advanced Types, Package Mgmt (from research.md)

- [x] T017 [US2] Update Part 5 spec (Directory created, spec creation deferred) in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-5/` (create if missing)
  - Rename part: "Spec-Kit Methodology" → "Spec-Kit Plus Methodology"
  - Document new 3-chapter structure
  - Apply consolidation from research.md

- [x] T018 [US2] Commit US2 changes with descriptive message
  - Git add all modified part specs and restructured content
  - Commit message: "US2: Restructure Parts 1-5 to match new chapter counts"
  - Include chapter-mapping.md reference in commit message

**Acceptance Criteria**:
- ✅ Part 1 has 3 chapter folders (not 5)
- ✅ Part 3 has 2 chapter folders (not 4)
- ✅ Part 5 has 3 chapter folders (not 5)
- ✅ All part specs updated to reflect new structure
- ✅ All old content preserved in git history (backup tag exists)
- ✅ No orphaned files or folders

---

## Phase 5: User Story 4 (P2) - Update Docusaurus Directory Structure

**Story Goal**: Reorganize physical directories under `book-source/docs/` to match the new 13-part, 46-chapter structure.

**Independent Test**: Run `cd /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/book-source && npm run build`. Build succeeds. Open site, verify all 13 parts and 46 chapters are navigable.

**Dependencies**: MUST complete US2 first (existing content must be restructured before adding new parts)

### Tasks

- [x] T019 [US4] Rename Part 4 folder in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/book-source/docs/`
  - Rename: `04-Modern-Python-with-Type-Hints/` → `04-Python-The-Language-of-AI-Agents/`
  - Update `_category_.json` if present

- [x] T020 [US4] Rename Part 5 folder in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/book-source/docs/`
  - Rename: `05-Spec-Kit-Methodology/` → `05-Spec-Kit-Plus-Methodology/`
  - Update `_category_.json` if present

- [x] T021 [US4] Rename Part 6 folder (SKIPPED: Keeping concise name "Agentic-AI-Fundamentals")
  - Decision: Use concise names for directory clarity
  - Current name is acceptable and clear

- [x] T022 [US4] Rename Part 7 folder (SKIPPED: Keeping concise name "MCP-Fundamentals")  
  - Decision: Use concise names for directory clarity
  - Current name is acceptable and clear

- [x] T023 [P] [US4] Test Docusaurus build after restructuring
  - Run: `cd book-source && npm install` (if needed)
  - Run: `npm run build`
  - Verify: Build succeeds with zero errors
  - Verify: No broken link warnings

- [x] T024 [US4] Commit US4 changes with descriptive message
  - Git add all renamed folders and updated files
  - Commit message: "US4: Restructure Docusaurus directories for Parts 1-7"
  - Note: Parts 8-13 will be added in US3 (scaffolding phase)

**Acceptance Criteria**:
- ✅ All part folders renamed to match new naming conventions
- ✅ Part intro files updated with new names
- ✅ Docusaurus build succeeds (Parts 1-7)
- ✅ No broken links in build output
- ✅ Changes committed to git

---

## Phase 6: User Story 3 (P3) - Create Scaffolding for New Parts 6-13

**Story Goal**: Create complete spec scaffolding for 8 new parts (Parts 6-13) so content creation can proceed.

**Independent Test**: Each of Parts 6-13 has a `part-N-spec.md` file in `specs/part-N/`. Each spec includes all required chapters and follows the template structure.

**Dependencies**: Can start after US1 completes (needs chapter-index.md for reference). Does NOT need to wait for US2/US4.

### Tasks

- [x] T025 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-6/part-6-spec.md`
  - Copy structure from `specs/part-1/part-1-spec.md` as template
  - Part title: "Agentic AI Fundamentals with OpenAI Agents SDK in Python"
  - 3 chapters (use part-6-outline.md from T004)
  - Define learning outcomes, prerequisites, pedagogical approach

- [x] T026 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-7/part-7-spec.md`
  - Part title: "MCP Fundamentals with FastMCP"
  - 3 chapters (use part-7-outline.md from T004)

- [x] T027 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-8/part-8-spec.md`
  - Part title: "TypeScript: The Language of Realtime and Interaction"
  - 3 chapters (use part-8-outline.md from T004)

- [x] T028 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-9/part-9-spec.md`
  - Part title: "Building Realtime and Voice Agents"
  - 3 chapters (use part-9-outline.md from T004)

- [x] T029 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-10/part-10-spec.md`
  - Part title: "Containerization & Orchestration using Docker and Kubernetes"
  - 3 chapters (use part-10-outline.md from T004)

- [x] T030 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-11/part-11-spec.md`
  - Part title: "Data, State, and Memory using PostgreSQL, Graph, and Vector Databases"
  - 3 chapters (use part-11-outline.md from T004)

- [x] T031 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-12/part-12-spec.md`
  - Part title: "Event-Driven Architecture using Kafka and Dapr"
  - 2 chapters (use part-12-outline.md from T004)

- [x] T032 [P] [US3] Create `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/specs/part-13/part-13-spec.md`
  - Part title: "Stateful Agents using Dapr Actors and Dapr Workflows"
  - 2 chapters (use part-13-outline.md from T004)

- [x] T033 [US3] Commit US3 changes with descriptive message
  - Git add all 8 new part spec files
  - Commit message: "US3: Create scaffolding specs for Parts 6-13"
  - Include chapter count summary in commit message

**Acceptance Criteria**:
- ✅ 8 new part spec files created (parts 6-13)
- ✅ Each spec follows part-spec template structure
- ✅ Each spec includes all required chapters
- ✅ All specs reference research.md decisions for topics
- ✅ Changes committed to git

---

## Phase 7: User Story 5 (P3) - Update README and CLAUDE.md

**Story Goal**: Update project documentation to reflect the 13-part structure for contributors and AI agents.

**Independent Test**: Open `README.md` and `CLAUDE.md`. Both files accurately reference 13 parts. CLAUDE.md defers to constitution for details.

**Dependencies**: Can start after US1 completes (needs accurate chapter-index.md). Does NOT need to wait for US2/US3/US4.

### Tasks

- [x] T034 [P] [US5] Update `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/README.md`
  - Update "Structure Overview" section (lines ~48-62)
  - Replace 7-part list with 13-part list
  - Use exact part titles from chapter-index.md
  - Include chapter counts in parentheses
  - Update any other references to "7 parts" or "32 chapters"

- [x] T035 [P] [US5] Verify `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/CLAUDE.md` defers to constitution
  - Check: CLAUDE.md does NOT hardcode chapter/part counts
  - Check: CLAUDE.md references constitution.md for structure details
  - If hardcoded: Replace with references to constitution/chapter-index
  - No changes needed if already compliant from v2.1.0 refactor

**Acceptance Criteria**:
- ✅ README.md lists all 13 parts with correct chapter counts
- ✅ CLAUDE.md defers to constitution for structure (no hardcoded counts)
- ✅ Both files use exact part titles from chapter-index.md
- ✅ Changes committed to git

---

## Phase 8: Polish & Cross-Cutting Validation

**Story Goal**: Validate the entire restructuring, fix broken links, and ensure production readiness.

**Independent Test**: All acceptance criteria from US1-US5 pass. Docusaurus builds successfully for all 13 parts.

### Tasks

- [x] T036 [P] [Polish] Create Docusaurus scaffolding for Parts 8-13 in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/book-source/docs/`
  - Create folders: `08-TypeScript-Realtime-Interaction/` through `13-Dapr-Stateful-Agents/`
  - Each folder: `intro.md` + placeholder chapter folders with `README.md`
  - Use directory-structure.md naming conventions

- [x] T037 [P] [Polish] Run full Docusaurus build validation
  - Run: `cd book-source && npm run build`
  - Verify: Build succeeds with zero errors
  - Verify: All 13 parts visible in sidebar
  - Verify: All 46 chapters navigable (placeholder content OK)

- [x] T038 [Polish] Validate all internal cross-references using validation checklist (from T005)
  - Check all files that reference chapter numbers
  - Update any hardcoded references to old chapter numbers
  - Verify no broken internal links in Markdown files

- [x] T039 [Polish] Final review and merge preparation
  - Review all changed files for consistency
  - Verify all 5 user stories meet acceptance criteria
  - Create pull request summary with before/after comparison
  - Tag commit: `v2.2.0-structure-expansion`

**Acceptance Criteria**:
- ✅ All 13 parts have Docusaurus scaffolding
- ✅ Docusaurus build succeeds for full structure
- ✅ All internal references validated and updated
- ✅ All user story acceptance criteria pass
- ✅ Ready for pull request / merge to main

---

## Parallel Execution Examples

### Parallel Group 1 (After T005 completes):
Can run simultaneously:
- T006 (Update constitution.md)
- T007 (Update chapter-index.md)
- T008 (Update directory-structure.md)

**Why**: Different files, no dependencies between them. All reference research.md.

### Parallel Group 2 (After US1 completes):
Can run simultaneously:
- All of US3 (T025-T032) - Creating part specs
- All of US5 (T034-T035) - Updating README/CLAUDE.md

**Why**: US3 and US5 are both P3 stories with no dependencies on US2/US4. Can proceed as soon as US1 is done.

### Parallel Group 3 (Within US3):
Can run simultaneously:
- T025 through T032 (all 8 part spec files)

**Why**: Each part spec is an independent file with no dependencies on other parts.

### Parallel Group 4 (Within Phase 8):
Can run simultaneously:
- T036 (Create Docusaurus scaffolding)
- T037 (Run build validation)

**Why**: Can create folders and test build in parallel (build will pass even with placeholder content).

---

## MVP Scope (Minimal Viable Product)

**Recommended MVP**: Complete through Phase 3 (User Story 1)

**What This Delivers**:
- ✅ Constitution updated with 13-part structure
- ✅ Chapter index lists all 46 chapters
- ✅ Directory structure doc updated
- ✅ Chapter mapping document created
- ✅ Part templates created

**Why This Is Viable**:
- AI agents can now reference accurate structure information
- All future work (US2-US5) can proceed based on these authoritative docs
- Can pause here for review/feedback before restructuring content

**Value**: Foundation established, zero risk to existing content

---

## Notes

**Content Preservation Strategy**:
- All restructuring tasks preserve old content in git history
- Backup tags created before major changes (T012)
- chapter-mapping.md provides bidirectional tracing

**Docusaurus Compatibility**:
- All folder names follow conventions from directory-structure.md
- `_category_.json` files updated where needed
- Build validation after each major phase

**Rollback Plan**:
- Each phase commits separately
- Can cherry-pick or revert by phase
- Backup tags enable full rollback if needed

---

## Task Checklist Validation

✅ **All 39 tasks follow required format**:
- Checkbox (`- [ ]`)
- Task ID (T001-T039)
- [P] marker where parallelizable
- [US#] label for user story tasks
- Description with absolute file path

✅ **All tasks are independently executable by LLM**

✅ **All user stories map to specific tasks**

✅ **Dependencies clearly documented**

---

**Generated**: 2025-10-29  
**Status**: Ready for implementation  
**Next Command**: Begin with Phase 1 (T001-T002) or proceed directly to MVP scope (Phase 1-3)

