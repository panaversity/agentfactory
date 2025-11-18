---
id: 003
title: Decouple Constitution and Align Infrastructure with Reality
stage: constitution
date: 2025-10-29
surface: agent
model: claude-sonnet-4.5
feature: infrastructure-refactor
branch: 002-book-structure
user: mjs
command: /refactor
labels: ["constitution", "decoupling", "infrastructure", "book-scaffolding", "directory-structure", "maintainability"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/skills/book-scaffolding/SKILL.md (created)
  - .claude/skills/book-scaffolding/reference/*.md (4 files copied from book-architecture)
  - .claude/skills/book-scaffolding/templates/*.yml (2 files copied)
  - .claude/skills/book-scaffolding/scripts/*.py (2 files copied)
  - .specify/memory/constitution.md (v2.0.0 → v2.1.0)
  - specs/book/directory-structure.md (created - new authoritative source)
  - specs/book/chapter-index.md (updated references)
  - .claude/output-styles/chapters.md (updated references)
  - CLAUDE.md (added Book Content Organization section)
  - README.md (cleaned up, 202 → 93 lines)
  - specs/part-1/part-1-spec.md (updated skill reference)
  - .claude/skills/book-scaffolding.md (deleted - old single file)
  - .claude/skills/book-architecture/ (deleted entire directory)
tests:
  - No linter errors (verified CLAUDE.md, constitution.md, README.md)
---

## Prompt

User identified critical coupling issues in project documentation:

**Initial Request**: "You are an AI Pilot specializing in system architecture. We need to perform a critical decoupling refactor on CLAUDE.md. The file is currently too 'tightly coupled' to a specific project, which makes it brittle and hard to reuse."

**Two Major Coupling Issues**:

1. **Hardcoded Constitution**: The file violates its own 'Source of Truth' mandate by hardcoding project specs (like '7-part book') instead of reading them from constitution.md. You must find and replace all such hardcoded values with generic pointers that defer to the constitution.

2. **Project-Specific Skills**: The file implies the 8 domain skills in .claude/skills/ are for this book only. This is false. They are a generic, reusable library. You must add a clear statement to 'The 8 Mandatory Domain Skills' section clarifying that these skills are project-agnostic and part of our core reusable toolkit.

**Follow-up Discovery**: During analysis, discovered that constitution itself had similar coupling issues:
- Hardcoded "26 chapters" instead of 32
- Part 2 vs Part 5 Spec-Kit contradiction
- Missing chapters 27-32 in scaffolding ranges
- Infrastructure section didn't match actual files
- book-architecture skill vs book-scaffolding skill confusion

**Additional Request**: "Then we shall add in document to use @book-scaffolding.md skill to actually plan and build it first if not present in collaboration with user. This also hit the fact that @book-architecture skill doesn't make any sense and shall be deleted or replaced to formally have book-scaffolding as skill?"

**Final Request**: "Now it can be referenced in CLAUDE.md and constitution and those in progress book parts and chapter index. Also remove anything about it from @README.md"

## Response snapshot

Executed comprehensive infrastructure refactoring in 3 phases:

### Phase 1: Skill Restructuring (book-architecture → book-scaffolding)

**Created** `.claude/skills/book-scaffolding/` directory structure:
- **SKILL.md** with proper frontmatter (v2.0.0)
  - 9 proven principles from 002-book-structure sprint
  - 5-phase workflow (Global → Part → Chapter-planner → Lesson-writer → Validation)
  - Integration of valuable reference materials from book-architecture
- **reference/** (4 files): chapter-flow-patterns, structural-patterns, content-organization, chapter-dependencies
- **templates/** (2 files): book-outline-template, chapter-structure-template
- **scripts/** (2 files): validate-structure.py, analyze-flow.py

**Rationale**: book-scaffolding is based on **real learnings** from actual implementation, while book-architecture had project-specific contamination. The name "scaffolding" is pedagogically accurate (supporting learners).

**Deleted**:
- `.claude/skills/book-scaffolding.md` (old single file)
- `.claude/skills/book-architecture/` (entire directory)

### Phase 2: Constitution Refactoring (v2.0.0 → v2.1.0)

**Fixed ALL major inconsistencies**:

1. **Chapter Count**: 26 → 32 (Principle 6)

2. **Spec-Kit Contradiction**: 
   - Removed "Part 2 (5 chapters)" hardcoded reference
   - Now says "progressively introduced" with reference to chapter-index.md

3. **Scaffolding Ranges**:
   - Old: Chapters 1-5 (heavy), 6-15 (moderate), 16-26 (minimal) — missing 27-32!
   - New: Chapters 1-10 (heavy), 11-20 (moderate), 21-32 (minimal)

4. **Section III (Book Structure)** — **MAJOR DECOUPLING**:
   - Removed all hardcoded part details
   - Now defers to `specs/book/chapter-index.md` as authoritative source
   - Added: "Use book-scaffolding skill to plan in collaboration with user"

5. **Section VI (Infrastructure)** — **Aligned with Reality**:
   - References actual 8 CoLearning Domain Skills (not phantom files)
   - Lists 3 actual subagents (chapter-planner, content-implementer, validation-auditor)
   - References actual output-styles (chapters.md, lesson.md)

6. **Section V (Workflow)** — **Matches Actual Subagents**:
   - Updated SDD loop to reflect 3 actual subagents
   - Added: "Use book-scaffolding skill for structural planning"

7. **Section II.B (Skills)** — **Updated Skill 7**:
   - book-architecture → book-scaffolding with detailed description

8. **Version & Metadata**:
   - Version: 2.0.0 → 2.1.0
   - Last Amended: 2025-10-28 → 2025-10-29
   - Added comprehensive changelog in header comment

### Phase 3: Documentation Improvements

**Created**: `specs/book/directory-structure.md` (432 lines)
- **THE authoritative source** for file organization
- Documents actual structure from `book-source/docs/`
- 3-level hierarchy with exact naming rules
- Part folders: Title-Case (e.g., `01-Introducing-AI-Driven-Development/`)
- Chapter folders: lowercase (e.g., `01-welcome-to-ai-driven-development/`)
- Lesson files: lowercase (e.g., `01-lesson-1.md`)
- Conversion formula from chapter-index.md to directory paths
- Required files: `intro.md` (parts), `README.md` (chapters)
- Validation checklist and examples

**Updated Cross-References**:
- `specs/book/chapter-index.md` — Points to directory-structure.md for paths
- `.claude/output-styles/chapters.md` — References directory-structure.md
- `.claude/skills/book-scaffolding/SKILL.md` — Lists directory-structure.md first
- `.specify/memory/constitution.md` — Mentions both chapter-index.md and directory-structure.md
- `CLAUDE.md` — Added "Book Content Organization" section
- `README.md` — Cleaned up (202 → 93 lines), removed redundant details

## Outcome

- ✅ Impact: **Complete decoupling achieved**. Constitution now practices what it preaches (defers to authoritative sources). Infrastructure aligned with reality. Skills properly categorized as generic/reusable.
- 🧪 Tests: No linter errors. All cross-references validated.
- 📁 Files: 7 created/modified, 2 deleted, 13 updated with cross-references
- 🔁 Next prompts: Ready for content creation. Agents now have clear documentation for "WHAT to write" (chapter-index.md), "WHERE to put it" (directory-structure.md), "HOW to format it" (output-styles/), and "WHY" (constitution.md).
- 🧠 Reflection: This refactor addressed a critical architectural issue — the constitution and CLAUDE.md were violating their own "source of truth" mandate by hardcoding specifics. Now we have proper separation: constitution defines PRINCIPLES, chapter-index.md defines CONTENT, directory-structure.md defines ORGANIZATION, and output-styles define FORMAT. Each document has a single, clear responsibility. The book-scaffolding skill consolidates proven learnings from actual implementation, making it more valuable than the theoretical book-architecture. All agents now have an unambiguous reference chain.

## Evaluation notes (flywheel)

- Failure modes observed: **Tight coupling** (hardcoding specifics in generic documents), **phantom references** (documenting files/subagents that don't exist), **inconsistencies** (chapter counts, part numbers), **redundant documentation** (same info in multiple places with variations)
- Graders run and results (PASS/FAIL): **PASS** - No linter errors, all cross-references accurate, structure matches reality
- Prompt variant (if applicable): User provided clear architectural direction ("decouple", "source of truth"), then iteratively refined based on discoveries
- Next experiment (smallest change to try): Monitor agent usage of new documentation. If agents still create files in wrong locations, enhance directory-structure.md with more examples or add validation scripts.

