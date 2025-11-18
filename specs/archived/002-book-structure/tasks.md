# Tasks: Book Structure - Minimal MVP

**Feature**: `002-book-structure` | **Branch**: `002-book-structure` | **Date**: 2025-10-29
**Input**: spec.md + plan.md from `/specs/002-book-structure/`
**Objective**: Create book directory structure + Part 1 spec, enabling early-access launch and Part 1 writing

---

## Phase 1: Infrastructure Setup (3 tasks)

**Purpose**: Create directory structure and basic chapter templates

- [x] T001 ✅ DONE Create 7 part directories in `book-source/docs/`

- [x] T002 ✅ DONE Create part intro files (7 files):
  ```
  01-Introducing-AI-Driven-Development/intro.md
  02-AI-Tool-Landscape/intro.md
  03-Prompt-and-Context-Engineering/intro.md
  04-Modern-Python-with-Type-Hints/intro.md
  05-Spec-Kit-Methodology/intro.md
  06-Agentic-AI-Fundamentals/intro.md
  07-MCP-Fundamentals/intro.md
  ```
  Each intro.md includes:
  - Part title and number
  - Learning outcomes (from plan.md)
  - Chapter list with titles
  - Cognitive load level and scaffolding strategy (from spec.md)

- [x] T003 ✅ DONE [P] Create 32 chapter placeholder files with basic structure:
  ```
  ---
  sidebar_position: N
  title: "Chapter Title"
  ---

  # Chapter Title

  **Learning Outcomes:**
  - [To be completed in chapter spec]

  **Key Topics:**
  - [To be completed in chapter spec]

  **Prerequisites:**
  - [To be completed in chapter spec]

  ---

  [Content to be written per chapter spec]
  ```
  All 32 chapter files placed in respective part directories

**Checkpoint**: Directory structure ready, Docusaurus can build with 32 placeholder chapters visible

---

## Phase 2: Part 1 Specification (1 critical task)

**Purpose**: Create detailed Part 1 spec so chapter-planner subagent can begin planning

- [ ] T004 Create `specs/part-1/part-1-spec.md` defining:
  - Part 1 purpose and learning outcomes (from plan.md)
  - All 5 chapters with titles, scope, key topics
  - Prerequisites for Part 1
  - Cognitive load level (light), scaffolding level (heavy), expected review cycles (2-3)
  - Use plan.md "Per-Part Detailed Architecture" section as template
  - Reference: `.specify/templates/tasks-template.md` structure for consistency

**⏸️ Defer (Just-In-Time)**:
- Part 2-7 specs created only when each part is ready for planning during implementation
- Placeholder clarifications for each part (agent frameworks, case studies, etc.) resolved during chapter-planner phase
- This respects SDD loop: Spec → Plan → Implement per part (one at a time)

**Checkpoint**: Part 1 spec created and ready for chapter-planner subagent

---

## Phase 3: Docusaurus Validation (4 tasks) ✅ COMPLETE

**Purpose**: Verify book structure builds and displays correctly

- [x] T013 ✅ DONE [P] Run: `cd book-source && npm run build` → verify zero errors
- [x] T014 ✅ DONE [P] Verify all 7 part folders + 32 chapter files exist with correct names
- [x] T015 ✅ DONE [P] Verify `sidebars.ts` auto-generates all 7 parts and 32 chapters in navigation
- [x] T016 ✅ DONE Create `specs/002-book-structure/validation-report.md`:
  - Docusaurus build status (pass/fail)
  - Directory structure check (pass/fail)
  - Chapter count (32 present)
  - Navigation structure (7 parts visible)
  - Ready for early-access: YES/NO

**Checkpoint**: Book structure validated and ready for readers

---

## Critical Path Summary

```
Phase 1 (T001-T003): Create directories + chapter files
    ↓ (~1 day, parallelizable [P]) ✅ DONE
Phase 2 (T004): Create Part 1 spec with orchestrator narrative
    ↓ (~1 day)
Phase 3 (T013-T016): Validate Docusaurus + prepare for early access
    ↓ (~1 day) ✅ DONE
NEXT: Invoke chapter-planner subagent with Part 1 spec
    → Clarifications resolved during planning (just-in-time per chapter)
LAUNCH: Early access live, Part 1 writers ready to begin
```

**Total Timeline**: ~2-3 days remaining to Part 1 chapter planning

---

## What's NOT Needed

❌ **Removed (were redundant)**:
- Quality validation guides (plan.md already documents everything)
- Reader onboarding guides (part intro.md files serve this purpose)
- Writer communication plans (simple notification: "spec ready in specs/part-1/")
- Early-access launch checklists (T016 validation report covers this)
- Subagent workflow guides (CLAUDE.md already documents 4-phase SDD loop)
- Skill integration/validation guides (Constitution + skills are source of truth)
- Output style guides (already exist in `.claude/output-styles/`)

✅ **What we ACTUALLY have**:
- 32 chapter placeholder files (ready for writers)
- 7 part intro files (tell readers what each part is about)
- Part 1 spec (tells writers what to write)
- Validation report (confirms ready for launch)

---

## Execution

### Sequential (Current Path)
1. ✅ Phase 1 (T001-T003) - Complete
2. ✅ Phase 3 (T013-T016) - Complete
3. → Phase 2 (T004) - Create Part 1 spec NOW
4. → Invoke chapter-planner subagent with Part 1 spec
5. → chapter-planner will request clarifications during planning (just-in-time)

---

## Success Criteria

By end of Phase 2 (T004):
- ✅ All 7 part directories created (Phase 1)
- ✅ All 32 chapter folders created with README + 3 lesson placeholders (Phase 1)
- ✅ 7 part intro files created (explaining each part) (Phase 1)
- ✅ Docusaurus builds with zero errors (Phase 3)
- ✅ All 7 parts + 32 chapters visible in sidebar (Phase 3)
- → Part 1 spec created with orchestrator narrative (Phase 2, in progress)
- → Ready to invoke chapter-planner subagent for Part 1 planning
- → Early-access launch ready (all infrastructure complete)

---

## Next Phase (After T004)

Once Part 1 spec created:
1. **Invoke chapter-planner subagent**: Use `specs/part-1/part-1-spec.md` as input
2. **Output**: `specs/part-1/chapter-01-plan.md` through `specs/part-1/chapter-05-plan.md`
3. **chapter-planner will request clarifications**: Just-in-time during planning phase
4. **Then invoke content-implementer subagent**: Write each chapter's lessons one by one
5. **When Part 1 is in implementation**: Create Part 2 spec (same process, deferred)

## Deferred Tasks (Just-In-Time Per Part)

These clarifications will be resolved when each part is ready for planning:
- **Part 2**: Agent frameworks to teach, complete agent implementation example
- **Part 3**: Six-Part Prompting Framework definition
- **Part 4**: Real project selection for Chapter 21
- **Part 5**: Case studies for Chapter 25
- **Part 6**: Agent frameworks, implementation examples
- **Part 7**: MCP server implementation example, MCP integration applications

No separate planning documents, guides, or checklists needed. Just specs and chapters.
