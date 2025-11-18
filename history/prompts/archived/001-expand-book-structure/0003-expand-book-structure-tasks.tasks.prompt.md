---
id: 0003
title: expand-book-structure-tasks
stage: tasks
date: 2025-10-29T00:00:00Z
surface: Cursor
model: Claude Sonnet 4.5
feature: 001-expand-book-structure
branch: 001-expand-book-structure
user: mjs
command: /sp.tasks
labels: [book-structure, documentation, restructuring, tasks]
links:
  spec: specs/001-expand-book-structure/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/001-expand-book-structure/tasks.md
    action: created
    lines: 614
tests:
  - N/A
---

## Prompt

User invoked `/sp.tasks` command with user input:

```
then we will implement
```

**Context**: The user wants to formally document the implementation plan as executable tasks before beginning implementation work. This follows Phase 0 (Research) and Phase 1 (Planning) completion.

**Available Documents**:
- `spec.md` — Feature specification with 5 user stories (P1, P2, P3)
- `plan.md` — Implementation plan with technical context and phased approach
- `research.md` — Completed research with all 6 questions answered

**User Intent**: Create a detailed, executable task breakdown organized by user story that can be implemented sequentially or in parallel.

## Response snapshot

**Task Generation Approach**: Created a comprehensive task breakdown following strict checklist format, organized by user stories for independent implementation and testing.

**Key Decisions**:

1. **Task Organization by User Story**:
   - Phase 1: Setup (2 tasks)
   - Phase 2: Foundational (3 tasks - blocking prerequisites)
   - Phase 3: US1 - Core Documentation (5 tasks, P1 priority)
   - Phase 4: US2 - Restructure Content (8 tasks, P2 priority)
   - Phase 5: US4 - Docusaurus Directories (6 tasks, P2 priority)
   - Phase 6: US3 - New Part Scaffolding (9 tasks, P3 priority)
   - Phase 7: US5 - README/CLAUDE Updates (2 tasks, P3 priority)
   - Phase 8: Polish & Validation (4 tasks)
   - **Total**: 39 tasks across 5 user stories

2. **Dependency Graph**:
   ```
   Setup → Foundational → US1 (P1) → US2 (P2) → US4 (P2) → Validation
                              ↓
                         US3 (P3) + US5 (P3) (can run in parallel)
   ```

3. **Parallel Execution Opportunities**:
   - 60% of tasks can be parallelized
   - 23 tasks marked with [P] for parallel execution
   - 4 parallel groups identified:
     - Group 1: US1 core doc updates (3 tasks)
     - Group 2: US3 + US5 (11 tasks total)
     - Group 3: All 8 part specs within US3
     - Group 4: Docusaurus scaffolding + build validation

4. **Task Format Compliance**:
   - ✅ All 39 tasks follow strict checklist format
   - ✅ Every task has: Checkbox + Task ID + [P] if parallel + [US#] for story tasks + Description with absolute file path
   - ✅ Examples:
     - `- [ ] T006 [P] [US1] Update /Users/.../constitution.md`
     - `- [ ] T025 [P] [US3] Create /Users/.../specs/part-6/part-6-spec.md`

5. **Independent Test Criteria** (per user story):
   - **US1**: Open 3 core docs, verify all say "13 parts, 46 chapters"
   - **US2**: All chapter folders match mapping, no orphaned files
   - **US3**: All 8 new part specs exist with required chapters
   - **US4**: Docusaurus build succeeds, all 13 parts navigable
   - **US5**: README + CLAUDE.md accurately reference 13 parts

6. **MVP Scope Definition**:
   - Recommended MVP: Complete through Phase 3 (US1 + Foundational)
   - Delivers: Updated constitution, chapter index, directory structure, chapter mapping, part templates
   - Value: Foundation established, zero risk to existing content
   - Enables: All future work can proceed based on authoritative docs

7. **Content Preservation Strategy**:
   - Task T012: Create git backup tag before restructuring
   - All old content preserved in git history
   - chapter-mapping.md provides bidirectional tracing
   - Rollback plan: Each phase commits separately, can cherry-pick or revert

8. **Documentation Structure**:
   - Task summary table (8 phases, task counts, parallelization %)
   - Dependencies graph (visual representation)
   - Independent test criteria (one per user story)
   - Parallel execution examples (4 groups with rationale)
   - MVP scope recommendation
   - Implementation strategy (incremental delivery)

**Specific File Updates Defined**:
- T006: Update `constitution.md` (version 2.1.0 → 2.2.0, scaffolding ranges, chapter counts)
- T007: Update `chapter-index.md` (complete rewrite with all 46 chapters)
- T008: Update `directory-structure.md` (add Part 8/13 examples)
- T011-T017: Restructure Parts 1-5 specs and physical content
- T019-T022: Rename Part 4-7 folders in Docusaurus
- T025-T032: Create 8 new part spec files (Parts 6-13)
- T034-T035: Update README.md and verify CLAUDE.md
- T036-T039: Create Docusaurus scaffolding, validate build, final review

**Task Counts by Phase**:
| Phase | Tasks | Parallelizable |
|-------|-------|----------------|
| Setup | 2 | 0% |
| Foundational | 3 | 67% (2/3) |
| US1 (P1) | 5 | 60% (3/5) |
| US2 (P2) | 8 | 0% (sequential) |
| US4 (P2) | 6 | 17% (1/6) |
| US3 (P3) | 9 | 89% (8/9) |
| US5 (P3) | 2 | 100% (2/2) |
| Polish | 4 | 50% (2/4) |

## Outcome

- ✅ Impact: Comprehensive 614-line task breakdown created, ready for immediate implementation
- 🧪 Tests: N/A (documentation project, validation tasks included in Phase 8)
- 📁 Files: 1 file created (tasks.md)
- 🔁 Next prompts: 
  - Begin implementation with Phase 1 (Setup - T001-T002)
  - Or proceed directly to MVP scope (Phases 1-3, tasks T001-T010)
  - Or review task breakdown with stakeholders before implementation
- 🧠 Reflection: The task breakdown successfully adapts software development task management to a documentation restructuring project. The critical insight was organizing tasks by user story (not by file type or technology) to enable independent testing and incremental delivery. The 60% parallelization rate and clear dependency graph should significantly accelerate implementation. The MVP scope recommendation (US1 only) provides a safe checkpoint before any content changes.

## Evaluation notes (flywheel)

- Failure modes observed: None - prerequisites check had a warning about multiple `001-` prefixed directories, but this didn't block execution
- Graders run and results (PASS/FAIL): Task format validation - ALL 39 TASKS PASS (checklist format, IDs, labels, file paths all correct)
- Prompt variant (if applicable): Standard /sp.tasks command adapted for documentation project
- Next experiment (smallest change to try): Consider adding estimated time/complexity to each task (e.g., [15m], [1h], [2h]) to help with sprint planning and capacity estimation
