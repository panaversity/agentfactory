---
id: 0148
title: Chapter 31 Tasks Generation - Human Control Emphasis
stage: tasks
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 10-chapter-31-redesign
branch: 10-chapter-31-redesign
user: mjs
command: /sp.tasks
labels:
  [
    "tasks",
    "chapter-31",
    "human-control",
    "checkpoint-pattern",
    "adr-governance",
    "part-5",
  ]
links:
  spec: specs/10-chapter-31-redesign/spec.md
  ticket: null
  adr: history/adr/001-part-5-workflow-isomorphism-pedagogy.md, history/adr/002-part-5-progressive-proficiency-scaffolding.md, history/adr/003-part-5-human-control-checkpoint-pattern.md
  pr: null
files:
  - specs/10-chapter-31-redesign/tasks.md (completely regenerated with human control emphasis, checkpoint pattern, ADR governance)
tests:
  - none (task planning document)
---

## Prompt

now let's plan the tasks @specs/10-chapter-31-redesign/

**Context**: User ran `/sp.tasks` after completing plan revision (PHR-0007) and ADR creation (PHR-0004, ADRs 001-003). Request to regenerate tasks.md incorporating:

- Human control & checkpoint pattern emphasis (ADR-003)
- Part 5 chapter structure corrections (Chapters 30-32, not 25-27)
- Workflow isomorphism pattern (ADR-001)
- CEFR proficiency scaffolding (ADR-002)
- Updated plan.md with all recent changes

## Response snapshot

Generated comprehensive tasks.md with 52 tasks across 8 phases, organized by 4 user stories from spec.md.

**Critical Updates Incorporated**:

1. **Human Control Emphasis** (ADR-003):

   - All lessons emphasize "YOU control workflow; AI orchestrator collaborates"
   - Checkpoint pattern integrated in Lessons 6-8 (Tasks, Implement, Capstone)
   - "AI generates, YOU validate and decide" messaging throughout

2. **Checkpoint Pattern Documentation**:

   - **Lesson 6** (Tasks Phase): Demonstrate "Generate Phase 1 tasks, pause for review" pattern
   - **Lesson 7** (Implement Phase): Demonstrate "Implement Phase 1 tasks, pause for review" pattern
   - **Lesson 8** (Capstone): Apply checkpoints at every phase
   - Prevents autonomous loops; keeps human in control

3. **ADR Governance Applied**:

   - ADR-001: 8 lessons mirror actual Spec-Kit Plus phases (workflow isomorphism)
   - ADR-002: CEFR proficiency levels validated (A2: max 5 concepts, B1: max 7, B2: max 10)
   - ADR-003: Human control & checkpoint pattern emphasized throughout

4. **Part 5 Structure Corrections**:

   - Updated references from "Chapters 25-27" to "Chapters 30-32"
   - Correct book structure alignment

5. **Task Organization** (52 tasks across 8 phases):

   - **Phase 1**: Setup (2 tasks)
   - **Phase 2**: Foundational Documents (6 tasks)
   - **Phase 3**: User Story 1 - Installation (6 tasks) → Lesson 1
   - **Phase 4**: User Story 2 - Constitution & Specify (8 tasks) → Lessons 2-3
   - **Phase 5**: User Story 3 - Workflow Cascade with Checkpoints (12 tasks) → Lessons 4-6
   - **Phase 6**: User Story 4 - Implementation with Validation (8 tasks) → Lesson 7
   - **Phase 7**: Capstone Integration (4 tasks) → Lesson 8
   - **Phase 8**: Validation & Polish (6 tasks)

6. **Checkpoint Pattern Integration**:

   - Task T025 (Lesson 6): **Checkpoint Pattern Demonstration** with Part B showing "Generate Phase 1 tasks, pause"
   - Task T035 (Lesson 7): **Checkpoint Implementation Demo** with Part B showing "Implement Phase 1, pause"
   - Task T041 (Lesson 8): **Checkpoint Pattern Throughout** - applied at every workflow phase
   - Reference documents: checkpoint-pattern-reference.md, validation-mastery-guide.md

7. **Proficiency Scaffolding**:

   - Lessons 1-3: A2 (Basic Application) - Max 5 concepts
   - Lessons 4-6: B1 (Intermediate Application) - Max 7 concepts
   - Lesson 7: B1-B2 (Intermediate to Advanced) - Max 10 concepts
   - Lesson 8: B2 (Advanced) - Synthesis & design

8. **Deliverables**:
   - 8 lesson content files (01-08.md)
   - 30+ support files (examples, exercises, assessments, prompts, references)
   - 12+ spec/plan support files (assessments, rubrics, guides, reports, checklists)

**Key Files to Create**:

- Checkpoint pattern reference document
- Validation mastery guide
- Task lineage reference
- ADR creation guide
- Constitutional alignment report
- Proficiency validation report

## Outcome

- ✅ Impact: Complete tasks.md regenerated with human control emphasis, checkpoint pattern integration, ADR governance, and Part 5 structure corrections; provides clear implementation roadmap for 8-lesson chapter with 52 atomic tasks; aligns with all 3 ADRs (workflow isomorphism, proficiency scaffolding, human control pattern)
- 🧪 Tests: No tests required (planning document); acceptance criteria: All tasks follow checklist format [TaskID] [P?] [Story?] description + file path; tasks organized by user story; checkpoint pattern demonstrated in Lessons 6-8; proficiency levels validated
- 📁 Files: Updated specs/10-chapter-31-redesign/tasks.md (completely regenerated - 527 lines, 52 tasks, 8 phases); created PHR-0148
- 🔁 Next prompts: Begin implementation with Phase 1 (Setup); create directory structure and verify branch; start Phase 2 (Foundational Documents) with README, templates, glossary; implement lessons sequentially (MVP: Lessons 1-3, then 4-7, then 8)
- 🧠 Reflection: The checkpoint pattern is CRITICAL - without explicit emphasis, students might expect autonomous AI execution (dangerous misunderstanding). Tasks.md now makes checkpoint pattern visible in 3 key lessons (6, 7, 8) with explicit demonstration requirements. ADR governance ensures consistency: all 52 tasks align with architectural decisions for workflow isomorphism (ADR-001), proficiency scaffolding (ADR-002), and human control (ADR-003). The 4 user stories from spec.md provide natural organization boundaries (Installation, Constitution/Specify, Workflow Cascade, Implementation/Validation, Capstone). Each phase has clear gates ensuring quality progression. Parallel execution opportunities identified (up to 9 tasks parallel in Phase 5). Total time: 18-22 hours estimated.

## Evaluation notes (flywheel)

- Failure modes observed: Previous tasks.md (before today's updates) lacked checkpoint pattern emphasis and human control messaging; could lead students to misunderstand AI as autonomous agent rather than collaborator; Part 5 chapter numbering was incorrect (25-27 instead of 30-32)
- Graders run and results (PASS/FAIL): PASS - All 52 tasks follow checklist format with [TaskID] [P?] [Story?] tags; tasks organized by 4 user stories; checkpoint pattern explicit in T025, T035, T041; human control emphasized throughout; proficiency levels documented; ADR governance applied (references to ADR-001, ADR-002, ADR-003); Part 5 structure corrected
- Prompt variant (if applicable): User provided context reference (@specs/10-chapter-31-redesign/) after completing plan revision and ADR creation; regeneration required to incorporate recent updates
- Next experiment (smallest change to try): During implementation, validate that checkpoint pattern examples are clear enough for students to replicate; may need visual diagrams showing: Human → Command → Agent Work → Checkpoint → Human Review → Decision → Continue/Adjust cycle
