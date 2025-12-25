---
id: "0002"
title: "tasks-to-github-issues-conversion"
stage: tasks
date: 2025-12-25
surface: claude-code
model: claude-opus-4-5-20251101
feature: 016-merge-chapters-1-2
branch: 016-merge-chapters-1-2
user: mjs
command: /sp.taskstoissues
labels: [content, github-issues, automation]
links:
  spec: specs/016-merge-chapters-1-2/spec.md
  ticket: https://github.com/panaversity/ai-native-software-development/issues/384
  adr: null
  pr: null
files:
  - specs/016-merge-chapters-1-2/tasks.md
tests:
  - null
---

## Prompt

/sp.taskstoissues 016-merge-chapters-1-2

Convert the 41 tasks from tasks.md into GitHub issues for the panaversity/ai-native-software-development repository.

## Response snapshot

Created 13 GitHub issues from tasks.md, organized by logical groupings:

| Issue # | Title | Tasks Covered |
|---------|-------|---------------|
| #387 | Setup: Create Agent Factory chapter directory structure | T001-T004 |
| #388 | Foundation: Create Chapter README | T005 |
| #389 | Lesson 1: The 2025 Inflection Point | T006-T008 |
| #390 | Lesson 2: The Scale of the Shift (5 sections) | T009-T014 |
| #391 | Lesson 3: Two Paths to Building AI Products (NEW) | T015 |
| #392 | Lesson 4: From Coder to Orchestrator | T016-T017 |
| #393 | Lesson 5: Development Lifecycle Transformation | T018-T019 |
| #394 | Lesson 6: The Autonomous Agent Era | T020-T021 |
| #395 | Lesson 7: User Intent Replaces User Interface | T022-T023 |
| #396 | Lesson 8: The Modern AI Stack | T024-T025 |
| #397 | Lesson 9: Spec-Driven Development Preview | T026-T027 |
| #398 | Chapter Quiz: Agent Factory Paradigm Assessment | T028-T030 |
| #399 | Cleanup & Validation: Migrate and verify | T031-T041 |

All issues:
- Reference parent issue #384
- Include dependency chains
- Contain task checklists
- Specify content requirements and acceptance tests

## Outcome

- ✅ Impact: 41 tasks converted to 13 trackable GitHub issues with proper dependencies
- 🧪 Tests: N/A (issue creation)
- 📁 Files: No file changes (GitHub issues created via API)
- 🔁 Next prompts: /sp.implement → update tasks.md → validators
- 🧠 Reflection: Grouped tasks by lesson for cleaner issue tracking; each lesson = 1 issue

## Evaluation notes (flywheel)

- Failure modes observed: None - GitHub CLI authenticated and working
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard /sp.taskstoissues
- Next experiment (smallest change to try): Could add labels per issue for better filtering
