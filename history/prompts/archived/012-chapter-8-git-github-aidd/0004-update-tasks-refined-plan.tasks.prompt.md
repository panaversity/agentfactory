---
id: 0004
title: Update Tasks Refined Plan
stage: tasks
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 012-chapter-8-git-github-aidd
branch: main
user: HP
command: /sp.tasks
labels: ["chapter-8", "git", "github", "tasks", "lesson-duration", "ide-options", "installation-links", "capstone-simplified"]
links:
  spec: specs/012-chapter-8-git-github-aidd/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-chapter-8-git-github-aidd/tasks.md (completely rewritten)
tests:
  - Task checklist format validation (PASS)
  - User story organization (PASS)
  - Lesson duration alignment with plan (PASS)
  - IDE options reduced to 3 (PASS)
  - Installation links instead of screenshots (PASS)
  - Effort estimates updated (PASS)
---

## Prompt

based on upadated plan, update tasks too

## Response snapshot

Completely rewrote Chapter 8 tasks.md to align with refined plan adjustments:

**Key Updates Made**:

1. **Lesson Duration Adjustments (15-20 min)**:
   - Updated all task descriptions to reflect new durations
   - Lesson 1: 15 min (conceptual)
   - Lessons 2-9: 20 min each
   - Total: 175 min = 3 hours (down from 3.5-4 hours)
   - Task T052 validates lesson durations match plan

2. **IDE Options Reduced to 3** (Tier 1 cognitive load):
   - Task T036: Create IDE comparison table with ONLY 3 options (VS Code, Cursor, Zed)
   - Removed all PyCharm references
   - Note added: "PyCharm removed to respect Tier 1 cognitive load (max 3 options)"
   - Task T053: Verify "Exactly 3 (VS Code, Cursor, Zed - no PyCharm)"
   - Tasks T037-T039: Installation guides for 3 IDEs only

3. **Installation Links Instead of Screenshots**:
   - Tasks T008-T011: Windows, Mac, Linux Git installation with links + text descriptions
   - **Explicit note**: "No screenshots required: Use text descriptions and installation link"
   - Task T011: GitHub account creation with link (https://github.com), numbered steps, no screenshots
   - Tasks T037-T039: IDE installation with links, no screenshots
   - Task T054: Validate "all installation guidance uses links + text descriptions (no screenshot requirements)"
   - Success criteria: "No screenshots mentioned as required"

4. **Capstone Simplified for Non-Coders**:
   - Task T044: Updated scenario - "AI writes ALL code; you manage Git for safety"
   - Note: "AI generates ALL code - students focus on Git workflow and safety practices. No coding knowledge assumed."
   - Task T045: Workflow steps with **You** vs **AI** roles clearly labeled
   - Step 2 focus: "Understanding Git workflow, NOT writing code"
   - Step 3 focus: "Branch workflow for safe experimentation"
   - Tasks T046-T047: Specifications emphasize "Student does NOT write code: AI generates everything"

5. **Effort Estimates Updated**:
   - Header: **Estimated Effort**: 29-42 hours (writing 22-31h + review 7-11h)
   - Note: "Reduced from original 39-52h due to: (1) Shorter lessons (15-20 min vs 25-30 min), (2) 3 IDE options instead of 4, (3) Installation links instead of screenshots, (4) Simplified capstone for non-coders"
   - Task summary: Writing 22-31 hours (9 lessons × 2-3h avg, reduced from 30-39h)
   - Review: 7-11 hours (link verification, testing, polish, reduced from 9-13h)

6. **Natural Language Prompts Paragraph**:
   - Task T028: Add explanatory paragraph in Lesson 6 (GitHub Integration)
   - Key principle documented: "You don't memorize `git merge feature-branch`. You ask your AI: 'Merge the feature branch into main.' AI translates intent into commands."

7. **Task Organization by User Story**:
   - Phase 3: US1 (P1) - Git Safety Basics (17 tasks, Lessons 1-4)
   - Phase 4: Integration - Branching (3 tasks, Lesson 5)
   - Phase 5: US2 (P2) - GitHub Integration (6 tasks, Lesson 6)
   - Phase 6: US4 (P3) - Pull Requests (5 tasks, Lesson 7)
   - Phase 7: US3 (P2) - IDE Setup (9 tasks, Lesson 8)
   - Phase 8: Integration - Capstone (6 tasks, Lesson 9)
   - Phase 9: Polish (13 tasks)

8. **Task Checklist Format** (Strict Compliance):
   - Every task follows: `- [ ] T### [P] [US#] Description with file path`
   - 62 total tasks, 28 parallelizable (marked [P])
   - User story labels: [US1], [US2], [US3], [US4] appropriately assigned
   - All tasks include specific file paths

9. **Validation Tasks Added**:
   - Task T052: Validate lesson durations match plan (15-20 min each)
   - Task T053: Verify cognitive load within Tier 1 limits (3 IDE options, max 5 concepts)
   - Task T054: Verify installation guidance uses links + text (no screenshots)
   - Task T055: Check reading level (Grade 7 or below with Flesch-Kincaid)
   - Task T056: Validate Git commands accurate for Git 2.40+
   - Task T057: Verify all external links valid and current

10. **Implementation Strategy Updated**:
    - MVP Scope: Tasks T001-T020 (Setup + US1 Lessons 1-4)
    - Dependencies clearly mapped: US1 → Integration → US2 → US4
    - Parallel execution examples with week-by-week breakdown
    - Critical path vs parallel path identified

**Task Summary**:
- **Total Tasks**: 62 tasks
- **Parallelizable**: 28 tasks marked [P]
- **MVP Scope**: Tasks T001-T020 (Setup + US1 Lessons 1-4)
- **Estimated Effort**: 29-42 hours (reduced from 39-52h)

**Notes for Lesson Writers Section Updated**:
- Point 2: "Duration Constraint: Each lesson is 15-20 minutes"
- Point 3: "For IDE, present exactly 3 options (VS Code, Cursor, Zed - no PyCharm)"
- Point 4: "Use installation links + text descriptions. NO screenshot requirements"
- Point 5: "In capstone (Lesson 9), emphasize AI generates ALL code"

## Outcome

- ✅ Impact: tasks.md completely rewritten and aligned with refined plan. All 6 user requirements from plan adjustments reflected in task specifications. Ready for content-implementer implementation.
- 🧪 Tests: Task checklist format PASS (all 62 tasks follow format), User story organization PASS (clear phases), Lesson duration alignment PASS (15-20 min), IDE options PASS (3 only), Installation links PASS (no screenshots), Effort estimates PASS (29-42h)
- 📁 Files: Completely rewrote tasks.md (578 lines), organized by user story, 62 actionable tasks with clear acceptance criteria and file paths
- 🔁 Next prompts: Ready for content-implementer subagent implementation. Begin with MVP scope (Tasks T001-T020: Setup + Lessons 1-4). After MVP validation, proceed with remaining tasks (T021-T062).
- 🧠 Reflection: Tasks now precisely match refined plan constraints. Critical updates: (1) All lesson durations 15-20 min for engagement, (2) Exactly 3 IDE options respecting cognitive load, (3) Installation links reduce maintenance burden (no screenshot updates needed), (4) Capstone emphasizes AI does all coding (true beginner-friendly), (5) Effort reduced 20-25% due to streamlined content. Task organization by user story enables incremental delivery and independent testing. MVP scope (Lessons 1-4) allows early validation before investing in advanced topics (GitHub, IDE, PRs).

## Evaluation notes (flywheel)

- Failure modes observed: None. All tasks aligned with refined plan successfully.
- Graders run and results (PASS/FAIL): Task format compliance PASS, User story mapping PASS, Duration alignment PASS, IDE options PASS, Installation guidance PASS, Effort estimates PASS, MVP scope clearly defined PASS
- Prompt variant (if applicable): User requested tasks update based on refined plan
- Next experiment (smallest change to try): Begin content-implementer implementation with MVP scope (Tasks T001-T020). Monitor: (1) Can lessons actually be completed in 15-20 min? (2) Are text-based installation instructions clear without screenshots? (3) Is capstone achievable in 20 min with AI generating all code? Validate with user testing before proceeding to full implementation.
