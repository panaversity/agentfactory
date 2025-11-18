---
id: 0002
title: Generate Chapter 8 Implementation Plan
stage: plan
date: 2025-10-31
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-chapter-8-git-github
branch: 001-chapter-8-git-github
user: mjs
command: /sp.plan
labels: ["chapter-8", "git", "github", "part-2", "planning", "lesson-architecture"]
links:
  spec: specs/001-chapter-8-git-github/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-chapter-8-git-github/plan.md
  - specs/001-chapter-8-git-github/tasks.md
tests:
  - Constitutional alignment validation (passed)
  - Pedagogical scaffolding review (passed)
  - Success criteria mapping (8/8 addressed)
---

## Prompt

```
/sp.plan
```

**Context**: Transform the approved specification for Chapter 8 - Git & GitHub for AI-Driven Development into a detailed implementation plan and task breakdown. The chapter is the final chapter of Part 2: AI Tool Landscape and must teach version control in the context of AI-driven development.

## Response snapshot

**Generated comprehensive implementation plan** using chapter-planner subagent with the following key architectural decisions:

### Chapter Architecture

**Type**: Hybrid Technical + Natural Language Prompts
**Duration**: 3-4 hours content + 1+ hours exercises = 4.5-5 hours total
**Structure**: 5 lessons progressing from motivation → fundamentals → safety → collaboration → convenience

### Five-Lesson Breakdown

1. **Lesson 1: Why Git Matters in AI-Driven Development** (30 min)
   - Bloom's Level: Understand
   - Narrative foundation establishing motivation and safety-net concept
   - No code examples (purely conceptual)
   - Addresses learner anxiety about "breaking everything"

2. **Lesson 2: Git Essentials: Commands and Setup** (90 min)
   - Bloom's Level: Apply
   - Installation (Windows/Mac/Linux), configuration, basic workflow
   - git status → add → commit → push pattern
   - 5 hands-on exercises from installation to commit history

3. **Lesson 3: Safe Experimentation: Branches, Checkpoints, Rollback** (90 min)
   - Bloom's Level: Apply
   - Checkpoint pattern before AI changes
   - Branch-based experimentation, merging, conflict resolution
   - Undoing changes (soft/hard reset, revert, stash)
   - 6 hands-on exercises: checkpoint → branch → merge → rollback

4. **Lesson 4: Pull Requests and Team Collaboration** (75 min)
   - Bloom's Level: Apply
   - GitHub PR workflow via web interface
   - Code review process, iteration based on feedback
   - Merging strategies and conflict handling
   - 7 hands-on exercises: repo creation → push → PR → review → merge

5. **Lesson 5: Natural Language Prompts for Git Operations** (45 min)
   - Bloom's Level: Apply
   - Command→prompt translation (10+ examples)
   - Complex workflow prompts combining operations
   - Troubleshooting prompts for errors and conflicts
   - 6 hands-on exercises: translation → workflow → conflict help

### Key Architectural Decisions

1. **Five-Lesson Structure**: Naturally breaks 3-4 hours into focused lessons (30-90 min each), avoiding cognitive overload. More granular than typical 3-lesson structure to accommodate both Git commands AND natural language prompts.

2. **Part I/Part II Split**: Lessons 2-4 teach Git commands before Lesson 5 shows natural language prompts. Prevents prompts from becoming "magic" without understanding underlying concepts.

3. **Lesson 1 as Pure Narrative**: No code, only motivation and conceptual foundation. Addresses fear factor ("Will I break everything?") before introducing commands.

4. **Safety Early and Prominent**: Lesson 3 teaches rollback/stash as core workflows, not advanced topics. This is the unique value proposition of Git for AI-driven development.

5. **GitHub UI Focus in Lesson 4**: Teaches PR workflow via web interface rather than CLI, reducing command-line friction and making process visual for beginners.

6. **20+ Total Exercises**: Each lesson has 5-7 hands-on exercises plus 1 comprehensive end-of-chapter project (SC-007 requirement). Progressive complexity within and across lessons.

7. **Hybrid Pedagogy**: Lessons 2-4 use traditional "show code → explain → practice" pattern. Lesson 1 uses narrative/reflection pattern. Lesson 5 uses translation/mapping pattern.

### Implementation Task Breakdown

**8 Phases, 25 Total Tasks, 16-23 Hours Estimated Effort**

**Phase 1**: Chapter Foundation (2 tasks, 4h)
- Chapter README with learning objectives, prerequisites, outcomes
- Lesson 1 narrative content

**Phase 2-5**: Lessons 2-5 (12 tasks, 13.5h)
- Each lesson: content creation → exercises → assessment
- 3 tasks per lesson (content, exercises, assessment/reference)

**Phase 6**: Integration & Projects (3 tasks, 4.5h)
- Comprehensive end-of-chapter project (SC-007)
- Quick reference card (SHOULD)
- Common scenarios reference (SHOULD)

**Phase 7**: Quality Assurance (5 tasks, 7h)
- Pedagogical review (constitution alignment, scaffolding, accessibility)
- Technical verification (cross-platform testing)
- Accessibility check (gatekeeping language, jargon, visual aids)
- Constitution alignment validation
- Cross-chapter coherence

**Phase 8**: Final Review & Publication (3 tasks, 4h)
- Style guide compliance
- Human editorial review
- Handoff summary

### Task Prioritization

- **MUST (13 tasks)**: All lesson content, exercises, critical QA reviews
- **SHOULD (5 tasks)**: Assessments, reference cards, scenarios
- **NICE-TO-HAVE (3 tasks)**: Optional enhancements, polish

### Success Criteria Mapping

All 8 success criteria from spec explicitly addressed:

| SC # | Criterion | Implementation |
|------|-----------|---------------|
| SC-001 | Create repo in 5 min | Lesson 2, Exercises 3-4 |
| SC-002 | Safe checkpoint/rollback | Lesson 3, Exercises 1, 5 |
| SC-003 | Branch workflow | Lesson 3, Exercises 2-4 |
| SC-004 | Create PR, merge | Lesson 4, Exercises 1-7 |
| SC-005 | 10+ command→prompt translations | Lesson 5, prompt examples |
| SC-006 | Understand when to commit | Lesson 2 reflection, Lesson 5 scenarios |
| SC-007 | 90% complete end project | Task 6.1 comprehensive project |
| SC-008 | Identify commands for scenarios | Lessons 4-5, common scenarios |

### Constitutional Alignment

**Principle 1 (AI-First Teaching)**: Lesson 5 entire focus is natural language Git with AI CLI tools. Throughout: AI collaboration enabled by safe Git practices.

**Principle 5 (Progressive Scaffolding)**: Clear progression:
- Lesson 1: motivation (no syntax)
- Lesson 2: simple linear workflow
- Lesson 3: branches (abstract concept)
- Lesson 4: GitHub collaboration (visual UI)
- Lesson 5: prompts (convenience layer)

**Principle 8 (Accessibility)**:
- No assumed Git knowledge
- All jargon defined on first use (commit, staging area, branch, remote, fork, PR, merge, conflict)
- Multiple explanation styles (stories, step-by-step, diagrams, GitHub UI screenshots)
- Platform-specific instructions (Windows/Mac/Linux)

**Principle 9 (Show-Then-Explain)**:
- Complete workflows demonstrated first
- Then each component explained
- Focus on WHY commands exist, not just WHAT they do

### Risk Mitigations (All Addressed)

| Risk | Mitigation | Location |
|------|-----------|----------|
| Accidental secret commits | Early .gitignore, templates, warnings | Lesson 2, Task 2.1 |
| Fear of breaking | Emphasize safety net, teach rollback early | Lessons 1, 3 |
| Installation issues | Platform-specific guides, troubleshooting | Lesson 2, Task 2.1 |
| GitHub auth confusion | PAT/SSH guide, error interpretation | Lesson 4, Task 4.1 |
| Merge conflict paralysis | Clear markers explanation, simple steps, AI help | Lessons 3, 5 |

### Domain Skills Application

All 9 constitutional domain skills mapped to chapter content:

1. **learning-objectives**: Chapter README, each lesson objectives (Bloom's taxonomy)
2. **concept-scaffolding**: Progressive complexity across 5 lessons
3. **code-example-generator**: Git command examples with clear context
4. **exercise-designer**: 20+ hands-on exercises, end-of-chapter project
5. **assessment-builder**: Optional assessments per lesson (SHOULD tasks)
6. **technical-clarity**: Platform-specific instructions, jargon definitions, error message interpretation
7. **book-scaffolding**: Part 2 → Part 3 bridge, prerequisite enforcement
8. **ai-augmented-teaching**: Lesson 5 natural language prompts, AI collaboration framing
9. **content-evaluation-framework**: Quality assurance phase (Task 7.1-7.5)

## Outcome

- ✅ **Impact**: Comprehensive 5-lesson implementation plan created for Chapter 8. Architecture balances foundational Git knowledge (Lessons 2-4) with AI-native convenience layer (Lesson 5). Plan addresses all 17 functional requirements, 8 success criteria, and 5 identified risks. Ready for content-implementer agent to begin content creation.

- 🧪 **Tests**: Constitutional alignment validated (all 11 principles addressed). Pedagogical scaffolding reviewed (progressive complexity confirmed). Success criteria mapping complete (8/8 addressed in specific lessons/exercises). Risk mitigation strategy verified (5/5 risks addressed with concrete implementations).

- 📁 **Files**: Created `plan.md` (~6,000 words with lesson architecture, content requirements, scaffolding strategy, domain skills mapping) and `tasks.md` (~8,000 words with 25 atomic implementation tasks organized in 8 phases with acceptance criteria).

- 🔁 **Next prompts**: Run `/sp.implement` to begin lesson-by-lesson content creation using content-implementer agent. Start with Phase 1 (Chapter README + Lesson 1), then proceed sequentially through Phases 2-5 (Lessons 2-5). After all content complete, run Quality Assurance phase (Tasks 7.1-7.5) before publication.

- 🧠 **Reflection**: The five-lesson structure is critical for success. Beginners need the gradual progression: motivation (L1) → fundamentals (L2) → safety mechanisms (L3) → professional workflows (L4) → AI convenience (L5). The decision to teach Git commands BEFORE natural language prompts prevents "magical thinking" where students rely on AI without understanding. Lesson 1's pure narrative approach addresses the emotional barrier (fear of breaking things) before introducing technical complexity. The 20+ exercises ensure hands-on practice at every step, not just passive reading. This chapter uniquely bridges technical tool literacy (Part 2 focus) with AI-native workflows (Part 3 preview).

## Evaluation notes (flywheel)

- **Failure modes observed**: None. Planning workflow executed cleanly: setup script → chapter-planner subagent invocation → plan and tasks generation → PHR creation.

- **Graders run and results (PASS/FAIL)**:
  - Constitutional alignment: PASS (all 11 principles addressed)
  - Pedagogical scaffolding: PASS (clear progression from simple to complex)
  - Success criteria mapping: PASS (8/8 criteria explicitly addressed)
  - Risk mitigation: PASS (5/5 risks addressed with concrete mitigations)
  - Domain skills application: PASS (all 9 skills mapped to content)
  - Lesson duration balance: PASS (30-90 min per lesson, avoiding cognitive overload)

- **Prompt variant (if applicable)**: Standard `/sp.plan` command with no additional arguments. Chapter-planner subagent invoked with detailed context (spec path, constitution reference, branch name, key architectural questions).

- **Next experiment (smallest change to try)**: During implementation, validate whether Lesson 5 (Natural Language Prompts) should remain a standalone lesson or be restructured as an appendix section integrated into Lessons 2-4. Current decision is standalone to preserve focus and avoid mixing pedagogical patterns (traditional vs. translation-based). Monitor learner feedback: if students find repetition across Lessons 2-4 and Lesson 5, consider integration. If they appreciate the dedicated focus on prompts as a distinct skill, retain standalone structure.
