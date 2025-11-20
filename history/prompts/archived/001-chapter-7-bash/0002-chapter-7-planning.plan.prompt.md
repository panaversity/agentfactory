---
id: 0002
title: Chapter 7 Planning
stage: plan
date: 2025-10-31
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-chapter-7-bash
branch: 001-chapter-7-bash
user: HP
command: /sp.plan
labels: ["chapter-7", "bash", "planning", "lesson-design", "ai-driven-development"]
links:
  spec: specs/001-chapter-7-bash/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-chapter-7-bash/plan.md
tests:
  - Constitutional alignment check (all 11 principles addressed)
  - Requirements coverage verification (all 15 FRs mapped)
  - User story mapping validation (P1, P2, P3 priorities)
---

## Prompt

/sp.plan (no additional arguments)

## Response snapshot

Successfully generated comprehensive lesson-by-lesson implementation plan for Chapter 7 using the chapter-planner subagent.

**Plan Structure (292 lines)**:
- 8 lessons across two parts (Part I: Bash Commands, Part II: AI-Augmented Workflows)
- Total duration: 3-4 hours (180-240 minutes)
- Progressive scaffolding from beginner to AI-augmented professional

**Part I: Bash Commands Fundamentals (Lessons 1-5, ~140 min)**:
1. The Terminal Interface (25-30 min) - Understanding terminal anatomy, paths, command structure
2. Navigation and File Management (40-45 min) - cd, pwd, ls, mkdir, touch, cp, mv, rm with safety
3. Viewing and Searching Content (30-35 min) - cat, head, tail, less, grep, pipes, find
4. Environment Variables & Package Management (35-40 min) - export, .bashrc/.zshrc, pip, npm, brew
5. Process Management & Troubleshooting (30-35 min) - ps, top, kill, common errors, diagnosis

**Part II: AI-Augmented Bash Workflows (Lessons 6-8, ~110 min)**:
6. Natural Language Prompts for Bash (35-40 min) - Describing tasks to AI, validation, iteration
7. Professional Bash Habits (30-35 min) - Shortcuts (Ctrl+R, Tab), aliases, when to memorize vs. ask AI
8. Real-World AI-Assisted Workflows (35-40 min) - Integrated workflows, decision matrices, risk management

**Coverage Verification**:
✓ All 15 functional requirements (FR-001 to FR-015) mapped to specific lessons
✓ All 3 prioritized user stories addressed (P1→Lessons 1-5, P2→Lessons 6-8, P3→Lessons 7-8)
✓ All 10 success criteria with validation points in each lesson
✓ All 11 constitutional principles explicitly addressed
✓ All 9 domain skills integrated (learning objectives, scaffolding, examples, exercises, assessment, clarity, book scaffolding, AI-augmented teaching, evaluation)

**Key Pedagogical Decisions**:
- Two-part structure prevents AI over-reliance: foundation first, then AI augmentation
- Progressive complexity: Lesson 1 (comfortable) → Lessons 2-5 (gradual increase) → Lesson 6 (paradigm shift) → Lessons 7-8 (synthesis)
- Platform support: macOS, Linux, Windows (Git Bash/WSL) with specific notes
- Heavy scaffolding appropriate for complete beginners
- Emphasis on understanding commands before using AI to generate them

## Outcome

- ✅ Impact: Comprehensive 8-lesson plan ready for implementation. Transforms specification into actionable lesson structure with clear acceptance criteria, time estimates, and pedagogical rationale.
- 🧪 Tests: Constitutional alignment verified, requirements coverage validated, user story mapping confirmed, success criteria with validation points established
- 📁 Files: Created plan.md (292 lines) with complete lesson breakdown, prerequisites, content outlines, exercises, and acceptance criteria
- 🔁 Next prompts: Ready for lesson implementation - use content-implementer subagent to create individual lesson content files (01-08) following the plan
- 🧠 Reflection: The two-part structure effectively balances foundational command knowledge (Part I) with AI-augmented productivity (Part II), preventing over-reliance while maximizing learner capability. Progressive scaffolding and clear prioritization (P1/P2/P3) enable iterative development if needed.

## Evaluation notes (flywheel)

- Failure modes observed: None - chapter-planner subagent successfully generated complete plan on first attempt
- Graders run and results (PASS/FAIL): PASS - Constitutional alignment check passed, requirements coverage complete, all acceptance criteria defined
- Prompt variant (if applicable): Standard /sp.plan workflow with chapter-planner subagent
- Next experiment (smallest change to try): Implement Lesson 1 using content-implementer subagent to validate plan provides sufficient detail for content creation
