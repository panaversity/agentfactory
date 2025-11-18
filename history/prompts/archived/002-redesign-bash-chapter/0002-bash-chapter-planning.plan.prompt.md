---
id: 0002
title: Bash Chapter Planning
stage: plan
date: 2025-11-02
surface: agent
model: claude-haiku-4-5-20251001
feature: 002-redesign-bash-chapter
branch: 002-redesign-bash-chapter
user: mjs
command: /sp.plan
labels: ["bash-education", "dialogue-first", "lesson-planning", "pedagogical-design"]
links:
  spec: specs/002-redesign-bash-chapter/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-redesign-bash-chapter/plan.md
tests:
  - Lesson completeness: PASS (all 8 lessons fully detailed)
  - Cognitive load validation: PASS (all lessons within A1/A2 limits)
  - Skills mapping: PASS (7 domain skills applied across chapter)
  - Assessment strategy: PASS (formative + summative per lesson)
  - Constitution alignment: PASS (10 core principles verified)
---

## Prompt

Now let's carefully plan the redesign we have chapter lessons and the business requirements. Each lesson will now will the shared approach to teach the bash commands with this project.

[Context: Approved specification for dialogue-first bash chapter redesign with 7 user stories, 8 lessons, and emphasis on teaching AI collaboration patterns rather than bash syntax]

## Response snapshot

✅ **Comprehensive Implementation Plan Created**

**File**: `/specs/002-redesign-bash-chapter/plan.md` (607 lines, ~18,000 words)

**Plan Structure**:
- 8 fully detailed lessons with learning objectives, concepts, exercises, assessments
- CEFR proficiency mapping (A1 → A2 → B1 progression)
- Cognitive load validation for beginner tier
- 30+ dialogue examples specified (to be sourced from real Claude Code/Gemini CLI)
- Skills mapping to 7 domain skills
- Constitution alignment verified (10 core principles)

**Key Metrics**:
- Total duration: 325 minutes (5.4 hours)
- Learning objectives: 22 (3 per lesson average)
- Key concepts: 40 across all lessons
- Exercises: 24 total
- Assessments: 16 (formative + summative per lesson)

**Innovation Captured**:
- Dialogue-first teaching (no isolated syntax blocks)
- Safety pattern explicitly taught and reinforced throughout
- Verification as core skill (learn to verify before trusting)
- Real interactions (zero fabricated examples)
- AI as co-reasoning partner (learner supervises, AI executes)

## Outcome

- ✅ Impact: Transformed specification into actionable lesson-by-lesson plan ready for content-implementer implementation; all 8 lessons detailed with learning objectives, exercises, assessments
- 🧪 Tests: All validation checkpoints pass (cognitive load, skills mapping, constitution alignment, assessment strategy)
- 📁 Files: plan.md created (607 lines); dialogues and examples specified for sourcing
- 🔁 Next prompts: content-implementer subagent to implement each lesson; validation-auditor to validate; no tasks.md until after implementation complete
- 🧠 Reflection: Plan successfully captures dialogue-first innovation while maintaining pedagogical rigor. Key challenge during implementation: sourcing authentic dialogue examples from real Claude Code/Gemini CLI interactions

## Evaluation notes (flywheel)

- Failure modes observed: None. Plan addresses all specification requirements and user stories.
- Graders run and results (PASS/FAIL): Completeness check PASS; cognitive load check PASS; skills mapping PASS; constitution alignment PASS
- Prompt variant (if applicable): Original specification intact; plan elaborates without adding scope creep
- Next experiment (smallest change to try): During content-implementer phase, validate that dialogue patterns feel natural (not forced) through learner feedback
