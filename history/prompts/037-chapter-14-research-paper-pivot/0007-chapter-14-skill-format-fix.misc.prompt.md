---
id: "0007"
title: "Chapter 14 Skill Format Fix and Consistency Review"
stage: misc
date: 2025-11-27
surface: claude-code
model: opus
feature: 037-chapter-14-research-paper-pivot
branch: main
user: mjs
command: manual review
labels: [skill-format, cross-chapter-consistency, action-oriented-lessons]
links:
  spec: specs/037-chapter-14-research-paper-pivot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/09-designing-reusable-intelligence.md
    action: edit
  - path: apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/06-plan-phase.md
    action: edit
  - path: apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/04-specify-phase.md
    action: edit
  - path: specs/book/chapter-index.md
    action: edit
tests:
  - null
---

## Prompt

User identified multiple issues during Chapter 14 review:

1. Lesson 4 (Specify) was documentation-heavy, not action-oriented
2. Lesson 6 (Plan Phase) missing `/sp.plan` and `/sp.adr` commands
3. Lesson 9 (Designing RI) used incorrect skill file format
4. Chapter index showed "Needs Pivot" instead of "Implemented"

User provided reference materials:

- Old calculator examples showing action-oriented approach
- Anthropic's guide on conversation-based skill creation
- Chapter 5 Lesson 7 showing correct skill format: `.claude/skills/<skill-name>/SKILL.md`

## Response snapshot

**Key Fixes Applied:**

1. **Lesson 4 (Specify)**: Rewrote with pre-specification conversation, actual `/sp.specify` prompts, SMART criteria testing

2. **Lesson 6 (Plan)**: Added `/sp.plan` workflow, `/sp.adr` for architectural decisions, ADR significance test (long-term + alternatives + future questioning)

3. **Lesson 9 (Designing RI)**:

   - Fixed skill format from `.claude/skills/section-writer.md` to `.claude/skills/section-writer/SKILL.md`
   - Added proper YAML frontmatter (`name`, `description`, `version`)
   - Added conversation-based skill creation workflow
   - Added skill vs subagent decision framework (2-6 decisions = skill, 7+ = subagent)

4. **Chapter Index**: Updated Chapter 14 status to "Implemented"

## Outcome

- Impact: Fixed cross-chapter consistency issue with skill format; improved lesson action-orientation
- Tests: Manual review passed
- Files: 4 files modified across 3 commits
- Next prompts: Review lessons 10, 11, 12 for consistency
- Reflection: Always reference canonical examples (Chapter 5) when teaching patterns used elsewhere

## Evaluation notes (flywheel)

- Failure modes observed:

  1. **Format drift**: Teaching a pattern (skill files) without checking canonical source (Chapter 5)
  2. **Documentation-first writing**: Defaulting to explanation over action
  3. **Index staleness**: Forgetting to update chapter-index.md after completing work

- Graders run and results (PASS/FAIL): Manual review PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try):
  - Add pre-commit check: grep for `.claude/skills/.*\.md` (flat file pattern) to catch format errors
  - Reference checklist: "Before teaching a pattern, find and read its canonical source in the book"
