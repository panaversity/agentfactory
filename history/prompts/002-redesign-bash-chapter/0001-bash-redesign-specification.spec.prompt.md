---
id: 0001
title: Bash Redesign Specification
stage: spec
date: 2025-11-02
surface: agent
model: claude-haiku-4-5-20251001
feature: 002-redesign-bash-chapter
branch: 002-redesign-bash-chapter
user: mjs
command: /sp.specify
labels: ["bash-education", "dialogue-first", "chapter-redesign", "proofreader-feedback"]
links:
  spec: specs/002-redesign-bash-chapter/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-redesign-bash-chapter/spec.md
  - specs/002-redesign-bash-chapter/checklists/requirements.md
tests:
  - Quality checklist: PASS (all items pass)
  - Specification completeness: PASS (no NEEDS CLARIFICATION markers)
  - Acceptance scenarios: PASS (21 scenarios defined across 7 user stories)
---

## Prompt

Redesign Bash Chapter - the proofreaders feedback suggest that chapter 7 is causing confusion. You can look at current chapter and the spec it was created with. Now the core idea we will use to redesign and teach using each command using an approach like this:

```
You: "Show me my current directory path"
Agent: Tool -> Shell $ pwd
Tool Output: Users/mjs/Documents
Agent: "You are in Users/mjs/Documents directory."
```

## Response snapshot

✅ **Specification Created: Redesign Bash Chapter with Interactive Dialogue Pattern**

**Branch**: `002-redesign-bash-chapter`
**Status**: Ready for Planning Phase

**Key Elements**:
- 7 prioritized user stories (P1: Navigation, File Ops, Configuration, Safety | P2: Packages, Processes, Pipes)
- 21 acceptance scenarios (3-4 per story) following Given/When/Then format
- 10 functional requirements (FR-001 to FR-010) focusing on dialogue-first learning
- 6 content requirements (CR-001 to CR-006) ensuring authentic, real-world examples
- 6 quality metrics validating pedagogy and authenticity
- 8 lesson structure with 4-5 hour total duration
- Clear "before/after" comparison showing difference from current syntax-first approach

**Core Innovation**: Shifts from teaching bash commands in isolation to teaching how to have productive conversations with AI about bash. Aligns with AI-native development paradigm.

**Complexity Tier**: Tier 1 Beginner (Part 2, Chapter 7) with dialogue-first, safety-first approach

**Quality Checklist**: All items PASS (requirements complete, no ambiguities, success criteria measurable)

## Outcome

- ✅ Impact: Redesigned specification addresses proofreader feedback by introducing dialogue-first pedagogy, shifting focus from syntax memorization to understanding AI collaboration patterns
- 🧪 Tests: Quality checklist validates specification completeness (all 4 categories PASS)
- 📁 Files: Created spec.md (404 lines) and requirements.md (checklist); branch created with proper feature number (002)
- 🔁 Next prompts: `/sp.plan` to break spec into detailed lesson plans; `/sp.clarify` if clarifications needed during planning
- 🧠 Reflection: Specification successfully reframes bash education as AI collaboration rather than traditional CLI training. Requires real dialogue examples from Claude Code/Gemini CLI during implementation phase.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification is complete and unambiguous.
- Graders run and results (PASS/FAIL): Quality checklist PASS (all requirements met)
- Prompt variant (if applicable): Original user input integrated directly into vision statement and requirements
- Next experiment (smallest change to try): During planning phase, validate that 8-lesson structure can accommodate all 7 user stories without overwhelming learners
