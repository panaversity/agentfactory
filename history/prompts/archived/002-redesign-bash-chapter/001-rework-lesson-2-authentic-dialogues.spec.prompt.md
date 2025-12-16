---
id: 001
title: Rework Lesson 2 with Authentic Gemini Dialogues
stage: spec
date: 2025-11-03
surface: agent
model: claude-haiku-4-5-20251001
feature: 002-redesign-bash-chapter
branch: main
user: mjs
command: rework lesson 2 safety-first-pattern with real Gemini examples
labels:
  [
    "dialogue-authenticity",
    "bash-chapter",
    "spec-compliance",
    "real-examples",
    "gemini-testing",
  ]
links:
  spec: /specs/002-redesign-bash-chapter/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/02-safety-first-pattern.md
tests:
  - Real Gemini CLI dialogue: delete old test files (clarifying questions)
  - Real Gemini CLI dialogue: backup project before major changes
  - Real Gemini CLI dialogue: organize Python files (agent limitations)
---

## Prompt

Rework on this lesson @apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/02-safety-first-pattern.md. It have some examples I think AI in real won't do like this - for testing in shell you can run gemini and test with it. For reference the spec for this book was @specs/002-redesign-bash-chapter/

## Response snapshot

Identified critical issues with unrealistic examples:

1. **Agent "pauses" artificially** - Real AI (Gemini, Claude Code) doesn't have natural pause points; it executes unless told otherwise
2. **Example 2-1 perfect backup explanation** - Real agents are verbose, ask clarifying questions, suggest alternatives
3. **Perfectly linear dialogue flow** - Real conversations meander; learners change their mind mid-conversation
4. **Agent self-awaits verification** - Real behavior: agents try to execute, then show results
5. **Polarized good/bad questions** - Real questions are valid; it's about depth and safety focus

Created and executed real Gemini CLI tests:

- ✅ Test 1: "I want to delete old test files. Before you do anything, explain your approach step by step."
- ✅ Test 2: "I want to back up my project before making major changes. What's the best way?"
- ✅ Test 3: "I want to organize my Python files into a folder called my-scripts. Walk me through what you'd do, step by step."

Updated lesson with three authentic sections:

1. **Real Dialogue section (lines 109-165)**: Replaced hypothetical with real Gemini exchange showing agent asking clarifying questions, learner changing request, agent discovering limitations transparently
2. **Exercise 1 (lines 219-272)**: Real Gemini interaction where Step 4 is missing and agent hits practical limitation
3. **Try With AI section (lines 401-491)**: 3 real Gemini examples showing authentic AI behavior with actual outputs and interpretation guidance

## Outcome

- ✅ Impact: Lesson now aligns with CR-001 (authentic sourced dialogue), CR-002 (commands in natural context), QM-001/QM-002 (no fabricated examples)
- 🧪 Tests: All 3 Gemini dialogues executed and captured; output verified as authentic
- 📁 Files: 1 lesson file updated with authentic examples
- 🔁 Next prompts:
  - Apply same rework pattern to remaining 7 lessons (Lessons 1, 3-8)
  - Validate all examples meet CR-001-004 and QM-001-006 requirements
  - Technical review of pedagogical soundness
- 🧠 Reflection: Real AI collaboration is messier than textbooks—agents ask questions, hit limitations, require supervision. This is the core safety lesson: you stay safe by asking questions BEFORE execution, which naturally happens in realistic dialogue.

## Evaluation notes (flywheel)

- Failure modes observed: Original lesson taught textbook 5-step pattern as perfectly linear; real dialogue is iterative. Original Exercise 1 used hypothetical scenario instead of real agent behavior.
- Graders run and results (PASS/FAIL): PASS - All dialogue examples sourced from real Gemini CLI with documented source
- Prompt variant (if applicable): null
- Next experiment: Test whether learners can distinguish between hypothetical and real AI dialogue patterns; assess if authentic examples improve safety habit formation

## Key learnings

1. **Real AI behavior is transparent about limitations** - Gemini acknowledged "I don't have the ability to directly create directories or move files" mid-conversation
2. **Agents ask clarifying questions proactively** - "I will then need you to define what constitutes an 'old' file"
3. **Execution is request-based, not automatic** - Gemini showed commands but asked learner to run them
4. **Dialogue teaches safety through practice** - Seeing Step 4 missing in realistic context makes absence concrete and consequential

## Spec compliance checklist

- ✅ CR-001: Dialogue MUST be authentic—sourced from real interactions
- ✅ CR-002: Every bash command in natural conversation, not isolated
- ✅ CR-003: Safety concerns surfaced through agent questions
- ✅ CR-004: Verification demonstrated through dialogue
- ✅ CR-005: Error handling through actual error messages (e.g., agent tool execution failure)
- ✅ CR-006: Verification habit explicitly taught (Step 4 consequences shown)
- ✅ QM-001: All dialogue examples sourced from real Claude Code / Gemini CLI interactions
- ✅ QM-002: Zero dialogue examples are fabricated
- ✅ QM-003: Every bash command appears in complete dialogue example
- ✅ QM-004: Safety concerns surfaced naturally in dialogue (agent asks questions), not in separate warning blocks
- ✅ Learning Objective 1: Learner identifies missing Step 4 in Exercise 1 (A1 level)
- ✅ Learning Objective 2: Learner applies pattern by comparing textbook vs. real (A2 level)
- ✅ Learning Objective 3: Exercise 2 still prompts question-writing (A2 level)
