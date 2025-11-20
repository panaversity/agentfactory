---
id: 0160
title: quiz-generation-python-chapters
stage: general
date: 2025-11-19
surface: claude-code-cli
model: claude-sonnet-4-5
feature: none
branch: p4-quiz
user: mjs
command: none
labels: [quiz, assessment, python, part-4, reasoning-activation, sdd-ri]
links:
  spec: none
  ticket: https://github.com/panaversity/tutorsgpt/issues/268
  adr: none
  pr: none
files:
  - path: prompts/quiz-generation-python-chapters.md
    status: created
    purpose: Reasoning-activated prompt for generating Python chapter quizzes
tests: []
---

## Prompt

**User Request**: "See this issue No Quiz at the end of the Python chapters #268. There are no quizzes at the end of the chapter. Example no quiz in this chapter: https://ai-native.panaversity.org/docs/Python-Fundamentals/control-flow-loops. Write me a prompt using our papers @papers/ so we have proper quiz at each chapter end like in P1. This prompt will be for each paper."

**Context**:
- Part 1 chapters have comprehensive quizzes (e.g., `01-ai-development-revolution/09_chapter_01_quiz.md`)
- Part 4 Python chapters (13-30) have NO quizzes
- Need quizzes that align with AI-native learning philosophy
- Must use reasoning activation patterns from our papers (SDD-RI, Reasoning Activation, Skills Thinking Framework)

**Task**: Create a reusable, reasoning-activated prompt that will generate high-quality quizzes for Python chapters following our pedagogical frameworks.

## Response snapshot

Created comprehensive prompt: `prompts/quiz-generation-python-chapters.md` (v2.0)

**Key Components**:

1. **Skill Integration** (NEW in v2.0)
   - Integrates with `quiz-generator` skill (v5.0.0)
   - 50 questions total (not 10-15)
   - Interactive Quiz component (not static markdown)
   - Automated answer redistribution
   - Option length validation (±3 words)

2. **Reasoning Activation Framework**
   - Persona: Assessment architect (not generic question generator)
   - Questions: What layer? What proficiency tier? What should I test?
   - Principles: 5 core decision frameworks

3. **Core Principles**
   - Test understanding, not memorization
   - Align with AI-native learning philosophy
   - Progressive complexity (Bloom's taxonomy)
   - Production-relevant context (no toy examples)
   - Explanations that teach

4. **Question Distribution (50 questions)**
   - By Type: Scenario (20), Code review (15), Concept (10), Workflow (5)
   - By Bloom: Remember (10), Understand (12), Apply (15), Analyze (8), Evaluate (5)
   - By Difficulty: Easy (10), Medium (30), Hard (10)

5. **Chapter-Specific Guidance**
   - Tailored approaches for each chapter range (13-30)
   - Maps to 4-Layer Teaching Method
   - Aligns with part progression

6. **Complete Examples**
   - 4 detailed example questions (one per type)
   - Anti-patterns to avoid
   - Validation checklist (updated for 50 questions)

7. **Integration with Our Frameworks**
   - SDD-RI: Specifications and reusable intelligence
   - Reasoning Activation: Persona + Questions + Principles
   - Skills Thinking: Avoiding distributional convergence
   - Constitution: Three Roles framework, pedagogical principles
   - quiz-generator skill: 50-question format, immediate feedback, randomized batching

## Outcome

- ✅ Impact: Created reusable prompt template that integrates quiz-generator skill for all 18 Python chapters (13-30)
- 🧪 Tests: None (this is a prompt template, not code)
- 📁 Files: 1 updated (prompts/quiz-generation-python-chapters.md v1.0 → v2.0)
- 🔁 Next prompts: Invoke quiz-generator skill with this prompt for each Python chapter (13-30)
- 🧠 Reflection: v2.0 adds critical skill integration - ensures 50-question format, Quiz component usage, answer redistribution, and option length validation. Prompt activates reasoning mode AND leverages existing infrastructure.

## Evaluation notes (flywheel)

- Failure modes observed: v1.0 didn't specify quiz-generator skill integration, would have generated wrong format (10-15 static questions instead of 50 interactive)
- Graders run and results (PASS/FAIL): Not applicable (prompt template creation)
- Prompt variant (if applicable): v2.0 - integrates quiz-generator skill (v5.0.0)
- Next experiment: Test on Chapter 18 (Control Flow and Loops) to validate:
  1. Skill invocation works correctly
  2. 50 questions cover all concepts appropriately
  3. Option length validation passes
  4. Answer redistribution completes successfully
  5. Questions align with AI-native learning philosophy
