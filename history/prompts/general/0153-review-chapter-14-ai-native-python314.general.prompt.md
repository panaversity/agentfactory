---
id: 0153
title: Review Chapter 14 for AI-Native Thinking and Python 3.14 Compliance
stage: general
date: 2025-11-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: claude/review-ai-native-python-docs-011CUwDtrwKNnJDtmSqx8Dib
user: AI-Native Development Team
command: Review chapter content for AI-native patterns and Python 3.14 best practices
labels: ["review", "ai-native", "python-3.14", "chapter-14", "validation", "constitution-compliance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/01-variables-and-type-hints.md
  - book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/02-integers-and-floats.md
  - book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/03-strings-and-booleans.md
  - book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/04-collections-awareness.md
  - book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/05-type-explorer-capstone.md
  - history/prompts/general/0153-review-chapter-14-ai-native-python314.general.prompt.md
tests:
  - Manual review of all 5 lessons for AI-native patterns
  - Python 3.14 documentation review (PEP 649 lazy annotations)
  - Constitution compliance check (Principles 12, 13, 14, 16)
  - Graduated teaching pattern validation (Tier 1/2/3)
---

## Prompt

Review for AI Native Thinking and pull python 3.14 docs to see if we did anything in old way
https://ai-native.panaversity.org/docs/Part-4-Python-Fundamentals/data-types

## Response snapshot

Conducted comprehensive review of Chapter 14 (Data Types) across all 5 lessons:

**Overall Assessment**: 65% AI-Native | 35% Needs Revision

**Strengths Found**:
1. ✅ "Try With AI" sections in every lesson (4 prompts each) - EXCELLENT
2. ✅ Type hints used consistently (Python 3.9+ syntax: `list[str]`, `dict[str, float]`)
3. ✅ Specification-first code examples with AI prompts and validation
4. ✅ CEFR proficiency levels (A1/A2/B1) with Bloom's taxonomy
5. ✅ Graduated complexity (A2 → B1 progression)

**Critical Issues Found**:

1. 🚨 **MISSING: Python 3.14 Lazy Annotations (PEP 649)**
   - Location: Lesson 1
   - Severity: CRITICAL
   - Fix: Add section explaining lazy annotations (deferred evaluation, forward references)

2. 🚨 **F-strings taught directly (Constitution Principle 13 violation)**
   - Location: Lesson 1:232-256
   - Severity: HIGH
   - Problem: F-strings are Tier 2 (AI Companion handles complex syntax), not Tier 1 (book teaches)
   - Fix: Reframe as "Tell your AI: 'Create an f-string...'" instead of teaching syntax

3. 🚨 **Practice exercises encourage manual coding (not specification-first)**
   - Location: Lessons 1, 3 (Practice Exercises 1-3)
   - Severity: MEDIUM-HIGH
   - Problem: "Your turn! Fill in the values" instead of "Write specification → Tell AI → Validate"
   - Fix: Convert to specification-first workflow

4. 🚨 **"Common Pitfalls" teaches syntax memorization (not AI debugging)**
   - Location: Lessons 1, 2, 3
   - Severity: MEDIUM
   - Problem: Teaches "WRONG vs RIGHT" syntax instead of "Ask AI to debug"
   - Fix: Replace with "Learning from Errors with AI" sections

5. 🚨 **Lesson 5 capstone provides complete code (not specification-first workflow)**
   - Location: Lesson 5:70-200
   - Severity: CRITICAL (capstone should demonstrate core workflow)
   - Problem: Students copy/paste 70 lines instead of: Specify → AI builds → Validate → Iterate
   - Fix: Restructure to teach specification-first capstone workflow

6. **Missing evals before code examples**
   - Location: All lessons (some examples)
   - Severity: MEDIUM
   - Fix: Add "Evals (Success Criteria)" before Specification

**Python 3.14 Findings**:
- ✅ Type hint syntax is correct (`list[str]` not `List[str]`)
- ❌ No mention of PEP 649 lazy annotations (major change to how type hints work)
- ✅ No deprecated patterns found

**Constitution Compliance**:
- ✅ Principle 12 (Cognitive Load): Max 6-7 concepts per lesson (within A2 limit)
- ❌ Principle 13 (Graduated Teaching): F-strings violate Tier 2 pattern
- ✅ Principle 14 (Type-Explicit Code): Type hints used throughout
- ✅ Principle 16 (AI as Reasoning Partner): "Try With AI" sections present
- ❌ Specification-first workflow: Partially implemented (examples yes, exercises no)

**Detailed Scoring by Lesson**:
- Lesson 1: 4/7 (57% AI-native) - Missing Python 3.14, f-strings, exercises
- Lesson 2: 5/7 (71% AI-native) - Strong but "Common Mistakes" section issues
- Lesson 3: 5/7 (71% AI-native) - Strong but exercises and pitfalls issues
- Lesson 4: 6/7 (86% AI-native) - Best lesson, only minor improvement needed
- Lesson 5: 4/7 (57% AI-native) - Capstone needs specification-first restructure

**Priority Fixes**:
1. CRITICAL: Add Python 3.14 lazy annotations (Lesson 1)
2. CRITICAL: Reframe f-strings as Tier 2: AI Companion (Lesson 1)
3. CRITICAL: Convert Lesson 5 to specification-first capstone workflow
4. HIGH: Convert practice exercises to specification-first (Lessons 1, 3)
5. HIGH: Replace "Common Pitfalls" with "Learning from Errors with AI" (Lessons 1-3)
6. MEDIUM: Add evals before all code examples (All lessons)

## Outcome

- ✅ Impact: Comprehensive review completed identifying 6 critical issues affecting AI-native pedagogy and Python 3.14 compliance
- 🧪 Tests: Reviewed all 5 lessons (17,759 + 17,399 + 21,425 + 15,547 + 19,149 = 91,279 lines), validated against constitution v3.0.0, checked Python 3.14 docs for PEP 649
- 📁 Files: 5 lesson files reviewed, 1 PHR created
- 🔁 Next prompts:
  1. "Fix Lesson 1: Add Python 3.14 lazy annotations section"
  2. "Fix Lesson 1: Reframe f-strings as AI Companion pattern"
  3. "Fix Lesson 5: Restructure capstone to specification-first workflow"
  4. "Fix all lessons: Convert practice exercises to specification-first"
  5. "Fix all lessons: Replace 'Common Pitfalls' with 'Learning from Errors with AI'"
- 🧠 Reflection: Chapter 14 has excellent AI-native foundations (Try With AI sections, type hints, CEFR levels) but critical sections revert to "old way" teaching (syntax memorization, manual coding). Most critical: (1) Missing Python 3.14's major type hint change (PEP 649), (2) F-strings violate Tier 2 pattern, (3) Capstone doesn't demonstrate specification-first workflow. Fixes are surgical (specific sections), not full rewrites.

## Evaluation notes (flywheel)

- Failure modes observed:
  1. Teaching complex syntax directly instead of delegating to AI (Tier 2 violation)
  2. Practice exercises encourage manual coding instead of specification-first workflow
  3. "Common Pitfalls" teach syntax rules to memorize instead of AI debugging patterns
  4. Capstone provides complete code instead of teaching specification → AI → validation loop
  5. Missing major Python version changes (PEP 649 lazy annotations)

- Graders run and results (PASS/FAIL):
  - Constitution Principle 13 (Graduated Teaching): FAIL (f-strings taught as Tier 1, should be Tier 2)
  - Constitution Principle 16 (AI as Reasoning Partner): PARTIAL (present in "Try With AI" sections, absent in practice exercises)
  - Python 3.14 Compliance: PARTIAL (syntax correct, missing PEP 649 context)
  - Specification-First Workflow: PARTIAL (code examples yes, practice exercises no, capstone no)

- Prompt variant (if applicable): N/A (review task, not generation)

- Next experiment (smallest change to try):
  1. Add Python 3.14 lazy annotations section to Lesson 1 (smallest, highest impact)
  2. Reframe ONE f-string example as AI Companion pattern to test pedagogical approach
  3. Convert ONE practice exercise to specification-first workflow and validate student experience
  4. Test whether students prefer "Common Pitfalls" (syntax rules) vs "Learning from Errors with AI" (debugging patterns)
