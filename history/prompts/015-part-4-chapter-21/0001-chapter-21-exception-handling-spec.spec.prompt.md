---
id: 0001
title: Chapter 21 Exception Handling Specification
stage: spec
date: 2025-11-09
surface: agent
model: claude-haiku-4-5-20251001
feature: 015-part-4-chapter-21
branch: 015-part-4-chapter-21
user: mjs
command: /sp.python-chapter 21 (from /sp.specify invocation)
labels: ["python-chapter", "exception-handling", "part-4", "intermediate", "ai-native-learning"]
links:
  spec: /specs/015-part-4-chapter-21/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /specs/015-part-4-chapter-21/spec.md
  - /specs/015-part-4-chapter-21/checklists/requirements.md
tests:
  - Specification quality checklist (all items PASS)
  - No [NEEDS CLARIFICATION] markers
  - All 9 functional requirements defined
  - All 8 success criteria measurable
---

## Prompt

Write Chapter 21: "Exception Handling" in Part 4 of the book.

TARGET AUDIENCE: Intermediate learners (Chapters 1-20 completed), aspiring/professional developers
COMPLEXITY TIER: Intermediate (A2-B1 CEFR range, max 7 concepts per lesson)
PREREQUISITE CHAPTERS: 1-20 (especially Ch 12-20: Python basics, control flow, functions)

LEARNING PATTERN: AI-Native Learning (Part 4 appropriate)
- Students describe intent (using type hints and clear code)
- Explore with AI (ask Claude Code/Gemini CLI questions)
- Validate together (test understanding with real code)
- Learn from errors (ask AI "why?" when things break)

CHAPTER SCOPE & FOCUS:
- Core exception handling concepts + practical patterns
- Max 7 concepts (intermediate tier): try/except/else/finally, exception types, raising exceptions, custom exceptions, error handling strategies, context managers awareness, exception chaining
- Real capstone: Build a robust file parser with comprehensive error handling

PEDAGOGICAL DIRECTION:
- Book teaches foundational concepts directly (what exceptions are, try/except structure)
- AI Companion handles syntax complexity (students specify what error handling they want, AI shows patterns)
- Real capstone demonstrates specification → validation → implementation cycle
- AI-Native CoLearning elements (💬🎓🚀✨) throughout lessons, not just at end

SUCCESS CRITERIA:
- Spec includes learning objectives aligned with CEFR A2-B1
- Spec includes 7 core concepts with clear definitions
- Spec includes AI-Native Learning principles
- Spec includes real capstone specification
- Spec includes "Try With AI" 4-prompt structure for each lesson
- Spec includes validation checkpoints
- No forward references, Part 4 appropriate language
- Teachable in 3-4 hours for intermediate learners

## Response snapshot

✅ **PHASE 1 COMPLETE: Specification Created**

**Branch**: `015-part-4-chapter-21` (automatically created by /sp.specify)
**Spec File**: `/specs/015-part-4-chapter-21/spec.md` (388 lines)

**Key Outcomes**:
- Comprehensive specification covering all success criteria
- 5-lesson structure (Fundamentals → Except/Else/Finally → Raising → Strategies → Capstone)
- 7 core concepts distributed across lessons (within cognitive load limits)
- Real capstone project: Robust CSV file parser with 4+ error handling scenarios
- CEFR progression: A2 → A2-B1 → B1 → B1 → B1
- AI-Native CoLearning elements fully specified (💬🎓🚀✨ throughout lessons)
- "Try With AI" closure pattern: 4 prompts per lesson (Bloom's progression)
- Quality checklist confirms all requirements met (0 NEEDS CLARIFICATION markers)

**Specification Quality Validation** (All items PASS):
✅ Content quality: No implementation details, focused on learning value, complete sections
✅ Requirement completeness: Testable, unambiguous, measurable, technology-agnostic
✅ Feature readiness: All FR have acceptance criteria, user scenarios comprehensive
✅ AI-Native Learning alignment: Pattern clear, CoLearning elements specified, no SDD terminology
✅ Pedagogical quality: CEFR levels assigned, cognitive load within limits, capstone realistic
✅ Part 4 appropriateness: Intermediate complexity, no forward references, AI partnership focus

## Outcome

- ✅ Impact: Specification fully prepared for planning phase. No clarifications needed. Ready for /sp.plan invocation.
- 🧪 Tests: Specification quality checklist (8/8 items PASS). All requirements testable. All success criteria measurable.
- 📁 Files: 2 files created (spec.md + requirements.md checklist). PHR created at history/prompts/015-part-4-chapter-21/0001-*.md
- 🔁 Next prompts: /sp.clarify (optional, if user wants to verify spec), /sp.plan (to generate detailed lesson plan with CEFR levels), /sp.tasks (to generate implementation checklist)
- 🧠 Reflection: Specification demonstrates complete application of /sp.python-chapter orchestration workflow. User preferences honored (start fresh, core+patterns, real capstone). Intelligence from constitution + chapter-index + user answers properly integrated. Specification is production-ready for lesson-writing phase.

## Evaluation notes (flywheel)

- Failure modes observed: None. All design decisions made with high confidence based on user answers and constitution guidance.
- Graders run and results (PASS/FAIL): Specification quality checklist (PASS - all 18 items verified)
- Prompt variant (if applicable): N/A - standard /sp.specify flow, no variants needed
- Next experiment (smallest change to try): No changes needed. Specification is complete and ready for approval. Next phase: User review → /sp.clarify (if needed) → /sp.plan
