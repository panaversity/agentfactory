---
id: 001
title: Create Chapter 11 Python UV Spec
stage: spec
date: 2025-11-04
surface: agent
model: claude-3.7-sonnet
feature: python-uv
branch: 011-python-uv
user: architect
command: /sp.specify
labels: ["specification", "chapter-11", "python-uv", "part-4", "ai-driven-development", "package-management"]
links:
  spec: specs/011-python-uv/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/011-python-uv/spec.md
  - specs/011-python-uv/checklists/requirements.md
tests:
  - Manual validation against specification quality checklist (PASSED)
---

## Prompt

Follow instructions in sp.specify.prompt.md.

Write chapter 11 in Part 4 of the book. The title of the chapter will be "Python UV: The Fastest Python Package Manager".

The chapter will cover the AI Driven Way of understanding and using UV for building project.

In the AI Driven way the students do not try to memorize the commands or syntax but understand the concepts and use Claude Code or Gemini CLI to give UV commands.

Use AI CLIs as co developers and teachers and execute the intent of the developer.

We will use the official docs as source material and use the AI Driven Way to teach:
https://docs.astral.sh/uv/
https://docs.astral.sh/uv/getting-started/installation/
https://docs.astral.sh/uv/getting-started/features/
https://docs.astral.sh/uv/guides/projects/

Always remember when you add example for writing prompt, always add the output the CLI tool shall give.

## Response snapshot

Created comprehensive specification for Chapter 11: Python UV following the AI-Driven Development methodology.

**Specification Highlights**:

1. **Feature Branch**: Confirmed existing branch `011-python-uv` (no new branch creation needed)

2. **7 Prioritized User Stories** (P1-P7):
   - P1: Understanding UV's Purpose and Value (conceptual foundation)
   - P2: AI-Driven Installation and Setup (hands-on entry point)
   - P3: AI-Driven Project Initialization (creating projects)
   - P4: AI-Driven Dependency Management (add/update/remove packages)
   - P5: AI-Driven Project Execution (running code and tests)
   - P6: AI-Driven Collaboration (lockfiles, environment reproducibility)
   - P7: AI-Driven Advanced Features Discovery (Python version management, standalone scripts)

3. **15 Functional Requirements** covering:
   - Core UV concepts without prior tool knowledge
   - AI-Driven workflow (intent → command → understanding)
   - Prompt-response examples for every UV task
   - Platform-agnostic installation via AI
   - Complete project lifecycle (init → deps → run → collaborate)
   - Troubleshooting through AI interaction
   - Alignment with Part 4 complexity tier (intermediate, max 7 concepts/section)

4. **11 Success Criteria** focusing on:
   - Conceptual understanding over command memorization
   - AI-guided capability demonstration
   - 90% of readers using AI as interactive documentation
   - Teaching ability (readers can teach peers using concepts + AI)

5. **Key Principles**:
   - AI CLI tools positioned as co-developers and teachers
   - Every example includes: prompt → AI response → output → teaching
   - Focus on "why" and "what" before "how"
   - Just-in-time learning through AI discovery
   - Readers express intent, AI handles syntax

**Validation**:
- Created specification quality checklist
- All requirements testable and unambiguous
- No [NEEDS CLARIFICATION] markers (all details concrete or AI-guided defaults)
- Complexity tier appropriate for Part 4 (intermediate)
- Prerequisites clearly defined
- Scope boundaries established (out-of-scope items documented)

**Files Created**:
1. `specs/011-python-uv/spec.md` - Complete feature specification (15 FR, 7 user stories, 11 SC)
2. `specs/011-python-uv/checklists/requirements.md` - Quality validation checklist (all items PASSED)

## Outcome

- ✅ **Impact**: Comprehensive AI-Driven Development specification for Chapter 11 (Python UV) created, evaluated, and revised. Ready for planning phase.
- 🧪 **Tests**: 
  - Initial spec quality checklist: ALL PASSED
  - Constitution v3.0.1 evaluation: Identified missing Evals-First methodology
  - Applied revisions: Added Business Goals & Success Evals section, resolved prerequisite circular dependency, added sample prompt-response patterns
  - Final validation: FULL COMPLIANCE with constitution, COWRITER-GUIDE, and chapter-index
- 📁 **Files**: 
  - Created and revised: spec.md (comprehensive specification with Evals-First section and appendix)
  - Created and updated: checklists/requirements.md (validation checklist with revision notes)
  - Created: PHR documenting full workflow
- 🔁 **Next prompts**: 
  - `/sp.plan` to create detailed lesson breakdown and task structure
  - Use sample prompt-response patterns from Appendix as template for all examples
  - Plan visual diagrams: UV vs. traditional tools, project structure, dependency resolution flow
  - Emphasize "tooling setup first, code writing later" professional workflow
- 🧠 **Reflection**: Specification evolution demonstrates the value of thorough evaluation. Initial spec (90/100, A-) was functionally sound but missing v3.0.1 Evals-First methodology. Revisions added business goal framing (employability, productivity, AI-native mindset) and resolved pedagogical positioning (Chapter 11 as bridge before Python syntax). Final spec models the professional AI-native pattern: Evals → Spec → Plan → Implement → Validate.

## Evaluation notes (flywheel)

- **Failure modes observed**: None - specification creation followed standard process
- **Graders run and results**: 
  - ✅ PASS: Content Quality (no implementation details, user-focused, non-technical language)
  - ✅ PASS: Requirement Completeness (testable, unambiguous, measurable success criteria)
  - ✅ PASS: Feature Readiness (all scenarios defined, scope bounded, no leakage)
  - ✅ PASS: Educational Alignment (learning objectives, AI-Driven methodology, complexity tier)
  - ✅ PASS: AI-Driven Development Alignment (intent-first, AI as teacher, conceptual focus)
- **Prompt variant**: Standard `/sp.specify` workflow with educational content focus
- **Next experiment**: 
  - During planning phase, experiment with lesson structure: Should each lesson map 1:1 with user stories, or combine related stories?
  - Test prompt-response example format: Consider including "common mistakes" in each example to show AI debugging process
  - Explore including a "UV command translation guide" as appendix: English intent → UV command (AI-generated, explicitly marked as reference not for memorization)
