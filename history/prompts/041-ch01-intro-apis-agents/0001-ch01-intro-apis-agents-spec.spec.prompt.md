---
id: 0001
title: ch01-intro-apis-agents-spec
stage: spec
date: 2025-12-05
surface: claude-code-cli
model: claude-sonnet-4-5
feature: 041-ch01-intro-apis-agents
branch: 041-ch01-intro-apis-agents
user: HP
command: /sp.specify ai-native-ch01-intro-apis-agents
labels: [specification, educational-content, chapter-1, course-design]
links:
  spec: specs/041-ch01-intro-apis-agents/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/041-ch01-intro-apis-agents/spec.md
    action: created
    lines: 450
  - path: specs/041-ch01-intro-apis-agents/checklists/requirements.md
    action: created
    lines: 85
tests: []
---

## Prompt

**User Input**: "Create spec for 1st chapter for now then we will do step by step"

**Context from Previous Conversation**:
- User requested creation of AI-Native Software Development course covering OpenAI Agents SDK, ChatKit, and RAG chatbot development
- Course structure defined in `context/AI-Native-Course-Spec-v2 (2).md` with 16 chapters and ~60 lessons
- Project: DocuBot (production RAG chatbot built incrementally throughout course)
- Pedagogical approach: Project-based learning with progressive depth and concept→apply separation
- Orchestrator completed Phase 0 (routing confirmation) - classified as CONTENT work for Students stakeholder
- User confirmed to follow exact spec structure for chapters 1-16
- User requested step-by-step approach starting with Chapter 1 specification only

**Chapter 1 Overview from Source Spec**:
- Title: "Introduction to AI APIs & OpenAI Agents"
- Duration: 2-3 hours
- 4 Lessons:
  - Lesson 1.1: What Are APIs? (30-40 min) - API fundamentals, restaurant analogy, test API key
  - Lesson 1.2: Agents vs LLMs (30-40 min) - Conceptual distinction, comparison table exercise
  - Lesson 1.3: Environment Setup (40-50 min) - Install UV, create project, configure .env
  - Lesson 1.4: Course Project Architecture (25-35 min) - DocuBot blueprint, architecture.md document
- Cognitive Load: LOW (pure concepts, no code complexity)
- Learning Objectives: Explain APIs, distinguish LLM vs Agent, setup environment, visualize architecture
- DocuBot State After: Project folder created, environment configured, architecture documented (no code yet)

**Command Invoked**: `/sp.specify ai-native-ch01-intro-apis-agents`

## Response snapshot

**Specification Created**: `specs/041-ch01-intro-apis-agents/spec.md` (450 lines)

**Structure**:
- User Scenarios & Testing: 4 prioritized user stories (P1: API verification, P1: Environment setup, P2: LLM vs Agent concepts, P3: Architecture visualization)
- Requirements: 26 functional requirements (FR-001 through FR-026) covering all 4 lessons
- Success Criteria: 10 measurable outcomes (95% success rates, 30-minute time limits, 80% completion in 2-3 hours)
- Constraints: 21 constraints across 4 categories (Technical: 5, Pedagogical: 7, Content Structure: 5, Dependency: 4)
- Non-Goals: 10 excluded topics (Python syntax, Agent implementation, RAG pipeline, frontend UI, production deployment - all deferred to later chapters)
- Assumptions: 20 assumptions across 5 categories (Student Background, Technical Environment, Course Design, Content Format)
- Edge Cases: 5 failure scenarios with troubleshooting (network issues, invalid API key, OS differences, Python version, firewall)
- Acceptance Tests: 5 tests covering each lesson + overall chapter completion
- Dependencies: No prerequisites (entry chapter), downstream impacts on Ch2-16

**Validation Results** (spec-architect agent):
- Readiness Score: 9.8/10
- Testability: 10/10 (all criteria measurable, clear pass/fail)
- Completeness: 10/10 (comprehensive constraints, non-goals, edge cases)
- Ambiguity: 9/10 (one minor clarification on hint progression framework)
- Traceability: 10/10 (prerequisites, downstream impacts, business goals mapped)
- Formal Verification: PASS (5 invariants tested across 4 lessons, zero counterexamples found)
- Evals-First Pattern: PASS (success criteria defined before requirements)
- Verdict: ✅ READY FOR PLANNING (no clarification phase needed)

**Checklist Generated**: `specs/041-ch01-intro-apis-agents/checklists/requirements.md`

**Key Design Decisions**:
1. **No agent code in Chapter 1**: Enforced via TC-001 constraint - purely conceptual foundation
2. **Analogies as anchoring points**: Restaurant, smart person in room, chef's station, house blueprint - will be referenced in later chapters
3. **API key verification only code**: Lesson 1.1 has minimal 3-5 line script to verify setup
4. **Progressive hints framework**: Each lesson includes 4-6 graduated hints (conceptual → near-complete solution)
5. **Environment setup uses UV**: Aligned with course standards, not pip or poetry
6. **Lesson duration cap**: 50 minutes maximum per PC-007 (attention span constraint)
7. **Concept → Apply separation**: Every lesson teaches idea first, then applies to DocuBot project
8. **One concept per lesson**: PC-001 enforces single focus to prevent cognitive overload

## Outcome

- ✅ Impact: Created comprehensive specification for Chapter 1 (4 lessons) following evals-first pattern with 26 functional requirements, 10 measurable success criteria, 21 constraints, and 5 acceptance tests. Validated by spec-architect with 9.8/10 readiness score - READY FOR PLANNING.
- 🧪 Tests: 5 acceptance tests defined (API verification, conceptual understanding, environment setup completeness, architecture documentation, chapter completion criteria). Formal verification passed (5 invariants hold, 4 lessons tested, 0 counterexamples).
- 📁 Files: 2 files created - `specs/041-ch01-intro-apis-agents/spec.md` (450 lines), `specs/041-ch01-intro-apis-agents/checklists/requirements.md` (85 lines)
- 🔁 Next prompts: `/sp.plan 041-ch01-intro-apis-agents` to create implementation plan with lesson-by-lesson task breakdowns
- 🧠 Reflection: Specification demonstrates exceptional quality for educational content - all requirements are falsifiable, constraints prevent scope creep (no agent code allowed), and formal verification confirms internal consistency. The evals-first pattern ensures success criteria drive requirement design. Minor enhancement suggested for hint progression framework (FR-005) but not blocking.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification followed template structure correctly, included all mandatory sections, and passed formal verification.
- Graders run and results (PASS/FAIL): spec-architect validation = PASS (9.8/10), Testability = PASS (10/10), Completeness = PASS (10/10), Ambiguity = PASS (9/10 - minor clarification suggested), Traceability = PASS (10/10), Formal Verification = PASS (5 invariants hold, 0 counterexamples), Evals-First = PASS
- Prompt variant (if applicable): Standard `/sp.specify` command with feature description from user input
- Next experiment (smallest change to try): Apply hint progression framework enhancement to FR-005 in future lesson implementations to ensure consistent graduated disclosure across all courses
