---
id: "001"
title: "Redesign Chapter 31: SpecifyPlus Hands-On"
stage: "spec"
feature: "chapter-31-redesign"
date_created: "2025-11-03"
surface: "agent"
model: "claude-haiku-4-5-20251001"
branch: "10-chapter-31-redesign"
user: "architect"
command: "/sp.specify"
labels: ["chapter-31", "specifyplus", "sdd", "aidd", "hands-on"]
links:
  spec: "/specs/10-chapter-31-redesign/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "/specs/10-chapter-31-redesign/spec.md"
  - "/history/prompts/10-chapter-31-redesign/001-redesign-chapter-31-specifyplus.spec.prompt.md"
tests: []
---

## Prompt Text

Redesign Chapter 31 @book-source/docs/05-Spec-Kit-Plus-Methodology/31-spec-kit-plus-hands-on to eliminate hallucinated commands, teach actual SpecifyPlus workflow, center on AIDD paradigm with 7 hands-on lessons plus /sp.implement capstone

First fetch SpecifyPlus or SpecKit Plus documentation using context7 mcp server tools and carefully understand the opinionated SDD framework/toolkit. Then study current chapter to identify all hallucinated work. Finally rethink the user stories and specification.

You can look at the preface and previous chapters especially to see goals.

**User guidance**:

- Don't need lesson 1 (specs independently created in detail in chapter 1)
- uvx specifyplus init command IS valid (installation via pip or 1-time usage)
- Remove mini project 2
- Complete work with /sp.implement command as capstone

REVISED STRUCTURE: 7 lessons (not 8) + capstone using /sp.implement

---

## Response Text

**Analysis Complete**: Comprehensive redesign specification created for Chapter 31.

### Key Findings

**Critical Issues in Current Chapter**:

1. Lesson 2 misrepresents SpecifyPlus project initialization
2. Lessons 4-5 show `/sp.specify` and `/sp.plan` as standalone terminal commands
3. Unclear distinction between manual thinking vs. tool-assisted workflows
4. Missing `/sp.implement` as capstone integration tool
5. Lessons 8-9 (mini projects) not concrete enough for hands-on learning

### Corrected Architecture

**New 7-Lesson Structure**:

1. **Lesson 1**: SMART Acceptance Criteria (2h)

   - Start with vague criteria, see bad code generated
   - Learn SMART framework, refine criteria, see good code
   - Experiential learning of clarity principle

2. **Lesson 2**: SpecifyPlus Project Structure & Cascade (1.5h)

   - Initialize project: `pip install specifyplus` or `uvx specifyplus init`
   - Explore folder structure: `.specify/`, `specs/`, `history/`
   - Understand why structure enforces workflow

3. **Lesson 3**: Complete Specification Writing (2h)

   - Write full specification with all required components
   - Map manual thinking (Lesson 1) to formal template sections
   - Validate spec is "ready for planning"

4. **Lesson 4**: Refining Specs with /sp.specify (1.5h)

   - Use `/sp.specify` within Claude Code to analyze spec
   - Iterate based on AI feedback
   - Experience iterative nature of clarity

5. **Lesson 5**: Planning from Specification (/sp.plan) (1.5h)

   - Use `/sp.plan` to generate implementation plan
   - Understand cascade: clear spec → clear plan
   - Recognize dependencies and sequencing

6. **Lesson 6**: Decomposing Plan into Tasks (/sp.tasks) (1.5h)

   - Use `/sp.tasks` to break plan into atomic units
   - Trace lineage: Spec → Plan → Tasks
   - Understand task dependencies

7. **Lesson 7**: Implementation with /sp.implement (2-3h)
   - Use `/sp.implement` to orchestrate code generation
   - Validate generated code against acceptance criteria
   - Learn validation as core skill

**Capstone**: Full Workflow End-to-End (3-4h)

- Pick project (Calculator simple, or Grading System complex)
- Run complete workflow: Specify → Clarify → Plan → Tasks → Implement
- Validate final code against original specification
- Document learnings and reflection

### Specification Deliverables

**File**: `/specs/10-chapter-31-redesign/spec.md`

**Content**:

- 5 User Stories (P1-P3 priority)
- 8 Functional Requirements
- 7 Lessons + Capstone (fully specified)
- Learning Objectives per lesson
- CEFR/Bloom's proficiency mapping
- 7 Success Criteria
- Risk analysis with mitigations
- Assumptions and constraints
- Dependencies mapping

### Key Principles Reflected

✅ **AIDD Paradigm** (from Preface):

- Student: Intent → AI: Suggestions → Student: Validation → AI: Refinement
- Demonstrates co-learning throughout

✅ **Specification-First** (foundational):

- All 7 lessons build on clarity and specification
- Cascade effect demonstrated: Spec → Plan → Tasks → Code

✅ **Hands-On Learning**:

- Every lesson includes concrete projects (grading system, calculator)
- Students experience problems before learning solutions
- No pure theory; all concepts demonstrated

✅ **Correct SpecifyPlus Workflow**:

- Commands shown in correct context (Claude Code, not terminal)
- All hallucinated references removed
- Actual workflow: Constitution → Specify → Clarify → Plan → Tasks → Implement

---

## Evaluation

**Specification Quality**: ✅ **READY FOR PLANNING**

**Completeness Checklist**:

- [x] All 7 lessons clearly described with learning objectives
- [x] Each lesson has independent, testable deliverable
- [x] Capstone project concrete and achievable
- [x] All hallucinated commands eliminated/corrected
- [x] Actual SpecifyPlus workflow accurately described
- [x] AIDD paradigm from Preface reflected throughout
- [x] Cascade effect (Spec → Plan → Tasks → Code) demonstrated
- [x] Manual thinking (Lessons 1-3) distinguished from tool usage (Lessons 4-7)
- [x] Two real projects specified (Calculator + Grading System option)
- [x] Complexity matches "Intermediate" tier for Part 5
- [x] Each lesson clearly shows when students think vs. when tools assist
- [x] No forward references without explanation

**Specification Status**: Draft → **Ready for `/sp.clarify` or `/sp.plan`**

---

## Version History

| Date       | Version | Status                             |
| ---------- | ------- | ---------------------------------- |
| 2025-11-03 | 1.0     | Draft Complete, Ready for Planning |

---

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
