---
id: 0001
title: chapter-9-markdown-spec
stage: spec
date: 2025-11-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-chapter-9-markdown
branch: 001-chapter-9-markdown
user: mjs
command: /sp.specify
labels: ["chapter-9", "markdown", "specification", "beginner-tier", "part-3", "aidd-workflow"]
links:
  spec: ../../../specs/001-chapter-9-markdown/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-chapter-9-markdown/spec.md
  - specs/001-chapter-9-markdown/checklists/requirements.md
tests:
  - Validation: All spec quality checklist items passed
---

## Prompt

```text
Design markdown chapter 09 for AIDD with AIDD based on our formal discussion.
```

**Context from prior conversation**:

User provided detailed discussion about Chapter 9 requirements:
1. Original plan had AI teaching foundational markdown (violated Constitution Principle 13)
2. Correct approach: Book teaches Tier 1 (foundational), AI Companion handles Tier 2 (complex), AI Orchestration handles Tier 3 (scale)
3. Chapter must align with AIDD philosophy: markdown as specification language (Intent Layer)
4. Target audience: Complete beginners (Parts 1-3), max 5 concepts per section, max 2 options
5. Focus on three user stories: Writing specs (P1), Reading AI output (P2), GitHub collaboration (P3)

## Response snapshot

Created comprehensive specification for Chapter 9 following spec-template.md structure:

**User Scenarios** (3 prioritized stories):
- P1: Write Clear Project Specifications (foundation of AIDD)
- P2: Read and Understand AI-Generated Documentation (validation-first safety)
- P3: Collaborate on Specifications Using GitHub (real-world workflow)

**Functional Requirements** (24 total, organized by tier):
- FR-001 to FR-009: Tier 1 (Book teaches directly) - Headings, paragraphs, emphasis, lists, code blocks, links
- FR-010 to FR-012: Tier 2 (AI Companion) - Tables, nested lists, front matter
- FR-013 to FR-015: Tier 3 (AI Orchestration) - Multi-file generation, bulk conversion, site structure
- FR-016 to FR-019: AIDD Integration - Spec writing, AI output reading, workflow understanding
- FR-020 to FR-024: Constitution Compliance - Teaching tiers, cognitive load, validation

**Success Criteria** (11 measurable outcomes):
- SC-001 to SC-004: Skill acquisition (90%/85%/80%/75% benchmarks)
- SC-005 to SC-006: AIDD workflow integration
- SC-007 to SC-008: Comprehension (80% quiz pass, Grade 7-8 reading level)
- SC-009 to SC-010: Engagement (70%/80% completion rates)
- SC-011: Portfolio evidence (GitHub README)

**Edge Cases**: 7 common beginner mistakes documented

**Assumptions**: 10 documented (prerequisites, tools, environment, scope, pedagogy, assessment, etc.)

**Open Questions**: None (zero [NEEDS CLARIFICATION] markers)

**Validation**: Created requirements.md checklist - all items passed

## Outcome

- ✅ Impact: Specification complete and validated, ready for `/sp.plan` phase
- 🧪 Tests: Spec quality checklist validated (all items passed)
- 📁 Files: spec.md (215 lines), checklists/requirements.md (159 lines)
- 🔁 Next prompts: `/sp.plan` to break spec into lesson-by-lesson structure, then `/sp.tasks` for implementation checklist
- 🧠 Reflection: Successfully applied Constitution Principle 13 (Graduated Teaching Pattern) with explicit three-tier structure. Zero clarification markers achieved by making informed assumptions documented in Assumptions section. Success criteria are measurable and technology-agnostic. Beginner cognitive load limits enforced throughout.

## Evaluation notes (flywheel)

- Failure modes observed: None. Spec completed in first iteration with zero [NEEDS CLARIFICATION] markers.
- Graders run and results (PASS/FAIL): Requirements checklist validation PASSED (all items checked)
- Prompt variant (if applicable): N/A - Standard `/sp.specify` workflow
- Next experiment (smallest change to try): When moving to planning phase, validate that lesson breakdown maintains cognitive load limits (max 5 concepts per section) and three-tier teaching pattern consistency across all lessons.

## Key Decisions Documented

1. **Three-Tier Teaching Structure**: Tier 1 (Book teaches foundational syntax directly) → Tier 2 (AI Companion handles complex syntax) → Tier 3 (AI Orchestration handles scale). This aligns with Constitution Principle 13 and corrects the original "AI teaches basics" approach.

2. **Markdown as Specification Language**: Framed markdown not as "text formatting" but as "Intent Layer" in AIDD workflow. Students learn markdown to WRITE CLEAR SPECS (primary skill), not just format documents.

3. **Validation-First Safety**: P2 user story (Reading AI Output) teaches students to VALIDATE AI-generated markdown responses, not blindly trust them. This enforces constitution principle about validation.

4. **Zero Clarification Markers**: Made informed assumptions for all potential ambiguities (markdown flavor = CommonMark + GFM, reading level = Grade 7-8, assessment = combination of automated/observable/self-reported). Documented all assumptions explicitly.

5. **Measurable Success Criteria**: All SC items are quantitative (percentages, completion rates, readability scores) and user-facing (what students CAN DO, not system internals).

6. **Scope Boundaries**: IN SCOPE = Essential markdown for AIDD (specs, AI communication, GitHub). OUT OF SCOPE = Advanced flavors, HTML-in-markdown, editor comparisons. Keeps beginner chapter focused.

## Alignment with Constitution v3.0.0

- ✅ Principle 12: Cognitive Load Management (max 5 concepts per section, max 2 options)
- ✅ Principle 13: Graduated Teaching Pattern (explicit three-tier structure)
- ✅ Principle 14-17: Planning-First (specification created before implementation)
- ✅ Evals-First: Success criteria defined BEFORE implementation planning
- ✅ Specification-First Enforcement: Spec approved before moving to plan phase
- ✅ Validation-First Safety: P2 user story teaches reading/validating AI output

## AIDD Workflow Demonstration

This specification itself demonstrates AIDD:

1. **Intent Layer** (This spec): WHAT Chapter 9 should teach (markdown as specification language)
2. **Reasoning Layer** (Next: `/sp.plan`): AI will break spec into lesson structure
3. **Implementation** (Then: content-implementer): AI generates actual lesson content
4. **Validation** (Finally: validation-auditor): Verify output matches spec

The chapter content will teach students this same workflow using markdown as the medium.
