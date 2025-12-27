---
id: "0002"
title: "chapter-40-implementation-plan"
stage: plan
date: 2025-12-27
surface: cli
model: claude-opus-4-5
feature: 040-chapter-40-fastapi-for-agents
branch: main
user: mjs
command: /sp.plan
labels: [plan, chapter-40, fastapi, chapter-planner]
links:
  spec: specs/040-chapter-40-fastapi-for-agents/spec.md
  ticket: "#541, #542, #543, #544, #545, #546"
  adr: null
  pr: null
files:
  - specs/040-chapter-40-fastapi-for-agents/plan.md (created)
tests:
  - null
---

## Prompt

Create implementation plan for Chapter 40: FastAPI for Agents based on spec v3.0.0.

Key context:
- 13 lessons total (up from 8)
- Issues #541-#546 define the changes
- One concept per lesson principle
- FastAPI docs as canonical source
- L2 is L1 Manual First (pytest by hand)
- APIs become agent tools at end

## Response snapshot

Created comprehensive plan.md (v2.0.0) with:

1. **Pedagogical Arc**:
   - Foundation (L1-L5): FastAPI basics + testing
   - Production Essentials (L6-L9): Database, auth, security
   - Architecture (L10-L11): DI + streaming
   - Agent Integration (L12-L13): APIs become tools + Digital FTE

2. **Layer Progression**:
   - L02: Layer 1 (Manual) - Students write pytest tests by hand
   - L10, L12: Layer 2 (AI Collaboration)
   - L13: Layer 4 (Spec-Driven) - Digital FTE capstone

3. **File Operations**:
   - KEEP AS-IS: 1 (L01)
   - RENAME ONLY: 4 (L03, L04, L05, L11)
   - CREATE NEW: 5 (L02, L06, L07, L08, L09)
   - REWRITE: 3 (L10, L12, L13)

4. **Legacy Content Removal** (L12):
   - ~400 lines removed (complex handoffs, specialists)
   - Keep core insight: APIs -> Functions -> Tools

5. **Cognitive Load Validation**:
   - All 13 lessons within B1 limit (<=10 concepts)

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation
- 🧪 Tests: N/A (plan phase)
- 📁 Files: plan.md created at specs/040-chapter-40-fastapi-for-agents/plan.md
- 🔁 Next prompts: /sp.tasks to generate tasks.md
- 🧠 Reflection: chapter-planner subagent properly routed for content work. Plan includes detailed file operations, legacy removal, and validation checklists.

## Evaluation notes (flywheel)

- Failure modes observed: None - used chapter-planner subagent correctly
- Graders run and results (PASS/FAIL): N/A (plan phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to /sp.tasks
