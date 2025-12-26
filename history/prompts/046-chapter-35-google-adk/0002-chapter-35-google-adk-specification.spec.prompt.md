---
id: "0002"
title: chapter-35-google-adk-specification
stage: spec
date: 2025-12-26
surface: cli
model: claude-opus-4-5-20251101
feature: 046-chapter-35-google-adk
branch: 046-chapter-35-google-adk
user: mjs
command: /sp.specify
labels: [chapter-35, google-adk, specification, phase-b]
links:
  spec: specs/046-chapter-35-google-adk/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/046-chapter-35-google-adk/spec.md
  - specs/046-chapter-35-google-adk/checklists/requirements.md
tests:
  - Specification quality checklist passed
---

## Prompt

/sp.chapter Chapter 35: Google Agent Development Kit (ADK) This chapter teaches us to build custom AI agents that really matter - reliable, safe, and evaluated on real tasks. Key topics: ADK setup with Gemini/Vertex AI, Agent types (LlmAgent, SequentialAgent, ParallelAgent, LoopAgent), Tools (FunctionTool, AgentTool, McpToolset), Session management, Guardrails and callbacks, Evaluation with adk eval, Deployment to Vertex AI Agent Engine and Cloud Run. The running example is TaskManager Agent.

## Response snapshot

Created comprehensive specification for Chapter 35: Google Agent Development Kit (ADK) with:

**9 User Stories (P1-P3)**:
1. Setting Up First ADK Agent (P1)
2. Adding Tools to ADK Agents (P1)
3. Implementing Multi-Agent Workflows (P1)
4. Integrating MCP Servers with ADK (P2)
5. Managing Agent State with Sessions (P2)
6. Adding Guardrails with Callbacks (P2)
7. Evaluating Agents with adk eval (P2)
8. Deploying to Vertex AI Agent Engine (P3)
9. Comparing ADK to OpenAI SDK (P3)

**41 Functional Requirements** covering:
- Environment & Setup (FR-001 to FR-003)
- Agent Creation & Configuration (FR-004 to FR-006)
- Tools & Integration (FR-007 to FR-011)
- Multi-Agent Workflows (FR-012 to FR-015)
- Session & State Management (FR-016 to FR-019)
- Streaming & Async Patterns (FR-020 to FR-022)
- Guardrails & Safety (FR-023 to FR-025)
- Evaluation (FR-026 to FR-029)
- Deployment (FR-030 to FR-033)
- Framework Comparison (FR-034 to FR-037)
- Pedagogical Requirements (FR-038 to FR-041)

**18 Success Criteria** measuring learning outcomes.

**Educational Structure**:
- Proficiency Level: B1 (Intermediate)
- Layer Progression: L1 Manual → L2 Collaboration → L3 Intelligence → L4 Spec-Driven
- Suggested 8 lessons with clear progression
- TaskManager consistency with Chapter 34

## Outcome

- ✅ Impact: Specification complete and validated for Chapter 35
- 🧪 Tests: Requirements checklist passed (all items validated)
- 📁 Files: spec.md, checklists/requirements.md created
- 🔁 Next prompts: /sp.plan for implementation planning
- 🧠 Reflection: Phase A skill-first approach provided comprehensive API patterns for specification

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - all checklist items validated
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to /sp.plan
