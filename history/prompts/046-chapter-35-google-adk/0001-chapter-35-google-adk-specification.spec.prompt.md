---
id: "0001"
title: "chapter-35-google-adk-specification"
stage: spec
date: 2025-12-26
surface: cli
model: claude-sonnet-4-5-20250929
feature: "046-chapter-35-google-adk"
branch: "046-chapter-35-google-adk"
user: mjs
command: "/sp.specify"
labels: [educational-content, chapter-spec, agent-frameworks, google-adk]
links:
  spec: "specs/046-chapter-35-google-adk/spec.md"
  ticket: ""
  adr: ""
  pr: ""
files:
  - path: "specs/046-chapter-35-google-adk/spec.md"
    role: created
    lines: 500
  - path: "specs/046-chapter-35-google-adk/checklists/requirements.md"
    role: created
    lines: 65
tests: []
---

## Prompt

Create a comprehensive specification for Chapter 35: Google Agent Development Kit (ADK) based on this context:

**Feature Description**:
Chapter 35: Google Agent Development Kit (ADK) - This chapter teaches building custom AI agents that are reliable, safe, and evaluated on real tasks using Google's ADK framework. The chapter builds on Chapter 34 (OpenAI SDK) to show how the same agent patterns manifest in Google's ecosystem. Key topics: ADK setup with Gemini/Vertex AI, Agent types (LlmAgent, SequentialAgent, ParallelAgent, LoopAgent), Tools (FunctionTool, AgentTool, McpToolset), Session management, Guardrails and callbacks, Evaluation with adk eval, Deployment to Vertex AI Agent Engine and Cloud Run.

**Context from Part 6 README**:
- Part 6 bridges foundational skills into production agent development
- Chapter 35 is in the "Agent Frameworks (Chapters 33-36)" stage
- Running example: TaskManager Agent
- Proficiency Level: B1 (intermediate) based on typical Part 6 content

**Prerequisites**:
- Chapter 33: Introduction to AI Agents (conceptual foundation)
- Chapter 34: OpenAI Agents SDK (comparison baseline)
- Part 5: Python Fundamentals (async/await, type hints)

**What this chapter teaches from scratch**:
- Google ADK architecture (Agent, Runner, SessionService)
- Gemini/Vertex AI configuration
- ADK-specific patterns (SequentialAgent, ParallelAgent, LoopAgent)
- MCP integration with ADK (McpToolset)
- ADK evaluation framework (adk eval)
- Vertex AI Agent Engine deployment
- Cloud Run deployment

## Response snapshot

Created comprehensive specification for Chapter 35 with:

**9 User Stories** (prioritized P1-P3):
1. Setting Up First ADK Agent (P1) - Environment configuration and first agent
2. Adding Tools to ADK Agents (P1) - Function tools with auto-wrapping
3. Implementing Multi-Agent Workflows (P1) - SequentialAgent, ParallelAgent, LoopAgent
4. Integrating MCP Servers (P2) - McpToolset with MCP protocol
5. Managing Agent State with Sessions (P2) - SessionService persistence
6. Adding Guardrails with Callbacks (P2) - before_model_callback, before_tool_callback
7. Evaluating Agents with adk eval (P2) - JSON test files and evaluation
8. Deploying to Vertex AI Agent Engine (P3) - Production deployment
9. Comparing ADK to OpenAI SDK (P3) - Framework comparison and meta-cognition

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

**18 Success Criteria** measuring:
- Technical competency (SC-001 to SC-005)
- Evaluation & testing skills (SC-006 to SC-008)
- Deployment capabilities (SC-009 to SC-010)
- Framework understanding (SC-011 to SC-013)
- Capstone validation (SC-014 to SC-015)
- Learning experience (SC-016 to SC-018)

**Key Features**:
- Comprehensive **Assumed Knowledge** section mapping prerequisites from Chapters 33, 34, Part 4, Part 5
- Explicit **framework comparison** requirements (ADK vs OpenAI SDK)
- **TaskManager consistency** with Chapter 34 for direct comparison
- **8-lesson structure** suggestion with layer progression (L1→L2→L3→L4)
- **Edge cases** for authentication, debugging, MCP connections, deployment
- **References** to canonical sources (ADK skill, Chapter 33 spec, Constitution v7.0.0)

## Outcome

- ✅ Impact: Complete specification for Chapter 35 created, ready for planning phase
- 🧪 Tests: Specification quality checklist created and validated (all items pass)
- 📁 Files: spec.md (500 lines), checklists/requirements.md (65 lines)
- 🔁 Next prompts: `/sp.plan` to generate implementation plan, or `/sp.clarify` if clarification needed (none identified)
- 🧠 Reflection: Specification successfully integrates educational content requirements (Assumed Knowledge, proficiency level, layer progression) with technical requirements (9 user stories, 41 functional requirements, 18 success criteria). Strong alignment with Chapter 34 for framework comparison. ADK-specific patterns (workflow agents, Vertex AI deployment, built-in evaluation) clearly differentiated from OpenAI SDK patterns.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Specification quality checklist - PASS (all items validated)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A - specification complete and validated
