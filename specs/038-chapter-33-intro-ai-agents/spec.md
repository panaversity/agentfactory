# Feature Specification: Chapter 33 — Introduction to AI Agents

**Feature Branch**: `038-chapter-33-intro-ai-agents`
**Created**: 2025-11-27
**Revised**: 2025-11-27 (Aligned with Google "Introduction to Agents" whitepaper)
**Status**: Draft — Revised
**Input**: User description: "Chapter 33: Introduction to AI Agents — First chapter of Part 6: AI Native Software Development"
**Primary Source**: Google/Kaggle "Introduction to Agents" whitepaper (November 2025) by Blount, Gulli, Saboo, Zimmermann, Vuskovic

---

## Executive Summary

Chapter 33 introduces students to AI agents as the foundational chapter of Part 6 (AI Native Software Development). This is a **conceptual chapter** that establishes mental models using the **authoritative frameworks from Google's "Introduction to Agents" whitepaper** (November 2025).

**Core Teaching**: AI agents are "LMs in a loop with tools to accomplish an objective" — complete applications that combine a Model's reasoning ability with Tools to act, governed by an Orchestration Layer, and made accessible through Deployment services.

**Key Frameworks from Paper**:
1. **5-Level Taxonomy**: Level 0 (Core Reasoning) → Level 4 (Self-Evolving System)
2. **3+1 Architecture**: Model ("Brain") + Tools ("Hands") + Orchestration ("Nervous System") + Deployment ("Body")
3. **5-Step Operational Loop**: Get Mission → Scan Scene → Think → Act → Observe
4. **Agent Ops**: Metrics-driven development, LM-as-Judge evaluation, OpenTelemetry debugging
5. **Agent Interoperability**: A2A protocol, Agent Cards, human-agent-agent ecosystems

**Key Statistics to Embed** (from paper and McKinsey):
- 800+ million people use ChatGPT weekly
- 90%+ of developers use AI coding tools
- 44% of US work hours could involve AI agent tasks (McKinsey)
- $2.9 trillion economic value potential by 2030
- 7x growth in AI fluency demand (fastest-growing skill)

---

## User Scenarios & Testing

### User Story 1 — Understanding What an AI Agent Is (Priority: P1)

A student who has completed Parts 1-5 wants to understand what distinguishes an AI agent from the chatbots and AI tools they've already used (Claude Code, Gemini CLI).

**Why this priority**: Foundation for all subsequent content. Without this distinction, students cannot properly understand agent architectures or make informed framework choices.

**Independent Test**: Student can explain to a colleague the difference between a Level 0 system (LLM alone) and a Level 1-3 agent — using the paper's taxonomy and concrete examples.

**Acceptance Scenarios**:

1. **Given** a student has used Claude Code extensively, **When** they complete Lesson 1, **Then** they can articulate why Claude Code is a "Level 2 Strategic Problem-Solver" (context engineering, multi-step planning) vs a simple chatbot (Level 0 reasoning only).

2. **Given** a student reads the agent definition, **When** they encounter real-world examples, **Then** they can classify systems using the **5-Level Taxonomy**:
   - Level 0: Core Reasoning System (LLM alone, no tools)
   - Level 1: Connected Problem-Solver (LLM + tools for real-time data)
   - Level 2: Strategic Problem-Solver (context engineering for multi-step plans)
   - Level 3: Collaborative Multi-Agent System (team of specialists)
   - Level 4: Self-Evolving System (can create new tools/agents)

3. **Given** a student understands the taxonomy, **When** asked about career implications, **Then** they can explain why agent development skills are valuable (citing statistics and the paper's "director vs bricklayer" paradigm shift).

---

### User Story 2 — Grasping Agent Architecture (Priority: P1)

A student wants to understand the core architecture that makes up an AI agent system so they can recognize these components in any framework.

**Why this priority**: Architectural understanding is prerequisite for all SDK-specific chapters (34-36) and for understanding framework trade-offs.

**Independent Test**: Student can draw/describe the **3+1 Core Architecture** (Model + Tools + Orchestration + Deployment) and explain the 5-step operational loop.

**Acceptance Scenarios**:

1. **Given** a student reads about agent architecture, **When** they see a diagram, **Then** they can identify the paper's canonical architecture:
   - **Model** ("Brain"): The reasoning engine that processes information and makes decisions
   - **Tools** ("Hands"): Mechanisms connecting reasoning to the outside world (APIs, functions, data stores)
   - **Orchestration** ("Nervous System"): The governing process managing planning, memory, and reasoning strategy
   - **Deployment** ("Body"): Runtime services making the agent accessible (hosting, A2A API)

2. **Given** architecture knowledge, **When** tracing through a request, **Then** student can explain the **5-Step Operational Loop**:
   1. Get the Mission (goal or trigger)
   2. Scan the Scene (perceive environment/context)
   3. Think It Through (reasoning, planning)
   4. Take Action (tool invocation)
   5. Observe and Iterate (feedback loop)

3. **Given** understanding of architecture, **When** comparing frameworks, **Then** student recognizes these components across OpenAI, Google ADK, Anthropic implementations.

---

### User Story 3 — Recognizing Multi-Agent Patterns (Priority: P2)

A student wants to learn the common design patterns used in agentic systems so they can choose appropriate patterns for different use cases.

**Why this priority**: Pattern recognition enables informed architecture decisions in later chapters, but builds on architectural understanding.

**Independent Test**: Student can match use cases to appropriate patterns from the paper: Coordinator, Sequential, Iterative Refinement, Human-in-the-Loop.

**Acceptance Scenarios**:

1. **Given** pattern descriptions from the paper, **When** presented with a use case (e.g., "complex request needing multiple specialists"), **Then** student selects **Coordinator pattern** (manager agent routing to specialists) and explains why.

2. **Given** understanding of patterns, **When** asked about linear workflows, **Then** student can explain **Sequential pattern** (assembly line where output flows to next agent).

3. **Given** **Iterative Refinement pattern**, **When** asked about quality assurance, **Then** student can explain generator-critic feedback loop.

4. **Given** **Human-in-the-Loop pattern**, **When** asked about high-stakes decisions, **Then** student can explain deliberate workflow pauses for human approval.

---

### User Story 4 — Navigating the SDK Landscape (Priority: P2)

A student wants a mental map of the major agent frameworks/SDKs so they know what options exist and can make informed choices.

**Why this priority**: Prerequisite for SDK-specific chapters (34-36), prevents confusion when encountering different frameworks.

**Independent Test**: Student can name 4+ agent frameworks, describe their primary characteristics, and state when each might be preferred.

**Acceptance Scenarios**:

1. **Given** framework overview, **When** asked to compare OpenAI Agents SDK vs Google ADK, **Then** student can articulate 2-3 key differences.

2. **Given** SDK landscape knowledge, **When** starting a new agent project, **Then** student can list factors to consider when choosing a framework.

3. **Given** brief SDK descriptions, **When** reading documentation for any framework, **Then** student recognizes familiar concepts (tools, orchestration, memory).

---

### User Story 5 — Understanding Agent Ops (Priority: P2)

A student wants to understand how to operate agents in production — evaluation, debugging, and continuous improvement.

**Why this priority**: The paper emphasizes that "Agent Ops" is critical for production success. Building a prototype is easy; ensuring quality and reliability is the real challenge.

**Independent Test**: Student can describe the Agent Ops discipline and explain key operational practices.

**Acceptance Scenarios**:

1. **Given** the paper's Agent Ops framework, **When** asked about agent evaluation, **Then** student can explain:
   - **LM as Judge**: Using a model to assess agent output quality against rubrics
   - **Golden datasets**: Evaluation scenarios with ideal responses
   - **Metrics-driven development**: Go/no-go based on quality scores

2. **Given** debugging requirements, **When** asked how to understand agent behavior, **Then** student can explain **OpenTelemetry traces** for inspecting agent reasoning trajectories (prompts, tool calls, observations).

3. **Given** human feedback, **When** asked about improvement, **Then** student can explain the feedback loop: capture feedback → replicate issue → add to evaluation dataset → prevent recurrence.

---

### User Story 6 — Understanding Agent Interoperability (Priority: P2)

A student wants to understand how agents connect with humans, other agents, and systems — the "face" of the agent.

**Why this priority**: Agents don't operate in isolation. Understanding interoperability patterns is essential for designing useful agent systems.

**Independent Test**: Student can describe agent interaction patterns from the paper: human-agent, agent-agent, and security considerations.

**Acceptance Scenarios**:

1. **Given** the paper's interoperability framework, **When** asked about agent-human interaction, **Then** student can describe: chatbots, computer use, live mode (bidirectional streaming), and multimodal communication.

2. **Given** agent-agent communication needs, **When** asked about connecting agents, **Then** student can explain:
   - **A2A Protocol**: Universal handshake for agentic economy
   - **Agent Cards**: JSON files advertising capabilities and endpoints
   - **Task-oriented architecture**: Asynchronous task requests with streaming updates

3. **Given** security concerns, **When** asked about agent identity, **Then** student can explain the paper's framework:
   - Agents as a new class of **principal** (distinct from users and service accounts)
   - **Trust trade-off**: Utility vs security tension
   - **Defense in depth**: Guardrails + AI-powered guard models

---

### User Story 7 — Understanding Human-Agent Partnership (Priority: P2)

A student wants to understand the collaboration model between humans and AI agents — the new "director vs bricklayer" paradigm.

**Why this priority**: Career-critical understanding; the paper emphasizes that agent developers are "directors" guiding autonomous actors, not "bricklayers" defining every step.

**Independent Test**: Student can describe the human-agent partnership model using the paper's frameworks.

**Acceptance Scenarios**:

1. **Given** the paper's paradigm shift description, **When** asked about the developer role change, **Then** student can explain: "The traditional developer acts as a 'bricklayer,' precisely defining every logical step. The agent developer is more like a director—setting the scene, selecting the cast, providing context."

2. **Given** McKinsey research insights, **When** asked about job displacement fears, **Then** student can explain workflow transformation (not replacement) with specific examples.

3. **Given** partnership model, **When** designing an agent workflow, **Then** student includes appropriate human checkpoints for decisions, creativity, ethics.

---

### User Story 8 — Preparing for Hands-On Development (Priority: P3)

A student completing Chapter 33 wants to feel confident and excited to begin Chapter 34 (OpenAI Agents SDK hands-on).

**Why this priority**: Transition chapter must create readiness, not just convey information.

**Independent Test**: Student expresses confidence in starting SDK work and can articulate what they'll learn next.

**Acceptance Scenarios**:

1. **Given** chapter completion, **When** previewing Chapter 34, **Then** student recognizes conceptual foundations (components, patterns) they'll implement.

2. **Given** conceptual understanding, **When** approaching SDK documentation, **Then** student can map documentation concepts to learned mental models.

3. **Given** excitement about agents, **When** asked about learning goals, **Then** student can articulate specific agent capabilities they want to build.

---

### Edge Cases

- **Student confusion**: What if student thinks Claude Code IS an agent vs uses agentic patterns?
  - Address explicitly: Claude Code demonstrates agentic capabilities; Chapter 33 teaches the underlying concepts

- **Framework overwhelm**: What if student feels overwhelmed by SDK choices?
  - Address: Emphasize that concepts transfer across frameworks; specific SDK choice is secondary

- **Code expectations**: What if student expects to write code in this chapter?
  - Address: Explicitly state this is conceptual foundation; code begins Chapter 34

- **Statistics skepticism**: What if student questions the statistics?
  - Address: All statistics properly cited with sources; acknowledge rapid change in field

---

## Requirements

### Functional Requirements

**Core Frameworks from Paper (MUST align)**:

- **FR-001**: Chapter MUST teach the **5-Level Taxonomy** from the paper:
  - Level 0: Core Reasoning System (LLM alone)
  - Level 1: Connected Problem-Solver (LLM + tools)
  - Level 2: Strategic Problem-Solver (context engineering, multi-step planning)
  - Level 3: Collaborative Multi-Agent System (team of specialists)
  - Level 4: Self-Evolving System (creates new tools/agents)

- **FR-002**: Chapter MUST teach the **3+1 Core Architecture** from the paper:
  - Model ("Brain"): Reasoning engine with model selection guidance
  - Tools ("Hands"): RAG, APIs, code execution, function calling, MCP
  - Orchestration ("Nervous System"): Planning, memory (short-term + long-term), reasoning strategies
  - Deployment ("Body"): Runtime services, hosting, A2A APIs

- **FR-003**: Chapter MUST teach the **5-Step Operational Loop**:
  1. Get the Mission (goal/trigger)
  2. Scan the Scene (perceive environment)
  3. Think It Through (reasoning/planning)
  4. Take Action (tool invocation)
  5. Observe and Iterate (feedback loop)

- **FR-004**: Chapter MUST present the paper's **Multi-Agent Design Patterns**:
  - Coordinator pattern (manager routing to specialists)
  - Sequential pattern (assembly line flow)
  - Iterative Refinement pattern (generator-critic loop)
  - Human-in-the-Loop pattern (deliberate pauses for approval)

- **FR-005**: Chapter MUST introduce **Agent Ops** discipline:
  - LM as Judge evaluation
  - Golden datasets and metrics-driven development
  - OpenTelemetry debugging
  - Human feedback loops

- **FR-006**: Chapter MUST introduce **Agent Interoperability**:
  - Agent-Human: chatbots, computer use, live mode, multimodal
  - Agent-Agent: A2A protocol, Agent Cards, task-oriented architecture
  - Security: Agent identity as principal, trust trade-off, defense in depth

- **FR-007**: Chapter MUST provide overview of 4+ agent frameworks/SDKs with comparison guidance (OpenAI, Google ADK, Anthropic, LangChain)

- **FR-008**: Chapter MUST explain the **"director vs bricklayer"** paradigm shift with career relevance

- **FR-009**: Chapter MUST include accurate, cited statistics (800M users, 90% developer adoption, 44% work hours, $2.9T value, 7x growth)

**Pedagogical Requirements**:

- **FR-010**: Chapter MUST follow 4-Layer Teaching progression (L1 Manual → L2 AIDD → L3 Intelligence) per lesson where applicable
- **FR-011**: Each lesson MUST end with "Try With AI" section (action prompts, not meta-commentary)
- **FR-012**: Chapter MUST NOT include code implementations (deferred to Chapters 34+)
- **FR-013**: Chapter MUST connect to SDD-RI mindset from Part 4 (specifications before implementation)
- **FR-014**: The Google "Introduction to Agents" whitepaper MUST be the primary authoritative source

### Key Entities (Per Paper)

- **AI Agent**: "The combination of models, tools, an orchestration layer, and runtime services which uses the LM in a loop to accomplish a goal" — complete application making plans and taking actions
- **5-Level Taxonomy**: Classification of agentic systems from Level 0 (Core Reasoning) to Level 4 (Self-Evolving)
- **3+1 Architecture**: Model ("Brain") + Tools ("Hands") + Orchestration ("Nervous System") + Deployment ("Body")
- **5-Step Loop**: Get Mission → Scan Scene → Think → Act → Observe — the universal agentic process
- **Context Engineering**: "The agent's ability to actively select, package, and manage the most relevant information for each step of its plan"
- **Agent Ops**: Disciplined approach to evaluating, debugging, and operating agentic systems in production
- **A2A Protocol**: Agent-to-Agent communication standard with Agent Cards for discovery
- **Agent Identity**: Agents as a new class of principal requiring verifiable identity distinct from users and services
- **Agent Framework/SDK**: Development toolkit for building agents (OpenAI, Google ADK, Anthropic, LangChain)

---

## Success Criteria

### Measurable Outcomes (Aligned with Paper Frameworks)

- **SC-001**: Student can correctly classify 5+ systems using the **5-Level Taxonomy** (Level 0-4)
- **SC-002**: Student can draw and explain the **3+1 Core Architecture** (Model, Tools, Orchestration, Deployment)
- **SC-003**: Student can walk through the **5-Step Operational Loop** with a concrete example
- **SC-004**: Student can match 4 use cases to appropriate **Multi-Agent Patterns** (Coordinator, Sequential, Iterative Refinement, HITL)
- **SC-005**: Student can name 4+ agent frameworks and articulate key differences using paper's guidance
- **SC-006**: Student can explain **Agent Ops** basics: LM-as-Judge, golden datasets, traces
- **SC-007**: Student can describe **A2A Protocol** and **Agent Cards** for agent interoperability
- **SC-008**: Student can explain agent **security fundamentals**: identity as principal, trust trade-off
- **SC-009**: Student can articulate the **"director vs bricklayer"** paradigm shift
- **SC-010**: Student can cite 3+ statistics about AI agent adoption/impact with sources
- **SC-011**: Student expresses readiness for Chapter 34 (self-assessment or exit survey)
- **SC-012**: All lessons pass anti-convergence checklist (no meta-commentary, framework invisible)

---

## Constraints

- **No code implementations** — conceptual foundation only
- **No deep SDK dives** — brief overview, detailed coverage in Chapters 34-36
- **Layer 1 primary** — students need manual understanding before AI collaboration
- **Framework invisible** — Three Roles demonstrated through action, not exposition
- **Statistics cited** — all numbers must have sources
- **Production relevance** — examples must connect to real professional contexts

---

## Non-Goals

- Teaching how to build agents (that's Chapters 34+)
- Deep comparison of all agent frameworks (dedicated chapters exist)
- MCP fundamentals (Chapter 37)
- Multi-agent orchestration patterns (Chapter 45)
- Historical evolution of AI (only current landscape)

---

## Assumptions

- Students have completed Parts 1-5 (AIDD mindset, Claude Code proficiency, prompt engineering, SDD-RI, Python fundamentals)
- Students are familiar with Claude Code as an "agentic" tool (even if they don't use that terminology)
- Students have career interest in AI-native development
- Internet access for "Try With AI" sections

---

## References

### Primary Authoritative Source

- **Google/Kaggle Whitepaper**: "Introduction to Agents" (November 2025)
  - **Authors**: Alan Blount, Antonio Gulli, Shubham Saboo, Michael Zimmermann, Vladimir Vuskovic
  - **Content Contributors**: Enrique Chan, Mike Clark, Derek Egan, Anant Nawalgaria, Kanchana Patlolla, Julia Wiesinger
  - **URL**: https://www.kaggle.com/whitepaper-introduction-to-agents
  - **Cited as**: "Introduction to Agents, Google/Kaggle, November 2025"
  - **Local copy**: `specs/038-chapter-33-intro-ai-agents/Introduction to Agents.pdf`

### Supporting Sources

- **Related Whitepapers** (cited in primary source):
  - "Agents" by Wiesinger, Marlow, et al. (2024) — https://www.kaggle.com/whitepaper-agents
  - "Agents Companion" by Gulli, Nigam, et al. (2025) — https://www.kaggle.com/whitepaper-agent-companion

- **Academic Papers** (cited in primary source):
  - ReAct: Yao et al. (2022) — https://arxiv.org/abs/2210.03629
  - Chain-of-Thought: Wei et al. (2023) — https://arxiv.org/pdf/2201.11903.pdf
  - τ-bench: Yao et al. (2024) — https://arxiv.org/abs/2406.12045

- **McKinsey Research**: "Agents, robots, and us: Skill partnerships in the age of AI"
  - URL: https://www.mckinsey.com/mgi/our-research/agents-robots-and-us-skill-partnerships-in-the-age-of-ai

### Project References

- **Chapter Index**: `specs/book/chapter-index.md`
- **Constitution**: `.specify/memory/constitution.md` (v6.0.1)

---

## Suggested Lesson Structure

Based on user stories and paper alignment, suggested **8-lesson structure**:

| Lesson | Title | Paper Content | User Story | Layer |
|--------|-------|---------------|------------|-------|
| 1 | What Is an AI Agent? | Definition, 5-Level Taxonomy, "Director vs Bricklayer" paradigm | US1 | L1 Manual |
| 2 | Core Agent Architecture | 3+1 Architecture (Model, Tools, Orchestration, Deployment) | US2 | L1 Manual |
| 3 | The Agentic Problem-Solving Process | 5-Step Loop, Customer Support example walkthrough | US2 | L1 → L2 |
| 4 | Multi-Agent Design Patterns | Coordinator, Sequential, Iterative Refinement, HITL | US3 | L1 → L2 |
| 5 | Agent Ops: Operating Agents in Production | LM-as-Judge, metrics, traces, human feedback | US5 | L2 |
| 6 | Agent Interoperability & Security | A2A, Agent Cards, agent identity, trust trade-off | US6 | L2 |
| 7 | The Agent SDK Landscape | OpenAI, Google ADK, Anthropic, LangChain comparison | US4 | L2 |
| 8 | Your First Agent Concept | Specification design capstone, transition to Chapter 34 | US7, US8 | L2 → L3 |

**Lesson Justification** (from paper structure):
- Lessons 1-3: Foundation (Part 1 of paper: definition, architecture, process)
- Lessons 4-6: Operations (Part 2 of paper: patterns, Agent Ops, interoperability)
- Lesson 7: Frameworks (SDK comparison, informed by paper's guidance)
- Lesson 8: Synthesis (Apply learning, prepare for hands-on)

Each lesson ends with "Try With AI" section using action prompts.
