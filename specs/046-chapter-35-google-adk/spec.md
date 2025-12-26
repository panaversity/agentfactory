# Feature Specification: Chapter 35 — Building Reliable Agents with Google ADK

**Feature Branch**: `046-chapter-35-google-adk`
**Created**: 2025-12-26
**Status**: Draft
**Input**: "Build custom AI agents that really matter - reliable, safe, and evaluated on real tasks"

---

## Executive Summary

**Chapter 35 teaches the discipline of building agents that work in production** — not just agents that demo well.

Google's Agent Development Kit (ADK) is the same framework powering Agentspace and Google Customer Engagement Suite. It's designed from the ground up for **reliability**, not retrofitted for it. This chapter teaches students to leverage ADK's architecture to build agents that are:

1. **Testable** — Evaluation-first development with `adk eval`
2. **Predictable** — Workflow agents (Sequential, Parallel, Loop) for deterministic orchestration
3. **Safe** — Guardrails and callbacks as core architecture, not afterthoughts

**Core Teaching Philosophy**:

> "The difference between a demo agent and a production agent is not features — it's reliability engineering."

Students won't just learn ADK APIs. They'll learn **why ADK makes certain architectural choices** and how those choices enable agents that enterprises trust with real work.

**Proficiency Level**: B1 (Intermediate)

---

## The Three Pillars of Reliable Agents

This chapter is organized around three pillars that distinguish production agents from demos:

### Pillar 1: Testable Agents (Evaluation-First Development)

**The Problem**: Most agent tutorials treat testing as an afterthought. You build the agent, then maybe write some tests.

**ADK's Solution**: `adk eval` is a first-class citizen. You define expected behaviors in JSON, run systematic evaluations, and integrate with CI/CD. **Evaluation drives development**, not the other way around.

**What Students Learn**:
- Write eval cases BEFORE implementing agent behavior
- Test not just final responses, but tool execution paths
- Use evaluation failures to guide agent improvement
- Integrate `adk eval` with pytest for automated testing

### Pillar 2: Predictable Agents (Workflow Orchestration)

**The Problem**: LLM-based routing is flexible but unpredictable. Sometimes you need deterministic behavior.

**ADK's Solution**: Workflow agents (SequentialAgent, ParallelAgent, LoopAgent) provide **predictable, testable pipelines**. You choose LLM routing when flexibility matters, workflow agents when predictability matters.

**What Students Learn**:
- SequentialAgent for researcher → writer → editor pipelines
- ParallelAgent for concurrent analysis (fact-check AND sentiment simultaneously)
- LoopAgent for iterative refinement until quality criteria met
- **When to use workflow agents vs. LLM routing** — the critical design decision

### Pillar 3: Safe Agents (Guardrails as Architecture)

**The Problem**: Safety is often bolted on after the agent is built. This leads to gaps and inconsistencies.

**ADK's Solution**: Callbacks (`before_model_callback`, `before_tool_callback`) are **architectural interception points**. You design safety into the agent from day one.

**What Students Learn**:
- Input validation before LLM sees the message
- Tool execution restriction for dangerous operations
- Layered defense patterns (input → model → tool → output)
- **Safety as a design constraint**, not a feature

---

## Assumed Knowledge

**What students know BEFORE this chapter**:

From **Chapter 33 (Introduction to AI Agents)**:
- 5-Level Agent Taxonomy (Level 0-4)
- 3+1 Core Architecture (Model + Tools + Orchestration + Deployment)
- Multi-Agent Design Patterns (Coordinator, Sequential, Iterative Refinement)
- **Agent Ops fundamentals** — evaluation, tracing, observability

From **Chapter 34 (OpenAI Agents SDK)**:
- Agent + tools + handoffs pattern
- Tool execution lifecycle
- Basic guardrails and callbacks
- **The experience of building an agent that works in demos**

From **Part 5 (Python Fundamentals)**:
- async/await patterns
- Type hints and docstrings
- JSON parsing

**What this chapter teaches from scratch**:

- **The reliability mindset** — why production agents fail and how to prevent it
- **Evaluation-first development** — write tests before agents
- **Workflow agents** — SequentialAgent, ParallelAgent, LoopAgent
- **ADK callback architecture** — designing safety into agents
- **Production deployment** — Vertex AI Agent Engine (not just "run locally")

---

## User Scenarios & Testing

### User Story 1 — Evaluation-First Agent Development (Priority: P1)

A developer wants to build a TaskManager agent that **they can trust**. They've built demo agents before, but those agents fail in unpredictable ways. They want to start with tests, not end with tests.

**Why this priority**: This is the fundamental mindset shift. Everything else builds on evaluation-first thinking.

**Independent Test**: Student writes eval cases for TaskManager BEFORE implementing the agent, then iteratively improves the agent until all tests pass.

**Acceptance Scenarios**:

1. **Given** TaskManager requirements (add, list, complete, delete tasks), **When** student writes JSON eval cases defining expected tool calls and responses, **Then** they have a test suite BEFORE writing any agent code.

2. **Given** eval cases exist, **When** student creates a basic TaskManager agent and runs `adk eval`, **Then** they see which behaviors pass and which fail.

3. **Given** failing eval cases, **When** student improves agent instructions or tool definitions, **Then** test pass rate increases (measurable progress).

4. **Given** all eval cases pass, **When** student adds a new feature, **Then** they write new eval cases FIRST, then implement.

---

### User Story 2 — Predictable Multi-Agent Pipelines (Priority: P1)

A developer wants to build a content creation system: researcher → writer → editor. They need **guaranteed execution order**, not "LLM decides which agent to call."

**Why this priority**: Workflow agents are ADK's killer feature for reliability. This is what separates ADK from "just another agent framework."

**Independent Test**: Student builds a SequentialAgent pipeline and observes deterministic execution with predictable data flow between agents.

**Acceptance Scenarios**:

1. **Given** a content creation task, **When** student creates SequentialAgent with [researcher, writer, editor] sub-agents, **Then** execution follows exact order: researcher → writer → editor.

2. **Given** a SequentialAgent pipeline, **When** student observes execution in `adk web`, **Then** they see each sub-agent execute in order, with clear handoffs.

3. **Given** understanding of workflow agents, **When** comparing to OpenAI SDK handoffs, **Then** student articulates: "Workflow agents are predictable and testable. Handoffs are flexible but harder to test."

4. **Given** parallel analysis requirements (fact-check AND sentiment), **When** student creates ParallelAgent, **Then** both analyses run concurrently and results merge predictably.

---

### User Story 3 — Iterative Quality Improvement (Priority: P1)

A developer wants an agent that refines its work until quality criteria are met — not just "generate once and hope."

**Why this priority**: LoopAgent enables the iterative refinement pattern that's critical for production quality.

**Independent Test**: Student builds a LoopAgent that iteratively improves output until evaluation passes or max iterations reached.

**Acceptance Scenarios**:

1. **Given** a quality improvement task, **When** student creates LoopAgent with solver sub-agent and `exit_loop` tool, **Then** agent iterates until solver calls `exit_loop` (solution found).

2. **Given** LoopAgent configuration, **When** student sets `max_iterations=5`, **Then** loop terminates after 5 iterations even if solution not found (prevent infinite loops).

3. **Given** iterative workflow, **When** student adds evaluation criteria (e.g., "fact-checker must pass"), **Then** LoopAgent continues until criteria met.

4. **Given** completed LoopAgent implementation, **When** student writes eval cases, **Then** they can test: "Given input X, agent should converge in ≤3 iterations."

---

### User Story 4 — Safety by Design (Priority: P1)

A developer wants to build an agent that **cannot** perform dangerous actions — not "hopefully won't" but "architecturally prevented."

**Why this priority**: Safety isn't a feature, it's a constraint. Students must learn to design agents with safety baked in.

**Independent Test**: Student implements callbacks that prevent specific dangerous behaviors, and demonstrates the agent CANNOT bypass them.

**Acceptance Scenarios**:

1. **Given** a TaskManager agent, **When** student implements `before_tool_callback` that blocks `delete_task` for tasks with "PROTECTED" prefix, **Then** agent CANNOT delete protected tasks (returns error instead of executing).

2. **Given** `before_model_callback` that blocks inputs containing "UNSAFE", **When** user sends message with "UNSAFE", **Then** agent returns predefined response WITHOUT calling the LLM.

3. **Given** layered callbacks (input + tool), **When** attack attempts bypass one layer, **Then** other layers still block dangerous behavior.

4. **Given** callback implementations, **When** student writes eval cases for safety behaviors, **Then** safety is testable, not just assumed.

---

### User Story 5 — Production State Management (Priority: P2)

A developer wants agent conversations to persist across restarts — not just "works in a single session."

**Why this priority**: Production agents need production state. SessionService abstraction makes this pluggable.

**Independent Test**: Student switches from InMemorySessionService to FirestoreSessionService and observes conversation persistence across agent restarts.

**Acceptance Scenarios**:

1. **Given** InMemorySessionService, **When** student restarts agent, **Then** conversation history is lost (expected for dev).

2. **Given** FirestoreSessionService, **When** student restarts agent with same session_id, **Then** conversation history is preserved.

3. **Given** ToolContext access, **When** student implements stateful tool that stores user preferences, **Then** preferences persist via session state.

4. **Given** multiple SessionService implementations, **When** student evaluates options, **Then** they can choose: InMemory (dev) → Firestore (production) → VertexAI (enterprise managed).

---

### User Story 6 — MCP Integration for Tool Ecosystems (Priority: P2)

A developer wants to use existing MCP servers (filesystem, GitHub, databases) with ADK agents — not rebuild everything.

**Why this priority**: Tool ecosystems matter. MCP integration shows ADK plays well with standards.

**Independent Test**: Student connects MCP server to ADK agent and observes MCP tools working alongside native ADK tools.

**Acceptance Scenarios**:

1. **Given** filesystem MCP server, **When** student creates McpToolset with StdioConnectionParams, **Then** ADK agent can use `read_file`, `write_file` MCP tools.

2. **Given** agent with both FunctionTool and McpToolset, **When** task requires both, **Then** agent orchestrates across tool sources seamlessly.

3. **Given** MCP integration, **When** student writes eval cases, **Then** they can test MCP tool calls just like native tools.

---

### User Story 7 — Enterprise Deployment (Priority: P2)

A developer wants to deploy their agent to production — not just "run locally forever."

**Why this priority**: Deployment is where reliability meets reality. Vertex AI Agent Engine is enterprise-grade.

**Independent Test**: Student deploys TaskManager to Vertex AI Agent Engine and invokes it remotely with same behavior as local.

**Acceptance Scenarios**:

1. **Given** tested local agent, **When** student runs `adk deploy agent_engine`, **Then** agent deploys to Vertex AI and receives production endpoint.

2. **Given** deployed agent, **When** student invokes via SDK, **Then** behavior matches local testing (eval cases still pass).

3. **Given** production deployment, **When** student configures VertexAiSessionService, **Then** sessions persist in Google's managed infrastructure.

4. **Given** deployment options, **When** evaluating Agent Engine vs Cloud Run, **Then** student chooses: Agent Engine (managed, integrated) vs Cloud Run (more control).

---

### User Story 8 — Framework-Agnostic Thinking (Priority: P3)

A developer completing this chapter wants to understand **principles**, not just ADK APIs — so they can evaluate future frameworks.

**Why this priority**: Meta-learning. Students should think "what makes agents reliable?" not "what does ADK do?"

**Acceptance Scenarios**:

1. **Given** ADK and OpenAI SDK experience, **When** asked "what makes an agent reliable?", **Then** student answers: testability, predictability, safety — not framework features.

2. **Given** new framework (Anthropic, LangGraph), **When** evaluating it, **Then** student asks: "How do I test agents? How do I make them predictable? How do I enforce safety?"

3. **Given** agent design problem, **When** choosing framework, **Then** student matches framework strengths to reliability requirements.

---

### Edge Cases

- **Eval case design mistakes**: How to write eval cases that actually test behavior, not just outputs?
- **Workflow agent debugging**: How to debug when SequentialAgent sub-agent fails mid-pipeline?
- **Callback ordering**: What happens when multiple callbacks interact? What's the execution order?
- **Session migration**: How to handle state when switching SessionService implementations?
- **Deployment verification**: How to verify deployed agent matches local behavior?

---

## Requirements

### Functional Requirements

**Pillar 1: Testable Agents**

- **FR-001**: Chapter MUST teach writing eval cases BEFORE implementing agent behavior (evaluation-first)
- **FR-002**: Chapter MUST demonstrate JSON eval file format with expected tool calls (`intermediate_data.tool_uses`)
- **FR-003**: Chapter MUST show `adk eval` CLI with `--print_detailed_results` for debugging failures
- **FR-004**: Chapter MUST teach pytest integration with `AgentEvaluator.evaluate()` for CI/CD
- **FR-005**: Chapter MUST demonstrate iterative agent improvement driven by failing eval cases

**Pillar 2: Predictable Agents**

- **FR-006**: Chapter MUST teach SequentialAgent for deterministic pipeline execution
- **FR-007**: Chapter MUST teach ParallelAgent for concurrent execution with merged results
- **FR-008**: Chapter MUST teach LoopAgent for iterative refinement with `exit_loop` and `max_iterations`
- **FR-009**: Chapter MUST explicitly compare workflow agents (predictable) vs LLM handoffs (flexible)
- **FR-010**: Chapter MUST demonstrate testing workflow agents with eval cases

**Pillar 3: Safe Agents**

- **FR-011**: Chapter MUST teach `before_model_callback` for input validation/blocking
- **FR-012**: Chapter MUST teach `before_tool_callback` for tool execution restriction
- **FR-013**: Chapter MUST demonstrate callback return patterns: `None` (allow), `LlmResponse`/`dict` (override)
- **FR-014**: Chapter MUST teach layered defense patterns (input + model + tool layers)
- **FR-015**: Chapter MUST demonstrate testing safety behaviors with eval cases

**Agent Fundamentals**

- **FR-016**: Chapter MUST teach ADK installation and environment configuration (Gemini API + Vertex AI)
- **FR-017**: Chapter MUST teach Agent creation with `name`, `model`, `instruction`, `tools`
- **FR-018**: Chapter MUST teach FunctionTool via Python functions with docstrings (auto-wrapping)
- **FR-019**: Chapter MUST teach `adk run` (CLI), `adk web` (debugging), and programmatic Runner execution

**State & Integration**

- **FR-020**: Chapter MUST demonstrate SessionService abstraction (InMemory → Firestore → VertexAI)
- **FR-021**: Chapter MUST teach ToolContext for accessing/modifying session state in tools
- **FR-022**: Chapter MUST demonstrate McpToolset for MCP server integration
- **FR-023**: Chapter MUST teach AgentTool for coordinator patterns (agent as tool)

**Deployment**

- **FR-024**: Chapter MUST teach `adk deploy agent_engine` for Vertex AI deployment
- **FR-025**: Chapter MUST demonstrate deployment verification (remote matches local behavior)
- **FR-026**: Chapter MUST explain deployment prerequisites (gcloud auth, services, IAM)

**Pedagogical**

- **FR-027**: Chapter MUST use TaskManager as running example (consistency with Chapter 34)
- **FR-028**: Each lesson MUST end with "Try With AI" action prompts
- **FR-029**: Chapter MUST follow spec-driven development workflow
- **FR-030**: **Evaluation introduced in Lesson 1, used throughout** — not relegated to end

---

## Success Criteria

### Measurable Outcomes

**Reliability Mindset**

- **SC-001**: Student can write 5+ eval cases for a feature BEFORE implementing agent code
- **SC-002**: Student can diagnose failing eval cases and improve agent to pass them
- **SC-003**: Student can integrate `adk eval` with pytest for automated testing

**Predictable Orchestration**

- **SC-004**: Student can build SequentialAgent pipeline and explain why it's more testable than LLM handoffs
- **SC-005**: Student can build LoopAgent with convergence criteria and max_iterations safety
- **SC-006**: Student can choose workflow agents vs LLM routing based on reliability requirements

**Safety Architecture**

- **SC-007**: Student can implement callbacks that prevent 2+ categories of dangerous behavior
- **SC-008**: Student can demonstrate that blocked behavior is ARCHITECTURALLY prevented, not just "hopefully avoided"
- **SC-009**: Student can write eval cases that test safety behaviors

**Production Readiness**

- **SC-010**: Student can deploy agent to Vertex AI and verify remote behavior matches local tests
- **SC-011**: Student can configure production SessionService for persistent state
- **SC-012**: Student can articulate deployment verification checklist

**Meta-Learning**

- **SC-013**: Student can answer "what makes an agent reliable?" without mentioning framework names
- **SC-014**: Given new framework, student evaluates it against testability, predictability, safety criteria

---

## Suggested Lesson Structure

| Lesson | Title | Focus | Pillar |
|--------|-------|-------|--------|
| 1 | **The Reliability Mindset** | ADK intro + write first eval case for TaskManager BEFORE agent code | Testable |
| 2 | **Evaluation-Driven Development** | Build TaskManager by making eval cases pass, `adk eval` deep dive | Testable |
| 3 | **Predictable Pipelines** | SequentialAgent, ParallelAgent — deterministic orchestration | Predictable |
| 4 | **Iterative Quality** | LoopAgent for refinement until quality criteria met | Predictable |
| 5 | **Safety as Architecture** | Callbacks as design constraint, layered defense | Safe |
| 6 | **Production State** | SessionService, ToolContext, MCP integration | Infrastructure |
| 7 | **Deployment & Verification** | Vertex AI Agent Engine, verify remote matches local | Production |
| 8 | **Capstone: Reliable TaskManager** | Multi-agent TaskManager with full reliability engineering | All Pillars |

**Key Difference from Generic Spec**: Evaluation is introduced in Lesson 1 and used throughout. Students write tests before code in every lesson.

---

## Constraints

- **Evaluation-first ALWAYS** — Every new agent behavior starts with eval cases
- **TaskManager consistency** — Same domain as Chapter 34 for comparison
- **Reliability over features** — Teach fewer features deeply, with reliability focus
- **Safety is not optional** — Callbacks are core, not advanced topic

---

## Non-Goals

- Comprehensive Vertex AI platform coverage (only agent deployment)
- All ADK features (focus on reliability pillars)
- MCP server development (Chapter 38)
- Advanced multi-agent patterns (Chapter 45)

---

## References

- **Google ADK Documentation**: https://google.github.io/adk-docs/
- **ADK Skill**: `.claude/skills/building-with-google-adk/SKILL.md`
- **Chapter 34 Spec**: OpenAI SDK comparison baseline
- **Chapter 33 Spec**: Agent conceptual foundation

---

**Specification Status**: Ready for `/sp.plan` — Focused on reliability engineering, not feature coverage.
