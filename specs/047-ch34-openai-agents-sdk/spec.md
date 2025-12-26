# Feature Specification: Chapter 34 - OpenAI Agents SDK

**Feature Branch**: `047-ch34-openai-agents-sdk`
**Created**: 2025-12-26
**Status**: Draft
**Proficiency Level**: B1-B2 (Intermediate, building on Chapter 33 foundations)
**Input**: User description: "Chapter 34: OpenAI Agents SDK - Build production agents with OpenAI's official Python SDK. Covers: 1) SDK setup and first agent, 2) Function tools deep dive, 3) Multi-agent handoffs, 4) Guardrails and safety, 5) Streaming and async patterns, 6) Tracing and observability, 7) Capstone: Customer support system. Must include LiteLLM integration for free/alternative model usage in at least one lesson. Uses building-with-openai-agents skill as knowledge source. Running example: TaskManager agent from Part 6."

## Assumed Knowledge

**What students know BEFORE this chapter**:
- 5-Level Agent Taxonomy (Chapter 33, Lesson 1): Can classify systems from Level 0-4
- 3+1 Core Architecture (Chapter 33, Lesson 2): Model, Tools, Orchestration, Deployment
- 5-Step Operational Loop (Chapter 33, Lesson 3): Get Mission → Scan → Think → Act → Observe
- Multi-agent design patterns (Chapter 33, Lesson 4): Coordinator, Sequential, Iterative, Human-in-the-Loop
- Agent Ops fundamentals (Chapter 33, Lesson 5): Evaluation, tracing, golden datasets
- Python async/await, type hints, Pydantic models (Part 5)
- Specification-driven development workflow (Part 4)
- Director vs Bricklayer paradigm shift (Chapter 33, Lesson 1)

**What this chapter must explain from scratch**:
- OpenAI Agents SDK primitives: Agent, Runner, function_tool decorator
- How handoffs differ from calling agents as tools
- Guardrail decorators and tripwire patterns
- Session backends for conversation memory
- LiteLLM integration for alternative model providers
- MCP (Model Context Protocol) integration with agents
- Tracing processors and custom spans

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Create First Working Agent (Priority: P1)

A domain expert who completed Chapter 33 wants to create their first working agent using the OpenAI Agents SDK. They need to understand the minimal setup required, create an agent with instructions, and see it respond to a prompt.

**Why this priority**: Foundation for all subsequent lessons. Cannot proceed with tools, handoffs, or advanced patterns without understanding basic agent creation and execution.

**Independent Test**: Student can create a Python script with a basic agent, run it with `python agent.py`, and receive a coherent response. Demonstrates SDK installation, API key configuration, and core primitives work.

**Acceptance Scenarios**:

1. **Given** a student with Python and pip installed, **When** they run `pip install openai-agents` and set `OPENAI_API_KEY`, **Then** they can import from `agents` without errors
2. **Given** an installed SDK, **When** student creates an Agent with name and instructions, **Then** Runner.run_sync() executes successfully and returns a response
3. **Given** working agent code, **When** student modifies the instructions, **Then** agent behavior changes accordingly

---

### User Story 2 - Build TaskManager Agent with Tools (Priority: P1)

A domain expert wants to create an agent that can actually do something useful—add tasks, list tasks, and complete tasks—using function tools. This connects to the running TaskManager example from Part 6.

**Why this priority**: Tools are the core mechanism for agent utility. Without tools, agents can only converse. This establishes the pattern used throughout the chapter.

**Independent Test**: Student creates TaskManager agent with three function tools, interacts with it via prompts, and sees tasks being created, listed, and completed. Data persists across multiple tool invocations within a session.

**Acceptance Scenarios**:

1. **Given** the @function_tool decorator, **When** student decorates a Python function with type hints and docstring, **Then** the SDK automatically generates the tool schema
2. **Given** an agent with tools, **When** user asks to "add a task to buy groceries", **Then** agent calls add_task tool and confirms task creation
3. **Given** an agent with multiple tools, **When** user asks to "add a task and then list all tasks", **Then** agent chains tool calls in sequence

---

### User Story 3 - Run Agents with Free/Alternative Models (Priority: P2)

A domain expert without OpenAI API credits wants to learn the SDK using free or alternative model providers through LiteLLM integration.

**Why this priority**: Removes cost barrier to learning. Enables students to experiment extensively without financial constraints. Required by user specification.

**Independent Test**: Student configures agent with LitellmModel pointing to an alternative provider (e.g., Anthropic, Google, or local model), runs the TaskManager example, and receives valid responses.

**Acceptance Scenarios**:

1. **Given** LiteLLM installed (`pip install "openai-agents[litellm]"`), **When** student creates agent with `LitellmModel(model="anthropic/claude-3-5-sonnet")`, **Then** agent executes using Anthropic's API
2. **Given** tracing disabled (required for non-OpenAI), **When** student runs agent with alternative model, **Then** no tracing errors occur
3. **Given** alternative model configuration, **When** student uses identical TaskManager code, **Then** tools work identically across providers

---

### User Story 4 - Implement Multi-Agent Handoffs (Priority: P2)

A domain expert wants to build a system where specialized agents hand off to each other based on user intent—implementing the Coordinator pattern from Chapter 33 using handoffs.

**Why this priority**: Multi-agent systems are the production pattern. Handoffs distinguish sophisticated agent systems from simple single-agent tools.

**Independent Test**: Student creates a triage agent and two specialist agents. When user asks a billing question, triage hands off to billing agent. When user asks a technical question, triage hands off to technical agent. Each specialist maintains appropriate context.

**Acceptance Scenarios**:

1. **Given** a triage agent with handoffs to specialists, **When** user intent matches specialist domain, **Then** handoff occurs and specialist responds
2. **Given** a handoff with input_type, **When** handoff executes, **Then** structured data passes to receiving agent
3. **Given** conversation history before handoff, **When** specialist receives control, **Then** specialist can reference prior context

---

### User Story 5 - Add Input/Output Guardrails (Priority: P2)

A domain expert building user-facing agents needs to validate input before processing and output before sending, implementing safety patterns from Chapter 33.

**Why this priority**: Production agents require safety mechanisms. Guardrails prevent inappropriate content and protect against prompt injection.

**Independent Test**: Student creates agent with input guardrail that blocks PII. When user submits a message containing a social security number pattern, guardrail triggers and agent refuses to process. When user submits clean input, agent responds normally.

**Acceptance Scenarios**:

1. **Given** @input_guardrail decorated function, **When** tripwire_triggered=True, **Then** InputGuardrailTripwireTriggered exception raised
2. **Given** @output_guardrail decorated function, **When** output contains blocked patterns, **Then** OutputGuardrailTripwireTriggered exception raised
3. **Given** guardrail exception caught, **When** handler executes, **Then** user receives appropriate feedback without exposing system details

---

### User Story 6 - Enable Streaming Responses (Priority: P3)

A domain expert building interactive agents wants real-time response streaming rather than waiting for complete responses.

**Why this priority**: User experience improvement. Long agent responses feel faster with streaming. Common production pattern.

**Independent Test**: Student modifies TaskManager to use Runner.run_streamed(), iterates over stream_events(), and sees tokens appear progressively rather than all at once.

**Acceptance Scenarios**:

1. **Given** Runner.run_streamed() call, **When** agent generates response, **Then** stream_events() yields events as they occur
2. **Given** streaming response, **When** iterating events, **Then** student can print tokens incrementally to console
3. **Given** streaming complete, **When** accessing result, **Then** final_output contains complete response

---

### User Story 7 - Implement Session-Based Memory (Priority: P3)

A domain expert wants agents that remember conversation history across multiple turns without manually managing context.

**Why this priority**: Production agents need memory. Sessions abstract away history management complexity.

**Independent Test**: Student creates SQLiteSession, runs multiple agent turns, and sees agent correctly reference earlier conversation without explicit context passing.

**Acceptance Scenarios**:

1. **Given** SQLiteSession("user_123"), **When** agent run includes session=session, **Then** history persists automatically
2. **Given** persisted session, **When** user asks "what did I say earlier?", **Then** agent can reference prior turns
3. **Given** file-based session, **When** process restarts, **Then** conversation history survives restart

---

### User Story 8 - Debug Agent Behavior with Tracing (Priority: P3)

A domain expert troubleshooting agent behavior needs visibility into what the agent is doing—which tools it called, what the LLM generated, and where failures occurred.

**Why this priority**: Production debugging capability. Tracing is essential for understanding and improving agent behavior.

**Independent Test**: Student runs agent with tracing enabled, views trace in OpenAI Dashboard (or custom processor), and can identify exactly which tool calls occurred and in what order.

**Acceptance Scenarios**:

1. **Given** default tracing enabled, **When** agent runs, **Then** trace appears in OpenAI Dashboard
2. **Given** custom span via `with trace("my-workflow")`, **When** multiple runs execute, **Then** all runs appear under single trace
3. **Given** external tracing processor, **When** configured via add_trace_processor(), **Then** events route to external system

---

### User Story 9 - Build Customer Support System Capstone (Priority: P1)

A domain expert completing the chapter wants to build a production-style multi-agent customer support system that combines all learned patterns: triage, specialist handoffs, guardrails, memory, and tracing.

**Why this priority**: Capstone demonstrates comprehensive understanding. Synthesizes all chapter concepts into a deployable system. Required by chapter structure.

**Independent Test**: Student builds complete system with triage agent, billing specialist, technical specialist, and escalation agent. System handles realistic support scenarios, maintains conversation state, blocks inappropriate requests, and produces debuggable traces.

**Acceptance Scenarios**:

1. **Given** user describes billing issue, **When** triage processes request, **Then** handoff to billing specialist occurs with context
2. **Given** complex issue requiring escalation, **When** specialist cannot resolve, **Then** handoff to escalation agent with full history
3. **Given** abusive input, **When** guardrail detects, **Then** system responds appropriately without processing harmful content
4. **Given** complete interaction, **When** reviewed via tracing, **Then** full conversation flow is visible and debuggable

---

### Edge Cases

- What happens when OpenAI API key is invalid or expired? (Lesson 1: graceful error with clear message)
- What happens when LiteLLM model doesn't support tool use? (Lesson 1: feature compatibility notes)
- What happens when tool function raises an exception? (Lesson 2: error handling patterns)
- What happens when handoff target agent doesn't exist? (Lesson 3: configuration validation)
- What happens when guardrail itself fails? (Lesson 4: guardrail error handling)
- What happens when streaming connection drops mid-response? (Lesson 5: reconnection patterns)
- What happens when session storage is unavailable? (Lesson 7: fallback behavior)

## Requirements *(mandatory)*

### Functional Requirements

#### Lesson 1: SDK Setup & First Agent
- **FR-001**: Chapter MUST explain installation via `pip install openai-agents`
- **FR-002**: Chapter MUST cover API key configuration via environment variable
- **FR-003**: Chapter MUST introduce core primitives: Agent, Runner, instructions
- **FR-004**: Chapter MUST include LiteLLM setup for free/alternative models
- **FR-005**: Chapter MUST show disabling tracing for non-OpenAI models

#### Lesson 2: Function Tools Deep Dive
- **FR-006**: Chapter MUST explain @function_tool decorator usage
- **FR-007**: Chapter MUST cover tool parameter schemas via type hints
- **FR-008**: Chapter MUST demonstrate docstring-based tool descriptions
- **FR-009**: Chapter MUST implement TaskManager with add_task, list_tasks, complete_task tools
- **FR-010**: Chapter MUST cover tool error handling patterns (failure_error_function)

#### Lesson 3: Multi-Agent Handoffs
- **FR-011**: Chapter MUST explain handoff mechanism vs agents-as-tools
- **FR-012**: Chapter MUST demonstrate triage → specialist routing
- **FR-013**: Chapter MUST cover handoff callbacks (on_handoff)
- **FR-014**: Chapter MUST explain context preservation across handoffs
- **FR-015**: Chapter MUST compare Manager pattern vs Handoff pattern (from Ch33)

#### Lesson 4: Guardrails & Safety
- **FR-016**: Chapter MUST implement @input_guardrail decorator
- **FR-017**: Chapter MUST implement @output_guardrail decorator
- **FR-018**: Chapter MUST explain tripwire mechanism and exception handling
- **FR-019**: Chapter MUST demonstrate practical guardrail (PII detection, topic filtering)
- **FR-020**: Chapter MUST connect guardrails to Human-in-the-Loop pattern from Ch33

#### Lesson 5: Streaming & Async Patterns
- **FR-021**: Chapter MUST demonstrate Runner.run_streamed()
- **FR-022**: Chapter MUST show async iteration over stream_events()
- **FR-023**: Chapter MUST cover concurrent tool execution patterns
- **FR-024**: Chapter MUST explain RunResult vs RunResultStreaming

#### Lesson 6: Tracing & Observability
- **FR-025**: Chapter MUST explain automatic tracing behavior
- **FR-026**: Chapter MUST demonstrate custom spans with trace() context manager
- **FR-027**: Chapter MUST cover external integrations (Logfire, AgentOps mentioned)
- **FR-028**: Chapter MUST connect to Agent Ops concepts from Ch33 Lesson 5

#### Lesson 7: Capstone - Customer Support System
- **FR-029**: Chapter MUST implement specification-driven development workflow
- **FR-030**: Chapter MUST combine all patterns: handoffs, tools, guardrails, sessions, tracing
- **FR-031**: Chapter MUST produce deployable, testable system
- **FR-032**: Chapter MUST include at least 3 specialized agents (triage, billing, technical)

#### Cross-Cutting Requirements
- **FR-033**: All lessons MUST use TaskManager as running example where applicable
- **FR-034**: At least one lesson MUST include working LiteLLM alternative model example
- **FR-035**: All code examples MUST be complete and runnable
- **FR-036**: Chapter MUST reference building-with-openai-agents skill patterns

### Key Entities

- **Agent**: Core primitive with name, instructions, tools, handoffs, model, guardrails
- **Tool**: Python function decorated with @function_tool, auto-generates schema
- **Handoff**: Transfer of control between agents with optional data passing
- **Guardrail**: Validation function that can trigger tripwire to block execution
- **Session**: Persistent conversation history (SQLiteSession, SQLAlchemySession)
- **Runner**: Execution engine that runs agent loop until completion
- **Trace**: Observability record of agent execution for debugging

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can create and run a basic agent within 10 minutes of starting Lesson 1
- **SC-002**: TaskManager example works with both OpenAI API and LiteLLM alternative provider
- **SC-003**: Students can implement a 3-agent handoff system by end of Lesson 3
- **SC-004**: Guardrail examples successfully block at least 2 types of inappropriate input
- **SC-005**: Capstone system handles realistic support scenario end-to-end with all patterns integrated
- **SC-006**: 90% of code examples execute without modification (copy-paste ready)
- **SC-007**: Students report confidence in building custom agents after chapter completion (qualitative)
- **SC-008**: Clear conceptual bridge: every SDK pattern maps to a Chapter 33 concept

### Learning Objectives Alignment

| Lesson | Bloom's Level | Primary Outcome |
|--------|---------------|-----------------|
| 1. Setup & First Agent | Apply (L3) | Configure SDK, create agent, run successfully |
| 2. Function Tools | Apply (L3) | Implement TaskManager with working tools |
| 3. Multi-Agent Handoffs | Analyze (L4) | Design handoff topology, compare patterns |
| 4. Guardrails & Safety | Apply (L3) | Implement input/output validation |
| 5. Streaming & Async | Apply (L3) | Enable streaming, handle async patterns |
| 6. Tracing & Observability | Understand (L2) | Interpret traces, configure processors |
| 7. Capstone | Create (L6) | Build complete multi-agent system |

## Assumptions

1. **API Access**: Students have or can obtain OpenAI API key, OR will use LiteLLM with alternative provider
2. **Python Environment**: Students have Python 3.9+ with pip/venv capability (from Part 5)
3. **Async Familiarity**: Students understand Python async/await basics (from Part 5)
4. **Chapter 33 Completion**: Students completed Introduction to AI Agents before this chapter
5. **LiteLLM Providers**: At least one of: Anthropic, Google, or local Ollama available for free tier examples
6. **Running Example**: TaskManager from Part 6 is familiar context for students

## Dependencies

- **Chapter 33**: Provides conceptual foundation (taxonomy, architecture, patterns)
- **Part 5**: Python fundamentals including async, type hints, Pydantic
- **Part 4**: SDD-RI workflow for capstone specification approach
- **Skill**: building-with-openai-agents skill provides code patterns and API reference

## Out of Scope

- Deployment to production infrastructure (covered in Part 7)
- Voice/realtime agents (specialized extension, not core SDK)
- OpenAI Responses API details (SDK abstracts this)
- Custom model training or fine-tuning
- Enterprise authentication/authorization patterns
