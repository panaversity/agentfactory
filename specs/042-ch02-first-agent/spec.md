# Feature Specification: Chapter 2 - Your First Agent

**Feature Branch**: `042-ch02-first-agent`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Chapter 2: Your First Agent - Create working AI agent using OpenAI Agents SDK with reference to learn-agentic-ai repository examples"

## Overview

Chapter 2 is the first coding chapter of the AI-Native Software Development course. Students transition from conceptual understanding (Chapter 1) to hands-on agent development. The chapter focuses on creating a minimal working agent using the OpenAI Agents SDK, understanding the Agent class, using Runner.run_sync(), and basic debugging techniques.

**Pedagogical Layer**: Layer 1 (Manual Foundation) - Students learn syntax and concepts hands-on before AI collaboration.

**Prerequisites**: Chapter 1 completion (API fundamentals, LLM vs Agent concepts, environment setup with UV and OpenAI API key)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understand Agent Fundamentals (Priority: P1)

A student who completed Chapter 1 wants to understand what makes up an agent before writing code. They need to grasp the core components (name, instructions, model) that define an agent's identity and behavior.

**Why this priority**: Foundation for all subsequent coding. Without understanding the components, code becomes copy-paste without comprehension.

**Independent Test**: Student can explain in their own words what an Agent is and identify the three core properties (name, instructions, model) when shown agent code.

**Acceptance Scenarios**:

1. **Given** a student has completed Chapter 1, **When** they read Lesson 2.1, **Then** they can list the three core properties of an Agent and explain each in one sentence.
2. **Given** a student sees example agent code, **When** asked to identify the agent's "personality", **Then** they correctly point to the instructions parameter.
3. **Given** a student has never coded an agent, **When** they complete the 🤖 DocuBot Project section, **Then** they produce a written specification (not code) for DocuBot's name, instructions, and model choice.

---

### User Story 2 - Create First Working Agent (Priority: P1)

A student wants to write and run their first working agent. They should be able to create a file, write 4 lines of code, and see an AI response in their terminal.

**Why this priority**: Core deliverable of the chapter. This is the "hello world" moment for agent development.

**Independent Test**: Student runs `python docubot_v1.py` and sees a meaningful AI response printed to terminal.

**Acceptance Scenarios**:

1. **Given** a student has their environment set up from Chapter 1, **When** they follow Lesson 2.2 hints, **Then** they create docubot_v1.py with working code.
2. **Given** a student writes agent code with `Agent(name='DocuBot', instructions='...')`, **When** they run it with `Runner.run_sync()`, **Then** they receive a text response from the AI.
3. **Given** a student completes the minimal agent, **When** they ask "What can you help me with?", **Then** DocuBot responds based on its instructions.

---

### User Story 3 - Understand Agent Execution Flow (Priority: P2)

A student wants to understand what happens behind the scenes when an agent runs. They should visualize the agent loop and understand synchronous vs asynchronous execution.

**Why this priority**: Debugging requires understanding execution flow. Without this, students can't troubleshoot issues.

**Independent Test**: Student adds print statements showing execution flow and explains what Runner.run_sync() does.

**Acceptance Scenarios**:

1. **Given** a student has a working agent, **When** they add print('Starting DocuBot...') before and after Runner.run_sync(), **Then** they observe the synchronous blocking behavior.
2. **Given** a student runs modified code with debug prints, **When** they execute it, **Then** they can explain in their own words the sequence: input → LLM call → response → output.
3. **Given** a student has seen the agent loop concept, **When** they print `type(result)`, **Then** they understand the result object structure.

---

### User Story 4 - Debug Common Agent Errors (Priority: P2)

A student wants to learn how to handle errors when things go wrong. They should experience common errors and know how to fix them.

**Why this priority**: Essential skill for independent learning. Students must be able to self-diagnose issues.

**Independent Test**: Student can intentionally break code in 3 ways, recognize each error message, and fix it.

**Acceptance Scenarios**:

1. **Given** a student has working agent code, **When** they change `from agents import Agent` to `from agent import Agent`, **Then** they see ModuleNotFoundError and know to check the import.
2. **Given** a student removes their API key, **When** they run the agent, **Then** they see AuthenticationError and know to check .env file.
3. **Given** a student introduces a typo in code, **When** they run it, **Then** they can read the traceback and identify the line number with the error.

---

### Edge Cases

- What happens when instructions are empty or very short?
- How does the agent respond when asked something outside its instructions?
- What happens if the API key has no credits remaining?
- How long should students wait before assuming the request timed out?
- What if the OpenAI Agents SDK package is not installed?

## Requirements *(mandatory)*

### Functional Requirements

**Content Structure Requirements (CSC)**

- **CSC-001**: Chapter MUST contain exactly 4 lessons matching the course specification structure
- **CSC-002**: Each lesson MUST have a "🤖 Apply to DocuBot Project" section with Task (numbered steps) and Outcome (italicized description)
- **CSC-003**: Each lesson MUST have a "💡 Hints" section with 5 progressive hints in collapsible `<details>` tags
- **CSC-004**: Each lesson MUST have a simple analogy in a callout box (:::tip format)
- **CSC-005**: Chapter MUST end with a quiz containing 10 questions covering all 4 lessons
- **CSC-006**: Lessons MUST follow the sequence: 2.1 Agent Fundamentals, 2.2 First Agent in Python, 2.3 The Agent Loop, 2.4 Debugging Agents

**Pedagogical Constraints (PC)**

- **PC-001**: Chapter MUST be completable in 3-4 hours total
- **PC-002**: Each lesson MUST be 30-50 minutes duration
- **PC-003**: Cognitive load MUST stay MODERATE (first code chapter, small complete examples, one concept per lesson)
- **PC-004**: All code examples MUST use the pattern from reference: `from agents import Agent, Runner` (not `openai.agents`)
- **PC-005**: Code MUST use synchronous `Runner.run_sync()` for simplicity (async comes in Chapter 5)
- **PC-006**: Model MUST be `gpt-4o-mini` for cost-effectiveness during learning

**Technical Constraints (TC)**

- **TC-001**: All code examples MUST be runnable with the OpenAI Agents SDK (`openai-agents` package)
- **TC-002**: Code MUST follow the pattern: create Agent → run with Runner → access result.final_output
- **TC-003**: Lessons MUST NOT introduce tools, handoffs, or guardrails (those are Chapter 3+)
- **TC-004**: Agent instructions MUST be strings (no dynamic instructions in this chapter)
- **TC-005**: All starter code MUST include necessary imports and be copy-paste runnable
- **TC-006**: Installation instruction MUST be `uv add openai-agents` (consistent with UV from Chapter 1)

**DocuBot Project Requirements (DBP)**

- **DBP-001**: Lesson 2.1 outcome MUST be a written specification (not code) for DocuBot
- **DBP-002**: Lesson 2.2 outcome MUST be a working file `docubot_v1.py` that runs and responds
- **DBP-003**: Lesson 2.3 outcome MUST show execution flow understanding through added print statements
- **DBP-004**: Lesson 2.4 outcome MUST be a troubleshooting notes document with 3 common errors and fixes

### Key Entities

- **Agent**: An AI entity with name (identity), instructions (behavior), and model (AI brain)
- **Runner**: The execution engine that runs agents and manages the agent loop
- **Result**: The object returned by Runner containing final_output and conversation items
- **Instructions**: A string defining how the agent should behave and respond

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of students with working Chapter 1 environment can run `docubot_v1.py` within 10 minutes of starting Lesson 2.2
- **SC-002**: Students can create a new agent from scratch (without hints) with name, instructions, and runner within 5 minutes after completing the chapter
- **SC-003**: Students correctly identify at least 3 error types and their fixes after completing Lesson 2.4
- **SC-004**: Chapter completion rate of 90%+ (students who start finish all 4 lessons)
- **SC-005**: Quiz pass rate of 80%+ on first attempt (indicates concept retention)
- **SC-006**: Students produce 4 artifacts: docubot spec, docubot_v1.py, execution flow notes, troubleshooting notes

## Constraints *(mandatory)*

### Scope Boundaries

**In Scope**:
- Agent class basics (name, instructions, model)
- Runner.run_sync() synchronous execution
- result.final_output access
- Print-based debugging
- Common error identification

**Out of Scope (Covered in Later Chapters)**:
- Tools and function_tool decorator (Chapter 3)
- Model configuration and temperature (Chapter 3)
- Sessions and memory (Chapter 4)
- Streaming responses (Chapter 5)
- Multi-agent handoffs (Chapter 6)
- Guardrails (Chapter 7)
- Structured output (Chapter 8)
- Tracing (Chapter 9)

### Dependencies

- Chapter 1 completion (environment setup, API key verification)
- OpenAI API key with active credits
- UV package manager installed
- Python 3.11+ installed

## Non-Goals *(mandatory)*

- Teaching advanced agent configuration
- Covering async/await patterns (save for Chapter 5)
- Building production-ready agents
- Explaining the internals of the OpenAI Agents SDK
- Teaching multiple model providers (Gemini integration shown in reference but not taught yet)

## Assumptions

- Students have successfully completed Chapter 1 and verified their API key works
- Students have basic Python syntax familiarity (from Parts 1-4 of the book)
- The OpenAI Agents SDK API remains stable at `from agents import Agent, Runner`
- Students are comfortable with terminal/command line from Chapter 1
- `gpt-4o-mini` model remains available and affordable for learning

## Reference Materials

### Source Course Specification
- Location: `context/AI-Native-Course-Spec-v2 (2).md`
- Chapter 2 section: Lines 237-457

### Reference Code Repository
- Main example: https://github.com/panaversity/learn-agentic-ai/tree/main/01_ai_agents_first/04_hello_agent
- Code file: https://github.com/panaversity/learn-agentic-ai/blob/main/01_ai_agents_first/04_hello_agent/hello_agent/main.py

### Official Documentation
- OpenAI Agents SDK: Context7 library ID `/openai/openai-agents-python`
- Key patterns: Agent creation, Runner.run_sync(), result.final_output

## Lesson Structure Summary

| Lesson | Title | Duration | Code Level | Key Outcome |
|--------|-------|----------|------------|-------------|
| 2.1 | Agent Fundamentals | 35-45 min | Reading Code | Written DocuBot specification |
| 2.2 | First Agent in Python | 40-50 min | Writing Code | Working docubot_v1.py |
| 2.3 | The Agent Loop | 35-45 min | Reading & Modifying | Execution flow understanding |
| 2.4 | Debugging Agents | 30-40 min | Writing & Debugging | Troubleshooting notes |

## DocuBot State After Chapter 2

*A simple agent named DocuBot that responds to questions. It can't search documents yet (that comes later), but it works and responds intelligently based on its instructions. Students have a working `docubot_v1.py` file.*
