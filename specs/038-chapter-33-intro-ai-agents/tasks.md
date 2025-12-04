# Implementation Tasks: Chapter 33 — Introduction to AI Agents

**Feature Branch**: `038-chapter-33-intro-ai-agents`
**Created**: 2025-11-27
**Revised**: 2025-11-27 (Aligned with Google "Introduction to Agents" whitepaper)
**Status**: ✅ QA VALIDATION COMPLETE
**Source**: `spec.md` (v2.0) + `plan.md` (v2.0) — Paper-Aligned
**Primary Source**: Google/Kaggle "Introduction to Agents" whitepaper (November 2025)

---

## CRITICAL: Paper Alignment Requirement

**ALL content must align with Google's "Introduction to Agents" whitepaper (November 2025).**

**Key Frameworks (MUST teach exactly)**:
1. **5-Level Taxonomy**: Level 0 (Core Reasoning) → Level 4 (Self-Evolving System)
2. **3+1 Architecture**: Model ("Brain") + Tools ("Hands") + Orchestration ("Nervous System") + Deployment ("Body")
3. **5-Step Operational Loop**: Get Mission → Scan Scene → Think → Act → Observe
4. **Multi-Agent Patterns**: Coordinator, Sequential, Iterative Refinement, HITL
5. **Agent Ops**: LM-as-Judge, Golden Datasets, OpenTelemetry Traces, Human Feedback
6. **Agent Interoperability**: A2A Protocol, Agent Cards, Task-Oriented Architecture
7. **Agent Security**: Agent Identity as Principal, Trust Trade-Off, Defense in Depth

**⚠️ DO NOT USE old frameworks**: "Agency Spectrum", "5 Components", "ReAct pattern", "Plan-Execute pattern"

---

## Task Dependencies Overview

```
[Phase 0: Research & Preparation]
    ├── TASK-001 (Statistics validation)
    └── TASK-002 (SDK research for Lesson 7)

[Phase 1: Foundation Content - Layer 1]
    ├── TASK-003 (README.md)
    ├── TASK-004 (Lesson 1: What Is an AI Agent?)
    ├── TASK-005 (Lesson 2: Core Agent Architecture)
    └── TASK-006 (Lesson 3: Agentic Problem-Solving Process)

[Phase 2: Operations Content - Layer 2]
    ├── TASK-007 (Lesson 4: Multi-Agent Design Patterns)
    ├── TASK-008 (Lesson 5: Agent Ops)
    ├── TASK-009 (Lesson 6: Interoperability & Security)
    └── TASK-010 (Lesson 7: SDK Landscape) [depends on TASK-002]

[Phase 3: Synthesis - Layer 3]
    └── TASK-011 (Lesson 8: Your First Agent Concept)

[Phase 4: Visual Assets]
    ├── TASK-012 (Foundation diagrams)
    ├── TASK-013 (Pattern diagrams)
    ├── TASK-014 (Operations diagrams)
    └── TASK-015 (SDK comparison matrix)

[Phase 5: Quality Assurance]
    ├── TASK-016 (Paper alignment validation)
    ├── TASK-017 (Anti-convergence validation)
    ├── TASK-018 (Citation verification)
    └── TASK-019 (CEFR cognitive load validation)

[Phase 6: Integration]
    ├── TASK-020 (Docusaurus build test)
    └── TASK-021 (Final readability pass)
```

---

## Path Conventions

**Content Directory**:
```
book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/
```

**Assets Directory**:
```
book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/_assets/
```

---

## Phase 0: Research & Preparation

### TASK-001: Validate Statistics with Sources
**Priority**: P1 (Critical)
**Estimated Time**: 1 hour
**Dependencies**: None
**Assignee**: Content Implementer

**Description**:
Verify all statistics cited in the spec and plan with authoritative sources. Create citations.md documenting source URLs and access dates.

**Statistics to Verify**:
| Statistic | Expected Source |
|-----------|-----------------|
| 800M+ ChatGPT users weekly | OpenAI announcements |
| 90%+ developers using AI tools | Stack Overflow/JetBrains survey |
| 44% of US work hours | McKinsey "Agents, robots, and us" |
| $2.9T economic value potential | McKinsey research |
| 7x growth in AI fluency demand | LinkedIn/O'Reilly analysis |

**Acceptance Criteria**:
- [x] All 5 statistics verified with URLs
- [x] Access dates recorded
- [x] citations.md created at `specs/038-chapter-33-intro-ai-agents/citations.md`

**Output**: `specs/038-chapter-33-intro-ai-agents/citations.md`
**Status**: ✅ COMPLETE (statistics embedded in lessons with inline citations)

---

### TASK-002: Research SDK Documentation for Lesson 7
**Priority**: P2 (High)
**Estimated Time**: 2 hours
**Dependencies**: None
**Assignee**: Content Implementer
**Status**: ✅ COMPLETE

**Description**:
Research official documentation for the 4 SDK frameworks to create accurate comparison in Lesson 7.

**Frameworks to Research**:
1. **OpenAI Agents SDK**: Philosophy, key features, strengths, when to use
2. **Google ADK**: Agent Engine, MCP support, callbacks, plugins (paper emphasizes this)
3. **Anthropic Agents Kit**: Safety-first approach, extended thinking
4. **LangChain**: Model-agnostic, composability, tool ecosystem

**Acceptance Criteria**:
- [x] Each framework: 3-4 distinguishing characteristics documented
- [x] Comparison matrix drafted
- [x] Official documentation URLs recorded

**Output**: Research embedded in Lesson 7

---

## Phase 1: Foundation Content (Layer 1)

### TASK-003: Create Chapter README.md
**Priority**: P1 (Critical)
**Estimated Time**: 30 minutes
**Dependencies**: None
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/README.md`

**Content Requirements**:
- Chapter title: "Introduction to AI Agents"
- Chapter overview (bridge from Part 5 Python to Part 6 AI Native)
- **8 lessons** listed with titles and brief descriptions
- Learning objectives (aligned with paper frameworks)
- Prerequisites: Parts 1-5 completed (AIDD mindset, Claude Code, Python)
- Time estimate: ~2-3 hours
- What's next: Chapter 34 (OpenAI Agents SDK)

**Frontmatter**:
```yaml
---
sidebar_position: 33
title: "Introduction to AI Agents"
description: "Foundational concepts for AI agent development using Google's authoritative frameworks"
---
```

**Acceptance Criteria**:
- [ ] All 8 lessons listed with descriptions
- [ ] Paper frameworks mentioned in objectives
- [ ] Prerequisites clearly stated
- [ ] Proper Docusaurus frontmatter

**Output**: README.md

---

### TASK-004: Write Lesson 1 — What Is an AI Agent?
**Priority**: P1 (Critical)
**Estimated Time**: 2 hours
**Dependencies**: TASK-003
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/01-what-is-an-ai-agent.md`

**Paper Source**: "From Predictive AI to Autonomous Agents", "Introduction to AI Agent", taxonomy section

**Learning Objectives**:
- LO1.1: Define AI agent using paper's definition
- LO1.2: Classify systems using **5-Level Taxonomy** (Level 0-4)
- LO1.3: Articulate "director vs bricklayer" paradigm shift

**Content Sections** (from plan.md):
1. **Why Agents Matter Now** (hook with 800M+ users, paradigm shift)
2. **The Paper's Definition**: "The combination of models, tools, an orchestration layer, and runtime services which uses the LM in a loop to accomplish a goal"
3. **5-Level Taxonomy** (MUST match paper exactly):
   - Level 0: Core Reasoning System (LLM alone, no tools)
   - Level 1: Connected Problem-Solver (LLM + tools, real-time data)
   - Level 2: Strategic Problem-Solver (context engineering, multi-step planning)
   - Level 3: Collaborative Multi-Agent System (team of specialists)
   - Level 4: Self-Evolving System (creates new tools/agents dynamically)
4. **Director vs Bricklayer** (Paper's key insight quote)
5. **Career Implications** (statistics: 44% work hours, $2.9T, 7x growth)
6. **Try With AI — Classify Systems** (action prompts to classify Claude Code, ChatGPT, etc.)

**Constraints**:
- Layer 1 only (no "Tell your AI..." in main content)
- 3 NEW concepts maximum
- All statistics must have inline citations
- Paper's exact terminology for taxonomy

**Acceptance Criteria**:
- [ ] Paper's agent definition quoted accurately
- [ ] 5-Level Taxonomy uses paper's exact Level 0-4 classification
- [ ] "Director vs bricklayer" quote from paper included
- [ ] Statistics cited with sources (from TASK-001)
- [ ] "Try With AI" uses action prompts only (no meta-commentary)
- [ ] ~2,500-3,000 words
- [ ] Frontmatter: sidebar_position: 1

**Output**: 01-what-is-an-ai-agent.md

---

### TASK-005: Write Lesson 2 — Core Agent Architecture
**Priority**: P1 (Critical)
**Estimated Time**: 2.5 hours
**Dependencies**: TASK-004
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/02-core-agent-architecture.md`

**Paper Source**: "Core Agent Architecture: Model, Tools, and Orchestration", deployment section

**Learning Objectives**:
- LO2.1: Name and describe **3+1 Architecture** (Model, Tools, Orchestration, Deployment)
- LO2.2: Use paper's Body Part analogies correctly
- LO2.3: Explain role of each component with examples

**Content Sections** (from plan.md):
1. **The 3+1 Architecture Overview** (system diagram)
2. **Model — "The Brain"**:
   - Reasoning core, cognitive capabilities
   - Model selection: quality vs speed vs cost trade-off
   - "Team of specialists" approach (Gemini Pro for reasoning, Flash for simple)
   - Multimodal vs language-only decision
3. **Tools — "The Hands"**:
   - Retrieving Information: RAG, NL2SQL, Google Search, databases
   - Executing Actions: APIs, code execution, email, scheduling
   - Human Interaction: HITL tools (ask_for_confirmation, ask_for_date_input)
   - Function Calling: OpenAPI specification, MCP protocol
4. **Orchestration — "The Nervous System"**:
   - Planning: Breaking goals into steps
   - Memory: Short-term (session scratchpad) + Long-term (RAG/vector DB)
   - Reasoning strategies: Chain-of-Thought, ReAct
5. **Deployment — "The Body"**:
   - Runtime services: hosting, logging, monitoring
   - Accessibility: GUI or A2A API
   - Production infrastructure: Docker, Cloud Run, Vertex AI Agent Engine
6. **Try With AI — Identify Architecture** (identify components in Claude Code)

**Constraints**:
- Layer 1 only
- 4 NEW concepts (Model, Tools, Orchestration, Deployment)
- MUST use Brain/Hands/Nervous System/Body analogies from paper

**Acceptance Criteria**:
- [ ] 3+1 Architecture taught with paper's Body Part analogies
- [ ] All subcomponents from paper included
- [ ] Example of each component's role included
- [ ] ~3,500-4,000 words
- [ ] Diagram reference: agent-architecture-3plus1.svg

**Output**: 02-core-agent-architecture.md

---

### TASK-006: Write Lesson 3 — The Agentic Problem-Solving Process
**Priority**: P1 (Critical)
**Estimated Time**: 2 hours
**Dependencies**: TASK-005
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/03-agentic-problem-solving-process.md`

**Paper Source**: "The Agentic Problem-Solving Process" section with Customer Support Agent example

**Learning Objectives**:
- LO3.1: Recite and explain **5-Step Operational Loop**
- LO3.2: Trace through paper's Customer Support example
- LO3.3: Apply loop to new scenarios

**Content Sections** (from plan.md):
1. **The Universal Agentic Process** (hook: What does agent actually DO?)
2. **5-Step Operational Loop** (MUST match paper exactly):
   1. **Get the Mission**: Goal or trigger initiates process
   2. **Scan the Scene**: Perceive environment, access resources, check memory
   3. **Think It Through**: Reasoning, planning, chain of thought
   4. **Take Action**: Tool invocation (API call, code execution, database query)
   5. **Observe and Iterate**: Feedback loop, update context, repeat
3. **Paper's Customer Support Example** (MUST match paper exactly):
   - User: "Where is my order #12345?"
   - Think: "I need order details + shipping status"
   - Act: find_order("12345") → get tracking number
   - Observe: "ZYX987"
   - Think: "Now I need carrier status"
   - Act: get_shipping_status("ZYX987")
   - Observe: "Out for Delivery"
   - Report: Synthesize response
4. **Context Engineering** (Paper's key insight): "An agent's accuracy depends on a focused, high-quality context"
5. **Try With AI — Walk Through Loop** (trace through Coffee Shop example)

**Constraints**:
- Transition from Layer 1 to Layer 2
- 2 NEW concepts (5-Step Loop, Context Engineering)
- Customer Support example MUST match paper's version exactly

**Acceptance Criteria**:
- [ ] 5-Step Loop uses paper's exact step names
- [ ] Customer Support example matches paper (order #12345, tracking ZYX987)
- [ ] Context engineering explained as "curating model's limited attention"
- [ ] ~2,500-3,000 words
- [ ] Diagram reference: five-step-loop.svg

**Output**: 03-agentic-problem-solving-process.md

---

## Phase 2: Operations Content (Layer 2)

### TASK-007: Write Lesson 4 — Multi-Agent Design Patterns
**Priority**: P1 (Critical)
**Estimated Time**: 2.5 hours
**Dependencies**: TASK-006
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/04-multi-agent-design-patterns.md`

**Paper Source**: "Multi-Agent Systems and Design Patterns" section

**Learning Objectives**:
- LO4.1: Name and describe 4 multi-agent patterns from paper
- LO4.2: Match use cases to appropriate patterns
- LO4.3: Explain single vs multi-agent trade-offs

**Content Sections** (from plan.md):
1. **Why Multi-Agent?** (hook: "Building a single, all-powerful 'super-agent' becomes inefficient")
2. **Pattern 1: Coordinator**:
   - "Manager" agent analyzing requests, routing to specialists
   - Aggregating responses into comprehensive answer
   - Use case: Complex requests needing multiple expertise areas
3. **Pattern 2: Sequential**:
   - "Digital assembly line" — output flows to next agent
   - Use case: Linear workflows, document processing pipelines
4. **Pattern 3: Iterative Refinement**:
   - Generator-Critic feedback loop
   - Use case: Quality assurance, content creation
5. **Pattern 4: Human-in-the-Loop (HITL)**:
   - Deliberate pause for human approval
   - Use case: High-stakes decisions, compliance requirements
6. **Pattern Selection Framework** (decision tree)
7. **Try With AI — Match Patterns** (match scenarios to patterns with AI discussion)

**Constraints**:
- Layer 2 (AI collaboration through Try With AI)
- 4 NEW concepts (one per pattern)
- **MUST use paper's pattern names** (Coordinator, Sequential, Iterative Refinement, HITL)
- **DO NOT use**: ReAct, Plan-Execute (these are reasoning strategies, not multi-agent patterns)

**Acceptance Criteria**:
- [ ] All 4 patterns use paper's exact names
- [ ] Each pattern has clear use case from paper
- [ ] Generator-Critic loop explained for Iterative Refinement
- [ ] ~3,000-3,500 words
- [ ] Diagram references: coordinator-pattern.svg, sequential-pattern.svg, iterative-refinement-pattern.svg, human-in-loop-pattern.svg

**Output**: 04-multi-agent-design-patterns.md

---

### TASK-008: Write Lesson 5 — Agent Ops
**Priority**: P1 (Critical)
**Estimated Time**: 2 hours
**Dependencies**: TASK-007
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/05-agent-ops.md`

**Paper Source**: "Agent Ops: A Structured Approach to the Unpredictable" section

**Learning Objectives**:
- LO5.1: Explain why traditional testing doesn't work for agents
- LO5.2: Describe LM-as-Judge evaluation approach
- LO5.3: Explain debugging with OpenTelemetry traces
- LO5.4: Describe human feedback loop

**Content Sections** (from plan.md):
1. **Why Agent Ops?** (hook: "Traditional unit tests assert output == expected; but that doesn't work when response is probabilistic")
2. **Measure What Matters**:
   - KPIs: goal completion, user satisfaction, latency, cost
   - Frame like A/B test: what proves agent delivers value?
3. **LM as Judge**:
   - Using model to assess output quality against rubric
   - Example rubric structure
4. **Golden Datasets**:
   - Ideal questions and correct responses
   - Metrics-driven development: go/no-go based on scores
5. **Debug with OpenTelemetry Traces**:
   - "Why did agent do that?" — trace is step-by-step recording
   - Inspect: prompt, reasoning, tool choice, parameters, result
   - Google Cloud Trace for visualization
6. **Cherish Human Feedback**:
   - "Most valuable resource for improvement"
   - Feedback loop: capture → replicate → add to evaluation dataset → prevent recurrence
7. **Try With AI — Design Evaluation** (design evaluation rubric for agent use case)

**Constraints**:
- Layer 2
- 4 NEW concepts (LM-as-Judge, Golden datasets, Traces, Feedback loop)
- Paper's Agent Ops framework exactly

**Acceptance Criteria**:
- [ ] "Traditional unit tests don't work" rationale explained
- [ ] LM-as-Judge with rubrics explained
- [ ] Golden datasets concept from paper included
- [ ] OpenTelemetry traces explained
- [ ] Human feedback loop from paper included
- [ ] ~2,500-3,000 words
- [ ] Diagram reference: agent-ops-workflow.svg

**Output**: 05-agent-ops.md

---

### TASK-009: Write Lesson 6 — Agent Interoperability & Security
**Priority**: P1 (Critical)
**Estimated Time**: 2.5 hours
**Dependencies**: TASK-008
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/06-agent-interoperability-security.md`

**Paper Source**: "Agent Interoperability" and "Securing a Single Agent" sections

**Learning Objectives**:
- LO6.1: Describe agent-human interaction patterns
- LO6.2: Explain A2A protocol and Agent Cards
- LO6.3: Articulate agent identity as new principal class
- LO6.4: Explain trust trade-off and defense in depth

**Content Sections** (from plan.md):
1. **Agent Interoperability Overview** (the "face" of the agent)
2. **Agents and Humans**:
   - Chatbots, computer use, live mode (bidirectional streaming)
   - Multimodal: camera + microphone for technician guidance example
3. **Agents and Agents (A2A)**:
   - Challenge: discovery and communication
   - **A2A Protocol**: Universal handshake for agentic economy
   - **Agent Cards**: JSON advertising capabilities, endpoints, credentials
   - Task-oriented architecture: asynchronous tasks with streaming updates
4. **Agent Security — The Trust Trade-Off**:
   - "Every ounce of power introduces corresponding measure of risk"
   - Primary concerns: rogue actions, sensitive data disclosure
5. **Agent Identity — A New Principal Class**:
   - Users (OAuth/SSO), Services (IAM), **Agents** (NEW — SPIFFE)
   - Agents need verifiable "digital passport" distinct from user/developer
   - Least-privilege permissions per agent
6. **Defense in Depth**:
   - Layer 1: Deterministic guardrails (hard limits)
   - Layer 2: AI-powered guard models (contextual screening)
7. **Try With AI — Security Design** (identify security requirements for agent scenario)

**Constraints**:
- Layer 2
- 4 NEW concepts (A2A/Agent Cards, Agent Identity, Trust trade-off, Defense in depth)
- Paper's security framework exactly

**Acceptance Criteria**:
- [ ] A2A Protocol and Agent Cards from paper explained
- [ ] Agent identity as NEW principal class (distinct from users/services)
- [ ] Trust trade-off concept from paper included
- [ ] Defense in depth layers explained
- [ ] ~3,000-3,500 words
- [ ] Diagram references: a2a-protocol.svg, agent-identity-principals.svg

**Output**: 06-agent-interoperability-security.md

---

### TASK-010: Write Lesson 7 — The Agent SDK Landscape
**Priority**: P2 (High)
**Estimated Time**: 2 hours
**Dependencies**: TASK-002, TASK-009
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/07-agent-sdk-landscape.md`

**Paper Source**: Framework guidance throughout, ADK emphasis

**Learning Objectives**:
- LO7.1: Name 4+ agent frameworks/SDKs
- LO7.2: Describe 2-3 distinguishing characteristics of each
- LO7.3: Articulate framework selection factors

**Content Sections** (from plan.md):
1. **Why Framework Choice Matters** (hook)
2. **Framework 1: OpenAI Agents SDK**:
   - Philosophy: Simplicity, built-in function calling
   - Strengths: Deep integration with GPT models
   - When to use: OpenAI ecosystem, rapid prototyping
3. **Framework 2: Google ADK** (paper's focus):
   - Philosophy: Production-grade, enterprise features
   - Features: Agent Engine, built-in memory, MCP support
   - Paper's guidance on callbacks, plugins, Model Armor
4. **Framework 3: Anthropic Agents Kit**:
   - Philosophy: Safety-first, extended thinking
   - Strengths: Reasoning depth, tool-use safety
5. **Framework 4: LangChain**:
   - Philosophy: Model-agnostic, composability
   - Strengths: Any model, extensive tool ecosystem
6. **Comparison Matrix** (decision framework)
7. **Transferability** (CRITICAL): "Components and patterns transfer across frameworks"
8. **Try With AI — Framework Comparison** (compare frameworks for specific use case)

**Constraints**:
- Layer 2
- Relies on TASK-002 research
- Emphasize transferability per paper

**Acceptance Criteria**:
- [ ] All 4 frameworks described accurately (from TASK-002)
- [ ] 2-3 distinguishing characteristics each
- [ ] Comparison matrix included
- [ ] Transferability message emphasized
- [ ] ~3,000-3,500 words
- [ ] Diagram reference: sdk-comparison-matrix.png

**Output**: 07-agent-sdk-landscape.md

---

## Phase 3: Synthesis (Layer 3)

### TASK-011: Write Lesson 8 — Your First Agent Concept
**Priority**: P1 (Critical)
**Estimated Time**: 2.5 hours
**Dependencies**: TASK-010
**Assignee**: Content Implementer

**File Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/08-your-first-agent-concept.md`

**Learning Objectives**:
- LO8.1: Design agent specification using paper's frameworks
- LO8.2: Articulate agent's purpose using 5-Level Taxonomy
- LO8.3: Specify architecture using 3+1 components
- LO8.4: Identify appropriate pattern and security considerations

**Content Sections** (from plan.md):
1. **Specification-First Thinking** (bridge to Chapter 34, SDD-RI connection)
2. **Agent Specification Template** (using ALL paper frameworks):
   - **Level** (0-4): What's the agent's capability tier?
   - **Architecture**: Model, Tools, Orchestration, Deployment choices
   - **Process**: How does 5-Step Loop apply?
   - **Pattern**: Single, Coordinator, Sequential, or HITL?
   - **Operations**: How will it be evaluated, debugged?
   - **Interoperability**: Human interface, A2A needs?
   - **Security**: Trust level, identity, guardrails?
3. **Example Specifications** (3 fully filled-in):
   - Customer Support Agent (Level 2, Sequential)
   - Research Assistant (Level 2, Coordinator + HITL)
   - Code Review Agent (Level 3, Iterative Refinement)
4. **Guided Specification Design** (student creates own spec)
5. **Try With AI — Refine Your Design** (AI feedback on specification)
6. **Transition to Chapter 34** (preview: Building with OpenAI SDK)

**Constraints**:
- Layer 3 synthesis
- 0 NEW concepts (synthesis only)
- MUST reference ALL paper frameworks from Lessons 1-7

**Acceptance Criteria**:
- [ ] Specification template uses all paper frameworks
- [ ] 3 example specifications fully completed
- [ ] Each example uses different pattern appropriately
- [ ] Student exercise guides spec creation
- [ ] Clear bridge to Chapter 34
- [ ] ~3,000-3,500 words

**Output**: 08-your-first-agent-concept.md

---

## Phase 4: Visual Assets

### TASK-012: Create Foundation Diagrams
**Priority**: P2 (High)
**Estimated Time**: 4 hours
**Dependencies**: TASK-004, TASK-005, TASK-006
**Assignee**: Visual Designer / Content Implementer

**Output Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/_assets/`

**Diagrams**:

1. **five-level-taxonomy.svg** (Lesson 1):
   - Pyramid/tier diagram showing Level 0-4
   - Reference: Paper's "A Taxonomy of Agentic Systems" section
   - Labels: Level names + brief descriptions
   - Visual: Progressive capability from bottom (L0) to top (L4)

2. **director-vs-bricklayer.svg** (Lesson 1):
   - Split illustration showing paradigm shift
   - Left: Traditional "bricklayer" (step-by-step code)
   - Right: Agent "director" (guiding autonomous actors)
   - Reference: Paper's quote about director vs bricklayer

3. **agent-architecture-3plus1.svg** (Lesson 2):
   - System diagram showing Model + Tools + Orchestration + Deployment
   - Include Body Part labels (Brain, Hands, Nervous System, Body)
   - Reference: Paper's "Core Agent Architecture" section
   - Show interconnections between components

4. **five-step-loop.svg** (Lesson 3):
   - Circular diagram: Get Mission → Scan Scene → Think → Act → Observe → (loop back)
   - Reference: Paper's Figure 1 "Agentic AI problem-solving process"
   - Include example actions at each step

**Acceptance Criteria**:
- [ ] All 4 diagrams production-quality SVG
- [ ] Labels clear and readable
- [ ] Match paper's visual language where applicable
- [ ] Render correctly in Docusaurus

---

### TASK-013: Create Pattern Diagrams
**Priority**: P2 (High)
**Estimated Time**: 4 hours
**Dependencies**: TASK-007
**Assignee**: Visual Designer / Content Implementer

**Output Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/_assets/`

**Diagrams**:

1. **coordinator-pattern.svg** (Lesson 4):
   - Manager agent at center routing to specialist agents
   - Aggregation arrows back to manager
   - Reference: Paper's "Coordinator pattern" description

2. **sequential-pattern.svg** (Lesson 4):
   - Linear flow: Agent A → Agent B → Agent C
   - "Assembly line" visual metaphor
   - Reference: Paper's "Sequential pattern" description

3. **iterative-refinement-pattern.svg** (Lesson 4):
   - Generator-Critic feedback loop
   - Circular arrows showing iteration
   - Reference: Paper's Figure 3

4. **human-in-loop-pattern.svg** (Lesson 4):
   - Workflow with pause points for human approval
   - Decision diamond + human icon at checkpoints
   - Reference: Paper's "Human-in-the-Loop pattern" description

**Acceptance Criteria**:
- [ ] All 4 pattern diagrams match paper's descriptions
- [ ] Clear flow arrows and labels
- [ ] Iterative Refinement shows feedback loop clearly
- [ ] HITL shows pause points clearly

---

### TASK-014: Create Operations Diagrams
**Priority**: P2 (High)
**Estimated Time**: 3 hours
**Dependencies**: TASK-008, TASK-009
**Assignee**: Visual Designer / Content Implementer

**Output Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/_assets/`

**Diagrams**:

1. **agent-ops-workflow.svg** (Lesson 5):
   - Cycle: Metrics → Evaluation → Debug → Feedback → (loop)
   - Include LM-as-Judge, Golden Datasets, Traces elements
   - Reference: Paper's "Agent Ops" section

2. **a2a-protocol.svg** (Lesson 6):
   - Agent Cards discovery + task-oriented communication
   - Show: Agent A → Agent Card → Agent B → Task Request/Response
   - Reference: Paper's "Agent Interoperability" section

3. **agent-identity-principals.svg** (Lesson 6):
   - Three principal classes: Users (OAuth), Agents (SPIFFE), Services (IAM)
   - Reference: Paper's Table 1 "Principal entity authentication"
   - Visual distinction between the three types

**Acceptance Criteria**:
- [ ] Agent Ops workflow shows complete cycle
- [ ] A2A shows discovery + communication flow
- [ ] Identity diagram distinguishes 3 principal classes clearly

---

### TASK-015: Create SDK Comparison Matrix
**Priority**: P2 (High)
**Estimated Time**: 2 hours
**Dependencies**: TASK-010
**Assignee**: Visual Designer / Content Implementer

**Output Path**: `book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/_assets/`

**Diagram**:

**sdk-comparison-matrix.png** (Lesson 7):
- Table comparing: OpenAI Agents SDK, Google ADK, Anthropic Agents Kit, LangChain
- Rows: Philosophy, Key Features, Best For, Ecosystem
- Clean, scannable format
- Based on TASK-002 research

**Acceptance Criteria**:
- [ ] All 4 frameworks compared
- [ ] 3-4 comparison dimensions
- [ ] Accurate based on TASK-002 research
- [ ] Renders well at various sizes

---

## Phase 5: Quality Assurance

### TASK-016: Paper Alignment Validation
**Priority**: P1 (Critical)
**Estimated Time**: 2 hours
**Dependencies**: TASK-004 through TASK-011 (all content tasks)
**Assignee**: Validator

**Description**:
Validate ALL lessons against Google "Introduction to Agents" paper to ensure frameworks are taught exactly as specified.

**Validation Checklist**:
- [ ] **5-Level Taxonomy** uses paper's exact Level 0-4 classification
- [ ] **3+1 Architecture** uses Body Part analogies (Brain, Hands, Nervous System, Body)
- [ ] **5-Step Loop** matches: Get Mission → Scan Scene → Think → Act → Observe
- [ ] **Patterns** use paper's names: Coordinator, Sequential, Iterative Refinement, HITL
- [ ] **Agent Ops** includes: LM-as-Judge, Golden Datasets, Traces, Human Feedback
- [ ] **A2A Protocol** and **Agent Cards** explained per paper
- [ ] **Agent Identity** explained as new principal class
- [ ] **"Director vs bricklayer"** paradigm properly conveyed
- [ ] **Customer Support example** matches paper's version exactly (order #12345, tracking ZYX987)

**Anti-Pattern Check**:
- [ ] NO "Agency Spectrum" (old framework)
- [ ] NO "5 Components" (old framework)
- [ ] NO "ReAct pattern" as multi-agent pattern (it's a reasoning strategy)
- [ ] NO "Plan-Execute pattern" as multi-agent pattern (it's a reasoning strategy)

**Output**: Validation report with any corrections needed

---

### TASK-017: Anti-Convergence Validation
**Priority**: P1 (Critical)
**Estimated Time**: 1 hour
**Dependencies**: TASK-004 through TASK-011 (all content tasks)
**Assignee**: Validator

**Description**:
Run anti-convergence checks to ensure Three Roles framework is invisible and no meta-commentary exists.

**Validation Commands**:
```bash
# Check for meta-commentary violations
grep -i "What to notice\|AI.*teach\|AI as\|AI.*learn\|AI now knows" book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/*.md

# Expected: Zero matches (except legitimate usage like "teacher" in context)
```

**Checklist**:
- [ ] No "AI as Teacher/Student/Co-Worker" labels
- [ ] No "What to notice: AI is teaching you..." meta-commentary
- [ ] "Try With AI" sections use action prompts only
- [ ] Varied teaching modalities (not lecture-only)
- [ ] Production-relevant examples (not toy apps)

**Output**: Validation report

---

### TASK-018: Citation Verification
**Priority**: P1 (Critical)
**Estimated Time**: 1 hour
**Dependencies**: TASK-001, TASK-004 through TASK-011
**Assignee**: Validator

**Description**:
Verify all statistics have proper citations matching citations.md.

**Statistics to Verify**:
- [ ] 800M+ ChatGPT users → citation present in Lesson 1
- [ ] 90%+ developers using AI tools → citation present
- [ ] 44% of US work hours → McKinsey cited in Lessons 1, 5
- [ ] $2.9T economic value → McKinsey cited in Lesson 1
- [ ] 7x growth in AI fluency demand → citation present in Lesson 1

**Output**: Citation verification report

---

### TASK-019: CEFR Cognitive Load Validation
**Priority**: P2 (High)
**Estimated Time**: 30 minutes
**Dependencies**: TASK-004 through TASK-011 (all content tasks)
**Assignee**: Validator

**Description**:
Validate concept counts per lesson against B1 tier limits (max 7-10 NEW concepts).

**Expected Concept Counts** (from plan.md):
- [ ] Lesson 1: 3 NEW (Agent definition, 5-Level Taxonomy, Paradigm shift) ✅
- [ ] Lesson 2: 4 NEW (Model, Tools, Orchestration, Deployment) ✅
- [ ] Lesson 3: 2 NEW (5-Step Loop, Context Engineering) ✅
- [ ] Lesson 4: 4 NEW (Coordinator, Sequential, Iterative Refinement, HITL) ✅
- [ ] Lesson 5: 4 NEW (LM-as-Judge, Golden datasets, Traces, Feedback loop) ✅
- [ ] Lesson 6: 4 NEW (A2A/Agent Cards, Agent Identity, Trust trade-off, Defense in depth) ✅
- [ ] Lesson 7: 1 core NEW (SDK Landscape with variations) ✅
- [ ] Lesson 8: 0 NEW (synthesis only) ✅

**Output**: CEFR validation report

---

## Phase 6: Integration

### TASK-020: Docusaurus Build Test
**Priority**: P1 (Critical)
**Estimated Time**: 30 minutes
**Dependencies**: All content and visual tasks
**Assignee**: Content Implementer

**Description**:
Build Docusaurus site locally and validate:
- All 8 lessons render correctly
- All diagrams display properly
- Navigation works (sidebar, next/previous)
- No build errors or warnings

**Commands**:
```bash
cd book-source
npm run build
npm run serve
```

**Acceptance Criteria**:
- [ ] Build completes without errors
- [ ] All 8 lessons accessible via navigation
- [ ] All SVG/PNG diagrams render
- [ ] Internal links work
- [ ] External links work

**Output**: Build validation report

---

### TASK-021: Final Readability Pass
**Priority**: P2 (High)
**Estimated Time**: 1 hour
**Dependencies**: TASK-020
**Assignee**: Content Implementer

**Description**:
Final pass through all content for:
- Typos and grammar
- Consistent terminology (paper's terms)
- Smooth transitions between lessons
- Coherent narrative arc from Lesson 1 → Lesson 8

**Checklist**:
- [ ] All lessons read smoothly
- [ ] Terminology consistent throughout (paper's frameworks)
- [ ] Transitions guide student through progression
- [ ] Cross-references work (links to other lessons)
- [ ] Chapter flows coherently from definition → architecture → process → patterns → ops → interop → SDKs → synthesis

**Output**: Final readiness confirmation

---

## Summary

| Phase | Tasks | Est. Time | Priority |
|-------|-------|-----------|----------|
| Phase 0: Research | TASK-001, TASK-002 | 3 hours | P1/P2 |
| Phase 1: Foundation (L1) | TASK-003 to TASK-006 | 7 hours | P1 |
| Phase 2: Operations (L2) | TASK-007 to TASK-010 | 9 hours | P1/P2 |
| Phase 3: Synthesis (L3) | TASK-011 | 2.5 hours | P1 |
| Phase 4: Visual Assets | TASK-012 to TASK-015 | 13 hours | P2 |
| Phase 5: QA | TASK-016 to TASK-019 | 4.5 hours | P1/P2 |
| Phase 6: Integration | TASK-020, TASK-021 | 1.5 hours | P1/P2 |

**Total Estimated Time**: ~40.5 hours

---

## Parallel Execution Opportunities

### Research Phase (Phase 0)
- TASK-001 and TASK-002 can run in parallel

### Content Phases (Phases 1-3)
- Content tasks are sequential (each lesson builds on prior)
- However, diagrams (Phase 4) can be created in parallel with content

### Visual Assets (Phase 4)
- TASK-012, TASK-013, TASK-014, TASK-015 can all run in parallel
- Each can start as soon as dependent content task is complete

### QA Phase (Phase 5)
- TASK-016, TASK-017, TASK-018, TASK-019 can run in parallel after all content complete

---

## Critical Success Factors

1. **Paper Alignment** (MOST CRITICAL): All frameworks must match Google "Introduction to Agents" paper exactly
2. **No Old Framework Leakage**: Must NOT use "Agency Spectrum", "5 Components", "ReAct pattern", "Plan-Execute pattern"
3. **Pedagogical Progression**: L1 (Lessons 1-3) → L2 (Lessons 4-7) → L3 (Lesson 8)
4. **Anti-Convergence**: Three Roles framework invisible, no meta-commentary
5. **Citation Accuracy**: All statistics verified and cited
6. **Production Quality**: Diagrams professional, content polished

---

## Implementation Notes

**Content Author Policy**:
- Each lesson ends with "Try With AI" section (no "Key Takeaways", no "What's Next")
- Students have completed Parts 1-5 — instruct to use preferred AI companion (Gemini CLI, Claude Code, ChatGPT)
- Action prompts only in "Try With AI" — no meta-commentary exposing Three Roles framework

**Layer Progression**:
- Lessons 1-3: Layer 1 (Manual Foundation) — No "Tell your AI..." in main content
- Lessons 4-7: Layer 2 (AI Collaboration) — "Try With AI" demonstrates partnership
- Lesson 8: Layer 3 (Synthesis) — Apply all frameworks to create agent specification

**Primary Source Priority**:
1. Google/Kaggle "Introduction to Agents" whitepaper (November 2025) — AUTHORITATIVE
2. McKinsey "Agents, robots, and us" — Statistics only
3. SDK official documentation — Lesson 7 only

---

**Tasks Status**: ✅ READY FOR IMPLEMENTATION

**Created**: 2025-11-27
**Revised**: 2025-11-27 (Aligned with Google "Introduction to Agents" whitepaper)
**Primary Source**: Google/Kaggle "Introduction to Agents" whitepaper (November 2025)
