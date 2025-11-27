---
title: "Core Agent Architecture"
sidebar_position: 2
description: "Understand the 3+1 Architecture of AI agents: Model (Brain), Tools (Hands), Orchestration (Nervous System), and Deployment (Body)."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 50 minutes
learning_objectives:
  - "Name and describe the 3+1 Core Architecture components (Model, Tools, Orchestration, Deployment)"
  - "Explain the role of each component using body-part analogies from Google's framework"
  - "Trace how these components work together in a real agent system"
  - "Identify these components when examining any agent framework"
skills:
  agent_architecture:
    proficiency: B1
  systems_thinking:
    proficiency: B1
generated_by: content-implementer v1.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 1.0.0
---

# Core Agent Architecture

In Lesson 1, you learned what distinguishes an agent from a chatbot: an agent is a system that reasons, acts, observes, and iterates in a loop. But what's inside that loop? What components make an agent actually work?

This lesson answers that question by showing you the **internal architecture of agents**—the "machinery" that enables autonomous action. You'll learn the four components that appear in every agent, from Claude Code to specialized research agents. Understanding this architecture means you'll recognize these components regardless of which SDK you eventually use, and you'll understand the trade-offs developers face when designing agent systems.

---

## The 3+1 Architecture Overview

Every agent, regardless of complexity, is built from four integrated components. The Google "Introduction to Agents" whitepaper calls this the **3+1 Core Architecture**, where each component has a distinct role:

**The "3"** (reasoning and action):
- **Model**: The "Brain" — reasoning engine, core intelligence
- **Tools**: The "Hands" — mechanisms for action, data access, interaction
- **Orchestration**: The "Nervous System" — the controlling process that governs planning, memory, and reasoning strategies

**The "+1"** (accessibility and reliability):
- **Deployment**: The "Body" — runtime services, hosting, accessibility layer

This isn't just a way to describe agents; it's the mental model that major cloud platforms have converged on. When you evaluate agent frameworks in Chapter 7, you'll be checking whether they have complete, well-designed implementations of these four components.

Let's examine each component in depth, starting with the reasoning core and ending with how agents reach users.

---

## Component 1: Model — "The Brain"

The Model is the reasoning engine at the core of every agent. It's the component that understands problems, generates plans, and makes decisions.

### What the Model Does

The Model processes information and generates reasoning. When you ask an agent to accomplish something, the Model:
1. **Reads and understands** the goal and context
2. **Reasons about the problem** — breaking it into steps, identifying dependencies
3. **Generates reasoning traces** — showing the logic behind decisions
4. **Decides what to do next** — which tools to use, what information to gather, how to proceed

This is fundamentally different from traditional software. Traditional code follows predetermined logic branches. An agent's Model *generates* the logic based on context.

### Model Selection: Quality, Speed, and Cost Trade-offs

One of the critical design decisions in agent development is **which model to use**. Different models have different capabilities and trade-offs:

**High-reasoning models** (e.g., Claude 3.5 Sonnet, GPT-4o Turbo):
- **Strengths**: Superior reasoning for complex planning, nuanced decision-making, generating creative solutions
- **Cost**: More expensive per request
- **Speed**: Slower (more tokens, more computation)
- **Best for**: Complex multi-step reasoning, ambiguous problems requiring judgment

**Fast, lightweight models** (e.g., Claude 3.5 Haiku, GPT-4o Mini):
- **Strengths**: Quick responses, low cost, sufficient reasoning for routine tasks
- **Cost**: Cheaper per request
- **Speed**: Faster
- **Best for**: High-volume tasks (customer service routing, data classification), time-sensitive decisions

### The "Team of Specialists" Approach

Rather than using a single model for all decisions, production agents often employ a **team of specialists** — different models for different decision types.

**Example**: A research agent:
- **Complex analysis**: Uses Claude 3.5 Sonnet for in-depth reasoning about paper relationships and novel insights
- **Paper retrieval**: Uses Claude 3.5 Haiku to quickly classify which papers match search criteria (fast, cheap)
- **Simple summarization**: Uses Haiku again — the task is routine, doesn't need premium reasoning

This "team" approach reduces costs (using expensive reasoning only when it matters) while maintaining speed (simple decisions happen fast).

### Multimodal vs Language-Only

Modern agents increasingly handle multiple input types:
- **Language-only models**: Text input, text output. Good for traditional reasoning tasks.
- **Multimodal models**: Accept text, images, audio, video. Can reason about visual content, diagrams, screenshots.

This matters for agents handling real-world complexity. A document analysis agent that can read charts from PDFs doesn't need to extract them as text first. A customer support agent that sees the user's actual interface can understand problems more precisely.

### The Brain Recognizes Context and Constraints

An important nuance: the Model isn't just reasoning capability. It includes how the Model is *prompted and guided*. The same underlying Claude or GPT model can behave very differently depending on:
- **System prompts** defining the agent's role and constraints
- **Context packaging** — what information the Model sees when making decisions
- **Interaction patterns** — how the orchestration layer structures the reasoning

This is why context engineering (introduced in Chapter 12) becomes critical in agent development. The Model's effectiveness depends partly on how well you frame problems and provide context.

---

## Component 2: Tools — "The Hands"

If the Model is the reasoning engine, Tools are the mechanisms for *acting on that reasoning*. Without tools, an agent is just a chatbot producing text. Tools are what enable agents to:
- Retrieve information from external sources
- Execute actions in the world
- Interact with humans and systems
- Validate and refine solutions

The Google paper organizes tools into four functional categories:

### Category 1: Retrieving Information (Data Access)

These tools bring external knowledge into the agent's reasoning process:

**Real-time data access**:
- **Web search APIs**: Google Search, Bing Search, or custom search to find current information
- **Database queries**: SQL tools to query company databases, product catalogs, user information
- **API calls**: Accessing real-time data from weather APIs, stock feeds, calendar services

**Knowledge indexing and search**:
- **RAG (Retrieval Augmented Generation)**: Tools that search a knowledge base (company documentation, research papers, code repositories) and return relevant context to the Model
- **Vector databases**: Semantic search over embeddings (better than keyword search for "find documents similar to this concept")
- **NL2SQL**: Natural language to SQL — convert user questions directly into database queries

**Example**: A customer support agent needs a tool to retrieve customer history. The tool accepts "Show me this customer's last 3 support tickets" and returns structured data the Model can reason about.

### Category 2: Executing Actions (External System Integration)

These tools let agents *do things* in external systems:

**API integrations**:
- **CRM APIs**: Create tickets, update customer records, log interactions
- **Email/messaging**: Send emails, post to Slack, update project management tools
- **Cloud services**: Provision resources, deploy code, manage infrastructure

**Code execution**:
- **Python/JavaScript sandboxes**: Run code to analyze data, transform formats, perform calculations
- **Shell commands**: Execute system commands safely (within guardrails)
- **Specialized tools**: Mathematical computation, image generation, document processing

**Scheduling and persistence**:
- **Task scheduling**: "Run this check daily," "Remind me in 3 days"
- **Data storage**: Save agent reasoning, decisions, learned patterns

**Example**: A financial agent needs tools to: retrieve stock data (retrieval), run calculations (execution), and submit trades (execution). Each is a separate tool with specific inputs/outputs.

### Category 3: Human Interaction (The "HITL" Tools)

Agents don't operate autonomously in high-stakes domains. They need tools to involve humans when necessary:

**Approval tools**:
- **ask_for_confirmation**: "I'm about to delete 100 customer records. Is this correct? [Y/N]"
- **ask_for_clarification**: "I found 3 matching products. Which one do you want?"
- **ask_for_date_input**: "What's the deadline for this task?"

**Escalation tools**:
- **escalate_to_human**: Transfer to a human specialist when the problem exceeds agent's authority
- **get_human_feedback**: "Was this analysis helpful? Any corrections?" — for improvement loops

These are critical in production. Agents must know *when they don't know* and ask for help.

### Category 4: Function Calling (Connecting Tools to Models)

How does the Model actually *invoke* these tools? Through **function calling** — a structured interface between Models and external systems.

**Function schema**:
```
Tool: retrieve_customer_history
Inputs: customer_id (string), num_records (integer)
Returns: list of customer support tickets with timestamps, issues, resolutions
```

The Model sees the available functions and their descriptions. When it needs customer history, it calls the function with appropriate parameters. The tool returns results. The Model receives those results and continues reasoning.

**Standard formats for function calling**:
- **OpenAPI specifications**: Standard format describing API endpoints and parameters
- **MCP (Model Context Protocol)**: Anthropic's protocol for connecting agents to external tools and data sources

The key insight: function calling is the *contract* between the Model's reasoning and the outside world. Well-designed function schemas make agents more reliable.

### Tool Limitations and Reliability

Not all tools are created equal. A critical design consideration:

**Tool reliability vs complexity trade-off**:
- **Simple tools** (just return data): Reliable, predictable, easy to orchestrate
- **Complex tools** (with multiple decision steps): Powerful, but harder for agents to use correctly

Production agents prioritize tool clarity. Rather than one "do everything" tool, it's better to have multiple focused tools with clear inputs/outputs.

---

## Component 3: Orchestration — "The Nervous System"

The orchestration layer is the *control system* that governs how the Model and Tools work together. It's the "nervous system" directing the agent's behavior.

### What Orchestration Does

Orchestration manages three critical functions:

**1. Planning**: Breaking complex goals into steps
- The agent receives: "Analyze this code for security vulnerabilities"
- The orchestration layer decides: "First, read the code. Then analyze for authentication issues. Then check for data exposure risks. Then check for injection vulnerabilities. Synthesize findings."
- The Model executes each step in sequence

**2. Memory**: Maintaining context across the problem-solving process
- **Short-term memory** (session scratchpad): Reasoning artifacts from current session. "What have we discovered so far? What's the next step?"
- **Long-term memory** (persistent): Earlier conversations, learned patterns, user preferences. Used for personalization and learning across sessions

**3. Reasoning strategies**: Different approaches to generate solutions
- **Chain-of-Thought**: Model explains its reasoning step-by-step
- **ReAct** (Reasoning + Acting): Alternate between reasoning steps and tool invocations; observe tool results, reason about them, act again
- **Other patterns**: Multi-turn dialogue, critique loops, self-correction

### The Orchestration Layer in Action

Let's see how orchestration governs behavior in a concrete example:

**Scenario**: A code review agent receiving: "Review this Python file for security issues."

**Orchestration directs**:
1. **Planning**: Break into steps — read file, analyze patterns, check vulnerability categories
2. **Reasoning Strategy** (ReAct): Alternate between thinking and doing
   - Reason: "The code has authentication. I should check how passwords are handled"
   - Act: Call `retrieve_code_section("password handling")`
   - Observe: Returns code snippet
   - Reason: "I see plaintext passwords. This is a vulnerability."
   - Act: Call `check_for_injection_patterns()`
   - Observe: Returns analysis
   - ... continue pattern
3. **Memory**: Maintain list of findings discovered so far
4. **Reasoning**: Synthesize findings into coherent security report

Without orchestration, the Model and Tools would be chaotic. Orchestration is what makes agent behavior *intentional and controlled*.

### Context Engineering at the Orchestration Layer

This is where **context engineering** (from Chapter 12) becomes critical. How does the orchestration layer decide what context the Model sees?

**Example**: A research agent analyzing 500 papers to answer "What's the current consensus on AI safety?"

**Strategic context selection**:
- Don't feed all 500 papers at once (overwhelming)
- Step 1: Use search tools to identify most relevant 20 papers
- Step 2: Analyze those 20 for consensus vs disagreement
- Step 3: If new disagreement found, search for papers specifically on that point
- Step 4: Return synthesized summary

The orchestration layer *actively manages* which information the Model sees at each step. This is the "context engineering" capability that distinguishes Level 2 agents from Level 1 agents.

---

## Component 4: Deployment — "The Body"

The final component is Deployment — how agents are made accessible to users and systems, and how they run reliably in production.

### What Deployment Covers

**Accessibility**: How users and systems interact with the agent
- **Chat interface**: Traditional chatbot UI (web or mobile)
- **API endpoint**: Agent as a service other applications call (A2A, agent-to-agent)
- **Computer use** (emerging): Agent that controls mouse/keyboard to interact with arbitrary systems
- **Live mode** (streaming): Bidirectional streaming where agent can send partial outputs as they're generated

**Runtime services**: Infrastructure that keeps agents running reliably
- **Hosting**: Where code runs (cloud platforms like Google Cloud, AWS, or local servers)
- **Scaling**: Handling spikes in usage (multiple agent instances)
- **Monitoring and logging**: Tracking agent behavior, debugging issues, auditing decisions
- **Rate limiting and access control**: Preventing abuse, controlling who can use the agent

### Hosting Decisions and Trade-offs

Where agents run is an important design decision:

**Cloud platforms** (Google Vertex AI, OpenAI Platform, AWS SageMaker):
- **Strengths**: Managed infrastructure, built-in monitoring, handled scaling
- **Trade-off**: Less control, potential vendor lock-in
- **Best for**: Production systems where reliability matters more than customization

**Self-hosted** (on-premises or custom cloud):
- **Strengths**: Full control, potentially lower cost at scale, meets compliance requirements
- **Trade-off**: Responsibility for operations, security, scaling
- **Best for**: Specialized requirements, sensitive data, high-volume custom deployments

**Hybrid**: Some core reasoning on a platform, specialized tools self-hosted
- **Strengths**: Flexibility, cost optimization
- **Trade-off**: Complexity in orchestration

### Logging, Monitoring, and Debugging

A critical aspect of deployment is *observability* — the ability to understand what an agent is doing.

**OpenTelemetry traces** (mentioned briefly here, deep dive in Lesson 5):
- Record the agent's complete reasoning trajectory
- Show: prompts sent to Model, tool calls made, results received, decisions reached
- Enable debugging: "Why did the agent choose this path?"

**Metrics**:
- Success rate: What percentage of requests achieve their goal?
- Performance: How long does an agent take?
- Cost: How much does each request cost?
- Quality: Evaluated against "golden dataset" (Lesson 5)

Without good logging and monitoring, you can't debug agent problems or improve performance. This is why "Agent Ops" (Lesson 5) is critical in production.

---

## How the Four Components Work Together

Understanding each component separately is useful. But agents are powerful because these components *integrate*.

### The Integration Loop

When you give an agent a task, here's what happens:

1. **Deployment** receives the task (via chat interface, API, or other access point)

2. **Orchestration** takes over:
   - Develops a plan
   - Manages memory and context
   - Decides on reasoning strategy

3. **Model** (the Brain) does the thinking:
   - Receives: "Here's the goal, here's what you know, here are available tools"
   - Reasons: "To accomplish this goal, I should..."
   - Decides: "I'll call Tool X"

4. **Tools** (the Hands) execute:
   - Retrieve data: "Query the database for this information"
   - Execute actions: "Create this ticket in the system"
   - Interact: "Ask the user for clarification"

5. **Orchestration** observes the results:
   - Updates memory: "We now know X"
   - Decides next step: "Given this information, let's do Y next"
   - Continues the loop

6. **Model** receives updated context and reasons again

7. This cycle repeats until:
   - The goal is achieved (success)
   - The agent reaches an impasse and escalates to a human (HITL)
   - A maximum iteration limit is reached

8. **Deployment** returns the final result to the user

### Example: Research Agent Architecture

Let's trace these components through a concrete agent:

**Task**: "Find the three most significant recent advances in protein folding and summarize why they matter."

**Model** (reasoning):
- Initial thought: "I need to search for recent papers on protein folding, find significant ones, understand their contributions"

**Orchestration** (planning + reasoning strategy):
- Step 1: Use web search to find recent papers (ReAct: think then act)
- Step 2: Read the 5 most relevant papers (using RAG tool to access their content)
- Step 3: Analyze contributions (this requires reasoning)
- Step 4: Synthesize into summary

**Tools** being invoked:
- `web_search("protein folding advances 2024 2025")`
- `retrieve_paper_content(paper_id)` (RAG)
- `summarize_findings()` (code execution tool)

**Model** observes results and continues reasoning:
- Sees: 5 papers found, AlphaFold 3 mentioned twice, OmegaFold mentioned
- Thinks: "These seem important. Let me analyze their specific contributions"
- Acts: Call `retrieve_paper_content()` for full papers

**Deployment** aspects:
- User submits via API endpoint
- Agent queries run with rate limiting
- Results streamed back as agent generates them
- Complete trace logged for audit

This is agent architecture in practice — four components working in concert to solve a goal that would be difficult for either a Model alone or traditional software.

---

## Why This Architecture Matters

The 3+1 architecture isn't arbitrary. It emerges from trying to build autonomous systems that work reliably:

**Separation of concerns**: Each component has a distinct responsibility. This makes agents easier to debug and improve.

**Composability**: You can upgrade components independently. Better Model available? Swap it in. Need faster responses? Use a cheaper Model for simple decisions. All other components continue working.

**Observability**: Having distinct components makes it easier to log, monitor, and understand agent behavior. You can see what reasoning happened, which tools were called, what results came back.

**Reliability**: By explicitly managing orchestration, memory, and human interaction, you build controls into the system. Agents don't drift off course because orchestration keeps them focused.

---

## Mapping to Real Frameworks

When you encounter actual agent frameworks (OpenAI Agents SDK, Google ADK, Anthropic frameworks), you'll see these components consistently:

- **OpenAI SDK**: Model selection, tool registration, message loop (orchestration)
- **Google ADK**: Model, tools (via generative AI APIs), orchestration patterns
- **Anthropic Claude SDK**: Model (Claude variants), tool use, orchestration guidance

Different frameworks implement these components differently, but they all have them. By learning this architecture now, you'll recognize these patterns immediately when you study specific SDKs in Chapters 34-36.

---

## Try With AI

Open Claude, ChatGPT, or Gemini and explore how these components work together.

**Setup**: You'll identify the 3+1 architecture components in Claude Code and real-world agent examples.

**Prompt Set 1: Identifying Components**

Ask your AI: "I'm learning about agent architecture. Here's the 3+1 model: Model (Brain, reasoning), Tools (Hands, actions), Orchestration (Nervous System, planning and memory), Deployment (Body, accessibility). When I ask you to 'write Python code to process a CSV file and save results to a database,' walk me through which components are involved in your response. Point out where reasoning happens, where you'd need tools, and how orchestration would manage the workflow."

**Expected Outcome**: The AI should identify that reasoning (Model) determines the approach, tools would include file reading and database writing, orchestration would manage steps, and deployment would handle how the code runs.

---

**Prompt Set 2: Model Selection Trade-offs**

Ask your AI: "I'm building an agent that processes 10,000 customer support requests daily. It needs to classify requests (easy), analyze sentiment (medium), and suggest escalation paths for complex issues (hard). How would I use different models for different decision types? Why would 'team of specialists' approach be better than using the same model for all three?"

**Expected Outcome**: The AI should explain using fast/cheap models for classification, medium models for sentiment, premium models for complex reasoning. Cost and speed benefits of specialization should be clear.

---

**Prompt Set 3: Orchestration in Action**

Ask your AI: "Walk me through how orchestration would manage a code review agent that must: (1) Read a file, (2) check for security issues, (3) check for performance issues, (4) ensure test coverage exists, (5) provide feedback. How would planning work? Where would memory matter? What reasoning strategy would be effective?"

**Expected Outcome**: The AI should explain step-by-step planning, memory tracking findings, ReAct strategy alternating between analysis and verification.

---

**Prompt Set 4: Deployment Considerations**

Ask your AI: "An agent needs to process documents securely without data leaving a company network. The agent also needs real-time access to a knowledge base. What deployment considerations matter? Should it be cloud-hosted, self-hosted, or hybrid?"

**Expected Outcome**: The AI should discuss compliance needs, data sensitivity, knowledge base location, and suggest a deployment architecture matching those constraints.

---

**Optional Stretch Challenge**:

Identify a tool you use daily (email, project management, note-taking). If that tool had agentic capabilities, which of the 3+1 components would be most important to implement first? Why?

Ask your AI: "I want my [tool] to have agentic capabilities — like 'organize my emails and summarize decisions' or 'create project tasks from my notes.' If I could only implement one component of the 3+1 architecture well, which would deliver the most value? Why would the others be less important as a starting point?"

This exercise builds your intuition for architecture prioritization.

