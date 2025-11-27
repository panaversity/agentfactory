---
title: "The Agent SDK Landscape"
sidebar_position: 7
description: "Survey the major agent development frameworks: OpenAI Agents SDK, Google ADK, Anthropic Agents Kit, and LangChain. Understand framework selection factors and recognize concept transfer across platforms."
proficiency_level: B1
cognitive_load:
  new_concepts: 9
  estimated_difficulty: B1
estimated_time: 30 minutes
learning_objectives:
  - "Name 4+ agent frameworks and describe their primary design philosophy"
  - "Articulate 2-3 distinguishing characteristics of each framework"
  - "Understand framework selection factors for different use cases"
  - "Recognize that core concepts (tools, orchestration, memory) transfer across frameworks"
  - "Identify production-relevant features like monitoring, security, and deployment support"
skills:
  framework_comparison:
    proficiency: B1
  sdk_selection_reasoning:
    proficiency: B1
  conceptual_transferability:
    proficiency: B1
generated_by: content-implementer v1.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 1.0.0
---

# The Agent SDK Landscape

You've learned what agents are, how they work architecturally, what patterns guide their design, and how to operate them reliably. Now comes a practical question: which framework should you actually use to build one?

This lesson surveys the major agent development frameworks available today. By the end, you'll understand the ecosystem well enough to make informed decisions and recognize that the core concepts from Lessons 1-6 transfer across all of them. The framework you choose is less important than understanding that same architectural principles appear everywhere.

## The Framework Question: Why It Matters

When you decide to build your first agent, you'll encounter a landscape of SDKs, each with a different design philosophy and set of strengths. The question isn't "which is best"—it's "which is right for this project, this team, this constraint set?"

Consider this decision scenario from real agent development teams:

**Your startup's decision context**:
- Team expertise: Strong in Python, using OpenAI GPT-4
- Use case: Customer support agent that integrates with your Salesforce instance
- Constraint: Need deployment within 2 weeks
- Requirements: Real-time feedback from users, continuous improvement based on customer interactions

Which framework would you choose? To answer that question well, you need to understand what each framework excels at—and what tradeoffs come with each choice.

This lesson gives you that mental map.

---

## Framework 1: OpenAI Agents SDK

**Philosophy**: Simplicity first, tight integration with GPT models.

**What it is**: OpenAI's official toolkit for building agents that use their models (GPT-4, GPT-4 Turbo). The SDK emphasizes developer experience—getting a working agent running quickly, with minimal configuration overhead.

### Distinguishing Characteristics

**1. Model Integration**: The framework is deeply optimized for OpenAI models, particularly GPT-4. Function calling (the mechanism for connecting the model to tools) is a first-class feature, since OpenAI developed this capability.

**Real scenario**: When you define a tool in the OpenAI Agents SDK, the framework automatically handles model-specific function calling formatting. The model understands your tool definitions natively because OpenAI built both the model and the framework.

**2. Developer Experience**: The API is intentionally minimal. You define your agent with a few Python objects: a model, a system prompt, tool definitions, and you're ready to go. There's less configuration ceremony than some alternatives.

**Code philosophy** (illustrative, not a code example):
- Model selection is straightforward
- Tools are defined with simple decorators or function wrapping
- The execution loop is managed automatically

**3. Production Features**: OpenAI has invested in production-grade infrastructure:
- **Structured outputs**: Agents can return reliably formatted responses (JSON schemas)
- **Logging and tracing**: Built-in support for understanding what your agent is doing (essential for Agent Ops)
- **Rate limiting and cost controls**: Production concerns are baked in

### When to Use

OpenAI Agents SDK is the right choice when:
- Your team already uses OpenAI models and has API relationships established
- You need fast prototyping (2-week timeline, like the startup scenario above)
- Your use case is primarily conversational (customer support, documentation agents, research assistants)
- You want deep integration with OpenAI's ecosystem (which continuously gets new features)

### Transferability Note

The OpenAI framework demonstrates the 3+1 architecture you learned in Lesson 2:
- **Model** ("Brain"): Explicitly selected (GPT-4)
- **Tools** ("Hands"): Function calling definitions
- **Orchestration** ("Nervous System"): Managed by the framework (planning, reasoning loop)
- **Deployment** ("Body"): OpenAI's API infrastructure

When you learn other frameworks, you'll recognize these same components, even if they're labeled differently or configured differently.

---

## Framework 2: Google ADK (Agent Development Kit)

**Philosophy**: Production-grade complexity, enterprise features, comprehensive platform.

**What it is**: Google's toolkit for building agents on Google Cloud. The ADK is designed for teams and enterprises building agents that need to integrate with existing cloud infrastructure, require sophisticated monitoring, or demand security-critical deployments.

The Google "Introduction to Agents" whitepaper that anchors this entire chapter emphasizes the ADK because it represents Google's vision for agent development at scale.

### Distinguishing Characteristics

**1. Deployment and Scalability**: The ADK is built to connect to Google Cloud's full infrastructure. This matters for production systems:
- Agents can run on Vertex AI (Google's managed ML platform)
- Automatic scaling for variable workloads
- Integration with Google Cloud's identity and access management (critical for security)

**Real scenario**: A financial services company builds an agent to analyze market data and recommend trades. The agent needs to:
- Process 10,000 requests per day (but with variable hourly spikes)
- Meet strict security requirements (agent can only access approved data)
- Integrate with existing data warehouses on Google Cloud

The ADK's Vertex AI deployment handles the scaling automatically. Google Cloud's IAM ensures the agent only accesses what it should.

**2. Built-In Memory and State Management**: The ADK includes sophisticated mechanisms for the orchestration layer (from the 3+1 architecture):
- Short-term context (current conversation)
- Long-term memory (persistent knowledge across conversations)
- Vector databases for semantic search and retrieval

These are production concerns that toy frameworks skip. Serious agent systems need reliable memory.

**3. MCP Integration**: The ADK natively supports Model Context Protocol (MCP), a standard for connecting agents to external systems. This is important because it means your tools aren't locked to Google's ecosystem—you can compose tools from multiple providers.

**Real scenario**: You're building an agent that needs:
- Google Cloud data (BigQuery datasets)
- Salesforce CRM access
- Slack integration
- Custom internal APIs

With MCP support, you can connect all of these through a standard protocol rather than writing custom integrations for each.

**4. Agent Ops Features**: The paper emphasizes that Google built evaluation, debugging, and improvement infrastructure directly into the ADK:
- Built-in support for LM-as-Judge evaluation
- Traces and observability for understanding agent behavior (from Lesson 5)
- Integration with Vertex AI's experiment tracking

### When to Use

Google ADK is the right choice when:
- Your organization uses Google Cloud (or is comfortable adopting it)
- You need sophisticated monitoring and production reliability
- Your use case requires integration with multiple external systems (MCP is valuable here)
- You want evaluation and improvement infrastructure built in
- Your team values comprehensive documentation and structured learning (Google's documentation is detailed)

### Transferability Note

The ADK explicitly implements the **Agent Ops** discipline you learned in Lesson 5:
- Evaluation mechanisms are first-class features
- Traces and debugging are built-in
- The feedback loop from human corrections back to agent improvement is supported

This transfer of Lesson 5 concepts to the ADK's implementation demonstrates how frameworks instantiate the abstract patterns you've learned.

---

## Framework 3: Anthropic Agents Kit

**Philosophy**: Safety-first reasoning, extended thinking, alignment-focused development.

**What it is**: Anthropic's toolkit for building agents that prioritize safety and interpretability. This framework emphasizes agents you can understand and trust, with capabilities for deep reasoning.

### Distinguishing Characteristics

**1. Extended Thinking**: Anthropic introduced "extended thinking"—the ability for Claude to reason extensively before responding. This is a significant architectural difference:

**Real scenario**: You're building an agent for medical research assistance. Doctors need to trust the reasoning, not just the answer. Extended thinking allows the agent to show its work:
- Problem interpretation (what is the doctor actually asking?)
- Literature search strategy (which research should we look for?)
- Evidence evaluation (how strong is this study?)
- Synthesis (what's the reliable conclusion?)

The extended thinking output is transparent—doctors can audit the reasoning and catch errors before acting on the agent's recommendation.

**2. Constitutional AI Principles**: Anthropic's framework includes mechanisms for embedding values and constraints into agents:
- Agents can be trained to follow specific ethical guidelines
- Tool use is safety-aware (agents understand which tools are safe to call and under what conditions)
- Outputs are evaluated against value criteria (helpful, harmless, honest)

**Real scenario**: A content moderation agent must reject harmful requests while helping legitimate users. The Constitutional AI approach lets you define:
- What kinds of content are unacceptable
- How the agent should respond when encountering them
- What transparency looks like

**3. Tool Use Safety**: The framework includes mechanisms for evaluating tool safety:
- Can this agent safely call this API?
- Does the agent understand the real-world consequences of this tool call?
- What guardrails should we add?

### When to Use

Anthropic Agents Kit is the right choice when:
- Your use case requires transparent, auditable reasoning (medical, legal, financial decisions)
- Safety and alignment matter as much as capability (high-stakes applications)
- You want agents that can show their work and justify their decisions
- Your team is already using Claude and has API relationships with Anthropic
- You value Anthropic's research on AI safety and alignment

### Transferability Note

The Anthropic kit demonstrates how **orchestration** (from the 3+1 architecture) can be designed with safety as a first-class concern:
- Planning isn't just "make a step-by-step plan," it's "make a plan we can understand and verify"
- Tool selection isn't just "call this function," it's "is this tool safe to call in this context?"

This shows how the same architectural patterns can be implemented with different values and priorities.

---

## Framework 4: LangChain

**Philosophy**: Model-agnostic flexibility, composability, tooling ecosystem.

**What it is**: LangChain is a popular open-source framework that treats the agent as a composable system. Rather than being tied to a specific model or cloud platform, LangChain lets you build agents that can use any model (OpenAI, Google, Anthropic, open-source models, etc.) and compose tools from diverse sources.

### Distinguishing Characteristics

**1. Model Flexibility**: LangChain doesn't care which model you use. This is valuable in several scenarios:

**Real scenario 1 - Cost optimization**: You prototype with GPT-4 (more capable), but production uses a cheaper open-source model (Llama, Mistral). LangChain lets you swap models without rewriting your agent.

**Real scenario 2 - Regulatory requirements**: Some jurisdictions require data to stay within borders. You might prototype with Claude (which uses Anthropic's servers), but production uses a self-hosted open-source model. LangChain's abstraction layer makes this transition smooth.

**Real scenario 3 - Multi-model agents**: You use Claude for reasoning tasks, GPT-4 for specialized domain knowledge, and Llama for cost-sensitive operations. LangChain lets you compose these together.

**2. Extensive Tool Ecosystem**: LangChain has integrations with hundreds of external services:
- Database queries (SQL, vector databases)
- APIs (Slack, GitHub, Salesforce, etc.)
- Search engines (Google Search, DuckDuckGo)
- Code execution environments

This ecosystem means you don't have to write custom integrations. The tool you need probably already exists.

**Real scenario**: You're building an agent that needs to query a PostgreSQL database and search the web. Both integrations already exist in LangChain. You compose them together rather than writing from scratch.

**3. Open-Source and Community-Driven**: LangChain is open-source, which means:
- You can inspect the code and modify it if needed
- Community extensions add new capabilities constantly
- You're not locked into a single company's roadmap

**Tradeoff**: The breadth and flexibility come at a cost. LangChain is more complex than OpenAI's SDK. You have more knobs to turn, which is powerful but requires more understanding.

### When to Use

LangChain is the right choice when:
- You need flexibility across multiple models (not locked to one provider)
- You want to compose tools from diverse sources
- Your team values open-source and wants to avoid vendor lock-in
- You're building a complex agent that needs sophisticated composition
- Your use case doesn't fit neatly into any single company's framework

### Tradeoff Recognition

LangChain demonstrates an important principle: **flexibility and simplicity are often in tension**. OpenAI's SDK is simpler because it makes assumptions (you're using OpenAI models). LangChain is more flexible because it doesn't make those assumptions—but that flexibility requires more configuration.

---

## Framework Comparison Matrix

To help with decision-making, here's a structured comparison across key dimensions:

| Dimension | OpenAI SDK | Google ADK | Anthropic Kit | LangChain |
|-----------|-----------|-----------|---------------|-----------|
| **Primary Strength** | Simplicity, fast prototyping | Production scale, enterprise features | Safety, transparency, reasoning | Flexibility, model agnosticism |
| **Model Lock-In** | OpenAI only | Google models + others | Anthropic (Claude) | Any model |
| **Deployment** | OpenAI API | Google Cloud (Vertex AI) | Anthropic API | Self-hosted or any cloud |
| **Evaluation/Ops** | Basic logging | Comprehensive (built-in) | Safety-focused | Community extensions |
| **Complexity** | Low | High | Medium | High |
| **Ease of Learning** | Easiest | Steepest | Medium | Medium-High |
| **Production Readiness** | Good (for simple cases) | Excellent | Good (safety-focused) | Good (flexibility focus) |
| **Tool Ecosystem** | Moderate | Google Cloud services | Anthropic tools | Extensive (hundreds) |
| **Open Source** | No | No | No | Yes |
| **Security/Safety Features** | Standard | Enterprise-grade IAM | Constitutional AI | Community-provided |

**Reading this matrix**: Each framework excels in different dimensions. There's no "objectively best" choice—the best choice depends on your specific constraints, team expertise, and use case.

---

## How to Choose: A Decision Framework

When you face the framework decision, ask these questions in order:

**Question 1: Model Constraints**

"Which model(s) can I use?"

- **Only OpenAI available** → OpenAI SDK
- **Only Google available** → Google ADK
- **Only Anthropic (Claude) available** → Anthropic Kit
- **Flexibility across models required** → LangChain
- **Using Anthropic but value safety/transparency** → Anthropic Kit (even if other models available)

**Question 2: Deployment Context**

"Where does this agent need to run?"

- **OpenAI API directly** → OpenAI SDK
- **Google Cloud platform** → Google ADK
- **Own infrastructure required** → LangChain or self-hosted Anthropic
- **Enterprise data center** → Google ADK or LangChain (self-hosted)

**Question 3: Feature Priority**

"What features matter most?"

- **Simplicity and speed to prototype** → OpenAI SDK
- **Production monitoring and evaluation** → Google ADK
- **Safety and interpretable reasoning** → Anthropic Kit
- **Tool composition and flexibility** → LangChain

**Question 4: Team Expertise**

"What does your team know well?"

- **Familiar with OpenAI ecosystem** → OpenAI SDK (reduces learning curve)
- **Google Cloud engineers on staff** → Google ADK (leverage existing knowledge)
- **Safety and alignment expertise valued** → Anthropic Kit
- **Full-stack engineering culture** → LangChain (the flexibility payoff requires more skill)

**Real Decision Example**: Returning to the startup scenario from earlier—customer support agent, 2-week timeline, OpenAI models, needs continuous improvement:

1. Model constraints: OpenAI available → OpenAI SDK likely
2. Deployment: API-based (not on-premises) → OpenAI SDK confirmed
3. Feature priority: Speed to prototype + basic ops (understanding failures) → OpenAI SDK
4. Team: Already using OpenAI → OpenAI SDK decision confirmed

The decision framework points toward OpenAI SDK, but if that team had Google Cloud infrastructure already in place, the ADK's production features might overcome the prototyping speed advantage.

---

## A Critical Insight: Concepts Transfer, Implementations Differ

Here's the most important principle to internalize: **the core concepts you learned in Lessons 1-6 appear in every framework.**

You've learned:

- **3+1 Architecture** (Model + Tools + Orchestration + Deployment)
- **5-Step Loop** (Get Mission → Scan → Think → Act → Observe)
- **Tool Integration** (connecting reasoning to action)
- **Memory and Context** (managing the state the agent operates with)
- **Agent Ops** (evaluation, debugging, improvement)

Every framework implements these same components. They're named differently, configured differently, and integrated differently—but the architectural principles are universal.

**Practical implication**: When you learn OpenAI's SDK and later need to use Google's ADK for a project, you won't be learning agent development from scratch. You'll be learning how ADK implements the same patterns you already understand, with different tooling.

This is why learning Chapter 33 (conceptual foundations) before Chapter 34+ (SDK hands-on) matters. You're learning the timeless patterns once. The SDK details are secondary.

---

## Practical Consideration: Switching Costs and Vendor Lock-In

When choosing a framework, a real concern is switching costs. What happens if you build an agent in OpenAI's SDK and later need to move to Google's platform?

**The good news**: The architectural patterns transfer. Your knowledge of what an agent *is* and how it *works* transfers completely.

**The realistic news**: Implementation details don't transfer. You'll need to rewrite the code, reconfigure deployments, and rebuild integrations. This is a real cost.

**How to minimize**:

- Separate architectural decisions (5-step loop, tool definitions, memory strategy) from implementation details (which SDK, which models, which cloud)
- Document architectural decisions explicitly (ADR documents from Chapter 13 are valuable here)
- Keep implementation logic modular—avoid deeply embedding framework-specific patterns throughout your codebase

LangChain's approach (model-agnostic abstraction layer) is designed specifically to mitigate this risk. You trade simplicity for flexibility and portability.

---

## What's NOT in This Survey

This lesson deliberately doesn't cover:

- **Emerging frameworks** (new SDKs launch constantly; the landscape shifts rapidly)
- **Deep implementation details** (that's Chapters 34-36)
- **Specialized frameworks** (domain-specific agent tools, vertical solutions)
- **Custom in-house frameworks** (some large companies build their own)

This survey covers the major, production-proven frameworks available at the time of writing. As you move into Chapters 34-36, you'll implement using specific SDKs and develop hands-on understanding.

---

## Try With AI

**Setup**: Open ChatGPT, Claude, or Gemini and explore agent framework characteristics through dialogue. The goal is to deepen your understanding through conversation rather than just reading.

**Prompt 1 — Framework Decision**:

```
I'm building an agent for medical research assistance.
The constraints are:
- Team expertise: Python, familiar with OpenAI
- Use case: Doctors query research findings and get evidence-based recommendations
- Requirement: Results must show reasoning transparently (doctors need to audit the logic)

Based on the agent frameworks (OpenAI SDK, Google ADK, Anthropic, LangChain),
which would you recommend? What are the tradeoffs?
```

**Expected Outcome**: The AI will likely recommend Anthropic (for reasoning transparency) or acknowledge the tradeoff between OpenAI (simplicity, team familiarity) and Anthropic (transparency). Use the response to test your own understanding—do their recommendations match what you learned about framework strengths?

**Prompt 2 — Concept Transfer**:

```
I'm learning agent frameworks and want to understand portability.
If I build an agent using OpenAI's SDK that implements:
- A 5-step problem-solving loop (Get → Scan → Think → Act → Observe)
- Tools for code execution, database queries, and API calls
- Memory for conversation history

How much of this knowledge transfers if I later need to reimplement in Google's ADK?
What stays the same? What changes?
```

**Expected Outcome**: You're testing the transferability principle. The AI should confirm that architectural patterns transfer but implementations require rewriting. This should feel validating—Chapter 33 knowledge is indeed durable across SDKs.

**Prompt 3 — Production Scenario** (Optional, Advanced):

```
Our team has built agents in OpenAI's SDK for the past year.
Now we have a new requirement: the agent needs to operate in an air-gapped environment
(no internet access, on-premise only).

How would switching to a different framework help?
What would we need to do?
```

**Expected Outcome**: The AI will discuss self-hosted options (LangChain with local models, or proprietary on-premise deployments). This explores a real constraint that framework choice impacts directly.

**Safety Note**: When exploring frameworks and deployment options, verify current documentation against what the AI suggests. Framework ecosystems evolve, and cloud platform capabilities change frequently. Use the AI as a thinking partner, but validate specifics in official documentation before making architecture decisions.

**Optional Stretch**: Research and compare the latest framework releases (Chapter 34-36 may reference newer versions). How do new features in your chosen framework compare to the landscape presented here?

