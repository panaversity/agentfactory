---
title: "Lesson 3: Two Paths to Building AI Products"
chapter: 1
lesson: 3
duration_minutes: 18
sidebar_position: 3
skills:
  agent-factory-paradigm:
    proficiency: A1-A2
    bloom_level: Understand, Apply
learning_objectives:
  - "Distinguish General Agents (multi-purpose tools) from Custom Agents (purpose-built products)"
  - "Explain how General Agents compose into Custom Agents (Agent Factory metaphor)"
  - "Identify when to use each approach for different development scenarios"
  - "Recognize the OODA loop as core reasoning architecture for both paths"
---

# Two Paths to Building AI Products

You've seen the evidence: 2024-2025 is genuinely different. You've understood the economic scale—a $3 trillion software economy being transformed. Now comes the crucial question that will shape how you think about AI development going forward:

**How do you actually build AI products?**

The answer surprises most developers: There isn't one path. There are two fundamentally different approaches, each with distinct tools, roles, and applications. Understanding which path to take—and when to transition between them—is the core strategic insight of the Agent Factory paradigm.

## Path A: General Agents

A **General Agent** is a multi-purpose reasoning system designed to handle ANY task you throw at it. Think of it as a flexible professional who can turn their hand to almost anything.

### The Tools

Three General Agents dominate 2025:

**Claude Code** (Anthropic)
- Your natural language interface to AI-native development
- Designed for exploration, prototyping, and iterative problem-solving
- Activates reasoning mode through extended thinking and artifact generation
- Built for human-in-the-loop collaboration

**Gemini CLI** (Google)
- Open-source, CLI-first approach to agentic development
- Lightweight, accessible from terminal or programmatic context
- Strong structured reasoning through function calling
- Community-driven ecosystem

**Goose** (Block/Cognition)
- Browser automation + code execution + reasoning
- Specialized for web-based tasks and visual understanding
- Growing ecosystem for agent composition
- Strong on multimodal reasoning

### Your Role: Director

When you use a General Agent, your role is **Director**. You:

- Specify intent clearly ("Build a registration system with validation")
- Let the agent handle tactical decisions (implementation, testing, optimization)
- Evaluate quality and provide feedback ("This doesn't handle rate limiting")
- Refine direction based on what emerges ("Actually, let's add email verification")

This is **zero-shot planning capability**: You don't need to pre-specify every detail. The agent reasons through the problem dynamically as new information surfaces.

### Reasoning Loop: OODA

Both General and Custom Agents operate through the **OODA loop**—a reasoning framework from military strategy that applies perfectly to AI systems:

1. **Observe**: Gather information (your prompt, current code, test results, error messages)
2. **Orient**: Interpret context (What problem are we solving? What constraints apply? What patterns fit?)
3. **Decide**: Choose approach (Which implementation strategy? What architecture? What tradeoffs?)
4. **Act**: Execute the decision (Generate code, run tests, commit changes, report back)

**What makes this powerful**: Good OODA loops are FAST. Claude Code completes this cycle in seconds. When you redirect ("Actually, I need it to handle X"), the agent re-orients instantly and produces new output.

**Why General Agents matter here**: General Agents are powerful precisely because they can OODA on ANY domain. Legal reasoning, robotics code, marketing copy, system architecture—same engine, applied to different contexts.

## Path B: Custom Agents

A **Custom Agent** is purpose-built for a specific workflow. Instead of zero-shot reasoning on anything, it's optimized for one job—and does that job better than a General Agent ever could.

### The Tools

Three SDKs for building Custom Agents:

**OpenAI Agents SDK**
- Built on OpenAI's function-calling and structured reasoning
- Integrates native with OpenAI models
- Mature tool ecosystem, strong for production workloads
- Growing market for packaged agents

**Anthropic Agents Kit** (Claude SDK)
- Anthropic's native agent framework
- Deep integration with Claude's reasoning capabilities
- Multi-turn conversation continuity and state management
- Strong for complex reasoning chains

**Google ADK** (Agentic Design Kit)
- Google's approach to structured agent design
- Emphasis on multimodal reasoning and vision
- Integration with Google's ecosystem (Search, Workspace, Cloud)
- Emerging framework gaining adoption

### Your Role: Builder

When you build Custom Agents, your role is **Builder**. You:

- Define the agent's purpose precisely (scope, constraints, success criteria)
- Build guardrails and safety constraints into the agent's design
- Create specialized prompts, tools, and workflows
- Deploy as a product others depend on

This is **deeply scoped purpose-building**: You're not asking the agent to figure out what to do. You've already decided what it does, and engineered it to do that reliably.

### Why Custom Agents Exist

General Agents are flexible—but that flexibility comes at a cost:

- **Slower**: Zero-shot reasoning takes more tokens (more latency, higher cost)
- **Less reliable**: Generic approaches miss domain-specific optimizations
- **Harder to govern**: Flexibility makes safety constraints harder to enforce
- **Not scalable for production**: Users expect consistent, optimized behavior

Custom Agents solve this by specializing:

- **Faster**: Trained on domain patterns, skip irrelevant reasoning
- **More reliable**: Guardrails prevent off-topic behavior
- **Governed**: Constraints are built-in, not learned
- **Production-ready**: Optimized for cost, latency, and user experience

## The Key Insight: General Agents BUILD Custom Agents

Here's where the "Agent Factory" concept becomes clear:

**General Agents don't compete with Custom Agents. General Agents BUILD Custom Agents.**

Think about the workflow:

### Step 1: Explore with a General Agent
You use Claude Code to prototype a customer support system. You iterate rapidly—Claude suggests patterns, you refine requirements, the solution emerges.

**Duration**: Exploration phase takes hours or days.

### Step 2: Transition to a Custom Agent
Once the pattern stabilizes and you understand the requirements, you take what you learned and build a Custom Agent using the OpenAI SDK:

```
Custom Agent Purpose: "Handle routine customer support queries"

Tools Available:
- Search knowledge base
- Create support ticket
- Send email notification
- Escalate to human

Constraints:
- Never answer pricing questions (escalate)
- All refund requests require human approval
- Response time: <2 seconds
- Token limit: 500 (keep responses concise)
```

This Custom Agent is faster, cheaper, and safer than having the General Agent handle every query.

### Step 3: Scale the Product
Your Custom Agent now runs in production, handling 1000+ support tickets daily. The General Agent's role shifts—you use it to:

- Analyze patterns in customer queries (improvement opportunities)
- Redesign the Custom Agent's knowledge base
- Build new Custom Agents for other support categories
- Optimize prompts and constraints based on performance data

**The Factory In Action**: The same General Agent that built your first Custom Agent now optimizes it, learns from its outputs, and architects the next generation.

## When to Use Each Path

This isn't a "which is better" question. It's a "which is appropriate right now" question.

### Use Path A (General Agent) When:

- **Exploring unknown problems** ("What does a good registration system look like?")
- **Prototyping quickly** (Get from idea to working code in hours)
- **Learning by iteration** (Try approaches, learn what works, refine)
- **Complex reasoning needed** (System design, architecture decisions, novel problems)
- **One-off solutions** (Tooling, internal utilities, experiments)

**Question to ask**: "Do I understand the problem well enough to define a Custom Agent?"
- If no → Use a General Agent to explore first

### Use Path B (Custom Agent) When:

- **Problem is well-defined** ("This support system must handle X, Y, Z reliably")
- **Repeated use** (Will this agent run 100+ times a day? 1000+?)
- **Production environment** (Users depend on consistency and reliability)
- **Cost matters** (Every prompt costs money—optimization is worth engineering)
- **Safety is critical** (Must prevent certain behaviors, enforce constraints)

**Question to ask**: "Would specialized engineering of this workflow pay for itself?"
- If yes (repeated use, scale, safety) → Build a Custom Agent

## A Development Lifecycle Perspective

The practical workflow for most teams:

1. **General Agent Phase** (Claude Code)
   - Prototype the solution
   - Discover requirements through iteration
   - Build shared understanding with stakeholders
   - Identify patterns worth optimizing

2. **Translation Phase**
   - Extract lessons from prototype
   - Define Custom Agent specifications
   - Plan deployment architecture

3. **Custom Agent Phase** (SDK of choice)
   - Build production-grade implementation
   - Add guardrails and safety constraints
   - Optimize for cost and latency
   - Deploy with monitoring

4. **Continuous Improvement Phase**
   - Use General Agent to analyze patterns
   - Refine Custom Agent based on real usage
   - Build adjacent Custom Agents
   - Architect increasingly sophisticated systems

This is the **Agent Factory paradigm in action**: General purpose reasoning powers the exploration and iteration that informs specialized, production-grade agents.

## The Mental Model You Need

As you move through this book, keep this distinction clear in your mind:

**General Agents** = Thinking partners who help you understand problems and build solutions
- Tools: Claude Code, Gemini CLI, Goose
- Your role: Director (specify intent, evaluate quality, redirect)
- Reasoning: OODA loop applied to ANY domain
- Outcome: Working solution, shared understanding, discovered requirements

**Custom Agents** = Specialized products deployed at scale
- Tools: OpenAI SDK, Claude SDK, Google ADK
- Your role: Builder (design, engineer, govern)
- Architecture: Purpose-built with guardrails and optimization
- Outcome: Production system, reliability, cost efficiency

And the insight tying them together:

**Claude Code (General Agent) is an Agent Factory**—it builds the Custom Agents you'll learn to construct in Parts 5-7 of this book.

This is why understanding both paths matters: You're learning to think as an AI-native developer who moves fluidly between exploration and production engineering, powered by understanding how these two approaches compose.

## Try With AI

Use this section to explore the two paths yourself through prompted reflection. These questions activate the mental models we've just established.

### Scenario 1: Internal Tool Decision

You need to build a system that helps your team extract customer insights from support tickets.

**Your questions:**
- Is this a one-time analysis or ongoing system?
- Could a General Agent explore the problem space first?
- At what point does it make sense to transition to a Custom Agent?
- What constraints would the Custom Agent need?

**Prompt 1**: Ask Claude Code: "What would a good approach look like for extracting customer insights from support ticket text? What patterns should we look for?"

Let Claude respond with exploration thoughts. Notice how it OODA's through the problem.

**Prompt 2**: "Now imagine we need this to run 100+ times daily, automatically. What would need to change about the solution?"

Observe how the conversation shifts from exploration to engineering constraints. This illustrates the General-to-Custom transition.

**Reflect**: At what point in this conversation did it become clear you'd need a Custom Agent instead of relying on the General Agent?

### Scenario 2: Product Feature Decision

You're building an AI-native product and need to decide whether to use a General Agent or build Custom Agents.

**Your thinking framework:**
- General Agents are appropriate for features where **flexibility and exploration** matter more than optimization
- Custom Agents are appropriate for features where **reliability and cost** drive decisions
- Most products use BOTH: General Agents for reasoning, Custom Agents for execution

**Prompt 3**: "I'm building [describe a feature you care about]. Should I use a General Agent approach or build Custom Agents? What factors should influence this decision?"

Pay attention to:
- How the response distinguishes scope vs. generality
- What guardrails look like for Custom Agents
- How cost-benefit analysis applies

**Reflect**: For a feature you work with daily, how would you apply this two-paths framework?

### Closing Question

Before moving to the next lesson, ask yourself:

**"Which path do I use Claude Code (General Agent) for right now? And what would trigger me to transition to building a Custom Agent?"**

Your answer reveals how deeply you've internalized the Agent Factory paradigm.
