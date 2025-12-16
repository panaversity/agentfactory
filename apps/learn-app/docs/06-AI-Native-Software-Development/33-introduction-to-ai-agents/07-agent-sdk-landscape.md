---
title: "The Agent SDK Landscape"
sidebar_position: 7
description: "Survey major agent frameworks and understand how to choose between them."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 20 minutes
learning_objectives:
  - "Name major agent frameworks and their design philosophies"
  - "Apply selection criteria to choose frameworks for specific projects"
  - "Recognize that core concepts transfer across all frameworks"
skills:
  framework_selection:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# The Agent SDK Landscape

You've learned what agents are and how they work. Now comes a practical question: which framework should you use to build one?

This lesson surveys the four major agent development frameworks you'll master in Chapters 35-38. Each has a distinct philosophy—understanding these differences helps you match frameworks to problems.

## The Framework Question

When you decide to build an agent, you'll encounter SDKs with fundamentally different approaches. The question isn't "which is best?" It's "which philosophy matches my problem?"

In an **AIDD/SDD workflow**, you don't manually write boilerplate. You write a **Specification** defining the agent's Persona, Tools, Workflow, and Guardrails—then use an AI Architect to generate the implementation for your chosen SDK. Understanding each framework's philosophy tells you what your spec should emphasize.

## Four Major Frameworks

| Framework | Core Philosophy | SDD Spec Focus | Best For |
|-----------|-----------------|----------------|----------|
| **OpenAI Agents SDK** | Handoff-Centric | Handoff networks, guardrails | Routing/triage workflows |
| **Google ADK** | Service-Centric | Artifact schemas, A2A protocols | Stateful business processes |
| **Anthropic Agents Kit** | Capability-Centric | MCP tool definitions, system prompts | Task execution (coding, research) |
| **Microsoft Agent Framework** | Conversation-Centric | Group interaction patterns, plugins | Multi-agent collaboration |

### OpenAI Agents SDK (Chapter 35)

**Philosophy**: Handoff-Centric—simple agents transferring control like a relay race.

OpenAI's approach focuses on **handoffs**. An agent is a configuration that can "transfer" the conversation to another agent when appropriate.

**Architecture**: Lightweight Python/Node implementation using "Handoff" primitives. Define a Triage Agent and Specialist Agents, with explicit rules for when Agent A passes control to Agent B.

**SDD Spec Focus**: Your spec defines the **Router Pattern**—which specialists exist, and the `handoff_instructions` that govern transfers.

```
agent = Agent(
  name="Support",
  instructions="...",
  handoffs=[sales_agent, tech_agent]
)
```

**When to use**: Customer support, simple workflows where control transfers between specialists. Best for developers who want a "keep it simple" approach without complex graph orchestration.

### Google ADK (Chapter 36)

**Philosophy**: Service-Centric—agents as microservices with strict state management.

Google ADK treats agents like **software components**. It emphasizes **Artifacts**—structured objects that store state—and agents that manipulate them through well-defined interfaces.

**Architecture**: Engineering-heavy structure with strong typing (TypeScript/Pydantic), state management hooks, and the **Agent2Agent (A2A)** protocol for agent-to-agent communication. Integrates with Google Cloud (Vertex AI, Firebase Genkit).

**SDD Spec Focus**: Your spec defines **data schemas**—what Artifacts the agent manipulates, `input_schema`, `output_schema`, and `persistence_layer`.

```
ai.defineFlow(...) with strict input/output Zod schemas
```

**When to use**: Enterprise applications needing strict state management, business processes with data requirements (loan processing, order management), Google Cloud integration. Best for "engineering-first" teams building robust, stateful systems.

### Anthropic Agents Kit (Chapter 37)

**Philosophy**: Capability-Centric—single powerful agents with deep tool and computer use.

Anthropic's ecosystem centers on the **Claude Agent SDK** and **Model Context Protocol (MCP)**. It assumes the model is smart enough to handle complex flows if given the right tools.

**Architecture**: Heavy reliance on MCP servers (connectors) and "Computer Use" capabilities. You don't script the flow—you script the *environment* the agent lives in.

**SDD Spec Focus**: Your spec defines **capabilities**—the tools the agent accesses (FileSystem, GitHub API, databases). The key attribute is `MCP Server Configuration`.

```
Client connects to mcp-server; Claude acts, observes tool output, acts again
```

**When to use**: Complex execution tasks—agents that actually *do work* (coding, file editing, research, data analysis) rather than just routing conversations. Best for "super-worker" agents that operate computers and tools.

### Microsoft Agent Framework (Chapter 38)

**Philosophy**: Conversation-Centric—multi-agent collaboration and group chats.

Microsoft's framework combines **Semantic Kernel** (enterprise integration) and **AutoGen** (multi-agent orchestration). It enables complex "Group Chats" where multiple agents collaborate to solve problems.

**Architecture**: Multiple agent classes coordinated by a Group Chat Manager. Agents speak to each other in loops with defined interaction rules and termination criteria.

**SDD Spec Focus**: Your spec defines **social dynamics**—roles (Manager, Coder, Reviewer), rules of engagement ("Reviewer must approve before Coder commits"), `GroupChatSelectionMethod`, and `TerminationCriteria`.

```
group_chat = GroupChat(agents=[user_proxy, coder, pm], max_round=12)
manager = GroupChatManager(groupchat=group_chat)
```

**When to use**: Complex problem-solving requiring multiple perspectives, team simulations, brainstorming processes, enterprise process automation. The gold standard for simulating a "digital workforce."

## Framework Comparison Matrix

| Dimension | OpenAI Agents SDK | Google ADK | Anthropic Agents Kit | Microsoft Framework |
|-----------|-------------------|------------|---------------------|---------------------|
| **Core Pattern** | Handoffs | Artifacts/State | MCP Tools | Group Chats |
| **Spec Emphasis** | Transfer rules | Data schemas | Tool definitions | Interaction rules |
| **Complexity** | Low | High | Medium | High |
| **State Management** | Minimal | Strict | Tool-dependent | Conversation-based |
| **Multi-Agent** | Sequential handoffs | A2A protocol | Single agent focus | Native group chat |
| **Best Strength** | Simplicity | Enterprise rigor | Deep execution | Collaboration |

## How to Choose: Match Philosophy to Problem

### Problem Type → Framework

| Your Problem | Best Framework | Why |
|--------------|----------------|-----|
| **Routing/Triage** (customer service) | OpenAI Agents SDK | Handoff pattern matches naturally |
| **Business Process** (strict data requirements) | Google ADK | Artifact schemas enforce state |
| **Task Execution** (coding, research) | Anthropic Agents Kit | MCP enables deep tool use |
| **Team Collaboration** (brainstorming, review) | Microsoft Framework | Group chat orchestration |

### Decision Questions

**Question 1: What's the core interaction pattern?**
- Handing off between specialists → **OpenAI**
- Manipulating structured data → **Google ADK**
- Operating tools and computers → **Anthropic**
- Multiple agents discussing/debating → **Microsoft**

**Question 2: What does your spec naturally describe?**
- Transfer conditions ("when X, hand to Y") → **OpenAI**
- Data schemas and state transitions → **Google ADK**
- Tool capabilities and permissions → **Anthropic**
- Roles, rules of engagement, termination → **Microsoft**

**Question 3: What's your deployment context?**
- Quick MVP, simple flows → **OpenAI**
- Google Cloud, enterprise requirements → **Google ADK**
- Complex autonomous work → **Anthropic**
- Multi-perspective problem solving → **Microsoft**

## The Critical Insight: Concepts Transfer

**Core concepts from Lessons 1-6 appear in every framework:**

- **3+1 Architecture**: All frameworks have Model, Tools, Orchestration, Deployment—implemented differently
- **5-Step Loop**: Every agent reasons, acts, observes, iterates—the primitives vary
- **Multi-agent patterns**: Coordinator, Sequential, HITL exist across frameworks with different APIs
- **Agent Ops**: Evaluation and debugging principles apply universally
- **Security**: Guardrails and identity requirements transcend frameworks

When you learn OpenAI's handoff pattern and later need Google's artifact-based approach, you're not starting over. You're learning how ADK implements patterns you already understand.

**This is why Chapter 33 before Chapters 35-38 matters.** The conceptual foundation is framework-agnostic. Implementation details change; architectural thinking persists.

## Try With AI

Use Claude, ChatGPT, or Gemini to explore framework selection.

> "I'm building a customer support agent that routes inquiries to billing, technical, or sales specialists. Each specialist handles their domain, then control returns to triage. Which framework? Why?"

**Expected**: OpenAI Agents SDK—the handoff pattern matches exactly. Spec defines triage agent + specialists + transfer conditions.

> "I'm building an agent that processes loan applications: validate documents, check credit, calculate terms, generate approval letter. Strict data requirements, audit trail needed. Which framework?"

**Expected**: Google ADK—artifact-based state management, strict schemas, enterprise audit requirements match the service-centric philosophy.

> "I need an agent that can clone a GitHub repo, analyze the code, make improvements, run tests, and submit a PR. It needs to operate tools autonomously. Which framework?"

**Expected**: Anthropic Agents Kit—MCP tool definitions, computer use capabilities. Spec focuses on tool access, not conversation flow.

> "I want a product design simulation: a PM proposes features, an engineer estimates effort, a designer critiques UX, they iterate until consensus. Which framework?"

**Expected**: Microsoft Agent Framework—group chat with roles, interaction rules, termination criteria. This is multi-agent collaboration.

**Key insight**: The framework you choose shapes what your specification emphasizes. Match the framework's philosophy to your problem's structure.
