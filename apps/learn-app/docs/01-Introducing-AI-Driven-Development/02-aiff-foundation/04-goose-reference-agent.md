---
sidebar_position: 4
title: "goose - A Reference Agent"
description: "Understanding goose - Block's open source MCP-native agent framework"
keywords: [goose, Block, MCP, agent framework, reference implementation, open source]
chapter: 2
lesson: 4
duration_minutes: 12

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Reference Implementations"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why studying reference implementations accelerates learning and reduces risk"

  - name: "Distinguishing General vs Custom Agents"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can classify agents as Path A (General) or Path B (Custom) and explain when to use each"

  - name: "Identifying Production Patterns"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify architectural patterns in goose that transfer to Custom Agent development"

learning_objectives:
  - objective: "Explain why goose matters even if you use Claude Code as your primary tool"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Articulation of reference architecture value"

  - objective: "Distinguish between goose and Claude Code in the Two Paths framework"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Correct classification as General Agents validating same standards"

  - objective: "Identify goose patterns to apply when building Custom Agents"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Recognition of MCP-native, local-first, multi-model patterns"

cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (reference implementation, General vs Custom Agents, production patterns, enterprise validation) within A1-A2 limit (5-7 concepts) ✓"

differentiation:
  extension_for_advanced: "Clone goose repository; read the MCP client implementation; trace how tool calls flow through the system"
  remedial_for_struggling: "Focus on the Two Paths distinction; understand goose as 'Claude Code but open source you can study'"
---

# goose - A Reference Agent

MCP tells your Digital FTE how to connect to tools. AGENTS.md tells it how to behave in different environments.

But what does a production agent that implements both actually look like?

You could read specifications. You could imagine architectures. Or you could study working code that handles the same problems you'll face—authentication, error handling, multi-model support, context management.

**goose answers this question with running code.**

goose is Block's open source AI agent—the same technology where 75% of Block engineers report saving 8-10+ hours every week. It's not a demo. It's not a prototype. It's production software running at enterprise scale, and you can read every line of its source code.

---

## Why Reference Implementations Matter

When you build Custom Agents (Part 6 of this book), you'll face questions like:

- How should I structure MCP client connections?
- How do I handle streaming responses?
- What's the right way to manage conversation context?
- How do I support multiple AI providers?

You could solve these from first principles. Or you could study how goose solved them—then adapt those patterns to your needs.

**Reference implementations reduce risk.** Block's engineers have already made mistakes, fixed bugs, and refined patterns over thousands of hours of real usage. goose embodies that learning.

| Learning Path | What You Get |
|---------------|--------------|
| **From specs only** | Correct but untested patterns |
| **From tutorials** | Simplified patterns that break at scale |
| **From goose** | Battle-tested patterns from enterprise use |

This doesn't mean copying goose blindly. It means understanding how production agents work before designing your own.

---

## goose in the Two Paths Framework

Remember Chapter 1's Two Paths?

| Path A: General Agents | Path B: Custom Agents |
|------------------------|----------------------|
| Ready-to-use agents you configure | Agents you build from SDKs |
| Claude Code, Gemini CLI, goose | Using OpenAI SDK, Claude SDK |
| Use immediately | Build for specific workflows |

**goose is a Path A agent.** Like Claude Code, it's a ready-to-use development assistant. You install it, configure MCP servers, and start working.

But goose has a unique advantage: **it's open source under Apache 2.0**.

This means:
- You can read exactly how it implements MCP
- You can see how it handles multi-model support
- You can trace how context flows through the system
- You can learn patterns before building your own

**When you're ready for Path B (Custom Agents), goose becomes your blueprint.**

---

## Key Architecture Patterns

goose demonstrates patterns you'll use in your Custom Agents:

### 1. Local-First Execution

goose runs on your machine. This matters more than convenience.

**Privacy:** Your code and data stay local. For enterprise clients with sensitive IP, this isn't optional—it's required.

**Control:** You decide what the agent can access. No data leaves your machine unless you explicitly connect remote services.

**Speed:** No round-trips to remote servers for basic operations. File reads, code analysis, and local builds happen instantly.

When you build Digital FTEs for enterprise clients, "where does the agent run?" is a design decision. goose shows local-first architecture.

### 2. MCP-Native Design

goose was one of the first agents to implement MCP comprehensively. Adding capabilities means connecting MCP servers:

```bash
# Add file system access
goose mcp add filesystem

# Add database connectivity
goose mcp add postgres

# Add your custom server
goose mcp add ./my-server
```

**No custom integration code.** Every capability follows the same pattern. This is the extensibility model your Digital FTEs will use.

### 3. Multi-Model Support

goose doesn't lock you into one AI provider:

| Provider | Models Available |
|----------|-----------------|
| Anthropic | Claude 3.5, Claude 4 |
| OpenAI | GPT-4, GPT-5 |
| Google | Gemini 1.5, Gemini 2 |
| Local | Ollama, llama.cpp |

You can even configure different models for different tasks—use a cheaper model for simple operations, premium model for complex reasoning.

**Why this matters for your Agent Factory:** When you sell Digital FTEs, clients may have model preferences or restrictions. Multi-model support means your agent adapts to their requirements.

---

## Enterprise Validation

goose isn't theoretical. Block deployed it across their engineering organization:

| Metric | Result |
|--------|--------|
| **Engineer adoption** | 75% report 8-10+ hours saved weekly |
| **Growth rate** | Adoption doubled in one month |
| **Engagement** | 40-50% weekly engagement growth |
| **Code generation** | One engineer: "90% of my lines are written by goose" |

These aren't pilot project numbers. This is enterprise-scale adoption across thousands of engineers.

**What this validates:**
- MCP-native architecture works in production
- Local-first execution meets enterprise security needs
- Multi-model support is practical, not just theoretical
- The patterns scale beyond demos

When you present your Digital FTE to enterprise clients, goose provides precedent: "Block runs this architecture at scale. Here's the evidence."

---

## goose vs Claude Code

Both goose and Claude Code are General Agents. They're not competitors—they're validation of the same standards.

| Aspect | Claude Code | goose |
|--------|-------------|-------|
| **Creator** | Anthropic | Block |
| **License** | Proprietary | Open Source (Apache 2.0) |
| **MCP Support** | Yes | Yes |
| **AGENTS.md Support** | Yes | Yes |
| **Default Model** | Claude | Configurable |
| **Source Code** | Closed | Open |

**What you can do with Claude Code:**
- Get the best Anthropic-optimized experience
- Access latest Claude features immediately
- Work without thinking about implementation

**What you can do with goose:**
- Read the source code
- Understand MCP client implementation
- Study context management patterns
- Learn multi-model architecture
- Modify for your needs (Apache 2.0)

**The insight:** Use Claude Code for productivity today. Study goose for building Custom Agents tomorrow.

---

## goose in the AAIF Context

goose is an AAIF founding project alongside MCP and AGENTS.md:

| Project | Creator | Purpose |
|---------|---------|---------|
| MCP | Anthropic | How agents connect to tools |
| AGENTS.md | OpenAI | How agents adapt to environments |
| goose | Block | What a production agent looks like |

Block contributed goose to neutral Linux Foundation governance. This means:
- The project evolves with community input
- No single company controls its direction
- Standards integration will deepen over time

goose isn't just Block's tool anymore—it's part of the open agentic AI ecosystem. When you build on these patterns, you're building on stable, community-governed infrastructure.

---

## Your Agent Factory Blueprint

As you progress through this book, goose serves different purposes:

**Part 2 (AI Tool Landscape):** Use goose alongside Claude Code. Experience how MCP servers work in practice.

**Part 5 (Custom MCP Servers):** Study how goose consumes MCP servers. Understand client-side patterns.

**Part 6 (Custom Agents):** Apply goose's architecture patterns:
- Local-first execution for enterprise clients
- MCP-native design for extensibility
- Multi-model support for flexibility

The fact that goose exists and is open source accelerates your learning. You're not inventing—you're adapting proven patterns.

---

## Getting Started with goose

goose is available in multiple forms:

| Form | Best For |
|------|----------|
| **Desktop App** | Visual interface, casual exploration |
| **CLI** | Developer workflows, scripting |
| **MCP Server Registry** | Pre-approved enterprise servers |

Installation:
```bash
# macOS
brew install block/goose/goose

# Or via pipx
pipx install goose-ai
```

*Installation commands vary by platform—see goose documentation for your OS.*

Configuration:
```bash
# Set your preferred model provider
goose configure provider anthropic

# Add MCP servers
goose mcp add filesystem
goose mcp add github
```

*These commands configure goose to use Anthropic models and add filesystem/GitHub MCP servers.*

The goal isn't to replace Claude Code—it's to understand how production agents work. Even installing goose and exploring its commands teaches patterns.

---

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini, or similar) to explore:

### Prompt 1: Understand Reference Implementation Value

```
I'm learning about goose as a "reference implementation" for AI agents.

Help me understand:
1. What specifically can I learn from reading goose's source code that I couldn't learn from documentation alone?
2. If I plan to build my own Custom Agents, what parts of goose should I study first?
3. What mistakes might I make if I built agents without studying existing implementations?
```

**What you're learning:** Meta-learning skills. Understanding how to learn from existing code is itself a skill that compounds.

### Prompt 2: Compare Agent Architectures

```
Compare goose and Claude Code as AI coding agents:

1. They both support MCP and AGENTS.md—what does this prove about the standards?
2. What can I do with goose (open source) that I can't do with Claude Code (proprietary)?
3. If I were building a Digital FTE for enterprise clients, how might I use insights from both?

I want to understand trade-offs, not just features.
```

**What you're learning:** Comparative analysis. Understanding multiple implementations of the same standards deepens your grasp of the patterns themselves.

### Prompt 3: Apply Enterprise Patterns

```
I want to build a Digital [your role] for enterprise clients.

Based on goose's architecture patterns:
- Local-first execution
- MCP-native design
- Multi-model support

Help me think through:
1. Which pattern matters most for my use case? Why?
2. What enterprise requirements would push me toward each pattern?
3. What questions should I ask potential clients to determine which patterns they need?
```

**What you're learning:** Pattern application. Moving from "here's what exists" to "here's how I'll use it" is the bridge from learning to building.
