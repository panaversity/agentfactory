---
sidebar_position: 6
title: "Lesson 6: MCP-UI & Apps SDK - Agent Interfaces"
description: "Understanding how agent interfaces are evolving beyond chat"
keywords: [MCP-UI, OpenAI Apps SDK, agent interfaces, ChatGPT apps, interactive UI]
---

# Lesson 6: MCP-UI & Apps SDK - Agent Interfaces

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand that agent interfaces are evolving beyond text chat
- Explain what the OpenAI Apps SDK enables
- Recognize MCP-UI as an emerging interface standard

---

## Beyond Chat

Chat interfaces are powerful. You type, the AI responds. Simple, flexible, natural.

But chat has limits.

- Want to show a data visualization? You describe it in text.
- Need a form with validation? The AI asks questions one at a time.
- Displaying a complex table? Hope your formatting is readable.

**What if agents could show you buttons, forms, charts, and interactive elements?**

This is where agent interfaces are heading: from pure text to rich, interactive experiences.

```
Evolution of Agent Interfaces

Text Only → Structured Output → Interactive Components
    ↓              ↓                    ↓
  Chat         Markdown/Code      Buttons, Forms,
              Formatting          Visualizations
```

---

## OpenAI Apps SDK: Use Today

The OpenAI Apps SDK lets you build applications that run inside ChatGPT.

Launched October 2025, Apps SDK combines:

- **MCP tools**: The agent's capabilities (what it can do)
- **Custom UI components**: Rich visual interfaces (what users see)
- **ChatGPT integration**: Distribution to millions of users

### What Apps Can Do

| Use Case | How It Works |
|----------|--------------|
| Order groceries | Browse products visually, add to cart, checkout |
| Create presentations | Interactive slide builder, not just text suggestions |
| Search apartments | Map views, filters, photo galleries |
| Manage projects | Kanban boards, timelines, team views |

Apps aren't just answering questions. They're providing full application experiences *inside the chat interface*.

### The Key Point

**Apps SDK is built on MCP.**

The tools your app exposes use the same Model Context Protocol you learned in Lesson 2. Same standard, different presentation layer.

---

## How Apps SDK Works

The architecture connects four pieces:

```
┌─────────────────────────────────────────────┐
│               ChatGPT                        │
│  ┌─────────────┐    ┌──────────────────┐   │
│  │  AI Model   │◄──►│   Your App UI    │   │
│  │             │    │   (in iframe)     │   │
│  └──────┬──────┘    └────────┬─────────┘   │
└─────────┼────────────────────┼─────────────┘
          │                    │
          ▼                    ▼
    ┌──────────┐        ┌──────────┐
    │ MCP      │        │ UI       │
    │ Server   │        │ Assets   │
    │ (Tools)  │        │ (React)  │
    └──────────┘        └──────────┘
```

1. **Define tools** via an MCP server (what the AI can do)
2. **Build UI components** (what users see and interact with)
3. **Connect to ChatGPT** (distribution)
4. **User interacts** with rich interface, AI handles the logic

You build once. ChatGPT handles the AI, hosting, and distribution.

---

## Who Can Use Apps SDK

Currently available to:

- ChatGPT Business subscribers
- ChatGPT Enterprise organizations
- ChatGPT Edu institutions

Developers can submit apps for distribution. The reach is significant: **800+ million ChatGPT users**.

If your Digital FTE needs massive distribution, Apps SDK provides a path.

---

## MCP-UI: The Emerging Standard

While Apps SDK is production-ready today, a broader standard is emerging: **MCP-UI**.

MCP-UI extends the Model Context Protocol to include user interface capabilities. Instead of MCP servers returning just data, they can return interactive UI components.

### The Problem MCP-UI Solves

Right now, every AI platform handles UI differently:

- ChatGPT has Apps SDK
- Claude has Artifacts
- Other agents have their own approaches

This is the same fragmentation problem MCP solved for tools—now appearing in interfaces.

### The MCP-UI Solution

A standard where:

- Any MCP server can return UI components
- Any MCP client can render them
- One skill/server works across all platforms

The same "write once, run everywhere" promise that MCP brought to tools, applied to interfaces.

---

## The Collaboration

Something notable is happening: competitors are collaborating again.

The MCP-UI effort includes:

- **Anthropic** (Claude, MCP creators)
- **OpenAI** (ChatGPT, Apps SDK)
- **Community contributors** from across the AI development ecosystem

The goal: prevent interface fragmentation before it becomes entrenched.

This follows the AAIF pattern—competitors recognizing that open standards benefit everyone.

---

## Current State: What to Do Now

| Standard | Status | Recommendation |
|----------|--------|----------------|
| **Apps SDK** | Production-ready | Use today for ChatGPT distribution |
| **MCP-UI** | Emerging | Watch this space, prepare for future |

If you need to build agent interfaces now:
- **Apps SDK** is your path for ChatGPT
- Other platforms have their own current solutions

If you're planning long-term:
- **MCP-UI** will likely become the unified standard
- Skills and MCP servers you build today will work with future MCP-UI

The foundation is stable (MCP). The interface layer is evolving (MCP-UI). Build on the foundation.

---

## Connection to Digital FTEs

Your Digital FTEs need interfaces. Users interact with them somehow.

| Distribution Path | Interface Technology |
|-------------------|---------------------|
| Inside ChatGPT | Apps SDK |
| Custom applications | MCP-UI (emerging) |
| CLI/Terminal | Text-based (current) |
| Web apps | Your own UI + MCP backend |

The same Digital FTE can have different "faces":

- A chat interface for quick queries
- A rich dashboard for complex tasks
- An API for integration with other systems

**MCP provides the brain. The interface is the face.**

### Monetization through Apps SDK

Remember the monetization models from Chapter 1? Apps SDK unlocks the **Marketplace** path:

| Advantage | Impact |
|-----------|--------|
| **800M+ ChatGPT users** | Massive distribution potential |
| **Low customer acquisition cost** | Users discover you in the app directory |
| **Platform billing** | OpenAI handles payments |
| **Volume play** | Hundreds of small customers vs few large contracts |

When you package your Digital FTE as a ChatGPT app, you reach customers you couldn't acquire otherwise. This is platform distribution—the same model that made mobile app stores lucrative.

Your Custom Agent (built in Part 6) becomes a product sold to millions through ChatGPT's marketplace. Apps SDK is your distribution channel.

**MCP-UI will extend this to all platforms.** When the standard matures, your Digital FTE's interface works across ChatGPT, Claude, Gemini, and custom applications—write once, distribute everywhere.

---

## Quick Knowledge Check

Before moving on, make sure you can answer:

1. Why are agent interfaces evolving beyond text chat?
2. What does the OpenAI Apps SDK enable?
3. What is MCP-UI and what problem does it solve?
4. What's the current recommendation: Apps SDK or MCP-UI?

---

## Try With AI

Ask your AI assistant:

> "What's the difference between a ChatGPT GPT and an OpenAI App?"

Evaluate the response:
- Does it explain that GPTs are conversation-focused while Apps include custom UI?
- Does it mention that Apps use MCP for tools?
- Does it note the richer interaction possibilities with Apps?

---

## Summary

- **Beyond chat**: Agent interfaces are evolving from text to interactive components
- **Apps SDK**: OpenAI's production-ready platform for building apps inside ChatGPT, built on MCP
- **MCP-UI**: Emerging standard for universal agent interfaces, prevents fragmentation
- **The collaboration**: Anthropic, OpenAI, and community working toward unified standard
- **Current state**: Use Apps SDK today, prepare for MCP-UI future
- **Digital FTE connection**: Your FTEs can have different interfaces for different contexts, all powered by the same MCP foundation

---

## Chapter Complete

You've finished Chapter 2: AIFF Foundation & Agent Standards.

You now understand:

| Standard | What It Does |
|----------|--------------|
| **AAIF** | Governs agentic AI standards under Linux Foundation |
| **MCP** | Connects agents to tools (universal connectivity) |
| **AGENTS.md** | Provides project-specific context to agents |
| **goose** | Reference agent showing standards in action |
| **Agent Skills** | Packages domain expertise for agents |
| **MCP-UI/Apps SDK** | Evolving interfaces beyond text chat |

These standards form the infrastructure layer of your Digital FTEs. Every agent you build will use them.

**Next**: Part 2 (AI Tool Landscape) will put these standards into practice as you explore the actual tools—Claude Code, Gemini CLI, and more.
