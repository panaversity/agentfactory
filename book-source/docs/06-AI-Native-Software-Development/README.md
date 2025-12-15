---
sidebar_position: 6
title: "Part 6: AI Native Software Development"
---

# Part 6: AI Native Software Development

Part 6 bridges your foundational skills from Parts 1-5 into production agent development. You've learned SDD-RI fundamentals (Part 4) and Python fundamentals (Part 5)—now you'll apply these skills to design, implement, test, and deploy AI agents for real-world applications.

This part introduces agent frameworks, integration patterns, testing strategies, and data persistence—the building blocks of production AI systems. You'll move from educational examples to patterns you'd encounter building professional agent applications.

---

## Goals

By completing Part 6, you will:

- **Understand agent architectures**: Learn how modern agent frameworks (OpenAI Agents SDK, Google ADK, Anthropic Agents Kit) structure agentic systems
- **Implement integration patterns**: Practice connecting agents through MCP (Model Context Protocol) and building interoperable systems
- **Design agent APIs**: Build FastAPI endpoints that expose agent capabilities as services
- **Apply testing strategies**: Practice test-driven development (TDD) and evaluation frameworks (evals) for agent reliability
- **Deploy design patterns**: Implement memory, state management, and compositional patterns for complex agent behaviors
- **Integrate data persistence**: Connect agents to vector databases (RAG), relational databases, and graph databases

---

## Hands-on Lab
Want to explore the code without installing anything? You can interact with the full project directly below.  

:::info How to Edit
The embedded view below is **read-only**. To edit the code or run your own agents:
1. Click **"Open Editor"** in the top right corner of the embed.
2. Click the **Fork** button to save a copy to your own account.
:::

<iframe
  src="https://codesandbox.io/p/devbox/fvy37r?embed=1"
  style={{
    width: "100%",
    height: "500px",
    border: "0",
    borderRadius: "4px",
    overflow: "hidden",
    marginBottom: "-20px"
  }}
  title="Hackathon I"
  allow="accelerometer; ambient-light-sensor; camera; encrypted-media; geolocation; gyroscope; hid; microphone; midi; payment; usb; vr; xr-spatial-tracking"
  sandbox="allow-forms allow-modals allow-popups allow-presentation allow-same-origin allow-scripts"
/>

<a href="https://codesandbox.io/p/devbox/fvy37r?embed=1">
  <img alt="Edit Hackathon I" src="https://codesandbox.io/static/img/play-codesandbox.svg"/>
</a>


## Chapter Progression

Part 6's 16 chapters build through six thematic stages:

### Agent Frameworks (Chapters 33-36)
Establish core agent concepts and framework-specific patterns. Start with agent fundamentals (33), then explore implementation across OpenAI (34), Google (35), and Anthropic (36) platforms using AIDD and spec-driven development.

### Integration Patterns (Chapters 37-39)
Learn how agents connect and communicate. Understand MCP fundamentals (37), develop custom MCP servers (38), and implement code execution capabilities for more efficient agents (39).

### Implementation (Chapter 40)
Build production-ready agent services. FastAPI for Agents (40) teaches you to expose agent capabilities through RESTful APIs.

### Quality Practices (Chapters 41-43)
Validate agent correctness and reliability. Apply schema-enforced generation and reliability patterns (41), test-driven development patterns (42) and implement comprehensive evaluation frameworks (43) to ensure agents behave as specified.

### Advanced Patterns (Chapters 44-45)
Handle complexity at scale. Design effective memory and state management (44), and compose agents using combo patterns (45).

### Data Layer (Chapters 46-48)
Add persistence and knowledge retrieval. Integrate vector databases for RAG (46), relational databases for structured data (47), and graph databases for relationship-rich knowledge (48).

**Why this sequence?** You'll build conceptual understanding (what are agents?) before diving into integration (how do they connect?), then implementation (how do I build them?), quality (how do I validate them?), patterns (how do I scale them?), and data (how do I make them persistent?). Each stage builds on previous capabilities.

---

## Methodology Note

Part 6 continues the teaching approach from earlier parts: chapters introduce concepts through hands-on practice, guide you in collaborating with AI to implement solutions, help you build reusable components when patterns recur, and culminate in capstone projects where you apply spec-driven development to compose what you've learned.

You'll experience the same progression—foundation building, AI collaboration, creating reusable intelligence, and specification-first projects—now applied to production agent development rather than language fundamentals.