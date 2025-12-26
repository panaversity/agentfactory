---
title: "Chapter 2 Quiz"
sidebar_position: 7
---

# Chapter 2 Quiz: The AIFF Foundation

Test your understanding of the open standards that power portable Digital FTEs. Each question checks a specific learning objective from the lessons.

<Quiz
  title="Chapter 2: The AIFF Foundation"
  questions={[
    {
      question: "What is the primary business benefit of AAIF (Agentic AI Foundation) for building Digital FTEs? (Lesson 1)",
      options: [
        "It provides free hosting for AI agents",
        "It ensures your Digital FTEs are portable investments that work across platforms, not locked to a single vendor",
        "It automatically generates code for your agents",
        "It provides AI models at discounted prices"
      ],
      correctOption: 1,
      explanation: "AAIF provides neutral governance for open standards (MCP, AGENTS.md, goose), ensuring your Digital FTEs can connect to any CRM, work with any AI platform, and adapt to any client's workflow—without custom integration per platform.",
      source: "Lesson 1: The Agentic AI Foundation"
    },
    {
      question: "Which three projects were donated to AAIF by competing companies (OpenAI, Anthropic, Block) on December 9, 2025? (Lesson 1)",
      options: [
        "ChatGPT, Claude, Gemini",
        "MCP, AGENTS.md, goose",
        "Cursor, VS Code, GitHub Copilot",
        "TensorFlow, PyTorch, JAX"
      ],
      correctOption: 1,
      explanation: "The three founding projects are: MCP (Model Context Protocol) from Anthropic for tool connectivity, AGENTS.md from OpenAI for project context, and goose from Block as a reference agent implementation.",
      source: "Lesson 1: The Agentic AI Foundation"
    },
    {
      question: "In MCP's three primitives, what is the difference between Resources and Tools? (Lesson 2)",
      options: [
        "Resources are free; Tools require payment",
        "Resources provide data to read (agent's 'eyes'); Tools execute actions (agent's 'hands')",
        "Resources work locally; Tools work remotely",
        "Resources are for developers; Tools are for end users"
      ],
      correctOption: 1,
      explanation: "Resources are what your Digital FTE can see—lead data from CRM, email history, company information. Tools are what your Digital FTE can do—create records, send emails, schedule meetings. Resources read; Tools act.",
      source: "Lesson 2: MCP - Connecting Agents to Tools"
    },
    {
      question: "Which of the Five Powers from Chapter 1 does MCP specifically enable for Digital FTEs? (Lesson 2)",
      options: [
        "See (visual understanding)",
        "Hear (audio processing)",
        "Reason (multi-step planning)",
        "Act (execute code, call APIs, operate tools)"
      ],
      correctOption: 3,
      explanation: "MCP enables the 'Act' power. Without MCP, your Digital FTE can reason about what to do but can't actually do it. With MCP, it connects to CRMs, email platforms, calendars, and databases to take real action.",
      source: "Lesson 2: MCP - Connecting Agents to Tools"
    },
    {
      question: "What is the MCP architecture pattern for connecting an AI application to business tools? (Lesson 2)",
      options: [
        "Client → Server → Database",
        "Host → Client → Server",
        "Frontend → Backend → API",
        "User → Agent → Model"
      ],
      correctOption: 1,
      explanation: "MCP uses Host → Client → Server: the Host is your AI application (Claude Desktop, ChatGPT), the Client lives inside the Host and manages connections, and the Server exposes business tools (CRM, database) to the agent.",
      source: "Lesson 2: MCP - Connecting Agents to Tools"
    },
    {
      question: "What is the key business advantage of AGENTS.md for selling Digital FTEs to multiple clients? (Lesson 3)",
      options: [
        "It reduces the AI model cost per query",
        "It enables zero-config deployments—same Digital FTE adapts to each client's environment automatically",
        "It encrypts all communications between agent and client",
        "It provides automatic scaling for high-traffic applications"
      ],
      correctOption: 1,
      explanation: "AGENTS.md is README for AI agents. Your Digital FTE reads each client's AGENTS.md to understand their coding conventions, build commands, and security requirements—deploying to 100 different organizations without customization.",
      source: "Lesson 3: AGENTS.md - Project Context for Agents"
    },
    {
      question: "In a monorepo with multiple AGENTS.md files, which file takes precedence when an agent edits code in a subdirectory? (Lesson 3)",
      options: [
        "The root AGENTS.md always takes precedence",
        "The newest AGENTS.md file",
        "The nearest AGENTS.md file (closest to the edited file)",
        "AGENTS.md files are merged together"
      ],
      correctOption: 2,
      explanation: "The hierarchy rule: nearest AGENTS.md wins. When editing packages/frontend/src/, the agent uses packages/frontend/AGENTS.md if it exists, enabling project-wide defaults at root with specific overrides in subdirectories.",
      source: "Lesson 3: AGENTS.md - Project Context for Agents"
    },
    {
      question: "Why does goose matter for your Agent Factory even though you might use Claude Code as your primary tool? (Lesson 4)",
      options: [
        "goose is faster than Claude Code",
        "goose provides a reference architecture—study it to understand what production-ready agents look like when building Custom Agents",
        "goose is the only agent that supports MCP",
        "goose is required to deploy agents to production"
      ],
      correctOption: 1,
      explanation: "goose is Block's open source agent where 75% of engineers save 8-10+ hours weekly. When you build Custom Agents (Path B), goose shows MCP integration patterns, local-first execution, and multi-model support—your blueprint for enterprise-ready agents.",
      source: "Lesson 4: goose - A Reference Agent"
    },
    {
      question: "What is the relationship between goose and Claude Code in the Two Paths framework? (Lesson 4)",
      options: [
        "They are direct competitors—choose one or the other",
        "goose is for beginners; Claude Code is for experts",
        "Both are General Agents (Path A) that validate the same open standards (MCP, AGENTS.md)",
        "Claude Code uses goose internally"
      ],
      correctOption: 2,
      explanation: "Both goose and Claude Code are General Agents (Path A). They're not competitors—they're validation that the standards work. Both support MCP and AGENTS.md, proving the ecosystem is real and production-tested.",
      source: "Lesson 4: goose - A Reference Agent"
    },
    {
      question: "What problem does 'progressive disclosure' solve in Agent Skills? (Lesson 5)",
      options: [
        "It makes skills easier to write",
        "It prevents token waste by loading only skill names at startup, full content when activated, and supporting files on-demand",
        "It hides complex functionality from users",
        "It automatically upgrades skills over time"
      ],
      correctOption: 1,
      explanation: "Loading all skills upfront wastes tokens. Progressive disclosure loads names/descriptions at startup (~100 tokens per skill), full SKILL.md when activated, and supporting files only when needed—achieving 80-98% token reduction.",
      source: "Lesson 5: Agent Skills - Packaging Expertise"
    },
    {
      question: "What is the key difference between MCP and Agent Skills? (Lesson 5)",
      options: [
        "MCP is open source; Skills are proprietary",
        "MCP provides connectivity (how agents talk to tools); Skills provide expertise (what agents know how to do)",
        "MCP is for reading; Skills are for writing",
        "MCP works with Claude; Skills work with ChatGPT"
      ],
      correctOption: 1,
      explanation: "MCP and Skills are complementary, not competing. For Stripe payments: MCP Server connects to Stripe API (access), while a Skill knows how to handle payment scenarios properly (expertise). Your Digital FTEs combine both.",
      source: "Lesson 5: Agent Skills - Packaging Expertise"
    },
    {
      question: "What does 'YOUR domain expertise → Skills → Digital FTE → Recurring Revenue' represent? (Lesson 5)",
      options: [
        "A training curriculum for learning AI",
        "The Agent Factory monetization path—encoding your expertise into sellable products",
        "A debugging workflow for AI systems",
        "The hiring process for AI developers"
      ],
      correctOption: 1,
      explanation: "Skills are how you encode what makes you valuable. Financial analysis expertise becomes a skill, powers a Digital FTE, and sells as a subscription. The $650M CoCounsel acquisition was encoded legal expertise—Skills let you create similar value.",
      source: "Lesson 5: Agent Skills - Packaging Expertise"
    },
    {
      question: "What is the relationship between OpenAI's Apps SDK and MCP? (Lesson 6)",
      options: [
        "Apps SDK replaces MCP entirely",
        "Apps SDK is built on MCP—same tools, different presentation layer with rich UI components",
        "Apps SDK and MCP are competing standards",
        "MCP is required for Apps SDK to work with Claude only"
      ],
      correctOption: 1,
      explanation: "Apps SDK is built on MCP. The tools your app exposes use the same Model Context Protocol—your existing MCP servers work with Apps SDK immediately. Apps SDK adds custom UI components for rich interactive experiences inside ChatGPT.",
      source: "Lesson 6: MCP-UI & Apps SDK - Agent Interfaces"
    },
    {
      question: "What marketplace monetization advantage does Apps SDK provide for Digital FTEs? (Lesson 6)",
      options: [
        "Free AI model usage for all apps",
        "Access to 800M+ ChatGPT users with low customer acquisition cost and platform-handled billing",
        "Automatic code generation for apps",
        "Guaranteed revenue for published apps"
      ],
      correctOption: 1,
      explanation: "Apps SDK unlocks the Marketplace monetization path: 800M+ ChatGPT users, low customer acquisition cost (users discover you in the app directory), and OpenAI handles payments—platform distribution like mobile app stores.",
      source: "Lesson 6: MCP-UI & Apps SDK - Agent Interfaces"
    },
    {
      question: "What is the MCP Apps Extension (SEP-1865)? (Lesson 6)",
      options: [
        "A new AI model from Anthropic",
        "A standardized protocol for enabling interactive user interfaces in MCP—allowing servers to deliver UI directly to any MCP host",
        "A replacement for the MCP protocol",
        "A ChatGPT-only feature"
      ],
      correctOption: 1,
      explanation: "The MCP Apps Extension (SEP-1865), announced November 2025, allows MCP servers to deliver interactive UI (forms, charts, dashboards) to any MCP host application. It uses ui:// URI scheme for pre-declared templates rendered in sandboxed iframes.",
      source: "Lesson 6: MCP Apps Extension - Agent Interfaces"
    },
    {
      question: "Which projects collaborated to create the MCP Apps Extension standard? (Lesson 6)",
      options: [
        "Google and Microsoft only",
        "MCP-UI (open source, adopted by Postman, Shopify, HuggingFace) and OpenAI Apps SDK—with Anthropic and OpenAI collaborating on standardization",
        "Block and Amazon Web Services",
        "GitHub Copilot and Cursor"
      ],
      correctOption: 1,
      explanation: "The MCP Apps Extension builds on MCP-UI (an open source project adopted by Postman, Shopify, HuggingFace, ElevenLabs) and OpenAI Apps SDK. Anthropic, OpenAI, and the MCP-UI creators collaborated to standardize these patterns.",
      source: "Lesson 6: MCP Apps Extension - Agent Interfaces"
    },
    {
      question: "If you're building a Digital FTE today that needs maximum distribution, what should you use for the interface layer? (Lesson 6)",
      options: [
        "Wait for MCP Apps Extension to be finalized",
        "Build a custom web interface from scratch",
        "Use Apps SDK for ChatGPT distribution now (production-ready), while preparing for MCP Apps Extension's cross-platform support",
        "Only use CLI/terminal interfaces"
      ],
      correctOption: 2,
      explanation: "Apps SDK is production-ready today with access to 800M+ users. MCP Apps Extension is proposed (SEP-1865) but still standardizing. The recommendation: use Apps SDK now for ChatGPT distribution, knowing your MCP foundation will work with MCP Apps Extension when it matures.",
      source: "Lesson 6: MCP Apps Extension - Agent Interfaces"
    }
  ]}
  questionsPerBatch={17}
/>
