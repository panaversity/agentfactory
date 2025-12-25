# Implementation Plan: Chapter 2 - AIFF Foundation & Agent Standards

**Specification**: `specs/036-chapter-2-aiff-foundation/spec.md`
**Created**: 2025-12-25
**Target Directory**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-aiff-foundation/`

---

## Constitutional Alignment

### Teaching Layer
**Layer 1: Manual Foundation** - Book teaches directly, student builds mental models

### Cognitive Tier
**A1 Beginner** - 3-5 concepts per section, very heavy scaffolding

### Anti-Convergence
Chapter 1 used paradigm/vision framing. Chapter 2 uses infrastructure/standards framing.

### Digital FTE Connection
Every lesson connects standards to Digital FTE production capability.

---

## Chapter Structure

```
apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-aiff-foundation/
├── README.md                              # Chapter overview
├── 01-agentic-ai-foundation.md           # AAIF ecosystem
├── 02-model-context-protocol.md          # MCP deep dive
├── 03-agents-md-project-context.md       # AGENTS.md standard
├── 04-goose-reference-agent.md           # Block's goose
├── 05-agent-skills-packaging-expertise.md # Agent Skills
└── 06-mcp-ui-and-apps-sdk.md             # Interface standards
```

---

## Lesson-by-Lesson Architecture

### Lesson 1: AAIF - The Agentic AI Foundation

**File**: `01-agentic-ai-foundation.md`
**Duration**: ~12 minutes
**Focus**: Ecosystem governance and orientation

#### Learning Objectives

| Objective | Bloom's Level | CEFR | DigComp 2.2 |
|-----------|--------------|------|-------------|
| Explain what AAIF is and why it was created | Understand | A1 | 1.1 Browsing |
| Name the three founding projects with origins | Remember | A1 | 1.1 Browsing |
| Articulate why open standards matter for AI | Understand | A1 | 2.1 Interacting |

#### Content Structure

1. **The Headline** (December 2025)
   - Linux Foundation announces AAIF
   - Why this matters for AI development
   - Avoiding "closed wall" proprietary stacks

2. **Three Founding Projects**
   - MCP (Anthropic) - Agent-to-tool connectivity
   - AGENTS.md (OpenAI) - Project-level agent instructions
   - goose (Block) - Reference agent implementation
   - Visual: Three pillars supporting Digital FTE house

3. **Who's Behind AAIF**
   - Platinum members table
   - Significance: competitors collaborating on standards
   - Open governance model

4. **Why Open Standards Matter**
   - Analogy: USB vs proprietary cables
   - Your Digital FTE works everywhere
   - Write skills once, run across agents

5. **Try With AI** (Layer 1 - minimal AI)
   - Ask AI: "What is the Agentic AI Foundation?"
   - Compare AI's answer to lesson content
   - Note: AI may have training cutoff before Dec 2025

#### Key Quotes/Facts
- "By bringing these projects together under the AAIF, we are now able to coordinate interoperability, safety patterns, and best practices specifically for AI agents." - Jim Zemlin, Linux Foundation
- Platinum members: AWS, Anthropic, Block, Bloomberg, Cloudflare, Google, Microsoft, OpenAI

---

### Lesson 2: MCP - Connecting Agents to Tools

**File**: `02-model-context-protocol.md`
**Duration**: ~18 minutes
**Focus**: Core connectivity protocol

#### Learning Objectives

| Objective | Bloom's Level | CEFR | DigComp 2.2 |
|-----------|--------------|------|-------------|
| Explain what problem MCP solves | Understand | A1 | 2.1 Interacting |
| Identify the three MCP primitives | Remember | A1 | 1.1 Browsing |
| Describe MCP architecture (Host→Client→Server) | Understand | A1 | 2.1 Interacting |
| Connect MCP to Digital FTE "Act" power | Apply | A2 | 2.3 Collaboration |

#### Content Structure

1. **The Problem MCP Solves**
   - Before MCP: Every AI had custom integrations
   - The fragmentation problem (diagram)
   - Vision: Universal connectivity standard

2. **The USB Analogy**
   - USB standardized device connections
   - MCP standardizes AI-to-tool connections
   - Any agent + any tool = just works

3. **Three Primitives** (core concepts)
   - **Resources**: Data the agent can read (files, database records, API responses)
   - **Tools**: Functions the agent can call (search, create, update, delete)
   - **Prompts**: Reusable templates for common interactions
   - Visual: Three boxes with icons and examples

4. **Architecture: Host → Client → Server**
   - **Host**: LLM application (Claude Desktop, ChatGPT)
   - **Client**: Connector inside the host
   - **Server**: External service providing capabilities
   - Visual: Three-tier diagram with arrows

5. **Transport Mechanisms** (awareness only)
   - stdio: For local tools (runs on your machine)
   - HTTP/SSE: For remote services (APIs, cloud tools)
   - Students don't need to implement yet

6. **MCP History**
   - November 2024: Anthropic releases MCP
   - December 2025: Donated to AAIF
   - Industry adoption: OpenAI, Google, Microsoft

7. **Digital FTE Connection**
   - MCP enables the "Act" power (Five Powers from Chapter 1)
   - Your Digital FTE uses MCP to access business tools
   - Example: Sales FTE connects to CRM, email, calendar via MCP

8. **Try With AI**
   - Ask AI: "Explain MCP like I'm 10 years old"
   - Ask AI: "What's the difference between MCP Resources and Tools?"

#### Key Quotes/Facts
- "When we open sourced it in November 2024, we hoped other developers would find it as useful as we did. A year later, it's become the industry standard for connecting AI systems to data and tools." - Mike Krieger, Anthropic CPO
- MCP takes inspiration from Language Server Protocol (LSP)
- Uses JSON-RPC 2.0 over stateful connections

---

### Lesson 3: AGENTS.md - Project Context for Agents

**File**: `03-agents-md-project-context.md`
**Duration**: ~12 minutes
**Focus**: Repository-level agent instructions

#### Learning Objectives

| Objective | Bloom's Level | CEFR | DigComp 2.2 |
|-----------|--------------|------|-------------|
| Explain what AGENTS.md is and its purpose | Understand | A1 | 1.1 Browsing |
| Identify typical AGENTS.md content sections | Remember | A1 | 1.1 Browsing |
| Understand the hierarchy rule (nearest wins) | Understand | A1 | 2.1 Interacting |

#### Content Structure

1. **README for Agents**
   - AGENTS.md complements README.md
   - README: For humans
   - AGENTS.md: For AI agents
   - Purpose: Project-specific guidance

2. **What Goes in AGENTS.md**
   - Build and test commands
   - Code style guidelines
   - Testing instructions
   - Security considerations
   - Commit message formats
   - PR guidelines
   - Visual: Example sections list

3. **Format: Simple Markdown**
   - No required fields
   - Use any headings relevant to your project
   - Plain Markdown - agents parse it easily

4. **The Hierarchy Rule**
   - Root AGENTS.md applies to whole project
   - Subdirectory AGENTS.md for specific areas
   - **Nearest file takes precedence**
   - Example: monorepo with different rules per package

5. **Adoption**
   - 60,000+ open source projects
   - Supported by: Claude, Cursor, GitHub Copilot, VS Code, Devin, Zed
   - OpenAI origin (August 2025), now AAIF

6. **Digital FTE Connection**
   - When Digital FTE works on your codebase, AGENTS.md tells it local rules
   - Prevents agents from violating project conventions
   - Encodes team knowledge for AI consumption

7. **Try With AI**
   - Ask AI: "What would you put in an AGENTS.md for a Python web API?"
   - Student evaluates: Does AI's suggestion make sense for the scenario?

#### Key Quotes/Facts
- "AGENTS.md is a simple, universal standard that gives AI coding agents a consistent source of project-specific guidance needed to operate reliably across different repositories and toolchains."
- 88 AGENTS.md files in OpenAI's main repository

---

### Lesson 4: goose - A Reference Agent

**File**: `04-goose-reference-agent.md`
**Duration**: ~12 minutes
**Focus**: Real agent implementation example

#### Learning Objectives

| Objective | Bloom's Level | CEFR | DigComp 2.2 |
|-----------|--------------|------|-------------|
| Explain what goose is and who created it | Remember | A1 | 1.1 Browsing |
| Identify goose's key features | Remember | A1 | 1.1 Browsing |
| Understand goose as reference MCP implementation | Understand | A1 | 2.1 Interacting |

#### Content Structure

1. **From Abstract to Concrete**
   - We've learned standards (MCP, AGENTS.md)
   - Now: What does a real agent look like?
   - goose: Block's open source implementation

2. **What is goose?**
   - Open source AI agent framework
   - Created by Block (Square)
   - AAIF founding project
   - "Your local AI agent, automating engineering tasks"

3. **Key Features**
   - **Local-first**: Runs on your machine
   - **MCP-native**: Built on Model Context Protocol
   - **Extensible**: Connect to any MCP server
   - **Any LLM**: Works with Claude, GPT, Gemini, local models

4. **Enterprise Adoption at Block**
   - Thousands of Block employees use goose daily
   - 50-75% time savings on common tasks
   - "Work that once took days can now be completed in hours"

5. **Why goose Matters**
   - Reference implementation of MCP integration
   - Shows what "good" looks like
   - Proves standards work at enterprise scale

6. **goose vs Claude Code**
   - Both are General Agents (Two Paths Framework)
   - Claude Code: Anthropic's agent
   - goose: Block's agent (open source)
   - Both use MCP, both work with AGENTS.md
   - Not competitors - validation of standards

7. **Digital FTE Connection**
   - goose demonstrates General Agent capabilities
   - Understanding goose helps you understand agent architecture
   - Your Digital FTEs will use similar patterns

8. **Try With AI**
   - Ask AI: "Compare goose and Claude Code as AI agents"
   - Evaluate: Does AI correctly identify they're both General Agents?

#### Key Quotes/Facts
- "Block has been collaborating closely with Anthropic to develop this protocol" (MCP)
- Available as both desktop app and CLI
- Released under Apache License 2.0

---

### Lesson 5: Agent Skills - Packaging Expertise

**File**: `05-agent-skills-packaging-expertise.md`
**Duration**: ~15 minutes
**Focus**: Encoding domain expertise

#### Learning Objectives

| Objective | Bloom's Level | CEFR | DigComp 2.2 |
|-----------|--------------|------|-------------|
| Explain what Agent Skills are | Understand | A1 | 1.1 Browsing |
| Describe SKILL.md format | Remember | A1 | 1.1 Browsing |
| Understand progressive disclosure mechanism | Understand | A2 | 2.1 Interacting |
| Articulate MCP + Skills relationship | Understand | A2 | 2.3 Collaboration |

#### Content Structure

1. **The Matrix Moment**
   - Trinity in Matrix loading helicopter pilot skill
   - Skills: Pre-packaged expertise agents can load when needed
   - Your domain expertise, encoded and portable

2. **What Are Agent Skills?**
   - Folders of instructions, scripts, and resources
   - Teach agents how to perform specific tasks
   - Reusable across projects and agents
   - Anthropic open standard (December 2025)

3. **SKILL.md Format**
   ```yaml
   ---
   name: skill-name
   description: What this skill does and when to use it
   ---

   # Skill Instructions

   ## When to Use
   [Trigger conditions]

   ## How to Execute
   [Step-by-step guidance]
   ```

4. **Progressive Disclosure** (key concept)
   - Problem: Loading all skill content wastes tokens
   - Solution: Load only what's needed
   - Level 1: Name + description (~100 tokens at startup)
   - Level 2: Full SKILL.md (when skill activated)
   - Level 3: Supporting files (only if referenced)
   - Visual: Pyramid showing token usage

5. **Token Efficiency**
   - MCP servers alone: 14,000-80,000+ tokens
   - Skills + Scripts: ~100 tokens
   - "80-98% token reduction" - Jared Palmer, Vercel

6. **MCP + Skills: Complementary**
   - MCP: **Connectivity** (how agents talk to tools)
   - Skills: **Expertise** (what agents know how to do)
   - Together: Connected agent + knowledgeable agent
   - Example: Stripe MCP server + payment-processing skill

7. **Adoption**
   - VS Code, GitHub, Cursor, goose, Amp, OpenCode
   - Partner skills: Canva, Stripe, Notion, Zapier

8. **Digital FTE Connection**
   - YOUR domain expertise → Skills → Digital FTE
   - Skills are how you encode what makes you valuable
   - Monetization: License your skills, sell skill-enhanced FTEs

9. **Try With AI**
   - Ask AI: "What would a SKILL.md look like for financial analysis?"
   - Evaluate: Does it follow the frontmatter + instructions format?

#### Key Quotes/Facts
- "Skills are organized collections of files that package composable procedural knowledge for agents"
- "The amount of context that can be bundled into a skill is effectively unbounded" (due to progressive disclosure)
- agentskills.io launched December 18, 2025

---

### Lesson 6: MCP-UI & Apps SDK - Agent Interfaces

**File**: `06-mcp-ui-and-apps-sdk.md`
**Duration**: ~15 minutes
**Focus**: Interactive agent interfaces

#### Learning Objectives

| Objective | Bloom's Level | CEFR | DigComp 2.2 |
|-----------|--------------|------|-------------|
| Understand agent UI is evolving beyond chat | Understand | A1 | 2.1 Interacting |
| Explain what OpenAI Apps SDK enables | Understand | A1 | 2.1 Interacting |
| Recognize MCP-UI as emerging standard | Remember | A1 | 1.1 Browsing |

#### Content Structure

1. **Beyond Chat**
   - Chat is powerful but limiting
   - What if agents had buttons, forms, visualizations?
   - UI evolution: Text → Structured output → Interactive components

2. **OpenAI Apps SDK (Use Today)**
   - Build apps that run inside ChatGPT
   - Combines: MCP tools + Custom UI components
   - Example uses: Order groceries, create presentations, search apartments
   - Key: **Apps SDK is built on MCP**

3. **How Apps SDK Works**
   - Define tools (via MCP server)
   - Build UI components (run in iframe)
   - Connect to ChatGPT
   - User interacts with rich interface

4. **Who Can Use Apps SDK**
   - Currently: ChatGPT Business, Enterprise, Edu
   - Open for developer submissions
   - Distribution: 800M+ ChatGPT users

5. **MCP-UI (Emerging Standard)**
   - Extension to MCP specification
   - Allows servers to return interactive UI
   - Not just data → actual visual components

6. **The Collaboration**
   - Anthropic + OpenAI + MCP-UI community
   - Working on unified standard
   - Goal: Any MCP server can provide UI, any client can render it
   - Prevents fragmentation

7. **Current State**
   - Apps SDK: Production-ready, use today
   - MCP-UI: Emerging, watch this space
   - Recommendation: Start with Apps SDK, know MCP-UI is coming

8. **Digital FTE Connection**
   - Your Digital FTEs might live inside ChatGPT (Apps SDK)
   - Or in custom interfaces (MCP-UI)
   - Same MCP foundation, different frontends

9. **Try With AI**
   - Ask AI: "What's the difference between a ChatGPT GPT and an OpenAI App?"
   - Evaluate: Does AI correctly explain Apps use MCP and custom UI?

#### Key Quotes/Facts
- Apps SDK launched October 6, 2025
- "Apps extend ChatGPT conversations by bringing in new context and letting users take actions"
- MCP-UI collaborators: Postman, Shopify, Hugging Face, ElevenLabs

---

## Chapter README

**File**: `README.md`

```markdown
---
sidebar_position: 2
title: "Chapter 2: AIFF Foundation & Agent Standards"
---

# Chapter 2: AIFF Foundation & Agent Standards

## Overview

Chapter 1 gave you the vision: building Digital FTEs that work 24/7. But how do these agents actually connect to tools, find project context, and package expertise? That's what you'll learn here.

In December 2025, something remarkable happened: OpenAI, Anthropic, and Block—competitors in the AI space—came together under the Linux Foundation to create the Agentic AI Foundation (AAIF). They donated their core standards to neutral governance, ensuring the future of AI agents is open, not locked into proprietary platforms.

This chapter teaches you the infrastructure layer that makes Digital FTEs possible.

## What You'll Learn

| Lesson | Standard | Why It Matters |
|--------|----------|----------------|
| 1 | AAIF | Who governs agentic AI standards |
| 2 | MCP | How agents connect to tools |
| 3 | AGENTS.md | How agents find project context |
| 4 | goose | What a real agent looks like |
| 5 | Agent Skills | How to package expertise |
| 6 | MCP-UI & Apps SDK | How agent interfaces are evolving |

## Prerequisites

- **Chapter 1**: Agent Factory Paradigm (Two Paths Framework, Digital FTE vision, Five Powers)

## Time Investment

~90 minutes total (including exercises and quiz)

## Outcome

After this chapter, you'll understand the standards ecosystem well enough to recognize MCP commands, understand skill folders, and articulate how these standards enable the Digital FTEs you'll build in later parts.

## Let's Begin

Start with [Lesson 1: The Agentic AI Foundation](./01-agentic-ai-foundation.md).
```

---

## Implementation Notes

### Layer 1 Teaching Approach

- **Book teaches directly**: Clear explanations, analogies, diagrams
- **Minimal AI role**: "Try With AI" sections for validation, not learning
- **No code execution**: Conceptual understanding only
- **Heavy scaffolding**: Every concept connected to Digital FTE outcome

### Diagram Requirements

| Lesson | Diagram |
|--------|---------|
| 1 | Three pillars (MCP, AGENTS.md, goose) supporting Digital FTE house |
| 2 | Host → Client → Server architecture |
| 2 | Three primitives (Resources, Tools, Prompts) with icons |
| 3 | Hierarchy: Root AGENTS.md + subdirectory files |
| 5 | Progressive disclosure pyramid (tokens at each level) |
| 6 | Apps SDK architecture (MCP + UI + ChatGPT) |

### Citation Requirements

| Fact | Source |
|------|--------|
| AAIF announcement Dec 2025 | linuxfoundation.org/press |
| MCP 2025-11-25 spec | modelcontextprotocol.io/specification |
| AGENTS.md 60K+ adoption | agents.md |
| goose 50-75% time savings | block.github.io/goose |
| Agent Skills Dec 18, 2025 | agentskills.io |
| Apps SDK Oct 6, 2025 | developers.openai.com/apps-sdk |

### Anti-Patterns to Avoid

- ❌ Implementation tutorials (save for Part 2+)
- ❌ Deep protocol internals (JSON-RPC, transport details)
- ❌ A2A/ACP/Commerce protocols (different problem space)
- ❌ Code examples requiring execution
- ❌ Meta-commentary ("Now you've learned...")
- ❌ Exceeding 5 concepts per section

---

## Success Indicators

### Plan Complete When:
- [x] All 6 lessons have learning objectives with Bloom's/CEFR/DigComp
- [x] Content structure defined for each lesson
- [x] Key quotes/facts cited with sources
- [x] Diagrams identified
- [x] "Try With AI" sections designed
- [x] Digital FTE connection explicit in every lesson

### Ready for Task Generation: ✅

---

**Status**: Plan complete. Awaiting human approval.
