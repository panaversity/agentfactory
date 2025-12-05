---
sidebar_position: 5
title: "Lesson 1.4: Architecture Overview"
description: "Visualize the complete DocuBot system and understand the 16-chapter roadmap"
keywords: [architecture, RAG, system design, components, roadmap]
---

# Lesson 1.4: Architecture Overview

**Duration**: 25-35 minutes
**Difficulty**: Beginner (A1-A2)

## Learning Goal

By the end of this lesson, you will:
- Visualize the complete DocuBot system you'll build
- Understand how all the components work together
- See the 16-chapter roadmap and know where each piece fits

---

## What You'll Learn

Before building a house, you study the blueprints. Before cooking a complex meal, you read the entire recipe. Before building DocuBot, you need to see the whole picture.

DocuBot is a **RAG chatbot** (Retrieval-Augmented Generation). Let's break down what you'll build:

1. **Agent**: The intelligent brain that processes requests and decides what to do
2. **RAG Pipeline**: The system that searches your documents and retrieves relevant information
3. **Backend API**: The server that handles communication between parts
4. **Frontend UI**: The chat interface users interact with

:::info RAG Explained Simply
**RAG** = Retrieval-Augmented Generation

Instead of the AI just making up answers from training data, it:
1. **Retrieves** relevant documents from your collection
2. **Augments** its response with that real information
3. **Generates** an answer grounded in your actual documents

This is why DocuBot can answer questions about YOUR documents, not just general knowledge.
:::

---

## Key Points

Four foundational concepts about system architecture:

- **Components have single responsibilities**: Each part does one thing well (agent thinks, database stores, frontend displays)
- **Components communicate through APIs**: The backend connects everything; parts don't talk directly to each other
- **Each chapter adds one piece**: You'll build incrementally, testing as you go
- **Production is the goal**: This isn't a toy — you're building something deployable

---

## Simple Analogy

:::tip The House Blueprint Analogy
Imagine you're building a house:

**Without a blueprint:**
- You might build rooms in random order
- The kitchen plumbing might not connect to the bathroom
- You don't know if all the pieces will fit

**With a blueprint:**
- You see where every room goes before laying bricks
- You understand how plumbing, electrical, and structure connect
- Each day's work makes sense because you see the end goal

**DocuBot's architecture is your blueprint:**
- You'll know what you're building before writing code
- Each chapter adds a specific component
- When things connect, you'll understand why

The next 16 chapters are like construction phases. Today, you study the blueprint.
:::

---

## The DocuBot System

Here's what you're building:

```
┌─────────────────────────────────────────────────────────────────┐
│                         USER                                     │
│                    (You, asking questions)                       │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       FRONTEND (ChatKit)                         │
│            • Chat interface                                      │
│            • Message history display                             │
│            • Document upload UI                                  │
└─────────────────────────────┬───────────────────────────────────┘
                              │ HTTP/WebSocket
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      BACKEND API (FastAPI)                       │
│            • Routes requests                                     │
│            • Manages sessions                                    │
│            • Handles authentication                              │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    AGENT (OpenAI Agents SDK)                     │
│            • Processes user questions                            │
│            • Calls tools (search, summarize)                     │
│            • Maintains conversation context                      │
│            • Generates responses                                 │
└─────────┬──────────────────────────────────────┬────────────────┘
          │                                      │
          ▼                                      ▼
┌─────────────────────┐              ┌─────────────────────────────┐
│   RAG Pipeline      │              │      OpenAI API             │
│   • Embed documents │              │      • GPT-4 / GPT-4o       │
│   • Vector search   │              │      • Text generation      │
│   • Retrieve chunks │              │      • Embeddings           │
└─────────┬───────────┘              └─────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────────┐
│                  VECTOR DATABASE (Qdrant)                        │
│            • Stores document embeddings                          │
│            • Fast similarity search                              │
│            • Persistent storage                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Component Breakdown

| Component | Technology | What It Does |
|-----------|------------|--------------|
| Frontend | ChatKit | Beautiful chat UI for users |
| Backend | FastAPI | Routes requests, manages auth |
| Agent | OpenAI Agents SDK | The "brain" that thinks and acts |
| RAG Pipeline | LangChain/Custom | Searches and retrieves documents |
| Vector DB | Qdrant | Stores and searches document embeddings |
| AI Provider | OpenAI API | Provides language model capabilities |

---

## The 16-Chapter Journey

Here's how the chapters map to components:

| Chapters | Focus | What You'll Build |
|----------|-------|-------------------|
| **1-3** | Agent Basics | Simple agent, tools, first interactions |
| **4-6** | Advanced Agent | Multi-step tasks, complex tools, handoffs |
| **7-9** | Memory & State | Conversation memory, persistent state |
| **10-11** | RAG Pipeline | Document processing, vector search |
| **12-13** | Frontend | Chat interface with ChatKit |
| **14-16** | Production | Deployment, scaling, monitoring |

### Your Progress Path

```
Chapter 1  [YOU ARE HERE] → Foundation: APIs, environment, architecture
    │
Chapter 2-3  → First Agent: Basic agent that can use tools
    │
Chapter 4-6  → Smart Agent: Complex reasoning, multiple tools
    │
Chapter 7-9  → Memory: Agent remembers conversations and learns
    │
Chapter 10-11 → RAG: Agent searches your documents
    │
Chapter 12-13 → Frontend: Beautiful chat interface
    │
Chapter 14-16 → Production: Deploy to the real world
    │
    ▼
[DOCUBOT COMPLETE] → Production-ready RAG chatbot!
```

---

## 🤖 Apply to DocuBot Project

Time to create your own architecture document! This will serve as your roadmap throughout the course.

### Task

Create an `architecture.md` file with three sections:

1. **System Overview** — A diagram showing how components connect
2. **Component Descriptions** — What each part does (in your own words)
3. **Chapter Roadmap** — Which chapters build which components

### Outcome

*A documented architecture that serves as your roadmap for the entire course. When you finish a chapter, you'll update this document with what you learned.*

---

## 💡 Hints

Work through these hints to build your architecture document.

<details>
<summary><strong>Hint 1</strong> (Start Simple)</summary>

Begin with the most basic diagram — just boxes and arrows:

```
User → Frontend → Backend → Agent → Database
```

You don't need fancy graphics. Text-based diagrams are fine!

</details>

<details>
<summary><strong>Hint 2</strong> (Add Component Names)</summary>

Label each box with the actual technology:

```
User → ChatKit (Frontend) → FastAPI (Backend) → Agents SDK (Agent) → Qdrant (Vector DB)
```

</details>

<details>
<summary><strong>Hint 3</strong> (Describe In Your Words)</summary>

For each component, write 1-2 sentences explaining:
- What it does
- Why it's needed

Example:
> **Backend (FastAPI)**: Routes requests between the frontend and agent. Needed because the frontend shouldn't talk directly to the AI — the backend manages security and sessions.

</details>

<details>
<summary><strong>Hint 4</strong> (Map Chapters to Components)</summary>

Create a simple table:

```markdown
| Component | Chapters | What I'll Learn |
|-----------|----------|-----------------|
| Agent | 1-9 | How to build intelligent systems |
| RAG | 10-11 | Document search and retrieval |
| Frontend | 12-13 | Chat interface design |
| Production | 14-16 | Deployment and scaling |
```

</details>

<details>
<summary><strong>Hint 5</strong> (Complete Template)</summary>

Use this structure:

```markdown
# DocuBot Architecture

## System Overview

[Your diagram here — can be text-based ASCII art]

## Component Descriptions

### Frontend (ChatKit)
[Your description]

### Backend (FastAPI)
[Your description]

### Agent (OpenAI Agents SDK)
[Your description]

### RAG Pipeline
[Your description]

### Vector Database (Qdrant)
[Your description]

## Chapter Roadmap

[Your table mapping chapters to components]

## Why This Architecture?

[2-3 sentences explaining why DocuBot needs all these parts]
```

</details>

---

## Component Diagram Example

Here's a simple text-based diagram you can adapt:

```
                    ┌──────────────┐
                    │     User     │
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │   ChatKit    │  ← Frontend (Ch 12-13)
                    │  (Frontend)  │
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │   FastAPI    │  ← Backend (Ch 14-16)
                    │  (Backend)   │
                    └──────┬───────┘
                           │
              ┌────────────▼────────────┐
              │    OpenAI Agents SDK    │  ← Agent (Ch 1-9)
              │        (Agent)          │
              └────────────┬────────────┘
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                 │
   ┌─────▼─────┐    ┌──────▼──────┐   ┌──────▼──────┐
   │  OpenAI   │    │ RAG Pipeline│   │   Memory    │
   │   API     │    │ (Ch 10-11)  │   │  (Ch 7-9)   │
   └───────────┘    └──────┬──────┘   └─────────────┘
                          │
                   ┌──────▼──────┐
                   │   Qdrant    │  ← Vector DB (Ch 10-11)
                   │ (Vector DB) │
                   └─────────────┘
```

Feel free to modify this or create your own style!

---

## Chapter Roadmap Table

Reference this throughout your journey:

| Chapters | Component Focus | Skills You'll Gain |
|----------|-----------------|-------------------|
| 1-3 | Agent Foundation | API calls, basic tools, simple agents |
| 4-6 | Advanced Agent | Multi-step reasoning, tool composition |
| 7-9 | Memory Systems | Conversation history, persistent state |
| 10-11 | RAG Pipeline | Embeddings, vector search, document retrieval |
| 12-13 | Frontend UI | ChatKit setup, message handling, UX |
| 14-16 | Production | Deployment, monitoring, optimization |

---

## Try With AI

:::tip Practice with ChatGPT
Use [ChatGPT](https://chat.openai.com) to explore architecture concepts:

1. "Explain RAG (Retrieval-Augmented Generation) architecture for chatbots"
2. "What's the difference between a vector database and a regular database?"
3. "Why do AI chat applications need a backend API layer?"

Understanding these concepts now will make the coding chapters much clearer!
:::

---

## What's Next

Congratulations! You've completed Chapter 1! You now have:

- ✅ Understanding of what APIs are and how they work
- ✅ Clear distinction between LLMs and Agents
- ✅ A fully configured development environment
- ✅ A documented architecture roadmap

**Your DocuBot progress after Chapter 1:**
- Created: `test_api.py` (API verification)
- Created: `llm-vs-agent-comparison.md` (conceptual understanding)
- Created: `architecture.md` (system roadmap)
- Environment: UV + virtual env + .env configured

**In Chapter 2**, you'll write your first real agent code. The foundation is set — now the building begins!

---

**Next Chapter**: [Chapter 2 — Your First Agent](../02-your-first-agent/README.md)
