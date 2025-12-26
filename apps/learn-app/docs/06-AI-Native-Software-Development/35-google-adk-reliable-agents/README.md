---
sidebar_position: 35
title: "Chapter 35: Building Custom Agents with Google ADK"
description: "Master Google's Agent Development Kit to build production-grade AI agents with Gemini models"
---

# Chapter 35: Building Custom Agents with Google ADK

You've built agents with OpenAI's SDK in Chapter 34. Now you'll learn Google's Agent Development Kit (ADK)—a framework designed for Gemini models with deep integration into Google Cloud's ecosystem. ADK powers Google's own Agentspace and Customer Engagement Suite, making it a production-proven choice for enterprise agents.

Google ADK emphasizes **declarative agent definition**, **strong typing**, and **native multimodal support**. Where OpenAI's SDK focuses on simplicity and handoffs, ADK provides richer orchestration primitives including workflow agents (Sequential, Parallel, Loop) and comprehensive callback systems for safety and control.

This chapter builds a complete AI News Podcast Agent—from a simple search agent to a multi-agent system that researches news, enriches it with financial data, and generates audio podcasts.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Build ADK agents**: Create agents with the Agent class, configure models, and write effective instructions
- **Create custom tools**: Build function tools with type annotations and docstrings that ADK auto-wraps
- **Manage state**: Use ToolContext and SessionService for persistent conversations
- **Implement callbacks**: Add guardrails with before/after callbacks for safety and control
- **Orchestrate multi-agent systems**: Use AgentTool to compose agents and coordinate complex workflows
- **Use workflow agents**: Build deterministic pipelines with SequentialAgent, ParallelAgent, and LoopAgent
- **Deploy to production**: Configure Vertex AI and understand deployment options

## Chapter Structure

1. **Your First ADK Agent** — Installation, Agent class, google_search tool, adk commands (run, web)
2. **Custom Function Tools** — Type annotations, docstrings, external API integration (yfinance example)
3. **Session State & Memory** — ToolContext, state management, conversation persistence
4. **Coordinator Patterns** — Background processing, file persistence, coordinator-dispatcher architecture
5. **Callbacks & Guardrails** — before_tool_callback, after_tool_callback, domain filtering, response enhancement
6. **Multi-Agent Orchestration** — AgentTool, Pydantic schemas for structured output, agent delegation
7. **Workflow Agents** — SequentialAgent, ParallelAgent, LoopAgent for deterministic pipelines
8. **Capstone: AI News Podcast Agent** — Complete multi-agent system with research, enrichment, and audio generation

## Prerequisites

- Chapter 33: Introduction to AI Agents (conceptual foundation)
- Chapter 34: OpenAI Agents SDK (comparison baseline)
- Part 5: Python Fundamentals (async/await, type hints)
- Google API Key (free tier) or Google Cloud account with Vertex AI access

## Key Differences from OpenAI SDK

| Feature | OpenAI SDK | Google ADK |
|---------|-----------|------------|
| Agent routing | Handoffs (flexible) | Workflow agents (deterministic) + LLM routing |
| Tool definition | Pydantic models | Function with docstring (auto-wrapped) |
| State management | context_variables | ToolContext + SessionService |
| Safety | Input/output guardrails | 6 callback types (before/after for agent, tool, model) |
| Deployment | Self-hosted | Vertex AI Agent Engine (managed) |
| Multimodal | Via API | Native Gemini support (live voice, images) |

## Running Example: AI News Podcast Agent

Throughout this chapter, you'll build an increasingly sophisticated news agent:

1. **Lesson 1**: Simple agent that searches for AI news
2. **Lesson 2**: Add financial data tool (yfinance)
3. **Lesson 3**: Persist conversation state
4. **Lesson 4**: Save research to markdown files
5. **Lesson 5**: Add callback-based guardrails
6. **Lesson 6**: Delegate to podcaster agent
7. **Lesson 7**: Use workflow agents for pipelines
8. **Lesson 8**: Complete podcast generation system
