---
sidebar_position: 34
title: "Chapter 34: OpenAI Agents SDK"
description: "Build production agents with OpenAI's official Python SDK for agentic applications"
---

# Chapter 34: OpenAI Agents SDK

Chapter 33 gave you the conceptual foundation—agent taxonomy, architecture, and patterns. Now you'll build your first production agent using OpenAI's official Agents SDK, the framework powering ChatGPT's agentic capabilities.

OpenAI released the Agents SDK in March 2025 as a lightweight, production-ready framework for building agentic applications. Unlike wrapper libraries, it's the same infrastructure OpenAI uses internally. The SDK embodies the architecture you learned: agents with instructions, tools they can call, and handoffs between specialized agents. You'll recognize every pattern from Chapter 33 as you implement them.

This chapter follows specification-driven development throughout. You'll write specs first, then implement with Claude Code as your pair programmer. By the end, you'll have built a multi-agent system that demonstrates the patterns enterprises use in production.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Configure the OpenAI Agents SDK**: Set up your development environment, manage API keys securely, and understand the SDK's core primitives (Agent, Runner, tools, handoffs)
- **Build single agents with tools**: Create agents that use function tools to interact with external systems, implementing the Tool-Use pattern from Chapter 33
- **Implement multi-agent handoffs**: Design systems where specialized agents hand off to each other, applying the Coordinator and Sequential patterns
- **Add guardrails and safety**: Implement input/output validation, content filtering, and the Human-in-the-Loop pattern for sensitive operations
- **Handle streaming and async**: Build responsive agents that stream responses and handle concurrent operations efficiently
- **Trace and debug agents**: Use OpenAI's tracing infrastructure to understand agent behavior and diagnose issues

## Chapter Structure

1. **SDK Setup & First Agent** — Environment configuration, API key management, creating your first agent with instructions
2. **Function Tools Deep Dive** — Defining tools, parameter schemas, error handling, and the tool execution lifecycle
3. **Multi-Agent Handoffs** — Agent-to-agent communication, context preservation, and handoff patterns
4. **Guardrails & Safety** — Input validation, output filtering, tripwires, and human approval workflows
5. **Streaming & Async Patterns** — Real-time responses, concurrent tool execution, and event handling
6. **Tracing & Observability** — Built-in tracing, custom spans, debugging agent behavior
7. **Capstone: Customer Support System** — Spec-driven implementation of a multi-agent support system with triage, specialist handoffs, and escalation

## Prerequisites

- Chapter 33: Introduction to AI Agents (conceptual foundation)
- Part 5: Python Fundamentals (async/await, type hints, dataclasses)
- Part 4: SDD-RI Fundamentals (specification-driven workflow)
- OpenAI API key with Agents SDK access
