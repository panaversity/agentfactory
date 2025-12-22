---
sidebar_position: 54
title: "Chapter 54: Dapr Actors & Workflows"
description: "Build stateful agents with virtual actors and durable workflows"
---

# Chapter 54: Dapr Actors & Workflows

Stateless pub/sub and service invocation handle many patterns. But agents often need state—conversation history, user preferences, task progress. And long-running operations—multi-step workflows that survive restarts, with retries and compensation logic.

Dapr Actors provide virtual, stateful entities that encapsulate agent state. Dapr Workflows provide durable orchestration for long-running processes. Together, they enable sophisticated agent patterns like persistent conversations, scheduled reminders, and multi-step task automation.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand the Actor model**: Virtual actors, state encapsulation, and turn-based concurrency
- **Implement Dapr Actors**: Define actors for agent state (conversations, user sessions)
- **Manage actor lifecycle**: Activation, deactivation, and reminders
- **Use actor timers & reminders**: Schedule future operations
- **Design Dapr Workflows**: Durable, restartable orchestration
- **Implement workflow patterns**: Sequential, parallel, and saga patterns
- **Handle failures**: Compensation, retries, and error handling in workflows
- **Combine actors & workflows**: Actors for state, workflows for orchestration

## Chapter Structure

1. **The Actor Model** — Concepts, benefits, and when to use actors
2. **Dapr Actors Fundamentals** — Defining actors, state, and methods
3. **Actor State Management** — Persisting and retrieving actor state
4. **Timers & Reminders** — Scheduling future actor invocations
5. **Dapr Workflows Overview** — Durable orchestration concepts
6. **Workflow Implementation** — Defining workflows, activities, and child workflows
7. **Workflow Patterns** — Sequential, fan-out/fan-in, and sagas
8. **Capstone: Stateful Agent with Reminders** — Build an agent with persistent conversations and scheduled task reminders

## Prerequisites

- Chapter 53: Dapr Fundamentals (building blocks, components)
- Your agent service using Dapr pub/sub and state

## Looking Ahead

You now have a stateful, event-driven agent on Kubernetes. Chapter 55 automates deployments with CI/CD and GitOps, ensuring every code change flows to production safely.
