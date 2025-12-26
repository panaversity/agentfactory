---
sidebar_position: 40
title: "Chapter 40: FastAPI for Agents"
description: "Expose the TaskManager Agent as a production-ready REST API with FastAPI"
---

# Chapter 40: FastAPI for Agents

You've built the **TaskManager Agent** with the OpenAI Agents SDK (Chapter 34), connected it via MCP (Chapters 37-38), and added reusable skills (Chapter 39). Now you'll expose it as an HTTP service. FastAPI is the natural choice: Python-native, async-first, automatic OpenAPI documentation, and battle-tested in production.

This chapter has two phases:
- **Lessons 1-5**: Learn FastAPI fundamentals using a simple Task API
- **Lessons 6-8**: Expose the TaskManager Agent from Chapter 34 via REST endpoints

By the end, the TaskManager Agent you built locally is callable via REST—triage routing, specialist handoffs, tool execution, streaming responses—all accessible to any HTTP client.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Create FastAPI applications**: Build HTTP APIs with automatic documentation and validation
- **Implement CRUD operations**: Design RESTful endpoints for Create, Read, Update, Delete
- **Handle errors properly**: Return appropriate HTTP status codes (200, 201, 400, 404, 422)
- **Use dependency injection**: Organize code with FastAPI's Depends() pattern
- **Stream responses**: Implement Server-Sent Events (SSE) for real-time updates
- **Expose TaskManager Agent**: Wrap the multi-agent system from Chapter 34 in REST endpoints
- **Stream agent responses**: SSE with tool calls, handoffs, and completion events

## Chapter Structure

| # | Lesson | Focus |
|---|--------|-------|
| 1 | Hello FastAPI | First app, uvicorn, Swagger UI |
| 2 | POST and Pydantic Models | Request validation, models |
| 3 | Full CRUD Operations | GET, PUT, DELETE patterns |
| 4 | Error Handling | HTTPException, status codes |
| 5 | Dependency Injection | Depends(), repository pattern |
| 6 | Streaming with SSE | Server-Sent Events foundation |
| 7 | Agent Integration | TaskManager Agent with handoffs |
| 8 | Capstone: TaskManager Agent Service | Complete multi-agent API |

## Prerequisites

- **Chapter 34**: OpenAI Agents SDK (Agent, Runner, function_tool, handoffs)
- **Chapters 37-38**: MCP experience (HTTP/SSE patterns familiar)
- **Part 5**: Python Fundamentals (async/await, type hints, Pydantic)

## The Running Example

Lessons 1-5 build a **Task API**—a simple CRUD service for task management. This teaches FastAPI patterns without agent complexity.

Lessons 6-8 transition to the **TaskManager Agent**—the same multi-agent system you built in Chapter 34. You'll recognize the triage pattern, specialist agents, and handoffs. The difference: now they're accessible via HTTP endpoints instead of CLI.

## Looking Ahead

This chapter gives you a REST API exposing full TaskManager capabilities. Chapter 41 (ChatKit Server) builds on this foundation to add conversational infrastructure—the session management, turn-taking, and chat-specific patterns that conversational interfaces require.
