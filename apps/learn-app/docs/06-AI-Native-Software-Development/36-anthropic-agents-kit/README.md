---
sidebar_position: 36
title: "Chapter 36: Anthropic Claude Agent SDK"
description: "Build agents with Anthropic's Claude SDK and understand the architecture behind Claude Code"
---

# Chapter 36: Anthropic Claude Agent SDK

You've built agents with OpenAI and Google frameworks. Now you'll learn Anthropic's Claude Agent SDK—the architecture that powers Claude Code, the tool you've been using throughout this book. Understanding how Claude Code works internally transforms you from user to architect.

Anthropic's approach emphasizes constitutional AI principles, computer use capabilities, and the extended thinking that makes Claude distinctive. The SDK provides primitives for building agents that can use computers like humans do—clicking, typing, navigating—alongside traditional tool use. You'll see how the general agent (Claude Code) and custom agents (your creations) share the same architectural foundation.

This chapter completes your framework survey. By the end, you'll understand three major approaches to agent development and know how to choose based on requirements, not vendor preference.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Configure the Claude Agent SDK**: Set up authentication, understand the SDK's architecture, and connect to Claude models
- **Implement tool use patterns**: Define tools using Anthropic's schema format, handle tool results, and manage conversation context
- **Build computer-use agents**: Create agents that interact with desktop applications, browsers, and GUIs
- **Apply extended thinking**: Leverage Claude's reasoning capabilities for complex multi-step problems
- **Understand Claude Code's architecture**: See how skills, subagents, and MCP servers compose into the agent you use daily
- **Compare all three frameworks**: Synthesize learnings across OpenAI, Google, and Anthropic approaches

## Chapter Structure

1. **Claude SDK Setup & Architecture** — Authentication, model selection, and SDK primitives
2. **Tool Use with Claude** — Tool definitions, parameter handling, and the tool-use conversation loop
3. **Computer Use Capabilities** — Screen reading, mouse/keyboard control, and GUI automation patterns
4. **Extended Thinking & Reasoning** — Leveraging Claude's chain-of-thought for complex problems
5. **Inside Claude Code** — Skills as reusable intelligence, subagent orchestration, and MCP integration
6. **Framework Synthesis** — Decision matrix for choosing frameworks based on requirements
7. **Capstone: Development Assistant** — Spec-driven agent that combines tool use, computer use, and reasoning for development workflows

## Prerequisites

- Chapter 33: Introduction to AI Agents (conceptual foundation)
- Chapter 34: OpenAI Agents SDK (first framework experience)
- Chapter 35: Google ADK (second framework comparison)
- Part 5: Python Fundamentals (async/await, type hints)
- Anthropic API key with Claude access
