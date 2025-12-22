---
sidebar_position: 35
title: "Chapter 35: Google Agent Development Kit (ADK)"
description: "Build agents with Google's ADK for Gemini-powered agentic applications"
---

# Chapter 35: Google Agent Development Kit (ADK)

You've built agents with OpenAI's SDK in Chapter 34. Now you'll learn Google's Agent Development Kit (ADK)—a framework designed for Gemini models with deep integration into Google Cloud's ecosystem. Understanding multiple frameworks isn't about choosing sides; it's about matching tools to requirements.

Google ADK emphasizes declarative agent definition, strong typing, and seamless integration with Vertex AI. Where OpenAI's SDK focuses on simplicity and handoffs, ADK provides richer orchestration primitives and native support for Google's multimodal capabilities. You'll see how the same architectural patterns from Chapter 33 manifest differently across frameworks.

This chapter continues the specification-driven approach. You'll compare how the same agent specification translates to different SDK implementations, building intuition for when each framework excels.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Configure Google ADK**: Set up authentication with Google Cloud, configure Vertex AI, and understand ADK's agent primitives
- **Define agents declaratively**: Use ADK's YAML-based agent definitions alongside Python implementations
- **Leverage Gemini's multimodal capabilities**: Build agents that process images, audio, and video alongside text
- **Implement ADK's orchestration patterns**: Use ADK's built-in support for sequential, parallel, and conditional agent flows
- **Connect to Google Cloud services**: Integrate agents with BigQuery, Cloud Functions, and other GCP services
- **Compare SDK implementations**: Translate the same specification to both OpenAI and Google implementations

## Chapter Structure

1. **ADK Setup & Architecture** — Google Cloud authentication, Vertex AI configuration, ADK primitives vs OpenAI SDK comparison
2. **Declarative Agent Definition** — YAML agent configs, instruction templates, and tool bindings
3. **Multimodal Agent Capabilities** — Processing images, audio, and video with Gemini models
4. **Orchestration Patterns in ADK** — Sequential flows, parallel execution, conditional branching, and loops
5. **Google Cloud Integration** — BigQuery for data, Cloud Functions for compute, Pub/Sub for events
6. **Cross-Framework Comparison** — Same specification, different implementations—when to choose which
7. **Capstone: Research Assistant** — Spec-driven multimodal agent that searches, analyzes documents, and synthesizes findings

## Prerequisites

- Chapter 33: Introduction to AI Agents (conceptual foundation)
- Chapter 34: OpenAI Agents SDK (comparison baseline)
- Part 5: Python Fundamentals (async/await, type hints)
- Google Cloud account with Vertex AI access
