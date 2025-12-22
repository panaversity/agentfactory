---
sidebar_position: 39
title: "Chapter 39: Agent Skills & MCP Code Execution"
description: "Create reusable agent skills and enable safe code execution through MCP sandboxes"
---

# Chapter 39: Agent Skills & MCP Code Execution

You learned to create skills in Chapter 5—SKILL.md files that encode domain expertise for Claude Code. You built MCP servers in Chapter 38. Now these concepts converge: skills become the intelligence layer that guides agents, while MCP code execution provides the sandbox where agents can safely run generated code.

This chapter bridges reusable intelligence (skills) with executable capability (code interpreters). Together, they transform agents from conversational assistants into autonomous problem-solvers that can write code, execute it, analyze results, and iterate—all within safe boundaries you control.

The combination is powerful: skills provide the "what to do" (domain patterns, best practices, decision frameworks), while code execution provides the "how to do it" (safe sandboxes for running Python, JavaScript, or other languages). This is the pattern behind data analysis agents, research assistants, and automated testing systems.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Design agent skills**: Structure SKILL.md files that guide agent behavior with personas, principles, and decision frameworks
- **Compose skill hierarchies**: Build skills that reference other skills, creating layered intelligence
- **Configure code execution MCP servers**: Set up sandboxed environments for safe code execution (E2B, Code Interpreter, local Docker)
- **Implement execution workflows**: Design patterns where agents write code, execute it, analyze output, and iterate
- **Apply safety boundaries**: Configure resource limits, timeout policies, and allowed operations for sandboxed execution
- **Combine skills with execution**: Create agents that use skills for guidance and code execution for implementation

## Chapter Structure

1. **Skills Architecture Deep Dive** — SKILL.md structure, persona patterns, and the Questions + Principles framework
2. **Skill Composition Patterns** — Skill dependencies, inheritance, and building skill libraries
3. **Code Execution Fundamentals** — Why sandboxing matters, execution models, and security considerations
4. **MCP Code Execution Servers** — E2B integration, Code Interpreter setup, and Docker-based sandboxes
5. **Execution Workflow Patterns** — Write-execute-analyze loops, error recovery, and iterative refinement
6. **Safety & Resource Management** — CPU/memory limits, network policies, filesystem restrictions, and timeout handling
7. **Skills + Execution Integration** — Agents that use skills for strategy and code execution for tactics
8. **Capstone: Data Analysis Agent** — Spec-driven agent that uses domain skills to guide analysis and code execution to process data

## Prerequisites

- Chapter 5: Claude Code Skills (SKILL.md fundamentals)
- Chapter 38: Custom MCP Servers (MCP implementation experience)
- Part 5: Python Fundamentals (code that agents will generate and execute)
