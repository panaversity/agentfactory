---
sidebar_position: 39
title: "Chapter 39: Agent Skills & MCP Code Execution"
description: "Build skills that implement the code execution pattern - from MCP-wrapping skills to script execution to full workflow orchestration"
---

# Chapter 39: Agent Skills & MCP Code Execution

You learned to create skills in Chapter 5—SKILL.md files that encode domain expertise for Claude Code. You built MCP servers in Chapter 38. Now you'll build skills that don't just advise—they EXECUTE.

The code execution pattern transforms agents from assistants that suggest solutions into autonomous systems that solve problems. A skill orchestrates code execution (MCP calls, scripts, data processing), analyzes results, and iterates until the problem is solved.

**Why MCP first?** You already know MCP from Chapters 37-38. MCP-wrapping skills like `fetching-library-docs` (which wraps Context7) show the pattern clearly: the skill adds an intelligence layer on top of raw MCP capabilities—deciding when to call, how to filter results, how to handle errors. Once you see this pattern with MCP, extending it to script execution becomes natural.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Design execution skill patterns**: Create personas, questions, and principles that encode autonomous behavior
- **Build MCP-wrapping skills**: Add intelligence layers on top of MCP servers (like `fetching-library-docs` wrapping Context7)
- **Implement script execution skills**: Write, execute, and iterate on Python/Bash scripts autonomously
- **Handle errors gracefully**: Recover from syntax errors, runtime failures, and timeouts
- **Orchestrate full workflows**: Combine MCP + scripts into multi-step execution pipelines
- **Ship Digital FTEs**: Package complete execution skills as deployable, sellable assets

## The Learning Arc

| Phase | What You Build | Example |
|-------|----------------|---------|
| **Foundation** (Lessons 1-2) | Advanced skill patterns | Persona + Questions + Principles for execution |
| **MCP First** (Lessons 3-4) | Skills that wrap MCP servers | Like `fetching-library-docs` wrapping Context7 |
| **Scripts Next** (Lessons 5-6) | Skills that write + execute scripts | Data processing, file manipulation |
| **Full Workflows** (Lesson 7) | Skills that orchestrate everything | MCP + scripts + iteration + error recovery |
| **Ship It** (Lesson 8) | Complete, shippable skill | Capstone Digital FTE |

## Chapter Structure

| Lesson | Title | Layer | Duration |
|--------|-------|-------|----------|
| 1 | Advanced Skill Patterns | L1 (Manual) | 25 min |
| 2 | Skill Composition & Dependencies | L2 (Collaboration) | 30 min |
| 3 | Anatomy of MCP-Wrapping Skills | L1 (Manual) | 30 min |
| 4 | Build Your MCP-Wrapping Skill | L2 (Collaboration) | 45 min |
| 5 | Script Execution Fundamentals | L1 (Manual) | 30 min |
| 6 | Build Script-Execution Skill | L2 (Collaboration) | 50 min |
| 7 | Full Workflow Orchestration | L3 (Intelligence) | 50 min |
| 8 | Capstone: Shippable Agent Skill | L4 (Spec-Driven) | 90 min |

**Total: ~6 hours**

## Prerequisites

- **Chapter 5**: Claude Code Skills (SKILL.md structure, basic skill creation)
- **Chapter 37**: MCP Fundamentals (architecture, client configuration)
- **Chapter 38**: Custom MCP Servers (building MCP servers you can now wrap)
- **Part 5**: Python Fundamentals (for script generation and execution)

## Success Criteria

By chapter completion, you will:
- Analyze existing MCP-wrapping skills and explain their intelligence layer
- Build a skill that wraps an MCP server with proper triggering and 30%+ token reduction
- Build a skill that writes, executes, and iterates on Python scripts
- Implement error recovery handling syntax, runtime, and timeout errors
- Complete a capstone: shippable skill implementing the full code execution pattern
