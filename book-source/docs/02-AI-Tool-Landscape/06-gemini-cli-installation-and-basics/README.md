---
sidebar_position: 6
title: "Chapter 6: Google Gemini CLI: Open Source and Everywhere"
---

# Chapter 6: Google Gemini CLI: Open Source and Everywhere

You've learned Claude Code inside and out. Now it's time for the bigger picture: **choosing the right AI tool for the job**.

Two months after Claude Code launched in October 2024, Google released Gemini CLI. On the surface, it looks like a competitor. But dig deeper, and you discover something different—not better or worse, just *different*. Open source. Generous free tier. One million token context window. Built on the Model Context Protocol (MCP), which means you can extend it with your own tools.

This chapter isn't about Google versus Anthropic. It's about **building judgment**—understanding when each tool is the right choice, and why context matters more than brand loyalty.

By the end of this chapter, you'll have two powerful AI development tools at your command, each suited to different workflows and scenarios.

## What You'll Learn

Through seven interconnected lessons, you'll understand:

- **Why open source matters**: Gemini CLI's transparency, customizability, and community model—and when those advantages matter for your work
- **Installation and getting started**: Quick setup on Windows, macOS, or Linux with a generous free tier (no credit card required)
- **Slash commands for orchestration**: `/memory`, `/stats`, `/mcp`—the commands that make you efficient
- **GEMINI.md for project context**: Automatic project understanding that persists across sessions
- **Memory checkpoints for persistence**: Save conversations and resume work across days without re-explaining
- **MCP servers for integration**: Connect Gemini to external systems (web browsing, APIs, GitHub, databases)
- **Custom commands for automation**: Build your own workflows and share them as reusable extensions
- **The decision framework**: When to choose Gemini CLI, when to choose Claude Code, and when to reach for something else entirely

## Lessons at a Glance

| Lesson | Topic | Time |
|--------|-------|------|
| 1 | Why Gemini CLI Matters | 15 min |
| 2 | Installation & First Steps | 40 min |
| 3 | Core Commands & Slash Commands | 40 min |
| 4 | GEMINI.md Context Files | 35 min |
| 5 | Persistent Memory & Sessions | 30 min |
| 6 | MCP Servers & Business Workflows | 45 min |
| 7 | Custom Commands & Extensions | 40 min |
| **Total** | **~3.5 hours core content** | |

## Prerequisites

Before starting, you should have:

- ✅ Completed Chapters 1-5 (basic understanding of AI development tools and Claude Code)
- ✅ Node.js 20+ installed ([nodejs.org](https://nodejs.org))
- ✅ Terminal comfort (navigating directories, running basic commands)
- ✅ A Google account (free tier requires login, not your work account)

## What You'll Be Able to Do

By completing this chapter, you'll:

1. **Install and authenticate** Gemini CLI on your machine in under 5 minutes
2. **Master slash commands** that orchestrate your work (save progress, check budget, manage integrations)
3. **Create GEMINI.md files** so Gemini understands your project automatically
4. **Save and load memory checkpoints** to resume work across days without re-explaining context
5. **Install and use MCP servers** to connect Gemini to GitHub, web browsing, and other systems
6. **Build custom commands** to automate your repeated workflows
7. **Decide which tool to use**: Claude Code or Gemini CLI, based on task requirements, not brand preference

## Chapter Structure

**Lesson 1: Why Gemini CLI Matters** — Understand the open source model, free tier economics, and when Gemini CLI is the right choice (and when Claude Code is better).

**Lesson 2: Installation & First Steps** — Get Gemini CLI running and have your first conversation in minutes.

**Lesson 3: Core Commands & Slash Commands** — Master the commands you'll use daily (`/memory`, `/stats`, `/mcp`, `/clear`, `/restore`, etc.).

**Lesson 4: GEMINI.md Context Files** — Create project context files with three-level hierarchy (global, project, subdirectory) so Gemini understands your codebase automatically.

**Lesson 5: Persistent Memory & Sessions** — Learn to checkpoint conversations and resume work across days without losing context.

**Lesson 6: MCP Servers & Business Workflows** — Deep dive into Playwright (active web browsing), Context7 (live API docs), and GitHub integration with real business scenarios.

**Lesson 7: Custom Commands & Extensions** — Build TOML-based custom commands for your workflows and share them as reusable extensions.

## Self-Assessment

You've mastered this chapter when you can:

- ✅ Install Gemini CLI and authenticate with your Google account
- ✅ Use Gemini CLI for a realistic task (analyze a CSV file, research a topic, understand an API)
- ✅ Explain why you chose Gemini CLI over Claude Code for that specific task
- ✅ Create a GEMINI.md file for one of your projects
- ✅ Save and load a memory checkpoint across two sessions
- ✅ Set up one MCP server (GitHub, for example) and use it in a workflow
- ✅ Create a custom command to automate one of your repeated tasks

## Architecture: Learning Progression

This chapter follows a **building-block approach**, similar to Chapter 5 (Claude Code):

**Foundation (Lessons 1-2):**
- Context (why Gemini CLI exists)
- Installation (getting started)

**Orchestration (Lesson 3):**
- Slash commands (how you control Gemini)

**Context Management (Lessons 4-5):**
- GEMINI.md (project understanding)
- Memory checkpoints (conversation persistence)

**Integration (Lesson 6):**
- MCP servers (external systems)

**Extension (Lesson 7):**
- Custom commands (automation)
- Extensions (bundled capabilities)

Each lesson builds on previous ones. By the end, you understand the complete system and can orchestrate sophisticated workflows.

---

**Ready to start?** Begin with [Lesson 1: Why Gemini CLI Matters](./01-why-gemini-cli-matters.md) to understand when and why to choose Gemini CLI as your AI development partner.

