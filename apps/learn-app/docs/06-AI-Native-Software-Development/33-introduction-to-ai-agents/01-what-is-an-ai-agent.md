---
title: "What Is an AI Agent?"
sidebar_position: 1
description: "Define AI agents using the 5-Level Taxonomy and understand the director vs bricklayer paradigm shift."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 20 minutes
learning_objectives:
  - "Distinguish agents from chatbots using the loop-based definition"
  - "Classify systems using the 5-Level Taxonomy (Level 0-4)"
  - "Explain the director vs bricklayer paradigm shift"
skills:
  agent_classification:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# What Is an AI Agent?

You've been using an AI agent for months. Claude Code reasons about your requests, uses tools, observes results, and iterates until it succeeds. That's not how ChatGPT works. This lesson explains what makes Claude Code different—and why that difference matters.

## The Definition

An AI agent uses a language model **in a loop** to accomplish a goal:

1. **Reason** about what to do
2. **Act** using tools
3. **Observe** the result
4. **Repeat** until done

ChatGPT responds once. Claude Code loops until your task is complete. That loop is what makes something an agent.

When you ask Claude Code to "refactor this code and verify it works," it:
- Reads code (observes)
- Plans refactoring approach (reasons)
- Writes new code (acts)
- Runs tests (observes)
- Fixes failures (loops)

You didn't specify those steps. The agent reasoned through them autonomously.

## The 5-Level Taxonomy

Not all agents are equal. The taxonomy classifies systems by autonomy:

| Level | Name | Who Controls Strategy? | Example |
|-------|------|----------------------|---------|
| 0 | Core Reasoning | No one (single response) | ChatGPT without tools |
| 1 | Connected | Human drives each step | ChatGPT with web search |
| 2 | Strategic | System plans multi-step | Claude Code on a refactor |
| 3 | Multi-Agent | System coordinates specialists | Code + test + docs agents |
| 4 | Self-Evolving | System creates new tools | Agent builds custom analyzers |

**Level 0**: Model responds once. No tools, no loop.

**Level 1**: Model uses tools, but you drive the strategy. "Search for X" → results → you decide next step.

**Level 2**: Model plans and executes multi-step strategies. You set the goal; it figures out how. Claude Code typically operates here.

**Level 3**: Multiple specialized agents coordinate. A code agent, test agent, and docs agent work together. Claude Code does this on complex tasks.

**Level 4**: The system creates new capabilities to solve novel problems. Rare today.

### Where Claude Code Fits

Claude Code is Level 2-3 depending on the task:

**Level 2 example**: "Debug this error."
- Asks clarifying questions
- Requests logs and code
- Reasons through causes
- Suggests and tests fixes
- Iterates until solved

**Level 3 example**: "Audit this codebase for security, performance, and correctness."
- Security specialist examines auth and data handling
- Performance analyst checks bottlenecks
- Quality agent reviews tests and docs
- Coordinator synthesizes findings

You experience one interface. Behind it, multiple perspectives coordinate.

## Director vs Bricklayer

The paradigm shift that matters:

**Bricklayer** (traditional): You specify every step. "Read file X, parse format Y, handle error Z." You're the executor.

**Director** (agent era): You specify intent and constraints. "Extract structured data from these documents. Ensure quality." The agent reasons about how.

| Bricklayer | Director |
|------------|----------|
| "Write validation with these exact checks" | "Validate input securely" |
| "Call API A, retry 3 times on timeout" | "Fetch data reliably" |
| "Parse JSON, extract fields X, Y, Z" | "Get the relevant information" |

Director-level thinking is harder. You must articulate intent clearly, anticipate failure modes, and trust the system to reason. But it's more powerful—agents adapt to situations you didn't anticipate.

This is why specification skills (from Part 4) become critical. Vague goals create unreliable agents. Precise specifications create predictable systems.

## Why This Matters

Agent development skills are scarce. Most developers can *use* ChatGPT. Few can *design* agent systems that work reliably.

The gap: Companies need people who can specify agent behavior precisely, debug agent reasoning, and decide when to use agents vs traditional code. That's what Chapters 34-36 teach.

## Try With AI

Use Claude, ChatGPT, or Gemini to practice classification.

> "A bank chatbot asks for my account number to check my balance. What level in the 5-Level Taxonomy? Why not higher?"

**Expected**: Level 1—has tools but humans drive strategy.

> "When Claude Code refactors a 500-line file and improves test coverage, describe its reason-act-observe loop. What level?"

**Expected**: Level 2-3—it plans strategy, executes multi-step, may coordinate specialists.

> "I tell an agent 'validate user input securely' vs 'write these exact validation checks.' Which is director thinking? Why is it harder but more powerful?"

**Expected**: First is director. Harder because you must specify intent precisely. More powerful because the agent adapts.

**Key insight**: The level isn't about sophistication of tools—it's about who controls the reasoning loop.
