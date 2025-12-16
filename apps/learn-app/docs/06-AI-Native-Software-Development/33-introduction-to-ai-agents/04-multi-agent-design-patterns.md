---
title: "Multi-Agent Design Patterns"
sidebar_position: 4
description: "Match design patterns to problems: Coordinator, Sequential, Iterative Refinement, Human-in-the-Loop."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 20 minutes
learning_objectives:
  - "Identify when to use each of the four patterns"
  - "Match patterns to real-world problems"
skills:
  design_patterns:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# Multi-Agent Design Patterns

Single agents hit limits. Complex tasks need multiple specialists working together. Four patterns cover most use cases.

## Why Multiple Agents?

A single agent trying to do everything faces problems:
- **Context overload**: Too many tools and responsibilities
- **Conflicting objectives**: Security analysis requires different thinking than performance optimization
- **Debugging difficulty**: When something fails, where did it go wrong?

Multiple specialized agents solve these problems through division of labor.

## The Four Patterns

| Pattern | Structure | When to Use |
|---------|-----------|-------------|
| **Coordinator** | One routes, many execute in parallel | Independent subtasks |
| **Sequential** | Each output feeds the next | Ordered pipeline |
| **Iterative Refinement** | Generator and critic loop | Quality improvement |
| **Human-in-the-Loop** | Agent pauses for human approval | High-stakes decisions |

## Coordinator Pattern

One agent routes tasks to specialists who work in parallel.

```
User Request → Coordinator → [Security Agent]
                          → [Performance Agent]  → Coordinator → Response
                          → [Style Agent]
```

**Use when**: Subtasks are independent. Results need synthesis but don't depend on each other.

### Example: Code Audit

A coordinator receives "audit this codebase" and delegates:
- **Security agent**: Checks for vulnerabilities, injection risks
- **Performance agent**: Identifies bottlenecks, inefficient patterns
- **Documentation agent**: Evaluates code clarity, missing comments

All three work simultaneously on the same codebase. The coordinator synthesizes their findings into a unified report.

### Why Coordinator Works Here

Each analysis is independent. The security agent doesn't need performance results to do its job. Parallel execution saves time. If one agent fails, the others still complete.

### When NOT to Use Coordinator

If results depend on each other, coordinator creates problems. "Write a function, then test it" requires sequence—you can't test code that doesn't exist yet.

## Sequential Pattern

Each agent's output becomes the next agent's input.

```
Input → Agent A → Agent B → Agent C → Output
```

**Use when**: Steps must happen in order. Later steps depend on earlier results.

### Example: Content Pipeline

1. **Research agent**: Gathers information on a topic
2. **Writing agent**: Creates draft from research
3. **Editor agent**: Polishes draft for clarity and style

Each step depends on the previous. Can't write without research. Can't edit without a draft.

### Example: Claude Code Refactoring

When you ask Claude Code to refactor a module:

1. **Read phase**: Understand current code structure
2. **Analyze phase**: Identify patterns to improve
3. **Plan phase**: Design changes
4. **Implement phase**: Make edits
5. **Test phase**: Verify changes work

Each phase needs the previous phase's output. This is sequential even within a single agent.

### Why Sequential Works Here

Order matters. The writing agent needs research results as input. The editor needs the draft. Skipping steps or reordering breaks the pipeline.

## Iterative Refinement Pattern

Agents critique and improve each other's work in cycles.

```
Generator → Critic → Generator (revised) → Critic → ... → Approved Output
```

**Use when**: Quality matters more than speed. First attempts need improvement.

### Example: Code Review Loop

1. **Coding agent**: Writes implementation
2. **Review agent**: Identifies issues, suggests improvements
3. **Coding agent**: Revises based on feedback
4. **Review agent**: Re-evaluates
5. Repeat until review agent approves

### The Critic's Role

The critic agent has specific evaluation criteria:
- Does the code handle edge cases?
- Are there security vulnerabilities?
- Is the logic clear and maintainable?

Each critique round makes the output better. The loop terminates when quality threshold is met.

### Quality vs Speed Trade-off

More iterations = better quality but slower output. Design decisions:
- Maximum iterations (prevent infinite loops)
- Quality threshold (when is "good enough"?)
- Escalation path (what if quality threshold can't be met?)

### Example: Document Generation

1. **Writer**: Produces first draft
2. **Fact-checker**: Verifies claims, flags inaccuracies
3. **Writer**: Corrects errors
4. **Fact-checker**: Re-verifies
5. Continue until all facts verified

The iterative pattern catches errors that a single pass would miss.

## Human-in-the-Loop Pattern

Agents pause for human approval before critical actions.

```
Agent work → Human approval gate → Agent continues
```

**Use when**: Stakes are high. Actions are irreversible. Legal, financial, or safety implications exist.

### Example: Refund Agent

- Under $100: Automatic approval
- $100-$500: Agent processes, logs for review
- Over $500: Requires human approval before processing
- Over $1000: Escalates to manager

The thresholds reflect risk tolerance. Small refunds aren't worth human time. Large refunds need oversight.

### Designing Approval Gates

Good approval gates are:
- **Clear**: Agent knows exactly what triggers human review
- **Efficient**: Most requests don't need human input
- **Safe**: High-risk actions always get reviewed

Bad approval gates:
- Too aggressive: Humans review everything (defeats the purpose of automation)
- Too permissive: Risky actions proceed without oversight

### Example: Code Deployment

Claude Code implementing a feature:

1. Agent makes changes
2. Agent runs tests
3. If tests pass: "Should I commit these changes?" (human gate)
4. If approved: Commit and optionally push

The human gate before commit prevents unwanted changes from persisting.

## Pattern Selection Guide

Match the pattern to the problem structure:

| Problem Structure | Best Pattern | Reasoning |
|-------------------|--------------|-----------|
| Multiple independent analyses | Coordinator | Parallelize for speed |
| Clear step-by-step process | Sequential | Maintain order dependencies |
| Quality-critical output | Iterative Refinement | Improve through cycles |
| High-risk decisions | Human-in-the-Loop | Human oversight for safety |

### Patterns Combine

Real systems often use multiple patterns:

**Customer Support System**:
- **Coordinator** routes requests to specialists (billing, technical, shipping)
- **Sequential** within each specialist's workflow (gather context → diagnose → resolve)
- **Human-in-the-Loop** for escalations (large refunds, angry customers, policy exceptions)

**Document Generation System**:
- **Sequential** pipeline (research → write → edit)
- **Iterative Refinement** at each stage (write → fact-check → revise)
- **Human-in-the-Loop** for final approval

## Claude Code's Patterns

Claude Code uses all four patterns internally:

**Coordinator**: When auditing a codebase, it may run security, performance, and style checks in parallel (using subagents).

**Sequential**: When refactoring, it reads → analyzes → plans → implements → tests in sequence.

**Iterative Refinement**: When tests fail, it revises code and re-runs until tests pass.

**Human-in-the-Loop**: When it asks "Should I proceed with this change?" before destructive operations.

Understanding these patterns helps you predict Claude Code's behavior and guide it more effectively.

## Try With AI

Use Claude, ChatGPT, or Gemini to practice pattern matching.

> "A contract review system needs to: extract key terms, check against templates, identify risks, verify legal compliance. Which pattern? Why not the others?"

**Expected**: Sequential—each step depends on the previous. Can't check compliance before extracting terms. Can't identify risks before comparing to templates.

> "A customer support system handles 100K requests daily. Compare a single super-agent vs a Coordinator with 5 specialists. What matters at scale?"

**Expected**: Coordinator—specialists can parallelize, failures are isolated, easier to debug and improve individual components. Single agent becomes a bottleneck.

> "A billing agent can refund up to $500 automatically. Design the Human-in-the-Loop pattern for amounts above that. What thresholds make sense?"

**Expected**: $500-$1000 might need supervisor approval. Above $1000 might need manager approval. Thresholds based on business risk tolerance.

> "I'm building a code generation system. First draft quality is usually around 70%. I need 95%+. Which pattern? How many iterations should I allow?"

**Expected**: Iterative Refinement with a critic agent. Allow 3-5 iterations with clear quality criteria. Escalate to human review if quality threshold not met after max iterations.

**Key insight**: Wrong patterns don't just underperform—they create failure modes. Match pattern to problem structure.
