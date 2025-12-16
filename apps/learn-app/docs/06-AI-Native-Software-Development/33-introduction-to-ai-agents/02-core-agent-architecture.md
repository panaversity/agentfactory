---
title: "Core Agent Architecture"
sidebar_position: 2
description: "Understand the 3+1 architecture that every agent needs: Model, Tools, Orchestration, and Deployment."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 20 minutes
learning_objectives:
  - "Identify the four components of agent architecture"
  - "Explain how orchestration coordinates reasoning and tools"
  - "Recognize these components in Claude Code"
skills:
  agent_architecture:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# Core Agent Architecture

Every agent—from Claude Code to enterprise customer support systems—shares the same four components. Understanding this architecture helps you design agents and debug when things go wrong.

## The 3+1 Architecture

| Component | Analogy | Function |
|-----------|---------|----------|
| **Model** | Brain | Reasons about what to do next |
| **Tools** | Hands | Acts on the world |
| **Orchestration** | Nervous System | Coordinates reasoning, planning, memory |
| **Deployment** | Body | Makes the agent accessible |

The "3+1" name reflects that Model, Tools, and Orchestration are the core trio, while Deployment wraps them for production use.

## Model (The Brain)

The language model is where reasoning happens. It decides what to do next based on context.

**Different tasks need different models:**

| Task Type | Model Tier | Example |
|-----------|------------|---------|
| Classification, routing | Fast/cheap | "Is this a billing question or technical support?" |
| Most reasoning tasks | Medium | "Analyze this customer's history and suggest solutions" |
| Complex analysis | Premium | "Review this codebase for security vulnerabilities" |

Claude Code demonstrates this. Simple file reads use fast models. Complex refactoring engages premium reasoning. The right model balances capability against cost and latency.

**Model selection questions:**
- How complex is the reasoning required?
- How much latency can users tolerate?
- What's the cost per interaction budget?
- Does the task need multimodal input (images, audio)?

## Tools (The Hands)

Tools let agents act on the world. Without tools, an agent can only talk. With tools, it can *do*.

**Claude Code's tools include:**
- File system: read, write, edit files
- Shell execution: run commands, install packages
- Web search: gather information
- Code execution: test solutions

**Tool anatomy:**
Every tool has defined inputs, outputs, and permissions.

```
Tool: ProcessRefund
  Input: customer_id (string), amount (number)
  Output: confirmation_id (string), status (success/failed)
  Permission: max $500 per transaction
```

The permission boundary is crucial. A billing agent with "issue refund" access needs limits. Without them, a compromised agent or bad prompt could drain accounts.

**Tool design questions:**
- What actions does the agent need?
- What inputs does each action require?
- What limits prevent misuse?
- What happens when a tool fails?

## Orchestration (The Nervous System)

Orchestration is where agent magic happens. It coordinates everything the model and tools do.

### Planning

Orchestration breaks goals into executable steps.

When you tell Claude Code "refactor this codebase for better error handling," orchestration creates a plan:
1. Read files to understand current structure
2. Identify patterns lacking error handling
3. Plan specific changes
4. Implement changes file by file
5. Run tests to verify
6. Iterate if tests fail

The model reasons about *what* to do. Orchestration manages *how* to sequence it.

### Memory

Agents need to track context across steps:
- **Short-term**: What files were read? What was tried? What failed?
- **Long-term**: Customer preferences, past interactions, learned patterns

Claude Code's memory tracks your current session—files read, changes made, test results. Enterprise agents might remember customer histories across months of interactions.

### Reasoning Strategies

Different strategies suit different problems:

**ReAct (Reason → Act → Observe → Repeat)**
The agent thinks about what to do, does it, observes the result, then decides what's next. Most agents use this.

**Chain-of-Thought**
The agent thinks step-by-step before taking any action. Good for complex reasoning where premature action causes problems.

**Reflection**
The agent evaluates its own outputs and revises. "Did my code actually fix the bug? Let me check the test output."

Claude Code combines these. It reasons about approach (Chain-of-Thought), executes steps (ReAct), and revises when tests fail (Reflection).

### Orchestration Decisions

When Claude Code works on your request, orchestration decides:
- Which files to read (and in what order)
- When to ask clarifying questions vs. make assumptions
- How to recover when a tool call fails
- When the task is complete

These decisions are what separates a capable agent from a simple chatbot.

## Deployment (The Body)

Deployment makes the agent accessible to users and systems.

**Key deployment questions:**

| Question | Options |
|----------|---------|
| Where does it run? | Cloud, local, hybrid |
| How do users interact? | CLI, API, chat interface, voice |
| What can it access? | File system, databases, external APIs |
| How does it scale? | Single user, thousands concurrent |

Claude Code runs locally on your machine with full file system access. A customer support agent runs in the cloud, handling thousands of concurrent sessions, with access only to approved systems.

**Security is a deployment concern:**
- What credentials does the agent have?
- What networks can it reach?
- What audit trail exists?

## How Components Interact

When you ask Claude Code to "fix the failing test in auth.py":

1. **Orchestration** creates a plan: read test → read implementation → analyze error → fix → verify

2. **Model** reasons: "The test expects a User object but gets None. The authentication function doesn't handle the case where the user doesn't exist."

3. **Tools** execute: Read auth.py, read test_auth.py, edit auth.py to add null check

4. **Model** observes: "Edit complete. Now run the test."

5. **Tools** execute: Run pytest test_auth.py

6. **Model** observes: "Test passes."

7. **Orchestration** decides: Goal achieved. Report success.

If the test still failed, orchestration would loop back—model reasons about *why* the fix didn't work, tools execute new changes, and the cycle continues.

## Recognizing Components in Any Agent

When you encounter a new agent system, ask these questions:

**Model**: What LLM powers it? One model or multiple specialists? What's the cost/capability trade-off?

**Tools**: What can it do? What are the limits? What happens when tools fail?

**Orchestration**: How does it plan? Does it have memory? What reasoning strategy does it use? How does it know when it's done?

**Deployment**: Where does it run? Who can access it? What security constraints exist?

These questions work for Claude Code, ChatGPT, customer support bots, or any agent you'll build.

## Try With AI

Use Claude, ChatGPT, or Gemini to explore architecture components.

> "When I ask Claude Code to 'read a CSV file and save the data to a database,' which component handles each part? Walk through Model, Tools, and Orchestration."

**Expected**: Model reasons about approach and SQL structure. Tools handle file reading and database writing. Orchestration sequences the steps and handles errors.

> "An agent processes 10,000 customer requests daily: classify intent (easy), analyze sentiment (medium), recommend escalation (hard). Why use different models for each task?"

**Expected**: Cost and speed optimization. Fast models for classification (high volume, simple task). Premium models for escalation decisions (lower volume, high stakes).

> "A code review agent reads files, checks security issues, analyzes performance, verifies tests pass, then gives feedback. How does orchestration manage this? Where does memory matter?"

**Expected**: Orchestration plans the sequence and tracks findings across checks. Memory accumulates issues found in each phase for the final report.

**Key insight**: When an agent fails, identify which component failed. Wrong reasoning? Model problem. Can't execute? Tool problem. Lost context mid-task? Orchestration problem. Can't connect? Deployment problem.
