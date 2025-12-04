---
title: "The Agentic Problem-Solving Process"
sidebar_position: 3
description: "Trace through the 5-Step Operational Loop that every agent follows."
proficiency_level: B1
cognitive_load:
  new_concepts: 2
  estimated_difficulty: B1
estimated_time: 15 minutes
learning_objectives:
  - "Trace through the 5-Step Loop on any agent task"
  - "Understand why context engineering determines agent quality"
skills:
  agent_process:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# The Agentic Problem-Solving Process

Every agent follows the same operational loop. Understanding it helps you predict agent behavior, design better prompts, and debug failures.

## The 5-Step Loop

1. **Get Mission**: Understand what to accomplish
2. **Scan Scene**: Gather relevant context, identify available tools
3. **Think Through**: Reason about approach, plan steps
4. **Take Action**: Execute using tools
5. **Observe and Iterate**: Check results, repeat if needed

This loop runs continuously until the agent achieves its goal or determines it cannot proceed.

## Example: Customer Support Agent

**User**: "Where is my order #12345?"

### Step 1 — Get Mission
The agent parses the request: Help user find order status. Success means providing accurate shipping information.

### Step 2 — Scan Scene
The agent identifies what it needs:
- Tool available: `lookup_order(order_id)`
- Context needed: Order details, shipping status, tracking number
- Constraints: Can only access order system (not inventory, billing, etc.)

### Step 3 — Think Through
The agent plans:
1. Call order lookup with ID "12345"
2. If shipped, extract tracking number
3. If not shipped, explain expected date
4. Provide clear response to user

### Step 4 — Take Action
```
lookup_order("12345") → {
  status: "shipped",
  tracking: "ZYX987",
  carrier: "UPS",
  expected_delivery: "Nov 29"
}
```

### Step 5 — Observe and Iterate
- Order found? Yes
- Status clear? Yes (shipped with tracking)
- Mission complete? Yes—can answer user

**Response**: "Your order #12345 shipped via UPS. Tracking number: ZYX987. Expected delivery: November 29."

If the order wasn't found, the agent would iterate—perhaps ask for email verification or suggest the user check their order confirmation email.

## Example: Claude Code Debugging

**User**: "Fix the failing test in auth.py"

### Step 1 — Get Mission
Understand which test fails and make it pass.

### Step 2 — Scan Scene
- Read test file to see the failing test
- Read implementation to understand current code
- Run test to capture exact error message
- Check related files if needed

### Step 3 — Think Through
Analyze the error. "Test expects `User` object but gets `None`. The `authenticate()` function returns `None` when user doesn't exist instead of raising an exception. The test expects the function to raise `AuthenticationError`."

Plan the fix:
1. Modify `authenticate()` to raise `AuthenticationError` instead of returning `None`
2. Run test to verify

### Step 4 — Take Action
Edit `auth.py` to add the exception. Run `pytest test_auth.py`.

### Step 5 — Observe and Iterate
- Test passes? If yes, mission complete.
- Test still fails? Back to Step 2 with new information. Read the new error, reason about what else might be wrong, try another fix.

The loop continues until the test passes or the agent determines it needs human input.

## Context Engineering

**The critical insight**: Agent quality depends on context quality.

Context engineering means actively selecting relevant information for each step. Not everything the agent *could* access—just what it *needs*.

### Poor Context
Give the agent access to everything: order system, inventory, accounting, scheduling, HR, maintenance databases. The agent gets overwhelmed filtering through irrelevant data. Response time suffers. Accuracy drops.

### Good Context
Give the agent access to the order system only. It finds what it needs immediately. Fast, focused, accurate.

**More tools ≠ better agent. Focused context = better agent.**

### Why This Matters for You

When you work with Claude Code:
- Specific error messages > "it doesn't work"
- Pointing to the relevant file > "somewhere in the codebase"
- Clear success criteria > vague goals

When Claude Code asks clarifying questions, it's engineering better context. Help it by providing focused, relevant information.

### Context Engineering Decisions

Good context engineering asks:
- What information does the agent actually need for this step?
- What would distract or confuse it?
- How can I narrow the search space?

A customer support agent doesn't need access to the HR database. A code review agent doesn't need access to the billing system. Constraints improve performance.

## Debugging with the Loop

When an agent fails, identify which step broke:

| Failure Mode | Step | Diagnosis | Fix |
|--------------|------|-----------|-----|
| Wrong goal pursued | Get Mission | Agent misunderstood request | Clarify the request |
| Missing information | Scan Scene | Needed context unavailable | Provide context or add tools |
| Bad reasoning | Think Through | Flawed logic or planning | Better prompting or different model |
| Tool error | Take Action | Tool failed or returned bad data | Fix tool or check permissions |
| Didn't recognize done | Observe | Agent kept going or stopped early | Clarify success criteria |

### Debugging Example

You ask Claude Code to "add logging to the API." It adds logging to database queries instead.

**Which step failed?** Step 1 (Get Mission). "API" was ambiguous—did you mean the HTTP API? The internal API? The database layer?

**Fix**: Clarify the request: "Add request/response logging to the HTTP API endpoints in `api/routes.py`."

### Another Debugging Example

Claude Code reads the wrong file and proposes changes that don't make sense.

**Which step failed?** Step 2 (Scan Scene). It gathered wrong context.

**Fix**: Point it to the right file: "The authentication logic is in `src/auth/handlers.py`, not `src/auth/models.py`."

## The Loop is Universal

This 5-step loop appears everywhere:

**Self-driving cars**: Understand destination → sense environment → plan route → execute maneuver → check position → repeat

**Game-playing AI**: Understand winning → perceive game state → plan moves → execute action → observe result → repeat

**Your own problem-solving**: Understand goal → gather information → plan approach → take action → check results → adjust

Once you internalize the loop, you can analyze any agent system. Master it once, apply it everywhere.

## Try With AI

Use Claude, ChatGPT, or Gemini to trace through the loop.

> "A coffee shop agent receives: 'medium iced vanilla latte, extra shot, oat milk.' Walk through the 5-Step Loop. What context does it need at each step?"

**Expected**: Mission = make drink correctly. Context = recipe database, inventory status, cup sizes. Plan = preparation sequence. Action = make drink. Observe = verify each component added.

> "Compare two customer service agents. Agent A accesses: orders, inventory, accounting, HR, maintenance. Agent B accesses: orders only. Which handles 'where's my order?' better? Why?"

**Expected**: Agent B—focused context means faster, more accurate responses. Agent A wastes cycles filtering irrelevant systems.

> "Claude Code tried to fix a bug but made it worse. The test that was failing now passes, but three other tests fail. Which step of the loop broke? How would you diagnose?"

**Expected**: Step 5 (Observe) or Step 2 (Scan Scene). Either it didn't check all tests, or it didn't gather enough context about related code. Fix by asking it to run the full test suite before declaring success.

**Key insight**: The loop is universal. When agents fail, trace through the steps to find where things went wrong.
