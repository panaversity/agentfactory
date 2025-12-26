---
sidebar_position: 4
title: "Iterative Quality"
description: "Build LoopAgent for iterative refinement until quality criteria met"
keywords: [google adk, loop agent, iterative refinement, exit_loop, max_iterations]
chapter: 35
lesson: 4
duration_minutes: 45

# HIDDEN SKILLS METADATA
skills:
  - name: "LoopAgent Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can build iterative refinement loops with exit conditions"

learning_objectives:
  - objective: "Build LoopAgent with exit_loop tool and max_iterations safety"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates quality improvement loop that converges"

  - objective: "Define quality criteria that determine when iteration should stop"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student specifies measurable exit conditions"

cognitive_load:
  new_concepts: 1
  assessment: "1 new concept (LoopAgent) with depth, appropriate for B1"

differentiation:
  extension_for_advanced: "Implement adaptive max_iterations based on initial quality score"
  remedial_for_struggling: "Focus on basic loop with fixed iterations before adding exit_loop logic"
---

# Iterative Quality

Generate once and hope? That's demo thinking. Production agents refine their work until quality criteria are met. LoopAgent runs a sub-agent repeatedly, improving output each iteration, until the agent calls `exit_loop` or hits the safety limit.

**The Problem**: Single-pass generation often produces "almost good enough" outputs that need human polish. A quality agent runs multiple iterations—evaluating, refining, and validating—until the work meets explicit criteria.

Think about how you approach real tasks. You write a first draft, read it, notice issues, revise. You test code, find edge cases, rewrite. You iterate toward quality, not toward "done on first try."

LoopAgent brings this human refinement process into agent workflows.

## The Iterative Refinement Pattern

Single-pass generation fails for quality-critical tasks:

| Approach | Result | Problem |
|----------|--------|---------|
| **One Pass** | "Good enough" output | Misses edge cases, lacks polish |
| **Human Review** | Catches issues | Slow, requires context switching |
| **Agent Loop** | Autonomous refinement | Converges to quality criteria |

The pattern is straightforward:

```
1. Generate initial output
2. Evaluate: Does it meet quality criteria?
3. If not: Improve it
4. Repeat until criteria met OR hit max iterations
5. Return best version found
```

**Why this matters for production agents**: You cannot ship outputs that "seemed good on the first try." Enterprise workflows require validation, refinement, and convergence. LoopAgent encodes this discipline into agent architecture.

## LoopAgent with exit_loop

LoopAgent takes a sub-agent and runs it repeatedly until the agent signals completion via `exit_loop`:

```python
from google.adk.agents import LlmAgent, LoopAgent
from google.adk.tools import exit_loop

quality_improver = LlmAgent(
    name="quality_improver",
    model="gemini-2.5-flash",
    instruction="""You are a quality improvement agent.
Review the current output. If it meets quality criteria, call exit_loop.
Otherwise, improve it and continue.

Quality criteria:
- Clear and concise (no jargon)
- No grammatical errors
- Addresses the original request completely
- Evidence-backed claims have citations""",
    tools=[exit_loop]
)

refinement_loop = LoopAgent(
    name="refinement_loop",
    description="Iterate until quality criteria met.",
    sub_agents=[quality_improver],
    max_iterations=5
)
```

**Output:**
```
Initial output: "This is good"
Iteration 1: Adds evidence, improves clarity
Iteration 2: Fixes grammar, expands explanation
Iteration 3: Agent recognizes criteria met → calls exit_loop
Final output: Quality-validated result
```

### How exit_loop Works

The sub-agent has `exit_loop` as a tool. When the agent calls this tool (with no arguments), the loop terminates immediately:

```python
# Inside the quality_improver agent's reasoning:
# "The output now meets all criteria. I should call exit_loop."

# Agent generates:
# Tool use: exit_loop()

# LoopAgent stops immediately, returns current output
```

**Critical insight**: exit_loop is not a function the agent calls in code. It's a tool the agent calls when reasoning is complete. The agent decides: "Is this good enough?" If yes, tool use triggers loop exit.

## The max_iterations Safety Limit

**Without max_iterations, a loop could run forever.**

Consider this failure mode:

```python
# DANGEROUS: No safety limit
loop_without_safety = LoopAgent(
    name="infinite_loop",
    sub_agents=[quality_improver]
    # Missing: max_iterations
)

# Possible outcome:
# Iteration 1: Output improved
# Iteration 2: Output improved
# Iteration 3: Agent keeps iterating, criteria never satisfied
# Iteration 4: Still refining...
# Iteration 100: Still refining...
# API calls exhausted, user credits spent
```

max_iterations prevents this:

```python
refinement_loop = LoopAgent(
    name="refinement_loop",
    description="Iterate until quality criteria met.",
    sub_agents=[quality_improver],
    max_iterations=5  # REQUIRED safety limit
)

# Possible outcomes:
# - Agent calls exit_loop before iteration 5: Loop stops early ✓
# - Iteration 5 reached without exit_loop: Loop stops, returns best output ✓
# - No infinite loop: Always bounded ✓
```

**When max_iterations is reached**, LoopAgent returns the output from the final iteration gracefully. It doesn't error or fail—it just stops and uses what it has.

This is the correct design: **Always set max_iterations. Start conservative (3-5), increase only after observing actual convergence patterns.**

### Choosing max_iterations Values

| max_iterations | Use Case | Trade-off |
|---|---|---|
| 2-3 | Simple improvements (formatting, typos) | May stop before full quality |
| 4-5 | Standard quality refinement | Good balance of depth vs cost |
| 6-10 | Complex outputs needing major restructuring | Higher API cost but better results |
| >10 | Only if observing consistent convergence | Risk of endless refinement |

## Testing LoopAgent Convergence

How do you validate that your loop converges reasonably, not that it takes exactly N iterations?

Create test cases that verify:
1. **Convergence happens**: Sub-agent eventually calls exit_loop
2. **Convergence is reasonable**: Within configured max_iterations
3. **Output quality improves**: Later iterations better than earlier

```python
# Test case structure
test_case = {
    "name": "quality_refinement_test",
    "input": "Write a paragraph about AI agents",
    "expected_tool_use": ["exit_loop"],
    "max_iterations_allowed": 5,
    "quality_checks": [
        "Output has no grammatical errors",
        "Output addresses the prompt fully",
        "Output is substantive (100+ words)"
    ]
}
```

**Evaluation approach**:
- Run the loop
- Check: Did agent call exit_loop before hitting max_iterations?
- Check: Does final output pass quality checks?
- Check: Is output better than iteration 1?

**Avoid**: Hardcoding "must exit on exactly iteration 3." Agents are non-deterministic. Your test should validate outcomes, not execution paths.

## Real-World Example: Content Review Loop

Here's a production pattern for content quality:

```python
from google.adk.agents import LlmAgent, LoopAgent
from google.adk.tools import exit_loop

reviewer = LlmAgent(
    name="content_reviewer",
    model="gemini-2.5-flash",
    instruction="""You are a content quality reviewer.
Given content and a quality rubric, review and improve it.

Rubric:
1. Clarity: Simple language, no jargon
2. Completeness: All aspects covered
3. Accuracy: Claims backed by evidence
4. Structure: Logical flow with headings
5. Conciseness: No redundant sections

Steps:
1. Evaluate against rubric
2. If all 5 criteria met: Call exit_loop
3. If any criterion fails:
   - Identify the issue
   - Explain the fix needed
   - Provide improved version
   - Wait for next iteration with improved content

IMPORTANT: Only call exit_loop when ALL criteria are satisfied.""",
    tools=[exit_loop]
)

content_quality_loop = LoopAgent(
    name="content_quality",
    description="Iterate content until it meets quality rubric.",
    sub_agents=[reviewer],
    max_iterations=4
)
```

**Flow in action**:

```
Input: "AI agents are cool. They do stuff. Companies use them."

Iteration 1:
Reviewer evaluates:
- Clarity: Vague ("stuff", "cool") ✗
- Completeness: Missing details ✗
- Accuracy: No evidence ✗
- Structure: No headings ✗
- Conciseness: OK ✓

Output: [Improved version with clarity, examples, headings]

Iteration 2:
Reviewer evaluates:
- Clarity: Better ✓
- Completeness: More detail ✓
- Accuracy: Has citations ✓
- Structure: Clear headings ✓
- Conciseness: Some redundancy ✗

Output: [Removed redundant paragraph]

Iteration 3:
Reviewer evaluates:
- Clarity: ✓
- Completeness: ✓
- Accuracy: ✓
- Structure: ✓
- Conciseness: ✓

→ Calls exit_loop

Final output: Quality-validated content
```

## Debugging Loop Convergence Issues

What if your agent keeps hitting max_iterations without calling exit_loop?

**Common causes and fixes**:

| Symptom | Cause | Fix |
|---------|-------|-----|
| Never calls exit_loop | Criteria too strict | Loosen constraints in instruction |
| Exits too early | Criteria too loose | Add specific quality checks |
| No improvement between iterations | Agent unclear on what to fix | Add rubric or examples |
| Different output each iteration | Instruction vague | Make criteria measurable |

**Diagnostic prompt** (ask your AI helper):

```
My LoopAgent hits max_iterations without calling exit_loop.
Here's my quality criteria: [your criteria]
Here's a sample output from iteration 1: [output]
Here's output from iteration 3: [output]

Does the quality improve across iterations?
Is my criteria measurable enough for the agent to judge?
What's unclear in my instruction?
```

## Composing Multiple Loops

Can you nest loops? Combine a loop with SequentialAgent?

**Nesting loops** (loop inside loop):

```python
# RISKY: Nested loops can multiply iterations
fact_check_loop = LoopAgent(
    name="fact_check",
    sub_agents=[fact_checker],
    max_iterations=4
)

quality_loop = LoopAgent(
    name="quality",
    sub_agents=[fact_check_loop],  # Loop inside loop!
    max_iterations=4
)

# Worst case: 4 × 4 = 16 total iterations
# Each inner iteration calls outer loop
# Exponential complexity
```

**Better: Sequential loops**:

```python
# SAFER: Run loops in sequence
draft_improver = LoopAgent(
    name="draft_improve",
    sub_agents=[drafter],
    max_iterations=3
)

fact_checker = LoopAgent(
    name="fact_check",
    sub_agents=[checker],
    max_iterations=3
)

pipeline = SequentialAgent(
    name="content_pipeline",
    sub_agents=[draft_improver, fact_checker]
)

# Total iterations: 3 + 3 = 6 (linear, not exponential)
```

**Rule of thumb**: Nest loops only if inner loop is truly autonomous (its own exit condition). If outer loop depends on inner loop completing, use SequentialAgent instead.

## Safety Note

Always set `max_iterations`. Infinite loops waste API credits, block system resources, and frustrate users. Start conservative (3-5 iterations). Increase only after observing actual agent behavior and confirming convergence patterns. Monitor loop runtime—if iterations are taking 10+ seconds each, reconsider your approach or increase iterations further to allow more refinement time.

## Try With AI

### Prompt 1: Define Quality Criteria

```
I'm building a LoopAgent that refines [describe your output type:
customer service responses / code review comments / technical documentation].
Help me define measurable quality criteria that the agent can evaluate.

Ask me:
- What makes output "good enough" to stop iterating?
- How do I know when more iterations won't help?
- What specific checks can the agent perform to validate quality?
```

**What you're learning**: Quality specification—translating your intuition about "good enough" into measurable criteria the agent can evaluate and iterate toward.

### Prompt 2: Diagnose Loop Convergence

```
My LoopAgent hits max_iterations without calling exit_loop.
Here's my instruction to the agent:
[paste your agent instruction]

Here's a sample output from iteration 1:
[paste first iteration output]

Here's output from iteration 3:
[paste third iteration output]

Help me diagnose:
- Is the output actually improving across iterations?
- Is my quality criteria measurable/clear enough for the agent?
- What's ambiguous in my instruction that's preventing convergence?
```

**What you're learning**: Loop debugging—understanding why agents don't recognize when quality criteria are met and how to clarify your instruction.

### Prompt 3: Design Nested Iteration Patterns

```
I have two sequential workflows I want to iterate:
1. [describe first refinement task]
2. [describe second refinement task]

Should I:
A) Nest them (loop inside loop)
B) Sequence them (loop → loop → done)
C) Parallelize somehow (run parts concurrently)

Help me think through:
- What happens if the inner loop never converges?
- How many total iterations am I planning? What's the cost?
- Which approach gives me better control and observability?
```

**What you're learning**: Workflow composition—combining iterative patterns safely, understanding tradeoffs between complexity and control.
