---
title: "Agent Ops"
sidebar_position: 5
description: "Operate agents in production: evaluation, debugging, and continuous improvement."
proficiency_level: B1
cognitive_load:
  new_concepts: 4
  estimated_difficulty: B1
estimated_time: 20 minutes
learning_objectives:
  - "Design evaluation using LM-as-Judge and golden datasets"
  - "Debug agents using traces"
  - "Build feedback loops for continuous improvement"
skills:
  agent_ops:
    proficiency: B1
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-33-intro-ai-agents/spec.md
created: 2025-11-27
last_modified: 2025-11-27
git_author: Claude Code
workflow: /sp.implement
version: 2.0.0
---

# Agent Ops

Building an agent is the easy part. Operating it reliably is where teams struggle. Agent Ops is the discipline of evaluation, debugging, and continuous improvement.

## The Core Challenge

Traditional software testing: Input → Expected output. Pass or fail.

Agent testing: Input → Many valid outputs. How do you evaluate "good enough"?

When a customer asks "where's my order?" a good response might be:
- "Your order #12345 shipped yesterday via UPS, tracking ZYX987"
- "Order 12345 is on its way! Here's your tracking: ZYX987 (UPS)"
- "Great news! Your order shipped. Track it here: [link]"

All valid. None identical. Traditional testing breaks down.

## Four Pillars of Agent Ops

### 1. LM-as-Judge

Use a language model to evaluate agent outputs against quality criteria.

**How it works:**
```
Agent output + Evaluation rubric → Judge LLM → Score + Explanation
```

**Example rubric for customer support:**
```
Criterion: Response helpfulness
10 points: Directly solves user's problem with clear, actionable information
7 points:  Addresses problem but missing some detail
4 points:  Partially relevant, user needs follow-up questions
0 points:  Misunderstands problem or provides wrong information
```

The judge LLM applies this rubric consistently across thousands of responses. It can evaluate nuance that simple string matching can't.

**Why this works:**
- LLMs understand intent, not just keywords
- Consistent evaluation at scale
- Can explain *why* a response scored low
- Catches subtle failures (wrong tone, missing context, confusing explanations)

**Designing good rubrics:**
- Make criteria specific and measurable
- Provide examples for each score level
- Test the rubric on edge cases
- Iterate until judge scores match human judgment

### 2. Golden Datasets

Curated test cases with known-good responses.

**Structure:**
```
Scenario: "Where is my order #12345?"
Context: Order shipped yesterday, tracking ZYX987, UPS, arriving tomorrow

Expected elements:
  ✓ Confirms order number
  ✓ Provides shipping status
  ✓ Includes tracking number
  ✓ Mentions carrier
  ✓ Estimates delivery

Golden response: "Your order #12345 shipped yesterday via UPS.
Tracking: ZYX987. Expected delivery: tomorrow."
```

Run golden datasets after every change. If scores drop, you broke something.

**Building golden datasets:**
1. Start with real user interactions
2. Select diverse scenarios (happy path, edge cases, errors)
3. Document expected elements, not exact wording
4. Include challenging cases that previously failed
5. Update as you learn new failure modes

**How many test cases?**
- Start with 20-50 covering main scenarios
- Add every bug you fix as a new test case
- Aim for 100+ for production systems
- Quality matters more than quantity

### 3. Traces

Record every step an agent takes. When things fail, traces show exactly where.

**Trace structure:**
```
Request: "Fix the failing test in auth.py"
Timestamp: 2024-01-15 10:23:45

Step 1: [10:23:46] Read test file
  Tool: read_file("test_auth.py")
  Result: 245 lines read

Step 2: [10:23:47] Read implementation
  Tool: read_file("auth.py")
  Result: 180 lines read

Step 3: [10:23:48] Analyze error
  Reasoning: "Test expects AuthenticationError but function returns None"

Step 4: [10:23:49] Edit code
  Tool: edit_file("auth.py", changes=[...])
  Result: Success

Step 5: [10:23:52] Run test
  Tool: execute("pytest test_auth.py")
  Result: PASSED

Outcome: Success
Duration: 7 seconds
```

**What traces enable:**
- **Root cause analysis**: Exact step where failure occurred
- **Performance debugging**: Which steps are slow?
- **Pattern recognition**: Do similar failures share common trace patterns?
- **Audit trail**: What did the agent access? What actions did it take?

**Trace debugging workflow:**
1. User reports problem
2. Find the trace for that interaction
3. Walk through steps to identify where things went wrong
4. Was it wrong reasoning? Tool failure? Missing context?
5. Fix the root cause
6. Add to golden dataset to prevent regression

### 4. Human Feedback Loops

Capture user reactions and feed them back into improvement.

**Feedback capture:**
```
"Was this helpful?"
[ ] Yes, solved my problem
[ ] Partially helpful
[ ] Not helpful - wrong information
[ ] Not helpful - missing information
```

Simple feedback captures signal. Detailed feedback captures insight.

**The improvement loop:**
1. User reports unhelpful response
2. Find the trace for that interaction
3. Identify root cause (reasoning error? missing tool? bad context?)
4. Fix the underlying issue
5. Add scenario to golden dataset
6. Verify fix with LM-as-Judge
7. Monitor for recurrence

**Feedback analysis patterns:**
- Cluster similar complaints to find systematic issues
- Track feedback by scenario type (which areas struggle most?)
- Compare feedback before/after changes to measure improvement

## The Mindset Shift

**Before Agent Ops**: "Is my agent correct?"

**With Agent Ops**: "Is my agent improving toward my KPIs?"

Define measurable success:
- Response time < 3 seconds (P95)
- User satisfaction > 85% (feedback surveys)
- Resolution rate > 70% (issues closed without escalation)
- Escalation rate < 10% (complex issues passed to humans)

Then track, debug, and improve systematically.

## Applying to Your Work

Even without production agents, you can practice Agent Ops thinking:

**With Claude Code:**
- Notice when it succeeds vs fails
- Identify patterns in failures (missing context? wrong reasoning? tool errors?)
- Give specific feedback that helps it improve
- Track which prompts work well (your personal golden dataset)

This is the same mental model. You're doing informal Agent Ops every time you work with Claude Code.

**Example:**
Claude Code fails to fix a bug. Instead of just trying again, analyze:
- Did it understand the goal? (Check its reasoning)
- Did it have enough context? (Did it read the right files?)
- Did it execute correctly? (Check tool outputs)
- Did it verify success? (Did it run tests?)

This analysis improves your next prompt—and your mental model of how agents work.

## Try With AI

Use Claude, ChatGPT, or Gemini to design Agent Ops.

> "I want to evaluate a document search agent. It takes a query and returns top 3 relevant documents. What KPIs should I measure? Give me specific, measurable metrics."

**Expected**: Relevance of results (% queries with relevant top result), response time (P50, P95), query understanding accuracy (evaluated by judge), user satisfaction (feedback scores).

> "Create an LM-as-Judge rubric for 'document relevance' with 4 scoring levels (10/7/4/0). What distinguishes each level?"

**Expected**: Clear criteria. 10 = "All 3 documents directly relevant to query." 7 = "2 of 3 relevant." 4 = "1 of 3 relevant." 0 = "No relevant documents returned."

> "A customer support agent gave a generic shipping policy instead of the user's specific order status. The trace shows it called lookup_order() but the tool returned an error. How would you diagnose and fix this?"

**Expected**: Tool failure caused fallback to generic response. Fix: Better error handling, retry logic, or escalation when tools fail. Add test case for tool failures.

> "My agent's satisfaction scores dropped 15% after last week's update. How do I use traces and feedback to diagnose what changed?"

**Expected**: Compare traces before/after update. Look for new failure patterns. Cluster negative feedback to find common issues. Check if specific scenarios degraded.

**Key insight**: The teams building reliable agents aren't using better models—they have better operational discipline. Measure, trace, debug, improve, repeat.
