---
sidebar_position: 2
title: "Evaluation-Driven Development"
description: "Build TaskManager agent by making eval cases pass with adk eval"
keywords: [google adk, adk eval, evaluation driven development, pytest, agentevahluator, iterative improvement]
chapter: 35
lesson: 2
duration_minutes: 60

# HIDDEN SKILLS METADATA
skills:
  - name: "Running adk eval"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run adk eval and interpret pass/fail results from detailed output"

  - name: "Iterative Agent Improvement"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can diagnose failing eval cases and improve agent instruction to fix them"

  - name: "Pytest Integration with ADK"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can integrate adk eval with pytest for automated testing in CI/CD pipelines"

learning_objectives:
  - objective: "Run adk eval and interpret detailed results showing pass/fail status"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student executes adk eval with --print_detailed_results flag and identifies which cases pass/fail and why"

  - objective: "Diagnose failing eval cases and improve agent instructions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student identifies why an eval case fails and modifies agent instruction to fix it, then re-runs eval"

  - objective: "Integrate adk eval with pytest for automated testing"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates pytest test file using AgentEvaluator.evaluate() that validates agent against eval set"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (adk eval CLI, iterative improvement workflow, pytest integration) + 2 reviewed (eval cases, TaskManager) = 5 total, within B1 limit (7-10 concepts)"

differentiation:
  extension_for_advanced: "Write custom evaluator that checks response latency or token usage in addition to correctness; implement retry logic in pytest"
  remedial_for_struggling: "Focus on running adk eval and reading pass/fail output; skip pytest integration initially; use template pytest file"
---

# Evaluation-Driven Development

In Lesson 1, you wrote eval cases. They're sitting there. Silent. Waiting. They're not decorative—they're a specification. Every test case is a promise: "The agent MUST behave this way."

Now comes the hard part: **making the agent keep those promises**.

This lesson teaches you the core workflow of reliable agent development:

1. **Write eval cases** (you did this)
2. **Build an agent** (spec-based, not demo-based)
3. **Run adk eval** (watch tests fail)
4. **Diagnose failures** (read the output carefully)
5. **Improve agent** (modify instruction or tools)
6. **Run adk eval again** (measure progress)
7. **Repeat until all tests pass** (done is a specific state, not a feeling)

This isn't just testing. **This is the primary development feedback loop.** Tests drive the work, not afterthoughts.

## The TaskManager Agent: From Specification to Implementation

Let's build a TaskManager agent using evaluation-driven development. We'll start with eval cases from Lesson 1, then incrementally build the agent until all tests pass.

### The Specification (From Lesson 1)

Your eval cases define the contract. Let's recap what the agent MUST do:

```json
{
  "eval_set_id": "taskmanager_basics",
  "eval_cases": [
    {
      "eval_id": "add_task_simple",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Add a task called 'Buy groceries'"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Task 'Buy groceries' added successfully"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {"name": "add_task", "args": {"title": "Buy groceries"}}
            ]
          }
        }
      ],
      "session_input": {
        "app_name": "task_manager",
        "user_id": "test_user"
      }
    }
  ]
}
```

**Output:**

```
This eval case says: When a user asks to add a task, the agent MUST call the
add_task tool with the task title. Non-negotiable. The test passes only if
this exact behavior happens.
```

### Minimal Agent Implementation

Let's write the simplest agent that could possibly pass these tests. Not perfect. Not feature-complete. Just specification-compliant.

```python
from google.adk.agents import Agent
from typing import List, Optional

# In-memory task storage (for development only)
tasks: List[dict] = []

def add_task(title: str, description: Optional[str] = None) -> dict:
    """Add a new task to the list.

    Args:
        title: Name of the task to add.
        description: Optional task description.

    Returns:
        Confirmation with task ID and details.
    """
    task = {
        "id": len(tasks) + 1,
        "title": title,
        "description": description,
        "completed": False
    }
    tasks.append(task)
    return {"status": "created", "id": task["id"], "title": task["title"]}

def list_tasks() -> dict:
    """List all current tasks.

    Returns:
        Dictionary containing all tasks.
    """
    if not tasks:
        return {"tasks": [], "count": 0}
    return {"tasks": tasks, "count": len(tasks)}

def complete_task(task_id: int) -> dict:
    """Mark a task as complete.

    Args:
        task_id: ID of the task to complete.

    Returns:
        Confirmation of completion.
    """
    for task in tasks:
        if task["id"] == task_id:
            task["completed"] = True
            return {"status": "completed", "id": task_id, "title": task["title"]}
    return {"status": "error", "message": f"Task {task_id} not found"}

def delete_task(task_id: int) -> dict:
    """Delete a task from the list.

    Args:
        task_id: ID of the task to delete.

    Returns:
        Confirmation of deletion.
    """
    global tasks
    task_to_delete = None
    for task in tasks:
        if task["id"] == task_id:
            task_to_delete = task
            break

    if task_to_delete:
        tasks.remove(task_to_delete)
        return {"status": "deleted", "id": task_id, "title": task_to_delete["title"]}
    return {"status": "error", "message": f"Task {task_id} not found"}

# The agent definition
root_agent = Agent(
    name="task_manager",
    model="gemini-2.5-flash",
    instruction="""You are a TaskManager assistant. Help users manage their tasks.

When users ask to ADD tasks, use the add_task tool with the task title.
When users ask to LIST or VIEW tasks, use the list_tasks tool.
When users ask to COMPLETE, FINISH, or MARK DONE tasks, use the complete_task tool.
When users ask to DELETE or REMOVE tasks, use the delete_task tool.

Always confirm what action you took. Be specific about which task was affected.""",
    tools=[add_task, list_tasks, complete_task, delete_task]
)
```

**Output:**

```
This agent:
- Defines four tools matching the eval case expectations
- Has clear instruction that maps user requests to tool calls
- Stores tasks in memory (sufficient for dev/testing)
- Returns structured responses so the agent can describe what happened
```

## Running adk eval: The First Test

Now the moment of truth. Does your agent pass the eval cases you defined?

### The adk eval Command

```bash
# Run evaluation with detailed output
adk eval ./task_manager.py ./evals/taskmanager_basics.test.json --print_detailed_results
```

**Output:**

```
Running evaluation: taskmanager_basics
Total cases: 3
Passed: 1
Failed: 2

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PASS: add_task_simple
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: "Add a task called 'Buy groceries'"
Expected tool call: add_task(title="Buy groceries")
Actual tool call: add_task(title="Buy groceries")
Expected response pattern: "Task 'Buy groceries' added successfully"
Actual response: "Task 'Buy groceries' added successfully"
✓ PASS

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
FAIL: list_tasks_default
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: "Show my tasks"
Expected tool call: list_tasks(args={})
Actual tool call: NOT CALLED - Agent responded with text only
Expected response pattern: "Here are your tasks"
Actual response: "I would help you list your tasks, but you haven't added any yet."
✗ FAIL - Tool not called as expected

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
FAIL: complete_task_first
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: "Mark the first task as done"
Expected tool call: complete_task(task_id=1)
Actual tool call: NOT CALLED - Agent asked for clarification
Expected response pattern: "Task marked complete"
Actual response: "Which task would you like to mark complete?"
✗ FAIL - Tool not called; agent sought clarification instead
```

### Interpreting the Output

The output tells you exactly what went wrong:

| Case | Status | Diagnosis |
|------|--------|-----------|
| **add_task_simple** | PASS | Agent understood "add a task" and called the right tool |
| **list_tasks_default** | FAIL | Agent didn't recognize "Show my tasks" as a list request |
| **complete_task_first** | FAIL | Agent sought clarification instead of assuming first task = id 1 |

**This is critical information.** You now know exactly what to fix.

## Diagnosing and Fixing Failures

The eval output tells you the gap between specification and implementation. Now you improve the agent instruction to close that gap.

### Failure 1: List Tasks Not Recognized

**Problem**: Agent didn't call `list_tasks` when user said "Show my tasks"

**Root cause**: The instruction doesn't explicitly mention "show" as a list synonym. The model might interpret "show" as explanation, not tool invocation.

**Fix**: Update the agent instruction with explicit phrase matching:

```python
root_agent = Agent(
    name="task_manager",
    model="gemini-2.5-flash",
    instruction="""You are a TaskManager assistant. Help users manage their tasks.

When users say "add", "create", "new task", etc. → Call add_task with the title
When users say "list", "show", "view", "what are my", "get my" → Call list_tasks
When users say "complete", "finish", "mark done", "check off" → Call complete_task with task ID
When users say "delete", "remove" → Call delete_task with task ID

Always call the appropriate tool. Always confirm the action with specific details.
Never ask for clarification—interpret requests decisively.""",
    tools=[add_task, list_tasks, complete_task, delete_task]
)
```

### Failure 2: Complete Task Requires Task ID

**Problem**: Agent asked for clarification ("Which task?") instead of calling `complete_task` with id=1

**Root cause**: User said "first task" but didn't specify a numeric ID. The instruction doesn't tell the agent to treat "first" as id=1.

**Fix**: Add explicit interpretation rules:

```python
root_agent = Agent(
    name="task_manager",
    model="gemini-2.5-flash",
    instruction="""You are a TaskManager assistant. Help users manage their tasks.

When users say "add", "create", "new task" → Call add_task with the title
When users say "list", "show", "view", "what are my" → Call list_tasks immediately
When users say "complete", "finish", "mark done" → Call complete_task with task ID

Task ID interpretation:
- "first task" or "task 1" → ID is 1
- "second task" or "task 2" → ID is 2
- "the task" (if only one exists) → ID is 1
- If user mentions a task title, search and use matching ID

When users say "delete" or "remove" → Call delete_task with task ID

RULES:
- NEVER ask "Which task?" - interpret "first", "last", numeric IDs decisively
- ALWAYS call a tool - don't respond with text only
- ALWAYS confirm what you did with specific task names""",
    tools=[add_task, list_tasks, complete_task, delete_task]
)
```

### Running eval Again

Now re-run the evaluation to see if you fixed the issues:

```bash
adk eval ./task_manager.py ./evals/taskmanager_basics.test.json --print_detailed_results
```

**Output:**

```
Running evaluation: taskmanager_basics
Total cases: 3
Passed: 3
Failed: 0

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ PASS: add_task_simple
✓ PASS: list_tasks_default
✓ PASS: complete_task_first
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

All 3 eval cases passed. Agent specification met.
```

You've just completed one iteration of evaluation-driven development:

1. Ran tests and got failures
2. Diagnosed root causes
3. Improved agent based on diagnosis
4. Re-ran tests and passed

**This is the workflow.** No hand-waving. No assumptions. Tests drive the work.

## Debugging Failed Evals

When `adk eval` shows failures, the error message alone rarely tells you why. Here's a systematic debugging workflow that transforms cryptic failures into actionable insights.

### The Systematic Debugging Workflow

When an eval case fails, follow these steps in order:

**Step 1: Read the failure diff**
Compare what the agent actually said vs what was expected. Look for:
- Did agent refuse to call a tool? (Why?)
- Did agent call wrong tool? (Misunderstood the request?)
- Did agent call right tool with wrong arguments? (Missing context?)
- Did agent respond without calling any tool? (Ignored instruction?)

**Step 2: Check the query clarity**
Is the test case ambiguous? Does the user request contain implicit assumptions the agent doesn't share?

Example: "Delete the second task" assumes:
- There is a second task
- The agent knows which task is "second" (by ID? by creation order?)
- Deletion is permitted (no permission model in spec?)

**Step 3: Trace the tool calls**
If the agent called a tool, did it call the right sequence? Did it:
- Call tools in logical order?
- Wait for responses before calling next tool?
- Use tool outputs correctly?

**Step 4: Inspect the session state**
Was session state correct when the tool ran? Did previous operations set up state properly?

Example: If test adds a task then deletes it, did the add actually create a task? Check the session state between operations.

**Step 5: Review the agent instruction**
Does the instruction need refinement? Common issues:
- Ambiguous language ("handle gracefully" vs specific behavior)
- Missing examples (agent doesn't know what to do)
- Conflicting instructions (do X, but also do Y)

### Common Failure Patterns and Root Causes

These patterns appear repeatedly. Recognize them and diagnose faster:

| Symptom | Likely Root Cause | Diagnostic Question | Fix |
|---------|------------------|---------------------|-----|
| **Tool not called** | Agent didn't recognize request as requiring action | Does instruction mention this phrasing? Is example included? | Add synonyms, add example to instruction |
| **Wrong tool called** | Ambiguous request; agent chose differently | Does request have multiple valid interpretations? | Clarify test case query or refine instruction |
| **Right tool, wrong args** | Tool docstring unclear or missing examples | Does tool docstring show example usage? | Add detailed docstring with examples |
| **Partial match only** | Response format mismatch; agent said something close | Does eval case check response text or only tool calls? | Check expected_tool_use vs expected response |
| **Timeout/hanging** | Infinite loop in tool, LoopAgent, or callback | Does tool have max iterations? Any circular logic? | Add max_iterations, trace loops in callbacks |
| **Empty response** | No matching tool found for intent | Does agent have tools needed? Are tools visible in instruction? | Add missing tool or clarify tool availability |
| **Refusal ("I can't...")** | Guardrail/instruction conflict; agent refusing from caution | Does instruction say "never" something? Is guardrail too strict? | Review safety constraints vs requirements |

### Real Debugging Example: The "Mark done" Failure

Let's walk through a real failure and trace it:

**Eval case**:
```json
{
  "eval_id": "complete_task_first",
  "conversation": [{
    "user_content": {"text": "Mark the first task as done"},
    "expected_tool_use": [{"name": "complete_task", "args": {"task_id": 1}}]
  }]
}
```

**Agent response**:
```
"Which task would you like to mark complete? Please give me the task ID."
```

**Initial diagnosis**: Agent asked for clarification instead of calling tool. But WHY?

**Step 1 - Read diff**: Agent didn't call complete_task. This is clear.

**Step 2 - Check query**: "Mark the first task as done"
- Assumes "first task" = task ID 1
- Assumes agent can infer this without session context
- TEST PROBLEM or INSTRUCTION PROBLEM?

**Step 3 - Trace calls**: No tool calls made. Agent went straight to asking for clarification.

**Step 4 - Check state**: Before this request, was a task actually created? Does session state show a task with id=1?

**Step 5 - Review instruction**:
```
"When users say 'complete', call complete_task with task ID"
```

The instruction doesn't say how to interpret "first task" as ID. The agent doesn't have a rule like "first = 1, second = 2."

**Diagnosis**: Instruction is incomplete. Missing interpretation rule.

**Fix**: Add to instruction:
```
Task ID interpretation:
- "first task" or "task 1" → ID is 1
- "second task" or "task 2" → ID is 2
- "the task" (if only one exists) → ID is 1
```

Re-run: Agent now calls `complete_task(task_id=1)` and test passes.

### Debugging Checklist

When eval fails:

- [ ] **Failure type identified** (tool not called, wrong tool, wrong args, format mismatch)
- [ ] **Query clarity verified** (is test case ambiguous?)
- [ ] **Tool call sequence checked** (correct order and arguments?)
- [ ] **Session state confirmed** (state correct before tool runs?)
- [ ] **Instruction reviewed** (does instruction cover this case?)
- [ ] **Fix implemented** (which element changed?)
- [ ] **Test re-run** (does it pass now?)

## Exercise: Debug This Broken Agent

You're going to experience a real debugging workflow. Here's a TaskManager agent that's failing 3 out of 5 eval cases. Your job: diagnose and fix each failure.

### The Broken Agent Code

```python
from google.adk.agents import Agent
from typing import List, Optional

tasks: List[dict] = []

def add_task(title: str) -> dict:
    """Add a new task."""
    task = {
        "id": len(tasks) + 1,
        "title": title,
        "completed": False
    }
    tasks.append(task)
    return {"status": "created", "id": task["id"]}

def list_tasks() -> dict:
    """List all tasks."""
    return {"tasks": tasks, "count": len(tasks)}

def complete_task(task_id: int) -> dict:
    """Mark task as complete."""
    for task in tasks:
        if task["id"] == task_id:
            task["completed"] = True
            return {"status": "completed", "id": task_id}
    return {"status": "error", "message": f"Task {task_id} not found"}

def delete_task(task_id: int) -> dict:
    """Delete a task."""
    global tasks
    task_to_delete = None
    for task in tasks:
        if task["id"] == task_id:
            task_to_delete = task
            break
    if task_to_delete:
        tasks.remove(task_to_delete)
        return {"status": "deleted", "id": task_id}
    return {"status": "error", "message": f"Task {task_id} not found"}

# BUGGY AGENT - Notice the instruction problems
root_agent = Agent(
    name="task_manager",
    model="gemini-2.5-flash",
    instruction="""You are a task manager. Help users with their tasks.

Use the right tool when asked. Always be helpful.""",
    tools=[add_task, list_tasks, complete_task, delete_task]
)
```

**Output:**
```
This agent has vague instructions. Notice:
- No mapping between user requests and tools
- No examples of what "helpful" means
- No rules for interpreting natural language (like "first task")
```

### The Failing Eval Cases

```json
{
  "eval_cases": [
    {
      "eval_id": "add_task",
      "query": "Add a task called 'Buy milk'",
      "expected_tool_use": [{"name": "add_task", "args": {"title": "Buy milk"}}]
    },
    {
      "eval_id": "list_empty",
      "query": "Show me my tasks",
      "expected_tool_use": [{"name": "list_tasks", "args": {}}]
    },
    {
      "eval_id": "complete_first",
      "query": "Mark the first task as done",
      "expected_tool_use": [{"name": "complete_task", "args": {"task_id": 1}}]
    },
    {
      "eval_id": "delete_by_id",
      "query": "Delete task 2",
      "expected_tool_use": [{"name": "delete_task", "args": {"task_id": 2}}]
    },
    {
      "eval_id": "error_handling",
      "query": "Complete task 99",
      "expected_tool_use": [{"name": "complete_task", "args": {"task_id": 99}}]
    }
  ]
}
```

### Your Task

1. **Run adk eval** and identify which cases fail
2. **Use the debugging workflow** to diagnose each failure
3. **Fix the agent instruction** to address root causes
4. **Re-run adk eval** until all cases pass

Here's what you'll discover:

**Failure 1: List not recognized**
- Symptom: "Show me my tasks" doesn't call list_tasks
- Root cause: Instruction doesn't mention "show" as list synonym
- Fix: Add "show, view, list" to instruction

**Failure 2: Complete needs context**
- Symptom: "Mark the first task as done" gets "Which task?"
- Root cause: Instruction doesn't interpret "first" as task_id=1
- Fix: Add task ID interpretation rules

**Failure 3: Error case ignored**
- Symptom: Complete task 99 called, but agent doesn't handle error gracefully
- Root cause: Instruction doesn't say what to do with errors
- Fix: Add error handling rule

### The Fixed Agent Instruction

```python
root_agent = Agent(
    name="task_manager",
    model="gemini-2.5-flash",
    instruction="""You are a TaskManager assistant. Help users manage their tasks.

REQUEST MAPPING:
- "add", "create", "new task" → Call add_task with title
- "list", "show", "view", "what are my" → Call list_tasks
- "complete", "finish", "mark done", "check off" → Call complete_task with task_id
- "delete", "remove" → Call delete_task with task_id

TASK ID INTERPRETATION:
- "first task" or "task 1" → task_id=1
- "second task" or "task 2" → task_id=2
- If user mentions task title, find matching task ID
- Never ask for clarification—interpret decisive""

RESPONSE RULES:
- ALWAYS call a tool—never respond with text only
- For success: "Task '[title]' successfully [action]"
- For error: "Task [id] not found. Current tasks: [list]"
- Never say "I don't know which task"—ask list_tasks to see all tasks first""",
    tools=[add_task, list_tasks, complete_task, delete_task]
)
```

**Output:**
```
Fixed instruction:
- Maps user language to specific tools
- Interprets natural language (first, second, task title)
- Defines error handling behavior
- Enforces tool-calling rule (never text-only)
```

### Run the Fixed Version

```bash
adk eval ./task_manager.py ./evals/taskmanager_basics.test.json --print_detailed_results
```

**Output:**
```
Running evaluation: taskmanager_basics
Total cases: 5
Passed: 5
Failed: 0

All eval cases passed.
```

You've now experienced the complete debugging cycle: identify failure → diagnose root cause → fix instruction → re-run → pass.

## Eval-Driven Iteration in Practice

The debugging workflow above is powerful for single failures. But real development means iterating through multiple features, each with failing evals. Here's what the full iteration loop looks like in practice.

### Feature Addition: Task Priority

You want to add task priorities. Here's the workflow:

**Phase 1: Write Eval Cases** (Monday morning)

```json
{
  "eval_cases": [
    {
      "eval_id": "add_with_priority",
      "query": "Add a high priority task: Review proposal",
      "expected_tool_use": [
        {
          "name": "add_task",
          "args": {
            "title": "Review proposal",
            "priority": "high"
          }
        }
      ]
    },
    {
      "eval_id": "list_by_priority",
      "query": "Show me my high priority tasks",
      "expected_tool_use": [
        {
          "name": "list_tasks",
          "args": {"priority": "high"}
        }
      ]
    },
    {
      "eval_id": "priority_default",
      "query": "Add a task: Read email",
      "expected_tool_use": [
        {
          "name": "add_task",
          "args": {
            "title": "Read email",
            "priority": "medium"
          }
        }
      ]
    }
  ]
}
```

**Critical principle**: Write evals BEFORE changing agent code. This is your spec.

**Phase 2: Run Eval (Monday 10am)**

```bash
adk eval ./task_manager.py ./evals/taskmanager_priority.test.json --print_detailed_results
```

**Output:**
```
Total cases: 3
Passed: 0
Failed: 3

FAIL: add_with_priority
Agent did not call add_task with priority parameter
```

All fail. Expected. Agent doesn't have priority feature yet.

**Phase 3: Analyze Failures** (Monday 10:15am)

Question: What needs to change?
- Tool signature? (add_task needs priority parameter)
- Agent instruction? (agent needs to recognize "high priority")
- Tool implementation? (add_task needs to store priority)

Answer: All three.

**Phase 4: Implement Changes** (Monday 10:30am - 11:30am)

Change 1: Update add_task signature
```python
def add_task(title: str, priority: str = "medium") -> dict:
    """Add a new task.

    Args:
        title: Task name
        priority: one of "low", "medium", "high" (default: "medium")
    """
    task = {
        "id": len(tasks) + 1,
        "title": title,
        "priority": priority,
        "completed": False
    }
    tasks.append(task)
    return {"status": "created", "id": task["id"], "priority": task["priority"]}
```

Change 2: Update list_tasks signature
```python
def list_tasks(priority: Optional[str] = None) -> dict:
    """List tasks, optionally filtered by priority.

    Args:
        priority: Filter by priority level (optional)
    """
    if priority:
        filtered = [t for t in tasks if t["priority"] == priority]
        return {"tasks": filtered, "count": len(filtered)}
    return {"tasks": tasks, "count": len(tasks)}
```

Change 3: Update agent instruction
```python
instruction="""...
REQUEST MAPPING:
- "add", "create" with priority word → Call add_task with title and priority
  - "high priority" or "urgent" → priority="high"
  - "low priority" or "whenever" → priority="low"
  - No priority mentioned → priority="medium" (default)
- "show [priority] tasks" → Call list_tasks with that priority
...
"""
```

**Key principle**: Never change multiple things at once. You won't know what fixed it.

**Phase 5: Re-run Eval** (Monday 11:30am)

```bash
adk eval ./task_manager.py ./evals/taskmanager_priority.test.json --print_detailed_results
```

**Output:**
```
Total cases: 3
Passed: 2
Failed: 1

FAIL: list_by_priority
Expected: list_tasks(priority="high")
Actual: Agent responded "I'll show high priority tasks" but didn't call list_tasks
```

Progress! 2 of 3 passing. One more failure.

**Phase 6: Debug the Remaining Failure** (Monday 11:45am)

The agent didn't call list_tasks. Why?

Check instruction: Does it mention "show [priority] tasks"?

Discovered issue: Instruction says "show", but query says "Show me my high priority tasks". Agent might not recognize this pattern.

Add more examples to instruction:
```
- "show my high priority tasks"
- "list high priority tasks"
- "what's important?"
```

**Phase 7: Re-run Again** (Monday 12:00pm)

```bash
adk eval ./task_manager.py ./evals/taskmanager_priority.test.json --print_detailed_results
```

**Output:**
```
Total cases: 3
Passed: 3
Failed: 0

✓ All eval cases passed
```

Done. Feature complete. 1.5 hours from spec to passing eval.

### The Iteration Loop Visualized

```
Write Evals (0 fail)
    ↓
Run adk eval (3 fail)
    ↓
Analyze failures (identify 3 changes needed)
    ↓
Implement ONE change (tools + instruction)
    ↓
Re-run adk eval (2 pass, 1 fail)
    ↓
Debug remaining failure (missing example)
    ↓
Implement fix
    ↓
Re-run adk eval (3 pass, 0 fail)
    ↓
Feature complete
```

### Key Insights from This Workflow

**1. Evals are the spec**
You never wondered "is this done?" because passing evals IS done. No ambiguity.

**2. Iteration is tight**
Each cycle takes 15-30 minutes. You get rapid feedback.

**3. One change at a time**
If you changed both tools AND instruction in one step and test failed, you wouldn't know which change broke it. Always change one thing, test, then change the next.

**4. Debugging is systematic**
You didn't guess. You read the failure, traced the cause, made targeted fix.

**5. Progress is visible**
0 → 2 → 3 passing. You see improvement. This maintains momentum.

This is the reality of evaluation-driven development. Not a one-shot test phase at the end. It's the primary development loop.

## Integrating with pytest for CI/CD

Eval cases are great for development. But in production, you need automated testing that integrates with your CI/CD pipeline. That's where `pytest` comes in.

### Create a pytest Test File

```python
import pytest
from google.adk.evaluation.agent_evaluator import AgentEvaluator
import asyncio

@pytest.mark.asyncio
async def test_taskmanager_basic_operations():
    """Test TaskManager agent against basic eval cases."""
    evaluator = AgentEvaluator(
        agent_module="task_manager",
        eval_dataset_file_path_or_dir="evals/taskmanager_basics.test.json"
    )

    results = await evaluator.evaluate()

    # Test passes if ALL eval cases pass
    assert results.passed, (
        f"Eval cases failed:\n"
        f"Passed: {results.passed_count}\n"
        f"Failed: {results.failed_count}\n"
        f"Details: {results.failure_details}"
    )

@pytest.mark.asyncio
async def test_taskmanager_edge_cases():
    """Test TaskManager against edge case eval cases."""
    evaluator = AgentEvaluator(
        agent_module="task_manager",
        eval_dataset_file_path_or_dir="evals/taskmanager_edge_cases.test.json"
    )

    results = await evaluator.evaluate()

    assert results.passed, f"Edge case tests failed: {results.failure_details}"

if __name__ == "__main__":
    # Run with: pytest tests/test_taskmanager.py -v
    pytest.main([__file__, "-v"])
```

**Output:**

```
tests/test_taskmanager.py::test_taskmanager_basic_operations PASSED
tests/test_taskmanager.py::test_taskmanager_edge_cases PASSED

========================= 2 passed in 3.24s =========================
```

### Running Pytest in CI/CD

Once you have pytest tests, they can run automatically on every commit:

```bash
# Local testing
pytest tests/ -v

# In GitHub Actions
- name: Run agent evaluations
  run: pytest tests/ --tb=short
```

When a test fails, your CI/CD pipeline catches it immediately. You don't deploy agents with failing eval cases.

## The Iteration Loop in Practice

Here's the full workflow for a feature you want to add:

```
┌─────────────────────────────────────────────────────────────┐
│ ITERATION 1: Write Eval Cases                              │
│ Define: "What should agent do with this feature?"          │
│ Output: New JSON eval cases in evals/ directory            │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ ITERATION 2: Run adk eval                                  │
│ Command: adk eval agent.py evals/*.test.json               │
│ Result: Tests fail (expected - agent doesn't have feature) │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ ITERATION 3: Read Detailed Output                          │
│ Study: Which tests fail? What's the gap?                  │
│ Action: Identify root cause of each failure               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ ITERATION 4: Improve Agent                                 │
│ Modify: Agent instruction, tools, or both                 │
│ Goal: Address root causes identified in output            │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ ITERATION 5: Re-run adk eval                               │
│ Check: Did your fix work? New failures?                   │
│ If passes → Done. If fails → Loop back to step 3          │
└─────────────────────────────────────────────────────────────┘
```

Every time through this loop, you're making measurable progress. Tests pass = feature works. Tests fail = more work needed.

## Why This Matters: The Bridge Between Spec and Reality

Without evaluation-driven development, you face a problem:

**Agent development is mushy.** There's no clear definition of "done." The agent works in some cases, fails in others. You don't know which behaviors are intentional and which are accidents.

Evaluation-driven development makes it concrete:

- **Specification is executable**: Your eval cases ARE your spec
- **Progress is measurable**: Passed tests = features that work
- **Quality is systematic**: Not "does it feel right?" but "do all tests pass?"
- **Regression is impossible**: If a test passes, that behavior stays stable

This is how you build reliable agents that users can depend on.

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini) to deepen your understanding of evaluation-driven development.

### Prompt 1: Diagnose a Failing Test

```
I'm running adk eval on my TaskManager agent and one eval case failed:

Input: "Delete the second task"
Expected: complete_task(task_id=2)
Actual: Agent responded with "I don't have authority to delete tasks"

My instruction says: "When users ask to delete, call delete_task"
But the agent is refusing. Help me diagnose:
1. What's causing this refusal?
2. Is this a problem with my instruction?
3. How would you rewrite my instruction to fix this?
```

**What you're learning**: Reading eval output closely and diagnosing intent mismatches—the core debugging skill for agent development.

### Prompt 2: Design Edge Case Tests

```
I have basic TaskManager eval cases working. Now I want to add edge cases.
What edge cases should I test for a task management agent?

Think about:
- Empty task lists
- Duplicate task names
- Invalid task IDs
- Ambiguous requests

For each edge case, help me write the eval case that specifies expected behavior.
Ask me: What should the agent do in each scenario? (Be specific—should it error, handle gracefully, etc.)
```

**What you're learning**: Test design thinking—anticipating failure modes and preventing them with specifications.

### Prompt 3: Set Up CI/CD Integration

```
I have pytest tests for my agent evaluation. Now I want to run them automatically.
I'm using [GitHub/GitLab/your platform]. Help me set up a workflow that:
1. Runs adk eval on every commit
2. Blocks merges if tests fail
3. Reports detailed results (which cases passed/failed)
4. Handles API rate limits and timeouts gracefully

What's the minimal CI/CD setup that meets these requirements?
```

**What you're learning**: Production thinking—moving evaluation-first development into automated pipelines.

### Production Safety Note

When running `adk eval` against real LLM APIs:
- **Monitor costs**: API calls accumulate quickly with evals; set usage alerts
- **Handle rate limits**: Add retry logic with exponential backoff
- **Test locally first**: Use smaller eval sets for development; scale in staging
- **Version your evals**: Store eval cases in git so you can track changes

For development, consider mocking the agent to avoid API costs while you're building.

Try With AI: Ask your AI companion how to mock an ADK agent for testing without incurring API charges.
