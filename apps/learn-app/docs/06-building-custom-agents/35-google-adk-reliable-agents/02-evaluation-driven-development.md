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
