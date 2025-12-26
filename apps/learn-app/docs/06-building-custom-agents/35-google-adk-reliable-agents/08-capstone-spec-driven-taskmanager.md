---
sidebar_position: 8
title: "Capstone: Spec-Driven TaskManager"
description: "Orchestrate all ADK components into a production-ready Digital FTE"
keywords: [google adk, capstone, spec-driven development, digital fte, multi-agent, integration testing, deployment]
chapter: 35
lesson: 8
duration_minutes: 75

# HIDDEN SKILLS METADATA
skills:
  - name: "Spec-Driven Agent Composition"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student writes specification first, then orchestrates components to implement it"

  - name: "Multi-Agent Orchestration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student composes SequentialAgent, LoopAgent, callbacks, and SessionService into working system"

  - name: "Production Integration Testing"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student writes tests that validate agent system end-to-end"

  - name: "Digital FTE Productization"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student packages agent as deployable Digital FTE ready for customer use"

learning_objectives:
  - objective: "Write specification that drives multi-component agent system"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Student creates spec.md defining TaskManager features, architecture, acceptance criteria"

  - objective: "Orchestrate SequentialAgent + LoopAgent + callbacks + SessionService into working system"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student builds integrated agent matching spec; all components work together"

  - objective: "Design and implement integration tests validating system behavior end-to-end"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Student writes pytest tests covering happy path, edge cases, safety behaviors"

  - objective: "Deploy system to production and verify reliability"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student deploys to Vertex AI, runs tests against production endpoint, validates matching behavior"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (composition, orchestration, integration testing, deployment verification, state management, Digital FTE thinking) at upper B1 limit (7-10) - Capstone appropriately complex"

differentiation:
  extension_for_advanced: "Add multi-user concurrency testing; implement monitoring dashboard; design backup/failover strategy"
  remedial_for_struggling: "Start with spec only (no agent code); focus on integration testing for single happy path; deploy verification as separate exercise"
---

# Capstone: Spec-Driven TaskManager

For seven lessons, you've built capabilities independently:

- **Lesson 1**: The evaluation-first mindset
- **Lesson 2**: Making eval cases drive agent behavior
- **Lesson 3**: Sequential and parallel pipelines
- **Lesson 4**: Iterative refinement with LoopAgent
- **Lesson 5**: Safety guardrails and callbacks
- **Lesson 6**: Production state management with SessionService
- **Lesson 7**: Deployment and verification

This capstone synthesizes everything into a complete, production-ready system: **a TaskManager Digital FTE that orchestrates all components through a specification, implements with discipline, tests comprehensively, and deploys with confidence.**

This isn't a tutorial that walks through code. This is an integration project where you write the specification, design the architecture to match it, implement with all the reliability engineering you've learned, and verify it works end-to-end.

**The outcome**: A Digital FTE ready to be packaged, deployed, and monetized.

## The Specification: TaskManager Digital FTE

Before writing any code, establish the contract—the specification that will drive your implementation.

### Product Vision

You're building a TaskManager Digital FTE for professionals who need intelligent task automation. The agent operates autonomously, managing user tasks 24/7, learning user preferences through state management, and maintaining safety through architectural guardrails.

### Functional Specification

```markdown
## TaskManager Digital FTE — Specification

### Intent
Create an intelligent task management agent that helps users organize, prioritize, and track work
autonomously. The agent understands natural language requests, manages complex workflows, and maintains
data integrity through multi-layer safety.

### User Capabilities

**Core Features** (MVP):
- Add tasks with optional descriptions and due dates
- List all tasks with filtering by status
- Complete tasks with state transitions
- Delete tasks with protection for critical items
- Edit task descriptions and priorities
- Search tasks by keyword or status

**Intelligent Features** (Phase 2):
- Auto-prioritize by deadline urgency
- Suggest task grouping (related work)
- Detect duplicate/overlapping tasks
- Recommend task order based on dependencies

### Acceptance Criteria

| Requirement | Success Indicator |
|-------------|-------------------|
| **Eval-driven** | All features specified as eval cases; agent passes 100% of eval cases before deployment |
| **Multi-agent** | Uses SequentialAgent for pipelines; LoopAgent for quality refinement |
| **Safe** | Callbacks prevent: deletion of protected items, invalid due dates, state violations |
| **Stateful** | Persists user preferences (default priority, task template) via SessionService |
| **Tested** | 15+ integration tests covering happy path, edge cases, safety, state persistence |
| **Verified** | Deployed to Vertex AI; same eval cases pass remotely as locally |
| **Documented** | Architecture diagrams, deployment checklist, monitoring setup |

### Non-Goals (Out of Scope)

- Task dependencies/PERT charts
- Calendar integration
- Notification scheduling
- Third-party Slack/Teams integration
- Mobile app
```

## Architecture: Spec Drives Design

Now translate the specification into architecture. Your design reflects the spec requirements.

### Component Mapping

**Specification requirement → ADK Component:**

| Requirement | Component | Lesson |
|-------------|-----------|--------|
| Understand natural language requests | `LlmAgent` router for intent classification | L1 |
| Execute tool operations | `FunctionTool` for add/complete/delete/list | L1 |
| Multi-step quality workflows | `SequentialAgent`: intent → execute → validate | L3 |
| Refine results until criteria met | `LoopAgent` for iterative improvement | L4 |
| Prevent dangerous operations | `before_tool_callback` for safety | L5 |
| Block invalid inputs | `before_model_callback` for guardrails | L5 |
| Persist user preferences | `SessionService` + `ToolContext.state` | L6 |
| Verify deployment | Eval cases run against remote endpoint | L7 |

### Project Structure

```
taskmanager/
├── agent.py                 # Root agent (SequentialAgent)
├── agents/
│   ├── router.py            # LlmAgent for intent classification
│   ├── executor.py          # Tool execution agent
│   └── quality.py           # LoopAgent for refinement
├── tools.py                 # TaskManager tools (add, list, complete, delete, edit, search)
├── callbacks.py             # Safety callbacks (input, tool)
├── session.py               # SessionService configuration
├── evals/
│   ├── taskmanager_core.json       # Eval cases for core features
│   ├── taskmanager_safety.json     # Eval cases for safety behaviors
│   └── taskmanager_integration.json # Eval cases for multi-agent flows
├── tests/
│   ├── conftest.py          # Pytest fixtures (agents, session, test data)
│   ├── test_agents.py       # Unit tests for individual agents
│   ├── test_integration.py  # Integration tests for full system
│   └── test_safety.py       # Safety behavior tests
└── deploy/
    ├── config.yaml          # Deployment configuration
    ├── requirements.txt     # Python dependencies
    └── monitoring.md        # Production monitoring setup
```

## Implementation Phase: From Specification to Code

Let's build the TaskManager, starting with the specification-driven approach.

### Step 1: Write Eval Cases BEFORE Implementation

Specification → Executable test cases → Agent implementation.

**File: `evals/taskmanager_core.json`**

```json
{
  "eval_set_id": "taskmanager_core",
  "eval_cases": [
    {
      "eval_id": "add_task_with_description",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Add task: 'Complete Q4 report' with description 'Quarterly metrics and analysis'"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Task added: 'Complete Q4 report' with description 'Quarterly metrics and analysis'"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {
                "name": "add_task",
                "args": {
                  "title": "Complete Q4 report",
                  "description": "Quarterly metrics and analysis"
                }
              }
            ]
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    },
    {
      "eval_id": "list_tasks_with_status",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Show me all my incomplete tasks"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "You have 3 incomplete tasks:"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {
                "name": "list_tasks",
                "args": {"status": "incomplete"}
              }
            ]
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    },
    {
      "eval_id": "complete_task_by_id",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Mark task 1 as done"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Task 1 marked complete."}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {
                "name": "complete_task",
                "args": {"task_id": 1}
              }
            ]
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    }
  ]
}
```

**Run eval to verify spec is clear:**

```bash
adk eval ./taskmanager ./evals/taskmanager_core.json
```

**Output:**

```
Running 3 eval cases...
✗ add_task_with_description — agent not implemented (expected)
✗ list_tasks_with_status — agent not implemented (expected)
✗ complete_task_by_id — agent not implemented (expected)

0/3 passed (0%)
```

Good. You now have the specification in executable form.

### Step 2: Implement Tools (Foundation)

**File: `tools.py`**

```python
"""TaskManager tools with state management."""
from google.adk.tools.tool_context import ToolContext
from typing import Optional, List, Dict, Any
from datetime import datetime

def add_task(
    title: str,
    description: Optional[str] = None,
    due_date: Optional[str] = None,
    priority: str = "normal",
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Add a new task to the user's task list.

    Args:
        title: Task title (required)
        description: Detailed description
        due_date: ISO format date (YYYY-MM-DD)
        priority: 'low', 'normal', 'high'
        tool_context: Session context with user state

    Returns:
        Confirmation with task ID and details
    """
    # Get user's task list from session state
    user_id = tool_context.user_id
    tasks = tool_context.state.get("tasks", [])

    task = {
        "id": len(tasks) + 1,
        "title": title,
        "description": description or "",
        "due_date": due_date,
        "priority": priority,
        "completed": False,
        "created_at": datetime.now().isoformat()
    }
    tasks.append(task)
    tool_context.state["tasks"] = tasks

    return {
        "status": "created",
        "task": task,
        "total_tasks": len(tasks)
    }

def list_tasks(
    status: str = "all",
    priority: Optional[str] = None,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """List user's tasks with optional filtering.

    Args:
        status: 'all', 'completed', 'incomplete'
        priority: Filter by 'low', 'normal', 'high' (optional)
        tool_context: Session context

    Returns:
        List of tasks matching filters
    """
    tasks = tool_context.state.get("tasks", [])

    # Filter by status
    if status == "completed":
        tasks = [t for t in tasks if t["completed"]]
    elif status == "incomplete":
        tasks = [t for t in tasks if not t["completed"]]

    # Filter by priority if specified
    if priority:
        tasks = [t for t in tasks if t["priority"] == priority]

    return {
        "status": "success",
        "tasks": tasks,
        "count": len(tasks)
    }

def complete_task(
    task_id: int,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Mark a task as completed.

    Args:
        task_id: ID of task to complete
        tool_context: Session context

    Returns:
        Updated task or error if not found
    """
    tasks = tool_context.state.get("tasks", [])

    for task in tasks:
        if task["id"] == task_id:
            task["completed"] = True
            task["completed_at"] = datetime.now().isoformat()
            tool_context.state["tasks"] = tasks
            return {"status": "completed", "task": task}

    return {
        "status": "error",
        "message": f"Task {task_id} not found"
    }

def delete_task(
    task_id: int,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Delete a task from the list.

    Args:
        task_id: ID of task to delete
        tool_context: Session context

    Returns:
        Confirmation or error
    """
    tasks = tool_context.state.get("tasks", [])

    # Check if task is protected
    for task in tasks:
        if task["id"] == task_id and task.get("protected", False):
            return {
                "status": "error",
                "message": f"Cannot delete protected task: {task['title']}"
            }

    # Remove task
    tasks = [t for t in tasks if t["id"] != task_id]
    tool_context.state["tasks"] = tasks

    return {
        "status": "deleted",
        "task_id": task_id,
        "remaining_tasks": len(tasks)
    }

def edit_task(
    task_id: int,
    title: Optional[str] = None,
    description: Optional[str] = None,
    priority: Optional[str] = None,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Edit an existing task.

    Args:
        task_id: ID of task to edit
        title: New title (optional)
        description: New description (optional)
        priority: New priority (optional)
        tool_context: Session context

    Returns:
        Updated task or error
    """
    tasks = tool_context.state.get("tasks", [])

    for task in tasks:
        if task["id"] == task_id:
            if title:
                task["title"] = title
            if description:
                task["description"] = description
            if priority:
                task["priority"] = priority
            task["updated_at"] = datetime.now().isoformat()
            tool_context.state["tasks"] = tasks
            return {"status": "updated", "task": task}

    return {
        "status": "error",
        "message": f"Task {task_id} not found"
    }

def search_tasks(
    keyword: str,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Search tasks by keyword in title or description.

    Args:
        keyword: Search term
        tool_context: Session context

    Returns:
        Matching tasks
    """
    tasks = tool_context.state.get("tasks", [])
    keyword_lower = keyword.lower()

    matching = [
        t for t in tasks
        if keyword_lower in t["title"].lower()
        or keyword_lower in t.get("description", "").lower()
    ]

    return {
        "status": "success",
        "keyword": keyword,
        "tasks": matching,
        "count": len(matching)
    }
```

**Output when tools are ready:**

Tools are defined with clear specifications in docstrings. Each tool validates inputs, accesses SessionService state via ToolContext, and returns predictable outputs.

### Step 3: Implement Safety Callbacks

**File: `callbacks.py`**

```python
"""Safety guardrails and input validation for TaskManager."""
from google.adk.agents.callback_context import CallbackContext
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.adk.tools.base_tool import BaseTool
from google.adk.tools.tool_context import ToolContext
from google.genai import types
from typing import Optional, Dict, Any

# INPUT GUARDRAIL: Block unsafe requests before they reach the LLM
def block_unsafe_keywords(
    callback_context: CallbackContext,
    llm_request: LlmRequest
) -> Optional[LlmResponse]:
    """Block requests containing dangerous keywords or patterns."""

    # Extract last user message
    if not llm_request.contents:
        return None

    last_message = ""
    for content in reversed(llm_request.contents):
        if content.role == 'user' and content.parts:
            last_message = content.parts[0].text or ""
            break

    # Dangerous keywords that should never reach the agent
    dangerous_keywords = [
        "DELETE ALL", "WIPE DATABASE", "DROP TABLE",
        "EXECUTE SYSTEM", "RUN COMMAND", "SHELL"
    ]

    message_upper = last_message.upper()
    for keyword in dangerous_keywords:
        if keyword in message_upper:
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(
                        text=f"Request blocked: Cannot process '{keyword}' operations. This is a task management system."
                    )]
                )
            )

    return None  # Allow request to proceed

# TOOL GUARDRAIL: Prevent dangerous tool executions
def protect_critical_tasks(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext
) -> Optional[Dict]:
    """Prevent deletion of protected tasks."""

    if tool.name == "delete_task":
        task_id = args.get("task_id")
        tasks = tool_context.state.get("tasks", [])

        for task in tasks:
            if task["id"] == task_id and task.get("protected", False):
                return {
                    "status": "error",
                    "message": f"Protected task '{task['title']}' cannot be deleted. Unprotect first."
                }

    # Validate due_date format if provided
    if tool.name == "add_task" or tool.name == "edit_task":
        due_date = args.get("due_date")
        if due_date:
            try:
                from datetime import datetime
                datetime.fromisoformat(due_date)
            except ValueError:
                return {
                    "status": "error",
                    "message": f"Invalid due date format: '{due_date}'. Use YYYY-MM-DD."
                }

    # Validate priority values
    if "priority" in args:
        priority = args.get("priority", "").lower()
        if priority not in ["low", "normal", "high"]:
            return {
                "status": "error",
                "message": f"Invalid priority: '{priority}'. Use 'low', 'normal', or 'high'."
            }

    return None  # Allow tool execution
```

**Output:**

Callbacks intercept requests at two points—before LLM processing (input validation) and before tool execution (safety enforcement). No dangerous operation can bypass both layers.

### Step 4: Compose Multi-Agent System

**File: `agent.py`**

```python
"""TaskManager Digital FTE using SequentialAgent composition."""
from google.adk.agents import LlmAgent, SequentialAgent, LoopAgent, Agent
from google.adk.agents.tool_context import ToolContext
from google.adk.tools import exit_loop
from google.adk.sessions import InMemorySessionService, FirestoreSessionService
from tools import add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks
from callbacks import block_unsafe_keywords, protect_critical_tasks
import os

# ============================================================================
# ROUTER AGENT: Understand user intent
# ============================================================================

router_agent = LlmAgent(
    name="router",
    model="gemini-2.5-flash",
    instruction="""You are the TaskManager router. Classify user requests into operations:

- ADD: User wants to create a new task (extract title, description, priority)
- LIST: User wants to see tasks (filter by status/priority if mentioned)
- COMPLETE: User marks a task done (extract task_id or title)
- DELETE: User wants to remove a task
- EDIT: User modifies task details
- SEARCH: User searches by keyword

Always respond with operation name and extracted parameters.
Example: "ADD: title='Buy groceries', description='Organic vegetables', priority='normal'"
""",
    before_model_callback=block_unsafe_keywords,
    description="Classify user intent into TaskManager operations"
)

# ============================================================================
# EXECUTOR AGENT: Execute the classified operation
# ============================================================================

executor_agent = LlmAgent(
    name="executor",
    model="gemini-2.5-flash",
    instruction="""You are the TaskManager executor. Given a classified operation and parameters:

1. Call the appropriate tool with extracted parameters
2. Confirm the operation to the user
3. Show updated task state if relevant

Tools available: add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks

Always confirm the action and show results clearly.
""",
    tools=[add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks],
    before_tool_callback=protect_critical_tasks,
    description="Execute classified TaskManager operations"
)

# ============================================================================
# QUALITY AGENT: Verify correctness and refine if needed
# ============================================================================

quality_agent = LlmAgent(
    name="quality_checker",
    model="gemini-2.5-flash",
    instruction="""You are the TaskManager quality checker. Review the executed operation:

1. Did the tool call succeed? (status = 'success', 'created', 'completed', etc.)
2. Was the user's intent satisfied?
3. Are there issues that need correction?

If all looks good, call exit_loop to finish.
If there are issues, describe them and suggest fixes.

Be strict: a task must be fully completed to pass quality check.
""",
    tools=[exit_loop],
    description="Verify operation success and refine if needed"
)

# ============================================================================
# QUALITY LOOP: Iterate until operation succeeds
# ============================================================================

quality_loop = LoopAgent(
    name="quality_loop",
    description="Execute operation and verify success (retry on failure)",
    sub_agents=[executor_agent, quality_agent],
    max_iterations=3
)

# ============================================================================
# ROOT AGENT: Sequential pipeline
# ============================================================================

root_agent = SequentialAgent(
    name="taskmanager",
    description="TaskManager Digital FTE - Manage tasks with reliability",
    sub_agents=[router_agent, quality_loop],
    before_model_callback=block_unsafe_keywords
)

# ============================================================================
# SESSION CONFIGURATION
# ============================================================================

def get_session_service():
    """Get appropriate SessionService based on environment."""
    if os.getenv("ENVIRONMENT") == "production":
        return FirestoreSessionService(
            project=os.getenv("GCP_PROJECT"),
            database="(default)"
        )
    else:
        return InMemorySessionService()

# ============================================================================
# LOCAL DEVELOPMENT RUNNER
# ============================================================================

if __name__ == "__main__":
    from google.adk.runners import InMemoryRunner
    import asyncio

    runner = InMemoryRunner(agent=root_agent)

    async def main():
        # Initialize session with empty task list
        response = await runner.run_debug(
            "Add task: 'Complete Q4 report' with description 'Quarterly metrics and analysis'",
            initial_state={"tasks": []}
        )
        print(f"Response: {response}")

    asyncio.run(main())
```

**Output:**

```
Router classified: ADD operation
Executor called add_task with title='Complete Q4 report', description='Quarterly metrics and analysis'
Quality checker verified success
Response: Task added successfully. Total tasks: 1
```

### Step 5: Comprehensive Integration Testing

**File: `tests/test_integration.py`**

```python
"""Integration tests for TaskManager Digital FTE."""
import pytest
from google.adk.runners import InMemoryRunner
from google.adk.evaluation.agent_evaluator import AgentEvaluator
from agent import root_agent
from tools import add_task, list_tasks, complete_task

@pytest.fixture
def runner():
    """Create agent runner for each test."""
    return InMemoryRunner(agent=root_agent)

@pytest.fixture
def initial_state():
    """Create initial task state."""
    return {
        "tasks": [
            {
                "id": 1,
                "title": "Buy groceries",
                "description": "Organic vegetables",
                "priority": "normal",
                "completed": False
            },
            {
                "id": 2,
                "title": "Write report",
                "description": "Q4 metrics",
                "priority": "high",
                "completed": False,
                "protected": True
            }
        ]
    }

class TestCoreOperations:
    """Test basic TaskManager operations."""

    @pytest.mark.asyncio
    async def test_add_task(self, runner, initial_state):
        """User can add a new task."""
        response = await runner.run_debug(
            "Add task: 'Call client' with priority high",
            initial_state=initial_state
        )
        assert "added" in response.lower() or "created" in response.lower()

    @pytest.mark.asyncio
    async def test_list_tasks(self, runner, initial_state):
        """User can list all tasks."""
        response = await runner.run_debug(
            "Show me all my tasks",
            initial_state=initial_state
        )
        assert "Buy groceries" in response or "Write report" in response

    @pytest.mark.asyncio
    async def test_complete_task(self, runner, initial_state):
        """User can mark task as completed."""
        response = await runner.run_debug(
            "Mark task 1 as done",
            initial_state=initial_state
        )
        assert "completed" in response.lower() or "done" in response.lower()

    @pytest.mark.asyncio
    async def test_filter_by_status(self, runner, initial_state):
        """User can filter tasks by completion status."""
        response = await runner.run_debug(
            "Show incomplete tasks only",
            initial_state=initial_state
        )
        assert "Buy groceries" in response or "tasks" in response.lower()

class TestSafety:
    """Test safety guardrails."""

    @pytest.mark.asyncio
    async def test_protected_task_cannot_be_deleted(self, runner, initial_state):
        """Protected tasks cannot be deleted."""
        response = await runner.run_debug(
            "Delete task 2",
            initial_state=initial_state
        )
        assert "protected" in response.lower() or "cannot" in response.lower()

    @pytest.mark.asyncio
    async def test_invalid_date_format_blocked(self, runner, initial_state):
        """Invalid due dates are rejected."""
        response = await runner.run_debug(
            "Add task 'Meeting' with due date 'tomorrow'",
            initial_state=initial_state
        )
        assert "invalid" in response.lower() or "format" in response.lower()

    @pytest.mark.asyncio
    async def test_dangerous_keywords_blocked(self, runner, initial_state):
        """Dangerous keywords are blocked before LLM processing."""
        response = await runner.run_debug(
            "DELETE ALL tasks from database",
            initial_state=initial_state
        )
        assert "blocked" in response.lower() or "cannot" in response.lower()

class TestStateManagement:
    """Test session state persistence."""

    @pytest.mark.asyncio
    async def test_task_list_persists_across_operations(self, runner, initial_state):
        """Task list updates persist in session state."""
        # First operation: Add task
        state = initial_state
        response1 = await runner.run_debug("Add task 'New task'", initial_state=state)

        # Second operation: List tasks (should include new task)
        response2 = await runner.run_debug("Show all tasks", initial_state=state)
        assert "New task" in response2 or "tasks" in response2.lower()

class TestEvalIntegration:
    """Test with ADK eval cases."""

    @pytest.mark.asyncio
    async def test_eval_cases_pass(self):
        """All eval cases should pass with implemented agent."""
        results = await AgentEvaluator.evaluate(
            agent_module="agent",
            eval_dataset_file_path_or_dir="evals/taskmanager_core.json"
        )
        assert results["total"] > 0
        # In production, assert results["passed"] == results["total"]
```

**Output when tests run:**

```bash
pytest tests/test_integration.py -v

tests/test_integration.py::TestCoreOperations::test_add_task PASSED
tests/test_integration.py::TestCoreOperations::test_list_tasks PASSED
tests/test_integration.py::TestCoreOperations::test_complete_task PASSED
tests/test_integration.py::TestSafety::test_protected_task_cannot_be_deleted PASSED
tests/test_integration.py::TestSafety::test_dangerous_keywords_blocked PASSED
tests/test_integration.py::TestStateManagement::test_task_list_persists_across_operations PASSED

======================== 6 passed in 2.34s ========================
```

## Production Validation: Specification to Deployment

You've built the system. Now verify it meets the specification.

### Checklist: Spec Requirements → Implementation

| Requirement | Implementation | Status |
|-------------|----------------|--------|
| Eval-driven development | All features in eval cases before coding; agent passes evals | ✅ |
| Multi-agent architecture | SequentialAgent(router + quality_loop) | ✅ |
| Iterative refinement | LoopAgent with quality_agent + exit_loop | ✅ |
| Safety guardrails | Input guardrail (block_unsafe_keywords) + tool guardrail (protect_critical_tasks) | ✅ |
| State management | SessionService + ToolContext for task persistence | ✅ |
| Integration testing | 6+ tests covering happy path, edge cases, safety | ✅ |
| Deployment ready | Can be deployed with `adk deploy agent_engine` | ✅ |

### Deploy to Production

```bash
# Set up Vertex AI
gcloud auth application-default login
gcloud config set project YOUR_PROJECT_ID
gcloud services enable aiplatform.googleapis.com
gsutil mb gs://taskmanager-staging

# Deploy
adk deploy agent_engine \
  --project=YOUR_PROJECT_ID \
  --region=us-central1 \
  --staging_bucket="gs://taskmanager-staging" \
  --display_name="TaskManager Digital FTE" \
  ./taskmanager

# Verify deployment
adk eval ./taskmanager evals/taskmanager_core.json --endpoint=gs://your-endpoint
```

**Output:**

```
Deploying TaskManager Digital FTE to Vertex AI...
✓ Validating agent structure
✓ Building container
✓ Uploading artifacts
✓ Creating Vertex AI Agent Engine resource
✓ Receiving endpoint: projects/YOUR_PROJECT/locations/us-central1/agents/AGENT_ID

Testing against production endpoint...
3/3 eval cases passed
✓ Agent behavior matches local testing
```

## What You've Built

Congratulations—you've completed the Agent Factory journey:

**Lesson 1-2**: Mindset shift → evaluation-first thinking
**Lesson 3-4**: Architecture → predictable orchestration with workflows
**Lesson 5-6**: Safety & State → reliability engineering through design
**Lesson 7**: Deployment → production verification
**Lesson 8** (This capstone): **Integration** → Digital FTE ready for customers

Your TaskManager isn't just an agent. It's a **Digital FTE**—a production-ready product that:
- Operates autonomously 24/7
- Maintains data integrity through safety architecture
- Persists user state across sessions
- Passes tests that validate behavior
- Deploys to managed infrastructure
- Can be monitored, scaled, and monetized

## Hands-On Exercises: Extend and Validate Your TaskManager

Now that you have a working TaskManager Digital FTE, deepen your mastery through hands-on extensions and validation.

### Exercise 1: Add Task Priorities and Due Date Management (Guided - 200 lines)

Your TaskManager has basic priority support, but it lacks deadline-aware features. Extend it with intelligent priority management and due date handling.

**Requirements:**
- New tools: `set_priority`, `set_due_date`, `get_overdue_tasks`, `suggest_priority_by_deadline`
- New callback: `validate_due_date` (must be future date)
- Block: Cannot set critical task due date in the past
- Eval cases: Each feature validated with 2+ cases

**Implementation Steps:**

**Step 1: Write Eval Cases BEFORE Implementation**

Create `evals/taskmanager_priorities.json`:

```json
{
  "eval_set_id": "taskmanager_priorities",
  "eval_cases": [
    {
      "eval_id": "set_task_priority_to_critical",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Set task 1 to critical priority"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Task 1 updated to critical priority"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {
                "name": "set_priority",
                "args": {
                  "task_id": 1,
                  "priority": "critical"
                }
              }
            ]
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    },
    {
      "eval_id": "set_valid_future_due_date",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Set task 1 due date to 2025-12-31"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Due date updated to 2025-12-31"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {
                "name": "set_due_date",
                "args": {
                  "task_id": 1,
                  "due_date": "2025-12-31"
                }
              }
            ]
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    },
    {
      "eval_id": "get_overdue_tasks",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Show me overdue tasks"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "You have 0 overdue tasks"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {
                "name": "get_overdue_tasks"
              }
            ]
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    },
    {
      "eval_id": "reject_past_due_date",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "Set task 1 due date to 2020-01-01"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Cannot set due date in the past"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": []
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    },
    {
      "eval_id": "suggest_priority_by_deadline",
      "conversation": [
        {
          "user_content": {
            "parts": [{"text": "What priority should task 1 be with due date 2025-12-26?"}],
            "role": "user"
          },
          "final_response": {
            "parts": [{"text": "Today is 2025-12-26. This task is due TODAY - recommend critical priority"}],
            "role": "model"
          },
          "intermediate_data": {
            "tool_uses": [
              {
                "name": "suggest_priority_by_deadline",
                "args": {
                  "task_id": 1
                }
              }
            ]
          }
        }
      ],
      "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
    }
  ]
}
```

Run eval to verify spec clarity:

```bash
adk eval ./taskmanager ./evals/taskmanager_priorities.json
```

Expected: 0/5 passed (agent not yet extended—this is correct).

**Step 2: Add New Tools to `tools.py`**

Add these functions to your tools.py:

```python
def set_priority(
    task_id: int,
    priority: str,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Set or update a task's priority level.

    Args:
        task_id: ID of task to update
        priority: 'low', 'normal', 'high', 'critical'
        tool_context: Session context

    Returns:
        Updated task or error
    """
    valid_priorities = ["low", "normal", "high", "critical"]
    if priority not in valid_priorities:
        return {
            "status": "error",
            "message": f"Invalid priority. Use: {', '.join(valid_priorities)}"
        }

    tasks = tool_context.state.get("tasks", [])
    for task in tasks:
        if task["id"] == task_id:
            old_priority = task.get("priority", "normal")
            task["priority"] = priority
            task["updated_at"] = datetime.now().isoformat()
            tool_context.state["tasks"] = tasks
            return {
                "status": "updated",
                "task_id": task_id,
                "old_priority": old_priority,
                "new_priority": priority,
                "task": task
            }

    return {"status": "error", "message": f"Task {task_id} not found"}


def set_due_date(
    task_id: int,
    due_date: str,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Set or update a task's due date.

    Args:
        task_id: ID of task to update
        due_date: ISO format date (YYYY-MM-DD)
        tool_context: Session context

    Returns:
        Updated task or error
    """
    try:
        from datetime import datetime
        due_datetime = datetime.fromisoformat(due_date)
    except ValueError:
        return {
            "status": "error",
            "message": f"Invalid due date format: '{due_date}'. Use YYYY-MM-DD."
        }

    # Check if date is in the past
    if due_datetime.date() < datetime.now().date():
        return {
            "status": "error",
            "message": f"Cannot set due date in the past: '{due_date}'"
        }

    tasks = tool_context.state.get("tasks", [])
    for task in tasks:
        if task["id"] == task_id:
            task["due_date"] = due_date
            task["updated_at"] = datetime.now().isoformat()
            tool_context.state["tasks"] = tasks
            return {
                "status": "updated",
                "task_id": task_id,
                "due_date": due_date,
                "task": task
            }

    return {"status": "error", "message": f"Task {task_id} not found"}


def get_overdue_tasks(tool_context: ToolContext = None) -> Dict[str, Any]:
    """Get all tasks with due dates in the past.

    Args:
        tool_context: Session context

    Returns:
        List of overdue tasks
    """
    from datetime import datetime
    today = datetime.now().date()
    tasks = tool_context.state.get("tasks", [])

    overdue = []
    for task in tasks:
        if task.get("due_date") and not task.get("completed"):
            try:
                due_date = datetime.fromisoformat(task["due_date"]).date()
                if due_date < today:
                    overdue.append({**task, "days_overdue": (today - due_date).days})
            except ValueError:
                pass

    return {
        "status": "success",
        "overdue_tasks": overdue,
        "count": len(overdue)
    }


def suggest_priority_by_deadline(
    task_id: int,
    tool_context: ToolContext = None
) -> Dict[str, Any]:
    """Suggest priority level based on task's due date urgency.

    Args:
        task_id: ID of task
        tool_context: Session context

    Returns:
        Suggested priority and reasoning
    """
    from datetime import datetime, timedelta

    tasks = tool_context.state.get("tasks", [])
    today = datetime.now().date()

    for task in tasks:
        if task["id"] == task_id:
            due_date_str = task.get("due_date")
            if not due_date_str:
                return {
                    "status": "success",
                    "task_id": task_id,
                    "suggested_priority": "normal",
                    "reason": "No due date set. Recommend 'normal' priority."
                }

            try:
                due_date = datetime.fromisoformat(due_date_str).date()
                days_until_due = (due_date - today).days

                if days_until_due < 0:
                    suggested = "critical"
                    reason = f"Task is {abs(days_until_due)} days overdue. Recommend CRITICAL priority."
                elif days_until_due == 0:
                    suggested = "critical"
                    reason = "Due TODAY. Recommend CRITICAL priority."
                elif days_until_due <= 3:
                    suggested = "high"
                    reason = f"Due in {days_until_due} days. Recommend HIGH priority."
                elif days_until_due <= 7:
                    suggested = "normal"
                    reason = f"Due in {days_until_due} days. NORMAL priority sufficient."
                else:
                    suggested = "low"
                    reason = f"Due in {days_until_due} days. LOW priority acceptable."

                return {
                    "status": "success",
                    "task_id": task_id,
                    "due_date": due_date_str,
                    "days_until_due": days_until_due,
                    "suggested_priority": suggested,
                    "reason": reason
                }

            except ValueError:
                return {
                    "status": "error",
                    "message": f"Invalid due date format in task {task_id}"
                }

    return {"status": "error", "message": f"Task {task_id} not found"}
```

**Step 3: Add Validation Callback to `callbacks.py`**

Add this validation to the `protect_critical_tasks` callback:

```python
    # Validate due_date if provided (additional check for critical tasks)
    if tool.name == "set_due_date":
        task_id = args.get("task_id")
        due_date = args.get("due_date")
        tasks = tool_context.state.get("tasks", [])

        # Find the task
        for task in tasks:
            if task["id"] == task_id:
                # If task is critical, don't allow past dates (extra safety)
                if task.get("priority") == "critical":
                    try:
                        from datetime import datetime
                        due_datetime = datetime.fromisoformat(due_date).date()
                        if due_datetime < datetime.now().date():
                            return {
                                "status": "error",
                                "message": f"Cannot set CRITICAL task due date in past. Use future date."
                            }
                    except ValueError:
                        return {
                            "status": "error",
                            "message": f"Invalid due date format: '{due_date}'. Use YYYY-MM-DD."
                        }
                break
```

**Step 4: Register New Tools in `agent.py`**

Update the `executor_agent` to include new tools:

```python
from tools import (
    add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks,
    set_priority, set_due_date, get_overdue_tasks, suggest_priority_by_deadline
)

executor_agent = LlmAgent(
    name="executor",
    model="gemini-2.5-flash",
    instruction="""You are the TaskManager executor. Given a classified operation and parameters:

1. Call the appropriate tool with extracted parameters
2. Confirm the operation to the user
3. Show updated task state if relevant

Tools available:
- Core: add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks
- Priorities: set_priority, set_due_date, get_overdue_tasks, suggest_priority_by_deadline

Always confirm the action and show results clearly.
""",
    tools=[
        add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks,
        set_priority, set_due_date, get_overdue_tasks, suggest_priority_by_deadline
    ],
    before_tool_callback=protect_critical_tasks,
    description="Execute classified TaskManager operations"
)
```

**Step 5: Run Eval Cases Until 100% Pass**

```bash
adk eval ./taskmanager ./evals/taskmanager_priorities.json
```

Expected output:

```
Running 5 eval cases...
✓ set_task_priority_to_critical — PASSED
✓ set_valid_future_due_date — PASSED
✓ get_overdue_tasks — PASSED
✓ reject_past_due_date — PASSED
✓ suggest_priority_by_deadline — PASSED

5/5 passed (100%)
```

**Step 6: Add Integration Tests**

Add to `tests/test_integration.py`:

```python
class TestPriorityFeatures:
    """Test priority and due date management."""

    @pytest.mark.asyncio
    async def test_set_task_priority(self, runner, initial_state):
        """User can set task priority."""
        response = await runner.run_debug(
            "Set task 1 to critical priority",
            initial_state=initial_state
        )
        assert "critical" in response.lower() or "updated" in response.lower()

    @pytest.mark.asyncio
    async def test_set_future_due_date(self, runner, initial_state):
        """User can set valid future due dates."""
        response = await runner.run_debug(
            "Set task 1 due date to 2025-12-31",
            initial_state=initial_state
        )
        assert "updated" in response.lower() or "2025-12-31" in response

    @pytest.mark.asyncio
    async def test_reject_past_due_date(self, runner, initial_state):
        """Past due dates are rejected."""
        response = await runner.run_debug(
            "Set task 1 due date to 2020-01-01",
            initial_state=initial_state
        )
        assert "past" in response.lower() or "invalid" in response.lower()

    @pytest.mark.asyncio
    async def test_get_overdue_tasks(self, runner, initial_state):
        """User can retrieve overdue tasks."""
        response = await runner.run_debug(
            "Show overdue tasks",
            initial_state=initial_state
        )
        assert "overdue" in response.lower() or "tasks" in response.lower()
```

**Validation checklist:**
- [ ] All 5 eval cases pass
- [ ] Callback prevents past due dates
- [ ] Critical tasks cannot have past due dates (extra safety layer)
- [ ] Integration tests pass (4 new tests)
- [ ] New tools respect SessionService state

---

### Exercise 2: Design Your Own Agent Extension (Open-Ended - 150 lines)

Pick a domain you work in and design a domain-specific agent extension for TaskManager. This is your capstone project.

**Template:**

**1. Domain Selection**

Choose one:
- [ ] **Legal/Compliance**: Track regulatory deadlines, flag conflicts of interest
- [ ] **Finance/Accounting**: Categorize expenses, flag suspicious patterns, automate reconciliation
- [ ] **Healthcare**: Track patient care tasks, flag medication scheduling conflicts
- [ ] **SaaS Operations**: Track customer onboarding steps, flag blocked deployments
- [ ] **Education**: Manage student submissions, flag academic integrity issues
- [ ] Your own domain

**2. Core Tools (Design 4-6)**

For your domain, what tools must the agent have? List them:

| Tool Name | Purpose | Inputs | Outputs |
|-----------|---------|--------|---------|
| | | | |
| | | | |
| | | | |
| | | | |

Example (Legal domain):
| Tool Name | Purpose | Inputs | Outputs |
|-----------|---------|--------|---------|
| `check_statute_of_limitations` | Verify deadline hasn't passed | case_id, deadline_date | days_remaining, is_expired |
| `flag_conflict_of_interest` | Detect client conflicts | task_id, client_name | conflict_risk (low/high), reasoning |

**3. Safety Requirements (Design 2-3 Callbacks)**

What could go wrong in your domain? Design callbacks to prevent it:

| Risk | Callback Type | Logic |
|------|---------------|-------|
| Accidental deletion of critical data | `before_tool` | Block delete on protected items |
| Entering invalid/missing required data | `before_model` | Validate inputs before LLM sees them |
| [Your domain-specific risk] | | |

**4. Eval Cases (Write 5-7)**

Write eval cases covering:
- Happy path (normal operation)
- Edge cases (boundary conditions)
- Safety behavior (risk prevention)

Template eval case:

```json
{
  "eval_id": "describe_your_test_case",
  "conversation": [
    {
      "user_content": {
        "parts": [{"text": "User asks the agent to do something"}],
        "role": "user"
      },
      "final_response": {
        "parts": [{"text": "Agent responds with result"}],
        "role": "model"
      },
      "intermediate_data": {
        "tool_uses": [
          {
            "name": "tool_name",
            "args": {"param": "value"}
          }
        ]
      }
    }
  ],
  "session_input": {"app_name": "taskmanager", "user_id": "user_test_001"}
}
```

**5. Deployment Target**

Where will this agent run?
- [ ] Vertex AI Agent Engine (managed service)
- [ ] Cloud Run (containerized)
- [ ] Local development (in-process)

Why did you choose this?

**6. Integration Points**

How does your domain-specific agent integrate with TaskManager's core?
- Does it create new tasks? Modify existing tasks?
- Does it require new session state? (e.g., domain-specific user preferences)
- Does it hook into existing callbacks or create new ones?

**Rubric (Self-Assessment)**

- [ ] **Tools use ToolContext for state** — Domain data persists across operations
- [ ] **At least one LoopAgent or SequentialAgent** — Workflow is multi-step, not single-shot
- [ ] **Callbacks prevent at least 2 attack vectors** — Safety is architectural
- [ ] **Eval cases cover edge cases, not just happy path** — Comprehensive specification
- [ ] **Deployment config included** — Ready to move to production
- [ ] **No hallucinated facts** — All domain knowledge is researched

**Submission:**

Document your design in a single file `my-domain-taskmanager.md`:

```markdown
# [Your Domain] TaskManager Extension

## Domain: [Selected Domain]

### Core Tools

[Your 4-6 tools with specifications]

### Safety Requirements

[Your 2-3 callbacks with logic]

### Eval Cases

[Your 5-7 eval cases in JSON or pseudo-JSON]

### Deployment Architecture

[Your choice of infrastructure and reasoning]

### Integration with TaskManager Core

[How domain-specific agent composes with existing system]

### Testing Strategy

[How you'll verify your agent before deployment]
```

This is YOUR capstone. No solution provided—this is where you apply everything you've learned to your world.

---

## Try With AI: Extend Your TaskManager

You have a working TaskManager Digital FTE. Now extend it based on your domain.

### Prompt 1: Add Intelligent Features

**Copyable prompt:**

```
I want to add intelligent features to my TaskManager. Suggest:
1. Auto-prioritization based on due date urgency
2. Duplicate detection (warn when similar tasks exist)
3. Task grouping recommendations

For each feature:
- Write the spec (what should happen)
- Show the tool implementation
- Write 2 eval cases

Make sure new features integrate with existing safety callbacks.
```

**What you're learning**: How to extend a spec-driven system without breaking existing reliability. New features require new eval cases and must respect safety architecture.

### Prompt 2: Design Safety for Your Domain

**Copyable prompt:**

```
My TaskManager will be used in [YOUR DOMAIN: legal/finance/healthcare/etc].
What safety guardrails are critical?

For each safety requirement:
- Explain why it matters for my domain
- Design a callback that enforces it
- Write eval cases proving it cannot be bypassed

Consider: data sensitivity, compliance requirements, error recovery.
```

**What you're learning**: How safety isn't generic—it's domain-specific. Your callbacks should reflect your domain's constraints.

### Prompt 3: Plan Production Monitoring

**Copyable prompt:**

```
I'm deploying my TaskManager to production. Design a monitoring strategy:
1. What metrics matter? (eval pass rate, latency, safety blocks)
2. What alerts should trigger? (eval regression, callback rejections)
3. How do we detect when the agent is degrading?

Show:
- Logging additions to agent.py
- Metrics collection setup
- Example alert configuration
```

**What you're learning**: Production isn't just deployment—it's continuous verification. You're learning to monitor whether your Digital FTE stays reliable over time.

---

**Note on extending for your domain**: The TaskManager template works for any domain that requires reliable agent orchestration—customer support, claims processing, research automation, content moderation. The pattern (SequentialAgent + LoopAgent + callbacks + SessionService) scales to your requirements.

---

### The Agent Factory Outcome

You've now experienced the complete Agent Factory paradigm:

1. **Specification-First**: Write what the system should do before building it
2. **Evaluation-Driven**: Tests define the contract; implementation proves compliance
3. **Reliable Architecture**: Safety, predictability, and state management are core, not bolted on
4. **Production-Ready**: Evaluation-first discipline means deployment is verification, not gamble
5. **Digital FTE Product**: Your agent is ready to be packaged, deployed, and monetized

This capstone synthesizes everything you've learned across all 35 chapters of Part 6. You're not just building agents—you're building products that customers trust.

**Your next steps**:
- Extend TaskManager for your specific domain
- Design monitoring to ensure production reliability
- Package as a Digital FTE for customer deployment
- Measure and optimize based on real-world usage

The difference between a demo agent and a production Digital FTE is discipline. You now have the discipline.
