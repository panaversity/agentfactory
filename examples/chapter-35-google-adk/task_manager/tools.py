"""
Task Management Tools - All task operations using ToolContext for state.

Each tool follows ADK patterns:
- Uses ToolContext for session state access
- Returns dict responses for agent consumption
- Includes comprehensive docstrings for LLM understanding
- Handles edge cases gracefully
"""

from datetime import datetime
from typing import Optional, List, Dict, Any
from google.adk.tools.tool_context import ToolContext
import uuid


def _get_tasks(tool_context: ToolContext) -> List[Dict[str, Any]]:
    """Get tasks from session state, initializing if needed."""
    if "tasks" not in tool_context.state:
        tool_context.state["tasks"] = []
    return tool_context.state["tasks"]


def _save_tasks(tool_context: ToolContext, tasks: List[Dict[str, Any]]) -> None:
    """Save tasks to session state."""
    tool_context.state["tasks"] = tasks


def add_task(
    title: str,
    tool_context: ToolContext,
    description: Optional[str] = None,
    due_date: Optional[str] = None,
    priority: Optional[str] = "medium"
) -> Dict[str, Any]:
    """Add a new task to the task list.

    Args:
        title: The task title (required).
        tool_context: ADK tool context for state access.
        description: Optional detailed description of the task.
        due_date: Optional due date in YYYY-MM-DD format.
        priority: Task priority - one of 'low', 'medium', 'high'. Defaults to 'medium'.

    Returns:
        A dict with status and the created task details.

    Example:
        add_task("Complete project proposal", description="Include budget", priority="high")
    """
    tasks = _get_tasks(tool_context)

    # Validate priority
    valid_priorities = ["low", "medium", "high"]
    if priority and priority.lower() not in valid_priorities:
        return {
            "status": "error",
            "message": f"Invalid priority '{priority}'. Must be one of: {valid_priorities}"
        }

    # Validate due_date format if provided
    if due_date:
        try:
            datetime.strptime(due_date, "%Y-%m-%d")
        except ValueError:
            return {
                "status": "error",
                "message": f"Invalid date format '{due_date}'. Use YYYY-MM-DD format."
            }

    task = {
        "id": str(uuid.uuid4())[:8],  # Short UUID for readability
        "title": title,
        "description": description,
        "due_date": due_date,
        "priority": priority.lower() if priority else "medium",
        "status": "pending",
        "created_at": datetime.now().isoformat(),
        "updated_at": datetime.now().isoformat(),
    }

    tasks.append(task)
    _save_tasks(tool_context, tasks)

    return {
        "status": "created",
        "message": f"Task '{title}' created successfully.",
        "task": task
    }


def list_tasks(
    tool_context: ToolContext,
    filter_status: Optional[str] = None,
    filter_priority: Optional[str] = None
) -> Dict[str, Any]:
    """List all tasks, optionally filtered by status or priority.

    Args:
        tool_context: ADK tool context for state access.
        filter_status: Optional filter by status ('pending', 'completed').
        filter_priority: Optional filter by priority ('low', 'medium', 'high').

    Returns:
        A dict with the list of tasks matching the filters.

    Example:
        list_tasks(filter_status="pending", filter_priority="high")
    """
    tasks = _get_tasks(tool_context)

    # Apply filters
    filtered_tasks = tasks

    if filter_status:
        filtered_tasks = [t for t in filtered_tasks if t.get("status") == filter_status.lower()]

    if filter_priority:
        filtered_tasks = [t for t in filtered_tasks if t.get("priority") == filter_priority.lower()]

    # Sort by priority (high > medium > low) then by due_date
    priority_order = {"high": 0, "medium": 1, "low": 2}
    filtered_tasks = sorted(
        filtered_tasks,
        key=lambda t: (
            priority_order.get(t.get("priority", "medium"), 1),
            t.get("due_date") or "9999-99-99"  # Tasks without due date go last
        )
    )

    return {
        "status": "success",
        "count": len(filtered_tasks),
        "total_tasks": len(tasks),
        "filters_applied": {
            "status": filter_status,
            "priority": filter_priority
        },
        "tasks": filtered_tasks
    }


def complete_task(task_id: str, tool_context: ToolContext) -> Dict[str, Any]:
    """Mark a task as completed.

    Args:
        task_id: The unique identifier of the task to complete.
        tool_context: ADK tool context for state access.

    Returns:
        A dict with status and the updated task details.

    Example:
        complete_task("abc123")
    """
    tasks = _get_tasks(tool_context)

    for task in tasks:
        if task["id"] == task_id:
            if task["status"] == "completed":
                return {
                    "status": "already_completed",
                    "message": f"Task '{task['title']}' is already completed.",
                    "task": task
                }

            task["status"] = "completed"
            task["completed_at"] = datetime.now().isoformat()
            task["updated_at"] = datetime.now().isoformat()
            _save_tasks(tool_context, tasks)

            return {
                "status": "completed",
                "message": f"Task '{task['title']}' marked as completed.",
                "task": task
            }

    return {
        "status": "error",
        "message": f"Task with ID '{task_id}' not found."
    }


def delete_task(task_id: str, tool_context: ToolContext) -> Dict[str, Any]:
    """Delete a task from the task list.

    Args:
        task_id: The unique identifier of the task to delete.
        tool_context: ADK tool context for state access.

    Returns:
        A dict with status and confirmation of deletion.

    Example:
        delete_task("abc123")
    """
    tasks = _get_tasks(tool_context)

    for i, task in enumerate(tasks):
        if task["id"] == task_id:
            deleted_task = tasks.pop(i)
            _save_tasks(tool_context, tasks)

            return {
                "status": "deleted",
                "message": f"Task '{deleted_task['title']}' has been deleted.",
                "deleted_task": deleted_task
            }

    return {
        "status": "error",
        "message": f"Task with ID '{task_id}' not found."
    }


def edit_task(
    task_id: str,
    tool_context: ToolContext,
    title: Optional[str] = None,
    description: Optional[str] = None,
    due_date: Optional[str] = None,
    priority: Optional[str] = None
) -> Dict[str, Any]:
    """Edit an existing task's properties.

    Args:
        task_id: The unique identifier of the task to edit.
        tool_context: ADK tool context for state access.
        title: New title for the task (optional).
        description: New description for the task (optional).
        due_date: New due date in YYYY-MM-DD format (optional).
        priority: New priority - 'low', 'medium', or 'high' (optional).

    Returns:
        A dict with status and the updated task details.

    Example:
        edit_task("abc123", title="Updated title", priority="high")
    """
    tasks = _get_tasks(tool_context)

    # Validate priority if provided
    if priority:
        valid_priorities = ["low", "medium", "high"]
        if priority.lower() not in valid_priorities:
            return {
                "status": "error",
                "message": f"Invalid priority '{priority}'. Must be one of: {valid_priorities}"
            }

    # Validate due_date format if provided
    if due_date:
        try:
            datetime.strptime(due_date, "%Y-%m-%d")
        except ValueError:
            return {
                "status": "error",
                "message": f"Invalid date format '{due_date}'. Use YYYY-MM-DD format."
            }

    for task in tasks:
        if task["id"] == task_id:
            # Track what changed
            changes = []

            if title and title != task["title"]:
                task["title"] = title
                changes.append("title")

            if description is not None and description != task.get("description"):
                task["description"] = description
                changes.append("description")

            if due_date and due_date != task.get("due_date"):
                task["due_date"] = due_date
                changes.append("due_date")

            if priority and priority.lower() != task.get("priority"):
                task["priority"] = priority.lower()
                changes.append("priority")

            if changes:
                task["updated_at"] = datetime.now().isoformat()
                _save_tasks(tool_context, tasks)

                return {
                    "status": "updated",
                    "message": f"Task '{task['title']}' updated. Changed: {', '.join(changes)}",
                    "changes": changes,
                    "task": task
                }
            else:
                return {
                    "status": "no_changes",
                    "message": "No changes were made to the task.",
                    "task": task
                }

    return {
        "status": "error",
        "message": f"Task with ID '{task_id}' not found."
    }


def search_tasks(query: str, tool_context: ToolContext) -> Dict[str, Any]:
    """Search tasks by title or description.

    Args:
        query: The search query to match against task titles and descriptions.
        tool_context: ADK tool context for state access.

    Returns:
        A dict with matching tasks.

    Example:
        search_tasks("project proposal")
    """
    tasks = _get_tasks(tool_context)
    query_lower = query.lower()

    matching_tasks = []
    for task in tasks:
        title_match = query_lower in task.get("title", "").lower()
        desc_match = query_lower in (task.get("description") or "").lower()

        if title_match or desc_match:
            matching_tasks.append({
                **task,
                "match_in": "title" if title_match else "description"
            })

    return {
        "status": "success",
        "query": query,
        "count": len(matching_tasks),
        "tasks": matching_tasks
    }
