"""
Safety Callbacks - Guardrails for the TaskManager agent.

Implements three types of callbacks:
1. before_model_callback: Block dangerous user inputs
2. before_tool_callback: Protect critical tasks from modification
3. after callbacks: Audit logging for all operations
"""

from datetime import datetime
from typing import Optional, Dict, Any
from google.adk.agents.callback_context import CallbackContext
from google.adk.models.llm_request import LlmRequest
from google.adk.models.llm_response import LlmResponse
from google.adk.tools.base_tool import BaseTool
from google.adk.tools.tool_context import ToolContext
from google.genai import types


# Keywords that should trigger input blocking
BLOCKED_KEYWORDS = [
    "delete all",
    "remove everything",
    "clear all tasks",
    "drop database",
    "sql injection",
    "system prompt",
    "ignore previous",
]

# Task IDs that are protected from modification/deletion
# In production, this would come from a database or config
CRITICAL_TASK_PREFIXES = ["CRIT-", "PROD-", "SYS-"]


def block_dangerous_input(
    callback_context: CallbackContext,
    llm_request: LlmRequest
) -> Optional[LlmResponse]:
    """Block requests containing dangerous keywords.

    This before_model_callback inspects user input before it reaches
    the model. If dangerous patterns are detected, it returns a
    safe response instead of allowing the request to proceed.

    Args:
        callback_context: Context about the current callback execution.
        llm_request: The request that will be sent to the LLM.

    Returns:
        LlmResponse if request should be blocked, None to allow.
    """
    # Extract the last user message
    last_user_message = ""
    if llm_request.contents:
        for content in reversed(llm_request.contents):
            if content.role == 'user' and content.parts:
                for part in content.parts:
                    if part.text:
                        last_user_message = part.text.lower()
                        break
                if last_user_message:
                    break

    # Check for blocked keywords
    for keyword in BLOCKED_KEYWORDS:
        if keyword.lower() in last_user_message:
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(text=(
                        f"I cannot process requests containing '{keyword}'. "
                        "For safety reasons, bulk operations and potentially "
                        "destructive actions must be performed individually. "
                        "Please rephrase your request to work with specific tasks."
                    ))]
                )
            )

    # Check for prompt injection attempts
    injection_patterns = [
        "you are now",
        "forget your instructions",
        "new instructions:",
        "override system",
    ]

    for pattern in injection_patterns:
        if pattern in last_user_message:
            return LlmResponse(
                content=types.Content(
                    role="model",
                    parts=[types.Part(text=(
                        "I'm designed to help you manage tasks. "
                        "How can I help you add, view, or manage your tasks today?"
                    ))]
                )
            )

    # Allow request to proceed
    return None


def protect_critical_tasks(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext
) -> Optional[Dict[str, Any]]:
    """Protect critical tasks from modification or deletion.

    This before_tool_callback runs before each tool execution.
    It prevents deletion or modification of tasks marked as critical.

    Args:
        tool: The tool that is about to be executed.
        args: The arguments that will be passed to the tool.
        tool_context: Context with access to session state.

    Returns:
        Dict with error if blocked, None to allow execution.
    """
    # Only intercept delete and edit operations
    protected_operations = ["delete_task", "edit_task", "complete_task"]

    if tool.name not in protected_operations:
        return None

    # Get the task_id from arguments
    task_id = args.get("task_id", "")

    # Check if task ID starts with a critical prefix
    for prefix in CRITICAL_TASK_PREFIXES:
        if task_id.startswith(prefix):
            return {
                "status": "blocked",
                "message": (
                    f"Task '{task_id}' is marked as critical and cannot be "
                    f"{tool.name.replace('_task', '')}d. Please contact an "
                    "administrator if you need to modify this task."
                ),
                "task_id": task_id,
                "operation": tool.name
            }

    # Additional protection: prevent editing completed high-priority tasks
    if tool.name == "edit_task":
        tasks = tool_context.state.get("tasks", [])
        for task in tasks:
            if (task["id"] == task_id and
                task.get("status") == "completed" and
                task.get("priority") == "high"):
                return {
                    "status": "blocked",
                    "message": (
                        f"Task '{task['title']}' is a completed high-priority "
                        "task and cannot be edited. This is an audit requirement."
                    ),
                    "task_id": task_id,
                    "operation": tool.name
                }

    # Allow the operation
    return None


def log_all_operations(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext,
    result: Dict[str, Any]
) -> None:
    """Log all tool operations for audit purposes.

    This after_tool_callback runs after each tool execution.
    It creates an audit log entry in the session state.

    Args:
        tool: The tool that was executed.
        args: The arguments that were passed to the tool.
        tool_context: Context with access to session state.
        result: The result returned by the tool.
    """
    # Initialize audit log if not exists
    if "audit_log" not in tool_context.state:
        tool_context.state["audit_log"] = []

    # Create log entry
    log_entry = {
        "timestamp": datetime.now().isoformat(),
        "operation": tool.name,
        "args": _sanitize_args(args),
        "status": result.get("status", "unknown"),
        "success": result.get("status") not in ["error", "blocked"],
    }

    # Add task details for relevant operations
    if "task" in result:
        log_entry["task_id"] = result["task"].get("id")
        log_entry["task_title"] = result["task"].get("title")
    elif "deleted_task" in result:
        log_entry["task_id"] = result["deleted_task"].get("id")
        log_entry["task_title"] = result["deleted_task"].get("title")
    elif "task_id" in result:
        log_entry["task_id"] = result["task_id"]

    # Append to audit log
    tool_context.state["audit_log"].append(log_entry)

    # Keep only last 100 entries to prevent unbounded growth
    if len(tool_context.state["audit_log"]) > 100:
        tool_context.state["audit_log"] = tool_context.state["audit_log"][-100:]


def _sanitize_args(args: Dict[str, Any]) -> Dict[str, Any]:
    """Sanitize arguments for logging (remove sensitive data)."""
    sanitized = {}
    for key, value in args.items():
        # Skip tool_context from logs
        if key == "tool_context":
            continue
        # Truncate long values
        if isinstance(value, str) and len(value) > 200:
            sanitized[key] = value[:200] + "..."
        else:
            sanitized[key] = value
    return sanitized


def create_after_tool_callback(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext,
    result: Dict[str, Any]
) -> Dict[str, Any]:
    """Factory function to create after_tool_callback that logs operations.

    Note: This is structured as a wrapper to match ADK's expected callback
    signature while still calling our logging function.

    Args:
        tool: The tool that was executed.
        args: The arguments that were passed to the tool.
        tool_context: Context with access to session state.
        result: The result returned by the tool.

    Returns:
        The original result (unmodified).
    """
    log_all_operations(tool, args, tool_context, result)
    return result
