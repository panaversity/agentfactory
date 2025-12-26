"""
TaskManager Agent - A complete ADK example demonstrating multi-agent workflows.

This package implements a task management agent using Google's Agent Development Kit,
showcasing:
- SequentialAgent for pipeline orchestration
- LlmAgent for intent classification and execution
- LoopAgent for quality validation
- Tool callbacks for safety guardrails
- Session state management

Usage:
    python -m task_manager
    # or
    adk run task_manager
"""

from task_manager.agent import root_agent
from task_manager.tools import (
    add_task,
    list_tasks,
    complete_task,
    delete_task,
    edit_task,
    search_tasks,
)
from task_manager.callbacks import (
    block_dangerous_input,
    protect_critical_tasks,
    log_all_operations,
)
from task_manager.session import get_session_service

__all__ = [
    # Main agent
    "root_agent",
    # Tools
    "add_task",
    "list_tasks",
    "complete_task",
    "delete_task",
    "edit_task",
    "search_tasks",
    # Callbacks
    "block_dangerous_input",
    "protect_critical_tasks",
    "log_all_operations",
    # Session
    "get_session_service",
]
