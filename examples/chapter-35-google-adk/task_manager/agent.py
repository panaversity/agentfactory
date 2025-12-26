"""
TaskManager Agent - Multi-agent workflow for task management.

This module implements a complete TaskManager using ADK's multi-agent patterns:

Architecture:
    ┌─────────────────────────────────────────────────────────────┐
    │                    TaskManager (SequentialAgent)            │
    │                                                             │
    │  ┌───────────────┐    ┌───────────────┐    ┌─────────────┐ │
    │  │    Router     │ -> │   Executor    │ -> │   Quality   │ │
    │  │  (LlmAgent)   │    │  (LlmAgent)   │    │  (LoopAgent)│ │
    │  │               │    │               │    │             │ │
    │  │ Classifies    │    │ Executes task │    │ Validates   │ │
    │  │ user intent   │    │ operations    │    │ results     │ │
    │  └───────────────┘    └───────────────┘    └─────────────┘ │
    │                                                             │
    └─────────────────────────────────────────────────────────────┘

Usage:
    # Run with adk CLI
    adk run task_manager

    # Or run directly
    python -m task_manager
"""

from google.adk.agents import Agent, LlmAgent, SequentialAgent, LoopAgent
from google.adk.tools import exit_loop

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
)


# Model configuration
MODEL = "gemini-2.5-flash"


# -----------------------------------------------------------------------------
# Router Agent - Intent Classification
# -----------------------------------------------------------------------------

router = LlmAgent(
    name="router",
    model=MODEL,
    instruction="""You are an intent classifier for a task management system.

Your job is to understand what the user wants to do and prepare clear instructions
for the executor agent.

Analyze the user's request and output a structured response with:
1. INTENT: One of [ADD_TASK, LIST_TASKS, COMPLETE_TASK, DELETE_TASK, EDIT_TASK, SEARCH_TASKS, HELP, UNKNOWN]
2. PARAMETERS: Extracted parameters relevant to the intent
3. CONFIDENCE: HIGH, MEDIUM, or LOW

Examples:
- "Add buy groceries" -> INTENT: ADD_TASK, PARAMETERS: {title: "buy groceries"}, CONFIDENCE: HIGH
- "Show me all high priority tasks" -> INTENT: LIST_TASKS, PARAMETERS: {filter_priority: "high"}, CONFIDENCE: HIGH
- "Mark task abc123 as done" -> INTENT: COMPLETE_TASK, PARAMETERS: {task_id: "abc123"}, CONFIDENCE: HIGH
- "Find tasks about project" -> INTENT: SEARCH_TASKS, PARAMETERS: {query: "project"}, CONFIDENCE: HIGH
- "What can you do?" -> INTENT: HELP, PARAMETERS: {}, CONFIDENCE: HIGH

If the request is unclear, set CONFIDENCE to LOW and ask for clarification.

Output format:
---
INTENT: <intent>
PARAMETERS: <json_parameters>
CONFIDENCE: <level>
NOTES: <any clarifications needed>
---""",
    description="Classifies user intent and extracts parameters for task operations.",
)


# -----------------------------------------------------------------------------
# Executor Agent - Task Operations
# -----------------------------------------------------------------------------

executor = LlmAgent(
    name="executor",
    model=MODEL,
    instruction="""You are a task executor with access to task management tools.

Based on the router's classification, execute the appropriate tool:

Tools available:
- add_task(title, description, due_date, priority): Create a new task
- list_tasks(filter_status, filter_priority): List tasks with optional filters
- complete_task(task_id): Mark a task as completed
- delete_task(task_id): Remove a task
- edit_task(task_id, title, description, due_date, priority): Modify a task
- search_tasks(query): Find tasks matching a query

Guidelines:
1. Execute the tool that matches the classified intent
2. Use the parameters extracted by the router
3. If parameters are missing, make reasonable assumptions or ask the user
4. After executing, provide a clear summary of what was done
5. For list operations, format the output in a readable way

When listing tasks, format them nicely:
- Show task ID, title, priority, status, and due date
- Use bullet points or numbered lists
- Highlight high-priority or overdue tasks

Always be helpful and confirm actions were completed successfully.""",
    description="Executes task management operations using the available tools.",
    tools=[add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks],
    before_tool_callback=protect_critical_tasks,
)


# -----------------------------------------------------------------------------
# Quality Checker Agent - Validation Loop
# -----------------------------------------------------------------------------

quality_checker = LlmAgent(
    name="quality_checker",
    model=MODEL,
    instruction="""You are a quality checker for task management operations.

Your job is to validate that the executor's response is:
1. COMPLETE: All requested actions were performed
2. ACCURATE: The response correctly reflects what was done
3. HELPFUL: The user has clear information about next steps

Check the executor's output:
- Did it successfully perform the requested operation?
- Is the response clear and informative?
- Are there any errors that need to be addressed?

If the quality is satisfactory, call exit_loop to complete the workflow.
If issues are found, provide feedback for improvement.

Call exit_loop when:
- Operation completed successfully
- User received clear confirmation
- No errors or ambiguities remain""",
    description="Validates operation results and ensures quality responses.",
    tools=[exit_loop],
)

quality_loop = LoopAgent(
    name="quality_loop",
    description="Iteratively validates results until quality standards are met.",
    sub_agents=[quality_checker],
    max_iterations=2,  # Maximum 2 validation passes
)


# -----------------------------------------------------------------------------
# Root Agent - Sequential Pipeline
# -----------------------------------------------------------------------------

root_agent = SequentialAgent(
    name="task_manager",
    description="""A comprehensive task management assistant that helps users:
- Add new tasks with titles, descriptions, due dates, and priorities
- List and filter tasks by status or priority
- Mark tasks as completed
- Delete tasks
- Edit existing tasks
- Search for tasks by keyword

The agent processes requests through a pipeline:
1. Router: Understands user intent
2. Executor: Performs the requested operation
3. Quality Checker: Validates the result""",
    sub_agents=[router, executor, quality_loop],
    before_model_callback=block_dangerous_input,
)


# -----------------------------------------------------------------------------
# Main Entry Point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    import asyncio
    from google.adk import Runner
    from google.genai import types
    from task_manager.session import get_session_service

    async def demo():
        """Demonstrate the TaskManager agent."""
        print("=" * 60)
        print("TaskManager Agent Demo")
        print("=" * 60)

        # Create runner with in-memory session
        session_service = get_session_service("development")
        runner = Runner(
            app_name="task_manager_demo",
            agent=root_agent,
            session_service=session_service
        )

        # Create a session
        session = await session_service.create_session(
            app_name="task_manager_demo",
            user_id="demo_user"
        )

        print(f"\nSession created: {session.id}")
        print("-" * 60)

        # Demo interactions
        demo_messages = [
            "Add a task: Complete project proposal with high priority, due 2024-12-31",
            "Add another task: Review code changes, medium priority",
            "Show me all my tasks",
            "Search for proposal",
        ]

        for message in demo_messages:
            print(f"\nUser: {message}")
            print("-" * 40)

            user_content = types.Content(
                role='user',
                parts=[types.Part(text=message)]
            )

            async for event in runner.run_async(
                user_id=session.user_id,
                session_id=session.id,
                new_message=user_content
            ):
                if event.content and event.content.parts:
                    for part in event.content.parts:
                        if part.text:
                            print(f"Agent: {part.text}")

            print()

        print("=" * 60)
        print("Demo complete!")
        print("=" * 60)

    # Run the demo
    asyncio.run(demo())
