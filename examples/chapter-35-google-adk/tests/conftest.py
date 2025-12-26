"""
Pytest fixtures for Google ADK tests.

This module provides fixtures for:
- Mock agents (no API calls)
- Real agents (requires API key)
- Session services
- Sample task data
"""

import os
import pytest
from typing import List, Dict, Any, Optional
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from dataclasses import dataclass, field


# ============================================================================
# Sample Data Fixtures
# ============================================================================

@pytest.fixture
def sample_tasks() -> List[Dict[str, Any]]:
    """Provide sample task data for testing.

    Returns:
        List of task dictionaries with id, title, description, completed fields.

    Example:
        def test_with_tasks(sample_tasks):
            assert len(sample_tasks) == 3
            assert sample_tasks[0]["title"] == "Write documentation"
    """
    return [
        {
            "id": 1,
            "title": "Write documentation",
            "description": "Document the API endpoints",
            "completed": False
        },
        {
            "id": 2,
            "title": "Fix bug #123",
            "description": "Memory leak in session handler",
            "completed": False
        },
        {
            "id": 3,
            "title": "Review PR",
            "description": "Review the new feature branch",
            "completed": True
        }
    ]


@pytest.fixture
def empty_tasks() -> List[Dict[str, Any]]:
    """Provide empty task list for edge case testing.

    Returns:
        Empty list representing no tasks.
    """
    return []


@pytest.fixture
def sample_user_messages() -> List[Dict[str, str]]:
    """Provide sample user messages for agent testing.

    Returns:
        List of message dictionaries with role and content.
    """
    return [
        {"role": "user", "content": "Add a task: Buy groceries"},
        {"role": "user", "content": "Show me all my tasks"},
        {"role": "user", "content": "Mark task 1 as complete"},
        {"role": "user", "content": "Delete task 2"},
        {"role": "user", "content": "Search for tasks about groceries"}
    ]


# ============================================================================
# Mock Types (for type hints)
# ============================================================================

@dataclass
class MockSession:
    """Mock session object for testing."""
    id: str = "test-session-123"
    user_id: str = "test-user-456"
    app_name: str = "test-app"
    state: Dict[str, Any] = field(default_factory=dict)


@dataclass
class MockContent:
    """Mock content object mimicking google.genai.types.Content."""
    role: str
    parts: List[Any] = field(default_factory=list)


@dataclass
class MockPart:
    """Mock part object mimicking google.genai.types.Part."""
    text: Optional[str] = None
    function_call: Optional[Any] = None
    function_response: Optional[Any] = None


@dataclass
class MockEvent:
    """Mock event object for runner events."""
    content: Optional[MockContent] = None
    author: str = "model"
    is_final: bool = False


# ============================================================================
# Session Service Fixtures
# ============================================================================

@pytest.fixture
def mock_session_service():
    """Create a mock InMemorySessionService.

    Returns:
        Mock session service with async methods.

    Example:
        async def test_session(mock_session_service):
            session = await mock_session_service.create_session(
                app_name="test", user_id="user1"
            )
            assert session.id == "test-session-123"
    """
    service = AsyncMock()
    mock_session = MockSession()

    service.create_session = AsyncMock(return_value=mock_session)
    service.get_session = AsyncMock(return_value=mock_session)
    service.delete_session = AsyncMock(return_value=True)
    service.list_sessions = AsyncMock(return_value=[mock_session])

    return service


@pytest.fixture
def mock_firestore_service():
    """Create a mock FirestoreSessionService.

    Returns:
        Mock Firestore session service for production-like tests.
    """
    service = AsyncMock()
    mock_session = MockSession()
    mock_session.state = {"persist_test": True}

    service.create_session = AsyncMock(return_value=mock_session)
    service.get_session = AsyncMock(return_value=mock_session)
    service.update_session = AsyncMock(return_value=mock_session)
    service.delete_session = AsyncMock(return_value=True)

    return service


# ============================================================================
# Tool Context Fixtures
# ============================================================================

@pytest.fixture
def mock_tool_context():
    """Create a mock ToolContext for tool function testing.

    Returns:
        Mock ToolContext with state dictionary.

    Example:
        def test_tool_with_context(mock_tool_context):
            mock_tool_context.state["key"] = "value"
            assert mock_tool_context.state.get("key") == "value"
    """
    context = Mock()
    context.state = {}
    context.session_id = "test-session-123"
    context.user_id = "test-user-456"
    return context


@pytest.fixture
def mock_callback_context():
    """Create a mock CallbackContext for guardrail testing.

    Returns:
        Mock CallbackContext with agent and session info.
    """
    context = Mock()
    context.agent_name = "test_agent"
    context.session_id = "test-session-123"
    context.state = {}
    return context


# ============================================================================
# Agent Fixtures (Mocked - No API Calls)
# ============================================================================

@pytest.fixture
def mock_agent():
    """Create a mock Agent that doesn't make API calls.

    Returns:
        Mock Agent object for unit testing.

    Example:
        def test_agent_config(mock_agent):
            assert mock_agent.name == "test_agent"
            assert mock_agent.model == "gemini-2.5-flash"
    """
    agent = Mock()
    agent.name = "test_agent"
    agent.model = "gemini-2.5-flash"
    agent.instruction = "You are a helpful test assistant."
    agent.tools = []
    agent.sub_agents = []
    agent.before_model_callback = None
    agent.before_tool_callback = None
    return agent


@pytest.fixture
def mock_runner(mock_agent, mock_session_service):
    """Create a mock Runner that doesn't make API calls.

    Returns:
        Mock Runner with async run method.

    Example:
        async def test_runner(mock_runner):
            events = []
            async for event in mock_runner.run_async(...):
                events.append(event)
    """
    runner = Mock()
    runner.agent = mock_agent
    runner.session_service = mock_session_service
    runner.app_name = "test_app"

    # Create async generator for run_async
    async def mock_run_async(*args, **kwargs):
        events = [
            MockEvent(
                content=MockContent(
                    role="model",
                    parts=[MockPart(text="Task added successfully.")]
                ),
                is_final=True
            )
        ]
        for event in events:
            yield event

    runner.run_async = mock_run_async
    return runner


# ============================================================================
# Real Agent Fixtures (Requires API Key)
# ============================================================================

def has_api_key() -> bool:
    """Check if Google API key is available."""
    return bool(os.environ.get("GOOGLE_API_KEY"))


@pytest.fixture
def real_session_service():
    """Create a real InMemorySessionService for integration tests.

    Requires:
        google-adk package installed

    Returns:
        Real InMemorySessionService instance.
    """
    try:
        from google.adk.sessions import InMemorySessionService
        return InMemorySessionService()
    except ImportError:
        pytest.skip("google-adk not installed")


@pytest.fixture
def real_taskmanager_agent():
    """Create a real TaskManager agent for integration tests.

    Requires:
        GOOGLE_API_KEY environment variable

    Returns:
        Real Agent configured as TaskManager.

    Example:
        @pytest.mark.slow
        async def test_real_agent(real_taskmanager_agent):
            # This makes actual API calls
            ...
    """
    if not has_api_key():
        pytest.skip("GOOGLE_API_KEY not set")

    try:
        from google.adk.agents import Agent

        # Task storage for this agent
        tasks: List[Dict[str, Any]] = []

        def add_task(title: str, description: Optional[str] = None) -> Dict:
            """Add a new task."""
            task = {
                "id": len(tasks) + 1,
                "title": title,
                "description": description,
                "completed": False
            }
            tasks.append(task)
            return {"status": "created", "task": task}

        def list_tasks() -> Dict:
            """List all tasks."""
            return {"tasks": tasks}

        def complete_task(task_id: int) -> Dict:
            """Mark a task as completed."""
            for task in tasks:
                if task["id"] == task_id:
                    task["completed"] = True
                    return {"status": "completed", "task": task}
            return {"status": "error", "message": f"Task {task_id} not found"}

        def delete_task(task_id: int) -> Dict:
            """Delete a task."""
            nonlocal tasks
            original_len = len(tasks)
            tasks = [t for t in tasks if t["id"] != task_id]
            if len(tasks) < original_len:
                return {"status": "deleted", "task_id": task_id}
            return {"status": "error", "message": f"Task {task_id} not found"}

        def search_tasks(query: str) -> Dict:
            """Search tasks by title or description."""
            matches = [
                t for t in tasks
                if query.lower() in t["title"].lower() or
                   (t["description"] and query.lower() in t["description"].lower())
            ]
            return {"matches": matches, "count": len(matches)}

        return Agent(
            name="task_manager",
            model="gemini-2.5-flash",
            instruction="""You are a TaskManager assistant.

Help users manage their tasks:
- Add new tasks with add_task
- List all tasks with list_tasks
- Complete tasks with complete_task
- Delete tasks with delete_task
- Search tasks with search_tasks

Always confirm actions and show current task list after changes.""",
            tools=[add_task, list_tasks, complete_task, delete_task, search_tasks]
        )
    except ImportError:
        pytest.skip("google-adk not installed")


@pytest.fixture
def real_runner(real_taskmanager_agent, real_session_service):
    """Create a real Runner for integration tests.

    Requires:
        GOOGLE_API_KEY environment variable
        google-adk package installed

    Returns:
        Real Runner instance with TaskManager agent.
    """
    try:
        from google.adk import Runner

        return Runner(
            app_name="test_taskmanager",
            agent=real_taskmanager_agent,
            session_service=real_session_service
        )
    except ImportError:
        pytest.skip("google-adk not installed")


# ============================================================================
# Workflow Agent Fixtures
# ============================================================================

@pytest.fixture
def mock_sequential_agent():
    """Create a mock SequentialAgent for workflow testing.

    Returns:
        Mock SequentialAgent with sub_agents list.
    """
    agent = Mock()
    agent.name = "sequential_workflow"
    agent.description = "Process tasks in sequence"

    sub1 = Mock()
    sub1.name = "step_1"
    sub1.execution_order = 1

    sub2 = Mock()
    sub2.name = "step_2"
    sub2.execution_order = 2

    agent.sub_agents = [sub1, sub2]
    return agent


@pytest.fixture
def mock_parallel_agent():
    """Create a mock ParallelAgent for concurrent testing.

    Returns:
        Mock ParallelAgent with sub_agents list.
    """
    agent = Mock()
    agent.name = "parallel_workflow"
    agent.description = "Process tasks in parallel"

    sub1 = Mock()
    sub1.name = "analyzer_1"

    sub2 = Mock()
    sub2.name = "analyzer_2"

    agent.sub_agents = [sub1, sub2]
    return agent


@pytest.fixture
def mock_loop_agent():
    """Create a mock LoopAgent for iterative testing.

    Returns:
        Mock LoopAgent with max_iterations setting.
    """
    agent = Mock()
    agent.name = "loop_workflow"
    agent.description = "Iterate until condition met"
    agent.max_iterations = 5

    solver = Mock()
    solver.name = "solver"

    agent.sub_agents = [solver]
    return agent


# ============================================================================
# Eval Fixtures
# ============================================================================

@pytest.fixture
def eval_cases_path() -> str:
    """Provide path to eval cases JSON file.

    Returns:
        Absolute path to taskmanager_evals.json
    """
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_dir, "evals", "taskmanager_evals.json")


@pytest.fixture
def sample_eval_cases() -> List[Dict[str, Any]]:
    """Provide sample eval cases for testing evaluator.

    Returns:
        List of eval case dictionaries.
    """
    return [
        {
            "query": "Add a task called 'Test task'",
            "expected_tool_use": ["add_task"],
            "reference": "Task 'Test task' added successfully"
        },
        {
            "query": "Show me all my tasks",
            "expected_tool_use": ["list_tasks"],
            "reference": "Here are your tasks"
        }
    ]


# ============================================================================
# Pytest Markers
# ============================================================================

def pytest_configure(config):
    """Register custom pytest markers."""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (requires API key)"
    )
    config.addinivalue_line(
        "markers", "integration: marks tests as integration tests"
    )
    config.addinivalue_line(
        "markers", "unit: marks tests as unit tests"
    )


# ============================================================================
# Auto-skip for missing dependencies
# ============================================================================

@pytest.fixture(autouse=True)
def skip_if_no_adk():
    """Auto-skip tests if google-adk is not installed (for real tests)."""
    # This is handled per-fixture above
    pass
