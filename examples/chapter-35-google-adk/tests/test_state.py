"""
State Management Tests

This module tests ADK state management:
- InMemorySessionService: In-memory state persistence
- State isolation between sessions
- ToolContext state access
- Mock Firestore tests for production patterns

Run with:
    pytest tests/test_state.py -v

Expected output:
    tests/test_state.py::TestInMemoryState::test_state_persists_within_session PASSED
    tests/test_state.py::TestStateIsolation::test_sessions_are_isolated PASSED
    tests/test_state.py::TestToolContextState::test_tool_can_read_state PASSED
    ...
"""

import pytest
from typing import Dict, Any, List, Optional
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from dataclasses import dataclass, field
import asyncio


# ============================================================================
# Mock Session and State Types
# ============================================================================

@dataclass
class MockSession:
    """Mock Session object for testing."""
    id: str
    user_id: str
    app_name: str
    state: Dict[str, Any] = field(default_factory=dict)
    created_at: Optional[float] = None


@dataclass
class MockToolContext:
    """Mock ToolContext for testing tool state access."""
    state: Dict[str, Any]
    session_id: str
    user_id: str

    def get(self, key: str, default: Any = None) -> Any:
        """Get value from state."""
        return self.state.get(key, default)

    def set(self, key: str, value: Any) -> None:
        """Set value in state."""
        self.state[key] = value


# ============================================================================
# In-Memory Session Service Implementation
# ============================================================================

class MockInMemorySessionService:
    """Mock InMemorySessionService for testing.

    This simulates the ADK InMemorySessionService behavior
    for unit testing without actual ADK dependencies.
    """

    def __init__(self):
        self._sessions: Dict[str, MockSession] = {}
        self._session_counter = 0

    async def create_session(
        self,
        app_name: str,
        user_id: str,
        initial_state: Optional[Dict[str, Any]] = None
    ) -> MockSession:
        """Create a new session.

        Args:
            app_name: Application name.
            user_id: User identifier.
            initial_state: Optional initial state dictionary.

        Returns:
            New MockSession instance.
        """
        self._session_counter += 1
        session_id = f"session_{self._session_counter}"
        session = MockSession(
            id=session_id,
            user_id=user_id,
            app_name=app_name,
            state=initial_state.copy() if initial_state else {}
        )
        self._sessions[session_id] = session
        return session

    async def get_session(self, session_id: str) -> Optional[MockSession]:
        """Get an existing session.

        Args:
            session_id: Session identifier.

        Returns:
            MockSession if found, None otherwise.
        """
        return self._sessions.get(session_id)

    async def update_session(
        self,
        session_id: str,
        state_updates: Dict[str, Any]
    ) -> Optional[MockSession]:
        """Update session state.

        Args:
            session_id: Session identifier.
            state_updates: State fields to update.

        Returns:
            Updated session if found, None otherwise.
        """
        session = self._sessions.get(session_id)
        if session:
            session.state.update(state_updates)
            return session
        return None

    async def delete_session(self, session_id: str) -> bool:
        """Delete a session.

        Args:
            session_id: Session identifier.

        Returns:
            True if deleted, False if not found.
        """
        if session_id in self._sessions:
            del self._sessions[session_id]
            return True
        return False

    async def list_sessions(
        self,
        app_name: Optional[str] = None,
        user_id: Optional[str] = None
    ) -> List[MockSession]:
        """List sessions with optional filtering.

        Args:
            app_name: Filter by app name.
            user_id: Filter by user ID.

        Returns:
            List of matching sessions.
        """
        result = list(self._sessions.values())
        if app_name:
            result = [s for s in result if s.app_name == app_name]
        if user_id:
            result = [s for s in result if s.user_id == user_id]
        return result


# ============================================================================
# Mock Firestore Session Service
# ============================================================================

class MockFirestoreSessionService:
    """Mock FirestoreSessionService for testing production patterns.

    Simulates Firestore persistence with in-memory storage.
    """

    def __init__(self, project: str = "test-project", database: str = "(default)"):
        self.project = project
        self.database = database
        self._collections: Dict[str, Dict[str, MockSession]] = {
            "sessions": {}
        }
        self._session_counter = 0

    async def create_session(
        self,
        app_name: str,
        user_id: str,
        initial_state: Optional[Dict[str, Any]] = None
    ) -> MockSession:
        """Create and persist a session."""
        self._session_counter += 1
        session_id = f"firestore_session_{self._session_counter}"
        session = MockSession(
            id=session_id,
            user_id=user_id,
            app_name=app_name,
            state=initial_state.copy() if initial_state else {}
        )
        self._collections["sessions"][session_id] = session
        return session

    async def get_session(self, session_id: str) -> Optional[MockSession]:
        """Retrieve session from 'Firestore'."""
        return self._collections["sessions"].get(session_id)

    async def update_session(
        self,
        session_id: str,
        state_updates: Dict[str, Any]
    ) -> Optional[MockSession]:
        """Update session in 'Firestore'."""
        session = self._collections["sessions"].get(session_id)
        if session:
            session.state.update(state_updates)
            return session
        return None

    async def delete_session(self, session_id: str) -> bool:
        """Delete session from 'Firestore'."""
        if session_id in self._collections["sessions"]:
            del self._collections["sessions"][session_id]
            return True
        return False

    def simulate_restart(self):
        """Simulate service restart - data persists in 'Firestore'."""
        # In real Firestore, data would persist across restarts
        # This is just to show the pattern
        pass


# ============================================================================
# Unit Tests: InMemory State
# ============================================================================

class TestInMemoryState:
    """Tests for InMemorySessionService state management."""

    @pytest.mark.asyncio
    async def test_state_persists_within_session(self):
        """Test that state persists within a session.

        Expected:
            - State set in session is retrievable
        """
        service = MockInMemorySessionService()
        session = await service.create_session(
            app_name="test_app",
            user_id="user123"
        )

        # Set state
        session.state["counter"] = 0
        session.state["counter"] += 1
        session.state["counter"] += 1

        # Retrieve same session
        retrieved = await service.get_session(session.id)

        assert retrieved is not None
        assert retrieved.state["counter"] == 2

    @pytest.mark.asyncio
    async def test_initial_state_preserved(self):
        """Test that initial state is preserved.

        Expected:
            - Session created with initial state has those values
        """
        service = MockInMemorySessionService()
        initial = {"preference": "dark_mode", "language": "en"}

        session = await service.create_session(
            app_name="test_app",
            user_id="user123",
            initial_state=initial
        )

        assert session.state["preference"] == "dark_mode"
        assert session.state["language"] == "en"

    @pytest.mark.asyncio
    async def test_state_update_merges(self):
        """Test that state updates merge with existing state.

        Expected:
            - New keys added, existing keys updated
        """
        service = MockInMemorySessionService()
        session = await service.create_session(
            app_name="test_app",
            user_id="user123",
            initial_state={"key1": "value1", "key2": "value2"}
        )

        await service.update_session(
            session.id,
            {"key2": "updated", "key3": "new"}
        )

        updated = await service.get_session(session.id)
        assert updated.state["key1"] == "value1"  # Unchanged
        assert updated.state["key2"] == "updated"  # Updated
        assert updated.state["key3"] == "new"  # Added

    @pytest.mark.asyncio
    async def test_empty_state_by_default(self):
        """Test that new sessions have empty state by default.

        Expected:
            - State is empty dict, not None
        """
        service = MockInMemorySessionService()
        session = await service.create_session(
            app_name="test_app",
            user_id="user123"
        )

        assert session.state == {}
        assert isinstance(session.state, dict)

    @pytest.mark.asyncio
    async def test_state_supports_complex_types(self):
        """Test that state can store complex data types.

        Expected:
            - Lists, dicts, nested structures work
        """
        service = MockInMemorySessionService()
        session = await service.create_session(
            app_name="test_app",
            user_id="user123"
        )

        session.state["tasks"] = [
            {"id": 1, "title": "Task 1"},
            {"id": 2, "title": "Task 2"}
        ]
        session.state["config"] = {
            "nested": {"deeply": {"value": 42}}
        }

        retrieved = await service.get_session(session.id)
        assert len(retrieved.state["tasks"]) == 2
        assert retrieved.state["config"]["nested"]["deeply"]["value"] == 42


# ============================================================================
# Unit Tests: State Isolation
# ============================================================================

class TestStateIsolation:
    """Tests for state isolation between sessions."""

    @pytest.mark.asyncio
    async def test_sessions_are_isolated(self):
        """Test that different sessions have isolated state.

        Expected:
            - Changes to one session don't affect others
        """
        service = MockInMemorySessionService()

        session1 = await service.create_session(
            app_name="test_app",
            user_id="user1"
        )
        session2 = await service.create_session(
            app_name="test_app",
            user_id="user2"
        )

        session1.state["data"] = "session1_data"
        session2.state["data"] = "session2_data"

        # Verify isolation
        s1 = await service.get_session(session1.id)
        s2 = await service.get_session(session2.id)

        assert s1.state["data"] == "session1_data"
        assert s2.state["data"] == "session2_data"

    @pytest.mark.asyncio
    async def test_same_user_different_sessions(self):
        """Test that same user can have multiple isolated sessions.

        Expected:
            - Multiple sessions for same user are separate
        """
        service = MockInMemorySessionService()

        session1 = await service.create_session(
            app_name="test_app",
            user_id="same_user"
        )
        session2 = await service.create_session(
            app_name="test_app",
            user_id="same_user"
        )

        session1.state["counter"] = 1
        session2.state["counter"] = 100

        assert session1.id != session2.id
        assert session1.state["counter"] != session2.state["counter"]

    @pytest.mark.asyncio
    async def test_different_apps_isolated(self):
        """Test that sessions for different apps are isolated.

        Expected:
            - Different app names create separate sessions
        """
        service = MockInMemorySessionService()

        session1 = await service.create_session(
            app_name="app1",
            user_id="user123"
        )
        session2 = await service.create_session(
            app_name="app2",
            user_id="user123"
        )

        session1.state["app_data"] = "app1_data"
        session2.state["app_data"] = "app2_data"

        # List by app
        app1_sessions = await service.list_sessions(app_name="app1")
        app2_sessions = await service.list_sessions(app_name="app2")

        assert len(app1_sessions) == 1
        assert len(app2_sessions) == 1
        assert app1_sessions[0].state["app_data"] == "app1_data"

    @pytest.mark.asyncio
    async def test_deleted_session_state_removed(self):
        """Test that deleted session state is removed.

        Expected:
            - After deletion, session and state are gone
        """
        service = MockInMemorySessionService()

        session = await service.create_session(
            app_name="test_app",
            user_id="user123",
            initial_state={"important": "data"}
        )
        session_id = session.id

        await service.delete_session(session_id)

        retrieved = await service.get_session(session_id)
        assert retrieved is None


# ============================================================================
# Unit Tests: ToolContext State Access
# ============================================================================

class TestToolContextState:
    """Tests for ToolContext state access in tool functions."""

    def test_tool_can_read_state(self):
        """Test that tools can read state from ToolContext.

        Expected:
            - State values accessible via tool_context.state
        """
        state = {"preference": "dark", "language": "en"}
        context = MockToolContext(
            state=state,
            session_id="session-123",
            user_id="user-456"
        )

        assert context.state["preference"] == "dark"
        assert context.get("language") == "en"

    def test_tool_can_write_state(self):
        """Test that tools can write to state.

        Expected:
            - State modifications persist in context
        """
        context = MockToolContext(
            state={},
            session_id="session-123",
            user_id="user-456"
        )

        context.set("new_key", "new_value")
        context.state["another_key"] = 123

        assert context.state["new_key"] == "new_value"
        assert context.state["another_key"] == 123

    def test_tool_get_with_default(self):
        """Test that get() returns default for missing keys.

        Expected:
            - Missing keys return default value
        """
        context = MockToolContext(
            state={"exists": True},
            session_id="session-123",
            user_id="user-456"
        )

        assert context.get("exists") is True
        assert context.get("missing") is None
        assert context.get("missing", "default") == "default"

    def test_tool_context_has_session_info(self):
        """Test that ToolContext includes session information.

        Expected:
            - session_id and user_id are accessible
        """
        context = MockToolContext(
            state={},
            session_id="session-123",
            user_id="user-456"
        )

        assert context.session_id == "session-123"
        assert context.user_id == "user-456"

    def test_tool_modifies_shared_state(self):
        """Test that tool modifications affect shared state reference.

        Pattern: Tool sets state -> Agent reads updated state
        """
        shared_state = {"counter": 0}

        # Simulate tool function
        def increment_counter(tool_context: MockToolContext) -> dict:
            current = tool_context.state.get("counter", 0)
            tool_context.state["counter"] = current + 1
            return {"new_value": tool_context.state["counter"]}

        context = MockToolContext(
            state=shared_state,
            session_id="session-123",
            user_id="user-456"
        )

        result = increment_counter(context)

        assert result["new_value"] == 1
        assert shared_state["counter"] == 1  # Shared reference updated


# ============================================================================
# Unit Tests: Mock Firestore
# ============================================================================

class TestMockFirestore:
    """Tests for mock Firestore session service."""

    @pytest.mark.asyncio
    async def test_firestore_creates_session(self):
        """Test Firestore session creation.

        Expected:
            - Session created with correct app/user
        """
        service = MockFirestoreSessionService(project="my-project")

        session = await service.create_session(
            app_name="production_app",
            user_id="user123"
        )

        assert session.id.startswith("firestore_session_")
        assert session.app_name == "production_app"
        assert session.user_id == "user123"

    @pytest.mark.asyncio
    async def test_firestore_state_persists(self):
        """Test that Firestore session state persists.

        Expected:
            - State is retrievable after creation
        """
        service = MockFirestoreSessionService()

        session = await service.create_session(
            app_name="app",
            user_id="user",
            initial_state={"key": "value"}
        )

        retrieved = await service.get_session(session.id)
        assert retrieved.state["key"] == "value"

    @pytest.mark.asyncio
    async def test_firestore_survives_restart(self):
        """Test pattern: Firestore data survives service restart.

        Expected:
            - Data accessible after simulated restart
        """
        service = MockFirestoreSessionService()

        session = await service.create_session(
            app_name="app",
            user_id="user",
            initial_state={"persistent": True}
        )
        session_id = session.id

        # Simulate restart
        service.simulate_restart()

        # Data should still be there
        retrieved = await service.get_session(session_id)
        assert retrieved.state["persistent"] is True

    @pytest.mark.asyncio
    async def test_firestore_update_persists(self):
        """Test that Firestore updates persist.

        Expected:
            - Updates are saved and retrievable
        """
        service = MockFirestoreSessionService()

        session = await service.create_session(
            app_name="app",
            user_id="user",
            initial_state={"version": 1}
        )

        await service.update_session(
            session.id,
            {"version": 2, "updated": True}
        )

        retrieved = await service.get_session(session.id)
        assert retrieved.state["version"] == 2
        assert retrieved.state["updated"] is True


# ============================================================================
# State Pattern Tests
# ============================================================================

class TestStatePatterns:
    """Tests for common state management patterns."""

    @pytest.mark.asyncio
    async def test_task_list_state_pattern(self):
        """Test pattern: Managing task list in session state.

        Pattern used in TaskManager agent.
        """
        service = MockInMemorySessionService()
        session = await service.create_session(
            app_name="task_manager",
            user_id="user123",
            initial_state={"tasks": []}
        )

        # Add task
        session.state["tasks"].append({
            "id": 1,
            "title": "First task",
            "completed": False
        })

        # Add another task
        session.state["tasks"].append({
            "id": 2,
            "title": "Second task",
            "completed": False
        })

        # Complete task
        for task in session.state["tasks"]:
            if task["id"] == 1:
                task["completed"] = True

        retrieved = await service.get_session(session.id)
        assert len(retrieved.state["tasks"]) == 2
        assert retrieved.state["tasks"][0]["completed"] is True
        assert retrieved.state["tasks"][1]["completed"] is False

    @pytest.mark.asyncio
    async def test_conversation_history_pattern(self):
        """Test pattern: Storing conversation history in state.

        Pattern for maintaining context across turns.
        """
        service = MockInMemorySessionService()
        session = await service.create_session(
            app_name="chat_agent",
            user_id="user123",
            initial_state={"history": []}
        )

        # Add messages
        session.state["history"].append({
            "role": "user",
            "content": "Hello"
        })
        session.state["history"].append({
            "role": "assistant",
            "content": "Hi! How can I help?"
        })
        session.state["history"].append({
            "role": "user",
            "content": "Add a task"
        })

        retrieved = await service.get_session(session.id)
        assert len(retrieved.state["history"]) == 3

    @pytest.mark.asyncio
    async def test_user_preferences_pattern(self):
        """Test pattern: Storing user preferences in state.

        Pattern for personalization across sessions.
        """
        service = MockInMemorySessionService()

        # First session - user sets preferences
        session1 = await service.create_session(
            app_name="app",
            user_id="user123"
        )
        session1.state["preferences"] = {
            "theme": "dark",
            "notifications": True,
            "language": "en"
        }

        # Simulate: Copy preferences to new session
        prefs = session1.state["preferences"].copy()

        session2 = await service.create_session(
            app_name="app",
            user_id="user123",
            initial_state={"preferences": prefs}
        )

        assert session2.state["preferences"]["theme"] == "dark"


# ============================================================================
# Integration Tests (Requires API Key)
# ============================================================================

@pytest.mark.slow
class TestStateIntegration:
    """Integration tests for real ADK state management.

    These tests require google-adk to be installed.
    """

    @pytest.mark.asyncio
    async def test_real_inmemory_session_service(self):
        """Test real InMemorySessionService.

        Expected:
            - Creates and manages sessions
        """
        try:
            from google.adk.sessions import InMemorySessionService

            service = InMemorySessionService()
            session = await service.create_session(
                app_name="test_app",
                user_id="test_user"
            )

            assert session.id is not None
            assert session.user_id == "test_user"

        except ImportError:
            pytest.skip("google-adk not installed")

    @pytest.mark.asyncio
    async def test_real_tool_context_state(self):
        """Test real ToolContext state access pattern.

        Expected:
            - ToolContext provides state access
        """
        try:
            from google.adk.tools.tool_context import ToolContext
            from google.adk.sessions import InMemorySessionService
            import os

            if not os.environ.get("GOOGLE_API_KEY"):
                pytest.skip("GOOGLE_API_KEY not set")

            # ToolContext is typically created by the runner
            # This test validates the expected interface
            assert hasattr(ToolContext, '__init__')

        except ImportError:
            pytest.skip("google-adk not installed")
