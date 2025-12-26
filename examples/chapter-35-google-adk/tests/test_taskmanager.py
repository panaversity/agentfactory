"""
TaskManager Agent Tests

This module tests the TaskManager agent functionality:
- Task creation (add_task)
- Task listing (list_tasks)
- Task completion (complete_task)
- Task deletion (delete_task)
- Task search (search_tasks)
- Edge cases (empty list, not found, duplicates)

Run with:
    pytest tests/test_taskmanager.py -v

Expected output:
    tests/test_taskmanager.py::TestAddTask::test_add_task_creates_task PASSED
    tests/test_taskmanager.py::TestAddTask::test_add_task_with_description PASSED
    tests/test_taskmanager.py::TestListTasks::test_list_tasks_returns_tasks PASSED
    ...
"""

import pytest
from typing import List, Dict, Any, Optional
from unittest.mock import Mock, patch


# ============================================================================
# Tool Function Implementations (for testing)
# ============================================================================

class TaskManager:
    """TaskManager class for testing tool functions.

    This class encapsulates the task management logic that would
    typically be exposed as individual tool functions to an ADK agent.
    """

    def __init__(self):
        self.tasks: List[Dict[str, Any]] = []
        self._next_id = 1

    def add_task(
        self,
        title: str,
        description: Optional[str] = None
    ) -> Dict[str, Any]:
        """Add a new task.

        Args:
            title: The task title.
            description: Optional task description.

        Returns:
            Dict with status and created task.
        """
        task = {
            "id": self._next_id,
            "title": title,
            "description": description,
            "completed": False
        }
        self.tasks.append(task)
        self._next_id += 1
        return {"status": "created", "task": task}

    def list_tasks(self) -> Dict[str, Any]:
        """List all tasks.

        Returns:
            Dict with list of all tasks.
        """
        return {"tasks": self.tasks}

    def complete_task(self, task_id: int) -> Dict[str, Any]:
        """Mark a task as completed.

        Args:
            task_id: The ID of the task to complete.

        Returns:
            Dict with status and updated task, or error if not found.
        """
        for task in self.tasks:
            if task["id"] == task_id:
                task["completed"] = True
                return {"status": "completed", "task": task}
        return {"status": "error", "message": f"Task {task_id} not found"}

    def delete_task(self, task_id: int) -> Dict[str, Any]:
        """Delete a task.

        Args:
            task_id: The ID of the task to delete.

        Returns:
            Dict with status and deleted task_id, or error if not found.
        """
        original_len = len(self.tasks)
        self.tasks = [t for t in self.tasks if t["id"] != task_id]
        if len(self.tasks) < original_len:
            return {"status": "deleted", "task_id": task_id}
        return {"status": "error", "message": f"Task {task_id} not found"}

    def search_tasks(self, query: str) -> Dict[str, Any]:
        """Search tasks by title or description.

        Args:
            query: Search query string.

        Returns:
            Dict with matching tasks and count.
        """
        matches = [
            t for t in self.tasks
            if query.lower() in t["title"].lower() or
               (t["description"] and query.lower() in t["description"].lower())
        ]
        return {"matches": matches, "count": len(matches)}


# ============================================================================
# Unit Tests: Add Task
# ============================================================================

class TestAddTask:
    """Tests for the add_task functionality."""

    def test_add_task_creates_task(self):
        """Test that add_task creates a new task with correct fields.

        Expected:
            - Task is added to the list
            - Task has correct id, title, completed=False
            - Returns status "created"
        """
        manager = TaskManager()

        result = manager.add_task(title="Buy groceries")

        assert result["status"] == "created"
        assert result["task"]["title"] == "Buy groceries"
        assert result["task"]["id"] == 1
        assert result["task"]["completed"] is False
        assert len(manager.tasks) == 1

    def test_add_task_with_description(self):
        """Test that add_task includes optional description.

        Expected:
            - Task includes description field
            - Description matches input
        """
        manager = TaskManager()

        result = manager.add_task(
            title="Review PR",
            description="Check the new authentication feature"
        )

        assert result["task"]["description"] == "Check the new authentication feature"

    def test_add_task_increments_id(self):
        """Test that task IDs increment correctly.

        Expected:
            - First task has id=1
            - Second task has id=2
            - IDs are unique
        """
        manager = TaskManager()

        result1 = manager.add_task(title="Task 1")
        result2 = manager.add_task(title="Task 2")

        assert result1["task"]["id"] == 1
        assert result2["task"]["id"] == 2
        assert len(manager.tasks) == 2

    def test_add_task_without_description_has_none(self):
        """Test that task without description has None value.

        Expected:
            - Description field exists
            - Description is None
        """
        manager = TaskManager()

        result = manager.add_task(title="Simple task")

        assert result["task"]["description"] is None

    def test_add_multiple_tasks(self):
        """Test adding multiple tasks in sequence.

        Expected:
            - All tasks are stored
            - Each task has unique ID
        """
        manager = TaskManager()

        for i in range(5):
            manager.add_task(title=f"Task {i + 1}")

        assert len(manager.tasks) == 5
        ids = [t["id"] for t in manager.tasks]
        assert len(set(ids)) == 5  # All unique IDs


# ============================================================================
# Unit Tests: List Tasks
# ============================================================================

class TestListTasks:
    """Tests for the list_tasks functionality."""

    def test_list_tasks_returns_tasks(self, sample_tasks):
        """Test that list_tasks returns all tasks.

        Expected:
            - Returns dict with "tasks" key
            - Contains all added tasks
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.list_tasks()

        assert "tasks" in result
        assert len(result["tasks"]) == 3

    def test_list_tasks_empty_returns_empty_list(self, empty_tasks):
        """Test that list_tasks with no tasks returns empty list.

        Expected:
            - Returns dict with "tasks" key
            - Tasks list is empty
        """
        manager = TaskManager()
        manager.tasks = empty_tasks

        result = manager.list_tasks()

        assert "tasks" in result
        assert result["tasks"] == []

    def test_list_tasks_includes_all_fields(self, sample_tasks):
        """Test that listed tasks include all required fields.

        Expected:
            - Each task has id, title, description, completed
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.list_tasks()

        for task in result["tasks"]:
            assert "id" in task
            assert "title" in task
            assert "description" in task
            assert "completed" in task

    def test_list_tasks_preserves_order(self):
        """Test that list_tasks preserves insertion order.

        Expected:
            - Tasks are returned in order of creation
        """
        manager = TaskManager()
        manager.add_task(title="First")
        manager.add_task(title="Second")
        manager.add_task(title="Third")

        result = manager.list_tasks()

        assert result["tasks"][0]["title"] == "First"
        assert result["tasks"][1]["title"] == "Second"
        assert result["tasks"][2]["title"] == "Third"


# ============================================================================
# Unit Tests: Complete Task
# ============================================================================

class TestCompleteTask:
    """Tests for the complete_task functionality."""

    def test_complete_task_changes_status(self, sample_tasks):
        """Test that complete_task marks task as completed.

        Expected:
            - Task completed field becomes True
            - Returns status "completed"
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.complete_task(task_id=1)

        assert result["status"] == "completed"
        assert result["task"]["completed"] is True

    def test_complete_task_not_found_returns_error(self, sample_tasks):
        """Test that completing non-existent task returns error.

        Expected:
            - Returns status "error"
            - Includes "not found" message
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.complete_task(task_id=999)

        assert result["status"] == "error"
        assert "not found" in result["message"]

    def test_complete_task_empty_list(self, empty_tasks):
        """Test completing task when list is empty.

        Expected:
            - Returns error status
        """
        manager = TaskManager()
        manager.tasks = empty_tasks

        result = manager.complete_task(task_id=1)

        assert result["status"] == "error"

    def test_complete_already_completed_task(self, sample_tasks):
        """Test completing an already completed task.

        Expected:
            - Still returns success (idempotent)
            - Task remains completed
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        # Task 3 is already completed in sample_tasks
        result = manager.complete_task(task_id=3)

        assert result["status"] == "completed"
        assert result["task"]["completed"] is True

    def test_complete_task_returns_updated_task(self, sample_tasks):
        """Test that complete_task returns the updated task object.

        Expected:
            - Returned task has completed=True
            - Other fields unchanged
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.complete_task(task_id=1)

        assert result["task"]["id"] == 1
        assert result["task"]["title"] == "Write documentation"
        assert result["task"]["completed"] is True


# ============================================================================
# Unit Tests: Delete Task
# ============================================================================

class TestDeleteTask:
    """Tests for the delete_task functionality."""

    def test_delete_task_removes_task(self, sample_tasks):
        """Test that delete_task removes the task.

        Expected:
            - Task is removed from list
            - List length decreases by 1
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()
        original_len = len(manager.tasks)

        result = manager.delete_task(task_id=1)

        assert result["status"] == "deleted"
        assert len(manager.tasks) == original_len - 1
        assert not any(t["id"] == 1 for t in manager.tasks)

    def test_delete_task_not_found_returns_error(self, sample_tasks):
        """Test that deleting non-existent task returns error.

        Expected:
            - Returns status "error"
            - Includes "not found" message
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.delete_task(task_id=999)

        assert result["status"] == "error"
        assert "not found" in result["message"]

    def test_delete_task_empty_list(self, empty_tasks):
        """Test deleting task when list is empty.

        Expected:
            - Returns error status
        """
        manager = TaskManager()
        manager.tasks = empty_tasks

        result = manager.delete_task(task_id=1)

        assert result["status"] == "error"

    def test_delete_task_returns_deleted_id(self, sample_tasks):
        """Test that delete_task returns the deleted task ID.

        Expected:
            - Result includes task_id of deleted task
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.delete_task(task_id=2)

        assert result["task_id"] == 2

    def test_delete_multiple_tasks(self, sample_tasks):
        """Test deleting multiple tasks in sequence.

        Expected:
            - Each deletion succeeds
            - Final list has correct length
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        manager.delete_task(task_id=1)
        manager.delete_task(task_id=2)
        manager.delete_task(task_id=3)

        assert len(manager.tasks) == 0


# ============================================================================
# Unit Tests: Search Tasks
# ============================================================================

class TestSearchTasks:
    """Tests for the search_tasks functionality."""

    def test_search_tasks_finds_matches(self, sample_tasks):
        """Test that search_tasks finds matching tasks.

        Expected:
            - Returns matching tasks
            - Count reflects number of matches
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.search_tasks(query="documentation")

        assert result["count"] == 1
        assert result["matches"][0]["title"] == "Write documentation"

    def test_search_tasks_case_insensitive(self, sample_tasks):
        """Test that search is case-insensitive.

        Expected:
            - Finds matches regardless of case
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.search_tasks(query="BUG")

        assert result["count"] == 1
        assert "bug" in result["matches"][0]["title"].lower()

    def test_search_tasks_no_matches(self, sample_tasks):
        """Test search with no matching results.

        Expected:
            - Returns empty matches list
            - Count is 0
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.search_tasks(query="nonexistent")

        assert result["count"] == 0
        assert result["matches"] == []

    def test_search_tasks_matches_description(self, sample_tasks):
        """Test that search matches in description field.

        Expected:
            - Finds tasks with matching description
        """
        manager = TaskManager()
        manager.tasks = sample_tasks.copy()

        result = manager.search_tasks(query="API")

        assert result["count"] == 1
        assert "API" in result["matches"][0]["description"]

    def test_search_tasks_empty_list(self, empty_tasks):
        """Test search on empty task list.

        Expected:
            - Returns empty matches
            - Count is 0
        """
        manager = TaskManager()
        manager.tasks = empty_tasks

        result = manager.search_tasks(query="anything")

        assert result["count"] == 0
        assert result["matches"] == []

    def test_search_tasks_multiple_matches(self):
        """Test search returning multiple matches.

        Expected:
            - All matching tasks returned
            - Count reflects total matches
        """
        manager = TaskManager()
        manager.add_task(title="Write tests")
        manager.add_task(title="Write documentation")
        manager.add_task(title="Review code")

        result = manager.search_tasks(query="Write")

        assert result["count"] == 2

    def test_search_tasks_partial_match(self):
        """Test that partial string matches work.

        Expected:
            - Finds tasks with partial matches
        """
        manager = TaskManager()
        manager.add_task(title="Documentation update")

        result = manager.search_tasks(query="doc")

        assert result["count"] == 1


# ============================================================================
# Edge Cases
# ============================================================================

class TestEdgeCases:
    """Edge case tests for TaskManager."""

    def test_duplicate_titles_allowed(self):
        """Test that tasks with same title are allowed.

        Expected:
            - Both tasks are created
            - Each has unique ID
        """
        manager = TaskManager()

        result1 = manager.add_task(title="Same title")
        result2 = manager.add_task(title="Same title")

        assert len(manager.tasks) == 2
        assert result1["task"]["id"] != result2["task"]["id"]

    def test_empty_title(self):
        """Test creating task with empty title.

        Expected:
            - Task is created (no validation in basic implementation)
        """
        manager = TaskManager()

        result = manager.add_task(title="")

        assert result["status"] == "created"
        assert result["task"]["title"] == ""

    def test_very_long_title(self):
        """Test creating task with very long title.

        Expected:
            - Task is created without truncation
        """
        manager = TaskManager()
        long_title = "A" * 1000

        result = manager.add_task(title=long_title)

        assert result["task"]["title"] == long_title

    def test_special_characters_in_title(self):
        """Test task with special characters.

        Expected:
            - Special characters preserved
        """
        manager = TaskManager()
        special_title = "Task with @#$%^&*() special chars!"

        result = manager.add_task(title=special_title)

        assert result["task"]["title"] == special_title

    def test_unicode_in_title(self):
        """Test task with unicode characters.

        Expected:
            - Unicode preserved
        """
        manager = TaskManager()
        unicode_title = "Task with emoji 🎯 and 日本語"

        result = manager.add_task(title=unicode_title)

        assert result["task"]["title"] == unicode_title

    def test_search_special_characters(self):
        """Test searching with special characters.

        Expected:
            - Search works with special chars
        """
        manager = TaskManager()
        manager.add_task(title="Fix bug #123")

        result = manager.search_tasks(query="#123")

        assert result["count"] == 1


# ============================================================================
# Integration Tests (Requires API Key)
# ============================================================================

@pytest.mark.slow
class TestTaskManagerIntegration:
    """Integration tests that make real API calls.

    These tests require GOOGLE_API_KEY to be set.
    Run with: pytest tests/test_taskmanager.py -v -m slow
    """

    @pytest.mark.asyncio
    async def test_real_agent_add_task(self, real_runner, real_session_service):
        """Test real agent adding a task.

        Expected output:
            Agent creates task and confirms
        """
        if real_runner is None:
            pytest.skip("Real runner not available")

        try:
            from google.genai import types

            session = await real_session_service.create_session(
                app_name="test_taskmanager",
                user_id="test_user"
            )

            message = types.Content(
                role='user',
                parts=[types.Part(text="Add a task: Buy groceries")]
            )

            responses = []
            async for event in real_runner.run_async(
                user_id=session.user_id,
                session_id=session.id,
                new_message=message
            ):
                if event.content and event.content.parts:
                    for part in event.content.parts:
                        if part.text:
                            responses.append(part.text)

            # Verify agent responded
            assert len(responses) > 0
            combined_response = " ".join(responses).lower()
            # Agent should confirm task creation
            assert any(word in combined_response for word in
                      ["added", "created", "task", "groceries"])

        except ImportError:
            pytest.skip("google-adk not installed")

    @pytest.mark.asyncio
    async def test_real_agent_list_empty_tasks(
        self,
        real_runner,
        real_session_service
    ):
        """Test real agent listing empty task list.

        Expected output:
            Agent reports no tasks or empty list
        """
        if real_runner is None:
            pytest.skip("Real runner not available")

        try:
            from google.genai import types

            session = await real_session_service.create_session(
                app_name="test_taskmanager",
                user_id="test_user_empty"
            )

            message = types.Content(
                role='user',
                parts=[types.Part(text="Show me all my tasks")]
            )

            responses = []
            async for event in real_runner.run_async(
                user_id=session.user_id,
                session_id=session.id,
                new_message=message
            ):
                if event.content and event.content.parts:
                    for part in event.content.parts:
                        if part.text:
                            responses.append(part.text)

            assert len(responses) > 0

        except ImportError:
            pytest.skip("google-adk not installed")
