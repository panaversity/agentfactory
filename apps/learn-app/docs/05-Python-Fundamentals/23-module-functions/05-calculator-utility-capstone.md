---
sidebar_position: 5
title: "Building a Todo Console App (Capstone Project)"
description: "Integrate modules, functions, parameters, returns, and scope into a real multi-module todo management application"
keywords: ["capstone", "modules", "multi-file", "separation of concerns", "testing", "project organization", "todo app", "task management"]
chapter: 23
lesson: 5
duration_minutes: 90
proficiency_level: "B1-B2"
estimated_time: "90 minutes"
proficiency_by_end: "Students complete a working multi-module todo console app with clear separation of concerns and persistent task storage preview"
prerequisites: ["Lessons 1-4: All module and function concepts", "Chapter 19-22: Data types, lists, operations, and dictionaries"]

# HIDDEN SKILLS METADATA
skills:
  - name: "Multi-Module Project Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student organizes functions across task_operations.py, utils.py, and main.py with clear responsibility boundaries"

  - name: "Function-Based Todo Operations"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student implements add_task, complete_task, list_tasks, filter_tasks with proper type hints and docstrings"

  - name: "Separation of Concerns Principle"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Understand"
    digcomp_area: "Software Design"
    measurable_at_this_level: "Student explains why task operations, I/O, and orchestration belong in separate modules for testability and reusability"

  - name: "Data Structure Design (Task Dictionary)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Data Management"
    measurable_at_this_level: "Student creates and manipulates task dictionaries with canonical fields (id, title, priority, status, done)"

  - name: "List Manipulation with Functions"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Data Structures"
    measurable_at_this_level: "Student appends, filters, and iterates through task lists using list methods and comprehensions"

  - name: "Type Hints and Function Contracts"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Code Quality"
    measurable_at_this_level: "Student uses Optional types, Union types, and None returns for operations that may fail"

  - name: "Testing Function Collections"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Quality Assurance"
    measurable_at_this_level: "Student writes test functions covering normal cases, edge cases, and error handling for task operations"

  - name: "Module Import Patterns"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student uses `import module` and `module.function()` patterns to coordinate multiple modules"

learning_objectives:
  - objective: "Organize code into multiple modules with clear responsibilities"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Project structure review—task_operations.py, utils.py, main.py each have focused purpose"

  - objective: "Implement and test todo operations using functions with proper type hints"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code review—functions handle add, complete, list, filter operations with Type | None returns for edge cases"

  - objective: "Design task data structure following canonical Task entity specification"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Data validation—tasks include id, title, priority, status, done fields with correct types"

  - objective: "Test multi-module project comprehensively covering normal, edge, and error cases"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Test suite execution—70%+ code coverage with assertions for all operations"

  - objective: "Import and coordinate custom modules in a main orchestration function"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Main program review—uses `import` statements and `module.function()` pattern throughout"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (task structure, add_task, complete_task, list_tasks, filter_tasks, save/load preview, module imports, testing, main orchestration) appropriate for B1 (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Implement task.json persistence using json.dump/load (preview of Ch25); add due_date filtering; create TaskManager class (preview of Ch27)"
  remedial_for_struggling: "Start with fewer operations (add_task, complete_task, list_tasks only); skip filtering initially; focus on one module at a time"

ai_native_learning: true
try_with_ai_prompts: 4
colearning_elements: ["architecture_prompt", "specification_challenge", "implementation_tip", "extension_opportunity"]
---

# Building a Todo Console App (Capstone Project)

**The Challenge**: You manage dozens of tasks across work, personal projects, and learning. Your todo list is scattered across sticky notes, apps, and your head. You need a single source of truth—but building a full web application feels like overkill.

What if you could build a command-line todo manager that lets you add, complete, and organize tasks from the terminal? No database required. Just clean Python functions that do one thing well. This capstone teaches you exactly that.

By the end, you'll have a working multi-module todo console app that demonstrates professional code organization. And more importantly, you'll understand why this structure matters: **functions that are easy to test, modules that are easy to reuse, and a main program that's easy to understand**.

---

## Capstone Project Overview

**The Task**: Build a multi-module todo application that demonstrates:
- **Separation of concerns**: Task operations live separately from I/O and orchestration
- **Function-based architecture**: Each todo operation (add, complete, list, filter) is a testable function
- **Canonical task structure**: Tasks use a consistent dictionary format across the entire app
- **Type safety**: Functions use type hints and return Optional values for edge cases
- **Multi-module imports**: Main program imports and coordinates specialized modules
- **Comprehensive testing**: Validate that functions work correctly with various inputs

**Project Structure**:
```
todo_app/
├── task_operations.py      # Task manipulation functions
├── utils.py               # I/O and display formatting
├── main.py                # Main program orchestrating the app
└── test_todo.py           # Tests for task operations
```

**The Transition**: You've learned functions in isolation. This capstone shows how functions work together to build something real. You'll experience why good design (modules with clear purposes) makes code easier to test, modify, and extend.

### Understanding Task Structure

Before you code, understand how tasks will be represented. A task is a dictionary with these fields:

```python
task = {
    "id": 1,                          # Unique identifier
    "title": "Review PR",             # What needs to be done
    "description": "Check code quality",  # Optional notes
    "priority": 2,                    # 1 (highest) to 10 (lowest), default 5
    "status": "pending",              # One of: pending, in_progress, completed
    "done": False                      # Quick flag for completion
}
```

**Note**: This matches the canonical Task entity used throughout Part 5. When you move to Chapter 27, you'll wrap this in a `Task` class. For now, dictionaries are simpler and let you focus on function design.

---

## Organizing Task Operations

When you're building a todo app:

> "I'm designing a todo application. Should I put task operations (add, complete, list) in the same module as user interface (display, input)? Or should I separate them?"

**AI Response**: "Separation makes sense here. Task operations are pure functions—they take tasks and return modified tasks. UI is different—it handles printing, input validation, menu display. If you keep them separate, you can:
- Test operations without simulating user input
- Reuse operations in different UIs (web, mobile, terminal)
- Change how the display works without touching task logic"

**Your insight**: "So I'd have task_operations.py for the logic and utils.py for the interface?"

**AI**: "Exactly. Plus main.py orchestrates them both. And test_todo.py validates operations without touching the UI."

This conversation shows real architecture thinking. Don't just follow the structure blindly—understand *why* separation helps.

---

## Step 1: Create `task_operations.py` — Task Manipulation Functions

This module contains pure functions that create, modify, and manage tasks. No printing, no input—just data transformation.

### 💻 Code Idea: Task Operations Module

```python
# File: task_operations.py
"""
Task operations module.

Contains pure functions for todo task management:
- Creating and adding tasks
- Marking tasks complete
- Filtering and listing tasks
- Managing task IDs

All functions use type hints and handle edge cases gracefully.
"""

# Global counter for task IDs (in real apps, would come from database)
_next_id = 1


def _get_next_id() -> int:
    """Get the next task ID (internal helper)."""
    global _next_id
    current_id = _next_id
    _next_id += 1
    return current_id


def create_task(title: str, description: str = "", priority: int = 5) -> dict:
    """
    Create a new task dictionary.

    Parameters:
        title (str): Task title (required)
        description (str): Detailed description (optional)
        priority (int): Priority 1-10, default 5

    Returns:
        dict: Task with id, title, description, priority, status, done
    """
    if not title or not title.strip():
        raise ValueError("Task title cannot be empty")
    if not (1 <= priority <= 10):
        raise ValueError("Priority must be between 1 and 10")

    return {
        "id": _get_next_id(),
        "title": title.strip(),
        "description": description.strip(),
        "priority": priority,
        "status": "pending",
        "done": False
    }


def add_task(tasks: list, title: str, description: str = "", priority: int = 5) -> dict:
    """
    Add a new task to the task list.

    Parameters:
        tasks (list): Existing tasks list
        title (str): Task title
        description (str): Task description (optional)
        priority (int): Priority 1-10 (default 5)

    Returns:
        dict: The newly created task
    """
    new_task = create_task(title, description, priority)
    tasks.append(new_task)
    return new_task


def complete_task(tasks: list, task_id: int) -> bool:
    """
    Mark a task as completed.

    Parameters:
        tasks (list): List of tasks
        task_id (int): ID of task to complete

    Returns:
        bool: True if task found and marked complete, False otherwise
    """
    for task in tasks:
        if task["id"] == task_id:
            task["status"] = "completed"
            task["done"] = True
            return True
    return False


def list_tasks(tasks: list) -> list[dict]:
    """
    Return all tasks (supports further filtering).

    Parameters:
        tasks (list): List of tasks

    Returns:
        list[dict]: All tasks (empty list if none)
    """
    return tasks.copy()


def filter_tasks(tasks: list, done: bool | None = None, priority: int | None = None) -> list[dict]:
    """
    Filter tasks by completion status and/or priority.

    Parameters:
        tasks (list): List of tasks
        done (bool | None): Filter by done status (True/False/None=all)
        priority (int | None): Filter by priority (None=all)

    Returns:
        list[dict]: Filtered tasks
    """
    result = tasks.copy()

    if done is not None:
        result = [t for t in result if t["done"] == done]

    if priority is not None:
        result = [t for t in result if t["priority"] == priority]

    return result


def get_task_by_id(tasks: list, task_id: int) -> dict | None:
    """
    Find a task by ID.

    Parameters:
        tasks (list): List of tasks
        task_id (int): Task ID to find

    Returns:
        dict | None: Task if found, None otherwise
    """
    for task in tasks:
        if task["id"] == task_id:
            return task
    return None


def update_task_priority(tasks: list, task_id: int, new_priority: int) -> bool:
    """
    Update a task's priority.

    Parameters:
        tasks (list): List of tasks
        task_id (int): Task ID
        new_priority (int): New priority (1-10)

    Returns:
        bool: True if task found and updated, False otherwise
    """
    if not (1 <= new_priority <= 10):
        raise ValueError("Priority must be between 1 and 10")

    task = get_task_by_id(tasks, task_id)
    if task:
        task["priority"] = new_priority
        return True
    return False
```

**Output:**
```
# No output yet—these are just function definitions.
# They'll be used by main.py and tested by test_todo.py
```

**Design Choices**:
- All functions are pure (no printing, no input)
- Functions take `tasks: list` as parameter (modifying in place for efficiency)
- Return types are explicit: `bool` for success/failure, `dict | None` for optional results
- Edge cases handled gracefully (task not found → False, validation errors → ValueError)
- Global `_next_id` simulates database auto-incrementing (temporary—Chapter 25 will use JSON files)

---

## Step 2: Create `utils.py` — User Interface and Display

This module handles everything the user sees and inputs. It orchestrates task_operations but doesn't modify tasks directly.

### 💻 Code Idea: Utilities Module

```python
# File: utils.py
"""
Utility functions for todo app.

Handles user interface:
- Menu display
- Input collection
- Output formatting
- Input validation
"""

import task_operations as ops


def display_menu() -> None:
    """Display the main menu."""
    print("\n" + "=" * 40)
    print("         TODO CONSOLE APP")
    print("=" * 40)
    print("1. Add a new task")
    print("2. List all tasks")
    print("3. Mark task complete")
    print("4. List pending tasks")
    print("5. List completed tasks")
    print("6. Filter by priority")
    print("7. Update task priority")
    print("8. Exit")
    print("=" * 40)


def get_menu_choice() -> str:
    """
    Get user's menu choice.

    Returns:
        str: User's choice (1-8)
    """
    choice = input("\nEnter choice (1-8): ").strip()
    return choice


def get_task_input() -> tuple[str, str, int] | None:
    """
    Get task details from user.

    Returns:
        tuple[str, str, int] | None: (title, description, priority) if valid, None if error
    """
    try:
        title = input("Task title: ").strip()
        if not title:
            print("⚠️ Title cannot be empty")
            return None

        description = input("Description (optional): ").strip()

        priority_str = input("Priority (1=highest, 10=lowest, default 5): ").strip()
        priority = int(priority_str) if priority_str else 5

        if not (1 <= priority <= 10):
            print("⚠️ Priority must be between 1 and 10")
            return None

        return (title, description, priority)

    except ValueError:
        print("⚠️ Invalid input. Priority must be a number.")
        return None


def get_task_id() -> int | None:
    """
    Get a task ID from user.

    Returns:
        int | None: Task ID if valid, None otherwise
    """
    try:
        task_id_str = input("Enter task ID: ").strip()
        task_id = int(task_id_str)
        return task_id
    except ValueError:
        print("⚠️ Task ID must be a number.")
        return None


def display_tasks(tasks: list[dict]) -> None:
    """
    Display tasks in formatted table.

    Parameters:
        tasks (list[dict]): Tasks to display
    """
    if not tasks:
        print("📭 No tasks to display.")
        return

    # Header
    print("\n" + "-" * 80)
    print(f"{'ID':<5} {'Title':<25} {'Priority':<10} {'Status':<12} {'Done':<6}")
    print("-" * 80)

    # Rows
    for task in tasks:
        done_icon = "✓" if task["done"] else "✗"
        print(f"{task['id']:<5} {task['title']:<25} {task['priority']:<10} "
              f"{task['status']:<12} {done_icon:<6}")

    print("-" * 80 + "\n")


def display_task_detail(task: dict) -> None:
    """
    Display detailed task information.

    Parameters:
        task (dict): Task to display
    """
    print("\n" + "=" * 40)
    print(f"Task #{task['id']}")
    print("=" * 40)
    print(f"Title:       {task['title']}")
    print(f"Description: {task['description'] or '(none)'}")
    print(f"Priority:    {task['priority']}")
    print(f"Status:      {task['status']}")
    print(f"Done:        {'Yes' if task['done'] else 'No'}")
    print("=" * 40 + "\n")


def display_message(message: str) -> None:
    """
    Display a status message.

    Parameters:
        message (str): Message to display
    """
    print(f"\n✨ {message}")
```

**Output:**
```
# No output yet—these are display functions called by main.py
```

**Design Choices**:
- Input functions return tuples or None (not direct task dictionaries)
- Display functions take already-created data (from task_operations) and format it
- Input validation handles numbers, empty strings, and invalid ranges
- All user-facing feedback is centralized here (easier to change UI style globally)

---

## Step 3: Create `main.py` — Main Program

This file orchestrates the app by importing and coordinating the other modules.

### 💻 Code Idea: Main Program

```python
# File: main.py
"""
Main todo console application.

Imports task_operations and utils modules, orchestrates the program flow.
Demonstrates multi-module integration and separation of concerns.
"""

import task_operations as ops
import utils


def run_todo_app() -> None:
    """Run the todo application in a loop until user exits."""
    tasks: list = []

    while True:
        utils.display_menu()
        choice = utils.get_menu_choice()

        if choice == "1":
            # Add a new task
            result = utils.get_task_input()
            if result:
                title, description, priority = result
                new_task = ops.add_task(tasks, title, description, priority)
                utils.display_message(f"Added task #{new_task['id']}: {title}")

        elif choice == "2":
            # List all tasks
            all_tasks = ops.list_tasks(tasks)
            print("\n📋 ALL TASKS")
            utils.display_tasks(all_tasks)

        elif choice == "3":
            # Mark task complete
            task_id = utils.get_task_id()
            if task_id is not None:
                if ops.complete_task(tasks, task_id):
                    utils.display_message(f"Task #{task_id} marked complete")
                else:
                    print(f"❌ Task #{task_id} not found")

        elif choice == "4":
            # List pending tasks
            pending = ops.filter_tasks(tasks, done=False)
            print("\n⏳ PENDING TASKS")
            utils.display_tasks(pending)

        elif choice == "5":
            # List completed tasks
            completed = ops.filter_tasks(tasks, done=True)
            print("\n✓ COMPLETED TASKS")
            utils.display_tasks(completed)

        elif choice == "6":
            # Filter by priority
            try:
                priority_input = input("Enter priority to filter (1-10): ").strip()
                priority = int(priority_input)
                if 1 <= priority <= 10:
                    filtered = ops.filter_tasks(tasks, priority=priority)
                    print(f"\n🎯 TASKS WITH PRIORITY {priority}")
                    utils.display_tasks(filtered)
                else:
                    print("⚠️ Priority must be between 1 and 10")
            except ValueError:
                print("⚠️ Priority must be a number")

        elif choice == "7":
            # Update task priority
            task_id = utils.get_task_id()
            if task_id is not None:
                try:
                    new_priority_str = input("New priority (1-10): ").strip()
                    new_priority = int(new_priority_str)
                    if ops.update_task_priority(tasks, task_id, new_priority):
                        utils.display_message(f"Task #{task_id} priority updated to {new_priority}")
                    else:
                        print(f"❌ Task #{task_id} not found")
                except ValueError:
                    print("⚠️ Priority must be a number")

        elif choice == "8":
            print("\n👋 Thank you for using Todo Console App. Goodbye!")
            break

        else:
            print("❌ Invalid choice. Please try again.")


if __name__ == "__main__":
    run_todo_app()
```

**Output** (when run):
```
========================================
         TODO CONSOLE APP
========================================
1. Add a new task
2. List all tasks
3. Mark task complete
4. List pending tasks
5. List completed tasks
6. Filter by priority
7. Update task priority
8. Exit
========================================

Enter choice (1-8): 1
Task title: Review PR
Description (optional): Check code quality
Priority (1=highest, 10=lowest, default 5): 2

✨ Added task #1: Review PR

...
```

**Key Patterns**:
- `import task_operations as ops`: Imports custom module
- `import utils`: Imports second custom module
- Function calls like `ops.add_task()` and `utils.display_tasks()` show module.function pattern
- Logic is clear: display menu, get input, call operation, display result
- Each choice orchestrates multiple modules

---

## Step 4: Create `test_todo.py` — Comprehensive Testing

This file validates that functions in `task_operations.py` work correctly for various inputs.

### 💻 Code Idea: Test Module

```python
# File: test_todo.py
"""
Test the todo app operations.

Run with: python test_todo.py
Or: python -m pytest test_todo.py
"""

import task_operations as ops


def reset_id_counter():
    """Reset ID counter for testing (allows deterministic IDs)."""
    ops._next_id = 1


def test_create_task() -> None:
    """Test creating a new task."""
    reset_id_counter()
    task = ops.create_task("Buy groceries", "Milk, eggs, bread", 3)
    assert task["title"] == "Buy groceries"
    assert task["description"] == "Milk, eggs, bread"
    assert task["priority"] == 3
    assert task["status"] == "pending"
    assert task["done"] is False
    assert task["id"] == 1
    print("✓ test_create_task PASSED")


def test_create_task_validation() -> None:
    """Test that create_task validates inputs."""
    reset_id_counter()
    # Empty title should raise error
    try:
        ops.create_task("", "description", 5)
        assert False, "Should have raised ValueError for empty title"
    except ValueError:
        pass

    # Priority out of range should raise error
    try:
        ops.create_task("Task", "", 15)
        assert False, "Should have raised ValueError for invalid priority"
    except ValueError:
        pass

    print("✓ test_create_task_validation PASSED")


def test_add_task() -> None:
    """Test adding tasks to a list."""
    reset_id_counter()
    tasks = []
    task1 = ops.add_task(tasks, "Task 1", "", 5)
    task2 = ops.add_task(tasks, "Task 2", "", 3)
    assert len(tasks) == 2
    assert tasks[0]["id"] == 1
    assert tasks[1]["id"] == 2
    print("✓ test_add_task PASSED")


def test_complete_task() -> None:
    """Test marking a task complete."""
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "Task 1", "", 5)
    ops.add_task(tasks, "Task 2", "", 3)

    # Mark first task complete
    result = ops.complete_task(tasks, 1)
    assert result is True
    assert tasks[0]["done"] is True
    assert tasks[0]["status"] == "completed"

    # Task not found
    result = ops.complete_task(tasks, 999)
    assert result is False

    print("✓ test_complete_task PASSED")


def test_list_tasks() -> None:
    """Test listing all tasks."""
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "Task 1", "", 5)
    ops.add_task(tasks, "Task 2", "", 3)

    listed = ops.list_tasks(tasks)
    assert len(listed) == 2
    assert listed[0]["title"] == "Task 1"

    print("✓ test_list_tasks PASSED")


def test_filter_tasks_by_status() -> None:
    """Test filtering tasks by completion status."""
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "Task 1", "", 5)
    ops.add_task(tasks, "Task 2", "", 3)
    ops.complete_task(tasks, 1)

    pending = ops.filter_tasks(tasks, done=False)
    assert len(pending) == 1
    assert pending[0]["id"] == 2

    completed = ops.filter_tasks(tasks, done=True)
    assert len(completed) == 1
    assert completed[0]["id"] == 1

    print("✓ test_filter_tasks_by_status PASSED")


def test_filter_tasks_by_priority() -> None:
    """Test filtering tasks by priority."""
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "High priority", "", 1)
    ops.add_task(tasks, "Medium priority", "", 5)
    ops.add_task(tasks, "Low priority", "", 10)

    high = ops.filter_tasks(tasks, priority=1)
    assert len(high) == 1
    assert high[0]["title"] == "High priority"

    medium = ops.filter_tasks(tasks, priority=5)
    assert len(medium) == 1

    print("✓ test_filter_tasks_by_priority PASSED")


def test_filter_tasks_combined() -> None:
    """Test filtering by status and priority together."""
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "Task 1", "", 1)
    ops.add_task(tasks, "Task 2", "", 1)
    ops.add_task(tasks, "Task 3", "", 5)
    ops.complete_task(tasks, 1)

    # High priority and done
    result = ops.filter_tasks(tasks, done=True, priority=1)
    assert len(result) == 1
    assert result[0]["id"] == 1

    print("✓ test_filter_tasks_combined PASSED")


def test_get_task_by_id() -> None:
    """Test finding a task by ID."""
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "Task 1", "", 5)
    ops.add_task(tasks, "Task 2", "", 3)

    task = ops.get_task_by_id(tasks, 2)
    assert task is not None
    assert task["title"] == "Task 2"

    task = ops.get_task_by_id(tasks, 999)
    assert task is None

    print("✓ test_get_task_by_id PASSED")


def test_update_task_priority() -> None:
    """Test updating a task's priority."""
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "Task 1", "", 5)

    result = ops.update_task_priority(tasks, 1, 2)
    assert result is True
    assert tasks[0]["priority"] == 2

    # Task not found
    result = ops.update_task_priority(tasks, 999, 1)
    assert result is False

    # Invalid priority
    try:
        ops.update_task_priority(tasks, 1, 15)
        assert False, "Should have raised ValueError"
    except ValueError:
        pass

    print("✓ test_update_task_priority PASSED")


if __name__ == "__main__":
    test_create_task()
    test_create_task_validation()
    test_add_task()
    test_complete_task()
    test_list_tasks()
    test_filter_tasks_by_status()
    test_filter_tasks_by_priority()
    test_filter_tasks_combined()
    test_get_task_by_id()
    test_update_task_priority()
    print("\n✓ All tests passed!")
```

**Output:**
```
✓ test_create_task PASSED
✓ test_create_task_validation PASSED
✓ test_add_task PASSED
✓ test_complete_task PASSED
✓ test_list_tasks PASSED
✓ test_filter_tasks_by_status PASSED
✓ test_filter_tasks_by_priority PASSED
✓ test_filter_tasks_combined PASSED
✓ test_get_task_by_id PASSED
✓ test_update_task_priority PASSED

✓ All tests passed!
```

**Testing Patterns**:
- `reset_id_counter()`: Ensures deterministic IDs across test runs
- Each test function checks one operation or edge case
- Use `assert` statements to verify expected behavior
- Test normal cases, edge cases, and error cases
- Catch exceptions with try/except to test error handling
- Run all tests to validate the entire project

---

## How to Run the Project

### Run the Todo App (Interactive)

```bash
# Navigate to project directory
cd todo_app/

# Run the main program
python main.py
```

The app will display a menu. Choose an option (1-8) and follow prompts.

**Example Session**:
```
========================================
         TODO CONSOLE APP
========================================
1. Add a new task
2. List all tasks
3. Mark task complete
4. List pending tasks
5. List completed tasks
6. Filter by priority
7. Update task priority
8. Exit
========================================

Enter choice (1-8): 1
Task title: Review PR
Description (optional): Check for code quality
Priority (1=highest, 10=lowest, default 5): 2

✨ Added task #1: Review PR
```

### Run the Tests (Validation)

```bash
# Run tests to verify operations work
python test_todo.py

# Output should show:
# ✓ test_create_task PASSED
# ✓ test_add_task PASSED
# ... (all tests)
# ✓ All tests passed!
```

---

## Understanding the Project Structure

### Why Separate Files?

**`task_operations.py`** — Pure Task Functions
- Does: Manipulate task data (add, complete, filter, update)
- Doesn't: Print, read input, modify global state (except for ID counter)
- Benefit: Easy to test in isolation, easy to reuse with different UIs (web, mobile, etc.)

**`utils.py`** — User Interface
- Does: User interaction, input validation, display formatting
- Doesn't: Manipulate task data directly (calls task_operations for that)
- Benefit: Can be reused for other programs that need similar input/output patterns

**`main.py`** — Orchestration
- Does: Import and coordinate other modules, implement application flow
- Doesn't: Do task manipulation or complex I/O directly
- Benefit: Clear, readable flow of program logic—shows how modules work together

**`test_todo.py`** — Verification
- Does: Test task_operations functions with various inputs
- Benefit: Confidence that functions work before using them in the UI

### Example: Why This Separation Matters

Imagine you want to add a web interface. Here's what happens:

**Without separation** (calculator.py):
```python
# All code in one file
# Would have to extract functions, separate concerns
# High risk of breaking things
```

**With separation** (our todo app):
```python
# task_operations.py: Already pure functions, no dependencies on terminal UI
# Use the exact same file for web interface
# Just create a new web_interface.py instead of utils.py
# Run: python -m flask app  (web server uses same task_operations.py)
```

This is why separation of concerns scales.

---

## Module Imports — How It Works

```python
# In main.py:
import task_operations as ops  # Import task_operations.py
import utils                   # Import utils.py

# Now you can use:
ops.add_task(tasks, "Buy milk")      # Calls add_task() from task_operations.py
utils.display_menu()                  # Calls display_menu() from utils.py
```

Python searches for modules in this order:
1. Same directory as main.py ✓ (where our modules are)
2. Standard library (`json`, `math`, etc.)
3. Installed packages (`requests`, `numpy`, etc.)

Since all files are in the same `todo_app/` directory, imports work automatically.

---

## Understanding the Task Data Structure

Every task is a dictionary with consistent structure:

```python
task = {
    "id": 1,              # Auto-assigned by create_task()
    "title": "Buy milk",  # What needs to be done (required)
    "description": "",    # Optional notes
    "priority": 5,        # 1 (highest) to 10 (lowest)
    "status": "pending",  # pending → in_progress → completed
    "done": False         # Quick boolean flag
}
```

**Important**: All functions expect tasks in this format. This consistency is what makes the app work:
- Display functions know which fields to show
- Filter functions can reliably check "done" and "priority"
- Tests can verify all fields are present and correct

When you move to Chapter 27, you'll replace this dictionary with a `Task` class—but the structure remains the same.

---

## Extend Your Todo App

Your todo app is working! Now extend it:

1. **Add a new operation** to `task_operations.py` — for example:
   - `delete_task(tasks, task_id)` — Remove a task completely
   - `update_description(tasks, task_id, new_description)` — Edit a task's description
   - `get_high_priority_tasks(tasks)` — Return tasks with priority 1-3

2. **Add the operation to** `main.py` — add it to the menu and create a branch for it

3. **Add tests** for your new operation to `test_todo.py`

**Example**: Implementing delete_task
```python
# In task_operations.py:
def delete_task(tasks: list, task_id: int) -> bool:
    """Delete a task by ID."""
    for i, task in enumerate(tasks):
        if task["id"] == task_id:
            tasks.pop(i)
            return True
    return False

# In test_todo.py:
def test_delete_task():
    reset_id_counter()
    tasks = []
    ops.add_task(tasks, "Task 1", "", 5)
    ops.add_task(tasks, "Task 2", "", 3)
    assert ops.delete_task(tasks, 1) is True
    assert len(tasks) == 1
    print("✓ test_delete_task PASSED")

# In main.py menu:
elif choice == "9":
    task_id = utils.get_task_id()
    if task_id is not None:
        if ops.delete_task(tasks, task_id):
            utils.display_message(f"Task #{task_id} deleted")
        else:
            print(f"❌ Task #{task_id} not found")
```

---

## Try With AI

Build a complete multi-module todo console app integrating all Chapter 23 concepts.

**🏗️ Explore Architecture Decisions:**

> "I want to build a todo app with task operations, a user interface, and tests. Should task operations (add, complete, filter) be in the same module as user interface code (displaying menus, getting input)? What are the pros and cons?"

**🎯 Practice Task Operations Implementation:**

> "Implement task_operations.py with these functions: create_task (validates title, priority 1-10), add_task (appends to list), complete_task (marks done=True, status='completed'), filter_tasks (by done status and/or priority). Use type hints like `def complete_task(tasks: list, task_id: int) -> bool` for operations that may fail. Include docstrings for each function."

**🧪 Test Multi-Module Behavior:**

> "Write comprehensive tests for task_operations.py covering: normal cases (add task, mark complete), edge cases (task doesn't exist, invalid priority), error cases (empty title). For each operation, verify the task list is modified correctly and return values match expectations."

**🚀 Integrate Everything:**

> "Build working main.py that imports task_operations and utils. Implement a menu system (1-8 choices): Add, List All, Mark Complete, List Pending, List Completed, Filter by Priority, Update Priority, Exit. For each choice, get input using utils functions, call task_operations functions, and display results using utils display functions. Make sure to handle invalid inputs gracefully."
