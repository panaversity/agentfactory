# Task Entity Reference

**Purpose**: Canonical Task entity definition for consistency across all Part 5 lessons.
**Source**: Gold standard from Ch29-04 (TodoList dataclass)
**Created**: 2025-12-26

---

## Core Entity: Task

The `Task` entity represents a single todo item in the Todo console app that students build incrementally across Part 5.

### Canonical Attributes

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `title` | `str` | Yes | - | Short description of the task (1-100 chars) |
| `description` | `str` | No | `""` | Detailed description or notes |
| `due_date` | `datetime \| None` | No | `None` | When task should be completed |
| `priority` | `int` | No | `5` | Priority level (1=highest, 10=lowest) |
| `status` | `str` | No | `"pending"` | One of: "pending", "in_progress", "completed" |
| `done` | `bool` | No | `False` | Whether task is completed |

### Variable Naming Conventions

When renaming variables in lessons, use these consistent names:

| Context | Variable Name | Example Usage |
|---------|---------------|---------------|
| Single task | `task` | `task = {"title": "Review PR"}` |
| Task list | `tasks` | `tasks = [task1, task2, task3]` |
| Task title | `task_title` | `task_title = "Buy groceries"` |
| Task ID | `task_id` | `task_id = 1` |
| Task status | `is_done` | `is_done = False` |
| Task priority | `priority` | `priority = 1` |
| Task manager | `task_manager` | `task_manager = TaskManager()` |
| Todo list | `todo_list` | `todo_list = TodoList(name="Work")` |

### Python Class Definition (Reference)

```python
from dataclasses import dataclass, field
from datetime import datetime

@dataclass
class Task:
    """A single todo item."""
    title: str
    description: str = ""
    due_date: datetime | None = None
    priority: int = 5
    status: str = "pending"
    done: bool = False

    def mark_complete(self) -> None:
        """Mark task as completed."""
        self.done = True
        self.status = "completed"

    def update_priority(self, new_priority: int) -> None:
        """Update task priority (1-10)."""
        if 1 <= new_priority <= 10:
            self.priority = new_priority
        else:
            raise ValueError("Priority must be between 1 and 10")
```

### Dict Representation (Early Chapters)

Before students learn OOP, tasks are represented as dictionaries:

```python
task = {
    "title": "Review PR",
    "description": "Check for code quality",
    "due_date": "2025-01-15",
    "priority": 2,
    "status": "pending",
    "done": False
}
```

### List of Tasks

```python
tasks = [
    {"title": "Task 1", "done": False, "priority": 1},
    {"title": "Task 2", "done": True, "priority": 5},
    {"title": "Task 3", "done": False, "priority": 3},
]
```

---

## Related Entity: TodoList

Container for multiple tasks (from Ch29-04 gold standard):

```python
from dataclasses import dataclass, field

@dataclass
class TodoList:
    name: str
    items: list[str] = field(default_factory=list)
    tags: dict[str, str] = field(default_factory=dict)
    priority: int = 5
```

---

## Usage Guidelines

1. **Early chapters (Ch16-22)**: Use dict representation
2. **Functions chapter (Ch23)**: Use dict + functions
3. **OOP chapters (Ch27-28)**: Introduce Task class
4. **Dataclass chapter (Ch29)**: Show TodoList dataclass
5. **Later chapters (Ch30-32)**: Build on class-based approach

### Progression Example

| Chapter | Task Representation |
|---------|---------------------|
| Ch17 | `task = {"title": "...", "done": False}` |
| Ch21 | `tasks = [task1, task2]` (list of dicts) |
| Ch23 | `add_task(tasks, title, priority)` |
| Ch27 | `class Task: ...` |
| Ch29 | `@dataclass class TodoList: ...` |
