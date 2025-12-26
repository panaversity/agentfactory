---
sidebar_position: 3
title: "Polymorphism Through Dunder Methods"
chapter: 28
lesson: 3
duration_minutes: 75

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Dunder Methods as Protocol Contracts"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can recognize how dunder methods enable polymorphic behavior without inheritance"

  - name: "String Representation for Domain Objects (__str__, __repr__)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement __str__ and __repr__ for domain objects like Task to support readable output"

  - name: "Comparison Protocols (__eq__, __lt__, __hash__)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement comparison methods to enable sorting and set/dict usage"

  - name: "Container Protocols (__contains__, __len__)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement container methods to make objects work with built-in operations"

  - name: "Polymorphism Without Inheritance"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain how dunder method protocols enable polymorphic behavior across unrelated classes"

learning_objectives:
  - objective: "Implement __str__ and __repr__ for domain objects to enable human-readable and debug-friendly output"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Create Task class with both __str__ and __repr__, demonstrate different output formats"

  - objective: "Implement __eq__ and __lt__ to enable comparison operations and sorting of domain objects"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Implement Task comparison by priority, verify sorted() works correctly"

  - objective: "Implement __hash__ to enable use of domain objects in sets and dictionaries"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Create set of Task objects and verify no duplicates based on equality"

  - objective: "Understand how dunder methods create implicit contracts across unrelated classes"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Compare Task, Case, and Appointment classes implementing different dunder methods"

cognitive_load:
  new_concepts: 7
  assessment: "7 new dunder methods (__str__, __repr__, __eq__, __lt__, __hash__, __contains__, __len__) within B1 limit of 10, organized around Task domain object ✓"

differentiation:
  extension_for_advanced: "Implement __add__ for Task aggregation; explore descriptor protocol; implement custom container with __getitem__/__setitem__"
  remedial_for_struggling: "Start with just __str__ for basic output; practice __eq__ with simple comparisons; use Task examples throughout for consistency"

# Generation metadata
generated_by: "content-implementer v4.1.0"
source_spec: "Task entity reference - Part 5 todo integration"
created: "2025-12-26"
last_modified: "2025-12-26"
git_author: "Claude Code"
workflow: "/task.reframe"
version: "1.0.0"
---

# Polymorphism Through Dunder Methods

## Introduction: Making Objects Behave Like Built-In Types

In the last lesson, you learned that polymorphism doesn't require inheritance. Objects can work together if they implement the same interface—this is duck typing. But how do you design interfaces that Python *understands*?

The answer lies in **dunder methods** (double-underscore methods, also called "magic methods"). These special methods are Python's protocol definitions. When you implement `__str__()`, Python knows your object supports `print()`. When you implement `__eq__()`, Python knows your object supports `==` comparisons. When you implement `__lt__()`, Python knows your object can be sorted.

In this lesson, you'll see how **dunder methods enable polymorphism across completely unrelated classes**. A Task in a todo app, a Case in a legal system, and an Appointment in a calendar might have nothing in common structurally—but if they all implement `__lt__()`, they can all be sorted by the same `sorted()` function. That's polymorphism through protocol.

**Why this matters for AI systems**: When building multi-agent systems, you often need agents, tasks, messages, and logs to work with Python's built-in operations (`len()`, sorting, string conversion, comparisons). Dunder methods make this possible without forcing inheritance hierarchies.

---

## The Task Entity: Our Primary Example

Throughout this lesson, we'll use a concrete domain object—the **Task** class from todo applications. This is the canonical Task definition used across Part 5 lessons.

```python
class Task:
    """A single todo item with priority-based sorting."""

    def __init__(self, title: str, priority: int = 5, done: bool = False):
        self.title = title
        self.priority = priority  # 1=highest, 10=lowest
        self.done = done
```

This simple class will demonstrate how dunder methods enable polymorphic behavior—making Task objects work naturally with Python's syntax and operations.

---

## String Representation: Making Objects Printable

When you `print()` an object or view it in the Python shell, Python needs to convert it to a string. Two dunder methods control this.

### __str__: User-Friendly Display

`__str__()` returns a string optimized for **end users**. Python calls it when you use `print()` or `str()`.

```python
class Task:
    def __init__(self, title: str, priority: int = 5, done: bool = False):
        self.title = title
        self.priority = priority
        self.done = done

    def __str__(self) -> str:
        """User-friendly string for print()."""
        status = "✓" if self.done else "○"
        return f"[{status}] {self.title}"

# Usage
task = Task("Review PR", priority=2)
print(task)  # Output: [○] Review PR

completed = Task("Write tests", priority=5, done=True)
print(completed)  # Output: [✓] Write tests
```

The user sees a clean, readable format. No implementation details, no `<Task object at 0x...>` nonsense.

### __repr__: Developer-Friendly Display

`__repr__()` returns a string for **developers debugging code**. Python calls it in the interactive shell or when you call `repr()`.

```python
class Task:
    def __init__(self, title: str, priority: int = 5, done: bool = False):
        self.title = title
        self.priority = priority
        self.done = done

    def __str__(self) -> str:
        """User-friendly string"""
        status = "✓" if self.done else "○"
        return f"[{status}] {self.title}"

    def __repr__(self) -> str:
        """Developer-friendly string"""
        return f"Task(title={self.title!r}, priority={self.priority}, done={self.done})"

# Usage
task = Task("Review PR", priority=2)
print(task)       # [○] Review PR (calls __str__)
print(repr(task)) # Task(title='Review PR', priority=2, done=False) (calls __repr__)
```

**Convention**: `repr()` output should ideally be valid Python code that could recreate the object. This helps debugging—you can copy the repr output and paste it into code.

#### Comparing Tasks in Different Contexts

Notice how `__str__()` and `__repr__()` serve different purposes:

```python
tasks = [
    Task("Fix bug", priority=1),
    Task("Update docs", priority=5),
    Task("Review PR", priority=2)
]

# End-user sees clean output
print("Tasks to do:")
for task in tasks:
    print(f"  {task}")  # Calls __str__()

# Output:
# Tasks to do:
#   [○] Fix bug
#   [○] Update docs
#   [○] Review PR

# Developer sees complete state
print("Task list internal state:")
print(tasks)  # Calls __repr__() on list items

# Output:
# [Task(title='Fix bug', priority=1, done=False),
#  Task(title='Update docs', priority=5, done=False),
#  Task(title='Review PR', priority=2, done=False)]
```

---

## Comparison Methods: Making Objects Orderable

What happens when you try to sort tasks by priority? Without implementing comparison methods, Python raises an error. The `__lt__()` method (and related comparison dunder methods) teach Python how to compare your objects.

### __eq__ and __lt__: Equality and Ordering

```python
class Task:
    def __init__(self, title: str, priority: int = 5, done: bool = False):
        self.title = title
        self.priority = priority
        self.done = done

    def __eq__(self, other: object) -> bool:
        """Define equality: tasks are equal if they have the same title."""
        if not isinstance(other, Task):
            return NotImplemented
        return self.title == other.title

    def __lt__(self, other: "Task") -> bool:
        """Define less-than: compare by priority (lower number = higher priority)."""
        if not isinstance(other, Task):
            return NotImplemented
        return self.priority < other.priority

    def __str__(self) -> str:
        status = "✓" if self.done else "○"
        return f"[{status}] {self.title}"

# Usage
task1 = Task("Fix bug", priority=1)
task2 = Task("Update docs", priority=5)
task3 = Task("Fix bug", priority=3)  # Same title as task1

print(task1 == task3)  # True (same title)
print(task1 < task2)   # True (priority 1 < priority 5)

# Sort tasks by priority (uses __lt__)
tasks = [task2, task1, Task("Review PR", priority=2)]
sorted_tasks = sorted(tasks)

for task in sorted_tasks:
    print(task)

# Output:
# [○] Fix bug (priority 1)
# [○] Review PR (priority 2)
# [○] Update docs (priority 5)
```

**Key insight**: By implementing just `__lt__()` and `__eq__()`, Python can derive all other comparisons (`<=`, `>`, `>=`) through the `@functools.total_ordering` decorator if needed.

### __hash__: Making Objects Usable in Sets and Dicts

When you implement `__eq__()`, you should also implement `__hash__()`. This allows Task objects to be used as dictionary keys or members of sets.

```python
class Task:
    def __init__(self, title: str, priority: int = 5, done: bool = False):
        self.title = title
        self.priority = priority
        self.done = done

    def __eq__(self, other: object) -> bool:
        """Tasks are equal if they have the same title."""
        if not isinstance(other, Task):
            return NotImplemented
        return self.title == other.title

    def __hash__(self) -> int:
        """Hash must match __eq__: if tasks are equal, their hashes must be equal."""
        return hash(self.title)

    def __str__(self) -> str:
        status = "✓" if self.done else "○"
        return f"[{status}] {self.title}"

# Now we can use Task objects in sets
task1 = Task("Fix bug", priority=1)
task2 = Task("Fix bug", priority=3)  # Same title, different priority
task3 = Task("Update docs", priority=5)

task_set = {task1, task2, task3}
print(len(task_set))  # 2 (task1 and task2 are equal, so duplicates removed)

# We can use Task objects as dictionary keys
task_deadlines = {
    Task("Fix bug"): "2025-01-15",
    Task("Update docs"): "2025-01-20"
}

for task, deadline in task_deadlines.items():
    print(f"{task} - Due: {deadline}")
```

**Critical rule**: Objects that compare equal (via `__eq__()`) **must** have the same hash (via `__hash__()`). Breaking this rule causes mysterious bugs in sets and dicts.

---

## Container Protocols: Making Objects Behave Like Collections

Dunder methods also let objects behave like containers. A TodoList might contain tasks. Using `__len__()` and `__contains__()`, we make TodoList work with Python's built-in `len()` function and the `in` operator.

```python
class TodoList:
    """A container of Task objects."""

    def __init__(self, name: str):
        self.name = name
        self._tasks: list[Task] = []

    def add_task(self, task: Task) -> None:
        """Add a task to the list."""
        self._tasks.append(task)

    def __len__(self) -> int:
        """Support len(todo_list) - return number of tasks."""
        return len(self._tasks)

    def __contains__(self, task: Task) -> bool:
        """Support task in todo_list - check if task exists."""
        return task in self._tasks

    def __str__(self) -> str:
        return f"TodoList({self.name}): {len(self)} tasks"

# Usage
todo_list = TodoList("Work")
task1 = Task("Fix bug", priority=1)
task2 = Task("Update docs", priority=5)

todo_list.add_task(task1)
todo_list.add_task(task2)

print(len(todo_list))        # 2 (uses __len__)
print(task1 in todo_list)    # True (uses __contains__)
print(todo_list)             # TodoList(Work): 2 tasks
```

---

## Polymorphism Across Different Domain Objects

Here's where dunder methods create powerful polymorphism without inheritance. Different domain objects—Task, Case, Appointment—can work with the **same functions** if they implement the same dunder methods.

```python
class Task:
    """Todo item with priority."""
    def __init__(self, title: str, priority: int = 5):
        self.title = title
        self.priority = priority

    def __str__(self) -> str:
        return f"Task: {self.title} (priority {self.priority})"

    def __lt__(self, other: "Task") -> bool:
        return self.priority < other.priority

class Case:
    """Legal case with urgency level."""
    def __init__(self, number: str, urgency: int = 5):
        self.number = number
        self.urgency = urgency

    def __str__(self) -> str:
        return f"Case #{self.number} (urgency {self.urgency})"

    def __lt__(self, other: "Case") -> bool:
        return self.urgency < other.urgency

class Appointment:
    """Calendar appointment with importance."""
    def __init__(self, title: str, importance: int = 5):
        self.title = title
        self.importance = importance

    def __str__(self) -> str:
        return f"Appointment: {self.title} (importance {self.importance})"

    def __lt__(self, other: "Appointment") -> bool:
        return self.importance < other.importance

# All three types work with the SAME sorted() function!
items = [
    Task("Fix bug", priority=3),
    Case("CA-2025-001", urgency=1),
    Appointment("Board meeting", importance=2),
    Task("Update docs", priority=5),
    Case("CA-2025-002", urgency=4),
]

sorted_items = sorted(items)  # Uses __lt__ from each class

for item in sorted_items:
    print(item)

# Output:
# Case #CA-2025-001 (urgency 1)
# Appointment: Board meeting (importance 2)
# Task: Fix bug (priority 3)
# Case #CA-2025-002 (urgency 4)
# Task: Update docs (priority 5)
```

**This is polymorphism without inheritance.** Task, Case, and Appointment have **no common parent class**. Yet they all work with `sorted()` because they all implement `__lt__()`. This is the power of protocol-based polymorphism.

---

## Practical Example: Task Management Operations

Let's see Task dunder methods in action with realistic operations:

```python
class Task:
    """A todo item for task management."""

    def __init__(self, title: str, priority: int = 5, done: bool = False):
        self.title = title
        self.priority = priority
        self.done = done

    def __str__(self) -> str:
        """Human-readable format for display."""
        status = "✓" if self.done else "○"
        return f"[{status}] {self.title}"

    def __repr__(self) -> str:
        """Developer-friendly format for debugging."""
        return f"Task(title={self.title!r}, priority={self.priority}, done={self.done})"

    def __eq__(self, other: object) -> bool:
        """Tasks are equal if they have the same title."""
        if not isinstance(other, Task):
            return NotImplemented
        return self.title == other.title

    def __lt__(self, other: "Task") -> bool:
        """Sort by priority (lower number = higher priority)."""
        if not isinstance(other, Task):
            return NotImplemented
        return self.priority < other.priority

    def __hash__(self) -> int:
        """Hash based on title for set/dict usage."""
        return hash(self.title)

# Real-world usage
work_tasks = [
    Task("Review PR", priority=2),
    Task("Fix bug", priority=1),
    Task("Update docs", priority=5),
    Task("Refactor code", priority=3),
]

# Sort by priority (uses __lt__)
print("Tasks by priority:")
for task in sorted(work_tasks):
    print(f"  {task}")

# Use in set to remove duplicates (uses __eq__ and __hash__)
duplicate_tasks = [
    Task("Fix bug", priority=1),
    Task("Fix bug", priority=2),  # Different priority, same title = duplicate
    Task("Update docs", priority=5),
]
unique_tasks = set(duplicate_tasks)
print(f"\nUnique tasks: {len(unique_tasks)}")  # 2

# Use as dictionary keys (uses __hash__ and __eq__)
task_status = {}
for task in work_tasks:
    task_status[task] = "pending"

print("\nTask status:")
for task, status in task_status.items():
    print(f"  {repr(task)} -> {status}")
```

**Output:**
```
Tasks by priority:
  [○] Fix bug
  [○] Review PR
  [○] Refactor code
  [○] Update docs

Unique tasks: 2

Task status:
  Task(title='Fix bug', priority=1, done=False) -> pending
  Task(title='Review PR', priority=2, done=False) -> pending
  Task(title='Refactor code', priority=3, done=False) -> pending
  Task(title='Update docs', priority=5, done=False) -> pending
```

---

## Common Patterns and Best Practices

### Pattern 1: Type Checking in Dunder Methods

Always check types before operating on `other`:

```python
def __eq__(self, other: object) -> bool:
    if not isinstance(other, Task):
        return NotImplemented  # Not False! NotImplemented
    return self.title == other.title

def __lt__(self, other: "Task") -> bool:
    if not isinstance(other, Task):
        return NotImplemented
    return self.priority < other.priority
```

Return `NotImplemented` (not `False`) when you don't know how to handle the operation. This lets Python try the reverse operation on the other object.

### Pattern 2: Hash Consistency

If you implement `__eq__()`, implement `__hash__()` to match:

```python
def __eq__(self, other: object) -> bool:
    return self.title == other.title

def __hash__(self) -> int:
    return hash(self.title)  # Must use the SAME field as __eq__
```

Objects that compare equal must have the same hash. Breaking this rule causes cryptic bugs with sets and dictionaries.

### Pattern 3: __repr__ Should Be Valid Python

Ideally, `repr()` output can be evaluated to recreate the object:

```python
def __repr__(self) -> str:
    return f"Task(title={self.title!r}, priority={self.priority}, done={self.done})"

# Usage
task = Task("Fix bug", priority=1, done=False)
repr_str = repr(task)  # Task(title='Fix bug', priority=1, done=False)

# Can paste into code to recreate:
recreated = eval(repr_str)  # Works!
```

---

## Try With AI

**Setup**: Create a Task class and explore how dunder methods enable polymorphic behavior.

### Prompt 1: Discovering String Representation

> "Create a Task class with __init__(title, priority). Implement both __str__() and __repr__(). Show me:
> 1. What print(task) returns (should call __str__)
> 2. What task in the shell returns (should call __repr__)
> 3. When would a user see __str__ output vs __repr__ output?
> 4. Why does __repr__ show all the details but __str__ shows a clean format?"

**What you're learning**: The difference between user-facing and developer-facing output, and when each is appropriate.

### Prompt 2: Making Tasks Sortable

> "Implement `__eq__()` and `__lt__()` for Task based on priority. Show me:
> 1. How `tasks = [Task('Fix bug', 3), Task('Update docs', 5)]; sorted(tasks)` works
> 2. What Python does internally when it calls sorted() - trace through __lt__
> 3. How __eq__() determines if two tasks are the same
> 4. Design choice: should __eq__ compare by title or priority? Why?"

**What you're learning**: How comparison dunder methods enable sorting without inheritance, and the design choices involved.

### Prompt 3: Using Tasks in Collections

> "Implement `__hash__()` for Task and show:
> 1. How `task_set = {Task('Fix bug'), Task('Fix bug')}` removes duplicates
> 2. Why `__hash__` must match `__eq__` (use the same field)
> 3. How tasks work as dictionary keys: `task_deadlines = {task: '2025-01-15'}`
> 4. What breaks if you implement __eq__ but forget __hash__?"

**What you're learning**: The contract between `__hash__` and `__eq__`, and why breaking it causes mysterious bugs.

### Prompt 4: Polymorphism Across Domains

> "Create three unrelated classes (Task, Case, Appointment) - NO inheritance. Each has different attributes but all implement `__lt__()` using different priority fields:
> 1. Task sorts by priority (1 is high, 10 is low)
> 2. Case sorts by urgency (same scale)
> 3. Appointment sorts by importance (same scale)
>
> Now show me one sorted() call that works on a mixed list of all three types. Explain why this works WITHOUT a common parent class. This is polymorphism through protocol."

**What you're learning**: The core insight—dunder methods create implicit contracts that multiple unrelated classes can follow, enabling polymorphism without inheritance.

---

## Summary

Dunder methods are Python's protocol definitions. By implementing `__str__()`, `__repr__()`, `__eq__()`, `__lt__()`, and `__hash__()`, you teach Python how to work with your objects using built-in operations and functions.

**Key takeaways**:
- `__str__()`: User-friendly output for `print()`
- `__repr__()`: Developer-friendly output for debugging
- `__eq__()`: Define equality for `==` comparisons
- `__lt__()`: Define ordering for `sorted()`
- `__hash__()`: Enable use in sets and as dictionary keys

Most importantly: **Dunder methods enable polymorphism across unrelated classes.** Task, Case, and Appointment have nothing in common—no inheritance, no ABC—yet they all work with `sorted()` because they all implement `__lt__()`. This is the power of protocol-based design.

In the next lesson, you'll explore even more dunder methods (`__add__`, `__len__`, `__getitem__`, `__iter__`, `__call__`) that enable operators, container behavior, and callable objects.
