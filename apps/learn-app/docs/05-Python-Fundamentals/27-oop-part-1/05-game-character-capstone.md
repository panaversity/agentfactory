---
sidebar_position: 5
title: "Task and TaskManager Classes (Capstone)"
description: "Capstone project integrating OOP fundamentals by building a Task management system with full encapsulation, validation, and composition patterns."
keywords:
  - "classes and objects"
  - "constructors"
  - "encapsulation"
  - "methods"
  - "composition"
  - "OOP design"
  - "task management"
chapter: 27
lesson: 5
duration_minutes: 70

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Task Class Design with Encapsulation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student designs Task class with validated properties, proper initialization, and methods that enforce constraints (priority ranges, status values)"

  - name: "Composition and Object Relationships"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student implements TaskManager that manages collection of Task objects, demonstrating HAS-A relationships"

  - name: "String Representation and Equality"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student implements __repr__ and __eq__ for meaningful Task display and comparison"

  - name: "Method Design for State Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student designs methods (mark_complete, update_priority) that safely modify object state"

  - name: "List Operations and Filtering"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student implements find_by_title, filter by priority, and list operations within TaskManager"

  - name: "Multi-Class System Integration"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student integrates Task and TaskManager classes into working system with composition pattern"

  - name: "Project Planning with AI"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Create"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student collaboratively designs system architecture with AI, refines requirements, validates implementation"

  - name: "OOP Design Synthesis"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student synthesizes all Chapter 27 OOP concepts (classes, attributes, methods, encapsulation, properties, composition) into production-quality system"

learning_objectives:
  - objective: "Design and implement a Task class with proper encapsulation, validation, and utility methods following OOP principles"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Task class implementation with __init__, methods, properties, __repr__, __eq__ used correctly"

  - objective: "Build a TaskManager class that manages multiple Task objects using composition and collection operations"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "TaskManager implementation with add_task, list_tasks, find_by_title, filter operations"

  - objective: "Apply encapsulation and validation patterns to prevent invalid object states (priority ranges, status values)"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Validation logic in Task methods with error handling"

  - objective: "Extend base system with new features (RecurringTask, Priority filtering) demonstrating design flexibility"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Extended Task class or new subclass with inheritance/composition"

  - objective: "Collaborate with AI to design, implement, validate, and extend a real-world system at professional standards"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Full project workflow: planning → implementation → validation → extension"

cognitive_load:
  new_concepts: 0
  assessment: "Integration lesson - no new concepts, only synthesis of Chapter 27 (classes, constructors, attributes, properties, methods, composition). All concepts taught in Lessons 1-4."

differentiation:
  extension_for_advanced: "Add TaskList dataclass wrapper (preview of Ch29), implement priority sorting with custom comparators, add due date handling with datetime, create recurring task patterns, implement task filtering by multiple criteria, add serialization preview for save/load patterns"
  remedial_for_struggling: "Build Task class completely before TaskManager. Test each Task method independently before integration. Focus on single feature at a time (creation → completion → priority) rather than implementing all at once. Use AI-generated test cases to validate."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/046-part5-todo-integration/reference/task-entity.md"
created: "2025-12-26"
last_modified: "2025-12-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Task and TaskManager Classes (Capstone)

Every application manages collections of things: inventory items, user profiles, support tickets, or todo tasks. This capstone teaches you how professionals structure such systems using OOP. Instead of manually typing 200+ lines of code, you'll work at a higher level: **planning the architecture with AI, having it generate code, then validating and extending the design**. This mirrors how production systems are built.

**Key shift**: You focus on design decisions and validation. AI handles typing and syntax details.

---

## Part 1: Plan Task System Architecture with AI

**Your Role**: System designer collaborating with AI to plan (not write formal specs - just plain language design thinking)

### Planning Exercise: Design Your Task Management System

You'll have a **design conversation** with AI about what you want to build. No formal specs - just system design thinking.

#### 💬 AI CoLearning Prompt - Initial System Design

> "I want to build a todo task management system. Help me think through the design (don't write code yet, just help me plan):
>
> **What I want:**
> - Task items that track title, description, priority, and completion status
> - A TaskManager that stores multiple tasks and lets me search by title
> - Tasks should validate that priority stays between 1-10
> - Mark tasks complete and update priorities
> - Get a readable string representation of tasks
>
> **Help me decide:**
> 1. Should I have a Task class for individual items and a separate TaskManager class? Why?
> 2. How do I prevent invalid priority values (negative, over 10)?
> 3. What methods should Task have? (create, complete, modify priority, display)
> 4. What methods should TaskManager have? (add, find, list, maybe filter by priority)
> 5. How should tasks display themselves as text?
> 6. How should I structure this so I can extend it later (recurring tasks, due dates)?
>
> Give me a simple design plan with class structure and responsibilities. Use plain language, not technical jargon."

**Expected AI Output**: A plain-language plan explaining:
- Task class = individual todo item with validation
- TaskManager class = container managing multiple tasks (composition)
- Methods for both classes with clear responsibilities
- Validation strategy (properties or validation in methods)
- Design rationale in simple terms

**Your task**: Read AI's plan. Does the design make sense? Can you visualize how the classes work together? What questions do you have?

---

### Challenge AI's Design with "What If" Scenarios

Now test if AI thought through edge cases:

#### 💬 AI CoLearning Prompt - Edge Case Planning

> "I like your plan, but I need to think through some real-world scenarios:
>
> **Scenario 1**: Someone tries to create a Task with priority 15. What should happen?
> **Scenario 2**: User marks a task complete, then wants to unmark it later. How should that work?
> **Scenario 3**: User searches for a task with title "Buy Milk" but all titles are case-sensitive. What if they search "buy milk"? Exact match only or fuzzy?
> **Scenario 4**: Two tasks have the same title. Should find_by_title return the first match, all matches, or raise an error?
> **Scenario 5**: TaskManager is empty. What should list_tasks() return?
>
> For each, explain what makes sense from a user perspective. How should my design handle these cases?"

**Expected Learning**: AI will explain design patterns (validation with exceptions, stateful properties, case-insensitive search, defensive programming). You're learning **design thinking**, not just coding syntax.

---

### Your Simple Design Plan

After your conversation with AI, write down a simple plan:

**My Task System Design Notes:**
- Task class = individual todo item
  - title (required, non-empty)
  - description (optional)
  - priority (1-10, defaults to 5)
  - status ("pending" or "completed")
  - done (True/False boolean)
  - Methods: mark_complete(), update_priority(new_priority)
  - Methods: __repr__() for display, __eq__() for comparison

- TaskManager class = manages multiple Task objects
  - tasks list (collection of Task instances)
  - Methods: add_task(task), list_tasks(), find_by_title(title)
  - Methods: remove_task(task), maybe filter_by_priority(priority)

- Validation:
  - Priority must be 1-10 (enforce in update_priority method)
  - Title can't be empty
  - Status/done track completion state

- Design principles:
  - Task HAS-A data (title, priority, etc.)
  - TaskManager HAS-A collection of Tasks (composition)
  - Methods modify state safely (encapsulation)

**Key Learning**: You planned the ARCHITECTURE through conversation, not by writing formal specs. That skill comes later in Part 5.

**Deliverable**: Simple task system design notes from your AI conversation. You've planned the architecture - now AI will help generate the implementation.

---

## Part 2: Generate Code and Validate Design

**Your Role**: Design validator working with AI to generate and refine the system

### AI-Assisted Implementation

Instead of typing 200+ lines manually, you'll have AI generate the system, then YOU validate it against your design plan.

#### 💬 AI CoLearning Prompt - Generate Task and TaskManager Classes

> "Based on my task management design plan, generate the complete Python code:
>
> **Task Class Requirements:**
> - Attributes: title (str), description (str, default ""), priority (int, default 5), status (str, default "pending"), done (bool, default False), due_date (optional)
> - Constructor: __init__(self, title: str, description: str = "", priority: int = 5)
> - Methods:
>   - mark_complete() → sets done to True, status to "completed"
>   - update_priority(new_priority: int) → validates 1-10, raises ValueError if invalid
>   - __repr__() → returns readable string like "[✓] Buy milk (P2)" or "[○] Buy milk (P2)"
>   - __eq__(other) → compares tasks by title
> - Use type hints on all methods
> - Use docstrings for all methods
>
> **TaskManager Class Requirements:**
> - Attributes: tasks (list of Task objects, empty initially)
> - Methods:
>   - add_task(task: Task) → adds task to list
>   - list_tasks() → returns copy of tasks list
>   - find_by_title(title: str) → returns Task if found, None otherwise
>   - remove_task(task: Task) → removes from list, returns True if found
>   - count_completed() → returns number of done tasks
> - Use type hints and docstrings
>
> Generate complete working code. I'll validate it against my design."

**Expected AI Output**: ~80-120 lines of complete code implementing both classes.

---

### Validation Exercise: Check AI's Implementation

**Your task**: Read the AI-generated code and validate these design decisions:

**✅ Validation Checklist:**

1. **Class Structure**:
   - Does Task have all required attributes? ✓
   - Does Task have all required methods? ✓
   - Does TaskManager exist as separate class? ✓
   - Is TaskManager's tasks list initialized as empty list? ✓

2. **Encapsulation (Lesson 4 concept)**:
   - Does update_priority validate that 1 is less than or equal to priority and priority is less than or equal to 10? ✓
   - Does update_priority raise ValueError with clear message? ✓
   - Does mark_complete set both done and status correctly? ✓

3. **String Representation**:
   - Does __repr__ show meaningful info (title, done status, priority)? ✓
   - Does __repr__ use symbols like ✓ and ○ for visibility? ✓
   - Does __eq__ compare by title (two tasks with same title are equal)? ✓

4. **Type Hints (Chapter 27 standard)**:
   - Does every method have parameter type hints? ✓
   - Does every method have return type hints? ✓
   - Does Task.__eq__ return bool? ✓

5. **TaskManager Composition**:
   - Does TaskManager HAS-A collection of Task objects? ✓
   - Do methods return appropriate types (Task | None, list[Task], bool)? ✓

6. **Edge Cases (from Part 1 planning)**:
   - Does find_by_title return None if not found? ✓
   - Does list_tasks return copy (not reference to internal list)? ✓
   - Does update_priority handle invalid values with exception? ✓

---

### Example Code (What AI Generated)

```python
class Task:
    """A single todo item with validation and state management."""

    def __init__(self, title: str, description: str = "", priority: int = 5):
        if not title or not title.strip():
            raise ValueError("Task title cannot be empty")
        self.title = title
        self.description = description
        self.priority = priority
        self.status = "pending"
        self.done = False
        self.due_date = None

    def mark_complete(self) -> None:
        """Mark task as completed."""
        self.done = True
        self.status = "completed"

    def mark_pending(self) -> None:
        """Mark task as pending (undo completion)."""
        self.done = False
        self.status = "pending"

    def update_priority(self, new_priority: int) -> None:
        """Update task priority (1=highest, 10=lowest).

        Args:
            new_priority: Integer between 1 and 10

        Raises:
            ValueError: If priority outside valid range
        """
        if not isinstance(new_priority, int) or new_priority < 1 or new_priority > 10:
            raise ValueError("Priority must be integer between 1 and 10")
        self.priority = new_priority

    def __repr__(self) -> str:
        """Return readable string representation."""
        status = "✓" if self.done else "○"
        return f"[{status}] {self.title} (P{self.priority})"

    def __eq__(self, other) -> bool:
        """Two tasks are equal if they have same title."""
        if not isinstance(other, Task):
            return False
        return self.title == other.title


class TaskManager:
    """Manages a collection of Task objects."""

    def __init__(self):
        """Initialize empty task collection."""
        self.tasks: list[Task] = []

    def add_task(self, task: Task) -> None:
        """Add task to collection.

        Args:
            task: Task object to add
        """
        if not isinstance(task, Task):
            raise TypeError("Can only add Task objects")
        self.tasks.append(task)

    def list_tasks(self) -> list[Task]:
        """Return copy of all tasks.

        Returns:
            List of Task objects
        """
        return self.tasks.copy()

    def find_by_title(self, title: str) -> Task | None:
        """Find first task with matching title.

        Args:
            title: Task title to search for

        Returns:
            Task if found, None otherwise
        """
        for task in self.tasks:
            if task.title == title:
                return task
        return None

    def remove_task(self, task: Task) -> bool:
        """Remove task from collection.

        Args:
            task: Task to remove

        Returns:
            True if task was removed, False if not found
        """
        if task in self.tasks:
            self.tasks.remove(task)
            return True
        return False

    def count_completed(self) -> int:
        """Count tasks marked as completed.

        Returns:
            Number of completed tasks
        """
        return sum(1 for task in self.tasks if task.done)

    def filter_by_priority(self, priority: int) -> list[Task]:
        """Get all tasks with specified priority.

        Args:
            priority: Priority level to filter by

        Returns:
            List of matching tasks
        """
        return [task for task in self.tasks if task.priority == priority]
```

**Output Examples:**

```python
# Create tasks
task1 = Task("Buy groceries", "Milk, eggs, bread", priority=2)
task2 = Task("Review PR", "Check for code quality", priority=1)
task3 = Task("Study Python", "Complete Chapter 27", priority=5)

# Display tasks
print(task1)  # Output: [○] Buy groceries (P2)
print(task2)  # Output: [○] Review PR (P1)

# Mark complete
task1.mark_complete()
print(task1)  # Output: [✓] Buy groceries (P2)

# Update priority
task3.update_priority(1)
print(task3)  # Output: [○] Study Python (P1)

# Validate priority
try:
    task2.update_priority(15)  # Invalid!
except ValueError as e:
    print(f"Error: {e}")  # Output: Error: Priority must be integer between 1 and 10

# TaskManager composition
manager = TaskManager()
manager.add_task(task1)
manager.add_task(task2)
manager.add_task(task3)

# List all tasks
for task in manager.list_tasks():
    print(task)
# Output:
# [✓] Buy groceries (P2)
# [○] Review PR (P1)
# [○] Study Python (P1)

# Find task
found = manager.find_by_title("Review PR")
print(found)  # Output: [○] Review PR (P1)

# Filter and count
priority_1_tasks = manager.filter_by_priority(1)
print(f"High priority tasks: {len(priority_1_tasks)}")  # Output: High priority tasks: 2

completed_count = manager.count_completed()
print(f"Completed: {completed_count}/3")  # Output: Completed: 1/3
```

---

### Challenge AI with Edge Cases

#### 💬 AI CoLearning Prompt - Validate Edge Cases

> "Validate your code handles these edge cases:
>
> **Test 1: Empty Title Prevention**
> ```python
> try:
>     bad_task = Task("")
> except ValueError as e:
>     print(f"Caught error: {e}")
> ```
>
> **Test 2: Priority Validation**
> ```python
> task = Task("Important", priority=5)
> try:
>     task.update_priority(0)  # Below range
> except ValueError:
>     print("Prevented invalid priority")
> ```
>
> **Test 3: Equality Comparison**
> ```python
> task1 = Task("Buy milk")
> task2 = Task("Buy milk")
> task3 = Task("Buy bread")
> print(task1 == task2)  # Should be True (same title)
> print(task1 == task3)  # Should be False (different title)
> ```
>
> **Test 4: TaskManager with Empty List**
> ```python
> manager = TaskManager()
> print(manager.find_by_title("Nonexistent"))  # Should return None
> print(manager.count_completed())  # Should return 0
> print(len(manager.list_tasks()))  # Should return 0
> ```
>
> **Test 5: Remove Non-existent Task**
> ```python
> manager = TaskManager()
> task = Task("Clean room")
> result = manager.remove_task(task)  # Should return False
> print(result)
> ```
>
> Run these tests and show me output. Do they all work as expected?"

**Expected Learning**: You validate AI's implementation through testing, not by reading every line. Professional workflow = specify → generate → validate.

**Your task**: Write and test the implementation:
- Create Task objects with valid and invalid parameters
- Create TaskManager and add multiple tasks
- Test find_by_title, filter_by_priority, count_completed
- Test edge cases (empty title, invalid priority, nonexistent tasks)
- Document what works and what needs refinement

**Deliverable**: Fully functional task system passing all validation tests.

---

## Part 3: Challenge AI with "What If" Extensions

**Your Role**: System designer testing design flexibility

### Extension Challenge Exercise

Now that you have a working system, challenge AI to extend it. Can the design handle new features without major refactoring?

#### 💬 AI CoLearning Prompt - Add Recurring Tasks

> "I want to add recurring tasks to my system. Recurring tasks repeat on a schedule (daily, weekly, monthly):
>
> **Requirements:**
> - Recurring task repeats until marked complete or cancelled
> - Has: title, description, priority, recurrence_pattern ('daily', 'weekly', 'monthly')
> - Methods: same as Task, plus next_occurrence() that returns when task recurs
> - When marked complete, should create a new instance for next occurrence
>
> Design options:
> 1. Add recurrence_pattern to Task class (all tasks have it, even if null)?
> 2. Create RecurringTask subclass inheriting from Task?
> 3. Create separate Task and RecurringTask classes both managed by TaskManager?
>
> Which design is better? Show me the code for your recommendation. Explain trade-offs."

**Expected Learning**: AI will show you how to extend your system through inheritance or composition. You'll see if your original design was flexible.

---

#### 💬 AI CoLearning Prompt - Priority Filtering and Sorting

> "I want to display tasks sorted by priority (highest first). Currently I have filter_by_priority() which returns a list.
>
> **Options:**
> 1. Add get_high_priority_tasks() method returning tasks with priority 1-3?
> 2. Add sort_by_priority() to TaskManager returning sorted list?
> 3. Implement custom comparator for Task so I can use Python's sorted() function?
>
> Which is most Pythonic? Show me code for the best approach. What would production code do?"

**Expected Learning**: AI introduces Python's built-in sorting mechanisms and design patterns for filtering/sorting collections.

---

#### 💬 AI CoLearning Prompt - Add Due Date Support

> "I want tasks to have due dates and show overdue status:
> - Task has optional due_date (datetime or None)
> - is_overdue() method returns True if done=False and due_date < today
> - TaskManager has get_overdue_tasks() returning tasks past deadline
>
> How should due dates affect priority sorting? Should overdue tasks always sort first? Show me implementation with datetime usage."

**Expected Learning**: You see how object attributes and methods interact with real-world domain concepts (dates, deadlines).

---

### Your Extensions

Implement 1-2 new features with AI assistance:
- Recurring tasks (inheritance approach with RecurringTask subclass)
- Priority-based sorting (custom methods or sorting support)
- Due date support (datetime integration)
- Task templates (create common tasks from patterns)

**Deliverable**: Extended task system with 1-2 new features generated and validated through AI.

---

## Part 4: Build Your OOP Design Framework

**Your Role**: Experienced designer extracting lessons learned

### Reflection Exercise

After building and extending your system with AI, extract the **design patterns** you learned.

#### 💬 AI CoLearning Prompt - Synthesize Design Patterns

> "Based on my Task Management System project, help me create a design guide for future OOP projects:
>
> **What I learned:**
> 1. When to create separate classes vs add attributes to existing class
> 2. When to use composition (TaskManager HAS-A list of Tasks) vs inheritance (RecurringTask IS-A Task)
> 3. How to design for validation and prevent invalid states
> 4. How to use __repr__ and __eq__ for meaningful object behavior
> 5. How to structure classes so they're easy to extend
>
> Give me a framework with:
> - When to use each design pattern
> - Common mistakes to avoid
> - Quick decision guide ('If X, use Y')
> - Examples from my task system
>
> Use my actual project as examples throughout."

**Expected AI Output**: A personalized OOP design guide based on your project experience.

---

### Your OOP Design Framework

Save AI's framework as your reference. Example structure:

**My OOP Design Guide (From Task System Project):**

**Single Responsibility Principle:**
- Task class = manages single task (title, priority, completion, validation)
- TaskManager class = manages collection of tasks (add, find, list, filter)
- Each class has one reason to change

**When to Use Composition (HAS-A):**
- TaskManager HAS-A list of Task objects
- "Does X contain Y?" → Yes, use composition
- Composition is more flexible than inheritance
- Example: RecurringTask HAS-A base Task pattern

**When to Use Inheritance (IS-A):**
- RecurringTask IS-A Task (shares all Task behavior)
- "Is X a specialized version of Y?" → Yes, use inheritance
- Subclasses inherit constructor, methods, validation
- Example: RecurringTask adds recurrence_pattern, next_occurrence()

**String Representation and Comparison:**
- __repr__() makes Task display nicely: "[✓] Title (P5)"
- __eq__() enables "task1 == task2" comparison by meaningful attribute
- Essential for debugging and testing

**Design for Extensibility:**
- Use clear method names and consistent signatures
- Design validation in constructors and methods (update_priority checks range)
- Use type hints so subclasses know expected types
- Composition is often more flexible than inheritance

**Common Mistakes I Avoided:**
- Making everything inherit (sometimes composition is simpler)
- Forgetting validation (prevented invalid states)
- Not planning for extensions (able to add recurring tasks easily)
- Exposing internal data directly (return .copy() for lists)

**Deliverable**: Personal OOP design framework extracted from your project. You've learned by building, not just reading.

---

## Part 5: AI Validation and Professional Polish

**Your Role**: Engineer ensuring production quality

### Final Validation Exercise

Ask AI:

> "Final review: Does my Task Management System demonstrate:
> 1. Proper use of OOP principles (encapsulation, composition, type hints)?
> 2. Professional Python style (naming, docstrings, error handling)?
> 3. Robust validation and edge case handling?
> 4. Clear class responsibilities and method design?
> 5. Extensibility for future features?
>
> Give me a final checklist of what would make this production-ready and how I'm meeting those standards."

---

### Your Final Deliverables

1. **task_system_specification.md** - Full design specification from Part 1
2. **task_manager_system.py** - Complete Task and TaskManager classes
3. **ai_review_feedback.md** - AI's code review and your responses
4. **extended_system.py** - One implemented extension (RecurringTask or DueDate support)
5. **production_checklist.md** - Production-readiness criteria and how you meet them

---

## Real-World Applications

The Task + TaskManager pattern appears everywhere:

**Legal Domain:**
```python
class Case:
    """Legal case with status tracking."""
    def __init__(self, title: str, case_number: str, priority: int = 5):
        self.title = title
        self.case_number = case_number
        self.priority = priority
        self.status = "open"  # Same validation as Task
        self.due_date = None

class CaseManager:
    """Manages law firm's active cases."""
    def find_by_case_number(self, number: str) -> Case | None:
        # Same pattern as TaskManager
        pass
```

**Finance Domain:**
```python
class Invoice:
    """Billing invoice with status and priority."""
    def __init__(self, title: str, amount: float, priority: int = 5):
        self.title = title
        self.amount = amount
        self.priority = priority
        self.status = "pending"  # Same validation pattern

class InvoiceManager:
    """Manages company invoices."""
    def filter_by_status(self, status: str) -> list[Invoice]:
        # Same composition pattern as TaskManager
        pass
```

**Healthcare Domain:**
```python
class Appointment:
    """Medical appointment with scheduling and priority."""
    def __init__(self, title: str, patient_name: str, priority: int = 5):
        self.title = title
        self.patient_name = patient_name
        self.priority = priority
        self.status = "scheduled"

class AppointmentScheduler:
    """Manages clinic's appointments."""
    def find_by_patient(self, name: str) -> list[Appointment]:
        # Same pattern, different domain
        pass
```

**Key Insight**: Once you master the Task + Manager pattern, you can apply it anywhere: Projects + ProjectManager, Tickets + TicketManager, Users + UserManager. This is fundamental OOP architecture.

---

### Transition to Chapter 29

Objects organize your task data beautifully. Your system is properly structured with clear separation of concerns. In **Chapter 29**, you'll learn about **dataclasses**, which are Python's way of making classes like Task even simpler to define while keeping all the power:

```python
# Chapter 27 (what you built now):
class Task:
    def __init__(self, title: str, description: str = ""):
        self.title = title
        self.description = description

# Chapter 29 (coming soon):
@dataclass
class Task:
    title: str
    description: str = ""  # Same structure, cleaner syntax
```

Same functionality, better syntax. Your understanding of classes and objects is the foundation.

---

## Try With AI

Ready to design and build a task management system that demonstrates OOP mastery?

**🏗️ Design the Architecture:**
> "I want to build a task management system. Help me design: should I have a Task class for individual items and a TaskManager class for the collection? What methods does each need? What validation should prevent invalid tasks? Draw me a simple class diagram or structure."

**What you're learning:** Architectural thinking - how to organize classes and their relationships before writing code. This is how professionals approach OOP projects.

---

**💻 Build the Foundation:**
> "Build me a Task class with: title (required), description (optional), priority (1-10, defaults to 5), and done status (boolean). Include an __init__ method, a mark_complete() method, and a __repr__() that shows '[✓] Title (P5)' or '[○] Title (P5)'. Use type hints and validation in update_priority() method. Show test code creating two tasks and marking one complete."

**What you're learning:** Class structure with proper initialization, validation, and methods that safely modify state. This is encapsulation in practice.

---

**🔍 Extend the System:**
> "Add a TaskManager class that manages a collection of Task objects. Include: add_task(task), list_tasks(), find_by_title(title) returning Task or None, count_completed() showing how many tasks are done. Show me code that creates a TaskManager, adds 3 tasks, finds one by title, marks it complete, and shows the count."

**What you're learning:** Composition pattern - TaskManager HAS-A collection of Task objects. This shows how real systems are built from interacting classes.

---

**⚙️ Validate Design Decisions:**
> "Test my Task system for edge cases: (1) Creating task with empty title should fail, (2) Priority 15 should raise error, (3) Two tasks with same title should be equal (task1 == task2), (4) Marking complete should set both done=True and status='completed'. Show me test code and expected outputs."

**What you're learning:** Design validation through testing. Professional workflow = design → generate → test → refine.

---

**🚀 Plan an Extension:**
> "I want to add a RecurringTask that repeats daily/weekly/monthly. Should RecurringTask inherit from Task or be separate? What new methods does it need? How does TaskManager handle both regular and recurring tasks? Show me a design comparison of inheritance vs composition for this feature."

**What you're learning:** Advanced design patterns and trade-offs. How good designs remain flexible when requirements change.

---
