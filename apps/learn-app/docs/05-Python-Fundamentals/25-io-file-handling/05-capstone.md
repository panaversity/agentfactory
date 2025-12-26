---
title: "Capstone - Todo Data Manager"
chapter: 25
lesson: 5
duration_minutes: 90
estimated_total_time: 120

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Integrating Multiple I/O Concepts in Single Application"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design and build an application using console I/O, file I/O, path handling, and structured formats together"

  - name: "Implementing CRUD Operations (Create, Read, Update, Delete)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement all CRUD operations for file-based data storage with proper error handling"

  - name: "Designing Menu-Driven Interactive Loops"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design loop structures that display menu, accept input, execute action, and return to menu"

  - name: "Implementing Search and Filter Operations"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can search data structures by keyword and filter results based on criteria"

  - name: "Application-Level Error Handling"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can handle errors at multiple levels (user input, file operations, JSON parsing) and recover gracefully"

  - name: "Data Validation at Application Scale"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can validate user input, file structure, and application state to maintain consistency"

  - name: "Organizing Code for Maintainability"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can structure application with functions, clear separation of concerns, and reusable components"

  - name: "Date Serialization in JSON"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can serialize and deserialize dates in JSON format, handling timezone-aware datetime objects"

# Learning objectives
learning_objectives:
  - objective: "Integrate all I/O concepts (console, file, pathlib, JSON) into a complete, production-quality CLI todo application"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Build working Todo Data Manager with all CRUD operations"

  - objective: "Implement Create, Read, Update, Delete operations for file-based data persistence with proper JSON serialization"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Demonstrate each operation working correctly with data saved to files"

  - objective: "Design menu-driven interactive loops with proper input validation and error recovery"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Test menu loop handling valid/invalid input, returning to menu after each operation"

  - objective: "Search and filter data structures to find tasks by keyword and priority"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Implement search function that returns matching tasks"

  - objective: "Handle errors at application scale (user input, file operations, JSON parsing)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Test application with edge cases (missing files, invalid input, corrupted data)"

  - objective: "Serialize date objects in JSON format and deserialize them back to Python datetime objects"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Demonstrate tasks with due_date persisting correctly across save/load cycles"

# Cognitive load tracking
cognitive_load:
  new_concepts: 0
  assessment: "0 new concepts (integration of Lessons 1-4 only) - appropriate for B1 capstone synthesis"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Add task filtering by priority, implement task status transitions (pending→in_progress→completed), add due date sorting, implement task categories with nested structure, create CSV export feature, implement undo/redo with history"
  remedial_for_struggling: "Provide starter code with menu loop and function stubs, focus on one CRUD operation at a time, use simpler JSON structure without due dates initially, work through creating a single task before attempting full app"

# Generation metadata
generated_by: "content-implementer v4.0.0"
source_spec: "Task #046 - Todo Integration Capstone"
created: "2025-12-26"
last_modified: "2025-12-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Capstone - Todo Data Manager

## Introduction: Building a Production CLI Todo Application

Welcome to the capstone project for Chapter 25! Everything you've learned in Lessons 1-4 comes together here. You're going to build a complete, production-quality CLI todo management application that integrates:

- **Lesson 1**: Console I/O with input validation (menu-driven interface)
- **Lesson 2**: Safe file operations (reading and writing tasks)
- **Lesson 3**: Cross-platform paths (organizing tasks in files)
- **Lesson 4**: JSON data format (persisting structured task data)

This isn't a toy project—it's a real application that demonstrates professional-level coding practices. When you're done, you'll have a program that:

- Runs without crashing, even with invalid user input
- Persists task data to disk in organized JSON files
- Handles multiple tasks with responsive performance
- Recovers gracefully from common errors (missing files, corrupted data)
- Organizes code clearly so others can read and extend it

**Time Estimate**: 90-120 minutes of focused development (plus extension ideas if you want to go deeper)

**What You'll Build**: A fully functional Todo Data Manager with these features:
1. **Menu-driven interface** - Display options, accept user choice, execute action, return to menu
2. **Create tasks** - Prompt for title, description, priority, and due date
3. **Read tasks** - List existing tasks, display selected task details
4. **Update tasks** - Modify title, description, priority, or status
5. **Delete tasks** - Remove tasks with confirmation
6. **Search tasks** - Find tasks by keyword across title and description
7. **List tasks** - Show all tasks organized by priority or status
8. **Exit gracefully** - Clean shutdown with goodbye message

## Application Architecture Introduction

Before writing code, let's understand the design. A well-architected application separates concerns:

### Components

**Menu Loop** (Lesson 1)
- Displays menu options
- Accepts user choice
- Routes to appropriate function
- Returns to menu after operation

**CRUD Functions** (Lessons 2-4)
- `get_all_tasks()` → Load all tasks from disk
- `save_task()` → Write or update a task to disk
- `search_tasks()` → Find tasks by keyword
- `delete_task()` → Remove task file

**Data Structure** (Lesson 4)
```json
{
  "id": 1,
  "title": "Review PR",
  "description": "Check for code quality",
  "due_date": "2025-01-15",
  "priority": 2,
  "status": "pending",
  "done": false
}
```

**File Organization** (Lesson 3)
```
tasks/
├── tasks.json
└── backup/
    └── tasks_backup.json
```

### Why This Design?

- **Separation of Concerns**: Menu loop doesn't know about file operations; functions don't know about UI
- **Scalability**: JSON file-based storage handles 10-100 tasks without performance issues
- **Maintainability**: Clear functions mean future extensions are easy
- **Testability**: Each function can be tested independently

#### 💬 AI Colearning Prompt

> "I'm designing a todo app. Should I store all tasks in a single JSON file or separate files per task? What are the tradeoffs? How do I serialize datetime objects to JSON?"

**Expected Outcome**: You'll understand data persistence patterns and JSON serialization before writing code. AI can help you think through design decisions before implementation.

---

## Setting Up Project Structure

Every good application starts with organization. Let's initialize the directory structure and prepare for task data storage.

### Creating the Tasks Directory

First, we ensure the `tasks/` directory exists and is ready to store our JSON data:

```python
from pathlib import Path

# Setup
BASE_DIR: Path = Path("tasks")
BASE_DIR.mkdir(exist_ok=True)

# Create data file path for all tasks
TASKS_FILE: Path = BASE_DIR / "tasks.json"

print("✅ Todo data manager initialized!")
print(f"Data file: {TASKS_FILE.resolve()}")
```

**Output:**
```
✅ Todo data manager initialized!
Data file: /home/user/tasks/tasks.json
```

**Why this matters**:
- Uses **pathlib** (Lesson 3) to create cross-platform paths
- Creates directories only if they don't exist (idempotent)
- Resolves absolute path to show user exactly where data is stored
- Single JSON file is simpler for small-to-medium task lists

#### 🎓 Expert Insight

> In AI-native development, you don't debug path errors manually—you design paths clearly at startup. Your job: specify the directory structure. AI can help you verify it's correct with `Path.resolve()`. Also, notice that we're using ISO format strings ("2025-01-15") for due_date instead of datetime objects—this makes JSON serialization automatic.

---

## Code Example 5.1: Project Structure and Data Initialization

Before we write the full app, let's look at how to initialize the project with proper JSON validation:

```python
from pathlib import Path
import json
from datetime import datetime

# Setup
BASE_DIR: Path = Path("tasks")
BASE_DIR.mkdir(exist_ok=True)

TASKS_FILE: Path = BASE_DIR / "tasks.json"

def initialize_tasks_file() -> None:
    """
    Create tasks.json if it doesn't exist.

    If file already exists, validates it's valid JSON.
    If missing, creates empty array.
    """
    if not TASKS_FILE.exists():
        with open(TASKS_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f, indent=2)
        print(f"✅ Created {TASKS_FILE}")
    else:
        # Validate existing file is valid JSON
        try:
            with open(TASKS_FILE, 'r', encoding='utf-8') as f:
                json.load(f)
            print(f"✅ Tasks file loaded: {TASKS_FILE}")
        except json.JSONDecodeError:
            print(f"⚠️  Tasks file corrupted. Creating backup and resetting...")
            backup_file = TASKS_FILE.with_stem(f"{TASKS_FILE.stem}_backup")
            TASKS_FILE.rename(backup_file)
            with open(TASKS_FILE, 'w', encoding='utf-8') as f:
                json.dump([], f, indent=2)

# Call initialization
initialize_tasks_file()

# Example: Create a sample task
task: dict = {
    "id": 1,
    "title": "Review Chapter 25 Material",
    "description": "Complete the I/O file handling lessons",
    "due_date": "2025-01-15",  # ISO format string, not datetime object
    "priority": 2,
    "status": "pending",
    "done": False
}

print(f"Sample task: {json.dumps(task, indent=2)}")
```

**Specification Reference**: Lesson 4 (JSON) with error handling from Chapter 24 exception patterns

**Prompt Used**: "Show me how to initialize a JSON file with validation and handle corrupted data by creating a backup"

**Validation Steps**:
1. ✅ File doesn't exist → `initialize_tasks_file()` creates empty array
2. ✅ File exists and is valid → Loads without error
3. ✅ File corrupted → Creates backup, resets with empty array
4. ✅ Task serializes to JSON correctly with all fields

---

## Core CRUD Functions

Now let's implement the functions that handle data persistence. These are the workhorses of the application—they orchestrate file I/O, error handling, and JSON operations.

### Code Example 5.2: Complete CRUD Functions

```python
from pathlib import Path
import json
from typing import Optional
import os
from datetime import datetime

BASE_DIR: Path = Path("tasks")
TASKS_FILE: Path = BASE_DIR / "tasks.json"

def get_all_tasks() -> list[dict]:
    """
    Load all tasks from the tasks.json file.

    Returns empty list if file doesn't exist (first run).
    Handles corrupted files gracefully with error messages.
    """
    try:
        if not TASKS_FILE.exists():
            return []

        with open(TASKS_FILE, 'r', encoding='utf-8') as f:
            tasks = json.load(f)
            return tasks if isinstance(tasks, list) else []
    except json.JSONDecodeError:
        print(f"⚠️  Warning: Tasks file corrupted - skipping")
        return []
    except Exception as e:
        print(f"⚠️  Error reading tasks: {e}")
        return []


def save_tasks(tasks: list[dict]) -> bool:
    """
    Save all tasks to the tasks.json file.

    Takes a list of task dictionaries and writes to JSON file with
    UTF-8 encoding and pretty formatting.
    """
    try:
        BASE_DIR.mkdir(parents=True, exist_ok=True)

        with open(TASKS_FILE, 'w', encoding='utf-8') as f:
            json.dump(tasks, f, indent=2, ensure_ascii=False)

        return True
    except Exception as e:
        print(f"❌ Error saving tasks: {e}")
        return False


def save_task(task: dict) -> bool:
    """
    Save or update a single task by loading all tasks, updating, and saving.

    This pattern is simpler for small task lists (10-100 items).
    For larger lists, you'd want database or file-per-task approach.
    """
    try:
        tasks = get_all_tasks()

        # Check if task with this ID already exists
        found = False
        for i, existing_task in enumerate(tasks):
            if existing_task.get("id") == task.get("id"):
                tasks[i] = task
                found = True
                break

        # If not found, add as new task
        if not found:
            tasks.append(task)

        return save_tasks(tasks)
    except Exception as e:
        print(f"❌ Error saving task: {e}")
        return False


def search_tasks(keyword: str) -> list[dict]:
    """
    Find all tasks containing keyword in title or description (case-insensitive).

    Loads all tasks, filters by searching title and description fields,
    returns list of matching tasks.
    """
    all_tasks = get_all_tasks()
    keyword_lower = keyword.lower()

    matching: list[dict] = [
        task for task in all_tasks
        if keyword_lower in task.get("title", "").lower() or
           keyword_lower in task.get("description", "").lower()
    ]

    return matching


def delete_task(task_id: int) -> bool:
    """
    Delete a task by ID.

    Loads all tasks, removes matching task, and saves the updated list.
    Returns True if successful, False if task not found.
    """
    try:
        tasks = get_all_tasks()
        original_count = len(tasks)

        tasks = [task for task in tasks if task.get("id") != task_id]

        if len(tasks) < original_count:
            # Task was found and removed
            return save_tasks(tasks)
        else:
            # Task not found
            return False
    except Exception as e:
        print(f"❌ Error deleting task: {e}")
        return False


def get_task_by_id(task_id: int) -> Optional[dict]:
    """Helper: Find a single task by ID."""
    all_tasks = get_all_tasks()
    for task in all_tasks:
        if task.get("id") == task_id:
            return task
    return None


def get_next_task_id() -> int:
    """Helper: Get the next available task ID."""
    tasks = get_all_tasks()
    if not tasks:
        return 1
    return max(task.get("id", 0) for task in tasks) + 1
```

**Specification Reference**: Lessons 2, 3, 4 - file I/O, pathlib directory handling, JSON serialization and deserialization

**Prompts Used**:
1. "Write a function that loads JSON data from a file and handles FileNotFoundError and JSONDecodeError"
2. "Implement save_tasks that writes a list of dictionaries to JSON with UTF-8 encoding"
3. "Write a search function that finds tasks by keyword in title or description"

**Validation Steps**:
1. ✅ Load from empty file → `get_all_tasks()` returns empty list
2. ✅ Save new task → file updated with correct JSON structure
3. ✅ Search finds matches → `search_tasks("python")` returns tasks with "python" in title/description
4. ✅ Delete removes task → `delete_task(id)` removes task and returns True
5. ✅ Error handling → corrupted JSON shows warning but doesn't crash
6. ✅ Date strings preserved → ISO format dates survive save/load cycle

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "My todo app is slow loading 1000 tasks—it loads everything every time. How would you optimize this? Should I use a database? When should I move away from JSON files? What's the performance limit for this approach?"

**Expected Outcome**: You'll understand performance trade-offs and scaling patterns beyond this capstone's 10-100 task scope.

---

## Menu Loop Implementation

The menu loop is where user interaction happens. It displays options, accepts input with validation, routes to appropriate functions, and returns to menu after each operation.

### Code Example 5.3: Complete Application Menu Loop

```python
from pathlib import Path
import json
from datetime import datetime

# [CRUD functions from Example 5.2 would be included here]

def create_task() -> None:
    """Prompt for new task and save."""
    print("--- Create New Task ---")
    title: str = input("Task title: ").strip()
    description: str = input("Task description: ").strip()

    # Get priority with validation
    while True:
        try:
            priority: int = int(input("Priority (1=high, 5=normal, 10=low) [5]: ").strip() or "5")
            if 1 <= priority <= 10:
                break
            print("❌ Priority must be between 1 and 10.")
        except ValueError:
            print("❌ Please enter a number.")

    # Get due date (optional, ISO format)
    due_date_str: str = input("Due date (YYYY-MM-DD) or press Enter: ").strip() or None

    # Validate date format if provided
    if due_date_str:
        try:
            datetime.strptime(due_date_str, "%Y-%m-%d")
        except ValueError:
            print("❌ Invalid date format. Use YYYY-MM-DD.")
            return

    # Validate input
    if not title:
        print("❌ Error: Title cannot be empty.")
        return

    # Create task dictionary
    task: dict = {
        "id": get_next_task_id(),
        "title": title,
        "description": description or "",
        "due_date": due_date_str,
        "priority": priority,
        "status": "pending",
        "done": False
    }

    # Save task
    if save_task(task):
        print(f"✅ Task created successfully! ID: {task['id']}")
    else:
        print("❌ Error creating task.")


def read_task() -> None:
    """List tasks and display selected task."""
    tasks = get_all_tasks()

    if not tasks:
        print("📋 No tasks found.")
        return

    # Display list
    print("--- Your Tasks ---")
    for i, task in enumerate(tasks, 1):
        status = "✅" if task.get("done") else "⏳"
        due = f" (Due: {task.get('due_date')})" if task.get('due_date') else ""
        print(f"{i}. [{status}] {task['title']} (Priority: {task['priority']}){due}")

    # Get selection
    try:
        choice = int(input("Select task (number): ")) - 1
        if 0 <= choice < len(tasks):
            task = tasks[choice]
            print(f"\n📄 {task['title']}")
            print(f"Description: {task.get('description', '(none)')}")
            print(f"Priority: {task['priority']}")
            print(f"Status: {task.get('status', 'pending')}")
            print(f"Done: {'Yes' if task.get('done') else 'No'}")
            if task.get('due_date'):
                print(f"Due: {task['due_date']}")
        else:
            print("❌ Invalid selection.")
    except ValueError:
        print("❌ Please enter a number.")


def update_task() -> None:
    """Update title, description, priority, or status of existing task."""
    tasks = get_all_tasks()

    if not tasks:
        print("📋 No tasks found.")
        return

    # Display list
    print("--- Select Task to Update ---")
    for i, task in enumerate(tasks, 1):
        print(f"{i}. {task['title']}")

    # Get selection
    try:
        choice = int(input("Select task (number): ")) - 1
        if 0 <= choice < len(tasks):
            task = tasks[choice]

            # Prompt for updates
            new_title = input(f"New title [{task['title']}]: ").strip() or task['title']
            new_description = input(f"New description [{task.get('description', '')}]: ").strip() or task.get('description', '')
            new_priority = input(f"New priority [{task['priority']}]: ").strip()

            # Update task
            task['title'] = new_title
            task['description'] = new_description
            if new_priority:
                try:
                    task['priority'] = int(new_priority)
                except ValueError:
                    print("⚠️  Invalid priority, keeping original.")

            if save_task(task):
                print("✅ Task updated successfully!")
            else:
                print("❌ Error updating task.")
        else:
            print("❌ Invalid selection.")
    except ValueError:
        print("❌ Please enter a number.")


def delete_task_menu() -> None:
    """Delete task with confirmation."""
    tasks = get_all_tasks()

    if not tasks:
        print("📋 No tasks found.")
        return

    # Display list
    print("--- Select Task to Delete ---")
    for i, task in enumerate(tasks, 1):
        print(f"{i}. {task['title']}")

    # Get selection
    try:
        choice = int(input("Select task (number): ")) - 1
        if 0 <= choice < len(tasks):
            task = tasks[choice]

            # Confirm deletion
            confirm = input(f"Delete '{task['title']}'? (yes/no): ").strip().lower()
            if confirm == 'yes':
                if delete_task(task.get('id')):
                    print("✅ Task deleted successfully!")
                else:
                    print("❌ Error deleting task.")
            else:
                print("Cancelled.")
        else:
            print("❌ Invalid selection.")
    except ValueError:
        print("❌ Please enter a number.")


def search_tasks_menu() -> None:
    """Search tasks by keyword."""
    keyword: str = input("Search keyword: ").strip()

    if not keyword:
        print("❌ Please enter a search term.")
        return

    results = search_tasks(keyword)

    if results:
        print(f"--- Found {len(results)} task(s) ---")
        for task in results:
            priority_label = "HIGH" if task['priority'] <= 3 else "NORMAL" if task['priority'] <= 7 else "LOW"
            print(f"• {task['title']} (Priority: {priority_label})")
    else:
        print(f"No tasks found matching '{keyword}'")


def list_all_tasks() -> None:
    """Display all tasks organized by priority."""
    tasks = get_all_tasks()

    if not tasks:
        print("📋 No tasks found.")
        return

    # Sort by priority
    sorted_tasks = sorted(tasks, key=lambda t: t.get('priority', 5))

    print("--- All Tasks (sorted by priority) ---")
    for task in sorted_tasks:
        status = "✅" if task.get('done') else "⏳"
        due = f" (Due: {task.get('due_date')})" if task.get('due_date') else ""
        print(f"[{status}] {task['title']} (Priority: {task['priority']}){due}")


def mark_task_complete() -> None:
    """Mark a task as completed."""
    tasks = get_all_tasks()

    if not tasks:
        print("📋 No tasks found.")
        return

    print("--- Select Task to Complete ---")
    for i, task in enumerate(tasks, 1):
        status = "✅" if task.get('done') else "⏳"
        print(f"{i}. [{status}] {task['title']}")

    try:
        choice = int(input("Select task (number): ")) - 1
        if 0 <= choice < len(tasks):
            task = tasks[choice]
            task['done'] = True
            task['status'] = 'completed'

            if save_task(task):
                print(f"✅ Task '{task['title']}' marked as complete!")
            else:
                print("❌ Error updating task.")
        else:
            print("❌ Invalid selection.")
    except ValueError:
        print("❌ Please enter a number.")


def main() -> None:
    """Main application loop."""
    print("="*50)
    print("✅ Welcome to Todo Data Manager!")
    print("="*50)

    # Initialize data file
    BASE_DIR: Path = Path("tasks")
    BASE_DIR.mkdir(exist_ok=True)

    # Ensure tasks.json exists
    if not TASKS_FILE.exists():
        with open(TASKS_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f)

    while True:
        # Display menu
        print("\n" + "="*50)
        print("1. Create Task")
        print("2. Read Task")
        print("3. Update Task")
        print("4. Delete Task")
        print("5. Mark Complete")
        print("6. Search Tasks")
        print("7. List All Tasks")
        print("8. Exit")
        print("="*50)

        # Get user choice with validation
        try:
            choice_str: str = input("Enter choice (1-8): ").strip()
            choice: int = int(choice_str)

            if choice not in range(1, 9):
                print("❌ Invalid choice. Please enter 1-8.")
                continue

            # Route to appropriate function
            if choice == 1:
                create_task()
            elif choice == 2:
                read_task()
            elif choice == 3:
                update_task()
            elif choice == 4:
                delete_task_menu()
            elif choice == 5:
                mark_task_complete()
            elif choice == 6:
                search_tasks_menu()
            elif choice == 7:
                list_all_tasks()
            elif choice == 8:
                print("👋 Goodbye! Your tasks are saved.")
                break

        except ValueError:
            print("❌ Invalid input. Please enter a number between 1 and 8.")
        except Exception as e:
            print(f"❌ Unexpected error: {e}")


if __name__ == "__main__":
    main()
```

**Specification Reference**: Lessons 1, 2, 3, 4 - all I/O concepts combined with date handling

**Prompts Used**:
1. "Write a menu loop that displays options 1-8 and validates user input"
2. "Create functions for each CRUD operation with error handling"
3. "Implement search that finds tasks by keyword and priority sorting"

**Validation Steps**:
1. ✅ Menu displays and accepts 1-8 → invalid input shows error and re-prompts
2. ✅ Create task → prompts for title/description/priority/due date, saves to file, returns to menu
3. ✅ Read task → lists tasks, accepts selection, displays selected task, returns to menu
4. ✅ Update task → allows changing title/description/priority, saves to file, returns to menu
5. ✅ Delete task → requires confirmation before removing from file
6. ✅ Mark complete → toggles done flag and status, returns to menu
7. ✅ Search → finds tasks by keyword, shows results
8. ✅ List → shows all tasks sorted by priority
9. ✅ Exit → graceful shutdown with goodbye message

#### ✨ Teaching Tip

Use Claude Code to test edge cases in your implementation:

> "What happens if I create a task with an empty title? Or try to delete a task twice? What if I enter an invalid due date format? Show me each error and how my code should handle it."

---

## Error Handling and Validation

Production-quality applications handle errors gracefully. Let's review the validation patterns used throughout:

### Input Validation (Lesson 1 Pattern)
```python
# Menu choice validation
try:
    choice: int = int(input("Enter choice (1-8): ").strip())
    if choice not in range(1, 9):
        print("Invalid choice. Please enter 1-8.")
        continue
except ValueError:
    print("Invalid input. Please enter a number.")
```

### File Operation Errors (Lesson 2 Pattern)
```python
# Handle missing/corrupted files
try:
    with open(TASKS_FILE, 'r', encoding='utf-8') as f:
        tasks = json.load(f)
except FileNotFoundError:
    return []  # No file yet - return empty list
except json.JSONDecodeError:
    print("File is corrupted - returning empty list")
    return []
```

### Date Format Validation
```python
# Validate ISO date format
due_date_str = input("Due date (YYYY-MM-DD): ").strip()
try:
    datetime.strptime(due_date_str, "%Y-%m-%d")
except ValueError:
    print("Invalid format. Use YYYY-MM-DD.")
```

### Path Safety (Lesson 3 Pattern)
```python
# Use pathlib for safe path handling
tasks_file: Path = Path("tasks") / "tasks.json"
tasks_file.parent.mkdir(parents=True, exist_ok=True)
```

### Data Validation
```python
# Ensure required fields exist and have valid values
if not title or not title.strip():
    print("Error: Title cannot be empty.")
    return
if not (1 <= priority <= 10):
    print("Error: Priority must be 1-10.")
    return
```

---

## Testing and Refinement

### How to Test Your Application

1. **Test Create**: Run the app, select "Create Task", enter title/description/priority, verify file is updated
   ```bash
   $ cat tasks/tasks.json
   # Should show task with all fields
   ```

2. **Test Read**: Select "Read Task", choose one from the list, verify it displays correctly

3. **Test Update**: Select "Update Task", change the title/priority, verify file is updated

4. **Test Delete**: Create a task, delete it, verify it's removed from file

5. **Test Search**: Create tasks with different keywords, search for them, verify results

6. **Test Mark Complete**: Create a task, mark it complete, verify "done" is True

7. **Test Menu Loop**: Go through several operations, verify app returns to menu each time

8. **Test Error Handling**:
   - Enter invalid menu choice → should show error and re-prompt
   - Try to read tasks when none exist → should show "No tasks found"
   - Try to delete non-existent task → should handle gracefully
   - Enter invalid date format → should show error
   - Enter non-numeric priority → should show error

### Edge Cases to Consider

- What happens if someone creates a task with special Unicode characters? (UTF-8 should preserve them)
- What if the tasks/ directory is deleted while the app is running? (mkdir will recreate it)
- What if someone has 50 tasks—does the app still respond quickly? (Yes, loading/saving JSON is efficient)
- What if you edit tasks.json manually while the app is running? (Next operation will use latest version from disk)
- What if due_date is in the past? (App accepts it—you could add validation to warn)
- What if priority is outside 1-10 range? (Validation prevents invalid entries)

#### 🎓 Expert Insight

> In AI-native development, you don't test by hand 50 times—you test strategically. Your AI can help generate test cases: "What edge cases should I test for a todo app?" Then you verify each one systematically. Notice how we validate dates using `datetime.strptime`—this is where you delegate format checking to the standard library rather than writing regex patterns.

---

## Project Deliverables

Your complete application should consist of:

1. **Main Application File** (e.g., `todo_app.py` or `main.py`)
   - Imports all necessary modules (pathlib, json, datetime)
   - Defines BASE_DIR and TASKS_FILE paths
   - Initializes data file
   - Implements all CRUD functions
   - Implements menu loop with input validation
   - Has `if __name__ == "__main__"` guard with `main()` call

2. **Data Directory** (`tasks/`)
   - Automatically created when app runs
   - Contains `tasks.json` file
   - Stores all tasks as JSON array

3. **Working Features** - All of these must function:
   - Menu displays correctly and accepts valid input
   - Create task saves to JSON file with all fields
   - Read task loads and displays from file
   - Update task modifies file and saves changes
   - Delete task removes from file after confirmation
   - Mark complete toggles done flag
   - Search finds tasks by keyword
   - List displays tasks sorted by priority
   - Exit closes app gracefully

### Success Criteria Checklist

Your application is complete when:

- ✅ Application runs without crashing on valid input
- ✅ All CRUD operations work correctly
- ✅ User input is validated (menu choices, required fields, date format, priority range)
- ✅ Tasks persist between sessions (data saved to JSON files)
- ✅ Search finds tasks by keyword in title or description
- ✅ Application handles errors gracefully (missing files, corrupted JSON, invalid input)
- ✅ Code is organized with functions for each operation
- ✅ Application supports 10-50 tasks at responsive speed
- ✅ Menu returns to top after each operation
- ✅ Date fields serialize correctly to/from ISO format strings
- ✅ Task status and done fields track state correctly

---

## Try With AI

Build a production-ready todo data management application integrating all Chapter 25 I/O concepts. Focus on data persistence, JSON serialization, and error handling.

**🔍 Explore Data Persistence Architecture:**
> "Design a todo app that persists tasks to JSON. Explain: single file vs file-per-task trade-offs, how to serialize/deserialize date strings in ISO format, JSON structure with id/title/description/due_date/priority/status/done fields, how to load all tasks on startup vs lazy loading, error handling for corrupted JSON, and how context managers ensure file operations complete safely."

**What you're learning**: Understanding how applications balance simplicity (single JSON file) with scalability (multiple files or databases). You'll see why ISO date format strings work better than datetime objects in JSON.

**🎯 Practice CRUD with Date Handling:**
> "Implement todo CRUD: Create task with ISO date validation (YYYY-MM-DD), Read with context managers and JSON parsing, Update by task ID lookup, Delete with confirmation. Handle FileNotFoundError (first run), JSONDecodeError (corrupted file), ValueError (invalid date format), and show user-friendly error messages for each."

**What you're learning**: Building robust applications that anticipate errors and recover gracefully. Notice how we validate dates using `datetime.strptime` before storing—this prevents invalid dates in the JSON.

**🧪 Test Persistence and Edge Cases:**
> "Handle edge cases: corrupted JSON (skip with warning and reset), Ctrl+C during save (context manager ensures cleanup), 100+ tasks in JSON (still fast for small lists), concurrent app instances (warning about data conflicts), past due dates (accept but could warn), and international task titles (ensure_ascii=False in json.dump)."

**What you're learning**: Production-quality applications anticipate failures and user mistakes. The context manager pattern (`with open(...) as f:`) ensures files close properly even if errors occur mid-operation.

**🚀 Apply Complete Integration:**
> "Build full todo app: menu with 8 options (Create/Read/Update/Delete/Mark Complete/Search/List/Exit), single tasks.json file storage, ISO date serialization, priority levels (1-10 scale), task status tracking (pending/in_progress/completed), search filtering, timestamp validation, user input validation throughout, error handling for all file/format errors, and reflection showing how Chapter 25 concepts (pathlib, JSON, context managers, exception handling) integrate into working software."

**What you're learning**: How professional applications are built by combining foundational concepts (file I/O, JSON, paths, validation) into coherent systems. Each piece serves a purpose: pathlib for cross-platform paths, JSON for structured data, context managers for resource cleanup, exception handling for resilience.

---

**Congratulations!** You've completed the Todo Data Manager capstone, integrating all I/O concepts from Chapter 25. Your application demonstrates professional-level CLI development using Python 3.14+, pathlib, JSON serialization, file I/O, menu-driven interaction, and proper error handling. Your implementation proves you understand not just the "how" but the "why" of each design decision. You're now ready to extend this pattern to larger applications with databases, and ready for Chapter 27+ (object-oriented programming with Task classes, etc.).
