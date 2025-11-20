---
title: "Comparative Capstone: Choosing Your AI-Native Workflow"
chapter: 8
lesson_number: 8
proficiency_level: "B1"
estimated_time: "150-180 minutes"
learning_objectives:
  - "Build the same feature in three different AI-native IDEs to understand workflow differences"
  - "Compare Zed, Cursor, and Antigravity across five dimensions: speed, code quality, autonomy, learning curve, and testing"
  - "Develop selection criteria for choosing IDE by project type and personal workflow preference"
  - "Create a personal IDE strategy document informed by hands-on experience"
  - "Evaluate strengths and limitations of each AI-native approach through critical reflection"
skills:
  ai-ide-selection:
    proficiency: "B1"
  comparative-analysis:
    proficiency: "B1"
  workflow-evaluation:
    proficiency: "B1"
generated_by: content-implementer v1.0.0
source_spec: specs/029-chapter8-ai-native-ides/spec.md
created: "2025-11-20"
last_modified: "2025-11-20"
git_author: "Content Implementation"
workflow: /sp.implement
version: 1.0.0
---

# Comparative Capstone: Choosing Your AI-Native Workflow

You've spent the last week exploring three distinct AI-native IDEs: Zed's performance-focused design, Cursor's evolution of the familiar VS Code paradigm, and Antigravity's agent-centric control plane. Each IDE approaches AI collaboration differently. Each makes different trade-offs between speed, autonomy, code quality, and learning curve.

This lesson brings them together in a single capstone project. You'll build the same feature—a task manager CLI application—in all three IDEs sequentially. By building identical functionality through three different workflows, you'll internalize the real differences between these tools: not in marketing claims or architectural diagrams, but in how they actually feel to work with, how fast they move, how much thinking they require from you, and what kind of code emerges from each approach.

The goal isn't to crown a "winner." The goal is to build your intuition about which tool fits which project, which workflow matches your thinking style, and how to adapt your approach when switching between IDEs.

---

## The Capstone Project: Task Manager CLI

Before diving into the three implementations, let's establish what you're building. This project tests all three IDEs against the same requirements, ensuring fair comparison.

### Project Requirements

**Core Functionality**:
- **`add <task>`**: Add a new task to the list. Each task gets a unique ID (auto-increment from 1), creation timestamp, and default priority level (medium).
- **`list`**: Display all tasks in a formatted table showing ID, description, priority, completion status, and creation date.
- **`complete <id>`**: Mark a task by ID as completed (toggle completion status).
- **`delete <id>`**: Remove a task by ID from the list permanently.

**Data Structure**:
- Tasks stored in a JSON file (`tasks.json`) in the current directory.
- Each task is a dictionary with: `id`, `description`, `priority` (low/medium/high), `completed` (boolean), `created_at` (ISO timestamp).

**Advanced Features** (add these after core is working):
- **Priority filtering**: `list --priority high` shows only high-priority tasks.
- **Search**: `search <keyword>` finds tasks containing keyword in description.
- **Completion statistics**: `stats` shows total tasks, completed count, completion percentage.

**Success Criteria**:
- ✅ All four core commands work correctly
- ✅ JSON storage persists between runs
- ✅ Timestamps are ISO format (YYYY-MM-DD HH:MM:SS)
- ✅ Error handling for invalid task IDs
- ✅ At least two advanced features implemented

**Why This Project**: The task manager exercises the full breadth of IDE capabilities. You'll write functions (testing code generation), manage state (testing context awareness), handle file I/O (testing integration with system), and iterate on requirements (testing collaboration patterns). No single simple function—this requires multi-file thinking, multiple refinement rounds, and critical evaluation of AI-generated code.

---

## Part 1: Building with Zed — The Inline Workflow

You're starting fresh in Zed. This workflow emphasizes **incremental inline requests**, where you build the project function-by-function, staying in the editor, using Ctrl+I to iterate rapidly.

### Time Tracking Setup

Before you start, create a simple time log. Open a text editor on the side and record:
- **Start time**: [your time]
- **Phase 1 complete**: [time after core CLI structure]
- **Phase 2 complete**: [time after all core functions]
- **Testing phase complete**: [time after manual testing]
- **Advanced features complete**: [time after optional features]
- **Total time**: [final time]

### Step 1: Project Structure in Zed

Create a new folder `task_manager_zed` and open it in Zed. Create `task_manager.py` (your main file) and keep it empty for now.

**Your first inline request** (Ctrl+I in the empty file):

```
Create a Python CLI task manager application.

Start with:
1. Necessary imports (argparse for CLI, json for file storage, datetime for timestamps)
2. A Task class with fields: id, description, priority, completed, created_at
3. A TaskManager class with methods for loading/saving tasks from JSON

Add type hints. Include docstrings for all classes and methods.
Don't implement the CLI commands yet—just the data structure and persistence layer.
```

**What Zed generates** (high-quality foundation):

```python
import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import Optional

class Task:
    """Represents a single task with metadata."""

    def __init__(self, id: int, description: str, priority: str = "medium",
                 completed: bool = False, created_at: Optional[str] = None):
        self.id = id
        self.description = description
        self.priority = priority
        self.completed = completed
        self.created_at = created_at or datetime.now().isoformat()

    def to_dict(self) -> dict:
        """Convert task to dictionary for JSON serialization."""
        return {
            "id": self.id,
            "description": self.description,
            "priority": self.priority,
            "completed": self.completed,
            "created_at": self.created_at
        }

    @staticmethod
    def from_dict(data: dict) -> "Task":
        """Create Task from dictionary."""
        return Task(**data)


class TaskManager:
    """Manages task persistence and operations."""

    def __init__(self, filename: str = "tasks.json"):
        self.filename = Path(filename)
        self.tasks: dict[int, Task] = {}
        self.next_id = 1
        self.load()

    def load(self) -> None:
        """Load tasks from JSON file."""
        if self.filename.exists():
            with open(self.filename, 'r') as f:
                data = json.load(f)
                for task_dict in data:
                    task = Task.from_dict(task_dict)
                    self.tasks[task.id] = task
                    self.next_id = max(self.next_id, task.id + 1)

    def save(self) -> None:
        """Save tasks to JSON file."""
        with open(self.filename, 'w') as f:
            tasks_list = [task.to_dict() for task in self.tasks.values()]
            json.dump(tasks_list, f, indent=2)

    def add_task(self, description: str, priority: str = "medium") -> Task:
        """Add new task and return it."""
        task = Task(self.next_id, description, priority)
        self.tasks[self.next_id] = task
        self.next_id += 1
        self.save()
        return task

    def get_task(self, task_id: int) -> Optional[Task]:
        """Get task by ID."""
        return self.tasks.get(task_id)

    def delete_task(self, task_id: int) -> bool:
        """Delete task by ID. Return True if successful."""
        if task_id in self.tasks:
            del self.tasks[task_id]
            self.save()
            return True
        return False

    def list_tasks(self) -> list[Task]:
        """Return all tasks sorted by ID."""
        return sorted(self.tasks.values(), key=lambda t: t.id)
```

**What you observe about this response**:
- Clear class hierarchy (Task stores data, TaskManager orchestrates)
- Type hints throughout (enables IDE intelligence and catches bugs early)
- Proper docstrings (explains intent without verbose comments)
- `to_dict()` and `from_dict()` methods (essential for JSON serialization)
- `next_id` tracking (ensures unique IDs even after deletions)
- File I/O abstraction (TaskManager handles all persistence)

The foundation is production-grade. You could literally stop here and have a usable data layer. The AI understood your intent to build incrementally—it gave you just the foundation, not the CLI yet.

### Step 2: Building the CLI Interface

Position your cursor at the end of the file and press Ctrl+I:

```
Now add the CLI interface using argparse.

Create a main() function that:
1. Sets up argument parser with subcommands: add, list, complete, delete
2. Handles each subcommand:
   - add: takes task description and optional --priority argument
   - list: displays all tasks in formatted table (use tabulate library if available, or format manually)
   - complete: takes task ID, toggles completion status
   - delete: takes task ID, removes task
3. Shows helpful error messages for invalid task IDs
4. Add if __name__ == "__main__": main() at the end

Make the list output human-readable with clear column headers.
```

**What Zed generates**:

```python
def format_task_table(tasks: list[Task]) -> None:
    """Print tasks in formatted table."""
    if not tasks:
        print("No tasks found.")
        return

    print("\n" + "=" * 80)
    print(f"{'ID':<4} {'Status':<10} {'Task':<40} {'Priority':<10} {'Created':<15}")
    print("=" * 80)

    for task in tasks:
        status = "✓ Done" if task.completed else "  Pending"
        # Truncate long descriptions
        desc = task.description[:37] + "..." if len(task.description) > 40 else task.description
        created = task.created_at[:10]  # Just the date part

        print(f"{task.id:<4} {status:<10} {desc:<40} {task.priority:<10} {created:<15}")

    print("=" * 80 + "\n")


def main() -> None:
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Task Manager CLI - Organize your tasks efficiently"
    )

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Add command
    add_parser = subparsers.add_parser("add", help="Add a new task")
    add_parser.add_argument("description", help="Task description")
    add_parser.add_argument(
        "--priority",
        choices=["low", "medium", "high"],
        default="medium",
        help="Task priority (default: medium)"
    )

    # List command
    list_parser = subparsers.add_parser("list", help="List all tasks")
    list_parser.add_argument(
        "--priority",
        choices=["low", "medium", "high"],
        help="Filter by priority"
    )

    # Complete command
    complete_parser = subparsers.add_parser("complete", help="Mark task as complete")
    complete_parser.add_argument("id", type=int, help="Task ID")

    # Delete command
    delete_parser = subparsers.add_parser("delete", help="Delete a task")
    delete_parser.add_argument("id", type=int, help="Task ID")

    # Stats command
    stats_parser = subparsers.add_parser("stats", help="Show completion statistics")

    args = parser.parse_args()
    manager = TaskManager()

    if args.command == "add":
        task = manager.add_task(args.description, args.priority)
        print(f"✓ Added task #{task.id}: {task.description}")

    elif args.command == "list":
        tasks = manager.list_tasks()
        if args.priority:
            tasks = [t for t in tasks if t.priority == args.priority]
        format_task_table(tasks)

    elif args.command == "complete":
        task = manager.get_task(args.id)
        if not task:
            print(f"Error: Task #{args.id} not found")
            return
        task.completed = not task.completed
        manager.save()
        status = "completed" if task.completed else "uncompleted"
        print(f"✓ Task #{args.id} marked as {status}")

    elif args.command == "delete":
        if not manager.delete_task(args.id):
            print(f"Error: Task #{args.id} not found")
            return
        print(f"✓ Deleted task #{args.id}")

    elif args.command == "stats":
        tasks = manager.list_tasks()
        completed = sum(1 for t in tasks if t.completed)
        total = len(tasks)
        percentage = (completed / total * 100) if total > 0 else 0
        print(f"\nTask Statistics:")
        print(f"  Total tasks: {total}")
        print(f"  Completed: {completed}")
        print(f"  Pending: {total - completed}")
        print(f"  Completion rate: {percentage:.1f}%\n")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
```

**What emerged from this response**:
- Clever formatting function that handles table display without external dependencies (works anywhere)
- All subcommands implemented with consistent error handling
- Bonus `stats` command (AI anticipated beyond minimal requirements)
- Type hints and docstrings maintained throughout
- Clear, user-friendly output messages

Notice Zed anticipated your needs by adding a stats command beyond your minimal specification. The AI recognized a pattern (tracking progress) and extended the scope. Through this interaction, you got a feature you would have eventually added manually—discovering it through collaboration rather than planning.

### Step 3: Manual Testing

Now open your terminal in Zed (Ctrl+`) and test:

```bash
python task_manager.py add "Buy groceries" --priority high
python task_manager.py add "Finish project report"
python task_manager.py list
python task_manager.py complete 1
python task_manager.py list
python task_manager.py stats
python task_manager.py delete 2
python task_manager.py list
```

**Observations during testing**:
- All commands work first try (no syntax errors)
- Output is readable and well-formatted
- Timestamps are stored correctly
- Tasks persist between runs (JSON storage works)
- Priority filtering works as expected
- Stats calculation is accurate

The entire application emerges without a single manual debugging session. The AI understood the requirements completely and generated code that executes correctly.

### Step 4: Adding Advanced Features

Position at the end of `main()` before `if __name__` and request:

```
Add two new commands:
1. search <keyword> - Find tasks containing keyword (case-insensitive)
2. priority <id> <new_priority> - Update task priority level

Keep the code style consistent with existing commands.
```

**What Zed generates**:

```python
    # Search command
    search_parser = subparsers.add_parser("search", help="Search tasks by keyword")
    search_parser.add_argument("keyword", help="Keyword to search for")

    # Priority command
    priority_parser = subparsers.add_parser("priority", help="Update task priority")
    priority_parser.add_argument("id", type=int, help="Task ID")
    priority_parser.add_argument(
        "level",
        choices=["low", "medium", "high"],
        help="New priority level"
    )

    # ... in the command handling section:

    elif args.command == "search":
        tasks = manager.list_tasks()
        results = [t for t in tasks if args.keyword.lower() in t.description.lower()]
        if not results:
            print(f"No tasks found matching '{args.keyword}'")
            return
        format_task_table(results)

    elif args.command == "priority":
        task = manager.get_task(args.id)
        if not task:
            print(f"Error: Task #{args.id} not found")
            return
        old_priority = task.priority
        task.priority = args.level
        manager.save()
        print(f"✓ Updated task #{args.id}: priority changed from {old_priority} to {args.level}")
```

**What you observe**: The AI didn't just add the commands—it integrated them seamlessly into the existing argparse structure, maintained error handling patterns, and followed the output message convention you established. The code reads as if it was written by one consistent author.

### Zed Workflow Summary

**Record your observations**:

```
Zed Workflow Evaluation:

Total Time: [your time in minutes]
Start-to-finish approach: Inline assistant prompts, incremental building
Number of AI requests: ~4 (foundation, CLI, testing/refinement, advanced features)
Manual fixes required: [count them - likely 0-1]

Response Speed: Very fast (200-400ms typical)
Suggestion Accuracy: Excellent (code worked first try)
Code Quality: Production-grade (type hints, docstrings, error handling)
Context Awareness: Exceptional (understood full project from partial prompts)

Most Helpful Feature: Inline assistant staying in editing flow
Biggest Challenge: [describe any issues you encountered]

Key Insight: The inline workflow favors incremental thinking. You break requirements into small prompts, stay in the editor, and build progressively. This mirrors how you'd naturally think through a project.
```

---

## Part 2: Building with Cursor — The Chat-Agent Workflow

Now you're switching IDEs. Cursor's approach is fundamentally different. Rather than small inline prompts, you'll write a complete specification, provide it to Cursor's Chat interface, then review and apply diffs. This workflow emphasizes **planning upfront** and **agent autonomy with review**.

### Setup: Create `.cursorrules` File

Before starting, create a file `.cursorrules` in your project root:

```
You are a Python expert helping build a CLI task manager application.

Code Style Guidelines:
- Always include type hints on all functions and class methods
- Use docstrings in Google style ("""summary. Detailed explanation.""")
- Handle errors explicitly with meaningful error messages
- Use pathlib.Path instead of os.path for file operations
- Prefer clear, simple code over clever one-liners
- Add comments only for non-obvious logic

Testing:
- Include error cases in implementation
- Consider edge cases like invalid input, missing files, empty lists
- Validate all user inputs before processing

Output Format:
- When generating code, format it for production use
- Ensure all imports are at the top of files
- Group related functions and classes logically
```

**Why this matters**: Cursor uses `.cursorrules` to guide AI behavior. By setting expectations upfront, you're essentially programming the AI's code style before it writes anything. This is **specification-first thinking applied to code generation**.

### Step 1: Write Complete Specification

Create `SPEC.md`:

```markdown
# Task Manager CLI - Complete Specification

## Overview
A command-line task management application that stores tasks in JSON and supports CRUD operations with advanced features.

## Core Requirements

### Data Model
- **Task** class with fields:
  - `id` (integer, auto-increment from 1)
  - `description` (string, required)
  - `priority` (string: "low", "medium", "high", default "medium")
  - `completed` (boolean, default False)
  - `created_at` (ISO timestamp string, auto-generated)

- **TaskManager** class:
  - Load/save tasks from `tasks.json`
  - Track next available ID
  - Provide methods: add, get, delete, list tasks

### CLI Commands

1. **add <description> [--priority LEVEL]**
   - Create new task
   - Output: "✓ Added task #N: description"

2. **list [--priority LEVEL] [--completed | --pending]**
   - Display all tasks in formatted table
   - Columns: ID, Status, Description, Priority, Created Date
   - Optional: filter by priority or completion status

3. **complete <id>**
   - Toggle completion status
   - Output: "✓ Task #N marked as completed/uncompleted"

4. **delete <id>**
   - Remove task permanently
   - Output: "✓ Deleted task #N"

5. **stats**
   - Show: total tasks, completed count, pending count, completion percentage

### Advanced Features
- **search <keyword>** - Case-insensitive search in task descriptions
- **priority <id> <level>** - Update task priority
- **update <id> <new_description>** - Change task description

## Success Criteria
- All core commands work correctly
- Error handling for invalid IDs
- JSON persists between runs
- Table formatting is readable
- Timestamps are consistent ISO format

## File Structure
- `task_manager.py` - All code in single file for this project
```

### Step 2: Use Cursor's Chat Interface

Open Cursor's Chat pane (Cmd+L on macOS). Paste your specification and request:

```
I've provided a task manager specification in SPEC.md.

Please implement the complete application in task_manager.py following these guidelines:
1. Follow the .cursorrules file for code style
2. Implement all core commands and advanced features from SPEC.md
3. Include comprehensive error handling
4. Make the table output human-readable without external dependencies
5. Ensure JSON storage works reliably

Generate the complete, production-ready implementation.
```

**What Cursor does** (different from Zed):

Cursor enters Agent Mode and proposes a multi-file structure (or offers diffs). Rather than inline suggestions, you get a full implementation proposal with this pattern:

1. **Summary** of what Cursor will do
2. **Proposed structure** (file organization)
3. **Diff view** showing changes file-by-file
4. **Acceptance buttons** - Accept All, Review Changes, Request Modifications

### Step 3: Review the Agent Proposal

This is where Cursor's workflow differs significantly from Zed. Instead of incremental requests, Cursor proposes the full solution. You review the diffs and decide:

**The Diff Shows**:
```
+ task_manager.py (NEW FILE)
  + import statements
  + class Task (complete with all methods)
  + class TaskManager (complete with all methods)
  + helper functions for formatting
  + main() function with complete argparse setup
  + all commands implemented
```

**Your evaluation**: Does this implementation match your specification?

- ✅ Type hints present throughout
- ✅ Google-style docstrings
- ✅ Error handling for invalid IDs
- ✅ All core commands present
- ✅ Advanced features (search, priority update) included
- ✅ Stats command implemented
- ⚠️ One thing to check: Are timestamps ISO format?

### Step 4: Request Refinements

In Cursor's Chat, if you notice issues, you can refine:

```
The implementation looks good overall. Two refinements:

1. In the Task class, ensure created_at always uses ISO format.
   Current: datetime.now().isoformat()
   This is good, but let's ensure consistency. If you add an update_at field,
   it should use the same format.

2. In the stats command, add a "last_modified" statistic showing when
   tasks.json was last updated.

Make these changes to task_manager.py.
```

**What happens next**: Cursor updates the diff, shows you what changed, and asks for approval. Once approved, changes apply to your actual file.

### Step 5: Test the Implementation

In Cursor's integrated terminal:

```bash
python task_manager.py add "Design system architecture" --priority high
python task_manager.py add "Write documentation"
python task_manager.py add "Code review" --priority medium
python task_manager.py list
python task_manager.py search "system"
python task_manager.py stats
python task_manager.py priority 1 low
python task_manager.py complete 2
python task_manager.py list
```

### Cursor Workflow Summary

**Record your observations**:

```
Cursor Workflow Evaluation:

Total Time: [your time in minutes]
Start-to-finish approach: Specification upfront → Full agent proposal → Review diffs → Refine → Done
Number of AI interactions: ~2-3 (spec proposal + refinements)
Manual fixes required: [count them - likely 0-2]

Agent Autonomy: High (proposes complete solution, not incremental)
Diff Review Experience: Clear (easy to see what changed)
.cursorrules Effectiveness: [Did they guide code quality?]
Integration with editor: Seamless (diffs applied directly)

Most Helpful Feature: Agent mode handling complex multi-part tasks
Biggest Challenge: [describe any issues]

Key Insight: Cursor's workflow favors specification-driven thinking. You write intent upfront, agent handles full implementation, you review and refine. This mirrors how professional teams work—spec → implementation → review.
```

---

## Part 3: Building with Antigravity — The Plan-Artifact Workflow

Now switch to Antigravity. This is fundamentally different from both Zed and Cursor. Antigravity's approach emphasizes **planning**, **artifacts**, and **parallel work opportunities**. Rather than inline requests or chat diffs, you'll create an agent that generates a comprehensive implementation plan, review it, then watch the agent execute.

### Step 1: Spawn the Agent with Full Context

In Antigravity's Agent Manager, create a new agent:

**Agent Name**: TaskManagerBuilder

**System Prompt**:
```
You are an expert Python developer building a CLI task manager application.

Your job is to:
1. Create a comprehensive Implementation Plan artifact listing all tasks needed
2. Break down the project into logical pieces (data structures, CLI interface, storage, testing)
3. Generate the complete Python implementation
4. Provide a Walkthrough artifact showing usage examples and output

Follow these standards:
- All code includes type hints and docstrings
- Use pathlib for file operations
- Include comprehensive error handling
- Make output human-readable

Create these artifacts in this order:
1. Task List - Break project into 5-6 tasks with dependencies
2. Implementation Plan - Detailed plan for each task
3. task_manager.py - Complete implementation
4. Walkthrough - Usage examples with expected output
```

### Step 2: Review the Agent's Task List Artifact

Antigravity's agent first generates a Task List artifact. This shows how the agent plans to break down your project:

```
Task 1: Design Data Model (1-2 hours estimated)
- Create Task class with id, description, priority, completed, created_at fields
- Create TaskManager class with persistence layer
- Dependency: None (foundational)

Task 2: Implement File I/O (30-45 min)
- JSON load/save methods
- Handle missing file gracefully
- Dependency: Task 1

Task 3: Build CLI Parser (45-60 min)
- Set up argparse with all commands
- Validate arguments
- Dependency: Task 2

Task 4: Implement Commands (60-90 min)
- add, list, complete, delete, stats
- Error handling for invalid IDs
- Dependency: Task 3

Task 5: Advanced Features (30-45 min)
- search command
- priority update command
- update description command
- Dependency: Task 4

Task 6: Testing & Verification (30-45 min)
- Manual test all commands
- Verify JSON persistence
- Test edge cases
- Dependency: Task 5
```

**Your evaluation**: Does this breakdown make sense?

This is where Antigravity differs from Cursor. You're not just seeing the code—you're seeing the PLAN for how the agent will build the code. You can comment on the plan, suggest changes, reorder tasks, or add constraints before the agent writes a single line of code.

### Step 3: Review and Comment on the Plan

In Antigravity, add a comment to the Implementation Plan artifact:

```
Good breakdown. Two comments:

1. On Task 2 (File I/O): Please handle the edge case where tasks.json is corrupted
   (invalid JSON). Raise a clear error message.

2. On Task 5 (Advanced Features): Add a "filter" command that shows only
   high-priority tasks. This is a commonly-needed feature.

3. On Task 6 (Testing): Include a test for the case where user provides invalid
   task ID. Verify the error message is clear.

Otherwise, the plan looks solid. Please proceed with implementation.
```

**What happens**: The agent reads your feedback and updates its Implementation Plan before writing code. This is a **critical difference from Zed and Cursor**—you shape the work before it happens, not after.

### Step 4: Watch the Agent Execute

The agent now generates the full `task_manager.py` implementation. Rather than showing you one giant code block, Antigravity shows this in an Implementation artifact with these sections:

1. **Imports and Setup** (top 30 lines)
2. **Task Class** (complete with methods)
3. **TaskManager Class** (with persistence)
4. **CLI Functions** (command implementations)
5. **Main Function** (argument parsing)
6. **Error Handling** (custom exceptions or utilities)

Each section is reviewable, and you can comment on specific parts without re-running the whole thing.

### Step 5: Review Generated Code

Key things to verify in the generated code:

**Task Model**:
```python
✅ Task class exists with all required fields
✅ to_dict() and from_dict() for JSON serialization
✅ Type hints throughout
✅ Docstrings clear
```

**TaskManager**:
```python
✅ Loads existing tasks.json
✅ Handles missing file gracefully
✅ Handles corrupted JSON (your feedback was applied)
✅ Auto-increments IDs correctly
✅ Persists to JSON after each operation
```

**CLI Interface**:
```python
✅ All commands have help text
✅ Argument validation present
✅ Error messages for invalid IDs
✅ Search command included
✅ Filter command included (your feedback applied)
✅ Stats command included
```

### Step 6: Request the Walkthrough Artifact

Now ask the agent to generate a **Walkthrough** artifact showing actual usage:

```
Please create a Walkthrough artifact showing the complete usage flow:
1. Start with empty task list
2. Add 5 tasks with varying priorities
3. Show list output
4. Show search results
5. Mark some tasks complete
6. Show updated stats
7. Show filter by priority
8. Delete a task
9. Show final list

Include expected output for each command.
```

**What the Walkthrough shows** (in artifact format):

```
## Walkthrough: Complete Task Manager Usage

### Step 1: Add Tasks
$ python task_manager.py add "Build API" --priority high
✓ Added task #1: Build API

$ python task_manager.py add "Write tests" --priority high
✓ Added task #2: Write tests

$ python task_manager.py add "Documentation" --priority medium
✓ Added task #3: Documentation

$ python task_manager.py add "Code review" --priority low
✓ Added task #4: Code review

$ python task_manager.py add "Deploy to production" --priority high
✓ Added task #5: Deploy to production

### Step 2: List All Tasks
$ python task_manager.py list
================================================================================
ID   Status     Task                                     Priority   Created
================================================================================
1    Pending    Build API                                high       2025-11-20
2    Pending    Write tests                              high       2025-11-20
3    Pending    Documentation                            medium     2025-11-20
4    Pending    Code review                              low        2025-11-20
5    Pending    Deploy to production                     high       2025-11-20
================================================================================

### Step 3: Search for Tasks
$ python task_manager.py search "code"
================================================================================
ID   Status     Task                                     Priority   Created
================================================================================
4    Pending    Code review                              low        2025-11-20
================================================================================

### Step 4: Filter by Priority
$ python task_manager.py filter --priority high
================================================================================
ID   Status     Task                                     Priority   Created
================================================================================
1    Pending    Build API                                high       2025-11-20
2    Pending    Write tests                              high       2025-11-20
5    Pending    Deploy to production                     high       2025-11-20
================================================================================

### Step 5: Complete Task
$ python task_manager.py complete 2
✓ Task #2 marked as completed

### Step 6: Show Statistics
$ python task_manager.py stats
Task Statistics:
  Total tasks: 5
  Completed: 1
  Pending: 4
  Completion rate: 20.0%
  Last modified: 2025-11-20 14:32:15

### Step 7: Final State
$ python task_manager.py list
(shows list with task 2 marked as complete)
```

This Walkthrough is crucial—it shows you exactly what the agent will produce before anything runs. You can review expected behavior, suggest changes, or approve for actual testing.

### Step 7: Manual Testing

Run the commands shown in the Walkthrough in your actual Antigravity environment:

```bash
python task_manager.py add "Build API" --priority high
python task_manager.py list
python task_manager.py search "API"
python task_manager.py filter --priority high
python task_manager.py complete 1
python task_manager.py stats
python task_manager.py delete 1
python task_manager.py list
```

**Observations**:
- Code matches Walkthrough expectations
- All commands work as planned
- JSON persistence functions correctly
- Error handling works as specified

### Antigravity Workflow Summary

**Record your observations**:

```
Antigravity Workflow Evaluation:

Total Time: [your time in minutes]
Start-to-finish approach: Agent creates Task List → Review plan → Agent implements → Review code → Walkthrough artifact → Test
Number of AI interactions: ~4 (plan, implementation, walkthrough, refinements)
Manual fixes required: [count them]

Plan Review Value: Very high (shaped work before implementation)
Artifact Clarity: Excellent (clear structure, reviewable sections)
Parallel Work: Could you review plan while agent works? Did this save time?
Error Handling Quality: [Did the agent's error handling match your feedback?]

Most Helpful Feature: [What surprised you about Antigravity?]
Biggest Challenge: [What was different from Zed/Cursor?]

Key Insight: Antigravity's workflow favors deliberate planning. You shape the architecture upfront, review the plan before code is written, and watch artifacts accumulate. This suits complex projects where upfront design matters.
```

---

## Comparative Analysis: Five Dimensions

Now that you've built the same feature three times, let's systematically compare these IDEs across dimensions that matter for real work.

### Dimension 1: Development Speed

**Zed Workflow Speed**:
- Inline requests are fast to type (5-10 seconds per prompt)
- AI response is instant (200-400ms)
- Iterating is quick—just press Ctrl+I again
- **Total time for this project**: Likely 20-30 minutes

**Cursor Workflow Speed**:
- Specification writing takes time upfront (5-10 minutes)
- Agent proposal takes longer (2-3 minutes for complex code)
- Diff review adds time (5-10 minutes)
- **Total time for this project**: Likely 30-40 minutes
- **Advantage**: Fewer iterations once correct, less overall back-and-forth

**Antigravity Workflow Speed**:
- Plan review creates upfront decision-making (10-15 minutes)
- Implementation artifact takes time (3-5 minutes)
- Walkthrough verification adds time (5-10 minutes)
- **Total time for this project**: Likely 40-50 minutes
- **Advantage**: Catches architectural issues before code, prevents wrong implementations

**When Each Shines**:
- **Zed**: Quick prototypes, exploratory coding, "let me try this" moments
- **Cursor**: Medium-complexity features where you want agent autonomy
- **Antigravity**: Complex features, architectural decisions, team projects where planning matters

### Dimension 2: Code Quality

**Zed Code Quality**:
- Excellent for the scope requested
- Type hints present and correct
- Docstrings clear but sometimes terse
- Error handling solid but basic
- **Strength**: Production-ready code without bloat

**Cursor Code Quality**:
- Excellent overall
- Type hints comprehensive
- Docstrings detailed (Google or NumPy style)
- Error handling thoughtful (catches edge cases)
- **Strength**: Code that considers edge cases and future maintenance

**Antigravity Code Quality**:
- Excellent with context of plan
- Type hints thorough
- Docstrings clear with context from plan
- Error handling comprehensive (catches all cases you mentioned)
- **Strength**: Code that directly addresses your concerns

**Pattern**: All three produce quality code, but Cursor and Antigravity tend to anticipate problems better because of planning/spec-driven approaches.

### Dimension 3: Learning Curve

**Zed Learning Curve**:
- **Easiest**: Ctrl+I is intuitive
- You learn through iteration
- Fast feedback loop builds intuition
- **Drawback**: Can be tempting to make requests without thinking deeply

**Cursor Learning Curve**:
- **Medium**: Need to learn when to use Chat vs Agent modes
- Need to understand `.cursorrules` concept
- Diff-based review takes adjustment
- **Advantage**: Teaches specification thinking earlier

**Antigravity Learning Curve**:
- **Steepest**: Need to understand Task Lists, Implementation Plans, Walkthroughs
- Need to think about artifact-based workflows
- Benefits emerge over time (first project feels slower)
- **Advantage**: Teaches architectural thinking

**Recommendation by Experience Level**:
- **Beginner**: Start with Zed, then add Cursor
- **Intermediate**: Use Cursor primarily, Zed for quick tasks
- **Advanced/Team**: Antigravity for complex projects

### Dimension 4: Autonomy Level

**Zed Autonomy**:
- Very controlled (you direct every function)
- You stay in control of scope
- Best when you know what you're building
- Risk: Might miss elegant solutions from outside your mental model

**Cursor Autonomy**:
- High (agent proposes full implementation)
- You guide through `.cursorrules` and refinement requests
- Good balance: agent thinks independently, you retain control
- Benefit: Agent might suggest patterns you hadn't considered

**Antigravity Autonomy**:
- Very high (agent plans and implements)
- You guide through plan review
- Agent makes implementation decisions within plan constraints
- Benefit: Agent might propose architectural improvements

**Matching Autonomy to Project Type**:
- **Refactoring existing code**: Zed (maintain control, make surgical changes)
- **Building new features**: Cursor (let agent think, you review)
- **New project architecture**: Antigravity (let agent design, you guide)

### Dimension 5: Testing and Iteration

**Zed Testing**:
- Manual testing is natural (terminal right there in editor)
- Quick to fix bugs (Ctrl+I + "fix this" + iterate)
- Testing feels interleaved with coding
- **Strength**: Tight feedback loop for debugging

**Cursor Testing**:
- Manual testing after applying diffs
- Refinements create new diffs (easier to see what changed)
- Can request tests as code generation
- **Strength**: Cleanly separated test phase

**Antigravity Testing**:
- Walkthrough artifact shows expected behavior before testing
- Agent can generate test scripts
- Can request integration tests as artifacts
- **Strength**: Testing strategy visible before implementation

**Testing Pattern Recommendation**:
- **Exploratory coding**: Use Zed's tight loop
- **Feature implementation**: Use Cursor's cleanly separated phases
- **Complex systems**: Use Antigravity's test artifact planning

### Exercise: Rate Each IDE

For each of the five dimensions, rate the three IDEs on a scale of 1-5 based on your actual experience:

```
Dimension 1: Development Speed
  Zed:        ___/5
  Cursor:     ___/5
  Antigravity: ___/5

Dimension 2: Code Quality
  Zed:        ___/5
  Cursor:     ___/5
  Antigravity: ___/5

Dimension 3: Learning Curve (inverse scale: 5 = easiest, 1 = hardest)
  Zed:        ___/5
  Cursor:     ___/5
  Antigravity: ___/5

Dimension 4: Autonomy (5 = maximum, 1 = minimal)
  Zed:        ___/5
  Cursor:     ___/5
  Antigravity: ___/5

Dimension 5: Testing & Iteration
  Zed:        ___/5
  Cursor:     ___/5
  Antigravity: ___/5

Total Scores:
  Zed:        ___/25
  Cursor:     ___/25
  Antigravity: ___/25
```

---

## IDE Selection Decision Framework

Based on what you've learned, use this framework to choose IDEs for different scenarios.

### Scenario 1: Quick Prototype (Web Calculator)

**Project characteristics**:
- Single Python file
- < 200 lines of code
- You know exactly what you want
- Time budget: 30 minutes
- Testing: Manual, minimal

**Framework decision**:
1. Do I need upfront planning? NO
2. Do I need agent autonomy? NO
3. Is performance critical? NO
4. Is this refactoring or new? NEW (but very simple)

**Recommendation**: **ZED**
- Inline requests stay in your mental flow
- Fast iteration matches tight time budget
- Single file, no architectural questions
- You'll complete faster than other IDEs

### Scenario 2: Refactor Existing Feature

**Project characteristics**:
- Existing codebase (500+ lines)
- Multi-file changes needed
- Risk: breaking existing functionality
- Time budget: 1-2 hours
- Testing: Need to verify nothing breaks

**Framework decision**:
1. Do I need upfront planning? YES (understand impact)
2. Do I need agent autonomy? MEDIUM (prefer review control)
3. Is there existing context to preserve? YES
4. Are multi-file changes? YES

**Recommendation**: **CURSOR**
- Chat mode gives you context awareness of whole codebase
- Diff view shows exactly what's changing (risk mitigation)
- `.cursorrules` ensures style consistency
- Agent mode can handle multi-file refactoring with your approval

### Scenario 3: Build New Component (Authentication Module)

**Project characteristics**:
- New code, but significant
- Security matters (affects whole app)
- Needs testing strategy
- Complex: users, sessions, tokens
- Time budget: 3-4 hours
- Testing: Unit tests + integration tests

**Framework decision**:
1. Do I need upfront planning? YES (security-critical)
2. Do I need to review architecture? YES
3. Is implementation complex? YES
4. Should tests be visible upfront? YES

**Recommendation**: **ANTIGRAVITY**
- Task List + Plan help you think through security implications
- Walkthrough artifact shows expected behavior before code
- Agent can generate unit tests alongside implementation
- Plan review catches architectural issues early

### Scenario 4: Data Processing Pipeline

**Project characteristics**:
- Clear requirements (transform data, load/save)
- Multiple processing stages
- Moderate complexity
- Some uncertainty about implementation details
- Time budget: 1-2 hours
- Testing: Input/output validation

**Framework decision**:
1. Do I need upfront planning? MEDIUM (stages are clear)
2. Do I need agent to suggest improvements? YES (might optimize)
3. How certain am I about approach? MEDIUM
4. Is performance important? YES

**Recommendation**: **CURSOR** (primary) **or ZED** (if simple enough)
- Cursor's agent can suggest efficient patterns
- `.cursorrules` for performance best practices
- Chat mode lets you discuss alternatives before committing

### Scenario 5: Team Project (with Code Review)

**Project characteristics**:
- Multiple developers involved
- Code standards important
- Need to document decisions
- Medium-to-large feature
- Time budget: 4-8 hours
- Testing: Comprehensive (CI/CD pipeline)

**Framework decision**:
1. Do I need upfront planning? YES (coordination)
2. Does team need to review architecture? YES
3. Are implementation decisions important? YES
4. Should test strategy be planned? YES

**Recommendation**: **ANTIGRAVITY**
- Plan artifact can be shared/reviewed by team
- Implementation Plan documents reasoning
- Artifacts serve as communication medium
- Test strategy visible from start

---

## Creating Your Personal IDE Strategy

Now it's time to reflect on your experience and create a personal strategy document.

### Your IDE Selection Matrix

Create a table for your actual workflow:

```
Project Type       | Primary IDE | Secondary IDE | Why Primary
-------------------+-------------+---------------+-----------------------
Quick Prototype    | Zed         | Cursor        | Fast iteration, simple
Refactoring        | Cursor      | Zed           | Diff control, context
New Component      | Antigravity | Cursor        | Planning matters
Data Processing    | Cursor      | Zed           | Agent autonomy valued
Team Collaboration | Antigravity | Cursor        | Documentation, planning
Bug Fix            | Zed         | Cursor        | Speed, focused scope
Exploration        | Zed         | Antigravity   | Learning-focused
Learning Project   | Zed         | Antigravity   | Iterative feedback
```

### Your Personal Workflow Strategy

Complete this template:

```markdown
# My AI-Native IDE Strategy

## Primary IDE: [Choose one: Zed, Cursor, Antigravity]

Reason: [Why this fits your workflow best]

Examples of projects I'll use this for:
- [Project type 1]
- [Project type 2]
- [Project type 3]

## Secondary IDE: [Choose one of the remaining two]

Reason: [When you'll switch to this]

Specific use cases:
- [Use case 1]
- [Use case 2]

## Tertiary IDE: [The remaining one]

When I might use this:
- [Situation 1]
- [Situation 2]

## My Workflow Preferences

**Speed vs Autonomy**: I prefer [fast iteration / agent autonomy / planned approach]
because [your reason]

**Inline vs Chat**: I prefer [inline requests / chat interface] because [your reason]

**Code Review Style**: I prefer [iterative refinement / diff review / artifact review]
because [your reason]

## IDE Switching Strategy

When I switch from [IDE A] to [IDE B], I'll:
1. [Adjustment 1]
2. [Adjustment 2]
3. [Adjustment 3]

## Areas to Improve

Things that surprised me about AI-native IDEs:
- [Surprise 1]
- [Surprise 2]

Things I want to practice more:
- [Practice area 1]
- [Practice area 2]

## Integration with My Workflow

How I'll use these tools alongside:
- **Terminal**: [How you'll use terminal + IDE together]
- **Git**: [How you'll handle version control]
- **Testing**: [How you'll test code]
- **Collaboration**: [How you'll work with others]

## 30-Day Experiment Plan

For the next 30 days, I'll:
1. **Week 1**: Use my primary IDE exclusively for all projects
2. **Week 2**: Introduce secondary IDE for specific use cases
3. **Week 3**: Experiment with tertiary IDE on one project
4. **Week 4**: Reflect on which combinations work best
```

---

## Key Outcomes from This Capstone

In this capstone, you've built the same feature three times using three fundamentally different AI-native development approaches. This experience teaches you how different workflows shape your thinking and decision-making as a developer.

**What Emerged from This Comparative Work**:

1. **No universal "best" IDE exists**. Each makes different trade-offs. Zed prioritizes speed. Cursor balances autonomy and control. Antigravity emphasizes planning. The "right" choice depends on the project and your preferences.

2. **Your thinking changes with your tool**. When you use Zed's inline workflow, you think incrementally. When you use Cursor, you think specificationally. When you use Antigravity, you think architecturally. Experienced developers adapt their thinking to their tools.

3. **Planning upfront saves time later**. Cursor and Antigravity seem slower initially, but they prevent wrong implementations. On complex projects, they're actually faster because they reduce iteration cycles.

4. **Context awareness matters enormously**. All three IDEs understand your project context, but in different ways. Zed has tight local context. Cursor has codebase context. Antigravity has intention context (from your plan).

5. **Code quality matters less than code process**. All three produced working code. The difference is in how you got there. Speed matters for some projects. Correctness matters for others. Maintainability matters for teams.

6. **AI-native development is about decision-making, not typing**. None of these IDEs force you to write code. They force you to make clearer decisions about what you want before implementation happens. This is the paradigm shift.

---

## Try With AI

Apply what you've learned in a new project.

**Setup**: Choose one of these projects based on what you want to practice:
- **Speed focus**: Build a JSON analyzer CLI (read file, pretty-print, stats)
- **Planning focus**: Design a note-taking app (richer data model)
- **Collaboration focus**: Create a configuration validator (clear spec, error messages matter)

**Your task**:

1. **Specify clearly**: Write requirements for your chosen project (1 paragraph minimum)

2. **Choose your IDE** based on this decision framework:
   - Quick project with clear scope? → Zed
   - Feature with multiple considerations? → Cursor
   - Complex system needing architecture decisions? → Antigravity

3. **Build the project** using the workflow you selected

4. **Document your experience**:
   - Total time elapsed
   - Number of AI interactions
   - Any manual fixes required
   - What surprised you compared to the task manager project

5. **Reflect in writing** (300+ words):
   - Did your IDE choice match the project well?
   - What would you do differently next time?
   - How confident are you in your IDE selection process?
   - Which IDE feels most natural for your thinking style?

**Success criteria**: Your project works (all features functional), and you can articulate why you chose that IDE for this project type.

**Optional Challenge**: Build the same project in a different IDE and compare the experience. Notice how your thinking changes with the tool.

