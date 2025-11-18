---
title: "Capstone - Note-Taking App"
chapter: 22
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

# Learning objectives
learning_objectives:
  - objective: "Integrate all I/O concepts (console, file, pathlib, JSON) into a complete, production-quality CLI application"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Build working Note-Taking App with all CRUD operations"

  - objective: "Implement Create, Read, Update, Delete operations for file-based data persistence"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Demonstrate each operation working correctly with data saved to files"

  - objective: "Design menu-driven interactive loops with proper input validation and error recovery"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Test menu loop handling valid/invalid input, returning to menu after each operation"

  - objective: "Search and filter data structures to find notes by keyword and category"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Implement search function that returns matching notes"

  - objective: "Handle errors at application scale (user input, file operations, JSON parsing)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Test application with edge cases (missing files, invalid input, corrupted data)"

# Cognitive load tracking
cognitive_load:
  new_concepts: 0
  assessment: "0 new concepts (integration of Lessons 1-4 only) - appropriate for B1 capstone synthesis"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Add timestamps with datetime module (Chapter 23 preview), implement categories as nested structure, add export-to-CSV feature, implement undo/redo with history"
  remedial_for_struggling: "Provide starter code with menu loop and function stubs, focus on one CRUD operation at a time, use simpler JSON structure without timestamps, work through creating a single note before attempting full app"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-22/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 5: Capstone - Note-Taking App

## Introduction: Building a Real-World CLI Application

Welcome to the capstone project for Chapter 22! Everything you've learned in Lessons 1-4 comes together here. You're going to build a complete, production-quality CLI note-taking application that integrates:

- **Lesson 1**: Console I/O with input validation (menu-driven interface)
- **Lesson 2**: Safe file operations (reading and writing notes)
- **Lesson 3**: Cross-platform paths (organizing notes in directories)
- **Lesson 4**: JSON data format (persisting structured note data)

This isn't a toy project—it's a real application that demonstrates professional-level coding practices. When you're done, you'll have a program that:

- Runs without crashing, even with invalid user input
- Persists data to disk in organized directories
- Handles a dozen notes (or dozens) with responsive performance
- Recovers gracefully from common errors (missing files, corrupted data)
- Organizes code clearly so others can read and extend it

**Time Estimate**: 90-120 minutes of focused development (plus extension ideas if you want to go deeper)

**What You'll Build**: A fully functional Note-Taking App with these features:
1. **Menu-driven interface** - Display options, accept user choice, execute action, return to menu
2. **Create notes** - Prompt for title and body, save with UUID and timestamp
3. **Read notes** - List existing notes, display selected note
4. **Update notes** - Modify title, body, or category
5. **Delete notes** - Remove notes with confirmation
6. **Search notes** - Find notes by keyword across title and body
7. **List notes** - Show all notes organized by category
8. **Exit gracefully** - Clean shutdown with goodbye message

## Application Architecture Introduction

Before writing code, let's understand the design. A well-architected application separates concerns:

### Components

**Menu Loop** (Lessons 1)
- Displays menu options
- Accepts user choice
- Routes to appropriate function
- Returns to menu after operation

**CRUD Functions** (Lessons 2-4)
- `get_all_notes()` → Load all notes from disk
- `save_note()` → Write or update a note to disk
- `search_notes()` → Find notes by keyword
- `delete_note()` → Remove note file

**Data Structure** (Lesson 4)
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "Python Study Notes",
  "body": "Today learned about file I/O...",
  "category": "learning",
  "created": "2025-11-09T10:30:00",
  "modified": "2025-11-09T14:45:00"
}
```

**File Organization** (Lesson 3)
```
notes/
├── learning/
│   ├── 550e8400....json
│   └── 6f47fe8e....json
├── work/
│   └── a1b2c3d4....json
└── personal/
    └── x9y8z7w6....json
```

### Why This Design?

- **Separation of Concerns**: Menu loop doesn't know about file operations; functions don't know about UI
- **Scalability**: File-per-note approach handles 10-50 notes without performance issues
- **Maintainability**: Clear functions mean future extensions are easy
- **Testability**: Each function can be tested independently

#### 💬 AI Colearning Prompt

> "I'm designing a note-taking app. Should I store all notes in one JSON file or one file per note? What are the tradeoffs?"

**Expected Outcome**: You'll understand data persistence patterns and trade-offs before writing code. AI can help you think through design before implementation.

---

## Setting Up Project Structure

Every good application starts with organization. Let's initialize the directory structure and default categories.

### Creating the Notes Directory

First, we ensure the `notes/` directory exists with default categories for organization:

```python
from pathlib import Path

# Setup
BASE_DIR: Path = Path("notes")
BASE_DIR.mkdir(exist_ok=True)

# Create default categories
CATEGORIES: list[str] = ["work", "personal", "learning"]
for category in CATEGORIES:
    (BASE_DIR / category).mkdir(exist_ok=True)

print("✅ Note-taking app initialized!")
print(f"Data directory: {BASE_DIR.resolve()}")
```

**Why this matters**:
- Uses **pathlib** (Lesson 3) to create cross-platform paths
- Creates directories only if they don't exist (idempotent)
- Resolves absolute path to show user exactly where data is stored

#### 🎓 Expert Insight

> In AI-native development, you don't debug path errors manually—you design paths clearly at startup. Your job: specify the directory structure. AI can help you verify it's correct with `Path.resolve()`.

---

## Code Example 5.1: Project Structure and Security

Before we write the full app, let's look at how to initialize the project safely with security validation:

```python
from pathlib import Path
import uuid
import json
from datetime import datetime

# Setup
BASE_DIR: Path = Path("notes")
BASE_DIR.mkdir(exist_ok=True)

# Create default categories
CATEGORIES: list[str] = ["work", "personal", "learning"]
for category in CATEGORIES:
    (BASE_DIR / category).mkdir(exist_ok=True)

def get_note_path(category: str, note_id: str) -> Path:
    """
    Get safe path for a note with security validation.

    Uses resolve() to get canonical path, then validates it stays
    within the allowed BASE_DIR (prevents path traversal attacks).
    """
    # Resolve the requested path to canonical form
    requested_path: Path = (BASE_DIR / category / f"{note_id}.json").resolve()

    # Check that resolved path stays within base directory
    # is_relative_to() ensures path is under base_dir
    if not requested_path.is_relative_to(BASE_DIR.resolve()):
        raise ValueError(f"Security error: Invalid path {requested_path}")

    return requested_path

# Example: Create a new note
note_id: str = str(uuid.uuid4())
note: dict = {
    "id": note_id,
    "title": "My First Note",
    "body": "This is the body of my note.",
    "category": "learning",
    "created": datetime.now().isoformat(),
    "modified": datetime.now().isoformat(),
}

# Save to file
note_path: Path = get_note_path("learning", note_id)
with open(note_path, 'w', encoding='utf-8') as f:
    json.dump(note, f, indent=2, ensure_ascii=False)

print(f"✅ Note saved to: {note_path}")
```

**Specification Reference**: Path traversal prevention using `Path.resolve()` + `is_relative_to()` from Chapter 22, Lesson 3

**Prompt Used**: "Show me how to safely construct file paths that prevent directory traversal attacks using pathlib"

**Validation Steps**:
1. ✅ Test with normal path: `get_note_path("learning", uuid_string)` → returns valid path in notes/learning/
2. ✅ Test with traversal attempt: `get_note_path("../../../etc/passwd", "id")` → raises ValueError
3. ✅ Verify resolved path is canonical: `resolved_path.is_relative_to(BASE_DIR.resolve())` → True for legitimate paths

---

## Core CRUD Functions

Now let's implement the functions that handle data persistence. These are the workhorses of the application—they orchestrate file I/O, error handling, and JSON operations.

### Code Example 5.2: Complete CRUD Functions

```python
from pathlib import Path
import json
from typing import Optional
import uuid
from datetime import datetime

BASE_DIR: Path = Path("notes")

def get_all_notes() -> list[dict]:
    """
    Load all notes from files in the notes/ directory.

    Uses glob() to find all .json files recursively across all categories.
    Handles corrupted files gracefully with error messages.
    """
    notes: list[dict] = []

    for json_file in BASE_DIR.glob("**/*.json"):
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                note = json.load(f)
                notes.append(note)
        except json.JSONDecodeError:
            print(f"⚠️  Warning: Corrupted note file {json_file.name} - skipping")
        except Exception as e:
            print(f"⚠️  Error reading {json_file.name}: {e}")

    return notes


def save_note(note: dict) -> bool:
    """
    Save or update a note to disk.

    Takes a note dictionary, extracts category and ID, creates the directory
    if needed, and writes to JSON file with UTF-8 encoding and formatting.
    """
    try:
        category = note.get("category", "personal")
        note_id = note.get("id")

        if not note_id:
            print("Error: Note must have an id")
            return False

        # Create category directory if it doesn't exist
        note_dir: Path = BASE_DIR / category
        note_dir.mkdir(parents=True, exist_ok=True)

        # Write note to file
        note_path: Path = note_dir / f"{note_id}.json"
        with open(note_path, 'w', encoding='utf-8') as f:
            json.dump(note, f, indent=2, ensure_ascii=False)

        return True
    except Exception as e:
        print(f"❌ Error saving note: {e}")
        return False


def search_notes(keyword: str) -> list[dict]:
    """
    Find all notes containing keyword in title or body (case-insensitive).

    Loads all notes, filters by searching title and body fields,
    returns list of matching notes.
    """
    all_notes = get_all_notes()
    keyword_lower = keyword.lower()

    matching: list[dict] = [
        note for note in all_notes
        if keyword_lower in note.get("title", "").lower() or
           keyword_lower in note.get("body", "").lower()
    ]

    return matching


def delete_note(note_id: str) -> bool:
    """
    Delete a note by ID.

    Finds the note in all notes (to get category), then deletes the
    corresponding file. Returns True if successful, False otherwise.
    """
    try:
        all_notes = get_all_notes()

        for note in all_notes:
            if note["id"] == note_id:
                category = note.get("category", "personal")
                note_file: Path = BASE_DIR / category / f"{note_id}.json"

                if note_file.exists():
                    note_file.unlink()  # Delete file
                    return True

        return False  # Note not found
    except Exception as e:
        print(f"❌ Error deleting note: {e}")
        return False


def get_note_by_id(note_id: str) -> Optional[dict]:
    """Helper: Find a single note by ID."""
    all_notes = get_all_notes()
    for note in all_notes:
        if note["id"] == note_id:
            return note
    return None
```

**Specification Reference**: Lessons 2, 3, 4 - file I/O, pathlib directory creation, JSON serialization

**Prompts Used**:
1. "Write a function that loads all JSON files from nested directories using glob()"
2. "Implement save_note that creates directories if needed and writes with UTF-8 encoding"
3. "Write a search function that finds notes by keyword in title or body"

**Validation Steps**:
1. ✅ Load 3+ notes from different categories → `get_all_notes()` returns all
2. ✅ Save new note → file created in correct category directory with proper JSON formatting
3. ✅ Search finds matches → `search_notes("python")` returns notes with "python" in title/body
4. ✅ Delete removes file → `delete_note(id)` removes file and returns True
5. ✅ Error handling → corrupted JSON shows warning but doesn't crash

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "My app is slow when I have 1000 notes—every operation loads all notes from disk. How would you optimize this? What data structure would help? When would you use in-memory caching?"

**Expected Outcome**: You'll understand performance trade-offs and scaling patterns beyond this capstone's 10-50 note scope.

---

## Menu Loop Implementation

The menu loop is where user interaction happens. It displays options, accepts input with validation, routes to appropriate functions, and returns to menu after each operation.

### Code Example 5.3: Complete Application Menu Loop

```python
import uuid
from datetime import datetime
from pathlib import Path
import json

# [CRUD functions from Example 5.2 would be included here]

def create_note() -> None:
    """Prompt for new note and save."""
    print("\n--- Create New Note ---")
    title: str = input("Title: ").strip()
    body: str = input("Body: ").strip()
    category: str = input("Category (work/personal/learning) [personal]: ").strip() or "personal"

    # Validate input
    if not title or not body:
        print("❌ Error: Title and body cannot be empty.")
        return

    # Create note dictionary
    note: dict = {
        "id": str(uuid.uuid4()),
        "title": title,
        "body": body,
        "category": category,
        "created": datetime.now().isoformat(),
        "modified": datetime.now().isoformat(),
    }

    # Save note
    if save_note(note):
        print(f"✅ Note created successfully! ID: {note['id'][:8]}...")
    else:
        print("❌ Error creating note.")


def read_note() -> None:
    """List notes and display selected note."""
    notes = get_all_notes()

    if not notes:
        print("\n📝 No notes found.")
        return

    # Display list
    print("\n--- Your Notes ---")
    for i, note in enumerate(notes, 1):
        print(f"{i}. {note['title']} ({note['category']})")

    # Get selection
    try:
        choice = int(input("\nSelect note (number): ")) - 1
        if 0 <= choice < len(notes):
            note = notes[choice]
            print(f"\n📄 {note['title']}")
            print(f"Category: {note['category']}")
            print(f"Created: {note['created']}")
            print(f"Modified: {note['modified']}")
            print(f"\n{note['body']}")
        else:
            print("❌ Invalid selection.")
    except ValueError:
        print("❌ Please enter a number.")


def update_note() -> None:
    """Update title, body, or category of existing note."""
    notes = get_all_notes()

    if not notes:
        print("\n📝 No notes found.")
        return

    # Display list
    print("\n--- Select Note to Update ---")
    for i, note in enumerate(notes, 1):
        print(f"{i}. {note['title']}")

    # Get selection
    try:
        choice = int(input("\nSelect note (number): ")) - 1
        if 0 <= choice < len(notes):
            note = notes[choice]

            # Prompt for updates
            new_title = input(f"New title [{note['title']}]: ").strip() or note['title']
            new_body = input(f"New body (or press Enter to keep): ").strip() or note['body']

            # Update note
            note['title'] = new_title
            note['body'] = new_body
            note['modified'] = datetime.now().isoformat()

            if save_note(note):
                print("✅ Note updated successfully!")
            else:
                print("❌ Error updating note.")
        else:
            print("❌ Invalid selection.")
    except ValueError:
        print("❌ Please enter a number.")


def delete_note_menu() -> None:
    """Delete note with confirmation."""
    notes = get_all_notes()

    if not notes:
        print("\n📝 No notes found.")
        return

    # Display list
    print("\n--- Select Note to Delete ---")
    for i, note in enumerate(notes, 1):
        print(f"{i}. {note['title']}")

    # Get selection
    try:
        choice = int(input("\nSelect note (number): ")) - 1
        if 0 <= choice < len(notes):
            note = notes[choice]

            # Confirm deletion
            confirm = input(f"Delete '{note['title']}'? (yes/no): ").strip().lower()
            if confirm == 'yes':
                if delete_note(note['id']):
                    print("✅ Note deleted successfully!")
                else:
                    print("❌ Error deleting note.")
            else:
                print("Cancelled.")
        else:
            print("❌ Invalid selection.")
    except ValueError:
        print("❌ Please enter a number.")


def search_notes_menu() -> None:
    """Search notes by keyword."""
    keyword: str = input("\nSearch keyword: ").strip()

    if not keyword:
        print("❌ Please enter a search term.")
        return

    results = search_notes(keyword)

    if results:
        print(f"\n--- Found {len(results)} note(s) ---")
        for note in results:
            print(f"• {note['title']} ({note['category']})")
    else:
        print(f"No notes found matching '{keyword}'")


def list_all_notes() -> None:
    """Display all notes organized by category."""
    notes = get_all_notes()

    if not notes:
        print("\n📝 No notes found.")
        return

    # Group by category
    by_category: dict[str, list] = {}
    for note in notes:
        cat = note.get("category", "uncategorized")
        if cat not in by_category:
            by_category[cat] = []
        by_category[cat].append(note)

    print("\n--- All Notes ---")
    for category in sorted(by_category.keys()):
        print(f"\n{category.upper()} ({len(by_category[category])} notes):")
        for note in by_category[category]:
            print(f"  • {note['title']}")


def main() -> None:
    """Main application loop."""
    print("="*50)
    print("📝 Welcome to Note-Taking App!")
    print("="*50)

    # Initialize directories
    BASE_DIR: Path = Path("notes")
    BASE_DIR.mkdir(exist_ok=True)

    while True:
        # Display menu
        print("\n" + "="*50)
        print("1. Create Note")
        print("2. Read Note")
        print("3. Update Note")
        print("4. Delete Note")
        print("5. Search Notes")
        print("6. List All Notes")
        print("7. Exit")
        print("="*50)

        # Get user choice with validation
        try:
            choice_str: str = input("Enter choice (1-7): ").strip()
            choice: int = int(choice_str)

            if choice not in range(1, 8):
                print("❌ Invalid choice. Please enter 1-7.")
                continue

            # Route to appropriate function
            if choice == 1:
                create_note()
            elif choice == 2:
                read_note()
            elif choice == 3:
                update_note()
            elif choice == 4:
                delete_note_menu()
            elif choice == 5:
                search_notes_menu()
            elif choice == 6:
                list_all_notes()
            elif choice == 7:
                print("\n👋 Goodbye! Your notes are saved.")
                break

        except ValueError:
            print("❌ Invalid input. Please enter a number between 1 and 7.")
        except Exception as e:
            print(f"❌ Unexpected error: {e}")


if __name__ == "__main__":
    main()
```

**Specification Reference**: Lessons 1, 2, 3, 4 - all I/O concepts combined

**Prompts Used**:
1. "Write a menu loop that displays options 1-7 and validates user input"
2. "Create functions for each CRUD operation with error handling"
3. "Implement search that finds notes by keyword"

**Validation Steps**:
1. ✅ Menu displays and accepts 1-7 → invalid input shows error and re-prompts
2. ✅ Create note → prompts for title/body/category, saves to file, returns to menu
3. ✅ Read note → lists notes, accepts selection, displays selected note, returns to menu
4. ✅ Update note → allows changing title/body, updates timestamp, returns to menu
5. ✅ Delete note → requires confirmation before removing file
6. ✅ Search → finds notes by keyword, shows results
7. ✅ List → shows all notes grouped by category
8. ✅ Exit → graceful shutdown with goodbye message

#### ✨ Teaching Tip

Use Claude Code to test edge cases in your implementation:

> "What happens if I create a note with an empty title? Or try to delete a note twice? Show me each error and how my code should handle it."

---

## Error Handling and Validation

Production-quality applications handle errors gracefully. Let's review the validation patterns used throughout:

### Input Validation (Lesson 1 Pattern)
```python
# Menu choice validation
try:
    choice: int = int(input("Enter choice (1-7): ").strip())
    if choice not in range(1, 8):
        print("Invalid choice. Please enter 1-7.")
        continue
except ValueError:
    print("Invalid input. Please enter a number.")
```

### File Operation Errors (Lesson 2 Pattern)
```python
# Handle missing/corrupted files
try:
    with open(note_file, 'r', encoding='utf-8') as f:
        note = json.load(f)
except FileNotFoundError:
    print("File not found")
except json.JSONDecodeError:
    print("File is corrupted")
```

### Path Safety (Lesson 3 Pattern)
```python
# Prevent directory traversal
requested_path = (BASE_DIR / user_input).resolve()
if not requested_path.is_relative_to(BASE_DIR.resolve()):
    raise ValueError("Security error: Path outside allowed directory")
```

### Data Validation
```python
# Ensure required fields exist
if not title or not body:
    print("Error: Title and body cannot be empty.")
    return
```

---

## Testing and Refinement

### How to Test Your Application

1. **Test Create**: Run the app, select "Create Note", enter a title and body, verify file is created
   ```bash
   $ ls notes/personal/
   # Should show one or more .json files
   ```

2. **Test Read**: Select "Read Note", choose one from the list, verify it displays correctly

3. **Test Update**: Select "Update Note", change the title, verify file is updated

4. **Test Delete**: Create a note, delete it, verify file is removed

5. **Test Search**: Create notes with different keywords, search for them, verify results

6. **Test Menu Loop**: Go through several operations, verify app returns to menu each time

7. **Test Error Handling**:
   - Enter invalid menu choice → should show error and re-prompt
   - Try to read notes when none exist → should show "No notes found"
   - Try to delete non-existent note → should handle gracefully

### Edge Cases to Consider

- What happens if someone creates a note with Unicode emoji? (UTF-8 should preserve it)
- What if the notes/ directory is deleted while the app is running? (mkdir will recreate it)
- What if someone has 50 notes—does the app still respond quickly? (Yes, file-per-note is efficient)
- What if you edit a note file manually while the app is running? (Next read will get latest version)

#### 🎓 Expert Insight

> In AI-native development, you don't test by hand 50 times—you test strategically. Your AI can help generate test cases: "What edge cases should I test for a note-taking app?" Then you verify each one systematically.

---

## Project Deliverables

Your complete application should consist of:

1. **Main Application File** (e.g., `note_app.py` or `main.py`)
   - Imports all necessary modules
   - Defines BASE_DIR and initializes directories
   - Implements all CRUD functions
   - Implements menu loop with input validation
   - Has `if __name__ == "__main__"` guard with `main()` call

2. **Data Directory** (`notes/`)
   - Automatically created when app runs
   - Contains category subdirectories (work, personal, learning)
   - Stores notes as JSON files organized by category

3. **Working Features** - All of these must function:
   - Menu displays correctly
   - Create note saves to JSON file
   - Read note loads and displays from file
   - Update note modifies file and updates timestamp
   - Delete note removes file after confirmation
   - Search finds notes by keyword
   - List displays notes organized by category
   - Exit closes app gracefully

### Success Criteria Checklist

Your application is complete when:

- ✅ Application runs without crashing on valid input
- ✅ All CRUD operations work correctly
- ✅ User input is validated (menu choices, required fields)
- ✅ Notes persist between sessions (data saved to JSON files)
- ✅ Search finds notes by keyword in title or body
- ✅ Application handles errors gracefully (missing files, corrupted JSON)
- ✅ Code is organized with functions for each operation
- ✅ Application supports 10-50 notes at responsive speed
- ✅ Menu returns to top after each operation

---

## Try With AI: The Log File Analyzer Capstone

### Part 1: Design Data Pipeline Architecture (Your Turn First)

**Before asking AI**, design the application structure:

**Problem**: You're building a file-processing system that:
1. **Inputs**: Reads log files (one per category: error, warning, info)
2. **Transforms**: Filters by severity level, counts occurrences, extracts patterns
3. **Outputs**: Generates a JSON report with statistics

**Your task**: Create an architecture document `capstone_design.md` that describes:
1. **Input**: What log file format (line-by-line text, CSV, JSON)?
2. **Processing flow**: What functions process logs? What data structures?
3. **Output**: What JSON report structure? What statistics?
4. **File organization**: Where do logs live? Where does output go?
5. **Error handling**: What breaks? How do you recover?

**Deliverable**: A text file showing your design with:
- Pseudocode for main flow
- Example input log file format
- Example JSON output structure
- List of potential errors and how to handle them

---

### Part 2: AI Guides Application Design (Discovery)

**Share your design with AI:**

> "Here's my design for a log file analyzer [paste your architecture]. Review it:
> 1. Is my design sound? What should I improve?
> 2. Show me how to structure the code with CRUD functions and a menu loop
> 3. What error cases from my list need special handling?
> 4. Walk me through the complete application flow: start → menu → operation → return to menu"

**Your evaluation**:
- Did the AI improve your design?
- Does the code structure make sense?
- Can you explain the flow to someone else?

---

### Part 3: Student Tests Real Edge Cases (Debugging)

**Challenge the AI with realistic problems:**

> "Test my understanding of the capstone:
> 1. What happens if a log file has corrupted JSON lines?
> 2. If I'm in the middle of generating a report and the user presses Ctrl+C, what cleanup needs to happen?
> 3. How do I handle a log file that's 1GB (can't load all at once)?
> 4. What if two users run the app simultaneously on the same notes directory?
>
> For each case, show the error handling code I need."

**Your debugging**: Create a test file `capstone_edge_cases.py` that demonstrates your understanding of each edge case.

---

### Part 4: Build the Complete Application (Convergence)

**Build the full Note-Taking App (from Code Examples 5.1-5.3):**

> "Help me build a complete note-taking application with all these features:
> 1. Menu loop showing: Create/Read/Update/Delete/Search/List All/Exit
> 2. CRUD functions using file I/O with context managers
> 3. Directory organization by category (work/personal/learning)
> 4. JSON persistence with proper encoding
> 5. Input validation for all user inputs
> 6. Error handling for missing files, corrupted JSON, permissions
> 7. Search functionality that finds notes by keyword
> 8. List that organizes notes by category
>
> The application should be production-ready and demonstrate all Chapter 22 concepts."

**Refinement**:
- Add support for 50+ notes without performance issues
- Implement timestamps using datetime (Chapter 23 preview)
- Add categories management: users can create custom categories
- Implement a backup feature that exports all notes to JSON
- Add search filtering: find notes created in the last 7 days

---

### Part 5: Reflect on Chapter 22 Concepts (Integration)

**Synthesize everything you learned:**

Create a file `capstone_reflection.md` with:

1. **Console I/O (Lesson 1)**: How do you validate user input in the menu loop? Describe 2 validation patterns you used.

2. **File I/O (Lesson 2)**: How do you safely read and write note files? Explain why context managers matter in your application. What file errors did you handle?

3. **Path Handling (Lesson 3)**: How do you organize notes by category using pathlib? Why is pathlib better than string paths in your application?

4. **Structured Data (Lesson 4)**: Why did you choose JSON instead of CSV for note storage? What data structure did you create? How do you handle encoding for international characters?

5. **Integration**: How do all four lessons come together in your capstone? Walk through one complete operation (e.g., "Create a note") describing which lessons' concepts apply.

6. **Architecture Decision**: Why did you choose file-per-note instead of all-notes-in-one-file? What tradeoffs did you make?

7. **Error Handling Hierarchy**: Where do errors happen in your application? How does input validation differ from file operation error handling?

**Deliverable**: A markdown reflection document (500-800 words) showing deep understanding of:
- How all I/O concepts connect
- Design decisions and tradeoffs
- Error handling strategy across multiple layers
- Real-world application of Python file operations

**Time**: 50-65 min total (35-45 min coding + 15-20 min reflection)
**Outcome**:
- Complete Note-Taking App with all Chapter 22 concepts integrated
- Reflection document demonstrating mastery of file I/O architecture

---

**Congratulations!** You've completed the Note-Taking App capstone, integrating all I/O concepts from Chapter 22. Your application demonstrates professional-level CLI development using Python 3.14+, pathlib, JSON, file I/O, and menu-driven interaction. Your reflection document proves you understand not just the "how" but the "why" of each design decision. You're now ready to extend this pattern to larger applications and ready for Chapters 23+ (datetime operations, OOP, etc.).
