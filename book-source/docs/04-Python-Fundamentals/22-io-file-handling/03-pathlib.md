---
title: "Cross-Platform Path Handling with pathlib"
chapter: 22
lesson: 3
duration_minutes: 75

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Creating Path Objects with pathlib"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create Path objects using Path(), string literals, and / operator; understand path immutability"

  - name: "Checking File and Directory Existence"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use .exists(), .is_file(), .is_dir() to determine file types before operations"

  - name: "Creating and Organizing Directories"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use .mkdir(parents=True, exist_ok=True) to create nested directories safely"

  - name: "Listing and Globbing Files"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use .iterdir() and .glob() patterns to find files matching criteria"

  - name: "Understanding Cross-Platform Compatibility"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why pathlib is preferred over hardcoded paths and os.path, understand Windows vs Unix path differences"

  - name: "Path Traversal Security and Validation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Cybersecurity"
    measurable_at_this_level: "Student can use .resolve() to canonicalize paths and validate that resolved paths stay within allowed directory bounds"

learning_objectives:
  - objective: "Create cross-platform file paths using pathlib.Path that work identically on Windows, Mac, and Linux"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write code using pathlib / operator to construct paths"

  - objective: "Check if files and directories exist before operations using .exists(), .is_file(), .is_dir()"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write defensive code that verifies file existence before reading"

  - objective: "Create nested directories safely using .mkdir(parents=True, exist_ok=True)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Create multi-level directory structures programmatically"

  - objective: "Find files using glob patterns and .iterdir()"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write code that searches for files by pattern"

  - objective: "Understand and prevent path traversal security attacks using .resolve() and .is_relative_to()"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explain security risks and implement boundary validation"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (Path objects, existence checking, directory creation, glob patterns, cross-platform separation, path security) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore recursive glob patterns (**/*.txt), implement path validation for user input, design secure file access systems"
  remedial_for_struggling: "Focus on basic Path() and exists() checks first; practice / operator before glob patterns; use reference solutions for security section"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ain/specs/001-part-4-chapter-22/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Cross-Platform Path Handling with pathlib

## Introduction: Why pathlib Is Essential

In Lesson 2, you learned to read and write files using context managers. But there's a problem you might have glossed over: **how do you construct file paths?**

If you're on Windows, paths use backslashes: `C:\Users\YourName\Documents\notes.txt`. On Mac and Linux, they use forward slashes: `/home/yourname/documents/notes.txt`. If you hardcode paths in your code like this:

```python
file_path = "C:\\Users\\YourName\\notes.txt"  # Only works on Windows!
```

Your code breaks on other systems. This isn't a theoretical problem—it's real. When you share code with colleagues, deploy to a server, or try to run your application on a different operating system, hardcoded paths fail.

The solution is **pathlib**, Python's modern way to handle file paths. Instead of treating paths as strings, pathlib gives you **Path objects** that understand your operating system and automatically use the correct separators. Write once, run anywhere—that's the promise of pathlib.

This lesson teaches you to work with paths as first-class Python objects, check if files exist, create directories, find files by pattern, and prevent security vulnerabilities. By the end, you'll be able to build file-based applications that work reliably across Windows, Mac, and Linux.

## Understanding pathlib vs os.path: A Better Way to Build Paths

For decades, Python programmers used the `os.path` module to work with file paths. It works, but it's awkward. Here's why pathlib is better:

**With os.path (old approach)**:
```python
import os
file_path = os.path.join("data", "notes", "important.txt")
# Works, but returns a string—you don't have file operations available
file_exists = os.path.exists(file_path)  # Separate function call
```

**With pathlib (modern approach)**:
```python
from pathlib import Path
file_path: Path = Path("data") / "notes" / "important.txt"
# Returns a Path object with methods for operations
if file_path.exists():  # Use . notation, it's an object
    content = file_path.read_text()
```

Notice the difference? With pathlib:
- You use the `/` operator to join paths (intuitive and readable)
- Path objects have methods like `.exists()`, `.read_text()`, `.write_text()`
- The same code works on Windows, Mac, and Linux without modification
- Path objects are immutable—operations return new Path objects

This is an example of **object-oriented thinking**: paths aren't just strings, they're objects with capabilities. You'll see this pattern throughout professional Python code.

#### 💬 AI Colearning Prompt

> "Explain how the `/` operator works with Path objects. Why is this better than using string concatenation like `"config" + "/" + "app.json"`?"

**Expected Outcome**: You'll understand that Path objects handle OS-specific separators automatically, making your code cross-platform.

## Creating and Manipulating Paths

Let's start with the fundamentals: creating Path objects and accessing their properties.

**Creating Path objects** is straightforward:

```python
from pathlib import Path

# Create a path to a file relative to current directory
config_file: Path = Path("config.json")

# Create a path using the / operator (joining segments)
notes_dir: Path = Path("data") / "notes"

# Combine both patterns
specific_note: Path = notes_dir / "meeting.txt"

# Get your home directory (cross-platform)
home: Path = Path.home()
```

The `/` operator is the key insight here. Instead of worrying about backslashes or forward slashes, you just use `/` and Python handles the OS-specific details.

**Path properties** let you extract useful information:

```python
config_file: Path = Path("config") / "app.json"

print(f"Full path: {config_file}")          # config/app.json (or config\app.json on Windows)
print(f"File name: {config_file.name}")     # app.json
print(f"Suffix (extension): {config_file.suffix}")  # .json
print(f"Stem (name without extension): {config_file.stem}")  # app
print(f"Parent directory: {config_file.parent}")     # config
print(f"Parent's parent: {config_file.parent.parent}")  # . (current dir)
```

These properties are useful for extracting information about files without having to manually parse strings.

#### 🎓 Instructor Commentary

> In AI-native development, you don't hardcode paths with backslashes or forward slashes—you use pathlib. The syntax is cheap; understanding that your code must work on Windows, Mac, AND Linux is gold. Your AI can generate paths; your job is specifying the directory structure.

## Checking Existence and File Types: Defensive Programming

Before you try to read a file, it's wise to check that it actually exists. This prevents crashes from attempting to open files that don't exist.

```python
from pathlib import Path

config_path: Path = Path("config.json")

# Check if path exists (could be file or directory)
if config_path.exists():
    print(f"{config_path} exists")
else:
    print(f"{config_path} not found")

# Check if it's specifically a file
if config_path.is_file():
    print(f"{config_path} is a file")

# Check if it's specifically a directory
if config_path.is_dir():
    print(f"{config_path} is a directory")
```

Here's a practical pattern: **check before operating**:

```python
data_dir: Path = Path("data")

if not data_dir.exists():
    print(f"Error: {data_dir} directory doesn't exist yet")
else:
    print(f"Found {data_dir}")
    # Safe to read files from this directory
    file_list: list[Path] = list(data_dir.glob("*.txt"))
    print(f"Found {len(file_list)} text files")
```

This defensive approach prevents your program from crashing when files or directories are missing.

## Creating Directories: Building Nested Structures

Often, your application needs to organize files into directories. Rather than creating directories manually, you can ask Python to create them.

The `.mkdir()` method has two important parameters:

- `parents=True` — Create parent directories if they don't exist
- `exist_ok=True` — Don't raise an error if the directory already exists

```python
from pathlib import Path

# Create a nested directory structure
notes_dir: Path = Path("data") / "notes" / "2025" / "november"

# Create all missing parent directories, don't error if it exists
notes_dir.mkdir(parents=True, exist_ok=True)

print(f"Directory created (or already existed): {notes_dir}")
```

**Without these parameters**, `.mkdir()` would fail if parent directories don't exist:

```python
# This fails if "data" or "data/notes" don't exist
bad_path: Path = Path("data") / "notes" / "2025" / "november"
bad_path.mkdir()  # FileNotFoundError!
```

Here's a practical pattern for applications that organize data by category:

```python
base_dir: Path = Path("notes")
categories: list[str] = ["work", "personal", "learning"]

for category in categories:
    category_dir: Path = base_dir / category
    category_dir.mkdir(parents=True, exist_ok=True)
    print(f"Created: {category_dir}")
```

This pattern will be essential for your Capstone project (Lesson 5).

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Generate code that creates a directory structure for 5 projects, each with 'src', 'tests', and 'docs' subdirectories. Then show me how to use glob() to count total Python files across all projects."

**Expected Outcome**: You'll see pathlib for scaling (creating many dirs) and glob for discovering files across your entire project structure.

## Finding Files with Glob Patterns

You now know how to create directories and check if files exist. Next: how do you **find** files matching a pattern?

The `.glob()` method searches for files matching a pattern. This is powerful for applications that need to discover files dynamically.

**Basic glob patterns**:

```python
from pathlib import Path

notes_dir: Path = Path("notes")
notes_dir.mkdir(exist_ok=True)

# Create some sample files to find
(notes_dir / "meeting.txt").write_text("Team standup notes")
(notes_dir / "todo.txt").write_text("Buy groceries")
(notes_dir / "notes.md").write_text("# Python Study")
(notes_dir / "archive.txt").write_text("Old notes")

# Find all .txt files (not .md)
txt_files: list[Path] = list(notes_dir.glob("*.txt"))
print(f"Found {len(txt_files)} .txt files:")
for file in txt_files:
    print(f"  - {file.name}")

# Find all files (any type)
all_files: list[Path] = list(notes_dir.glob("*"))
print(f"All files: {[f.name for f in all_files]}")
```

**Common glob patterns**:
- `*.txt` — All files ending in .txt
- `*.py` — All Python files
- `test_*` — All files starting with "test_"
- `*` — All items in the directory
- `**/*.txt` — All .txt files recursively (Python 3.12+) [research current version]

Using `.glob()` instead of manually listing files is more maintainable: your code doesn't hardcode filenames, it discovers them dynamically.

#### ✨ Teaching Tip

> Use Claude Code to explore glob patterns: "What would `data/**/2025/*.txt` match in a directory tree? Show me an example of a directory structure and which files it would find."

## Code Example 3.1: Creating Paths with pathlib

Let's write code that demonstrates Path object creation and properties. This example shows how pathlib handles cross-platform compatibility automatically.

**Specification**: Create paths using different syntax, access properties, and show how pathlib abstracts away OS-specific separators.

**Prompt**: "Show me how to create and manipulate file paths using pathlib, including accessing properties like name, suffix, and parent."

```python
from pathlib import Path

# Create Path objects using different approaches
home: Path = Path.home()  # User's home directory
docs: Path = Path("documents")  # Relative path
config_file: Path = Path("config") / "app.json"  # Using / operator

# Access path properties
print(f"Full path: {config_file}")          # config/app.json (auto-formatted for OS)
print(f"File name: {config_file.name}")     # app.json
print(f"Suffix: {config_file.suffix}")      # .json
print(f"Stem: {config_file.stem}")          # app
print(f"Parent: {config_file.parent}")      # config

# The / operator works cross-platform (Windows uses \, Unix uses /)
notes_dir: Path = Path("data") / "notes" / "personal"
print(f"Cross-platform path: {notes_dir}")

# Verify the path is what we expect
print(f"Path type: {type(notes_dir)}")  # <class 'pathlib.PosixPath'> or WindowsPath
```

**Expected Output** (on Mac/Linux):
```
Full path: config/app.json
File name: app.json
Suffix: .json
Stem: app
Parent: config
Cross-platform path: data/notes/personal
Path type: <class 'pathlib.PosixPath'>
```

**Validation Steps**:
1. Path objects are created successfully
2. The `/` operator works as expected
3. Properties return correct values
4. Same code produces correct paths on different operating systems

**Key Insight**: Notice that `.parent` returns a Path object, not a string. This means you can chain operations: `config_file.parent.parent` to go up multiple levels.

## Code Example 3.2: Checking File Existence and Type

This example demonstrates the defensive programming pattern: always check if files/directories exist before operating on them.

**Specification**: Create validation checks for files and directories, showing the pattern for safe file operations.

**Prompt**: "Show me how to check if files and directories exist before operations using pathlib, and how to distinguish between files and directories."

```python
from pathlib import Path

# Create some test paths
config_path: Path = Path("config.json")
data_dir: Path = Path("data")

# Check existence and type
print(f"Does {config_path} exist? {config_path.exists()}")
print(f"Does {data_dir} exist? {data_dir.exists()}")

# Check specific types
if config_path.exists():
    if config_path.is_file():
        print(f"{config_path} is a file")
    elif config_path.is_dir():
        print(f"{config_path} is a directory")

# Common pattern: check and act
if not data_dir.exists():
    print(f"Creating {data_dir} because it doesn't exist")
    data_dir.mkdir(parents=True, exist_ok=True)
else:
    print(f"{data_dir} already exists")
    file_count: int = len(list(data_dir.glob("*")))
    print(f"{data_dir} contains {file_count} items")
```

**Expected Output** (if paths don't exist):
```
Does config.json exist? False
Does data exist? False
Creating data because it doesn't exist
data already exists
data contains 0 items
```

**Validation Steps**:
1. `.exists()` correctly reports missing files
2. `.is_file()` and `.is_dir()` distinguish types
3. Directory can be created safely
4. Subsequent existence checks show created directory

**Key Insight**: This pattern prevents crashes. Your code never tries to read missing files or create existing directories.

## Code Example 3.3: Creating Nested Directories

This example shows how to safely create multi-level directory structures—essential for the capstone project that organizes notes by category.

**Specification**: Create nested directories with `parents=True, exist_ok=True`, demonstrate the pattern for building application directory structures.

**Prompt**: "Show me how to create nested directories safely, including when parent directories don't exist. Demonstrate creating multiple category subdirectories."

```python
from pathlib import Path

# Create nested directories safely
notes_base: Path = Path("data") / "notes" / "2025" / "november"

# parents=True creates all missing parent directories
# exist_ok=True doesn't raise error if directory already exists
notes_base.mkdir(parents=True, exist_ok=True)

print(f"Created: {notes_base}")

# Create category subdirectories (common for applications)
categories: list[str] = ["work", "personal", "learning"]
for category in categories:
    category_dir: Path = notes_base / category
    category_dir.mkdir(parents=True, exist_ok=True)
    print(f"Created: {category_dir}")

# Verify the structure was created
if notes_base.exists():
    print(f"\nDirectory structure created at: {notes_base}")
    subdirs: list[Path] = [d for d in notes_base.iterdir() if d.is_dir()]
    print(f"Subdirectories: {[d.name for d in subdirs]}")
```

**Expected Output**:
```
Created: data/notes/2025/november
Created: data/notes/2025/november/work
Created: data/notes/2025/november/personal
Created: data/notes/2025/november/learning

Directory structure created at: data/notes/2025/november
Subdirectories: ['work', 'personal', 'learning']
```

**Validation Steps**:
1. All directories created (including parents)
2. Running again doesn't error (exist_ok=True works)
3. `.iterdir()` correctly lists created subdirectories
4. Structure is exactly what was specified

**Key Insight**: This pattern is used in Lesson 5's Capstone project to organize notes by category. You can create complex directory hierarchies programmatically.

## Code Example 3.4: Finding Files with Glob Patterns

This example demonstrates how to search for files dynamically using glob patterns—critical for applications that need to discover files without hardcoding paths.

**Specification**: Create sample files and demonstrate glob patterns to find specific file types.

**Prompt**: "Show me how to find all files matching a pattern in a directory using glob(). Demonstrate finding files by extension."

```python
from pathlib import Path

# Create sample directory and files
notes_dir: Path = Path("notes")
notes_dir.mkdir(exist_ok=True)

# Create some test files
(notes_dir / "meeting.txt").write_text("Team standup")
(notes_dir / "todo.txt").write_text("Buy groceries")
(notes_dir / "notes.md").write_text("# Python Study")
(notes_dir / "archive.txt").write_text("Old data")

# Find all .txt files
txt_files: list[Path] = list(notes_dir.glob("*.txt"))
print(f"Found {len(txt_files)} .txt files:")
for file in txt_files:
    print(f"  - {file.name}")

# Find all files of any type
all_files: list[Path] = list(notes_dir.glob("*"))
print(f"\nAll files in {notes_dir.name}:")
print(f"  Files: {[f.name for f in all_files if f.is_file()]}")

# Count files by type
txt_count: int = len(list(notes_dir.glob("*.txt")))
md_count: int = len(list(notes_dir.glob("*.md")))
print(f"\nSummary: {txt_count} .txt files, {md_count} .md files")
```

**Expected Output**:
```
Found 3 .txt files:
  - meeting.txt
  - todo.txt
  - archive.txt

All files in notes:
  Files: ['meeting.txt', 'todo.txt', 'notes.md', 'archive.txt']

Summary: 3 .txt files, 1 .md files
```

**Validation Steps**:
1. Files are created successfully
2. Glob pattern `*.txt` finds only .txt files
3. Glob pattern `*` finds all files
4. File counting works correctly

**Key Insight**: Using glob patterns makes your code flexible. If new files are added to the directory, your code automatically discovers them without modification.

## Code Example 3.5: Path Traversal Security and Validation

This is the most important example. When users provide file paths as input, malicious users might try to access files outside your intended directory (path traversal attack). This example shows how to defend against that.

**Specification**: Implement secure path validation that prevents users from accessing files outside an allowed base directory.

**Prompt**: "Show me how to safely read files from a user-provided path without allowing access outside a directory. Include security validation using .resolve() and .is_relative_to()."

```python
from pathlib import Path

def read_note(base_dir: Path, user_input: str) -> str | None:
    """Read a note file with security validation to prevent path traversal attacks."""

    # Resolve the requested path to its canonical form
    # This converts relative paths (with .., .) to absolute paths
    requested_path: Path = (base_dir / user_input).resolve()

    # CRITICAL: Check that resolved path stays within base directory
    # is_relative_to() returns True only if requested_path is under base_dir
    if not requested_path.is_relative_to(base_dir.resolve()):
        print(f"Security error: Attempted access outside allowed directory")
        return None

    # Now safe to check other properties
    if not requested_path.exists():
        print(f"File not found: {user_input}")
        return None

    if not requested_path.is_file():
        print(f"Not a file: {user_input}")
        return None

    try:
        return requested_path.read_text(encoding='utf-8')
    except Exception as e:
        print(f"Error reading file: {e}")
        return None

# Demonstration
base_dir: Path = Path("notes")
base_dir.mkdir(exist_ok=True)

# Create a safe note
(base_dir / "note1.txt").write_text("Important note")

# Safe access: reads the file
print("=== Safe Access ===")
content = read_note(base_dir, "note1.txt")
if content:
    print(f"Read: {content}")

# Unsafe attempt: blocked by security check
print("\n=== Attempted Path Traversal ===")
content = read_note(base_dir, "../../etc/passwd")
# Output: Security error: Attempted access outside allowed directory

# Another unsafe attempt: using relative path tricks
print("\n=== Another Traversal Attempt ===")
content = read_note(base_dir, "../outside_file.txt")
# Output: Security error: Attempted access outside allowed directory
```

**Expected Output**:
```
=== Safe Access ===
Read: Important note

=== Attempted Path Traversal ===
Security error: Attempted access outside allowed directory

=== Another Traversal Attempt ===
Security error: Attempted access outside allowed directory
```

**Validation Steps**:
1. Safe file access works (note1.txt is read)
2. Path traversal with `../../etc/passwd` is blocked
3. Relative path tricks with `../outside_file.txt` are blocked
4. `.resolve()` canonicalizes paths before checking

**Key Insight**: This pattern is critical for applications that take user input for filenames. The two-step process:
1. **Resolve** the path to canonical form (resolves `..` and `.`)
2. **Validate** the resolved path is still within the allowed base directory

This prevents attackers from escaping your intended directory.

## Practice Exercise 1: Create Nested Directories

**Task**: Create a nested directory structure `projects/2025/november` using pathlib with `mkdir(parents=True, exist_ok=True)`. Verify the directories exist using `.exists()`.

**Acceptance Criteria**:
- All directories created successfully
- Running the code twice doesn't error (exist_ok=True works)
- `.exists()` returns True for created directories

**Validation Approach**:
```python
# After your code, add this validation
test_path: Path = Path("projects") / "2025" / "november"
assert test_path.exists(), "Directory was not created"
assert test_path.is_dir(), "Path exists but is not a directory"
print("✓ All checks passed")
```

## Practice Exercise 2: List Files by Extension

**Task**: Create a directory with several files of different types (.txt, .py, .md). Write code using `.glob()` to count how many files exist of each type.

**Acceptance Criteria**:
- Code creates at least 3 files of different types
- `.glob()` patterns correctly identify files by extension
- Counts are accurate

**Validation Approach**:
```python
# After your code, verify counts
txt_files: list[Path] = list(test_dir.glob("*.txt"))
py_files: list[Path] = list(test_dir.glob("*.py"))
assert len(txt_files) == 2, f"Expected 2 .txt files, got {len(txt_files)}"
assert len(py_files) == 1, f"Expected 1 .py file, got {len(py_files)}"
print("✓ All file discovery checks passed")
```

## Practice Exercise 3: Path Validation Preventing Traversal

**Task**: Implement a simple version of the `read_note()` function from Code Example 3.5. Test it with both safe filenames and attempted path traversal patterns.

**Acceptance Criteria**:
- Safe filenames are allowed
- `../` patterns are blocked
- `../../` patterns are blocked
- Clear error messages indicate why access was denied

**Validation Approach**:
```python
# Test safe access
result = read_note(test_dir, "safe.txt")
assert result is not None, "Safe access should succeed"

# Test traversal blocking
result = read_note(test_dir, "../outside.txt")
assert result is None, "Traversal should be blocked"

print("✓ All security checks passed")
```

### Python 3.14 New Features

**Python 3.14 added powerful new path manipulation methods** that make file operations even easier:

- **`Path.copy(destination)`**: Copy a file to a new location
- **`Path.copy_into(directory)`**: Copy a file into a directory (name preserved)
- **`Path.move(destination)`**: Move/rename a file atomically
- **`Path.move_into(directory)`**: Move a file into a directory (name preserved)

These methods handle edge cases automatically and work recursively for directories.

**Example** (Python 3.14+):
```python
from pathlib import Path

# Old way (using shutil)
import shutil
shutil.copy("source.txt", "dest.txt")

# New way (Python 3.14+)
source: Path = Path("source.txt")
source.copy(Path("dest.txt"))  # Built into pathlib!

# Copy into directory (preserves name)
source.copy_into(Path("backup/"))  # Creates backup/source.txt
```

**Why this matters for AI-native development**: Your AI companion can now use pathlib for ALL file operations—no need to import shutil for copying/moving. Simpler code = fewer dependencies = easier to understand and maintain.

**For this chapter**: We focus on the core pathlib patterns that work in Python 3.4+. When you're ready to use Python 3.14+ exclusively, explore these new methods with your AI tool.

## Try With AI

Use your preferred AI companion tool (Claude Code, Gemini CLI, or ChatGPT web) to deepen your understanding of pathlib and explore advanced patterns.

### Prompt 1: Remember and Understand (Foundational)

**Ask your AI**: "What is pathlib and why is it better than using os.path or string concatenation for file paths?"

**Expected Outcome**: You'll understand pathlib solves cross-platform path problems elegantly and why object-oriented path handling is superior to string manipulation.

### Prompt 2: Apply (Practical Implementation)

**Ask your AI**: "Write code that creates a directory structure with subdirectories for each month of 2025, then tells me how many directories were created."

**Expected Outcome**: You'll see pathlib's power for organizing data programmatically and understand `.mkdir(parents=True)` in a realistic context.

### Prompt 3: Analyze (Deeper Understanding)

**Ask your AI**: "Compare these three approaches: (1) hardcoding paths like 'data/notes/file.txt', (2) using os.path.join(), (3) using pathlib with the / operator. Which works on all operating systems and why?"

**Expected Outcome**: You'll understand the historical evolution of path handling in Python and why pathlib is the modern standard adopted by professional developers.

### Prompt 4: Synthesize and Create (Cognitive Closure)

**Ask your AI**: "Design a file management system that finds all notes in a directory tree, validates they're not trying to escape the notes directory using path traversal, and displays them organized by type. What pathlib methods would you use?"

**Expected Outcome**: You've connected pathlib to security, file discovery, and real-world application patterns. You're ready for Lesson 4 (structured data formats) and especially Lesson 5 (Capstone Note-Taking App) which heavily uses pathlib for organizing files by category.
