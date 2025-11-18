# Feature Specification: Chapter 22 - IO and File Handling

**Feature Branch**: `001-part-4-chapter-22`
**Created**: 2025-11-09
**Status**: Draft
**Input**: User description: "Chapter 22: IO and File Handling - Console I/O, File I/O, pathlib, CSV/JSON with Note-Taking App capstone"

## Chapter Metadata

- **Chapter Number**: 22
- **Title** (from chapter-index.md): "IO and File Handling"
- **Part**: 4 (Python Fundamentals)
- **Complexity Tier**: Intermediate (Chapters 17-23)
- **CEFR Range**: A2-B1
- **Max Concepts/Lesson**: 7
- **Prerequisites**: Chapters 1-21 (especially Python foundations, control flow, functions, exceptions, data structures)
- **Learning Pattern**: AI-Native Learning (describe intent → explore → validate → learn from errors)
- **Target Audience**: Aspiring/Professional/Founders with graduated complexity
- **Estimated Time**: 5-6 hours total

## Clarifications

### Session 2025-11-09

- Q: Data volume & scale for Note-Taking App capstone → A: Dozens of notes (10-50) - Simple scale for basic proof-of-concept
- Q: CLI interaction pattern for capstone → A: Menu-driven interactive loop - Display menu, user enters choice, app executes action, returns to menu
- Q: Path traversal security approach → A: Use pathlib.resolve() + bounded directory checking - Canonical paths, verify result starts with allowed base path

## Success Evals (Business-Goal-Aligned)

### Reader Comprehension Evals

- **EVAL-001**: 75%+ can explain difference between console I/O and file I/O (concept quiz)
- **EVAL-002**: 80%+ can describe when to use pathlib vs open() (application scenario)
- **EVAL-003**: 70%+ can identify security risks in file handling code (critical thinking)

### Skill Acquisition Evals

- **EVAL-004**: 85%+ can write code to read file with proper error handling (hands-on exercise)
- **EVAL-005**: 75%+ can use pathlib for cross-platform file operations (practical task)
- **EVAL-006**: 70%+ can work with CSV/JSON formats correctly (capstone requirement)

### Engagement Evals

- **EVAL-007**: 80%+ complete capstone project (Note-Taking App)
- **EVAL-008**: Average 3.5+ "Try With AI" prompts attempted per lesson
- **EVAL-009**: 70%+ report understanding improves after AI exploration

### Accessibility Evals

- **EVAL-010**: Reading level: Grade 8-9 (intermediate, not beginner)
- **EVAL-011**: Code complexity: Intermediate (functions, error handling, type hints)
- **EVAL-012**: Cognitive load: 7 concepts max per lesson (A2-B1 appropriate)

**Connection to Business Goal**: Students completing Chapter 22 can independently build file-based applications (CLI tools, data processors, config managers) using AI assistance, preparing them for professional development work in Parts 5+.

## User Scenarios & Testing

### User Story 1 - Console Interaction with Validation (Priority: P1)

As an intermediate Python learner, I want to understand how to safely gather user input from the console and display formatted output, so I can build interactive CLI programs that handle invalid input gracefully.

**Why this priority**: Console I/O is the foundation for all user interaction in CLI applications. Without this skill, students cannot build the capstone project or any interactive tool.

**Independent Test**: Student can write a program that asks for user age, validates it's a positive integer, handles invalid input with clear error messages, and displays formatted output using f-strings.

**Acceptance Scenarios**:

1. **Given** user runs the program, **When** they enter a valid positive integer for age, **Then** program displays formatted message with their age and continues
2. **Given** user runs the program, **When** they enter invalid input (negative number, text, empty), **Then** program displays helpful error message and asks again
3. **Given** user runs the program, **When** they enter valid input after 2 failed attempts, **Then** program accepts input and continues normally

---

### User Story 2 - Safe File Operations (Priority: P1)

As an intermediate Python learner, I want to understand how to read from and write to files safely using context managers and error handling, so I can build programs that persist data without corrupting files or leaking resources.

**Why this priority**: File I/O is essential for any real-world application (config files, data processing, logging). Safe file handling prevents data loss and resource leaks.

**Independent Test**: Student can write code that reads a text file line by line using `with` statement, handles `FileNotFoundError`, and writes data to a new file with proper cleanup.

**Acceptance Scenarios**:

1. **Given** a text file exists in the directory, **When** student's code opens and reads it, **Then** all lines are processed and file is automatically closed
2. **Given** specified file doesn't exist, **When** student's code attempts to open it, **Then** `FileNotFoundError` is caught and user-friendly message is displayed
3. **Given** student's code writes to a file, **When** program completes or encounters error, **Then** file handle is properly closed and data is saved (if write succeeded)

---

### User Story 3 - Cross-Platform Path Handling (Priority: P2)

As an intermediate Python learner, I want to understand how to work with file paths using pathlib so my code works correctly on Windows, Mac, and Linux without hardcoding slashes.

**Why this priority**: Modern Python development requires cross-platform compatibility. Using os.path is outdated; pathlib is the professional standard (Python 3.4+).

**Independent Test**: Student can write code using pathlib to check if directory exists, create it if missing, list all .txt files, and construct paths that work on any OS.

**Acceptance Scenarios**:

1. **Given** student needs to check if directory exists, **When** using `Path.exists()`, **Then** code works identically on Windows (`C:\Users\...`) and Unix (`/home/...`) systems
2. **Given** student needs to join path segments, **When** using `Path / "subdir" / "file.txt"`, **Then** correct path separator is used for the operating system
3. **Given** student needs to create nested directories, **When** using `Path.mkdir(parents=True, exist_ok=True)`, **Then** all parent directories are created without error if they already exist

---

### User Story 4 - Structured Data Formats (Priority: P2)

As an intermediate Python learner, I want to understand how to read and write CSV and JSON files with proper encoding, so I can work with real-world data formats used in data analysis and API integration.

**Why this priority**: CSV and JSON are ubiquitous in professional development (data import/export, API responses, config files). Proper encoding handling prevents corruption of international text.

**Independent Test**: Student can read a CSV file into Python data structures, process rows with proper type conversion, and write results to JSON with UTF-8 encoding.

**Acceptance Scenarios**:

1. **Given** CSV file with headers and mixed data types, **When** student reads using csv.DictReader, **Then** each row is accessible as a dictionary with correct keys
2. **Given** Python data structure (list of dicts), **When** student writes to JSON with `ensure_ascii=False` and UTF-8 encoding, **Then** international characters (emoji, accented letters) are preserved
3. **Given** malformed JSON file, **When** student attempts to load it, **Then** JSONDecodeError is caught and user-friendly error message explains the issue

---

### User Story 5 - Complete Note-Taking App (Priority: P1 - Capstone)

As an intermediate Python learner, I want to build a complete CLI note-taking application that integrates all I/O concepts, so I can demonstrate mastery of console I/O, file I/O, pathlib, and JSON handling in a real-world project.

**Why this priority**: Capstone project demonstrates integration of all chapter concepts. This is the measurable outcome for the chapter's success evals.

**Independent Test**: Student builds a working Note-Taking App with create/read/update/delete notes functionality, search by keyword, and category organization, all persisted to JSON files using pathlib. App is designed to handle dozens of notes (10-50) comfortably.

**Acceptance Scenarios**:

1. **Given** user runs the app, **When** main menu displays (Create/Read/Update/Delete/Search/Exit), **Then** user can enter choice (1-6) and app responds appropriately
2. **Given** user selects "Create note" from menu, **When** they enter title and body, **Then** notes directory is created using pathlib and note is saved as JSON with UTF-8 encoding, returning to menu
3. **Given** user has multiple notes saved, **When** they select "Search" and enter keyword, **Then** all notes containing that keyword in title or body are displayed with their metadata, returning to menu
4. **Given** user wants to delete a note, **When** they select "Delete", choose note, and confirm deletion, **Then** note file is removed using pathlib, confirmation message shown, and app returns to menu
5. **Given** user wants to organize notes by category, **When** they select "Update" and assign category to note, **Then** note metadata is updated in JSON and saved correctly, returning to menu
6. **Given** user is in the app, **When** they select "Exit" from menu, **Then** app closes gracefully with goodbye message

---

### Edge Cases

- **What happens when** user tries to read a file that's being written by another program? (File locking, permission errors)
- **What happens when** user provides a path with spaces, special characters, or Unicode in filenames? (pathlib handles correctly, but worth teaching)
- **What happens when** user attempts path traversal (e.g., "../../../etc/passwd")? (Use Path.resolve() to canonicalize, then verify resolved path starts with allowed base directory)
- **What happens when** JSON file is corrupted or has syntax errors? (JSONDecodeError handling)
- **What happens when** user runs out of disk space while writing file? (IOError handling)
- **What happens when** user tries to open a binary file as text (or vice versa)? (Encoding errors, teach text vs binary distinction)
- **What happens when** CSV file has inconsistent column counts in different rows? (csv module behavior, error handling)

## Requirements

### Functional Requirements

#### Console I/O (Lesson 1)

- **FR-001**: Students MUST be able to explain what console I/O is and why it's used for user interaction in CLI programs
- **FR-002**: Students MUST be able to use `input()` to gather user input and handle the string return type correctly
- **FR-003**: Students MUST be able to use f-strings to create formatted output with `print()`
- **FR-004**: Students MUST be able to validate user input (type checking, range checking) with try/except error handling
- **FR-005**: Students MUST be able to convert user input from string to appropriate type (int, float) safely

#### File I/O Fundamentals (Lesson 2)

- **FR-006**: Students MUST be able to explain file modes (r, w, a, r+) and when to use each
- **FR-007**: Students MUST be able to use context managers (`with` statement) for automatic file cleanup
- **FR-008**: Students MUST be able to read files using read(), readline(), and readlines() methods
- **FR-009**: Students MUST be able to write to files using write() and writelines() methods
- **FR-010**: Students MUST be able to handle FileNotFoundError and PermissionError exceptions
- **FR-011**: Students MUST be able to explain difference between text and binary file modes

#### Path Handling (Lesson 3)

- **FR-012**: Students MUST be able to use pathlib.Path to create cross-platform file paths
- **FR-013**: Students MUST be able to check if file or directory exists using Path.exists() and Path.is_file()/is_dir()
- **FR-014**: Students MUST be able to create directories using Path.mkdir() with parents and exist_ok parameters
- **FR-015**: Students MUST be able to list files in directory using Path.iterdir() and Path.glob()
- **FR-016**: Students MUST be able to join path segments using the `/` operator
- **FR-017**: Students MUST understand why pathlib is preferred over os.path for modern Python (3.4+)
- **FR-018**: Students MUST be able to use Path.resolve() to canonicalize paths and validate they remain within allowed base directory (path traversal prevention)

#### Structured Data Formats (Lesson 4)

- **FR-019**: Students MUST be able to read CSV files using csv.reader and csv.DictReader
- **FR-020**: Students MUST be able to write CSV files using csv.writer and csv.DictWriter with proper headers
- **FR-021**: Students MUST be able to load JSON from file using json.load() with UTF-8 encoding
- **FR-022**: Students MUST be able to save Python data structures to JSON using json.dump() with indent and ensure_ascii parameters
- **FR-023**: Students MUST be able to handle JSONDecodeError for malformed JSON files
- **FR-024**: Students MUST understand character encoding (UTF-8) and why it matters for international text

#### Capstone Integration (Lesson 5)

- **FR-025**: Students MUST be able to integrate console I/O, file I/O, pathlib, and JSON in a single application
- **FR-026**: Students MUST be able to implement CRUD operations (Create, Read, Update, Delete) for data persisted to files
- **FR-027**: Students MUST be able to organize application data using directory structure (pathlib)
- **FR-028**: Students MUST be able to search and filter data loaded from files
- **FR-029**: Students MUST be able to validate user input throughout the application flow
- **FR-030**: Students MUST be able to implement menu-driven interactive loop (display menu, accept choice, execute action, return to menu)

### Key Entities

- **Note**: Represents a user-created text note with title, body, category, and metadata (created date, modified date)
  - **Attributes**: title (string), body (string), category (string), created (ISO datetime string), modified (ISO datetime string), id (UUID string)
  - **Storage**: JSON file per note, organized in notes/ directory by category
  - **Scale**: Expected 10-50 notes total (dozens, simple scale for proof-of-concept)
  - **Relationships**: Each note belongs to one category (categories stored as directory structure)

- **User Input**: Represents data gathered from console (input() function)
  - **Types**: age (integer), name (string), choice (integer from menu), search keyword (string)
  - **Validation**: Type checking, range checking, non-empty string checking
  - **Error Handling**: Retry on invalid input with helpful error messages

- **File Path**: Represents cross-platform file/directory location using pathlib
  - **Components**: Root, parents, stem, suffix, name
  - **Operations**: exists(), mkdir(), read_text(), write_text(), iterdir(), glob()
  - **Immutability**: Path objects are immutable (operations return new Path objects)

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can complete console I/O exercise (input validation + formatted output) in under 15 minutes with AI assistance
- **SC-002**: Students can write file reading/writing code using context managers without syntax errors on first attempt (80%+ success rate)
- **SC-003**: Students can create cross-platform paths using pathlib that work identically on Windows, Mac, and Linux
- **SC-004**: Students can successfully read and write both CSV and JSON formats with correct encoding (85%+ complete capstone project using these formats)
- **SC-005**: Note-Taking App capstone is completable in 90 minutes for B1-level learners following the lesson plan
- **SC-006**: 70%+ of students report feeling confident building file-based CLI applications after completing chapter
- **SC-007**: Students can explain security risks (path traversal, arbitrary file access) and mitigate them in their code

## AI-Native Learning Pedagogy

### Teaching Pattern (Constitutional Mandate)

This chapter applies the **Graduated Teaching Pattern** from Constitution Principle 13:

**Tier 1 - Foundational Concepts** (Book Teaches Directly):
- What is I/O? (Input/Output concept, console vs file)
- Why do we need files? (Persistence, data sharing)
- What are file modes? (read, write, append)
- What is a context manager? (Resource cleanup pattern)
- What is pathlib? (Modern path handling)
- What are CSV and JSON? (Common data formats)

**Tier 2 - Complex Execution** (AI Companion Handles):
- Student describes: "I want to read a config file and validate required fields exist"
- AI generates: Code with json.load(), error handling, validation logic
- Student learns: Strategy (what to validate, why) not syntax (how to access dict keys)

**Tier 3 - Scaling Operations** (AI Orchestration):
- Student describes: "Set up directory structure for 100 notes organized by 10 categories"
- AI generates: Batch pathlib operations with mkdir(parents=True)
- Student learns: Orchestration mindset (organize data, automate repetition)

### CoLearning Elements (Throughout Lessons, NOT Just End)

Each lesson MUST include:

- **💬 AI Colearning Prompt** (2-3 per lesson): Explore concepts with AI after foundations taught
- **🎓 Instructor Commentary** (2-3 per lesson): "Syntax is cheap, semantics is gold"
- **🚀 CoLearning Challenge** (2-3 per lesson): Practice specification-driven thinking
- **✨ Teaching Tip** (2-3 per lesson): Build AI tool literacy

### Lesson Closure Pattern (Constitutional Mandate)

**ALL lessons end with "Try With AI" section ONLY**:
- 4 prompts following Bloom's taxonomy progression (Remember → Understand → Apply → Analyze)
- Each prompt has explicit "Expected outcome" describing what student learns
- Prompt 4 provides cognitive closure (synthesis + forward-looking insight)
- **NO** summaries, checklists, or "what's next" sections after "Try With AI"

## Technical Standards (Non-Negotiable)

### Python Version
- **Python 3.14+** (latest stable release from python.org)

### Type Hints
- **Required throughout**: Every function signature must have type hints
- **Modern syntax**: Use `list[str]`, `dict[str, int]`, `X | None` (NOT `Optional[X]`)

### Modern Patterns
- **pathlib** (NOT os.path): All path operations use `pathlib.Path`
- **Context managers**: All file operations use `with` statements
- **f-strings only**: No %-formatting or .format()

### Security
- **No shell=True**: Never use subprocess with shell=True
- **No eval()**: Never use eval() on user input
- **Validate input**: All user input must be validated before file operations
- **Handle encoding**: Always specify encoding='utf-8' for text files
- **Path traversal prevention**: Use `Path.resolve()` to canonicalize paths, then verify resolved path starts with allowed base directory (e.g., `if not resolved_path.is_relative_to(base_dir): raise SecurityError`)

## Assumptions

### What Students Already Know (Chapters 1-21)
- Python basics: variables, types, operators
- Control flow: if/elif/else, for/while loops
- Functions: definition, parameters, return, type hints
- Exception handling: try/except/finally, custom exceptions (Chapter 21)
- Data structures: lists, tuples, dicts, sets (Chapters 18-19)
- Modules and imports: Using standard library modules (Chapter 20)

### What Students Will Learn Later (Chapters 23-29)
- Advanced datetime operations (Chapter 23)
- Object-oriented programming (Chapters 24-26)
- Pydantic and generics (Chapter 27)
- Async/await (Chapter 28)
- CPython internals (Chapter 29)

## Preliminary Content Outline (5 Lessons)

### Lesson 1: Console I/O and Formatted Output (A2) - 75 minutes
**Concepts** (5): input(), print(), f-strings, error handling for input, type conversion
**Focus**: User interaction patterns, input validation
**AI Partnership**: Exploring edge cases, error handling strategies

### Lesson 2: File I/O Fundamentals (A2-B1) - 90 minutes
**Concepts** (7): open(), file modes, with statements, reading methods, writing methods, exceptions, text vs binary
**Focus**: Safe file handling, context managers, resource cleanup
**AI Partnership**: Generating file handling code, validating safety

### Lesson 3: Path Handling with pathlib (B1) - 75 minutes
**Concepts** (6): Path objects, existence checking, joining paths, creating directories, listing files, cross-platform compatibility
**Focus**: Modern path handling, avoiding os.path pitfalls
**AI Partnership**: Exploring pathlib patterns, cross-platform testing

### Lesson 4: Structured Data Formats (CSV, JSON) (B1) - 90 minutes
**Concepts** (7): csv module, json module, encoding (UTF-8), serialization/deserialization, error handling, parameters
**Focus**: Working with real-world data formats
**AI Partnership**: Validating format correctness, handling encoding issues

### Lesson 5: Capstone - Note-Taking App (CLI) (B1 Integration) - 90 minutes
**Concepts** (0 new): Integration of all concepts from Lessons 1-4
**Project**: Build complete CLI app with CRUD operations, search, categories
**AI Partnership**: Architectural decisions, code generation, validation

**Total Time**: 5-6 hours

## Next Steps

After specification approval:
1. Run `/sp.clarify` to identify any underspecified areas (quality gate)
2. Run `/sp.plan` to create detailed lesson-by-lesson breakdown with CEFR proficiency mapping
3. Run `/sp.adr` to document architecturally significant decisions
4. Run `/sp.tasks` to create implementation checklist
5. Run `/sp.analyze` to validate cross-artifact consistency
6. Optionally run `/sp.implement` to generate lessons
