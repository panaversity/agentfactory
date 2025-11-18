# Chapter 22: IO and File Handling — Detailed Implementation Plan

**Chapter Type**: Technical / Code-Focused
**Chapter Objective(s)**: Students can safely read from and write to files using context managers, work with cross-platform paths using pathlib, handle console input/output with proper validation, work with structured data formats (CSV, JSON) with correct encoding, and build a complete interactive CLI application integrating all I/O concepts.
**Estimated Total Time**: 5–6 hours (5 lessons × 75–90 minutes each)
**Part**: Part 4 - Python Fundamentals
**Complexity Tier**: A2-B1 (Intermediate)
**Status**: Ready for Implementation
**Feature Branch**: `022-io-file-handling`

---

## 🎯 Chapter Overview & Context

**Building on Chapter 21 Foundation**:
- Chapter 21 taught: Exception handling fundamentals (try/except/finally, raising exceptions, custom exceptions)
- Chapter 22 builds on this by teaching: File I/O with error handling, path management, structured data formats, console interaction

**Bridge to Chapter 23**:
- Chapter 22 completes Python's core I/O capabilities (console, file operations, structured formats)
- Chapter 23 (Math, DateTime, Calendar) will build on file-based workflows — students will timestamp files, store configuration

**Prerequisites Established**:
- Chapter 13: Python basics (variables, print(), basic I/O)
- Chapter 14: Data types (strings, lists, dicts, understanding structured data)
- Chapter 15: Operators and type conversion (string/int conversion for input validation)
- Chapter 16: Strings and type casting (f-strings for formatted output)
- Chapter 17: Control flow and loops (iterating through file lines, menu-driven loops)
- Chapter 18: Lists, tuples, dicts (in-memory data structures before file persistence)
- Chapter 19: Sets and collections (understanding different container types)
- Chapter 20: Modules and functions (importing csv, json, pathlib modules; function design for reusability)
- Chapter 21: Exception handling (try/except for FileNotFoundError, JSONDecodeError, IOError)

**AI-Native Learning Pattern** (how students learn I/O with AI):
- **Tier 1 - Foundational**: Book teaches stable concepts: What is I/O? Why context managers? What are file modes? Pathlib basics.
- **Tier 2 - Complex Execution**: AI companion handles syntax-heavy operations. Student describes intent ("read a config file and validate required fields"), AI generates code with json.load() and validation, student learns strategy not syntax.
- **Tier 3 - Scaling Operations**: AI orchestrates batch operations. Student describes ("set up 10-category directory structure"), AI generates mkdir(parents=True) loop, student learns orchestration mindset.

**Key Scope Boundaries** (Chapter 22 Responsibility):
- ✅ **Comprehensive Coverage**: Console I/O (input validation, formatted output), file modes and context managers, pathlib (existence checks, directory creation, path operations, glob patterns), CSV/JSON formats with encoding, error handling for file operations, path traversal security, menu-driven CLI loop design
- ✅ **Practical Application**: Note-Taking App capstone integrating all concepts at enterprise scale (10-50 notes, multiple categories, search functionality)
- ⚠️ **Awareness Only**: Binary file formats, pickle (security concerns), advanced asyncio file operations, streaming large files (10GB+), database connections
- ❌ **Deferred**: Advanced encoding (UTF-16, shift-jis), compression (gzip, zip), network I/O, memory-mapped files, Chapter 28 (Asyncio) for async file operations, Chapter 29 (CPython) for buffer internals

---

## 📊 Lesson Architecture

### Lesson 1: Console I/O and User Input Validation (A2)

**Learning Objective** (Bloom's: Understand & Apply): Students can use `input()` to gather user data with proper type validation, handle invalid input gracefully with error recovery, and format output using f-strings with clear, professional presentation.

**CEFR Proficiency Level**: A2 (Elementary)

**Skills Taught**:
- **Skill 1**: Using input() for User Interaction
  - CEFR Level: A2
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (user interaction)
  - Measurable at This Level: Student can write `user_input: str = input("Prompt: ")` and understand that input() always returns a string, regardless of expected type

- **Skill 2**: Implementing Input Validation with Type Conversion
  - CEFR Level: A2-B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (validation logic)
  - Measurable at This Level: Student can convert input to int/float using `int()`, handle ValueError when conversion fails, implement retry loops with clear error messages

- **Skill 3**: Formatting Output with F-Strings
  - CEFR Level: A2
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation (formatted text output)
  - Measurable at This Level: Student can use f-string expressions to format numbers, strings, and variables for professional console output

- **Skill 4**: Error Recovery in Interactive Programs
  - CEFR Level: A2-B1
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (graceful error handling)
  - Measurable at This Level: Student can design input validation loops that don't crash on invalid input and provide helpful messages

- **Skill 5**: Understanding Console I/O Patterns
  - CEFR Level: A2
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (input/output concept)
  - Measurable at This Level: Student can explain the difference between console I/O (interactive terminal) and file I/O (persistent storage)

**Key Concepts** (max 7):
1. Console I/O concept — interactive user input and formatted output
2. input() function — reads string from user, returns string type
3. Type conversion — int(), float(), str() for handling user input
4. ValueError exception — raised when type conversion fails
5. F-string formatting — f"text {variable} more text", including expressions
6. Input validation — checking type, range, format before processing
7. Error recovery — try/except loops that retry on invalid input

**Prerequisites**:
- Chapter 13: Basic print(), variable assignment
- Chapter 15: Type conversion operators
- Chapter 16: F-string fundamentals
- Chapter 21: try/except exception handling

**Duration**: 75–90 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why console I/O matters; connecting to interactive CLI programs
2. **Concept: Understanding input()** (8 min) - How input() works, string return type, terminal behavior
3. **Concept: Type Conversion and Validation** (10 min) - int(), float(), try/except pattern, ValueError explanation
4. **Concept: F-String Formatting** (8 min) - Basic f-strings, expressions inside {}, number formatting
5. **Concept: Error Recovery Loops** (7 min) - Retry logic for invalid input, user-friendly messages
6. **Code Examples** (20 min) - Input validation with retry, formatted output examples
7. **CoLearning Elements** (7 min) - Integrated throughout (see below)
8. **Practice Exercises** (5 min) - Students write simple input validation program

**Content Elements** (Tier 1 = Book teaches, Tier 2 = AI companion, Tier 3 = AI orchestration):

**Tier 1 - Foundational Concepts (Book Teaches Directly)**:
- What is console I/O and when to use it
- How input() returns a string always
- Why type conversion is necessary for int/float input
- What try/except pattern looks like
- How to format numbers and strings in f-strings

**Tier 2 - Complex Execution (AI Companion Handles)**:
- Student describes: "I want to ask for a user's age and make sure it's a valid positive integer"
- AI generates: Complete input validation loop with error messages
- Student learns: Strategy (what validation steps are needed), not syntax details

**Tier 3 - Scaling Operations**:
- Scaling not applicable at Lesson 1 (single input); covered in Lesson 5 menu loop

**Code Examples** (3 for Lesson 1):

**Code Example 1.1: Basic input() with Type Conversion**
```python
# Gather and convert user input
name: str = input("What is your name? ")
age_str: str = input("How old are you? ")
age: int = int(age_str)

# Display with f-strings
print(f"Hello, {name}! You are {age} years old.")
```
- **Purpose**: Demonstrate input() returns string, conversion syntax, f-string display
- **Complexity**: A2 (foundational)
- **AI Prompt Specification**: "Show me how to use input() and convert it to an integer"
- **Expected Outcome**: Student understands input() always returns string and conversion is needed

**Code Example 1.2: Input Validation with Error Handling**
```python
# Validate user input with retry logic
def get_positive_integer(prompt: str) -> int:
    while True:
        try:
            value_str: str = input(prompt)
            value: int = int(value_str)
            if value <= 0:
                print("Please enter a positive number.")
                continue
            return value
        except ValueError:
            print(f"Invalid input. Please enter a whole number.")

age: int = get_positive_integer("Enter your age (positive number): ")
print(f"Your age is {age}.")
```
- **Purpose**: Show complete validation pattern with try/except and range checking
- **Complexity**: A2-B1 (combining concepts)
- **AI Prompt Specification**: "Write a function that asks for a positive integer and keeps retrying until valid input"
- **Expected Outcome**: Student sees how to handle ValueError and implement validation loops

**Code Example 1.3: Formatted Output with Numbers**
```python
# Formatted output examples
price: float = 19.999
items: int = 5
total: float = price * items

# Format currency and alignment
print(f"Price per item: ${price:.2f}")          # Rounds to 2 decimals
print(f"Number of items: {items:>5}")           # Right-align in 5 chars
print(f"Total cost: ${total:,.2f}")             # Thousands separator, 2 decimals

# Named parameter from input
quantity: int = int(input("How many? "))
subtotal: float = 100.0 * quantity
print(f"\nYour order:\n  Quantity: {quantity}\n  Subtotal: ${subtotal:,.2f}")
```
- **Purpose**: Show f-string formatting options for professional output
- **Complexity**: A2-B1 (format specifiers)
- **AI Prompt Specification**: "Show me how to format currency with 2 decimal places and thousands separators"
- **Expected Outcome**: Student can format numbers for clear, professional console output

**CoLearning Elements** (integrated throughout, not just at end):

#### 💬 AI Colearning Prompt (after Code Example 1.1)
> "Explain what happens inside Python when I use `int()` on the string '25'. What error would I get if I tried `int('hello')`?"

**Expected Outcome**: Student understands type conversion mechanism and error conditions before practicing

#### 🎓 Instructor Commentary (after showing try/except)
> In AI-native development, you don't memorize exception types — you understand WHEN type conversion fails. Ask AI: "What exceptions can occur when converting user input?" Your job is designing the validation strategy; syntax is cheap.

#### 🚀 CoLearning Challenge (after Code Example 1.2)
Ask your AI Co-Teacher:
> "Generate a function that asks for a user's email and validates it contains '@' and a domain. Then explain how string methods like `.count()` and `.find()` work."

**Expected Outcome**: You'll understand string validation and method chains beyond f-strings

#### ✨ Teaching Tip (before console output formatting)
> Use Claude Code to explore: "How many different ways can I format a floating-point number in an f-string? Show me examples for currency, percentages, and scientific notation."

**Practice Approach**:

**Exercise 1**: Write a program that asks for a user's name and favorite number, validates the number is positive, then displays formatted output
- Acceptance: Code runs without crashes on both valid and invalid input
- Validation: Try with positive int, negative int, text input

**Exercise 2**: Create an input validation function that ensures a string has minimum length
- Acceptance: Function rejects empty strings and strings shorter than specified minimum
- Validation: Test with edge cases (empty, exactly minimum, longer)

**End-of-Lesson: Try With AI** (4 prompts with Bloom's progression):

**Prompt 1 - Remember/Understand**:
```
Ask your AI: "What does the input() function return? Show me an example."

Expected Outcome: You'll understand that input() always returns a string,
regardless of what the user types.
```

**Prompt 2 - Apply**:
```
Ask your AI: "Write a program that asks for a user's age and height,
converts both to numbers, then calculates BMI. Add error handling for invalid input."

Expected Outcome: You'll see input validation, type conversion, and calculation patterns combined.
```

**Prompt 3 - Analyze**:
```
Ask your AI: "Compare two approaches to input validation: (1) try/except with retry loop,
(2) input validation in a separate function. Which is better and why?"

Expected Outcome: You'll understand different validation patterns and when each applies.
```

**Prompt 4 - Synthesize/Create** (cognitive closure):
```
Ask your AI: "Design a quiz program that asks 5 multiple-choice questions,
validates user input (A-D only), tracks score, and displays a formatted final result.
What would the overall structure look like?"

Expected Outcome: You'll see how console I/O patterns combine to build an interactive application,
preparing you for the menu-driven capstone.
```

**Success Criteria**:
- Student can write input validation code without runtime crashes
- Student can explain why input() returns string and conversion is necessary
- Student can use f-strings to format output professionally
- Student can implement try/except retry loops for invalid input
- 80%+ of students complete all "Try With AI" prompts

---

### Lesson 2: File I/O Fundamentals with Context Managers (A2-B1)

**Learning Objective** (Bloom's: Understand & Apply): Students can safely open and close files using context managers, read files using different methods (read, readline, readlines), write files with proper data formatting, handle file-related exceptions (FileNotFoundError, PermissionError), and understand the difference between text and binary file modes.

**CEFR Proficiency Level**: A2-B1 (Elementary-Intermediate)

**Skills Taught**:
- **Skill 1**: Using Context Managers (with statement) for File Operations
  - CEFR Level: A2-B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (resource management)
  - Measurable at This Level: Student can write `with open(filename) as f:` and understand that the file is automatically closed after the block

- **Skill 2**: Understanding File Modes and When to Use Each
  - CEFR Level: A2
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (file operations)
  - Measurable at This Level: Student can explain when to use 'r', 'w', 'a', 'r+' and predict file state after each operation

- **Skill 3**: Reading Files with Different Methods
  - CEFR Level: A2-B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data processing)
  - Measurable at This Level: Student can choose appropriate read method (read() vs readline() vs readlines()) based on use case and implement without errors

- **Skill 4**: Writing Files with Proper Formatting
  - CEFR Level: A2-B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data storage)
  - Measurable at This Level: Student can write strings to files with newline handling and maintain clean file structure

- **Skill 5**: Handling File I/O Exceptions
  - CEFR Level: A2-B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (error management)
  - Measurable at This Level: Student can catch FileNotFoundError and PermissionError with appropriate error messages

- **Skill 6**: Understanding Text vs Binary File Modes
  - CEFR Level: B1
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (file format awareness)
  - Measurable at This Level: Student can explain when to use 't' (text) vs 'b' (binary) mode and what happens with encoding

- **Skill 7**: Resource Cleanup and File Safety
  - CEFR Level: A2-B1
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (reliability)
  - Measurable at This Level: Student can explain why context managers prevent file corruption and resource leaks

**Key Concepts** (max 7):
1. Context managers (with statement) — automatic resource cleanup
2. File modes: 'r' (read), 'w' (write), 'a' (append), 'r+' (read/write)
3. Reading methods: read() (whole file), readline() (one line), readlines() (all lines as list)
4. Writing files: write() (single string), writelines() (multiple strings)
5. FileNotFoundError and PermissionError exceptions
6. Text mode ('t', default) vs binary mode ('b') — encoding implications
7. File handle lifecycle — open, use in context, automatic close

**Prerequisites**:
- Chapter 13: Basic file operations awareness
- Chapter 14: Understanding strings, lists
- Chapter 21: try/except exception handling

**Duration**: 90 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why files matter; moving from console to persistent storage
2. **Concept: Context Managers and File Safety** (10 min) - Why with statement is essential, automatic cleanup
3. **Concept: File Modes** (8 min) - 'r', 'w', 'a', 'r+' explained with examples of file state
4. **Concept: Reading Methods** (10 min) - read() vs readline() vs readlines(), choosing appropriate method
5. **Concept: Writing Files and Newlines** (8 min) - Proper formatting with '\n', writelines() pattern
6. **Concept: File Exceptions** (7 min) - FileNotFoundError, PermissionError, proper error handling
7. **Code Examples** (25 min) - Reading files, writing files, handling errors
8. **CoLearning Elements** (7 min) - Integrated throughout
9. **Practice Exercises** (5 min) - Write and read simple text files with error handling

**Content Elements** (Tier 1 = Book teaches, Tier 2 = AI companion):

**Tier 1 - Foundational Concepts**:
- What is a file and why files provide persistence
- How context managers work (with statement pattern)
- What are file modes and what each one does
- How to read files using different methods
- Why automatic cleanup is important

**Tier 2 - Complex Execution**:
- Student describes: "I want to read a multi-line configuration file, skip comments, and validate required fields"
- AI generates: Complete file reading code with line parsing and validation
- Student learns: Strategy (how to parse, validate), not syntax (exact string methods)

**Code Examples** (4 for Lesson 2):

**Code Example 2.1: Safe File Reading with Context Manager**
```python
# Safe file reading with automatic cleanup
filename: str = "notes.txt"

try:
    with open(filename, 'r', encoding='utf-8') as file:
        content: str = file.read()
        print(f"File content:\n{content}")
except FileNotFoundError:
    print(f"Error: File '{filename}' not found.")
```
- **Purpose**: Show context manager pattern, encoding specification, basic exception handling
- **Complexity**: A2-B1 (fundamental pattern)
- **AI Prompt Specification**: "Show me how to safely open and read a text file"
- **Expected Outcome**: Student understands why `with` is essential for file safety

**Code Example 2.2: Different File Modes and Reading Methods**
```python
# Create a sample file with multiple lines
filename: str = "data.txt"

# Write mode: create new file or overwrite
with open(filename, 'w', encoding='utf-8') as f:
    f.write("Alice,25,Engineering\n")
    f.write("Bob,30,Sales\n")
    f.write("Carol,28,Marketing\n")

# Read entire file at once
with open(filename, 'r', encoding='utf-8') as f:
    full_content: str = f.read()
    print("Full content:")
    print(full_content)

# Read line by line
print("\nReading line by line:")
with open(filename, 'r', encoding='utf-8') as f:
    for line in f:
        print(f"Line: {line.strip()}")  # strip() removes trailing \n

# Read all lines into list
with open(filename, 'r', encoding='utf-8') as f:
    lines: list[str] = f.readlines()
    print(f"\nTotal lines: {len(lines)}")
```
- **Purpose**: Demonstrate write mode, multiple reading methods, newline handling
- **Complexity**: A2-B1 (pattern variety)
- **AI Prompt Specification**: "Show me how to write a file with multiple lines, then read it back in different ways"
- **Expected Outcome**: Student understands when to use each reading method

**Code Example 2.3: Append Mode and File Growth**
```python
# Append mode: add to existing file without overwriting
filename: str = "log.txt"

# First write
with open(filename, 'w', encoding='utf-8') as f:
    f.write("2025-11-09 10:00 - Program started\n")

# Later, append more data
with open(filename, 'a', encoding='utf-8') as f:
    f.write("2025-11-09 10:15 - User logged in\n")
    f.write("2025-11-09 10:30 - Data processing complete\n")

# Read and display all
with open(filename, 'r', encoding='utf-8') as f:
    content: str = f.read()
    print("Log file contents:")
    print(content)
```
- **Purpose**: Demonstrate append mode, contrast with write mode, show log file pattern
- **Complexity**: A2-B1 (practical pattern)
- **AI Prompt Specification**: "Show me how to add lines to a file without deleting existing content"
- **Expected Outcome**: Student understands append vs write distinction

**Code Example 2.4: Error Handling for File Operations**
```python
# Robust file reading with comprehensive error handling
def safe_read_file(filename: str) -> str | None:
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None
    except PermissionError:
        print(f"Error: No permission to read '{filename}'.")
        return None
    except IOError as e:
        print(f"Error reading file: {e}")
        return None

# Usage
content: str | None = safe_read_file("config.txt")
if content:
    print(f"Successfully read {len(content)} characters")
else:
    print("Failed to read file")
```
- **Purpose**: Show proper exception handling for common file errors
- **Complexity**: B1 (comprehensive error handling)
- **AI Prompt Specification**: "Write a function that reads a file with proper error handling for different failure cases"
- **Expected Outcome**: Student understands multiple exception types and appropriate responses

**CoLearning Elements**:

#### 💬 AI Colearning Prompt (after Code Example 2.1)
> "Explain what happens if I forget the `with` statement and use `file = open(filename)` then try to read it. Why is `with` safer?"

**Expected Outcome**: Student understands context manager benefits before practicing manual file handling

#### 🎓 Instructor Commentary (after showing file modes)
> In AI-native development, you don't memorize file mode combinations — you understand your INTENT (create new, append, read, modify). Ask AI: "I want to add lines to an existing file without losing data—which mode?" Your job: specify intent; syntax is cheap.

#### 🚀 CoLearning Challenge (after Code Example 2.3)
Ask your AI Co-Teacher:
> "Generate code that reads a log file, counts how many errors occurred (lines containing 'ERROR'),
> and appends a summary line. Then explain how string methods like `.count()` and `.find()` work."

**Expected Outcome**: You'll see practical file processing patterns combining reading, searching, and appending

#### ✨ Teaching Tip (before error handling)
> Use Claude Code to explore file errors: "What happens if I try to open a file that doesn't exist? Show me the actual error and how to handle it gracefully."

**Practice Approach**:

**Exercise 1**: Create a program that writes 5 lines to a file, then reads it back and displays each line with a line number
- Acceptance: File contains exactly 5 lines, output shows "1: [line]" format
- Validation: Run twice to verify file is overwritten (not appended)

**Exercise 2**: Write a function that appends a timestamped log message to a file, handling the case where file doesn't exist initially
- Acceptance: Function works on first call (creates file) and subsequent calls (appends)
- Validation: Check file contains all messages in order

**Exercise 3**: Implement error handling that attempts to read a file, catches both FileNotFoundError and PermissionError, and provides appropriate messages
- Acceptance: Program doesn't crash on either error type
- Validation: Test with missing file and file with restricted permissions

**End-of-Lesson: Try With AI** (4 prompts):

**Prompt 1 - Remember/Understand**:
```
Ask your AI: "What is a context manager? Why do we use 'with' when opening files?"

Expected Outcome: You'll understand that context managers automatically clean up resources,
preventing file corruption.
```

**Prompt 2 - Apply**:
```
Ask your AI: "Write a program that reads a file containing CSV data (each line has name,age,city),
parses it, and writes a formatted report to a new file."

Expected Outcome: You'll see file reading, string parsing, and writing combined in practical workflow.
```

**Prompt 3 - Analyze**:
```
Ask your AI: "Compare reading a file with read() vs readlines(). When would you use each?
What are the memory implications for large files?"

Expected Outcome: You'll understand performance tradeoffs and method selection criteria.
```

**Prompt 4 - Synthesize/Create** (cognitive closure):
```
Ask your AI: "Design a file backup program that reads the original file, makes modifications
(e.g., removing blank lines), and writes to a backup file. Include error handling.
What's the overall structure?"

Expected Outcome: You'll see how file I/O integrates with data processing and error management.
```

**Success Criteria**:
- Student can write files using context managers without resource leaks
- Student can read files using appropriate methods for the use case
- Student can handle FileNotFoundError and PermissionError gracefully
- Student can explain text vs binary mode implications
- 80%+ of students complete all "Try With AI" prompts

---

### Lesson 3: Cross-Platform Path Handling with pathlib (B1)

**Learning Objective** (Bloom's: Understand & Apply): Students can use pathlib.Path to construct cross-platform file paths, check file/directory existence, create nested directories, list files using glob patterns, and understand path traversal security concerns.

**CEFR Proficiency Level**: B1 (Intermediate)

**Skills Taught**:
- **Skill 1**: Creating Path Objects with pathlib
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (path construction)
  - Measurable at This Level: Student can create Path objects using `Path()`, string literals, and `/` operator; understand path immutability

- **Skill 2**: Checking File and Directory Existence
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (file system queries)
  - Measurable at This Level: Student can use `.exists()`, `.is_file()`, `.is_dir()` to determine file types before operations

- **Skill 3**: Creating and Organizing Directories
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (file system management)
  - Measurable at This Level: Student can use `.mkdir(parents=True, exist_ok=True)` to create nested directories safely

- **Skill 4**: Listing and Globbing Files
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (file discovery)
  - Measurable at This Level: Student can use `.iterdir()` and `.glob()` patterns to find files matching criteria

- **Skill 5**: Understanding Cross-Platform Compatibility
  - CEFR Level: B1
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (OS independence)
  - Measurable at This Level: Student can explain why pathlib is preferred over hardcoded paths and os.path, understand Windows vs Unix path differences

- **Skill 6**: Path Traversal Security and Validation
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Safety & Cybersecurity (security best practices)
  - Measurable at This Level: Student can use `.resolve()` to canonicalize paths and validate that resolved paths stay within allowed directory bounds

**Key Concepts** (max 7):
1. pathlib.Path objects — modern, object-oriented path handling
2. Path construction: `Path("dir")`, `Path() / "subdir" / "file.txt"`
3. Path properties: `.name`, `.stem`, `.suffix`, `.parent`
4. Existence checking: `.exists()`, `.is_file()`, `.is_dir()`
5. Directory creation: `.mkdir(parents=True, exist_ok=True)` for nested directories
6. File listing: `.iterdir()` for directory contents, `.glob("*.txt")` for pattern matching
7. Path security: `.resolve()` for canonical paths, validating bounds to prevent traversal attacks

**Prerequisites**:
- Chapter 14: Understanding strings and data structures
- Chapter 17: Loops for iterating file collections
- Chapter 21: Exception handling for file operation errors

**Duration**: 75–90 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why pathlib is essential; moving beyond hardcoded paths
2. **Concept: pathlib vs os.path** (8 min) - Advantages of Path objects over string concatenation
3. **Concept: Creating and Manipulating Paths** (10 min) - Path(), `/` operator, path properties
4. **Concept: Checking Existence and File Types** (8 min) - exists(), is_file(), is_dir() patterns
5. **Concept: Creating Directories** (7 min) - mkdir(parents=True, exist_ok=True), why parameters matter
6. **Concept: Finding Files with Glob** (7 min) - iterdir(), glob() patterns, recursive search
7. **Concept: Path Security and Traversal Prevention** (8 min) - resolve(), boundary validation
8. **Code Examples** (25 min) - Path operations, directory creation, glob patterns
9. **CoLearning Elements** (7 min) - Integrated throughout
10. **Practice Exercises** (5 min) - Create directory structures, list files, validate paths

**Content Elements**:

**Tier 1 - Foundational Concepts**:
- What is pathlib and why it's better than string paths
- How to create Path objects using different syntax
- What file properties pathlib provides (name, suffix, parent)
- How to check if files/directories exist
- How to create directories with nested parents

**Tier 2 - Complex Execution**:
- Student describes: "I need to find all .txt files in a directory and subdirectories"
- AI generates: Complete glob() pattern with recursive=True
- Student learns: Strategy (what pattern matches what), not syntax

**Code Examples** (5 for Lesson 3):

**Code Example 3.1: Creating Paths with pathlib**
```python
from pathlib import Path

# Create Path objects
home: Path = Path.home()  # User's home directory
docs: Path = Path("documents")  # Relative path
config_file: Path = Path("config") / "app.json"  # Using / operator

# Path properties
print(f"Full path: {config_file}")          # config/app.json
print(f"File name: {config_file.name}")     # app.json
print(f"Suffix: {config_file.suffix}")      # .json
print(f"Stem: {config_file.stem}")          # app
print(f"Parent: {config_file.parent}")      # config

# Join paths across platforms (Windows uses \, Unix uses /)
notes_dir: Path = Path("data") / "notes" / "personal"
print(f"Cross-platform path: {notes_dir}")
```
- **Purpose**: Demonstrate Path object creation and property access
- **Complexity**: B1 (object-oriented path handling)
- **AI Prompt Specification**: "Show me how to create and manipulate file paths using pathlib"
- **Expected Outcome**: Student understands Path objects are superior to string concatenation

**Code Example 3.2: Checking File Existence and Type**
```python
from pathlib import Path

# Check existence and type
config_path: Path = Path("config.json")
data_dir: Path = Path("data")

# Existence checks
if config_path.exists():
    print(f"{config_path} exists")
    if config_path.is_file():
        print("  → It's a file")
    elif config_path.is_dir():
        print("  → It's a directory")
else:
    print(f"{config_path} does not exist")

# Common pattern: check before operations
if not data_dir.exists():
    print(f"{data_dir} directory doesn't exist yet")
else:
    file_count: int = len(list(data_dir.glob("*")))
    print(f"{data_dir} contains {file_count} items")
```
- **Purpose**: Show safe checking pattern before file operations
- **Complexity**: B1 (practical validation)
- **AI Prompt Specification**: "Show me how to check if files and directories exist before operations"
- **Expected Outcome**: Student understands defensive programming pattern

**Code Example 3.3: Creating Nested Directories**
```python
from pathlib import Path

# Create nested directories safely
notes_dir: Path = Path("data") / "notes" / "2025" / "november"

# parents=True creates all missing parent directories
# exist_ok=True doesn't raise error if directory already exists
notes_dir.mkdir(parents=True, exist_ok=True)

# Create category subdirectories
categories: list[str] = ["work", "personal", "learning"]
for category in categories:
    category_dir: Path = notes_dir / category
    category_dir.mkdir(parents=True, exist_ok=True)
    print(f"Created: {category_dir}")

# Verify structure
if notes_dir.exists():
    print(f"\nDirectory structure created at: {notes_dir}")
```
- **Purpose**: Show scalable directory creation pattern for applications
- **Complexity**: B1 (practical pattern for Capstone preparation)
- **AI Prompt Specification**: "Show me how to create nested directories even if parent directories don't exist"
- **Expected Outcome**: Student sees Tier 3 orchestration (creating multiple dirs in loop)

**Code Example 3.4: Finding Files with Glob Patterns**
```python
from pathlib import Path

# Create sample files
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

# Find all files (any type)
all_files: list[Path] = list(notes_dir.glob("*"))
print(f"\nAll files: {[f.name for f in all_files]}")

# Recursive search (Python 3.12+)
# all_nested: list[Path] = list(notes_dir.glob("**/*.txt"))
```
- **Purpose**: Demonstrate glob patterns for file discovery
- **Complexity**: B1 (pattern-based search)
- **AI Prompt Specification**: "Show me how to find all files matching a pattern in a directory"
- **Expected Outcome**: Student understands glob patterns for flexible file discovery

**Code Example 3.5: Path Traversal Security**
```python
from pathlib import Path

def read_note(base_dir: Path, user_input: str) -> str | None:
    """Read a note file with security validation."""
    # Resolve the requested path to canonical form
    requested_path: Path = (base_dir / user_input).resolve()

    # Check that resolved path stays within base directory
    # is_relative_to() ensures path is under base_dir
    if not requested_path.is_relative_to(base_dir.resolve()):
        print(f"Security error: Attempted access outside allowed directory")
        return None

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

# Safe usage
base_dir: Path = Path("notes")
base_dir.mkdir(exist_ok=True)
(base_dir / "note1.txt").write_text("Important note")

# Safe: Normal access
content = read_note(base_dir, "note1.txt")
print(f"Read: {content}")

# Unsafe: Attempted traversal (blocked)
# content = read_note(base_dir, "../../etc/passwd")
# Output: Security error: Attempted access outside allowed directory
```
- **Purpose**: Demonstrate security protection against path traversal attacks
- **Complexity**: B1 (security best practice)
- **AI Prompt Specification**: "Show me how to safely read files from a user-provided path without allowing access outside a directory"
- **Expected Outcome**: Student understands security implications and mitigation

**CoLearning Elements**:

#### 💬 AI Colearning Prompt (after Code Example 3.1)
> "Explain how the `/` operator works with Path objects. Why is this better than using string concatenation like `"config" + "/" + "app.json"`?"

**Expected Outcome**: Student understands Path objects handle OS-specific separators automatically

#### 🎓 Instructor Commentary (after showing cross-platform paths)
> In AI-native development, you don't hardcode paths with backslashes or forward slashes — you use pathlib.
> The syntax is cheap; understanding that your code must work on Windows, Mac, AND Linux is gold.
> Your AI can generate paths; your job is specifying the directory structure.

#### 🚀 CoLearning Challenge (after Code Example 3.3)
Ask your AI Co-Teacher:
> "Generate code that creates a directory structure for 5 projects, each with 'src', 'tests', and 'docs' subdirectories.
> Then show me how to use glob() to count total Python files across all projects."

**Expected Outcome**: You'll see pathlib for scaling (creating many dirs) and glob for discovering files

#### ✨ Teaching Tip (before security section)
> Use Claude Code to explore path security: "What would happen if someone passed '../../../etc/passwd' as user input?
> Show me the problem and how to defend against it using `.resolve()` and `.is_relative_to()`."

**Practice Approach**:

**Exercise 1**: Create a nested directory structure (data/2025/11/daily) using mkdir(parents=True, exist_ok=True)
- Acceptance: All directories created, running again doesn't error
- Validation: Verify with Path.exists() checks

**Exercise 2**: List all files with specific extension in a directory using glob()
- Acceptance: Correctly identifies all .txt or .py files
- Validation: Test with mixed file types

**Exercise 3**: Implement path validation that prevents traversal attacks
- Acceptance: Blocks '../' patterns, allows normal filenames
- Validation: Test with both safe paths and attack patterns

**End-of-Lesson: Try With AI** (4 prompts):

**Prompt 1 - Remember/Understand**:
```
Ask your AI: "What is pathlib? Why is it better than using os.path or string concatenation for file paths?"

Expected Outcome: You'll understand pathlib solves cross-platform path problems elegantly.
```

**Prompt 2 - Apply**:
```
Ask your AI: "Write code that creates a notes directory with subdirectories for each month of 2025,
then lists how many directories were created."

Expected Outcome: You'll see directory creation at scale and glob pattern usage.
```

**Prompt 3 - Analyze**:
```
Ask your AI: "Compare these approaches: (1) hardcoding paths like 'data/notes/file.txt',
(2) using os.path.join(), (3) using pathlib. Which works on all operating systems and why?"

Expected Outcome: You'll understand why pathlib is the modern standard.
```

**Prompt 4 - Synthesize/Create** (cognitive closure):
```
Ask your AI: "Design a file management system that finds all notes in a directory tree,
validates they're not trying to escape the notes directory, and displays them organized by date.
What pathlib methods would you use?"

Expected Outcome: You'll see pathlib for security, organization, and practical file management patterns.
```

**Success Criteria**:
- Student can create cross-platform paths that work on Windows, Mac, and Linux
- Student can check file existence before operations
- Student can create nested directories safely
- Student can find files using glob patterns
- Student can explain and implement path traversal security
- 80%+ of students complete all "Try With AI" prompts

---

### Lesson 4: Structured Data Formats (CSV and JSON) (B1)

**Learning Objective** (Bloom's: Understand & Apply): Students can read and write CSV files using the csv module with proper header handling, load and save JSON data with correct encoding, handle parsing errors gracefully, and understand when to use each format.

**CEFR Proficiency Level**: B1 (Intermediate)

**Skills Taught**:
- **Skill 1**: Reading CSV Files with csv.reader and csv.DictReader
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data import)
  - Measurable at This Level: Student can parse CSV files, handle headers, and iterate through rows as tuples or dictionaries

- **Skill 2**: Writing CSV Files with csv.writer and csv.DictWriter
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data export)
  - Measurable at This Level: Student can write data structures to CSV with proper headers and formatting

- **Skill 3**: Loading JSON Files with json.load()
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data deserialization)
  - Measurable at This Level: Student can load JSON from files, handle encoding='utf-8', and work with resulting Python structures

- **Skill 4**: Saving Data to JSON with json.dump()
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data serialization)
  - Measurable at This Level: Student can save Python objects to JSON with proper encoding, indentation, and ensure_ascii handling

- **Skill 5**: Understanding Character Encoding (UTF-8)
  - CEFR Level: B1
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (encoding awareness)
  - Measurable at This Level: Student can explain why UTF-8 matters, when to use ensure_ascii=False, and what happens with international characters

- **Skill 6**: Handling Format-Specific Exceptions
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (error handling)
  - Measurable at This Level: Student can catch JSONDecodeError, ValueError (malformed CSV), and provide helpful error messages

- **Skill 7**: Choosing Between CSV and JSON
  - CEFR Level: B1
  - Category: Conceptual
  - Bloom's Level: Analyze
  - DigComp Area: Information Literacy (format selection)
  - Measurable at This Level: Student can explain when CSV is appropriate (tabular, Excel-compatible) vs JSON (nested, API-friendly)

**Key Concepts** (max 7):
1. CSV (Comma-Separated Values) — tabular data format, plain text
2. csv.reader — iterates rows as tuples, csv.DictReader — rows as dictionaries with headers
3. csv.writer and csv.DictWriter for writing CSV files with proper formatting
4. JSON (JavaScript Object Notation) — structured format supporting nested data, arrays, objects
5. json.load() and json.dump() for file I/O with Python objects
6. Character encoding (UTF-8) — why it matters for international text (emoji, accented letters)
7. Encoding parameters: encoding='utf-8', ensure_ascii=False (preserve international characters)

**Prerequisites**:
- Chapter 14: Lists and dictionaries (understanding data structures)
- Chapter 18: Dict comprehensions (optional, for advanced examples)
- Chapter 21: Exception handling (try/except for format errors)
- Lesson 2 of this chapter: File I/O fundamentals

**Duration**: 90 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why structured formats matter; CSV vs JSON use cases
2. **Concept: CSV Format and csv Module** (10 min) - Rows, headers, DictReader advantage
3. **Concept: JSON Format and json Module** (8 min) - Objects, arrays, nested data, readability
4. **Concept: Character Encoding (UTF-8)** (8 min) - Why encoding matters, ensure_ascii implications
5. **Concept: Reading CSV Files** (8 min) - csv.reader vs csv.DictReader patterns
6. **Concept: Writing CSV Files** (7 min) - Headers, row formatting, DictWriter convenience
7. **Concept: JSON Serialization/Deserialization** (7 min) - json.dump(), json.load(), pretty printing
8. **Concept: Error Handling for Formats** (6 min) - JSONDecodeError, ValueError, validation
9. **Code Examples** (25 min) - Reading/writing CSV and JSON
10. **CoLearning Elements** (7 min) - Integrated throughout
11. **Practice Exercises** (5 min) - Convert between formats, handle errors

**Content Elements**:

**Tier 1 - Foundational Concepts**:
- What is CSV and how it's structured (rows, columns)
- What is JSON and how it represents data (objects, arrays, types)
- How to read CSV using csv.reader (tuples) vs csv.DictReader (dicts with headers)
- How to write CSV with proper headers
- How json.load() and json.dump() work
- Why UTF-8 encoding matters for international characters

**Tier 2 - Complex Execution**:
- Student describes: "Read CSV with employee data, convert to JSON, add timestamps"
- AI generates: Complete read/write code with format conversion and data enrichment
- Student learns: Transformation strategy, not API syntax details

**Code Examples** (5 for Lesson 4):

**Code Example 4.1: Reading CSV Files**
```python
import csv
from pathlib import Path

# Create sample CSV file
csv_file: Path = Path("employees.csv")
csv_file.write_text("""name,age,department
Alice,28,Engineering
Bob,35,Sales
Carol,32,Marketing
""")

# Read with csv.reader (returns tuples)
print("Using csv.reader:")
with open(csv_file, 'r', encoding='utf-8') as f:
    reader = csv.reader(f)
    header = next(reader)  # Get first row (header)
    print(f"Columns: {header}")
    for row in reader:
        print(f"  {row[0]} ({row[2]})")

# Read with csv.DictReader (returns dicts)
print("\nUsing csv.DictReader:")
with open(csv_file, 'r', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
        print(f"  {row['name']} - {row['department']}")
```
- **Purpose**: Demonstrate CSV reading with both reader types
- **Complexity**: B1 (practical data import)
- **AI Prompt Specification**: "Show me how to read CSV files with headers"
- **Expected Outcome**: Student understands DictReader is more convenient than csv.reader

**Code Example 4.2: Writing CSV Files**
```python
import csv
from pathlib import Path

# Data to write
employees: list[dict[str, str | int]] = [
    {"name": "Alice", "age": 28, "department": "Engineering"},
    {"name": "Bob", "age": 35, "department": "Sales"},
    {"name": "Carol", "age": 32, "department": "Marketing"},
]

csv_file: Path = Path("output.csv")

# Write using csv.DictWriter
with open(csv_file, 'w', newline='', encoding='utf-8') as f:
    fieldnames = ["name", "age", "department"]
    writer = csv.DictWriter(f, fieldnames=fieldnames)

    writer.writeheader()  # Write header row
    writer.writerows(employees)  # Write data rows

# Verify by reading back
with open(csv_file, 'r', encoding='utf-8') as f:
    content = f.read()
    print("Written CSV:")
    print(content)
```
- **Purpose**: Show CSV writing with proper header handling
- **Complexity**: B1 (data export pattern)
- **AI Prompt Specification**: "Show me how to write a list of dictionaries to a CSV file with headers"
- **Expected Outcome**: Student understands csv.DictWriter simplifies header management

**Code Example 4.3: Reading and Writing JSON**
```python
import json
from pathlib import Path

# Python data structure
notes: list[dict[str, str]] = [
    {"id": "1", "title": "Python Basics", "tags": ["learning", "python"]},
    {"id": "2", "title": "File I/O Guide", "tags": ["io", "files", "python"]},
]

# Write to JSON with formatting
json_file: Path = Path("notes.json")
with open(json_file, 'w', encoding='utf-8') as f:
    json.dump(notes, f, indent=2, ensure_ascii=False)

print(f"Written JSON file: {json_file}")

# Read JSON back
with open(json_file, 'r', encoding='utf-8') as f:
    loaded_notes: list[dict] = json.load(f)
    print(f"\nLoaded {len(loaded_notes)} notes:")
    for note in loaded_notes:
        print(f"  - {note['title']} (tags: {', '.join(note['tags'])})")
```
- **Purpose**: Demonstrate JSON serialization/deserialization
- **Complexity**: B1 (practical data persistence)
- **AI Prompt Specification**: "Show me how to save Python objects to JSON and read them back"
- **Expected Outcome**: Student sees JSON preserves nested structure and ensure_ascii=False preserves Unicode

**Code Example 4.4: Handling Encoding for International Text**
```python
import json
from pathlib import Path

# Data with international characters
messages: list[dict[str, str]] = [
    {"user": "Alice", "text": "Hello! 👋"},
    {"user": "Bob", "text": "Привет (Russian for hello)"},
    {"user": "Carol", "text": "你好 (Chinese for hello)"},
]

json_file: Path = Path("messages.json")

# Write with proper encoding
with open(json_file, 'w', encoding='utf-8') as f:
    json.dump(messages, f, indent=2, ensure_ascii=False)

# Display raw file content
with open(json_file, 'r', encoding='utf-8') as f:
    print("File contents (properly encoded):")
    print(f.read())

# Read back and verify
with open(json_file, 'r', encoding='utf-8') as f:
    loaded = json.load(f)
    print("\nLoaded messages:")
    for msg in loaded:
        print(f"  {msg['user']}: {msg['text']}")
```
- **Purpose**: Demonstrate encoding importance for international characters
- **Complexity**: B1 (real-world requirement)
- **AI Prompt Specification**: "Show me how to save and load JSON with emoji and international characters"
- **Expected Outcome**: Student understands ensure_ascii=False and encoding='utf-8' work together

**Code Example 4.5: Error Handling for Format Errors**
```python
import json
import csv
from pathlib import Path

def safe_load_json(filename: str) -> list[dict] | None:
    """Load JSON with error handling."""
    try:
        path = Path(filename)
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return None
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON format - {e}")
        return None
    except Exception as e:
        print(f"Unexpected error: {e}")
        return None

# Test with valid JSON
valid_file = Path("valid.json")
valid_file.write_text('[{"name": "Alice"}, {"name": "Bob"}]')
data = safe_load_json("valid.json")
print(f"Loaded: {data}")

# Test with invalid JSON
invalid_file = Path("invalid.json")
invalid_file.write_text('[{"name": "Alice"} BROKEN')
data = safe_load_json("invalid.json")
# Output: Error: Invalid JSON format - Expecting ',' delimiter...
```
- **Purpose**: Show proper exception handling for format errors
- **Complexity**: B1 (robust data loading)
- **AI Prompt Specification**: "Show me how to load JSON with error handling for corrupted files"
- **Expected Outcome**: Student sees JSONDecodeError and appropriate recovery

**CoLearning Elements**:

#### 💬 AI Colearning Prompt (after Code Example 4.1)
> "Explain the difference between csv.reader (returns tuples) and csv.DictReader (returns dicts).
> When would you prefer each approach?"

**Expected Outcome**: Student understands tradeoffs and can choose appropriate approach

#### 🎓 Instructor Commentary (after showing CSV/JSON)
> In AI-native development, you don't memorize csv module parameters — you understand your DATA STRUCTURE.
> Need tabular data for Excel? CSV. Nested data for APIs? JSON. Your AI generates the code; your job is specifying format intent.

#### 🚀 CoLearning Challenge (after Code Example 4.2)
Ask your AI Co-Teacher:
> "Read a CSV file with employee data, filter for employees in 'Engineering' department,
> convert to JSON with nested structure (grouped by department), and save. Then explain the transformation logic."

**Expected Outcome**: You'll see CSV→JSON transformation and data restructuring patterns

#### ✨ Teaching Tip (before encoding section)
> Use Claude Code to explore encoding: "What happens if I save JSON with emoji but use the wrong encoding?
> Show me the problem and how ensure_ascii=False solves it."

**Practice Approach**:

**Exercise 1**: Read a CSV file with at least 3 columns and 5 rows, then display each row in a formatted table
- Acceptance: All rows displayed correctly, headers shown
- Validation: Test with file containing special characters

**Exercise 2**: Convert CSV data to JSON format and save, then load it back and verify
- Acceptance: Data matches original structure, international characters preserved
- Validation: Compare loaded data to source CSV

**Exercise 3**: Implement error handling for both invalid CSV and invalid JSON files
- Acceptance: Doesn't crash on either format error, shows helpful message
- Validation: Test with corrupted file samples

**End-of-Lesson: Try With AI** (4 prompts):

**Prompt 1 - Remember/Understand**:
```
Ask your AI: "What's the difference between CSV and JSON? When would you use each format?"

Expected Outcome: You'll understand format selection based on data structure and use case.
```

**Prompt 2 - Apply**:
```
Ask your AI: "Write code that reads a CSV file containing contacts (name, email, phone),
filters for a specific domain, and saves the results to JSON."

Expected Outcome: You'll see CSV→JSON transformation with filtering.
```

**Prompt 3 - Analyze**:
```
Ask your AI: "Compare memory usage: reading a large CSV with csv.reader vs DictReader.
What's the tradeoff between convenience and memory efficiency?"

Expected Outcome**: You'll understand performance implications of different approaches.
```

**Prompt 4 - Synthesize/Create** (cognitive closure):
```
Ask your AI: "Design a data import system that reads CSV, validates required fields,
handles encoding issues with international names, converts to JSON, and saves.
What error cases must you handle?"

Expected Outcome: You'll see how CSV/JSON integrates with validation and error handling in real applications.
```

**Success Criteria**:
- Student can read and write CSV files with proper headers
- Student can load and save JSON with correct encoding
- Student can handle both formats with appropriate error handling
- Student can explain when to use CSV vs JSON
- 80%+ of students complete all "Try With AI" prompts

---

### Lesson 5: Capstone - Note-Taking App (B1 Integration)

**Learning Objective** (Bloom's: Apply & Analyze): Students integrate all I/O concepts (console I/O, file I/O, pathlib, CSV/JSON) into a complete, production-quality CLI application that persists data, handles user input safely, and manages file-based data structures at application scale.

**CEFR Proficiency Level**: B1 (Intermediate) — Integration of all lessons

**Skills Taught**:
- **Skill 1**: Integrating Multiple I/O Concepts in Single Application
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (system integration)
  - Measurable at This Level: Student can design an application using console I/O, file I/O, path handling, and structured formats together

- **Skill 2**: Implementing CRUD Operations (Create, Read, Update, Delete)
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data persistence)
  - Measurable at This Level: Student can implement all CRUD operations for file-based data storage

- **Skill 3**: Designing Menu-Driven Interactive Loops
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (interaction design)
  - Measurable at This Level: Student can design loop structures that display menu, accept input, execute action, return to menu

- **Skill 4**: Implementing Search and Filter Operations
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Analyze
  - DigComp Area: Problem-Solving (data querying)
  - Measurable at This Level: Student can search data structures for matching criteria and display results

- **Skill 5**: Application-Level Error Handling
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (robustness)
  - Measurable at This Level: Student can handle errors at multiple levels (user input, file operations, JSON parsing) and recover gracefully

- **Skill 6**: Data Validation at Application Scale
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (data integrity)
  - Measurable at This Level: Student can validate user input, file structure, and application state to maintain consistency

- **Skill 7**: Organizing Code for Maintainability
  - CEFR Level: B1
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (code organization)
  - Measurable at This Level: Student can structure application with functions, clear separation of concerns, and reusable components

**Key Concepts** (0 new concepts; integration of Lessons 1-4):
- All concepts from Lessons 1-4 combined in single application
- Application architecture: Main menu loop → functions for each operation → file I/O helpers
- Data structure: List of notes, each with id, title, body, category, created timestamp, modified timestamp
- File organization: notes/ directory with subdirectories for each category

**Prerequisites**:
- Lessons 1-4 of this chapter (all I/O concepts)
- Chapter 20: Functions and modules (for code organization)
- Chapter 21: Exception handling (for robust error handling)

**Duration**: 90–120 minutes (project time)

**Application Specification**:

**Note-Taking App Features**:

1. **Menu-Driven Interface**:
   - Display main menu with options: Create Note, Read Note, Update Note, Delete Note, Search, List Notes, Exit
   - Accept user choice (1-7)
   - Execute corresponding action
   - Return to menu after each action

2. **Create Note**:
   - Prompt for title, body, category
   - Generate UUID for note ID
   - Create notes/[category]/ directory if needed
   - Save as notes/[category]/[id].json with metadata (created, modified timestamps)
   - Confirm creation

3. **Read Note**:
   - List existing notes (by category or all)
   - Prompt user to select note
   - Display title, body, category, dates
   - Return to menu

4. **Update Note**:
   - List existing notes
   - Select note to update
   - Prompt for new title/body/category
   - Save with updated modified timestamp
   - Confirm update

5. **Delete Note**:
   - List existing notes
   - Confirm deletion (safety check)
   - Delete file
   - Confirm deletion

6. **Search**:
   - Prompt for search keyword
   - Search all notes' titles and bodies
   - Display matching notes
   - Return to menu

7. **List Notes**:
   - Show all notes organized by category
   - Display count per category

8. **Exit**:
   - Graceful shutdown with goodbye message

**Data Format** (JSON per note):
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "Python Study Notes",
  "body": "Today learned about file I/O with context managers...",
  "category": "learning",
  "created": "2025-11-09T10:30:00",
  "modified": "2025-11-09T14:45:00"
}
```

**Scale**:
- Expected 10-50 notes across 3-5 categories
- No database required; file-based storage is sufficient
- Performance acceptable for interactive use

**Content Outline**:
1. **Application Architecture Introduction** (10 min) - Design overview, user flow diagram
2. **Setting Up Project Structure** (10 min) - Initialize notes/ directory, explain organization
3. **Core Functions** (40 min) - Code functions for each CRUD operation
   - get_all_notes() → list[dict]
   - save_note(note: dict) → None
   - delete_note(note_id: str) → bool
   - search_notes(keyword: str) → list[dict]
4. **Menu Loop Implementation** (20 min) - Main loop structure with user input handling
5. **Error Handling and Validation** (10 min) - Input validation, file operation error handling
6. **Testing and Refinement** (10 min) - Manual testing, edge case handling

**Code Examples** (3-4 for Capstone):

**Code Example 5.1: Project Structure and Initialization**
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
    """Get path for a note, with security validation."""
    requested_path = (BASE_DIR / category / f"{note_id}.json").resolve()
    if not requested_path.is_relative_to(BASE_DIR.resolve()):
        raise ValueError(f"Security error: Invalid path")
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

note_path: Path = get_note_path("learning", note_id)
with open(note_path, 'w', encoding='utf-8') as f:
    json.dump(note, f, indent=2, ensure_ascii=False)

print(f"Note saved to: {note_path}")
```
- **Purpose**: Show project initialization with security
- **Complexity**: B1 (integrating pathlib, UUID, JSON)
- **Expected Outcome**: Student sees application structure before capstone

**Code Example 5.2: CRUD Functions**
```python
from pathlib import Path
import json
from datetime import datetime
from typing import Optional

def get_all_notes() -> list[dict]:
    """Load all notes from files."""
    notes: list[dict] = []
    base_dir = Path("notes")

    for json_file in base_dir.glob("**/*.json"):
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                note = json.load(f)
                notes.append(note)
        except json.JSONDecodeError:
            print(f"Warning: Corrupted note file {json_file.name}")

    return notes

def save_note(note: dict) -> bool:
    """Save or update a note."""
    try:
        category = note.get("category", "personal")
        note_id = note.get("id")

        note_dir = Path("notes") / category
        note_dir.mkdir(parents=True, exist_ok=True)

        note_path = note_dir / f"{note_id}.json"
        with open(note_path, 'w', encoding='utf-8') as f:
            json.dump(note, f, indent=2, ensure_ascii=False)
        return True
    except Exception as e:
        print(f"Error saving note: {e}")
        return False

def search_notes(keyword: str) -> list[dict]:
    """Find notes containing keyword."""
    all_notes = get_all_notes()
    keyword_lower = keyword.lower()

    matching = [
        note for note in all_notes
        if keyword_lower in note.get("title", "").lower() or
           keyword_lower in note.get("body", "").lower()
    ]
    return matching

def delete_note(note_id: str) -> bool:
    """Delete a note by ID."""
    try:
        all_notes = get_all_notes()
        for note in all_notes:
            if note["id"] == note_id:
                note_dir = Path("notes") / note["category"]
                note_file = note_dir / f"{note_id}.json"
                if note_file.exists():
                    note_file.unlink()  # Delete file
                    return True
        return False
    except Exception as e:
        print(f"Error deleting note: {e}")
        return False
```
- **Purpose**: Show CRUD function implementations
- **Complexity**: B1 (integration of file I/O, JSON, search)
- **Expected Outcome**: Student sees how functions handle data persistence

**Code Example 5.3: Menu Loop with Input Validation**
```python
import uuid
from datetime import datetime
from pathlib import Path
import json

def main():
    """Main application loop."""
    print("Welcome to Note-Taking App!")

    while True:
        # Display menu
        print("\n" + "="*40)
        print("1. Create Note")
        print("2. Read Note")
        print("3. Update Note")
        print("4. Delete Note")
        print("5. Search Notes")
        print("6. List All Notes")
        print("7. Exit")
        print("="*40)

        # Get user choice with validation
        try:
            choice_str: str = input("Enter choice (1-7): ").strip()
            choice: int = int(choice_str)

            if choice not in range(1, 8):
                print("Invalid choice. Please enter 1-7.")
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
                print("Goodbye!")
                break

        except ValueError:
            print("Invalid input. Please enter a number between 1 and 7.")
        except Exception as e:
            print(f"Unexpected error: {e}")

def create_note():
    """Prompt for new note and save."""
    print("\n--- Create New Note ---")
    title: str = input("Title: ").strip()
    body: str = input("Body: ").strip()
    category: str = input("Category (work/personal/learning): ").strip() or "personal"

    if not title or not body:
        print("Error: Title and body cannot be empty.")
        return

    note = {
        "id": str(uuid.uuid4()),
        "title": title,
        "body": body,
        "category": category,
        "created": datetime.now().isoformat(),
        "modified": datetime.now().isoformat(),
    }

    if save_note(note):
        print(f"Note created successfully! ID: {note['id']}")
    else:
        print("Error creating note.")

def list_all_notes():
    """Display all notes organized by category."""
    notes = get_all_notes()
    if not notes:
        print("No notes found.")
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
        print(f"\n{category.upper()}:")
        for note in by_category[category]:
            print(f"  • {note['title']} (ID: {note['id'][:8]}...)")

# [Other functions: read_note, update_note, delete_note_menu, search_notes_menu]
# (Abbreviated for space; full implementation in actual lesson)

if __name__ == "__main__":
    main()
```
- **Purpose**: Show menu loop structure and input validation
- **Complexity**: B1 (orchestrating all I/O concepts)
- **Expected Outcome**: Student sees application flow connecting user input to operations

**CoLearning Elements**:

#### 💬 AI Colearning Prompt (at project start)
> "I'm building a note-taking app. Before I write code, what data structure should I use?
> Should each note be one file or all notes in one file? What are the tradeoffs?"

**Expected Outcome**: Student thinks through design decisions before implementation

#### 🎓 Instructor Commentary (after showing CRUD functions)
> In AI-native development, you don't memorize file paths or JSON syntax — you understand APPLICATION FLOW.
> Your job: specify what the app should do (create, read, update, delete). AI generates the file handling code.
> You validate that it works.

#### 🚀 CoLearning Challenge (during project development)
Ask your AI Co-Teacher:
> "My app is slow when I have 1000 notes—every operation loads all notes.
> How would you optimize this? What data structure would help?"

**Expected Outcome**: Student thinks about scaling and performance tradeoffs

#### ✨ Teaching Tip (during testing)
> Use Claude Code to test edge cases: "What happens if someone creates a note with title ''empty'',
> then tries to update it? Or deletes it twice? Show me the error and how to prevent it."

**Project Deliverables**:

Students will build:
1. **Main application file** (app.py or main.py) with menu loop
2. **Helper functions module** (optional) with CRUD and utility functions
3. **Data directory** (notes/) with category subdirectories
4. **Working application** that:
   - Accepts user menu input without crashing
   - Creates notes with UUID and timestamps
   - Saves/loads notes as JSON with proper encoding
   - Searches for notes by keyword
   - Updates and deletes notes
   - Returns to menu after each operation

**Success Criteria**:
- ✅ Application runs without crashes
- ✅ All CRUD operations work correctly
- ✅ User input is validated (no empty notes, valid menu choices)
- ✅ Notes persist between sessions (JSON files remain)
- ✅ Search finds notes by keyword
- ✅ Application handles errors gracefully (missing file, corrupted JSON)
- ✅ Code organized with functions for each operation
- ✅ Supports 10-50 notes at reasonable performance
- ✅ 80%+ of students complete capstone project

**End-of-Lesson: Try With AI** (4 prompts with capstone focus):

**Prompt 1 - Remember/Understand**:
```
Ask your AI: "Show me the overall architecture of a note-taking app.
What are the main components and how do they interact?"

Expected Outcome: You'll understand application structure before coding.
```

**Prompt 2 - Apply**:
```
Ask your AI: "Write the main menu loop that displays options 1-7 and calls different functions
based on user input. Add input validation to reject invalid choices."

Expected Outcome: You'll see how menu routing and validation work together.
```

**Prompt 3 - Analyze**:
```
Ask your AI: "I'm building a note-taking app and deciding: should I store one note per file or
all notes in one JSON file? Compare these approaches. Which is better and why?"

Expected Outcome: You'll understand data organization tradeoffs.
```

**Prompt 4 - Synthesize/Create** (cognitive closure):
```
Ask your AI: "Design a complete Note-Taking app with Create/Read/Update/Delete operations.
Walk me through the application flow: user sees menu → selects option → app responds → returns to menu.
What error cases must we handle? What does the data structure look like?"

Expected Outcome: You've integrated all I/O concepts into a cohesive, production-quality application
demonstrating mastery of Chapter 22. Ready for file-based applications in Chapters 23+.
```

**Success Criteria**:
- Student builds working Note-Taking App with all CRUD operations
- Application persists data to JSON files in category directories
- User input is validated and errors handled gracefully
- Application is scalable (10-50 notes) and maintainable
- 80%+ of students complete capstone project
- Students can explain architectural decisions

---

## 📈 Skills Proficiency Mapping

**Comprehensive skills taught across all 5 lessons with CEFR progression:**

| Lesson | Skill Name | CEFR Level | Category | Bloom's Level | Assessment Method |
|--------|-----------|-----------|----------|---------------|-------------------|
| 1 | Using input() for User Interaction | A2 | Technical | Apply | Write program that takes user input |
| 1 | Input Validation with Type Conversion | A2-B1 | Technical | Apply | Validate user age/number with retry |
| 1 | Formatting Output with F-Strings | A2 | Technical | Apply | Format currency, numbers, alignment |
| 1 | Error Recovery in Interactive Programs | A2-B1 | Conceptual | Understand | Explain try/except retry pattern |
| 1 | Understanding Console I/O Patterns | A2 | Conceptual | Understand | Distinguish console vs file I/O |
| 2 | Context Managers (with statement) | A2-B1 | Technical | Apply | Use `with open()` for safe file handling |
| 2 | File Modes and When to Use Each | A2 | Conceptual | Understand | Explain 'r', 'w', 'a', 'r+' differences |
| 2 | Reading Files with Different Methods | A2-B1 | Technical | Apply | Choose read/readline/readlines appropriately |
| 2 | Writing Files with Formatting | A2-B1 | Technical | Apply | Write strings to files with proper newlines |
| 2 | File I/O Exception Handling | A2-B1 | Technical | Apply | Catch FileNotFoundError, PermissionError |
| 2 | Text vs Binary File Modes | B1 | Conceptual | Understand | Explain encoding implications |
| 2 | Resource Cleanup and File Safety | A2-B1 | Conceptual | Understand | Explain why context managers prevent corruption |
| 3 | Creating Path Objects | B1 | Technical | Apply | Use pathlib.Path with `/` operator |
| 3 | Path Existence and Type Checking | B1 | Technical | Apply | Use exists(), is_file(), is_dir() |
| 3 | Creating and Managing Directories | B1 | Technical | Apply | Use mkdir(parents=True, exist_ok=True) |
| 3 | File Listing with Glob Patterns | B1 | Technical | Apply | Use glob() and iterdir() for discovery |
| 3 | Cross-Platform Path Compatibility | B1 | Conceptual | Understand | Explain why pathlib beats hardcoded paths |
| 3 | Path Traversal Security | B1 | Technical | Apply | Use resolve() and is_relative_to() for validation |
| 4 | Reading CSV with csv.reader/DictReader | B1 | Technical | Apply | Parse CSV files, handle headers |
| 4 | Writing CSV with csv.DictWriter | B1 | Technical | Apply | Write data with proper headers |
| 4 | Loading JSON with json.load() | B1 | Technical | Apply | Load JSON from file with UTF-8 |
| 4 | Saving Data to JSON with json.dump() | B1 | Technical | Apply | Save Python objects as JSON |
| 4 | Character Encoding (UTF-8) | B1 | Conceptual | Understand | Explain encoding importance, ensure_ascii |
| 4 | CSV/JSON Exception Handling | B1 | Technical | Apply | Catch JSONDecodeError, ValueError |
| 4 | CSV vs JSON Format Selection | B1 | Conceptual | Analyze | Determine appropriate format by use case |
| 5 | Integrating I/O Concepts | B1 | Technical | Apply | Combine console/file/path/formats |
| 5 | CRUD Operations Implementation | B1 | Technical | Apply | Implement Create/Read/Update/Delete |
| 5 | Menu-Driven Interactive Loops | B1 | Technical | Apply | Design and implement menu dispatch |
| 5 | Search and Filter Operations | B1 | Technical | Analyze | Query data structures by criteria |
| 5 | Application-Level Error Handling | B1 | Technical | Apply | Handle errors at multiple levels |
| 5 | Data Validation at Scale | B1 | Technical | Apply | Validate data integrity in application |
| 5 | Code Organization for Maintainability | B1 | Technical | Apply | Structure code with functions and separation of concerns |

**Proficiency Progression**:
- **Lessons 1-2 (A2)**: Foundation skills, basic operations, understanding concepts
- **Lessons 2-3 (A2-B1)**: Bridging skills, combining concepts, practical patterns
- **Lessons 4-5 (B1)**: Advanced integration, real-world scenarios, architectural decisions

**Cognitive Load Analysis**:
- **Lesson 1**: 5 concepts (input, type conversion, f-strings, validation, error recovery) ✓ A2 limit
- **Lesson 2**: 7 concepts (context managers, file modes, reading methods, writing, exceptions, text/binary, cleanup) ✓ A2-B1 limit
- **Lesson 3**: 6 concepts (Path objects, existence checking, directory creation, glob patterns, cross-platform, security) ✓ B1 limit
- **Lesson 4**: 7 concepts (CSV reading/writing, JSON load/dump, encoding, error handling, format selection) ✓ B1 limit
- **Lesson 5**: 0 new concepts (integration only) ✓ B1 integration limit

---

## 🔄 Content Flow & Dependencies

**Progressive Complexity**:
1. **Lesson 1** (Console I/O) → Foundation for user interaction
2. **Lesson 2** (File I/O) → Build on input validation, add file persistence
3. **Lesson 3** (Pathlib) → Organize files, cross-platform compatibility
4. **Lesson 4** (CSV/JSON) → Structure data in files (requires Lessons 2-3)
5. **Lesson 5** (Capstone) → Integrate ALL concepts (requires Lessons 1-4)

**Key Integrations**:
- Lesson 1 error handling feeds into Lesson 2 file operations
- Lesson 2 file safety (context managers) applies to Lessons 3-4
- Lesson 3 path organization essential for Lesson 5 directory structure
- Lesson 4 data formats used in Lesson 5 note storage

---

## 🏗️ Scaffolding Strategy

**A2 → A2-B1 → B1 Progression**:

1. **Lesson 1 (A2)**: Simple operations with guidance
   - input() is straightforward
   - Type conversion has expected errors (ValueError)
   - F-strings are familiar from Chapter 16
   - Validation requires try/except (Chapter 21)

2. **Lesson 2 (A2-B1)**: Add resource management complexity
   - Context managers are NEW (with statement pattern)
   - Multiple file modes introduced
   - Multiple reading methods (choose appropriate)
   - Exception types vary (FileNotFoundError, PermissionError, IOError)

3. **Lesson 3 (B1)**: Object-oriented thinking with paths
   - Path objects are new (not strings)
   - Immutability concept (Path()/resolve() return new objects)
   - Methods for files vs directories
   - Security thinking (traversal prevention) is sophisticated

4. **Lesson 4 (B1)**: Data format complexity
   - CSV module is new (csv.reader vs csv.DictReader choice)
   - JSON serialization/deserialization is inverse concept
   - Encoding considerations (UTF-8, ensure_ascii=False)
   - Format selection requires understanding of use cases

5. **Lesson 5 (B1)**: Architectural thinking
   - Students orchestrate multiple I/O concepts
   - Data structure design (one file per note vs single file)
   - Application flow (menu loop, CRUD dispatch)
   - Error handling at multiple levels
   - Scaling thinking (handles 10-50 notes)

**Cognitive Load Management**:
- Each lesson introduces ONE major new concept (input→file→path→format→integration)
- Related skills grouped in same lesson
- Spiral learning: each lesson builds on prior without overwhelming
- Practice exercises validate mastery before moving forward

---

## 🔗 Integration Points

**Within Chapter 22**:
- Lesson 1 input validation → Lesson 5 menu input validation
- Lesson 2 file operations → Lesson 5 note persistence
- Lesson 3 path organization → Lesson 5 notes/[category]/[id].json structure
- Lesson 4 JSON format → Lesson 5 note storage format

**Bridge to Chapter 23** (Math, DateTime, Calendar):
- Lesson 5 uses datetime.now().isoformat() for timestamps
- Chapter 23 will teach advanced datetime operations
- Path operations from Lesson 3 used for file timestamping

**Prior Chapters**:
- Chapter 13: Basic print(), variable basics
- Chapter 14: Data types (understanding dicts, lists for JSON/CSV)
- Chapter 15: Type conversion (int(), float() for input validation)
- Chapter 16: F-strings (formatting output)
- Chapter 17: Loops (iterating files, menu loop)
- Chapter 18: Dicts (CSV DictReader, JSON objects)
- Chapter 19: Collections understanding
- Chapter 20: Functions (code organization in Lesson 5)
- Chapter 21: Exception handling (file errors, JSON errors)

---

## ✅ Validation Strategy

**For Technical Chapter**, students demonstrate understanding through:

1. **Console I/O** (Lesson 1):
   - Exercise: Input validation program that doesn't crash
   - Try With AI: Design quiz program structure

2. **File I/O** (Lesson 2):
   - Exercise: Read/write files with error handling
   - Try With AI: File backup program with format conversion

3. **Pathlib** (Lesson 3):
   - Exercise: Create directory structures, list files, validate paths
   - Try With AI: File management system with security

4. **Structured Formats** (Lesson 4):
   - Exercise: CSV→JSON conversion with error handling
   - Try With AI: Data import system with validation

5. **Capstone** (Lesson 5):
   - Project: Complete Note-Taking App with all CRUD operations
   - Assessment: App persists data, handles errors, supports 10-50 notes
   - Demonstration: Explain architectural decisions

**Success Metrics**:
- 75%+ can write input validation code without crashes
- 80%+ can use context managers for file safety
- 75%+ can work with pathlib for cross-platform paths
- 70%+ can read/write CSV and JSON correctly
- 85%+ can build complete Note-Taking App capstone
- 80%+ report confidence building file-based CLI applications

---

## 🎯 Architectural Decisions

**Decision 1: Five 75-90 minute lessons vs. Four longer lessons**
- **Rationale**: A2-B1 students need frequent breaks and practice cycles. Five lessons with 75-90 min duration allows more scaffolding checkpoints. Four long lessons would exceed cognitive load limits and reduce engagement.
- **ADR Candidate**: Yes — "Chapter 22 Lesson Structure: Five Lesson Pattern for I/O Mastery" (determines baseline for similar Part 4 chapters)

**Decision 2: File-Per-Note vs. Single File for Capstone**
- **Rationale**: File-per-note approach teaches pathlib directory organization, demonstrates scalability beyond list-in-memory, and prevents large JSON file parsing overhead. Single-file approach simpler but pedagogically weaker.
- **ADR Candidate**: Yes — "Note-Taking App Architecture: File-Per-Note Pattern" (establishes scaling pattern for later chapters)

**Decision 3: Menu Loop in Main App vs. Functions**
- **Rationale**: Capstone shows menu loop in main app with separate CRUD functions. This teaches separation of concerns (UI logic vs data logic) essential for Chapter 24 OOP.
- **ADR Candidate**: No — Standard pattern, not significant deviation

**Decision 4: JSON with ensure_ascii=False instead of CSV for notes**
- **Rationale**: JSON better supports nested metadata (id, title, body, category, timestamps). CSV is flat and harder to extend. Teaches students when JSON is appropriate.
- **ADR Candidate**: No — Technical choice, not architectural impact

**ADR Suggestions**:
1. Run `/sp.adr 22-lesson-structure-five-vs-four` to document lesson count decision
2. Run `/sp.adr 22-capstone-architecture-file-per-note` to document scalability pattern

---

## 📋 Dependencies and Risks

**Chapter Dependencies**:
- ✅ Chapter 21 (Exception Handling) — REQUIRED, foundational for file error handling
- ✅ Chapter 20 (Functions) — REQUIRED, needed for Lesson 5 function organization
- ✅ Chapter 17 (Loops) — REQUIRED, for file iteration and menu loops
- ✅ Chapters 13-19 — REQUIRED, general Python foundations

**Risk 1: Lesson 2 (File I/O) Complexity**
- **Issue**: Context managers (with statement) are conceptually challenging for A2 students; context manager syntax unfamiliar
- **Impact**: Could cause 15-20% of students to struggle with file safety concepts
- **Mitigation**:
  - Lead with explicit "Why context managers matter" explanation (resource cleanup, file corruption prevention)
  - Provide "before/after" comparison: manual close vs with statement
  - Emphasize "syntax is cheap; safety is gold" in CoLearning Commentary
  - Offer extended practice exercises with and without context managers

**Risk 2: Lesson 3 (Pathlib) Abstraction**
- **Issue**: Path objects as first-class values (immutable, methods) require OOP thinking students haven't formally learned (Chapter 24)
- **Impact**: Could cause confusion about `.resolve()`, `.is_relative_to()`, path semantics
- **Mitigation**:
  - Introduce with "Path objects are Python's smart tool for paths" (tool framing, not OOP jargon)
  - Show pathlib before os.path — students won't compare unfavorably
  - Use "method chains" language ("path.resolve() returns NEW path") instead of OOP terminology
  - Defer deep OOP explanation to Chapter 24

**Risk 3: Lesson 4 (CSV/JSON) Format Concepts**
- **Issue**: CSV headers vs data rows, JSON serialization/deserialization are subtle. Students may confuse csv.reader (tuples) with csv.DictReader (dicts)
- **Impact**: Could cause 10-15% to struggle with format selection and header handling
- **Mitigation**:
  - Show direct comparison: csv.reader vs csv.DictReader side-by-side with same data
  - Provide "when to use each" decision table
  - Emphasize DictReader for practical use (headers as keys more convenient)
  - Multiple practice exercises with both approaches

**Risk 4: Lesson 5 (Capstone Scope)**
- **Issue**: Complete Note-Taking App is ambitious for B1 students; risk of incomplete projects or feature creep
- **Impact**: Could cause 20-30% to not finish or feel overwhelmed
- **Mitigation**:
  - Provide starter code skeleton (menu loop, function stubs)
  - Break into incremental steps: (1) Menu only, (2) Create note, (3) Read note, (4) Update/Delete, (5) Search
  - Define MVP (Minimum Viable Product): CRUD only, no advanced features
  - Offer "extension ideas" for B1+ students (categories, search, timestamps)
  - Provide reference solution with explanations

**Risk 5: Time Constraints**
- **Issue**: 5-6 hours total; Lesson 5 capstone 90-120 minutes could pressure students in live classroom
- **Impact**: Could cause incomplete capstone or skipped practice exercises
- **Mitigation**:
  - Position Lesson 5 as "homework/project time" with optional in-class checkpoint
  - Provide complete capstone code as reference for students who run short on time
  - Emphasize that attempting project is success (even if incomplete); learning happens in trying

**Mitigation Summary**:
- Lesson 2: Extra explanation and before/after examples for context managers
- Lesson 3: Tool framing (not OOP jargon), side-by-side comparisons
- Lesson 4: Direct format comparison, decision tables, multiple exercises
- Lesson 5: Starter code, MVP definition, incremental steps, reference solution
- Overall: Provide "Try With AI" prompts that can scaffold struggling students

---

## 📚 Next Steps

After this plan is approved:

1. **Quality Gate**: Run `/sp.analyze` to validate cross-artifact consistency (spec ↔ plan alignment)
2. **Architecture Review**: Review ADR suggestions above; run `/sp.adr [decision-title]` for significant decisions
3. **Task Decomposition**: Run `/sp.tasks` to create implementation checklist with acceptance criteria
4. **Implementation**: Use `/sp.implement` or content-implementer agent to generate lesson files from plan
5. **Validation**: Technical and content review to ensure:
   - All skills metadata included in lesson frontmatter
   - Cognitive load verified (max 5-7 concepts per lesson)
   - CoLearning elements distributed throughout (not just at end)
   - "Try With AI" sections properly formatted with Bloom's progression
   - Code examples tested and working with Python 3.14+
   - Type hints on all functions
   - pathlib used instead of os.path
   - context managers for all file operations
   - UTF-8 encoding explicit for text files

---

## 📊 Chapter-Level Success Criteria

**All criteria from approved specification must be met**:

- ✅ SC-001: Students complete console I/O exercise in under 15 minutes with AI assistance
- ✅ SC-002: 80%+ write file reading/writing code without syntax errors on first attempt
- ✅ SC-003: Cross-platform paths work identically on Windows, Mac, Linux
- ✅ SC-004: 85%+ complete capstone using CSV/JSON formats
- ✅ SC-005: Note-Taking App completable in 90 minutes for B1 learners
- ✅ SC-006: 70%+ report confidence building file-based CLI applications
- ✅ SC-007: Students explain and mitigate path traversal security risks

**Chapter validation complete when**:
- All 5 lessons written and reviewed
- All code examples tested with Python 3.14+
- All 20 "Try With AI" prompts (4 per lesson) validated for clarity
- Cognitive load analysis shows all lessons ≤ 7 concepts
- All skills (31 total) have proficiency levels and assessment methods
- Capstone project completable in documented time with reference solution provided
- Chapter integrates with Chapters 13-21 (prerequisites) and bridges to Chapter 23 (continuation)

---

**Plan Status**: ✅ READY FOR IMPLEMENTATION

This detailed plan provides complete guidance for transforming the Chapter 22 specification into production-quality lessons with full pedagogical scaffolding, skills metadata, and CoLearning elements integrated throughout.
