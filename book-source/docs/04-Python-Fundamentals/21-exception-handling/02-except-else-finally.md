---
sidebar_position: 2
title: "Except, Else, and Finally"
description: "Master multi-block exception handling to control flow when errors occur"
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Multiple Exception Handling"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write 2+ except blocks to handle different exception types appropriately in guided contexts"

  - name: "Control Flow Understanding"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can predict which block (except/else/finally) executes for different error scenarios"

  - name: "Else/Finally Patterns"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can use else block (no exception) and finally block (guaranteed execution) appropriately for cleanup and validation"

learning_objectives:
  - objective: "Write multiple except blocks to handle different exception types with appropriate recovery strategies"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Code examples demonstrating exception-type-specific handling"

  - objective: "Explain when and why else and finally blocks execute in different error scenarios"
    proficiency_level: "A2-B1"
    bloom_level: "Understand"
    assessment_method: "Tracing code execution through all four blocks (try/except/else/finally)"

  - objective: "Apply finally block for guaranteed cleanup operations regardless of exception occurrence"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Writing cleanup code in finally blocks that works for both success and failure paths"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (multiple except blocks, else block, finally block) within A2-B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore exception inheritance and catching parent exception classes; research nested try/except blocks"
  remedial_for_struggling: "Focus on simple two-except case first; use flowchart diagrams showing execution path; test interactively"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "content-implementer subagent"
version: "2.0.0"
---

# Except, Else, and Finally

You've already learned that `try/except` catches errors and prevents crashes. But what if your code could encounter different types of errors? What if you want to run code only when NO error occurs? What if you need to clean up resources no matter what happens?

In this lesson, you'll master the complete four-block exception handling structure through prediction and discovery. You'll start by predicting flow control in file operations, learn from AI about else vs finally semantics, challenge AI with return statement edge cases, and build a file operation template for production use.

---

## Part 1: Student Predicts Flow Control in File Operations

**Your Role**: Flow control analyst predicting execution paths

Before learning the syntax, develop intuition about control flow. Professional developers reason about execution paths before writing code.

### Prediction Exercise: Which Block Runs When?

Study this code structure (don't run it yet):

```python
def read_file_safely(filename: str) -> str:
    """Read file with complete error handling."""
    content = ""

    try:
        print("A: Opening file...")
        file = open(filename, 'r')
        content = file.read()
        print("B: File read successfully")
    except FileNotFoundError:
        print("C: File not found, handling error")
    else:
        print("D: No errors occurred, success path")
    finally:
        print("E: Cleanup phase")
        if 'file' in locals():
            file.close()

    return content
```

### Your Prediction Task

**Before running code**, predict which print statements execute for each scenario:

**Scenario 1**: File exists and is readable
- Which prints execute?
- In what order?
- Your prediction: ________________

**Scenario 2**: File does not exist
- Which prints execute?
- In what order?
- Your prediction: ________________

**Scenario 3**: File exists but has permission issues (PermissionError, not caught)
- Which prints execute?
- What happens to the program?
- Your prediction: ________________

### Test Your Predictions

Now create test files and run the code:

```python
# Test 1: File exists
with open("test_success.txt", "w") as f:
    f.write("Success case")

result = read_file_safely("test_success.txt")
print(f"Result: {result}\n")

# Test 2: File doesn't exist
result = read_file_safely("nonexistent.txt")
print(f"Result: {result}\n")
```

**Compare your predictions to actual output.**

### Discovery Questions

After running tests, answer these:

1. **When does the `else` block run?** (Only when try succeeds? Even if except runs?)
2. **When does the `finally` block run?** (Always? Only on success? Only on error?)
3. **What's the execution order?** (try → except → else → finally, or different?)
4. **What happens if you have multiple except blocks?** (Which one runs? All of them? First match?)

### Flow Control Rules You Discovered

Based on your experiments, document these rules:

| Scenario | try | except | else | finally |
|----------|-----|--------|------|---------|
| No error | Runs completely | Skipped | Runs | Runs |
| Error caught | Runs until error | Matching block runs | Skipped | Runs |
| Error NOT caught | Runs until error | None match | Skipped | Runs, then propagates |

**Deliverable**: Create `flow_control_predictions.txt` documenting:
- Your initial predictions for all 3 scenarios
- Actual results from running the code
- The rules you discovered about when each block executes
- One new scenario you designed to test an edge case

---

## Part 2: AI Explains Else (Success Path) vs Finally (Cleanup Guarantee)

**Your Role**: Student learning semantic distinctions from AI Teacher

Now that you've observed the behavior, understand the WHY behind it. Ask AI to explain the semantic purpose of each block.

### AI Teaching Prompt

Ask your AI companion:

> "I've experimented with try/except/else/finally blocks. I observed that:
> - `else` runs only when try succeeds (no exception)
> - `finally` runs always, even after exceptions
>
> Explain:
> 1. What is the SEMANTIC purpose of the else block? Why not just put success code at the end of try?
> 2. What is the SEMANTIC purpose of the finally block? Why is 'always runs' important?
> 3. Show me a real-world file operation example where else and finally have different purposes."

### What You'll Learn from AI

**Expected AI Response** (summary):

**Else block purpose**:
- Separates "success-only" code from "might fail" code
- Makes exception source clear (if exception happens in else, it's not from try block)
- Communicates intent: "This code should only run if NO errors occurred"

**Finally block purpose**:
- Guarantees cleanup regardless of success, failure, or even return statements
- Essential for resource management (files, network connections, locks)
- Runs even if exception is re-raised or not caught

**Key distinction**:
- `else` = "success path logic"
- `finally` = "cleanup that must happen no matter what"

### Convergence Activity

After AI explains, test your understanding:

Ask AI: "Show me a file operation where I open a file, process it, log success in else block, and close it in finally. Then explain: what happens if processing raises an exception not caught by any except block?"

**Example AI might show**:
```python
def process_log_file(filename: str) -> int:
    """Process log file, count lines, ensure file is closed."""
    file = None
    line_count = 0

    try:
        file = open(filename, 'r')
        for line in file:
            line_count += 1
            # Processing could raise various exceptions
    except FileNotFoundError:
        print(f"Log file not found: {filename}")
        return 0
    except PermissionError:
        print(f"Cannot read {filename}: permission denied")
        return 0
    else:
        # Success path: log the result
        print(f"Successfully processed {line_count} lines")
    finally:
        # Guaranteed cleanup: close file if opened
        if file is not None:
            file.close()
            print("File closed")

    return line_count
```

**Your turn**: Explain back to AI:
- Why is `file.close()` in finally, not in else?
- What happens if an `IOError` occurs during line processing (not caught)?
- Does finally run before or after the function returns?

**Deliverable**: Write a 1-paragraph summary explaining the semantic difference between else (success-specific logic) and finally (guaranteed cleanup), with one real-world example.

---

## Part 3: Student Challenges AI with Return Statement Edge Cases

**Your Role**: Student teaching AI by exploring complex control flow

Now reverse the roles. You'll design challenges that test AI's understanding of try/except/finally with return statements—one of the trickiest edge cases.

### Challenge Design Pattern

Create scenarios where:
1. Return statements appear in different blocks
2. AI must predict which value is actually returned
3. Finally block interacts with return statements

### Challenge 1: Multiple Return Points

**Your prompt to AI**:
> "Predict what this function returns for each scenario. Explain your reasoning BEFORE running the code.
>
> ```python
> def mystery_return(should_fail: bool) -> str:
>     try:
>         if should_fail:
>             raise ValueError("Intentional error")
>         return "A: try block return"
>     except ValueError:
>         return "B: except block return"
>     else:
>         return "C: else block return"
>     finally:
>         print("D: finally block executed")
> ```
>
> What does this return when:
> 1. `mystery_return(False)` — which return executes?
> 2. `mystery_return(True)` — which return executes?
> 3. Why doesn't the else block return ever execute?"

**Expected AI Response**:
1. Returns "A: try block return" (try succeeds, returns immediately, else is skipped)
2. Returns "B: except block return" (exception caught, except returns)
3. Else only runs when try completes WITHOUT returning (rare case)

**Key insight**: Return in try or except skips else, but finally still runs.

### Challenge 2: Finally Overriding Return

**Your prompt to AI**:
> "What does this function return? Explain what's happening and whether this is good practice.
>
> ```python
> def finally_override(value: int) -> int:
>     try:
>         return value * 2
>     finally:
>         return value * 3
> ```
>
> When called with `finally_override(10)`, what's returned: 20 or 30? Why is this pattern dangerous?"

**Expected AI Response**: Returns 30 (finally return overrides try return). This is dangerous because finally is meant for cleanup, not altering control flow. It hides the intended return value.

### Challenge 3: Finally Without Return

**Your prompt to AI**:
> "Contrast the previous example with this one. What's the difference?
>
> ```python
> def finally_cleanup(value: int) -> int:
>     result = 0
>     try:
>         result = value * 2
>         return result
>     finally:
>         print(f"Finally: result was {result}")
>         result = value * 3  # This won't change the return value
> ```
>
> When called with `finally_cleanup(10)`, what's returned? Why is this version better than the previous one?"

**Expected AI Response**: Returns 20 (try block return). Finally runs, prints the message, modifies local variable, but doesn't override return. This is proper use of finally—cleanup and logging without altering control flow.

### Your Analysis

After AI responds to all three challenges, write:
- Which pattern is correct professional practice?
- Why should finally avoid return statements?
- When would you use return in except blocks?

**Deliverable**: Document three return statement challenges you posed to AI, AI's predictions, and your analysis of best practices for return placement in try/except/else/finally blocks.

---

## Part 4: Build File Operation Template with Proper Cleanup

**Your Role**: Template designer creating reusable production patterns

Now integrate everything into a production-ready file operation template that handles all scenarios correctly.

### Your File Operation Template Library

Create a Python file called `file_operations_template.py` with these patterns:

```python
"""
File Operation Templates with Comprehensive Error Handling
Chapter 21, Lesson 2
"""

from typing import Optional
import json


def read_text_file(filename: str, default: str = "") -> str:
    """
    Read text file with guaranteed cleanup.

    Pattern: try/except/else/finally for file reading

    Args:
        filename: Path to file
        default: Value to return if file not found

    Returns:
        File contents or default value

    Error handling:
        - FileNotFoundError: Return default
        - PermissionError: Log and return default
        - Finally: Ensure file is closed
    """
    file = None
    content = default

    try:
        file = open(filename, 'r')
        content = file.read()
    except FileNotFoundError:
        print(f"File not found: {filename}, using default")
        return default
    except PermissionError:
        print(f"Permission denied: {filename}, using default")
        return default
    else:
        print(f"Successfully read {len(content)} characters from {filename}")
    finally:
        if file is not None:
            file.close()
            print(f"File closed: {filename}")

    return content


def write_text_file(filename: str, content: str) -> bool:
    """
    Write text file with error handling.

    Pattern: try/except/finally for file writing

    Args:
        filename: Path to file
        content: Text to write

    Returns:
        True if successful, False otherwise

    Error handling:
        - PermissionError: Log and return False
        - IOError: Log and return False
        - Finally: Ensure file is closed
    """
    file = None
    success = False

    try:
        file = open(filename, 'w')
        file.write(content)
        success = True
    except PermissionError:
        print(f"Cannot write to {filename}: permission denied")
    except IOError as e:
        print(f"IO error writing {filename}: {e}")
    else:
        print(f"Successfully wrote {len(content)} characters to {filename}")
    finally:
        if file is not None:
            file.close()

    return success


def load_json_file(filename: str, default: Optional[dict] = None) -> dict:
    """
    Load JSON file with fallback to default.

    Pattern: Multiple except blocks for different error types

    Args:
        filename: Path to JSON file
        default: Default dict if file missing or invalid

    Returns:
        Parsed JSON dict or default

    Error handling:
        - FileNotFoundError: Return default
        - json.JSONDecodeError: Log error, return default
        - Finally: Ensure file is closed
    """
    if default is None:
        default = {}

    file = None

    try:
        file = open(filename, 'r')
        data = json.load(file)
    except FileNotFoundError:
        print(f"JSON file not found: {filename}, using default")
        return default
    except json.JSONDecodeError as e:
        print(f"Invalid JSON in {filename}: {e}, using default")
        return default
    except PermissionError:
        print(f"Cannot read {filename}: permission denied, using default")
        return default
    else:
        print(f"Successfully loaded JSON from {filename}")
        return data
    finally:
        if file is not None:
            file.close()


def process_file_lines(filename: str, max_errors: int = 10) -> tuple[list[str], int]:
    """
    Process file line by line with graceful degradation.

    Pattern: Nested try/except (outer for file, inner for line processing)

    Args:
        filename: Path to file
        max_errors: Maximum errors before stopping

    Returns:
        Tuple of (valid_lines, error_count)

    Error handling:
        - FileNotFoundError: Return empty list
        - Errors per line: Skip line, increment counter
        - Finally: Ensure file is closed
    """
    file = None
    valid_lines = []
    error_count = 0

    try:
        file = open(filename, 'r')

        for line_num, line in enumerate(file, start=1):
            try:
                # Process line (example: strip and validate)
                processed = line.strip()
                if not processed:
                    raise ValueError("Empty line")
                valid_lines.append(processed)
            except ValueError as e:
                error_count += 1
                print(f"Line {line_num}: {e}, skipping")

                if error_count >= max_errors:
                    print(f"Max errors ({max_errors}) reached, stopping")
                    break

    except FileNotFoundError:
        print(f"File not found: {filename}")
        return [], 0
    except PermissionError:
        print(f"Cannot read {filename}: permission denied")
        return [], 0
    else:
        print(f"Processed {len(valid_lines)} valid lines")
    finally:
        if file is not None:
            file.close()

    return valid_lines, error_count


# Example usage and testing
if __name__ == "__main__":
    print("=== Testing File Operation Templates ===\n")

    # Test 1: Read existing file
    print("Test 1: Read text file")
    content = read_text_file("test_read.txt", default="No content")
    print(f"Content: {content[:50]}...\n")

    # Test 2: Write file
    print("Test 2: Write text file")
    success = write_text_file("test_write.txt", "Hello, World!")
    print(f"Write successful: {success}\n")

    # Test 3: Load JSON
    print("Test 3: Load JSON file")
    data = load_json_file("config.json", default={"theme": "light"})
    print(f"Config: {data}\n")

    # Test 4: Process lines with errors
    print("Test 4: Process file lines")
    lines, errors = process_file_lines("data.txt")
    print(f"Valid lines: {len(lines)}, Errors: {errors}")
```

### Template Requirements

Your file operation template library must include:

1. **Read text file template**
   - FileNotFoundError handling
   - PermissionError handling
   - Finally block for cleanup
   - Else block for success logging

2. **Write text file template**
   - IOError handling
   - Permission handling
   - Success/failure return value

3. **Load JSON template**
   - Multiple except blocks for different errors
   - Fallback to default values
   - Proper JSON decoding error handling

4. **Process lines template**
   - Nested try/except (file-level and line-level)
   - Graceful degradation (skip bad lines)
   - Error threshold enforcement

5. **Testing section**
   - Test cases for each template
   - Both success and failure scenarios

### Validation with AI

Once your template is complete, validate it by asking AI:

> "Review my file operation templates. For each function:
> 1. Is the exception handling appropriate for the error types?
> 2. Are else and finally blocks used correctly?
> 3. What edge cases am I missing?
> 4. Are there any resource leaks or cleanup issues?
> 5. Suggest one improvement for each template."

**Deliverable**: Complete `file_operations_template.py` with all 4 templates, comprehensive error handling, and test cases demonstrating both success and failure paths.

---

## Summary: Bidirectional Learning Pattern

In this lesson, you experienced all three roles:

**Part 1 (Student predicts)**: You predicted flow control before running code, building intuition
**Part 2 (AI teaches)**: AI explained the semantic purpose of else (success) vs finally (cleanup)
**Part 3 (Student teaches)**: You challenged AI with return statement edge cases
**Part 4 (Knowledge synthesis)**: You built production-ready file operation templates

This pattern ensures you understand exception handling control flow not just as syntax, but as a reasoning framework for robust resource management.

### What You've Built

1. `flow_control_predictions.txt` — Your predictions vs actual behavior for each scenario
2. Semantic distinction summary — Understanding of else vs finally purposes
3. Return statement edge case analysis — Three challenges and best practices
4. `file_operations_template.py` — Reusable file operation templates with proper cleanup

### Key Insights

**Flow control rules**:
- **try**: Runs first, stops at first exception
- **except**: Runs only if matching exception occurs
- **else**: Runs only if try completed without exception (no return)
- **finally**: ALWAYS runs, even with return statements

**Best practices**:
- Use **else** for success-specific logic (separates concerns)
- Use **finally** for guaranteed cleanup (resource management)
- Avoid **return in finally** (overrides other returns)
- Use **multiple except blocks** for different error types (specific handling)

### Next Steps

Lesson 3 builds on this foundation, teaching you how to raise your own exceptions and create custom exception classes for domain-specific errors. Your file operation templates will serve as examples of where custom exceptions make code clearer.
