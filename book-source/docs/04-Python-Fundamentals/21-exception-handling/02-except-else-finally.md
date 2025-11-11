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
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "lesson-writer subagent"
version: "1.0.0"
---

# Except, Else, and Finally

You've already learned that `try/except` catches errors and prevents crashes. But what if your code could encounter *different* types of errors? What if you want to run code only when NO error occurs? What if you need to clean up resources no matter what happens?

In this lesson, you'll master the complete four-block exception handling structure: `try`, multiple `except` blocks, `else`, and `finally`. You'll understand when each block executes and build robust programs that handle complex error scenarios elegantly.

Think of it like this: `try` is your attempt to do something, `except` is how you recover from each type of failure, `else` is what happens on success, and `finally` is your cleanup crew that always arrives—whether things went right or wrong.

## Quick Review: Single Try/Except

From Lesson 1, you know the basic structure:

```python
try:
    # Code that might raise an exception
    risky_operation()
except ValueError:
    # Code to run if ValueError occurs
    print("Error: Invalid value")
```

This works well when you only expect one type of error. But what about real-world code?

## Multiple Except Blocks: Handling Different Errors

When your code could raise **different** exception types, you need **multiple except blocks**. Each block handles a specific error type.

**Why multiple except blocks?** Because different errors deserve different recovery strategies. A `FileNotFoundError` means "file missing—create it or ask user." A `ValueError` means "user entered wrong type—prompt for retry." Treating them the same way loses important context.

Here's the syntax:

```python
try:
    # Code that might raise multiple exception types
    risky_operation()
except ValueError:
    # Handles ValueError specifically
    print("Invalid value")
except TypeError:
    # Handles TypeError specifically
    print("Wrong type")
except ZeroDivisionError:
    # Handles ZeroDivisionError specifically
    print("Can't divide by zero")
```

**Critical rule**: Python evaluates except blocks **top-to-bottom**. The first matching except block runs, and Python skips the rest.

Let's see this with real code:

```python
# Example 1: Multiple Except Blocks for Different Errors

def divide_numbers(a: str, b: str) -> None:
    """Attempt to divide two numbers, handling multiple error types."""
    try:
        num_a: int = int(a)
        num_b: int = int(b)
        result: float = num_a / num_b
        print(f"{num_a} / {num_b} = {result}")
    except ValueError:
        print("Error: Could not convert input to number. Please enter integers.")
    except ZeroDivisionError:
        print("Error: Cannot divide by zero.")

# Test cases
divide_numbers("10", "2")      # Works: 10 / 2 = 5.0
divide_numbers("10", "0")      # ZeroDivisionError caught
divide_numbers("abc", "2")     # ValueError caught
divide_numbers("10", "xyz")    # ValueError caught
```

**Output:**
```
10 / 2 = 5.0
Error: Cannot divide by zero.
Error: Could not convert input to number. Please enter integers.
Error: Could not convert input to number. Please enter integers.
```

#### 💬 AI Colearning Prompt
> "Show me two exceptions in the same function. Why do we need to catch them separately? What happens if we catch them the same way?"

## When to Use Multiple Except Blocks

**Use multiple except blocks when**:
- Different error types need different recovery strategies
- Your code touches multiple systems (file system, network, user input)
- You want to log or report different error causes

**Example scenario**: Reading a user CSV file
- `FileNotFoundError`: Tell user "file not found"
- `PermissionError`: Tell user "you don't have permission"
- `ValueError`: Skip the malformed row and continue

## The Else Block: Success Path Only

Here's something powerful: what if you want code to run *only when NO exception occurred*?

That's what the `else` block does.

**Syntax:**

```python
try:
    # Code that might raise an exception
    risky_operation()
except SpecificError:
    # Code to run if SpecificError occurs
    handle_error()
else:
    # Code to run only if NO exception occurred
    handle_success()
```

**Critical**: The `else` block is optional. Use it only when you have distinct "success" code to run.

Let's see it in action:

```python
# Example 2: Try/Except/Else Showing When Else Runs

def process_user_input(input_str: str) -> None:
    """Process user input, showing different paths for success vs error."""
    try:
        age: int = int(input_str)
        print(f"Attempting to process age: {age}")
    except ValueError:
        print(f"Error: '{input_str}' is not a valid integer.")
    else:
        # This runs ONLY if no exception occurred
        if age >= 18:
            print(f"Success: User is an adult (age {age})")
        else:
            print(f"Success: User is a minor (age {age})")

# Test cases
process_user_input("25")    # No error → else block runs
process_user_input("abc")   # ValueError → else block SKIPPED
```

**Output:**
```
Attempting to process age: 25
Success: User is an adult (age 25)
Error: 'abc' is not a valid integer.
```

#### 🎓 Instructor Commentary
> In AI-native development, you don't memorize when else vs except runs—you understand the **semantics**. The semantics: else runs on the *success path*, except runs on the *error path*. That's the distinction that matters. Syntax comes later.

## The Finally Block: Guaranteed Execution

Now for the most powerful block: `finally`. Code in the finally block runs **no matter what**—whether an exception occurred or not, whether you caught it or not.

**Why does finally matter?** Cleanup. If you open a file, allocate memory, or start a database connection, you absolutely must clean up those resources. Finally guarantees cleanup runs regardless of success or failure.

**Syntax:**

```python
try:
    # Code that might raise an exception
    risky_operation()
except SpecificError:
    # Code to run if error occurs
    handle_error()
finally:
    # Code to run ALWAYS (success, error, or even if except block fails)
    cleanup()
```

Let's see finally in action:

```python
# Example 3: Try/Except/Finally Showing Guaranteed Execution

def validate_age_with_cleanup(age_str: str) -> None:
    """Validate age, demonstrating finally's guaranteed execution."""
    try:
        print(f"Processing age input: '{age_str}'")
        age: int = int(age_str)

        if age < 0:
            raise ValueError("Age cannot be negative")

        print(f"Success: Age is {age}")
    except ValueError as error:
        print(f"Error caught: {error}")
    finally:
        # This runs regardless of success or error
        print("Cleanup: Releasing resources")
        print()  # Blank line for readability

# Test cases
validate_age_with_cleanup("25")      # Success
validate_age_with_cleanup("abc")     # ValueError from int()
validate_age_with_cleanup("-5")      # ValueError from if condition
```

**Output:**
```
Processing age input: '25'
Success: Age is 25
Cleanup: Releasing resources

Processing age input: 'abc'
Error caught: invalid literal for int() with base 10: 'abc'
Cleanup: Releasing resources

Processing age input: '-5'
Error caught: Age cannot be negative
Cleanup: Releasing resources
```

Notice: The "Cleanup: Releasing resources" message printed in all three cases. That's finally at work.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Show me a real file operation using try/except/finally. Then explain why finally is essential for closing files."

**Expected Outcome**: You'll understand how finally prevents resource leaks in real-world code.

## Order Matters: The Complete Four-Block Structure

When you use all four blocks together, order is **absolutely critical**:

```python
try:
    # 1. Code that might raise an exception
    risky_operation()
except SpecificError:
    # 2. Exception handlers (can have multiple)
    handle_error()
else:
    # 3. Runs only if no exception occurred
    handle_success()
finally:
    # 4. Runs always—ALWAYS LAST
    cleanup()
```

**The execution order for different scenarios:**

| Scenario | Execution |
|----------|-----------|
| **No exception** | try → else → finally |
| **Exception caught** | try → except → finally |
| **Exception not caught** | try → except → finally (then exception propagates) |

Let's demonstrate this with the complete structure:

```python
# Example 4: Complete Four-Block Structure

def robust_calculation(a_str: str, b_str: str) -> None:
    """Calculate a/b with all four blocks showing complete control flow."""
    try:
        print(f"Attempting: {a_str} ÷ {b_str}")
        a: int = int(a_str)
        b: int = int(b_str)
        result: float = a / b
        print(f"Calculation successful: {a} / {b} = {result}")
    except ValueError:
        print("Error: Could not convert to number")
    except ZeroDivisionError:
        print("Error: Cannot divide by zero")
    else:
        # Runs only if calculation succeeded
        print(f"Success path: Result is {result}")
    finally:
        # Runs always
        print("Cleanup: Releasing calculation resources\n")

# Test scenarios
print("Scenario 1: Successful calculation")
robust_calculation("10", "2")

print("Scenario 2: Invalid input")
robust_calculation("abc", "2")

print("Scenario 3: Division by zero")
robust_calculation("10", "0")
```

**Output:**
```
Scenario 1: Successful calculation
Attempting: 10 ÷ 2
Calculation successful: 10 / 2 = 5.0
Success path: Result is 5.0
Cleanup: Releasing calculation resources

Scenario 2: Invalid input
Attempting: abc ÷ 2
Error: Could not convert to number
Cleanup: Releasing calculation resources

Scenario 3: Division by zero
Attempting: 10 ÷ 0
Error: Cannot divide by zero
Cleanup: Releasing calculation resources
```

## Real-World Pattern: File Operations with Finally

One of the most important uses of finally is ensuring files always close, even if an error occurs:

```python
# Pattern: File operations with guaranteed cleanup

def read_file_safely(filename: str) -> str:
    """Read file, ensuring proper cleanup with finally."""
    content: str = ""
    file = None

    try:
        file = open(filename, "r")
        content = file.read()
        print(f"Successfully read {len(content)} characters")
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
    else:
        print("File reading succeeded")
    finally:
        # This ensures file closes even if exception occurred
        if file is not None:
            file.close()
            print("File closed successfully")

# Test
read_file_safely("example.txt")  # File may or may not exist
```

#### ✨ Teaching Tip
> Use Claude Code to test different exception scenarios. Write code that triggers each path (success, specific errors, finally). Ask: "What order do these blocks execute in?" Then trace execution yourself before running the code.

## Exercises: Master Multi-Block Exception Handling

### Exercise 1: Multiple Except Blocks

Write a function that reads two numbers from strings and calculates their sum. Handle both `ValueError` (invalid number) and display different error messages. Test with valid numbers, invalid numbers, and edge cases.

**Starter code:**
```python
def add_numbers(a_str: str, b_str: str) -> None:
    """Add two numbers from string input, handling errors."""
    try:
        # Your code here
        pass
    except ValueError:
        # Your error message
        pass
    # Add another except block for a different error if desired

# Test cases
add_numbers("5", "3")      # Should print: 5 + 3 = 8
add_numbers("5", "abc")    # Should print error
```

### Exercise 2: Try/Except/Else Pattern

Write a function that validates email addresses. If the input can be converted to a string (always true for strings), print success. If validation fails, catch it. Use else to confirm validation succeeded.

**Starter code:**
```python
def validate_email(email: str) -> None:
    """Validate email format using try/except/else."""
    try:
        # Check if email contains '@'
        if "@" not in email:
            raise ValueError("Email must contain '@'")
        print(f"Email format looks good: {email}")
    except ValueError as error:
        print(f"Validation failed: {error}")
    else:
        # Code runs only if no exception
        print("Email is valid")

# Test
validate_email("user@example.com")  # Success path
validate_email("user")              # Error path
```

### Exercise 3: Try/Except/Finally Pattern

Write a function that reads a configuration file. Even if the file doesn't exist or has an error, always print "Configuration cleanup complete" using finally.

**Starter code:**
```python
def load_config(filename: str) -> None:
    """Load configuration file with guaranteed cleanup."""
    config_file = None
    try:
        config_file = open(filename, "r")
        content: str = config_file.read()
        print(f"Config loaded: {len(content)} bytes")
    except FileNotFoundError:
        print(f"Config file not found: {filename}")
    finally:
        # Guaranteed cleanup
        if config_file:
            config_file.close()
        print("Configuration cleanup complete")

# Test
load_config("settings.txt")
```

## Common Mistakes to Avoid

**Mistake 1**: Forgetting that except blocks are evaluated top-down
```python
# WRONG: More specific exception first, then generic
try:
    risky()
except Exception:           # This catches everything
    print("Error")
except ValueError:          # This never runs!
    print("Value error")    # Unreachable

# RIGHT: Specific exceptions first, generic last (if needed)
try:
    risky()
except ValueError:
    print("Value error")
except TypeError:
    print("Type error")
except Exception:           # Catches everything else
    print("Other error")
```

**Mistake 2**: Using else when you don't need it
```python
# UNNECESSARY: else does nothing special
try:
    result = int("5")
except ValueError:
    print("Error")
else:
    print(f"Success: {result}")  # Could just go in try block

# CLEANER: Put success code in try if no else-specific logic
try:
    result = int("5")
    print(f"Success: {result}")
except ValueError:
    print("Error")
```

**Mistake 3**: Not using finally for cleanup
```python
# BAD: File might not close if exception occurs
file = open("data.txt")
try:
    data = file.read()
except IOError:
    print("Error reading file")
file.close()  # Might not reach here!

# GOOD: finally guarantees cleanup
file = open("data.txt")
try:
    data = file.read()
except IOError:
    print("Error reading file")
finally:
    file.close()  # Always runs
```

## Try With AI

Your AI companion has learned about multiple except blocks, else, and finally. Use it to deepen your understanding and test your control flow knowledge.

**Tool**: Use your preferred AI companion (Claude Code CLI, Gemini CLI, or ChatGPT web).

### Prompt 1 (Remember): Recall the Blocks
> "What's the difference between an except block and a finally block? When does each one run?"

**Expected Outcome**: AI explains that except runs when an error occurs, finally always runs.

---

### Prompt 2 (Understand): Explain the Execution Path
> "Write a function with try/except/else/finally. Now trace through what happens if: (a) no exception occurs, (b) ValueError is raised."

**Expected Outcome**: AI shows execution traces for both scenarios, demonstrating the order of block execution.

---

### Prompt 3 (Apply): Write Code with All Blocks
> "Write a Python function that: (1) tries to open a file, (2) catches FileNotFoundError, (3) runs else to process file content on success, (4) uses finally to close the file. Include all four blocks."

**Expected Outcome**: AI provides working code showing all four blocks in proper sequence with realistic file handling.

---

### Prompt 4 (Analyze): Compare Finally vs Else
> "When would you use finally instead of else? Show an example where they have different purposes and explain the difference."

**Expected Outcome**: AI demonstrates that finally handles cleanup (always needed), while else handles success-path logic (conditional). Finally is about resources; else is about flow.

---

**Safety Note**: Exception handling is about being defensive. When you see code, ask yourself: "What could go wrong here?" The answer guides whether you need multiple except blocks (different error types), else (success path logic), or finally (cleanup). Test your assumptions by intentionally triggering errors.
