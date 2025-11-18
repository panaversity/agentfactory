---
title: "Capstone: Robust CSV Parser"
chapter: 21
lesson: 5
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Comprehensive Error Handling"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student integrates multiple exception handling patterns (try/except/else/finally, custom exceptions, error recovery strategies) in realistic project"

  - name: "Specification to Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student translates functional requirements (file I/O, validation, error handling) into working code with appropriate exception handling"

  - name: "Testing and Validation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student creates test cases for error scenarios and validates that error handling works without crashing"

learning_objectives:
  - objective: "Integrate all exception handling concepts (Lessons 1-4) in realistic project"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Build working CSV parser handling 4+ error types without crashing"

  - objective: "Apply error handling strategies strategically based on error type"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Justify choice of exception handling strategy (skip record, provide default, retry, report) for each error type"

  - objective: "Test error handling through intentional edge cases"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Create test cases that verify parser responds correctly to each error scenario"

cognitive_load:
  new_concepts: 0
  assessment: "0 new concepts introduced; pure integration of Lessons 1-4 patterns. All prior learning reviewed. ✓"

differentiation:
  extension_for_advanced: "Add features: support multiple file formats (JSON, XML), implement retry with exponential backoff, create custom exception hierarchy with specific error recovery for each exception type"
  remedial_for_struggling: "Start with provided template code; focus on understanding each try/except block before adding new error handling"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Capstone: Robust CSV Parser

You've learned the foundations of exception handling across four lessons. Now it's time to put everything together in a realistic project. In this capstone, you'll build a **CSV file parser** that reads user data, validates each record, and handles multiple error scenarios gracefully. This project integrates all the exception handling concepts you've learned—try/except/else/finally blocks, custom exceptions, strategic error recovery, and testing your error handling.

By the end of this lesson, you'll have a working program that demonstrates professional-quality error handling. You'll understand how to think defensively about code, anticipate problems, and build systems that don't crash when things go wrong.

## Project Specification

Your mission is to build a Python program that reads a CSV file and validates user records. Here's what your parser must do:

**Input File Format** (CSV with three columns):
```
name,age,email
Alice,28,alice@example.com
Bob,thirty-five,bob@example.com
Charlie,45,charlie@invalid
```

**Validation Rules**:
1. **Name**: Must be non-empty string
2. **Age**: Must be a positive integer between 0 and 150
3. **Email**: Must contain '@' symbol

**Error Handling Requirements**:
1. **FileNotFoundError**: File doesn't exist → Tell user the location they tried
2. **ValueError**: Malformed data in a row → Log error, skip row, continue processing
3. **PermissionError**: Can't read file → Tell user permissions issue
4. **General errors**: Unexpected issues → Report what happened

**Output**: Report summary showing:
- Total rows processed
- Rows successfully validated
- Rows skipped due to errors (with reasons)
- Log entries for debugging

**Success Criteria**:
- Parser never crashes, even with bad data
- Every error scenario produces helpful feedback
- Summary report shows what happened
- User can understand what went wrong and potentially fix it

## Planning Your Error Handling Strategy

Before diving into code, let's think strategically about where errors could occur and how to handle them.

**Where Errors Happen**:
- **Opening the file**: FileNotFoundError, PermissionError
- **Reading each line**: Malformed CSV structure (rare, but possible)
- **Validating age field**: ValueError when `int()` fails on non-numeric string
- **Validating email**: No error from validation logic itself (just checking for '@')

**Error Handling Strategy by Scenario**:

| Error | Root Cause | Strategy | Action |
|-------|-----------|----------|--------|
| FileNotFoundError | File path incorrect | Report and exit | Tell user where to find file |
| PermissionError | User lacks read rights | Report and exit | Tell user to check permissions |
| ValueError (age) | Non-numeric age value | Skip row | Log the bad value, continue |
| Invalid email | No '@' symbol | Skip row | Log which email was invalid, continue |

Notice the pattern: **fatal errors (file access) stop the program early**; **data validation errors skip just that row** and continue.

#### 💬 AI Colearning Prompt

> "Walk me through what errors could happen when reading a CSV file. Where would try/except blocks go in my parser?"

Think about this: errors in opening the file are different from errors in validating individual rows. One stops everything; the other should let you continue.

## Implementation Strategy

We'll build the parser step-by-step, starting simple and adding error handling for each scenario. The key is testing each error case as you go.

### Step 1: Validation Functions

First, let's create functions that validate each field and raise exceptions when needed:

```python
def validate_age(age_str: str) -> int:
    """
    Validate that age is a positive integer between 0 and 150.

    Raises:
        ValueError: If age is not a valid positive integer in range
    """
    try:
        age = int(age_str)
        if age < 0 or age > 150:
            raise ValueError(f"Age must be 0-150, got {age}")
        return age
    except ValueError as e:
        # Re-raise with clearer message
        raise ValueError(f"Invalid age: {age_str}") from e


def validate_email(email: str) -> str:
    """
    Validate that email contains '@' symbol.

    Raises:
        ValueError: If email doesn't contain '@'
    """
    if "@" not in email:
        raise ValueError(f"Email must contain '@', got: {email}")
    return email


def validate_name(name: str) -> str:
    """
    Validate that name is non-empty.

    Raises:
        ValueError: If name is empty
    """
    if not name or not name.strip():
        raise ValueError("Name cannot be empty")
    return name.strip()
```

#### 🎓 Expert Insight

> Notice that validation functions *raise* exceptions rather than returning success/failure flags. This is professional Python style. Your function either returns the validated data or signals an error. No ambiguity.

### Step 2: CSV Parser with Error Handling

Now let's build the parser that reads the file and validates each row:

```python
def parse_csv_file(filename: str) -> dict:
    """
    Parse CSV file with user data and validate each row.

    Args:
        filename: Path to CSV file

    Returns:
        Dictionary with 'valid': list of validated records,
                    'invalid': list of error messages,
                    'total': total rows processed

    Raises:
        FileNotFoundError: If file doesn't exist
        PermissionError: If file cannot be read
    """
    results = {
        'valid': [],
        'invalid': [],
        'total': 0
    }

    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        raise FileNotFoundError(f"File not found: {filename}. Check the file path and try again.")
    except PermissionError:
        raise PermissionError(f"Permission denied reading {filename}. Check file permissions.")

    # Skip header row and process data rows
    for row_num, line in enumerate(lines[1:], start=2):
        results['total'] += 1

        try:
            # Parse CSV line (simple split for this example)
            parts = line.strip().split(',')
            if len(parts) != 3:
                raise ValueError(f"Expected 3 fields, got {len(parts)}")

            name, age_str, email = parts

            # Validate each field
            validate_name(name)
            validate_age(age_str)
            validate_email(email)

            # If all validation passes, store the record
            results['valid'].append({
                'name': name,
                'age': int(age_str),
                'email': email
            })

        except ValueError as e:
            # Record the error but continue processing
            error_msg = f"Row {row_num}: {str(e)}"
            results['invalid'].append(error_msg)

    return results
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Review my CSV parser. What error scenarios am I NOT handling? What edge cases might break my code?"

**Expected Outcome**: You'll discover edge cases like empty files, lines with trailing spaces, or unusual CSV formatting. This is how professionals build robust systems—they think about what could go wrong.

### Step 3: Main Program with Complete Error Handling

Here's the complete program that ties everything together:

```python
def main() -> None:
    """
    Main program: read CSV file and report results.
    """
    filename = "users.csv"

    try:
        results = parse_csv_file(filename)

        # Print summary report
        print("\n" + "="*50)
        print("CSV PARSING SUMMARY")
        print("="*50)
        print(f"Total rows processed: {results['total']}")
        print(f"Valid records: {len(results['valid'])}")
        print(f"Invalid records: {len(results['invalid'])}")

        if results['valid']:
            print("\nValid Records:")
            for record in results['valid']:
                print(f"  - {record['name']} ({record['age']}) {record['email']}")

        if results['invalid']:
            print("\nErrors Encountered:")
            for error in results['invalid']:
                print(f"  - {error}")

        print("="*50 + "\n")

    except FileNotFoundError as e:
        # File doesn't exist—this stops the whole program
        print(f"ERROR: {e}")
        print("Stopping. Please check the file path and try again.")
    except PermissionError as e:
        # Can't read file—this also stops the program
        print(f"ERROR: {e}")
        print("Stopping. Please check file permissions and try again.")
    except Exception as e:
        # Unexpected error—report and stop
        print(f"UNEXPECTED ERROR: {e}")
        print("The program encountered an unexpected problem. Please report this error.")


if __name__ == "__main__":
    main()
```

#### ✨ Teaching Tip

> Use Claude Code to test your parser with different inputs. Ask: "Create sample CSV files with valid data, malformed data, permission errors, and missing files. Test my parser against each."

Notice the error-handling hierarchy:
- **Top level** (`main()`) catches fatal errors (file access)
- **Middle level** (`parse_csv_file()`) processes data and logs row errors
- **Bottom level** (validation functions) validate individual fields

## Testing Your Parser

Professional programmers test error handling as rigorously as happy-path scenarios. Let's build test data for each error case.

### Test Case 1: Valid Data

Create a file called `users.csv`:
```
name,age,email
Alice,28,alice@example.com
Bob,35,bob@example.com
Charlie,45,charlie@example.com
```

**Expected output**:
```
Total rows processed: 3
Valid records: 3
Invalid records: 0
```

### Test Case 2: Missing File

Try running with a non-existent file:
```python
# Change filename in main()
filename = "nonexistent.csv"
```

**Expected error**:
```
ERROR: File not found: nonexistent.csv. Check the file path and try again.
Stopping. Please check the file path and try again.
```

### Test Case 3: Malformed Data

Create `users_bad_data.csv`:
```
name,age,email
Alice,twenty-eight,alice@example.com
Bob,,bob@example.com
Charlie,150,charlie-at-example.com
Dave,35,dave@example.com
```

**Expected output**:
```
Total rows processed: 4
Valid records: 1
Invalid records: 3
Errors Encountered:
  - Row 2: Invalid age: twenty-eight
  - Row 3: Age must be 0-150, got
  - Row 4: Email must contain '@', got: charlie-at-example.com
```

### Test Case 4: File Permissions (Advanced)

On Linux/Mac, you can test permissions:
```bash
# Create a file
echo "name,age,email" > restricted.csv

# Remove read permissions
chmod 000 restricted.csv

# Run your parser (will get PermissionError)
# Restore permissions when done
chmod 644 restricted.csv
```

**Expected error**:
```
ERROR: Permission denied reading restricted.csv. Check file permissions.
Stopping. Please check file permissions and try again.
```

## Debugging Common Issues

When your parser doesn't work as expected, use this checklist:

**Issue**: Parser crashes with uncaught exception
- **Check**: Did you add try/except around the file open?
- **Fix**: Wrap file operations in try/except for FileNotFoundError and PermissionError

**Issue**: Valid records missing even though validation logic looks correct
- **Check**: Are you raising exceptions in validation functions when rules fail?
- **Fix**: Use `raise ValueError()` to signal validation failures

**Issue**: Error messages aren't helpful
- **Check**: Are you including context (row number, field name, actual value)?
- **Fix**: Build error messages like `f"Row {row_num}: Invalid age: {age_str}"`

**Issue**: Parser stops on first error instead of continuing
- **Check**: Where are the try/except blocks? Is row validation inside the loop?
- **Fix**: Wrap individual row processing in try/except; only let fatal errors escape

## Extending Your Parser

Once your basic parser works, here are ways to extend it:

**Extension 1: Support Multiple File Formats**
- Read JSON files instead of (or in addition to) CSV
- Detect format by file extension
- Create separate validation for each format

**Extension 2: Implement Retry Logic**
- If file is temporarily locked, retry 3 times before giving up
- Add exponential backoff between retries

**Extension 3: Advanced Error Recovery**
- For invalid age, suggest valid range in error message
- For invalid email, suggest what valid email looks like
- Let user correct errors and re-validate

**Extension 4: Logging and Audit Trail**
- Write all validation errors to a log file
- Include timestamp for each error
- Create audit trail of what was processed

**Challenge Exercise**: Pick one extension above and implement it. Use Claude Code to help design the additional exception handling needed.

---

## Try With AI: The Robust CSV Parser Challenge

### Part 1: Design Error Handling Architecture (Your Turn First)

**Before asking AI**, analyze the CSV parser specification and predict all error scenarios:

**Given specification**:
- Read CSV file with name, age, email columns
- Validate: name (non-empty), age (0-150 integer), email (contains '@')
- Handle: FileNotFoundError, PermissionError, ValueError

**Your architecture task**: Create `error_scenarios.md` documenting:

1. **Fatal errors** (stop entire program):
   - What errors fall here? (file not found, permission denied, ...?)
   - Where in code do these occur? (before loop, during loop, after loop?)
   - What should user see when each happens?

2. **Recoverable errors** (skip row, continue):
   - What errors fall here? (bad age, invalid email, ...?)
   - How do you track which rows failed?
   - What information should error messages contain?

3. **Edge cases**:
   - What if file exists but is empty?
   - What if CSV has wrong number of columns?
   - What if age is empty string `""`?
   - What if row has trailing commas?

**Your prediction**: For each error type above, write:
- Which exception type will Python raise?
- Where will you catch it? (outer try/except or inner loop try/except?)
- What happens after you catch it? (stop program, skip row, use default?)

---

### Part 2: AI Explains Error Handling Layers (Discovery)

Share your architecture with AI:

> "I designed error handling for a CSV parser. Here's my architecture:
>
> [paste your error_scenarios.md]
>
> Questions:
> 1. You categorized errors as 'fatal' vs 'recoverable'. What's the rule for deciding which category an error belongs to?
> 2. I said FileNotFoundError should stop the program. But what if I want to retry with a different filename? How would the architecture change?
> 3. For recoverable errors (bad age), I'll skip the row. But should I track error details? What information is useful for debugging?
> 4. I listed edge cases (empty file, wrong columns). Are these ValueError, or different exception types? How do I handle each?"

**Your evaluation task**:
- Can you explain why file errors (FileNotFoundError) should be caught OUTSIDE the row loop, but validation errors (ValueError) should be caught INSIDE?
- What's the difference between catching an exception and re-raising it with `raise`?

---

### Part 3: Student Teaches AI (Test-Driven Error Handling)

Challenge AI by writing test cases BEFORE implementing the parser:

> "I'm writing tests for my CSV parser before implementing it. Here are my test cases:
>
> **Test 1: Valid data**
> ```
> name,age,email
> Alice,28,alice@example.com
> Bob,35,bob@example.com
> ```
> Expected: 2 valid records, 0 errors
>
> **Test 2: Mixed valid/invalid**
> ```
> name,age,email
> Alice,28,alice@example.com
> Bob,invalid,bob@example.com
> Charlie,35,charlie-at-example.com
> ```
> Expected: 1 valid record, 2 errors (bad age on row 2, bad email on row 3)
>
> **Test 3: Fatal error**
> File: `nonexistent.csv`
> Expected: FileNotFoundError with message showing filename
>
> **For EACH test**:
> 1. What assertions should I write? (what to check for success/failure?)
> 2. Show me pytest code that verifies my parser behaves correctly
> 3. What if my parser raises an exception instead of returning a result dict? How do I test that?
> 4. How do I test that error messages contain useful information (row number, field name, bad value)?"

**Your implementation task**:
- Write the 3 pytest test functions AI suggested
- Run them (they'll fail—parser doesn't exist yet)
- Now implement the parser to make tests pass
- Which test was hardest to satisfy? Why?

---

### Part 4: Build Production-Ready CSV Parser (Convergence)

Create a complete, robust CSV parser with AI:

> "Generate a production-ready CSV parser with comprehensive error handling. Include:
>
> **Validation functions** (3 functions):
> - `validate_name(name: str) -> str` — raises ValueError if empty
> - `validate_age(age_str: str) -> int` — raises ValueError if not 0-150
> - `validate_email(email: str) -> str` — raises ValueError if no '@'
>
> **Parser function**:
> - `parse_csv_file(filename: str) -> dict`
> - Returns: `{'valid': [...], 'invalid': [...], 'total': int}`
> - Handles: FileNotFoundError, PermissionError, ValueError
> - Logs each error with row number and details
>
> **Main function**:
> - Calls parser
> - Prints summary report (total rows, valid count, invalid count)
> - Shows all validation errors with context
>
> For EACH component:
> - Type hints on all functions
> - Docstrings with Raises section
> - Example usage in docstring
> - Error messages that help user fix the problem"

**Refinement**:
> "Now extend the parser with these production features:
> 1. **Retry logic**: If file is locked (PermissionError), retry 3 times with 1-second delays
> 2. **Logging**: Write all errors to `parser.log` with timestamps
> 3. **Graceful degradation**: If validation fails on ALL rows, still return summary (don't crash)
> 4. **Multiple file formats**: Support both CSV and JSON input
>
> Show me the modified code and how each feature integrates with existing error handling."

---

### Part 5: Reflect on Chapter 21 Concepts (Integration)

> "Let's reflect on the CSV parser project and all of Chapter 21:
>
> **Exception Fundamentals (Lesson 1)**:
> - Which exception types did your parser encounter? (FileNotFoundError, ValueError, PermissionError)
> - How did try/except prevent crashes? Give specific example from your code.
> - Did you use exception chaining (`raise ... from e`)? Where and why?
>
> **Except-Else-Finally (Lesson 2)**:
> - Did you use `else` block anywhere? (runs when try succeeds)
> - Did you use `finally` for cleanup? (close file, release resources)
> - How did flow control work in your nested try/except blocks?
>
> **Custom Exceptions (Lesson 3)**:
> - Did you create custom exceptions (CSVParsingError, ValidationError)?
> - If not, where COULD custom exceptions improve your parser?
> - Show me a custom exception hierarchy for CSV parsing errors
>
> **Error Strategies (Lesson 4)**:
> - Which strategies did you apply? (retry, fallback, degradation, logging)
> - For FileNotFoundError, you chose fail-fast. Why not retry or fallback?
> - For ValueError (bad age), you chose skip-row. Why not fail-fast?
> - What would change if this parser ran in production handling 1M rows?
>
> **Integration**:
> - What was hardest about combining all these concepts?
> - If you could redesign error handling from scratch, what would you change?
> - What error handling pattern will you reuse in future projects?
>
> Write `chapter_21_reflection.md` documenting insights and reusable patterns."

---

**Time**: 55-65 minutes
**Outcome**: You'll have a production-grade CSV parser demonstrating all Chapter 21 concepts (try/except/else/finally, custom exceptions, error strategies, logging), comprehensive test coverage, and clear understanding of when to apply each error handling pattern. The reflection integrates all 5 lessons into cohesive understanding.

---

### Safety and Ethics Note

When working with file operations and user data:
- **Never expose file paths or system errors** to end users in production (log them, but show friendly messages)
- **Validate input data carefully**—malformed CSV files can cause issues if not handled properly
- **Consider data privacy**—if CSV contains sensitive information, handle it carefully and don't expose details in error messages

### Next Steps

You've now completed all five lessons on exception handling. You've learned to:
1. Understand what exceptions are and how try/except prevents crashes
2. Use multiple except blocks, else, and finally for sophisticated error handling
3. Write functions that raise custom exceptions for domain-specific errors
4. Apply error handling strategies (retry, fallback, graceful degradation, logging)
5. Build a realistic project integrating all these concepts

From here, Chapter 22 (IO & File Handling) builds directly on this foundation—exception handling is the primary tool for safe file operations. You're ready for that next step.
