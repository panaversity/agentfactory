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
generated_by: "lesson-writer v3.0.0"
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

#### 🎓 Instructor Commentary

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

## Try With AI

Now let's apply everything you've learned by building and testing your CSV parser with AI guidance.

### Using Claude Code or Gemini CLI

**Setup**: Open Claude Code (or your preferred AI companion) and prepare to implement the CSV parser interactively.

### Prompt Set (Progressive Scope)

**Prompt 1 (Understand Specification)**:
> "I'm building a CSV parser that validates user data and handles errors gracefully. The file contains name, age, email. I need to catch FileNotFoundError, ValueError (for bad age), and report bad rows without crashing. Show me the structure of a solution (not full code yet, just the outline and error handling strategy)."

**Expected Output**: AI should describe:
- File opening with error handling for file not found
- Loop processing each row
- Try/except around validation
- Summary reporting

---

**Prompt 2 (Implement Core Logic)**:
> "Now write the validation functions for age (must be 0-150 integer), email (must contain '@'), and name (non-empty). Each should raise ValueError with a helpful message if invalid. Use type hints and docstrings."

**Expected Output**: Three functions with proper error handling, type hints, and docstrings. Validate the functions work by asking: "Show me test code that calls each function with both valid and invalid inputs."

---

**Prompt 3 (Test Error Handling)**:
> "I'm testing my parser. Create 3 test CSV files: one with valid data, one with bad age values, one with invalid emails. Then write test code that runs my parser against each file and shows the results."

**Expected Output**: Sample CSV files and test script demonstrating your parser works correctly for valid data and gracefully handles invalid data without crashing.

---

**Prompt 4 (Debug and Validate)**:
> "Test my parser with these edge cases: empty file, file with only header, file where age is an empty string, email without '@'. Does it handle all of these without crashing? What improvements would make it more robust?"

**Expected Output**: AI identifies missing edge cases and suggests improvements like:
- Checking for empty lines
- Validating CSV format (correct number of fields)
- Better error messages including row numbers
- Handling files with unusual line endings (Windows vs Unix)

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
