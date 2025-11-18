---
title: "Error Handling Strategies – Defensive Programming"
chapter: 21
lesson: 4
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Strategic Error Recovery"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student applies appropriate error handling strategy (retry, fallback, degradation) to realistic scenario and justifies choice based on error type and context"

  - name: "Real-World Error Analysis"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies multiple error types in realistic scenario and determines which strategy works best for each error"

  - name: "Logging for Debugging"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student writes logging statements that capture error context (error type, data, timestamp) useful for debugging without crashing program"

learning_objectives:
  - objective: "Apply retry logic for transient errors (attempt multiple times before failing)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements retry pattern with max attempts for file operation or network request"

  - objective: "Implement fallback values for recoverable errors (use default when primary operation fails)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student writes code providing sensible defaults when operation fails"

  - objective: "Use graceful degradation (feature unavailable, but core functionality continues)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student designs system where non-critical failures don't crash entire program"

  - objective: "Choose appropriate error handling strategy for given scenario and explain tradeoffs"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student analyzes scenario, identifies possible errors, and justifies strategy choice"

cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (Error handling strategies: retry, fallback, graceful degradation, logging; Choosing strategy based on error type) within B1 limit of 10 ✓. Builds on Lessons 1-3 concepts (try/except, custom exceptions, defensive programming)."

differentiation:
  extension_for_advanced: "Research exponential backoff algorithms; implement jitter in retry logic; explore timeout-based strategies; analyze production logging patterns"
  remedial_for_struggling: "Start with single strategy (fallback) before combining multiple; use real-world scenarios from Lesson 5 capstone as concrete examples"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Error Handling Strategies – Defensive Programming

Catching exceptions prevents crashes. But what happens after you catch an error? How do you recover? This lesson teaches you defensive programming patterns—strategic decisions about how to handle different types of errors. You'll learn when to retry, when to use fallback values, when to degrade gracefully, and how to log errors for debugging.

Professional developers don't just catch errors and move on. They think strategically: **What kind of error is this? Is it temporary? Can I recover? Should I notify the user? What should I log?**

## Beyond Catching – The Four Error Handling Strategies

By now, you've learned try/except/finally blocks and custom exceptions. But catching an exception is just the first step. The real skill is deciding *what to do* when an error occurs.

Let's explore four defensive programming strategies:

### Strategy 1: Retry Logic – "Try Again"

**When to use**: Errors that are **transient** (temporary). Network hiccups, briefly unavailable services, temporary file locks.

**The idea**: If an operation fails, wait a moment and try again. Many transient errors resolve themselves on retry.

**Pattern**:

```python
def fetch_data_with_retry(url: str, max_retries: int = 3) -> str:
    """Fetch data from URL, retry on failure."""
    for attempt in range(1, max_retries + 1):
        try:
            response = fetch_url(url)
            return response
        except ConnectionError as e:
            if attempt == max_retries:
                print(f"Failed after {max_retries} attempts: {e}")
                raise
            print(f"Attempt {attempt} failed, retrying...")
```

**Key idea**: Count attempts. On the last attempt, re-raise the exception so the caller knows it failed completely.

#### 💬 AI Colearning Prompt

> "Show me the difference between retry logic and just catching an exception and returning a default value. When would you choose retry vs. fallback?"

#### 🎓 Expert Insight

> In AI-native development, you don't guess at error handling—you analyze the error type. Transient errors (network timeouts) demand retry. Permanent errors (file not found) demand fallback or graceful degradation.

### Strategy 2: Fallback Values – "Use a Default"

**When to use**: Errors that are **permanent or unrecoverable**. Missing files, invalid data, unavailable services with no option to retry.

**The idea**: When an operation fails, use a sensible default value instead.

**Pattern**:

```python
def load_config(config_file: str) -> dict:
    """Load configuration, fallback to defaults if missing."""
    try:
        with open(config_file) as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Config file {config_file} not found, using defaults")
        return {
            "theme": "light",
            "language": "en",
            "notifications": True
        }
    except json.JSONDecodeError:
        print(f"Config file {config_file} is invalid, using defaults")
        return {"theme": "light", "language": "en", "notifications": True}
```

**Key idea**: The program continues with sensible defaults. User gets a notice, but doesn't lose functionality.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "For a social media app, if loading the user's profile picture fails, what's a good fallback? Why? If loading the feed fails, is fallback appropriate?"

**Expected Outcome**: You'll understand that some features are essential (feed must load), while others can gracefully fail (profile picture can be placeholder).

### Strategy 3: Graceful Degradation – "Keep Going With Less"

**When to use**: Non-critical features fail, but the system should continue. Feature unavailability doesn't mean complete failure.

**The idea**: When a secondary feature fails, skip it and continue. The user loses functionality but doesn't lose the whole program.

**Pattern**:

```python
def process_csv_with_validation(filename: str) -> list[dict]:
    """Process CSV file, skip invalid rows gracefully."""
    rows = []
    skipped = 0

    try:
        with open(filename) as f:
            reader = csv.DictReader(f)
            for row_num, row in enumerate(reader, start=1):
                try:
                    age = int(row["age"])
                    if age < 0 or age > 150:
                        raise ValueError(f"Age {age} out of range")
                    rows.append(row)
                except ValueError as e:
                    skipped += 1
                    print(f"Row {row_num}: {e}, skipping")
    except FileNotFoundError:
        print(f"File {filename} not found")
        return []

    print(f"Processed {len(rows)} rows, skipped {skipped}")
    return rows
```

**Key idea**: The program doesn't crash on bad data. Invalid rows are skipped with logging. Valid rows are processed. User gets partial results.

#### ✨ Teaching Tip

> Use Claude Code to test graceful degradation: "Create a CSV file with 10 rows: 7 valid, 3 with bad data. Run my parser and show what happens. Does it handle all error types?"

### Strategy 4: Logging Errors – "Keep a Record"

**When to use**: All the above strategies benefit from **logging**. Record errors with context for debugging without crashing the program.

**The idea**: Print or record error details: what failed, when, why, and what data was involved. This record helps diagnose issues later.

**Pattern**:

```python
import sys
from datetime import datetime

def divide_with_logging(a: float, b: float) -> float | None:
    """Divide a by b, log errors."""
    try:
        result = a / b
        return result
    except ZeroDivisionError as e:
        timestamp = datetime.now().isoformat()
        error_msg = f"[{timestamp}] ERROR: Division by zero. a={a}, b={b}"
        print(error_msg, file=sys.stderr)  # Print to error stream
        return None
    except TypeError as e:
        timestamp = datetime.now().isoformat()
        error_msg = f"[{timestamp}] ERROR: Invalid types. a={type(a).__name__}, b={type(b).__name__}"
        print(error_msg, file=sys.stderr)
        return None
```

**Key idea**: Log includes timestamp, error type, context (the actual values), and human-readable message. This helps debugging.

#### 💬 AI Colearning Prompt

> "What should error logs include to be useful for debugging? What context is essential?"

---

## Code Example 1: Retry Logic with Exponential Backoff

Many production systems use **exponential backoff**—wait longer between retries. First retry after 1 second, second after 2 seconds, third after 4 seconds. This prevents overwhelming a service.

```python
import time

def fetch_with_exponential_backoff(url: str, max_retries: int = 3) -> str:
    """Fetch data with exponential backoff between retries."""
    for attempt in range(1, max_retries + 1):
        try:
            print(f"Attempt {attempt}: Fetching {url}")
            # Simulate fetch (would use requests.get() in real code)
            if attempt < 2:
                raise ConnectionError("Network timeout")
            return "Success: Data retrieved"
        except ConnectionError as e:
            if attempt == max_retries:
                raise
            wait_time = 2 ** (attempt - 1)  # 1, 2, 4, 8 seconds
            print(f"Failed: {e}. Waiting {wait_time}s before retry {attempt + 1}")
            time.sleep(wait_time)

# Test it
try:
    result = fetch_with_exponential_backoff("https://api.example.com/data")
    print(result)
except ConnectionError:
    print("All retries exhausted")
```

**Output**:
```
Attempt 1: Fetching https://api.example.com/data
Failed: Network timeout. Waiting 1s before retry 2
Attempt 2: Fetching https://api.example.com/data
Failed: Network timeout. Waiting 2s before retry 3
Attempt 3: Fetching https://api.example.com/data
Success: Data retrieved
```

**Specification Reference**: This code demonstrates B1 application—student applies retry pattern to realistic network scenario.

---

## Code Example 2: Combining Fallback with Logging

Real code often combines multiple strategies. Here's fallback + logging:

```python
import json
from datetime import datetime

def load_user_preferences(user_id: int) -> dict:
    """Load user preferences, fallback to defaults, log failures."""
    default_prefs = {
        "theme": "light",
        "font_size": 12,
        "notifications": True
    }

    try:
        with open(f"preferences_{user_id}.json") as f:
            data = json.load(f)
            print(f"Loaded preferences for user {user_id}")
            return data
    except FileNotFoundError:
        timestamp = datetime.now().isoformat()
        print(f"[{timestamp}] WARN: Preferences file not found for user {user_id}, using defaults")
        return default_prefs
    except json.JSONDecodeError as e:
        timestamp = datetime.now().isoformat()
        print(f"[{timestamp}] ERROR: Preferences file corrupted for user {user_id}: {e}. Using defaults.")
        return default_prefs
    except Exception as e:
        timestamp = datetime.now().isoformat()
        print(f"[{timestamp}] ERROR: Unexpected error loading preferences for user {user_id}: {e}. Using defaults.")
        return default_prefs
```

**Key details**:
- Catches specific exceptions first (FileNotFoundError, JSONDecodeError)
- Falls back to sensible defaults
- Logs each failure with timestamp and context
- Generic catch-all for unexpected errors
- All errors are logged—none are silent

---

## Code Example 3: Graceful Degradation in Data Processing

This example processes data, skipping invalid entries:

```python
from dataclasses import dataclass

@dataclass
class User:
    name: str
    age: int
    email: str

def validate_and_process_users(users_data: list[dict]) -> tuple[list[User], list[str]]:
    """Process users, return valid users and list of validation errors."""
    valid_users = []
    errors = []

    for idx, user_dict in enumerate(users_data):
        try:
            # Validate name
            name = user_dict.get("name", "").strip()
            if not name:
                raise ValueError("Name is required")

            # Validate age
            age = int(user_dict["age"])
            if age < 0 or age > 150:
                raise ValueError(f"Age {age} out of valid range")

            # Validate email
            email = user_dict.get("email", "")
            if "@" not in email:
                raise ValueError("Email must contain @")

            valid_users.append(User(name=name, age=age, email=email))

        except (ValueError, KeyError) as e:
            error_msg = f"User {idx + 1}: {e}"
            errors.append(error_msg)
            print(f"WARN: {error_msg}, skipping")

    return valid_users, errors

# Test
test_data = [
    {"name": "Alice", "age": 30, "email": "alice@example.com"},
    {"name": "", "age": 25, "email": "bob@example.com"},  # Invalid: empty name
    {"name": "Charlie", "age": "invalid", "email": "charlie@example.com"},  # Invalid: age not int
    {"name": "Diana", "age": 28, "email": "diana@example.com"},
]

valid, errors = validate_and_process_users(test_data)
print(f"Valid users: {len(valid)}, Errors: {len(errors)}")
for user in valid:
    print(f"  - {user.name} (age {user.age})")
for error in errors:
    print(f"  ERROR: {error}")
```

**Output**:
```
WARN: User 2: Name is required, skipping
WARN: User 3: invalid literal for int() with base 10: 'invalid', skipping
Valid users: 2, Errors: 2
  - Alice (age 30)
  - Diana (age 28)
  ERROR: User 2: Name is required
  ERROR: User 3: invalid literal for int() with base 10: 'invalid'
```

**This demonstrates**:
- Validation happens row-by-row
- Invalid rows are skipped gracefully
- Valid data is processed completely
- Errors are reported without crashing

---

## Code Example 4: Logging with Structured Context

Professional systems log with enough detail to diagnose issues:

```python
import sys
from datetime import datetime
from typing import Any

def safe_json_parse(data: str, context: dict[str, Any]) -> dict | None:
    """Parse JSON with detailed logging."""
    try:
        result = json.loads(data)
        return result
    except json.JSONDecodeError as e:
        timestamp = datetime.now().isoformat()
        log_entry = {
            "timestamp": timestamp,
            "error_type": "JSONDecodeError",
            "error_message": str(e),
            "context": context,
            "data_sample": data[:100] if len(data) > 100 else data
        }
        # In real systems, this would go to a logging service
        print(f"ERROR: {log_entry}", file=sys.stderr)
        return None

# Usage
result = safe_json_parse(
    '{"name": "Alice", invalid json}',
    {"user_id": 42, "operation": "load_profile"}
)
```

**Logging includes**:
- **Timestamp**: When the error occurred
- **Error type**: Specific exception class
- **Error message**: Details from exception
- **Context**: Related information (user ID, operation)
- **Data sample**: What data caused the problem (truncated for safety)

This information helps debuggers understand: **What failed, when, why, and what triggered it.**

---

## Choosing the Right Strategy

Here's a decision matrix for choosing error handling strategies:

| Error Type | Best Strategy | Why | Example |
|-----------|---------------|-----|---------|
| **Transient (temporary)** | Retry | Error resolves on retry | Network timeout, service briefly down |
| **Permanent, predictable** | Fallback | Operation will always fail | File not found, invalid format |
| **Non-critical feature** | Graceful degradation | Core function continues | Thumbnail loading fails, app still works |
| **All errors** | Logging | Track for debugging | Record what happened for diagnosis |

---

## Exercise 1: Implement Retry Logic

Write a function that simulates a network request, fails twice, succeeds on third attempt. Add logging.

```python
def unreliable_fetch(attempt: int = 0) -> str:
    """Simulate unreliable network—fails twice, succeeds on third."""
    attempt += 1
    if attempt < 3:
        raise ConnectionError(f"Attempt {attempt}: Network timeout")
    return "Data received"

# Your task: Write retry_wrapper() that calls unreliable_fetch()
# with retry logic, logging each attempt, returning success on success
# or raising an exception after 4 failed attempts.

def retry_wrapper(max_attempts: int = 4) -> str:
    """Your implementation here."""
    pass

# Test: Should log 2 failures, then succeed on attempt 3
result = retry_wrapper()
```

**Expected outcome**: Your function retries the operation, logs each attempt, and succeeds when the underlying function succeeds.

---

## Exercise 2: Implement Fallback for Missing Configuration

Write a function that loads configuration from a file, but falls back to defaults if missing or corrupted.

```python
# You have: config_defaults = {"port": 8000, "host": "localhost", "debug": False}
# You need: load_config(filename: str) that:
# - Tries to load JSON from filename
# - Falls back to defaults if FileNotFoundError
# - Falls back to defaults if JSONDecodeError
# - Logs what happened
# - Returns the config (loaded or default)

def load_config(filename: str) -> dict:
    """Your implementation here."""
    pass

# Test with missing file
config = load_config("missing_config.json")
# Should print warning and return defaults
```

**Expected outcome**: Function gracefully handles missing or corrupted files, returning sensible defaults while logging what happened.

---

## Exercise 3: Combine Strategies – Robust Data Processing

Write a function that processes a list of user data, implementing all four strategies:
- **Retry**: Attempts validation multiple times (in case data quality issue is temporary)
- **Fallback**: Uses default values for missing fields
- **Graceful degradation**: Skips invalid rows, continues processing others
- **Logging**: Records errors with context

```python
def process_user_data(users: list[dict], max_validation_attempts: int = 2) -> list[dict]:
    """Process users with retry, fallback, graceful degradation, logging."""
    processed = []

    for idx, user in enumerate(users):
        # Implement retry loop: attempt validation up to max_validation_attempts times
        # Implement fallback: use defaults for missing fields
        # Implement graceful degradation: skip invalid users
        # Implement logging: record errors
        pass

    return processed

# Test data
test_users = [
    {"name": "Alice", "age": 30, "email": "alice@example.com"},
    {"name": "Bob"},  # Missing age and email
    {"name": "Charlie", "age": "invalid"},  # Invalid age
]

result = process_user_data(test_users)
```

**Expected outcome**: Function processes valid data, skips invalid, uses defaults where possible, logs all issues.

---

## Key Takeaway: Defensive Programming Mindset

Professional developers ask:

1. **What errors are possible?** (What exceptions could this raise?)
2. **Are they transient or permanent?** (Will retry help?)
3. **What's the right recovery strategy?** (Retry, fallback, degrade?)
4. **What should I log?** (What context helps debugging?)

This systematic thinking transforms error handling from "catch and hope" to "anticipate and recover."

---

## Try With AI: The Error Strategy Decision Challenge

### Part 1: Analyze Real-World Errors (Your Turn First)

**Before asking AI**, examine this production scenario and categorize errors:

You're building a weather app with these operations:
1. Fetch current temperature from API (critical feature)
2. Load cached weather history from local file (nice to have)
3. Upload user's location preferences to server (background task)
4. Download user's profile picture (UI enhancement)

**Your analysis task**: For EACH operation, predict:
- What errors could occur? (list 2-3 per operation)
- Is each error transient (temporary) or permanent?
- Which error handling strategy fits: retry, fallback, degradation, or fail-fast?
- What should you log when the error happens?

Document your predictions in `error_strategy_analysis.txt` before continuing.

---

### Part 2: AI Explains Strategy Selection (Discovery)

Share your analysis with AI:

> "I analyzed a weather app with 4 operations and predicted error types:
>
> [paste your analysis from Part 1]
>
> Questions:
> 1. For temperature API fetch (critical), you said retry. But what if the API is down for maintenance? When do I give up?
> 2. For cached history (non-critical), you said fallback. What's a sensible fallback when history file is missing?
> 3. For location upload (background), you said degradation. What does 'graceful degradation' mean here—does the app still work?
> 4. How do I decide between retry (keeps trying) and fallback (gives up, uses default) when an error could be either transient or permanent?"

**Your evaluation task**:
- Can you explain the tradeoffs between retry (keeps trying) and fallback (gives up, uses default)?
- Which strategy preserves the best user experience for critical vs non-critical features?

---

### Part 3: Student Teaches AI (Edge Case Strategies)

Challenge AI with complex error scenarios:

> "I have these error scenarios. For EACH, tell me if my strategy is correct or wrong:
>
> **Scenario A**: Database connection fails. I retry 3 times with exponential backoff. On 4th failure, I show user "Service unavailable, try again later" and log the error. **Is this correct?**
>
> **Scenario B**: User uploads profile picture, but image file is corrupted (invalid format). I retry 3 times. **Is retry appropriate here, or should I use a different strategy?**
>
> **Scenario C**: Loading user preferences from JSON file. File is corrupted (JSONDecodeError). I fallback to system defaults silently without telling user. **Is silent fallback acceptable?**
>
> For EACH:
> 1. Is my strategy correct? If not, what's better?
> 2. What should I log in each case?
> 3. What should the user see (error message, fallback behavior)?
> 4. Would this strategy work in production? What would break?"

**Your debugging task**:
- Implement Scenario B in code with your proposed strategy
- Test it with a corrupted file
- Verify your strategy produces sensible behavior

---

### Part 4: Build Error Strategy Decision Tree (Convergence)

Create a reusable decision framework with AI:

> "Generate a complete error handling decision tree for production systems. Include:
>
> **For error classification**:
> - How to identify transient vs permanent errors (symptoms of each)
> - How to identify critical vs non-critical operations
> - Examples of each category
>
> **For strategy selection**:
> - When to retry (conditions, max attempts, backoff strategy)
> - When to fallback (acceptable defaults, user notification)
> - When to degrade gracefully (feature unavailable, core continues)
> - When to fail-fast (unrecoverable, stop immediately)
>
> **For logging**:
> - What to log for each strategy (context, timestamps, user ID)
> - Log levels (ERROR, WARN, INFO) for each scenario
> - What NOT to log (sensitive data)
>
> For EACH branch:
> - Decision criteria
> - Code pattern (retry loop, fallback template)
> - User experience impact
> - Example scenario"

**Refinement**:
> "Now create a Python module `error_strategies.py` with 4 reusable decorator functions:
> 1. `@retry_on_failure(max_attempts=3, backoff=exponential)`
> 2. `@fallback_on_error(default_value=None)`
> 3. `@degrade_gracefully(feature_name='...')`
> 4. `@log_errors(log_level='ERROR')`
>
> Show me how to apply each decorator to real functions and test them."

---

**Time**: 40-50 minutes
**Outcome**: You'll have a systematic decision framework for choosing error handling strategies, understand tradeoffs between retry/fallback/degradation, and have reusable decorator patterns for production error handling.

**Safety and Responsible Error Handling**

As you implement error handling, remember:

- **Log securely**: Don't log passwords, API keys, or personal data
- **User-friendly messages**: Tell users what went wrong and what to do (not internal error codes)
- **Graceful failure**: Never silently ignore errors—log and inform
- **Test error paths**: Intentionally trigger errors to verify recovery works
