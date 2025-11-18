---
sidebar_position: 3
title: "Lesson 3: Function Parameters and Returns"
description: "Master positional parameters, default parameters, multiple returns, and keyword arguments"
keywords: ["parameters", "default parameters", "keyword arguments", "unpacking", "multiple returns", "tuples"]
cefr_level: "B1"
estimated_time: "60 minutes"
proficiency_by_end: "Students correctly call functions with positional and keyword arguments, and unpack multiple return values"
prerequisites: ["Lesson 1-2: Function basics and type hints", "Chapter 18: Tuples (used for multiple returns)", "Chapter 14: Type hints with generics"]
learning_objectives:
  - "Distinguish between required and optional parameters in function signatures"
  - "Call functions using positional arguments, keyword arguments, and mixed patterns"
  - "Write functions that return multiple values as tuples"
  - "Unpack multiple return values into separate variables"
  - "Understand parameter order constraints in Python"
ai_native_learning: true
try_with_ai_prompts: 4
colearning_elements: ["conceptual_prompt", "teaching_commentary", "specification_challenge", "ai_tip"]
---

## Lesson 3: Function Parameters and Returns

**CEFR Level**: B1 (Intermediate Application)
**Time Estimate**: 60 minutes
**What You'll Learn**: Real functions have optional behavior. You'll learn to write functions where some parameters are required and others are optional with defaults, and how to call functions flexibly using positional and keyword arguments. You'll also master returning and unpacking multiple values.

---

## Required vs. Optional Parameters — Flexibility

**Pattern**: Functions often have some parameters you must provide and others that are optional with sensible defaults.

### 💻 Code Idea: Parameters with Defaults

```python
def create_user_account(username: str, email: str, role: str = "user") -> dict[str, str]:
    """
    Create a user account with optional role assignment.

    Parameters:
        username (str): User's login name (required)
        email (str): User's email address (required)
        role (str): User's role in system (default: "user")

    Returns:
        dict[str, str]: Account information
    """
    account: dict[str, str] = {
        "username": username,
        "email": email,
        "role": role,
        "status": "active"
    }
    return account

# Call with just required parameters (uses default role)
user1 = create_user_account("alice", "alice@example.com")
print(user1)
# {'username': 'alice', 'email': 'alice@example.com', 'role': 'user', 'status': 'active'}

# Call with optional parameter
user2 = create_user_account("bob", "bob@example.com", "admin")
print(user2)
# {'username': 'bob', 'email': 'bob@example.com', 'role': 'admin', 'status': 'active'}
```

**Key Observation**: The parameter `role: str = "user"` means it's optional. If you don't provide it, Python uses `"user"` by default.

### 💬 AI Colearning Prompt

> Ask your AI: "Show me 3 different ways to call this function with these parameters: `def order_pizza(size: str, toppings: str = "cheese", delivery: bool = True)`. Which calls are valid? What happens if I use keyword arguments?"

**Expected Understanding**: You see that there are multiple valid ways to call the same function, and keyword arguments provide clarity.

### 🎓 Instructor Commentary

Parameter order is not arbitrary—it's a language rule with a reason. Required parameters must come first because Python needs to know which arguments are required before it can apply defaults. Understanding WHY rules exist (not just memorizing them) is the key to professional development. When you design functions, you're making a contract: "Here's what you must provide, and here's what's optional." Clear parameter design makes your code readable and your intent unmistakable.

---

## Parameter Order Rules — Required First, Optional Second

**Rule**: In Python, parameters with default values (optional) must come AFTER parameters without defaults (required).

### 💻 Code Idea: Correct Parameter Order

```python
# CORRECT: Required parameters first, then optional
def send_message(recipient: str, subject: str, priority: str = "normal") -> str:
    """Send a message with optional priority level."""
    return f"Message to {recipient}: {subject} (Priority: {priority})"

# INCORRECT: This won't work in Python
# def send_message(priority: str = "normal", recipient: str, subject: str):
#     # Error! Required parameters can't come after optional ones

# Valid calls with default:
msg1 = send_message("alice@example.com", "Hello")
print(msg1)  # Message to alice@example.com: Hello (Priority: normal)

# Valid call with optional parameter:
msg2 = send_message("bob@example.com", "Urgent", "high")
print(msg2)  # Message to bob@example.com: Urgent (Priority: high)
```

**Why this rule exists**: Python processes arguments left to right. If an optional parameter came before a required one, Python wouldn't know which argument maps to which parameter.

---

## Keyword Arguments — Clarity Through Naming

**Pattern**: You can call functions using keyword arguments, where you specify parameter names explicitly. This makes code clearer, especially with many optional parameters.

### 💻 Code Idea: Using Keyword Arguments

```python
def schedule_meeting(
    title: str,
    date: str,
    time: str = "9:00 AM",
    duration_minutes: int = 60,
    online: bool = True
) -> str:
    """Schedule a meeting with many optional parameters."""
    meeting_info = f"Meeting: {title}\nDate: {date}\nTime: {time}\nDuration: {duration_minutes} min\nOnline: {online}"
    return meeting_info

# Positional arguments (order matters):
meeting1 = schedule_meeting("Budget Review", "2025-11-15")
print(meeting1)

# Keyword arguments (order doesn't matter, much clearer):
meeting2 = schedule_meeting(
    title="Quarterly Planning",
    date="2025-11-20",
    time="2:00 PM",
    duration_minutes=90,
    online=False  # In-person meeting
)
print(meeting2)

# Mixed: positional first, then keyword
meeting3 = schedule_meeting("Team Standup", "2025-11-18", time="10:00 AM")
print(meeting3)
```

**Advantage of Keyword Arguments**: When you see the function call, you immediately understand what each argument means. No guessing what the third argument is.

---

## Functions Returning Multiple Values

**Pattern**: When you need to return several pieces of information, return a tuple and unpack it.

### 💻 Code Idea: Multiple Returns with Unpacking

```python
def analyze_numbers(numbers: list[int]) -> tuple[int, int, float]:
    """
    Analyze a list of numbers.

    Returns:
        tuple[int, int, float]: (minimum, maximum, average)
    """
    min_val: int = min(numbers)
    max_val: int = max(numbers)
    avg_val: float = sum(numbers) / len(numbers)
    return min_val, max_val, avg_val

# Calling and unpacking return values
data: list[int] = [10, 25, 15, 30, 20]
minimum, maximum, average = analyze_numbers(data)

print(f"Min: {minimum}, Max: {maximum}, Avg: {average:.2f}")
# Output: Min: 10, Max: 30, Avg: 20.00

# Alternative: capture entire tuple
result: tuple[int, int, float] = analyze_numbers(data)
print(f"Result: {result}")  # (10, 30, 20.0)
```

**Type Hint Reading**: `tuple[int, int, float]` says the function returns a tuple with first element int, second element int, third element float. Unpacking assigns each to a variable.

---

## Functions with Optional Return Values (Union Types)

**Pattern**: Sometimes a function might not be able to return a value. Use `Type | None` to indicate this.

### 💻 Code Idea: Optional Return Values

```python
def parse_date(date_string: str) -> tuple[int, int, int] | None:
    """
    Parse a date string in format 'YYYY-MM-DD'.

    Returns:
        tuple[int, int, int] | None: (year, month, day) if valid, None if invalid
    """
    try:
        parts: list[str] = date_string.split('-')
        if len(parts) != 3:
            return None
        year, month, day = int(parts[0]), int(parts[1]), int(parts[2])
        return year, month, day
    except ValueError:
        return None

# Calling and handling optional return
date_result = parse_date("2025-11-15")
if date_result:
    year, month, day = date_result
    print(f"Parsed: {year}-{month:02d}-{day:02d}")
else:
    print("Invalid date format")

# Invalid input
invalid = parse_date("not-a-date")
print(f"Invalid result: {invalid}")  # None
```

**Type Hint Meaning**: `tuple[int, int, int] | None` means "return either a tuple of three ints, or None." This makes error cases explicit in the type system.

---

## Chaining Function Calls — Data Flow

**Pattern**: The return value of one function becomes the input to another. This is how real code works.

### 💻 Code Idea: Function Composition

```python
def calculate_discount(price: float, discount_percent: float) -> float:
    """Calculate discounted price."""
    return price * (1 - discount_percent / 100)

def add_tax(price: float, tax_rate: float) -> float:
    """Calculate price with tax."""
    return price * (1 + tax_rate / 100)

def format_price(price: float) -> str:
    """Format price as currency."""
    return f"${price:.2f}"

# Chain function calls: apply discount, then add tax, then format
original_price: float = 100.0
discounted: float = calculate_discount(original_price, 20)  # 20% off = $80
final_price: float = add_tax(discounted, 8)                 # 8% tax on $80 = $86.40
formatted: str = format_price(final_price)

print(f"Original: {format_price(original_price)}")
print(f"After 20% discount: {format_price(discounted)}")
print(f"After 8% tax: {formatted}")

# Or chain it in one expression:
result: str = format_price(add_tax(calculate_discount(100.0, 20), 8))
print(f"Final result: {result}")
```

**Key Insight**: The return value of one function flows into the next. Understanding this data flow is essential to reading and writing code.

---

## Common Parameter Patterns

### 💻 Code Idea: Various Parameter Combinations

```python
def build_url(domain: str, path: str = "/", protocol: str = "https", port: int = 443) -> str:
    """
    Build a complete URL from components.

    Parameters:
        domain (str): Domain name (required)
        path (str): URL path (default: "/")
        protocol (str): Protocol (default: "https")
        port (int): Port number (default: 443)

    Returns:
        str: Complete URL
    """
    return f"{protocol}://{domain}:{port}{path}"

# Various ways to call:
url1 = build_url("example.com")
print(url1)  # https://example.com:443/

url2 = build_url("example.com", "/api/users")
print(url2)  # https://example.com:443/api/users

url3 = build_url("example.com", protocol="http", port=8080, path="/dev")
print(url3)  # http://example.com:8080/dev

url4 = build_url("example.com", "/secure", port=8443)
print(url4)  # https://example.com:8443/secure
```

**Key Pattern**: Mix positional and keyword arguments strategically. Required parameters as positional, optional parameters as keyword arguments for clarity.

---

## 🚀 Specification Challenge

You have two functions you've written:

```python
def convert_celsius_to_fahrenheit(celsius: float) -> float:
    """Convert temperature from Celsius to Fahrenheit."""
    return (celsius * 9/5) + 32

def round_to_decimals(value: float, decimal_places: int = 2) -> float:
    """Round a number to specified decimal places."""
    return round(value, decimal_places)
```

Call `convert_celsius_to_fahrenheit`, then pass its result to `round_to_decimals`, all in one line. Ask your AI: *"Is this the correct way to do function composition? Are there better patterns?"* This teaches you how functions connect and flow data through your code.

---

## ✨ AI Tool Tip

When a function returns multiple values, you can unpack them in one line:

```python
x, y = get_coordinates()
name, email, age = get_user_info("alice")
```

This is very Pythonic and your AI will expect this pattern. Use it in your specifications and code. It's clean, readable, and makes your intent explicit.

---

## Try With AI: Parameter Pattern Detective

### Part 1: Decode Function Signatures (Your Turn First)

**Before asking AI**, analyze these three function signatures:

```python
# Function A
def send_email(recipient: str, subject: str, body: str = "", cc: list[str] = []) -> bool:
    pass

# Function B
def calculate_price(base: float, tax: float, discount: float = 0) -> float:
    pass

# Function C
def process_data(data: list[int], sort: bool = True, reverse: bool = False, limit: int | None = None) -> list[int]:
    pass
```

**Your analysis tasks**:
- For each function, list required vs. optional parameters
- Predict: Which function calls are VALID? `send_email("alice@test.com")` or `calculate_price(100.0)` or `process_data([1,2,3], limit=5)`?
- Spot the bug: What's dangerous about Function A's default parameter `cc: list[str] = []`?

Write your analysis before Part 2.

---

### Part 2: AI Guides Parameter Design (Discovery)

Now share with AI:

> "Here's my analysis of these function signatures: [paste your analysis]
>
> Validate my predictions:
> 1. Which function calls are valid/invalid? Show me the errors for invalid ones
> 2. Explain the mutable default argument bug in Function A - why is `cc: list[str] = []` dangerous?
> 3. Show me the CORRECT way to write Function A with a default empty list
> 4. For Function C, demonstrate calling it with: all positional args, all keyword args, and mixed style"

**Your evaluation task**:
- Test each valid call pattern AI suggests - do they work?
- Run the buggy Function A multiple times - do you see the mutable default problem?
- Compare the corrected version - how does `None` avoid the bug?

---

### Part 3: Student Teaches AI (Challenge Return Unpacking)

Challenge AI with unpacking edge cases:

> "Analyze this code:
> ```python
> def get_stats(numbers: list[int]) -> tuple[int, int, float]:
>     return min(numbers), max(numbers), sum(numbers)/len(numbers)
>
> data = [10, 20, 30]
> result = get_stats(data)  # Not unpacking!
> print(result[0])  # Accessing by index
> ```
>
> This works, but is it good style? Show me THREE different ways to handle the return value:
> 1. Full unpacking (min_val, max_val, avg = ...)
> 2. Partial unpacking with _ for unused values
> 3. Tuple indexing (current approach)
>
> For EACH approach, explain when it's appropriate and when it's bad practice."

**Your debugging task**:
- Write a function returning 4 values, but you only need the first and last
- Test AI's suggestion for partial unpacking with `_`
- Which approach makes your intent clearest?

---

### Part 4: Build a Flexible Configuration Function (Convergence)

Create a function demonstrating professional parameter design:

> "Write a function `create_database_connection()` that:
> 1. Required parameters: `host` (str), `database` (str)
> 2. Optional parameters: `port` (int, default 5432), `timeout` (int, default 30), `ssl` (bool, default True), `retries` (int, default 3)
> 3. Returns: `tuple[bool, str]` where bool is success/failure, str is connection string or error message
> 4. Include full type hints and docstring
> 5. Demonstrate calling it THREE ways: minimal args, all keyword args, mixed style
>
> Then add validation: raise ValueError if port < 1 or port > 65535"

**Refinement**:
> "Extend this function to return `tuple[bool, str, dict[str, Any] | None]` where the third value is connection metadata (host, port, ssl status) on success, None on failure. Show me how to unpack and handle both success and failure cases."

---

**Time**: 30 minutes
**Outcome**: You understand required vs. optional parameters, avoid mutable default argument bugs, unpack return values strategically, and design flexible function signatures with proper validation.
