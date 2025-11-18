---
sidebar_position: 2
title: "Pattern Matching with match-case"
description: "Learn Python's modern match-case syntax for clearer decision-making code"
reading_level: "Grade 7-9"
time_estimate: "50-65 minutes"
generated_by: "content-implementer"
source_spec: "specs/001-part-4-chapter-17/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "content-implementer-agent"
workflow: "spec-driven-development"
version: "1.0.0"
skills:
  - name: "Pattern Matching Syntax"
    cefr_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
  - name: "Wildcard Pattern Recognition"
    cefr_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
  - name: "Code Readability Assessment"
    cefr_level: "A2-B1"
    category: "Soft"
    bloom_level: "Analyze"
  - name: "Python 3.10+ Syntax Awareness"
    cefr_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
  - name: "Intent Clarity in Code"
    cefr_level: "B1"
    category: "Soft"
    bloom_level: "Analyze"
---

# Pattern Matching with match-case

You just learned how to make decisions using `if`, `elif`, and `else` statements. They work great! But sometimes you find yourself writing long chains of `elif` statements that all check the same variable against different values. When that happens, there's a clearer way to express your intent: Python's `match-case` syntax.

Think of `match-case` as a structured decision table. Instead of reading through a chain of conditions one by one, you can see all possible values and their outcomes at a glance. This lesson introduces you to pattern matching—a modern Python feature (added in Python 3.10) that makes certain types of decisions easier to read and maintain.

**What You'll Learn:**
- When `match-case` improves code readability over `if/elif/else`
- How to write basic `match-case` statements
- How to match specific values (literal patterns)
- How to handle "everything else" cases with the wildcard pattern `_`
- How to ask your AI companion to convert between styles

**Prerequisites:** Lesson 1 (Making Decisions with Conditionals)
**CEFR Level:** A2-B1 (Transitional)
**Time:** 50-65 minutes

---

## Why match-case Exists: Readability Matters

Let's say you're building a program that responds to HTTP status codes. Here's how you might write it with `if/elif/else`:

```python
# Using if/elif/else to handle HTTP status codes
status_code: int = 404

if status_code == 200:
    print("OK - Request succeeded")
elif status_code == 404:
    print("Not Found - Resource doesn't exist")
elif status_code == 500:
    print("Internal Server Error - Something went wrong")
elif status_code == 403:
    print("Forbidden - Access denied")
else:
    print("Unknown status code")
```

This works perfectly, but notice the pattern: you're checking the **same variable** (`status_code`) against **different values** over and over. The intent is clear once you read it, but the structure is repetitive.

Now look at the same logic using `match-case`:

```python
# Using match-case for the same logic
status_code: int = 404

match status_code:
    case 200:
        print("OK - Request succeeded")
    case 404:
        print("Not Found - Resource doesn't exist")
    case 500:
        print("Internal Server Error - Something went wrong")
    case 403:
        print("Forbidden - Access denied")
    case _:
        print("Unknown status code")
```

**What's different?**
- You only mention `status_code` **once** (in the `match` line)
- Each `case` shows a value, not a comparison
- The structure looks like a table of possibilities
- The wildcard `_` (underscore) handles "everything else"

**When does this improve your code?** When you're comparing one variable against multiple specific values. The `match-case` structure makes your intent crystal clear: "Here are all the possibilities for this value."

---

## 🎓 Tier 1: Book Teaches - Basic match-case Syntax

### The match-case Structure

Here's the pattern you'll use:

```python
match <variable_to_check>:
    case <value1>:
        # Code to run if variable equals value1
    case <value2>:
        # Code to run if variable equals value2
    case _:
        # Code to run if no other case matched (default)
```

**Key points:**
- `match` starts the decision structure (like the first `if`)
- Each `case` represents one possible value
- The colon (`:`) is required after `match` and each `case`
- Indentation matters (just like with `if` statements)
- The underscore `_` is the wildcard pattern—it matches anything

**IMPORTANT:** `match-case` was added in Python 3.10 (October 2021). If you're using Python 3.9 or earlier, this syntax won't work. You'll see a `SyntaxError`. Your code will run fine on Python 3.14+ (the version we're using).

---

### Example 1: HTTP Status Codes (Literal Patterns)

Let's see the complete example in action:

```python
# match-case with HTTP status codes
status_code: int = 200  # Try changing this to 404, 500, 403, or 999

match status_code:
    case 200:
        message: str = "OK - Request succeeded"
    case 404:
        message: str = "Not Found - Resource doesn't exist"
    case 500:
        message: str = "Internal Server Error - Something went wrong"
    case 403:
        message: str = "Forbidden - Access denied"
    case _:
        message: str = "Unknown status code"

print(message)
```

**Expected output (with status_code=200):**
```
OK - Request succeeded
```

**Expected output (with status_code=999):**
```
Unknown status code
```

**How this executes:**
1. Python looks at the value of `status_code`
2. It checks each `case` from top to bottom
3. When it finds a match, it runs that code block
4. It **stops checking** after the first match (no fall-through)
5. If no case matches, it runs the `_` wildcard case

**Note:** Unlike `if/elif/else`, you don't need `break` statements. Python automatically stops after the first matching case.

---

### Example 2: Menu Selection (Wildcard Pattern in Action)

Let's build a simple menu system:

```python
# Menu selection with match-case
choice: str = "save"  # Try: "save", "load", "quit", "help", "unknown"

match choice:
    case "save":
        print("Saving your work...")
    case "load":
        print("Loading saved data...")
    case "quit":
        print("Exiting program. Goodbye!")
    case "help":
        print("Available commands: save, load, quit, help")
    case _:
        print(f"Unknown command: '{choice}'. Type 'help' for options.")

# Example output with choice="save": Saving your work...
# Example output with choice="unknown": Unknown command: 'unknown'. Type 'help' for options.
```

**Why use the wildcard `_`?**
Users might type anything—`"sav"` (typo), `"exit"` (synonym), or `"123"` (random input). The wildcard pattern catches all unexpected inputs and gives helpful feedback. Without it, your program would just do nothing when users enter invalid commands.

**Comparison to if/elif:**
```python
# Same logic with if/elif/else
if choice == "save":
    print("Saving your work...")
elif choice == "load":
    print("Loading saved data...")
elif choice == "quit":
    print("Exiting program. Goodbye!")
elif choice == "help":
    print("Available commands: save, load, quit, help")
else:
    print(f"Unknown command: '{choice}'. Type 'help' for options.")
```

Both work. The `match-case` version makes the intent clearer: "Here's a menu of options." The `if/elif` version emphasizes the sequential checking: "Is it this? Or this? Or this?"

---

## 🚀 Tier 2: AI Companion - Converting and Extending Patterns

### Example 3: Converting if/elif to match-case (Traffic Light Logic)

You learned about traffic lights in Lesson 1. Let's ask your AI companion to convert that logic to `match-case`.

**💬 AI CoLearning Prompt:**

"I have this if/elif/else logic for traffic light colors:

```python
color: str = "yellow"

if color == "red":
    action: str = "Stop"
elif color == "yellow":
    action: str = "Slow down"
elif color == "green":
    action: str = "Go"
else:
    action: str = "Unknown signal"

print(action)
```

Convert this to match-case syntax. Keep the type hints and comments."

**🤖 AI-Generated Code:**

```python
# Traffic light logic with match-case
color: str = "yellow"

match color:
    case "red":
        action: str = "Stop"
    case "yellow":
        action: str = "Slow down"
    case "green":
        action: str = "Go"
    case _:
        action: str = "Unknown signal"

print(action)

# Expected output: Slow down
```

**When you validate AI-generated code:**
- Does it produce the same output for the same inputs? (Test with `color="red"`, `color="green"`, `color="yellow"`, `color="blue"`)
- Are the type hints still present? ✅
- Is the wildcard case `_` at the end? ✅

---

### Example 4: Multiple match-case Statements (Calculator Operations)

Let's build a simple calculator that handles basic operations:

**💬 AI CoLearning Prompt:**

"Create a match-case that takes an operation ('+', '-', '*', '/') and two numbers (10 and 5), then performs the calculation. Include type hints and handle unknown operations with the wildcard pattern."

**🤖 AI-Generated Code:**

```python
# Simple calculator with match-case
operation: str = "+"
num1: int = 10
num2: int = 5
result: int | float | str  # Result can be int, float, or error message

match operation:
    case "+":
        result = num1 + num2
    case "-":
        result = num1 - num2
    case "*":
        result = num1 * num2
    case "/":
        if num2 != 0:
            result = num1 / num2
        else:
            result = "Error: Division by zero"
    case _:
        result = f"Unknown operation: {operation}"

print(f"{num1} {operation} {num2} = {result}")

# Example outputs:
# With operation="+": 10 + 5 = 15
# With operation="/": 10 / 5 = 2.0
# With operation="%": 10 % 5 = Unknown operation: %
```

#### 🎓 Expert Insight
> Notice that `case "/"` includes extra logic (checking for division by zero). You can have multiple lines of code in each case block—just keep them indented. Also notice the type hint `int | float | str` on `result`—this is a **union type** that says "result can be an integer, a float, or a string." Modern Python lets you express this clearly.

**Your job when reviewing this code:**
- Test all four operations: `+`, `-`, `*`, `/`
- Test the division by zero case (change `num2` to `0`)
- Test an unknown operation (change `operation` to `"%"`)

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a match-case statement that converts numeric grades (1-5) to letter grades: 5='A', 4='B', 3='C', 2='D', 1='F'. Include type hints and handle invalid grades with the wildcard pattern. Then explain when match-case is preferable to if/elif for this scenario."

**Expected Outcome**: You'll understand how match-case provides clearer structure for exact value matching compared to if/elif chains, and practice specification-writing for AI-generated code.

---

## ⚠️ Red Flags: Common match-case Mistakes

Watch out for these common errors when writing `match-case` statements:

### 1. Using Python 3.9 or Earlier

**Error you'll see:**
```
SyntaxError: invalid syntax
```

**Why:** `match-case` was added in Python 3.10 (October 2021). If you're using an older version, Python doesn't recognize the syntax.

**Fix:** Check your Python version:
```bash
python --version
```
If it shows Python 3.9.x or earlier, you need to use `if/elif/else` instead, or upgrade Python.

**💬 Ask Your AI:**
"I'm getting a SyntaxError with match-case. I'm using Python 3.9. Can you rewrite this using if/elif/else instead?"

---

### 2. Missing Colon After case

**❌ Wrong:**
```python
match status_code
    case 200
        print("OK")
```

**✅ Correct:**
```python
match status_code:
    case 200:
        print("OK")
```

**Error you'll see:** `SyntaxError: invalid syntax`
**Fix:** Add colons after `match <variable>` and after each `case <value>`.

---

### 3. Unreachable Patterns (Wildcard in the Middle)

**❌ Wrong:**
```python
match status_code:
    case 200:
        print("OK")
    case _:
        print("Unknown")
    case 404:  # This will NEVER run!
        print("Not Found")
```

**Why this fails:** Once Python matches the wildcard `_`, it stops checking. The `case 404` below it becomes unreachable—it can never execute.

**✅ Correct:** Always put the wildcard `_` **last**:
```python
match status_code:
    case 200:
        print("OK")
    case 404:
        print("Not Found")
    case _:
        print("Unknown")
```

**💬 Ask Your AI:**
"Why does my match-case always print 'Unknown' even when I pass 404? Here's my code: [paste code]"

---

### 4. Type Mismatches

**❌ Wrong:**
```python
status_code: int = 200

match status_code:
    case "200":  # This is a string, not an integer!
        print("OK")
```

**Why this fails:** You're comparing an `int` (200) to a `str` ("200"). They're not equal, so the case won't match.

**✅ Correct:** Match the types:
```python
status_code: int = 200

match status_code:
    case 200:  # Integer matches integer
        print("OK")
```

**💬 Ask Your AI:**
"My match-case isn't matching status_code=200. The case uses '200' (string). What's wrong?"

---

### 5. Missing Default Case

**⚠️ Not an error, but risky:**
```python
match choice:
    case "save":
        print("Saving...")
    case "load":
        print("Loading...")
# What happens if choice = "help"? Nothing!
```

**Why this is risky:** If none of the cases match, your program does nothing. This can confuse users (or you, when debugging).

**✅ Better:** Always include a default case:
```python
match choice:
    case "save":
        print("Saving...")
    case "load":
        print("Loading...")
    case _:
        print(f"Unknown command: {choice}")
```

**When you can skip `case _`:** Only when you've covered **all possible values**. For example, if you're matching `True` or `False` (a boolean), you only need two cases.

---

## Try With AI: Pattern Matching Design Challenge

You've learned `match-case` syntax for exact value matching. Now discover when pattern matching improves readability vs. when `if-elif` is better—with AI as your design critic.

### Part 1: Build HTTP Status Handler (Your Turn First)

**Before asking AI**, implement an HTTP status code handler using `match-case`:

**Scenario**: Handle HTTP response codes:
```python
status_code: int = 404  # Could be 200, 404, 500, etc.

# Your task: Use match-case to map:
# 200 → "Success"
# 404 → "Not Found"
# 500 → "Server Error"
# _ → "Unknown Status"
```

**Your task**:
1. Write the complete `match-case` structure
2. Test with status_code = 200, 404, 500, 999
3. Predict: Why is `match-case` better here than `if-elif`?

---

### Part 2: AI Explains Match vs If-Elif (Discovery)

Share your implementation with AI:

> "Here's my HTTP status handler using match-case: [paste code]. Explain:
> 1. Is my syntax correct? (Python 3.10+ required)
> 2. Why is match-case better for this than if status_code == 200: elif status_code == 404:?
> 3. What makes a problem 'match-case friendly' vs. 'if-elif friendly'?
> 4. Show me a counter-example where if-elif is BETTER (hint: grade ranges with >=)"

**Your task**: Evaluate AI's design guidance.
- Does it explain that `match-case` is clearer for **discrete values**?
- Does it show that `if-elif` is better for **ranges** (>=, &lt;)?
- Can you articulate the rule: "Exact values → match-case, Comparisons → if-elif"?

---

### Part 3: Student Teaches AI (When Match-Case Fails)

AI explained when to use `match-case`. But does it know its limitations?

Challenge AI with impossible scenarios:

> "I tried to use match-case for grade ranges and it doesn't work:
> ```python
> grade: int = 85
> match grade:
>     case >= 90:  # SyntaxError!
>         letter = 'A'
> ```
>
> Explain:
> 1. Why does this fail? (match-case doesn't support comparison operators)
> 2. Could I use match-case with structural patterns instead? (Show if possible)
> 3. For each scenario, tell me: match-case or if-elif?
>    - Menu choices (1=Save, 2=Load, 3=Quit)
>    - Age verification (>= 18)
>    - HTTP status codes (200, 404, 500)
>    - Temperature ranges (< 0, 0-32, > 32)"

**Your task**: Understand the boundaries.
- Can you now identify which problems suit `match-case`?
- Does AI explain that Python 3.10 match-case is for **structural pattern matching**, not comparison logic?
- Which of the 4 scenarios should use `match-case`? (Menu: yes, Age: no, HTTP: yes, Temp: no)

---

### Part 4: Convert Between Patterns (Convergence)

Now practice converting between `if-elif` and `match-case`:

> "Convert this if-elif chain to match-case if appropriate:
> ```python
> command: str = 'save'
>
> if command == 'save':
>     action = 'Saving file...'
> elif command == 'load':
>     action = 'Loading file...'
> elif command == 'quit':
>     action = 'Exiting...'
> else:
>     action = 'Unknown command'
> ```
>
> Then convert THIS one (grade ranges):
> ```python
> if grade >= 90:
>     letter = 'A'
> elif grade >= 80:
>     letter = 'B'
> else:
>     letter = 'F'
> ```
>
> For each: Should it be converted? If yes, show match-case version. If no, explain why."

**Your task**: Review conversions.
- Does AI convert the command example (exact string matching)?
- Does AI refuse to convert the grade example (range comparisons)?
- Can you explain WHY the first converts but the second doesn't?

---

**Time**: 20-25 minutes total
**Outcome**: You've mastered when to use `match-case` (exact values, discrete options) vs. `if-elif` (ranges, complex conditions), and can convert between patterns appropriately.

**Safety Note**: `match-case` requires Python 3.10+. If you get `SyntaxError: invalid syntax`, check your Python version with `python --version`.

