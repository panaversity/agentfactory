---
sidebar_position: 4
title: "Controlling Loops"
description: "Master break, continue, and loop-else patterns to control loop execution"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Break Statement"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement break to exit loops early when a search condition is met"

  - name: "Continue Statement"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use continue to skip specific iterations based on filtering conditions"

  - name: "For...Else Pattern"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Understand-Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can apply for...else to detect loop completion vs early exit scenarios"

  - name: "While...Else Pattern"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement retry logic with while...else for validation scenarios"

  - name: "Loop Control Flow Mastery"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can analyze when break/continue/else patterns improve code clarity and correctness"

  - name: "When to Use Each Control Pattern"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate which loop control pattern is most appropriate for a given scenario"

learning_objectives:
  - objective: "Implement break statements to exit loops early when search conditions are met"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Code examples demonstrating early loop exit with break"

  - objective: "Use continue statements to skip specific iterations based on filtering logic"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Code examples filtering data with continue"

  - objective: "Apply for...else and while...else patterns to detect loop completion vs early termination"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Search and retry scenarios using loop-else patterns"

  - objective: "Analyze when each loop control pattern improves code clarity and correctness"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Evaluation of break vs continue vs else in given scenarios"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (break, continue, for...else, while...else, early exit, skip iteration, completion detection) at A2-B1 limit ✓"

differentiation:
  extension_for_advanced: "Explore nested loop control (break in outer vs inner loop), loop labeling patterns, and when to refactor complex loop logic into functions"
  remedial_for_struggling: "Focus on break (early exit) and continue (skip) with simple list examples before introducing else clauses"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-17/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Controlling Loops

Sometimes you need more control over how loops execute. You might want to **exit a loop early** when you find what you're searching for, **skip certain iterations** that don't meet your criteria, or **detect whether a loop completed naturally** or was interrupted. Python provides three powerful tools for this: `break`, `continue`, and the loop-`else` clause.

In this lesson, you'll learn how to control loop execution with precision, making your code more efficient and expressive.

## Why Loop Control Matters

Consider these real-world scenarios where basic loops aren't enough:

**Search and Stop**: You're searching a list of 10,000 items for a specific value. Once you find it, there's no point continuing through the remaining items. You need to **exit early**.

**Skip Invalid Data**: You're processing user records, but some have missing or invalid fields. Instead of crashing or processing bad data, you want to **skip those records** and continue with the rest.

**Retry with Limit**: You're asking a user for input and they keep entering invalid data. You want to give them 3 chances, then give up. You need to know whether they **succeeded within the limit** or **ran out of attempts**.

These scenarios require more than simple `for` and `while` loops—they need **loop control statements**.

## Breaking Out Early: The `break` Statement

The `break` statement **immediately exits the loop**, skipping any remaining iterations. It's like saying "I found what I was looking for—stop searching."

### Basic Break: Search for a Number

```python
# Search for a target number in a list
numbers: list[int] = [3, 7, 12, 5, 19, 8]
target: int = 12

for num in numbers:
    if num == target:
        print(f"Found {target}!")
        break  # Exit the loop immediately
    print(f"Checking {num}...")

# Output:
# Checking 3...
# Checking 7...
# Found 12!
```

**What happens**:
1. Loop starts, checks 3 (not the target)
2. Checks 7 (not the target)
3. Checks 12 (matches!) → prints "Found 12!" → `break` exits the loop
4. Numbers 5, 19, 8 are **never checked** because the loop exited early

#### 💬 AI Colearning Prompt
> "Why does `break` immediately exit the loop instead of just ending the current iteration? What's happening in memory?"

### When to Use Break

Use `break` when:
- ✅ Searching for a specific item (stop when found)
- ✅ Validating input with a retry limit (stop when valid)
- ✅ Processing until a termination condition is met
- ❌ You need to skip ONE iteration but continue the loop (use `continue` instead)

#### 🎓 Expert Insight
> In AI-native development, you don't memorize every use case for `break`—you understand the **early exit pattern**: "I got what I needed; no point continuing." When you recognize that pattern in your logic, `break` is your tool.

## Skipping Iterations: The `continue` Statement

The `continue` statement **skips the rest of the current iteration** and moves to the next one. It's like saying "This one doesn't count—move to the next."

### Basic Continue: Skip Even Numbers

```python
# Print only odd numbers from 1 to 10
for num in range(1, 11):
    if num % 2 == 0:  # If the number is even
        continue      # Skip to next iteration
    print(f"{num} is odd")

# Output:
# 1 is odd
# 3 is odd
# 5 is odd
# 7 is odd
# 9 is odd
```

**What happens**:
1. When `num = 2` (even), `continue` skips the `print` and jumps to `num = 3`
2. When `num = 3` (odd), condition is False → `print` executes
3. Pattern repeats for all numbers

**Difference from Break**:
- `break` → Exit the **entire loop**
- `continue` → Skip the **current iteration** only

### Real-World Continue: Filtering Invalid Data

```python
# Process user records, skip invalid ones
user_records: list[dict[str, str]] = [
    {"name": "Alice", "email": "alice@example.com"},
    {"name": "Bob", "email": ""},  # Invalid: missing email
    {"name": "Charlie", "email": "charlie@example.com"},
    {"name": "", "email": "dave@example.com"},  # Invalid: missing name
]

for record in user_records:
    # Skip records with missing data
    if not record["name"] or not record["email"]:
        print(f"Skipping invalid record: {record}")
        continue

    # Process valid records
    print(f"Processing {record['name']} ({record['email']})")

# Output:
# Processing Alice (alice@example.com)
# Skipping invalid record: {'name': 'Bob', 'email': ''}
# Processing Charlie (charlie@example.com)
# Skipping invalid record: {'name': '', 'email': 'dave@example.com'}
```

This pattern is common in **data validation**: check for problems, skip bad data, process good data.

## Detecting Completion: The `for...else` Pattern

Python's loops have an **else clause** that executes **only if the loop completes normally** (without hitting `break`). This is one of Python's most misunderstood but powerful features.

### For...Else: Search with "Not Found" Detection

```python
# Search for a name in a list
names: list[str] = ["Alice", "Bob", "Charlie", "Diana"]
search_name: str = "Eve"

for name in names:
    if name == search_name:
        print(f"Found {search_name}!")
        break
else:
    # This runs ONLY if break was never called
    print(f"{search_name} not found in the list")

# Output:
# Eve not found in the list
```

**How it works**:
- If `break` is called → `else` is **skipped**
- If loop completes naturally → `else` **executes**

**Try it with a name that exists**:

```python
search_name: str = "Charlie"

for name in names:
    if name == search_name:
        print(f"Found {search_name}!")
        break
else:
    print(f"{search_name} not found in the list")

# Output:
# Found Charlie!
# (else clause skipped because break was called)
```

#### 💬 AI Colearning Prompt
> "Why is the loop-else pattern called 'else' if it's really 'if loop completed without break'? Explain the naming reasoning and show alternatives without using else."

### Common Mistake: Misunderstanding When Else Runs

**❌ WRONG ASSUMPTION**: "Else runs if the loop has no items"

```python
# Empty list
empty_list: list[int] = []

for item in empty_list:
    print(item)
else:
    print("This runs because the loop completed (even though it had 0 iterations)")

# Output:
# This runs because the loop completed (even though it had 0 iterations)
```

**✅ CORRECT UNDERSTANDING**: Else runs when the loop **completes naturally**, whether it had 0 iterations or 1,000.

The **only** way to prevent the `else` clause from running is with `break`.

### Why Use For...Else?

**Without `for...else`** (using a flag variable):

```python
found: bool = False

for name in names:
    if name == search_name:
        print(f"Found {search_name}!")
        found = True
        break

if not found:
    print(f"{search_name} not found")
```

**With `for...else`** (cleaner):

```python
for name in names:
    if name == search_name:
        print(f"Found {search_name}!")
        break
else:
    print(f"{search_name} not found")
```

The `for...else` version is **shorter and more readable**—no need for a flag variable.

#### 🎓 Expert Insight
> In AI-native development, the loop-else pattern is rare but powerful. You don't memorize when to use it—you recognize the "search with failure case" scenario. When you need to know "Did I find it or exhaust all options?", loop-else is your answer.

## While...Else: Retry with Limit

The `while...else` pattern works the same way: **else runs if the loop exits normally** (not via `break`).

This is perfect for **retry scenarios** where you want to know if the user succeeded or ran out of attempts.

### Retry Input with Limit

```python
# Give user 3 chances to enter a positive number
max_attempts: int = 3
attempt: int = 0

while attempt < max_attempts:
    user_input: str = input("Enter a positive number: ")

    # Check if input is a valid number using string methods
    if user_input.isdigit():  # Returns True for positive integers
        number: int = int(user_input)
        if number > 0:
            print(f"Success! You entered {number}")
            break  # Exit loop on success
        else:
            print("That's not positive. Try again.")
    else:
        print("That's not a valid number. Try again.")

    attempt += 1
else:
    # This runs ONLY if we exhausted all attempts without break
    print("Sorry, you've run out of attempts.")
```

**Execution scenarios**:

**Scenario 1: User succeeds on attempt 2**
```
Enter a positive number: abc
That's not a valid number. Try again.
Enter a positive number: -5
That's not positive. Try again.
Enter a positive number: 10
Success! You entered 10
(else clause skipped because break was called)
```

**Scenario 2: User fails all 3 attempts**
```
Enter a positive number: abc
That's not a valid number. Try again.
Enter a positive number: -5
That's not positive. Try again.
Enter a positive number: 0
That's not positive. Try again.
Sorry, you've run out of attempts.
(else clause runs because loop completed without break)
```

### Why While...Else?

**Without `while...else`**:
```python
success: bool = False

while attempt < max_attempts:
    # ... validation logic ...
    if valid:
        success = True
        break
    attempt += 1

if not success:
    print("Sorry, you've run out of attempts.")
```

**With `while...else`**:
```python
while attempt < max_attempts:
    # ... validation logic ...
    if valid:
        break
    attempt += 1
else:
    print("Sorry, you've run out of attempts.")
```

Again, the `while...else` version eliminates the need for a flag variable.

## AI Companion: Complex Loop Control Scenarios

Now that you understand the basics, let's explore more complex patterns with your AI companion.

### Combined Break and Continue: Input Validation Loop

Ask your AI to generate this scenario:

#### 🎓 Expert Insight: You Might See try-except in AI Responses

When you ask your AI to validate user input, it might generate code using `try` and `except` keywords. This is **exception handling**—a powerful Python feature you'll learn in **Chapter 21**. For now:

- **Don't worry if you see it**: Your AI is showing you professional Python code
- **You don't need to memorize it yet**: Focus on the loop control (`break`, `continue`, `while...else`)
- **Ask for an alternative**: If you see `try-except`, ask your AI: *"Can you show me a version using `.isdigit()` and `.startswith()` instead of try-except? I haven't learned exception handling yet."*

The key learning here is **how `break` and `continue` work together**, not exception handling.

#### 🤝 Practice Exercise

> **Ask your AI**: "Generate a while loop that asks users to enter numbers. Use `continue` to skip negative numbers (with a warning). Use `break` to exit when they enter 0. Calculate the sum of positive numbers only. Use `.isdigit()` for validation (not try-except). Then explain the flow with a step-by-step trace."

**Expected Outcome**: You'll understand how `break` and `continue` work together in the same loop through specification-driven practice.

**Sample AI-generated solution**:

```python
# Sum positive numbers until user enters 0
total: int = 0

while True:  # Infinite loop—relies on break to exit
    user_input: str = input("Enter a number (0 to quit): ")

    # Check if input is a valid number (handles positive, negative, and zero)
    # Note: isdigit() only works for positive integers, so we check for "-" prefix
    is_valid_number: bool = user_input.isdigit() or (
        user_input.startswith("-") and user_input[1:].isdigit()
    )

    if not is_valid_number:
        print("Invalid input. Try again.")
        continue  # Skip this iteration

    number: int = int(user_input)

    if number == 0:
        print(f"Total sum: {total}")
        break  # Exit the loop

    if number < 0:
        print("Negative numbers are ignored.")
        continue  # Skip adding to total

    # Only positive numbers reach here
    total += number
    print(f"Added {number}. Running total: {total}")
```

**Trace execution**:
```
Input: "5" → Valid, positive → total = 5
Input: "-3" → Valid, negative → skip (continue), total = 5
Input: "abc" → Invalid → skip (continue), total = 5
Input: "10" → Valid, positive → total = 15
Input: "0" → Valid, zero → print total (15), break
```

#### ✨ Teaching Tip
> When combining `break` and `continue`, ask your AI: "Trace this loop with inputs [5, -3, 'abc', 10, 0]. Show me which line executes for each input." Seeing the step-by-step flow helps you internalize control flow logic.

## Red Flags: Common Loop Control Mistakes

### 🚩 Red Flag 1: Misunderstanding When Else Executes

**❌ WRONG**:
```python
for i in range(3):
    print(i)
else:
    print("This runs because loop finished")  # Correct behavior

# Student thinks: "Else runs if condition is False"
# Reality: "Else runs if loop completes without break"
```

**✅ AI Troubleshooting Prompt**:
> "I thought loop-else was like if-else, but it's not. Explain the difference and show when else does NOT run in a for loop."

---

### 🚩 Red Flag 2: Using Continue Outside a Loop

**❌ ERROR**:
```python
numbers: list[int] = [1, 2, 3]

if 2 in numbers:
    continue  # SyntaxError: 'continue' not properly in loop
```

**Error Message**:
```
SyntaxError: 'continue' not properly in loop
```

**✅ Fix**: `continue` only works **inside** `for` or `while` loops.

**✅ AI Troubleshooting Prompt**:
> "I'm getting 'SyntaxError: continue not properly in loop'. Why can't I use continue in an if statement? Show me the correct pattern."

---

### 🚩 Red Flag 3: Break After Print (Unreachable Code)

**❌ LOGICAL ERROR**:
```python
for i in range(10):
    break  # Exits immediately on first iteration
    print(i)  # This NEVER runs
```

**✅ Fix**: Put `break` **after** the code you want to execute on the final iteration.

**✅ Correct Pattern**:
```python
for i in range(10):
    print(i)
    if i == 5:
        break  # Exit after printing 5
```

**✅ AI Troubleshooting Prompt**:
> "My loop with `break` prints nothing. Help me trace the execution and find where break should be placed."

---

### 🚩 Red Flag 4: Overusing Break (When Functions Are Better)

**❌ UNCLEAR CODE**:
```python
for i in range(100):
    for j in range(100):
        if some_condition:
            break  # Which loop does this exit?
```

**✅ CLEARER with Functions** (coming in Chapter 18):
```python
def find_pair(limit: int) -> tuple[int, int] | None:
    for i in range(limit):
        for j in range(limit):
            if some_condition:
                return (i, j)  # Exit entire function
    return None
```

**Note**: You'll learn about functions in the next chapter. For now, know that deeply nested loops with multiple `break` statements often signal that code should be refactored into separate functions.

---

### 🚩 Red Flag 5: Break in Nested Loops Only Exits Inner Loop

**❌ COMMON MISTAKE**:
```python
# Trying to exit both loops at once
for i in range(5):
    for j in range(5):
        if i == 2 and j == 3:
            break  # Only exits the INNER loop (j loop)
    # Outer loop (i loop) continues
```

**Output**:
```
# Processes i=0 with all j values
# Processes i=1 with all j values
# Processes i=2, exits inner loop at j=3, but continues i=3, i=4
```

**✅ AI Troubleshooting Prompt**:
> "I want to exit both nested loops at once, but `break` only exits the inner loop. What are my options? Explain flag variables vs functions vs other patterns."

**✅ Temporary Fix (Using a Flag)**:
```python
found: bool = False

for i in range(5):
    for j in range(5):
        if i == 2 and j == 3:
            found = True
            break  # Exit inner loop
    if found:
        break  # Exit outer loop
```

(Better solutions involve functions with `return`, which you'll learn in Chapter 18.)

## Try With AI: Search and Validation Patterns

You've learned `break`, `continue`, and `for...else`. Now master search and validation patterns by discovering when each control statement fits—with AI as your pattern validator.

### Part 1: Build Prime Number Finder (Your Turn First)

**Before asking AI**, implement a prime number checker using loop control:

**Task**: Find the first prime number after 20 using `break`:
```python
# Find first prime > 20
number: int = 21
while True:
    # Check if number is prime
    # If prime: print and break
    # If not prime: increment and continue
```

**Your implementation**:
1. Use nested loop to check divisors (2 to number-1)
2. Use `break` when prime is found
3. Use `continue` implicitly (or explicitly) to skip non-primes

Write the code BEFORE asking AI.

---

### Part 2: AI Explains For...Else Pattern (Discovery)

Share your prime finder with AI:

> "Here's my prime number finder: [paste code]. Explain:
> 1. Could I use `for...else` to simplify the prime check?
> 2. Show me how for...else works for search patterns
> 3. Rewrite my code using `for...else` instead of flag variables
> 4. When is for...else better than a flag variable like `is_prime = True`?"

**Your task**: Evaluate AI's for...else explanation.
- Does it show that `else` runs when loop completes WITHOUT break?
- Does it demonstrate that for...else eliminates the need for flag variables?
- Can you explain the pattern: "Search loop + break on found + else for not-found"?

---

### Part 3: Student Teaches AI (Break Limitations)

AI explained for...else. But does it know when break DOESN'T work?

Challenge AI with nested loop problems:

> "I want to exit BOTH loops when I find a match:
> ```python
> for i in range(10):
>     for j in range(10):
>         if i * j == 24:
>             break  # Only exits inner loop!
>     # Outer loop continues - how do I stop it?
> ```
>
> Show me THREE solutions:
> 1. Using a flag variable (`found = True`)
> 2. Using for...else on BOTH loops
> 3. Why would a function with `return` be better? (preview Chapter 20)
>
> For each, explain tradeoffs: readability vs. complexity."

**Your task**: Compare break strategies.
- Does AI show the flag pattern correctly (check flag after inner loop)?
- Does it explain that nested for...else can get confusing?
- Does it preview that functions with `return` are cleaner for nested exits?
- Which approach would YOU use and why?

---

### Part 4: Build Retry Validator Together (Convergence)

Now combine all loop control concepts—while, break, continue, else:

> "Build a password validator with retry logic:
> 1. User gets 3 attempts to enter password 'secret123'
> 2. Skip empty inputs with `continue` (don't count as attempt)
> 3. Exit with `break` on correct password
> 4. Use `while...else` to show 'Account locked' if all attempts fail
> 5. Use type hints and track attempt count
>
> Show complete code that handles:
> - Correct password on attempt 2
> - Empty input (doesn't count)
> - 3 failed attempts (triggers else)"

**Your task**: Review the complete pattern.
- Does it use `while attempts < 3:` with increment?
- Does it use `continue` for empty input (before incrementing)?
- Does it use `break` on success?
- Does `else` clause execute only when attempts exhausted?

Iterate if needed:
> "The empty input check increments attempt counter. Fix it so empty inputs don't count against the limit."

---

**Time**: 25-30 minutes total
**Outcome**: You've mastered search patterns (for...else), nested break strategies, and retry validation combining all loop control statements.

**Safety Note**: Always test break/continue logic with edge cases (empty input, first attempt success, exhausted attempts) to verify control flow.
