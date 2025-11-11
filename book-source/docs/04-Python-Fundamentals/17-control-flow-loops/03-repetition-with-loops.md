---
title: "Repetition with Loops"
chapter: 17
lesson: 3
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "For Loop with range()"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write for loops with range() to repeat code a known number of times; understand 0-based iteration"

  - name: "Range() Function Mastery"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use range() with one, two, or three arguments (stop, start/stop, start/stop/step); predict sequence output"

  - name: "While Loop Implementation"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write while loops with proper termination conditions; understand condition-based repetition"

  - name: "Loop Termination Logic"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student can identify termination conditions in while loops; recognize infinite loop risks; write loops that exit correctly"

  - name: "For vs While Decision"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain when to use for vs while; choose appropriate loop type for given scenarios"

  - name: "Infinite Loop Prevention"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student can identify patterns that cause infinite loops; describe how to avoid and fix them"

learning_objectives:
  - objective: "Implement for loops using range() for definite iteration with known repetition count"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes for loops that correctly iterate specified number of times"

  - objective: "Use range() with start, stop, and step parameters to control iteration sequences"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student generates number sequences (even numbers, countdown) using range() variations"

  - objective: "Implement while loops for indefinite iteration based on conditions"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Student writes while loops with proper termination logic for condition-based repetition"

  - objective: "Recognize when to use for vs while loops in given scenarios"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student explains loop choice for different problem types and implements correct loop type"

  - objective: "Identify and prevent infinite loops by ensuring proper termination conditions"
    proficiency_level: "A2-B1"
    bloom_level: "Understand"
    assessment_method: "Student describes infinite loop causes and demonstrates fixes"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (for loop, range(), start/stop/step, while loop, termination conditions, infinite loop dangers, for vs while decision) at A2-B1 limit ✓"

differentiation:
  extension_for_advanced: "Explore nested loops with range() (preview of Lesson 5); investigate list comprehensions as alternative to simple for loops"
  remedial_for_struggling: "Practice range() variations separately before combining with loops; use visual number line diagrams for range() sequences"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-part-4-chapter-17/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 3: Repetition with Loops

Imagine you need to print the numbers 1 through 100. You *could* write 100 separate `print()` statements—but that would be tedious, error-prone, and wasteful. Or imagine checking a password: you want to give the user three attempts to enter the correct password. You don't know *how many tries they'll need*, but you know the loop should stop after three failures.

This is where **loops** shine. Loops automate repetition, letting you run the same code multiple times without copying it. Python gives you two fundamental loop types: **`for` loops** for definite iteration (when you know how many times to repeat) and **`while` loops** for indefinite iteration (when repetition depends on a changing condition). In this lesson, you'll learn both, understand when to use each, and recognize how to avoid common pitfalls like infinite loops.

## Why Automate Repetition?

In real-world programs, repetition is everywhere:

- **Processing data**: Apply the same transformation to every item in a list
- **User input validation**: Keep asking until the user provides valid input
- **Countdown timers**: Count down from 10 to 1 for a game or app
- **Batch operations**: Convert 100 files from one format to another

Without loops, you'd copy-paste code hundreds of times. With loops, you write the logic once and let Python handle the repetition. This makes your code shorter, clearer, and easier to maintain.

## For Loops: Definite Iteration

A **`for` loop** repeats code a specific number of times. You use it when you know *how many iterations* you need—or when you're iterating over a sequence (like a list of items).

### Basic For Loop with range()

The simplest `for` loop uses `range()` to generate a sequence of numbers:

```python
# Print numbers 1 through 10
for i in range(1, 11):
    print(i)
```

**Expected Output:**
```
1
2
3
4
5
6
7
8
9
10
```

**How it works:**

1. `range(1, 11)` generates numbers from 1 up to (but **not including**) 11
2. The loop variable `i` takes each value in sequence: 1, then 2, then 3, ... up to 10
3. The indented `print(i)` runs once for each value of `i`
4. When `range()` runs out of numbers, the loop ends

#### 💬 AI Colearning Prompt
> "Explain how `for` loops work under the hood with `range()` and iteration in Python."

### Understanding range()

The `range()` function generates number sequences. It has three forms:

**1. One argument (stop)**

```python
for i in range(5):
    print(i)
```

Output:
```
0
1
2
3
4
```

Generates: 0, 1, 2, 3, 4 (5 numbers total, starting from 0)

**2. Two arguments (start, stop)**

```python
for i in range(2, 6):
    print(i)
```

Output:
```
2
3
4
5
```

Generates: 2, 3, 4, 5 (starts at 2, stops before 6)

**3. Three arguments (start, stop, step)**

```python
# Print even numbers from 2 to 10
for i in range(2, 11, 2):
    print(i)
```

Output:
```
2
4
6
8
10
```

Generates: 2, 4, 6, 8, 10 (starts at 2, stops before 11, increments by 2)

**Key Insight:** `range(start, stop, step)` NEVER includes the `stop` value. This is Python's convention: `range(1, 11)` gives you 1-10, not 1-11.

#### 🎓 Instructor Commentary
> In AI-native development, you don't memorize `range()` parameters—you understand WHEN you need a number sequence. Syntax is cheap; recognizing "I need to repeat N times" is gold.

### Practical Example: Countdown Timer

```python
# Countdown from 10 to 1
for i in range(10, 0, -1):
    print(f"T-minus {i} seconds")

print("Liftoff!")
```

**Expected Output:**
```
T-minus 10 seconds
T-minus 9 seconds
T-minus 8 seconds
T-minus 7 seconds
T-minus 6 seconds
T-minus 5 seconds
T-minus 4 seconds
T-minus 3 seconds
T-minus 2 seconds
T-minus 1 seconds
Liftoff!
```

**How it works:**
- `range(10, 0, -1)` counts DOWN from 10 to 1 (step = -1)
- Loop runs 10 times (i = 10, 9, 8, ..., 1)
- After the loop finishes, `print("Liftoff!")` runs once

## While Loops: Indefinite Iteration

A **`while` loop** repeats code as long as a condition remains `True`. You use it when you don't know in advance how many iterations you'll need—the loop continues until some condition changes.

### Basic While Loop Syntax

```python
# Count from 1 to 5 using while
count: int = 1

while count <= 5:
    print(count)
    count += 1  # CRITICAL: Update the loop variable
```

**Expected Output:**
```
1
2
3
4
5
```

**How it works:**

1. Before each iteration, Python checks: `Is count <= 5?`
2. If `True`, the indented code runs
3. `count += 1` increases `count` by 1
4. Loop repeats until `count` becomes 6 (then `count <= 5` is `False`)
5. Loop ends

**CRITICAL:** The loop variable (`count`) MUST change inside the loop. If it doesn't, the condition stays `True` forever, creating an **infinite loop**.

### Practical Example: User Input Validation

```python
# Keep asking until user enters valid age
age: int = 0

while age < 1 or age > 120:
    age = int(input("Enter your age (1-120): "))

    if age < 1 or age > 120:
        print("Invalid age. Please try again.")

print(f"Thank you! You are {age} years old.")
```

**How it works:**
- Loop continues while age is outside valid range (less than 1 OR greater than 120)
- User is prompted repeatedly until they enter a valid age (1-120)
- Once a valid age is entered, condition becomes `False` and loop exits

#### ✨ Teaching Tip
> Use your AI companion to explore edge cases: "What happens if the user enters a negative number? What if they type 'abc' instead of a number? Show me the error and explain it."

## Loop Termination Conditions: How to Avoid Infinite Loops

An **infinite loop** runs forever because its condition never becomes `False`. This is one of the most common beginner errors.

### Example of Infinite Loop (DON'T DO THIS)

```python
# ⚠️ INFINITE LOOP - DO NOT RUN
count: int = 1

while count <= 5:
    print(count)
    # ERROR: Forgot to update count!
```

**What happens:**
- `count` starts at 1
- Condition `count <= 5` is `True`
- Loop prints 1, then checks condition again
- `count` is still 1, so condition is still `True`
- Loop prints 1 again... and again... forever

**How to avoid infinite loops:**

1. **Always update the loop variable** inside the loop body
2. **Ensure the condition can eventually become `False`**
3. **Test with small values** to verify loop exits correctly
4. **Use Ctrl+C** to stop a runaway loop if you accidentally create one

### Fixed Version

```python
# ✅ CORRECT - Loop terminates
count: int = 1

while count <= 5:
    print(count)
    count += 1  # ✅ Updates count each iteration
```

## For vs While: When to Use Each

Both loop types can accomplish many of the same tasks, but each has its ideal use cases:

| Loop Type | Use When | Example Scenarios |
|-----------|----------|-------------------|
| **`for`** | You know how many iterations you need | Print numbers 1-100, process each item in a list, run code exactly N times |
| **`while`** | Repetition depends on a changing condition | User input validation, retry logic until success, game loop until player quits |

**Rule of Thumb:**
- **Known count → `for` loop**
- **Condition-based → `while` loop**

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Show me the same task (print numbers 1-5) implemented with BOTH a for loop and a while loop. Then explain when each approach is more appropriate."

**Expected Outcome:** You'll see both implementations side-by-side and understand the tradeoffs.

## AI Companion Section: Comparing For and While

Let's solve the same problem both ways to see the differences:

**Task:** Print numbers 1 through 5

**Using `for` loop:**

```python
for i in range(1, 6):
    print(i)
```

**Using `while` loop:**

```python
i: int = 1

while i <= 5:
    print(i)
    i += 1
```

**Comparison:**

| Aspect | For Loop | While Loop |
|--------|----------|------------|
| **Clarity** | More concise: one line manages iteration | Requires explicit variable initialization and update |
| **Safety** | Can't accidentally create infinite loop with `range()` | Risk of infinite loop if you forget to update variable |
| **Use Case** | When you know iteration count | When iteration depends on condition |

**Which is better?** For this task, **`for` loop** is clearer and safer. Use `while` when you truly need condition-based repetition (like user input validation).

### Infinite Loop Demonstration + Fix

**Infinite Loop (⚠️ Warning: Don't run this):**

```python
# ⚠️ INFINITE LOOP
counter: int = 0

while counter < 10:
    print(f"Counter: {counter}")
    # ERROR: Forgot to increment counter
```

**Fixed Version:**

```python
# ✅ CORRECT
counter: int = 0

while counter < 10:
    print(f"Counter: {counter}")
    counter += 1  # ✅ Increments counter each iteration
```

**The fix:** Added `counter += 1` so the loop variable changes and the condition eventually becomes `False`.

## Red Flags to Watch For

When working with loops, watch for these common errors:

### 🚩 Infinite Loops

**Symptom:** Program hangs, repeats output forever, or becomes unresponsive

**Causes:**
- Forgot to update loop variable in `while` loop
- Condition can never become `False`
- Update logic is inside an unreachable `if` block

**Fix:** Ensure loop variable changes each iteration and condition can become `False`

**AI Troubleshooting Prompt:**
> "I have a while loop that runs forever. Here's my code: [paste code]. What's wrong and how do I fix it?"

### 🚩 Off-by-One Errors

**Symptom:** Loop runs one too many or one too few times

**Example:**
```python
# Want to print 1-10, but this prints 1-9
for i in range(1, 10):  # ❌ Stops at 9, not 10
    print(i)

# Correct version
for i in range(1, 11):  # ✅ Stops at 10
    print(i)
```

**Fix:** Remember `range(start, stop)` goes UP TO but DOES NOT INCLUDE `stop`

**AI Troubleshooting Prompt:**
> "My for loop with range(1, 10) only prints up to 9 instead of 10. Why?"

### 🚩 Loop Variable Scope Confusion

**Symptom:** Trying to use loop variable outside loop or getting unexpected value

**Example:**
```python
for i in range(5):
    print(i)

print(f"Final value: {i}")  # i = 4 (last value from loop)
```

**What happens:** Loop variable `i` persists after loop ends with its last value (4)

**Best Practice:** Don't rely on loop variable value after loop; use meaningful names (`for item in items`)

**AI Troubleshooting Prompt:**
> "Why does my loop variable still exist after the loop ends? Is that expected?"

### 🚩 Step Zero Error

**Symptom:** `ValueError: range() arg 3 must not be zero`

**Example:**
```python
for i in range(0, 10, 0):  # ❌ Step can't be zero
    print(i)
```

**Fix:** Step must be non-zero (positive to count up, negative to count down)

**AI Troubleshooting Prompt:**
> "I got 'range() arg 3 must not be zero' error. What does this mean?"

## Try With AI

Now that you understand `for` and `while` loops, test your knowledge with your AI companion. Use **ChatGPT web** (or your AI companion tool if you've already set one up from earlier chapters).

**1. Recall: Understand the Difference**

> "What's the difference between `for` loops and `while` loops in Python? When should I use each?"

**Expected Outcome:** Your AI will explain that `for` is for known iteration counts, `while` is for condition-based repetition. You'll see examples of ideal use cases for each.

**2. Understand: Trace Execution Step-by-Step**

> "Trace this `for` loop step-by-step: `for i in range(2, 6, 1): print(i)`. What values does `i` take, and what is the output?"

**Expected Outcome:** Your AI will walk through each iteration: i=2 (prints 2), i=3 (prints 3), i=4 (prints 4), i=5 (prints 5). Loop stops before 6. You'll understand how `range(start, stop, step)` works.

**3. Apply: Generate Countdown Loop**

> "Generate a `while` loop that counts down from 10 to 1, printing each number. Include type hints and proper termination logic."

**Expected Outcome:** Your AI will provide code with:
- Type-hinted variable initialization (`count: int = 10`)
- While condition (`while count >= 1`)
- Print statement
- Decrement logic (`count -= 1`)

Review the code and verify it would print: 10, 9, 8, ..., 1

**4. Analyze: Diagnose Infinite Loop**

> "What happens if I forget to update the loop counter in this `while` loop? Show me an example of the error and how to fix it:
> ```
> i = 0
> while i < 5:
>     print(i)
> ```
> "

**Expected Outcome:** Your AI will explain that the loop runs forever because `i` never changes. It will show the fix: adding `i += 1` inside the loop. You'll understand why loop variable updates are critical.

**Safety Note:** AI-generated code may contain infinite loops. Always review loop termination logic before running code. If your program hangs, use Ctrl+C to stop execution.
