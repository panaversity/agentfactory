---
sidebar_position: 4
title: "Lesson 4: Scope and Nested Functions"
description: "Understand variable scope (local, global, enclosing) and how nested functions work"
keywords: ["scope", "local scope", "global scope", "nested functions", "closures", "global keyword", "LEGB rule"]
cefr_level: "B1-B2"
estimated_time: "55 minutes"
proficiency_by_end: "Students predict variable scope correctly and write nested functions with closures"
prerequisites: ["Lessons 1-3: Function definition and calling", "Chapter 17: If/else logic", "Mental model of variable existence in code"]
learning_objectives:
  - "Predict whether a variable is accessible in a given scope"
  - "Understand the global keyword and when to use it (rarely)"
  - "Write nested functions that access outer scope variables (closures)"
  - "Apply scope understanding to avoid bugs and design better code"
  - "Recognize nested functions as preparation for advanced patterns (Chapter 26)"
ai_native_learning: true
try_with_ai_prompts: 4
colearning_elements: ["conceptual_prompt", "teaching_commentary", "specification_challenge", "ai_tip"]
---

## Lesson 4: Scope and Nested Functions

**CEFR Level**: B1-B2 (Intermediate to Advanced Application)
**Time Estimate**: 55 minutes
**What You'll Learn**: Variables don't exist everywhere in your code—they have scope. Understanding where a variable exists and is accessible is critical for writing correct code. You'll also explore nested functions, which prepare you for advanced patterns in later chapters.

---

## Why Scope Matters — Preventing Bugs and Clarifying Intent

**The Problem**: You define a variable inside a function, then try to use it outside. Python says "NameError: variable not defined." You modify a global variable inside a function expecting the change to persist, but it doesn't.

**The Reality**: Variables have scope—regions of code where they exist and are accessible. Understanding scope prevents bugs and clarifies your intent.

### 💻 Code Idea: Scope Boundaries

```python
# Global scope
message: str = "I'm global"

def demonstrate_scope():
    """Show scope boundaries."""
    local_message: str = "I'm local to this function"
    print(local_message)          # Works: local_message exists here

# print(local_message)  # ERROR: NameError - doesn't exist outside function
# local_message is only accessible inside demonstrate_scope()

print(message)  # Works: message is in global scope
```

**Key Insight**: Variables defined inside a function are local. Variables defined at module level are global. Each function has its own local scope.

### 💬 AI Colearning Prompt

> Ask your AI: "Explain what's wrong with this code and why. Then ask: How would you fix it?" (Provide code with scope error like modifying global without declaration, or trying to access local variable outside function)

**Expected Understanding**: You see that scope errors are real and understanding scope prevents them.

### 🎓 Instructor Commentary

Scope is about clarity and preventing bugs. When you see a variable, understanding WHERE it exists helps you predict WHAT the code does. This is how you validate code before running it—a critical AI-native developer skill. Good code minimizes dependencies on global state. When you're tempted to use `global`, that's often a signal to reconsider your design. Ask yourself: "Should this really be global? Could I pass it as a parameter instead?" Often the answer is yes.

---

## Local Scope — Variables Inside Functions

**Pattern**: Variables defined inside a function exist only within that function. Once the function returns, those variables cease to exist.

### 💻 Code Idea: Local Scope Isolation

```python
def example_local_scope():
    """Demonstrate that function variables are local."""
    message: str = "Inside function"
    print(f"Inside: {message}")  # Works: message is in local scope

example_local_scope()
# print(message)  # ERROR: NameError - message doesn't exist here

# Another function can have a variable with same name—no conflict
def another_function():
    message: str = "Different function"
    print(f"In another: {message}")

another_function()

# Each function has its own message variable
# They don't interfere with each other
```

**Key Observation**: Functions are isolated units. Variables inside one function don't affect another function with the same variable name. This isolation is a feature, not a bug.

---

## Global Scope — Module-Level Variables

**Pattern**: Variables defined at module level (outside any function) are in global scope. Any function can READ them, but MODIFYing requires the `global` keyword.

### 💻 Code Idea: Reading vs. Modifying Global Variables

```python
# Global scope (module level)
application_name: str = "MyApp"
version: float = 1.0

def show_app_info():
    """Show global app information (reading only)."""
    print(f"App: {application_name} v{version}")
    # This function reads global variables—no global keyword needed

show_app_info()  # App: MyApp v1.0

def reset_app():
    """Reset app state (modifying globals)."""
    global application_name, version  # Declare we're modifying these globals
    application_name = "MyApp (Reset)"
    version = 2.0

reset_app()  # Modifies global variables
show_app_info()  # App: MyApp (Reset) v2.0
```

**Critical Distinction**:
- **READING** a global variable: Just use it, no `global` keyword needed
- **MODIFYING** a global variable: Use `global variable_name` first

### 💻 Code Idea: Variable Shadowing (Accidental Mistake)

```python
counter: int = 0  # Global counter

def increment_counter():
    """This does NOT modify global counter."""
    counter: int = 1  # Local variable shadows global
    counter += 1      # Modifies LOCAL counter, not global
    print(f"Inside: {counter}")

print(f"Before: {counter}")    # 0
increment_counter()            # Inside: 2
print(f"After: {counter}")     # Still 0! (global unchanged)

def increment_global_counter():
    """This DOES modify global counter."""
    global counter
    counter += 1
    print(f"Inside: {counter}")

print(f"Before: {counter}")        # 0
increment_global_counter()         # Inside: 1
print(f"After: {counter}")         # 1 (global changed)
```

**Shadowing Lesson**: When you assign to a variable inside a function, Python creates a LOCAL variable with that name. It shadows (hides) the global variable. To modify the global, you must use the `global` keyword first.

---

## The `global` Keyword — Use Sparingly

**When to Use**: Rarely. When you absolutely must modify module-level state, use `global`.

**Better Approach**: Design your functions to take parameters and return values. Avoid global state.

### 💻 Code Idea: Design Without Global State

```python
# AVOID: Global mutable state (not recommended)
# counter: int = 0
# def increment():
#     global counter
#     counter += 1

# BETTER: Pass value in, return new value
def increment(current: int) -> int:
    """Return the value incremented by 1."""
    return current + 1

counter: int = 0
counter = increment(counter)
print(counter)  # 1
```

**Why better?** The function's behavior is clear from its signature. Input and output are explicit. No hidden dependencies on global state.

---

## Enclosing Scope and Nested Functions

**Pattern**: A function defined inside another function is a nested function. The inner function can access variables from the outer function's scope.

### 💻 Code Idea: Nested Functions and Closures

```python
def outer_function(multiplier: int):
    """Create a multiplier function."""

    def inner_function(number: int) -> int:
        """Inner function accesses multiplier from outer scope."""
        return number * multiplier  # Closure: accesses multiplier

    return inner_function

# Create multiplier functions with different multipliers
double = outer_function(2)   # multiplier = 2
triple = outer_function(3)   # multiplier = 3

# Each returned function "remembers" its multiplier
print(double(5))   # 5 * 2 = 10
print(triple(5))   # 5 * 3 = 15
print(double(10))  # 10 * 2 = 20
```

**Key Concept - Closure**: The inner function "closes over" the outer function's variables. Even after `outer_function` returns, the returned `double` function remembers the value of `multiplier` (2). This is called a closure.

### 💻 Code Idea: Practical Closure Example

```python
def calculate_discounted_price(original_price: float, discount_percent: float) -> callable:
    """
    Create a function that applies the same discount repeatedly.
    Demonstrates closure: inner function remembers discount info.
    """

    def apply_discount(quantity: int) -> float:
        """Apply the discount to a quantity of items."""
        total: float = original_price * quantity
        discount_amount: float = total * (discount_percent / 100)
        final_price: float = total - discount_amount
        return final_price

    return apply_discount

# Create a discount function for 20% off $10 items
bulk_discount = calculate_discounted_price(10, 20)

# Reuse the discount function
price_for_5: float = bulk_discount(5)    # 5 items at $10 each, 20% off
price_for_10: float = bulk_discount(10)  # 10 items at $10 each, 20% off

print(f"5 items: ${price_for_5:.2f}")   # $40.00
print(f"10 items: ${price_for_10:.2f}")  # $80.00
```

**Why this matters**: Closures enable partial application and function factories. The outer function creates specialized versions of the inner function. This is a powerful pattern used throughout Python.

---

## LEGB Rule — Python's Scope Order

**LEGB** stands for: **L**ocal, **E**nclosing, **G**lobal, **B**uilt-in

Python searches for variables in this order:

1. **Local** — Inside current function
2. **Enclosing** — In outer function (for nested functions)
3. **Global** — At module level
4. **Built-in** — Python's built-ins (print, len, etc.)

### 💻 Code Idea: LEGB Rule Demonstration

```python
x: str = "global"  # Global scope

def outer():
    x: str = "enclosing"  # Enclosing scope (for nested function)

    def inner():
        x: str = "local"  # Local scope
        print(f"Local x: {x}")

    inner()
    print(f"Enclosing x: {x}")

outer()
print(f"Global x: {x}")

# Output:
# Local x: local
# Enclosing x: enclosing
# Global x: global
```

**How it works**: Each `x` is in a different scope. When Python looks up `x`, it starts in Local, then Enclosing, then Global. It stops at the first match.

---

## Scope as Design Tool — Avoiding Mistakes

### 💻 Code Idea: Scope-Aware Function Design

```python
def create_counter(start: int = 0) -> dict:
    """
    Create a counter object (demonstrating design without global state).

    Returns:
        dict: Dictionary with increment and get_count functions
    """
    count: int = start

    def increment() -> None:
        """Increment the counter (closure)."""
        nonlocal count  # Modify enclosing scope variable
        count += 1

    def get_count() -> int:
        """Get current count (closure)."""
        return count

    return {"increment": increment, "get_count": get_count}

# Create a counter
counter = create_counter(0)
counter["increment"]()
counter["increment"]()
print(counter["get_count"]())  # 2

# Each counter is independent
counter2 = create_counter(100)
counter2["increment"]()
print(counter2["get_count"]())  # 101
```

**Note**: The `nonlocal` keyword (for completeness) modifies enclosing scope variables. Like `global`, use it sparingly.

---

## 🚀 Specification Challenge

Write a nested function that creates a personalized greeting generator:

1. The outer function takes a greeting phrase (e.g., "Hello")
2. The inner function takes a name and returns a full greeting
3. Test that multiple returned functions remember their own greetings

Then ask your AI: *"Is this a closure? How? Why is this useful?"*

This teaches you how nested functions and closures enable elegant designs.

---

## ✨ AI Tool Tip

When you see the `global` keyword in code, pause and think: **"Is this really necessary?"** Good Python code rarely uses `global`. If you're tempted to use `global`, ask your AI: **"Is there a better design pattern?"** Often there is—like returning values, using classes, or using closures.

---

## Try With AI

Use your AI companion (Claude Code or Gemini CLI). You'll explore scope behavior by running code and predicting results.

### Prompt 1: Predict Scope Behavior (Analyze Level)

```
Run this code and predict what will print. Then actually run it.
Was your prediction correct? If not, what did you learn?

x: int = 10

def modify_x():
    x = 20  # Local variable, shadows global
    print(f"Inside: {x}")

modify_x()
print(f"Outside: {x}")
```

**Expected outcome**: You predict Inside: 20, Outside: 10. You understand variable shadowing.

---

### Prompt 2: Analyze Scope Design (Analyze/Evaluate Level)

```
Here's code that uses global to track state:

count: int = 0
def increment():
    global count
    count += 1

Ask your AI: "Is this good design? What are the risks?
What would be a better approach?"

Listen to the reasoning about state management and design patterns.
```

**Expected outcome**: You evaluate trade-offs and understand why global state can be problematic.

---

### Prompt 3: Write Nested Function with Closure (Apply Level)

```
Write a function called `create_adder` that:
1. Takes one number as input
2. Returns a function
3. The returned function takes another number and returns the sum

Example:
add_5 = create_adder(5)
result = add_5(3)
print(result)  # 8

Include type hints and docstrings.
Test your implementation with different values.
```

**Expected outcome**: You write closure correctly and demonstrate understanding of nested functions.

---

### Prompt 4: Connect Scope to Architecture (Analyze/Synthesize Level)

```
Think about a project where you might need to maintain state across function calls
(like a game with a score, a shopping cart with items, a settings object).

Sketch the design:
- Where would state go (global, parameter, return value, closure)?
- What are the pros and cons of each approach?
- Which approach feels cleanest?

Ask your AI: "Are there design patterns that reduce dependency on global state?
Why do they matter?"

This teaches you that scope is an architectural decision.
```

**Expected outcome**: You see scope choices as design decisions with trade-offs. You understand that good design minimizes hidden dependencies.
