---
sidebar_position: 1
title: "Lesson 1: Understanding Modules and Imports"
description: "Learn what modules are, how Python organizes code, and the three main import patterns"
keywords: ["modules", "imports", "built-in modules", "import variants", "math", "random", "os"]
cefr_level: "A2"
estimated_time: "55 minutes"
proficiency_by_end: "Students can import built-in modules (math, random, os) and use their functions correctly"
prerequisites: ["Chapter 13: Python basics (variables, print, execution flow)", "Chapter 14: Type hints syntax"]
learning_objectives:
  - "Explain what a module is and why Python developers organize code into modules"
  - "Use three import variants (import, from X import Y, from X import Y as Z) correctly"
  - "Access and use functions from built-in modules (math, random, os)"
  - "Read module documentation to discover available functions"
ai_native_learning: true
try_with_ai_prompts: 4
colearning_elements: ["conceptual_prompt", "teaching_commentary", "specification_challenge", "ai_tip"]
---

## Lesson 1: Understanding Modules and Imports

**CEFR Level**: A2 (Foundational)
**Time Estimate**: 55 minutes
**What You'll Learn**: Python code is organized into modules (files with reusable code). You'll learn how to bring that code into your scripts using imports, and explore three different import patterns to see when each one makes sense.

---

## What Is a Module? — Why Organization Matters

**The Problem**: Imagine writing a huge Python script with thousands of lines. Finding a specific function becomes hard. Sharing code with others becomes confusing. Reusing code in different projects requires copying and pasting.

**The Solution**: Organize code into modules. A module is simply a `.py` file containing Python code—functions, variables, classes. Instead of writing everything in one file, Python developers split code logically:

- `math_operations.py` — calculations
- `utilities.py` — helper functions
- `main.py` — main program

Python comes with built-in modules (like `math` and `random`) that are ready to use immediately.

### 💻 Code Idea: What a Module Is

Think of a module like a toolbox. The toolbox contains tools (functions), and you access them by opening the toolbox (importing the module).

```python
# This is what importing looks like:
import math

# Now you have access to all the tools in the math toolbox
result: float = math.sqrt(16)
print(f"Square root of 16: {result}")  # Output: 4.0
print(f"Pi value: {math.pi}")          # Output: 3.14159...
```

**Why this matters**: The `import math` statement brings the entire math module into your script. Everything the math module offers becomes available.

### 💬 AI Colearning Prompt

> Ask your AI: "What does a Python module look like? Show me the structure of a simple module file with 2-3 functions."

**Expected Understanding**: You see that a module is just a Python file with functions (and variables) defined in it. Nothing magical—just organized code.

### 🎓 Instructor Commentary

Syntax is cheap; organization is gold. The `import` statement is easy to write, but understanding **why** modules matter is the real skill. Modules let developers think in chunks: "Here's a file that handles math," "Here's a file that handles user input." This mental organization scales from small scripts to large applications. When you work with AI, clear module organization tells AI exactly where to find or modify code.

---

## Import Pattern 1: `import module_name` — Full Module Access

**What it does**: The statement `import math` brings the entire math module into your script. You access functions and variables using dot notation: `math.function_name()`.

### 💻 Code Idea: Accessing a Module's Functions

```python
import math

# Access functions and constants using the module name
result: float = math.sqrt(16)      # Access sqrt function
ceiling: int = math.ceil(3.2)      # Round up
floor_val: int = math.floor(3.8)   # Round down
power: float = math.pow(2, 8)      # 2 to the power of 8

print(f"Square root: {result}")     # 4.0
print(f"Ceiling of 3.2: {ceiling}") # 4
print(f"Floor of 3.8: {floor_val}") # 3
print(f"2^8: {power}")              # 256.0
```

**Why use this pattern?**
- **Clarity**: When you see `math.sqrt()`, you immediately know where `sqrt` comes from
- **Avoids naming conflicts**: If you write your own `sqrt` function, `math.sqrt` still refers to the module's version
- **Professional pattern**: This is how most Python code is written

**When to use**: Most of the time. Especially when a module name is short and you'll use multiple functions from it.

---

## Import Pattern 2: `from module import specific_function` — Direct Import

**What it does**: The statement `from math import sqrt` brings only the `sqrt` function into your script. You use it directly without the module prefix: just `sqrt()`, not `math.sqrt()`.

### 💻 Code Idea: Importing Specific Functions

```python
from math import sqrt, ceil, floor

# Use functions directly without module prefix
result: float = sqrt(25)   # No math. prefix needed
rounded_up: int = ceil(3.2)
rounded_down: int = floor(3.8)

print(f"Square root of 25: {result}")     # 5.0
print(f"Ceiling of 3.2: {rounded_up}")    # 4
print(f"Floor of 3.8: {rounded_down}")    # 3
```

**Why use this pattern?**
- **Brevity**: Shorter code (`sqrt()` instead of `math.sqrt()`)
- **When you use one function repeatedly**: Reduces typing

**Trade-off (be careful!)**
- **Less clarity**: Someone reading your code might wonder where `sqrt` comes from
- **Potential conflicts**: If you have your own `sqrt` variable, it shadows the imported one

**When to use**: When you're importing just 1-2 specific functions and you'll use them many times in your script.

---

## Import Pattern 3: `from module import function as alias` — Aliased Import

**What it does**: The statement `from math import sqrt as square_root` imports the function but renames it. You use the new name in your code.

### 💻 Code Idea: Using Aliases for Clarity

```python
from math import sqrt as square_root
import random as rnd

# Use the alias instead of the original function name
value: float = square_root(36)         # Clearer name (maybe for your domain)
roll: int = rnd.randint(1, 6)          # Short alias for frequently used module

print(f"Square root of 36: {value}")    # 6.0
print(f"Die roll: {roll}")              # Random number 1-6
```

**Why use aliases?**
- **Rename for clarity**: `square_root` might be clearer in your code than `sqrt`
- **Avoid naming conflicts**: If you have a variable named `datetime`, you can import the module as `from datetime import datetime as dt`
- **Shorthand for long names**: `import numpy as np` is industry standard

**When to use**: When a function name is unclear, or when you're avoiding a naming conflict, or when the module name is long.

---

## Using Built-In Modules — Practical Examples

### Math Module — Calculations

```python
import math

numbers: list[float] = [1.5, 2.7, 3.9]

# Built-in functions (work on any sequence)
max_val: float = max(numbers)          # Max value
min_val: float = min(numbers)          # Min value
sum_val: float = sum(numbers)          # Sum

# Math module functions (specialized math operations)
floor_val: int = math.floor(3.9)       # Round down
ceil_val: int = math.ceil(3.1)         # Round up
sqrt_val: float = math.sqrt(16)        # Square root

print(f"Max: {max_val}, Min: {min_val}, Sum: {sum_val}")
print(f"Floor(3.9): {floor_val}, Ceil(3.1): {ceil_val}")
print(f"Square root of 16: {sqrt_val}")
```

**Key Point**: The `max()` and `sum()` are Python built-ins (always available). The `math.floor()` and `math.sqrt()` are from the math module (must import first).

### Random Module — Non-Deterministic Operations

```python
import random

# Pick a random element from a list
options: list[str] = ["rock", "paper", "scissors"]
choice: str = random.choice(options)
print(f"Random choice: {choice}")

# Generate a random integer in a range
roll: int = random.randint(1, 6)       # Die roll (1-6)
print(f"Die roll: {roll}")

# Generate a random float (0.0 to 1.0)
random_value: float = random.random()
print(f"Random value: {random_value}")

# Shuffle a list in place
cards: list[str] = ["A", "2", "3", "4", "5"]
random.shuffle(cards)
print(f"Shuffled cards: {cards}")
```

**Why this matters**: The random module gives you non-deterministic behavior (unpredictable results). This is essential for games, simulations, and testing.

---

## Reading Module Documentation — How to Learn What a Module Offers

**The Reality**: You don't need to memorize every function in every module. Instead, learn how to **discover** what a module offers.

### 💻 Code Idea: Discovering Module Functions

```python
import math

# In Python interactive mode, you can run: help(math)
# This shows all available functions and their documentation

# Or in your script, you can list all functions:
# functions: list[str] = [name for name in dir(math) if not name.startswith('_')]
# print(functions)

# For now, know that:
# 1. Official Python docs (python.org) document all modules
# 2. Your AI companion can answer "What functions does the random module have?"
# 3. You can use help(math.sqrt) to read about a specific function

# Example: Ask your AI these questions
# "How do I generate a random choice from a list?"
# "What functions does the math module have for rounding?"
# "How do I shuffle a list randomly?"
```

**The key skill**: Being able to **ask the right questions** and **explore modules with AI** is more valuable than memorizing every function.

### 🚀 Specification Challenge

Think like a specification writer: You need code that picks a random element from a list. Describe what you want to your AI: *"I have a list of options and I want to pick one at random. Which module should I use? How would I import it?"*

Then tell your AI: *"Show me the code for this."* Notice how your clear description (specification) guides what AI generates.

### ✨ AI Tool Tip

Claude Code and Gemini CLI know all Python modules. Instead of guessing syntax, ask your AI:
- "How do I import the math module and calculate the square root of 25?"
- "Show me 3 useful functions from the random module"
- "What's the difference between these import styles?"

This is how professionals work—not from memory, but from clear thinking and AI partnership.

---

## Try With AI: Import Strategy Workshop

### Part 1: Organize Code Into Modules (Your Turn First)

**Before asking AI**, you have this scattered calculator code in one file:

```python
# messy_calculator.py - everything in one file!
import math

price = 100.0
tax_rate = 0.08
discount = 0.15

discounted = price * (1 - discount)
with_tax = discounted * (1 + tax_rate)
final = math.ceil(with_tax)

print(f"Final price: ${final}")

# ... more calculations
# ... validation functions
# ... display functions
# ... all mixed together
```

**Your task**: Design a module structure
- Sketch 2-3 module files (what would you name them?)
- For each module, list what code belongs there (calculations? validation? display?)
- Predict: Which import pattern would you use for each module and why?

Write your module organization plan before Part 2.

---

### Part 2: AI Explains Import Strategies (Discovery)

Share your design with AI:

> "I have calculator code scattered in one file. Here's my module organization plan: [paste your design]
>
> Help me understand import strategies:
> 1. Compare `import math` vs `from math import ceil` - which is better for my calculator and why?
> 2. Show me the difference between these three patterns with real examples: import X, from X import Y, from X import Y as Z
> 3. Create a decision guide: when should I use each pattern?
> 4. What's the danger of `from math import *`? (importing everything)"

**Your evaluation**:
- Test each import pattern AI shows you - run the code
- Try the `from math import *` pattern - what happens? Can you see why it's risky?
- Which pattern makes your code easiest to read?

---

### Part 3: Student Teaches AI (Name Collision Chaos)

Challenge AI with this scenario:

> "I'm building a data processing tool. Analyze this code:
> ```python
> from statistics import mean
> from numpy import mean  # Same name, different library!
>
> data = [10, 20, 30]
> result = mean(data)  # Which mean() gets called?
> ```
>
> For EACH of these questions:
> 1. Which `mean` function actually runs? Why?
> 2. How would you fix this to use BOTH libraries in the same file?
> 3. In a 1000-line project, how would you prevent this naming collision bug?
> 4. Show me three valid import patterns that avoid the collision"

**Your debugging**:
- Create a file with this collision yourself
- Add print statements to trace which function runs
- Try AI's suggested fixes - do they all work?

---

### Part 4: Build Import Organization Template (Convergence)

Create a reusable import organization pattern:

> "Create a Python project template showing professional import organization:
>
> **data_processor/**
> - `calculations.py` - math operations using `math` module
> - `validators.py` - data validation (import using `from X import Y` pattern)
> - `formatters.py` - output formatting using aliases
> - `main.py` - imports all custom modules and orchestrates them
>
> Requirements:
> 1. Each file uses a DIFFERENT import pattern (demonstrate all three)
> 2. Include comments explaining why each pattern was chosen
> 3. Add a `module_guide.md` documenting your import decisions
> 4. Demonstrate circular import prevention (if main imports calculations, calculations shouldn't import main)"

**Refinement**:
> "Add this challenge: create a `config.py` module that's imported by multiple files. Show me how to structure it to avoid repeated initialization. Bonus: use `if __name__ == '__main__'` to demonstrate the difference between importing vs running."

---

**Time**: 30 minutes
**Outcome**: You can organize messy code into clean modules, choose import patterns that optimize for clarity and maintainability, prevent naming collisions in large projects, and create professional project structures.
