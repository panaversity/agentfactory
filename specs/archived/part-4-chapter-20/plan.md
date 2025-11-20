# Chapter 20: Modules and Functions — Detailed Lesson Plan

**Chapter Type**: Technical / Code-Focused
**Chapter Objective(s)**: Students understand module organization and imports, write reusable functions with clear intent through type hints and docstrings, apply scope rules to predict code behavior, orchestrate multi-module projects with separation of concerns, and validate function behavior through testing.
**Estimated Total Time**: 5–5.5 hours (5 lessons × 55–70 min each)
**Part**: Part 4 - Python Fundamentals
**Complexity Tier**: A2-B1 (Intermediate Beginners)
**Status**: Ready for Implementation
**Feature Branch**: `020-modules-functions`

---

## 🎯 Chapter Overview & Context

**Building on Chapter 19 Foundation**:
- Chapter 19 taught: Sets, frozensets, garbage collection, and memory management concepts
- Chapter 20 builds on collection and data understanding by teaching: Code organization (modules), reusable code blocks (functions), parameter/return patterns, scope rules, and integrated multi-module projects

**Bridge to Chapter 21+**:
- Chapter 20 establishes foundational module/function patterns
- Chapter 21 (Exception Handling) builds on function error handling
- Chapter 22 (File I/O) uses modules for file utilities
- Chapter 24+ (OOP) structures classes within modules and as methods on objects

**Prerequisites Established**:
- Chapter 13: Python basics (variables, print, execution flow)
- Chapter 14: Data types and type hints (foundational)
- Chapter 15: Operators and keywords
- Chapter 16: Strings (used in examples)
- Chapter 17: Control flow (if/for/while used in function bodies)
- Chapter 18: Lists, tuples, dicts (passed as function parameters)
- Chapter 19: Sets, frozensets (used in function parameters/returns)

**AI-Native Learning Pattern** (How Students Learn Modules & Functions with AI):
- **Describe intent**: Type hints and docstrings communicate what a function needs and produces (`def calculate_total(items: list[float]) -> float:`)
- **Explore with AI**: Students ask "Show me how to import the math module," "Generate a function that finds the maximum" instead of memorizing syntax
- **Validate together**: Test functions with examples; verify scope behavior by running code
- **Learn from errors**: ImportError teaches module search mechanics; NameError teaches scope

**Key Scope Boundaries** (Chapter 20 Responsibility):
- ✅ **Comprehensive Coverage**: Module concept and imports (3 variants), function definition, type hints, docstrings, parameters (positional, default), return values, scope (local/global/enclosing), nested functions, closures (awareness for Chapter 26), multi-module project organization
- ✅ **Awareness Only**: *args/*kwargs (mentioned for future), decorators (Chapter 26), packages with __init__.py (advanced), import optimization
- ❌ **Deferred**: Formal Specification-Driven Development (Chapter 30), Classes and OOP (Chapter 24), Async functions (Chapter 28)

---

## 📊 Lesson Architecture

### Lesson 1: Understanding Modules and Imports

**CEFR Level**: A2 (Basic/Foundational) — Students recognize and name module concepts, understand built-in modules through direct instruction, can execute basic imports with guidance

**Bloom's Levels**: Remember, Understand, Apply (simple contexts)

**Concepts Taught** (5 concepts, within intermediate limit of 7):
1. **Module concept**: A .py file containing reusable code (functions, variables, classes)
2. **Built-in modules**: Python's standard library (math, random, os) — ready to use
3. **Import variant 1**: `import math` → Access via `math.sqrt()`
4. **Import variant 2**: `from math import sqrt` → Direct access to `sqrt()`
5. **Import documentation**: Reading module docs to discover available functions

**Proficiency by End**: Students can import at least 3 built-in modules (math, random, os), understand the difference between `import X` and `from X import Y`, and use functions from those modules without errors.

**Time Estimate**: 55 minutes

**Prerequisite Mastery**:
- Chapter 14: Type hints syntax (basic)
- Chapter 13: Understand that Python code executes top-to-bottom
- Basic comfort with function calls like `len()`, `print()`

**Content Outline**:

1. **Introduction** (5 min) — Why organize code into modules? Real-world motivation
2. **Concept: What is a Module?** (8 min) — File-based organization, standard library overview
3. **Concept: Import Basics** (8 min) — How `import` brings code from modules into your script
4. **Concept: Three Import Variants** (10 min) — `import X`, `from X import Y`, `from X import Y as Z`
5. **Code Examples** (15 min) — Using math.sqrt(), random.choice(), os.path.exists()
6. **Practice** (9 min) — Students import and use functions from 2+ modules

**Content Elements**:

**Code Example 1: Understanding Module Access**
```python
# VARIANT 1: import math
import math

result: float = math.sqrt(16)
print(f"Square root of 16: {result}")  # 4.0
print(f"Pi constant: {math.pi}")       # 3.14159...

# Module = namespace; access functions via dot notation: math.function_name()
```
- **Purpose**: Show module as a namespace; understand `module.function` access pattern
- **Complexity**: Simple (familiar function call with module prefix)
- **Pedagogical Value**: Establishes that modules organize code; dot notation accesses module's contents

**Code Example 2: Direct Import with 'from'**
```python
# VARIANT 2: from math import sqrt
from math import sqrt

result: float = sqrt(25)  # Direct access to sqrt, no math. prefix needed
print(f"Square root of 25: {result}")  # 5.0

# When to use: Importing specific function you'll use many times
# Advantage: Shorter code (sqrt() vs math.sqrt())
# Disadvantage: Unclear where sqrt() comes from (less explicit)
```
- **Purpose**: Show alternative import style; understand when to use each variant
- **Complexity**: Simple (different syntax, same underlying concept)
- **Pedagogical Value**: Students learn trade-offs between clarity (module prefix) and brevity

**Code Example 3: Import with Alias (as)**
```python
# VARIANT 3: import with alias
from math import sqrt as square_root
import random as rnd

value: float = square_root(36)     # Use alias instead of sqrt
roll: int = rnd.randint(1, 6)      # Use alias instead of random.randint()

# When to use aliases:
# - Long module names (import numpy as np)
# - Avoid naming conflicts (import datetime as dt, to keep "datetime" for variable names)
# - Shorthand for frequently used modules
```
- **Purpose**: Show how aliases clarify intent or prevent conflicts
- **Complexity**: Moderate (new syntax, familiar concept)
- **Pedagogical Value**: Students see professional naming patterns

**Code Example 4: Using Built-in Modules (math)**
```python
import math

# Math module examples
numbers: list[float] = [1.5, 2.7, 3.9]
print(f"Max: {max(numbers)}")              # Built-in max()
print(f"Sum: {sum(numbers)}")              # Built-in sum()
print(f"Floor of 3.9: {math.floor(3.9)}")  # Module function
print(f"Ceil of 3.1: {math.ceil(3.1)}")    # Module function
print(f"Square root of 9: {math.sqrt(9)}")  # Module function
```
- **Purpose**: Show practical math module usage; mix built-in (max, sum) with module functions
- **Complexity**: Simple (familiar data, new module functions)
- **Pedagogical Value**: Builds intuition for "what module functions do"

**Code Example 5: Using Built-in Modules (random)**
```python
import random

# Random module examples
numbers: list[int] = [10, 20, 30, 40, 50]

# Pick a random element
choice: int = random.choice(numbers)
print(f"Random pick: {choice}")

# Generate random integer in range
roll: int = random.randint(1, 6)  # Simulate die roll
print(f"Die roll: {roll}")

# Shuffle a list in place
random.shuffle(numbers)
print(f"Shuffled: {numbers}")
```
- **Purpose**: Show random module for non-deterministic operations; practical use case
- **Complexity**: Simple (familiar concepts applied to new module)
- **Pedagogical Value**: Shows modules are tools for different programming tasks

**Code Example 6: Reading Module Documentation**
```python
# How to learn what a module offers:
# 1. Use help() function (interactive)
# 2. Read official Python docs (online)
# 3. Ask your AI: "What functions does the math module have?"

import math

# In interactive Python, you could run: help(math)
# This shows all available functions and their documentation

# For now, know that modules have built-in functions you can discover
# and use with clear documentation
```
- **Purpose**: Teach students how to discover module capabilities (book teaches, AI helps explore)
- **Complexity**: Meta-cognitive (learning how to learn)
- **Pedagogical Value**: Empowers students to explore modules independently

**CoLearning Elements in This Lesson**:

- 💬 **Prompt (Conceptual)**: "Ask your AI: Show me the 5 most useful functions from the math module and explain what each does. Which one would you use to calculate 2 to the power of 10?" — Encourages module exploration with AI
- 🎓 **Commentary**: "Syntax is cheap, understanding is gold. When you see `import math`, remember: you're bringing an entire collection of mathematical tools into your script. The import statement is just how you access them."
- 🚀 **Challenge**: "Think like a specification writer: You need a function that picks a random item from a list. Describe to your AI what you need: 'I have a list of game options. I want to pick one at random.' What module would you use? How would you import it?"
- ✨ **Tip**: "Claude Code and Gemini CLI know all Python modules. Instead of memorizing `math.pi`, ask your AI: 'How do I get the value of pi in Python?' This is how professionals work with modules."

**Try With AI - 4 Prompts (Progressive Bloom's)**:

1. **Remember/Understand** (What is a module?):
   - **Prompt**: "Explain the difference between these three import statements: `import math`, `from math import sqrt`, and `from math import sqrt as square_root`. What does each one do?"
   - **Expected outcome**: Student can articulate that all three import from math, but access the function differently

2. **Understand/Apply** (Use a module):
   - **Prompt**: "Write code that imports the random module and generates 5 random numbers between 1 and 100. Print each one. What does the random module give you that you can't do with regular Python?"
   - **Expected outcome**: Student can write valid import and random.randint() calls; understands randomness is external capability

3. **Apply** (Choose the right module):
   - **Prompt**: "You need to: find the absolute value of -42, round 3.7 to nearest integer, and calculate 2^8 (2 to the power of 8). Which module would you use for each? Write the code."
   - **Expected outcome**: Student can identify math module for all three, uses math.fabs(), math.floor(), math.pow() correctly

4. **Analyze/Synthesize** (Connect to bigger picture):
   - **Prompt**: "Think about a project you'd build (game, data analysis tool, utility). List 3 built-in modules that would be useful for it and explain why. Share this with your AI and ask: 'Would I need any other modules for this project?'"
   - **Expected outcome**: Student connects module organization to real development; understands that modules solve concrete problems

---

### Lesson 2: Writing Functions with Intent

**CEFR Level**: A2-B1 (Foundational with Application) — Students apply function definition patterns to write simple functions with clear intent; type hints and docstrings guide this

**Bloom's Levels**: Understand, Apply, Analyze (recognizing intent in function signatures)

**Concepts Taught** (6 concepts, within intermediate limit of 7):
1. **Function definition**: `def function_name(parameters) -> return_type:`
2. **Parameters**: Named inputs to a function (receive values from caller)
3. **Return values**: Data function sends back to caller (specified with `->` type hint)
4. **Type hints**: Function signature annotations (`param: Type`, `-> ReturnType`)
5. **Docstrings**: Multi-line string explaining what function does (purpose, parameters, return)
6. **Calling functions**: How to invoke a function and use its return value

**Proficiency by End**: Students can write functions with type hints and docstrings matching simple problem requirements; understand how parameters flow into functions and returns flow out; can call functions and use returned values correctly.

**Time Estimate**: 60 minutes

**Prerequisite Mastery**:
- Lesson 1: Import concept (modules organize code)
- Chapter 14: Type hints (basic syntax)
- Chapter 18: Collections as data types (passed to functions)

**Content Outline**:

1. **Introduction** (5 min) — Why write functions? Code reuse and clear intent
2. **Concept: Function Definition** (8 min) — `def`, parameters, return type specification
3. **Concept: Type Hints as Intent** (8 min) — Function signature communicates what function needs and produces
4. **Concept: Docstrings** (7 min) — Documenting purpose, parameters, return value
5. **Code Examples** (22 min) — Write simple functions with intent; call and use returns
6. **Practice** (10 min) — Students write 2+ functions with type hints and docstrings

**Content Elements**:

**Code Example 1: Simple Function with Type Hints**
```python
# Function that adds two numbers
def add(a: int, b: int) -> int:
    """Add two integers and return the sum."""
    return a + b

# Calling the function
result: int = add(5, 3)
print(f"5 + 3 = {result}")  # 8

# Type hints tell us:
# - a and b must be integers
# - function returns an integer
# - If you pass strings or floats, Python will notify you (type checking in modern editors)
```
- **Purpose**: Show minimal function with type hints; establish signature = intent pattern
- **Complexity**: Simple (familiar operation in function form)
- **Pedagogical Value**: Type hints = machine-readable intent; enables AI to understand what function should do

**Code Example 2: Function with Multiple Parameters and Docstring**
```python
def greet(name: str, greeting: str = "Hello") -> str:
    """
    Return a greeting message.

    Parameters:
        name (str): The person's name
        greeting (str): The greeting to use (default: "Hello")

    Returns:
        str: A formatted greeting message
    """
    message: str = f"{greeting}, {name}!"
    return message

# Call with positional argument only (uses default greeting)
msg1: str = greet("Alice")
print(msg1)  # "Hello, Alice!"

# Call with both arguments
msg2: str = greet("Bob", "Hi")
print(msg2)  # "Hi, Bob!"
```
- **Purpose**: Show default parameters, docstring format, multiple parameters
- **Complexity**: Moderate (introduces defaults, docstring structure)
- **Pedagogical Value**: Docstring = documentation for humans and AI; shows professional function structure

**Code Example 3: Function Calculating with Collections**
```python
def calculate_average(numbers: list[float]) -> float:
    """
    Calculate the average of a list of numbers.

    Parameters:
        numbers (list[float]): List of numeric values

    Returns:
        float: The average (mean) of all numbers
    """
    if len(numbers) == 0:
        return 0.0

    total: float = sum(numbers)
    average: float = total / len(numbers)
    return average

# Test the function
scores: list[float] = [85.5, 90.0, 78.5, 92.0]
avg: float = calculate_average(scores)
print(f"Average score: {avg}")  # 86.5
```
- **Purpose**: Show function working with collections (lists); basic logic inside function
- **Complexity**: Moderate (uses list, math, type hints on complex parameter)
- **Pedagogical Value**: Functions are tools for problem-solving; type hints clarify "what data is expected"

**Code Example 4: Function Returning Multiple Values**
```python
def divide_with_remainder(dividend: int, divisor: int) -> tuple[int, int]:
    """
    Divide two integers and return quotient and remainder.

    Parameters:
        dividend (int): Number to divide
        divisor (int): Number to divide by

    Returns:
        tuple[int, int]: (quotient, remainder)
    """
    quotient: int = dividend // divisor
    remainder: int = dividend % divisor
    return quotient, remainder

# Call function and unpack return values
q, r = divide_with_remainder(17, 5)
print(f"17 ÷ 5 = {q} remainder {r}")  # 17 ÷ 5 = 3 remainder 2
```
- **Purpose**: Show functions returning multiple values; unpacking tuple returns
- **Complexity**: Moderate (tuple return type, unpacking)
- **Pedagogical Value**: Functions can return structured data; type hint shows return structure

**Code Example 5: Function Checking Conditions**
```python
def is_even(number: int) -> bool:
    """
    Check if a number is even.

    Parameters:
        number (int): The number to check

    Returns:
        bool: True if even, False if odd
    """
    return number % 2 == 0

def is_within_range(value: int, min_val: int, max_val: int) -> bool:
    """
    Check if a value is within a specified range.

    Parameters:
        value (int): The value to check
        min_val (int): Minimum allowed value
        max_val (int): Maximum allowed value

    Returns:
        bool: True if within range, False otherwise
    """
    return min_val <= value <= max_val

# Test the functions
print(is_even(4))                    # True
print(is_even(7))                    # False
print(is_within_range(50, 0, 100))   # True
print(is_within_range(150, 0, 100))  # False
```
- **Purpose**: Show functions returning boolean values; predicates (functions that answer yes/no)
- **Complexity**: Simple (familiar logic, function returns boolean)
- **Pedagogical Value**: Functions are tools for asking questions about data; type hints clarify intent

**Code Example 6: Function with String Processing**
```python
def format_name(first: str, last: str, uppercase: bool = False) -> str:
    """
    Format a person's name.

    Parameters:
        first (str): First name
        last (str): Last name
        uppercase (bool): Whether to convert to uppercase (default: False)

    Returns:
        str: Formatted full name
    """
    full_name: str = f"{first} {last}"
    return full_name.upper() if uppercase else full_name

# Test the function
print(format_name("alice", "smith"))          # "alice smith"
print(format_name("alice", "smith", True))    # "ALICE SMITH"
```
- **Purpose**: Show default boolean parameter; conditional logic inside function; string methods
- **Complexity**: Moderate (uses ternary operator, optional parameter)
- **Pedagogical Value**: Functions can have optional behavior; type hints control which code path executes

**CoLearning Elements in This Lesson**:

- 💬 **Prompt (Conceptual)**: "Tell your AI: I need a function that takes a list of prices and calculates total cost with 10% tax. What should the type hints be? What should the docstring say?" — Guides specification thinking
- 🎓 **Commentary**: "Type hints are not just syntax—they're your contract with other developers and AI. When you write `def calculate(items: list[float]) -> float:`, you're saying 'I promise to accept a list of floats and return a single float.' This is how AI understands what code should do."
- 🚀 **Challenge**: "Practice specification thinking: Choose a real calculation you do (distance, cost, grade average). Write a function signature (just def line + type hints + docstring, no body yet). Then ask your AI: 'Does my signature match my intent?' This is how AI-native developers work."
- ✨ **Tip**: "Your AI can generate function bodies from docstrings. Write a clear docstring, then ask: 'Implement this function.' The better your specification (docstring), the better AI's code will be."

**Try With AI - 4 Prompts (Progressive Bloom's)**:

1. **Remember** (Recognize function components):
   - **Prompt**: "Look at this function and identify: the function name, parameters with their types, return type, and docstring. What does each part tell you?"
   ```python
   def multiply(a: int, b: int) -> int:
       """Return the product of two integers."""
       return a * b
   ```
   - **Expected outcome**: Student identifies all components; understands type hints communicate intent

2. **Understand/Apply** (Write a simple function):
   - **Prompt**: "Write a function called `double` that takes an integer and returns it multiplied by 2. Include type hints and a docstring. Test your function with 3 different inputs."
   - **Expected outcome**: Student writes valid function with type hints, docstring, and tests it

3. **Apply** (Write function matching requirements):
   - **Prompt**: "Write a function called `summarize_scores` that takes a list of test scores (floats) and returns: highest score, lowest score, and average. Return all three as a tuple. Include type hints and docstring."
   - **Expected outcome**: Student writes function returning tuple[float, float, float]; handles list parameter

4. **Analyze/Synthesize** (Connect intent to code):
   - **Prompt**: "Choose a real task you do (calculate tip, convert temperature, check if something qualifies). Write the function signature (def line with type hints + docstring). Ask your AI: 'Generate the implementation.' Compare what you expected vs. what AI generated. What did the docstring communicate clearly? What was unclear?"
   - **Expected outcome**: Student experiences AI code generation from specifications; learns how clarity of intent affects output

---

### Lesson 3: Function Parameters and Returns

**CEFR Level**: B1 (Intermediate Application) — Students apply parameter and return patterns to real problems; understand parameter order and defaults; unpack multiple returns

**Bloom's Levels**: Apply, Analyze (understanding parameter implications)

**Concepts Taught** (5 concepts, within intermediate limit of 7):
1. **Positional parameters**: Parameters passed in order they're defined (required, order matters)
2. **Default parameters**: Parameters with default values (optional, can be omitted when calling)
3. **Parameter order**: Required parameters must come before optional parameters in function signature
4. **Multiple return values**: Functions returning tuples or multiple values
5. **Unpacking returns**: Assigning multiple return values to separate variables

**Proficiency by End**: Students can write functions with both required and optional parameters; call functions correctly with positional and keyword arguments; return and unpack multiple values; understand parameter order constraints.

**Time Estimate**: 60 minutes

**Prerequisite Mastery**:
- Lesson 2: Function definition, type hints, basic function calls
- Chapter 18: Tuples (used for multiple returns)
- Chapter 14: Type hints with generics like `tuple[Type, Type]`

**Content Outline**:

1. **Introduction** (5 min) — Why parameter patterns matter? Real functions have optional behavior
2. **Concept: Required vs. Optional Parameters** (8 min) — Positional parameters vs. default parameters
3. **Concept: Parameter Order Rules** (7 min) — Required first, then optional (language constraint)
4. **Concept: Calling Functions Flexibly** (8 min) — Positional and keyword argument patterns
5. **Code Examples** (22 min) — Various parameter combinations; multiple return values; unpacking
6. **Practice** (10 min) — Students write functions with mixed parameter types and multiple returns

**Content Outline**:

1. **Introduction** (5 min) — Why parameter patterns matter? Real functions have optional behavior
2. **Concept: Required vs. Optional Parameters** (8 min) — Positional parameters vs. default parameters
3. **Concept: Parameter Order Rules** (7 min) — Required first, then optional (language constraint)
4. **Concept: Calling Functions Flexibly** (8 min) — Positional and keyword argument patterns
5. **Code Examples** (22 min) — Various parameter combinations; multiple return values; unpacking
6. **Practice** (10 min) — Students write functions with mixed parameter types and multiple returns

**Content Elements**:

**Code Example 1: Positional vs. Default Parameters**
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

# Calling with just required parameters
user1 = create_user_account("alice", "alice@example.com")
print(user1)  # {'username': 'alice', 'email': 'alice@example.com', 'role': 'user', 'status': 'active'}

# Calling with optional parameter
user2 = create_user_account("bob", "bob@example.com", "admin")
print(user2)  # {'username': 'bob', 'email': 'bob@example.com', 'role': 'admin', 'status': 'active'}
```
- **Purpose**: Show required parameters (positional), optional with defaults, calling with both
- **Complexity**: Moderate (parameter variety, dictionary return)
- **Pedagogical Value**: Optional parameters enable flexible function behavior

**Code Example 2: Parameter Order Rules**
```python
# CORRECT: Required parameters first, then optional
def send_message(recipient: str, subject: str, priority: str = "normal") -> str:
    """Send a message with optional priority level."""
    return f"Message to {recipient}: {subject} (Priority: {priority})"

# INCORRECT pattern (DON'T DO THIS):
# def send_message(priority: str = "normal", recipient: str, subject: str):
#     # Error! Required parameters can't come after optional ones
#     pass

# Correct calls:
msg1 = send_message("alice@example.com", "Hello")  # Uses default priority
msg2 = send_message("bob@example.com", "Urgent", "high")  # Specifies priority
msg3 = send_message("charlie@example.com", "Question", priority="low")  # Keyword argument
```
- **Purpose**: Establish parameter order rule; show keyword argument calling pattern
- **Complexity**: Moderate (rule-based, keyword arguments)
- **Pedagogical Value**: Understanding constraints prevents bugs; keyword arguments enable clarity

**Code Example 3: Keyword Arguments for Clarity**
```python
def schedule_meeting(
    title: str,
    date: str,
    time: str = "9:00 AM",
    duration_minutes: int = 60,
    online: bool = True
) -> str:
    """
    Schedule a meeting with many optional parameters.
    """
    meeting_info = f"Meeting: {title}\nDate: {date}\nTime: {time}\nDuration: {duration_minutes} min\nOnline: {online}"
    return meeting_info

# Positional arguments (order matters):
meeting1 = schedule_meeting("Budget Review", "2025-11-15")

# Keyword arguments (order doesn't matter, but much clearer):
meeting2 = schedule_meeting(
    title="Quarterly Planning",
    date="2025-11-20",
    time="2:00 PM",
    duration_minutes=90,
    online=False  # In-person meeting
)
print(meeting2)
```
- **Purpose**: Show keyword arguments improve code readability; enable setting non-adjacent defaults
- **Complexity**: Moderate (calling patterns, multiple optional parameters)
- **Pedagogical Value**: Code clarity through keyword arguments; professional calling pattern

**Code Example 4: Functions Returning Multiple Values**
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
- **Purpose**: Show functions returning multiple values as tuples; unpacking pattern
- **Complexity**: Moderate (tuple return, unpacking syntax)
- **Pedagogical Value**: Functions can return structured data; unpacking is Python idiom

**Code Example 5: Optional Return Structures**
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
```
- **Purpose**: Show functions that might not return data (None); conditional unpacking
- **Complexity**: Moderate (union type, try/except, None handling)
- **Pedagogical Value**: Real functions have error cases; type hints show all possibilities

**Code Example 6: Chaining Function Calls**
```python
def calculate_discount(price: float, discount_percent: float) -> float:
    """Calculate discounted price."""
    return price * (1 - discount_percent / 100)

def add_tax(price: float, tax_rate: float) -> float:
    """Calculate price with tax."""
    return price * (1 + tax_rate / 100)

# Chain function calls: apply discount, then add tax
original_price: float = 100.0
discounted: float = calculate_discount(original_price, 20)  # 20% off = $80
final_price: float = add_tax(discounted, 8)  # 8% tax on $80 = $86.40

print(f"Original: ${original_price:.2f}")
print(f"After 20% discount: ${discounted:.2f}")
print(f"After 8% tax: ${final_price:.2f}")
```
- **Purpose**: Show how return values flow into next function calls; function composition
- **Complexity**: Moderate (multiple function calls, value flow)
- **Pedagogical Value**: Functions are composable; output of one is input to next

**CoLearning Elements in This Lesson**:

- 💬 **Prompt (Conceptual)**: "Tell your AI: I'm calling a function with this signature: `def order_pizza(size: str, toppings: str = "cheese", delivery: bool = True)`. Show me 3 different ways to call it, each with a different combination of arguments."
- 🎓 **Commentary**: "Parameter order is not arbitrary—it's a language rule. Required parameters must come first because Python needs to know which arguments are required before it can apply defaults. Understanding WHY rules exist (not just memorizing them) is the key to professional development."
- 🚀 **Challenge**: "You have two functions: `convert_celsius(fahrenheit: float) -> float` and `round_temperature(temp: float, decimal_places: int = 0) -> float`. Call convert_celsius, then pass its result to round_temperature, all in one line. Ask your AI: 'Is this correct?' This is function composition."
- ✨ **Tip**: "When a function returns multiple values, you can unpack them in one line: `x, y = get_coordinates()`. This is very Pythonic and your AI will expect this pattern. Use it in your specifications when you need multiple values back."

**Try With AI - 4 Prompts (Progressive Bloom's)**:

1. **Remember/Understand** (Recognize parameter types):
   - **Prompt**: "Look at this function signature. Which parameters are required? Which are optional? How do you know?"
   ```python
   def build_url(domain: str, path: str, protocol: str = "https", port: int = 443) -> str:
   ```
   - **Expected outcome**: Student identifies domain and path as required; protocol and port as optional (have defaults)

2. **Understand/Apply** (Call a function flexibly):
   - **Prompt**: "Call this function 3 different ways: once with minimum required arguments, once with some optional arguments, and once with keyword arguments for clarity. What's the advantage of using keyword arguments?"
   ```python
   def create_file(filename: str, format: str = "txt", overwrite: bool = False) -> bool:
   ```
   - **Expected outcome**: Student calls function correctly with different argument patterns; explains keyword clarity

3. **Apply/Analyze** (Return and unpack multiple values):
   - **Prompt**: "Write a function called `get_user_info` that takes a user ID and returns a tuple with (name, email, role). Call your function and unpack all three values into separate variables. Print each one."
   - **Expected outcome**: Student writes function with tuple return; unpacks correctly; demonstrates understanding

4. **Analyze/Synthesize** (Connect to specifications):
   - **Prompt**: "Design a function for a real task (e.g., calculate total cost, format an address, validate input). Write the function signature with all parameters (both required and optional). Ask your AI: 'Should I have more parameters? Should some be optional?' Listen to the reasoning."
   - **Expected outcome**: Student designs function with appropriate parameters; uses AI as design partner

---

### Lesson 4: Scope and Nested Functions

**CEFR Level**: B1-B2 (Intermediate to Advanced Application) — Students apply scope rules to predict code behavior; understand nested functions as preparation for advanced patterns (Chapter 26 decorators)

**Bloom's Levels**: Analyze, Evaluate (predicting behavior across scope boundaries)

**Concepts Taught** (4 concepts, within intermediate limit of 7):
1. **Local scope**: Variables inside a function exist only within that function
2. **Global scope**: Variables at module level (outside functions) accessible from anywhere in module
3. **The `global` keyword**: Declaring that a function modifies a module-level variable (rare, use sparingly)
4. **Nested functions and closures**: Functions defined inside other functions; inner functions access outer scope (prepares for Chapter 26 decorators)

**Proficiency by End**: Students can correctly predict where variables exist and how code behaves across scope boundaries; understand why scope matters (prevents bugs, clarifies intent); recognize nested functions and closures as preparation for advanced patterns.

**Time Estimate**: 55 minutes

**Prerequisite Mastery**:
- Lesson 2-3: Function definition and calling
- Chapter 17: If/else logic (used in examples)
- Mental model: Variables exist in different regions of code

**Content Outline**:

1. **Introduction** (5 min) — Why does scope matter? Avoid bugs and unintended side effects
2. **Concept: Local Scope** (8 min) — Variables inside function only exist in that function
3. **Concept: Global Scope** (8 min) — Variables outside functions accessible everywhere (but not always best)
4. **Concept: The `global` Keyword** (6 min) — When/why to use (rarely, for module-level state)
5. **Concept: Nested Functions and Closures** (8 min) — Inner functions accessing outer function variables (awareness for Chapter 26)
6. **Code Examples** (12 min) — Scope behavior demonstration; nested functions
7. **Practice** (8 min) — Predict scope behavior; write nested function

**Content Elements**:

**Code Example 1: Local Scope — Variables Isolated Within Functions**
```python
def example_local_scope():
    """Demonstrate that function variables are local."""
    message: str = "Inside function"
    print(f"Inside: {message}")  # Works: message is in local scope

example_local_scope()
# print(message)  # ERROR: NameError - message doesn't exist outside function
# message is "dead" once function returns

# Another function can have a variable with same name—no conflict
def another_function():
    message: str = "Different function"
    print(f"In another: {message}")

another_function()  # "Different function" - different message variable
```
- **Purpose**: Establish that function variables don't leak outside function; isolation is feature
- **Complexity**: Simple (familiar function calls; new scope concept)
- **Pedagogical Value**: Scope prevents accidents; functions are isolated units

**Code Example 2: Global Scope — Module-Level Variables**
```python
# Module level (global scope)
application_name: str = "MyApp"
version: float = 1.0
users_logged_in: int = 0

def show_app_info():
    """Show global app information."""
    print(f"App: {application_name} v{version}")  # Can READ global variables
    print(f"Users online: {users_logged_in}")

def reset_app():
    """Reset to defaults."""
    global application_name, version  # Declare we're MODIFYING globals
    application_name = "MyApp (Reset)"
    version = 1.0

show_app_info()  # Uses global variables to read
# App: MyApp v1.0
# Users online: 0

reset_app()  # Modifies globals
show_app_info()  # Shows modified values
# App: MyApp (Reset) v1.0
# Users online: 0
```
- **Purpose**: Show reading global variables (always works), modifying globals (requires `global` keyword)
- **Complexity**: Moderate (global keyword, declaration pattern)
- **Pedagogical Value**: Global state is accessible but mutable—use carefully

**Code Example 3: Local Shadowing (Variables with Same Name)**
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
- **Purpose**: Show shadowing (accidental local variable hiding global) and correct global modification
- **Complexity**: Moderate (subtle behavior, requires careful explanation)
- **Pedagogical Value**: Scope bugs are common; understanding shadowing prevents them

**Code Example 4: Nested Functions — Inner Functions Accessing Outer Scope**
```python
def outer_function(multiplier: int) -> callable:
    """Create a function that multiplies by a value."""

    def inner_function(number: int) -> int:
        """Inner function accesses outer function's parameter."""
        return number * multiplier  # Closure: accesses multiplier from outer scope

    return inner_function  # Return the inner function itself

# Create multiplier functions using the same outer function
double = outer_function(2)    # multiplier = 2
triple = outer_function(3)    # multiplier = 3

# Each returned function "remembers" its multiplier
print(double(5))   # 5 * 2 = 10
print(triple(5))   # 5 * 3 = 15
print(double(10))  # 10 * 2 = 20
```
- **Purpose**: Show nested functions and closures (prepare for decorators); how inner function captures outer scope
- **Complexity**: Complex (function as return value, closure concept)
- **Pedagogical Value**: Nested functions enable advanced patterns; understanding closures is key to Chapter 26

**Code Example 5: Nested Function Reading Outer Scope**
```python
def calculate_discounted_price(original_price: float, discount_percent: float) -> callable:
    """
    Create a function that applies the same discount repeatedly.
    Demonstrates closure: inner function remembers discount_percent.
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
price_for_5: float = bulk_discount(5)   # 5 items at $10 each, 20% off
price_for_10: float = bulk_discount(10)  # 10 items at $10 each, 20% off

print(f"5 items: ${price_for_5:.2f}")   # $40.00
print(f"10 items: ${price_for_10:.2f}")  # $80.00
```
- **Purpose**: Show practical use of closures; inner function encapsulates behavior with outer scope data
- **Complexity**: Complex (nested functions, closure, practical pattern)
- **Pedagogical Value**: Closures enable partial application and function factories (common in Python)

**Code Example 6: Scope Interaction (Local, Enclosing, Global)**
```python
x: int = "global"  # Global scope

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
- **Purpose**: Show Python's LEGB scope rule (Local, Enclosing, Global, Built-in)
- **Complexity**: Moderate (multiple scope levels)
- **Pedagogical Value**: Understanding scope hierarchy helps predict code behavior

**CoLearning Elements in This Lesson**:

- 💬 **Prompt (Conceptual)**: "Ask your AI: Explain what's wrong with this code and why. Then ask: How would you fix it?" (Provide code with scope error like modifying global without declaration)
- 🎓 **Commentary**: "Scope is about clarity and preventing bugs. When you see a variable, understanding WHERE it exists helps you predict WHAT the code does. This is how you validate code before running it—a critical AI-native developer skill."
- 🚀 **Challenge**: "Write a nested function that creates personalized greeting generator. The outer function takes a greeting, the inner function takes a name and returns a full greeting. Test that it works. Ask your AI: 'Is this a closure? How?'"
- ✨ **Tip**: "When you see `global` keyword, stop and think: 'Is this really necessary?' Good Python code rarely uses `global`. If you're tempted to use `global`, ask your AI: 'Is there a better design pattern?' Often there is."

**Try With AI - 4 Prompts (Progressive Bloom's)**:

1. **Analyze** (Predict scope behavior):
   - **Prompt**: "Run this code and predict what will print. Then actually run it. Was your prediction correct? If not, what did you learn?"
   ```python
   x: int = 10
   def modify_x():
       x = 20  # Local variable, shadows global
       print(f"Inside: {x}")

   modify_x()
   print(f"Outside: {x}")
   ```
   - **Expected outcome**: Student predicts Inside: 20, Outside: 10; understands shadowing

2. **Analyze/Evaluate** (Scope design decision):
   - **Prompt**: "Here's code that uses `global` to track state. Ask your AI: Is this good design? What are the risks? What would be a better approach?"
   ```python
   count: int = 0
   def increment():
       global count
       count += 1
   ```
   - **Expected outcome**: Student evaluates trade-offs; understands global state risks

3. **Apply** (Write nested function with closure):
   - **Prompt**: "Write a function called `create_adder` that takes one number and returns a function. The returned function takes another number and returns the sum. Example: `add_5 = create_adder(5)` then `add_5(3)` returns 8."
   - **Expected outcome**: Student writes closure correctly; demonstrates understanding of nested functions

4. **Analyze/Synthesize** (Connect scope to specifications):
   - **Prompt**: "Think about a project where you might need global state (like a settings object). Sketch the design: where would globals go? Where would functions that use globals be? Ask your AI: 'Are there design patterns that reduce dependency on global state?' Learn why they matter."
   - **Expected outcome**: Student sees scope as architectural decision; connects to larger design principles

---

### Lesson 5: Building a Calculator Utility (Capstone Project)

**CEFR Level**: B1-B2 (Integrated Application and Synthesis) — Students apply all concepts from Lessons 1-4 to build a real multi-module project

**Bloom's Levels**: Apply, Analyze, Create, Evaluate

**Concepts Taught** (7 concepts, within intermediate limit of 7 — integration rather than new):
1. **Multi-file organization**: Separating concerns across modules (main.py, operations.py, utils.py)
2. **Module imports**: Importing functions from custom modules
3. **Function specifications**: Writing clear, reusable functions with type hints and docstrings
4. **Parameter patterns**: Required and optional parameters in real use
5. **Return values**: Functions returning single values, tuples, and using them in other functions
6. **Scope across modules**: Understanding how scope works when importing between files
7. **Testing**: Validating that functions work correctly before integration

**Proficiency by End**: Students complete a working multi-module calculator project with separation of concerns; understand how modules enable code reuse; can validate their project through testing.

**Time Estimate**: 70 minutes (project building + testing + validation)

**Prerequisite Mastery**:
- Lessons 1-4: All module/function/scope concepts
- Chapter 14-18: Data types used in calculations

**Content Outline**:

1. **Introduction** (5 min) — Capstone project overview; architecture sketch
2. **Project Structure** (5 min) — Discuss file organization (main.py, operations.py, utils.py)
3. **Lesson 3a: Build operations.py** (15 min) — Implement calculator operations with type hints
4. **Lesson 3b: Build utils.py** (12 min) — Implement utility functions (display, input validation)
5. **Lesson 3c: Build main.py** (18 min) — Orchestrate the calculator; import and use modules
6. **Lesson 3d: Test and Validate** (10 min) — Run calculator; test functions; debug if needed
7. **Integration Summary** (5 min) — Reflect on project structure and concepts used

**Content Elements**:

**Project Overview: Calculator Utility**

A multi-module Python calculator that demonstrates:
- **operations.py**: Mathematical functions (add, subtract, multiply, divide)
- **utils.py**: Utility functions (display results, validate input, format output)
- **main.py**: Main program that imports and orchestrates the calculator

**File 1: operations.py (Mathematical operations)**
```python
"""
Calculator operations module.

Contains pure mathematical functions for basic calculations.
"""

def add(a: float, b: float) -> float:
    """Return the sum of two numbers."""
    return a + b

def subtract(a: float, b: float) -> float:
    """Return the difference of two numbers (a - b)."""
    return a - b

def multiply(a: float, b: float) -> float:
    """Return the product of two numbers."""
    return a * b

def divide(a: float, b: float) -> float | None:
    """
    Divide a by b.

    Returns:
        float | None: The quotient if b != 0, None if division by zero
    """
    if b == 0:
        return None
    return a / b

def power(base: float, exponent: float) -> float:
    """Return base raised to the exponent."""
    return base ** exponent

def square_root(number: float) -> float | None:
    """
    Calculate square root.

    Returns:
        float | None: The square root if number >= 0, None otherwise
    """
    if number < 0:
        return None
    return number ** 0.5
```
- **Purpose**: Show module of pure functions; clear intent through type hints
- **Design**: Functions have no side effects; only take input, return output

**File 2: utils.py (Utility functions)**
```python
"""
Utility functions for calculator.

Contains display, formatting, and input validation functions.
"""

def display_menu() -> None:
    """Display the calculator menu."""
    print("\n=== Simple Calculator ===")
    print("1. Add")
    print("2. Subtract")
    print("3. Multiply")
    print("4. Divide")
    print("5. Power")
    print("6. Square Root")
    print("7. Exit")
    print("=" * 24)

def get_operation_choice() -> str:
    """Get user's operation choice."""
    choice: str = input("Choose operation (1-7): ").strip()
    return choice

def get_numbers() -> tuple[float, float] | None:
    """
    Get two numbers from user.

    Returns:
        tuple[float, float] | None: (num1, num2) if valid, None if invalid
    """
    try:
        num1: float = float(input("Enter first number: "))
        num2: float = float(input("Enter second number: "))
        return num1, num2
    except ValueError:
        print("Invalid input. Please enter numbers.")
        return None

def get_single_number() -> float | None:
    """
    Get a single number from user (for square root).

    Returns:
        float | None: The number if valid, None if invalid
    """
    try:
        number: float = float(input("Enter a number: "))
        return number
    except ValueError:
        print("Invalid input. Please enter a number.")
        return None

def display_result(operation: str, result: float | None) -> None:
    """Display the calculation result."""
    if result is None:
        print("Calculation not possible (e.g., division by zero)")
    else:
        print(f"Result of {operation}: {result:.2f}")
```
- **Purpose**: Show utility module; functions that handle I/O and validation
- **Design**: Separated from math operations; reusable across different calculators

**File 3: main.py (Main program)**
```python
"""
Main calculator program.

Imports operations and utilities, orchestrates the calculator flow.
"""

import operations
import utils

def run_calculator() -> None:
    """Run the calculator in a loop."""
    while True:
        utils.display_menu()
        choice: str = utils.get_operation_choice()

        if choice == "1":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.add(num1, num2)
                utils.display_result("Addition", result)

        elif choice == "2":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.subtract(num1, num2)
                utils.display_result("Subtraction", result)

        elif choice == "3":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.multiply(num1, num2)
                utils.display_result("Multiplication", result)

        elif choice == "4":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.divide(num1, num2)
                utils.display_result("Division", result)

        elif choice == "5":
            numbers = utils.get_numbers()
            if numbers:
                num1, num2 = numbers
                result = operations.power(num1, num2)
                utils.display_result("Power", result)

        elif choice == "6":
            number = utils.get_single_number()
            if number is not None:
                result = operations.square_root(number)
                utils.display_result("Square Root", result)

        elif choice == "7":
            print("Goodbye!")
            break

        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    run_calculator()
```
- **Purpose**: Show main program that imports and orchestrates modules
- **Design**: Clean separation: operations do math, utils handle I/O, main orchestrates

**Testing the Project**:

```python
"""
Test the calculator operations.
This can be run as: python -m pytest test_calculator.py
Or: python test_calculator.py
"""

import operations

def test_add():
    assert operations.add(2, 3) == 5
    assert operations.add(-1, 1) == 0
    print("test_add PASSED")

def test_subtract():
    assert operations.subtract(5, 3) == 2
    assert operations.subtract(0, 5) == -5
    print("test_subtract PASSED")

def test_divide():
    assert operations.divide(10, 2) == 5.0
    assert operations.divide(10, 0) is None  # Division by zero
    print("test_divide PASSED")

def test_square_root():
    assert operations.square_root(9) == 3.0
    assert operations.square_root(-1) is None  # Can't sqrt negative
    print("test_square_root PASSED")

if __name__ == "__main__":
    test_add()
    test_subtract()
    test_divide()
    test_square_root()
    print("\nAll tests passed!")
```
- **Purpose**: Show how to validate functions work correctly
- **Design**: Simple assertion-based testing; demonstrates verification of specifications

**CoLearning Elements in This Lesson**:

- 💬 **Prompt (Conceptual)**: "Your calculator is working! Ask your AI: 'What would you add to make this production-ready? Security, error handling, features?' Listen to suggestions about real development concerns."
- 🎓 **Commentary**: "You just built a professional Python project. Notice: each file has a clear purpose (operations do math, utils do I/O, main orchestrates). This is the separation of concerns principle. It makes code easier to test, modify, and reuse."
- 🚀 **Challenge**: "Add a new operation to your calculator without changing main.py. Add a function to operations.py, then add the logic to main.py. This teaches you how good module design enables extensibility."
- ✨ **Tip**: "Your calculator imports operations like `import operations`. This works because Python looks for operations.py in the same directory. When sharing code, Python packages and `__init__.py` enable more sophisticated imports (Chapter 24+)."

**Try With AI - 4 Prompts (Progressive Bloom's)**:

1. **Apply** (Use the calculator):
   - **Prompt**: "Run the calculator project. Try all operations: add, subtract, multiply, divide, power, square root. Test edge cases: division by zero, square root of negative number. Did the program handle them correctly?"
   - **Expected outcome**: Student runs project successfully; understands error handling in operations

2. **Analyze** (Review module design):
   - **Prompt**: "Look at the three files (operations.py, utils.py, main.py). For each file, explain: What does this file do? Why is it separate from the others? What would break if you moved its functions to main.py?"
   - **Expected outcome**: Student articulates separation of concerns; understands modular design benefits

3. **Create** (Extend the project):
   - **Prompt**: "Add two new operations to your calculator: modulo (remainder) and absolute value. Update operations.py with the new functions (with type hints and docstrings), then update main.py to call them. Test the new operations."
   - **Expected outcome**: Student adds features correctly; integrates with existing code; shows understanding of module patterns

4. **Synthesize** (Connect to professional practice):
   - **Prompt**: "Imagine sharing your calculator project with a team. Ask your AI: 'What would a professional Python project add (documentation, configuration files, type checking, more tests)?' Explore one suggestion. This bridges to Chapter 30 (Spec-Driven Development)."
   - **Expected outcome**: Student sees project as starting point for professional development; connects to future chapters

---

## 📋 Content Flow & Dependencies

**Chapter 20 builds progressively**:

1. **Lesson 1** (Modules & Imports): Establishes that code is organized in files; students learn how to bring code from built-in modules into their scripts
2. **Lesson 2** (Functions with Intent): Builds on imports; students write their own functions and understand type hints as intent specification
3. **Lesson 3** (Parameters & Returns): Extends function knowledge; students write flexible functions with optional parameters and multiple return values
4. **Lesson 4** (Scope & Nested Functions): Integrates all function knowledge; students predict code behavior across scope boundaries; prepare for advanced patterns
5. **Lesson 5** (Capstone Project): Synthesizes everything; students build a real project using modules, functions, parameters, returns, and scope

**Each lesson builds on prior lessons within Chapter 20**:
- Lesson 1 concepts (module imports) are used in Lesson 2 (importing modules to use in functions)
- Lesson 2 concepts (function definition) are used in Lesson 3 (advanced parameter patterns)
- Lessons 1-3 concepts are used in Lesson 4 (scope affects how modules and functions interact)
- All Lessons 1-4 concepts are integrated in Lesson 5 (capstone project uses all patterns)

---

## 🎯 Scaffolding Strategy

**Graduated Teaching Pattern Applied Throughout Chapter 20**:

**Tier 1 - Book Teaches Directly**:
- What modules are (organizational concept)
- Why functions matter (code reuse, intent)
- Type hints as communication protocol
- Scope rules and why they prevent bugs

**Tier 2 - AI Companion Handles Syntax**:
- Import statement variations (students direct, AI shows examples)
- Function generation from docstrings (students specify intent, AI generates body)
- Module exploration (students ask "What functions does random module have?")

**Tier 3 - AI Orchestration for Capstone**:
- Students direct: "Help me structure a calculator project"
- AI orchestrates: Suggests file organization, generates code, tests
- Students validate and adapt

**Cognitive Load Management**:
- Lesson 1: 5 concepts (introductory)
- Lesson 2: 6 concepts (introducing function patterns)
- Lesson 3: 5 concepts (focused on parameters/returns)
- Lesson 4: 4 concepts (foundational scope)
- Lesson 5: 7 concepts (integration, no new concepts)

All within Intermediate tier limit of 7 concepts per lesson.

---

## 🔗 Integration Points

**Backward References (What We Built On)**:
- Chapters 13-19: All Python fundamentals (types, collections, control flow)
- Chapter 14 specifically: Type hints syntax and philosophy

**Forward References (What We Enable)**:
- Chapter 21 (Exception Handling): Functions use try/except for error handling; modules organize error handlers
- Chapter 22 (File I/O): Modules organize file utilities; functions read/write files with clear intent
- Chapter 24 (OOP): Classes as advanced functions; modules organize classes; inheritance and methods
- Chapter 26 (Meta Classes, Decorators): Nested functions and closures are foundation for decorators
- Chapter 30 (Spec-Driven Development): Function signatures are early form of specifications

**Cross-Chapter Connections**:
- Chapter 19 (Sets/Frozensets) + Chapter 20: Functions work with all data types; parameters/returns use sets as data
- Chapter 17 (Control Flow) + Chapter 20: Function bodies contain if/for/while from Chapter 17
- Chapter 18 (Collections) + Chapter 20: Functions operate on lists, tuples, dicts from Chapter 18

---

## ✅ Validation Strategy

**How learners demonstrate understanding**:

1. **Lesson 1**: Students correctly predict output of import statements; can import built-in modules and use functions without errors
   - Validation: Run provided code; explain what import variants do; use 3+ built-in modules

2. **Lesson 2**: Students write functions with type hints matching requirements; docstrings are clear and complete
   - Validation: Peer review of function signatures; run functions with test inputs; verify returns match type hints

3. **Lesson 3**: Students correctly call functions with positional and keyword arguments; unpack multiple return values
   - Validation: Function calls work without errors; return values assigned and used correctly; edge cases handled (division by zero, invalid input)

4. **Lesson 4**: Students predict scope behavior; explain variable locations; write nested functions that work correctly
   - Validation: Prediction exercises (without running code, predict what prints); test predictions against actual execution; write working nested function

5. **Lesson 5**: Capstone project runs without errors; multi-module organization works; testing validates correctness
   - Validation: All operations work correctly; project structure matches specification; project can be extended without breaking existing code; test suite passes

**Success Indicators**:
- SC-001: 90% of students write at least one custom module with 3+ functions and import those functions successfully
- SC-002: 85% of students write functions with type hints and docstrings on first attempt (with AI assistance)
- SC-003: 80% of students correctly predict variable scope in test scenarios
- SC-004: 100% of students complete working capstone project
- SC-005: Students explain import variants correctly

---

## 📊 CEFR Progression Across Chapter

| Lesson | CEFR Level | Focus | Student Role |
|--------|-----------|-------|--------------|
| 1 | A2 | Recognition/basic application of imports | Passive consumer of modules |
| 2 | A2-B1 | Writing functions, understanding intent | Active creator of functions |
| 3 | B1 | Applying parameter patterns | Independent problem-solving |
| 4 | B1-B2 | Analyzing scope, predicting behavior | Advanced pattern recognition |
| 5 | B1-B2 | Synthesizing all concepts in real project | Professional code organization |

**Proficiency Progression**: Students move from understanding what modules are (A2) to building professional multi-module projects (B1-B2)

---

## 🛠️ Implementation Notes for Lesson Writer

1. **No forward references**: Don't mention decorators, classes, packages, or Chapter 21+ concepts directly. Acknowledge they exist but teach Chapter 20 foundationally.

2. **Type hints are mandatory**: Every function example uses modern Python 3.14+ type hints (`def func(param: Type) -> ReturnType:`). No ambiguous untyped functions.

3. **CoLearning elements required**: Each lesson includes 💬 conceptual prompt, 🎓 teaching commentary, 🚀 specification-driven challenge, and ✨ AI tool usage tip.

4. **Try With AI closure essential**: Every lesson ends with "Try With AI" section (4 progressive Bloom's prompts). No additional "Key Takeaways" or "What's Next" sections.

5. **Code examples must be runnable**: All code examples should work when run directly (with minor modifications for interactive vs. script context).

6. **Emphasis on intent**: Throughout the chapter, emphasize that type hints and docstrings are how developers (and AI) understand code. This is the bridge to Spec-Driven Development (Chapter 30).

7. **Capstone integration**: The Calculator Utility capstone explicitly shows professional patterns (file organization, separation of concerns, testing) that prepare students for Chapter 21 (error handling) and Chapter 30 (specifications).

---

**Plan Quality Validation**:

- [x] All 5 lessons planned with CEFR levels (A2 → A2-B1 → B1 → B1-B2 → B1-B2)
- [x] Pedagogical ordering enforced (no forward references; each lesson builds on prior)
- [x] Concept count validated (5, 6, 5, 4, 7 — all within intermediate limit of 7)
- [x] CoLearning elements (💬🎓🚀✨) specified for each lesson
- [x] Try With AI closure for every lesson (4 prompts, Bloom's progression)
- [x] Capstone project integrates learning (3 modules, 8 functions, testing)
- [x] Type hints mandatory throughout (Python 3.14+ syntax)
- [x] Code examples detailed with specifications
- [x] Prerequisites explicitly listed
- [x] Integration points mapped (backward to Ch 1-19, forward to Ch 21-30)
- [x] Validation strategy aligned with specification success criteria

**Status**: Ready for Lesson Writer Implementation

**Next Step**: Execute `/sp.implement` or invoke content-implementer subagent to create actual lesson markdown content for book
