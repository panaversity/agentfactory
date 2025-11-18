---
title: "Console I/O and User Input Validation"
chapter: 22
lesson: 1
duration_minutes: 75

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using input() for User Interaction"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write input() to gather user data and understand it always returns string type"

  - name: "Input Validation with Type Conversion"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can convert user input using int()/float(), handle ValueError exceptions, implement retry loops"

  - name: "Formatting Output with F-Strings"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can use f-string expressions to format numbers, strings, and variables professionally"

  - name: "Error Recovery in Interactive Programs"
    proficiency_level: "A2-B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design input validation loops that handle errors gracefully and provide helpful messages"

  - name: "Understanding Console I/O Patterns"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain difference between console I/O (interactive terminal) and file I/O (persistent storage)"

learning_objectives:
  - objective: "Use input() to gather user data and understand its return type"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Write program that accepts user input and displays it"

  - objective: "Validate user input with type conversion and handle errors gracefully"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Write program that asks for number, validates type, retries on error"

  - objective: "Format output using f-strings with expressions and format specifications"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Create formatted output with currency, alignment, and number precision"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (input(), type conversion, ValueError, f-strings, retry loops) within A2 limit of 5 ✓"

differentiation:
  extension_for_advanced: "Explore advanced f-string formatting with scientific notation and custom alignment; implement email validation logic"
  remedial_for_struggling: "Start with just input() and print() together; practice type conversion with simple examples before adding try/except"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ain/specs/001-part-4-chapter-22/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Console I/O and User Input Validation

## Introduction: Why Console I/O Matters

When you write a program, you need to communicate with users. **Console I/O** — short for input/output — is how your Python programs interact with people sitting at the terminal. Input is what the user types. Output is what your program displays.

Think about the difference between a calculator you use on paper and one in your hand. On paper, you write down numbers, calculate, and write the answer. With a handheld calculator, you press buttons (input), the calculator processes, and it displays the result (output). Python programs work the same way: they ask questions, receive answers, process them, and show results.

This lesson teaches you how to build interactive programs that ask for user data, validate that data is correct, and display professional-looking output. By the end, you'll understand the foundation for all CLI (command-line interface) applications—everything from simple scripts to complex tools.

## Understanding input(): How User Data Flows In

Before we write any complex code, let's understand the simplest part: **input()**.

When you call `input()`, Python pauses your program and waits for the user to type something into the terminal. Whatever they type, Python captures it and **returns it as a string**. This is critical: **input() always returns a string, no matter what the user types**.

Let me show you:

```python
name: str = input("What is your name? ")
age_text: str = input("How old are you? ")

print(f"Hello, {name}!")
print(f"You typed: {age_text}")
print(f"Type of age_text: {type(age_text)}")
```

If a user runs this and types "Alice" and "28", the output is:

```
What is your name? Alice
How old are you? 28
Hello, Alice!
You typed: 28
Type of age_text: <class 'str'>
```

Notice: **age_text is the string "28", not the number 28**. This is the root of many beginner questions. You can't do math with a string "28"—you need the actual number.

#### 💬 AI Colearning Prompt

> "Explain what happens inside Python when I use `int()` on the string '25'. What error would I get if I tried `int('hello')`?"

**Expected Outcome**: You'll understand type conversion mechanics and the ValueError exception before practicing.

## Type Conversion and Validation: From Strings to Numbers

Since `input()` always returns a string, we need to **convert** it to the right type if we want to do math or comparisons.

Python provides conversion functions: `int()`, `float()`, `str()`. Here's how they work:

```python
age_string: str = "25"
age_number: int = int(age_string)  # Convert string to integer

price_string: str = "19.99"
price_number: float = float(price_string)  # Convert string to float
```

But what happens if the user types something that can't be converted? If you ask for a number and they type "hello", Python raises a `ValueError`. This is where **error handling** comes in.

The pattern is: **try** to convert, **except** catch the error if it fails.

```python
try:
    age_str: str = input("Enter your age: ")
    age: int = int(age_str)
    print(f"You are {age} years old.")
except ValueError:
    print("That's not a valid number. Please try again.")
```

If the user types "hello", the program doesn't crash. Instead, it catches the error and shows a friendly message.

#### 🎓 Expert Insight

> In AI-native development, you don't memorize exception types — you understand **when** type conversion fails. Ask your AI: "What exceptions can occur when converting user input?" Your job is designing the validation **strategy** (when to accept input, when to reject it, what error messages to show); syntax is cheap.

## F-String Formatting: Professional Output

Now that you can get numbers from users, you want to display them nicely. **F-strings** let you embed expressions inside text and format numbers beautifully.

The basic syntax is `f"text {variable} more text"`. Inside the curly braces `{}`, you can put any Python expression:

```python
name: str = "Alice"
age: int = 28

# Simple variable insertion
print(f"Hello, {name}!")

# Expressions
print(f"Next year, {name} will be {age + 1} years old.")

# Format specifications
price: float = 19.999
print(f"Price: ${price:.2f}")  # Rounds to 2 decimal places
```

The last example shows a **format specifier** — the part after the colon. `:.2f` means "format as a float with 2 decimal places."

Here are common format specifiers you'll use:

- `:.2f` — Float with 2 decimal places
- `:,` — Thousands separator for large numbers
- `:>10` — Right-align in 10 characters
- `:05d` — Integer padded with zeros to 5 digits

```python
quantity: int = 5
subtotal: float = 99.50
total: float = subtotal * quantity

print(f"Items: {quantity}")
print(f"Subtotal: ${subtotal:.2f}")
print(f"Total: ${total:,.2f}")  # With thousands separator
```

Output:
```
Items: 5
Subtotal: $99.50
Total: $497.50
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Generate a function that asks for a user's email and validates it contains '@' and a domain. Then explain how string methods like `.find()` and `.count()` work. Show me at least two different validation approaches."

**Expected Outcome**: You'll see multiple validation strategies and understand when each applies.

## Error Recovery Loops: Asking Again When Input Is Wrong

Here's the problem: if a user enters invalid input and you show an error message, the program ends. That's not friendly. Instead, use a **loop** to keep asking until the input is valid.

The pattern is a `while True` loop that breaks when you get valid input:

```python
def get_positive_integer(prompt: str) -> int:
    """Ask user for a positive integer until they provide valid input."""
    while True:
        try:
            value_str: str = input(prompt)
            value: int = int(value_str)

            # Check if it's positive
            if value <= 0:
                print("Please enter a positive number.")
                continue  # Ask again

            return value  # Valid input, exit function

        except ValueError:
            print("That's not a whole number. Try again.")
            # Loop continues automatically
```

This function:
1. Shows a prompt
2. Gets user input
3. Tries to convert to `int`
4. Checks if it's positive
5. If anything goes wrong, shows a helpful error and **loops back to step 1**
6. When input is valid, returns the number and the function ends

Users can make mistakes multiple times, and the program handles each one gracefully.

#### ✨ Teaching Tip

> Use Claude Code to explore f-string variations: "How many different ways can I format a floating-point number in an f-string? Show me examples for currency, percentages, and scientific notation. Which would I use for financial data?"

## Code Examples

### Code Example 1.1: Basic input() with Type Conversion

**Specification Reference**: Demonstrate input() returns string; show type conversion to int

**AI Prompt Used**: "Show me how to use input() and convert it to an integer"

```python
# Example 1.1: Basic Input and Type Conversion
"""
This example shows:
1. input() always returns a string
2. You must convert to other types
3. Simple f-string display
"""

# Gather information from user
name: str = input("What is your name? ")
age_str: str = input("How old are you? ")

# Convert string to integer
age: int = int(age_str)

# Display with f-strings
print(f"Hello, {name}!")
print(f"You are {age} years old.")

# Demonstrate types
print(f"Type of name: {type(name)}")
print(f"Type of age: {type(age)}")
```

**Validation Steps/Results**:
- Run with valid input: `Alice` and `28` → Program displays greeting and age correctly
- The `type()` calls confirm that `name` is `str` and `age` is `int`
- This demonstrates the foundation: input gives strings, conversion gives numbers

### Code Example 1.2: Input Validation with Error Handling

**Specification Reference**: Show complete validation pattern with try/except and range checking

**AI Prompt Used**: "Write a function that asks for a positive integer and keeps retrying until valid input"

```python
# Example 1.2: Input Validation with Error Recovery
"""
This example shows:
1. try/except to catch ValueError
2. Range checking (positive numbers)
3. Loop to retry on invalid input
4. User-friendly error messages
"""

def get_positive_integer(prompt: str) -> int:
    """
    Ask user for a positive integer until they provide valid input.

    Args:
        prompt: The question to display to the user

    Returns:
        A positive integer the user provided
    """
    while True:
        try:
            # Get input as string
            value_str: str = input(prompt)

            # Convert to integer (may raise ValueError)
            value: int = int(value_str)

            # Check range
            if value <= 0:
                print("Error: Please enter a positive number (greater than 0).")
                continue

            # Valid input, return it
            return value

        except ValueError:
            # Conversion failed (user typed non-numeric text)
            print("Error: That's not a whole number. Please enter digits only.")
            # Loop continues, user is asked again


# Use the function
print("Welcome to the Age Calculator!\n")
age: int = get_positive_integer("Enter your age: ")
print(f"You entered: {age}")
print(f"Next year you will be: {age + 1}")
```

**Validation Steps/Results**:
- Run with valid input `28` → Immediately returns 28
- Run with invalid input `hello` → Shows ValueError message, asks again
- Run with invalid input `-5` → Shows "positive number" message, asks again
- Run with invalid input `28.5` → Shows "whole number" message, asks again
- After valid input on retry, program continues normally

### Code Example 1.3: Formatted Output with Numbers

**Specification Reference**: Show f-string formatting options for professional output

**AI Prompt Used**: "Show me how to format currency with 2 decimal places and thousands separators"

```python
# Example 1.3: Professional Formatted Output
"""
This example shows:
1. F-string format specifiers
2. Currency formatting
3. Number alignment and precision
4. Thousands separators
"""

# Example 1: Currency Formatting
price: float = 19.999
items: int = 5
total: float = price * items

print("=" * 40)
print("INVOICE EXAMPLE")
print("=" * 40)

# Format currency with 2 decimal places
print(f"Price per item: ${price:.2f}")

# Right-align numbers in a field
print(f"Number of items: {items:>5}")

# Thousands separator with 2 decimals
print(f"Total cost: ${total:,.2f}")

print("=" * 40)

# Example 2: Building a receipt with user input
print("\nRECEIPT BUILDER\n")

# Get quantity from user
quantity_str: str = input("How many items? ")
quantity: int = int(quantity_str)

# Calculate subtotal
unit_price: float = 25.50
subtotal: float = unit_price * quantity
tax_rate: float = 0.08
tax: float = subtotal * tax_rate
final_total: float = subtotal + tax

# Display receipt
print("\n" + "=" * 40)
print("RECEIPT")
print("=" * 40)
print(f"Unit price:        ${unit_price:>8.2f}")
print(f"Quantity:          {quantity:>8}")
print(f"Subtotal:          ${subtotal:>8.2f}")
print(f"Tax (8%):          ${tax:>8.2f}")
print("-" * 40)
print(f"TOTAL:             ${final_total:>8.2f}")
print("=" * 40)
```

**Validation Steps/Results**:
- Currency values display with exactly 2 decimal places
- Thousands separators appear for large numbers
- Numbers are right-aligned for professional appearance
- Receipt layout is clear and readable
- Run with `quantity_str = "10"` → Shows full receipt with correct calculations

## Practice Exercises

### Exercise 1: User Profile with Validation

Write a program that:
1. Asks for a user's name (string)
2. Asks for their favorite number (integer, must be positive)
3. Validates the number is positive
4. Displays a formatted message using f-strings

**Acceptance Criteria**:
- Program runs without crashes on both valid and invalid input
- Invalid input shows error message and asks again
- Valid input displays formatted output

**Test Cases**:
- Input: name="Alice", number="42" → Should display success
- Input: name="Bob", number="-5" → Should reject and ask again
- Input: name="Carol", number="hello" → Should reject and ask again, then accept valid number

### Exercise 2: Input Validation Function

Create a function `get_positive_float()` that:
1. Asks user for a positive decimal number
2. Validates input is a valid float
3. Validates input is positive (> 0)
4. Returns the valid float
5. Shows friendly error messages for each error type

**Acceptance Criteria**:
- Function rejects non-numeric input with appropriate message
- Function rejects negative/zero values with appropriate message
- Function returns valid positive float
- Function loops until valid input received

**Test Cases**:
- Input: "3.14" → Returns 3.14
- Input: "-2.5", then "5.0" → Rejects negative, accepts positive
- Input: "abc", "12.5" → Rejects non-numeric, accepts valid number

## Try With AI: The Interactive CLI Builder

### Part 1: Design a Menu System (Your Turn First)

**Before asking AI**, design a simple calculator CLI with a menu that:
- Shows the user 4 options: (1) Add, (2) Subtract, (3) Multiply, (4) Exit
- Gets their choice
- Gets two numbers
- Performs the operation
- Returns to menu

**Your task**: Write pseudocode or a rough outline describing:
1. How the menu loop starts
2. What happens after the user selects an option
3. Where validation should occur (menu choice, number inputs)
4. How the program exits

Create a file `calculator_design.md` with your design notes.

**Deliverable**: A text file showing your planned structure before writing Python code.

---

### Part 2: AI Explains Input Validation Patterns (Discovery)

**Share your design with AI:**

> "Here's my calculator CLI design [paste your pseudocode]. Explain the input validation strategy I should use. Show me the difference between validating the menu choice vs validating numbers. What try/except patterns should I use for each?"

**Your evaluation**:
- Does the AI explain when to use try/except vs when to check conditions?
- Does it show at least two different validation patterns?
- Can you understand which pattern to use for each input type?

---

### Part 3: Student Teaches AI—Edge Cases (Debugging)

**Challenge the AI with scenarios:**

> "What happens if a user:
> 1. Types 'abc' when asked for a number?
> 2. Types 5 when menu shows only options 1-4?
> 3. Types nothing and just presses Enter?
> 4. Enters a floating-point number (3.5) when I need an integer?
>
> Show me specific error messages my code should display for each case, and which error type Python will raise."

**Your debugging**: Test the AI's answers by predicting what your calculator code should do, then verify with actual Python.

---

### Part 4: Build an Interactive Calculator (Convergence)

**Build the complete calculator:**

> "Now help me write a complete interactive calculator that:
> 1. Displays menu (Add, Subtract, Multiply, Exit)
> 2. Validates menu choice (must be 1-4)
> 3. Gets two numbers with try/except handling
> 4. Performs calculation
> 5. Shows formatted result
> 6. Returns to menu automatically
>
> The calculator should handle all the edge cases we discussed. Show me the complete, production-ready code with input validation throughout."

**Refinement**:
- Extend the calculator to add Division (with zero-division checking)
- Add input history: show the last 3 calculations before the menu
- Test with invalid inputs and verify error messages are helpful

**Time**: 25-35 min
**Outcome**: Working interactive CLI calculator with robust input validation

---

**Safety & Ethics Note**: When you work with user input, always validate it before using it. Never assume user input is correct or safe. In real applications, this extends to protecting against attacks. For now, focus on making programs that handle mistakes gracefully and provide helpful error messages.
