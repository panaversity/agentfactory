---
title: "Arithmetic Operators — Doing Math with Python"
chapter: 15
lesson: 1
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Arithmetic Operations with Type Safety"
    proficiency_level: "A1-A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write `result: int = 5 + 3` and use `type(result)` to verify result type; perform all 7 arithmetic operations without referencing examples"

  - name: "Understanding Type Hint Feedback from Operations"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can explain 'Why does int / int give float?' and 'Why does int // int give int?' using type validation"

  - name: "AI-Driven Exploration of Edge Cases"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can formulate questions like 'What happens if I divide by zero?' and validate AI responses through code testing"

learning_objectives:
  - objective: "Understand what each of the seven arithmetic operators (+, -, *, /, //, %, **) does in plain language"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Explanation of operators; code with type verification"

  - objective: "Apply arithmetic operators correctly to perform calculations using type hints and validation"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Write arithmetic expressions without errors; use type() to verify results"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (the 7 operators grouped as arithmetic family, type behavior with int/float, division differences, modulo for remainders, exponentiation) within A1-A2 limit of 5 ✓"

differentiation:
  extension_for_advanced: "Explore operator precedence with complex expressions; ask AI about edge cases like 0.1 + 0.2; investigate negative number behavior with modulo and floor division"
  remedial_for_struggling: "Focus on first 4 operators (+, -, *, /); use concrete single-digit numbers; build confidence with simple expressions before advancing to //, %, **"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/part-4-chapter-15/spec.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "content-implementer subagent"
version: "1.0.0"
---

# Arithmetic Operators — Doing Math with Python

You've been doing math your whole life. Five plus three equals eight. Ten minus four equals six. Now you're going to do the exact same math with Python—except Python calls them **operators** instead of just "math".

An **operator** is a symbol that tells Python to do something with values. In this lesson, you'll learn the seven arithmetic operators that let you perform calculations. You already know the concepts; Python just has its own symbols for them.

## What It Is: Operators as Verbs for Numbers

Think of operators like verbs in English. Just like "run", "jump", and "walk" are actions, operators are actions you perform on numbers.

Python has **seven arithmetic operators**:

- `+` **Addition** — combines values
- `-` **Subtraction** — finds the difference
- `*` **Multiplication** — scales values
- `/` **Division** — splits into parts
- `//` **Floor division** — counts whole groups (no decimals)
- `%` **Modulus** — finds the remainder
- `**` **Exponentiation** — raises to a power

All of these do exactly what you learned in math class. The difference is that Python needs to know what to do with types—and that's where it gets interesting.

## The Seven Arithmetic Operators

Let's see all seven operators in action. Each one takes two numbers (called **operands**) and produces a result.

### Addition and Subtraction: The Basics

```python
# Arithmetic operators: doing math with Python
x: int = 10
y: int = 3

# Addition: combine values
add_result: int = x + y
print(f"{x} + {y} = {add_result}")  # 10 + 3 = 13

# Subtraction: find the difference
sub_result: int = x - y
print(f"{x} - {y} = {sub_result}")  # 10 - 3 = 7
```

Both addition and subtraction keep the type consistent. When you add two integers, you get an integer back. Same with subtraction.

#### 💬 AI Colearning Prompt

> "Python is designed to be helpful with numbers. But why does Python have both `/` and `//` for division when math just has one division symbol? Explain the difference and why this matters."

Notice something: we're asking AI to explain the **design choice** behind operators, not just what they do. That's more useful than memorization.

### Multiplication, Division, and Floor Division

```python
# Continuing from previous example (x=10, y=3)
x: int = 10
y: int = 3

# Multiplication: scaling
mul_result: int = x * y
print(f"{x} * {y} = {mul_result}")  # 10 * 3 = 30

# Division: splits into parts (always float)
div_result: float = x / y
print(f"{x} / {y} = {div_result}")  # 10 / 3 = 3.3333...

# Floor division: counts whole groups (integer result)
floor_result: int = x // y
print(f"{x} // {y} = {floor_result}")  # 10 // 3 = 3
```

Here's something important: **division with `/` always returns a float, even when both numbers are integers**. This surprises many beginners because they expect `10 / 5` to give `2` (an int), but it actually gives `2.0` (a float).

Why? Because division often produces decimals, so Python designed `/` to always return float. If you want an integer result (discarding the decimal), you use `//` (floor division).

#### 🎓 Expert Insight

> In AI-native development, you don't memorize why Python made this choice. You understand what's happening (division produces floats, floor division gives integers) and use the right operator for your intent. Then you verify with `type()` to be sure.

Let's verify this with `type()`:

```python
# Continuing from previous example (x=10, y=3)
x: int = 10
y: int = 3

# Calculate results
div_result: float = x / y
floor_result: int = x // y

# Verify the types
print(f"Type of 10 / 3: {type(div_result)}")    # <class 'float'>
print(f"Type of 10 // 3: {type(floor_result)}")  # <class 'int'>

# What if we use floor division with floats?
f1: float = 10.5
f2: float = 3.0
float_floor: float = f1 // f2
print(f"Type of 10.5 // 3.0: {type(float_floor)}")  # <class 'float'> (not int!)
```

Notice: when you floor divide two floats, the result is still a float. Python follows this rule: **if either operand is a float, the result is a float**.

### Modulus: The Remainder Operator

```python
# Continuing from previous example (x=10, y=3)
x: int = 10
y: int = 3

# Modulus: finds the remainder
mod_result: int = x % y
print(f"{x} % {y} = {mod_result}")  # 10 % 3 = 1
# (10 divided by 3 is 3 with remainder 1)
```

The modulus operator `%` is useful when you care about what's left over. For example:
- Check if a number is even: `5 % 2` gives 1 (odd), `4 % 2` gives 0 (even)
- Find the last digit: `234 % 10` gives 4
- Cycle through values: `7 % 3` gives 1, `8 % 3` gives 2, `9 % 3` gives 0 (then it repeats)

### Exponentiation: Raising to a Power

```python
# Continuing from previous example (x=10)
x: int = 10

# Exponentiation: raising to a power
exp_result: int = x ** 2
print(f"{x} ** 2 = {exp_result}")  # 10 ** 2 = 100

# More examples
print(f"2 ** 3 = {2 ** 3}")        # 8 (2 × 2 × 2)
print(f"5 ** 0 = {5 ** 0}")        # 1 (anything to the 0th power is 1)
print(f"2 ** 10 = {2 ** 10}")      # 1024 (exponents grow fast!)
```

Notice the symbol: `**` (two asterisks), not `^`. The caret symbol `^` does something different in Python (bitwise XOR), so don't use it for exponentiation.

#### 🤝 Practice Exercise

> **Ask your AI**: "I understand that arithmetic operators in Python are +, -, *, /, //, %, **. But why does `/` always return a float even when dividing two integers? And when would I actually use `//` vs. `/`? Give me a concrete example where using the wrong one would cause a bug."

**Expected Outcome**: You'll understand the design decisions behind Python's division operators and how they interact with types.

## Type Behavior: Why Types Change

Here's where it gets interesting. When you combine operands of different types, Python follows rules about what type the result is.

```python
# All integer operations
int_add: int = 5 + 3           # 8 (int)
int_mult: int = 5 * 3          # 15 (int)

# Division always gives float
int_div: float = 5 / 2         # 2.5 (float)

# Mixing int and float
mixed: float = 5 + 2.0         # 7.0 (float) - float "wins"
mixed2: float = 5 * 2.5        # 12.5 (float)

# Verify with type()
print(type(int_add))           # <class 'int'>
print(type(int_div))           # <class 'float'>
print(type(mixed))             # <class 'float'>
```

**The pattern**: When you mix `int` and `float`, the result is always `float`. Python considers this safe because a float can represent any integer value (though with potential precision loss for very large numbers).

## Operator Precedence: Order Matters

Just like in math, operators have an **order of operations**. Multiplication happens before addition.

```python
# Wrong order: 2 + 3 * 4
result1: int = 2 + 3 * 4
print(result1)  # 14 (not 20!)
# Python evaluates: 3 * 4 = 12, then 2 + 12 = 14

# Use parentheses to control order
result2: int = (2 + 3) * 4
print(result2)  # 20 (addition happens first)
```

Python follows PEMDAS (Parentheses, Exponents, Multiplication/Division, Addition/Subtraction). But honestly, **the best practice is to use parentheses**. Make your intent clear.

```python
# Real-world example: calculating total with tax
price: float = 100.0
tax_rate: float = 0.08
fee: float = 10.0

# Without parentheses, this is confusing
total1: float = price * (1 + tax_rate) + fee  # 118.0 (tax then fee)

# The parentheses make it obvious: first multiply by (1 + 0.08), then add fee
total2: float = (price * (1 + tax_rate)) + fee  # 118.0 (same result, clearer intent)
```

Parentheses aren't just for changing order—they're for **clarifying your intent** to anyone reading your code (including yourself a month from now).

#### 💬 AI Colearning Prompt

> "When I see `2 + 3 * 4`, how does Python know to do multiplication first? Explain operator precedence. And when should I use parentheses even if they're not required?"

## Putting It Together: A Complete Example

Let's see all seven operators working together:

```python
# Working with arithmetic operators
x: int = 20
y: int = 7

# All seven operators
print(f"Addition:       {x} + {y} = {x + y}")           # 27
print(f"Subtraction:    {x} - {y} = {x - y}")           # 13
print(f"Multiplication: {x} * {y} = {x * y}")           # 140
print(f"Division:       {x} / {y} = {x / y}")           # 2.857...
print(f"Floor division: {x} // {y} = {x // y}")         # 2
print(f"Modulus:        {x} % {y} = {x % y}")           # 6 (remainder when 20 ÷ 7)
print(f"Exponentiation: {x} ** 2 = {x ** 2}")           # 400 (20 squared)

# Verify types
print(f"\nType verification:")
print(f"Type of {x} + {y}: {type(x + y)}")              # <class 'int'>
print(f"Type of {x} / {y}: {type(x / y)}")              # <class 'float'>
print(f"Type of {x} // {y}: {type(x // y)}")            # <class 'int'>
```

This example shows:
- All seven operators working
- Different types in the results (int vs. float)
- Type verification with `type()`

## Common Mistakes (And Why They Happen)

**Mistake 1: Expecting `5 / 2` to give `2`**

```python
result: float = 5 / 2   # Gives 2.5, not 2
# Use // if you want an integer result
result_int: int = 5 // 2  # Gives 2
```

**Mistake 2: Forgetting that mixing int and float gives float**

```python
result: float = 5 + 2.0   # Result is 2.0, not an int
# This often surprises people because 5 is an int, but the result is float
```

**Mistake 3: Trying to use `/` for exponentiation**

```python
# WRONG: This is division by 2, not 2 to the power
result: float = 5 / 2           # 2.5 (not what you want)

# CORRECT: Use ** for exponentiation
result: int = 5 ** 2          # 25 (5 squared)
```

---

## Try With AI: Calculator Building Challenge

You've learned 7 arithmetic operators. Now build a real calculator to apply them—with AI as your code review partner.

### Part 1: Build Basic Calculator (Your Turn First)

**Before asking AI**, write a calculator program that:

1. Takes two numbers: `num1 = 10` and `num2 = 3`
2. Performs ALL 7 operations and displays results:
   - Addition: `num1 + num2`
   - Subtraction: `num1 - num2`
   - Multiplication: `num1 * num2`
   - Division: `num1 / num2`
   - Floor division: `num1 // num2`
   - Modulus: `num1 % num2`
   - Exponentiation: `num1 ** num2`
3. Uses type hints for all variables
4. Uses f-strings to display: `"10 + 3 = 13"`

Write this yourself. Save as `calculator.py` and run it.

---

### Part 2: AI Reviews Your Calculator

Share your code with AI:

> "Here's my Python calculator: [paste code]. Review it for: (1) Correct use of all 7 arithmetic operators, (2) Proper type hints (are results int or float?), (3) Clear output formatting. What did I do well? What should I improve?"

**Your task**: Read AI's feedback.
- Did it catch type errors? (`/` returns float, not int!)
- Did it suggest better f-string formatting?
- Do you understand WHY each operator produces its result?

---

### Part 3: Student Teaches AI (Division Edge Case)

AI reviewed your code. But does it understand the `/` vs `//` distinction deeply?

Challenge AI with a scenario:

> "I'm calculating how many groups of 3 people I can make from 10 people. Should I use `10 / 3` or `10 // 3`? Explain: (1) What each gives, (2) Which one makes sense for counting groups, (3) What type each result is. Show code with type() to verify."

**Your task**: Evaluate AI's explanation.
- Does it explain that `//` gives whole groups (no partial people!)?
- Does it show that `/` gives 3.333... which doesn't make sense for counting?
- Can you now explain when to use `/` vs `//` in your own words?

---

### Part 4: Extend Calculator with Validation

Now iterate to add error handling:

> "Extend my calculator to: (1) Check if num2 is zero before dividing, (2) Show an error message instead of crashing, (3) Add a calculation that shows `num1 % num2` only makes sense when num2 != 0. Keep type hints and clear output."

**Your task**: Review AI's enhanced version.
- Does it prevent division by zero crashes?
- Does it explain WHY we check for zero?
- Can you add more validations yourself now?

Iterate if needed:
> "Explain why `10 / 0` crashes but `10 ** 100` doesn't. What's the difference?"

---

**Time**: 25-30 minutes total
**Outcome**: You've built a complete calculator applying all 7 operators, learned code review with AI, discovered the `/` vs `//` distinction through practical use, and added error handling for edge cases.
