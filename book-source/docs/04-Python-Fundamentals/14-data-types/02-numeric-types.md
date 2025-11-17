---
title: "Numeric Types: Integers, Floats, and Complex Numbers"
chapter: 14
lesson: 2
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Distinguishing Numeric Types (int, float, complex)"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can identify differences between int, float, and complex numbers; explain when to use each type"

  - name: "Choosing Appropriate Numeric Type for Scenarios"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can analyze real-world scenarios and select the correct numeric type with reasoning"

  - name: "Using type() to Verify Numeric Types"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can use type() function to inspect and verify what type Python assigned to a numeric value"

  - name: "Identifying Common Numeric Type Mistakes"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can recognize and explain type-related errors like integer division results, float precision issues, and type loss"

learning_objectives:
  - objective: "Distinguish between int, float, and complex numbers with reasoning about when each is appropriate"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Classification exercises and scenario analysis"

  - objective: "Choose the appropriate numeric type for real-world scenarios"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Scenario-based selection with written justification"

  - objective: "Use type() to verify and inspect numeric types"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Type inspection exercises with code examples"

  - objective: "Identify and explain common numeric type mistakes and their consequences"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Error analysis and debugging exercises"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (int, float, complex, decision framework, type(), common mistakes) within A2-B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Research IEEE 754 floating-point standard; explore complex numbers in scientific computing; investigate precision issues in financial calculations"
  remedial_for_struggling: "Focus exclusively on int vs float distinction; use physical analogies (whole objects vs. divided objects); skip complex numbers initially; practice basic type() calls"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/013-chapter-14-redesign/plan.md"
created: "2025-11-15"
last_modified: "2025-11-15"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Numeric Types: Integers, Floats, and Complex Numbers

## What You'll Learn

In this lesson, you'll master:
- Understanding integers (int) for whole numbers and when to use them
- Working with floats for decimal numbers and understanding precision
- Recognizing complex numbers and their specialized applications
- Choosing the right numeric type for different scenarios
- Avoiding common numeric type mistakes

## Opening Hook: The Price Tag Problem

You walk into a store. You see two prices:
- **Age**: 25 years old
- **Item Price**: $25.99

Why is one written `25` and the other `25.99`? They're both numbers, but they're **different kinds** of numbers. Age is a **whole number**—you can't be 25.5 years old. Price is a **decimal number**—you need cents. Python has different types for these different kinds of numbers.

In this lesson, you'll learn **when to use each numeric type** and **why it matters** for your code.

---

## WHAT Is an Integer?

A **integer** (or `int`) is a whole number. No decimal points. No fractions.

### Characteristics of Integers

- **Whole numbers only**: 5, 100, -3, 0
- **Positive, negative, or zero**: Can represent direction or absence
- **Unlimited range**: Python 3 has no upper limit (unlike older languages)
- **Exact values**: 25 is always exactly 25—never 25.0

### Real-World Examples

Think about when you naturally count whole numbers:

- **Age**: "I'm 25 years old" (not 25.5)
- **Student count**: "There are 30 students in class" (not 30.7 students)
- **Array index**: "The first item is at position 0"
- **Inventory**: "We have 150 boxes in stock"
- **Score**: "You earned 95 points"

### WHY Use Integers?

Use `int` when you need **exact whole numbers** and **precision matters**. You can't have half a person, half a product, or a fractional index. Integers guarantee no rounding errors.

### WHEN to Choose Integer Type

Ask yourself: "Is this something that makes sense as a whole number only?"

- Counting discrete items → `int`
- Measuring continuous quantities (distance, weight) → not `int`
- Storing True/False → not `int` (use `bool`)
- Building arrays/lists → `int` (for indices)

---

## NOW Show Integer Code

Here's how you declare integer variables with **type hints**:

```python
age: int = 25
student_count: int = 100
list_index: int = 0
temperature: int = -5
inventory: int = 500
```

**Notice**: Each variable includes a **type hint** (`: int`). This tells Python (and anyone reading your code) that this variable should hold an integer.

To verify the type, use the `type()` function:

```python
print(type(age))              # Output: <class 'int'>
print(type(student_count))    # Output: <class 'int'>
print(type(-5))               # Output: <class 'int'>
```

The `type()` function inspects what type Python assigned to a value. Always outputs `<class 'typename'>`.

#### 💬 AI Colearning Prompt

> "Why does Python differentiate between 25 (int) and 25.0 (float) when they represent the same value? What operations would differ?"

This helps you understand that types aren't just labels—they represent **different computational behaviors**.

---

## WHAT Is a Float?

A **float** is a number with a decimal point. It can represent fractions and very large/small numbers.

### Characteristics of Floats

- **Decimal points**: 3.14, 0.5, -2.75
- **Represent fractions and measurements**: Can be 3.7 km, not just 3 km or 4 km
- **Scientific notation**: `1.5e-3` means 0.0015 (small) or `2.5e6` means 2,500,000 (large)
- **Approximate values**: Due to IEEE 754 floating-point standard (Advanced topic), some floats can't be represented exactly

### Real-World Examples

When do you naturally work with decimals?

- **Price**: "$19.99" (dollars and cents)
- **Measurement**: "3.14 meters of cloth"
- **Percentage**: "0.85" (85% as a decimal)
- **Scientific data**: "3.14159" (π in calculations)
- **Temperature**: "37.5 degrees Celsius"
- **Weight**: "2.5 kilograms"

### WHY Use Floats?

Use `float` when **precision after the decimal point matters**. Prices, measurements, and scientific values need decimal accuracy.

### WHEN to Choose Float Type

Ask yourself: "Does this value need a decimal point?"

- Prices with cents → `float`
- Measurements of continuous quantities → `float`
- Percentages expressed as decimals → `float`
- Scientific/mathematical calculations → `float`
- Counts of discrete items → not `float` (use `int`)

---

## NOW Show Float Code

Here's how you declare float variables:

```python
price: float = 19.99
pi: float = 3.14159
percentage: float = 0.85
height: float = 1.75
temperature: float = 37.5
epsilon: float = 1e-6
```

Verify the type with `type()`:

```python
print(type(price))           # Output: <class 'float'>
print(type(3.14))            # Output: <class 'float'>
print(type(1e-6))            # Output: <class 'float'>
```

#### 🎓 Expert Insight

In AI-native development, you don't memorize floating-point edge cases—you understand WHEN precision matters and ask AI to explain unexpected behavior. Syntax is cheap; recognizing "this calculation needs decimal accuracy" is gold.

---

## Decision Guide: Integer vs. Float

When faced with a numeric scenario, use this table to decide:

| Scenario | Type | Why |
|----------|------|-----|
| Age (years) | `int` | No fractional ages exist |
| Price (dollars and cents) | `float` | Needs decimal precision |
| Student count | `int` | Can't have 25.5 students |
| Distance traveled (km) | `float` | Can be 3.7 km, not just 3 or 4 |
| Number of items sold | `int` | Discrete count |
| Average score (points) | `float` | May not be whole (87.5 average) |
| Inventory items | `int` | Counting whole units |
| Weight in kilograms | `float` | Continuous measurement |
| Array index | `int` | Always whole number |
| Calculation result (ratio) | `float` | May include decimals |

#### 🤝 Practice Exercise

> **Ask your AI**: "Give me 15 real-world scenarios (like 'distance to store', 'number of emails received'). For each, should I use int or float? Explain your reasoning, then I'll classify them."

**Expected Outcome**: You'll build intuition for when decimals matter. When you disagree with AI, ask why—this reveals edge cases you haven't considered.

---

## WHAT Is Complex? — Advanced Topic

A **complex** number combines a real part and an imaginary part: `a + bj`.

### Characteristics of Complex Numbers

- **Format**: `real_part + imaginary_part*j` (example: `2+3j`)
- **Real part**: Normal decimal number
- **Imaginary part**: Coefficient of `j` (the imaginary unit)
- **Specialized use**: Scientific computing, engineering, signal processing, physics simulations

### Real-World Application (Why It Exists)

Complex numbers aren't just theoretical—they solve real problems:

- **Electrical engineering**: AC circuit analysis
- **Physics**: Quantum mechanics simulations
- **Signal processing**: Audio/image analysis
- **Control systems**: Stability analysis

### Code Example

```python
z: complex = 2 + 3j
print(type(z))     # Output: <class 'complex'>
print(z.real)      # Output: 2.0 (real part)
print(z.imag)      # Output: 3.0 (imaginary part)
print(z + (1+2j))  # Output: (3+5j) (complex addition)
```

### The Honest Message for Beginners

**Know it exists. You'll rarely need it as a beginner.**

Complex numbers are specialized. Unless you're working in physics, engineering, or signal processing, you won't use them in early projects. Come back to this when you need it—and the book will guide you then.

---

## Common Numeric Type Mistakes

Even experienced developers make these mistakes. Here are five to watch for:

### Mistake 1: Storing a Decimal in an Integer Variable (Type Loss)

```python
# WRONG: Trying to use int for a decimal value
age: int = 25.5  # Type hint says int, but value is 25.5
# Python allows this but might truncate: age becomes 25
```

**Why it matters**: You lose information. The `.5` disappears.

**Fix**: Use `float` when decimals are involved.

```python
age: float = 25.5  # Correct
```

### Mistake 2: Integer Division Surprise

```python
result: int = 5 / 2
print(result)  # You might expect 2, but get 2.5!
print(type(result))  # <class 'float'> — not int!
```

**Why it happens**: In Python 3, `/` always returns a `float`, even if both numbers are integers.

**Fix**: Use `//` for integer division (discards the decimal):

```python
result: int = 5 // 2  # Now result is 2 (exact)
print(type(result))   # <class 'int'>
```

### Mistake 3: Float Precision Issues (Advanced)

```python
result: float = 0.1 + 0.2
print(result)  # Output: 0.30000000000000004 (not 0.3!)
```

**Why it happens**: IEEE 754 floating-point representation can't store 0.1 exactly. Tiny rounding errors accumulate.

**When it matters**: Financial calculations (use the `Decimal` module instead).

**For most cases**: This is fine. Use `round()` if you need clean output:

```python
print(round(0.1 + 0.2, 1))  # Output: 0.3
```

### Mistake 4: Using Float for Money (Anti-Pattern)

```python
# DON'T: Using float for financial calculations
price: float = 19.99
# Precision issues make this risky for transactions
```

**Why it matters**: Floating-point rounding can cause tiny errors that become big problems at scale.

**Fix**: For production financial systems, use the `Decimal` module (covered in later chapters).

```python
from decimal import Decimal
price: Decimal = Decimal("19.99")  # Exact representation
```

### Mistake 5: Confusing Type Hints with Type Conversion

```python
# Type hint says what type the variable SHOULD hold
value: int = 5  # ✓ Correct: value is an integer

# Python doesn't enforce type hints at runtime (that's optional linting)
value: int = "hello"  # Python allows this, but it's wrong!
print(type(value))    # <class 'str'> — violated the hint
```

**The lesson**: Type hints are promises to yourself and other developers, not enforced rules. Tools like `mypy` check them, but Python runtime doesn't.

---

## Practice Exercises

### Exercise 1: Classify 15 Numbers

Classify each value as `int`, `float`, or `complex`. Write your reasoning.

```
1. 42
2. 3.14
3. -5
4. 0.5
5. 100
6. 2 + 3j
7. -0.001
8. 1000000
9. 0.0
10. 1e10
11. -3 - 4j
12. 99.99
13. 0
14. 7.5
15. 2e-5
```

**Check your work with `type()`:**

```python
values = [42, 3.14, -5, 0.5, 100, 2+3j, -0.001, 1000000, 0.0, 1e10, -3-4j, 99.99, 0, 7.5, 2e-5]
for v in values:
    print(f"{v} is {type(v)}")
```

### Exercise 2: Fix the Type Errors

Identify the type mismatch in each code snippet. Explain what's wrong, then fix it.

```python
# Snippet 1
student_count: int = 25.7
print(student_count)

# Snippet 2
result: float = 10 / 3
print(f"10 divided by 3 is {result}")

# Snippet 3
total_price: float = 19.99
tax: float = 0.08
final_price: int = total_price * (1 + tax)

# Snippet 4
age: int = 5 / 2
print(age)
```

### Exercise 3: Type Inspection

Use `type()` to verify the type of each calculation. Predict the type BEFORE running code, then verify:

```python
# Predict type, then check:
a = 10 + 5
b = 10.0 + 5
c = 10 + 5.0
d = 10 / 2
e = 10 // 2
f = 10.0 / 2
g = type(10)  # Special: what's the type of type()?

for value in [a, b, c, d, e, f, g]:
    print(f"{value} has type {type(value)}")
```

---

## Try With AI

Open ChatGPT or your AI companion tool. Use these prompts in sequence:

**Prompt 1: Generate Scenarios**
> "Give me 20 real-world data scenarios (like 'number of employees', 'average temperature', 'price per item'). For each, should I use int or float? Format as a numbered list with your reasoning."

**Expected Outcome**: AI generates diverse scenarios. You'll see patterns in when decimals matter.

**Prompt 2: Test Your Understanding**
> "Classify these 5 values and tell me the reasoning: 17, -3.5, 50.0, 0, 2+4j. Then explain why complex numbers are rarely used."

**Expected Outcome**: AI confirms your type classification and explains when you'd need complex numbers (advanced use cases).

**Prompt 3: Complex Numbers Exposure**
> "Create 3 real-world examples where complex numbers solve problems. For each, explain the real and imaginary parts."

**Expected Outcome**: You'll see that complex numbers have genuine applications, even if you won't use them soon.

**Stretch Prompt (For Advanced Learners)**
> "Explain the IEEE 754 floating-point standard and why 0.1 + 0.2 ≠ 0.3 in Python. Show me how to work around this issue."

**Expected Outcome**: Deep understanding of why float precision is an issue (and when to care).

**Safety Note**: AI-generated code might have type hints. Always verify the logic makes sense. Ask "Why did you choose float instead of int?" to deepen understanding.
