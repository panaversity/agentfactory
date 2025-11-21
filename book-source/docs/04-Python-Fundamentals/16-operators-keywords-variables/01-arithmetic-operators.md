---
title: "Arithmetic Operators"
sidebar_position: 1
---

# Arithmetic Operators

## What You'll Learn

- Seven Arithmetic Operators
- Division vs Floor Division
- Modulus (Remainders)
- Exponentiation (Powers)
- Operator Precedence

## The Problem

You're building a fuel tracker for a road trip app. The tank starts with 100 liters, the journey uses 37, then you refuel 25 more. You know the math—but how do you tell Python to calculate it?

## Why You Need Arithmetic Operators

Arithmetic operators are symbols that tell Python to perform calculations. You already know addition, subtraction, multiplication from math class. Python uses the same concepts, just with specific symbols. Without these operators, you couldn't:

- Calculate totals, differences, or products
- Split items into groups
- Find remainders
- Compute powers

Let's discover how Python does math.

## Discovery: What Operators Does Python Have?

Let's start with a problem and discover how Python solves it.

**Problem**: You're tracking game scores. Player 1 has 50 points, Player 2 has 30. What's the combined score?

```python
# Your prediction: What symbol adds numbers in Python?
player1 = 50
player2 = 30
total = player1 + player2
print(total)
```

**Run it.** You get `80`. The `+` symbol works exactly like math class.

Now try the others:

```python
# Experiment: Try each operator
a = 20
b = 6

print(a + b)   # Addition - what do you get?
print(a - b)   # Subtraction
print(a * b)   # Multiplication
print(a / b)   # Division
```

**What did you discover?**
- `+` adds (26)
- `-` subtracts (14)
- `*` multiplies (120)
- `/` divides (3.333...)

These four work like math class. But Python has three more operators you might not know.

## Discovery: The Three Special Operators

### Floor Division: Counting Whole Groups

**Problem**: You're organizing a party. You have 20 cookies to put in gift bags that hold 6 each. How many full bags can you make?

```python
cookies = 20
bag_size = 6

# Try regular division first
print(cookies / bag_size)   # What do you get?
```

You get `3.333...` but you can't make 3.33 bags. You need whole bags only.

```python
# Now try floor division
print(cookies // bag_size)  # What do you get?
```

You get `3`. The `//` operator divides and drops the decimal—it gives whole groups only.

**Pattern discovered**: `/` gives decimal results, `//` gives whole number results.

### Modulus: Finding What's Left Over

**Problem**: After filling 3 bags with 6 cookies each, how many cookies are left over?

```python
cookies = 20
bag_size = 6

# The modulus operator finds remainders
leftover = cookies % bag_size
print(leftover)  # What do you get?
```

You get `2`. The `%` operator gives the remainder after division.

**Try it yourself**: What's `17 % 5`? Predict first, then run it.

Common uses for modulus:
- Check if even: `number % 2` gives 0 if even
- Get last digit: `number % 10` gives the ones place
- Cycle through values: `count % 3` cycles 0, 1, 2, 0, 1, 2...

### Exponentiation: Powers

**Problem**: You're calculating compound growth. If something doubles 8 times, what's the multiplier? (2×2×2×2×2×2×2×2)

```python
# Exponentiation uses **
result = 2 ** 8
print(result)  # What do you get?
```

You get `256`. The `**` operator raises numbers to powers.

```python
# More examples
print(3 ** 2)   # 9 (3 squared)
print(10 ** 3)  # 1000 (10 cubed)
print(5 ** 0)   # 1 (anything to the 0 is 1)
```

**Warning**: Don't use `^` for powers—it does something else in Python. Always use `**`.

## Reference Table: All Seven Operators

| Operator | Name | Example | Result | Use When... |
|----------|------|---------|--------|-------------|
| `+` | Addition | `10 + 3` | `13` | Combining values |
| `-` | Subtraction | `10 - 3` | `7` | Finding differences |
| `*` | Multiplication | `10 * 3` | `30` | Scaling values |
| `/` | Division | `10 / 3` | `3.333...` | Need decimal precision |
| `//` | Floor Division | `10 // 3` | `3` | Need whole groups |
| `%` | Modulus | `10 % 3` | `1` | Need remainders |
| `**` | Exponentiation | `10 ** 3` | `1000` | Need powers |

## Discovery: Order of Operations

**Problem**: What does `2 + 3 * 4` equal?

```python
# Predict the result before running
result = 2 + 3 * 4
print(result)
```

Did you get `14` or `20`?

If you got `14`, you're right. Python follows math rules: multiplication before addition.

```python
# Python evaluates: 3 * 4 = 12, then 2 + 12 = 14
print(2 + 3 * 4)   # 14

# Use parentheses to change the order
print((2 + 3) * 4)  # 20
```

**The pattern**: PEMDAS applies—Parentheses, Exponents, Multiplication/Division, Addition/Subtraction.

**Best practice**: Use parentheses to make your intent clear, even when not required.

```python
# Confusing without parentheses
total = 100 * 1.08 + 5

# Clear with parentheses
total = (100 * 1.08) + 5  # Tax first, then add fee
```

## Practice: Solve Real Problems

### Challenge 1: Fuel Calculator

You're tracking fuel for your road trip app. Calculate the remaining fuel:

```python
# Starting fuel
fuel = 100

# Journey uses 37 liters
fuel = fuel - 37

# Refuel adds 25 liters
fuel = fuel + 25

print(fuel)  # What's left?
```

**Extend it**: Add another journey that uses 42 liters. What's the final amount?

### Challenge 2: Pizza Distribution

You have 23 slices of pizza for 5 people.

```python
slices = 23
people = 5

# How many slices per person?
per_person = slices // people
print(f"Each person gets {per_person} slices")

# How many slices left over?
leftover = slices % people
print(f"Leftover slices: {leftover}")
```

**Change the numbers**: Try 17 slices for 4 people. What happens?

### Challenge 3: Compound Growth

A value doubles every cycle. Start with 1 and double it 10 times:

```python
# Method 1: Exponentiation
result = 2 ** 10
print(f"After 10 doublings: {result}")

# What if it triples instead?
result = 3 ** 10
print(f"After 10 triplings: {result}")
```

## Common Mistakes

**Mistake 1: Using `/` when you need whole numbers**

```python
# Wrong: You can't have 3.33 boxes
boxes = 20 / 6   # 3.333...

# Right: Use floor division for counting
boxes = 20 // 6  # 3
```

**Mistake 2: Using `^` for powers**

```python
# Wrong: ^ is not exponentiation in Python
result = 2 ^ 3   # This gives 1 (bitwise XOR)

# Right: Use **
result = 2 ** 3  # This gives 8
```

**Mistake 3: Forgetting parentheses**

```python
# Wrong: Tax calculated incorrectly
total = 100 + 100 * 0.08   # 108 (adds 8% of 100 only)

# Right: Calculate price first, then add tax
total = (100 + 100) * 0.08  # Depends on what you want
# Or: total = 100 * 1.08 for price with tax
```

## Try With AI

**Explore edge cases:**
> "What happens when I divide by zero in Python? Show me 10 / 0 and 10 // 0. What error appears and why?"

**Solve a real problem:**
> "I'm building a timer. I have 3661 seconds and need to convert to hours, minutes, and seconds. Which operators should I use? Show me step by step."

**Understand precedence:**
> "Break down how Python evaluates: 2 ** 3 * 4 + 5. Show each step in order."

## What You Discovered

- Python has 7 arithmetic operators: `+`, `-`, `*`, `/`, `//`, `%`, `**`
- `/` gives decimals, `//` gives whole numbers
- `%` finds remainders (useful for even/odd, cycling, leftovers)
- `**` raises to powers (not `^`)
- Parentheses control order and clarify intent

**Next**: You'll discover comparison operators—how to ask Python True/False questions like "Is this greater than that?"
