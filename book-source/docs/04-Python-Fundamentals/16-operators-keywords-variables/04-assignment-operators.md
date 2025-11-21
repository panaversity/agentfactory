---
title: "Assignment Operators"
sidebar_position: 4
---

# Assignment Operators

## What You'll Learn

- Five Assignment Operators
- Shorthand vs Expanded Form
- Counter and Accumulator Patterns
- Common Update Operations

## The Problem

You're building a game score tracker. The player starts with 100 points, collects 50 more, loses 25, then doubles their score with a power-up. You need to update the same variable multiple times.

## Why You Need Assignment Operators

Assignment operators are shortcuts for updating variables. Without them, you'd write `score = score + 50` every time. With them, you write `score += 50`. This matters because:

- Cleaner code that's easier to read
- Less typing and fewer mistakes
- Standard patterns everyone recognizes (counters, accumulators)
- Expresses intent clearly: "add to this variable"

Let's discover the shortcuts.

## Discovery: What's the Shortcut?

**Problem**: You're tracking a player's health. It starts at 100, and they take 25 damage. How do you update it?

```python
health = 100

# Long way
health = health - 25
print(health)  # What do you get?
```

You get `75`. But there's a shorter way:

```python
health = 100

# Short way
health -= 25
print(health)  # What do you get?
```

Same result: `75`. The `-=` operator subtracts and assigns in one step.

**Pattern discovered**: `variable -= value` is the same as `variable = variable - value`.

## Discovery: The Five Assignment Operators

### Addition Assignment: `+=`

**Problem**: You're counting website visitors. Each visit adds 1 to the counter.

```python
visitors = 0

# A visitor arrives
visitors += 1
print(visitors)  # What do you get?

# Another visitor
visitors += 1
print(visitors)  # What now?

# Five more visitors
visitors += 5
print(visitors)  # Final count?
```

You get `1`, then `2`, then `7`. The `+=` operator adds to the current value.

**Equivalence**:
```python
# These do the same thing:
count = count + 1
count += 1
```

### Subtraction Assignment: `-=`

**Problem**: You're tracking inventory. You have 50 items and sell 12.

```python
inventory = 50

# Sell items
inventory -= 12
print(inventory)  # What's left?

# Sell more
inventory -= 8
print(inventory)  # Now?
```

You get `38`, then `30`.

### Multiplication Assignment: `*=`

**Problem**: You're calculating compound growth. A value doubles each round.

```python
value = 1

# Round 1: double it
value *= 2
print(value)  # ?

# Round 2: double again
value *= 2
print(value)  # ?

# Round 3: double again
value *= 2
print(value)  # ?
```

You get `2`, then `4`, then `8`.

**Real use case**: Applying percentage increases:

```python
price = 100

# Apply 10% markup
price *= 1.10
print(price)  # 110.0
```

### Division Assignment: `/=`

**Problem**: You're splitting a bill. Total is $120 among 4 people.

```python
total = 120

# Split among 4
total /= 4
print(total)  # What does each person pay?
```

You get `30.0`. Notice it's a float, even though 120 divides evenly by 4.

**Important**: `/=` always produces a float, just like `/`.

### Floor Division Assignment: `//=`

**Problem**: You need whole number results when dividing.

```python
items = 25
boxes = 25

# How many items per box (whole numbers only)?
items //= 4
print(items)  # ?
```

You get `6`. Floor division drops the decimal.

## Reference Table: All Assignment Operators

| Operator | Name | Example | Equivalent To |
|----------|------|---------|---------------|
| `=` | Assign | `x = 5` | — |
| `+=` | Add assign | `x += 3` | `x = x + 3` |
| `-=` | Subtract assign | `x -= 3` | `x = x - 3` |
| `*=` | Multiply assign | `x *= 3` | `x = x * 3` |
| `/=` | Divide assign | `x /= 3` | `x = x / 3` |
| `//=` | Floor divide assign | `x //= 3` | `x = x // 3` |

## Discovery: Common Patterns

### The Counter Pattern

The most common use of `+=` is counting things:

```python
count = 0

# Process items
items = ["apple", "banana", "cherry"]

for item in items:
    count += 1
    print(f"Processed {count}: {item}")

print(f"Total: {count}")
```

You'll use `count += 1` constantly when you learn loops in Chapter 18.

### The Accumulator Pattern

Adding up values as you go:

```python
total = 0

# Add up prices
prices = [10.50, 20.00, 15.75]

for price in prices:
    total += price
    print(f"Running total: ${total:.2f}")

print(f"Final total: ${total:.2f}")
```

### The Multiplier Pattern

Applying repeated multiplications:

```python
# Compound interest: 5% growth for 3 years
principal = 1000
rate = 1.05

principal *= rate  # Year 1
print(f"After year 1: ${principal:.2f}")

principal *= rate  # Year 2
print(f"After year 2: ${principal:.2f}")

principal *= rate  # Year 3
print(f"After year 3: ${principal:.2f}")
```

## Practice: Solve Real Problems

### Challenge 1: Game Score Tracker

Track a player's score through various events:

```python
score = 0

# Collect coins (+10 each)
score += 10
score += 10
score += 10
print(f"After coins: {score}")

# Hit by enemy (-15)
score -= 15
print(f"After hit: {score}")

# Power-up doubles score
score *= 2
print(f"After power-up: {score}")
```

**Extend it**: Add more events. What's the final score?

### Challenge 2: Shopping Cart

Calculate a cart total with discounts:

```python
cart = 0

# Add items
cart += 29.99  # Shirt
cart += 49.99  # Pants
cart += 15.00  # Socks
print(f"Subtotal: ${cart:.2f}")

# Apply 20% discount
cart *= 0.80
print(f"After discount: ${cart:.2f}")

# Add tax (8%)
cart *= 1.08
print(f"Final total: ${cart:.2f}")
```

### Challenge 3: Resource Management

Track resources in a simulation:

```python
fuel = 100
distance = 0

# Travel 30 units (costs 15 fuel)
distance += 30
fuel -= 15
print(f"Distance: {distance}, Fuel: {fuel}")

# Travel 50 more units (costs 25 fuel)
distance += 50
fuel -= 25
print(f"Distance: {distance}, Fuel: {fuel}")

# Refuel (+40)
fuel += 40
print(f"Distance: {distance}, Fuel: {fuel}")
```

## Common Mistakes

**Mistake 1: Forgetting the equals sign**

```python
# Wrong: This is subtraction, not assignment
score = 100
score - 10  # Does nothing to score!
print(score)  # Still 100

# Right: Use -=
score -= 10
print(score)  # 90
```

**Mistake 2: Using assignment in comparisons**

```python
score = 100

# Wrong: = assigns, doesn't compare
# if score = 100:  # SyntaxError!

# Right: == compares
if score == 100:
    print("Perfect score!")
```

**Mistake 3: Expecting integer from `/=`**

```python
value = 10

# This becomes a float
value /= 2
print(value)       # 5.0 (not 5)
print(type(value)) # <class 'float'>

# Use //= for integer result
value = 10
value //= 2
print(value)       # 5
print(type(value)) # <class 'int'>
```

## Try With AI

**Explore equivalence:**
> "Show me that `x += 5` and `x = x + 5` do exactly the same thing. Are there any cases where they behave differently?"

**Solve a real problem:**
> "I'm building a fitness tracker. I need to track: steps (add each walk), calories (subtract each meal), and distance (accumulate). Show me how to use assignment operators for each."

**Understand types:**
> "What happens to the type when I use `/=` on an integer? Why does `//=` keep it as an integer? Show examples."
