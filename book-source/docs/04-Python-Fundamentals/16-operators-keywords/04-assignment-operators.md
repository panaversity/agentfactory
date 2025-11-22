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

## Tracking Player Health

You're tracking a player's health. It starts at 100, and they take 25 damage. How do you update it?

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

## The Five Assignment Operators

### Addition Assignment: Counting Visitors

You're counting website visitors. Each visit adds 1 to the counter.

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

### Subtraction Assignment: Managing Inventory

You're tracking inventory. You have 50 items and sell 12.

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

### Multiplication Assignment: Compound Growth

You're calculating compound growth. A value doubles each round.

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

### Division Assignment: Splitting Bills

You're splitting a bill. Total is $120 among 4 people.

```python
total = 120

# Split among 4
total /= 4
print(total)  # What does each person pay?
```

You get `30.0`. Notice it's a float, even though 120 divides evenly by 4.

**Important**: `/=` always produces a float, just like `/`.

### Floor Division Assignment: Whole Number Results

You need whole number results when dividing.

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

## Common Patterns in Games

### The Counter Pattern

The most common use of `+=` is counting things:

```python
count = 0

# Process items one by one
count += 1
print(f"Processed {count}: apple")

count += 1
print(f"Processed {count}: banana")

count += 1
print(f"Processed {count}: cherry")

print(f"Total: {count}")
```

You'll use `count += 1` constantly when you learn loops in Chapter 18—that's when you'll automate this pattern instead of writing each line manually.

### The Accumulator Pattern

Adding up values as you go:

```python
total = 0

# Add up prices one by one
total += 10.50
print(f"Running total: ${total:.2f}")

total += 20.00
print(f"Running total: ${total:.2f}")

total += 15.75
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

### Challenge 2: Health and Mana

Track player resources during combat:

```python
health = 100
mana = 50

# Take damage (-30)
health -= 30
print(f"After hit: Health={health}")

# Use healing spell (costs 20 mana, restores 25 health)
mana -= 20
health += 25
print(f"After heal: Health={health}, Mana={mana}")

# Drink mana potion (+40 mana)
mana += 40
print(f"After potion: Health={health}, Mana={mana}")
```

### Challenge 3: Experience and Leveling

Track XP with multipliers:

```python
xp = 0

# Defeat enemies (+50 each)
xp += 50
xp += 50
xp += 50
print(f"After enemies: {xp} XP")

# Apply XP boost (double)
xp *= 2
print(f"After boost: {xp} XP")

# Complete quest (+100)
xp += 100
print(f"After quest: {xp} XP")
```

## Debugging With AI

When your variable updates don't work as expected, here's how to get help:

**"The variable isn't changing"**

You want to subtract 10 from the score:

```python
score = 100
score - 10
print(score)  # Still 100!
```

Tell AI: *"I subtracted 10 from score but it's still 100."*

AI will explain: `score - 10` calculates the result but doesn't save it anywhere. You need `score = score - 10` or the shortcut `score -= 10`.

**"I'm getting decimals instead of whole numbers"**

You want to split 10 coins between 2 players:

```python
coins = 10
coins /= 2
print(coins)  # 5.0 (not 5)
```

Tell AI: *"I divided coins by 2 but got 5.0 instead of 5. I want a whole number."*

AI will explain: `/=` always gives decimals. Use `//=` for whole number results.

**"I'm confused about = vs =="**

You want to check if score equals 100:

```python
score = 100
is_winner = score = 100  # This is assignment, not comparison!
print(is_winner)  # Prints 100, not True or False
```

Tell AI: *"I want to check if score equals 100 but I used = instead of ==."*

AI will explain: `=` assigns a value, `==` checks equality. Use `is_winner = score == 100` to get True or False.

## Try With AI

**Explore equivalence:**
> "Show me that `x += 5` and `x = x + 5` do exactly the same thing. Are there any cases where they behave differently?"

**Solve a real problem:**
> "I'm building a fitness tracker. I need to track: steps (add each walk), calories (subtract each meal), and distance (accumulate). Show me how to use assignment operators for each."

**Understand types:**
> "What happens to the type when I use `/=` on an integer? Why does `//=` keep it as an integer? Show examples."
