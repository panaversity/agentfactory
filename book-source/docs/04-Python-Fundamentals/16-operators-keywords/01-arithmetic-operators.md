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

You're building a game. Players collect coins, defeat enemies, and earn bonuses. You need to calculate scores: 10 coins worth 5 points each, a bonus multiplier of 2x, and split points among team members. How do you tell Python to do this math?

## Why You Need Arithmetic Operators

Arithmetic operators are symbols that tell Python to perform calculations. Without them, your game couldn't:

- Calculate total points from coins collected
- Apply bonus multipliers to scores
- Split rewards among team members
- Compute power-ups and level bonuses

Let's discover how Python does the math for your game.

## Calculating Team Scores

Let's start by discovering how Python does math.

Player 1 scored 50 points, Player 2 scored 30. What's the team's combined score?

```python
# Your prediction: What symbol adds numbers in Python?
player1 = 50
player2 = 30
team_score = player1 + player2
print(team_score)
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

## The Three Special Operators

### Floor Division: Distributing Power-Ups Equally

You have 20 power-ups to distribute equally among 6 players. How many does each player get?

```python
power_ups = 20
players = 6

# Try regular division first
print(power_ups / players)   # What do you get?
```

You get `3.333...` but you can't give 3.33 power-ups. You need whole numbers only.

```python
# Now try floor division
print(power_ups // players)  # What do you get?
```

You get `3`. The `//` operator divides and drops the decimal—each player gets 3 power-ups.

**Pattern discovered**: `/` gives decimal results, `//` gives whole number results.

### Modulus: What's Left in the Pool?

After giving 3 power-ups to each of 6 players, how many are left over?

```python
power_ups = 20
players = 6

# The modulus operator finds remainders
leftover = power_ups % players
print(leftover)  # What do you get?
```

You get `2`. The `%` operator gives the remainder—2 power-ups left in the pool.

**Try it yourself**: What's `17 % 5`? Predict first, then run it.

Common uses for modulus:
- Check if even: `number % 2` gives 0 if even
- Get last digit: `number % 10` gives the ones place
- Cycle through values: `count % 3` cycles 0, 1, 2, 0, 1, 2...

### Exponentiation: Combo Multipliers

Your game has a combo system. Each consecutive hit doubles the bonus. After 8 consecutive hits, what's the multiplier? (2×2×2×2×2×2×2×2)

```python
# Exponentiation uses **
combo_multiplier = 2 ** 8
print(combo_multiplier)  # What do you get?
```

You get `256`. The `**` operator raises numbers to powers—that's a 256x combo bonus!

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

## Order of Operations

What does `2 + 3 * 4` equal?

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

### Challenge 1: Score Calculator

Calculate total score from different point sources:

```python
# Points from different sources
coins = 10
coin_value = 5
enemies = 3
enemy_value = 20

# Calculate totals
coin_points = coins * coin_value
enemy_points = enemies * enemy_value
total_score = coin_points + enemy_points

print(f"Coin points: {coin_points}")
print(f"Enemy points: {enemy_points}")
print(f"Total score: {total_score}")
```

**Extend it**: Add bonus points of 50. What's the new total?

### Challenge 2: Loot Distribution

You have 23 gold coins to split among 5 party members.

```python
gold = 23
members = 5

# How much does each member get?
per_member = gold // members
print(f"Each member gets {per_member} gold")

# How much is left for the guild bank?
guild_bank = gold % members
print(f"Guild bank gets: {guild_bank}")
```

**Change the numbers**: Try 100 gold among 7 members. What happens?

### Challenge 3: Combo Multiplier

Your combo system multiplies damage. Calculate the multiplier for different combo lengths:

```python
# Base multiplier doubles with each hit
print(f"5-hit combo: {2 ** 5}x")   # 32x
print(f"10-hit combo: {2 ** 10}x") # 1024x

# What if it triples instead?
print(f"5-hit triple combo: {3 ** 5}x")  # 243x
```

## Debugging With AI

When your code gives unexpected results, here's how to get help:

**"I got decimals but need whole numbers"**

You want to split 20 power-ups among 6 players:

```python
per_player = 20 / 6
print(per_player)  # 3.333...
```

Tell AI: *"I'm splitting 20 items among 6 players but getting 3.333. I need whole numbers."*

AI will suggest using `//` (floor division) instead of `/`.

**"I expected 8 but got 1"**

You want 2 to the power of 3:

```python
result = 2 ^ 3
print(result)  # 1 (not 8!)
```

Tell AI: *"I tried 2 ^ 3 expecting 8, but got 1. How do I calculate powers?"*

AI will explain that Python uses `**` for powers, not `^`.

**"My calculation gives the wrong answer"**

You want to add two scores, then take 8%:

```python
total = 100 + 100 * 0.08
print(total)  # 108 (not 16!)
```

Tell AI: *"I want (100 + 100) × 0.08 = 16, but I'm getting 108."*

AI will explain that multiplication happens before addition, and show you how to use parentheses.

## Try With AI

**Explore edge cases:**
> "What happens when I divide by zero in Python? Show me 10 / 0 and 10 // 0. What error appears and why?"

**Solve a real problem:**
> "I'm building a timer. I have 3661 seconds and need to convert to hours, minutes, and seconds. Which operators should I use? Show me step by step."

**Understand precedence:**
> "Break down how Python evaluates: 2 ** 3 * 4 + 5. Show each step in order."
