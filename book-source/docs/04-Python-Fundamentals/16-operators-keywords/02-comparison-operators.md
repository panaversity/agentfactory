---
title: "Comparison Operators"
sidebar_position: 2
---

# Comparison Operators

## What You'll Learn

- Six Comparison Operators
- Equality vs Assignment
- Greater/Less Than Comparisons
- Boolean Results
- Common Comparison Patterns

## The Problem

Your game needs to check conditions: Did the player reach 100 points to win? Do they have enough health to survive? Is their score higher than the current high score? You're not calculating—you're asking yes/no questions.

## Why You Need Comparison Operators

Comparison operators ask questions and return answers: `True` or `False`. Without them, your game couldn't:

- Check if player reached the winning score
- Verify if health dropped to zero (game over)
- Test if player beat the high score
- Make any decisions based on game state

Every comparison gives you `True` or `False`. Let's discover how to ask these questions.

## Checking Win Conditions

Player has 85 points. They need 100 to win. Did they win?

```python
score = 85
win_threshold = 100

# Your prediction: What will this return?
player_won = score >= win_threshold
print(player_won)
```

**Run it.** You get `False`. The `>=` operator asks "is left side greater than or equal to right side?" and returns the answer.

Now try other comparisons:

```python
# Experiment: Try each comparison
a = 10
b = 5

print(a == b)   # Equal to - what do you get?
print(a != b)   # Not equal to
print(a > b)    # Greater than
print(a < b)    # Less than
```

**What did you discover?**
- `==` checks equality (False - 10 is not equal to 5)
- `!=` checks inequality (True - 10 is different from 5)
- `>` checks greater than (True - 10 is greater than 5)
- `<` checks less than (False - 10 is not less than 5)

**Important**: Notice `==` uses two equals signs. One `=` assigns a value, two `==` compares values. This is a common mistake.

## The Six Comparison Operators

### Equality and Inequality: Code Matching

Player enters a secret code to unlock a bonus level. Did they enter the right code?

```python
entered_code = 1234
secret_code = 1234

# Check if they match
unlocked = entered_code == secret_code
print(unlocked)  # What do you get?
```

You get `True`. Now check the opposite:

```python
# Check if they're different
wrong_code = entered_code != secret_code
print(wrong_code)  # What do you get?
```

You get `False`. They're not different—they match.

**Pattern discovered**: `==` returns True if values match, `!=` returns True if values differ.

### Greater Than and Less Than: Health Thresholds

Player has 75 health. Is it above the danger threshold of 30?

```python
health = 75
danger_threshold = 30

# Is health above danger?
is_safe = health > danger_threshold
print(is_safe)  # What do you get?
```

You get `True`. Now check if it's critical (below 10):

```python
critical = 10

is_critical = health < critical
print(is_critical)  # What do you get?
```

You get `False`. 75 is not less than 10.

### Greater/Less Than OR Equal: Level Unlocks

Player needs exactly 100 points or more to unlock the next level.

```python
score = 100
unlock_threshold = 100

# Does 100 unlock it? (must be 100 OR more)
unlocked = score >= unlock_threshold
print(unlocked)  # What do you get?
```

You get `True`. The `>=` includes the equal case.

**Try it**: What if score is 99? What about 101?

```python
# Experiment with different scores
print(99 >= 100)   # ?
print(100 >= 100)  # ?
print(101 >= 100)  # ?
```

**Pattern discovered**: `>=` and `<=` include the equal case, `>` and `<` exclude it.

## Reference Table: All Six Operators

| Operator | Name | Example | Result | Question Asked |
|----------|------|---------|--------|----------------|
| `==` | Equal | `5 == 5` | `True` | Are they the same? |
| `!=` | Not equal | `5 != 3` | `True` | Are they different? |
| `>` | Greater than | `5 > 3` | `True` | Is left bigger? |
| `<` | Less than | `5 < 3` | `False` | Is left smaller? |
| `>=` | Greater or equal | `5 >= 5` | `True` | Is left bigger or same? |
| `<=` | Less or equal | `5 <= 3` | `False` | Is left smaller or same? |

## Comparisons Always Return Booleans

Every comparison returns either `True` or `False`—these are called **boolean values**.

```python
# All comparisons return True or False
result1 = 10 > 5
result2 = 10 < 5

print(result1)        # True
print(result2)        # False
print(type(result1))  # What type is this?
```

You get `<class 'bool'>`. Boolean is Python's type for True/False values.

**Why this matters**: In Chapter 18, you'll use these True/False values to make decisions with `if` statements. The comparison gives you the answer, the `if` acts on it.

## Value Equality Surprises

Is the integer 5 equal to the float 5.0?

```python
int_five = 5
float_five = 5.0

# Predict: True or False?
print(int_five == float_five)
```

**Surprising result**: You get `True`. Python compares **values**, not types. 5 and 5.0 represent the same value.

Now try this:

```python
int_five = 5
string_five = "5"

# Predict: True or False?
print(int_five == string_five)
```

You get `False`. The string "5" is text, not a number—completely different values.

**Pattern discovered**:
- Same value, different numeric types → `True` (5 == 5.0)
- Different value types entirely → `False` (5 == "5")

## Practice: Solve Real Problems

### Challenge 1: Level Progression

Check if player qualifies for different game milestones:

```python
player_score = 85

# Bronze badge (50+)
bronze = player_score >= 50
print(f"Bronze badge: {bronze}")

# Silver badge (100+)
silver = player_score >= 100
print(f"Silver badge: {silver}")

# Gold badge (200+)
gold = player_score >= 200
print(f"Gold badge: {gold}")
```

**Change the score**: Try 100, 200, and 45. What changes?

### Challenge 2: Inventory Check

Check if player has enough items. First, you need to know how many they have.

**The `len()` function**: Python has a built-in function called `len()` that counts items in a list (or characters in a string). It returns a number.

```python
# len() counts items
inventory = ["sword", "shield", "potion"]
item_count = len(inventory)
print(item_count)  # 3
```

Now use it with comparisons:

```python
inventory = ["sword", "shield", "potion", "key", "map"]
required_items = 3
max_capacity = 10

# Has enough items for quest?
ready_for_quest = len(inventory) >= required_items
print(f"Ready for quest: {ready_for_quest}")

# Inventory full?
is_full = len(inventory) >= max_capacity
print(f"Inventory full: {is_full}")

# Has room for more?
has_room = len(inventory) < max_capacity
print(f"Has room: {has_room}")
```

**Try it**: What if inventory has only 2 items? What if it has 10?

### Challenge 3: High Score Check

Check if player beat various records:

```python
player_score = 850
high_score = 1000
personal_best = 850

# Beat the high score?
new_record = player_score > high_score
print(f"New high score: {new_record}")

# Tied personal best?
tied_best = player_score == personal_best
print(f"Tied personal best: {tied_best}")

# Below high score?
keep_trying = player_score < high_score
print(f"Keep trying: {keep_trying}")
```

## Debugging With AI

When your comparisons give unexpected results, here's how to get help:

**"I get a syntax error when checking equality"**

You want to check if score equals 100:

```python
score = 100
if score = 100:  # SyntaxError!
    print("Won!")
```

Tell AI: *"I'm trying to check if score equals 100, but I get a syntax error."*

AI will explain: Use `==` for comparison, not `=`. One equals sign assigns a value, two equals signs compare values.

**"Score of 100 doesn't pass my check"**

You want players with 100 or more to win:

```python
score = 100
won = score > 100
print(won)  # False (but 100 should win!)
```

Tell AI: *"A score of exactly 100 should win, but my check returns False."*

AI will explain: `>` means "greater than" (excludes 100). Use `>=` for "greater than or equal to".

**"My comparison is always False"**

You compare user input to a number:

```python
user_score = input("Enter score: ")  # User types 100
print(user_score == 100)  # False (why?)
```

Tell AI: *"I typed 100 but the comparison to 100 is False."*

AI will explain: `input()` returns text, not a number. `"100"` (text) is not the same as `100` (number). Convert it with `int()`.

## Try With AI

**Explore edge cases:**
> "What happens when I compare True to 1 and False to 0? Are booleans secretly numbers in Python?"

**Solve a real problem:**
> "I'm building a thermostat. I need to check if temperature is below 60 (turn on heat), above 75 (turn on AC), or between (do nothing). What comparisons do I need?"

**Understand the difference:**
> "Explain why `=` and `==` exist as separate operators. Show me what goes wrong if I accidentally use the wrong one."
