---
title: "Logical Operators"
sidebar_position: 3
---

# Logical Operators

## What You'll Learn

- Three Logical Operators
- Combining Conditions with `and`
- Alternative Conditions with `or`
- Inverting with `not`
- Truth Tables

## The Problem

Your game has complex rules. To enter the boss level, the player must have score >= 100 AND health > 0. To use a special attack, they need mana >= 50 OR have a power potion. One comparison isn't enough—you need to combine conditions.

## Why You Need Logical Operators

Logical operators combine True/False values to make complex decisions. Without them, your game couldn't:

- Check win conditions (score >= 100 AND lives > 0)
- Allow alternative abilities (has sword OR has magic)
- Block defeated players (NOT alive)
- Express complex game rules

Instead of asking one question, you can ask: "Is this true AND that true?" or "Is this true OR that true?" Let's discover how.

## Combining Conditions for Boss Entry

To enter the boss level, player needs score >= 100 AND health > 0. Can this player enter?

```python
score = 120
health = 50

# Your prediction: What will this return?
can_enter_boss = score >= 100 and health > 0
print(can_enter_boss)
```

**Run it.** You get `True`. The `and` operator returns True only when BOTH conditions are True.

Now change the values:

```python
score = 120
health = 0  # Player is defeated

can_enter_boss = score >= 100 and health > 0
print(can_enter_boss)  # What now?
```

You get `False`. Even though score is high enough, health is zero. Both must be true.

## The Three Logical Operators

### The `and` Operator: Unlocking Treasure Chests

To unlock the treasure chest, player needs key AND lockpick skill >= 5.

```python
has_key = True
lockpick_skill = 7

# Both conditions must be true
can_open = has_key and lockpick_skill >= 5
print(can_open)  # What do you get?
```

You get `True`. Both conditions are met.

**Experiment**: What happens when one condition fails?

```python
# Test different combinations
print(True and True)    # ?
print(True and False)   # ?
print(False and True)   # ?
print(False and False)  # ?
```

**Pattern discovered**: `and` returns True ONLY when both sides are True.

### The `or` Operator: Alternative Attack Methods

Player can attack if they have sword OR magic spell available.

```python
has_sword = False
has_magic = True

# At least one must be true
can_attack = has_sword or has_magic
print(can_attack)  # What do you get?
```

You get `True`. They have magic, so they can attack.

**Experiment**: What happens with different combinations?

```python
# Test different combinations
print(True or True)    # ?
print(True or False)   # ?
print(False or True)   # ?
print(False or False)  # ?
```

**Pattern discovered**: `or` returns True when AT LEAST ONE side is True. Only False when both are False.

### The `not` Operator: Checking Movement Status

Player can move if they are NOT frozen.

```python
is_frozen = False

# Flip the value
can_move = not is_frozen
print(can_move)  # What do you get?
```

You get `True`. The player is not frozen, so they can move.

**Experiment**: What does `not` do to each value?

```python
print(not True)   # ?
print(not False)  # ?
```

**Pattern discovered**: `not` flips True to False and False to True.

## Reference Table: Truth Tables

### `and` - Both must be True

| A | B | A and B |
|---|---|---------|
| True | True | **True** |
| True | False | False |
| False | True | False |
| False | False | False |

### `or` - At least one must be True

| A | B | A or B |
|---|---|--------|
| True | True | **True** |
| True | False | **True** |
| False | True | **True** |
| False | False | False |

### `not` - Flip the value

| A | not A |
|---|-------|
| True | False |
| False | **True** |

## Combining Multiple Conditions: Raid Requirements

You can chain logical operators together.

To start a raid, player needs (level >= 10 OR has party) AND has quest item.

```python
level = 5
has_party = True
has_quest_item = True

# Complex condition
can_raid = (level >= 10 or has_party) and has_quest_item
print(can_raid)  # What do you get?
```

You get `True`. They're under level 10 but have a party, and they have the quest item.

**Try removing the quest item**:

```python
has_quest_item = False
can_raid = (level >= 10 or has_party) and has_quest_item
print(can_raid)  # What now?
```

You get `False`. The first part is True (has party), but no quest item means the whole thing is False.

**Important**: Use parentheses to make your logic clear. Without them, Python follows precedence rules that might surprise you.

## Short-Circuit Evaluation: Python's Smart Optimization

Python is smart about evaluating logical operators:

```python
# With 'and', if first is False, Python doesn't check second
x = False
result = x and (10 / 0)  # No error! Python stops at False
print(result)  # False

# With 'or', if first is True, Python doesn't check second
y = True
result = y or (10 / 0)  # No error! Python stops at True
print(result)  # True
```

**Why this matters**: You can use this to avoid errors:

```python
# Safe division check
denominator = 0

# This is safe - Python won't divide if denominator is 0
if denominator != 0 and 10 / denominator > 2:
    print("Result is greater than 2")
else:
    print("Can't divide or result too small")
```

## Practice: Solve Real Problems

### Challenge 1: Boss Fight Entry

Check if player can enter the boss fight:

```python
is_champion = False
level = 15
has_key = True

# Champion enters automatically
# Others need level >= 10 AND key
can_fight_boss = is_champion or (level >= 10 and has_key)
print(f"Can fight boss: {can_fight_boss}")
```

**Experiment**: What if level is 15 but has_key is False?

### Challenge 2: Loot Drop Bonus

Player gets bonus loot if: is_VIP OR in_guild OR score > 1000.

```python
is_vip = False
in_guild = False
score = 1500

# Any of these conditions qualifies
gets_bonus = is_vip or in_guild or score > 1000
print(f"Gets bonus loot: {gets_bonus}")
```

**Change values**: What combination gives False?

### Challenge 3: Valid Character Name

Character name must be: not empty AND between 3-15 characters.

```python
name = "DragonSlayer"

# Check all conditions
is_valid = len(name) > 0 and len(name) >= 3 and len(name) <= 15
print(f"Valid name: {is_valid}")

# Cleaner way using chained comparison
is_valid = len(name) > 0 and 3 <= len(name) <= 15
print(f"Valid name: {is_valid}")
```

## Debugging With AI

When your logical conditions give unexpected results, here's how to get help:

**"My condition is always False"**

You want to check if player qualifies for special rate (under 18 or over 65):

```python
age = 25
special_rate = age < 18 and age > 65
print(special_rate)  # Always False!
```

Tell AI: *"I want players under 18 OR over 65 to get special rate, but it's always False."*

AI will explain: You used `and`, which requires BOTH conditions. No one can be under 18 AND over 65 at the same time! Use `or` instead.

**"The result isn't what I expected"**

You want to check: (can attack OR can defend) AND has energy:

```python
result = True or False and False
print(result)  # True (expected False!)
```

Tell AI: *"I want (True or False) and False, which should be False, but I'm getting True."*

AI will explain: Python evaluates `and` before `or`. Add parentheses to control the order: `(True or False) and False`.

**"This logic is confusing"**

Your variable names make the code hard to read:

```python
is_not_invalid = not is_invalid
```

Tell AI: *"This double negative is confusing. How can I make it clearer?"*

AI will suggest: Use positive names like `is_valid = not is_invalid`, or better yet, design with positive names from the start.

## Try With AI

**Explore short-circuit:**
> "Explain Python's short-circuit evaluation for `and` and `or`. When does Python skip evaluating the second operand? Show examples where this prevents errors."

**Solve a real problem:**
> "I'm building a game. Player can move if: not frozen AND (has stamina OR has energy potion). Write the logical expression and test it with different True/False combinations."

**Understand precedence:**
> "What's the precedence order for `not`, `and`, `or`? Show how `not True or False and True` is evaluated step by step."

