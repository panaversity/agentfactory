---
title: "Capstone: Game Score System"
sidebar_position: 5
---

# Capstone: Game Score System

## What You'll Build

- Complete Game Score System
- All Four Operator Types Combined
- Win/Lose Condition Checking
- Score Tracking and Bonuses

## The Challenge

You've learned arithmetic, comparison, logical, and assignment operators separately—all using game examples. Now combine them into one working game score system.

Build a game score system that demonstrates all four operator types working together.

### Requirements

Your game system must:

1. **Get player stats from user** using `input()` (score and health)
2. **Calculate bonuses** using arithmetic: coin bonus, combo multiplier
3. **Check win/lose conditions** using comparisons: score threshold, health check
4. **Validate game state** with logical operators: can continue playing
5. **Update running totals** using assignment operators
6. **Handle edge cases** safely (e.g., division for team splits)

### Expected Output

When the user enters score `85` and health `50`, your program should display something like:

```
========================================
GAME SCORE SYSTEM
========================================

Enter your score: 85
Enter your health: 50

--- SCORE CALCULATIONS ---
Base score: 85
Coin bonus (10 coins × 5): 50
Total with bonus: 135
With 2x combo multiplier: 270

--- WIN/LOSE CONDITIONS ---
Score >= 100 to win: True
Health > 0 (alive): True
Beat high score (200): True

--- GAME STATE ---
Can continue (score >= 50 AND health > 0): True
Gets bonus loot (score > 100 OR health == 100): True

--- RUNNING TOTALS ---
Starting total: 0
After adding score: 270
After enemy hit (-20): 250
After power-up (×1.5): 375.0

========================================
Game complete!
========================================
```

## Hints

<details>
<summary>Hint 1: Getting player stats from user</summary>

Use `input()` and convert to int:
```python
score = int(input("Enter your score: "))
health = int(input("Enter your health: "))
```
</details>

<details>
<summary>Hint 2: Score calculations (arithmetic)</summary>

Store results in variables:
```python
coin_bonus = 10 * 5  # 10 coins worth 5 each
total = score + coin_bonus
final_score = total * 2  # combo multiplier
```
</details>

<details>
<summary>Hint 3: Win/lose conditions (comparisons)</summary>

Comparisons return True or False:
```python
won = score >= 100
alive = health > 0
print(f"Won: {won}")
```
</details>

<details>
<summary>Hint 4: Game state (logical)</summary>

Combine conditions with `and` and `or`:
```python
can_continue = (score >= 50) and (health > 0)
gets_bonus = (score > 100) or (health == 100)
```
</details>

<details>
<summary>Hint 5: Running totals (assignment)</summary>

Update a variable step by step:
```python
total = 0
total += final_score
total -= 20  # enemy damage
total *= 1.5  # power-up
```
</details>

<details>
<summary>Hint 6: Safe team split</summary>

For now, assume team_size is never 0:
```python
team_size = 4
per_player = total / team_size
print(f"Each player gets: {per_player}")
```

**Note**: To handle the case where team_size might be 0, you'll need `if/else` statements—which you'll learn in Chapter 18. For this capstone, use a non-zero team size.
</details>

## Testing Your Game System

Test with these input combinations:

| Test Case | Score | Health | What to Check |
|-----------|-------|--------|---------------|
| Normal win | 85 | 50 | All conditions and calculations work |
| Low score | 30 | 100 | Can't continue (score < 50) |
| Zero health | 100 | 0 | Game over (health check fails) |
| Perfect game | 200 | 100 | Gets bonus loot (health == 100) |
| Edge case | 50 | 1 | Minimum to continue playing |

## Success Criteria

Your game system is complete when:

- [ ] Gets score and health from user input
- [ ] Calculates bonuses using arithmetic (+, *, /)
- [ ] Checks win/lose conditions with comparisons (>=, >, ==)
- [ ] Uses `and` and `or` for game state logic
- [ ] Tracks running total with +=, -=, *=
- [ ] Handles team split safely (checks for zero)
- [ ] Displays clear, formatted output with sections
