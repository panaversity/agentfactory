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

You're building a login system. The user enters a password, and you need to check: does it match the stored password? You're not calculating anything—you're asking a yes/no question. Is this equal to that?

## Why You Need Comparison Operators

Comparison operators ask questions and return answers: `True` or `False`. Without these operators, you couldn't:

- Check if a password matches
- Verify if a user is old enough
- Test if a value is within a range
- Make any decisions in your code

Every comparison in Python gives you one of two values: `True` or `False`. Let's discover how to ask these questions.

## Discovery: How Do You Ask Questions in Python?

**Problem**: You're checking if a user is old enough to vote. Voting age is 18, user is 16. How do you ask "Is 16 greater than or equal to 18?"

```python
user_age = 16
voting_age = 18

# Your prediction: What will this return?
can_vote = user_age >= voting_age
print(can_vote)
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

## Discovery: The Six Comparison Operators

### Equality and Inequality

**Problem**: You're verifying a PIN code. User entered 1234, correct PIN is 1234. Do they match?

```python
entered_pin = 1234
correct_pin = 1234

# Check if they match
is_correct = entered_pin == correct_pin
print(is_correct)  # What do you get?
```

You get `True`. Now check the opposite:

```python
# Check if they're different
is_wrong = entered_pin != correct_pin
print(is_wrong)  # What do you get?
```

You get `False`. They're not different—they match.

**Pattern discovered**: `==` returns True if values match, `!=` returns True if values differ.

### Greater Than and Less Than

**Problem**: You're checking temperature thresholds. Current temp is 75°F. Is it above the 70°F comfort threshold?

```python
current_temp = 75
comfort_threshold = 70

# Is it warmer than the threshold?
is_warm = current_temp > comfort_threshold
print(is_warm)  # What do you get?
```

You get `True`. Now check if it's below freezing:

```python
freezing = 32

is_freezing = current_temp < freezing
print(is_freezing)  # What do you get?
```

You get `False`. 75 is not less than 32.

### Greater/Less Than OR Equal

**Problem**: You're checking if someone qualifies for a discount. Must be 65 or older.

```python
customer_age = 65
discount_age = 65

# Does 65 qualify? (must be 65 OR older)
qualifies = customer_age >= discount_age
print(qualifies)  # What do you get?
```

You get `True`. The `>=` includes the equal case.

**Try it**: What if customer_age is 64? What about 66?

```python
# Experiment with different ages
print(64 >= 65)  # ?
print(65 >= 65)  # ?
print(66 >= 65)  # ?
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

## Discovery: Comparisons Always Return Booleans

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

## Discovery: Value Equality Surprises

**Problem**: Is the integer 5 equal to the float 5.0?

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

### Challenge 1: Age Verification

You're building an age gate for a website. Check multiple age thresholds:

```python
user_age = 17

# Voting (18+)
can_vote = user_age >= 18
print(f"Can vote: {can_vote}")

# Driving (16+)
can_drive = user_age >= 16
print(f"Can drive: {can_drive}")

# Senior discount (65+)
is_senior = user_age >= 65
print(f"Senior discount: {is_senior}")
```

**Change the age**: Try 18, 65, and 15. What changes?

### Challenge 2: Password Checker

You're validating a password. But first, you need to know how long it is.

**The `len()` function**: Python has a built-in function called `len()` that counts characters in a string (or items in a list). It returns a number.

```python
# len() counts characters
password = "secret123"
length = len(password)
print(length)  # 9
```

Now use it with comparisons:

```python
password = "secret123"
min_length = 8
stored_password = "secret123"

# Is it long enough?
long_enough = len(password) >= min_length
print(f"Long enough: {long_enough}")

# Does it match?
matches = password == stored_password
print(f"Matches stored: {matches}")

# Is it too short?
too_short = len(password) < min_length
print(f"Too short: {too_short}")
```

**Try it**: What if password is "abc"? What if it's "wrongpassword"?

### Challenge 3: Score Checker

You're grading a test. Check various thresholds:

```python
score = 85
passing = 70
perfect = 100

# Did they pass?
passed = score >= passing
print(f"Passed: {passed}")

# Perfect score?
is_perfect = score == perfect
print(f"Perfect: {is_perfect}")

# Failed?
failed = score < passing
print(f"Failed: {failed}")
```

## Common Mistakes

**Mistake 1: Using `=` instead of `==`**

```python
# Wrong: This assigns, doesn't compare
x = 5
# if x = 5:  # SyntaxError!

# Right: Use == to compare
if x == 5:
    print("x is 5")
```

**Mistake 2: Confusing `>` and `>=`**

```python
age = 18
limit = 18

# Wrong: Excludes exactly 18
print(age > limit)   # False

# Right: Includes 18
print(age >= limit)  # True
```

**Mistake 3: Expecting string-number equality**

```python
# Wrong assumption: "5" equals 5
print("5" == 5)  # False - string is not a number

# Right: Convert first if needed
print(int("5") == 5)  # True
```

## Try With AI

**Explore edge cases:**
> "What happens when I compare True to 1 and False to 0? Are booleans secretly numbers in Python?"

**Solve a real problem:**
> "I'm building a thermostat. I need to check if temperature is below 60 (turn on heat), above 75 (turn on AC), or between (do nothing). What comparisons do I need?"

**Understand the difference:**
> "Explain why `=` and `==` exist as separate operators. Show me what goes wrong if I accidentally use the wrong one."

## What You Discovered

- Python has 6 comparison operators: `==`, `!=`, `>`, `<`, `>=`, `<=`
- All comparisons return `True` or `False` (boolean values)
- `=` assigns, `==` compares (common mistake!)
- `>=` and `<=` include the equal case
- Python compares values, not types (5 == 5.0 is True)

**Next**: You'll discover logical operators—how to combine multiple True/False values with `and`, `or`, and `not`.
