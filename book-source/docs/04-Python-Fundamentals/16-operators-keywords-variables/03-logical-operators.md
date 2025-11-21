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

You're building a login system. The user must enter the correct password AND be at least 18 years old. One condition isn't enough—you need both to be true. How do you combine multiple conditions?

## Why You Need Logical Operators

Logical operators combine True/False values to make complex decisions. Without these operators, you couldn't:

- Check multiple requirements at once (password correct AND age verified)
- Offer alternatives (pay with credit OR debit)
- Invert conditions (user is NOT banned)
- Express real-world business logic

Instead of asking one question, you can ask: "Is this true AND that true?" or "Is this true OR that true?" Let's discover how.

## Discovery: How Do You Combine Conditions?

**Problem**: You're checking if someone can rent a car. They must be at least 21 AND have a valid license. How do you check both?

```python
age = 25
has_license = True

# Your prediction: What will this return?
can_rent = age >= 21 and has_license
print(can_rent)
```

**Run it.** You get `True`. The `and` operator returns True only when BOTH conditions are True.

Now change the values:

```python
age = 19
has_license = True

can_rent = age >= 21 and has_license
print(can_rent)  # What now?
```

You get `False`. Even though they have a license, they're not 21 yet. Both must be true.

## Discovery: The Three Logical Operators

### The `and` Operator: Both Must Be True

**Problem**: You're approving a loan. Applicant needs income >= $50,000 AND credit score >= 700.

```python
income = 60000
credit_score = 720

# Both conditions must be true
approved = income >= 50000 and credit_score >= 700
print(approved)  # What do you get?
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

### The `or` Operator: At Least One Must Be True

**Problem**: You're checking payment options. Customer can pay with credit card OR debit card.

```python
has_credit = False
has_debit = True

# At least one must be true
can_pay = has_credit or has_debit
print(can_pay)  # What do you get?
```

You get `True`. They have debit, so they can pay.

**Experiment**: What happens with different combinations?

```python
# Test different combinations
print(True or True)    # ?
print(True or False)   # ?
print(False or True)   # ?
print(False or False)  # ?
```

**Pattern discovered**: `or` returns True when AT LEAST ONE side is True. Only False when both are False.

### The `not` Operator: Flip the Value

**Problem**: You're checking if a user is NOT banned.

```python
is_banned = False

# Flip the value
can_access = not is_banned
print(can_access)  # What do you get?
```

You get `True`. The user is not banned, so they can access.

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

## Discovery: Combining Multiple Conditions

You can chain logical operators together:

**Problem**: You're checking movie theater eligibility. Must be 13+ OR accompanied by adult, AND must have a ticket.

```python
age = 10
has_adult = True
has_ticket = True

# Complex condition
can_enter = (age >= 13 or has_adult) and has_ticket
print(can_enter)  # What do you get?
```

You get `True`. They're under 13 but have an adult, and they have a ticket.

**Try removing the ticket**:

```python
has_ticket = False
can_enter = (age >= 13 or has_adult) and has_ticket
print(can_enter)  # What now?
```

You get `False`. The first part is True (has adult), but no ticket means the whole thing is False.

**Important**: Use parentheses to make your logic clear. Without them, Python follows precedence rules that might surprise you.

## Discovery: Short-Circuit Evaluation

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

### Challenge 1: Access Control

You're building a security system. Check multiple access levels:

```python
is_admin = False
is_employee = True
has_badge = True

# Admin gets in automatically
# Employees need a badge
can_enter = is_admin or (is_employee and has_badge)
print(f"Can enter: {can_enter}")
```

**Experiment**: What if is_employee is True but has_badge is False?

### Challenge 2: Discount Eligibility

You're calculating discounts. Customer gets discount if: senior (65+) OR student OR spending over $100.

```python
age = 30
is_student = False
purchase = 150

# Any of these conditions qualifies
gets_discount = age >= 65 or is_student or purchase > 100
print(f"Gets discount: {gets_discount}")
```

**Change values**: What combination gives False?

### Challenge 3: Form Validation

You're validating a registration form. Username must be: not empty AND between 3-20 characters.

```python
username = "alex"

# Check all conditions
is_valid = len(username) > 0 and len(username) >= 3 and len(username) <= 20
print(f"Valid username: {is_valid}")

# Cleaner way using chained comparison
is_valid = len(username) > 0 and 3 <= len(username) <= 20
print(f"Valid username: {is_valid}")
```

## Common Mistakes

**Mistake 1: Forgetting parentheses with mixed operators**

```python
# Confusing: What does this mean?
result = True or False and False
print(result)  # True (and has higher precedence)

# Clear: Use parentheses
result = (True or False) and False
print(result)  # False
```

**Mistake 2: Using `and` when you mean `or`**

```python
# Wrong: This is always False
age = 25
in_range = age < 18 and age > 65  # Can't be both!

# Right: Use or for either/or
out_of_range = age < 18 or age > 65
```

**Mistake 3: Double negatives**

```python
# Confusing
is_not_invalid = not is_invalid

# Clearer
is_valid = not is_invalid
# Or even better, use a positive variable name from the start
```

## Try With AI

**Explore short-circuit:**
> "Explain Python's short-circuit evaluation for `and` and `or`. When does Python skip evaluating the second operand? Show examples where this prevents errors."

**Solve a real problem:**
> "I'm building a game. Player can move if: not frozen AND (has stamina OR has energy potion). Write the logical expression and test it with different True/False combinations."

**Understand precedence:**
> "What's the precedence order for `not`, `and`, `or`? Show how `not True or False and True` is evaluated step by step."

## What You Discovered

- Python has 3 logical operators: `and`, `or`, `not`
- `and` requires BOTH to be True
- `or` requires AT LEAST ONE to be True
- `not` flips True to False and vice versa
- Use parentheses to make complex conditions clear
- Python uses short-circuit evaluation (stops early when result is known)

**Next**: You'll discover assignment operators—shortcuts for updating variables like `+=` and `-=`.
