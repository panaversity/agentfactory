---
title: "Capstone: Calculator Program"
sidebar_position: 5
---

# Capstone: Calculator Program

## What You'll Build

- Complete Calculator Program
- All Four Operator Types Combined
- Input Validation
- Running Total Tracker

## The Problem

You've learned arithmetic, comparison, logical, and assignment operators separately. But real programs use them all together. How do you combine these operator types into one working application?

## The Challenge

Build a calculator program that demonstrates all four operator types working together.

### Requirements

Your calculator must:

1. **Get two numbers from the user** using `input()`
2. **Perform arithmetic operations**: addition, subtraction, multiplication, division
3. **Compare the numbers**: equal, greater than, less than
4. **Check conditions with logical operators**: both positive, either large
5. **Track a running total** using assignment operators
6. **Handle division by zero** safely

### Expected Output

When the user enters `10` and `3`, your program should display something like:

```
========================================
CALCULATOR PROGRAM
========================================

Enter first number: 10
Enter second number: 3

--- ARITHMETIC OPERATIONS ---
10.0 + 3.0 = 13.0
10.0 - 3.0 = 7.0
10.0 * 3.0 = 30.0
10.0 / 3.0 = 3.3333333333333335

--- COMPARISONS ---
10.0 == 3.0: False
10.0 > 3.0: True
10.0 < 3.0: False

--- LOGICAL CHECKS ---
Both numbers positive: True
Either number > 100: False

--- RUNNING TOTAL ---
Starting total: 0
After adding sum: 13.0
After subtracting 5: 8.0
After doubling: 16.0

========================================
Calculation complete!
========================================
```

### Pseudocode

Here's the structure to guide you:

```
# Display header

# Get two numbers from user (convert to float)

# ARITHMETIC OPERATIONS
# Calculate: sum, difference, product, quotient
# Print each result

# COMPARISONS
# Check: equal, greater, less
# Print each result (True/False)

# LOGICAL CHECKS
# Check: both positive (num1 > 0 AND num2 > 0)
# Check: either large (num1 > 100 OR num2 > 100)
# Print each result

# RUNNING TOTAL
# Start at 0
# Add the sum result (+=)
# Subtract 5 (-=)
# Double it (*=)
# Print after each operation

# SAFE DIVISION
# Check if num2 is not zero before dividing
# Print result or error message

# Display footer
```

## Hints

<details>
<summary>Hint 1: Getting numbers from user</summary>

Use `input()` and convert to float:
```python
num1 = float(input("Enter first number: "))
```
</details>

<details>
<summary>Hint 2: Arithmetic operations</summary>

Store results in variables:
```python
sum_result = num1 + num2
print(f"{num1} + {num2} = {sum_result}")
```
</details>

<details>
<summary>Hint 3: Comparison operations</summary>

Comparisons return True or False:
```python
is_equal = num1 == num2
print(f"{num1} == {num2}: {is_equal}")
```
</details>

<details>
<summary>Hint 4: Logical operations</summary>

Combine conditions with `and` and `or`:
```python
both_positive = (num1 > 0) and (num2 > 0)
```
</details>

<details>
<summary>Hint 5: Running total with assignment operators</summary>

Update a variable step by step:
```python
total = 0
total += sum_result
total -= 5
total *= 2
```
</details>

<details>
<summary>Hint 6: Safe division</summary>

Check before dividing:
```python
if num2 != 0:
    result = num1 / num2
    print(f"Result: {result}")
else:
    print("Cannot divide by zero")
```
</details>

## Testing Your Calculator

Test with these input combinations:

| Test Case | num1 | num2 | What to Check |
|-----------|------|------|---------------|
| Normal | 10 | 3 | All operations work |
| Equal numbers | 5 | 5 | Equality is True |
| Zero divisor | 10 | 0 | Division handled safely |
| Negative | -5 | 3 | "Both positive" is False |
| Large numbers | 150 | 200 | "Either > 100" is True |

## Common Errors

**Error 1: Forgetting to convert input**
```python
# Wrong: input() returns string
num1 = input("Number: ")
result = num1 + 5  # TypeError!

# Right: Convert to float
num1 = float(input("Number: "))
result = num1 + 5  # Works!
```

**Error 2: Using = instead of ==**
```python
# Wrong: Assignment, not comparison
is_equal = num1 = num2  # Assigns num2 to num1!

# Right: Use == for comparison
is_equal = num1 == num2
```

**Error 3: Missing parentheses in logical expressions**
```python
# Confusing: What gets evaluated first?
result = num1 > 0 and num2 > 0 or num1 == num2

# Clear: Use parentheses
result = (num1 > 0 and num2 > 0) or (num1 == num2)
```

**Error 4: Dividing without checking for zero**
```python
# Dangerous: Crashes if num2 is 0
result = num1 / num2

# Safe: Check first
if num2 != 0:
    result = num1 / num2
```

## Extensions

Once your basic calculator works, try these enhancements:

### Extension 1: Add More Operations
Add floor division, modulus, and exponentiation:
```python
# Floor division
floor_result = num1 // num2

# Modulus (remainder)
mod_result = num1 % num2

# Exponentiation
exp_result = num1 ** num2
```

### Extension 2: Add Input Validation
Check that inputs are valid numbers before calculating.

### Extension 3: Add Type Display
Show the type of each result:
```python
print(f"Result: {result} (type: {type(result).__name__})")
```

### Extension 4: Menu System
Let user choose which operation to perform (requires concepts from Chapter 18).

## Reflection Questions

After completing your calculator, consider:

1. **Which operator type did you use most?** Why?
2. **Where did logical operators help?** What would happen without the zero check?
3. **How did assignment operators simplify the running total?** Compare `total = total + value` vs `total += value`.
4. **What surprised you?** Any unexpected results or behaviors?

## Success Criteria

Your calculator is complete when:

- [ ] Gets two numbers from user input
- [ ] Performs all four arithmetic operations (+, -, *, /)
- [ ] Shows three comparisons (==, >, <)
- [ ] Uses `and` and `or` for logical checks
- [ ] Tracks running total with +=, -=, *=
- [ ] Handles division by zero without crashing
- [ ] Displays clear, formatted output

## Try With AI

After building your calculator, use AI to enhance it:

**Debug your code:**
> "Here's my calculator code: [paste code]. It's not working correctly when I enter [specific input]. Help me find the bug."

**Add features:**
> "I built a basic calculator with arithmetic, comparison, logical, and assignment operators. Help me add floor division and modulus operations."

**Improve output:**
> "My calculator works but the output is hard to read. Show me how to format it with aligned columns and clear section headers."

## What You've Accomplished

By completing this capstone, you've demonstrated:

- **Arithmetic operators**: Calculations with +, -, *, /
- **Comparison operators**: True/False questions with ==, >, <
- **Logical operators**: Combined conditions with and, or
- **Assignment operators**: Efficient updates with +=, -=, *=
- **Safe programming**: Checking before dividing by zero

You now have a solid foundation in Python operators—the building blocks for everything that comes next.

**Next Chapter**: You'll learn about strings and type casting, where you'll see that some operators work with text too!
