---
sidebar_position: 5
title: "Nested Control Structures"
description: "Combine conditionals and loops to solve complex multi-step problems"
duration_minutes: 60-75

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Nested If Statements"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement decision trees with multiple levels of nested conditionals to solve multi-criteria problems"

  - name: "Nested Loops"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create two-dimensional iteration patterns using nested loops for grid-based or matrix-like operations"

  - name: "Conditionals Inside Loops"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can filter or process items selectively while iterating through collections"

  - name: "Loops Inside Conditionals"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can conditionally execute entire iteration blocks based on runtime conditions"

  - name: "Loop Invariants & Complexity"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand-Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can recognize when nesting depth impacts code maintainability and identify complexity thresholds"

  - name: "Refactoring for Clarity"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify code that would benefit from simplification and articulate refactoring strategies"

learning_objectives:
  - objective: "Combine nested if statements to create multi-level decision trees"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Implementation of multi-criteria eligibility logic with correct indentation and logic flow"

  - objective: "Implement nested loops to perform two-dimensional iteration"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creation of multiplication table or grid-based pattern with accurate output"

  - objective: "Integrate conditionals within loops for selective processing"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Filtering operations that combine iteration and decision-making correctly"

  - objective: "Recognize when nesting increases vs. decreases code clarity"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Evaluation of code samples identifying readability issues and suggesting simplifications"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (nested if, nested loops, conditionals in loops, loops in conditionals, complexity awareness, refactoring) - all review/synthesis of Lessons 1-4 concepts, no new syntax introduced. Within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore three-level nesting scenarios; practice recognizing when nesting becomes too complex (readability and cognitive load awareness)"
  remedial_for_struggling: "Focus on two-level nesting only; use visual diagrams to trace execution flow; practice with simpler multiplication table (5×5 instead of 10×10)"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-part-4-chapter-17/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Nested Control Structures

You've learned how to make decisions with conditionals (Lessons 1-2) and automate repetition with loops (Lessons 3-4). Now it's time to combine these tools to solve more complex problems that require both decision-making **and** repetition working together.

Think of nested control structures like a set of Russian nesting dolls: each layer contains another layer inside. When you nest an `if` statement inside a loop, or a loop inside another loop, you create powerful patterns that can solve multi-step problems.

In this lesson, you'll integrate everything you've learned in Chapter 17 to handle scenarios like:
- Checking multiple criteria before making a decision (nested `if` statements)
- Processing two-dimensional data like grids or tables (nested loops)
- Filtering items while iterating through a list (conditionals inside loops)
- Processing entire collections only when certain conditions are met (loops inside conditionals)

By the end of this lesson, you'll understand how to combine control structures effectively—and when nesting becomes too complex and needs simplification.

---

## Quick Review: Building Blocks from Lessons 1-4

Before we combine these concepts, let's quickly recall what we've learned:

**From Lesson 1 (Conditionals)**:
- `if`, `elif`, `else` let you make decisions based on conditions
- You can chain multiple conditions with `and`, `or`, `not`
- Indentation defines which code belongs to which branch

**From Lesson 2 (Pattern Matching)**:
- `match-case` provides a cleaner way to handle multiple specific values
- The wildcard pattern `_` acts as a default case

**From Lesson 3 (Loops)**:
- `for` loops repeat code for each item in a sequence (definite iteration)
- `while` loops repeat code as long as a condition is true (indefinite iteration)
- `range()` generates number sequences for counting loops

**From Lesson 4 (Loop Control)**:
- `break` exits a loop early
- `continue` skips to the next iteration
- `for...else` and `while...else` detect whether loops completed normally

Now we'll combine these tools to solve more sophisticated problems.

---

## Nested If Statements: Decision Trees

Sometimes a single decision isn't enough—you need to make **multiple sequential decisions**. This creates a **decision tree** where each branch can have its own sub-branches.

### Example: Multi-Criteria Eligibility

Imagine you're building a system to determine if someone qualifies for a special program. They must meet multiple criteria:

1. Age must be between 18 and 65
2. If they're under 25, they need a recommendation letter
3. If they're over 60, they need a health clearance

Here's how nested `if` statements handle this:

```python
# Test case 1: Young applicant (22) with recommendation
age: int = 22
has_recommendation: bool = True
has_health_clearance: bool = False

if age >= 18 and age <= 65:
    # Age is valid, now check additional criteria
    if age < 25:
        # Young applicant - needs recommendation
        if has_recommendation:
            print("Eligible: Young applicant with recommendation")
        else:
            print("Not eligible: Young applicants need recommendation")
    elif age > 60:
        # Senior applicant - needs health clearance
        if has_health_clearance:
            print("Eligible: Senior applicant with health clearance")
        else:
            print("Not eligible: Senior applicants need health clearance")
    else:
        # Age 25-60, no additional requirements
        print("Eligible: Standard applicant")
else:
    print("Not eligible: Age must be between 18 and 65")
```

**Output**:
```
Eligible: Young applicant with recommendation
```

**More test cases you can try**:
```python
# Test case 2: Young without recommendation
age, has_recommendation, has_health_clearance = 22, False, False
# Output: Not eligible: Young applicants need recommendation

# Test case 3: Standard age (no special requirements)
age, has_recommendation, has_health_clearance = 45, False, False
# Output: Eligible: Standard applicant

# Test case 4: Senior with health clearance
age, has_recommendation, has_health_clearance = 62, False, True
# Output: Eligible: Senior applicant with health clearance

# Test case 5: Too old
age, has_recommendation, has_health_clearance = 70, False, True
# Output: Not eligible: Age must be between 18 and 65
```

#### 💬 AI Colearning Prompt
> "Explain how the indentation in this nested if statement creates the decision tree structure. What happens at each level?"

**Notice the structure**:
- **First level**: Check age range (18-65)
- **Second level**: Check age subranges (under 25, over 60, or in between)
- **Third level**: Check additional requirements (recommendation or health clearance)

Each level of indentation represents a deeper decision in the tree.

#### 🎓 Instructor Commentary
> In AI-native development, you don't memorize nesting patterns—you understand the decision logic you need, then ask your AI to structure it clearly. The key skill is **describing your multi-criteria logic** so AI can help you implement it correctly.

---

## Nested Loops: Two-Dimensional Iteration

When you need to process data arranged in rows and columns (like a grid, table, or matrix), you use **nested loops**—a loop inside another loop.

### Example: Multiplication Table

Let's create a multiplication table showing products from 1×1 through 10×10:

```python
# Generate a 10x10 multiplication table
size: int = 10
print(f"\nMultiplication Table (1 to {size}):\n")

# Outer loop: iterate through rows (first number)
for row in range(1, size + 1):
    # Inner loop: iterate through columns (second number)
    for col in range(1, size + 1):
        product: int = row * col
        # Format each number to take 4 spaces (right-aligned)
        print(f"{product:4}", end="")
    # After completing all columns in a row, move to next line
    print()  # Newline after each row

# You can try different table sizes by changing the size variable:
# size = 5  # For a 5x5 table
# size = 12  # For a 12x12 table
```

**Output (partial)**:
```
Multiplication Table (1 to 10):

   1   2   3   4   5   6   7   8   9  10
   2   4   6   8  10  12  14  16  18  20
   3   6   9  12  15  18  21  24  27  30
   4   8  12  16  20  24  28  32  36  40
   5  10  15  20  25  30  35  40  45  50
   6  12  18  24  30  36  42  48  54  60
   7  14  21  28  35  42  49  56  63  70
   8  16  24  32  40  48  56  64  72  80
   9  18  27  36  45  54  63  72  81  90
  10  20  30  40  50  60  70  80  90 100
```

**How it works**:
1. **Outer loop** (`row`): Runs 10 times (rows 1-10)
2. **Inner loop** (`col`): For EACH row, runs 10 times (columns 1-10)
3. **Total iterations**: 10 × 10 = 100 calculations

#### 💬 AI Colearning Prompt
> "Trace through the first 3 iterations of both loops when size=3. What are the values of `row`, `col`, and `product` at each step?"

**Key insight**: The inner loop **completes all its iterations** for each single iteration of the outer loop. Think of it like reading a book: for each page (outer loop), you read every line on that page (inner loop) before turning to the next page.

---

## Conditionals Inside Loops: Selective Processing

Often you want to iterate through items but only process some of them based on a condition. This combines the power of loops (repetition) with conditionals (decision-making).

### Example: Sum Only Positive Numbers

Let's sum only the positive numbers from a mixed list:

```python
# Calculate the sum of only positive numbers in a list
numbers: list[int] = [5, -3, 8, 0, -1, 12, -7, 4]
total: int = 0

# Iterate through all numbers
for num in numbers:
    # Conditional inside loop: only add if positive
    if num > 0:
        total += num
        print(f"Added {num}, running total: {total}")
    else:
        print(f"Skipped {num} (not positive)")

print(f"\nFinal sum of positive numbers: {total}")

# You can try different lists:
# numbers = [10, -5, 3, 0, 7, -2]  # Mix of positive, negative, and zero
# numbers = [1, 2, 3, 4, 5]  # All positive
```

**Output**:
```
Added 5, running total: 5
Skipped -3 (not positive)
Added 8, running total: 13
Skipped 0 (not positive)
Skipped -1 (not positive)
Added 12, running total: 25
Skipped -7 (not positive)
Added 4, running total: 29

Final sum of positive numbers: 29
```

**Pattern**: The loop visits **every** item, but the `if` statement controls **which** items get processed.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Generate code that finds all even numbers in a list and stores them in a new list. Use a for loop with an if statement to filter even numbers. Include type hints and explain the filtering logic. **Use inline code without functions.**"

**Expected Outcome**: You'll understand how to combine iteration with conditional filtering to build new collections.

---

## Loops Inside Conditionals: Conditional Iteration

Sometimes you want to run an entire loop only if a certain condition is met. This is the reverse pattern: a conditional **controls whether** the loop runs at all.

### Example: Process List Only If Valid

Let's validate a shopping cart before processing items:

```python
# Test case 1: Valid checkout - process shopping cart items
cart: list[str] = ["Laptop", "Mouse", "Keyboard"]
user_authenticated: bool = True
has_payment_method: bool = True

# Check prerequisites BEFORE attempting to process
if user_authenticated and has_payment_method:
    print("✓ User authenticated and payment method verified")
    print("Processing cart items:\n")

    # Loop runs only if condition above is True
    for item in cart:
        print(f"  - Processing: {item}")

    print("\n✓ All items processed successfully")
else:
    # Condition failed - loop never runs
    print("✗ Cannot process cart:")
    if not user_authenticated:
        print("  - User must log in")
    if not has_payment_method:
        print("  - Payment method required")

print("\n" + "="*50 + "\n")

# Test case 2: Missing authentication
user_authenticated = False
has_payment_method = True

if user_authenticated and has_payment_method:
    print("✓ User authenticated and payment method verified")
    print("Processing cart items:\n")

    for item in cart:
        print(f"  - Processing: {item}")

    print("\n✓ All items processed successfully")
else:
    print("✗ Cannot process cart:")
    if not user_authenticated:
        print("  - User must log in")
    if not has_payment_method:
        print("  - Payment method required")
```

**Output**:
```
✓ User authenticated and payment method verified
Processing cart items:

  - Processing: Laptop
  - Processing: Mouse
  - Processing: Keyboard

✓ All items processed successfully

==================================================

✗ Cannot process cart:
  - User must log in
```

**Why this matters**: In real applications, you often need to validate prerequisites before running expensive or sensitive operations. The loop only executes when it's safe and appropriate to do so.

#### 🎓 Instructor Commentary
> In AI-native development, recognizing WHEN to nest structures is more important than memorizing HOW. Ask yourself: "Do I need to decide BEFORE repeating, or decide DURING repetition?" That question determines your structure.

---

#### 🎓 Note: You Might See Functions in AI Responses

When you ask your AI to generate code examples, it might create code using `def` keyword to define **functions**. Functions are reusable blocks of code—a powerful Python feature you'll learn in **Chapter 20**.

For now, when working through this chapter:

- **Don't worry if you see functions**: Your AI is showing you professional Python patterns
- **You don't need to understand `def`, `return`, or function calls yet**: Focus on the control flow logic (loops, conditionals, nesting) inside the function body
- **Ask for inline code instead**: If you see a function, ask your AI: *"Can you show me this same logic using inline code without functions? Just use variables and direct statements."*

**Example prompt to request non-function code**:
> "Generate the code using inline statements with variables, not as a function. I haven't learned functions yet."

---

## AI Companion: Complex Nested Structures

Now let's tackle a more complex scenario that combines multiple levels of nesting. This is where AI becomes especially helpful—describing the logic clearly and letting AI handle the intricate syntax.

### Example: Simple Game Grid Logic

Imagine a simple grid-based game where you need to:
1. Iterate through a 5×5 grid
2. For each cell, determine if it contains a special item
3. Apply different rules based on the item type

**Specification for AI**:

#### 🚀 CoLearning Challenge

Tell your AI Co-Teacher:
> "Create Python code that processes a 5×5 game grid. Each cell contains either 'empty', 'coin', or 'trap'. The code should:
> - Use nested loops to visit every cell
> - If the cell is 'coin', add 10 points
> - If the cell is 'trap', subtract 5 points
> - Track the total score and print what's found at each position
> Include type hints and detailed comments explaining the nesting structure. **Use inline code without functions.**"

**Your AI might generate something like**:

```python
# Sample game grid (5x5)
# Each cell is 'empty', 'coin', or 'trap'
game_grid: list[list[str]] = [
    ["empty", "coin", "empty", "trap", "coin"],
    ["coin", "empty", "empty", "coin", "empty"],
    ["trap", "empty", "coin", "empty", "trap"],
    ["empty", "coin", "empty", "empty", "coin"],
    ["coin", "trap", "empty", "coin", "empty"]
]

score: int = 0

# Outer loop: iterate through rows (y-coordinate)
for row_index in range(len(game_grid)):
    row: list[str] = game_grid[row_index]

    # Inner loop: iterate through columns (x-coordinate)
    for col_index in range(len(row)):
        cell: str = row[col_index]

        # Nested conditionals: determine what's in this cell
        if cell == "coin":
            score += 10
            print(f"Row {row_index}, Col {col_index}: Found coin! (+10) Score: {score}")
        elif cell == "trap":
            score -= 5
            print(f"Row {row_index}, Col {col_index}: Hit trap! (-5) Score: {score}")
        # If empty, no action needed (could add else for verbosity)

print(f"\n🎮 Game Over! Final Score: {score}")
```

**Output (partial)**:
```
Row 0, Col 1: Found coin! (+10) Score: 10
Row 0, Col 3: Hit trap! (-5) Score: 5
Row 0, Col 4: Found coin! (+10) Score: 15
Row 1, Col 0: Found coin! (+10) Score: 25
Row 1, Col 3: Found coin! (+10) Score: 35
...
🎮 Game Over! Final Score: 65
```

**Notice the nesting levels**:
1. **Outer loop**: Rows (position 0-4)
2. **Inner loop**: Columns (position 0-4)
3. **Conditionals**: Determine item type and action

This creates a total of 25 iterations (5 rows × 5 columns), with a decision made at each cell.

---

## Managing Complexity: When to Simplify

As you nest structures deeper, code becomes harder to read and maintain. Here's when to consider refactoring:

### Red Flag #1: Deep Indentation (3+ Levels)

```python
# ⚠️ Too deeply nested - hard to follow
for item in items:
    if item.is_valid:
        if item.price > 100:
            if item.in_stock:
                if item.category == "electronics":
                    # Four levels deep! Hard to maintain
                    process_item(item)
```

**Better approach**: Combine conditions using `and` to flatten the structure:

```python
# ✅ Flatter structure - easier to read
for item in items:
    # Combine conditions with 'and'
    if item.is_valid and item.price > 100 and item.in_stock and item.category == "electronics":
        process_item(item)
```

### Red Flag #2: Nested Loops with High Iteration Counts

```python
# ⚠️ This runs 1,000,000 times (1000 × 1000)
for i in range(1000):
    for j in range(1000):
        # Expensive operation
        do_something(i, j)
```

If you have nested loops with large ranges, consider:
- Do you really need to visit every combination?
- Can you filter before looping?
- Is there a more efficient algorithm?

#### ✨ Teaching Tip
> Use your AI to evaluate complexity: "Is this nested structure too complex? How could I simplify it while keeping the same logic?" Your AI can suggest refactoring patterns and explain the tradeoffs.

**When nesting gets too deep**: If your code is indented more than 3-4 levels, consider:
- Combining conditions with `and` / `or`
- Using early `continue` or `break` to reduce nesting
- Asking your AI: "How can I flatten this nested structure while keeping the same logic?"

For now, recognize when your nesting is getting too deep—that's your signal to simplify.

---

## Red Flags to Watch

### 1. Indentation Errors

**Problem**: Python relies on indentation to understand nesting. Mix tabs and spaces, and you'll get `IndentationError`.

```python
# ❌ Inconsistent indentation
for i in range(5):
    if i % 2 == 0:
      print(i)  # Two spaces
        print("even")  # Four spaces - ERROR!
```

**Solution**: Configure your editor to use 4 spaces per indentation level (Python standard). Most modern editors handle this automatically.

#### ✨ Teaching Tip
> If you get `IndentationError`, paste your code into your AI and ask: "Why am I getting an Indentation error? Show me where the indentation is inconsistent."

### 2. Loop Variable Confusion in Nested Loops

**Problem**: Using the same variable name for inner and outer loops:

```python
# ❌ Variable name collision
for i in range(3):
    for i in range(3):  # Reusing 'i' - overwrites outer loop variable!
        print(i, i)
```

**Solution**: Use descriptive, unique names:

```python
# ✅ Clear, distinct names
for row in range(3):
    for col in range(3):
        print(row, col)
```

### 3. Logic Errors from Incorrect Nesting

**Problem**: Putting code at the wrong indentation level changes when it runs:

```python
# ❌ print() is inside inner loop - prints too many times
for row in range(3):
    for col in range(3):
        calculate(row, col)
        print(f"Finished row {row}")  # Prints 3 times per row!
```

```python
# ✅ print() after inner loop - prints once per row
for row in range(3):
    for col in range(3):
        calculate(row, col)
    print(f"Finished row {row}")  # Correct placement
```

**Debugging tip**: If output appears more (or fewer) times than expected, check indentation levels carefully.

### 4. Infinite Nested Loops

**Problem**: Forgetting to update loop variables in nested `while` loops:

```python
# ❌ Infinite loop - col never increments
row: int = 0
while row < 3:
    col: int = 0
    while col < 3:
        print(row, col)
        # Missing: col += 1
    row += 1
```

**Solution**: Ensure **every** loop has proper termination logic:

```python
# ✅ Both loops terminate correctly
row: int = 0
while row < 3:
    col: int = 0
    while col < 3:
        print(row, col)
        col += 1  # Inner loop advances
    row += 1  # Outer loop advances
```

### 5. Deep Nesting Reducing Readability

**Problem**: More than 3 levels of nesting makes code hard to understand:

```python
# ⚠️ Four levels - very difficult to follow
for item in items:
    if item.active:
        for tag in item.tags:
            if tag.important:
                # Deep inside - what context are we in?
                process(item, tag)
```

**Solution**: Look for ways to flatten:
- Combine conditions with `and`/`or`
- Use `continue` to skip early
- Break complex logic into smaller, sequential steps

#### 💬 AI Colearning Prompt
> "Show me three ways to flatten this nested structure. Which approach is most readable and why?"

---

## Try With AI

Now let's synthesize everything you've learned across all five lessons. Use your AI companion (Claude Code, Gemini CLI, or ChatGPT web) to deepen your understanding of nested control structures.

**Tool**: Use your preferred AI companion tool (Claude Code, Gemini CLI) if you've set one up. If not, use ChatGPT web.

### 1. Recall (Foundational Understanding)

Copy this prompt to your AI:

```
Why is indentation critical in nested control structures in Python?
What happens if I mix indentation levels incorrectly?
```

**Expected outcome**: Your AI will explain how Python uses indentation to determine nesting levels, and show examples of `IndentationError`. Understanding this prevents one of the most common nesting mistakes.

### 2. Understand (Trace Execution)

Copy this prompt to your AI:

```
Trace through this nested loop step-by-step for the first 6 iterations.
For each iteration, tell me the values of `row`, `col`, and `product`:

for row in range(1, 3):
    for col in range(1, 4):
        product = row * col
        print(f"{row} × {col} = {product}")

What's the pattern? How many total iterations will occur?
```

**Expected outcome**: Your AI will trace the execution showing how the inner loop completes fully for each iteration of the outer loop. This reinforces the execution model for nested loops.

### 3. Apply (Generate Code)

Copy this prompt to your AI:

```
Generate Python code with type hints that uses nested loops to find
all pairs (i, j) where i + j = 10. Both i and j should range from 1 to 9.

Print each pair in the format: "i + j = 10" (e.g., "3 + 7 = 10")

Include:
- Type hints on all variables
- Comments showing the nesting structure
- Use inline code without functions (I haven't learned functions yet)
```

**Expected outcome**: You'll receive code that combines nested loops with a conditional inside to filter pairs. This practices the "conditionals inside loops" pattern.

### 4. Synthesize (Real-World Application)

Copy this prompt to your AI:

```
Describe a real-world problem that requires nested control flow
(combining loops and conditionals). Then ask the AI to implement it.

Example scenarios:
- Validating a seating chart (rows × seats, checking availability)
- Processing a weekly schedule (days × time slots, filtering conflicts)
- Analyzing a tic-tac-toe board (3×3 grid, checking win conditions)

Choose your own scenario, describe it clearly, then ask AI to implement it
with type hints and comments. After receiving the code, trace through one
complete execution path manually.
```

**Expected outcome**: You'll practice translating real-world multi-step problems into nested control structures. Tracing execution builds debugging skills and deepens understanding of how nesting works.

---

**Safety & Ethics Note**: Always review AI-generated nested code carefully. Verify:
- Indentation is consistent (4 spaces per level)
- Loop variables have unique, descriptive names
- Nesting depth is reasonable (3 levels or fewer)
- Logic matches your intended problem statement

**What you've mastered**: You can now combine all control flow concepts from this chapter—conditionals, pattern matching, loops, and loop control—to solve complex, multi-dimensional problems. As you continue learning Python, you'll discover how to organize and reuse this logic even more effectively.
