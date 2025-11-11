---
title: "List Comprehensions — Transforming Data"
chapter: 18
lesson: 5
duration_minutes: 210

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "List Comprehension Syntax"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes `[expression for item in iterable]` with correct semantics and proper type hints"

  - name: "Filtering with Comprehensions"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student adds `if condition` to filter elements in comprehension independently"

  - name: "Loop-to-Comprehension Translation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student analyzes for-loops and converts to equivalent comprehensions with understanding of transformation"

  - name: "Readability Judgment"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student evaluates when comprehension enhances vs harms code clarity and makes reasoned choices"

learning_objectives:
  - objective: "Write list comprehensions for simple transformations using correct `[expr for item in iterable]` syntax"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Exercises transforming data with working comprehensions"

  - objective: "Add filtering conditions with `if` inside comprehensions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Exercises filtering data with comprehensions"

  - objective: "Convert traditional for-loops to equivalent list comprehensions"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Refactoring loop code into comprehensions"

  - objective: "Evaluate readability tradeoffs between comprehensions and traditional loops"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Code review and refactoring decisions in \"Try With AI\""

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (comprehension syntax, expression evaluation, iteration variable, if filtering, readability, nested comprehensions brief intro) within B1 limit of 10. Focused and scaffolded progression ✓"

differentiation:
  extension_for_advanced: "Students reaching toward B2 explore nested comprehensions with `for` loops inside main comprehension; analyze performance implications of complex comprehensions; practice with multi-condition filtering"
  remedial_for_struggling: "Struggling students practice step-by-step loop-to-comprehension transformation with explicit syntax breakdown; use simplified examples (doubling numbers, filtering even numbers) before moving to string filtering"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 5: List Comprehensions — Transforming Data

You've mastered creating, modifying, and sorting lists. Now comes one of Python's most elegant features: **list comprehensions**. They transform lists in a single line that reads almost like English: "a list of X for each Y in Z, if condition."

In professional Python code, comprehensions appear everywhere. They're concise, readable (when written well), and often faster than traditional loops. But here's the key insight: **clarity trumps cleverness**. A 5-line loop that's easy to understand beats a 1-line comprehension that makes your teammates pause.

Let's learn how to write them, when to use them, and—critically—when a regular loop is the better choice.

## From Loops to Comprehensions: The Transformation

You already know how to transform data with loops:

```python
# Traditional approach: start with empty list, append items
numbers: list[int] = [1, 2, 3, 4, 5]

squares_loop: list[int] = []
for x in numbers:
    squares_loop.append(x ** 2)

print(squares_loop)  # [1, 4, 9, 16, 25]
```

This is clear and explicit: create an empty list, iterate over numbers, append each squared value. But Python offers a shorter way to express this exact idea:

```python
# Comprehension approach: transform in one line
numbers: list[int] = [1, 2, 3, 4, 5]

squares_comp: list[int] = [x ** 2 for x in numbers]

print(squares_comp)  # [1, 4, 9, 16, 25]
```

Both produce identical results. The second is a **list comprehension**—a syntactic shortcut for creating a new list by transforming an existing one.

#### 💬 AI Colearning Prompt

> "Explain the mental transformation from a for-loop with `append()` to a list comprehension. What parts of the loop map to what parts of the comprehension syntax?"

This helps you see the structural connection so comprehensions don't feel magical.

## Understanding Comprehension Syntax

A list comprehension has **three essential parts**:

```
[expression   for   item   in   iterable]
  ^1           ^2   ^3     ^4   ^5
```

1. **Expression** — What gets added to the new list (e.g., `x ** 2`)
2. **`for` keyword** — Signals iteration starts
3. **Iteration variable** — Holds one item at a time (e.g., `x`)
4. **`in` keyword** — Connects to the source
5. **Iterable** — The source to iterate over (e.g., `numbers`)

Let's trace through an example:

```python
numbers: list[int] = [1, 2, 3, 4, 5]
squares: list[int] = [x ** 2 for x in numbers]
```

Execution:
- **Iteration 1**: `x = 1` → expression `1 ** 2 = 1` → add to result
- **Iteration 2**: `x = 2` → expression `2 ** 2 = 4` → add to result
- **Iteration 3**: `x = 3` → expression `3 ** 2 = 9` → add to result
- And so on...

**Result**: `[1, 4, 9, 16, 25]`

Notice: the expression runs for each item in the iterable, and the result of each expression becomes an element in the new list.

#### 🎓 Instructor Commentary

> In AI-native development, you don't memorize the syntax—you understand the intent: "I want to transform each item from a source list." The syntax is just Python's way of expressing that idea concisely. When syntax feels confusing, ask AI: "Break down this comprehension step-by-step."

### Exercise 1: Writing Simple Comprehensions

Write a comprehension for each transformation. Check your type hints.

1. **Double each number**: `[2, 4, 6, 8, 10]` from `[1, 2, 3, 4, 5]`
2. **Extract first letters**: `['a', 'b', 'c']` from `['apple', 'banana', 'cherry']`
3. **Convert to uppercase**: `['HELLO', 'WORLD']` from `['hello', 'world']`

**Your turn** — try these before looking at the solutions:

```python
# 1. Double numbers
numbers: list[int] = [1, 2, 3, 4, 5]
doubled: list[int] = [n * 2 for n in numbers]

# 2. Extract first letters
fruits: list[str] = ["apple", "banana", "cherry"]
first_letters: list[str] = [fruit[0] for fruit in fruits]

# 3. Convert to uppercase
words: list[str] = ["hello", "world"]
uppercase: list[str] = [word.upper() for word in words]
```

## Adding Conditions: Filtering with `if`

Now for a powerful variation: **filtering**. Add an `if` condition to include only items that match:

```
[expression   for   item   in   iterable   if   condition]
```

Example — keep only even numbers:

```python
numbers: list[int] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

evens: list[int] = [n for n in numbers if n % 2 == 0]

print(evens)  # [2, 4, 6, 8, 10]
```

The `if` condition acts as a **gate**: items that pass the condition are included; others are filtered out.

**Execution**:
- `n = 1`: `1 % 2 == 0`? No → skip
- `n = 2`: `2 % 2 == 0`? Yes → include `2`
- `n = 3`: `3 % 2 == 0`? No → skip
- `n = 4`: `4 % 2 == 0`? Yes → include `4`
- And so on...

**Result**: `[2, 4, 6, 8, 10]`

**Key insight**: The condition evaluates **before** the expression is added. If the condition is false, that item is skipped entirely.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "I have a list of words. Write a comprehension that keeps only words that start with a vowel. Then explain the condition you used and how it filters the list."

**Expected Outcome**: You understand how to write string conditions in comprehensions and see how the `if` gate works conceptually.

### Exercise 2: Filtering with Comprehensions

1. **Keep numbers greater than 5**: From `[2, 5, 7, 1, 9, 3]`, get `[7, 9]`
2. **Keep strings longer than 3 characters**: From `['it', 'hello', 'ok', 'python']`, get `['hello', 'python']`
3. **Keep only positive numbers**: From `[-5, 3, -1, 8, -2, 0, 6]`, get `[3, 8, 6]`

```python
# 1. Numbers > 5
numbers: list[int] = [2, 5, 7, 1, 9, 3]
above_five: list[int] = [n for n in numbers if n > 5]

# 2. Words longer than 3 characters
words: list[str] = ["it", "hello", "ok", "python"]
long_words: list[str] = [w for w in words if len(w) > 3]

# 3. Positive numbers
numbers: list[int] = [-5, 3, -1, 8, -2, 0, 6]
positive: list[int] = [n for n in numbers if n > 0]
```

#### ✨ Teaching Tip

> When your comprehension gets hard to read (many characters, complex condition), use Claude Code to split it into a traditional loop. Ask: "Make this comprehension into a for-loop that's easier to understand." Your AI will refactor it, and you can choose whichever version is clearer for your team.

## Combining Expression and Filtering

You can transform **and** filter in one comprehension:

```python
numbers: list[int] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# Keep only even numbers, then double them
evens_doubled: list[int] = [n * 2 for n in numbers if n % 2 == 0]

print(evens_doubled)  # [4, 8, 12, 16, 20]
```

Order matters: the `if` condition **filters first**, then the expression is applied to remaining items.

**Execution**:
- Filter: keep `[2, 4, 6, 8, 10]`
- Transform: apply `* 2` → `[4, 8, 12, 16, 20]`

This is very Pythonic and concise. But remember: **clarity is king**. If this makes someone pause during code review, consider splitting it:

```python
# More explicit version (easier to read and debug)
evens: list[int] = [n for n in numbers if n % 2 == 0]
evens_doubled: list[int] = [n * 2 for n in evens]
```

Both are correct. The second is just more readable and easier to test independently.

### Exercise 3: Transform and Filter

1. **Square only odd numbers**: From `[1, 2, 3, 4, 5]`, get `[1, 9, 25]`
2. **Uppercase only words longer than 4 characters**: From `['hi', 'hello', 'hey', 'python']`, get `['HELLO', 'PYTHON']`

```python
# 1. Square odd numbers
numbers: list[int] = [1, 2, 3, 4, 5]
odd_squares: list[int] = [n ** 2 for n in numbers if n % 2 != 0]

# 2. Uppercase long words
words: list[str] = ["hi", "hello", "hey", "python"]
long_upper: list[str] = [w.upper() for w in words if len(w) > 4]
```

## A Word on Readability: Clarity Trumps Cleverness

Comprehensions are beautiful when they're simple. But Python developers sometimes get carried away:

```python
# Nested comprehension with complex logic — AVOID THIS
data: list[int] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
result: list[int] = [[y * 2 for y in range(x)] for x in data if x % 2 == 0]
```

This is syntactically correct but difficult to understand at a glance. It combines:
- Nested loops (a comprehension inside another comprehension)
- Multiple conditions
- Complex expressions

**Professional approach**: When you can't read your comprehension in one breath, refactor to a loop.

```python
# More readable version
result: list[list[int]] = []
for x in data:
    if x % 2 == 0:
        row: list[int] = [y * 2 for y in range(x)]
        result.append(row)
```

This is longer but tells the story clearly: "For each number in data, if it's even, create a list of doubled values from 0 to that number."

**Rule of thumb**:
- **Simple comprehension** (single line, easy to read): Use it
- **Complex comprehension** (nested loops, multiple conditions): Use a traditional loop instead
- **When in doubt**: Ask your AI and your team. A 4-line loop beats a 1-line mystery.

#### 💬 AI Colearning Prompt

> "Is this comprehension readable? `[str(x*2) for x in range(10) if x % 3 == 0 and x > 2]` If not, rewrite it as a for-loop and explain why the loop is clearer."

## Brief Introduction to Nested Comprehensions

For completeness: you can put a comprehension inside another comprehension. But we're not diving deep here—focus on simple, readable comprehensions first.

```python
# Nested comprehension: create a matrix (list of lists)
matrix: list[list[int]] = [[i * j for j in range(1, 4)] for i in range(1, 4)]

# Equivalent (more readable) traditional loop
matrix: list[list[int]] = []
for i in range(1, 4):
    row: list[int] = []
    for j in range(1, 4):
        row.append(i * j)
    matrix.append(row)
```

Both create:
```
[[1, 2, 3],
 [2, 4, 6],
 [3, 6, 9]]
```

The traditional loop is much clearer. Use it.

**Key lesson**: Nested comprehensions exist, but they're often harder to debug and maintain. Master simple comprehensions first. Save nested ones for cases where the intent is crystal clear (rare).

---

## Try With AI

Use **Claude Code** (or your preferred AI companion) to reinforce comprehension mastery. Work through each prompt sequentially.

### Prompt 1 (Remember): Syntax Recall
> "Write the basic syntax for a list comprehension. What three parts must every comprehension have? Show examples."

**Expected Outcome**: You recall the three-part structure (`[expression for item in iterable]`) and understand each part's role. AI might mention optional `if` conditions as a fourth element.

---

### Prompt 2 (Understand): Tracing Execution
> "Trace through this comprehension step-by-step: `[x * 3 for x in [1, 2, 3, 4] if x > 2]`. Show which numbers are included and what the final list is."

**Expected Outcome**: You understand that `if` filtering happens first, then the expression is applied. You can trace execution mentally and predict the result without running code.

---

### Prompt 3 (Apply): Building Your Own
> "I have a list of prices: `[10.50, 25.00, 5.99, 100.00, 15.00]`. Write a comprehension that keeps only prices less than $50 and adds a 10% tax (multiply by 1.10). What's the final list?"

**Expected Outcome**: You write a working comprehension combining filtering and transformation, correctly apply the type hints (`list[float]`), and verify the output makes sense.

---

### Prompt 4 (Analyze): Readability Judgment
> "Compare these two approaches to the same problem. Which is more readable for a team code review? Why? When would you choose the comprehension vs the loop?
>
> **Option A** (Comprehension):
> ```python
> result: list[int] = [n**2 for n in range(1, 11) if n % 2 == 0]
> ```
>
> **Option B** (Loop):
> ```python
> result: list[int] = []
> for n in range(1, 11):
>     if n % 2 == 0:
>         result.append(n ** 2)
> ```"

**Expected Outcome**: You evaluate both styles objectively, recognizing that simple comprehensions are readable while complex ones aren't. You understand the tradeoff between conciseness and clarity, and you can make context-aware decisions about which to use.

---

**Safety Note**: List comprehensions are safe Python constructs. Always verify the output matches your intent—use `print()` to inspect results, and ask AI to explain unexpected behavior. When filtering or transforming data, check a few examples manually before deploying comprehensions in real applications.

