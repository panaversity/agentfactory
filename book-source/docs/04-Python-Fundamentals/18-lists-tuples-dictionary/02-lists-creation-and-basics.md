---
title: "Lists Part 1 — Creation and Basic Operations"
chapter: 18
lesson: 2
duration_minutes: 210

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "List Creation with Type Hints"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can write numbers: list[int] = [1, 2, 3] and explain intent behind type hints"

  - name: "Index-Based Access"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can retrieve list elements using positive indices (0, 1, 2) and negative indices (-1, -2)"

  - name: "Slice Syntax Understanding"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use list[start:stop:step] syntax to extract subsequences and predict outputs"

  - name: "List Length Querying"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use len() function and understand what it measures (count of elements)"

learning_objectives:
  - objective: "LO-002a: Create typed lists using literals and list() constructor with explicit type hints"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Code writing exercises creating lists with correct type annotations"

  - objective: "LO-002b: Access list elements by index (positive and negative) and understand index ordering"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Indexing expressions that retrieve specific elements"

  - objective: "LO-002c: Use slicing syntax to extract subsequences and understand slice parameters"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Slice notation exercises with various start, stop, and step values"

  - objective: "LO-002d: Explain the difference between copying and aliasing in list assignments"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Written explanation and code prediction about list assignment behavior"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (list literals, type hints, positive indexing, negative indexing, slicing, homogeneous vs heterogeneous) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore negative indexing patterns; create slices with step values; explain aliasing memory model with diagrams"
  remedial_for_struggling: "Focus on positive indexing first (0-based); defer negative indexing to practice; use concrete list examples (fruit, colors)"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lists Part 1 — Creation and Basic Operations

## Getting Started: Your First Lists

You already understand that lists are ordered, changeable sequences. Now let's learn how to actually create them and work with their elements. By the end of this lesson, you'll be able to create type-hinted lists and extract exactly the data you need using indexing and slicing.

---

## Concept 1: Creating Lists — Literals and Constructors

There are two main ways to create a list in Python: **list literals** (using square brackets) and the **list() constructor** (converting data into a list).

### List Literals: The Direct Way

The simplest way is to write a list literal with items inside square brackets:

```python
# A list of integers
numbers: list[int] = [1, 2, 3, 4, 5]

# A list of strings
names: list[str] = ["Alice", "Bob", "Charlie"]

# An empty list (we'll add items later)
empty: list[int] = []

# A list with mixed types (NOT RECOMMENDED but possible)
mixed: list = [1, "hello", 3.14, True]  # Avoid this in practice
```

Notice the **type hints** like `list[int]` and `list[str]`. These tell you (and AI tools like type checkers) exactly what kind of items belong in the list. This is crucial for **AI-native development**—clear type hints communicate intent to both humans and tools.

#### 💬 AI Colearning Prompt

> "What happens if I try to add a string to a list[int]? Does Python stop me immediately or only when I use a type checker?"

This exploration helps you understand the difference between **runtime behavior** (what Python actually does when you run code) and **static analysis** (what type checkers like mypy flag as potential bugs). Both matter in professional code.

#### 🎓 Expert Insight

> In AI-native development, type hints are documentation that enable tools. When you write `numbers: list[int]`, you're saying "this list holds integers." Python won't stop you from adding a string at runtime, but a type checker will warn you, and your AI partner can catch errors before they cause problems.

### List Constructor: Converting to a List

The `list()` constructor converts other collections into lists. Most commonly, you'll convert strings or ranges:

```python
# Convert a string into a list of characters
letters: list[str] = list("hello")
# Result: ["h", "e", "l", "l", "o"]

# Convert a range into a list of numbers
digits: list[int] = list(range(10))
# Result: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# Convert an empty list (does nothing interesting but shows the pattern)
fresh_list: list[int] = list()  # Same as []
```

---

## Concept 2: Type Hints Express Intent

Type hints tell a story about your data. Compare these:

```python
shopping_items: list[str] = ["milk", "eggs", "bread"]
# "This is a list of strings—each item is text"

scores: list[int] = [95, 87, 92, 88, 90]
# "This is a list of integers—each item is a number"

temperatures: list[float] = [98.6, 99.1, 97.8]
# "This is a list of floats—each item is a decimal number"
```

**Why this matters**: Type hints act as executable documentation. When someone (or an AI tool) reads `list[str]`, they immediately know what to expect.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Create a list of student names with a type hint. Then create another list of their GPAs. Now show me what type hint would represent 'a list of dictionaries where each dictionary has name and GPA.' Don't implement it—just show the type hint and explain it."

**Expected Outcome**: You'll understand how type hints scale from simple lists to complex nested structures. This prepares you for real-world data.

---

## Concept 3: Index-Based Access — Getting Items

Every item in a list has a **position** called an **index**. Python uses **zero-based indexing**, meaning the first item is at index 0, the second is at index 1, and so on.

### Positive Indexing: Counting from the Start

```python
fruits: list[str] = ["apple", "banana", "cherry", "date"]

# Index positions:
#   0: "apple"
#   1: "banana"
#   2: "cherry"
#   3: "date"

print(fruits[0])  # "apple"
print(fruits[1])  # "banana"
print(fruits[2])  # "cherry"
print(fruits[3])  # "date"
```

### Negative Indexing: Counting from the End

Python also supports **negative indices**, which count backward from the end:

```python
fruits: list[str] = ["apple", "banana", "cherry", "date"]

# Negative index positions:
#   -4: "apple"
#   -3: "banana"
#   -2: "cherry"
#   -1: "date"

print(fruits[-1])  # "date" (last item)
print(fruits[-2])  # "cherry" (second from last)
print(fruits[-4])  # "apple" (fourth from last, same as first)
```

Negative indexing is incredibly useful when you want to access items near the end without knowing the list's length.

#### ✨ Teaching Tip

> Use Claude Code to explore index errors: "What happens if I try fruits[10] on a 4-item list? Show me the error and explain it." Understanding IndexError is crucial for writing defensive code.

#### 💬 AI Colearning Prompt

> "Why does Python use zero-based indexing instead of one-based (where the first item is 1)? What are the advantages and disadvantages of each approach?"

This deeper question helps you appreciate Python's design decisions—useful when you need to explain code to teammates.

---

## Concept 4: Slicing — Getting Multiple Items

**Slicing** extracts a range of items from a list. The syntax is `list[start:stop:step]`:

- **start**: Index where the slice begins (inclusive, default 0)
- **stop**: Index where the slice ends (exclusive, default len(list))
- **step**: How many positions to jump (default 1)

### Basic Slicing (No Step)

```python
numbers: list[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# Get items 2 through 5 (not including 5)
print(numbers[2:5])  # [2, 3, 4]

# Get items 0 through 3
print(numbers[0:3])  # [0, 1, 2]

# Get items 5 through the end
print(numbers[5:])   # [5, 6, 7, 8, 9]

# Get items from start through 6
print(numbers[:6])   # [0, 1, 2, 3, 4, 5]

# Get the entire list
print(numbers[:])    # [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
```

### Slicing with Negative Indices

```python
numbers: list[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# Last 3 items
print(numbers[-3:])  # [7, 8, 9]

# Everything except the last 2
print(numbers[:-2])  # [0, 1, 2, 3, 4, 5, 6, 7]

# Middle 4 items (from index 3 to 7)
print(numbers[3:-2]) # [3, 4, 5, 6, 7]
```

### Slicing with Step (Every Nth Item)

```python
numbers: list[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# Every other item
print(numbers[::2])   # [0, 2, 4, 6, 8]

# Every item starting from index 1, step by 2
print(numbers[1::2])  # [1, 3, 5, 7, 9]

# Reverse the list (start at end, go backward, step -1)
print(numbers[::-1])  # [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
```

The `[::-1]` pattern is a Pythonic way to reverse any sequence!

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "I have a list of 10 numbers. Show me 5 different slice notations to get the middle 5 elements. Explain why each one works."

**Expected Outcome**: You'll understand that Python offers multiple ways to express the same operation. Choose the clearest one for your context.

#### 🎓 Expert Insight

> You don't memorize slice syntax. You understand: "I need items 2 through 5, so I write [2:5]." If you forget whether stop is inclusive or exclusive, ask your AI partner. The understanding—why you're slicing—matters far more than remembering the edge cases.

---

## Concept 5: The `len()` Function — How Many Items?

The **len()** function returns the number of items in a list:

```python
fruits: list[str] = ["apple", "banana", "cherry", "date"]

print(len(fruits))  # 4

# Using len() in expressions
numbers: list[int] = [10, 20, 30, 40, 50]
print(f"The list has {len(numbers)} items")

# Last item using len()
last_number: int = numbers[len(numbers) - 1]  # 50
# But negative indexing is cleaner: numbers[-1]
```

You'll use `len()` constantly when you need to know how many items you're working with.

---

## Concept 6: Aliasing vs Copying — A Critical Distinction

Here's a subtle but important concept: **what happens when you assign one list to another variable?**

### Aliasing: Two Names, Same List

```python
list1: list[int] = [1, 2, 3]
list2 = list1  # list2 points to THE SAME list

list2.append(4)  # Modify through list2
print(list1)     # [1, 2, 3, 4] — list1 ALSO changed!
```

**Why?** Both `list1` and `list2` are **aliases**—they point to the same list object in memory. When you modify through one, the other reflects the change.

### Copying: Two Independent Lists

If you want a separate copy, use the `.copy()` method:

```python
list1: list[int] = [1, 2, 3]
list2 = list1.copy()  # Create an independent copy

list2.append(4)  # Modify the copy
print(list1)     # [1, 2, 3] — list1 is UNCHANGED
print(list2)     # [1, 2, 3, 4]
```

This is a frequent source of bugs! You think you're modifying a copy, but you're actually modifying the original.

#### 💬 AI Colearning Prompt

> "Explain the difference between `list2 = list1` and `list2 = list1.copy()` using a memory diagram or analogy. Why would this bug be hard to catch?"

#### ✨ Teaching Tip

> Use Claude Code to explore aliasing: "Show me what happens when I modify list2 after `list2 = list1`. Draw a memory diagram explaining why the original list changed." This visual understanding is worth more than memorizing the rule.

---

## Practice: Apply What You've Learned

### Exercise 1: Create Lists with Type Hints

Create lists for the following scenarios and write them with appropriate type hints:

1. A list of your 5 favorite colors
2. A list of numbers from 1 to 10
3. An empty list that will eventually hold temperatures (as floats)

### Exercise 2: Indexing Practice

Given this list:

```python
animals: list[str] = ["cat", "dog", "elephant", "fish", "giraffe", "horse"]
```

Write expressions to retrieve:

1. The first animal
2. The last animal
3. The third animal
4. The second-to-last animal

### Exercise 3: Slicing Predictions

Given this list:

```python
numbers: list[int] = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
```

Predict what each slice returns (don't run code—reason through it):

1. `numbers[2:7]`
2. `numbers[-4:]`
3. `numbers[::3]`
4. `numbers[1:8:2]`
5. `numbers[::-1]`

### Exercise 4: Aliasing vs Copying

Predict what will be printed:

```python
original: list[int] = [1, 2, 3]
alias = original
copy = original.copy()

alias.append(4)
copy.append(5)

print("original:", original)  # What do you expect?
print("alias:", alias)        # What do you expect?
print("copy:", copy)          # What do you expect?
```

---

## Summary: What You Now Know

You can now create lists with type hints and extract exactly the data you need using indexing and slicing. You understand the critical difference between aliasing (pointing to the same list) and copying (creating a separate list). In the next lesson, you'll learn how to modify lists by adding and removing items.

---

## Try With AI

**Tool**: Claude Code, Gemini CLI, or your preferred AI companion (or ChatGPT web if you haven't set up a CLI tool yet)

Use your AI tool to deepen your understanding:

### Prompt 1 (Remember): Index and Slice Review

> "For the list ['apple', 'banana', 'cherry', 'date', 'elderberry'], what does each expression return?
> - `[0]`
> - `[-1]`
> - `[1:3]`
> - `[:2]`
> - `[::2]`
>
> Explain why each one works the way it does."

**Expected Outcome**: You'll confirm your understanding of indexing and slicing patterns and catch any misconceptions.

### Prompt 2 (Understand): Negative Indexing Deep Dive

> "I have a list of 8 items. Explain why `list[-3]` and `list[5]` refer to the same item (assuming the list has 8 items). Draw a numbered index diagram showing both positive and negative indices."

**Expected Outcome**: You'll understand the mapping between positive and negative indices, making you confident with backward counting.

### Prompt 3 (Apply): Slice for Real Problems

> "I have a list of 20 students. Write slice notation to:
> - Get the first 5 students
> - Get the last 3 students
> - Get every other student starting from index 0
> - Reverse the entire list
>
> For each slice, explain what you're trying to achieve."

**Expected Outcome**: You'll recognize that slicing solves real problems and learn when to reach for `[start:stop:step]`.

### Prompt 4 (Analyze): The Aliasing Trap

> "Explain why this code is problematic:
> ```python
> original = [1, 2, 3]
> backup = original
> original.append(4)
> # Now 'backup' is [1, 2, 3, 4]—why?
> ```
> Show how to fix it. Why would this bug be hard to catch in a large program?"

**Expected Outcome**: You'll internalize the aliasing concept and always remember to use `.copy()` when you need an independent list.

---

**Safety & Ethics**: When working with real data, always verify that modifications are intentional. Aliasing bugs can cause data corruption if you accidentally modify a "backup" that you thought was independent. Ask your AI partner to review code where lists are assigned to catch this pattern.
