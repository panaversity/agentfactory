---
title: "Set Basics"
chapter: 19
lesson: 1
sidebar_position: 1
description: "Learn Python set fundamentals: creation, uniqueness, hashability, and basic operations with type hints"
keywords: [python sets, set basics, hashability, type hints, duplicate elimination, unordered collections]
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Creating Sets with Type Hints"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can write set[int] and set[str] literals and use set() constructor with proper type hints"

  - name: "Understanding Set Uniqueness Property"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why sets automatically eliminate duplicates and when this property is useful"

  - name: "Modifying Sets with add/remove/discard"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use .add(), .remove(), and .discard() appropriately with understanding of error handling differences"

  - name: "Understanding Mutability vs Hashability"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why sets require immutable elements and what happens when trying to add a mutable type"

learning_objectives:
  - objective: "Create sets using literal syntax and constructor with proper type hints"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Code exercise creating sets with {1, 2, 3} and set() syntax"

  - objective: "Explain the uniqueness property and demonstrate duplicate elimination"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Hands-on example showing automatic duplicate removal"

  - objective: "Distinguish between hashable and unhashable types"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Error handling exercise attempting unhashable types in sets"

cognitive_load:
  new_concepts: 7
  assessment: "Concepts: 1) Sets, 2) Uniqueness, 3) Unordered nature, 4) Type hints, 5) Mutability requirement, 6) Hashability, 7) add/remove/discard. Total: 7 = at A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore set constructors with generators; discuss memory efficiency of sets vs lists"
  remedial_for_struggling: "Start with concrete set[int] examples before introducing mixed types; practice add() before remove()"

# Generation metadata
generated_by: "lesson-writer v1.0.0"
source_spec: "specs/001-part-4-chapter-19/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 1: Set Basics

Sets are Python's answer to a common problem: **How do you store a collection of unique items efficiently?** When you need to automatically eliminate duplicates and check if something exists quickly, sets are the perfect tool.

In this lesson, you'll learn what makes sets special, how to create them with modern Python type hints, why they require immutable elements, and how to add and remove items. By the end, you'll understand when to choose sets over lists—and more importantly, why that choice matters.

---

## What Makes Sets Different?

Before we dive into syntax, let's understand what sets actually do. A **set** is an unordered collection of unique, hashable elements. That's three important properties:

**Uniqueness**: Sets automatically eliminate duplicates. If you try to add "apple" twice, it stays in the set only once.

**Unordered**: Unlike lists, sets don't maintain insertion order. When you iterate over a set, elements might appear in any order (though the order is consistent within a Python session). This might seem like a limitation, but it's the price sets pay for their speed.

**Hash-Based Storage**: Sets use Python's hashing mechanism internally, which enables O(1) lookup time—meaning checking if an item exists is incredibly fast, even in massive sets.

#### 💬 AI Colearning Prompt

> "Why would I use a set instead of a list for checking if a value exists in a large collection?"

This question gets at the heart of why sets exist. Your AI can walk you through the performance difference and explain when the choice matters practically.

#### 🎓 Instructor Commentary

> In AI-native development, you don't memorize when to use which collection type. You understand the properties—uniqueness, speed of lookup, unordered nature—and choose based on what your code needs. Your AI assistant can explain trade-offs instantly.

---

## Creating Sets

There are two main ways to create a set: using **literal syntax** with curly braces, or using the **constructor**.

### Set Literal Syntax

The most straightforward way to create a set is with curly braces and values:

```python
# Create a set with integer literals
numbers: set[int] = {1, 2, 3, 4, 5}

# Create a set with string literals
colors: set[str] = {"red", "green", "blue"}

# Create a set with mixed immutable types (requires Python 3.10+ union syntax)
mixed: set[int | str] = {1, "two", 3, "four"}
```

Notice the type hints: `set[int]` means "a set containing integers," and `set[str]` means "a set containing strings." Modern Python (3.9+) lets you use this bracket syntax instead of older `Set[int]` from the `typing` module.

### Set Constructor

You can also create sets from other collections using the `set()` constructor:

```python
# Create a set from a list (duplicates automatically removed)
from_list: set[int] = set([1, 2, 3, 3, 2, 1])  # Result: {1, 2, 3}

# Create an empty set (must use set(), not {})
empty_set: set[int] = set()
```

**Important:** The empty set must be created with `set()`, not `{}`. The brackets `{}` alone create an empty dictionary, not an empty set. This is a common gotcha!

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Create a set from a list of numbers that has many duplicates. Show me how the set automatically removes the duplicates. Then explain why Python needed a separate `set()` constructor for empty sets instead of just using `{}`."

**Expected Outcome:** You'll see the deduplication in action and understand the historical reason for the `set()` syntax quirk.

#### 💬 AI Colearning Prompt
> "Why does Python use curly braces `{}` for sets when they look like dict syntax? How does Python distinguish between an empty set and an empty dict?"

---

## The Uniqueness Property

Sets automatically eliminate duplicates. This is their superpower. Let's see this in action:

### Code Example 1: Creating Sets with Type Hints

```python
# Set literal syntax with type hints
numbers: set[int] = {1, 2, 3, 4, 5}
colors: set[str] = {"red", "green", "blue"}

# Set constructor syntax
from_list: set[int] = set([1, 2, 3, 3, 2, 1])
empty_set: set[int] = set()

# Display results
print(f"numbers: {numbers}")              # {1, 2, 3, 4, 5}
print(f"colors: {colors}")                # {'red', 'green', 'blue'}
print(f"from_list: {from_list}")          # {1, 2, 3} - duplicates removed!
print(f"type(empty_set): {type(empty_set)}")  # <class 'set'>
```

**Key observation:** Notice that `from_list` shows `{1, 2, 3}`, not `{1, 2, 3, 3, 2, 1}`. The duplicates are gone automatically.

### Code Example 2: Understanding Uniqueness

```python
# Real-world scenario: Deduplicating email addresses
email_list: list[str] = [
    "alice@example.com",
    "bob@example.com",
    "alice@example.com",  # Duplicate
    "charlie@example.com",
    "bob@example.com"     # Duplicate
]

unique_emails: set[str] = set(email_list)

print(f"Original list length: {len(email_list)}")      # 5
print(f"Unique emails count: {len(unique_emails)}")    # 3
print(f"Unique emails: {unique_emails}")               # {'alice@...', 'bob@...', 'charlie@...'}

# Practical use: Track unique user IDs from a log
user_ids: set[int] = {100, 101, 102, 101, 100}  # Some users visit multiple times
print(f"Unique users: {user_ids}")                     # {100, 101, 102}
```

This pattern is incredibly common: you have a collection with potential duplicates, and you need to know the unique values. Sets solve this instantly.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize when to use sets vs. lists—you recognize the pattern. When you see "unique values" or "eliminate duplicates," that's your cue. Ask your AI: "Should I use a set here?" and it'll explain why uniqueness matters for your specific problem.

---

## The Hashability Requirement

Here's where sets get philosophical. Sets can only contain **hashable** elements—values that are immutable and won't change. Why? Because sets use Python's hashing mechanism to store and look up items efficiently.

A **hash value** is a special integer that Python computes from an object. For a set to work, an object's hash value must never change. If an object is mutable (can be modified), its hash might change, breaking the set's internal structure.

**Hashable types** (safe in sets):
- Integers: `{1, 2, 3}`
- Strings: `{"a", "b", "c"}`
- Tuples: `{(1, 2), (3, 4)}`
- Frozensets: `{frozenset([1, 2]), frozenset([3, 4])}`

**Unhashable types** (will cause errors):
- Lists: `{[1, 2, 3]}` ❌ Lists are mutable
- Dictionaries: `{{"a": 1}}` ❌ Dicts are mutable
- Sets: `{{1, 2}, {3, 4}}` ❌ Sets themselves are mutable

### Code Example 3: Modifying Sets

```python
# Create a set and add elements
visited_cities: set[str] = {"Paris", "Tokyo", "London"}

# Add new element
visited_cities.add("New York")
print(f"After adding: {visited_cities}")  # {'Paris', 'Tokyo', 'London', 'New York'}

# Adding duplicate does nothing (no error, no change)
visited_cities.add("Paris")
print(f"After re-adding: {visited_cities}")  # No change

# Remove element (raises error if not found)
visited_cities.remove("Paris")
print(f"After removing: {visited_cities}")   # {'Tokyo', 'London', 'New York'}

# Discard element (no error if not found)
visited_cities.discard("Berlin")  # No error, Berlin wasn't there
visited_cities.discard("Tokyo")
print(f"After discarding: {visited_cities}")  # {'London', 'New York'}
```

Notice the difference: `.remove()` raises an error if the element doesn't exist, while `.discard()` silently does nothing. Use `.remove()` when you're certain the element exists; use `.discard()` when you're not sure.

#### 💬 AI Colearning Prompt

> "When would I use `.remove()` instead of `.discard()` in real code? Show me a scenario where the difference matters."

Your AI can help you think through when you want an error to alert you versus when you want silent tolerance.

### Code Example 4: The Hashability Requirement

```python
# ✅ Hashable types work in sets
numbers: set[int] = {1, 2, 3}
strings: set[str] = {"a", "b", "c"}
tuples: set[tuple[int, int]] = {(1, 2), (3, 4)}
frozen: set[frozenset[int]] = {frozenset([1, 2]), frozenset([3, 4])}

print("Hashable types work fine!")

# ❌ Unhashable types fail
try:
    my_set: set[list[int]] = {[1, 2], [3, 4]}
except TypeError as e:
    print(f"Error with lists: {e}")  # unhashable type: 'list'

try:
    my_dict_set: set[dict[str, int]] = {{"a": 1}}
except TypeError as e:
    print(f"Error with dicts: {e}")  # unhashable type: 'dict'

# Why? Lists and dicts are mutable - they can change after being added to a set!
# If you could add [1, 2] to a set, then modify it to [1, 3], the set's lookup breaks.
```

The error message "unhashable type" is Python's way of saying "I can't guarantee this value won't change, so I won't let you put it in a set."

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a set containing both integers and tuples. Then try to add a list to that set and explain why it fails. What could I use instead of a list to make it work?"

**Expected Outcome**: You'll understand immutability requirements through hands-on experimentation and learn that tuples (immutable) work where lists (mutable) don't.

---

## Practice: Build Your Understanding

Try these exercises to reinforce what you've learned:

### Exercise 1: Create Sets with Type Hints

Create three different sets:
1. A set of integers from 1 to 5 (using literal syntax)
2. A set of your favorite colors as strings
3. An empty set with type hint `set[str]`

Verify each one by printing it and checking its type.

### Exercise 2: Deduplication Challenge

Create a list with many duplicate names:
```python
names: list[str] = ["Alice", "Bob", "Alice", "Charlie", "Bob", "Diana", "Alice"]
```

Convert it to a set and compare the lengths. How many duplicates were there?

### Exercise 3: Error Handling with Unhashable Types

Try adding different types to a set (integers, strings, tuples, then lists):

```python
test_set: set[int] = set()
test_set.add(42)  # This works
test_set.add((1, 2))  # Will this work?
test_set.add([1, 2])  # Will this work?
```

For each one, predict whether it will work. Then run it and observe the error (if any). Write a brief explanation of why each one succeeded or failed.

### Exercise 4: add() vs remove() vs discard()

Create a set with three colors. Then:
1. Add a new color
2. Remove an existing color (using `.remove()`)
3. Try to discard a color that doesn't exist (using `.discard()`)
4. Try to remove a color that doesn't exist and observe the error

Why did step 4 fail while step 3 succeeded?

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Create a real-world scenario where you'd want to deduplicate a list using a set. Show me the code with type hints. Then explain: if order matters, would a set still work? Why or why not?"

**Expected Outcome:** You'll see a practical application and understand the trade-off between uniqueness and order.

---

## Try With AI

### Setup

Use your preferred AI companion (ChatGPT web, Claude Code, or Gemini CLI if you've set one up from previous lessons).

### Prompt Set (Bloom's Progression)

**Prompt 1 - Concept Exploration (Understand Level):**

> "I'm learning about Python sets. Give me 3 real-world examples where you'd want a collection that automatically eliminates duplicates. For each example, explain why a set is better than a list."

**Expected Outcome:** You see practical business scenarios where sets solve actual problems. The AI helps you connect abstract uniqueness to concrete use cases.

---

**Prompt 2 - Syntax Clarification (Apply Level):**

> "Explain the difference between these three: `my_set = {1, 2, 3}` vs. `empty = {}` vs. `my_list = [1, 2, 3]`. What is each one, and why can't I use `{}` to create an empty set?"

**Expected Outcome:** You understand literal syntax variations and the quirk of `{}` being a dict, not an empty set. The distinction becomes clear.

---

**Prompt 3 - Hashability Deep Dive (Analyze Level):**

> "I tried to add a list to a set and got an error: 'unhashable type: list'. Explain what a hash value is, why sets need hashable elements, and why lists can't be in sets but tuples can be."

**Expected Outcome:** You grasp the deeper "why" behind the constraint—not memorizing a rule, but understanding the system design.

---

**Prompt 4 - Error Handling Connection (Evaluate Level):**

> "Show me code that demonstrates the difference between `.remove()` and `.discard()`. When should I use each one in production code? What principle guides this choice?"

**Expected Outcome:** You practice decision-making: when do you want an error to alert you (`.remove()`) versus when do you want silent tolerance (`.discard()`)? You think like a professional developer.

**Safety Note:** "Both methods work correctly. The choice depends on your program's intent: fail-fast on mistakes or silent tolerance? There's no universally 'right' answer—context determines the choice."

---

**Closing Reflection:**

You now understand sets as unique, unordered, hash-based collections. You can create them with modern type hints, add and remove elements safely, and understand why they require immutable elements. In the next lesson, you'll explore what sets actually **do**—the mathematical operations that make them powerful.
