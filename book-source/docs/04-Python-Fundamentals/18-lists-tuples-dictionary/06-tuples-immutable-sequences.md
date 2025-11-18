---
title: "Tuples — Immutable Sequences"
chapter: 18
lesson: 6
duration_minutes: 240

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Tuple Creation & Type Hints"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write `point: tuple[int, int] = (10, 20)` and understand fixed-size vs variable-length tuple type hints"

  - name: "Immutability Understanding"
    proficiency_level: "A2-B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why immutable guarantees are valuable (dict keys, function returns, data integrity) and recognize safety implications"

  - name: "Tuple Unpacking"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can use unpacking assignment: `x, y = coordinates` and understand pattern matching"

  - name: "Hashable Types Recognition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student understands tuples can be dict keys but lists cannot due to hashability; recognizes this constraint"

learning_objectives:
  - objective: "Explain immutability concept and consequences"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student articulates why tuples exist and what guarantee immutability provides"

  - objective: "Create tuples with type hints (fixed-size and variable-length)"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes correct tuple type hints and literals"

  - objective: "Unpack tuples into variables for multiple assignment"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Student unpacks coordinates and uses pattern in realistic scenarios"

  - objective: "Choose tuple over list based on immutability requirements"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student justifies tuple choice in given scenarios"

cognitive_load:
  new_concepts: 7
  assessment: "Exactly 7 concepts (tuple literals, immutability, type hints, indexing/slicing, unpacking, hashable property, tuple methods) at A2-B1 limit. Focused on immutability as core concept ✓"

differentiation:
  extension_for_advanced: "Explore when immutability is critical in multi-threaded code; research how Python's tuple caching works; compare tuple performance vs list"
  remedial_for_struggling: "Focus on single-element `(1,)` syntax gotcha; use game coordinates as primary concrete example; emphasize 'immutable = can't change' before diving into use cases"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 6: Tuples — Immutable Sequences

So far, you've learned that **lists are mutable**—you can change them after creation. But what if you *don't* want data to change? What if you need a guarantee that a value stays exactly as it is?

That's where **tuples** come in. A tuple is an ordered sequence—like a list—but with one fundamental difference: **once created, a tuple cannot be modified**. This immutability isn't a limitation; it's a promise. In this lesson, you'll discover why immutability matters, how to create and use tuples, and when tuples are the right choice over lists.

---

## What Is a Tuple?

A tuple is an ordered, immutable sequence of values. You create it using **parentheses** instead of brackets:

```python
point: tuple[int, int] = (10, 20)
rgb: tuple[int, int, int] = (255, 128, 0)
coordinates: tuple[float, float] = (40.7128, -74.0060)
```

Just like lists, tuples maintain order and support indexing and slicing. But unlike lists, you cannot modify a tuple after creation.

**Attempting to modify raises an error:**

```python
point = (10, 20)
point[0] = 5  # TypeError: 'tuple' object does not support item assignment
```

This error is Python's way of enforcing the immutability guarantee.

#### 💬 AI Colearning Prompt

> "Why does Python have both lists and tuples? What's the advantage of immutability if it prevents me from changing data?"

This is a great question to explore with your AI companion. The answer reveals a design philosophy about safety and intent.

---

## Tuple Syntax: Literals and Type Hints

Tuples use **parentheses** for literals and a special type hint syntax for type annotation.

### Tuple Literals

```python
# Creating tuples with literals
pair: tuple[int, int] = (10, 20)
colors: tuple[str, str, str] = ("red", "green", "blue")
mixed_types: tuple[int, str] = (42, "answer")

# Empty tuple (rare)
empty: tuple[()] = ()
```

### The Single-Element Gotcha

Python has a quirk: to create a single-element tuple, you *must* include a trailing comma:

```python
single_correct: tuple[int] = (1,)      # ✓ This is a tuple
single_wrong: tuple[int] = (1)         # ✗ This is just an int!
```

Without the comma, `(1)` is just the integer `1` in parentheses. With the comma, `(1,)` is a tuple containing the integer `1`.

#### ✨ Teaching Tip

> Use Claude Code to practice this: "Create both `(1)` and `(1,)` and print their types. What does `type()` show for each? Why the difference?"

### Type Hints for Tuples

Python offers two tuple type hint patterns:

**Fixed-size tuples** (when all elements have defined positions):

```python
# Exactly 3 integers
coordinates: tuple[int, int, int] = (10, 20, 30)

# Mixed types: int, str, float
mixed: tuple[int, str, float] = (42, "name", 3.14)
```

**Variable-length tuples** (when you have an unknown number of elements):

```python
# Any number of integers
many_numbers: tuple[int, ...] = (1, 2, 3, 4, 5)

# Any number of strings
words: tuple[str, ...] = ("hello", "world", "python")
```

The `...` (ellipsis) in `tuple[T, ...]` means "any number of items of type T".

---

## Indexing and Slicing

Since tuples are sequences like lists, indexing and slicing work identically:

```python
point: tuple[float, float] = (40.7128, -74.0060)

# Indexing
latitude: float = point[0]      # 40.7128
longitude: float = point[1]     # -74.0060
last: float = point[-1]         # -74.0060 (last element)

# Slicing
first_two: tuple[float, ...] = point[0:2]  # (40.7128, -74.0060)
```

#### 🎓 Expert Insight

> In AI-native development, you don't memorize whether tuples support indexing. You understand: "Tuples are sequences, so they have an order and positions." The syntax—that's what AI handles.

---

## Immutability: The Core Concept

**Immutability means: once created, the tuple cannot change.**

This is not just a programming detail—it's a semantic statement. When you write `point: tuple[int, int] = (10, 20)`, you're telling anyone reading your code (and your future self): "This is a fixed coordinate pair. It will never change."

### Why Immutability Matters

**Safety and Guarantees**: If a function receives a tuple, it knows the data inside won't be accidentally modified.

```python
def print_point(p: tuple[int, int]) -> None:
    print(f"Point: {p[0]}, {p[1]}")
    # Function can't accidentally modify p
```

**Use in Dict Keys**: Only immutable types can be dictionary keys (more on this below).

**Function Returns**: Multiple return values from functions naturally become tuples (preventing accidental modification).

#### 💬 AI Colearning Prompt

> "Imagine you're designing a banking system. You need to return an account balance and interest rate. Should you return a tuple or a list? Why does immutability matter for financial data?"

---

## Unpacking: Assigning Tuple Values to Variables

One of the most powerful tuple features is **unpacking**: assigning tuple elements directly to multiple variables.

```python
# Without unpacking (the long way)
coordinates: tuple[float, float] = (40.7128, -74.0060)
latitude: float = coordinates[0]
longitude: float = coordinates[1]

# With unpacking (the elegant way)
latitude, longitude = coordinates
# Now latitude = 40.7128, longitude = -74.0060
```

Unpacking is especially useful when functions return multiple values:

> **📘 Note**: Don't worry about the `def` syntax below—we'll learn functions in detail in Chapter 20. For now, just understand that `get_player_position()` is a reusable piece of code that returns a tuple `(5, 10)`, and we can unpack it into separate variables `x` and `y`. Focus on the **unpacking pattern**, not the function definition.

```python
def get_player_position() -> tuple[int, int]:
    # Player is at grid position (5, 10)
    return (5, 10)

# Call the function and unpack immediately
x, y = get_player_position()
print(f"Player at ({x}, {y})")  # Player at (5, 10)
```

**Unpacking with ignore patterns:**

If you don't need all values, use underscore to ignore:

```python
coordinates: tuple[int, int, int] = (10, 20, 30)
x, y, _ = coordinates  # Ignore the z value
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Write a function that simulates rolling two dice and returns the results as a tuple. Then unpack the return value to get `dice1` and `dice2`. Finally, ask your AI: 'Why is returning a tuple better than returning a list for this scenario?'"

**Expected Outcome**: You'll understand how unpacking makes code cleaner and how function returns naturally fit the tuple pattern.

---

## Hashable: Using Tuples as Dictionary Keys

This is where tuples become truly valuable: **tuples can be used as dictionary keys, but lists cannot.**

In Python, dictionary keys must be **hashable**—convertible to a unique fixed hash value. Tuples are hashable; lists are not (because lists can change, so their hash would be unreliable).

### Tuples as Dict Keys

```python
# Game map: coordinates map to location names
game_map: dict[tuple[int, int], str] = {
    (0, 0): "Starting Point",
    (10, 20): "Forest Village",
    (30, 40): "Mountain Peak",
    (50, 50): "Dragon's Lair"
}

# Access by coordinate
location: str = game_map[(10, 20)]  # "Forest Village"

# Check if coordinate exists
if (30, 40) in game_map:
    print(game_map[(30, 40)])  # "Mountain Peak"
```

**Why this matters:** Coordinates are naturally immutable—the game world's grid positions don't change. A tuple perfectly expresses this intent.

### Lists Cannot Be Dict Keys

```python
# This will raise an error
player_inventory: dict[list[str], int] = {
    ["sword", "shield"]: 1  # TypeError: unhashable type: 'list'
}
```

Lists are mutable, so Python prevents them as keys to avoid consistency issues.

#### 💬 AI Colearning Prompt

> "In a multi-player game, you track player inventory with coordinates. Why use `dict[tuple[int, int], list[str]]` (dict with tuple keys and list values) instead of `list[list[str]]`?"

---

## Tuple Methods: count() and index()

Tuples support a few methods—the same read-only methods that lists have:

```python
numbers: tuple[int, ...] = (1, 2, 3, 2, 4, 2, 5)

# count(): How many times does a value appear?
count_twos: int = numbers.count(2)  # 3

# index(): What's the position of the first occurrence?
first_three: int = numbers.index(3)  # 2 (position 2)
```

Unlike lists, tuples have **no mutation methods** (`append`, `remove`, etc.)—because tuples can't be modified.

#### 🎓 Expert Insight

> Tuple methods are read-only: `count()` and `index()`. They answer questions ("How many?" and "Where?") but never change the tuple. This reflects immutability throughout the API.

---

## When to Choose Tuples Over Lists

You now know tuples exist. But when should you actually *use* them?

**Choose tuples when:**

1. **Data shouldn't change**: Function coordinates, RGB colors, fixed configurations
2. **You need dict keys**: Use tuples as meaningful identifiers in dictionaries
3. **Multiple return values**: Functions naturally return tuples for multiple values
4. **Semantic intent**: Using a tuple communicates "this data is fixed"

**Choose lists when:**

1. **Data changes**: Growing a shopping cart, collecting dynamic input
2. **Order matters but contents evolve**: Queue of tasks, result accumulation

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "I have a game where locations are defined by coordinates: `locations = [(0, 0), (10, 20), (30, 40)]`. I want to store which locations have treasure. Should I use a list or a tuple for the locations? For the treasure map itself? Explain your reasoning."

**Expected Outcome**: You'll understand the distinction between mutable collections and immutable data, and how to choose accordingly.

---

## Putting It Together: A Game Coordinate System

Let's build a small example combining tuples as keys and unpacking:

```python
# Game world map: coordinates → location name
game_world: dict[tuple[int, int], str] = {
    (0, 0): "Starting Village",
    (10, 15): "Forest",
    (20, 30): "Mountain",
    (40, 50): "Dragon's Tower"
}

# Player position
player_position: tuple[int, int] = (10, 15)

# Unpack player position and look up location
x, y = player_position
current_location: str = game_world.get(player_position, "Unknown")
print(f"Player at ({x}, {y}): {current_location}")
# Output: Player at (10, 15): Forest

# Move player (create new tuple—can't modify current)
x_new, y_new = 20, 30
player_position = (x_new, y_new)  # Reassign entire tuple
new_location: str = game_world.get(player_position, "Unknown")
print(f"Moved to ({x_new}, {y_new}): {new_location}")
# Output: Moved to (20, 30): Mountain
```

Notice: We don't modify `player_position[0]` directly (that would error). We create a *new* tuple and reassign the variable. This is the tuple way.

#### ✨ Teaching Tip

> Use Claude Code to experiment: "Create a game map with 5 locations. Write a function that takes coordinates and returns the location name, with a default 'Unknown' for coordinates not in the map. Test with valid and invalid coordinates."

---

## Common Pitfall: Tuple vs Single-Element Syntax

Students often forget the trailing comma for single-element tuples:

```python
# Oops—this is an int, not a tuple
data = (42)
print(type(data))  # <class 'int'>

# Correct—this is a tuple
data = (42,)
print(type(data))  # <class 'tuple'>
```

Remember: **Parentheses alone don't make a tuple. The comma does.**

---

## Summary: Seven Key Concepts

1. **Tuple literals**: `(1, 2, 3)` using parentheses
2. **Single-element syntax**: Trailing comma required: `(1,)`
3. **Immutability guarantee**: Can't modify after creation
4. **Type hints**: `tuple[T, T]` for fixed-size, `tuple[T, ...]` for variable-length
5. **Indexing and slicing**: Works like lists—`tuple[0]`, `tuple[1:3]`
6. **Unpacking**: Assign values to multiple variables: `x, y = coordinates`
7. **Hashable dict keys**: Tuples as keys; lists not allowed

---

## Try With AI

Use your AI companion (ChatGPT web, Gemini CLI, or Claude CLI) to explore tuples further.

#### Prompt 1 (Remember): Tuple Fundamentals

**Tell your AI:**

> "Explain the difference between creating `data = (1, 2, 3)` and `data = [1, 2, 3]`. What can I do with one that I can't with the other?"

**Expected Outcome**: You'll articulate immutability as the core distinction—lists allow modification, tuples don't.

---

#### Prompt 2 (Understand): Immutability and Safety

**Tell your AI:**

> "In what real-world scenarios does immutability matter? Give me 3 examples where a tuple's 'can't change' property would be valuable (e.g., coordinates, function returns, settings). Explain why mutability would be dangerous in each case."

**Expected Outcome**: You'll recognize that immutability is a feature, not a limitation—it prevents bugs and communicates intent.

---

#### Prompt 3 (Apply): Unpacking and Dict Keys

**Tell your AI:**

> "I'm building a chess game. The board has coordinates (row, column) like (0, 0) to (7, 7). I need to store piece positions and their types. Should coordinates be a tuple or a list? Show me code using coordinates as dict keys to store piece types. Then unpack a coordinate and print the piece at that location."

**Expected Outcome**: You'll write correct code using tuples as dict keys and practice unpacking for practical use.

---

#### Prompt 4 (Analyze): Comparing Data Structures

**Tell your AI:**

> "I have a function that returns three values: player name, health, and position coordinates. Should I return `[name, health, position]`, `(name, health, position)`, or a dict? Analyze the tradeoffs—consider: unpacking, modification, readability, and intent. When would I choose each?"

**Expected Outcome**: You'll evaluate architectural decisions about data representation and recognize that tuples communicate immutability intent.

---

**Safety Note**: Tuples themselves are immutable, but if a tuple contains mutable objects (like a list), those objects *can* be modified. For example: `data = ([1, 2], 3)` has an immutable structure, but you can modify the list inside: `data[0].append(4)`. For now, use simple immutable types in tuples (integers, strings, floats).

