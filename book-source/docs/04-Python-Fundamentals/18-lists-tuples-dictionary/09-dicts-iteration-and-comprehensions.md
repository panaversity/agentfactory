---
title: "Dictionaries Part 3 — Iteration and Comprehensions"
chapter: 18
lesson: 9
duration_minutes: 210

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Dictionary Key Iteration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can iterate over dictionary keys using .keys() method and understand when this pattern is appropriate"

  - name: "Dictionary Value Iteration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can iterate over dictionary values using .values() method for value-only processing"

  - name: "Dictionary Items Iteration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can iterate over key-value pairs using .items() and unpack tuples idiomatically"

  - name: "Dictionary Comprehension Syntax"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write dict comprehensions using `{key: value for key, value in iter}` syntax"

  - name: "Dictionary Comprehension Filtering"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can filter dictionary items during comprehension using if conditions"

  - name: "Key and Value Transformation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can transform both keys and values independently or together using comprehensions"

  - name: "Nested Dictionary Concepts"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information"
    measurable_at_this_level: "Student understands that dictionaries can contain other dictionaries as values and can access nested data"

learning_objectives:
  - objective: "LO-004k (B1 - Apply): Iterate dictionaries using .keys(), .values(), and .items() methods idiomatically"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Writing code that selects the appropriate iteration method for different tasks"

  - objective: "LO-005b (B1 - Apply): Write dictionary comprehensions to transform data efficiently"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Converting for loops to dict comprehensions and writing comprehensions with filtering"

  - objective: "LO-005c (B1 - Analyze): Evaluate when to use loops vs comprehensions for readability"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Comparing code clarity and making defensible choices about comprehension use"

  - objective: "LO-004l (B1 - Apply): Transform dictionary keys or values using comprehensions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Writing comprehensions that modify both keys and values in data transformation tasks"

cognitive_load:
  new_concepts: 7
  assessment: "Exactly 7 concepts (B1 limit): .keys(), .values(), .items(), dict comprehension syntax, filtering, key/value transformation, nested dicts intro. No overflow. ✓"

differentiation:
  extension_for_advanced: "Explore nested dict iteration patterns; write complex comprehensions with multiple conditions; design dict-of-dicts data structures; practice with real JSON-like structures"
  remedial_for_struggling: "Start with .items() only; show loop-to-comprehension transformation explicitly; defer nested dicts until comfort with basic iteration; practice filtering separately from transformation"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Dictionaries Part 3 — Iteration and Comprehensions

## Processing Collections Efficiently

In Lesson 8, you learned how to manipulate dictionaries: adding keys, updating values, deleting entries. Now comes the payoff: **processing large collections efficiently**. When you have hundreds or thousands of key-value pairs, you need patterns that scale.

This lesson teaches you three iteration methods (`.keys()`, `.values()`, `.items()`), dictionary comprehensions for transforming data, and a brief introduction to nested dictionaries. By the end, you'll build a real-world word frequency counter and temperature conversion system—practical applications that professional developers use daily.

---

## Concept 1: Iterating Over Keys Only

When you want to loop through only the keys in a dictionary, use the `.keys()` method:

```python
grades: dict[str, int] = {"Alice": 95, "Bob": 87, "Charlie": 92}

# Iterate over keys only
for name in grades.keys():
    print(name)
```

Output:
```
Alice
Bob
Charlie
```

**Why would you want just keys?** When you need to do something *with* the keys themselves—checking their names, modifying them, or building a new collection from them.

Here's a practical example:

```python
student_database: dict[str, int] = {
    "alice.smith": 95,
    "bob.jones": 87,
    "charlie.lee": 92
}

# Extract all student usernames
usernames: list[str] = list(student_database.keys())
print(usernames)
# Output: ['alice.smith', 'bob.jones', 'charlie.lee']
```

#### 💬 AI Colearning Prompt

> "When would you use `.keys()` vs just iterating `for name in grades:`? Are they the same thing?"

The short answer: `for name in grades:` and `for name in grades.keys():` do the same thing. Using `.keys()` is explicit and clearer about intent. Explicit is better than implicit.

---

## Concept 2: Iterating Over Values Only

When you want to loop through only the values (ignoring keys), use `.values()`:

```python
grades: dict[str, int] = {"Alice": 95, "Bob": 87, "Charlie": 92}

# Iterate over values only
for grade in grades.values():
    print(grade)
```

Output:
```
95
87
92
```

**When would you use this?** When you need to analyze the values themselves—calculating averages, finding the highest score, or filtering by value.

Real-world example: calculating class statistics

```python
test_scores: dict[str, int] = {
    "Alice": 95,
    "Bob": 87,
    "Charlie": 92,
    "Diana": 88
}

# Calculate average grade
total: int = sum(test_scores.values())
count: int = len(test_scores)
average: float = total / count

print(f"Average grade: {average:.1f}")
# Output: Average grade: 90.5

# Find highest score
highest: int = max(test_scores.values())
print(f"Highest score: {highest}")
# Output: Highest score: 95
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Given a dictionary of product prices, write code to find the most expensive item and calculate the total inventory value. Which iteration method do you use and why?"

**Expected Outcome**: You'll recognize that `.values()` is perfect for price analysis, and you'll understand how built-in functions like `sum()` and `max()` work with dictionary values.

---

## Concept 3: Iterating Over Both Keys and Values

The most common pattern: you need both the key and value. Use `.items()` to get (key, value) pairs:

```python
grades: dict[str, int] = {"Alice": 95, "Bob": 87, "Charlie": 92}

# Iterate over both
for name, grade in grades.items():
    print(f"{name}: {grade}")
```

Output:
```
Alice: 95
Bob: 87
Charlie: 92
```

Notice the pattern: `for name, grade in grades.items()`. This is **tuple unpacking**. The `.items()` method returns (key, value) tuples, and Python automatically unpacks them into separate variables.

**This is one of Python's most elegant patterns.** It reads naturally: "For each name and grade in the dictionary, print them."

Here's a complete inventory example:

```python
inventory: dict[str, int] = {
    "apples": 50,
    "bananas": 30,
    "oranges": 20,
    "grapes": 15
}

# Show current stock with formatting
print("=== Current Inventory ===")
for item, quantity in inventory.items():
    print(f"{item:12} {quantity:3} units")

print("\n=== Low Stock Alert ===")
for item, quantity in inventory.items():
    if quantity < 25:
        print(f"⚠️ {item} is low: {quantity} units")
```

Output:
```
=== Current Inventory ===
apples            50 units
bananas           30 units
oranges           20 units
grapes            15 units

=== Low Stock Alert ===
⚠️ oranges is low: 20 units
⚠️ grapes is low: 15 units
```

#### 🎓 Expert Insight

> In AI-native development, you don't memorize "use `.items()` for both". You understand: "I need both key and value, so `.items()` is my tool." The same logic applies to `.keys()` and `.values()`—they match your intent. Syntax is cheap; recognizing "I need X from this dictionary" is the skill.

---

## Concept 4: Dictionary Comprehension Syntax

Just like list comprehensions (from Lesson 5), you can transform dictionaries using comprehensions. The syntax is similar:

**List comprehension**: `[expression for item in iterable]`
**Dict comprehension**: `{key: value for key, value in iterable}`

Here's a simple example—doubling all values:

```python
prices: dict[str, int] = {"apple": 2, "banana": 3, "orange": 4}

# Double all prices using a loop
doubled_loop: dict[str, int] = {}
for item, price in prices.items():
    doubled_loop[item] = price * 2

# Same thing as a comprehension
doubled_comp: dict[str, int] = {item: price * 2 for item, price in prices.items()}

print(doubled_comp)
# Output: {'apple': 4, 'banana': 6, 'orange': 8}
```

Both approaches produce the same result. The comprehension is more concise and reads like a mathematical set notation: "For each item and price in the dictionary, create a new entry with the doubled price."

#### 💬 AI Colearning Prompt

> "Compare the loop version and the comprehension version. Why might the comprehension be preferred even though it's harder to read at first?"

The answer: **conciseness and performance**. Comprehensions are typically faster (compiled as a single operation) and more readable once you're comfortable with the syntax.

---

## Concept 5: Filtering in Dictionary Comprehensions

Add an `if` condition to filter entries:

```python
prices: dict[str, int] = {
    "apple": 2,
    "banana": 3,
    "orange": 4,
    "grape": 1
}

# Keep only items under $3
affordable: dict[str, int] = {
    item: price for item, price in prices.items()
    if price < 3
}

print(affordable)
# Output: {'apple': 2, 'grape': 1}
```

The syntax: `{key: value for key, value in iterable if condition}`

**Practical example**: Temperature filtering

```python
temps_celsius: dict[str, int] = {
    "New York": 20,
    "Los Angeles": 25,
    "Chicago": 15,
    "Miami": 28,
    "Boston": 10
}

# Find cities above 20°C
warm_cities: dict[str, int] = {
    city: temp for city, temp in temps_celsius.items()
    if temp > 20
}

print(warm_cities)
# Output: {'Los Angeles': 25, 'Miami': 28}
```

#### ✨ Teaching Tip

> Use Claude Code to explore comprehension filtering: "Show me 3 different ways to filter dictionaries (with .items() loop, with comprehension, with filter() function). Which is most readable?"

---

## Concept 6: Transforming Keys and Values

You can transform **keys**, **values**, or **both**:

**Transform values (Celsius to Fahrenheit)**:

```python
temps_celsius: dict[str, int] = {"NY": 20, "LA": 25, "Chicago": 15}

# Convert to Fahrenheit
temps_fahrenheit: dict[str, float] = {
    city: (temp * 9/5) + 32
    for city, temp in temps_celsius.items()
}

print(temps_fahrenheit)
# Output: {'NY': 68.0, 'LA': 77.0, 'Chicago': 59.0}
```

**Transform keys (make them uppercase)**:

```python
student_emails: dict[str, str] = {
    "alice": "alice@school.edu",
    "bob": "bob@school.edu",
    "charlie": "charlie@school.edu"
}

# Create version with uppercase usernames as keys
uppercase_emails: dict[str, str] = {
    name.upper(): email
    for name, email in student_emails.items()
}

print(uppercase_emails)
# Output: {'ALICE': 'alice@school.edu', 'BOB': 'bob@school.edu', 'CHARLIE': 'charlie@school.edu'}
```

**Transform both** (swap keys and values):

```python
state_abbreviations: dict[str, str] = {
    "New York": "NY",
    "California": "CA",
    "Texas": "TX"
}

# Swap keys and values
abbreviation_to_state: dict[str, str] = {
    abbr: state
    for state, abbr in state_abbreviations.items()
}

print(abbreviation_to_state)
# Output: {'NY': 'New York', 'CA': 'California', 'TX': 'Texas'}
```

---

## Concept 7: Introduction to Nested Dictionaries

Sometimes a dictionary's value is itself a dictionary. This is called a **nested dictionary**:

```python
student_records: dict[str, dict[str, int | str]] = {
    "Alice": {
        "age": 20,
        "grade": 95,
        "major": "Computer Science"
    },
    "Bob": {
        "age": 21,
        "grade": 87,
        "major": "Physics"
    }
}

# Access nested values
print(student_records["Alice"]["grade"])
# Output: 95

print(student_records["Bob"]["major"])
# Output: Physics
```

**When would you use nested dictionaries?** When you have structured data with multiple attributes. Real-world examples:
- User profiles (user ID → `{name, email, age}`)
- API responses (resource → `{id, name, description, status}`)
- Configuration files (section → `{setting1, setting2, setting3}`)

**Iterating nested dictionaries:**

```python
student_records: dict[str, dict[str, int | str]] = {
    "Alice": {"age": 20, "grade": 95},
    "Bob": {"age": 21, "grade": 87}
}

# Print each student's info
for name, info in student_records.items():
    print(f"\n{name}:")
    for key, value in info.items():
        print(f"  {key}: {value}")
```

Output:
```
Alice:
  age: 20
  grade: 95

Bob:
  age: 21
  grade: 87
```

#### 🎓 Expert Insight

> Nested dictionaries are powerful for organizing complex data. But remember: clarity is essential. If a structure becomes too deeply nested (more than 2-3 levels), consider whether a simpler design exists. In professional code, deeply nested structures often become hard to understand.

---

## Practice Exercises

### Exercise 1: Choose the Right Iteration Method

Given this dictionary:

```python
book_inventory: dict[str, int] = {
    "The Great Gatsby": 5,
    "1984": 3,
    "To Kill a Mockingbird": 7,
    "Pride and Prejudice": 4
}
```

Write code to:
1. Print all book titles (keys only) — which method?
2. Calculate total books in inventory (values only) — which method?
3. Print each book with its count formatted nicely (both) — which method?

Expected output for #3:
```
The Great Gatsby: 5
1984: 3
To Kill a Mockingbird: 7
Pride and Prejudice: 4
```

---

### Exercise 2: Dictionary Comprehension — Temperature Conversion

Convert this dictionary of Celsius temperatures to Fahrenheit using a comprehension:

```python
temps_celsius: dict[str, int] = {
    "Monday": 20,
    "Tuesday": 22,
    "Wednesday": 19,
    "Thursday": 25,
    "Friday": 23
}

# Write a comprehension here
# Formula: F = (C * 9/5) + 32

# Expected output: {'Monday': 68.0, 'Tuesday': 71.6, ...}
```

Then, modify it to show only days with temperatures above 70°F using filtering.

---

### Exercise 3: Word Frequency Counter

Given this text, count how many times each word appears:

```python
text: str = "python is great. python is powerful. python is elegant."

# Convert to lowercase and split
words: list[str] = text.lower().split()

# Count frequency using a comprehension or loop
word_counts: dict[str, int] = {}

for word in words:
    word_counts[word] = word_counts.get(word, 0) + 1

print(word_counts)
# Expected: {'python': 3, 'is': 3, 'great.': 1, 'powerful.': 1, 'elegant.': 1}
```

**Challenge**: How would you remove punctuation to get cleaner counts?

---

## Common Patterns and Gotchas

### Pattern 1: Building a Dictionary from Iteration

Create a new dictionary by transforming data:

```python
numbers: list[int] = [1, 2, 3, 4, 5]

# Create a dict mapping number to its square
squares: dict[int, int] = {
    num: num**2 for num in numbers
}

print(squares)
# Output: {1: 1, 2: 4, 3: 9, 4: 16, 5: 25}
```

### Pattern 2: Grouping by Category

Create nested dictionaries for grouped data:

```python
students: list[dict[str, str]] = [
    {"name": "Alice", "grade": "A"},
    {"name": "Bob", "grade": "B"},
    {"name": "Charlie", "grade": "A"},
    {"name": "Diana", "grade": "B"}
]

# Group students by grade
by_grade: dict[str, list[str]] = {}

for student in students:
    grade: str = student["grade"]
    name: str = student["name"]

    if grade not in by_grade:
        by_grade[grade] = []

    by_grade[grade].append(name)

print(by_grade)
# Output: {'A': ['Alice', 'Charlie'], 'B': ['Bob', 'Diana']}
```

### Gotcha 1: Modifying While Iterating

Don't modify a dictionary while looping over it—it can skip items:

```python
inventory: dict[str, int] = {"apples": 50, "bananas": 30, "oranges": 0}

# ❌ Wrong: modifying during iteration
# for item, quantity in inventory.items():
#     if quantity == 0:
#         del inventory[item]  # Can skip items!

# ✓ Right: collect items to delete, then delete after
items_to_delete: list[str] = [
    item for item, qty in inventory.items() if qty == 0
]

for item in items_to_delete:
    del inventory[item]

print(inventory)
# Output: {'apples': 50, 'bananas': 30}
```

### Gotcha 2: Readability Over Cleverness

Comprehensions are powerful, but if they become unreadable, use a loop instead:

```python
prices: dict[str, float] = {"apple": 3.0, "banana": 7.5, "cherry": 4.0, "date": 10.0}

# ✓ Clear comprehension
data: dict[str, float] = {k: v * 2 for k, v in prices.items() if v > 5}

# ❌ Too clever (hard to read)
# result: dict[str, float] = {
#     k: (v * 9/5 + 32) if v > 100 else (v * 1.8 + 32)
#     for k, v in temps.items()
#     if any(x in k for x in ["city", "town"])
# }

# ✓ Better: use a loop for complex logic
result: dict[str, float] = {}
for k, v in temps.items():
    if any(x in k for x in ["city", "town"]):
        if v > 100:
            result[k] = v * 9/5 + 32
        else:
            result[k] = v * 1.8 + 32
```

---

## Real-World Example: Word Frequency Analyzer

Here's a complete example combining iteration, comprehensions, and filtering:

```python
# Sample text
article: str = """Python is amazing. Python is powerful. Python is elegant.
We use Python for AI. Python enables rapid development."""

# Convert to lowercase and split into words
words: list[str] = article.lower().split()

# Count frequency using dict and get()
word_counts: dict[str, int] = {}
for word in words:
    word_counts[word] = word_counts.get(word, 0) + 1

# Show words that appear more than once (filtering)
frequent_words: dict[str, int] = {
    word: count for word, count in word_counts.items()
    if count > 1
}

# Sort by frequency (descending)
sorted_words: list[tuple[str, int]] = sorted(
    frequent_words.items(),
    key=lambda item: item[1],
    reverse=True
)

print("=== Word Frequency ===")
for word, count in sorted_words:
    print(f"{word:12} appears {count} times")

print(f"\nTotal unique words: {len(word_counts)}")
print(f"Words appearing 2+ times: {len(frequent_words)}")
```

Output:
```
=== Word Frequency ===
python       appears 5 times
is           appears 3 times

Total unique words: 15
Words appearing 2+ times: 2
```

This example uses:
- **.items()** to iterate
- **dict comprehension** to filter frequent words
- **sorted()** to order results (a feature you'll use in real projects)

#### 💬 AI Colearning Prompt

> "Walk me through the word frequency analyzer. What does each line do? Why do we use `.items()` in the sorting step?"

---

## Try With AI

Use **ChatGPT web** (or your AI companion tool if already set up) for the following prompts.

### Prompt 1 (Remember): The Three Iteration Methods

> "In Python dictionaries, what are the three main ways to iterate: `.keys()`, `.values()`, and `.items()`? Show me a simple example for each one and explain what you get in each case."

**Expected Outcome**: You recall all three methods and understand what each returns. You see the tuple unpacking pattern in `.items()`.

---

### Prompt 2 (Understand): When to Use Each Method

> "Explain when you would use `.keys()` vs `.values()` vs `.items()`. Give me a realistic scenario for each one where that method is the right choice."

**Expected Outcome**: You understand that the choice depends on whether you need keys only, values only, or both. You can identify the appropriate method for a given task.

---

### Prompt 3 (Apply): Dictionary Comprehension for Temperature Conversion

> "Write a dictionary comprehension that converts temperatures from Celsius to Fahrenheit. Start with a dict like `{'New York': 20, 'Los Angeles': 25}`. Show the original dict, the comprehension code, and the output. Then modify it to only include cities above 70°F in the result."

**Expected Outcome**: You write a correct comprehension with the formula (C * 9/5) + 32, see the output, and add filtering. You understand the `{key: value for ...}` syntax and if conditions.

---

### Prompt 4 (Analyze): Comparing Loops and Comprehensions

> "Given a dictionary of student grades, show me how to filter only passing grades (>= 70) using: 1) a for loop, and 2) a comprehension. Which is more readable? When would you choose the loop over the comprehension?"

**Expected Outcome**: You compare both approaches, understand readability tradeoffs, and can make defensible decisions about when comprehensions are appropriate vs when loops are clearer.

---

**Safety Note**: When iterating dictionaries, don't modify (add/delete keys) while looping. Instead, collect the keys to modify, then apply changes after the loop completes. This prevents skipping items.

