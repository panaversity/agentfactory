---
title: "Choosing the Right Structure"
chapter: 18
lesson: 10
duration_minutes: 240

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Structure Decision Matrix"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can reference decision criteria (mutability, ordering, lookup speed) to choose appropriate data structure for unfamiliar problem"

  - name: "Performance Awareness"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain O(1) dict lookup vs O(n) list search conceptually (not implementation details)"

  - name: "Architectural Thinking"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can justify data structure choices in code review context and articulate tradeoffs"

  - name: "Anti-Pattern Recognition"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify and explain common anti-patterns (using wrong structure for the job)"

learning_objectives:
  - objective: "Choose the correct data structure (list/tuple/dict) based on mutability, ordering, and lookup requirements"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Scenario-based selection with written justification"

  - objective: "Evaluate architectural tradeoffs between data structures (memory, performance, semantics)"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Code review analysis and refactoring recommendations"

  - objective: "Recognize and explain common anti-patterns in data structure selection"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Identifying problematic patterns in provided code samples"

cognitive_load:
  new_concepts: 0
  assessment: "Lesson 10 is synthesis-focused (0 new concepts). Students apply and analyze all 46 prior concepts from Lessons 1-9. Cognitive load is managed through structured decision framework, not volume. ✓"

differentiation:
  extension_for_advanced: "Design data structures for complex scenarios (nested data, multi-level lookups, performance-critical operations). Research algorithmic complexity formally (Big O notation in depth)."
  remedial_for_struggling: "Focus on 3-scenario decision tree: (1) Need to change? List. (2) Need order? List/Tuple. (3) Need fast lookup? Dict. Use decision flowchart as reference throughout."

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement lesson-10"
version: "1.0.0"
---

# Lesson 10: Choosing the Right Structure

## Opening: The Architect's Moment

You've now learned three powerful data structures: **lists** (flexible, mutable sequences), **tuples** (immutable, hashable sequences), and **dictionaries** (fast key-value lookups). But here's the real challenge that separates good developers from great ones: knowing *which structure to use and why*.

This isn't about syntax anymore. You've mastered that. This is about **architectural thinking**—understanding that your data structure choice communicates intent, enables efficient algorithms, and prevents bugs. A professional developer doesn't just ask "Can I do this?" They ask "What's the *right* structure for this job?"

This lesson synthesizes everything you've learned across Lessons 1-9. You won't learn new concepts, but you'll learn how to *think* about data structures like an architect.

---

## Decision Framework: The Four Questions

When faced with a problem that requires storing and organizing data, you're really answering four questions:

### Question 1: Do I Need to Change Data?

**The Mutability Question**

- **List**: Yes, I'll modify this data (add, remove, sort, update items)
- **Tuple**: No, this data should be fixed and unchangeable
- **Dict**: Yes, I'll modify values (but keys stay stable)

**Example**:
```python
# Mutable: Shopping cart gets updated
shopping_cart: list[str] = ["milk", "eggs", "bread"]
shopping_cart.append("cheese")  # ✓ This works
```

```python
# Immutable: Coordinates are fixed
location: tuple[float, float] = (40.7128, -74.0060)  # NYC
location[0] = 40.7130  # ✗ TypeError: tuples don't support item assignment
```

#### 💬 AI Colearning Prompt
> "Why would I choose immutability for data that never changes? What problems does immutability prevent?"

This question alone eliminates half your options. If you need to change data, reach for a list or dict. If not, tuples offer benefits (hashable, memory-efficient, intent-signaling).

---

### Question 2: Does Order Matter?

**The Ordering Question**

- **List**: Order matters (I access by position, or order conveys meaning)
- **Tuple**: Order matters but I don't change it (positional data, multiple return values)
- **Dict**: Order doesn't matter (I access by meaningful key, not position)

**Example**:
```python
# Order matters: To-do list (first task is highest priority)
tasks: list[str] = ["Finish report", "Email client", "Prepare meeting"]
print(tasks[0])  # "Finish report" (position matters)

# Order doesn't matter: Phone directory
contacts: dict[str, str] = {
    "Alice": "555-1234",
    "Bob": "555-5678",
    "Charlie": "555-9012"
}
# I look up by name, not by position
alice_phone = contacts["Alice"]  # ✓
```

#### 🎓 Expert Insight
> In AI-native development, you're not memorizing how Python sorts dicts (insertion order in 3.7+). You're understanding the *intent*: lists when order is semantically meaningful, dicts when you need named lookup. Structure choice is communication.

---

### Question 3: How Do I Find What I Need?

**The Lookup Pattern Question**

- **List**: By position (index 0, 1, 2...) or by searching (slow, O(n))
- **Tuple**: By position (same as list)
- **Dict**: By key (fast, O(1) guaranteed)

This is where **performance awareness** enters. If you have 10,000 items and need to find one frequently, a dict with O(1) lookup is orders of magnitude faster than a list with O(n) search.

**Example: The Performance Difference**

```python
# List: Finding a value requires searching
student_ids: list[int] = [1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008]
target: int = 1007

# Slow approach: Search through every item until found
found: bool = target in student_ids  # O(n) - checks 1001, 1002, ..., 1007
```

```python
# Dict: Direct lookup by key
student_info: dict[int, str] = {
    1001: "Alice",
    1002: "Bob",
    1003: "Charlie",
    1007: "David"
}

# Fast approach: Direct key lookup
student_name: str = student_info[1007]  # O(1) - instant, no searching
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Explain the difference between O(1) and O(n) lookup. Show me a concrete example with a list of 1,000,000 items where a dict is dramatically faster."

**Expected Outcome**: You'll understand why lookup pattern determines structure choice for large datasets.

#### ✨ Teaching Tip
> When performance questions arise, use Claude Code to test: "Create a list with 100,000 names. Time how long it takes to find 'Zebra' using `in` operator. Then do the same with a dict. Show me the time difference."

---

### Question 4: What Does This Structure Say About My Intent?

**The Semantics Question**

Data structures communicate. When you write `list[str]`, you're saying "order and mutability matter." When you write `dict[str, int]`, you're saying "I'm mapping meaningful keys to values." When you write `tuple[int, int]`, you're saying "this pair is atomic, unchangeable."

**Examples of Semantic Communication**:

```python
# "This is an ordered sequence I'll modify"
playlist: list[str] = ["Song A", "Song B", "Song C"]

# "This is a fixed pair (coordinates can't change)"
coordinates: tuple[int, int] = (100, 200)

# "This maps user IDs to their profile data"
user_profiles: dict[int, dict[str, str]] = {
    101: {"name": "Alice", "email": "alice@example.com"},
    102: {"name": "Bob", "email": "bob@example.com"}
}
```

When other developers (or future you) read your code, the structure choice tells them your intent without additional comments.

---

## Decision Matrix: Quick Reference

Here's a framework to help you decide:

| Scenario | Use | Why |
|----------|-----|-----|
| **Ordered list that changes** (add/remove/sort) | `list[T]` | Mutable, ordered, flexible |
| **Fixed collection of values** (coordinates, RGB color) | `tuple[T, ...]` | Immutable, hashable, semantic clarity |
| **Use as dict key** | `tuple[T, ...]` | Only hashable types work as keys |
| **Fast lookup by name/ID** | `dict[K, V]` | O(1) lookup vs O(n) list search |
| **Config settings** (keys are meaningful) | `dict[str, Any]` | Natural mapping, self-documenting |
| **Iterate and modify** | `list[T]` | Natural iteration with index access |

---

## Real-World Scenarios: The Decisions Developers Make

### Scenario 1: Student Database

**Problem**: Store student records. You need to:
- Look up students by ID (frequently)
- Store multiple fields per student (name, email, major, GPA)
- Add/remove students (semester changes)

**Decision**:
```python
# ✓ CORRECT: Dict of student records
students: dict[int, dict[str, str | int]] = {
    1001: {"name": "Alice", "major": "CS", "gpa": 3.8},
    1002: {"name": "Bob", "major": "Math", "gpa": 3.2},
}

# Lookup is fast
student = students[1001]  # O(1)

# Easy to add new students
students[1003] = {"name": "Carol", "major": "Physics", "gpa": 3.5}
```

**Why not a list?**
```python
# ✗ LESS IDEAL: List of student records
students_list: list[dict[str, str | int]] = [
    {"id": 1001, "name": "Alice", "major": "CS", "gpa": 3.8},
    {"id": 1002, "name": "Bob", "major": "Math", "gpa": 3.2},
]

# Lookup requires searching
def find_student(sid: int):
    for student in students_list:  # O(n) - must check every student
        if student["id"] == sid:
            return student
```

The dict is the right structure because lookup speed matters and IDs are meaningful keys.

---

### Scenario 2: Shopping Cart

**Problem**: Store a customer's shopping cart. You need to:
- Keep items in order (display order on website)
- Add/remove items
- Possibly sort by price

**Decision**:
```python
# ✓ CORRECT: List of items
shopping_cart: list[str] = ["Laptop", "Mouse", "USB Cable", "Monitor"]

# Easy to modify
shopping_cart.append("Keyboard")
shopping_cart.remove("USB Cable")

# Easy to sort
shopping_cart.sort()  # Alphabetical order

# Order is preserved and meaningful
print(shopping_cart[0])  # First item in cart
```

**Why not a dict?**
```python
# ✗ LESS IDEAL: Dict of items
cart_dict: dict[str, bool] = {
    "Laptop": True,
    "Mouse": True,
    "USB Cable": True,
    "Monitor": True
}

# Problems:
# - Lost original order (website needs to show items in customer's order)
# - Sorting becomes complicated
# - No natural way to handle duplicates (multiple Laptops)
```

The list is the right structure because order is meaningful and modifications are common.

---

### Scenario 3: Application Configuration

**Problem**: Store app settings (theme, language, timeout values). You need to:
- Access settings by meaningful name (not index)
- Store different value types (strings, integers, booleans)
- Prevent accidental modification (sometimes)

**Decision**:
```python
# ✓ CORRECT: Dict of settings
config: dict[str, str | int | bool] = {
    "theme": "dark",
    "language": "en",
    "timeout_seconds": 30,
    "debug_mode": False
}

# Meaningful access
current_theme: str = config["theme"]

# Easy to add settings
config["max_retries"] = 3

# Self-documenting (keys are meaningful)
```

**Why not a list?**
```python
# ✗ LESS IDEAL: List of settings
config_list: list[str | int | bool] = [
    "dark",      # theme at position 0 - what does this mean?
    "en",        # language at position 1 - why this order?
    30,          # timeout at position 2 - who remembers this?
]

# Accessing requires remembering positions
current_theme = config_list[0]  # Unclear what this is

# Prone to index errors
# config_list[4] might not exist or might be the wrong setting
```

The dict is the right structure because keys communicate intent and random access doesn't require remembering positions.

---

## Anti-Patterns: What NOT to Do

### Anti-Pattern 1: Using List When Dict Makes Sense

```python
# ✗ BAD: List with manual searching
users: list[dict[str, str]] = [
    {"id": "user1", "name": "Alice"},
    {"id": "user2", "name": "Bob"},
]

def find_user(user_id: str) -> dict[str, str] | None:
    for user in users:  # O(n) lookup
        if user["id"] == user_id:
            return user
    return None
```

**Problem**: Every lookup is slow. With 10,000 users, you search 5,000 on average.

**Solution**:
```python
# ✓ GOOD: Dict with O(1) lookup
users: dict[str, str] = {
    "user1": "Alice",
    "user2": "Bob",
}

user_name = users.get("user1", "Unknown")  # O(1) - instant
```

#### 💬 AI Colearning Prompt
> "I have a list of 100,000 products and frequently search by product ID. Why would switching to a dict improve my application's speed? Show me the difference."

---

### Anti-Pattern 2: Using Dict When List Is Natural

```python
# ✗ BAD: Dict when list is more natural
tasks: dict[int, str] = {
    0: "First task",
    1: "Second task",
    2: "Third task",
}

# Problems:
# - Why use integers as keys if list exists?
# - Harder to iterate in order (relies on insertion order in Python 3.7+)
# - Harder to extend/rearrange
```

**Problem**: You're using dict features you don't need, making code less clear.

**Solution**:
```python
# ✓ GOOD: List when order and position matter
tasks: list[str] = [
    "First task",
    "Second task",
    "Third task",
]

# Natural iteration and ordering
for i, task in enumerate(tasks):
    print(f"Task {i}: {task}")
```

---

### Anti-Pattern 3: Mixing Tuple and List Confusingly

```python
# ✗ BAD: Unclear when to use tuple vs list
coordinates: list[int] = [10, 20]  # Looks like it could be modified
point_a = coordinates
coordinates.append(30)  # Now point_a changed too!

# Safer approach:
point_b: tuple[int, int] = (10, 20)  # Clear: immutable
```

**Problem**: Using list for immutable data creates aliasing bugs.

**Solution**: Use tuples when data is conceptually fixed, even if Python doesn't enforce it.

---

## Performance Implications: Understanding the Tradeoffs

This is **conceptual understanding**, not implementation details. You don't need to know how Python implements hash tables—you need to understand the implications:

| Operation | List | Dict |
|-----------|------|------|
| **Add item** | O(1) amortized | O(1) amortized |
| **Remove item** | O(n) (may shift items) | O(1) |
| **Find by position** | O(1) | O(n) (must iterate) |
| **Find by value/key** | O(n) | O(1) |
| **Memory overhead** | Minimal | Higher (hash table) |

**The Trade-off**: Dicts use more memory but give you O(1) lookup. Lists use less memory but require O(n) search. Choose based on your access patterns.

```python
# If you need FAST LOOKUP: Dict is worth the memory
user_count: int = 1_000_000
user_lookup: dict[int, str] = {}  # Higher memory, but instant lookups

# If you rarely lookup and memory matters: List is sufficient
user_count: int = 10
user_list: list[str] = []  # Lower memory, lookups are fine (small dataset)
```

#### ✨ Teaching Tip
> Ask your AI: "Design a solution for storing 1 million employee records. Compare a list-based search vs dict-based lookup. Show me timing differences and discuss when each is appropriate."

---

## Type Hints as Intent Communication

Your type hints tell the story of your choice:

```python
# "I have an ordered sequence I'll modify"
playlist: list[str]

# "I have a fixed pair of values"
location: tuple[float, float]

# "I have a mapping from IDs to names"
employees: dict[int, str]

# "I have a list of dictionaries (records)"
students: list[dict[str, str | int]]

# "I have a dictionary with nested structure"
user_data: dict[str, dict[str, str | int]]
```

Other developers (and future you) understand your intent by reading the type hints. This is part of your architectural thinking.

---

## Exercise 1: Decision Analysis

For each scenario, identify the best data structure and explain why:

**Scenario A**: Store a student's test scores. You need to track which test each score is from and calculate average.

**Scenario B**: Store the colors of a traffic light in order (red, yellow, green). Colors never change.

**Scenario C**: Store a phone book (name → phone number). Need to look up numbers by name frequently.

**Scenario D**: Store a list of items to buy. You'll add/remove items and sometimes reorder them.

**Your Task**: Write down your choice for each scenario and one sentence explaining why. Then, use Claude Code to verify your reasoning:
> "I chose [structure] for [scenario] because [reason]. Does this make sense?"

---

## Exercise 2: Anti-Pattern Recognition

Here's code with a structural choice problem. Identify it and explain how you'd refactor:

```python
# Current code
data: list[dict[str, str]] = [
    {"isbn": "978-0134494166", "title": "Clean Code"},
    {"isbn": "978-0201633610", "title": "Design Patterns"},
    {"isbn": "978-0134685991", "title": "Clean Architecture"},
]

def find_book(isbn: str) -> dict[str, str] | None:
    for book in data:
        if book["isbn"] == isbn:
            return book
    return None

# Every lookup requires searching all books
found = find_book("978-0134494166")
```

**Your Task**: Identify the anti-pattern, explain the performance problem, and refactor using the right structure.

---

## Exercise 3: Real-World Project Design

Design the data structures for a simple **music playlist application**:

- Users have playlists (multiple per user)
- Each playlist has songs in order
- Users can mark favorite songs (immutable list)
- You need to find songs by artist name quickly

**Your Task**:
1. Write the data structures using proper type hints
2. Explain each choice (mutability, ordering, lookup pattern)
3. Show example code creating a user with playlists

---

## Try With AI: Data Structure Architecture Challenge

You've learned the decision framework: mutability, ordering, lookup pattern, performance. Now apply it to real system design—with AI as your code reviewer.

### Part 1: Design Music Library (Your Turn First)

**Before asking AI**, design data structures for a music streaming app:

**Requirements**:
1. **Songs in playlist** - Ordered, can reorder, add/remove
2. **Artist → Song mapping** - Fast lookup of all songs by artist name
3. **Song metadata** - (title, artist, duration) never changes once created
4. **User favorites** - Need fast "is this song favorited?" check

**Your task**: For EACH, choose list/tuple/dict. Write:
1. Type hint showing your choice
2. WHY (mutability? ordering? lookup performance?)
3. Sample data for 2-3 items

Make all decisions BEFORE asking AI.

---

### Part 2: AI Reviews Your Architecture (Discovery)

Share your design with AI:

> "Here's my music library design: [paste your choices]. For each:
> 1. Is my structure choice optimal?
> 2. What's the performance cost? (O(1) vs O(n) lookup)
> 3. Would this scale to 100,000 songs?
> 4. Show me the code to create these structures with sample data"

**Your task**: Evaluate AI's performance feedback.
- Does it confirm list for playlist (ordered, mutable)?
- Does it confirm dict for artist→songs (O(1) lookup)?
- Does it confirm tuple for song metadata (immutable)?
- Does it suggest set for favorites (O(1) membership test)?
- Can you explain WHY each choice scales?

---

### Part 3: Student Teaches AI (Anti-Pattern Recognition)

AI reviewed your design. But does it catch ANTI-PATTERNS?

Challenge AI with bad architecture:

> "A developer wrote this:
> ```python
> # 1 million user records
> users: list[dict[str, str]] = [
>     {'id': 'user1', 'name': 'Alice'},
>     {'id': 'user2', 'name': 'Bob'},
>     # ... million more
> ]
>
> def find_user(user_id: str) -> dict | None:
>     for user in users:
>         if user['id'] == user_id:
>             return user
>     return None
> ```
>
> Problems:
> 1. What's the performance cost of find_user with 1M records?
> 2. Why is this an anti-pattern?
> 3. Refactor to dict[str, str] - show the new structure
> 4. Calculate: How much faster is dict lookup? (O(1) vs O(n))"

**Your task**: Understand performance reality.
- Does AI explain O(n) search means 500,000 comparisons average?
- Does it show dict gives O(1) instant lookup?
- Does it refactor to `users: dict[str, str] = {'user1': 'Alice', ...}`?
- Can you explain when list→dict refactoring is critical?

---

### Part 4: Design E-Commerce System Together (Convergence)

Now design a complete system with AI:

> "Design data structures for an e-commerce cart:
> 1. Shopping cart items (ordered, quantity per item, can modify)
> 2. Product catalog (10,000 products, fast lookup by SKU)
> 3. Product details (name, price, category - fixed after creation)
> 4. Order history (need chronological display)
>
> For EACH component:
> - Choose structure (list/tuple/dict/set)
> - Write type hint
> - Justify: Why this structure?
> - Show sample code
> - Explain performance implications"

**Your task**: Review the complete architecture.
- Does it use appropriate structures for each requirement?
- Does it nest structures where needed (dict of lists? list of tuples?)?
- Does it explain tradeoffs (memory vs. speed)?
- Can you defend every architectural decision?

Iterate if needed:
> "The cart uses list of dicts. What if user adds same item twice? Should we prevent duplicates or track quantity? How does this change the structure?"

---

**Time**: 30-40 minutes total
**Outcome**: You've designed multi-structure systems, understood O(1) vs O(n) performance implications, recognized anti-patterns, and made architecture decisions you can defend in code reviews.

**Safety Note**: Structure choice is an architectural decision. Wrong choice can make 1M record system unusable (O(n) search) vs. instant (O(1) dict). Always consider scale.

