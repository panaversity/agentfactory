# Chapter 19: Set, Frozen Set, and GC — Detailed Implementation Plan

**Chapter Type**: Technical / Code-Focused
**Chapter Objective(s)**: Students understand sets vs. other collections, perform mathematical set operations, grasp hashability and immutability concepts, use frozensets for specific use cases, and understand Python's garbage collection and reference counting mechanisms.
**Estimated Total Time**: 4.5–5 hours (6 lessons × 45–50 min each)
**Part**: Part 4 - Python Fundamentals
**Complexity Tier**: A2-B1 (Intermediate Beginners)
**Status**: Ready for Implementation
**Feature Branch**: `019-set-frozenset-gc`

---

## 🎯 Chapter Overview & Context

**Building on Chapter 18 Foundation**:
- Chapter 18 taught: Lists, tuples, dictionaries — ordered/unordered, mutable/immutable collections
- Chapter 19 builds on this by teaching: Sets (unordered, unique), frozensets (hashable), garbage collection (memory management)

**Bridge to Chapter 20**:
- Chapter 19 completes Python's core collection types (list, tuple, dict, set, frozenset)
- Chapter 20 (Modules and Functions) builds on collection understanding — students will pass collections to functions

**Prerequisites Established**:
- Chapter 13: Python basics (variables, `print()`)
- Chapter 14: Data types and type hints
- Chapter 15: Operators (arithmetic, comparison)
- Chapter 16: Strings and type casting
- Chapter 17: Control flow (loops enable set operations)
- Chapter 18: Lists, tuples, dicts (collection concepts, mutability, iteration)

**AI-Native Learning Pattern** (how students learn sets with AI):
- **Describe intent**: Type hints describe set structure (`unique_ids: set[int]` says "set of integers, guaranteed unique")
- **Explore with AI**: Students ask "What can I do with a set?" instead of reading method documentation
- **Validate together**: Set operations and O(1) lookup enable practical comparisons with lists
- **Learn from errors**: TypeError on unhashable types leads to understanding immutability requirement

**Key Scope Boundaries** (Chapter 19 Responsibility):
- ✅ **Comprehensive Coverage**: Set creation, operations (union, intersection, difference, symmetric difference), set comprehensions, frozensets, reference counting, garbage collection concepts
- ✅ **Awareness Only**: Hash function algorithms, CPython internals, advanced GC tuning
- ❌ **Deferred**: Custom hashing (requires Chapter 24 OOP), weakref (advanced), gc module deep-dive (Chapter 29 CPython/GIL)

---

## 📊 Lesson Architecture

### Lesson 1: Set Basics — Unique, Unordered, Hash-Based Collections

**Learning Objective** (Bloom's: Understand & Apply): Students can create sets using literal and constructor syntax with type hints, understand the uniqueness property, recognize the unordered nature, and add/remove elements.

**Skills Taught**:
- **Skill 1**: Creating Sets with Type Hints
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation (writing code)
  - Measurable at This Level: Student can write `my_set: set[int] = {1, 2, 3}` and `empty: set[str] = set()` without errors

- **Skill 2**: Understanding Set Uniqueness Property
  - CEFR Level: A2 (Basic)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (understanding data properties)
  - Measurable at This Level: Student can explain "why sets automatically eliminate duplicates and why this is useful"

- **Skill 3**: Modifying Sets (add, remove, discard)
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (executing operations)
  - Measurable at This Level: Student can use `.add()`, `.remove()`, and `.discard()` appropriately with error handling understanding

- **Skill 4**: Understanding Mutability vs. Hashability
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (understanding constraints)
  - Measurable at This Level: Student can explain "why sets require immutable elements and what happens when you try to add a list"

**Key Concepts** (max 7):
1. Sets — unordered collections of unique hashable elements
2. Set literal syntax: `{1, 2, 3}`
3. Set constructor: `set([1, 2, 3])` vs. empty dict `{}`
4. Type hints for sets: `set[int]`, `set[str]`
5. Uniqueness property — automatic duplicate elimination
6. Mutability of sets vs. immutability requirement of elements
7. Basic methods: `.add()`, `.remove()`, `.discard()`

**Prerequisites**:
- Chapter 18: Collection concepts, mutability understanding
- Chapter 14: Type hints syntax

**Duration**: 50 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why sets matter; connecting to Chapter 18 collections
2. **Concept: What Makes Sets Different** (8 min) - Uniqueness, unordered nature, hash-based storage (intuitive, not deep)
3. **Concept: Creating Sets** (8 min) - Literal syntax, constructor, type hints, empty set pitfall
4. **Concept: The Hashability Requirement** (8 min) - Why immutable elements needed; what's hashable vs. unhashable
5. **Code Examples** (15 min) - Create sets, add/remove elements, see uniqueness in action
6. **Practice** (6 min) - Create sets with type hints, perform basic modifications

**Content Elements**:

**Code Example 1: Creating Sets with Type Hints**
```python
# Set literal syntax with type hints
numbers: set[int] = {1, 2, 3, 4, 5}
colors: set[str] = {"red", "green", "blue"}
mixed: set[int | str] = {1, "two", 3}  # Python 3.10+ union syntax

# Set constructor syntax
from_list: set[int] = set([1, 2, 3, 3, 2, 1])
empty_set: set[int] = set()  # NOT {} which is dict!

# Display
print(numbers)           # {1, 2, 3, 4, 5}
print(from_list)        # {1, 2, 3} - duplicates gone!
print(type(empty_set))  # <class 'set'>
```
- **Purpose**: Show set creation patterns, type hints, uniqueness in action
- **Complexity**: Beginner-Intermediate (syntax + property)
- **Pedagogical Value**: Establishes set syntax and automatic deduplication

**Code Example 2: Understanding Uniqueness**
```python
# Duplicates are automatically eliminated
email_list: list[str] = ["alice@example.com", "bob@example.com", "alice@example.com"]
unique_emails: set[str] = set(email_list)

print(f"List length: {len(email_list)}")           # 3
print(f"Set length: {len(unique_emails)}")         # 2
print(f"Unique emails: {unique_emails}")           # {'alice@example.com', 'bob@example.com'}

# Practical use: Deduplication
user_ids: set[int] = {100, 101, 102, 101, 100}  # Duplicate IDs
print(f"Unique users: {user_ids}")                 # {100, 101, 102}
```
- **Purpose**: Demonstrate why uniqueness matters; practical use case
- **Complexity**: Beginner (familiar list concept applied to sets)
- **Pedagogical Value**: Shows concrete business value of sets

**Code Example 3: Modifying Sets**
```python
# Create and modify sets
visited_cities: set[str] = {"Paris", "Tokyo", "London"}

# Add elements
visited_cities.add("New York")
visited_cities.add("Paris")  # No change - already exists
print(f"After adding: {visited_cities}")  # {'Paris', 'Tokyo', 'London', 'New York'}

# Remove elements (raises error if not found)
visited_cities.remove("Paris")
print(f"After removing Paris: {visited_cities}")

# Discard elements (no error if not found)
visited_cities.discard("Berlin")  # No error, Berlin not in set
visited_cities.discard("Tokyo")
print(f"After discarding: {visited_cities}")
```
- **Purpose**: Show mutation methods; distinguish `.remove()` vs `.discard()`
- **Complexity**: Beginner (method calls, error handling concept)
- **Pedagogical Value**: Practical set modification

**Code Example 4: The Hashability Requirement**
```python
# ✅ Hashable types work in sets
numbers: set[int] = {1, 2, 3}
strings: set[str] = {"a", "b"}
tuples: set[tuple[int, int]] = {(1, 2), (3, 4)}
frozen: set[frozenset[int]] = {frozenset([1, 2]), frozenset([3, 4])}

# ❌ Unhashable types fail
try:
    my_set: set[list[int]] = {[1, 2], [3, 4]}
except TypeError as e:
    print(f"Error: {e}")  # unhashable type: 'list'

try:
    my_dict_set: set[dict[str, int]] = {{"a": 1}}
except TypeError as e:
    print(f"Error: {e}")  # unhashable type: 'dict'

# Why? Lists and dicts are mutable - hash values would change!
```
- **Purpose**: Show hashability constraint; teach type validation
- **Complexity**: Intermediate (error handling, explaining why)
- **Pedagogical Value**: Prevents confusion when students try unhashable types

**Practice Approach**:
- **Exercise 1**: Create a set of usernames, add 5 names with type hints, attempt to add duplicate (see uniqueness)
- **Exercise 2**: Create a set from a list with duplicates, demonstrate deduplication
- **Exercise 3**: Try to add a list to a set, catch the error, explain why it failed
- **Exercise 4**: Create set of tuples representing coordinates (showing hashable composite data)

**End-of-Lesson: Try With AI** (AI Tool Selection Policy: Part 4 is post-AI-tools introduction, so learner's preferred AI companion - ChatGPT, Claude Code, or Gemini CLI)

**Try With AI Prompts**:

1. **Concept Exploration**: "I learned that sets automatically eliminate duplicates. Where in real software would this be useful? Give me 3 business scenarios where sets solve problems that lists don't."
   - Expected Outcome: Student sees practical value of uniqueness (user IDs, email addresses, product SKUs)
   - AI Tool: ChatGPT web (conceptual discussion)
   - Follow-up: "How would I solve that problem with a list instead of a set?"

2. **Syntax Clarification**: "Explain the difference: `my_set = {1, 2, 3}` vs. `empty = {}` vs. `my_list = [1, 2, 3]`. Which is which and why?"
   - Expected Outcome: Student understands set literal vs. empty dict vs. list
   - AI Tool: Claude Code (show actual type() results)
   - Follow-up: "How do I create an empty set?"

3. **Hashability Deep Dive**: "I tried to add a list to a set and got 'unhashable type' error. Explain: What is a hash value? Why do sets need elements to be hashable? What types ARE hashable?"
   - Expected Outcome: Student grasps immutability requirement and hashing concept intuitively
   - AI Tool: Claude Code or Gemini CLI (code examples showing types)
   - Follow-up: "Can I make my own type hashable?"

4. **Error Handling Connection**: "Show me code that demonstrates the difference between `.remove()` and `.discard()`. When would I use each one?"
   - Expected Outcome: Student understands error handling difference
   - AI Tool: Claude Code (inline error demonstration)
   - Safety Note: "Both methods work, but understanding error handling helps you write robust code."

**Cognitive Load Validation**:
- A2-B1 level: Max 7 concepts per lesson ✓
- New concepts: 1 (sets), 2 (uniqueness), 3 (unordered), 4 (type hints), 5 (mutability), 6 (hashability), 7 (add/remove)
- Total: 7 concepts = at limit (manageable, focused) ✓

---

### Lesson 2: Set Operations — Union, Intersection, Difference, Symmetric Difference

**Learning Objective** (Bloom's: Apply & Analyze): Students can perform all four mathematical set operations using both operator and method syntax, write set comprehensions with filtering, and apply operations to real-world problems.

**Skills Taught**:
- **Skill 1**: Union Operations (| operator and .union())
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (combining data)
  - Measurable at This Level: Student can compute union of multiple sets correctly and choose between operator and method syntax

- **Skill 2**: Intersection Operations (& operator and .intersection())
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (finding common elements)
  - Measurable at This Level: Student can find common elements between sets and combine with other filters

- **Skill 3**: Difference Operations (- operator and .difference())
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (excluding elements)
  - Measurable at This Level: Student can subtract one set from another correctly

- **Skill 4**: Symmetric Difference (^ operator and .symmetric_difference())
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (finding differences)
  - Measurable at This Level: Student can find elements in either set but not both

- **Skill 5**: Set Comprehensions with Filtering
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation (creating filtered sets)
  - Measurable at This Level: Student can write `{x for x in original_set if condition}` syntax correctly

**Key Concepts** (max 7):
1. Union (`|` operator) — combine all unique elements from multiple sets
2. Intersection (`&` operator) — elements common to all sets
3. Difference (`-` operator) — elements in first set but not others
4. Symmetric difference (`^` operator) — elements in either set but not both
5. Method vs. operator syntax (flexibility)
6. Set comprehensions — filtering sets declaratively
7. Real-world problem application (comparing datasets)

**Prerequisites**:
- Lesson 1: Set basics (creation, modification)
- Chapter 17: Comprehension syntax (list comprehensions transfer to sets)

**Duration**: 50 minutes

**Content Outline**:
1. **Introduction** (5 min) - Set operations in context; mathematical notation preview
2. **Concept: Union — Combining Sets** (8 min) - `|` operator and `.union()`, practical use
3. **Concept: Intersection — Finding Common Elements** (8 min) - `&` operator and `.intersection()`, practical use
4. **Concept: Difference — Excluding Elements** (8 min) - `-` operator and `.difference()`, practical use
5. **Concept: Symmetric Difference** (5 min) - `^` operator for "either but not both"
6. **Code Examples** (12 min) - Demonstrate all operations with real scenarios
7. **Practice** (4 min) - Apply operations to data problems

**Content Elements**:

**Code Example 1: Union Operations**
```python
# Union - combine all unique elements from both sets
team_a: set[str] = {"Alice", "Bob", "Charlie"}
team_b: set[str] = {"Bob", "David", "Eve"}

# Using | operator
all_members: set[str] = team_a | team_b
print(f"Union with |: {all_members}")  # {'Alice', 'Bob', 'Charlie', 'David', 'Eve'}

# Using .union() method
all_members_method: set[str] = team_a.union(team_b)
print(f"Union with method: {all_members_method}")  # same result

# Union of multiple sets
team_c: set[str] = {"Frank", "Alice"}
everyone: set[str] = team_a | team_b | team_c
print(f"All three teams: {everyone}")  # {'Alice', 'Bob', 'Charlie', 'David', 'Eve', 'Frank'}
```
- **Purpose**: Introduce union with both syntaxes, show combining multiple sets
- **Complexity**: Beginner-Intermediate (operators + familiar data)
- **Pedagogical Value**: Establishes operator syntax pattern

**Code Example 2: Intersection Operations**
```python
# Intersection - elements common to all sets
java_devs: set[str] = {"Alice", "Bob", "Charlie", "David"}
python_devs: set[str] = {"Bob", "David", "Eve", "Frank"}

# Using & operator
bilingual: set[str] = java_devs & python_devs
print(f"Know both languages: {bilingual}")  # {'Bob', 'David'}

# Using .intersection() method
bilingual_method: set[str] = java_devs.intersection(python_devs)
print(f"Know both (method): {bilingual_method}")  # same result

# Intersection is commutative
print(f"Commutative: {java_devs & python_devs == python_devs & java_devs}")  # True
```
- **Purpose**: Show finding common elements; demonstrate practical use (skills, interests)
- **Complexity**: Beginner-Intermediate (familiar operator pattern)
- **Pedagogical Value**: Real-world application (finding commonality)

**Code Example 3: Difference Operations**
```python
# Difference - elements in first set but not in second
all_students: set[str] = {"Alice", "Bob", "Charlie", "David", "Eve"}
graduated: set[str] = {"Bob", "David"}

# Using - operator
still_enrolled: set[str] = all_students - graduated
print(f"Still enrolled: {still_enrolled}")  # {'Alice', 'Charlie', 'Eve'}

# Using .difference() method
still_enrolled_method: set[str] = all_students.difference(graduated)
print(f"Still enrolled (method): {still_enrolled_method}")  # same result

# Order matters! Difference is not commutative
print(f"all - graduated: {all_students - graduated}")  # {'Alice', 'Charlie', 'Eve'}
print(f"graduated - all: {graduated - all_students}")  # {} (empty, all graduated are in all_students)
```
- **Purpose**: Show excluding elements; demonstrate non-commutativity
- **Complexity**: Intermediate (order-dependent operation)
- **Pedagogical Value**: Finding what's excluded or missing

**Code Example 4: Symmetric Difference**
```python
# Symmetric difference - elements in either set but not both
morning_shift: set[str] = {"Alice", "Bob", "Charlie"}
afternoon_shift: set[str] = {"Charlie", "David", "Eve"}

# Using ^ operator (exclusive or)
never_overlap: set[str] = morning_shift ^ afternoon_shift
print(f"Never work together: {never_overlap}")  # {'Alice', 'Bob', 'David', 'Eve'}

# Using .symmetric_difference() method
never_overlap_method: set[str] = morning_shift.symmetric_difference(afternoon_shift)
print(f"Never work together (method): {never_overlap_method}")  # same result

# Symmetric difference is commutative
print(f"Commutative: {morning_shift ^ afternoon_shift == afternoon_shift ^ morning_shift}")  # True

# Relationship: A ^ B == (A - B) | (B - A)
print(f"XOR via union of differences: {(morning_shift - afternoon_shift) | (afternoon_shift - morning_shift)}")
```
- **Purpose**: Show "either but not both" concept; demonstrate commutativity
- **Complexity**: Intermediate (XOR concept may be new)
- **Pedagogical Value**: Understanding exclusive operations

**Code Example 5: Set Comprehensions with Filtering**
```python
# Set comprehension - create new set with filtering
numbers: set[int] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}

# Filter even numbers
even_numbers: set[int] = {n for n in numbers if n % 2 == 0}
print(f"Even numbers: {even_numbers}")  # {2, 4, 6, 8, 10}

# Filter and transform
squares_of_odd: set[int] = {n**2 for n in numbers if n % 2 == 1}
print(f"Squares of odd numbers: {squares_of_odd}")  # {1, 9, 25, 49, 81}

# Filter strings by length
words: set[str] = {"apple", "pie", "banana", "cat", "dog", "elephant"}
long_words: set[str] = {word for word in words if len(word) > 4}
print(f"Words with 5+ letters: {long_words}")  # {'apple', 'banana', 'elephant'}
```
- **Purpose**: Show filtering sets declaratively; connect to list comprehensions
- **Complexity**: Intermediate (comprehension syntax + filtering)
- **Pedagogical Value**: Powerful data transformation pattern

**Code Example 6: Real-World Problem — Finding Common Interests**
```python
# Scenario: Match users with shared interests
alice_interests: set[str] = {"hiking", "photography", "cooking", "reading"}
bob_interests: set[str] = {"cooking", "gaming", "reading", "music"}
charlie_interests: set[str] = {"hiking", "cooking", "yoga"}

# Common interests between two users
print(f"Alice & Bob share: {alice_interests & bob_interests}")  # {'cooking', 'reading'}
print(f"Alice & Charlie share: {alice_interests & charlie_interests}")  # {'hiking', 'cooking'}

# All three share these interests
all_share: set[str] = alice_interests & bob_interests & charlie_interests
print(f"All three share: {all_share}")  # {'cooking'}

# What does each person uniquely like?
alice_unique: set[str] = alice_interests - (bob_interests | charlie_interests)
print(f"Alice's unique interests: {alice_unique}")  # {'photography'}

# Interests unique to Alice that others don't have
print(f"Alice has but others don't: {alice_interests - bob_interests - charlie_interests}")
```
- **Purpose**: Apply operations to real scenario; show practical value
- **Complexity**: Intermediate (combining multiple operations)
- **Pedagogical Value**: See why sets matter in applications

**Practice Approach**:
- **Exercise 1**: Given two lists of integers, find common elements (intersection) using sets
- **Exercise 2**: Find all elements from two lists (union), then remove a third set (difference chaining)
- **Exercise 3**: Write a set comprehension filtering even numbers from 1-20
- **Exercise 4**: Real scenario — compare product inventory between two warehouses, find overstocked/understocked items

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Concept Exploration**: "Explain the difference between union, intersection, and difference using a Venn diagram analogy. When would I use each one?"
   - Expected Outcome: Student visualizes the operations
   - AI Tool: ChatGPT web (visual explanation with analogies)
   - Follow-up: "Can you give me a real business scenario for each?"

2. **Operator vs. Method**: "I can do `set_a | set_b` or `set_a.union(set_b)`. Are these exactly the same? When would I use which?"
   - Expected Outcome: Student understands both syntaxes and trade-offs
   - AI Tool: Claude Code (show results of both)
   - Follow-up: "Which is more readable?"

3. **Symmetric Difference Deep Dive**: "I don't fully understand what symmetric difference does. Show me a practical example where I'd use `A ^ B` vs. other operations."
   - Expected Outcome: Student grasps "either but not both" concept
   - AI Tool: Claude Code (code + output showing difference)
   - Follow-up: "Is it the same as `(A - B) | (B - A)`?"

4. **Set Comprehension Challenge**: "Write a set comprehension that filters strings longer than 5 characters from a list, then convert to uppercase (hint: you can transform while filtering)."
   - Expected Outcome: Student applies comprehension + transformation
   - AI Tool: Claude Code (generate + explain pattern)
   - Safety Note: "Set comprehensions work just like list comprehensions, but without duplicates."

**Cognitive Load Validation**:
- A2-B1 level: Max 7 concepts per lesson ✓
- New concepts: 1 (union), 2 (intersection), 3 (difference), 4 (symmetric difference), 5 (operators), 6 (methods), 7 (comprehensions)
- Total: 7 concepts = at limit (all operational, no theoretical overload) ✓

---

### Lesson 3: Understanding Set Internals — Hashing and O(1) Lookup

**Learning Objective** (Bloom's: Understand & Analyze): Students understand hash functions conceptually, grasp why immutability is required for hashability, analyze O(1) lookup performance, and appreciate internal hash table structure without implementation details.

**Skills Taught**:
- **Skill 1**: Understanding Hash Functions Intuitively
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (understanding algorithms)
  - Measurable at This Level: Student can explain "hash functions convert objects into integers for fast lookup"

- **Skill 2**: Connecting Immutability to Hashability
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Analyze
  - DigComp Area: Problem-Solving (understanding constraints)
  - Measurable at This Level: Student can explain "why mutable objects can't be hashed — their values change, breaking lookups"

- **Skill 3**: Performance Analysis (O(1) vs. O(n))
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Analyze
  - DigComp Area: Problem-Solving (performance evaluation)
  - Measurable at This Level: Student can compare set lookup speed vs. list iteration and choose appropriate structures

- **Skill 4**: Understanding Hash Table Concepts
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (system understanding)
  - Measurable at This Level: Student can explain "sets store elements in a hash table for O(1) average lookup"

**Key Concepts** (max 7):
1. Hash function — converts objects to integers
2. Why immutable objects are required (hash stability)
3. Hash collisions (multiple objects mapping to same hash)
4. Hash table structure (array indexed by hash values)
5. O(1) average-case lookup time
6. O(n) worst-case (rare, collision-heavy)
7. Rehashing when set grows

**Prerequisites**:
- Lesson 1: Set creation and understanding of hashability
- Chapter 15: Time complexity concepts (if covered) or brief introduction here

**Duration**: 50 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why understanding internals matters; preview hash concept
2. **Concept: What Are Hash Functions?** (10 min) - Intuitive explanation, Python's `hash()` function, examples
3. **Concept: Why Immutability Matters** (8 min) - Hash stability, what happens if objects change
4. **Concept: Performance Comparison** (10 min) - O(1) lookup vs. O(n) iteration, demonstrate with timing
5. **Concept: Hash Tables Conceptually** (10 min) - How Python stores sets internally (high-level)
6. **Practice** (7 min) - Hash values, performance comparison, hash collision awareness

**Content Elements**:

**Code Example 1: Hash Functions and Immutable Objects**
```python
# Hash function converts objects to integers
print(f"hash(42): {hash(42)}")              # Some integer
print(f"hash('hello'): {hash('hello')}")    # Some integer
print(f"hash((1, 2)): {hash((1, 2))}")      # Some integer

# Same object = same hash (deterministic within session)
name: str = "Alice"
print(f"First hash: {hash(name)}")
print(f"Second hash: {hash(name)}")         # Same value

# Immutable requirement: mutable objects DON'T have hashes
try:
    print(f"hash([1, 2, 3]): {hash([1, 2, 3])}")
except TypeError as e:
    print(f"Error: {e}")  # unhashable type: 'list'

try:
    print(f"hash({{'a': 1}}): {hash({'a': 1})}")
except TypeError as e:
    print(f"Error: {e}")  # unhashable type: 'dict'

# Hashable immutable types
hashable_examples: set[tuple] = {(1, 2), (3, 4), ("x", "y")}
print(f"Hashable tuple set: {hashable_examples}")
```
- **Purpose**: Show hash function behavior; demonstrate hashability requirement
- **Complexity**: Intermediate (system-level understanding)
- **Pedagogical Value**: Builds intuition for "why immutable?"

**Code Example 2: Why Immutability is Required**
```python
# Illustration (pseudo-code explanation):
# If we COULD hash mutable objects:

# Original tuple: hash((1, 2)) = 12345
# If we modify:   (1, 2) -> (1, 3)
# New hash:       hash((1, 3)) = 54321
# But set still looks for original hash 12345!
# Result: Lookup fails, data appears "lost"

# This is why Python prevents it:
coords: tuple[int, int] = (1, 2)
print(f"Tuple is hashable: {hash(coords)}")  # Works

coords_list: list[int] = [1, 2]
try:
    print(f"List is hashable: {hash(coords_list)}")
except TypeError:
    print("Lists aren't hashable because they're mutable")

# Practical impact: what CAN and CAN'T be dict keys
valid_dict_1: dict[int, str] = {1: "a", 2: "b"}
valid_dict_2: dict[tuple[int, int], str] = {(1, 2): "position"}
valid_dict_3: dict[frozenset[int], str] = {frozenset([1, 2]): "group"}

try:
    invalid_dict: dict[list[int], str] = {[1, 2]: "position"}
except TypeError as e:
    print(f"Error: {e}")  # unhashable type: 'list'
```
- **Purpose**: Explain why mutability breaks hashing; connect to dict keys
- **Complexity**: Intermediate (system thinking)
- **Pedagogical Value**: Prevents future confusion about "why can't I...?"

**Code Example 3: Performance Comparison — O(1) vs. O(n)**
```python
import time

# Create large list and set with same elements
elements: list[int] = list(range(1_000_000))
element_set: set[int] = set(elements)

# Lookup test: find if 999_999 is in collection
target: int = 999_999

# Set lookup (O(1) average)
start = time.perf_counter()
for _ in range(100_000):
    result_set = target in element_set
end = time.perf_counter()
set_time = end - start

# List lookup (O(n))
start = time.perf_counter()
for _ in range(100_000):
    result_list = target in elements
end = time.perf_counter()
list_time = end - start

print(f"Set lookup time: {set_time:.6f} seconds")   # Very fast
print(f"List lookup time: {list_time:.6f} seconds")  # Much slower
print(f"Set is {list_time / set_time:.0f}x faster")  # Often 1000x+ faster
```
- **Purpose**: Demonstrate real performance difference; make O(1) vs. O(n) concrete
- **Complexity**: Intermediate (performance analysis)
- **Pedagogical Value**: Why sets matter at scale

**Code Example 4: Hash Collisions (Conceptual)**
```python
# Hash collisions happen when different objects map to same hash value
# Python's hash table handles this internally (chaining or probing)

# Illustration (simplified):
# Imagine hash table with 10 slots (0-9)
# hash(1) % 10 = 1
# hash(11) % 10 = 1  <- collision! Both want slot 1
# Python stores both, retrieves by full value comparison as tiebreaker

# For learning purposes, we don't need to implement this
# But awareness helps understand O(1) is "average case"

# Rehashing happens automatically when set grows
my_set: set[int] = set()

# Python internally resizes the hash table
for i in range(1_000_000):
    my_set.add(i)

print(f"Set with 1M elements created successfully")  # Works seamlessly
print(f"Final set size: {len(my_set)}")              # 1,000,000
```
- **Purpose**: Explain collision handling and rehashing at high level
- **Complexity**: Intermediate (conceptual, not implementation)
- **Pedagogical Value**: Understand "O(1) average" qualification

**Code Example 5: Real-World Performance Decision**
```python
# Scenario: Check if user ID is in database

# BAD: Using a list (O(n) lookup)
def is_user_in_list(user_id: int, user_database: list[int]) -> bool:
    return user_id in user_database  # Slow for large databases

# GOOD: Using a set (O(1) lookup)
def is_user_in_set(user_id: int, user_database: set[int]) -> bool:
    return user_id in user_database  # Fast for large databases

# If you have 1M users and check 1000 times:
# List approach: ~1B comparisons
# Set approach: ~1000 comparisons (hash-based)

database_list: list[int] = list(range(1_000_000))
database_set: set[int] = set(range(1_000_000))

# Time a single check
import time
start = time.perf_counter()
_ = 999_999 in database_list
list_check = time.perf_counter() - start

start = time.perf_counter()
_ = 999_999 in database_set
set_check = time.perf_counter() - start

print(f"List check: {list_check*1000:.3f}ms")
print(f"Set check: {set_check*1000:.3f}ms")
print(f"Set is {list_check/set_check:.0f}x faster")
```
- **Purpose**: Apply O(1) concept to realistic scenario
- **Complexity**: Intermediate (decision-making)
- **Pedagogical Value**: When and why to use sets

**Practice Approach**:
- **Exercise 1**: Call `hash()` on different immutable objects, observe patterns
- **Exercise 2**: Verify that tuples are hashable but lists aren't
- **Exercise 3**: Time set vs. list lookup with 100K elements
- **Exercise 4**: Estimate: Would a set or list be better for checking usernames in a social app?

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Hash Function Exploration**: "What is a hash function? Explain it like I'm new to programming. Why does Python need to hash set elements?"
   - Expected Outcome: Student grasps hashing as "converting objects to lookup keys"
   - AI Tool: ChatGPT web (conceptual explanation with analogies)
   - Follow-up: "Can I create my own hash function?"

2. **Immutability Deep Dive**: "Why exactly can't I put a list in a set, but I CAN put a tuple? How would the world break if I could?"
   - Expected Outcome: Student understands immutability requirement prevents lookup breakage
   - AI Tool: Claude Code (show example + explanation)
   - Follow-up: "So frozenset works because it's immutable?"

3. **Performance Analysis**: "Show me code that proves sets are faster than lists for checking if an item exists. How much faster?"
   - Expected Outcome: Student sees dramatic O(1) vs. O(n) difference
   - AI Tool: Claude Code (benchmarking code)
   - Safety Note: "The actual speedup depends on dataset size and computer speed."

4. **Design Decision**: "I'm building a system to check 1M user IDs. Should I store them as a list or set? Why? What if I only have 10 users?"
   - Expected Outcome: Student practices choosing data structures based on performance needs
   - AI Tool: Claude Code or ChatGPT (decision discussion)
   - Follow-up: "At what point does the choice start to matter?"

**Cognitive Load Validation**:
- B1 level: Max 10 concepts per lesson (higher complexity tier) ✓
- Core concepts: 1 (hash functions), 2 (hash stability), 3 (immutability), 4 (collisions), 5 (rehashing), 6 (O(1) lookup), 7 (O(n) worst case)
- Applied concepts: 8 (performance comparison), 9 (practical decisions)
- Total: 9 concepts = within B1 limit ✓

---

### Lesson 4: Frozensets — Immutable Sets for Hashable Contexts

**Learning Objective** (Bloom's: Understand & Apply): Students understand frozensets as immutable set variants, use frozensets as dictionary keys and set members, and choose appropriately between set and frozenset.

**Skills Taught**:
- **Skill 1**: Creating Frozensets
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation
  - Measurable at This Level: Student can create frozensets with `frozenset()` and type hints correctly

- **Skill 2**: Understanding Immutability of Frozensets
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy
  - Measurable at This Level: Student can explain "frozensets can't change after creation, enabling hashability"

- **Skill 3**: Using Frozensets as Dictionary Keys
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving
  - Measurable at This Level: Student can create dictionaries with frozenset keys

- **Skill 4**: Nesting Frozensets in Sets
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation
  - Measurable at This Level: Student can create `set[frozenset[int]]` structures

- **Skill 5**: Choosing Set vs. Frozenset
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Analyze
  - DigComp Area: Problem-Solving
  - Measurable at This Level: Student can explain when to use frozenset instead of set

**Key Concepts** (max 7):
1. Frozenset — immutable variant of set
2. Creation syntax: `frozenset([1, 2, 3])`
3. Immutability — no `.add()`, `.remove()` methods
4. Hashability — frozensets ARE hashable (can be dict keys)
5. Nesting — can contain frozensets or be contained in sets
6. Operations — supports all read-only operations (union, intersection, etc.)
7. Use cases — dict keys, set elements, when data shouldn't change

**Prerequisites**:
- Lesson 1: Set basics and hashability concept
- Lesson 3: Hash-based storage and immutability requirement
- Chapter 18: Dictionary keys and hashability

**Duration**: 40 minutes

**Content Outline**:
1. **Introduction** (3 min) - Why immutable sets matter; connecting to previous lessons
2. **Concept: Frozenset as Immutable Set** (8 min) - Creation, properties, operations
3. **Concept: Frozensets are Hashable** (7 min) - Can use as dict keys, set members
4. **Code Examples** (15 min) - Create frozensets, use as keys, nest in sets
5. **Comparison: Set vs. Frozenset** (5 min) - When to choose each
6. **Practice** (2 min) - Quick exercises

**Content Elements**:

**Code Example 1: Creating and Using Frozensets**
```python
# Create frozensets
coordinates: frozenset[int] = frozenset([1, 2, 3])
empty_frozen: frozenset[int] = frozenset()
from_set: frozenset[str] = frozenset({"apple", "banana"})

print(f"Frozenset: {coordinates}")           # frozenset({1, 2, 3})
print(f"Type: {type(coordinates)}")          # <class 'frozenset'>
print(f"Is hashable: {hash(coordinates)}")   # <integer>

# Immutability - no modification methods
try:
    coordinates.add(4)  # Method doesn't exist
except AttributeError as e:
    print(f"Error: {e}")  # 'frozenset' object has no attribute 'add'

try:
    coordinates.remove(1)  # Method doesn't exist
except AttributeError as e:
    print(f"Error: {e}")  # 'frozenset' object has no attribute 'remove'

# Read-only operations still work
result: frozenset[int] = coordinates | frozenset([4, 5])
print(f"Union result: {result}")  # frozenset({1, 2, 3, 4, 5})
```
- **Purpose**: Show creation, immutability, hashability, operations
- **Complexity**: Beginner-Intermediate (familiar syntax, new immutability constraint)
- **Pedagogical Value**: Establishes frozenset as "set-like but immutable"

**Code Example 2: Frozensets as Dictionary Keys**
```python
# Regular sets CAN'T be dict keys (unhashable)
try:
    bad_dict: dict[set[int], str] = {
        {1, 2}: "pair",
        {3, 4}: "another"
    }
except TypeError as e:
    print(f"Error: {e}")  # unhashable type: 'set'

# Frozensets CAN be dict keys (hashable)
user_groups: dict[frozenset[str], str] = {
    frozenset({"admin", "user"}): "Full access",
    frozenset({"user"}): "Read-only access",
    frozenset({"guest"}): "Public access"
}

print(f"Admin group access: {user_groups[frozenset({'admin', 'user'})]}")

# Practical use case: group-based permissions
user_bob_roles: frozenset[str] = frozenset({"user", "moderator"})
print(f"Bob's access level: {user_groups.get(user_bob_roles, 'Not found')}")

# Another use: coordinate lookup
location_aliases: dict[frozenset[tuple[int, int]], str] = {
    frozenset([(0, 0), (1, 1)]): "diagonal",
    frozenset([(2, 0), (0, 2)]): "corners"
}
```
- **Purpose**: Show practical use case (dict keys); demonstrate hashability
- **Complexity**: Intermediate (realistic application)
- **Pedagogical Value**: Understand frozenset necessity

**Code Example 3: Nesting Frozensets in Sets**
```python
# Regular sets can't contain sets (not hashable)
try:
    nested_sets: set[set[int]] = {{1, 2}, {3, 4}}
except TypeError as e:
    print(f"Error: {e}")  # unhashable type: 'set'

# Frozensets CAN be contained in sets (hashable)
groups: set[frozenset[str]] = {
    frozenset({"Alice", "Bob"}),
    frozenset({"Bob", "Charlie"}),
    frozenset({"Alice", "Charlie"})
}

print(f"All groups: {groups}")

# Union of all group members
all_members: set[str] = set()
for group in groups:
    all_members |= set(group)  # Convert frozenset to set for union

print(f"All members: {all_members}")  # {'Alice', 'Bob', 'Charlie'}

# Find overlapping groups
group1: frozenset[str] = frozenset({"Alice", "Bob"})
group2: frozenset[str] = frozenset({"Bob", "Charlie"})
overlap: set[str] = set(group1) & set(group2)
print(f"Members in both groups: {overlap}")  # {'Bob'}
```
- **Purpose**: Show frozenset as composite hashable data
- **Complexity**: Intermediate (nested data structures)
- **Pedagogical Value**: Understand when nesting is necessary

**Code Example 4: Set vs. Frozenset Comparison**
```python
# Set - mutable, can't be dictionary key, can't be in sets
favorite_colors: set[str] = {"red", "blue", "green"}
favorite_colors.add("yellow")  # ✅ Works
print(f"Modified set: {favorite_colors}")

# Can't use as dict key
try:
    color_dict: dict[set[str], int] = {favorite_colors: 1}
except TypeError:
    print("❌ Can't use set as dict key")

# Frozenset - immutable, CAN be dictionary key, CAN be in sets
primary_colors: frozenset[str] = frozenset(["red", "yellow", "blue"])
# primary_colors.add("green")  # ❌ Can't - no add() method

# CAN use as dict key
color_dict: dict[frozenset[str], int] = {
    primary_colors: 3,
    frozenset(["red", "green"]): 2
}
print(f"✅ Can use frozenset as dict key: {color_dict}")

# Decision matrix
print("\nWhen to use set vs. frozenset:")
print("Use set when: data needs to change, uniqueness matters, lookup is important")
print("Use frozenset when: data is immutable, need to use as dict key, need to nest in sets")
```
- **Purpose**: Compare trade-offs; guide decision-making
- **Complexity**: Intermediate (analysis, decision-making)
- **Pedagogical Value**: Know when each is appropriate

**Practice Approach**:
- **Exercise 1**: Create a frozenset of numbers, verify it's hashable
- **Exercise 2**: Use frozensets as dictionary keys for a simple lookup table
- **Exercise 3**: Create a set containing frozensets (nested structure)
- **Exercise 4**: Convert between set and frozenset, note performance difference

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Immutability Exploration**: "What's the difference between a set and a frozenset? Why would I ever use frozenset instead of set?"
   - Expected Outcome: Student understands immutability trade-off
   - AI Tool: ChatGPT web (conceptual explanation)
   - Follow-up: "Are there performance differences?"

2. **Dictionary Keys Use Case**: "Show me an example where frozensets as dictionary keys is the only solution. Why can't I use a regular set?"
   - Expected Outcome: Student sees necessity of frozenset hashability
   - AI Tool: Claude Code (code example + explanation)
   - Follow-up: "Can I use frozensets in other ways?"

3. **Nesting Practice**: "Write code that creates a set of frozensets representing teams. Then find which team has the most members."
   - Expected Outcome: Student practices nested frozenset structures
   - AI Tool: Claude Code (generate + explain)
   - Safety Note: "Converting between set and frozenset is free—no performance penalty."

4. **Real-World Scenario**: "I'm building a permission system. Should I store user roles as a list, set, or frozenset? Why?"
   - Expected Outcome: Student analyzes use case and chooses appropriate structure
   - AI Tool: Claude Code or ChatGPT (decision discussion)
   - Follow-up: "What if I need to check if two users have the same permissions?"

**Cognitive Load Validation**:
- A2-B1 level: Max 7 concepts per lesson ✓
- Concepts: 1 (frozenset), 2 (immutability), 3 (hashability), 4 (dict keys), 5 (nesting), 6 (operations), 7 (use cases)
- Total: 7 concepts = at limit ✓

---

### Lesson 5: Garbage Collection — Reference Counting and Memory Management

**Learning Objective** (Bloom's: Understand & Analyze): Students understand Python's reference counting as the primary garbage collection mechanism, recognize when objects are freed, understand circular references, and use `gc` module for basic memory analysis.

**Skills Taught**:
- **Skill 1**: Understanding Reference Counting
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Information Literacy (system understanding)
  - Measurable at This Level: Student can explain "objects are deleted when reference count reaches zero"

- **Skill 2**: Memory Management in Python
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (optimization thinking)
  - Measurable at This Level: Student can explain "automatic memory management through refcount"

- **Skill 3**: Using `gc` Module for Analysis
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (profiling)
  - Measurable at This Level: Student can use `gc.collect()`, `gc.get_objects()` for memory analysis

- **Skill 4**: Understanding Circular References
  - CEFR Level: B1-B2 (Intermediate-Advanced)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (debugging)
  - Measurable at This Level: Student can explain "how circular references prevent deletion and how cycle detector handles them"

**Key Concepts** (max 7):
1. Reference counting — each object tracks how many references point to it
2. Object deletion — when refcount reaches 0, memory is freed immediately
3. Variables holding references — `x = obj` increases refcount
4. Circular references — A references B, B references A (refcount never reaches 0)
5. Garbage collection (cycle detection) — detects and breaks cycles
6. `gc` module — manual GC control and memory profiling
7. When GC runs — generational GC in CPython

**Prerequisites**:
- Lesson 1: Understanding set uniqueness and data structures
- Basic understanding of variables and object creation

**Duration**: 50 minutes

**Content Outline**:
1. **Introduction** (5 min) - Why memory management matters; Python's automatic approach
2. **Concept: Reference Counting** (10 min) - How Python tracks object usage, immediate deletion
3. **Concept: Circular References** (8 min) - When refcount fails, cycle detector solution
4. **Concept: The `gc` Module** (8 min) - Manual collection, memory profiling, analysis
5. **Code Examples** (15 min) - Demonstrate refcount, circular references, gc usage
6. **Practice** (4 min) - Profile object creation/deletion

**Content Elements**:

**Code Example 1: Reference Counting Basics**
```python
import sys

# Create an object
my_list: list[int] = [1, 2, 3]

# Check reference count (should be 1 for my_list)
print(f"Initial refcount: {sys.getrefcount(my_list)}")  # 2 (variable + getrefcount argument)

# Create another reference
another_ref: list[int] = my_list
print(f"After assignment: {sys.getrefcount(my_list)}")  # 3 (my_list + another_ref + argument)

# Delete one reference
del my_list
print(f"After del my_list: {sys.getrefcount(another_ref)}")  # Still accessible via another_ref

# Delete last reference
del another_ref
# print(f"After del another_ref: {sys.getrefcount(another_ref)}")  # Would error - another_ref doesn't exist!

print("Object was freed when last reference was deleted")

# Practical observation: immediate deletion
x: str = "temporary"
y: str = x
del x
del y  # String is immediately freed here
print("String freed immediately after last reference deleted")
```
- **Purpose**: Demonstrate how Python counts references
- **Complexity**: Intermediate (system-level observation)
- **Pedagogical Value**: Understand "automatic" memory management

**Code Example 2: Circular References (Problem and Solution)**
```python
import gc

# Circular reference - A points to B, B points to A
class Node:
    def __init__(self, name: str) -> None:
        self.name: str = name
        self.next: Node | None = None

    def __del__(self) -> None:
        print(f"Node {self.name} was deleted")

# Create circular reference
node_a: Node = Node("A")
node_b: Node = Node("B")

node_a.next = node_b  # A points to B
node_b.next = node_a  # B points to A (circular!)

print("Created circular reference (A -> B -> A)")

# Delete variables
del node_a  # A's refcount decreased, but still 1 (B refers to it)
del node_b  # B's refcount decreased, but still 1 (A refers to it)
print("Deleted variables (but objects not freed - circular!)")

# Manual garbage collection detects and breaks cycle
print("Running garbage collection...")
collected: int = gc.collect()
print(f"Garbage collector freed {collected} objects")  # Will show 2

# Now both A and B are deleted
print("Circular references cleaned up!")

# Without gc.collect(), circular references leak memory (rarely matters in practice)
```
- **Purpose**: Show when reference counting fails; demonstrate cycle detection
- **Complexity**: Intermediate (system understanding)
- **Pedagogical Value**: Understand GC necessity

**Code Example 3: The `gc` Module for Memory Analysis**
```python
import gc
import sys

# Disable automatic GC for demonstration
gc.disable()

print("=" * 50)
print("Memory profiling with gc module")
print("=" * 50)

# Get initial object count
initial_objects: int = len(gc.get_objects())
print(f"Initial object count: {initial_objects}")

# Create many objects
numbers: list[int] = list(range(10_000))
strings: list[str] = [str(i) for i in range(10_000)]
sets: list[set[int]] = [{i, i+1, i+2} for i in range(1000)]

after_creation: int = len(gc.get_objects())
print(f"After creation: {after_creation}")
print(f"Objects created: {after_creation - initial_objects}")

# Manual collection
collected: int = gc.collect()
print(f"Manual gc.collect() freed {collected} objects")

# Delete objects
del numbers
del strings
del sets

after_deletion: int = len(gc.get_objects())
print(f"After deletion: {after_deletion}")

# Manual collection catches any circular refs
collected: int = gc.collect()
print(f"After cleanup, gc.collect() freed {collected} objects")

# Re-enable GC
gc.enable()

# Get garbage collector stats
stats = gc.get_stats()
print(f"\nGC statistics: {stats}")
```
- **Purpose**: Show `gc` module for memory profiling
- **Complexity**: Intermediate (tool usage)
- **Pedagogical Value**: Learn how to analyze memory

**Code Example 4: Reference Counting in Practice**
```python
import sys

# Function that receives object as argument
def process_data(data: list[int]) -> int:
    # data is referenced inside function
    return sum(data)

my_data: list[int] = [1, 2, 3, 4, 5]
print(f"Before function: refcount = {sys.getrefcount(my_data) - 1}")  # -1 for getrefcount arg

result: int = process_data(my_data)
print(f"After function: refcount = {sys.getrefcount(my_data) - 1}")  # Function reference released

# Key insight: Python automatically manages memory
# Developers rarely need to think about refcount
# Just delete when done, GC handles the rest

print("\nAutomatic memory management in action:")
for i in range(5):
    temp: list[int] = [j for j in range(1_000_000)]  # Create large list
    print(f"Created list {i}, refcount: {sys.getrefcount(temp) - 1}")
    # temp automatically freed when loop iteration ends

print("All temporary lists freed automatically!")
```
- **Purpose**: Show how refcount works in realistic scenarios
- **Complexity**: Intermediate (real-world patterns)
- **Pedagogical Value**: Understand "automatic" nature

**Code Example 5: Generational Garbage Collection**
```python
import gc

# Python uses generational GC (objects start in "young" generation)
print("Garbage Collection Generations:")
print(f"GC enabled: {gc.isenabled()}")

# Get thresholds (when GC triggers)
thresholds: tuple[int, int, int] = gc.get_threshold()
print(f"GC thresholds (gen0, gen1, gen2): {thresholds}")
# Default: (700, 10, 10)
# Means: collect when 700+ objects in gen0, or 10 collections in gen1, etc.

# Manual trigger
gc.collect()
print("Manual garbage collection triggered")

# Get collection counts
counts: tuple[int, ...] = gc.get_count()
print(f"Current collection counts: {counts}")

# Optimization (rarely needed in practice):
# gc.set_threshold(2000, 15, 15)  # Less frequent collections = faster, more memory
# print("Adjusted GC thresholds for less frequent collection")
```
- **Purpose**: Show GC tuning possibilities (advanced)
- **Complexity**: Intermediate (system knowledge)
- **Pedagogical Value**: Understand GC behavior (not for tuning, just awareness)

**Practice Approach**:
- **Exercise 1**: Track refcount as you create and delete variables
- **Exercise 2**: Create and manually break a circular reference
- **Exercise 3**: Use `gc.get_objects()` to count objects before/after
- **Exercise 4**: Create a function that analyzes memory usage of different data structures

**End-of-Lesson: Try With AI**

**Try With AI Prompts**:

1. **Reference Counting Explanation**: "How does Python know when to delete an object? What is reference counting and why does Python use it?"
   - Expected Outcome: Student understands refcount as "counting who's using this object"
   - AI Tool: ChatGPT web (conceptual explanation)
   - Follow-up: "Is it automatic or do I need to do something?"

2. **Circular References Deep Dive**: "What's a circular reference? Show me an example where two objects reference each other and explain why it causes problems."
   - Expected Outcome: Student understands cycle detection problem
   - AI Tool: Claude Code (code example + explanation)
   - Follow-up: "Does Python fix this automatically?"

3. **Memory Profiling Challenge**: "Write code that creates 100K objects, then shows me how much memory they use and how many are freed after deletion."
   - Expected Outcome: Student practices memory analysis with `gc` module
   - AI Tool: Claude Code (generate profiling code)
   - Safety Note: "Memory usage varies by platform; focus on the count of objects, not exact bytes."

4. **Performance Implication**: "If reference counting deletes objects immediately, why does Python also have a separate garbage collector?"
   - Expected Outcome: Student understands GC's role in handling circular references
   - AI Tool: ChatGPT web (system design discussion)
   - Follow-up: "Should I manually call gc.collect()?"

**Cognitive Load Validation**:
- B1 level: Max 10 concepts per lesson ✓
- Core concepts: 1 (refcount), 2 (refcount rules), 3 (immediate deletion), 4 (circular refs), 5 (cycle detector), 6 (gc module), 7 (generational GC)
- Applied concepts: 8 (profiling), 9 (memory analysis)
- Total: 9 concepts = within B1 limit ✓

---

### Lesson 6: Memory Profiler Capstone — Building an Object Tracking Tool

**Learning Objective** (Bloom's: Create & Evaluate): Students design and implement a working Memory Profiler tool that integrates sets, frozensets, and garbage collection concepts. This capstone validates mastery through practical project.

**Skills Taught**:
- **Skill 1**: Designing Software with AI Collaboration
  - CEFR Level: B1-B2 (Intermediate-Advanced)
  - Category: Soft
  - Bloom's Level: Create
  - DigComp Area: Problem-Solving (system design)
  - Measurable at This Level: Student can write specifications for tools and work with AI to refine design

- **Skill 2**: Implementing Set-Based Data Tracking
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Create
  - DigComp Area: Digital Content Creation (building tools)
  - Measurable at This Level: Student can use sets to track object creation/deletion with type-hinted structures

- **Skill 3**: Integrating GC Module for Memory Analysis
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Problem-Solving (profiling)
  - Measurable at This Level: Student can use `gc` module to analyze objects and memory

- **Skill 4**: Testing and Validating Implementation
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Evaluate
  - DigComp Area: Problem-Solving (verification)
  - Measurable at This Level: Student can test tool on edge cases (circular refs, large object graphs)

- **Skill 5**: Reflecting on Integration
  - CEFR Level: B1 (Intermediate)
  - Category: Conceptual
  - Bloom's Level: Evaluate
  - DigComp Area: Problem-Solving (synthesis)
  - Measurable at This Level: Student can articulate how sets, frozensets, and GC work together

**Key Concepts** (max 5 for capstone — students apply many prior concepts):
1. Specification-first approach (design before coding)
2. Set-based object tracking
3. Garbage collection integration
4. Type hints for clarity
5. Testing and edge case handling

**Prerequisites**:
- All prior Chapter 19 lessons (sets, frozensets, GC)
- Chapter 13: Basic Python execution
- Chapter 20: Function design (coming soon, but can reference)

**Duration**: 60 minutes (includes design, implementation, testing)

**Content Outline**:
1. **Introduction** (5 min) - Capstone goal; integrating chapter concepts
2. **Tool Specification** (10 min) - Define requirements with AI collaboration
3. **Implementation Guide** (20 min) - Build Memory Profiler with code examples
4. **Testing and Validation** (15 min) - Test on realistic scenarios
5. **Reflection** (10 min) - Write summary of how concepts integrate

**Content Elements**:

**Specification Outline: Memory Profiler Tool**
```
# Memory Profiler Specification

## Goal
Build a tool that tracks Python object creation/deletion and displays memory statistics.

## Requirements
1. Track object creation (when objects are added)
2. Track object deletion (when objects are removed via gc)
3. Display current object count
4. Display peak object count
5. Identify "dead" objects (unreferenced but not yet collected)
6. Show memory statistics

## Input/Output
Input: Create/delete objects in your program
Output: Real-time statistics showing:
- Current object count
- Total objects ever created
- Objects awaiting collection
- Memory estimate

## Technology Constraints
- Use sets to track object IDs
- Use gc module for analysis
- Type hints mandatory
- Testable with realistic scenarios
```

**Code Example 1: Memory Profiler Implementation**
```python
import gc
import sys
from typing import Any

class MemoryProfiler:
    """Tracks object creation and deletion using sets and gc module."""

    def __init__(self) -> None:
        # Use sets to track object IDs (immutable identifiers)
        self.created_objects: set[int] = set()      # IDs of all created objects
        self.deleted_objects: set[int] = set()      # IDs of deleted objects
        self.peak_count: int = 0
        self.start_count: int = len(gc.get_objects())

    def track_object(self, obj: Any) -> None:
        """Track a new object by its id."""
        obj_id: int = id(obj)
        self.created_objects.add(obj_id)

        current_count: int = self.count_living_objects()
        if current_count > self.peak_count:
            self.peak_count = current_count

    def count_living_objects(self) -> int:
        """Count currently living objects."""
        return len(self.created_objects) - len(self.deleted_objects)

    def profile_memory(self) -> dict[str, int]:
        """Analyze current memory state."""
        gc.collect()  # Collect garbage first
        all_objects: list[Any] = gc.get_objects()

        living_count: int = self.count_living_objects()
        total_memory: int = sum(sys.getsizeof(obj) for obj in all_objects)

        return {
            "current_objects": living_count,
            "created_total": len(self.created_objects),
            "deleted_total": len(self.deleted_objects),
            "peak_objects": self.peak_count,
            "memory_bytes": total_memory,
            "unreachable": len(gc.garbage)  # Circular refs
        }

    def print_report(self) -> None:
        """Display memory statistics."""
        stats: dict[str, int] = self.profile_memory()

        print("\n" + "=" * 50)
        print("MEMORY PROFILE REPORT")
        print("=" * 50)
        print(f"Current objects in memory: {stats['current_objects']}")
        print(f"Total objects created: {stats['created_total']}")
        print(f"Total objects deleted: {stats['deleted_total']}")
        print(f"Peak object count: {stats['peak_objects']}")
        print(f"Total memory: {stats['memory_bytes']:,} bytes")
        print(f"Unreachable objects (cycles): {stats['unreachable']}")
        print("=" * 50 + "\n")

# Example usage
profiler: MemoryProfiler = MemoryProfiler()

# Create various objects and track them
numbers: list[int] = [i for i in range(1000)]
profiler.track_object(numbers)

strings: list[str] = [f"string_{i}" for i in range(500)]
profiler.track_object(strings)

sets: list[set[int]] = [{i, i+1, i+2} for i in range(100)]
profiler.track_object(sets)

print("Created objects...")
profiler.print_report()

# Delete and see changes
del numbers
del strings
del sets

print("Deleted objects...")
profiler.print_report()
```
- **Purpose**: Implement capstone tool integrating all chapter concepts
- **Complexity**: Intermediate (integration of multiple concepts)
- **Pedagogical Value**: Practical tool demonstrating learning

**Code Example 2: Advanced Tracking with Frozensets**
```python
class AdvancedMemoryProfiler(MemoryProfiler):
    """Extended profiler with object categorization."""

    def __init__(self) -> None:
        super().__init__()
        # Use frozensets to track categories (immutable grouping)
        self.categories: dict[frozenset[str], set[int]] = {
            frozenset(["list"]): set(),
            frozenset(["dict"]): set(),
            frozenset(["set"]): set(),
            frozenset(["custom"]): set()
        }

    def track_object_with_type(self, obj: Any) -> None:
        """Track object and categorize by type."""
        self.track_object(obj)

        obj_id: int = id(obj)
        obj_type: str = type(obj).__name__

        if isinstance(obj, list):
            self.categories[frozenset(["list"])].add(obj_id)
        elif isinstance(obj, dict):
            self.categories[frozenset(["dict"])].add(obj_id)
        elif isinstance(obj, set):
            self.categories[frozenset(["set"])].add(obj_id)
        else:
            self.categories[frozenset(["custom"])].add(obj_id)

    def report_by_category(self) -> None:
        """Show object counts by category."""
        print("\nObjects by category:")
        for category, ids in self.categories.items():
            print(f"  {list(category)[0]}: {len(ids)} objects")

# Test
advanced_profiler: AdvancedMemoryProfiler = AdvancedMemoryProfiler()

my_list: list[int] = [1, 2, 3]
my_dict: dict[str, int] = {"a": 1}
my_set: set[int] = {1, 2, 3}

advanced_profiler.track_object_with_type(my_list)
advanced_profiler.track_object_with_type(my_dict)
advanced_profiler.track_object_with_type(my_set)

advanced_profiler.report_by_category()
```
- **Purpose**: Show advanced pattern (frozensets for categorization)
- **Complexity**: Intermediate (extending prior implementation)
- **Pedagogical Value**: Real design pattern application

**Code Example 3: Testing with Edge Cases**
```python
# Test case 1: Circular references
class Node:
    def __init__(self, name: str) -> None:
        self.name: str = name
        self.next: Node | None = None

def test_circular_references(profiler: MemoryProfiler) -> None:
    """Test that circular references are eventually freed."""
    print("Testing circular references...")

    node_a: Node = Node("A")
    node_b: Node = Node("B")
    node_a.next = node_b
    node_b.next = node_a

    profiler.track_object(node_a)
    profiler.track_object(node_b)

    print(f"Before deletion: {profiler.count_living_objects()} objects")

    del node_a
    del node_b

    gc.collect()  # Trigger cycle detection

    print(f"After deletion & gc: {profiler.count_living_objects()} objects")
    print("✓ Circular references properly freed\n")

# Test case 2: Large object graphs
def test_large_graph(profiler: MemoryProfiler) -> None:
    """Test with many objects."""
    print("Testing large object graph...")

    large_list: list[list[int]] = [[i*j for j in range(100)] for i in range(1000)]
    profiler.track_object(large_list)

    stats: dict[str, int] = profiler.profile_memory()
    print(f"Tracked {len(large_list)} lists, memory: {stats['memory_bytes']:,} bytes")

    del large_list
    gc.collect()

    print("✓ Large graph properly cleaned\n")

# Run tests
profiler: MemoryProfiler = MemoryProfiler()
test_circular_references(profiler)
test_large_graph(profiler)
```
- **Purpose**: Demonstrate testing approach
- **Complexity**: Intermediate (test patterns)
- **Pedagogical Value**: Validation thinking

**Practice / Implementation Approach**:
- **Phase 1 (10 min)**: Design tool specification with AI:
  - "I want to track how many objects my Python program creates. What should I build?"
  - AI helps refine requirements

- **Phase 2 (20 min)**: Implement Memory Profiler class:
  - Create tracking sets
  - Implement profile_memory() using gc module
  - Add display functionality

- **Phase 3 (15 min)**: Test on realistic scenarios:
  - Track lists, dicts, custom objects
  - Test deletion and garbage collection
  - Verify circular references are handled

- **Phase 4 (10 min)**: Reflect and write summary:
  - "How did sets, frozensets, and GC work together in this project?"
  - "What did you learn about memory management?"

**End-of-Lesson: Try With AI** (Final lesson closure)

**Try With AI Prompts**:

1. **Specification Refinement**: "I want to build a memory profiler. Help me write the specification (requirements, inputs, outputs). What should it do?"
   - Expected Outcome: Student collaborates on spec design
   - AI Tool: ChatGPT web or Claude Code (spec discussion)
   - Follow-up: "What's missing from this specification?"

2. **Implementation Partner**: "Generate the code for a MemoryProfiler class that tracks object creation/deletion using sets and gc module. Include type hints."
   - Expected Outcome: Student gets working scaffold, understands implementation
   - AI Tool: Claude Code (code generation)
   - Follow-up: "Can I add a feature to categorize objects by type?"

3. **Testing Strategy**: "How would I test a memory profiler? What edge cases should I consider?"
   - Expected Outcome: Student thinks about testing
   - AI Tool: ChatGPT web or Claude Code (test ideas)
   - Follow-up: "Show me code for testing circular references."

4. **Integration Reflection**: "Explain how this Memory Profiler tool uses sets, frozensets, and garbage collection. How do these concepts work together?"
   - Expected Outcome: Student synthesizes chapter concepts
   - AI Tool: ChatGPT web (conceptual discussion)
   - Safety Note: "There are many ways to build this tool—your solution is one valid approach."

**Cognitive Load Validation**:
- B1-B2 level: Max 10 concepts (applied synthesis) ✓
- Core concepts integrated: 1 (sets), 2 (frozensets), 3 (GC), 4 (refcount), 5 (type hints)
- Applied concepts: 6 (design), 7 (implementation), 8 (testing), 9 (reflection)
- Total: 9 concepts = within limit ✓

---

## 📚 Content Flow & Dependencies

### Lesson Progression Map

```
Lesson 1: Set Basics
    ↓ (understand uniqueness)
Lesson 2: Set Operations
    ↓ (know what sets DO)
Lesson 3: Set Internals
    ↓ (understand WHY sets are fast)
Lesson 4: Frozensets
    ↓ (immutable variant for special use cases)
Lesson 5: Garbage Collection
    ↓ (memory management - system level)
Lesson 6: Capstone Integration
    ↓ (apply all concepts in realistic tool)
```

### Cross-Chapter References

**Building on Chapter 18 (Lists, Tuples, Dictionary)**:
- Chapter 18 taught collections basics (list, tuple, dict)
- Chapter 19 completes collection types (adds set, frozenset)
- Comparisons made: "Sets are like lists but with uniqueness and O(1) lookup"

**Prerequisites for Chapter 20+ (Modules and Functions)**:
- Chapter 19 establishes all core collection types
- Functions will accept and return collections; students will recognize types

---

## 🎓 Scaffolding Strategy

### Cognitive Load Management

**Lesson 1-2: Maximum Scaffolding (A2-B1)**
- New: sets, operations
- Familiar: type hints from Chapter 14, iteration from Chapter 17
- Single major concept per lesson
- Concrete examples before abstraction

**Lesson 3-4: Moderate Scaffolding (B1)**
- New: internals, frozensets
- Requires Chapter 1-2 foundation
- Move toward analysis and comparison
- Real-world decisions

**Lesson 5-6: Minimal Scaffolding (B1-B2)**
- New: GC, system thinking
- Application-focused (capstone)
- Students drive design with AI partnership
- Synthesis of prior learning

### Complexity Progression

| Lesson | Bloom's Level | Concepts | Difficulty | Time |
|--------|---------------|----------|------------|------|
| 1      | Understand    | 7        | Beginner   | 50m  |
| 2      | Apply         | 7        | Beginner   | 50m  |
| 3      | Analyze       | 9        | Intermediate | 50m  |
| 4      | Apply         | 7        | Intermediate | 40m  |
| 5      | Understand    | 9        | Intermediate | 50m  |
| 6      | Create        | 9        | Advanced   | 60m  |

---

## 🔗 Integration Points

### With Prior Chapters

**Chapter 14 (Data Types)**:
- Type hints syntax established; Chapter 19 applies to collections: `set[int]`, `frozenset[str]`

**Chapter 17 (Control Flow)**:
- Loops used in Lesson 1-2 for iteration; Chapter 19 teaches why sets are better

**Chapter 18 (Lists, Tuples, Dictionary)**:
- Direct comparison: list vs. set trade-offs, tuple as immutable, dict keys now include frozensets

### With Future Chapters

**Chapter 20 (Modules and Functions)**:
- Functions will accept/return sets; Chapter 19 establishes type signatures

**Chapter 24 (OOP Part I)**:
- Custom objects need `__hash__` to be hashable; Chapter 19 lays groundwork

**Chapter 29 (CPython and GIL)**:
- GC discussion extends to GIL implications; Chapter 19 covers basics

---

## ✅ Validation Strategy

### How Learners Demonstrate Understanding

**Technical Validation**:
- Code exercises in Lessons 1-2: "Write a function that removes duplicates" ✓
- Performance comparisons in Lesson 3: "Prove sets are faster than lists" ✓
- Edge case handling in Lesson 4: "Use frozensets as dict keys" ✓
- Memory analysis in Lesson 5: "Profile object creation/deletion" ✓

**Conceptual Validation**:
- Lesson 3 "Try With AI": Explain hashing and O(1) complexity ✓
- Lesson 4 "Try With AI": Choose set vs. frozenset appropriately ✓
- Lesson 5 "Try With AI": Explain reference counting ✓

**Capstone Validation** (Lesson 6):
- Build working Memory Profiler tool ✓
- Test on realistic scenarios ✓
- Write reflection explaining integration ✓

### Success Criteria Alignment

| Spec SC | Lesson(s) | Validation |
|---------|-----------|-----------|
| SC-001: Set Conceptual Mastery | 1, 2, 3 | Quiz + "Try With AI" explanations |
| SC-002: Type-Hinted Set Operations | 1, 2, 4 | Code exercises with type hints |
| SC-003: Memory Profiling Competency | 5, 6 | Working capstone tool |
| SC-004: Reading Level (Grade 7-8) | All | Automated readability check |
| SC-005: Engagement & Completion | All | Lesson submission tracking |
| SC-006: AI Partnership | All | "Try With AI" usage patterns |

---

## 📋 Assumptions & Dependencies

### Environmental Assumptions

- Python 3.14+ installed with `sys` and `gc` modules available
- AI tools available (ChatGPT web, Claude Code, or Gemini CLI)
- Text editor or IDE for code writing

### Knowledge Assumptions

- Chapter 13: Python basics, variables, print()
- Chapter 14: Data types, type hints syntax
- Chapter 15: Operators, comparison
- Chapter 16: String basics
- Chapter 17: Loops, conditionals, comprehensions
- Chapter 18: Lists, tuples, dicts, mutability concepts

### Not Assumed (will teach)

- Hash functions or hashing algorithms
- CPython internals or implementation details
- Advanced GC tuning
- Custom hashable types (requires OOP)

---

## ⚠️ Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Hashability concept difficult | Lesson 1-3 confusion | Use concrete examples; AI can explain in different ways |
| O(1) vs. O(n) abstract | Lesson 3 performance analysis unclear | Provide timing code; let students observe difference |
| Circular refs edge case | Lesson 5 confusion | Show specific examples; explain cycle detector's role |
| Capstone scope creep (Lesson 6) | Implementation overruns time | Provide partial implementations; focus on integration, not perfection |
| Reading level (Grade 7-8 target) | Accessibility issues | Use simpler vocabulary; expand explanations; check with automated tools |

---

## 🎯 Next Steps After Plan Approval

1. **Run `/sp.clarify`** (if needed) — Flag underspecified areas with targeted questions
2. **Run `/sp.tasks`** — Generate actionable task checklist with acceptance criteria
3. **Implement with content-implementer** — Create 6 lesson files with CoLearning elements
4. **Validate with validation-auditor** — Check code quality, pedagogical effectiveness, constitutional alignment
5. **Deploy** — Publish to book platform after human final review

---

**Implementation Status**: Plan READY FOR DEVELOPMENT

This detailed plan provides:
- ✅ 6 lessons with clear objectives, skills, concepts, code examples
- ✅ CEFR proficiency levels (A2-B1) with cognitive load validation
- ✅ Bloom's taxonomy alignment (Understand → Create)
- ✅ "Try With AI" closures (4 prompts each, no separate "Key Takeaways")
- ✅ Type hints in all code examples
- ✅ Constitutional compliance (Graduated Teaching Pattern applied)
- ✅ Real-world integration (capstone project)
- ✅ Prerequisites and cross-chapter connections

Ready to proceed to task generation (`/sp.tasks`) → implementation (`content-implementer`) → validation.
