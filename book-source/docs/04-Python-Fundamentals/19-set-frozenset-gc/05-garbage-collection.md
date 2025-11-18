---
title: "Garbage Collection"
sidebar_position: 5
description: "Learn Python's automatic memory management through reference counting, circular references, and the gc module"
chapter: 19
lesson: 5
duration_minutes: 50
complexity: B1

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Understanding Reference Counting"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "2.1 - Information Literacy"
    measurable_at_this_level: "Student can explain how Python counts who's using an object and deletes it when count reaches zero"

  - name: "Memory Management in Python"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "3.4 - Problem-Solving"
    measurable_at_this_level: "Student can explain Python's automatic memory management through reference counting"

  - name: "Using gc Module for Analysis"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3.4 - Problem-Solving"
    measurable_at_this_level: "Student can use gc.collect(), gc.get_objects() to profile memory and understand object counts"

  - name: "Understanding Circular References"
    proficiency_level: "B1-B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "3.4 - Problem-Solving"
    measurable_at_this_level: "Student can explain circular references and how cycle detector prevents memory leaks"

# Learning objectives aligned with proficiency levels
learning_objectives:
  - objective: "Understand Python's reference counting as the primary garbage collection mechanism"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Code observation and explanation"

  - objective: "Recognize when objects are freed (refcount reaches zero) and observe immediate deletion"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Practical observation using sys.getrefcount()"

  - objective: "Understand circular references and when reference counting fails"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Code example with cycle detection"

  - objective: "Use gc module for basic memory analysis and profiling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Memory profiling exercise"

# Cognitive load tracking
cognitive_load:
  new_concepts: 9
  max_for_b1: 10
  assessment: "Within B1 limit: 1) Reference counting, 2) Refcount rules, 3) Immediate deletion, 4) Circular references, 5) Cycle detector, 6) gc module, 7) Generational GC, 8) Memory profiling, 9) Memory analysis"
  status: "✓ Compliant"

# Generation metadata
generation_metadata:
  generated_by: "content-implementer (Claude Haiku 4.5)"
  source_spec: "specs/001-part-4-chapter-19/spec.md (FR-024 to FR-029)"
  source_plan: "specs/001-part-4-chapter-19/plan.md (Lesson 5, lines 962-1244)"
  created: "2025-11-09"
  last_modified: "2025-11-09"
  git_author: "Claude Code"
  workflow: "specification-first SDD"
  version: "1.0.0"
---

# Garbage Collection

🚀 **What You'll Learn**: Python manages memory automatically. In this lesson, you'll understand *how*—through reference counting and garbage collection. You'll observe objects being freed, handle tricky circular references, and use the `gc` module to analyze memory like a professional. No manual memory management needed; just smart understanding of how Python cleans up after itself.

## 💬 Why This Matters

Memory management sounds abstract, but it's about **trust**. When you create 10,000 objects in a loop, Python automatically frees them when they're no longer needed. No crashes, no memory explosions, no cleanup code. But understanding how it works prevents subtle bugs and helps you write efficient long-running applications.

Think of Python as a responsible roommate: it cleans up your dishes (deletes objects) as soon as you're done using them. We're learning to see that process happen.

---

## Concept: Reference Counting

Python uses **reference counting** as its primary memory management mechanism. Every object Python creates has a counter: "How many things are currently using me?"

### How It Works

When you create an object, its reference count starts at 1. When you create another variable pointing to it, the count increases. When you delete a reference, it decreases. **When the count hits zero, Python immediately frees the memory.**

```python
import sys

# Create a list
my_list: list[int] = [1, 2, 3]
print(f"Initial refcount: {sys.getrefcount(my_list)}")  # 2 (variable + function arg)

# Create another reference
another_ref: list[int] = my_list
print(f"After assignment: {sys.getrefcount(my_list)}")  # 3

# Delete one reference
del my_list
print(f"After del my_list: {sys.getrefcount(another_ref)}")  # 2 (still accessible)

# Delete the last reference
del another_ref
# Now the list is freed automatically
print("Object freed when last reference deleted")
```

**Key insight**: `sys.getrefcount()` returns one extra because the function itself holds a reference while measuring.

### Why This Matters

Reference counting is **simple and immediate**. When your function ends, its local variables vanish, their refcounts decrease, and objects are freed instantly. No waiting for garbage collection to run—automatic cleanup as you go.

#### 💬 AI Colearning Prompt
> "When I create a list inside a function and the function returns, what happens to that list? Walk me through the reference counting process step by step."

---

## Concept: Circular References (The Problem)

Here's where reference counting breaks: **circular references**. Two objects pointing to each other.

```
Object A → Object B
Object B → Object A (circle!)
```

When both refcounts are above zero but nothing external points to them, reference counting can't free them. They're "orphaned" but not technically unreferenced.

### Example: Circular Reference

```python
import gc
import sys

class Node:
    def __init__(self, name: str) -> None:
        self.name: str = name
        self.next: Node | None = None

    def __del__(self) -> None:
        print(f"Node {self.name} freed")

# Create circular reference
node_a: Node = Node("A")
node_b: Node = Node("B")

node_a.next = node_b  # A points to B
node_b.next = node_a  # B points to A (circle!)

print("Created circular reference")
print(f"node_a refcount: {sys.getrefcount(node_a)}")  # 2
print(f"node_b refcount: {sys.getrefcount(node_b)}")  # 2

# Delete external references
del node_a
del node_b
print("Deleted variables, but objects NOT freed (circular!)")

# Without manual intervention, they'd stay in memory forever
```

**Output**: The `__del__` methods don't print. Objects are orphaned.

### The Solution: Garbage Collection

Python includes a **cycle detector** (separate from reference counting) that periodically finds and breaks circular references. This happens automatically unless you disable it.

```python
import gc

# ... (circular reference code above) ...

# Manual collection finds and frees circular references
collected: int = gc.collect()
print(f"Garbage collector freed {collected} objects")
# Output: "Node A freed" and "Node B freed"
```

#### 🎓 Expert Insight
> In AI-native development, you trust Python's automatic memory management but understand the mechanism. When you see "memory leak" in production, you know to check for circular references. Ask your AI: "How do I detect circular references in my code?" and it'll show you profiling techniques using the gc module.

---

## Concept: The `gc` Module

The `gc` module gives you control over garbage collection. In normal operation, you rarely need it—GC happens automatically. But understanding it helps you profile memory and debug issues.

### Basic Operations

```python
import gc

# Check if GC is enabled
print(f"GC enabled: {gc.isenabled()}")  # True by default

# Get current object counts (by generation)
counts: tuple[int, ...] = gc.get_count()
print(f"Objects in each generation: {counts}")  # (123, 5, 2)

# Manually trigger garbage collection
collected: int = gc.collect()
print(f"Freed {collected} objects")

# Get all tracked objects (memory profiling)
all_objects: list = gc.get_objects()
print(f"Total tracked objects: {len(all_objects)}")
```

### Memory Profiling with gc Module

```python
import gc

# Disable automatic GC for controlled testing
gc.disable()

try:
    # Count initial objects
    initial: int = len(gc.get_objects())
    print(f"Initial: {initial} objects")

    # Create lots of objects
    numbers: list[int] = list(range(100_000))
    strings: list[str] = [str(i) for i in range(100_000)]

    # Check new count
    after_creation: int = len(gc.get_objects())
    print(f"After creation: {after_creation} objects")
    print(f"Created: {after_creation - initial} objects")

    # Delete objects
    del numbers, strings

    # Check after deletion (reference counting frees them immediately)
    after_deletion: int = len(gc.get_objects())
    print(f"After deletion: {after_deletion} objects")

finally:
    # Always re-enable GC in production
    gc.enable()
```

✨ **Important**: Reference counting handles deletion in this example. `gc.collect()` is mainly for circular references, which didn't exist here.

### Generational Garbage Collection

Python uses **generational GC**: young objects (recently created) are checked frequently; old objects less frequently. This optimizes performance (most objects die young).

```python
import gc

# Get thresholds (when GC triggers automatically)
thresholds: tuple[int, int, int] = gc.get_threshold()
print(f"Thresholds (gen0, gen1, gen2): {thresholds}")
# Default: (700, 10, 10)
# Meaning: collect gen0 when 700+ new objects, gen1 when 10 gen0 collections, etc.

# Get collection statistics
stats: list = gc.get_stats()
print(f"GC stats: {stats}")
```

#### 🤝 Practice Exercise

> **Ask your AI**: "Create two objects that reference each other (a circular reference). Use sys.getrefcount() to show their reference counts. Then delete both variables and use gc.collect() to prove the cycle detector freed them."

**Expected Outcome:** You'll observe circular references in action and understand how Python's two-tier memory management (reference counting + cycle detection) works together.

---

## Code Examples in Action

### Example 1: Observing Reference Counting

**Spec Reference**: FR-024 (understand reference counting), FR-025 (observe deletion)

**Prompt Used**: "Create a Python script that shows reference counting in action. Create an object, assign it to another variable, delete references one by one, and show refcount at each step."

**Generated Code**:

```python
import sys

print("=" * 50)
print("Reference Counting Demonstration")
print("=" * 50)

# Create initial list
my_list: list[int] = [1, 2, 3, 4, 5]
initial_refcount: int = sys.getrefcount(my_list) - 1  # -1 for getrefcount arg
print(f"\n1. Created list: refcount = {initial_refcount}")

# Create second reference
another_ref: list[int] = my_list
ref2_count: int = sys.getrefcount(my_list) - 1
print(f"2. Created another_ref: refcount = {ref2_count}")

# Create third reference
third_ref: list[int] = my_list
ref3_count: int = sys.getrefcount(my_list) - 1
print(f"3. Created third_ref: refcount = {ref3_count}")

# Delete first reference
del my_list
remaining_count: int = sys.getrefcount(another_ref) - 1
print(f"4. Deleted my_list: refcount = {remaining_count}")

# Delete second reference
del another_ref
final_count: int = sys.getrefcount(third_ref) - 1
print(f"5. Deleted another_ref: refcount = {final_count}")

# Delete last reference
del third_ref
print(f"6. Deleted third_ref: object is freed")

# Real-world example: temporary variables
print("\n" + "=" * 50)
print("Real-World: Temporary Variables in Loops")
print("=" * 50)

for i in range(3):
    temp_list: list[int] = list(range(1000))
    print(f"Iteration {i}: created temp_list, refcount = {sys.getrefcount(temp_list) - 1}")
    # temp_list is freed automatically here
print("All temp lists freed as loop iterations ended")
```

**Validation Steps**:
1. Run script and observe refcount increasing with each assignment
2. Observe refcount decreasing with each `del`
3. Confirm objects freed immediately when refcount drops to zero
4. In loop: confirm temporary variables don't accumulate

### Example 2: Circular References and Cycle Detection

**Spec Reference**: FR-027 (cycle detection)

**Prompt Used**: "Create a Node class with circular references. Show how del doesn't free circular objects, then use gc.collect() to detect and break the cycle."

**Generated Code**:

```python
import gc
import sys

print("=" * 50)
print("Circular References & Cycle Detection")
print("=" * 50)

class Node:
    """A node that can reference another node"""

    def __init__(self, name: str) -> None:
        self.name: str = name
        self.next: Node | None = None
        print(f"✓ Created Node({name})")

    def __del__(self) -> None:
        print(f"✗ Node({self.name}) freed")

# Create two nodes
print("\n1. Creating nodes...")
node_a: Node = Node("A")
node_b: Node = Node("B")

# Create circular reference
print("\n2. Creating circular reference...")
node_a.next = node_b  # A → B
node_b.next = node_a  # B → A (circle!)
print("   A → B → A (circular!)")

print(f"\n3. Reference counts:")
print(f"   node_a refcount: {sys.getrefcount(node_a) - 1}")
print(f"   node_b refcount: {sys.getrefcount(node_b) - 1}")

# Delete external references
print("\n4. Deleting external references...")
del node_a
del node_b
print("   Objects NOT freed (still reference each other!)")

# Manual garbage collection
print("\n5. Running garbage collection...")
collected: int = gc.collect()
print(f"   Freed {collected} objects")
print("   (Node A and B __del__ printed above)")
```

**Validation Steps**:
1. Run and observe nodes are created
2. After `del`: __del__ is NOT called (circular reference prevents freeing)
3. After `gc.collect()`: __del__ IS called (cycle detector frees them)
4. Compare behavior with and without gc.collect()

### Example 3: Memory Profiling with gc.get_objects()

**Spec Reference**: FR-026 (use gc module for analysis), FR-028 (profile memory)

**Prompt Used**: "Write a memory profiler that counts objects before and after creating different data structures. Use gc.get_objects() to track total objects."

**Generated Code**:

```python
import gc
from typing import Any

print("=" * 60)
print("Memory Profiling: Object Counts")
print("=" * 60)

# Disable automatic GC for precise measurement
gc.disable()

try:
    # Baseline
    initial_count: int = len(gc.get_objects())
    print(f"\nBaseline: {initial_count} objects")

    # Create integers
    print("\n1. Creating 50,000 integers...")
    integers: list[int] = list(range(50_000))
    after_integers: int = len(gc.get_objects())
    print(f"   Objects: {after_integers} (+{after_integers - initial_count})")

    # Create strings
    print("\n2. Creating 50,000 strings...")
    strings: list[str] = [str(i) for i in range(50_000)]
    after_strings: int = len(gc.get_objects())
    print(f"   Objects: {after_strings} (+{after_strings - after_integers})")

    # Create sets
    print("\n3. Creating 10,000 sets...")
    sets: list[set[int]] = [{i, i+1, i+2} for i in range(10_000)]
    after_sets: int = len(gc.get_objects())
    print(f"   Objects: {after_sets} (+{after_sets - after_strings})")

    # Delete everything
    print("\n4. Deleting all structures...")
    del integers, strings, sets
    after_deletion: int = len(gc.get_objects())
    print(f"   Objects: {after_deletion} (-{after_strings - after_deletion})")

    # Manual collection (mostly for circular refs)
    print("\n5. Running garbage collection...")
    collected: int = gc.collect()
    print(f"   Freed {collected} objects (circular refs)")

finally:
    # Re-enable GC
    gc.enable()

print("\n" + "=" * 60)
print("Key Insight: Reference counting freed everything immediately.")
print("gc.collect() freed any circular references (none in this example).")
print("=" * 60)
```

**Validation Steps**:
1. Run and observe object counts increasing
2. After `del`: reference counting frees objects immediately (count drops)
3. `gc.collect()` shows minimal freed (no circular refs)
4. Compare object counts for different data structures

### Example 4: Reference Counting in Functions

**Spec Reference**: FR-025 (automatic deletion)

**Prompt Used**: "Show how reference counting works when objects are passed to functions. Track refcount before, during, and after function calls."

**Generated Code**:

```python
import sys

def process_data(data: list[int]) -> int:
    """Function that receives a list as argument"""
    # Inside function, data is referenced here
    print(f"   Inside function: refcount = {sys.getrefcount(data) - 1}")
    return sum(data)

print("=" * 50)
print("Reference Counting in Functions")
print("=" * 50)

# Create data outside function
my_data: list[int] = [1, 2, 3, 4, 5]
print(f"\n1. Before function call: refcount = {sys.getrefcount(my_data) - 1}")

# Call function
result: int = process_data(my_data)
print(f"2. After function call: refcount = {sys.getrefcount(my_data) - 1}")
# Function's reference released, but object still exists (my_data references it)

# Delete reference
del my_data
print(f"3. After del my_data: object freed")

# Practical insight
print("\n" + "=" * 50)
print("Practical Pattern: Automatic Cleanup in Loops")
print("=" * 50)

for i in range(5):
    temp_data: list[int] = [j for j in range(10_000)]
    processed: int = sum(temp_data)
    print(f"Iteration {i}: processed {processed} (temp freed automatically)")

print("\nAll temporary lists freed as each iteration ended")
print("No memory accumulation despite creating 5 large lists")
```

**Validation Steps**:
1. Run and observe refcount during function call (increases)
2. After function returns: refcount decreases (reference released)
3. In loop: temporary variables freed each iteration
4. Verify memory doesn't accumulate

### Example 5: Generational Garbage Collection

**Spec Reference**: FR-029 (understand when GC runs)

**Prompt Used**: "Demonstrate Python's generational garbage collection. Show thresholds, collection counts, and how to manually trigger collection at different generations."

**Generated Code**:

```python
import gc

print("=" * 60)
print("Generational Garbage Collection")
print("=" * 60)

# Check if GC is enabled
print(f"\nGC enabled: {gc.isenabled()}")

# Get thresholds
thresholds: tuple[int, int, int] = gc.get_threshold()
print(f"\nGC Thresholds (generation 0, 1, 2): {thresholds}")
print("  Default (700, 10, 10) means:")
print("    - Collect gen0 when 700+ new objects created")
print("    - Collect gen1 when gen0 collected 10 times")
print("    - Collect gen2 when gen1 collected 10 times")

# Get current collection counts
counts: tuple[int, ...] = gc.get_count()
print(f"\nCurrent object counts per generation: {counts}")
print(f"  Gen0 (young): {counts[0]} objects (collected frequently)")
print(f"  Gen1 (middle): {counts[1]} collections")
print(f"  Gen2 (old): {counts[2]} collections")

# Trigger manual collection
print("\nManually triggering garbage collection...")
collected: int = gc.collect()
print(f"Freed {collected} objects")

# Check counts after collection
counts_after: tuple[int, ...] = gc.get_count()
print(f"After collection: {counts_after}")

# Optional: Adjust thresholds (rarely needed)
print("\n" + "=" * 60)
print("Advanced: Adjusting GC Thresholds")
print("=" * 60)
print("Original:", gc.get_threshold())
# gc.set_threshold(1000, 15, 15)  # Less frequent, faster but more memory
# print("New:", gc.get_threshold())
# (Not recommended unless profiling shows GC is bottleneck)
```

**Validation Steps**:
1. Run and observe default thresholds (700, 10, 10)
2. Observe collection counts change
3. After `gc.collect()`: counts reset
4. Understand generational strategy (young objects collected frequently)

---

## Practice Exercises

### Exercise 1: Track Reference Counting Manually

Create a script that:
1. Creates a list with three references
2. Prints refcount after each reference creation
3. Deletes references one by one
4. Shows refcount at each step
5. Confirms object is freed when refcount reaches zero

**What You're Learning**: Direct observation of reference counting in action.

### Exercise 2: Break a Circular Reference

Write a `Person` class where:
1. Each person has a `best_friend` attribute
2. Create two people with circular friend relationship
3. Delete both people
4. Observe they're NOT freed (circular reference)
5. Use `gc.collect()` to free them
6. Confirm they're freed

**What You're Learning**: Identifying and resolving circular references.

### Exercise 3: Profile Object Creation and Deletion

Use `gc.get_objects()` to:
1. Count objects at baseline
2. Create a dictionary with 10,000 entries
3. Count objects (how many were created?)
4. Delete the dictionary
5. Count objects (how many were freed?)
6. Verify reference counting handles deletion

**What You're Learning**: Memory profiling using `gc` module.

### Exercise 4: Compare Memory Usage Across Data Structures

Create a function `analyze_memory(structure_type: str) -> None:` that:
1. Creates 50,000 items in the specified structure (list, set, tuple, dict)
2. Uses `gc.get_objects()` to count objects before/after
3. Prints object counts for each structure
4. Deletes the structure and recounts
5. Shows how many objects each structure creates

**What You're Learning**: Real-world memory profiling patterns.

---

## Try With AI

🤖 **Using Your AI Companion** (ChatGPT web or Claude Code from previous lessons)

### Prompt 1: Understand Reference Counting (Bloom's: Understand)

**Tell your AI:**
```
How does Python know when to delete an object?
What is reference counting and why does Python use it instead of
other garbage collection methods?
```

**Expected Outcome**: AI explains refcount as "counting who's using this object." You understand:
- Each object has a counter
- Counter increases when new variable references it
- Counter decreases when reference is deleted
- Object freed when counter reaches zero

**Follow-up**: "Is this automatic or do I need to do something in my code?"

---

### Prompt 2: Circular References Deep Dive (Bloom's: Understand)

**Tell your AI:**
```
What's a circular reference in Python?
Show me an example where two objects reference each other and
explain why this causes problems for reference counting.
```

**Expected Outcome**: AI shows code where Object A references B and B references A, then explains:
- Neither object's refcount ever reaches zero
- Reference counting can't free them
- Python's cycle detector is needed as backup
- Most code doesn't have circular references

**Follow-up**: "Does Python fix this automatically or do I need to call gc.collect()?"

---

### Prompt 3: Memory Profiling Challenge (Bloom's: Apply)

**Tell your AI:**
```
Write Python code that:
1. Creates 100,000 objects (lists, strings, or sets)
2. Shows how many objects are in memory before and after creation
3. Deletes all objects
4. Shows how many objects are freed
5. Explains what reference counting did
```

**Expected Outcome**: AI generates profiling code using `gc.get_objects()` and `sys.getrefcount()`. You learn to:
- Use `gc.get_objects()` to count all tracked objects
- Create large object collections
- Observe immediate deletion via reference counting
- Confirm memory is freed

**Safety Note**: "Memory usage varies by platform; focus on the count of objects, not exact bytes."

---

### Prompt 4: Why Do We Need Two Memory Systems? (Bloom's: Evaluate)

**Tell your AI:**
```
If reference counting deletes objects immediately, why does Python
have a separate garbage collector? When would we need both?
```

**Expected Outcome**: AI explains the division of labor:
- **Reference counting**: Fast, immediate (95% of objects)
- **Cycle detector**: Periodic, catches circular references (5% of objects)
- Professional insight: Most code doesn't need manual GC intervention

**Follow-up**: "Should I manually call gc.collect() in my programs?"

---

✨ **After completing these prompts**, you understand Python's memory management at a professional level. You know *why* Python does what it does and can explain it to others—that's deep understanding.

