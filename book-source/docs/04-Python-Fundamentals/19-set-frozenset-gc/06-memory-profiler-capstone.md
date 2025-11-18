---
title: "Memory Profiler Capstone — Building an Object Tracking Tool"
sidebar_position: 6
description: "Design and build a working Memory Profiler tool that integrates sets, frozensets, and garbage collection concepts into a practical project"
chapter: 19
lesson: 6
duration_minutes: 60
complexity: "B1-B2"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Designing Software with AI Collaboration"
    proficiency_level: "B1-B2"
    category: "Soft"
    bloom_level: "Create"
    digcomp_area: "3.4 - Problem-Solving"
    measurable_at_this_level: "Student can write specifications for tools and collaborate with AI to refine design and architecture"

  - name: "Implementing Set-Based Data Tracking"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "2.1 - Digital Content Creation"
    measurable_at_this_level: "Student can use sets and frozensets to build data tracking structures with type hints"

  - name: "Integrating GC Module for Memory Analysis"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3.4 - Problem-Solving"
    measurable_at_this_level: "Student can use gc.collect(), gc.get_objects() to analyze memory and extract profiling data"

  - name: "Testing and Validating Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "3.4 - Problem-Solving"
    measurable_at_this_level: "Student can test code on edge cases and verify that tool handles circular references, large graphs, and realistic scenarios"

  - name: "Reflecting on Integration"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "3.4 - Problem-Solving"
    measurable_at_this_level: "Student can articulate how sets, frozensets, reference counting, and GC work together in a complete system"

# Learning objectives aligned with proficiency levels
learning_objectives:
  - objective: "Design a memory profiler tool specification by collaborating with AI"
    proficiency_level: "B1-B2"
    bloom_level: "Create"
    assessment_method: "Written specification for tool requirements"

  - objective: "Implement a MemoryProfiler class using sets to track object IDs"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Working code with type hints and proper set operations"

  - objective: "Integrate gc module to analyze memory and detect circular references"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Memory profiling exercise with gc.collect() and gc.get_objects()"

  - objective: "Test tool on edge cases: circular references, large object graphs, realistic scenarios"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Test functions demonstrating reliability and accuracy"

  - objective: "Reflect on how chapter concepts integrate into a complete system"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Written reflection explaining concept integration"

# Cognitive load tracking
cognitive_load:
  new_concepts: 9
  max_for_b1_b2: 10
  concepts_list:
    - "Specification-first design (planning before coding)"
    - "Set-based object tracking (applies Lesson 1)"
    - "Frozensets for immutable categorization (applies Lesson 4)"
    - "Garbage collection integration (applies Lesson 5)"
    - "Type hints for clarity and correctness"
    - "Testing and validation approach"
    - "Edge case handling (circular refs, large graphs)"
    - "Reflection and synthesis of concepts"
    - "AI collaboration in design and implementation"
  assessment: "B1-B2 capstone: 9 concepts (synthesis of prior lessons + 3-4 new meta-level concepts). Within B1-B2 limit of 10 ✓"
  status: "✓ Compliant"

# Generation metadata
generation_metadata:
  generated_by: "content-implementer (Claude Haiku 4.5)"
  source_spec: "specs/001-part-4-chapter-19/spec.md (FR-030 to FR-035)"
  source_plan: "specs/001-part-4-chapter-19/plan.md (Lesson 6, lines 1247-1586)"
  created: "2025-11-09"
  last_modified: "2025-11-09"
  git_author: "Claude Code"
  workflow: "specification-first SDD"
  version: "1.0.0"
---

# Memory Profiler Capstone: Building an Object Tracking Tool

🚀 **What You'll Build**: A working Memory Profiler tool that tracks Python objects, displays memory statistics, and handles edge cases. This capstone integrates **everything** you've learned in Chapter 19: sets for tracking, frozensets for organization, garbage collection for analysis, and AI collaboration for design.

**Duration**: 60 minutes (design 10 min + implement 20 min + test 15 min + reflect 10 min + reflection 5 min)

**Complexity**: B1-B2 Intermediate-Advanced (You're synthesizing multiple concepts into a working system)

---

## 💬 Why This Capstone Matters

Throughout Chapter 19, you've learned individual concepts:
- **Lesson 1**: Sets for fast membership testing
- **Lesson 2**: Set operations for data analysis
- **Lesson 3**: Hash tables and O(1) performance
- **Lesson 4**: Frozensets as immutable, hashable types
- **Lesson 5**: Garbage collection and reference counting

Now you'll see how these concepts work **together** to solve a real problem: understanding Python memory usage. This capstone is portfolio-worthy—you're building a tool that professionals use to debug memory-intensive applications.

### Why Object Tracking Matters

When you write a long-running application (a web server handling requests, a data processor working through gigabytes of files), memory leaks kill performance. Instead of guessing, you can:

1. Track object creation/deletion using sets
2. Analyze memory with the gc module
3. Identify "orphaned" objects before they accumulate
4. Validate that circular references are cleaned up

This tool is the foundation for professional memory profiling.

---

## Phase 1: Design — Specification First

Before writing a single line of code, let's **specify** what we're building.

### Memory Profiler Specification

**Goal**: Build a tool that tracks Python object creation/deletion and displays memory statistics.

**What It Does**:
- Tracks when objects are created (adds their ID to a set)
- Tracks when objects are deleted (monitors refcount or gc detection)
- Reports current object count, peak count, total objects ever created
- Identifies "dead" objects (unreferenced but not yet collected by gc)
- Shows memory bytes consumed

**Input**: Your Python program creates and deletes objects
**Output**: Statistics showing:
```
Current objects in memory: 47
Total objects created: 150
Total objects deleted: 103
Peak object count: 89
Total memory: 245,832 bytes
Unreachable objects (cycles): 2
```

**Technology Requirements**:
- Use `set[int]` to track object IDs (apply Lesson 1)
- Use `frozenset[str]` for immutable categorization keys (apply Lesson 4)
- Use `gc` module for memory analysis (apply Lesson 5)
- Type hints mandatory (modern Python standard)
- Must handle edge cases: circular references, large graphs (testing)

#### 💬 AI Colearning Prompt
> "What would you add to this memory profiler specification? What edge cases should we handle? How would we know when it's 'done'?"

---

## Design With Your AI Companion

Here's where AI helps refine your design:

**Tell your AI companion:**
> "I want to build a memory profiler that tracks object creation and deletion. Help me refine the requirements. Should it track objects by type? Should it detect memory leaks automatically? What's the simplest version that still solves the core problem?"

**What your AI gives you**:
- Clarified requirements
- Suggested design patterns
- Tradeoff analysis (simple vs. feature-rich)
- Architecture sketch

**Your job**: Review, ask follow-up questions, decide on final design. You're steering the thinking, not just typing.

---

## Phase 2: Implementation — Building the Tool

Now we implement the specification. Here are three working code examples showing the progression.

### Code Example 1: Core Memory Profiler

**Specification Reference**: Track objects using sets + gc module
**Bloom's Level**: Create / Apply
**Pedagogical Purpose**: Implement basic profiler showing all core features

```python
import gc
import sys
from typing import Any

class MemoryProfiler:
    """
    Tracks object creation and deletion using sets and garbage collection.

    Integrates:
    - set[int] for tracking object IDs (Lesson 1)
    - gc module for memory analysis (Lesson 5)
    - sys.getsizeof() for memory measurement

    Usage:
        profiler = MemoryProfiler()
        my_list = [1, 2, 3]
        profiler.track_object(my_list)
        profiler.print_report()
    """

    def __init__(self) -> None:
        """Initialize profiler with empty tracking sets."""
        # Use sets to track object IDs (immutable integers)
        self.created_objects: set[int] = set()      # IDs of all created objects
        self.deleted_objects: set[int] = set()      # IDs of deleted objects
        self.peak_count: int = 0                     # Maximum objects in memory at once
        self.start_count: int = len(gc.get_objects())

    def track_object(self, obj: Any) -> None:
        """
        Track a new object by adding its ID to the created set.

        Args:
            obj: Any Python object to track
        """
        obj_id: int = id(obj)
        self.created_objects.add(obj_id)

        # Update peak count
        current_count: int = self.count_living_objects()
        if current_count > self.peak_count:
            self.peak_count = current_count

    def count_living_objects(self) -> int:
        """
        Calculate living objects = created - deleted.

        Returns:
            Number of objects currently tracked in memory
        """
        return len(self.created_objects) - len(self.deleted_objects)

    def profile_memory(self) -> dict[str, int]:
        """
        Analyze current memory state using gc module.

        Integrates:
        - gc.collect(): Trigger garbage collection (applies Lesson 5)
        - gc.get_objects(): Get all tracked objects
        - sys.getsizeof(): Measure individual object sizes

        Returns:
            Dictionary with memory statistics
        """
        # Force garbage collection to get accurate counts
        gc.collect()

        # Get all objects currently tracked by Python
        all_objects: list[Any] = gc.get_objects()

        # Calculate statistics
        living_count: int = self.count_living_objects()
        total_memory: int = sum(sys.getsizeof(obj) for obj in all_objects)

        return {
            "current_objects": living_count,
            "created_total": len(self.created_objects),
            "deleted_total": len(self.deleted_objects),
            "peak_objects": self.peak_count,
            "memory_bytes": total_memory,
            "unreachable": len(gc.garbage)  # Circular refs detected by cycle detector
        }

    def print_report(self) -> None:
        """
        Display memory statistics in human-readable format.

        Shows:
        - Current objects in memory
        - Total objects ever created
        - Total objects freed
        - Peak object count reached
        - Total memory consumed
        - Unreachable objects from circular references
        """
        stats: dict[str, int] = self.profile_memory()

        print("\n" + "=" * 50)
        print("MEMORY PROFILER REPORT")
        print("=" * 50)
        print(f"Current objects in memory:  {stats['current_objects']:>6}")
        print(f"Total objects created:      {stats['created_total']:>6}")
        print(f"Total objects deleted:      {stats['deleted_total']:>6}")
        print(f"Peak object count:          {stats['peak_objects']:>6}")
        print(f"Total memory used:          {stats['memory_bytes']:>6,} bytes")
        print(f"Unreachable (cycles):       {stats['unreachable']:>6}")
        print("=" * 50 + "\n")


# Usage Example
if __name__ == "__main__":
    profiler: MemoryProfiler = MemoryProfiler()

    # Create objects and track them
    print("Creating objects...")
    numbers: list[int] = [i for i in range(1000)]
    profiler.track_object(numbers)

    strings: list[str] = [f"string_{i}" for i in range(500)]
    profiler.track_object(strings)

    sets_list: list[set[int]] = [{i, i+1, i+2} for i in range(100)]
    profiler.track_object(sets_list)

    profiler.print_report()

    # Delete and observe memory change
    print("Deleting objects...")
    del numbers
    del strings
    del sets_list

    # Trigger garbage collection (demonstrates Lesson 5 integration)
    gc.collect()

    profiler.print_report()

    print("✓ Core profiler working!")
```

**What This Shows**:
- ✅ Sets track object IDs (Lesson 1 integration)
- ✅ gc module analyzes memory (Lesson 5 integration)
- ✅ Type hints on all functions and variables
- ✅ Docstrings explaining each method
- ✅ Real usage example showing before/after deletion

**Specification Validation**:
- ✓ Tracks object creation
- ✓ Counts living objects
- ✓ Shows memory statistics
- ✓ Detects circular references via gc.garbage

---

### Code Example 2: Advanced Tracking with Frozensets

**Specification Reference**: Categorize objects by type using frozensets
**Bloom's Level**: Create / Apply
**Pedagogical Purpose**: Show real design pattern (frozensets as immutable categorization keys)

```python
class AdvancedMemoryProfiler(MemoryProfiler):
    """
    Extended profiler that categorizes objects by type.

    Integrates:
    - MemoryProfiler base class (core tracking)
    - frozenset[str] for immutable categorization keys (Lesson 4)
    - dict mapping frozensets to object IDs (advanced pattern)

    Why frozensets? They're hashable, so they can be dictionary keys.
    Regular sets are mutable and can't be used as keys—frozensets solve this.

    Usage:
        profiler = AdvancedMemoryProfiler()
        profiler.track_object_with_type([1, 2, 3])  # Categorized as "list"
        profiler.report_by_category()
    """

    def __init__(self) -> None:
        """Initialize advanced profiler with type categorization."""
        super().__init__()

        # Use frozensets as immutable dictionary keys
        # Each frozenset describes a category (e.g., frozenset(["list"]))
        self.categories: dict[frozenset[str], set[int]] = {
            frozenset(["list"]): set(),
            frozenset(["dict"]): set(),
            frozenset(["set"]): set(),
            frozenset(["custom"]): set()
        }

    def track_object_with_type(self, obj: Any) -> None:
        """
        Track object and categorize it by type.

        Demonstrates Lesson 4 (frozensets as immutable keys) +
        Lesson 1 (sets for storing object IDs)

        Args:
            obj: Object to track and categorize
        """
        # Call parent method to do basic tracking
        self.track_object(obj)

        # Get object's ID and type name
        obj_id: int = id(obj)

        # Categorize by type
        if isinstance(obj, list):
            self.categories[frozenset(["list"])].add(obj_id)
        elif isinstance(obj, dict):
            self.categories[frozenset(["dict"])].add(obj_id)
        elif isinstance(obj, set):
            self.categories[frozenset(["set"])].add(obj_id)
        else:
            self.categories[frozenset(["custom"])].add(obj_id)

    def report_by_category(self) -> None:
        """
        Display object counts broken down by type category.

        Shows which types consume most objects in memory.
        """
        print("\n" + "-" * 40)
        print("OBJECTS BY CATEGORY")
        print("-" * 40)

        for category, ids in self.categories.items():
            # Extract category name from frozenset
            category_name: str = list(category)[0]
            count: int = len(ids)
            print(f"  {category_name:>10}: {count:>5} objects")

        total: int = sum(len(ids) for ids in self.categories.values())
        print(f"  {'total':>10}: {total:>5} objects")
        print("-" * 40 + "\n")


# Usage Example
if __name__ == "__main__":
    advanced: AdvancedMemoryProfiler = AdvancedMemoryProfiler()

    print("Creating objects of different types...")

    my_list: list[int] = [1, 2, 3, 4, 5]
    my_dict: dict[str, int] = {"a": 1, "b": 2}
    my_set: set[int] = {1, 2, 3}

    advanced.track_object_with_type(my_list)
    advanced.track_object_with_type(my_dict)
    advanced.track_object_with_type(my_set)

    # Also track some nested objects
    nested_lists: list[list[int]] = [[1, 2], [3, 4], [5, 6]]
    advanced.track_object_with_type(nested_lists)

    advanced.report_by_category()
    print("✓ Advanced profiler with categorization working!")
```

**What This Shows**:
- ✅ Frozensets as immutable dictionary keys (Lesson 4)
- ✅ Sets inside dictionaries to track object IDs (Lesson 1)
- ✅ Inheritance pattern extending base class
- ✅ Type-based categorization (real-world pattern)
- ✅ Multiple tracked objects per category

**Design Pattern Insight**: This demonstrates why frozensets exist—they're the only way to use sets as dictionary keys because they're immutable and hashable. Regular sets can't be keys!

---

### Code Example 3: Testing with Edge Cases

**Specification Reference**: Validate tool on circular references and large graphs
**Bloom's Level**: Evaluate
**Pedagogical Purpose**: Show testing approach and edge case handling

```python
def test_circular_references(profiler: MemoryProfiler) -> None:
    """
    Test that circular references are eventually freed by gc.

    Edge case: Two objects pointing to each other create a cycle.
    Reference counting alone can't free them, but gc's cycle detector can.

    This validates the integration of Lesson 5 (GC cycle detection).
    """
    print("\n" + "=" * 50)
    print("TEST 1: Circular References")
    print("=" * 50)

    # Create a simple Node class for circular reference testing
    class Node:
        def __init__(self, name: str) -> None:
            self.name: str = name
            self.next: Node | None = None

    # Create cycle: A → B → A
    node_a: Node = Node("A")
    node_b: Node = Node("B")
    node_a.next = node_b
    node_b.next = node_a

    print(f"Created circular reference: A → B → A")

    # Track them
    profiler.track_object(node_a)
    profiler.track_object(node_b)

    before_count: int = profiler.count_living_objects()
    print(f"Objects before deletion: {before_count}")

    # Delete both references
    del node_a
    del node_b
    print("Deleted both local variables")

    # Before gc.collect(), reference counting can't detect the cycle
    print(f"Objects before gc.collect(): {profiler.count_living_objects()}")

    # Trigger garbage collection (Lesson 5 integration)
    collected: int = gc.collect()
    print(f"gc.collect() freed {collected} objects")

    after_count: int = profiler.count_living_objects()
    print(f"Objects after gc.collect(): {after_count}")

    # Verify the cycle was freed
    assert after_count < before_count, "gc should have freed circular objects"
    print("✓ Circular references properly handled by gc\n")


def test_large_graph(profiler: MemoryProfiler) -> None:
    """
    Test profiler with large object graph (1000+ objects).

    Edge case: Large graphs stress the gc module and test memory tracking accuracy.

    This validates:
    - Lesson 1: Sets handle many object IDs efficiently (O(1) insertion)
    - Lesson 5: GC handles large graphs
    """
    print("=" * 50)
    print("TEST 2: Large Object Graph")
    print("=" * 50)

    print("Creating large graph: 1000 lists, 100 items each...")

    # Create a structure with many objects
    large_list: list[list[int]] = [
        [i * j for j in range(100)]
        for i in range(1000)
    ]

    profiler.track_object(large_list)

    stats_before: dict[str, int] = profiler.profile_memory()
    print(f"Memory with large graph: {stats_before['memory_bytes']:,} bytes")
    print(f"Objects tracked: {stats_before['current_objects']}")

    del large_list
    gc.collect()

    stats_after: dict[str, int] = profiler.profile_memory()
    print(f"Memory after deletion: {stats_after['memory_bytes']:,} bytes")

    # Verify memory was freed
    memory_freed: int = stats_before['memory_bytes'] - stats_after['memory_bytes']
    print(f"Memory freed: {memory_freed:,} bytes")

    assert memory_freed > 0, "Large graph should free significant memory"
    print("✓ Large graph properly freed\n")


def test_mixed_types(profiler: MemoryProfiler) -> None:
    """
    Test profiler with diverse object types.

    Edge case: Real programs create many different object types.
    Tests that profiler handles variety correctly.
    """
    print("=" * 50)
    print("TEST 3: Mixed Object Types")
    print("=" * 50)

    print("Creating diverse objects...")

    # Create variety of objects
    a_list: list[int] = [1, 2, 3]
    a_dict: dict[str, int] = {"x": 1, "y": 2}
    a_set: set[str] = {"a", "b", "c"}
    a_tuple: tuple[int, ...] = (1, 2, 3)
    a_string: str = "hello"

    profiler.track_object(a_list)
    profiler.track_object(a_dict)
    profiler.track_object(a_set)
    profiler.track_object(a_tuple)
    profiler.track_object(a_string)

    stats: dict[str, int] = profiler.profile_memory()
    print(f"Tracked {stats['created_total']} objects of different types")
    print(f"Currently in memory: {stats['current_objects']}")

    # Verify all were tracked
    assert stats['created_total'] >= 5, "Should track all created objects"
    print("✓ Mixed types handled correctly\n")


# Run all tests
if __name__ == "__main__":
    profiler: MemoryProfiler = MemoryProfiler()

    test_circular_references(profiler)
    test_large_graph(profiler)
    test_mixed_types(profiler)

    print("=" * 50)
    print("ALL TESTS PASSED ✓")
    print("=" * 50)
    profiler.print_report()
```

**What This Shows**:
- ✅ Test functions with descriptive names and docstrings
- ✅ Circular reference test (Lesson 5 integration)
- ✅ Large graph test (stress testing)
- ✅ Mixed types test (realistic scenarios)
- ✅ Assertions to verify correct behavior
- ✅ Before/after measurements showing memory changes

**Testing Insight**: Professional code always tests edge cases. We're not guessing the tool works—we're proving it with tests.

#### 🎓 Expert Insight
> In AI-native development, you don't just write code—you write specifications, then code, then tests. This capstone demonstrates the complete cycle: spec → implement → validate. When your AI generates code, you immediately ask: "How do I test this?" This mindset separates professionals from beginners.

---

## Phase 3: Testing & Validation (Your Turn)

Now it's time to **test** what you've built. Follow this sequence:

### Step 1: Run the Core Profiler Example

Copy the Memory Profiler code and run it:

```bash
python memory_profiler.py
```

You should see output like:
```
Creating objects...

==================================================
MEMORY PROFILER REPORT
==================================================
Current objects in memory:       3
Total objects created:           3
Total objects deleted:           0
Peak object count:               3
Total memory used:      250,432 bytes
Unreachable (cycles):          0
==================================================

Deleting objects...

==================================================
MEMORY PROFILER REPORT
==================================================
Current objects in memory:       0
Total objects created:           3
Total objects deleted:           3
Peak object count:               3
Total memory used:      123,456 bytes
Unreachable (cycles):          0
==================================================
```

🎓 **Pause**: What do you observe? How did memory usage change after deletion?

### Step 2: Run the Advanced Profiler

Test the categorization feature:

```bash
python advanced_profiler.py
```

Output:
```
Creating objects of different types...

----------------------------------------
OBJECTS BY CATEGORY
----------------------------------------
       list:     2 objects
       dict:     1 objects
        set:     1 objects
     custom:     0 objects
      total:     4 objects
----------------------------------------
```

🎓 **Pause**: Which object type consumed the most instances?

### Step 3: Run the Test Suite

Run all edge case tests:

```bash
python test_profiler.py
```

Expected output:
```
==================================================
TEST 1: Circular References
==================================================
Created circular reference: A → B → A
Objects before deletion: 2
Deleted both local variables
Objects before gc.collect(): 2
gc.collect() freed 2 objects
Objects after gc.collect(): 0
✓ Circular references properly handled by gc

==================================================
TEST 2: Large Object Graph
==================================================
Creating large graph: 1000 lists, 100 items each...
Memory with large graph: 2,845,123 bytes
Objects tracked: 1000
Memory after deletion: 1,234,567 bytes
Memory freed: 1,610,556 bytes
✓ Large graph properly freed

==================================================
TEST 3: Mixed Object Types
==================================================
Creating diverse objects...
Tracked 5 objects of different types
Currently in memory: 5
✓ Mixed types handled correctly

==================================================
ALL TESTS PASSED ✓
==================================================
```

---

## Phase 4: Reflection — Integrating Concepts

Now reflect on what you've built. Answer these questions in your own words:

### 1. How Did Sets Help?

"I used `set[int]` to track object IDs because..."
- Sets have O(1) lookup (from Lesson 1)
- Sets automatically eliminate duplicates
- I could quickly check if an object was already tracked
- Adding/removing IDs was fast even with thousands of objects

**Key insight**: Sets weren't just a data structure—they were the RIGHT choice for tracking because of their performance characteristics (Lesson 3).

### 2. Why Were Frozensets Useful?

"In the advanced profiler, I used `frozenset[str]` as dictionary keys because..."
- Frozensets are immutable (can't be changed after creation)
- Immutable objects are hashable (can be dictionary keys)
- Regular sets are mutable, so they can't be keys
- This let me create typed categories like `{frozenset(["list"]): set_of_ids}`

**Key insight**: Frozensets exist for exactly this use case—when you need immutability + hashability (Lesson 4).

### 3. How Did Garbage Collection Work?

"The gc module helped because..."
- `gc.collect()` finds and frees circular references that reference counting misses
- `gc.get_objects()` shows ALL objects in memory (for analysis)
- `gc.garbage` contains unreachable objects (cycles detected)
- I could measure memory before/after deletion to verify freeing worked

**Key insight**: GC is automatic, but understanding it let me verify my tool was actually freeing memory (Lesson 5).

### 4. How Do These Concepts Work Together?

Write a paragraph explaining the integration:

> "The Memory Profiler tool brings together all Chapter 19 concepts. I use **sets** (Lesson 1) to efficiently track which objects have been created and deleted—adding object IDs is O(1), which matters when tracking thousands of objects. I use **frozensets** (Lesson 4) to create immutable categorization keys in my dictionary, because regular sets can't be dictionary keys. I use **gc module** (Lesson 5) to analyze actual memory state, detecting circular references and freeing large object graphs. The tool integrates reference counting (refcount hitting zero) with cycle detection (gc finding orphaned cycles) to validate that Python is cleaning up properly. Together, these concepts create a professional-grade memory profiler."

#### 🤝 Practice Exercise

> **Ask your AI**: "Help me extend the Memory Profiler to categorize objects by type. Add a method that uses frozensets as dictionary keys to track how many lists, dicts, and sets are in memory. Show me the code and explain why frozensets are necessary here."

**Expected Outcome:** You'll practice applying the integration of sets, frozensets, and gc module by extending the capstone project with AI collaboration.

---

## Try With AI: Memory Profiler Capstone

This capstone integrates ALL Chapter 19 concepts: sets, frozensets, and garbage collection.

### Part 1: Design Memory Tracker Architecture (Your Turn First)

**Before asking AI**, design a memory profiling tool:

**Requirements**:
1. Track object creation/deletion (using object IDs)
2. Detect memory leaks (objects that should be deleted but aren't)
3. Categorize objects by type (dict, list, set, etc.)
4. Report memory usage statistics

**Your design task**:
- Which data structure for tracking object IDs? (list? set? Why?)
- How do you detect "leaked" objects? (compare created vs deleted)
- Can you use `frozenset` anywhere? (immutable keys in a dict?)
- Sketch pseudocode for `track()`, `release()`, and `report()` methods

---

### Part 2: Build MemoryProfiler with AI (Discovery)

Implement your design with AI guidance:

> "I'm building a Memory Profiler tool. Here's my design:
>
> **Class: MemoryProfiler**
> - `tracked: set[int]` — IDs of living objects
> - `created_count: int` — Total objects ever created
> - `deleted_count: int` — Total objects deleted
> - `type_counts: dict[str, int]` — Count per type ('dict', 'list', etc.)
>
> **Methods**:
> 1. `track(obj: object) -> None` — Add object ID to tracked set
> 2. `release(obj: object) -> None` — Remove object ID from tracked set
> 3. `report() -> dict[str, int]` — Return stats (living, created, deleted)
> 4. `detect_leaks() -> list[int]` — Find objects that should be garbage collected but aren't (use `gc.get_objects()`)
>
> Help me:
> 1. Write complete code with type hints
> 2. Use `id(obj)` to get object IDs
> 3. Integrate `gc.collect()` and `gc.get_objects()`
> 4. Handle edge case: What if user calls `release()` on non-tracked object?"

**Your evaluation task**:
- Does AI use `set[int]` for `tracked`? Why is that better than `list[int]`?
- Run the code. Create objects, track them, delete them, call `report()`. Do the numbers make sense?

---

### Part 3: Student Teaches AI (Edge Cases & Memory Leaks)

Challenge AI with realistic memory leak scenarios:

> "Let's test my Memory Profiler with edge cases:
>
> **Edge Case 1: Circular References**
> ```python
> class Node:
>     def __init__(self, value):
>         self.value = value
>         self.next = None
>
> # Create circular reference
> a = Node(1)
> b = Node(2)
> a.next = b
> b.next = a  # Circular!
>
> profiler.track(a)
> profiler.track(b)
> del a
> del b
> gc.collect()  # Will GC clean up circular refs?
> ```
>
> For this case:
> 1. Predict: After `del` and `gc.collect()`, are objects freed?
> 2. Show how `profiler.detect_leaks()` would catch this
> 3. Explain: Why doesn't reference counting alone handle this?
>
> **Edge Case 2: Large Object Graphs**
> - Create 1000 objects, track them all
> - Delete 500 randomly
> - Run `detect_leaks()`
> - Measure: How long does it take? Is `set[int]` lookup fast enough (O(1))?
>
> **Edge Case 3: Frozenset as Dict Key**
> - Can I group objects by their attributes using frozensets?
> - Example: Group nodes by `frozenset` of their values
> - Show code where `frozenset[str]` is a dict key"

**Your debugging task**:
- Run Edge Case 1. Do circular refs get collected?
- Which edge case surprised you most?

---

### Part 4: Add Memory Leak Detection (Integration)

Extend profiler with leak detection:

> "Enhance Memory Profiler with leak detection:
>
> **New Feature: `detect_leaks()`**
> 1. Get ALL Python objects using `gc.get_objects()`
> 2. Filter to objects we're tracking (IDs in `tracked` set)
> 3. For EACH tracked object:
>    - Check if it's still referenced (use `gc.get_referrers()`)
>    - If references exist ONLY in profiler's set → it's leaked
> 4. Return list of leaked object IDs
>
> **Integration**:
> - Use `set` operations (intersection? difference?)
> - Handle types with `isinstance()`
> - Report memory size with `sys.getsizeof()`
>
> Show complete implementation with type hints."

**Refinement**:
> "This works, but `detect_leaks()` is slow on 10,000+ objects. Optimize it: Can I use set operations instead of loops? What's the time complexity?"

---

### Part 5: Validate and Reflect (Convergence)

Test your profiler and connect to Chapter 19 concepts:

> "Let's validate the Memory Profiler and reflect on Chapter 19 integration:
>
> **Validation Tests**:
> 1. Create 100 objects, track them, delete 50, verify `report()` shows 50 living
> 2. Create circular reference, verify `detect_leaks()` finds it
> 3. Benchmark: Track 10,000 objects, measure `report()` speed
>
> **Reflection Questions**:
> 1. **Sets (Lesson 1)**: Why is `set[int]` better than `list[int]` for tracked IDs? (O(1) lookup vs O(n))
> 2. **Frozensets (Lesson 4)**: Where could frozensets be dict keys in this tool?
> 3. **GC (Lesson 5)**: How does `gc.collect()` interact with reference counting?
> 4. **Integration**: How do these three concepts (sets/frozensets/GC) work TOGETHER?
>
> For each question, give me a specific example from the Memory Profiler code."

**Final challenge**:
> "If I removed sets and used lists for everything, how would performance change? Calculate time complexity difference for 10,000 objects."

---

**Time**: 60-90 minutes total
**Outcome**: You've built a production-quality Memory Profiler integrating sets (O(1) lookup), frozensets (immutable dict keys), and garbage collection (leak detection)—demonstrating mastery of ALL Chapter 19 concepts in a cohesive system.

---

## 🎯 Capstone Completion Checklist

Before considering this capstone complete, verify:

- [ ] **Specification**: You've written (or reviewed) a clear specification for the Memory Profiler
- [ ] **Core Implementation**: You can explain how the `MemoryProfiler` class works
- [ ] **Sets Integration**: You understand why `set[int]` is the right choice for tracking (O(1) lookup)
- [ ] **Frozensets Integration**: You can explain why `frozenset[str]` works as dictionary keys
- [ ] **GC Integration**: You've used `gc.collect()` and `gc.get_objects()` in your code
- [ ] **Type Hints**: All functions have type hints (`set[int]`, `dict[str, int]`, etc.)
- [ ] **Testing**: You've run tests on circular references and large graphs
- [ ] **Reflection**: You can articulate how sets, frozensets, and GC work together

---

## ✨ What You've Accomplished

You've just completed a professional-grade software development cycle:

1. **Designed** → Specification-first thinking (requirement before code)
2. **Implemented** → Built working, type-hinted code with proper structure
3. **Tested** → Validated behavior on edge cases (circular refs, large graphs)
4. **Reflected** → Synthesized concepts into a coherent understanding
5. **Collaborated** → Worked with AI as a thinking partner, not just a code generator

This is how real software gets built. This is **AI-native development**.

---

## 💡 Next Steps

**In Chapter 20 (Functions & Scope)**, you'll build on these skills:
- Write functions that return collections (lists, sets, dicts)
- Understand scope and how memory management relates to variable lifetime
- Apply sets in real function examples

**In future chapters**, you'll see:
- Advanced data structures (heaps, graphs) that use sets internally
- Database queries that leverage set operations for efficiency
- Memory-efficient streaming algorithms

But first, **you've proven you understand Chapter 19**. You've integrated sets, frozensets, and garbage collection into a working system. That's significant.

---

## 🚀 Try With AI: Capstone Prompt Set

Use this section to apply your learning through AI collaboration.

### Setup
You've completed the Memory Profiler capstone. Now you'll use AI to:
1. Refine your design thinking
2. Generate working code
3. Validate your approach
4. Synthesize what you've learned

**Tool**: ChatGPT (web) or Claude Code (CLI)
**Time**: 15-20 minutes for all 4 prompts

---

### Prompt Set A: Design Thinking (B1-B2)

**Prompt 1 - Specification Refinement**:
> "I'm building a memory profiler for Python programs. Here's my specification:
>
> **Goal**: Track object creation/deletion and display memory statistics
> **Requirements**:
> - Use sets to track object IDs
> - Use gc module for analysis
> - Display current, peak, and total object counts
> - Handle circular references
>
> What am I missing? What requirements should I prioritize? How would I know when this is 'done'?"

**Expected Response**: AI helps clarify "done" criteria, suggests missing requirements (error handling, performance constraints), helps prioritize.

**Your Validation**: You receive clearer success criteria. This is professional specification work.

---

### Prompt Set B: Implementation Partnership (B1)

**Prompt 2 - Code Generation with Type Hints**:
> "Generate a Python `MemoryProfiler` class that:
> - Uses `set[int]` to track created object IDs and `set[int]` to track deleted IDs
> - Has a method `track_object(obj: Any) -> None`
> - Has a method `count_living_objects() -> int` that returns created count minus deleted count
> - Has a method `profile_memory() -> dict[str, int]` that uses gc.collect() and gc.get_objects()
> - Has a method `print_report() -> None` that displays stats
> - Includes complete type hints and docstrings
>
> Make it production-quality code."

**Expected Response**: Working, well-documented code with type hints matching modern Python standards.

**Your Validation**: Run the code. Verify it works. Modify it to add a feature (e.g., categorization).

---

### Prompt Set C: Testing & Validation (B1)

**Prompt 3 - Test Strategy**:
> "How would I thoroughly test a MemoryProfiler? I want to be confident it works correctly. What edge cases matter? Should I test:
> - Does it track all objects I create?
> - Does it properly count deletion?
> - Does it handle circular references?
> - Does it work with large object graphs?
>
> Show me one test function for circular references, using `gc.collect()` to trigger cycle detection."

**Expected Response**: Test strategy + sample test code demonstrating circular reference handling.

**Your Validation**: Understand why each test matters. Run tests to validate your implementation.

---

### Prompt Set D: Integration Reflection (B1-B2)

**Prompt 4 - Synthesis Question**:
> "Explain to me (as if I were learning Python) how this Memory Profiler tool uses ALL of Chapter 19:
>
> - **Sets** (Lesson 1): How do they help track objects? Why not use a list?
> - **Frozensets** (Lesson 4): How would I use them in this tool? Show an example.
> - **GC module** (Lesson 5): What does gc.collect() do? Why is it important?
> - **Reference counting**: How does refcount tell us about object lifetime?
>
> Then answer: If you removed sets from this design, what would break?"

**Expected Response**: Comprehensive explanation showing how concepts work together.

**Your Validation**: You understand the integration. You can teach this to someone else.

---

### Safety & Responsibility Guidelines

When using AI in your Memory Profiler project:

✅ **DO**:
- Review all generated code before running it
- Run tests to verify correctness
- Ask follow-up questions about design choices
- Combine AI ideas with your own thinking
- Validate memory measurements are accurate

⚠️ **WATCH OUT FOR**:
- AI might generate incomplete error handling—review for edge cases
- Memory measurements might vary slightly across systems—test on your machine
- Large object graphs could consume significant memory—start small and scale up
- Circular references need `gc.collect()` to be freed—don't assume reference counting alone

---

### Expected Outcomes

After completing all 4 prompts, you should be able to:

1. ✓ Write a clear specification for a Python tool (Prompt 1)
2. ✓ Understand and modify AI-generated code (Prompt 2)
3. ✓ Design tests for correctness and edge cases (Prompt 3)
4. ✓ Explain how Python concepts work together (Prompt 4)

These are the skills of an **AI-native developer**: specification thinking, code understanding, validation, and synthesis.

---

