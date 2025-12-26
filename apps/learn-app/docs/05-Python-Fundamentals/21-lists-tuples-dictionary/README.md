---
sidebar_position: 21
title: "Chapter 21: Lists, Tuples, and Dictionary"
slides:
  source: "https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/slides/chapter-21-slides.pdf"
  title: "Chapter 21: Lists, Tuples, and Dictionary"
  height: 700
---

# Chapter 21: Lists, Tuples, and Dictionary

Real programs work with collections of data, not individual values. User lists, product inventories, shopping carts, API responses—all require storing multiple related items together. Python provides three foundational collection structures for this purpose: **lists** (mutable sequences), **tuples** (immutable sequences), and **dictionaries** (key-value mappings).

You'll learn when to use each structure, how to manipulate them effectively, and how to combine them in real-world applications. By the end of this chapter, you'll build a complete **Data Processing Pipeline** that ingests CSV data, filters it with comprehensions, aggregates statistics with dictionaries, and outputs formatted reports—demonstrating how all three structures work together in production code.

## 🎯 Before You Begin
## What You'll Learn
### Core Concepts (46+ unique concepts across 11 lessons)
**Lists** (Lessons 1-5):
- Creating and accessing lists with type hints
- Indexing, slicing, and length operations
- Mutation methods: `append()`, `extend()`, `insert()`, `remove()`, `pop()`, `clear()`
- Sorting and reversing: `sort()` vs `sorted()`, `reverse()` vs `[::-1]`
- List comprehensions with filtering
- Aliasing vs copying
**Tuples** (Lesson 6):
- Immutability as a design guarantee
- Single-element tuple syntax `(1,)`
- Unpacking for multiple assignment
- Using tuples as dict keys (hashable property)
- When to choose tuples over lists
**Dictionaries** (Lessons 7-9):
- Key-value mappings with union types
- CRUD operations: create, read, update, delete
- Safe access with `.get()` and `in` operator
- Iteration: `.keys()`, `.values()`, `.items()`
- Dict comprehensions for transformation
- Accumulator patterns for aggregation
**Architectural Thinking** (Lessons 10-11):
- Decision matrix: When to use which structure
- Performance implications (O(1) vs O(n))
- Mutability vs immutability trade-offs
- Integration patterns in real applications
---
## What's Next: From Data to Reusable Functions
You've learned to structure tasks as data: lists for ordered tasks, dictionaries for task properties (id, title, status, due_date), and tuples for immutable combinations. Your task storage is now organized. In **Chapter 23**, you'll wrap these data structure operations in reusable functions with clear inputs and outputs—creating a TaskManager module where `add_task()`, `complete_task()`, and `list_tasks()` encapsulate all the list and dictionary operations you've mastered.
Functions let you describe intent: instead of repeating list manipulation code everywhere, you call `add_task(tasks, "Buy groceries")`. This is the professional way Python developers work with data structures.