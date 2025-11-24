---
sidebar_position: 23
title: "Chapter 20: Set, Frozen Set, and GC"
slides:
  source: "slides/chapter-23-slides.pdf"
  title: "Chapter 20: Set, Frozen Set, and GC"
  height: 700
---

# Chapter 20: Set, Frozen Set, and GC

Need to check if a million user IDs exist in your database? Need to find common connections between social networks? Need to eliminate duplicates from a massive dataset? Sets are your answer.

Python's **set** and **frozenset** types offer O(1) average-time lookup—meaning checking if an item exists is nearly instant, regardless of collection size. While lists require checking every element (O(n) time), sets use hash-based storage to jump directly to the answer. This makes sets essential for performance-critical operations like deduplication, membership testing, and mathematical set operations.

But sets aren't magic—they require understanding **hashing** (how Python converts objects to lookup keys) and **immutability** (why only unchangeable objects can be set members). You'll also explore **garbage collection** (GC)—Python's automatic memory management system that frees unused objects and prevents memory leaks.

By the end of this chapter, you'll build a **Memory Profiler tool** that tracks object creation and deletion in your programs—integrating sets, frozensets, and garbage collection into a practical debugging aid.

## 🎯 Before You Begin

## What You'll Learn

This chapter teaches hash-based data structures and memory management through 6 progressive lessons:

### Foundation: Sets and Uniqueness

- **Lesson 1: Set Basics** — Understand what sets are, create them with type hints (`set[int]`), and grasp the uniqueness property and hashability requirement
- **Lesson 2: Set Operations** — Perform mathematical operations (union, intersection, difference) and write set comprehensions for filtered collections
- **Lesson 3: Set Internals & Hashing** — Understand how hash functions enable O(1) lookup, why immutability is required, and when sets outperform lists dramatically

### Immutable Variants and Memory Management

- **Lesson 4: Frozensets** — Use immutable sets as dictionary keys and nested set members; choose between `set` and `frozenset` based on requirements
- **Lesson 5: Garbage Collection** — Understand Python's automatic memory management through reference counting, handle circular references, and use the `gc` module for profiling

### Integration Capstone

- **Lesson 6: Memory Profiler Capstone** — Design and build a working tool that tracks object creation/deletion using sets, frozensets, and the `gc` module