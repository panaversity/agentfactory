---
sidebar_position: 23
title: "Chapter 23: Modules and Functions"
slides:
  source: "https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/slides/chapter-23-slides.pdf"
  title: "Chapter 23: Modules and Functions"
  height: 700
---

# Chapter 23: Modules and Functions

As programs grow, copy-pasting code becomes unsustainable. You need to organize code into reusable pieces—**functions** that perform specific tasks with clear inputs and outputs, and **modules** (`.py` files) that group related functions together. This is how professional Python developers build maintainable applications.

This chapter bridges foundational Python syntax (Chapters 15-19) with production-oriented patterns by teaching you to describe intent through function signatures with type hints, organize code into logical modules with clear separation of concerns, reuse code through imports and function calls, and validate behavior through testing.

By the end of this chapter, you'll build a real multi-module project (Calculator Utility) that demonstrates professional Python organization patterns.

## 🎯 Before You Begin
---
## What You'll Learn
- **Understanding Modules and Imports** — Learn what a module is and how Python organizes code, explore three import patterns (import module, from module import function, from module import function as alias), use built-in modules (math, random, os) immediately
- **Writing Functions with Intent** — Write functions with clear parameters, return values, type hints, and docstrings that tell other developers (and AI) exactly what your function needs and produces
- **Function Parameters and Returns** — Master positional parameters, default parameters, keyword arguments, and returning multiple values to design functions that work in multiple ways while maintaining clarity
- **Scope and Nested Functions** — Understand variable scope (local, global, enclosing) and how nested functions work with closures to prevent bugs and clarify your design
- **Building a Calculator Utility Capstone** — Integrate all concepts by building a real multi-module calculator project that uses modules for separation of concerns, clear functions with type hints, proper testing, and clean orchestration
---
## What's Next: Persisting Your Data to Disk
Your TaskManager functions now manipulate task data in memory with type hints and clear behavior. But here's the problem: when your program stops, all tasks vanish. In **Chapter 25**, you'll persist your tasks to disk using file I/O, learning to save task lists as JSON and load them back when the program restarts. You'll discover how to use context managers (`with` statements) to safely handle files, and how to structure your code so functions can read from and write to files without breaking their clarity.
This is when your Todo application becomes truly useful: create tasks → manipulate them with functions → save to disk → load them later.
