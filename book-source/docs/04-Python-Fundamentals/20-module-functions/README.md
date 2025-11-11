# Chapter 20: Modules and Functions

Chapter 20 teaches you how to organize and reuse Python code through **modules** and **functions**. Modules are `.py` files containing reusable code. Functions are reusable blocks of code with clear parameters and return values.

This chapter bridges foundational Python syntax (Chapters 13-19) with production-oriented patterns by teaching you to:
- **Describe intent** through function signatures with type hints
- **Organize code** into logical modules with clear separation of concerns
- **Reuse code** through imports and function calls
- **Validate behavior** through testing

By the end of this chapter, you'll build a real multi-module project (Calculator Utility) that demonstrates professional Python organization patterns.

---

## What You'll Learn

### Lesson 1: Understanding Modules and Imports (A2 - 55 min)
Learn what a module is and how Python organizes code. Explore three import patterns:
- `import module_name` — Import entire module, access via dot notation
- `from module import function` — Import specific function, use directly
- `from module import function as alias` — Import with custom name

**What you'll understand**: Modules are tools Python provides. You can use built-in modules (math, random, os) immediately.

### Lesson 2: Writing Functions with Intent (A2-B1 - 60 min)
Learn to write functions with clear parameters, return values, type hints, and docstrings.

**What you'll understand**: Type hints and docstrings are your specification. They tell other developers (and AI) exactly what your function needs and produces.

### Lesson 3: Function Parameters and Returns (B1 - 60 min)
Master positional parameters, default parameters, keyword arguments, and returning multiple values.

**What you'll understand**: Functions have flexible calling patterns. You can design them to work in multiple ways while maintaining clarity.

### Lesson 4: Scope and Nested Functions (B1-B2 - 55 min)
Understand variable scope (local, global, enclosing) and how nested functions work with closures.

**What you'll understand**: Variables have regions where they exist. Understanding scope prevents bugs and clarifies your design.

### Lesson 5: Building a Calculator Utility Capstone (B1-B2 - 70 min)
Integrate all concepts by building a real multi-module calculator project.

**What you'll understand**: Professional Python code uses modules for separation of concerns, clear functions with type hints, proper testing, and clean orchestration.