# Chapter 27: Pydantic and Generics

**Part 4: Python Fundamentals**
**Complexity Tier**: Advanced (B1-B2 CEFR Proficiency)
**Estimated Time**: 3.5-4 hours

## Overview

This chapter teaches advanced type safety and data validation for AI-native Python development through **Pydantic** (runtime validation) and **Generics** (static type safety). You'll learn to validate LLM outputs, build type-safe containers, and combine both patterns in a production-quality Config Manager.

**Key Topics**:
- Pydantic V2 for runtime data validation
- Modern PEP 695 Generic syntax (Python 3.14+)
- AI-Native Learning pedagogy (describe → explore → validate → iterate)
- Production patterns for config management and AI output validation

## Prerequisites

Before starting this chapter, you should have completed:
- **Chapters 1-26** (especially Chapters 24-26: OOP fundamentals)
- **Strong foundation in**:
  - Python classes, inheritance, and special methods
  - Type hints (`list[int]`, `dict[str, float]`, `X | None`)
  - Exception handling (try/except blocks)
  - JSON data structures

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Pydantic Data Validation (B1-B2)**
   - CREATE Pydantic models with nested validation and custom validators
   - APPLY Pydantic to validate LLM-generated JSON outputs
   - HANDLE validation errors gracefully in production code

2. **Generic Type Safety Patterns (B1-B2)**
   - WRITE generic functions and classes using TypeVar and PEP 695 syntax
   - ANALYZE when Generics improve type safety vs simpler approaches

3. **Integration Patterns (B2)**
   - INTEGRATE Pydantic validation with Generic containers
   - EVALUATE tradeoffs between Pydantic, TypedDict, and dataclasses

4. **AI-Native Application (B2)**
   - VALIDATE AI agent outputs (LLM JSON, structured data)
   - EXPLAIN how validation fits into AI-native development workflow

## Chapter Structure

### Lesson Index

| # | Lesson | Proficiency | Time | Topics |
|---|--------|-------------|------|--------|
| 1 | [Introduction to Pydantic and Data Validation](./01-introduction-to-pydantic.md) | B1 | 35-40 min | BaseModel, ValidationError, nested models |
| 2 | [Advanced Pydantic Patterns](./02-advanced-pydantic-patterns.md) | B1-B2 | 40-45 min | Custom validators, Field() constraints, BaseSettings |
| 3 | [Introduction to Generics and Type Variables](./03-introduction-to-generics.md) | B1 | 35-40 min | TypeVar, PEP 695 syntax, type preservation |
| 4 | [Generic Classes and Protocols](./04-generic-classes-and-protocols.md) | B1-B2 | 40-45 min | Generic[T], bounded types, Protocols |
| 5 | [Pydantic for AI-Native Development](./05-pydantic-for-ai-native-development.md) | B2 | 40-45 min | LLM output validation, iterative refinement |
| 6 | [Capstone: Type-Safe Config Manager](./06-capstone-type-safe-config-manager.md) | B2 | 60-90 min | Production config system (portfolio project) |

### Proficiency Progression

The chapter follows a graduated complexity arc:

```
Lesson 1-2: Pydantic Fundamentals (B1 → B1-B2)
Lesson 3-4: Generics Fundamentals (B1 → B1-B2)
Lesson 5-6: Integration & Capstone (B2)
```

## Success Evals

You'll know you've mastered this chapter when you can:

- ✅ **EVAL-001**: Explain the difference between runtime validation (Pydantic) and static type checking (Generics)
- ✅ **EVAL-004**: Create Pydantic models with nested validation and custom validators
- ✅ **EVAL-005**: Write generic functions with TypeVar that work with multiple types
- ✅ **EVAL-006**: Build a production-quality Config Manager (capstone project)
- ✅ **EVAL-007**: Validate LLM-generated JSON output with Pydantic

## Setup Instructions

### Install Dependencies

This chapter requires Pydantic V2. Install it using `uv`:

```bash
uv add pydantic
```

**Optional** (for .env file support in Lesson 2):
```bash
uv add python-dotenv
```

### Verify Installation

```python
import pydantic
print(f"Pydantic version: {pydantic.__version__}")
# Expected: 2.x.x (V2)
```

## Code Examples

This chapter includes **9 production-quality code examples**:

- **EX-001**: Basic Pydantic Model (Book validation)
- **EX-002**: Nested Models (Author + Book composition)
- **EX-003**: Field Validators (email validation)
- **EX-004**: Settings from Environment (BaseSettings)
- **EX-005**: Generic Function (get_first_item[T])
- **EX-006**: Generic Container Class (Stack[T])
- **EX-007**: Bounded Generic (Comparable Protocol)
- **EX-008**: LLM Output Validation (Recipe generation)
- **EX-009**: Config Manager Capstone (full production system)

All examples are tested with Python 3.14+ and Pydantic V2.

## Capstone Project

**Lesson 6** culminates in building a **Type-Safe Config Manager** - a production-quality configuration system that:
- Loads config from YAML files, environment variables, and CLI arguments
- Validates all config with Pydantic models
- Provides type-safe access with Generic containers
- Includes comprehensive tests and documentation

This is a **portfolio-worthy project** you can use in your own Python applications.

## How to Use This Chapter

### Sequential Learning Path

1. **Complete lessons in order** (Lesson 1 → Lesson 2 → ... → Lesson 6)
2. **Each lesson ends with "Try With AI"** - 4 prompts for hands-on practice
3. **Use your AI companion** (Claude Code, Gemini CLI, ChatGPT) for exercises
4. **Build the capstone project** to synthesize all concepts

### AI-Native Learning Pattern

This chapter follows the **AI-Native Learning** pedagogy:

1. **Describe** intent → 2. **Explore** with AI → 3. **Validate** → 4. **Learn from errors**

You'll practice this loop throughout all 6 lessons, especially in Lesson 5 (Pydantic for AI-Native Development).

## Additional Resources

- **Pydantic V2 Documentation**: [https://docs.pydantic.dev/](https://docs.pydantic.dev/)
- **PEP 695 (Generic Syntax)**: [https://peps.python.org/pep-0695/](https://peps.python.org/pep-0695/)
- **Python Type Hints**: [https://docs.python.org/3/library/typing.html](https://docs.python.org/3/library/typing.html)

## Status

**Implementation Status**: ✅ Complete

- [x] Lesson 1: Introduction to Pydantic and Data Validation
- [x] Lesson 2: Advanced Pydantic Patterns
- [x] Lesson 3: Introduction to Generics and Type Variables
- [x] Lesson 4: Generic Classes and Protocols
- [x] Lesson 5: Pydantic for AI-Native Development
- [x] Lesson 6: Capstone - Type-Safe Config Manager

---

**Next**: Start with [Lesson 1: Introduction to Pydantic and Data Validation](./01-introduction-to-pydantic.md)
