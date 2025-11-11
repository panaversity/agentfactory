---
title: "Chapter 25: Object-Oriented Programming Part II"
sidebar_position: 25
chapter: 25
duration_total_minutes: 340
skills_count: 9
proficiency_progression: "B1→B2"
---

# Chapter 25: Object-Oriented Programming Part II

## Overview

Object-Oriented Programming Part II builds on Chapter 24 foundations to teach advanced OOP patterns: inheritance hierarchies with Method Resolution Order, polymorphism and duck typing, composition over inheritance, special methods (magic methods), and professional design patterns.

After this chapter, you can design professional object-oriented systems using inheritance, composition, polymorphism, and industry-standard design patterns. You'll understand when to use inheritance vs composition, how Python's special methods make objects Pythonic, and how design patterns enable scalable architectures.

## Learning Outcomes

By completing this chapter, you will be able to:

1. **Create inheritance hierarchies** using `super()` and explain Method Resolution Order (MRO) through C3 linearization
2. **Implement polymorphic systems** using abstract base classes, @abstractmethod, and duck typing principles
3. **Choose composition over inheritance** for flexible designs and organize code into modules and packages
4. **Master special methods** to customize object behavior (`__str__`, `__repr__`, `__add__`, `__len__`, `__iter__`, `__eq__`, `__hash__`, `__call__`)
5. **Apply design patterns** (Singleton, Factory, Observer, Strategy) to build professional multi-agent architectures
6. **Analyze design tradeoffs** and select appropriate OOP approaches for real problems

## Prerequisites

**Chapter 24: Object-Oriented Programming Part I** (REQUIRED)
- You must understand: classes, objects, encapsulation, methods, basic abstract base classes
- If you haven't completed Chapter 24, start there first

**Basic Python knowledge** (from Chapters 1-23)
- Type hints, functions, control flow, data structures

## Chapter Structure

This chapter contains 5 lessons organized from foundational concepts through professional architecture patterns:

### Lesson 1: Inheritance and Method Resolution Order (70 minutes)

Learn how to build class hierarchies using single and multiple inheritance. Master the `super()` function and understand Method Resolution Order (MRO) through Python's C3 linearization algorithm.

**Key Skills**: Single inheritance, `super()` usage, multiple inheritance, diamond inheritance problem, MRO, C3 linearization, method overriding, inheritance design decisions

### Lesson 2: Polymorphism and Duck Typing (55 minutes)

Implement polymorphic behavior using abstract base classes and discover Python's duck typing philosophy. Learn when to enforce contracts through ABC vs rely on shared behavior.

**Key Skills**: Method overriding, abstract base classes deep dive, @abstractmethod decorator, duck typing philosophy, Protocol-based programming, type checking tradeoffs

### Lesson 3: Composition Over Inheritance and Code Organization (55 minutes)

Design flexible systems using composition ("has-a") instead of inheritance ("is-a"). Organize Python code into professional modules and packages.

**Key Skills**: Composition design patterns, has-a vs is-a relationships, aggregation vs composition, module organization, packages with `__init__.py`

### Lesson 4: Special Methods (Magic Methods) (80 minutes)

Make your custom objects behave like Python's built-in types by implementing special method protocols. Master `__str__`, `__repr__`, operator overloading, container protocols, iteration, comparison, and callable objects.

**Key Skills**: String representation (`__str__`, `__repr__`), operator overloading (`__add__`, `__sub__`, `__mul__`), container protocol (`__len__`, `__getitem__`), iteration protocol (`__iter__`, `__next__`), comparison and hashing (`__eq__`, `__lt__`, `__hash__`), callable objects (`__call__`)

### Lesson 5: Design Patterns (Capstone) (80 minutes)

Integrate all Chapter 25 knowledge by implementing four professional design patterns (Singleton, Factory, Observer, Strategy) in a real multi-agent system.

**Key Skills**: Singleton pattern, Factory pattern, Observer pattern, Strategy pattern, pattern selection and tradeoffs, multi-pattern integration

## Proficiency Progression

This chapter progresses from CEFR B1 (Understand complex inheritance) through B2 (Design professional OOP systems):

| Lesson | Starting Level | Ending Level | New Concepts | Cognitive Load |
|--------|----------------|--------------|--------------|-----------------|
| 1 | B1 | B2 | 8 concepts | Within limit ✓ |
| 2 | B2 | B2 | 7 concepts | Within limit ✓ |
| 3 | B2 | B2 | 6 concepts | Well below limit ✓ |
| 4 | B2 | B2 | 10 concepts | At B2 limit ✓ |
| 5 | B2 | B2 | 0 (synthesis) | Integration only ✓ |

## Time Commitment

- **Total chapter time**: ~340 minutes (5 hours 40 minutes)
- **Recommended pace**: 1-2 lessons per week
- **Per lesson**: 55-80 minutes depending on exercises
- **With "Try With AI" exercises**: Add 10-15 minutes per prompt

## What You'll Build

By the end of this chapter, you'll have:

1. **Multi-level inheritance hierarchies** with proper `super()` usage demonstrating complex class relationships
2. **Polymorphic systems** that work seamlessly with multiple agent types (ChatAgent, CodeAgent, DataAgent)
3. **Refactored designs** transforming rigid inheritance to flexible composition patterns
4. **Custom objects** that support arithmetic operators (`+`, `-`, `*`), length (`len()`), indexing, iteration, comparison, and function-call syntax
5. **Professional multi-agent architecture** combining Singleton, Factory, Observer, and Strategy patterns in an integrated system

## Chapter Themes

Throughout Chapter 25, we emphasize:

- **Inheritance is powerful but subtle**: Understand when it applies and when composition is better
- **Polymorphism unlocks flexibility**: Same interface, different implementations
- **Python protocols are elegant**: Special methods make objects feel native
- **Design patterns solve recurring problems**: Vocabulary of professional architecture
- **AI-native development uses these patterns**: Real multi-agent systems rely on these concepts

## Prerequisites for Chapter 26

After completing Chapter 25, you'll be ready for Chapter 26: **Metaclasses and Advanced Type Systems**, which explores:
- Metaclasses: Classes that create classes
- Dataclasses: Simplified class creation with @dataclass
- Type parameters and generics
- Advanced type checking with mypy and pyright

## Study Tips

1. **Code Along**: Type examples yourself; don't copy-paste. You learn by doing.
2. **Explore with AI**: Use "Try With AI" prompts in each lesson to deepen understanding with your AI companion.
3. **Always Verify MRO**: Use `.mro()` method to inspect inheritance order when confused.
4. **Test Patterns**: Write your own agents; apply patterns to problems you care about.
5. **Connect to Chapter 24**: If stuck, review Chapter 24 concepts first—they're the foundation.
6. **Design Before Coding**: Think about inheritance vs composition BEFORE writing classes.

## Accessibility & Learning Support

- **Reading Level**: Flesch-Kincaid Grade 11-12 (appropriate for CEFR B1-B2)
- **Dense Sections**: Lesson 4 (special methods) and Lesson 5 (patterns) are intensive. Take breaks.
- **Multiple Explanations**: Complex ideas are explained 2-3 ways:
  - Plain English explanation
  - Code examples with output
  - Real-world problem examples
- **Conceptual Diagrams**: Use Claude Code's visualization suggestions in CoLearning prompts

## Getting Help

- **Confused about MRO?**
  → Lesson 1 explains it three ways: written order, C3 algorithm, visual representation

- **Not sure when to use ABC vs duck typing?**
  → See Lesson 2's decision matrix ("Use ABC When" vs "Use Duck Typing When")

- **Stuck on special methods?**
  → Each method has worked examples with expected output; run them to build intuition

- **Pattern selection unclear?**
  → Lesson 5 includes real-world scenarios for each pattern and integration guidance

- **Code won't run?**
  → All examples use Python 3.14+; ensure your environment is updated

## Tools & Resources

This chapter uses:
- **Python 3.14+** (stable, widely available)
- **Claude Code, Gemini CLI, or ChatGPT** (for "Try With AI" exercises)
- **Standard library only**: `abc`, `functools`, `typing` (no external dependencies)

## Connection to AI-Native Development

Chapter 25 is foundational for AI-native software development:

- **Inheritance & Polymorphism**: How multi-agent systems organize agent types (ChatAgent, CodeAgent, DataAgent)
- **Composition**: How agents compose tools and capabilities (Agent HAS-A ReasoningEngine)
- **Special Methods**: How custom objects integrate seamlessly with Python (making agents callable, iterable, comparable)
- **Design Patterns**: Real patterns used in production AI systems (Singleton for orchestration, Factory for dynamic creation, Observer for event-driven communication, Strategy for algorithm selection)

Understanding these patterns prepares you for professional AI engineering in Parts 5-13.

---

**Ready to start?** Begin with **Lesson 1: Inheritance and Method Resolution Order**.

Each lesson takes 55-80 minutes and includes code examples, challenges, and AI-guided exploration prompts.
