# Chapter 26: Metaclasses and Dataclasses

## What You'll Learn

This chapter explores two advanced Python features that sit at opposite ends of the spectrum:

- **Metaclasses** (Lessons 1-2): The machinery that creates classes. You'll learn how Python uses `type` to create classes, when metaclasses solve real problems (validation, registration, framework design), and how to read framework source code that uses metaclasses (Django ORM, SQLAlchemy).

- **Dataclasses** (Lessons 3-4): Modern, declarative data containers. You'll master the `@dataclass` decorator for clean data modeling, advanced features like `field()`, `field(doc=...)` (NEW in Python 3.14), `__post_init__()`, and `frozen` parameters, and practical patterns for API models and configuration objects.

- **Synthesis** (Lesson 5): Compare approaches and make informed architectural decisions. You'll learn when to use metaclasses vs dataclasses vs traditional classes, understand framework design choices, and evaluate complexity-readability tradeoffs.

## Why This Matters

These features prepare you for professional Python development where:
- **Metaclasses** help you understand framework internals and design plugin systems
- **Dataclasses** eliminate boilerplate in data-heavy applications (APIs, configuration, DTOs)
- **Architectural decision-making** separates junior from senior developers

By the end of this chapter, you'll confidently choose the right tool for each scenario and understand both the "magic behind the curtain" (metaclasses) and the "practical daily tool" (dataclasses).

**Python 3.14 Modern Features**: This chapter uses Python 3.14's modern type hint syntax (`X | None` instead of `Optional[X]`, `X | Y` instead of `Union[X, Y]`). All forward references work without quotes thanks to PEP 649 (deferred annotation evaluation).

## Learning Outcomes

By completing this chapter, you will be able to:

1. **Explain metaclasses** — Describe how Python uses `type` to create classes and when metaclasses are appropriate
2. **Create custom metaclasses** — Implement validation, registration, and singleton patterns with metaclasses
3. **Identify framework patterns** — Recognize metaclass usage in Django, SQLAlchemy, and other frameworks
4. **Create dataclasses** — Build clean data models with type hints, defaults, frozen states, and advanced features
5. **Compare approaches** — Choose between metaclasses, dataclasses, and traditional classes for different scenarios
