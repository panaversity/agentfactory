# ADR-0007: F-Strings Only Teaching Approach for String Formatting

**Status**: Accepted
**Date**: 2025-11-08
**Context**: Chapter 16 (Strings and Type Casting), applies to Chapters 12-29
**Deciders**: chapter-planner, User
**Related**: Chapter 16 Spec (specs/part-4-chapter-16/spec.md), Chapter 16 Plan (specs/part-4-chapter-16/plan.md)

---

## Context and Problem Statement

Chapter 16 must teach string formatting to A1-A2 beginner students. Python provides three formatting methods with different syntax and capabilities:

1. **% Operator** (legacy, pre-Python 2.6):
   ```python
   "Hello, %s!" % name
   "Age: %d, Grade: %.2f" % (age, grade)
   ```

2. **.format() Method** (Python 2.6+):
   ```python
   "Hello, {}!".format(name)
   "Age: {0}, Grade: {1:.2f}".format(age, grade)
   ```

3. **F-Strings** (Python 3.6+, modern standard):
   ```python
   f"Hello, {name}!"
   f"Age: {age}, Grade: {grade:.2f}"
   ```

Students need to format output dynamically (displaying variables, numbers with decimals, combined expressions). The pedagogical question is: **Should we teach one method, two methods, or all three?**

**Constraints**:
- A1-A2 beginners have max 5 new concepts per lesson (Constitution Principle 12)
- Book teaches modern Python 3.14+ (not legacy compatibility)
- AI-Native Learning emphasizes intent over syntax memorization
- Decision affects 18 chapters (Ch 12-29) - architecturally significant

---

## Decision

**Teach ONLY f-strings. Skip % operator and .format() method entirely.**

Chapter 16 will:
- Introduce f-strings as THE Python formatting method
- Show f-string syntax: `f"prefix {expression} suffix"`
- Demonstrate variable embedding, expressions, and number formatting
- Use ONLY f-strings in all code examples throughout the book (Chapters 12-29)
- NOT mention %, .format(), or Template strings

---

## Rationale

### 1. Cognitive Load Management (Constitutional Principle 12)

**A1-A2 beginners have max 5 new concepts per lesson.** Teaching multiple formatting methods violates this:

- Teaching all 3 methods = 3 syntax patterns (60% of lesson budget on formatting alone)
- Teaching 2 methods = still requires comparison/choice (cognitive overhead)
- Teaching 1 method = clear, unambiguous approach (students focus on intent, not syntax comparison)

**Research foundation**: Miller's Law and Sweller's Cognitive Load Theory show novices learn better with single canonical approaches rather than multiple options.

### 2. F-Strings are the Modern Standard

- **PEP 498** (2016): F-strings introduced as recommended method
- **Industry adoption**: All major Python projects (Django, Flask, FastAPI, pytest) use f-strings
- **Official Python docs**: Tutorial uses f-strings as primary examples
- **Performance**: F-strings are faster than % and .format() (compile-time optimization)
- **Readability**: Code reads like natural language (`f"Hello, {name}"` vs `"Hello, %s" % name`)

**This book teaches modern Python (3.14+).** Using outdated methods contradicts our "current best practices" philosophy.

### 3. Eliminates Choice Paralysis

**AI-Native Learning** requires students to focus on **intent** (what output do I want?) rather than **syntax choices** (which of 3 methods should I use?).

- With 3 methods: Students ask "When do I use % vs .format() vs f-strings?"
- With 1 method: Students ask "How do I format this output?" (focuses on problem-solving)

**Research**: Barry Schwartz's Paradox of Choice shows that limiting options improves learning outcomes and satisfaction.

### 4. Consistency Across Book (Chapters 12-29)

Teaching f-strings only establishes a **canonical pattern**:
- All Python chapters use f-strings for output
- No confusion when students revisit earlier chapters
- Codebase consistency (important for debugging and AI assistance)

**This decision affects 18 chapters** (Ch 12-29), making it architecturally significant.

### 5. Alignment with AI-Driven Development

**When students ask AI for help**, modern LLMs (Claude, GPT-4, Gemini) default to f-strings because:
- Training data reflects current industry practice (f-strings dominant post-2020)
- F-strings are the most readable format (easier for AI to generate and explain)
- Asking AI "format this output" will produce f-strings by default

Teaching f-strings aligns student learning with AI collaboration patterns.

---

## Considered Alternatives

### Option A: Teach All Three Methods (%, .format(), f-strings)

**Approach**: Show historical evolution, compare syntax, explain when to use each.

**Rejected because**:
- **Cognitive overload**: 3 syntax patterns exceed A1-A2 capacity (5 concepts max)
- **Unnecessary complexity**: % and .format() offer no advantages for beginners
- **Outdated knowledge**: Students waste time learning deprecated approaches
- **Choice paralysis**: Students confused about which method to use
- **Maintenance burden**: Book examples would need to show all 3 methods for fairness

**Cost**: 3x cognitive load, 2x obsolete methods, unclear best practice

### Option B: Teach .format() and F-Strings (Skip %)

**Approach**: Acknowledge % as legacy, teach .format() as "older style", f-strings as "modern style".

**Rejected because**:
- **Still 2 methods**: Cognitive load still exceeds necessary (2 syntax patterns)
- **.format() is also obsolete**: Python community moved to f-strings (2016+)
- **No pedagogical value**: .format() teaches no concept f-strings don't cover
- **Confusion**: Students ask "Why learn .format() if f-strings are better?"

**Cost**: 2x cognitive load, 1x obsolete method, unclear best practice

### Option C: F-Strings Only (CHOSEN)

**Approach**: Introduce f-strings as THE Python formatting method. No mention of alternatives.

**Chosen because**:
- **Minimal cognitive load**: 1 syntax pattern = clear, unambiguous
- **Modern standard**: Aligns with Python 3.14+ best practices
- **Industry alignment**: Matches real-world code students will encounter
- **Future-proof**: F-strings are the stable, long-term standard
- **Clear guidance**: No choice paralysis; students know "the Python way"

**Cost**: Students don't learn legacy methods (acceptable tradeoff—if needed later, AI can explain)

---

## Consequences

### Positive

✅ **Cognitive clarity**: Students learn one clear approach (no syntax comparison overhead)
✅ **Modern skillset**: Students write production-quality Python from day 1
✅ **AI alignment**: AI collaboration defaults to f-strings (students and AI speak same language)
✅ **Code consistency**: All book examples use f-strings (easier to read, debug, reference)
✅ **Performance**: F-strings are faster (students learn the optimized approach)
✅ **Reduced confusion**: No "which method should I use?" questions

### Negative

⚠️ **Legacy code illiteracy**: Students won't recognize % or .format() in old tutorials/Stack Overflow
  - **Mitigation**: When students encounter legacy syntax, they ask AI "What is this syntax?" (AI explains % and .format() in context)

⚠️ **Compatibility constraint**: F-strings require Python 3.6+ (we use 3.14+, so non-issue)
  - **Mitigation**: Book specifies Python 3.14+ as minimum version

### Neutral

ℹ️ **Explanation overhead**: Lesson 3 includes brief "Why f-strings?" section comparing readability
ℹ️ **AI troubleshooting**: If AI generates % or .format(), students learn to ask "Can you use f-strings instead?"

---

## Implementation

**Lesson 3 Structure** (from plan.md):
1. What f-strings are (modern Python formatting)
2. F-string syntax and variable embedding
3. Expressions inside f-strings (calculations, method calls)
4. Number formatting (decimal places, currency)
5. "Why f-strings?" brief comparison (NOT teaching %, .format()—just showing why f-strings are clearer)

**Code Example 3.4** (pedagogical justification):
```python
# Show readable comparison (WITHOUT teaching old methods)
name: str = "Alice"
age: int = 25

# What we teach (f-string)
message: str = f"Hello, {name}! You are {age} years old."
print(message)

# Brief mention: "Older Python used different syntax, but f-strings are now the standard."
# Students understand f-strings are modern without needing to learn old methods.
```

**Throughout Chapters 12-29**:
- All print statements use f-strings
- All string formatting examples use f-strings
- No % or .format() appears in any code example
- If AI generates non-f-string syntax, lesson teaches "ask AI to use f-strings"

---

## References

- **PEP 498** (F-String Literal Strings): https://peps.python.org/pep-0498/
- **Python Official Tutorial**: Uses f-strings as primary formatting method
- **Constitution v3.0.2**: Principle 12 (Cognitive Load Management), Principle 13 (Graduated Teaching)
- **Cognitive Load Theory** (Sweller et al.): Single example better than multiple options for novices
- **Chapter 16 Spec**: `specs/part-4-chapter-16/spec.md`
- **Chapter 16 Plan**: `specs/part-4-chapter-16/plan.md` (Lesson 3: F-String Formatting)

---

## Related Decisions

- **Type Hints Throughout**: Related modern Python standard (Constitution requirement)
- **Python 3.14+ Minimum Version**: Enables f-strings, match/case, modern types

---

## Decision Impact

This ADR establishes the **canonical string formatting approach** for the entire Python fundamentals section (18 chapters). Future chapters reference this decision when formatting output.

**Scope**: Chapters 12-29 (Part 4: Python Fundamentals)
**Supersedes**: None (first formal decision on string formatting pedagogy)
**Review Date**: When Python introduces new formatting paradigm (unlikely before Python 4.x)
