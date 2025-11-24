---
title: "Comparison Operators — Making True/False Decisions"
chapter: 15
lesson: 2
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Comparison Logic with Type Awareness"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand + Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write comparisons like 5 > 3, x == y, predict True/False results, explain why 5 == 5.0 gives True"

  - name: "Boolean Results and Type Validation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can verify comparisons return bool type using type(), understand True/False as values not strings"

  - name: "Preparing for Conditional Logic (Chapter 22 Foundation)"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why comparisons precede if statements in Chapter 22 and give examples"

learning_objectives:
  - objective: "Understand what each comparison operator (==, !=, >, <, >=, <=) does"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of operators; code predictions"

  - objective: "Apply comparison operators correctly to evaluate expressions and predict True/False results"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Write comparisons; predict outcomes; verify with type()"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (equality comparison, magnitude comparison, combined comparison, boolean result type, value vs type equality) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore type coercion in comparisons; ask AI about why True == 1 and False == 0; investigate string comparison (lexicographic ordering)"
  remedial_for_struggling: "Start with simple integer comparisons (5 > 3); practice each operator in isolation before combining; use concrete numbers before variables"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/part-4-chapter-17/spec.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "content-implementer subagent"
version: "1.0.0"
---

# Comparison Operators — Making True/False Decisions

Now that you can do math with arithmetic operators, it's time to ask questions. Can I compare two values? Is 10 bigger than 5? Are these two numbers the same? Comparison operators let you answer these questions. Instead of doing math and getting a number back, comparisons return True or False—answers to yes/no questions.

Think of comparison operators like a referee in a game: they look at two things, make a judgment, and give you an answer: "True" (yes, this condition is met) or "False" (no, it's not).

## What It Is: Asking True/False Questions

A **comparison operator** evaluates whether a condition is true or false. When you write `5 > 3`, you're asking Python: "Is 5 greater than 3?" Python answers: "True."

Python has **six comparison operators**:

- `==` **Equality** — Are these values the same?
- `!=` **Not equal** — Are these values different?
- `>` **Greater than** — Is the left side bigger?
- `<` **Less than** — Is the left side smaller?
- `>=` **Greater than or equal** — Is the left side bigger or the same?
- `<=` **Less than or equal** — Is the left side smaller or the same?

All comparisons return a **boolean value**: either `True` or `False`. This is different from arithmetic operators, which return numbers. Comparisons return yes/no answers.

## The Six Comparison Operators

Let's see all six in action. Each one asks a different question about two values.

### Equality and Inequality: Are They the Same?

```python
# Equality: asking if values are the same
x: int = 10
y: int = 5

equal: bool = x == y            # False - 10 is not equal to 5
print(f"{x} == {y}: {equal}")   # 10 == 5: False

# Inequality: asking if values are different
not_equal: bool = x != y        # True - 10 is different from 5
print(f"{x} != {y}: {not_equal}")  # 10 != 5: True

# Type verification - comparisons return bool
print(f"Type of result: {type(equal)}")  # <class 'bool'>
```

Important distinction: In Lesson 1, we saw that `=` assigns a value to a variable. Here, `==` compares two values. One `=` stores; two `==` compare. This is a common mistake, so remember it well.

#### 💬 AI Colearning Prompt

> "Why does Python use = for assignment but == for equality checking? Compare this to other programming languages. Why is this distinction important?"

Notice that we're asking AI to explain the **design choice** behind the operators, not just what they do. This helps you understand Python's thinking, not memorize syntax.

### Magnitude Comparisons: Which is Bigger?

```python
# Greater than and less than
x: int = 10
y: int = 5

greater: bool = x > y           # True - 10 is greater than 5
print(f"{x} > {y}: {greater}")  # 10 > 5: True

less: bool = x < y              # False - 10 is NOT less than 5
print(f"{x} < {y}: {less}")     # 10 < 5: False

# Greater than or equal, less than or equal
greater_equal: bool = x >= y    # True - 10 is greater than or equal to 5
print(f"{x} >= {y}: {greater_equal}")  # 10 >= 5: True

less_equal: bool = x <= y       # False - 10 is NOT less than or equal to 5
print(f"{x} <= {y}: {less_equal}")     # 10 <= 5: False

# What about comparing a value with itself?
same: bool = x >= x             # True - 10 is equal to 10
print(f"{x} >= {x}: {same}")    # 10 >= 10: True
```

The operators `>`, `<`, `>=`, `<=` compare magnitude—which value is bigger. They answer: "Is the left side bigger, smaller, or equal?" The `=` in `>=` and `<=` means "or equal to," so `10 >= 10` is True (they're equal).

#### 🎓 Expert Insight

> In AI-native development, you don't memorize the difference between `>` and `>=`. You think: "Do I want to include the equal case or not?" Then you use the operator that matches your intent. If you're uncertain, you ask AI: "Should I use > or >= here?" and let AI help you reason through the condition.

### Value Equality vs. Type Equality

Here's something that surprises many beginners: Python compares **values**, not types.

```python
# Integer 5 and float 5.0 are VALUE-equal (but different types)
int_five: int = 5
float_five: float = 5.0

# These have different types
type_different: bool = type(int_five) != type(float_five)  # True - types differ
print(f"Types are different: {type_different}")  # True

# But the VALUES are equal
values_equal: bool = int_five == float_five     # True - values are the same
print(f"Values are equal: {values_equal}")      # True

# String "5" is NOT equal to integer 5
string_five: str = "5"
string_int_compare: bool = int_five == string_five  # False - different types AND values

print(f"5 == 5.0: {int_five == float_five}")   # True (value equality)
print(f"5 == '5': {int_five == string_five}")  # False (different types)
```

This is crucial: `5 == 5.0` returns True because the **values** are the same, even though one is int and one is float. But `5 == "5"` returns False because "5" is text, not a number.

#### 🤝 Practice Exercise

> **Ask your AI**: "I'm confused about why 5 == 5.0 is True, but 5 == '5' is False. Explain value equality vs. type equality. When does Python care about types vs. just values in comparisons?"

**Expected Outcome**: You'll understand that `==` checks if values are the same (regardless of type in most cases), but type mismatches between completely different types (int vs. string) make them not equal. You'll see how Python's flexibility can be useful but also requires careful thinking.

### Comparisons in Real Scenarios

```python
# Real scenario: Checking age eligibility
user_age: int = 16
voting_age: int = 18
driving_age: int = 16

can_vote: bool = user_age >= voting_age       # False (16 is not >= 18)
can_drive: bool = user_age >= driving_age     # True (16 is >= 16)

print(f"Can vote: {can_vote}")                 # False
print(f"Can drive: {can_drive}")               # True

# Another real scenario: Password length
password: str = "SecurePass123"
min_length: int = 8

is_long_enough: bool = len(password) >= min_length  # True
print(f"Password meets length requirement: {is_long_enough}")

# Checking if values are in a specific range (prepare for logical operators in Lesson 3)
test_score: int = 85
passing_score: int = 70
perfect_score: int = 100

is_passing: bool = test_score >= passing_score  # True
is_perfect: bool = test_score == perfect_score  # False
is_valid: bool = test_score <= perfect_score    # True (score can't exceed 100)
```

## Why Comparisons Matter for Chapter 22

You might wonder: "Why are we learning comparisons separately from if statements?" The answer is that comparisons are **building blocks**. In Chapter 22, you'll learn control flow: making decisions with `if` statements. When you write `if x > 5:`, that `x > 5` is a comparison operator at work. By mastering comparisons now, Chapter 22 will feel natural—you'll already understand how to ask True/False questions.

---

## Try With AI

Ready to master comparison operators and understand why = is different from ==?

**🔍 Explore Comparison Operators:**
> "Explain the 6 comparison operators: `==`, `!=`, `>`, `<`, `>=`, `<=`. For each, show me 2 examples with numbers and explain the True/False result. Why does Python use == for comparison but = for assignment? Show me what happens if I accidentally write if x = 5: instead of if x == 5:"

**🎯 Practice Comparison Logic:**
> "Create a movie ticket checker: movie_age_rating = 13, user_age = 12. Write comparisons to check: (1) if user can watch (user_age >= movie_age_rating), (2) if user is too young (user_age < movie_age_rating), (3) if ages are exactly equal (user_age == movie_age_rating). For each, predict True or False, then verify with code."

**🧪 Test Type Comparison Edge Cases:**
> "Test these comparisons and explain which return True: (1) 5 == 5.0 (int vs. float), (2) '5' == 5 (string vs. int), (3) True == 1 (bool vs. int), (4) False == 0 (bool vs. int). Why does Python treat some cross-type comparisons as equal but not others? Show me the type() of each value."

**🚀 Apply to Your Validation:**
> "I'm building [describe your application]. Help me write comparison logic for: age restrictions, price checks, quota limits, version compatibility, or other validation rules. For each condition, show me the correct comparison operator and explain why that operator is the right choice."

---
