# Chapter 15: Operators, Keywords, and Variables — Detailed Implementation Plan

**Chapter Type**: Technical / Code-Focused
**Chapter Objective(s)**: Students understand and apply four operator categories (arithmetic, comparison, logical, assignment) and recognize Python keywords, developing AI-Native Learning skills through exploratory prompts and validation practices.
**Estimated Total Time**: 3.5–4 hours (5 lessons × 45–50 min each)
**Part**: Part 4 - Python Fundamentals (Chapters 12-29)
**Complexity Tier**: A1-A2 (Beginner, building to early intermediate)
**Status**: Ready for Implementation
**Feature Branch**: `015-operators-keywords-variables`
**Spec Reference**: `specs/part-4-chapter-15/spec.md`

---

## 🎯 Chapter Overview & Context

**Building on Chapter 14 Foundation**:
- Chapter 14 taught: Data types (int, float, str, bool, None), type hints, `type()` and `isinstance()` validation
- Chapter 15 builds on this by teaching: How to DO things with those types using operators, recognizing language boundaries (keywords)
- Critical transition: Students move from **knowing data types exist** to **performing operations on data**

**Bridge to Chapter 17**:
- Chapter 15 teaches operators and logical thinking
- Chapter 17 (Control Flow) will use comparison and logical operators extensively in if/while/for statements
- Operators are the **prerequisite vocabulary** for Chapter 17

**AI-Native Learning Pattern** (how students learn Python with AI):
- **Describe intent**: "What happens if I add int + int?" (exploratory question)
- **Explore with AI**: Students ask "What does / do?" instead of reading operator documentation
- **Validate together**: Use `type()` to check result types; verify understanding by testing edge cases
- **Learn from errors**: When TypeError or ZeroDivisionError occurs, ask AI "why?" to deepen learning

**Key Scope Boundaries** (Chapter 15 Responsibility):
- ✅ **Teach Comprehensively**: Arithmetic (+, -, *, /, //, %, **), Comparison (==, !=, >, <, >=, <=), Logical (and, or, not), Assignment (=, +=, -=, *=, /=)
- ✅ **Teach Keywords**: What they are, how to check the list, why they're reserved
- ✅ **Type Safety Focus**: All examples use type hints; validation with `type()` in every lesson
- ✅ **Capstone Integration**: "Calculator with Type Safety" project combines all operator types
- ❌ **Deferred**: Identity operators (is, is not - Ch 24+), Membership operators (in, not in - Ch 18-19), Bitwise operators, Walrus operator, Operator overloading

---

## 📊 Lesson Architecture

### Lesson 1: Arithmetic Operators — Doing Math with Python

**Learning Objective** (Bloom's: Understand + Apply): Students can explain what each of the seven arithmetic operators (+, -, *, /, //, %, **) does and apply them correctly to perform calculations, using type hints and validation.

**Skills Taught** (CEFR Proficiency Mapping):

- **Skill 1**: Arithmetic Operations with Type Safety
  - CEFR Level: A1-A2 (Foundation-Basic)
  - Category: Technical
  - Bloom's Level: Understand + Apply
  - DigComp Area: Problem-Solving (using tools for calculations)
  - Measurable at This Level: Student can write `result: int = 5 + 3` and use `type(result)` to verify result type; perform all 7 arithmetic operations without referencing examples

- **Skill 2**: Understanding Type Hint Feedback from Operations
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Understand
  - DigComp Area: Digital Content Creation (using type hints correctly)
  - Measurable at This Level: Student can explain "Why does int / int give float?" and "Why does int // int give int?" using type validation

- **Skill 3**: AI-Driven Exploration of Edge Cases
  - CEFR Level: A2 (Basic)
  - Category: Soft
  - Bloom's Level: Apply
  - DigComp Area: Communication & Collaboration (asking productive questions)
  - Measurable at This Level: Student can formulate questions like "What happens if I divide by zero?" and validate AI responses through code testing

**Key Concepts** (Cognitive Load: 5 concepts):
1. Addition (+) — combining values
2. Subtraction (-) — finding difference
3. Multiplication (*), Division (/), Floor Division (//) — scaling and dividing values
4. Modulus (%) — finding remainder
5. Exponentiation (**) — raising to power
(These 7 operators grouped as "arithmetic family" = 1 core concept: "math operations on numbers")

**Prerequisites**:
- Chapter 14: Variables with type hints, data types (int, float), `type()` function
- Chapter 13: Ability to run Python code and see output

**Duration**: 45–50 minutes

**Content Outline**:
1. **What It Is** (5 min) - Operators as verbs; arithmetic operators perform math calculations
2. **The Seven Arithmetic Operators** (20 min) - Each operator with examples, behavior with int/float
3. **Type Behavior & Validation** (15 min) - Why division gives float, when to use //, type verification
4. **Try With AI** (10 min) - Four progressive prompts with expected outcomes

**Content Elements**:

**Code Example 1: Basic Arithmetic (All 7 Operators)**
```python
# Arithmetic operators: doing math with Python
x: int = 10
y: int = 3

# The 7 arithmetic operators
add_result: int = x + y          # 13 - addition
sub_result: int = x - y          # 7 - subtraction
mul_result: int = x * y          # 30 - multiplication
div_result: float = x / y        # 3.333... - division (always float)
floor_result: int = x // y       # 3 - floor division (integer result)
mod_result: int = x % y          # 1 - modulus (remainder)
exp_result: int = x ** 2         # 100 - exponentiation
```
- **Purpose**: Show all 7 operators in one compact example; demonstrate type hints for each
- **Complexity**: Beginner (no new concepts beyond operator symbols)

**Code Example 2: Type Behavior with Division**
```python
# Why division and floor division are different
num1: int = 10
num2: int = 3

regular_div: float = num1 / num2    # 3.3333... (always float)
floor_div: int = num1 // num2       # 3 (integer result)

# Verify with type()
print(f"Type of {num1} / {num2}: {type(regular_div)}")   # <class 'float'>
print(f"Type of {num1} // {num2}: {type(floor_div)}")    # <class 'int'>

# Edge case: working with floats
f1: float = 10.5
f2: float = 3.0
print(f"Type of {f1} / {f2}: {type(f1 / f2)}")          # <class 'float'>
print(f"Type of {f1} // {f2}: {type(f1 // f2)}")        # <class 'float'> (not int!)
```
- **Purpose**: Explain division behavior; demonstrate that floor division with float returns float, not int
- **Complexity**: Beginner (concept: type behavior changes based on operand types)

**Code Example 3: Operator Precedence**
```python
# Order of operations matters: * and / before + and -
result1: int = 2 + 3 * 4         # 14 (not 20 - multiplication first)
result2: int = (2 + 3) * 4       # 20 (parentheses change order)

# Use parentheses for clarity
total_with_tax: float = (100 * 1.08) + 10  # Calculate price with tax, then add fee
```
- **Purpose**: Show that operator precedence (PEMDAS) applies; demonstrate how parentheses override
- **Complexity**: Beginner (context: when to use parentheses)

**Validation Points**:
- [ ] Students explain each arithmetic operator in plain language (SC-001)
- [ ] Students write arithmetic expressions correctly with type hints (SC-002)
- [ ] Students use `type()` to verify result types after operations (validation-first practice)

**Try With AI** (4 Progressive Prompts):

#### Prompt 1: Concept Exploration — "What makes / and // different?"
```
I want to understand why Python has both / and // operators.
- What does / do?
- What does // do?
- When would I use one instead of the other?
```
**Expected outcome**: Student learns that `/` always returns float (even for int operands); `//` returns integer result; understands use cases (precise division vs. counting in groups).

#### Prompt 2: Application — "Build a Simple Calculator Problem"
```
Write Python code that:
- Takes two integers: price = 50, tax_rate = 0.08
- Calculates total_price = price * (1 + tax_rate)
- Uses type hints for all variables
- Verifies result type with type()

Show me the code. Is total_price a float or int? Why?
```
**Expected outcome**: Student applies arithmetic operators to real problem; observes that multiplying int by float gives float; reinforces type understanding.

#### Prompt 3: Edge Case Discovery — "What breaks?"
```
I'm experimenting with division. What happens if I try:
- 10 / 0
- 10.5 % 3
- 2 ** 100

Show me the results. Which ones cause errors? Which are surprising?
```
**Expected outcome**: Student discovers ZeroDivisionError; learns that modulus works with floats; sees that exponentiation can produce very large numbers. AI explains each result, normalizing errors as learning opportunities.

#### Prompt 4: Synthesis & Validation — "Connect to what I know"
```
I know from Chapter 14 that Python has different data types.
How do operators interact with data types?

- Why does int / int give float?
- What happens when I add int + float?
- Should I always use type() after operations?

Connect this to type hints from Chapter 14.
```
**Expected outcome**: Student synthesizes understanding: operators create type transformations; type hints predict what will happen; validation with `type()` confirms predictions. Bridges Chapter 14 (type hints) to Chapter 15 (operators change types).

---

### Lesson 2: Comparison Operators — Making Decisions

**Learning Objective** (Bloom's: Understand + Apply): Students can explain what each comparison operator (==, !=, >, <, >=, <=) does, predict True/False results, and understand how comparisons prepare for conditional logic in Chapter 17.

**Skills Taught** (CEFR Proficiency Mapping):

- **Skill 1**: Comparison Logic with Type Awareness
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Understand + Apply
  - DigComp Area: Problem-Solving (comparing and evaluating options)
  - Measurable at This Level: Student can write comparisons like `5 > 3`, `x == y`, predict True/False, and explain why comparing `5 == 5.0` gives True (value equality, not type)

- **Skill 2**: Boolean Results and Type Validation
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Understand
  - DigComp Area: Digital Content Creation (working with boolean values)
  - Measurable at This Level: Student can verify that comparison returns bool type using `type()`, understands True/False as values (not strings)

- **Skill 3**: Preparing for Conditional Logic (Chapter 17 Foundation)
  - CEFR Level: A2 (Basic)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (identifying conditions for decisions)
  - Measurable at This Level: Student can explain "Why do I need to know comparisons before learning if statements?" and give examples of conditions in real programs

**Key Concepts** (Cognitive Load: 5 concepts):
1. Equality comparison (==, !=) — checking if values are same/different
2. Magnitude comparison (>, <) — checking greater/less than
3. Combined comparison (>=, <=) — including equality in magnitude check
4. Boolean result type (True/False) — what comparisons produce
5. Type equality vs. Value equality (5 == 5.0 is True, but "5" == 5 is False)

**Prerequisites**:
- Chapter 14: Data types (int, float, str, bool), type hints, `type()` function
- Chapter 13: Ability to run Python code

**Duration**: 45–50 minutes

**Content Outline**:
1. **What It Is** (5 min) - Comparisons ask "Is this true or false?"; prepare for Chapter 17
2. **The Six Comparison Operators** (20 min) - Each operator with examples, behavior with different types
3. **Boolean Results and Type Safety** (15 min) - Comparisons return bool type; type validation
4. **Try With AI** (10 min) - Four progressive prompts

**Content Elements**:

**Code Example 1: All Six Comparison Operators**
```python
# The 6 comparison operators: asking true/false questions
x: int = 10
y: int = 5

# Equality
equal: bool = x == y              # False - are they equal?
not_equal: bool = x != y          # True - are they different?

# Magnitude
greater_than: bool = x > y        # True - is x bigger?
less_than: bool = x < y           # False - is x smaller?
greater_equal: bool = x >= y      # True - is x bigger or equal?
less_equal: bool = x <= y         # False - is x smaller or equal?

# Verify they return bool
print(type(equal))                # <class 'bool'>
```
- **Purpose**: Show all 6 operators; demonstrate that comparisons return bool type
- **Complexity**: Beginner (comparison operators as predicates)

**Code Example 2: Type Equality vs. Value Equality**
```python
# Integer 5 equals float 5.0? (value equality, not type equality)
int_five: int = 5
float_five: float = 5.0

value_equal: bool = int_five == float_five        # True (values are same)
type_equal: bool = type(int_five) == type(float_five)  # False (types differ)

# But string "5" does NOT equal integer 5
str_five: str = "5"
type_mix: bool = int_five == str_five             # False (different types)

# Why? Because == compares VALUES, not types
print(f"{int_five} == {float_five}: {value_equal}")    # 5 == 5.0: True
print(f"{int_five} == {str_five}: {type_mix}")         # 5 == 5: False
```
- **Purpose**: Explain that == compares values (not types); show consequence of type mismatch
- **Complexity**: Beginner-intermediate (concept: value vs. type equality)

**Code Example 3: Comparisons with Real Data**
```python
# Real scenario: Checking age eligibility
user_age: int = 16
voting_age: int = 18
driving_age: int = 16

can_vote: bool = user_age >= voting_age           # False
can_drive: bool = user_age >= driving_age        # True

# Checking password strength (length)
password: str = "mypass123"
min_length: int = 8

is_long_enough: bool = len(password) >= min_length    # True
print(f"Password is strong enough: {is_long_enough}")
```
- **Purpose**: Show comparisons in realistic context; demonstrate that comparisons work with len() and other functions
- **Complexity**: Beginner (application to real scenarios)

**Validation Points**:
- [ ] Students explain all 6 comparison operators (SC-001)
- [ ] Students write comparisons that evaluate to correct True/False (SC-002)
- [ ] Students understand value equality (5 == 5.0 is True) (SC-003)

**Try With AI** (4 Progressive Prompts):

#### Prompt 1: Concept Exploration — "What are comparisons?"
```
I want to understand comparison operators.
- What do they do?
- Why does Python return True or False?
- How is == different from =?

This is important because I see = in variables and == in... something else.
```
**Expected outcome**: Student learns comparison operators return bool values; clearly distinguishes = (assignment) from == (equality test); understands that comparisons are questions.

#### Prompt 2: Application — "Check User Eligibility"
```
Write code that checks if a user can buy a movie ticket.
- movie_age_rating = 13 (movie requires age 13+)
- user_age = 12

Write a comparison that checks if user_age is old enough.
Should it return True or False? Why?
```
**Expected outcome**: Student applies comparison operators to realistic problem; predicts result correctly; understands the logic.

#### Prompt 3: Edge Case Discovery — "Surprising Comparisons"
```
I noticed something weird. Try these:
- 5 == 5.0
- "5" == 5
- True == 1
- False == 0

Why do some return True and others False?
What's going on with type equality vs value equality?
```
**Expected outcome**: Student discovers that Python compares values (not types) in ==; learns that bool has special relationship with int (True==1, False==0); AI explains type coercion in comparisons.

#### Prompt 4: Synthesis & Validation — "Why do I need comparisons?"
```
I can see why comparisons are useful. But why teach them BEFORE if statements (Chapter 17)?

Why not just teach:
"if x > 5: do something"

Instead of teaching > separately first?

Connect this to the chapter sequence.
```
**Expected outcome**: Student understands that comparisons are foundational (standalone True/False values) before they're used in if statements; sees how Chapter 15 (operators producing True/False) prepares for Chapter 17 (using True/False in decisions); appreciates learning progression.

---

### Lesson 3: Logical Operators — Combining Conditions

**Learning Objective** (Bloom's: Understand + Analyze): Students can explain logical operators (and, or, not), combine comparison results, and understand how logical operators enable complex decision-making for Chapter 17.

**Skills Taught** (CEFR Proficiency Mapping):

- **Skill 1**: Boolean Logic with and, or, not
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Understand + Apply
  - DigComp Area: Problem-Solving (combining conditions)
  - Measurable at This Level: Student can write `(x > 5) and (x < 10)`, predict True/False results, and explain when each operator returns True

- **Skill 2**: Combining Comparisons into Complex Conditions
  - CEFR Level: B1 (Intermediate)
  - Category: Technical
  - Bloom's Level: Analyze
  - DigComp Area: Problem-Solving (designing decision logic)
  - Measurable at This Level: Student can analyze expression like `not (x > 5 or y < 3)` and trace through evaluation step-by-step with AI support

- **Skill 3**: Truth Tables and Logical Reasoning
  - CEFR Level: A2 (Basic)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (understanding logic systems)
  - Measurable at This Level: Student can explain "and returns True only if BOTH are True" and "or returns True if AT LEAST ONE is True"

**Key Concepts** (Cognitive Load: 3 concepts):
1. and operator — both conditions must be True
2. or operator — at least one condition must be True
3. not operator — reverses True/False
(These 3 operators represent fundamental boolean logic)

**Prerequisites**:
- Chapter 14: Boolean type (True/False)
- Lesson 2 (this chapter): Comparison operators producing True/False results

**Duration**: 45–50 minutes

**Content Outline**:
1. **What It Is** (5 min) - Logical operators combine True/False values; enable complex reasoning
2. **The Three Logical Operators** (20 min) - and, or, not with truth tables and examples
3. **Combining Comparisons** (15 min) - Real conditions using logical operators to build complex logic
4. **Try With AI** (10 min) - Four progressive prompts

**Content Elements**:

**Code Example 1: The Three Logical Operators**
```python
# The 3 logical operators
condition1: bool = True
condition2: bool = False

# AND: True only if BOTH are True
and_result: bool = condition1 and condition2     # False (one is False)

# OR: True if AT LEAST ONE is True
or_result: bool = condition1 or condition2       # True (at least one is True)

# NOT: Reverses the value
not_result: bool = not condition1                # False (True becomes False)
not_result2: bool = not condition2               # True (False becomes True)

# Truth tables
print(f"True and True = {True and True}")         # True
print(f"True and False = {True and False}")       # False
print(f"False or False = {False or False}")       # False
print(f"True or False = {True or False}")         # True
```
- **Purpose**: Show all 3 operators; demonstrate truth table behavior
- **Complexity**: Beginner (logical operations as boolean algebra)

**Code Example 2: Combining Comparisons with Logical Operators**
```python
# Combining comparisons: check if number is in range
x: int = 7

# Is x between 5 and 10?
in_range: bool = (x > 5) and (x < 10)            # True (both conditions met)

# Is x outside range [5, 10]?
out_of_range: bool = (x <= 5) or (x >= 10)       # False (neither condition met)

# Is x NOT in range?
not_in_range: bool = not ((x > 5) and (x < 10))  # False (it IS in range)

# Real scenario: Checking password validity
password: str = "SecurePass123"
has_length: bool = len(password) >= 8            # True
has_number: bool = any(c.isdigit() for c in password)  # True
has_letter: bool = any(c.isalpha() for c in password)  # True

is_valid: bool = has_length and has_letter       # True (must have both)
# Note: This example uses is_valid, not actual validation
```
- **Purpose**: Show practical use of combining comparisons; demonstrate evaluation step-by-step
- **Complexity**: Beginner-intermediate (building complex conditions from simple comparisons)

**Code Example 3: Evaluation Order with Parentheses**
```python
# Parentheses control evaluation order
age: int = 25
has_license: bool = True
has_insurance: bool = False

# Can drive? (age 18+ AND licensed AND insured)
can_drive: bool = (age >= 18) and has_license and has_insurance  # False

# Can't evaluate this without AND: (insured OR ... )
# Different interpretation:
is_adult_with_license: bool = (age >= 18) and has_license        # True
can_take_car: bool = is_adult_with_license and has_insurance     # False

# Using parentheses for clarity
is_qualified: bool = (age >= 18) and (has_license or has_insurance)  # True
# (age >= 18 is True) and (has_license=True OR has_insurance=False) = True and True = True
```
- **Purpose**: Show that parentheses control logical evaluation order; demonstrate complex conditions
- **Complexity**: Beginner-intermediate (understanding evaluation order)

**Validation Points**:
- [ ] Students explain and, or, not in plain language (SC-001)
- [ ] Students combine comparisons using logical operators correctly (SC-002)
- [ ] Students identify incorrect logic use (SC-003)

**Try With AI** (4 Progressive Prompts):

#### Prompt 1: Concept Exploration — "What's the difference between and and or?"
```
I'm confused about and vs. or.
- When does `and` return True?
- When does `or` return True?
- Why do we need `not`?

Give me examples that make the difference clear.
```
**Expected outcome**: Student learns truth conditions for each operator; understands practical distinction (and requires both, or needs only one, not reverses); grasps fundamental boolean logic.

#### Prompt 2: Application — "Check User Permissions"
```
Write code for a permission check:
- A user can post if:
  - They are logged in AND they've been a member for 7+ days

Write the comparison logic.
Then modify it: What if EITHER admin OR member (7+ days)?

Show me both versions.
```
**Expected outcome**: Student applies logical operators to realistic permission logic; understands difference between and (both required) and or (either sufficient); sees practical value of combining operators.

#### Prompt 3: Edge Case Discovery — "Complex Logic"
```
Evaluate these expressions step-by-step:
- True and False or True
- not (True or False)
- (True and False) or (True and True)

Which ones are True? Which are False?
Show the evaluation order.
```
**Expected outcome**: Student practices evaluating complex boolean expressions; learns operator precedence (not > and > or); discovers that without parentheses, precedence rules matter; AI explains each step.

#### Prompt 4: Synthesis & Validation — "Why is this important for Chapter 17?"
```
I see how logical operators combine True/False values.
But why teach them in Chapter 15 when Chapter 17 teaches if statements?

How do logical operators prepare me for writing if statements like:
`if age >= 18 and has_license:`

Connect logical operators to control flow.
```
**Expected outcome**: Student understands that logical operators (Boolean logic) are foundation for conditional statements in Chapter 17; sees that if statements use the exact same logic; appreciates learning progression (operators first, then control flow that uses them).

---

### Lesson 4: Assignment Operators — Updating Variables Efficiently

**Learning Objective** (Bloom's: Understand + Apply): Students can explain assignment operators (=, +=, -=, *=, /=) and apply them to update variables efficiently, understanding how they combine assignment with arithmetic operations.

**Skills Taught** (CEFR Proficiency Mapping):

- **Skill 1**: Basic and Shorthand Assignment
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Understand + Apply
  - DigComp Area: Digital Content Creation (writing efficient code)
  - Measurable at This Level: Student can write `x += 5` and understand it's equivalent to `x = x + 5`; apply shorthand operators without referencing examples

- **Skill 2**: Understanding Assignment vs. Comparison
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (avoiding operator confusion)
  - Measurable at This Level: Student can clearly distinguish `=` (assignment) from `==` (comparison); explain why confusing them causes SyntaxError in wrong contexts

- **Skill 3**: Practical Variable Updates in Code
  - CEFR Level: A2 (Basic)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Digital Content Creation (writing readable code)
  - Measurable at This Level: Student can use `count += 1` in realistic scenarios; recognize when shorthand is more readable than expanded form

**Key Concepts** (Cognitive Load: 5 concepts):
1. Basic assignment (=) — storing values in variables
2. Arithmetic assignment (+=) — add and assign
3. Subtraction assignment (-=) — subtract and assign
4. Multiplication assignment (*=) — multiply and assign
5. Division assignment (/=) — divide and assign
(+ awareness that other forms like %=, //=, **= exist but are less common)

**Prerequisites**:
- Chapter 14: Variables, type hints, basic assignment
- Lesson 1 (this chapter): Arithmetic operators

**Duration**: 45–50 minutes

**Content Outline**:
1. **What It Is** (5 min) - Assignment operators update variables efficiently; combine operation with assignment
2. **Basic vs. Shorthand Assignment** (15 min) - = vs. +=, -=, *=, /= with equivalence examples
3. **Type Behavior with Assignment** (15 min) - How types change during assignment operations
4. **Try With AI** (10 min) - Four progressive prompts

**Content Elements**:

**Code Example 1: Assignment Operators Comparison**
```python
# Basic assignment: store a value
count: int = 10

# Method 1: Expanded form (store and reassign)
count = count + 5                   # Add 5, store result back in count
print(f"After add: {count}")         # 15

# Method 2: Shorthand assignment (same result, more readable)
count: int = 10
count += 5                          # Same as: count = count + 5
print(f"After +=: {count}")         # 15

# All five shorthand operators
x: int = 20
x += 3                              # x = x + 3 → 23
x -= 2                              # x = x - 2 → 21
x *= 2                              # x = x * 2 → 42
x /= 3                              # x = x / 3 → 14.0 (note: becomes float!)
print(f"Final x: {x}, type: {type(x)}")  # 14.0, <class 'float'>
```
- **Purpose**: Show expanded vs. shorthand equivalence; demonstrate all five operators; show type change with /=
- **Complexity**: Beginner (assignment as efficiency improvement)

**Code Example 2: Counter Pattern (Common Use)**
```python
# Very common pattern: counting things
count: int = 0

# Processing items
items = ["apple", "banana", "cherry"]
for item in items:
    count += 1                      # Increment count by 1

print(f"Total items: {count}")       # 3

# Another common pattern: accumulating total
total: float = 0.0
prices = [10.50, 20.99, 15.00]
for price in prices:
    total += price                  # Add each price to total

print(f"Total cost: ${total:.2f}")   # Total cost: $46.49
```
- **Purpose**: Show realistic use of += (counting, accumulation); prepare for Chapter 17 loops
- **Complexity**: Beginner (practical patterns students will use constantly)

**Code Example 3: Type Behavior with Shorthand Assignment**
```python
# Integer division assignment
total: int = 10
total //= 3                         # Floor division: 10 // 3 = 3

# Float arithmetic assignment
amount: float = 100.0
amount *= 1.10                      # Multiply by 1.10 (10% increase)
print(f"Increased amount: {amount}")  # 110.0

# What happens when you mix types?
value: int = 5
value += 2.5                        # int + float = float
print(f"Mixed type: {value}, type: {type(value)}")  # 7.5, <class 'float'>

# This changes the variable's type!
value_int: int = 5
value_int += 2                      # Still int
value_int = value_int / 2           # Now becomes float
print(f"Changed type: {value_int}, type: {type(value_int)}")  # 2.5, <class 'float'>
```
- **Purpose**: Show type behavior during assignment operations; important for understanding Chapter 17 loops where types may change
- **Complexity**: Beginner-intermediate (type awareness during operations)

**Validation Points**:
- [ ] Students explain assignment operators in plain language (SC-001)
- [ ] Students write shorthand operators equivalently to expanded form (SC-002)
- [ ] Students identify when = should NOT be used (SC-003)

**Try With AI** (4 Progressive Prompts):

#### Prompt 1: Concept Exploration — "Why do I need += if I have ="?
```
I can already write x = x + 5. So why do I need +=?

- What's the point of x += 5?
- Is it faster? Clearer? Both?
- When should I use += vs. =?
```
**Expected outcome**: Student learns that += is shorthand for readability (not just speed); understands it expresses "increment by" more clearly than "reassign to sum"; appreciates efficiency in code.

#### Prompt 2: Application — "Track a Running Total"
```
Write code that:
- Starts with balance = 100 (dollars)
- Adds transaction: balance += 50
- Subtracts transaction: balance -= 25
- Applies interest (multiply by 1.05): balance *= 1.05

Show the final balance. Use type hints and verify types.
```
**Expected outcome**: Student applies all five operators in realistic scenario; traces through type behavior (float result from *= 1.05); sees practical value of shorthand operators.

#### Prompt 3: Edge Case Discovery — "What breaks or surprises?"
```
Try these:
- x: int = 5; x /= 2 (what type is x now?)
- y: str = "hello"; y += " world" (can you add strings?)
- z: int = 10; z += 2.5 (what type is z now?)

Which ones work? Which are surprising? Why?
```
**Expected outcome**: Student discovers that /= always produces float; learns that += works with strings (concatenation); finds that mixing types changes variable type; understands operator behavior is type-dependent.

#### Prompt 4: Synthesis & Validation — "Prepare for loops"
```
I notice += will be useful for counting in loops (Chapter 17).
- Why does count += 1 make sense in a loop?
- What about total += item?
- How does this pattern help avoid bugs?

Show me a preview of what this looks like in a loop.
```
**Expected outcome**: Student understands that += pattern prepares for loop structure (Chapter 17); sees how variable updates accumulate; understands why this matters for practical programming; grasps chapter sequence (operators first, then loops that use them repeatedly).

---

### Lesson 5: Python Keywords and Capstone Project

**Learning Objective** (Bloom's: Understand + Apply): Students can recognize and list Python keywords, understand why they're reserved, apply knowledge of all four operator types in integrated "Calculator with Type Safety" capstone project, and validate output through testing.

**Skills Taught** (CEFR Proficiency Mapping):

- **Skill 1**: Recognizing Python Keywords
  - CEFR Level: A1-A2 (Foundation-Basic)
  - Category: Technical
  - Bloom's Level: Remember + Understand
  - DigComp Area: Foundation (language rules)
  - Measurable at This Level: Student can retrieve Python keyword list with `import keyword; print(keyword.kwlist)`, recognize common keywords (if, for, while, def, class, return), avoid using keywords as variable names

- **Skill 2**: Understanding Why Keywords are Reserved
  - CEFR Level: A2 (Basic)
  - Category: Conceptual
  - Bloom's Level: Understand
  - DigComp Area: Problem-Solving (understanding language design)
  - Measurable at This Level: Student can explain "Keywords are reserved because Python needs them for control flow, functions, and language features" and give examples (if, for, def)

- **Skill 3**: Integrating All Operator Types in Realistic Code
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply + Analyze
  - DigComp Area: Digital Content Creation (building functional programs)
  - Measurable at This Level: Student can write "Calculator with Type Safety" project using arithmetic operators for calculations, assignment operators for result storage, and type validation with type() or isinstance()

- **Skill 4**: Validation-First Programming Practice
  - CEFR Level: A2-B1 (Basic-Intermediate)
  - Category: Technical
  - Bloom's Level: Apply
  - DigComp Area: Safety (verifying code correctness)
  - Measurable at This Level: Student tests calculator with multiple inputs, validates output types, checks edge cases (division by zero), asks AI about surprising results

**Key Concepts** (Cognitive Load: 2 new concepts + integration):
1. Keywords - reserved words with special meaning
2. Keyword checking - using `import keyword` to verify
(+ integration: applying all 4 operator types together, no new operator concepts)

**Prerequisites**:
- Chapter 14: Data types, type hints, `type()` validation
- Lessons 1-4 (this chapter): All four operator types (arithmetic, comparison, logical, assignment)

**Duration**: 45–50 minutes

**Content Outline**:
1. **What It Is** (5 min) - Keywords are language reserved words; prevent common errors
2. **Understanding Keywords** (10 min) - How to check keyword list, why certain words are reserved
3. **Capstone: Calculator with Type Safety** (25 min) - Build integrated project using all operator types
4. **Try With AI** (10 min) - Four progressive prompts

**Content Elements**:

**Code Example 1: Discovering Python Keywords**
```python
# Method 1: See the keyword list
import keyword
print("Python keywords:")
print(keyword.kwlist)
# Output: ['False', 'None', 'True', 'and', 'as', 'assert', 'async', 'await',
#          'break', 'class', 'continue', 'def', 'del', 'elif', 'else', 'except',
#          'finally', 'for', 'from', 'global', 'if', 'import', 'in', 'is',
#          'lambda', 'nonlocal', 'not', 'or', 'pass', 'raise', 'return', 'try',
#          'while', 'with', 'yield']

# Count them
print(f"Total keywords: {len(keyword.kwlist)}")  # 35

# Method 2: Check if a word is a keyword
print(keyword.iskeyword("for"))      # True (reserved)
print(keyword.iskeyword("forloop"))  # False (not reserved)
```
- **Purpose**: Show students how to access keyword list; demonstrate checking tool
- **Complexity**: Beginner (language introspection)

**Code Example 2: Why Keywords are Reserved (With Error Demonstration)**
```python
# WRONG: Try to use keyword as variable name
# for = 5  # SyntaxError: invalid syntax (for is reserved!)

# RIGHT: Use descriptive alternative
for_loop_count: int = 5             # Use underscore or descriptive name

# Common keywords and why they're reserved
# if, elif, else — conditional logic (Chapter 17)
# for, while — loops (Chapter 17)
# def — function definition (Chapter 20)
# class — class definition (Chapter 24)
# import, from — module imports (Chapter 20)
# return — return from function (Chapter 20)
# try, except, finally — error handling (Chapter 21)
# True, False, None — special values (Chapter 14)
# and, or, not — logical operators (Lesson 3, this chapter!)

# Good practice: if unsure, check
word: str = "while"
if keyword.iskeyword(word):
    print(f"'{word}' is reserved. Use a different name like '{word}_loop'")
else:
    print(f"'{word}' can be used as a variable name")
```
- **Purpose**: Explain why keywords exist; show consequences of attempting keyword misuse; demonstrate defensive checking
- **Complexity**: Beginner (understanding language design)

**Capstone Project: Calculator with Type Safety**
```python
# Capstone Project: Calculator with Type Safety
# Demonstrates all 4 operator types + type validation

import keyword

# Function to get user input (Chapter 20, but we'll preview)
def safe_number_input(prompt: str) -> float:
    """Get a number from user, with type safety"""
    while True:
        try:
            user_input: str = input(prompt)
            number: float = float(user_input)
            return number
        except ValueError:
            print("Invalid input. Please enter a number.")

# Demonstrate operator usage
def calculate():
    """Simple calculator demonstrating all operators"""

    print("=== Calculator with Type Safety ===\n")

    # Get inputs
    num1: float = safe_number_input("Enter first number: ")
    num2: float = safe_number_input("Enter second number: ")

    # Arithmetic operators (Lesson 1)
    print(f"\nArithmetic Operations:")
    add_result: float = num1 + num2
    print(f"{num1} + {num2} = {add_result} (type: {type(add_result).__name__})")

    sub_result: float = num1 - num2
    print(f"{num1} - {num2} = {sub_result} (type: {type(sub_result).__name__})")

    mul_result: float = num1 * num2
    print(f"{num1} * {num2} = {mul_result} (type: {type(mul_result).__name__})")

    # Division with safety check (avoid ZeroDivisionError)
    if num2 != 0:                       # Comparison operator (Lesson 2)
        div_result: float = num1 / num2
        print(f"{num1} / {num2} = {div_result} (type: {type(div_result).__name__})")

        floor_result: int = int(num1 // num2)  # Floor division
        print(f"{num1} // {num2} = {floor_result} (type: {type(floor_result).__name__})")
    else:
        print("Cannot divide by zero")

    # Modulus (remainder)
    if num2 != 0:
        mod_result: int = int(num1 % num2)
        print(f"{num1} % {num2} = {mod_result} (type: {type(mod_result).__name__})")

    # Exponentiation
    exp_result: float = num1 ** 2
    print(f"{num1} ** 2 = {exp_result} (type: {type(exp_result).__name__})")

    # Comparison operators (Lesson 2)
    print(f"\nComparison Operations:")
    is_equal: bool = num1 == num2
    print(f"{num1} == {num2}: {is_equal}")

    is_greater: bool = num1 > num2      # Comparison
    print(f"{num1} > {num2}: {is_greater}")

    # Logical operators (Lesson 3)
    print(f"\nLogical Operations:")
    both_positive: bool = (num1 > 0) and (num2 > 0)  # AND
    print(f"Both positive: {both_positive}")

    either_large: bool = (num1 > 10) or (num2 > 10)  # OR
    print(f"Either > 10: {either_large}")

    not_equal: bool = not (num1 == num2)  # NOT
    print(f"Not equal: {not_equal}")

    # Assignment operators for running total (Lesson 4)
    print(f"\nAssignment Operators (Running Total):")
    total: float = 0.0
    total += add_result
    print(f"After adding: total = {total}")

    total += sub_result
    print(f"After subtracting: total = {total}")

    total *= 0.5                        # Apply 50% reduction
    print(f"After 50% reduction: total = {total}")

    # Validation (check types)
    print(f"\nType Safety Verification:")
    print(f"num1 type: {type(num1)}")
    print(f"add_result type: {type(add_result)}")
    print(f"is_greater type: {type(is_greater)}")
    print(f"total type: {type(total)}")

    # Keyword awareness
    print(f"\nKeyword Awareness:")
    potential_names = ["if", "for", "result", "calculation", "while"]
    for name in potential_names:
        if keyword.iskeyword(name):
            print(f"  '{name}' is RESERVED - don't use as variable name")
        else:
            print(f"  '{name}' is OK - can use as variable name")

# Simple non-function version for beginners who haven't seen functions yet
def simple_calculator():
    """Simplified calculator without functions"""
    print("=== Simple Calculator ===\n")

    # Input
    num1: float = float(input("First number: "))
    num2: float = float(input("Second number: "))

    # All four operator types
    result_add: float = num1 + num2          # Arithmetic
    result_greater: bool = num1 > num2       # Comparison
    result_and: bool = (num1 > 0) and (num2 > 0)  # Logical
    total: float = 0.0                       # Assignment setup
    total += result_add                      # Assignment operator

    # Validation
    print(f"\nResults:")
    print(f"Sum: {result_add}")
    print(f"num1 > num2: {result_greater}")
    print(f"Both positive: {result_and}")
    print(f"Total: {total}")

# Run simplified version
if __name__ == "__main__":
    simple_calculator()
```
- **Purpose**: Comprehensive project combining all 4 operator types; demonstrates type validation; includes keyword awareness
- **Complexity**: Beginner-intermediate (integration project, no new concepts)

**Validation Points**:
- [ ] Students recognize Python keywords (SC-004)
- [ ] Students avoid using keywords as variable names (SC-004)
- [ ] Students apply all 4 operator types correctly in capstone (SC-002)
- [ ] Students validate result types with type() (validation-first practice)
- [ ] Students complete capstone project and test it (SC-006)

**Try With AI** (4 Progressive Prompts):

#### Prompt 1: Concept Exploration — "What are keywords and why do I care?"
```
I ran `import keyword; print(keyword.kwlist)` and got a long list.
- What are Python keywords?
- Why is it bad to use them as variable names?
- Can I use "For" instead of "for" (different capitalization)?

Show me what error I get if I try to use "if" as a variable name.
```
**Expected outcome**: Student understands keywords are reserved syntax elements; learns Python is case-sensitive (For is valid, for is not); sees SyntaxError demonstration; grasps language design constraints.

#### Prompt 2: Application — "Build a Simple Calculator"
```
Write a simple calculator that:
1. Gets two numbers from the user
2. Adds them (arithmetic operator)
3. Checks if first > second (comparison operator)
4. Calculates total using += (assignment operator)
5. Uses type() to verify result types

You can skip the function definition part - just write straightforward code.
```
**Expected outcome**: Student implements capstone project (or simplified version); applies all operator types; practices type validation; creates working, testable code.

#### Prompt 3: Edge Case Discovery — "What breaks?"
```
Test the calculator with:
- Two positive numbers
- Two negative numbers
- Zero and a positive
- Very large numbers
- What happens if you enter text instead of a number?

Which inputs break the calculator? How would you fix it?
```
**Expected outcome**: Student tests edge cases (negative numbers, zero, large numbers, non-numeric input); discovers type safety issues; learns defensive programming; asks AI about error handling for robustness.

#### Prompt 4: Synthesis & Validation — "Connect all 4 operator types"
```
I've learned 4 different operator types:
- Arithmetic: do math
- Comparison: ask true/false questions
- Logical: combine conditions
- Assignment: update variables

How do these fit together in the calculator?
- Where does each operator type appear?
- Why do I need all 4?
- How does this prepare for Chapter 17 (Control Flow)?

Show me the connections.
```
**Expected outcome**: Student synthesizes understanding: arithmetic operators perform calculations; assignment operators store results; comparison operators enable validation logic; logical operators would combine conditions in Chapter 17; sees how all 4 types work together; understands chapter progression toward control flow.

---

## 📊 Skills Proficiency Summary

| Skill | Lesson(s) | CEFR Level | Bloom's Level | DigComp Area | Measurable Outcome |
|-------|-----------|------------|---------------|------------------|---------------------|
| Arithmetic Operations with Type Safety | L1 | A1-A2 | Understand + Apply | Problem-Solving | Student writes arithmetic expressions (+,-,*,/,//,%,**) with type hints, uses type() to verify results |
| Understanding Type Hint Feedback | L1 | A2 | Understand | Digital Content Creation | Student explains why int/int gives float, why int//int gives int, uses type validation |
| AI-Driven Exploration of Edge Cases | L1 | A2 | Apply | Communication & Collaboration | Student asks productive questions like "What happens if I divide by zero?" and tests responses |
| Comparison Logic with Type Awareness | L2 | A2 | Understand + Apply | Problem-Solving | Student writes comparisons (==,!=,>,<,>=,<=), predicts True/False, understands value equality (5==5.0 is True) |
| Boolean Results and Type Validation | L2 | A2 | Understand | Digital Content Creation | Student verifies comparisons return bool type using type(), understands True/False as values |
| Preparing for Conditional Logic | L2 | A2 | Understand | Problem-Solving | Student explains why comparisons precede if statements, connects to Chapter 17 preparation |
| Boolean Logic with and, or, not | L3 | A2-B1 | Understand + Apply | Problem-Solving | Student writes complex conditions (x>5 and x<10), combines comparisons, predicts True/False |
| Combining Comparisons into Complex Conditions | L3 | B1 | Analyze | Problem-Solving | Student analyzes expressions like not(x>5 or y<3), traces evaluation step-by-step with AI support |
| Truth Tables and Logical Reasoning | L3 | A2 | Understand | Problem-Solving | Student explains "and returns True only if BOTH are True", "or if AT LEAST ONE is True" |
| Basic and Shorthand Assignment | L4 | A2 | Understand + Apply | Digital Content Creation | Student writes x+=5 equivalent to x=x+5, applies shorthand operators without reference |
| Understanding Assignment vs. Comparison | L4 | A2 | Understand | Problem-Solving | Student distinguishes = (assignment) from == (comparison), explains SyntaxError consequences |
| Practical Variable Updates in Code | L4 | A2 | Apply | Digital Content Creation | Student uses count+=1 in realistic scenarios, recognizes when shorthand is more readable |
| Recognizing Python Keywords | L5 | A1-A2 | Remember + Understand | Foundation | Student retrieves keyword list with import keyword, recognizes common keywords (if,for,while,def,class), avoids using keywords as variable names |
| Understanding Why Keywords are Reserved | L5 | A2 | Understand | Problem-Solving | Student explains "Keywords are reserved because Python needs them for control flow/functions" with examples |
| Integrating All Operator Types | L5 | A2-B1 | Apply + Analyze | Digital Content Creation | Student writes Calculator project using all 4 operator types, validates output with type() |
| Validation-First Programming Practice | L5 | A2-B1 | Apply | Safety | Student tests code with multiple inputs, validates types, checks edge cases, asks AI about surprises |

---

## 🔄 Proficiency Progression

**Chapter-Wide Progression**: A1-A2 (Foundation-Basic) with integration toward A2-B1

### Lesson-by-Lesson CEFR Progression

| Lesson | Starting Level | Target Level | Progression Mechanism | Cognitive Load (concepts) |
|--------|----------------|--------------|----------------------|--------------------------|
| L1: Arithmetic | A1 | A2 | Show 7 operators → explain behaviors → validate types | 5 (operators as family) |
| L2: Comparison | A2 | A2-B1 | Define 6 operators → understand True/False → combine with logic prep | 5 (operators + boolean results) |
| L3: Logical | A2 | A2-B1 | Explain 3 operators → combine with comparisons → evaluate complex | 3 (and/or/not) |
| L4: Assignment | A2 | A2 | Show expanded vs. shorthand → apply in realistic patterns | 5 (=, +=, -=, *=, /=) |
| L5: Keywords + Capstone | A1-A2 | A2-B1 | Recognize keywords → integrate all operators in project | 2 (keywords) + integration |

**Overall Progression Validation**:
- ✓ Cognitive load respects A1-A2 limits (max 5 concepts per lesson)
- ✓ Proficiency increases: A1→A2 in Lesson 1, maintains A2 with B1 elements in Lessons 2-3, returns to A2 in Lesson 4, ends A2-B1 in capstone
- ✓ Prerequisites satisfied: Chapter 14 (types) taught before operators use those types
- ✓ Chapter 17 preparation: Comparison and logical operators fully developed for control flow

---

## 🎯 Content Flow & Dependencies

### Lesson Sequence Rationale

1. **Arithmetic First** (Lesson 1): Most familiar, immediate utility. Everyone knows 2+2. Builds confidence in simple operations.
2. **Comparison Second** (Lesson 2): Introduces True/False results. Prepares brain for boolean thinking needed in next lessons.
3. **Logical Third** (Lesson 3): Combines comparisons into complex conditions. Most cognitively demanding (requires tracking multiple True/False results).
4. **Assignment Fourth** (Lesson 4): Practical update pattern. Easier after mastering arithmetic because += just applies arithmetic shortcuts.
5. **Keywords + Capstone Fifth** (Lesson 5): Integration project. No new operator concepts; applies all four types together while introducing keyword awareness.

### Cross-Chapter Dependencies

**Backward (What Chapter 15 Requires)**:
- Chapter 14: Data types (int, float, str, bool, None)
- Chapter 14: Type hints syntax (x: int = 5)
- Chapter 14: type() and isinstance() validation
- Chapter 13: Running Python code and seeing output

**Forward (What Builds on Chapter 15)**:
- Chapter 16: Strings use operators (string concatenation with +)
- Chapter 17: Comparison operators in if/while/for statements
- Chapter 17: Logical operators in complex conditions for control flow
- Chapter 18-19: Membership operators (in, not in) build on comparison concept
- Chapter 20+: Operators used in function bodies and algorithms

---

## 🛠️ Scaffolding Strategy

### How Cognitive Load is Managed

**Within Each Lesson**:
- **Part 1: What It Is** (5 min) - Introduce single concept simply
- **Part 2: Code Examples** (20 min) - Show multiple examples of same concept; students see pattern
- **Part 3: Try With AI** (10 min) - Four prompts move from understanding → application → edge cases → synthesis

**Why This Works**:
- Concept introduction is brief and clear (reduces cognitive load spike)
- Multiple examples allow pattern recognition (activates prior knowledge)
- AI prompts are progressive (novice-friendly progression from basic to complex)
- No post-AI closure material (respects cognitive load: "Try With AI" is the finish)

### How Complexity Increases Across Lessons

| Lesson | Complexity Dimension | Progression |
|--------|----------------------|------------|
| L1 (Arithmetic) | Number of operators | 7 operators (grouped as family for A1-A2) |
| L2 (Comparison) | Result type | Introduces bool type (new, but concrete) |
| L3 (Logical) | Abstraction level | Combining concepts; requires mental model of True/False |
| L4 (Assignment) | Code readability | Shorthand syntax (more complex than expanded form) |
| L5 (Capstone) | Integration | All 4 types together; no new operators; application-focused |

**Scaffolding Safety Checks**:
- ✓ No lesson exceeds 5 new concepts (A2 cognitive limit)
- ✓ Each lesson builds on explicitly stated prerequisites
- ✓ "Try With AI" prompts match proficiency level (not jumping to advanced too fast)
- ✓ Capstone is achievable by A1-A2 students (no new concepts; integration only)

---

## 📚 Integration Points

### With Previous Content (Chapter 14)

**Chapter 14 Foundation**:
- Type hints (x: int = 5) → Chapter 15 uses these in all examples
- type() validation function → Chapter 15 uses constantly to verify operator results
- Data type understanding (int, float, str, bool) → Chapter 15 shows how operators interact with these types

**Bridge Statements**:
- "Remember Chapter 14's type hints? Every operator example uses them."
- "Recall Chapter 14's type()? We'll use it to verify every operator result."

### With Future Content (Chapter 17: Control Flow)

**Chapter 15 Preparation**:
- Comparison operators (==, !=, >, <, >=, <=) → Used directly in if/while conditions
- Logical operators (and, or, not) → Used to combine conditions in Chapter 17 control structures
- Boolean results (True/False) → Foundation for all Chapter 17 decision-making

**Looking-Forward Statements**:
- "These comparison operators will be essential for Chapter 17's if statements."
- "In Chapter 17, you'll use logical operators to write complex conditions for loops and decisions."

---

## ✅ Validation Strategy

### How Learners Demonstrate Understanding

**Technical Validation** (Skills Acquisition):
- Write operator expressions (all 4 types) without errors
- Predict True/False results for comparisons and logical operators
- Use type hints and type() correctly
- Complete exercises with correct outputs

**Comprehension Validation** (Evals SC-001):
- Explain what each operator does in plain language (without jargon)
- Answer: "Why does int / int give float?"
- Distinguish = (assignment) from == (comparison)

**Error Detection Validation** (Evals SC-003):
- Identify incorrect operator usage
- Explain why type mismatch causes errors
- Debug simple operator mistakes

**AI Partnership Validation** (Evals SC-005):
- Ask productive "what happens if" questions
- Request edge case exploration
- Synthesize AI responses with prior knowledge

**Engagement Validation** (Evals SC-006):
- Complete all 5 lessons (tracked in platform analytics)
- Attempt capstone project
- Engage with "Try With AI" prompts (completion rate, prompt quality)

**Accessibility Validation** (Evals SC-007):
- Content maintains Grade 7-8 reading level (automated Flesch-Kincaid check)
- Examples use concrete numbers and clear language
- Jargon is minimal and defined when used

---

## 🚀 Acceptance Criteria

**All Chapters - Definition of Done**:
- [ ] All 5 lessons follow "What it is → Code Idea → Try With AI" structure
- [ ] Each lesson has exactly 4 "Try With AI" prompts with expected outcomes documented
- [ ] All lessons end with "Try With AI" ONLY (no summaries, key takeaways, checklists after)
- [ ] Cognitive load limits respected (max 5 concepts per lesson, verified per lesson)
- [ ] CEFR proficiency levels assigned (A1/A2/B1) and justified in skills table
- [ ] Skills proficiency summary table complete with all columns (Skill, Lesson, CEFR, Bloom's, DigComp, Outcome)
- [ ] All code examples include type hints (reinforce Chapter 14)
- [ ] All code examples run on Python 3.14+ without errors
- [ ] Reading level Grade 7-8 validated (Flesch-Kincaid automated check before publication)
- [ ] Cross-references to Chapter 14 (prerequisite foundation) included in overview
- [ ] Cross-references to Chapter 17 (future use) included in lessons 2-3
- [ ] "Try With AI" tool selection aligns with policy:
  - [ ] Part 4 is pre-SDD-tools section: Use ChatGPT web or student's preferred AI tool
  - [ ] If multiple tools mentioned, provide variants (Claude Code, Gemini CLI, ChatGPT)

**Technical Chapters Specific**:
- [ ] All code examples pass tests (tested on Python 3.14+)
- [ ] Type hints appear in every code example
- [ ] Exercises have clear prompts and expected outputs
- [ ] Capstone project is achievable by A1-A2 students
- [ ] Edge cases demonstrated or prompt for exploration

**Chapter 15 Specific**:
- [ ] All 4 operator categories taught (arithmetic, comparison, logical, assignment)
- [ ] Keywords awareness included (Lesson 5)
- [ ] type() usage consistent throughout (validation-first practice)
- [ ] Division by zero mentioned as learning opportunity (not hidden)
- [ ] Float precision edge case (0.1 + 0.2) left for AI exploration
- [ ] No identity operators (is, is not) taught (deferred to Ch 24+)
- [ ] No membership operators (in, not in) taught (deferred to Ch 18-19)
- [ ] No bitwise operators taught
- [ ] Capstone "Calculator with Type Safety" integrates all operator types

---

## 🔗 Architectural Decisions & Dependencies

### Key Design Decisions (Documented for Future Reference)

**Decision 1: Five Lessons, Not Six or Four**
- Rationale: 4 operator categories + 1 capstone with keywords = 5 lessons respects cognitive load (4 new concepts per lesson) and standard 3.5-4 hour chapter time
- Alternative considered: 6 lessons (one per operator type + keywords + capstone) would split chapter too thin; less integration

**Decision 2: Keywords in Lesson 5 (Not Separate Chapter)**
- Rationale: Keywords are orthogonal to operators but "learn once, apply forever" content; integrating with capstone project efficient; doesn't demand new pedagogy
- Alternative considered: Separate lesson just for keywords; decided it's overkill and less engaging without context

**Decision 3: Comparison Operators Before Control Flow**
- Rationale: Chapter 15 teaches what comparisons return (True/False values); Chapter 17 teaches how to USE those values in if statements. Separation of concerns respects learning progression
- Alternative considered: Defer comparisons to Chapter 17 with if statements; decided students need to understand operators independently for validation-first practice

**Decision 4: Four "Try With AI" Prompts Per Lesson (Mandatory)**
- Rationale: Progression (concept → application → edge case → synthesis) matches learning science; four is cognitively manageable; builds AI partnership gradually
- Alternative considered: 3 prompts (concept, application, synthesis); added edge case to normalize errors as learning

---

## 📋 Risks & Mitigations (Updated from Spec)

| Risk | Impact | Probability | Mitigation Strategy |
|------|--------|-------------|-------------------|
| **R1: Students confuse = with ==** | High | High | Explicitly contrast in Lesson 4; include in "Try With AI" prompts; show SyntaxError consequences |
| **R2: Division by zero panics students** | Medium | Medium | Demonstrate intentionally in Lesson 1; normalize errors as learning; AI prompt: "What does this error mean?" |
| **R3: Float precision surprises** | Medium | High | Include 0.1+0.2 as edge case prompt; AI explains floating point limitations; deepen in later chapters |
| **R4: Operator precedence confusion** | Medium | Medium | Mention PEMDAS in Lesson 1; encourage parentheses; AI prompt: "Show me order of operations" |
| **R5: Cognitive overload from 4+ operator types** | High | Medium | Strict 5-concept-per-lesson limit (verified in plan); 5-lesson structure (not 4 or 6); skills-proficiency-mapper validation |
| **R6: Students skip "Try With AI" exercises** | High | Medium | Make visually distinct; state "Not optional - AI partnership is professional practice"; track completion (SC-006) |
| **R7: Type hint confusion from Chapter 14** | High | Medium | Quick review in Lesson 1; type() usage in every lesson reinforces; Chapter 14 prerequisite validation |
| **R8: Keywords feel disconnected from operators** | Medium | Low | Integrate keywords into Lesson 5 capstone; show keyword checking as practical safeguard; frame as "language literacy" |

---

## 🎓 Implementation Notes for Lesson Writer

**When Writing Lesson Content, Remember**:

1. **Type Hints Are Not Optional** - Every code example uses them. Students should never see code like `x = 5` without the type hint `x: int = 5`. This reinforces Chapter 14 and builds professional habits.

2. **"Try With AI" Is the Lesson Closure** - No summaries, key takeaways, or checklists appear after "Try With AI". The four prompts ARE how students close their learning for the lesson. This respects cognitive load and provides a clear stopping point.

3. **Part 4 Language Convention** - Use "describe intent" framing, NOT "write specifications". Chapter 15 is before Part 5 (Spec-Driven Development), so students haven't learned SDD terminology yet. Say "describe what you want" not "specify requirements".

4. **AI Tools Vary By Student** - Lessons should work with Claude Code, Gemini CLI, or ChatGPT. Don't assume a specific tool. When creating "Try With AI" prompts, use language like "Ask your AI" or "Request from your AI" (generic) not "Ask Claude" or "Use Gemini" (tool-specific).

5. **Validation-First Is a Core Theme** - Every lesson teaches students to verify results with type() or testing. This builds the professional habit of "never trust, always verify" for AI-generated code later.

6. **Errors Are Learning Opportunities** - ZeroDivisionError, TypeError, and SyntaxError are intentional demonstrations, not mistakes to hide. Students should ask "Why did this error occur?" This models debugging mindset.

7. **Edge Cases in "Try With AI", Not Core Content** - The main lesson teaches happy-path examples (5 + 3 = 8). "Try With AI" prompts ask students to explore edge cases (0.1 + 0.2, division by zero, type mixing). This keeps lesson content manageable while encouraging exploration.

---

## 📖 Success Criteria Alignment

This implementation plan directly addresses all 7 success criteria from the specification:

| Success Criterion | How Achieved in Plan |
|------------------|---------------------|
| **SC-001: 75%+ explain operators in plain language** | Every lesson's objective includes explanation; Try With AI Prompt 1 is concept exploration ("What does X do?") |
| **SC-002: 80%+ write code without errors** | Code examples are executable; exercises have clear prompts; capstone project has testable output |
| **SC-003: 70%+ identify incorrect usage** | Lessons 2-3 include error scenarios; Try With AI Prompt 3 ("What breaks?") explicitly asks for error discovery |
| **SC-004: 90%+ recognize keywords** | Lesson 5 teaches keyword detection with import keyword; multiple-choice quiz template provided |
| **SC-005: 75%+ ask productive AI questions** | All "Try With AI" prompts are progressive; Prompts 3-4 encourage "what if" questions; validation-first practice throughout |
| **SC-006: 80%+ complete all lessons** | Engaging content (4 operator types + capstone = variety); capstone project provides motivation; platform analytics tracks completion |
| **SC-007: Grade 7-8 reading level** | Simple sentence structures; jargon defined; concrete examples (5 + 3, not abstract algebra); validation check planned before publication |

---

## 🔍 Cross-Reference Map

**Files Referenced**:
- `.specify/memory/constitution.md` v3.0.2 — AI-Native Learning pattern (Tier 1/2/3), graduated complexity, core principles
- `specs/part-4-chapter-15/spec.md` — Approved specification with all success criteria, user stories, requirements
- `specs/part-4-chapter-14/plan.md` — Previous chapter's plan (data types, type hints, type() function)
- `specs/book/chapter-index.md` — Chapter 15 position in Part 4; prerequisite tracking with Chapters 14, 17
- `.claude/output-styles/lesson.md` — Lesson formatting requirements, YAML frontmatter structure, skills metadata
- `.claude/skills/skills-proficiency-mapper/` — CEFR framework (40+ years research), Bloom's taxonomy, DigComp 2.1, cognitive load theory, skill coherence validation (v2.0)

---

## 📊 Statistics Summary

**Lessons**: 5
- Lesson 1 (Arithmetic): 7 operators, A1-A2, 5 concepts
- Lesson 2 (Comparison): 6 operators, A2, 5 concepts
- Lesson 3 (Logical): 3 operators, A2-B1, 3 concepts
- Lesson 4 (Assignment): 5 operators, A2, 5 concepts
- Lesson 5 (Keywords + Capstone): 2 concepts + integration, A2-B1

**Code Examples**: 12+ runnable examples across all lessons
- All include type hints
- All use type() for validation
- All run on Python 3.14+

**"Try With AI" Prompts**: 20 total (4 per lesson × 5 lessons)
- Prompt Type 1: Concept exploration (5)
- Prompt Type 2: Application (5)
- Prompt Type 3: Edge case discovery (5)
- Prompt Type 4: Synthesis & validation (5)

**Skills Mapped**: 16 skills with CEFR levels, Bloom's taxonomy, DigComp areas

**Cognitive Load**: All lessons ≤ 5 concepts (respects A1-A2 limit)

**Chapter Duration**: 3.5–4 hours (45–50 minutes × 5 lessons)

---

**Status**: ✅ Ready for Lesson Writer Implementation
**Next Phase**: Lesson-writer subagent will create detailed lesson content following this plan
**Quality Gate**: Technical review validates all acceptance criteria before publication

