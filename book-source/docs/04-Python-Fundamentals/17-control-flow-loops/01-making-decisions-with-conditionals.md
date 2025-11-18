---
sidebar_position: 1
title: "Making Decisions with Conditionals"
description: "Learn to use if, elif, and else statements to make decisions in Python programs"
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Conditional Logic (if/elif/else)"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write if/elif/else statements to make decisions based on multiple conditions in guided contexts"

  - name: "Boolean Expression Evaluation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate boolean expressions and predict True/False outcomes for comparison operations"

  - name: "Indentation & Code Structure"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student understands Python's indentation rules and applies them correctly in conditional blocks"

  - name: "Type-Safe Decision Making"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write conditionals with appropriate type hints and type-safe comparisons"

  - name: "Comparison Operators Review"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can apply comparison operators (==, !=, >, <, >=, <=) in conditional expressions"

  - name: "Logical Operators (and/or/not)"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can combine multiple conditions using and, or, and not operators"

learning_objectives:
  - objective: "Write conditional logic using if, elif, and else statements to make decisions based on multiple conditions"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Coding exercises requiring multi-condition decision logic"

  - objective: "Apply comparison and logical operators appropriately in conditional expressions"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Evaluation of boolean expressions in code examples and exercises"

  - objective: "Validate conditional logic by testing edge cases and boundary values"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Testing of conditional statements with edge case inputs"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (if, else, elif, comparison operators review, logical operators review, boolean expressions, indentation rules) at A2 limit ✓"

differentiation:
  extension_for_advanced: "Explore nested conditionals with complex logical expressions; research Python 3.10+ match-case as alternative to if/elif chains"
  remedial_for_struggling: "Focus on simple if/else binary decisions first; practice with flowchart visualizations before code; use concrete real-world scenarios (age verification, price calculation)"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-17/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "content-implementer subagent"
version: "1.0.0"
---

# Making Decisions with Conditionals

Every program you write needs to make decisions. Should we display an error message? Is the user old enough to access this content? Did the payment go through successfully? These yes-or-no questions drive the logic of your programs.

In this lesson, you'll learn how Python makes decisions using **conditional statements**—code that runs only when certain conditions are true. You'll start with simple yes-or-no decisions (`if` statements), progress to either-or choices (`if-else`), and build up to complex multi-way decisions (`if-elif-else` chains). By the end, you'll write programs that respond intelligently to different situations.

Think of conditionals as the traffic signals of your code: they control which path your program takes based on current conditions.

## Quick Review: Comparison Operators

Before we dive into conditionals, let's refresh what we learned in Chapter 15 about **comparison operators**—the building blocks of decision-making.

**Comparison operators** evaluate a relationship between two values and return `True` or `False`:

| Operator | Meaning | Example | Result |
|----------|---------|---------|--------|
| `==` | Equal to | `5 == 5` | `True` |
| `!=` | Not equal to | `5 != 3` | `True` |
| `>` | Greater than | `7 > 4` | `True` |
| `<` | Less than | `3 < 10` | `True` |
| `>=` | Greater than or equal | `5 >= 5` | `True` |
| `<=` | Less than or equal | `4 <= 9` | `True` |

Here's a quick example:

```python
age: int = 25
is_adult: bool = age >= 18  # Evaluates to True

print(f"Age: {age}")
print(f"Is adult? {is_adult}")
```

**Output:**
```
Age: 25
Is adult? True
```

The expression `age >= 18` evaluates to `True` because 25 is greater than or equal to 18. This boolean result (`True` or `False`) is what powers conditional statements.

#### 💬 AI Colearning Prompt
> "Why does Python use `==` for equality comparison instead of a single `=`? What does a single `=` do?"

## Quick Review: Logical Operators

Sometimes you need to combine multiple conditions. That's where **logical operators** come in:

| Operator | Meaning | Example | When True |
|----------|---------|---------|-----------|
| `and` | Both conditions must be true | `age >= 18 and has_license` | Both are true |
| `or` | At least one condition must be true | `is_weekend or is_holiday` | Either is true |
| `not` | Reverses the boolean value | `not is_raining` | Condition is false |

Quick example:

```python
temperature: int = 75
is_sunny: bool = True

# Both conditions must be true
nice_weather: bool = temperature > 70 and is_sunny

print(f"Temperature: {temperature}°F")
print(f"Sunny? {is_sunny}")
print(f"Nice weather? {nice_weather}")
```

**Output:**
```
Temperature: 75°F
Sunny? True
Nice weather? True
```

The expression `temperature > 70 and is_sunny` evaluates to `True` because both conditions are true.

Now that we've refreshed our memory, let's use these operators to make real decisions in our code!

---

## Tier 1: Book Teaches — Making Your First Decision

### The `if` Statement: Single-Condition Decisions

The simplest decision is: "If this condition is true, do something."

**Syntax:**
```python
if condition:
    # Code to run if condition is True
    # This code is indented (4 spaces)
```

**Critical**: Notice the **colon (`:`)** at the end of the `if` line and the **indentation** (4 spaces) for the code block that runs when the condition is true. Python uses indentation to define which code belongs to the `if` statement.

Let's see it in action with age verification:

```python
# Example 1: Age Verification
age: int = 20

if age >= 18:
    print("Access granted. You are an adult.")
    print("Welcome to the platform!")

print("This line runs regardless of age.")
```

**Expected Output:**
```
Access granted. You are an adult.
Welcome to the platform!
This line runs regardless of age.
```

**What's happening:**
1. We check: `age >= 18` → `20 >= 18` → `True`
2. Because the condition is `True`, the indented code runs
3. The final `print()` statement runs no matter what (it's not indented, so it's outside the `if` block)

If we change `age` to `16`:

```python
age: int = 16

if age >= 18:
    print("Access granted. You are an adult.")
    print("Welcome to the platform!")

print("This line runs regardless of age.")
```

**Expected Output:**
```
This line runs regardless of age.
```

The condition `16 >= 18` is `False`, so the indented code is skipped entirely.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize conditional syntax—you understand WHEN to make a decision. Syntax is cheap; recognizing "my program needs to respond differently based on conditions" is gold.

### The `if-else` Statement: Binary Decisions

What if you want to do one thing when the condition is true and something different when it's false? Use `if-else`:

**Syntax:**
```python
if condition:
    # Runs if condition is True
else:
    # Runs if condition is False
```

Let's check if a number is even or odd:

```python
# Example 2: Even or Odd Checker
number: int = 7

if number % 2 == 0:
    print(f"{number} is even.")
else:
    print(f"{number} is odd.")
```

**Expected Output:**
```
7 is odd.
```

**What's happening:**
1. We check: `number % 2 == 0` (is the remainder 0 when divided by 2?)
2. `7 % 2` equals `1`, so `1 == 0` is `False`
3. The `else` block runs instead

If we change `number` to `8`:

```python
number: int = 8

if number % 2 == 0:
    print(f"{number} is even.")
else:
    print(f"{number} is odd.")
```

**Expected Output:**
```
8 is even.
```

Now `8 % 2` equals `0`, so the condition is `True` and the `if` block runs.

### The `if-elif-else` Chain: Multiple Conditions

What if you have more than two possibilities? Use `elif` (short for "else if") to check multiple conditions in sequence:

**Syntax:**
```python
if condition1:
    # Runs if condition1 is True
elif condition2:
    # Runs if condition1 is False AND condition2 is True
elif condition3:
    # Runs if condition1 and condition2 are False AND condition3 is True
else:
    # Runs if all conditions are False
```

**Important**: Python evaluates conditions **from top to bottom** and stops at the first `True` condition. Once a condition matches, the rest are skipped.

Let's classify grades:

```python
# Example 3: Grade Classifier
score: int = 85

if score >= 90:
    grade: str = "A"
    print(f"Score: {score} → Grade: {grade} (Excellent!)")
elif score >= 80:
    grade: str = "B"
    print(f"Score: {score} → Grade: {grade} (Good job!)")
elif score >= 70:
    grade: str = "C"
    print(f"Score: {score} → Grade: {grade} (Passing)")
elif score >= 60:
    grade: str = "D"
    print(f"Score: {score} → Grade: {grade} (Needs improvement)")
else:
    grade: str = "F"
    print(f"Score: {score} → Grade: {grade} (Failing)")
```

**Expected Output:**
```
Score: 85 → Grade: B (Good job!)
```

**Execution trace for `score = 85`:**
1. Check `score >= 90` → `85 >= 90` → `False` (skip this block)
2. Check `score >= 80` → `85 >= 80` → `True` (execute this block, stop checking)
3. Skip remaining `elif` and `else` blocks

**What if `score = 95`?**

```python
score: int = 95

if score >= 90:
    grade: str = "A"
    print(f"Score: {score} → Grade: {grade} (Excellent!)")
elif score >= 80:
    grade: str = "B"
    print(f"Score: {score} → Grade: {grade} (Good job!)")
# ... rest of the conditions
```

**Expected Output:**
```
Score: 95 → Grade: A (Excellent!)
```

The first condition matches, so Python never even checks the rest.

#### 🤝 Practice Exercise

> **Ask your AI**: "Generate a Python if-elif-else statement that categorizes temperature: freezing (below 32°F), cold (32-50°F), mild (51-70°F), warm (71-85°F), or hot (above 85°F). Include type hints and expected output for temperature = 68."

**Expected Outcome**: You'll understand how to structure multi-condition decision trees and see how order matters (checking ranges from high to low or low to high).

---

## Tier 2: AI Companion — Complex Conditions and Nested Decisions

Now that you understand the basics, let's explore patterns that AI companions handle well: combining multiple conditions and nesting decisions.

### Complex Conditions with `and` / `or`

Real-world decisions often depend on multiple factors. Let's check eligibility for a discount:

**Problem**: A customer gets a discount if:
- They're a member **AND** the purchase is over $50, **OR**
- The purchase is over $100 (regardless of membership)

**Tell your AI:**
> "Generate a Python conditional that checks discount eligibility: customer gets discount if (is_member AND purchase > 50) OR (purchase > 100). Use type hints, test with purchase=75 and is_member=True, then purchase=120 and is_member=False. Show expected output."

**AI-Generated Code (Example):**

```python
# Example 4: Discount Eligibility with Complex Conditions
purchase_amount: float = 75.00
is_member: bool = True

# Complex condition combining AND and OR
if (is_member and purchase_amount > 50) or (purchase_amount > 100):
    discount_rate: float = 0.10  # 10% discount
    discount_amount: float = purchase_amount * discount_rate
    final_price: float = purchase_amount - discount_amount

    print(f"Purchase: ${purchase_amount:.2f}")
    print(f"Member: {is_member}")
    print(f"Discount applied: ${discount_amount:.2f} (10%)")
    print(f"Final price: ${final_price:.2f}")
else:
    print(f"Purchase: ${purchase_amount:.2f}")
    print(f"Member: {is_member}")
    print(f"No discount (must be member with $50+ purchase or $100+ purchase)")
```

**Expected Output (purchase=75, is_member=True):**
```
Purchase: $75.00
Member: True
Discount applied: $7.50 (10%)
Final price: $67.50
```

**Expected Output (purchase=120, is_member=False):**
```
Purchase: $120.00
Member: False
Discount applied: $12.00 (10%)
Final price: $108.00
```

**What your AI should explain:**
- Parentheses `()` control evaluation order: `(is_member and purchase_amount > 50)` is evaluated first
- The `or` operator connects two possible paths to discount eligibility
- First case: member with $50+ purchase → discount
- Second case: any purchase over $100 → discount

### Nested `if` Statements: Multi-Criteria Validation

Sometimes you need to check one condition, and **only if that's true**, check another condition. This is called **nesting**.

**Problem**: Verify if someone can rent a car:
- Must be at least 25 years old
- Must have a driver's license
- If both true, check if they want insurance

**Tell your AI:**
> "Generate nested if statements for car rental validation: check age >= 25, then check has_license, then ask about insurance. Use type hints. Test with age=28, has_license=True, wants_insurance=False. Show expected output and explain why nesting is used instead of a single complex condition."

**AI-Generated Code (Example):**

```python
# Example 5: Multi-Criteria Validation with Nested If
age: int = 28
has_license: bool = True
wants_insurance: bool = False

if age >= 25:
    print("✓ Age requirement met (25+)")

    if has_license:
        print("✓ Driver's license verified")

        # Calculate rental price based on insurance choice
        base_price: float = 50.00

        if wants_insurance:
            insurance_cost: float = 15.00
            total_price: float = base_price + insurance_cost
            print(f"✓ Insurance selected (+${insurance_cost:.2f})")
            print(f"Total rental cost: ${total_price:.2f}/day")
        else:
            print("○ Insurance declined")
            print(f"Total rental cost: ${base_price:.2f}/day")
    else:
        print("✗ Rental denied: No driver's license")
else:
    print("✗ Rental denied: Must be 25 or older")
```

**Expected Output (age=28, has_license=True, wants_insurance=False):**
```
✓ Age requirement met (25+)
✓ Driver's license verified
○ Insurance declined
Total rental cost: $50.00/day
```

**What if age=22?**

**Expected Output (age=22):**
```
✗ Rental denied: Must be 25 or older
```

The nested `if` statements never run because the first condition fails.

**Why nesting instead of `and`?**

You could write: `if age >= 25 and has_license and wants_insurance:`

But nesting lets you:
- Provide specific feedback at each stage ("age OK, but no license")
- Perform different actions at each level (calculate pricing only if eligible)
- Make the logic easier to read and debug

#### 🎓 Expert Insight
> In AI-native development, nested conditionals are a common pattern. You don't memorize the nesting depth—you describe the decision tree to your AI: "First check X, if true then check Y, if that's true then check Z." Your AI handles the indentation and structure; you verify the logic makes sense.

---

## Red Flags: Common Conditional Errors (and How to Fix Them)

Even experienced developers make these mistakes. Recognizing them will save you hours of debugging.

### 🚩 Red Flag 1: IndentationError

**Problem:**
```python
age: int = 20

if age >= 18:
print("Access granted")  # ❌ No indentation!
```

**Error:**
```
IndentationError: expected an indented block after 'if' statement on line 3
```

**Fix:**
```python
if age >= 18:
    print("Access granted")  # ✓ Indented with 4 spaces
```

**AI Troubleshooting Prompt:**
> "I'm getting IndentationError in my if statement. Here's my code: [paste code]. Explain what's wrong and show the corrected version."

---

### 🚩 Red Flag 2: Unreachable Code

**Problem:**
```python
score: int = 85

if score >= 60:
    grade: str = "D or better"
    print(f"Grade: {grade}")
elif score >= 80:  # ❌ This will NEVER run!
    grade: str = "B or better"
    print(f"Grade: {grade}")
```

**What's wrong:** If `score >= 60` is true (which it is for score=85), the `elif score >= 80` is never checked. Python stops at the first matching condition.

**Fix:** Order conditions from most specific to least specific:

```python
score: int = 85

if score >= 80:  # ✓ Check higher values first
    grade: str = "B or better"
    print(f"Grade: {grade}")
elif score >= 60:
    grade: str = "D or better"
    print(f"Grade: {grade}")
```

**AI Troubleshooting Prompt:**
> "My elif condition for score >= 80 never executes even when score is 85. Here's my code: [paste code]. Explain why and show the fix."

---

### 🚩 Red Flag 3: Logical Errors (Impossible Conditions)

**Problem:**
```python
age: int = 25

if age >= 18 and age < 13:  # ❌ Impossible! Can't be both >= 18 AND < 13
    print("Teen discount applied")
```

**What's wrong:** No value of `age` can satisfy both `age >= 18` and `age < 13` simultaneously. This code never runs.

**Fix:** Use `or` instead of `and`, or check your logic:

```python
# If you meant: age is EITHER teen (13-17) OR adult (18+)
if age >= 13 and age < 18:
    print("Teen discount applied")
elif age >= 18:
    print("Adult pricing")
```

**AI Troubleshooting Prompt:**
> "My condition `age >= 18 and age < 13` never executes. Is this logic correct? If not, what should it be?"

---

### 🚩 Red Flag 4: Type Mismatches

**Problem:**
```python
age: str = "25"  # ❌ String, not integer!

if age >= 18:
    print("Access granted")
```

**Error:**
```
TypeError: '>=' not supported between instances of 'str' and 'int'
```

**Why:** You can't compare a string (`"25"`) with an integer (`18`) using `>=`.

**Fix:** Convert the string to an integer:

```python
age: int = int("25")  # ✓ Convert string to int

if age >= 18:
    print("Access granted")
```

**AI Troubleshooting Prompt:**
> "I'm getting TypeError: '>=' not supported between instances of 'str' and 'int'. Here's my code: [paste code]. How do I fix this?"

---

## Try With AI: Conditional Logic Debugging Challenge

You've learned `if`, `elif`, `else`, and nested conditionals. Now debug broken decision logic to understand how conditionals fail—with AI as your logic validator.

### Part 1: Find the Bug (Your Turn First)

**Before asking AI**, analyze these broken conditionals and predict what's wrong:

```python
# Bug 1: Unreachable code
score: int = 85

if score >= 60:
    grade: str = "D or better"
elif score >= 80:  # Will this ever run?
    grade: str = "B or better"

# Your prediction: What's wrong? What will grade be for score=85?

# Bug 2: Impossible condition
age: int = 25

if age >= 18 and age < 13:  # Will this ever be True?
    print("Teen discount")

# Your prediction: Can this condition EVER be True? Why or why not?

# Bug 3: Type mismatch
user_age: str = "25"  # Notice: string, not int

if user_age >= 18:
    print("Access granted")

# Your prediction: Will this crash? If yes, what error?
```

**Your task**:
1. For each bug, predict: What will happen?
2. Write down: Why is it broken?
3. Sketch the fix BEFORE running the code

---

### Part 2: AI Explains Decision Logic (Discovery)

Share your bug predictions with AI:

> "I found 3 bugs in conditional logic: [paste code and your predictions]. For each:
> 1. Confirm if my prediction is correct
> 2. Explain WHY it's broken (unreachable code? impossible logic? type error?)
> 3. Show the correct version with explanation
> 4. What's the general principle to avoid this bug?"

**Your task**: Evaluate AI's explanations.
- Does it explain that Python stops at the FIRST matching condition (bug 1)?
- Does it show that `age >= 18 AND age < 13` is logically impossible (bug 2)?
- Does it explain you can't compare string "25" with int 18 (bug 3)?
- Can you now write rules to avoid these bugs?

---

### Part 3: Student Teaches AI (Condition Ordering)

AI explained the bugs. But does it know the BEST way to order conditions?

Challenge AI with complex logic:

> "I'm building a grade classifier:
> - 90+ = A
> - 80-89 = B
> - 70-79 = C
> - 60-69 = D
> - Below 60 = F
>
> Show me THREE different ways to write this:
> 1. Using `if-elif` chain (high to low)
> 2. Using `if-elif` chain (low to high)
> 3. Using multiple independent `if` statements
>
> For each, explain: Does it work correctly? Is it efficient? Which is most readable?"

**Your task**: Compare the patterns.
- Why does high-to-low ordering work but low-to-high breaks?
- Why is `if-elif` better than multiple independent `if` statements?
- Which pattern would YOU use and why?

This teaches AI about decision tree optimization.

---

### Part 4: Build Multi-Condition Validator Together (Convergence)

Now build complex logic with AI:

> "Build a login validator that grants access ONLY if:
> 1. User is 18+ years old
> 2. Account is verified (bool)
> 3. Has NOT failed login more than 3 times
>
> Show two implementations:
> 1. Using nested if statements (if inside if inside if)
> 2. Using a single if with `and` operators
>
> For each, show test cases: (age=20, verified=True, failures=2) should PASS, (age=17, verified=True, failures=0) should FAIL, (age=20, verified=False, failures=0) should FAIL."

**Your task**: Compare nested vs. combined conditions.
- Which is more readable?
- Which is easier to debug?
- Which would fail faster if one condition is False?

Iterate if needed:
> "The nested version is hard to read. Add clear error messages for EACH failure reason."

---

**Time**: 25-30 minutes total
**Outcome**: You've debugged unreachable code, impossible logic, and type errors; learned to order conditions correctly; and built multi-condition validators using both nested and combined patterns.

