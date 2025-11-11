---
title: "Variables and Type Hints — Storing Data with Specifications"
chapter: 14
lesson: 1
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Variables in Python"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain what variables are, why they're needed, and how they store data with meaningful names"

  - name: "Writing Type Hints for Clarity"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can write variables with type hints (age: int = 25) and understand that type hints describe what data means"

  - name: "Understanding How Type Hints Describe Intent"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why type hints serve as embedded specifications and how they help humans (and AI) understand code intent"

learning_objectives:
  - objective: "Understand what variables are and why we use them to store and label data"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of variable purpose; ability to create variables for different types of data"

  - objective: "Create variables with type hints using Python syntax"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Write variables with type hints for int, str, float, bool; use print() to display variables"

  - objective: "Recognize how type hints serve as embedded specifications that clarify intent"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Compare code with and without type hints; explain why clarity matters for learning with AI"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (variables, assignment, type hints, int/str/float/bool, printing variables, specifications) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Think about how type hints describe intent: `age: int = 25` tells AI and humans that 'age is a number'. How would you create a type hint for 'email_address' or 'is_premium_member'? Create 5 variables for a user profile (name, age, email, is_active, account_balance) with appropriate type hints and explain your choices to an AI."
  remedial_for_struggling: "Start with just one variable: `name: str = 'Alex'`. Print it. Change the value. Print again. See what happens. Then add one more variable. Build confidence gradually. Don't worry about memorizing types yet—focus on the pattern: name, colon, type, equals, value."

# Generation metadata
generated_by: "lesson-writer v1.0"
source_spec: "specs/part-4-chapter-14/spec.md"
source_plan: "specs/part-4-chapter-14/plan.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Variables and Type Hints — Storing Data with Specifications

## What Are Variables?

Imagine you're a restaurant manager writing down orders. Instead of saying "The customer ordered a coffee," you might write:

```
Order #5: coffee
```

Now you have a **name** (`Order #5`) for the **data** (`coffee`). You can refer to it later without repeating the whole sentence.

**Variables work exactly the same way.** A variable is a named container that holds data. It lets you:

1. **Store data** — Save information so you don't lose it
2. **Name data meaningfully** — Use descriptive names like `customer_name` instead of remembering it's "in position 3 of some list"
3. **Reuse data** — Refer to it as many times as you need
4. **Change data** — Update what it holds whenever you need to

In Chapter 13, you learned how to print messages directly:

```python
print("Hello, World!")
```

Now we're going to store data first, then print it:

```python
message: str = "Hello, World!"
print(message)
```

That second version is more powerful because the data is **named** and **reusable**. Let's explore why.

## Concept: Storing Data Without Type Hints

Before we talk about type hints, let's see how Python lets you create variables:

```python
# Simple variables - Python figures out the type
name = "Alex"
age = 25
height = 5.8
is_student = True
```

**What's happening here?**

- `name = "Alex"` — Creates a variable called `name` and stores the text `"Alex"`
- `age = 25` — Creates a variable called `age` and stores the number `25`
- `height = 5.8` — Creates a variable called `height` and stores a decimal number
- `is_student = True` — Creates a variable called `is_student` and stores `True` or `False`

The `=` is called the **assignment operator**. It means "store this value in this variable."

**But here's the problem**: If you look at `age = 25` later, you might wonder, "Is 25 a temperature? A score? A count of something?" The code doesn't explain what the data **means**.

That's where **type hints** come in.

## Concept: Type Hints as Specifications

A **type hint** is a note that says what **type** of data a variable holds. It's like adding a label to a jar:

```python
# Variables with type hints - we describe what data means
name: str = "Alex"
age: int = 25
height: float = 5.8
is_student: bool = True
```

Let's break down the pattern `age: int = 25`:

1. **`age`** — The variable name (what we're storing)
2. **`: int`** — The type hint (saying "this will be an integer, a whole number")
3. **`= 25`** — The value (what we're storing)

Read it like this: "age is an integer, and it equals 25."

### The Four Main Data Types You'll Use

| Type | What It Holds | Examples | Type Hint |
|------|--------------|----------|-----------|
| **str** (string) | Text | `"Alex"`, `"hello"`, `"2025"` | `name: str` |
| **int** (integer) | Whole numbers | `25`, `0`, `-5`, `1000` | `age: int` |
| **float** (floating-point) | Decimal numbers | `5.8`, `3.14`, `-2.5` | `height: float` |
| **bool** (boolean) | True or False | `True`, `False` | `is_active: bool` |

### Why Type Hints Matter

Type hints do THREE important things:

**1. Clarity for humans** — When you read `age: int = 25`, you instantly know `age` is a whole number, not a temperature or percentage.

**2. A specification for AI** — Type hints are embedded specifications. When you ask Claude or Gemini "What does this variable mean?", the type hint is the first clue. For example:
   - `email: str = "user@example.com"` tells AI, "This is text that should look like an email"
   - `account_balance: float = 150.50` tells AI, "This is money (a decimal number)"

**3. Catch mistakes early** — Tools can check if you're using a variable correctly. If you write `age: int = "twenty-five"` (storing text in an integer variable), Python's type checker says, "Wait, that's not right."

In AI-native development, **clear specifications are MORE valuable than memorized syntax**. Type hints are how you specify data intent to your AI collaborator.

### Python 3.14's Lazy Annotations (New in 2025)

**Advanced Note**: Python 3.14 introduced a major improvement to how type hints work, called **lazy annotations** (PEP 649).

**What Changed**:
- Type hints are now evaluated **only when needed** (not immediately when Python reads your code)
- Forward references work automatically (no need to put type names in quotes)
- Programs start faster because Python doesn't process all type hints at startup

**What This Means for You**:

Before Python 3.14, if you wanted to use a type before defining it, you'd write:
```python
def get_user(user_id: int) -> "User":  # "User" in quotes
    ...
```

In Python 3.14, this just works:
```python
def get_user(user_id: int) -> User:  # No quotes needed
    ...
```

**For Beginners**: You don't need to understand this deeply yet. Just know that Python 3.14 makes type hints smarter and faster "behind the scenes." As you learn more Python, you'll appreciate how this simplifies advanced code.

**For Advanced Learners**: Ask your AI: _"Explain Python 3.14's lazy annotations (PEP 649) and why they improve type hints. Show me an example of forward references."_

---

## Code Example 1: Basic Variables with Type Hints

Here's how to create your first variables with type hints:

```python
# Storing information about a person
name: str = "Alex"
age: int = 25
height: float = 5.8
is_student: bool = True

# Storing information about a product
product_name: str = "Python Book"
price: float = 29.99
in_stock: bool = True
quantity_remaining: int = 45
```

**Specification: Declaring variables that describe real-world data**

**AI Prompt Used:**
"Create variables with type hints for a student profile (name, age, gpa, is_full_time). Use appropriate types for each."

**Generated Code:**
```python
# Student profile with type hints
student_name: str = "Jordan"
student_age: int = 19
student_gpa: float = 3.85
is_full_time: bool = True
```

**Validation Steps:**
- ✓ Each variable has a name describing what it stores
- ✓ Each has a type hint (`str`, `int`, `float`, `bool`)
- ✓ Each has a value matching the type hint
- ✓ Code follows the pattern: `name: type = value`

## Code Example 2: Displaying Variables with print()

Now let's use `print()` from Chapter 13 to display variables:

```python
# Create variables with type hints
name: str = "Alex"
age: int = 25
height: float = 5.8
is_student: bool = True

# Display each variable
print(name)        # Output: Alex
print(age)         # Output: 25
print(height)      # Output: 5.8
print(is_student)  # Output: True
```

**Try this yourself:**

1. Create a file called `variables.py`
2. Copy the code above
3. Run it: `python variables.py`
4. Change the values and run again — you'll see the new values printed

## Code Example 3: Displaying Multiple Variables

Now let's print multiple variables together using `print()` from Chapter 13:

```python
# Variables with type hints
name: str = "Alex"
age: int = 25
is_student: bool = True

# Display each variable with a label
print("Name:", name)
print("Age:", age)
print("Student:", is_student)

# Display multiple variables in one print() statement
print("Name:", name, "Age:", age, "Student:", is_student)
```

**Expected output:**
```
Name: Alex
Age: 25
Student: True
Name: Alex Age: 25 Student: True
```

**Key insight**: Notice how the type hints (`name: str`, `age: int`, `is_student: bool`) make it clear what data we're printing. The type hint is a specification that helps us understand the code.

**Note**: In Chapter 16 (Strings and Type Casting), you'll learn more powerful ways to format text output. For now, simple `print()` statements with commas are all you need.

## Practice Exercise 1: Create Variables for Your Profile

Create a Python file with variables describing **you**. Use type hints for each:

```python
# Your turn! Fill in the values.
your_name: str = "..."
your_age: int = ...
your_favorite_food: str = "..."
your_height_in_feet: float = ...
are_you_learning_to_code: bool = ...

# Print them all
print(your_name)
print(your_age)
print(your_favorite_food)
print(your_height_in_feet)
print(are_you_learning_to_code)
```

**Success criteria:**
- All variables have type hints
- All variables have appropriate types for their values
- All variables print without errors
- You can change the values and see the output update

## Practice Exercise 2: Create Variables for Different Data Types

Create one variable for each data type. Challenge yourself to choose meaningful names:

```python
# String variables (text)
favorite_color: str = "..."
best_friend_name: str = "..."

# Integer variables (whole numbers)
birth_year: int = ...
favorite_number: int = ...

# Float variables (decimal numbers)
favorite_temperature: float = ...
average_test_score: float = ...

# Boolean variables (True/False)
do_you_like_python: bool = ...
have_you_coded_before: bool = ...

# Now print them all with labels
print("My favorite color is", favorite_color)
print("I was born in", birth_year)
print("I like Python:", do_you_like_python)
```

**Success criteria:**
- All 8 variables have correct type hints
- Values match their types (strings in quotes, numbers without quotes, True/False for booleans)
- All variables print without errors
- Each sentence reads naturally

## Practice Exercise 3: The Type-Hint Challenge

Create variables for a simple game character. Choose the **right type** for each attribute:

```python
character_name: str = "Knight"
health_points: int = 100
mana_points: int = 50
speed: float = 7.5
is_alive: bool = True
experience: int = 0

# Display the character's status
print(character_name, "has", health_points, "HP and", mana_points, "mana")
print("Speed:", speed, "Alive:", is_alive, "Experience:", experience)
```

**Questions to think about:**
- Why is `health_points` an `int` and not a `float`?
- Why is `speed` a `float` and not an `int`?
- Why is `is_alive` a `bool` (True/False) instead of text?
- What would happen if you tried `health_points: str = 100`? (Try it and see what error you get—don't worry if there's an issue, just observe.)

**Success criteria:**
- All variables have the correct type hint for their data
- The character display works without errors
- You can explain why each type was chosen

## Common Pitfalls

**Pitfall 1: Forgetting Quotes for Strings**

```python
# WRONG - string needs quotes
name: str = Alex

# RIGHT - string has quotes
name: str = "Alex"
```

If you see an error like `NameError: name 'Alex' is not defined`, you forgot quotes.

**Pitfall 2: Putting a Decimal in an Integer**

```python
# WRONG - int should be a whole number
age: int = 25.5

# RIGHT - use float for decimals
age: float = 25.5
# OR keep int if you want whole numbers
age: int = 25
```

**Pitfall 3: Type Hint Doesn't Match the Value**

```python
# WRONG - says int but stores text
age: int = "25"

# RIGHT - either change the type hint or the value
age: str = "25"      # if it's text
age: int = 25        # if it's a number
```

**Pitfall 4: Using the Wrong Capitalization for Booleans**

```python
# WRONG - lowercase true
is_student: bool = true

# RIGHT - Python uses True and False (uppercase)
is_student: bool = True
```

## Why This Matters for AI-Driven Development

Here's the big picture: **Type hints are how you write specifications.** When you write:

```python
user_email: str = "alex@example.com"
purchase_amount: float = 99.99
is_premium_member: bool = True
```

You're telling any AI collaborator (Claude, Gemini, Copilot):

1. **What data you have** — email (text), amount (money), membership status (yes/no)
2. **What it means** — The variable names describe purpose; the type hints describe structure
3. **How to use it safely** — Text gets email validation, numbers can do math, booleans trigger yes/no logic

When you ask Claude Code "How do I validate this email?" it immediately knows `user_email` is text and suggests string validation. When you ask "How do I add tax to this purchase?" it knows `purchase_amount` is a decimal number and can calculate correctly.

**Type hints are embedded specifications. Clear specifications make AI collaboration more powerful.**

---

## Try With AI

Use this section to reinforce your learning by applying the lesson with an AI tool. You've seen how variables with type hints describe data. Now let's explore this deeper with your AI collaborator.

**Using Claude Code, Gemini CLI, or ChatGPT (web):**

### Prompt Set

**Prompt 1: Understand Variable Purpose**

Copy this prompt and paste it into your AI tool:

```
You learned about print() in Chapter 13. Now explain what a variable is
in 2-3 sentences. Use a real-world analogy (like a labeled jar, a mailbox,
or a storage box). Why do we need variables instead of just using print()
with values directly?
```

**Expected outcome:** AI explains variables using a concrete analogy, clarifying why labeling data is useful.

**Prompt 2: Decode Type Hints**

```
What does this line of code mean? Break it down piece by piece:

age: int = 25

Explain what "age", ": int", and "= 25" each mean. Why do we add ": int"
to the variable?
```

**Expected outcome:** AI explains that the type hint describes data structure, separating "what it is" (int) from "what value it holds" (25).

**Prompt 3: Explore Data Types**

```
Show me 5 examples of variables with type hints for different data.
Create variables for: a person's name, their age, their weight in pounds,
whether they exercise regularly, and the date they joined. Choose the
correct type (str, int, float, or bool) for each. Explain why you chose
each type.
```

**Expected outcome:** AI creates realistic variables and explains type reasoning. You'll see patterns: text uses `str`, whole numbers use `int`, decimals use `float`, yes/no uses `bool`.

**Prompt 4: Type Hints as Specifications (Stretch)**

```
Here's a question: How does "age: int = 25" help you and me (an AI)
understand this code better than just "age = 25"? Why is the type hint
a specification? How does clarity help when learning with AI as your partner?
```

**Expected outcome:** AI explains that type hints reduce ambiguity, enable validation, and allow AI to suggest context-aware help. This connects variables to the broader concept of specification-driven development.

### Safety & Ethics Note

When exploring code with AI, always ask "Why?" before using it. Type hints are clear because they reduce guessing. As you write more code, you'll notice that **clarity is collaboration**—clear code helps you, helps teammates, and helps AI understand your intent. This is why the book emphasizes specifications as the primary skill, not syntax memorization.

### Next Self-Directed Variation

After working through these prompts:

- Create a variable for something in your own life (a hobby, a possession, a preference)
- Write it with a type hint
- Share it with your AI partner and ask: "Does this type hint match my intent? Could someone (or an AI) understand what this variable represents?"
- Experiment: Change the type hint to the wrong type (e.g., `hobby: int = "painting"`) and ask your AI, "What's wrong with this?"

This practice builds your ability to think in specifications—the foundation of AI-native development.
