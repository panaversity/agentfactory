---
title: "User Input: Interactive Programs"
chapter: 13
lesson: 4
duration_minutes: 30
proficiency_level: "CEFR A2"
blooms_level: "Apply"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using the input() Function"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write input() statements with prompts, store user responses in variables, and use those variables in program logic"

  - name: "Integrating print(), Variables, and input()"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create programs combining all Chapter 13 concepts: display prompts with print(), gather data with input(), store in variables, and generate dynamic output"

learning_objectives:
  - objective: "Use input() to gather user data in programs"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes 3-5 input() statements with clear prompts and stores responses in appropriately named variables"

  - objective: "Create interactive programs that respond to user input"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student builds program that asks questions, processes answers, and displays personalized output based on user responses"

cognitive_load:
  new_concepts: 1
  assessment: "1 new concept (input() function) at A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Explore type conversion (int(input())), input validation, multi-line prompts using triple quotes, f-strings for formatted output"
  remedial_for_struggling: "Start with single input() and print() pair; gradually add more inputs; use very clear prompt messages"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/014-chapter-13-redesign/spec.md"
created: "2025-01-18"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# User Input: Interactive Programs

## What You'll Learn

- The input() Function
- Storing User Input
- Interactive Programs
- Input Patterns

---

## Opening: Your Robot Learns to Listen

Our robot's evolution is almost complete. It can **speak** (`print()`) and **remember** (`variables`). But there's one critical skill missing: **the robot can't listen**.

Right now, all the robot's data is hard-coded. The robot knows facts only because *you* programmed them in. It can't *ask* for information or adapt to different users.

Imagine if the robot could have a conversation:
```
Robot: "What is your name?"
You: "Sarah"
Robot: "Hello, Sarah! How old are you?"
You: "30"
Robot: "Nice to meet you, Sarah. You're 30 years old!"
```

This is **dynamic interaction**. The robot asks, you answer, and it responds based on *your* specific input.

**This is what `input()` enables.** It transforms programs from static scripts into interactive conversations.

---

## The input() Function: How Programs Listen

### Understanding the input() Function

`input()` is a built-in Python function that:
1. Displays a prompt to the user
2. Waits for the user to type something and press Enter
3. Returns what the user typed (as a string)

Syntax:
```python
variable = input("Your question here: ")
```

**Example:**
```python
name = input("What is your name? ")
print("Hello,", name)
```

When you run this:
```
What is your name? Sarah
Hello, Sarah
```

**What happened:**
1. Line 1: Program displays "What is your name? " and waits
2. User types "Sarah" and presses Enter
3. Line 1: `input()` returns "Sarah", which is stored in `name`
4. Line 2: Program prints "Hello, Sarah"

### Why input() Matters

Without `input()`, every program would work the same way for everyone:

```python
name = "Alex"
print("Hello,", name)
```

Output: `Hello, Alex` (always "Alex", never changes)

With `input()`, programs adapt to each user:

```python
name = input("What is your name? ")
print("Hello,", name)
```

Output: `Hello, [whatever name you type]`

**This makes programs useful in the real world:**
- Games can ask for player names
- Calculators can ask for numbers
- Surveys can gather responses
- Login systems can ask for usernames/passwords

**Programs that listen are programs that adapt.**

---

## How input() Works

### The Prompt (Optional but Recommended)

The text inside `input()` is called the **prompt**—what the user sees before typing.

**With a prompt:**
```python
age = input("How old are you? ")
```

User sees:
```
How old are you? _
```
(The `_` represents the cursor waiting for input)

**Without a prompt:**
```python
age = input()
```

User sees:
```
_
```
(Just a blank cursor—confusing!)

**Best Practice:** Always include a clear prompt so users know what to enter.

### Input Always Returns a String

**Critical Detail:** `input()` always returns a **string** (text), even if the user types a number.

```python
age = input("How old are you? ")
print(type(age))  # <class 'str'>
```

Even if you type `25`, Python stores it as `"25"` (text), not `25` (number).

**Why This Matters:**
- If you need to do math, you'll need to convert the string to a number (covered in Chapter 14)
- For now, we'll work with text-based programs

---

## Basic Input Patterns

### Pattern 1: Ask and Store

```python
name = input("What is your name? ")
print("Nice to meet you,", name)
```

**Output:**
```
What is your name? Alex
Nice to meet you, Alex
```

---

### Pattern 2: Multiple Questions

```python
name = input("What is your name? ")
city = input("Where do you live? ")
hobby = input("What's your favorite hobby? ")

print("Hi", name, "from", city, "who enjoys", hobby)
```

**Output:**
```
What is your name? Sarah
Where do you live? Boston
What's your favorite hobby? Photography
Hi Sarah from Boston who enjoys Photography
```

---

### Pattern 3: Interactive Conversation

```python
print("Welcome to the Greeting Program!")
print()  # Blank line for spacing

name = input("What's your name? ")
print("Hello,", name)
print()

mood = input("How are you feeling today? ")
print("I hope you have a great day feeling", mood)
```

**Output:**
```
Welcome to the Greeting Program!

What's your name? Alex
Hello, Alex

How are you feeling today? excited
I hope you have a great day feeling excited
```

---

## Integrating All Chapter 13 Concepts

Now we bring everything together: `print()`, `variables`, and `input()`.

### Complete Example: Personal Introduction

```python
# Gather information
print("Let's create your introduction card!")
print()

name = input("What is your name? ")
age = input("How old are you? ")
city = input("Where do you live? ")
hobby = input("What's your favorite hobby? ")

# Display formatted introduction
print()
print("=" * 40)
print("INTRODUCTION CARD")
print("=" * 40)
print("Name:", name)
print("Age:", age)
print("City:", city)
print("Hobby:", hobby)
print("=" * 40)
```

**Output:**
```
Let's create your introduction card!

What is your name? Sarah
How old are you? 28
Where do you live? Chicago
What's your favorite hobby? Reading

========================================
INTRODUCTION CARD
========================================
Name: Sarah
Age: 28
City: Chicago
Hobby: Reading
========================================
```

**What's Happening:**
1. **Lines 2-3:** Introduction message
2. **Lines 5-8:** Gather user input with `input()`, store in variables
3. **Lines 11-18:** Display formatted output using stored variables

This program combines **all three core concepts**:
- ✅ `print()` for output
- ✅ `variables` for storage
- ✅ `input()` for gathering data

---

## Practice Exercises

### Exercise 1: Simple Questionnaire

Create `questionnaire.py`:

```python
favorite_color = input("What's your favorite color? ")
favorite_food = input("What's your favorite food? ")
favorite_animal = input("What's your favorite animal? ")

print()
print("Your Favorites:")
print("- Color:", favorite_color)
print("- Food:", favorite_food)
print("- Animal:", favorite_animal)
```

Run it and test with different inputs.

---

### Exercise 2: Mad Libs Story

Create `madlibs.py`:

```python
print("Let's create a silly story!")
print()

noun = input("Enter a noun: ")
verb = input("Enter a verb: ")
adjective = input("Enter an adjective: ")
place = input("Enter a place: ")

print()
print("Here's your story:")
print("Once upon a time, a", adjective, noun, "decided to", verb, "in", place + "!")
```

**Example Output:**
```
Let's create a silly story!

Enter a noun: robot
Enter a verb: dance
Enter an adjective: silly
Enter a place: the park

Here's your story:
Once upon a time, a silly robot decided to dance in the park!
```

---

### Exercise 3: Personal Profile

Create `profile.py`:

```python
print("=== Create Your Profile ===")
print()

username = input("Choose a username: ")
email = input("Enter your email: ")
favorite_language = input("Favorite programming language: ")

print()
print("Profile Created Successfully!")
print()
print("Username:", username)
print("Email:", email)
print("Favorite Language:", favorite_language)
print()
print("Welcome to the community,", username + "!")
```

---

## Common Mistakes

### Mistake 1: Forgetting to Store Input

**Wrong:**
```python
input("What is your name? ")
print("Hello,", name)
```

**Error:**
```
NameError: name 'name' is not defined
```

**Why:** You asked for input but didn't store it in a variable. Python asked the question, you answered, then Python forgot the answer.

**Fix:**
```python
name = input("What is your name? ")  # Store in variable
print("Hello,", name)
```

---

### Mistake 2: Confusing input() with print()

**Wrong:**
```python
print("What is your name? ")
print("Hello,", name)
```

**Problem:** `print()` doesn't wait for user input—it just displays text. Your program would show the question but immediately continue without waiting for an answer.

**Fix:**
```python
name = input("What is your name? ")  # input() waits for response
print("Hello,", name)
```

---

### Mistake 3: Missing Closing Parenthesis or Quote

**Wrong:**
```python
name = input("What is your name?
```

**Error:**
```
SyntaxError: unterminated string literal
```

**Fix:**
```python
name = input("What is your name? ")  # Close quote and parenthesis
```
---

## Try With AI

### Prompt 1: Input Variations

```
I just learned the input() function in Python.

Show me 5 creative ways to use input() I didn't see in the lesson.

For each example:
1. Show the code
2. Explain what it does
3. Give example output

Focus on different types of prompts (questions, commands, creative formats).
```

**What to look for:** AI shows variations like multiple inputs on one line, creative prompt formatting, or interesting use cases.

**Reflection:** Which example would you use in a real program?

---

### Prompt 2: Build a Quiz Program

```
Help me create a simple quiz program using input().

Requirements:
- Ask 3 questions about Python (from Chapter 13)
- Store each answer in a variable
- Display all answers at the end
- Use clear prompts

Show me the complete code with type hints.
```

**What to look for:** AI creates descriptive prompts and stores each answer.

**Reflection:** Run the program. Can you add 2 more quiz questions?

---

### Prompt 3: Interactive Story

```
Create an interactive story program that:
1. Asks the user for their name
2. Asks for their favorite animal
3. Asks for an adjective
4. Tells a short story using their inputs

Make it fun and engaging. Show the full code.
```

**What to look for:** AI builds a narrative using the user's responses.

**Reflection:** How does `input()` make the story personalized?

---

### Prompt 4: Concept Check

```
Quiz me on input() and integration concepts.

Ask me 3 questions:
1. What does input() return, even if the user types a number?
2. Why should you always include a prompt in input()?
3. How do you combine input() with variables and print() to create interactive programs?

After each answer, tell me if I'm correct and explain.
```

**What to look for:** Can you answer confidently?

**Reflection:** Which concept is clearest? Which needs review before the capstone?

