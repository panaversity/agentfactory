---
title: "Variables: Python's Memory"
chapter: 13
lesson: 3
duration_minutes: 40
proficiency_level: "CEFR A2"
blooms_level: "Apply"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Variables as Labeled Memory"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain variables as 'labeled boxes that store data' using own analogies"

  - name: "Creating and Modifying Variables"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create variables with assignment (=), modify values through reassignment, and print variable contents"

  - name: "Applying Variable Naming Conventions"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can write valid variable names following rules (no spaces, snake_case, descriptive), identify invalid names, and explain why naming matters"

learning_objectives:
  - objective: "Explain what variables are and why programs need them"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student describes variables as memory storage using own words and provides 2-3 examples of when variables are necessary"

  - objective: "Create and modify variables using Python assignment syntax"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes code creating 5 variables, modifies 2 of them, and prints results"

  - objective: "Write valid variable names following Python conventions"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Given 10 proposed names, student identifies which are valid/invalid and rewrites invalid names correctly"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (variables as memory, creating variables, naming rules) at A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Explore variable scope (global vs local preview), multiple assignment (a = b = 5), swapping values (a, b = b, a)"
  remedial_for_struggling: "Focus on single variable creation/modification first; use physical analogy (labeled box with paper inside); skip naming convention details initially"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/014-chapter-13-redesign/spec.md"
created: "2025-01-18"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Variables: Python's Memory

## What You'll Learn

- What variables are and why every program needs them
- How to create variables in Python
- How to modify (reassign) variable values
- Variable naming rules and best practices
- Common mistakes with variables

---

## Opening Hook: Your Robot Learns to Remember

Remember our robot? In Lesson 1, you taught it what programming is. In Lesson 2, it spoke its first words using `print()`. But there's a problem: **the robot has no memory**.

Imagine asking your robot:
- "Robot, what's my name?"
- "Robot, how old am I?"
- "Robot, what's my favorite color?"

The robot stares blankly. It can follow instructions in the moment, but it can't **remember** anything. Every interaction starts from scratch.

Now imagine teaching your robot to remember. You say:

- "Robot, my name is Alex. Remember that."
- "Robot, I'm 25 years old. Store that information."
- "Robot, my favorite color is blue. Don't forget."

Suddenly, the robot can recall facts:
- "What's my name?" → "Your name is Alex."
- "How old am I?" → "You're 25 years old."
- "What's my favorite color?" → "Your favorite color is blue."

**This is what variables do.** They give programs the ability to **remember** information—names, ages, scores, preferences, calculations, anything you need to store and use later.

Without variables, programs would be useless. With variables, programs become powerful tools that adapt, calculate, and respond intelligently.

---

## What Are Variables?

### The Concept (WHAT)

A **variable** is a named storage location in your computer's memory where you can store data.

Think of it like a labeled box:
- The **label** is the variable's name (`age`, `name`, `score`)
- The **box** is a spot in memory
- The **contents** are the data you store (25, "Alex", 100)

When you create a variable, you're telling Python: "Reserve a spot in memory, label it with this name, and put this data inside."

Later, when you use that variable name, Python retrieves the data for you.

### A Better Analogy: Pet Names

Here's an analogy that might click: **Variables are like naming your pet**.

When you get a dog, you don't call it "Dog #47" every time. You give it a name: "Max." From then on, whenever you say "Max," you're referring to that specific dog.

Variables work the same way:

```python
name = "Alex"
```

This says: "Create a variable called `name`, and store the text 'Alex' inside it."

Now whenever you write `name` in your code, Python knows you mean "Alex":

```python
print(name)  # Displays: Alex
```

Just like naming your pet lets you call them easily, naming your data lets you use it easily throughout your program.

### The Purpose (WHY)

Why do programs need variables? Because **programs need to remember things**.

**Example 1: A Calculator**
```python
first_number = 10
second_number = 5
result = first_number + second_number
print(result)  # Displays: 15
```

Without variables, you couldn't store the numbers or the calculation result. The program would have no way to "remember" what to add.

**Example 2: A Personalized Greeting**
```python
user_name = "Sarah"
print("Welcome back,", user_name)  # Displays: Welcome back, Sarah
```

Without the `user_name` variable, you couldn't personalize the message. You'd have to hard-code every possible name (impossible).

**Example 3: Tracking Game Score**
```python
score = 0
score = score + 10  # Player scored!
score = score + 5   # Scored again!
print("Final score:", score)  # Displays: Final score: 15
```

Without variables, games couldn't track scores, levels, health, or any changing data.

**Variables enable programs to be dynamic—they adapt based on data.**

---

## Creating Variables in Python

Now let's learn the syntax (HOW).

### The Assignment Operator: =

In Python, you create a variable using the **assignment operator**: `=`

```python
age = 25
```

This line does three things:
1. Creates a variable named `age`
2. Allocates memory to store a number
3. Puts the value `25` in that memory

**Important:** In programming, `=` doesn't mean "equals" (like in math). It means **"assign the value on the right to the variable on the left."**

Think of `=` as an arrow pointing left:

```python
age ← 25
```

"Put 25 into the variable called age."

### Variable Creation Examples

**Storing Numbers:**
```python
age = 25
price = 19.99
temperature = -5
```

**Storing Text:**
```python
name = "Alex"
city = "New York"
message = "Hello, World!"
```

**Storing True/False Values:**
```python
is_student = True
has_license = False
```

**Storing Calculation Results:**
```python
total = 10 + 5
average = (100 + 90 + 85) / 3
```

**Key Insight:** You can store any type of data in variables—numbers, text, true/false values, calculation results, and more.

---

## Using Variables

Once you've created a variable, you can use it anywhere in your program.

### Printing Variable Values

```python
name = "Alex"
age = 25

print(name)        # Displays: Alex
print(age)         # Displays: 25
print("Name:", name)  # Displays: Name: Alex
```

### Using Variables in Calculations

```python
length = 10
width = 5
area = length * width
print("Area:", area)  # Displays: Area: 50
```

### Using Variables in Other Variables

```python
first_name = "Alex"
last_name = "Johnson"
full_name = first_name + " " + last_name
print(full_name)  # Displays: Alex Johnson
```

**The `+` operator joins (concatenates) text together.**

---

## Modifying Variables (Reassignment)

Here's something powerful: **variables can change**. You can reassign new values to existing variables.

### Simple Reassignment

```python
score = 0
print(score)   # Displays: 0

score = 10
print(score)   # Displays: 10

score = 25
print(score)   # Displays: 25
```

Each time you assign a new value, the old value is replaced.

### Modifying Based on Current Value

This is common: change a variable based on what it currently holds.

```python
score = 10
score = score + 5
print(score)  # Displays: 15
```

What's happening on line 2?
1. Python reads the current value of `score` (10)
2. Adds 5 to get 15
3. Assigns 15 back to `score`
4. The old value (10) is replaced

**Shorthand Version:**
```python
score = 10
score += 5  # Same as: score = score + 5
print(score)  # Displays: 15
```

`+=` is shorthand for "add this to the current value."

### Common Modification Patterns

**Incrementing (adding):**
```python
count = 0
count += 1  # count is now 1
count += 1  # count is now 2
count += 1  # count is now 3
```

**Decrementing (subtracting):**
```python
lives = 3
lives -= 1  # lives is now 2
lives -= 1  # lives is now 1
```

**Doubling:**
```python
amount = 5
amount *= 2  # amount is now 10
amount *= 2  # amount is now 20
```

---

## Variable Naming Rules and Best Practices

Not all variable names are valid. Python has strict rules.

### The Rules (Must Follow)

**Rule 1: Only letters, numbers, and underscores**
- ✅ `user_name`
- ✅ `score2`
- ✅ `_temp`
- ❌ `user-name` (dashes not allowed)
- ❌ `score!` (special characters not allowed)
- ❌ `user name` (spaces not allowed)

**Rule 2: Cannot start with a number**
- ✅ `player1`
- ✅ `score_2`
- ❌ `1st_place`
- ❌ `2player`

**Rule 3: Case-sensitive**
- `age`, `Age`, and `AGE` are **three different variables**
- Convention: use lowercase (explained below)

**Rule 4: Cannot use Python keywords**
Python reserves certain words for its own use. You can't use these as variable names:
- ❌ `print = 5` (print is a function)
- ❌ `if = 10` (if is a keyword)
- ❌ `for = "hello"` (for is a keyword)

Common reserved words: `if`, `else`, `for`, `while`, `def`, `class`, `return`, `import`, `True`, `False`

### Best Practices (Should Follow)

**Practice 1: Use descriptive names**
- ✅ `user_age` (clear what it stores)
- ❌ `x` (what does x mean?)
- ❌ `a` (too vague)

Exception: In math-heavy code, single letters are okay: `x`, `y`, `z` for coordinates.

**Practice 2: Use snake_case for multi-word names**
- ✅ `first_name`
- ✅ `total_price`
- ✅ `is_logged_in`
- ❌ `firstName` (camelCase - used in JavaScript, not Python convention)
- ❌ `TotalPrice` (PascalCase - used for classes, not variables)

**Practice 3: Be consistent**
If you start with `user_name`, don't switch to `userAge` halfway through your program. Pick a style and stick with it.

**Practice 4: Avoid abbreviations (unless very common)**
- ✅ `maximum_score` (clear)
- ❌ `max_scr` (harder to read)
- ✅ `num_players` (num is widely understood)

---

## Practice Exercises

### Exercise 1: Create and Print Variables

Create a new file `variables.py` and write:

```python
name = "Your Name"
age = 25
city = "Your City"
favorite_food = "Pizza"

print("Name:", name)
print("Age:", age)
print("City:", city)
print("Favorite food:", favorite_food)
```

Run it: `python variables.py`

**Expected Output:**
```
Name: Your Name
Age: 25
City: Your City
Favorite food: Pizza
```

---

### Exercise 2: Modify Variables

Create `modify.py`:

```python
score = 0
print("Starting score:", score)

score = score + 10
print("After first round:", score)

score = score + 15
print("After second round:", score)

score = score - 5
print("After penalty:", score)

print("Final score:", score)
```

**Expected Output:**
```
Starting score: 0
After first round: 10
After second round: 25
After penalty: 20
Final score: 20
```

---

### Exercise 3: Calculate with Variables

Create `calculator.py`:

```python
length = 12
width = 8

area = length * width
perimeter = 2 * (length + width)

print("Rectangle dimensions:", length, "x", width)
print("Area:", area)
print("Perimeter:", perimeter)
```

**Expected Output:**
```
Rectangle dimensions: 12 x 8
Area: 96
Perimeter: 40
```

---

## Common Mistakes

### Mistake 1: Using a Variable Before Creating It

**Wrong:**
```python
print(name)
name = "Alex"
```

**Error:**
```
NameError: name 'name' is not defined
```

**Why:** Python reads line-by-line. Line 1 tries to print `name`, but `name` doesn't exist until line 2.

**Fix:** Create variables before using them:
```python
name = "Alex"
print(name)
```

---

### Mistake 2: Forgetting Quotes Around Text

**Wrong:**
```python
name = Alex
```

**Error:**
```
NameError: name 'Alex' is not defined
```

**Why:** Without quotes, Python thinks `Alex` is a variable name (which doesn't exist).

**Fix:**
```python
name = "Alex"  # Text needs quotes
age = 25       # Numbers don't need quotes
```

---

### Mistake 3: Using Invalid Variable Names

**Wrong:**
```python
first-name = "Alex"   # Dashes not allowed
2nd_place = "Bob"     # Can't start with number
user name = "Sarah"   # Spaces not allowed
```

**Fix:**
```python
first_name = "Alex"   # Use underscores
second_place = "Bob"  # Spell out "second"
user_name = "Sarah"   # No spaces
```

---

## For Curious Learners: Memory and the id() Function

*Optional section. You can skip this and still master variables.*

Every variable has a unique **memory address**—the actual location in your computer's RAM where the data is stored.

Python provides the `id()` function to see memory addresses:

```python
name = "Alex"
age = 25

print(id(name))  # Displays something like: 140234208567856
print(id(age))   # Displays something like: 140234208565232
```

The numbers are memory addresses (in decimal). They'll be different every time you run the program because Python allocates memory dynamically.

**Why This Matters:**

When you write:
```python
x = 5
y = x
```

Both `x` and `y` point to the **same memory location** initially. But if you reassign:
```python
x = 10
```

Now `x` points to a new location (10), but `y` still points to the old location (5). They're independent variables.

You don't need to think about memory addresses in everyday programming, but understanding that variables are *pointers to memory* helps explain some Python behaviors later (especially with lists and dictionaries in Chapter 18).

---

## Try With AI

### Prompt 1: Variable Naming Practice

```
I'm learning Python variable naming rules.

Give me 10 variable names. For each one, tell me:
1. Is it valid or invalid in Python?
2. If invalid, why?
3. If valid, is it following best practices (descriptive, snake_case)?

Examples to test:
- user_age
- 1st_place
- userName
- total-price
- _private
- my variable
- Score
- maxValue
- user123
- if
```

**What to look for:** AI should identify rules violations and explain best practices.

**Reflection:** Can you now write 5 valid, well-named variables for a game program?

---

### Prompt 2: Debug My Variable Code

```
I wrote this Python code but it has errors:

name = Alex
age = 25
print(age)
print(name)
score = score + 10
print(score)

Help me find all the bugs. For each bug:
1. What line is it on?
2. What's wrong?
3. How do I fix it?
```

**What to look for:** AI should catch: missing quotes, undefined variable `score`.

**Reflection:** After seeing the fixes, can you explain each error?

---

### Prompt 3: Build a Profile Program

```
Help me write a Python program that stores and displays a user profile.

Requirements:
- Create variables for: name, age, city, favorite_hobby
- Use my information: [provide your details]
- Print a formatted profile like:

  User Profile
  ------------
  Name: Alex
  Age: 25
  City: Boston
  Hobby: Photography

Show me the complete code, then explain how each variable is used.
```

**What to look for:** AI creates descriptive variables and formats output nicely.

**Reflection:** Run the program. Can you modify it to add two more profile fields?

---

### Prompt 4: Concept Check

```
Quiz me on variables.

Ask me 3 questions:
1. What is a variable and why do programs need them?
2. What's the difference between "age = 25" and "age == 25"?
3. Why is "user-name" an invalid variable name?

After each answer, tell me if I'm correct and explain.
```

**What to look for:** Can you answer without looking back?

**Reflection:** Which concept is clearest? Which needs review?

---

**Safety Note:** When practicing with AI-generated code, always run it yourself and verify it works as expected. The goal is understanding, not just copying code.

**Next Steps:** You've learned how to store data in variables! In [Lesson 4: Type Hints: Organizing Your Data](./04-type-hints-organizing-data.md), you'll learn how to tell Python what KIND of data each variable should hold.
