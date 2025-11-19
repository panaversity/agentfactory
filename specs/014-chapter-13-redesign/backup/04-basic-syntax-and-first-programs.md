---
title: "Basic Syntax and Your First Programs"
chapter: 13
lesson: 4
duration_minutes: 75

skills:
  - name: "Python Indentation and Code Structure"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student understands whitespace significance in Python and uses 4-space indentation consistently"

  - name: "Comments for Code Documentation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student writes comments explaining code intent and purpose"

  - name: "Print Function for Output"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student uses print() to display variables and text in terminal output"

  - name: "F-Strings for Formatted Output"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student creates f-strings with variable interpolation and displays formatted output"

  - name: "Running Python Programs"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student saves code to .py file, runs with `python filename.py`, and interprets output"

learning_objectives:
  - objective: "Write and run simple Python programs using syntax, comments, and output formatting"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student creates .py file with variables, comments, print, and f-strings; runs and verifies output"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (indentation, comments, print, f-strings, .py files) within A2 limit ✓"

differentiation:
  extension_for_advanced: "Explore older string formatting methods; research Python style guide (PEP 8) comprehensively"
  remedial_for_struggling: "Focus on simple print() before introducing f-strings; practice indentation with editor that shows whitespace"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/016-part-4-chapter-13/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 4: Basic Syntax and Your First Programs

You've created variables. Now you'll write full programs—code that does something and shows results.

In this lesson, you'll learn the syntax that makes Python unique: **indentation**, **comments**, how to **print output**, and how to **format text**. Most importantly, you'll learn the most important Python skill: **saving code to a file and running it**.

## Indentation: Python's Unique Syntax

Python is unusual among programming languages in one fundamental way: **it uses indentation (spaces) to show code structure.**

Unlike languages like JavaScript or Java that use curly braces `{}`, Python uses spaces.

```python
# Simple program with indentation
name: str = "Alice"
print(name)
```

This program is fine because both lines are at the beginning (no indentation).

Later, when you learn control flow (if statements, loops) in Chapters 16–17, you'll use indentation to show which code "belongs inside" the if statement or loop.

**The Standard**: Use **4 spaces per indentation level**. Not 2 spaces, not tabs, not 3 spaces. Four spaces. This is the Python standard (PEP 8).

**Pro Tip**: Configure your code editor to convert tabs to spaces automatically. Most modern editors (VS Code, Cursor, etc.) do this by default. If you accidentally mix tabs and spaces, Python will complain with an `IndentationError`.

## Comments: Explaining Your Code

A **comment** is a note in your code that Python ignores. Comments are for humans, not computers.

Comments start with `#`:

```python
# This is a comment explaining what the code does
age: int = 25

# This variable stores the user's name
name: str = "Alice"
```

Comments should explain **why** the code does something, not **what** it does. The code itself shows what it does.

Good comment:
```python
# We subtract 1 because Python counts from 0, not 1
index: int = current_position - 1
```

Poor comment:
```python
# Subtract 1 from current_position
index: int = current_position - 1
```

The poor comment just repeats what the code obviously does. The good comment explains the reasoning.

**Philosophy**: Code is for computers; comments are for humans (including your future self). Write comments you'd want to read in six months when you've forgotten why you wrote something.

## Print Function for Output

The `print()` function displays text and data to your terminal. It's how you see what your program is doing.

```python
print("Hello, World!")
```

Output:
```
Hello, World!
```

You can print variables:

```python
name: str = "Alice"
age: int = 25

print(name)      # Output: Alice
print(age)       # Output: 25
```

You can print multiple items:

```python
print("Name:", name, "Age:", age)
# Output: Name: Alice Age: 25
```

`print()` is the primary way you'll validate what your program is doing throughout Part 4.

## F-Strings: Modern Text Formatting

**F-strings** (formatted string literals) let you insert variables into text cleanly.

```python
name: str = "Alice"
age: int = 25

# Using f-string
print(f"My name is {name} and I am {age} years old")
# Output: My name is Alice and I am 25 years old
```

The `f` before the quote says "this is a formatted string." Variables go inside `{}` brackets.

Why f-strings instead of the old way?

Old way (not recommended):
```python
print("My name is " + name + " and I am " + str(age) + " years old")
```

Modern way (f-strings):
```python
print(f"My name is {name} and I am {age} years old")
```

F-strings are cleaner, easier to read, and more professional. This is what modern Python developers use.

#### 💬 AI Colearning Prompt
> "Explain why Python uses indentation (whitespace) for code blocks instead of curly braces like JavaScript or C++. What's the advantage for beginners? What's one potential problem?"

#### 🎓 Expert Insight
> In AI-native development, syntax is cheap—semantics is gold. We teach f-strings (not .format() or concatenation) because they're current. But syntax evolves: Python 2→3 changed dramatically, f-strings might be replaced in Python 3.20. Don't memorize syntax—understand patterns. "I want to combine text with variables" → ask AI for modern approach. That's the transferable skill.

## Creating and Running .py Files

You've been learning concepts, but now you'll create your first real program file.

### Step 1: Create a File

Open your text editor (VS Code, Cursor, or any code editor). Create a new file named `hello.py`.

The `.py` extension tells Python "this is a Python file."

### Step 2: Write Code

```python
# My first Python program
greeting: str = "Hello, Python!"
print(greeting)
```

Save the file as `hello.py` in a folder you can find (like your Desktop or Documents).

### Step 3: Run the Program

Open a terminal and navigate to the folder where you saved `hello.py`.

Type:
```
python hello.py
```

or on Mac/Linux:
```
python3 hello.py
```

Press Enter. Your program runs and displays:
```
Hello, Python!
```

Congratulations—you've written and executed your first Python program.

## Code Examples

### Example 1: Hello World with Variables

```python
# My first program
greeting: str = "Hello, Python!"
name: str = "Alice"

print(greeting)
print(name)
```

Output:
```
Hello, Python!
Alice
```

### Example 2: Variables and F-Strings

```python
# Introducing myself
name: str = "Bob"
age: int = 30
city: str = "Portland"

print(f"My name is {name}")
print(f"I'm {age} years old")
print(f"I live in {city}")
```

Output:
```
My name is Bob
I'm 30 years old
I live in Portland
```

### Example 3: Calculations and Output

```python
# Simple calculations
price: float = 19.99
quantity: int = 3
total: float = price * quantity

print(f"Price per item: ${price}")
print(f"Quantity: {quantity}")
print(f"Total: ${total}")
```

Output:
```
Price per item: $19.99
Quantity: 3
Total: $59.97
```

#### Tips

When you see an error you don't recognize, copy the error message and ask your AI: "What does this error mean?" This is a professional debugging skill.

**Indentation errors are frustrating but common.** They usually mean tabs and spaces got mixed. Use a text editor that shows whitespace (VS Code, Cursor). Your AI can help if you're stuck.

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a Python program that displays a mini 'About Me' card using: (1) typed variables for name, age, city, (2) f-strings for formatted output, (3) decorative borders using string multiplication (like '=' * 40), (4) comments explaining each section. Then explain why comments should describe 'why' not 'what'."

**Expected Outcome**: You'll practice combining syntax elements (variables, type hints, f-strings, comments, print statements), understand professional commenting practices, and see how string multiplication creates visual formatting.

## Common Mistakes

**Mistake 1**: Indentation errors (mixing tabs and spaces)

*Symptom*: `IndentationError: unexpected indent`

*Solution*: Use only spaces (not tabs). Configure your editor to show whitespace so you can see what's happening.

**Mistake 2**: Forgetting quotes around text

```python
print(Hello)      # ✗ Python looks for a variable named Hello
print("Hello")    # ✓ This prints the text Hello
```

Quotes tell Python "this is text, not a variable."

**Mistake 3**: Confusing `print()` with variable assignment

```python
print(age) = 25    # ✗ Can't assign to print
age = 25           # ✓ Create variable
print(age)         # ✓ Then print it
```

`print()` displays output. Assignment stores data. Different purposes.

**Mistake 4**: Using old string formatting

```python
# Old ways (don't do this)
"Name: " + name           # Concatenation
"Name: {}".format(name)   # Format method

# Modern way (do this)
f"Name: {name}"           # F-string
```

F-strings are cleaner and professional.

---

## Try With AI

Use your AI companion (Claude Code or Gemini CLI) for these prompts.

**Prompt 1: Recall – Syntax Elements**

```
What are the five syntax elements introduced in Lesson 4?
1. _______ (Python shows code structure with this)
2. _______ (marked with #, explains code)
3. _______ (function that displays output)
4. _______ (modern way to format text)
5. _______ (file extension for Python programs)

Bonus: What's the syntax for an f-string? (f"...{variable}...")
```

**Expected Outcome**: You recall core syntax. You demonstrate memorization of lesson concepts.

---

**Prompt 2: Understand – F-Strings vs. String Concatenation**

```
Explain the difference between these two approaches:
1. "Hello " + name + ", welcome to Python"
2. f"Hello {name}, welcome to Python"

Why does the lesson recommend f-strings?
Ask your AI: "What are the advantages of f-strings over string concatenation?"
```

**Expected Outcome**: You understand readability advantage. You learn professional conventions. You can explain why f-strings are preferred.

---

**Prompt 3: Apply – Write Your Own Program**

```
Create a Python program that:
1. Creates 3 variables with type hints (any types/values you choose)
2. Uses print() and f-strings to display them in a sentence
3. Includes at least 2 comments explaining what the code does

Example output might be:
"My name is Alice, I'm 25 years old, and I live in Portland"

Save it as `my_program.py`. Run it with `python my_program.py`.
Show your AI the code: "Does this look correct? Any improvements?"
```

**Expected Outcome**: You write working program. You demonstrate mastery of variables, type hints, print, and f-strings. You practice code review with AI.

---

**Prompt 4: Analyze – Error Interpretation and Debugging (Cognitive Closure)**

```
Here's a broken program—can you fix it?

name: str = "Alex"
age int = 30  # ← Something is wrong here!
print(f"Name: {name}, Age: {age}")

Questions:
1. What's the error in this code? (Hint: look at line 2)
2. Why is it an error? (What rule did it break?)
3. How would you fix it?
4. If Python gave you an error message, what would it say?

Ask your AI: "Here's the error message I got: [paste error]. Can you explain what went wrong?"
```

**Expected Outcome**: You identify missing type hint syntax (colon). You understand error messages. You learn debugging process. You demonstrate analytical thinking about code. You close Lesson 4 with confidence in syntax.

