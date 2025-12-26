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
source_spec: "specs/016-part-4-chapter-15/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Basic Syntax and Your First Programs

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

Throughout this chapter and beyond, we'll be building toward a Todo application—a practical project that brings together everything you learn. Let's start with your first program that previews this goal:

```python
# Your first Python program with a purpose
print("Hello, Todo App!")
print("Let's build something useful together.")

# Preview: This is what we're building toward
task = "Learn Python basics"
print(f"First task: {task}")
```

This program introduces the Todo context you'll develop throughout the chapter. Save the file as `hello.py` in a folder you can find (like your Desktop or Documents).

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

### Example 4: Your First Task (Todo Preview)

As we build toward a Todo application, here's how you create and display your first task using simple variables:

```python
# Your first task - just a simple variable for now
task_title = "Complete Python basics lesson"
task_done = False
print(f"Task: {task_title}")
print(f"Completed: {task_done}")
```

Output:
```
Task: Complete Python basics lesson
Completed: False
```

This simple structure is the foundation for the Todo application you'll build as you progress through this chapter. Notice how `task_title` stores text and `task_done` stores whether the task is complete. As you learn more Python concepts—lists, dictionaries, classes, and file I/O—you'll enhance this basic structure into a fully functional Todo manager.

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

Can you write a complete program and improve it through code review?

**🔍 Explore Code Review Process:**
> "Review this product information program for: correct type hints, proper indentation, good comments, and professional f-string usage. Then explain: What makes a comment GOOD (explains why) vs BAD (repeats what the code does)? [Paste your program code]"

**🧪 Test Comment Quality:**
> "Compare these two comment styles: `price: float = 29.99  # Set price to 29.99` vs `price: float = 29.99  # Launch price includes 25% promotional discount`. Which is better and why? Then review the comments in my program—are they explaining WHY or just repeating WHAT? Suggest improvements."

**🎯 Practice Program Enhancement:**
> "Improve this product display program by adding: a formatted header/footer using string multiplication (like `'=' * 40`), better visual spacing, currency formatting for price (show $ symbol), and boolean to 'Yes'/'No' text conversion. Show me the enhanced version and explain what each improvement does."

**🚀 Debug Broken Code:**
> "Find and fix ALL errors in this code: `product_name str = 'Keyboard'` and `quantity int = 20`. Explain WHY each is an error (missing colon? wrong syntax?) and show the corrected version. What Python syntax rules do these violations break?"

---

