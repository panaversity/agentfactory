---
title: "Your First Python Program"
chapter: 13
lesson: 2
duration_minutes: 35
proficiency_level: "CEFR A2"
blooms_level: "Apply"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Running Python Programs in Terminal"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can navigate to a directory, create a .py file, and execute it using 'python filename.py' command"

  - name: "Using the print() Function"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can write print() statements with strings, understand output appears in terminal, and fix common syntax errors (missing quotes, parentheses)"

learning_objectives:
  - objective: "Execute a Python program from the terminal"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student creates hello.py file, runs 'python hello.py', and sees output in terminal"

  - objective: "Write print() statements to display text"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes 3-5 different print() statements and predicts what each will output before running"

  - objective: "Troubleshoot common syntax errors in print() statements"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student identifies and fixes missing quotes, mismatched quotes, and missing parentheses errors"

cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (running Python programs, print() function) at A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Explore print() with multiple arguments, sep and end parameters, formatted output with escape sequences (\\n, \\t)"
  remedial_for_struggling: "Practice with single print('Hello') before variations; use online Python playground (repl.it) if terminal intimidating"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/014-chapter-13-redesign/spec.md"
created: "2025-01-18"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Your First Python Program

## What You'll Learn

- How to Run Python Programs
- The print() Function
- "Hello World" Program
- Common Syntax Errors

---

## Opening: Your Robot Speaks Its First Words

Remember our robot from Lesson 1? You taught it what programming is (giving instructions). Now it's time for a breakthrough moment: **the robot speaks**.

Imagine you've spent weeks teaching your robot to understand language. It's learned words, grammar, syntax. But it's never actually *said* anything. Today, you'll give it one simple instruction:

**"Robot, say 'Hello, World!'"**

And for the first time, the robot opens its mouth (or speaker, or text display) and responds:

```
Hello, World!
```

That simple phrase might seem trivial, but it's **proof of success**. Your robot understands you. It can follow instructions. It can produce output. This is the moment every programmer experiences when they write their first program—the moment the computer *listens* and *responds*.

In this lesson, you'll have that exact experience. You'll write a program, run it, and see the computer follow your instructions. This is where programming stops being theory and becomes **real**.

---

## Running Python Programs: How It Works

Before you write code, let's understand the process. How do you actually *run* a Python program?

### Understanding Python Programs

A **Python program** is just a text file with instructions written in Python language. The file extension is `.py` (like `hello.py` or `calculator.py`).

When you **run** the program, you're telling your computer: "Read this file, interpret the Python instructions, and execute them."

The result appears in your **terminal** (also called command line or shell)—the text-based interface where you type commands.

### Why Use the Terminal?

Why use the terminal instead of clicking a button? Two reasons:

1. **Universal Interface** — Terminals work the same way on Windows, Mac, and Linux. Once you learn terminal commands, you can program anywhere.
2. **Professional Workflow** — Real developers use terminals constantly. Web servers, databases, deployment tools—all controlled via command line. Learning this now builds professional habits.

### How to Run a Python Program

Here's the step-by-step workflow for running a Python program:

**Step 1: Create a Python file**
- Open your text editor (VS Code, Zed, Sublime, or any editor)
- Write your Python code
- Save the file with a `.py` extension (e.g., `hello.py`)

**Step 2: Open your terminal**
- On **Windows**: Press `Win + R`, type `cmd`, press Enter
- On **Mac**: Press `Cmd + Space`, type `terminal`, press Enter
- On **Linux**: Press `Ctrl + Alt + T`

**Step 3: Navigate to your file's directory**
- Use `cd` (change directory) command to go to where you saved your file
- Example: `cd Documents/python-practice`

**Step 4: Run the program**
- Type: `python hello.py` (or `python3 hello.py` on some systems)
- Press Enter

**Step 5: See the output**
- Your program's results appear in the terminal
- If there are errors, error messages appear here too

That's it! Every Python program follows this process.

---

## The print() Function: Your First Tool

Now that you know *how* to run programs, let's learn your first Python command: **print()**.

### Introducing the print() Function

`print()` is a **built-in function** in Python that displays text (or other data) in the terminal.

Think of it as Python's "voice." When you call `print("Hello")`, Python "speaks" the word "Hello" by displaying it on screen.

The syntax looks like this:

```python
print("Your message here")
```

- **`print`** — The function name (Python's built-in command)
- **`()`** — Parentheses that hold the data you want to display
- **`"Your message here"`** — The text (called a **string**) to display, enclosed in quotes

### Why print() Matters

Why is `print()` important? Because **programs need to communicate results**.

Imagine a calculator program that computes `5 + 3 = 8` but never shows you the answer. Useless, right? `print()` is how Python shows you results, progress messages, error information—anything you need to see.

Examples of what `print()` does:
- Show calculation results: `print(5 + 3)` → displays `8`
- Display messages: `print("Task complete!")` → confirms success
- Debug programs: `print("Reached line 42")` → helps you track execution
- Greet users: `print("Welcome to my program")` → friendly introduction

**Without `print()`, you'd have no way to see what your program is doing.**

### Using print() in Different Ways

Here are several ways to use `print()`:

**Basic Text:**
```python
print("Hello, World!")
```
Output: `Hello, World!`

**Numbers:**
```python
print(42)
```
Output: `42`

**Math Calculations:**
```python
print(10 + 5)
```
Output: `15`

**Multiple Items (separated by spaces):**
```python
print("The answer is", 42)
```
Output: `The answer is 42`

**Empty Line:**
```python
print()
```
Output: *(blank line)*

---

## Your First Program: "Hello, World!"

It's time. Let's write and run your first Python program.

### Why "Hello, World!"?

Every programming language tutorial starts with "Hello, World!" It's a tradition dating back to 1972 when Brian Kernighan wrote the first C programming book. The idea: **prove the basics work** with the simplest possible program.

When you successfully run "Hello, World!", you've confirmed:
- ✅ Python is installed correctly
- ✅ You can create and save `.py` files
- ✅ You can run programs from the terminal
- ✅ The `print()` function works

### Step-by-Step Guide

**Step 1: Create the file**
- Open your text editor
- Type this exactly:

```python
print("Hello, World!")
```

- Save as `hello.py` in a folder you can easily find (like `Documents/python-learning`)

**Step 2: Open terminal and navigate**
- Open your terminal
- Navigate to the folder where you saved `hello.py`:
  ```bash
  cd Documents/python-learning
  ```
- Verify the file is there:
  ```bash
  ls          # Mac/Linux
  dir         # Windows
  ```
  You should see `hello.py` listed.

**Step 3: Run the program**
- Type:
  ```bash
  python hello.py
  ```
  (If that doesn't work, try `python3 hello.py`)

**Step 4: See the magic**
- Your terminal displays:
  ```
  Hello, World!
  ```

**Congratulations!** You just wrote and executed your first Python program. The computer followed your instruction and spoke the words you told it to say.

---

## Practice Exercises

Now that you've run "Hello, World!", practice with variations:

### Exercise 1: Personal Greeting
Modify `hello.py` to print your own name:

```python
print("Hello, my name is [Your Name]!")
```

Run it: `python hello.py`

**Expected Output:**
```
Hello, my name is Alex!
```
(or whatever name you chose)

---

### Exercise 2: Multiple Print Statements
Edit `hello.py` to include several print statements:

```python
print("Welcome to my first Python program!")
print("My name is Alex")
print("I'm learning to code")
print("This is exciting!")
```

Run it: `python hello.py`

**Expected Output:**
```
Welcome to my first Python program!
My name is Alex
I'm learning to code
This is exciting!
```

**Key Insight:** Each `print()` statement creates a new line. Programs execute line-by-line, top to bottom.

---

### Exercise 3: Print Numbers and Math
Create a new file called `math.py`:

```python
print(100)
print(50 + 50)
print(10 * 5)
print("The answer is", 42)
```

Run it: `python math.py`

**Expected Output:**
```
100
100
50
The answer is 42
```

**Key Insight:** `print()` works with numbers and text. You can even mix them (last line).

---

## Common Mistakes and How to Fix Them

Every beginner makes these mistakes. Here's how to spot and fix them:

### Mistake 1: Forgetting Quotes Around Text

**Wrong:**
```python
print(Hello, World!)
```

**Error Message:**
```
SyntaxError: invalid syntax
```

**Why It Fails:** Python thinks `Hello` and `World` are variable names (not text). Without quotes, Python looks for variables that don't exist.

**Fix:**
```python
print("Hello, World!")
```

**Rule:** Always put quotes around text (strings). Numbers don't need quotes.

---

### Mistake 2: Mismatched Quotes

**Wrong:**
```python
print("Hello, World!')
```

**Error Message:**
```
SyntaxError: unterminated string literal
```

**Why It Fails:** You started with `"` but ended with `'`. Python gets confused.

**Fix:** Use matching quotes:
```python
print("Hello, World!")   # Both double quotes ✓
print('Hello, World!')   # Both single quotes ✓
```

**Rule:** Opening and closing quotes must match. Either use `"..."` or `'...'`, but not mixed.

---

### Mistake 3: Forgetting Parentheses

**Wrong:**
```python
print "Hello, World!"
```

**Error Message:**
```
SyntaxError: Missing parentheses in call to 'print'
```

**Why It Fails:** In Python 3, `print` is a function. Functions require parentheses.

**Fix:**
```python
print("Hello, World!")
```

**Rule:** `print()` always needs parentheses, even for empty lines: `print()`

---

### Mistake 4: Typos in Function Name

**Wrong:**
```python
pritn("Hello, World!")
```

**Error Message:**
```
NameError: name 'pritn' is not defined
```

**Why It Fails:** `pritn` isn't a Python function. You meant `print`.

**Fix:**
```python
print("Hello, World!")
```

**Rule:** Spelling matters. `print`, `Print`, and `PRINT` are all different to Python (case-sensitive).

---

## For Curious Learners: Why Indentation Matters in Python

*This section is optional. You can skip it and still master this lesson. But if you're curious about Python's unique feature, read on.*

One thing that makes Python different from other languages: **indentation is part of the syntax**.

In many languages (like JavaScript or Java), indentation is just for human readability. You could write everything on one line and it would still work (though it'd be ugly).

**Python is different.** Indentation tells Python which lines of code belong together.

You'll see this in Lesson 5 when we use `if` statements, but here's a preview:

**Correct:**
```python
age = 25
if age >= 18:
    print("You are an adult")
    print("You can vote")
print("Thank you")
```

**Wrong:**
```python
age = 25
if age >= 18:
print("You are an adult")
    print("You can vote")
print("Thank you")
```

The second version would cause `IndentationError` because Python can't tell which lines are inside the `if` statement.

**Why This Matters:** Unlike other languages that use `{}` braces to group code, Python uses indentation (usually 4 spaces). This forces code to be readable.

**For now, don't worry about this.** All the code in Lesson 2 has no indentation because we're not using structures that require it yet. Just know: when we get to `if` statements and loops (later chapters), indentation will matter.

---

## Try With AI

Use your AI companion to practice `print()` variations. These prompts build from simple to creative.

### Prompt 1: Print() Variations

```
I just learned the print() function in Python.

Show me 5 different ways to use print() that I didn't see in the lesson.
Include:
1. Printing multiple items on one line
2. Printing special characters
3. Printing with different quote types
4. Printing empty lines
5. One creative example

For each example, explain what it does.
```

**What to look for:** You'll learn new `print()` techniques. Try running each example yourself.

**Reflection:** Which example was most surprising? Why?

---

### Prompt 2: Fix My Errors

```
I wrote this Python code but it has 3 syntax errors:

print(Hello, World!')
pritn("My name is Alex)
Print("I'm learning Python")

Help me find and fix all 3 errors.
For each error, explain:
1. What's wrong?
2. Why Python can't run it?
3. What's the correct version?
```

**What to look for:** AI should identify: missing opening quote, typo in `pritn`, capital `P` in `Print`, missing closing quote.

**Reflection:** After seeing the fixes, can you explain each error in your own words?

---

### Prompt 3: Create a Story Program

```
Help me write a Python program that tells a short story using print() statements.

Requirements:
- At least 5 print() statements
- Tell a simple story (your choice of topic)
- Use my name (Alex) as the main character
- Include one print() with a number

After you show me the code, explain why you structured it that way.
```

**What to look for:** AI creates a narrative using `print()`. Run the program and see the story unfold.

**Reflection:** How did AI use `print()` creatively? Can you modify the story to make it your own?

---

### Prompt 4: Test Your Understanding

```
Quiz me on print() function concepts.

Ask me 3 questions:
1. What does print() do in Python?
2. Why do we need quotes around text but not around numbers?
3. What happens if I forget the closing parenthesis in print()?

After each answer, tell me if I'm correct and clarify any misconceptions.
```

**What to look for:** Can you answer without looking back? If not, review the lesson sections.

**Reflection:** Which concept is clearest? Which needs more practice?



