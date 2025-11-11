---
title: "Your First Python Program: Hello World and Print"
chapter: 13
lesson: 3
duration_minutes: 40

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Writing and Running Python Files"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create a .py file; write print() statements; run the file from terminal; see output"

  - name: "Using print() for Output"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can use print() with strings; print multiple lines; use basic f-strings for formatted output"

  - name: "Understanding Python Comments"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information & Data Literacy"
    measurable_at_this_level: "Student can write comments with # to document code intent"

learning_objectives:
  - objective: "Create and run your first Python program file"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student creates hello.py, runs it, sees expected output"

  - objective: "Use print() to display text and formatted messages"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes 3+ print statements with different content"

  - objective: "Add comments to explain code intent"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student adds # comments explaining what code does"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (Creating .py files, print() function, Comments with #, Running Python files) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore escape sequences (\\n, \\t); try multi-line strings with triple quotes; experiment with print() separator and end parameters"
  remedial_for_struggling: "Start with one print statement. Run it. Change the text. Run again. Build confidence through repetition before adding more"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/016-part-4-chapter-13/spec.md"
source_plan: "specs/016-part-4-chapter-13/plan.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "manual restructuring - Option A clean separation"
version: "1.0.0"
---

# Your First Python Program: Hello World and Print

**Building on Lesson 2**: In the last lesson, you installed Python and verified it works using `python -c "print('Hello, Python!')"`. That was a **one-line program** run directly from the terminal. Now you'll learn to create **Python files** that you can save, edit, and run repeatedly.

This lesson teaches you the most fundamental Python operation: displaying text to the screen using `print()`. This is how programs communicate with users—and it's the first skill you need before learning anything more complex.

---

## What Is a Python Program File?

In Lesson 2, you ran Python code directly in the terminal:
```bash
python -c "print('Hello, Python!')"
```

That works for quick tests, but real programs are saved in **files** with the `.py` extension. These files:
- Store your code so you can edit it later
- Can be run repeatedly without retyping
- Can be shared with others
- Can grow from 1 line to thousands of lines

**A Python file is just a text file** with a `.py` extension containing Python instructions. The Python interpreter reads the file and executes each instruction in order.

---

## Creating Your First Python File

**Step 1: Create a new file called `hello.py`**

Using any text editor (VS Code, Cursor, Notepad, TextEdit, etc.), create a new file and save it as `hello.py`.

**Step 2: Write your first program**

Type this into `hello.py`:

```python
print("Hello, World!")
```

Save the file.

**Step 3: Run your program**

Open your terminal, navigate to the folder where you saved `hello.py`, and run:

```bash
python hello.py
```

**You should see:**
```
Hello, World!
```

**🎉 Congratulations!** You just wrote and ran your first Python program from a file.

---

## Understanding print()

The `print()` function is how Python displays text to the screen (technically, to "standard output" which is your terminal).

**Basic syntax:**
```python
print("Your message here")
```

- **`print`**: The function name
- **`()`**: Parentheses tell Python you're calling a function
- **`"Your message here"`**: The text to display (inside quotes)

**Why quotes?** The quotes tell Python "this is text, not code." Without quotes, Python thinks you're referencing a variable or keyword.

---

## Multiple print() Statements

You can call `print()` as many times as you want. Each call displays a new line.

**Example** (`multiple_prints.py`):
```python
print("Welcome to Python!")
print("This is line 2")
print("This is line 3")
```

**Output:**
```
Welcome to Python!
This is line 2
This is line 3
```

Each `print()` adds a newline automatically. That's why they appear on separate lines.

---

## Adding Comments to Your Code

**Comments** are notes you write in your code that Python ignores. They're for humans to read, not for the computer to execute.

Use `#` to start a comment:

```python
# This is a comment - Python ignores this line
print("Hello, World!")  # You can also add comments after code
```

**Why use comments?**
- Explain *why* you wrote code a certain way
- Add reminders for yourself
- Help others (or your future self) understand your thinking

**Example:**
```python
# Display a greeting to the user
print("Hello, World!")

# Show multiple lines of output
print("Python is readable")
print("Python is powerful")
```

**Best practice:** Comment the *why*, not the obvious *what*. Don't write `# Print hello` before `print("Hello")`. That's obvious. Write `# Greet the user before collecting their input` to explain the *purpose*.

---

## Formatted Output with f-strings (Show-Then-Explain)

Python has a powerful feature called **f-strings** (formatted string literals) that let you embed expressions inside strings.

**Show first:**
```python
print(f"Python version: 3.14")
print(f"Status: Ready")
print(f"Next chapter: Data Types")
```

**Output:**
```
Python version: 3.14
Status: Ready
Next chapter: Data Types
```

**What's happening:**
- The `f` before the opening quote makes it an **f-string**
- F-strings let you format text dynamically
- In Chapter 14, you'll use f-strings with **variables** to display changing data

**For now**, just know that `f"..."` is how Python formats strings. You'll use this extensively when you learn variables in the next chapter.

---

## Combining Strings with + (Concatenation)

You can join strings together using `+`:

```python
print("Hello" + " " + "World")
```

**Output:**
```
Hello World
```

Notice we added `" "` (a space) between the words. Without it:
```python
print("Hello" + "World")  # Output: HelloWorld
```

**Concatenation** means "joining strings together." You'll use this when building messages from multiple pieces of text.

---

## Try It Yourself: Personal Greeting Program

Create a file called `greeting.py` and write:

```python
# Personal greeting program
# This displays information about me

print("=" * 40)  # Print 40 equal signs (a decorative line)
print("About Me")
print("=" * 40)
print()  # Print a blank line
print("Name: [Your Name]")
print("Learning: Python Programming")
print("Tool: Claude Code")
print("Goal: Build AI applications")
print()
print("Next step: Learning data types in Chapter 14")
```

**Run it:**
```bash
python greeting.py
```

**Expected output:**
```
========================================
About Me
========================================

Name: [Your Name]
Learning: Python Programming
Tool: Claude Code
Goal: Build AI applications

Next step: Learning data types in Chapter 14
```

**What's new here:**
- `"=" * 40`: Repeats the `=` character 40 times (string multiplication)
- `print()` with no arguments: Prints a blank line
- Multiple `print()` statements to build formatted output

---

## Common Mistakes (and How to Fix Them)

### Mistake 1: Forgetting Quotes
```python
print(Hello)  # ❌ Error: NameError: name 'Hello' is not defined
```

**Fix:** Add quotes around text:
```python
print("Hello")  # ✅ Correct
```

### Mistake 2: Mismatched Quotes
```python
print("Hello')  # ❌ Error: SyntaxError
```

**Fix:** Use matching quotes (both `"` or both `'`):
```python
print("Hello")  # ✅ Correct
print('Hello')  # ✅ Also correct
```

### Mistake 3: Missing Parentheses
```python
print "Hello"  # ❌ Error in Python 3 (works in Python 2)
```

**Fix:** Always use `print()` with parentheses:
```python
print("Hello")  # ✅ Correct
```

### Mistake 4: Typo in Function Name
```python
pint("Hello")  # ❌ Error: NameError: name 'pint' is not defined
```

**Fix:** Spell `print` correctly:
```python
print("Hello")  # ✅ Correct
```

---

## What You Learned

By completing this lesson, you now know how to:

✅ Create Python files with the `.py` extension
✅ Use `print()` to display text to the screen
✅ Write comments with `#` to document your code
✅ Run Python programs from the terminal
✅ Use basic f-strings for formatted output
✅ Combine strings with `+` (concatenation)
✅ Recognize and fix common print() mistakes

**Next chapter preview:** In Chapter 14 (Data Types), you'll learn to **store data in variables** using type hints. Instead of hardcoding text in `print()`, you'll store information and display it dynamically. Variables are where Python gets powerful—and where AI collaboration becomes essential.

---

## Try With AI

Practice using your AI learning partner (Claude Code or Gemini CLI) to deepen understanding:

**Prompt 1: Understand – What Is print() Really Doing? (Bloom's Level 2: Understand)**

```
Ask your AI: "When I run print('Hello'), what actually happens? Where does the
text go? What is 'standard output'? Explain in simple terms with an analogy."
```

**Expected Outcome**: You'll understand that `print()` writes to "standard output" (usually your terminal), and how this relates to other output destinations like files or network sockets.

---

**Prompt 2: Apply – Create Formatted Output (Bloom's Level 3: Apply)**

```
Ask your AI: "Show me 3 ways to print a decorative border around text. Use different
characters (* / = -) and make it look nice. Include the code and explain why each
approach works."
```

**Expected Outcome**: You'll see creative uses of `print()` with string multiplication and learn techniques for formatting terminal output.

---

**Prompt 3: Analyze – Comments vs. Code Clarity (Bloom's Level 4: Analyze)**

```
Show your AI this code:

print("Hello")  # Print hello
print("World")  # Print world

Ask: "Are these comments useful? Why or why not? Show me an example of
GOOD comments for this code. Explain the difference between commenting
the 'what' vs. the 'why'."
```

**Expected Outcome**: You'll learn professional commenting practices—when to comment, what to comment, and how to make comments valuable (not redundant).

---

**Prompt 4: Create – Build a Story Program (Bloom's Level 6: Create)**

```
Ask your AI: "Help me create a Python program that tells a short story (3-5 lines)
using print(). The story should have:
- A character introduction
- An action or event
- A conclusion or moral
Show the code and explain how to make it visually appealing with spacing and formatting."
```

**Expected Outcome**: You'll practice creative use of `print()` and learn how to structure program output for readability. This reinforces the fundamental skill before adding complexity in Chapter 14.

---

**You've completed Chapter 13!** You now understand what Python is, have it installed, and can write basic programs using `print()`. In **Chapter 14: Data Types**, you'll learn to store and manipulate data using **variables with type hints**—the foundation for all Python programming.
