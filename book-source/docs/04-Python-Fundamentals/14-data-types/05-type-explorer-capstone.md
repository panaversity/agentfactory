---
title: "Building a Type Explorer — Capstone Integration"
chapter: 14
lesson: 5
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Integrating Core Data Types"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can build a complete program using all core types (int, float, str, bool, None) in a single project"

  - name: "Type Validation and Conversion"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use isinstance() to validate types and handle conversion errors safely"

  - name: "AI-Guided Program Improvement"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can ask AI for code review, identify improvements, and apply feedback to extend a working program"

learning_objectives:
  - objective: "Build a working interactive program that demonstrates all core data types from Chapter 14"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Complete type explorer program that runs without errors and displays all five core types"

  - objective: "Apply type hints throughout a complete program to specify data intent"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Every function and variable in the type explorer includes appropriate type hints"

  - objective: "Use type validation techniques (isinstance(), type()) to explore type characteristics"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Program demonstrates type checking, type inspection, and truthy/falsy conversion"

  - objective: "Collaborate with AI to review, improve, and extend the type explorer"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student asks AI for code review, identifies at least two improvements, and applies one to their program"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (program integration, type validation, type conversion, error handling, function organization, AI code review) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Add a user input section that converts strings to numbers safely. Create a function that asks the user for their age as text, converts it to int with error handling, and validates it's a reasonable age. Then extend the type explorer to demonstrate what happens when conversion fails."
  remedial_for_struggling: "Start with just 2 types (int and str). Build those functions first. Test them thoroughly. Then add one type at a time (float, bool, None). Celebrate each success before adding the next. Quality over speed."

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

# Building a Type Explorer — Capstone Integration

## Welcome to Your Capstone Project

You've spent Lessons 1-4 learning Python's core data types one at a time:

- **Lesson 1**: Variables and type hints (the foundation)
- **Lesson 2**: Integers and floats (numbers)
- **Lesson 3**: Strings and booleans (text and decisions)
- **Lesson 4**: Collections awareness and None (everything else)

Now it's time to **integrate everything into a working program** that brings all these concepts together.

In this capstone lesson, you'll build an **interactive type explorer**—a program that demonstrates how each data type works, validates types, and shows why type hints matter. This is a real Python program (~70 lines) that you can run, modify, and extend.

By the end, you'll understand not just individual types, but how types work together in a complete program. You'll also learn how to collaborate with AI to review and improve your code.

---

## Project Overview: What Your Type Explorer Does

Your type explorer program will:

1. **Display each core type** with a clear example
2. **Show what `type()` returns** (type inspection)
3. **Validate types with `isinstance()`** (type checking)
4. **Demonstrate truthy/falsy conversion** (boolean context)
5. **Use type hints throughout** (specifications in code)

When you run it, you'll see output like:

```
Welcome to the Python Type Explorer!
Let's explore Python's core data types together.

=== Exploring Integer (int) ===
Example: age = 25
Type: <class 'int'>
Is this an int? True

=== Exploring Float (float) ===
[... and so on for each type]
```

The program teaches concepts through demonstration, not just explanation. **You'll learn by running it, understanding it, and then improving it with AI assistance.**

---

## Building the Type Explorer: Specification-First Workflow

In this capstone, you'll learn the **core AI-native development workflow**: Specify → AI Builds → Validate → Iterate. This is the same process professionals use to build software with AI collaborators.

### Step 1: Write Your Specification

Before asking AI for code, define exactly what your type explorer should do. This is your **specification**:

```
Specification: Type Explorer Program

Purpose: Demonstrate all 5 core Python data types interactively

Requirements:
1. Display a welcome message
2. For each type (int, float, str, bool, None):
   - Create an example variable with a type hint
   - Print the variable value
   - Print what type() returns
   - Print whether isinstance() validates it
3. Add a truthy/falsy demonstration for booleans
4. End with a summary of all 5 types
5. Use only concepts from Chapters 13-14 (no functions, loops, or error handling)

Success Criteria (Evals):
- [ ] All 5 types demonstrated
- [ ] Each type uses type() and isinstance()
- [ ] All variables have type hints
- [ ] Program runs without errors
- [ ] Output is clear and educational
- [ ] Code uses only print() and basic syntax (no advanced features)
```

**Key Insight**: Notice we wrote WHAT we want (the specification) before HOW to build it (the code). This is specification-first development.

---

### Step 2: Tell Your AI

Now that you have a clear specification, translate it into a prompt for your AI collaborator.

**Copy this prompt and paste it into Claude Code, Gemini CLI, or ChatGPT:**

```
Create a Python program called "Type Explorer" that demonstrates 5 core data types:
int, float, str, bool, and None.

Requirements:
1. Print a welcome message at the start
2. For each type, create an example variable with a type hint:
   - int: age = 25
   - float: temperature = 98.6
   - str: name = "Alex"
   - bool: is_student = True
   - None: value = None

3. For each type, print:
   - The variable value
   - What type() returns
   - Whether isinstance() validates it

4. Add a section showing truthy/falsy conversion:
   - bool(0)
   - bool(1)
   - bool('') (empty string)
   - bool('hello') (non-empty string)

5. End with a summary listing all 5 types

Constraints:
- Use type hints for all variables
- Use only print() for output (no f-strings)
- Keep it sequential (no functions, loops, or error handling)
- Add clear comments to organize sections

Make the output educational and easy to read.
```

**Important**: Don't just copy this prompt. Read it carefully and understand what you're asking for. This is how you learn to write good specifications.

---

### Step 3: Review AI's Output

Your AI will generate code similar to this (~70 lines):

```python
# Interactive Type Explorer
# Demonstrates core data types: int, float, str, bool, None
# Uses type hints to describe intent clearly

print("Welcome to the Python Type Explorer!")
print("Let's explore Python's core data types together.")

# Exploring Integer (int)
print("\n=== Exploring Integer (int) ===")
age: int = 25
print("Example: age =", age)
print("Type:", type(age))
print("Is this an int?", isinstance(age, int))

# Exploring Float (float)
print("\n=== Exploring Float (float) ===")
temperature: float = 98.6
print("Example: temperature =", temperature)
print("Type:", type(temperature))
print("Is this a float?", isinstance(temperature, float))

# Exploring String (str)
print("\n=== Exploring String (str) ===")
name: str = "Alex"
print("Example: name =", name)
print("Type:", type(name))
print("Is this a string?", isinstance(name, str))

# Exploring Boolean (bool)
print("\n=== Exploring Boolean (bool) ===")
is_student: bool = True
print("Example: is_student =", is_student)
print("Type:", type(is_student))
print("Is this a bool?", isinstance(is_student, bool))

# Truthy/falsy demonstration
print("\nTruthy/Falsy Examples:")
print("bool(0) =", bool(0))          # False
print("bool(1) =", bool(1))          # True
print("bool('') =", bool(''))        # False (empty string)
print("bool('hello') =", bool('hello'))  # True (non-empty string)

# Exploring None
print("\n=== Exploring None ===")
value: None = None
print("Example: value =", value)
print("Type:", type(value))
print("Is this None?", value is None)

# Summary
print("\n=== Summary ===")
print("You've now explored all core Python types:")
print("  - int: Whole numbers (counting)")
print("  - float: Decimal numbers (measuring)")
print("  - str: Text (words and characters)")
print("  - bool: True/False (decisions)")
print("  - None: No value (placeholders)")
```

**Don't just copy this code!** First, review it carefully:

1. **Does it match your specification?**
   - All 5 types? ✓ or ✗
   - type() and isinstance() for each? ✓ or ✗
   - Type hints throughout? ✓ or ✗
   - Truthy/falsy demo? ✓ or ✗
   - Summary at end? ✓ or ✗

2. **Do you understand each section?**
   - What does this line do?
   - Why is this type hint here?
   - What would happen if I changed this value?

3. **Ask AI to explain anything unclear**:

   **Tell your AI**: "Explain this section line by line: [paste code section]"

**This review step is critical.** You're learning to validate AI output against specifications.

---

### Step 4: Validate Against Success Criteria

Now create a file called `type_explorer.py` and paste the AI's code. Run it:

```bash
python type_explorer.py
```

**Check each success criterion** from Step 1:

- [ ] All 5 types demonstrated? (Run and verify output)
- [ ] Each type uses type() and isinstance()? (Check code)
- [ ] All variables have type hints? (Check code)
- [ ] Program runs without errors? (Run it)
- [ ] Output is clear and educational? (Read output)
- [ ] Code uses only print() and basic syntax? (Check code)

**Expected output:**
```
Welcome to the Python Type Explorer!
Let's explore Python's core data types together.

=== Exploring Integer (int) ===
Example: age = 25
Type: <class 'int'>
Is this an int? True

=== Exploring Float (float) ===
Example: temperature = 98.6
Type: <class 'float'>
Is this a float? True

[... more output for str, bool, None]

=== Summary ===
You've now explored all core Python types:
  - int: Whole numbers (counting)
  - float: Decimal numbers (measuring)
  - str: Text (words and characters)
  - bool: True/False (decisions)
  - None: No value (placeholders)
```

If all criteria pass: **Congratulations! You've completed the capstone using specification-first workflow.**

If any fail: Move to Step 5.

---

### Step 5: Iterate with AI

If something doesn't match your specification or success criteria, ask AI to fix it:

**Example iteration prompts:**

**If output isn't clear:**
```
The output is hard to read. Add blank lines between each type section
to make it more organized.
```

**If you want to add a feature:**
```
Add a new section that demonstrates type conversion between int and float.
Show examples of int(3.7) and float(5).
```

**If you want to understand better:**
```
I don't understand why isinstance() is used instead of type() == int.
Explain the difference and show me an example where it matters.
```

**This is the AI-native workflow**: Specify → Build → Validate → Iterate.

You keep refining until all success criteria pass and you understand the code fully.

---

## Code Walkthrough: Understanding the Key Patterns

Let's break down the key sections and understand how they work together.

### Section 1: Comments Organize the Code

```python
# Interactive Type Explorer
# Demonstrates core data types: int, float, str, bool, None
# Uses type hints to describe intent clearly
```

These **comments** at the top explain what the program does. When someone (or an AI) reads this file, they immediately understand the purpose.

### Section 2: Type Inspection and Validation Pattern

The program follows the same pattern for each type:

```python
age: int = 25
print("Example: age =", age)
print("Type:", type(age))
print("Is this an int?", isinstance(age, int))
```

**What's happening:**

1. **`age: int = 25`** — Variable with type hint (from Lesson 1)
2. **`type(age)`** — Returns the actual type (from Lesson 2)
3. **`isinstance(age, int)`** — Checks if `age` is an int, returns True/False (from Lesson 2)

This demonstrates the three ways to work with types:

- **Type hints** tell you what you intend (specification)
- **`type()`** tells you what Python sees (inspection)
- **`isinstance()`** validates types before operations (validation)

### Section 3: Truthy/Falsy Demonstration

```python
print("\nTruthy/Falsy Examples:")
print("bool(0) =", bool(0))          # False
print("bool(1) =", bool(1))          # True
print("bool('') =", bool(''))        # False (empty string)
print("bool('hello') =", bool('hello'))  # True (non-empty string)
```

This shows **how Python converts different types to boolean**. From Lesson 3, you learned that in boolean contexts (like `if` statements in Chapter 17), Python evaluates truthiness:

- `0` is falsy (means False)
- Any non-zero number is truthy (means True)
- Empty string `""` is falsy
- Non-empty string is truthy

### Section 4: Sequential Organization

Notice the program is organized **sequentially**—it runs from top to bottom:

1. Print welcome message
2. Explore int
3. Explore float
4. Explore str
5. Explore bool (with truthy/falsy examples)
6. Explore None
7. Print summary

This straightforward structure makes the program easy to read and understand. In Chapter 20 (Functions), you'll learn how to reorganize code like this into reusable pieces.

---

## Running Your Type Explorer

### Step 1: Create the File

Create a new file called `type_explorer.py` in your text editor or IDE.

### Step 2: Copy the Complete Program

Copy the entire program from the "Complete Type Explorer Program" section above into your file.

### Step 3: Save and Run

Open a terminal and run:

```bash
python type_explorer.py
```

You should see output like:

```
Welcome to the Python Type Explorer!
Let's explore Python's core data types together.

=== Exploring Integer (int) ===
Example: age = 25
Type: <class 'int'>
Is this an int? True

=== Exploring Float (float) ===
Example: temperature = 98.6
Type: <class 'float'>
Is this a float? True

[... more output for str, bool, None]

=== Summary ===
You've now explored all core Python types:
  - int: Whole numbers (counting)
  - float: Decimal numbers (measuring)
  - str: Text (words and characters)
  - bool: True/False (decisions)
  - None: No value (placeholders)
```

### Step 4: Experiment and Modify

Now that it works, **modify it**:

- Change `age = 25` to `age = 100` and run again. Notice how the output updates.
- Change `name = "Alex"` to `name = "Your Name"` and run again.
- Try adding a new variable in one of the functions and use `isinstance()` to validate it.

This is how you learn programming: **make, run, modify, observe**.

---

## Integration Summary: How This Capstone Brings Everything Together

Let's see how each lesson's concepts show up in your type explorer:

### From Lesson 1: Variables and Type Hints

Every variable in the program uses type hints:

```python
age: int = 25
temperature: float = 98.6
name: str = "Alex"
is_student: bool = True
value: None = None
```

**Integration**: You're not just using type hints—you're using them consistently in a real program. Type hints are now **your standard way of declaring variables**.

### From Lesson 2: Integers and Floats, type() and isinstance()

Your program demonstrates:

```python
print("Type:", type(age))
print("Is this an int?", isinstance(age, int))
```

**Integration**: You're using `type()` and `isinstance()` for their intended purposes: inspection and validation.

### From Lesson 3: Strings and Booleans

Your program demonstrates strings:

```python
name: str = "Alex"
print("Example: name =", name)
```

And boolean truthy/falsy conversion:

```python
print("bool(0) =", bool(0))  # False
print("bool('hello') =", bool('hello'))  # True
```

**Integration**: Types aren't isolated concepts—they interact. Empty strings are falsy. Non-empty strings are truthy. Numbers interact with boolean logic.

### From Lesson 4: Collections and None

Your program explores None:

```python
value: None = None
print("Is this None?", value is None)
```

**Integration**: None is a real type that you'll use constantly as a placeholder value.

### What's Missing (On Purpose!)

You might notice the program doesn't use functions, loops, or error handling. That's **intentional**! Those concepts are taught in later chapters:

- **Functions** (Chapter 20): Organizing code into reusable blocks
- **Loops** (Chapter 17): Repeating code automatically
- **Error handling** (Chapter 21): Safely handling `try/except`

For now, the sequential structure (top-to-bottom) is perfect for learning type concepts.

---

## Common Questions About Your Type Explorer

**Q: Why doesn't the program use functions?**

A: Functions are taught in Chapter 20. For now, the program runs sequentially (top-to-bottom), which is simpler to understand when learning type concepts. Once you learn functions, you can refactor this code as practice!

**Q: What's the difference between `type()` and `isinstance()`?**

A: `type()` answers "What **is** this?" (returns the exact type). `isinstance()` answers "Is this **compatible with** X?" (returns True/False). You'll use both for different purposes.

**Q: Why does the truthy/falsy section exist for booleans?**

A: Because `bool` is special. Unlike `int`, `float`, or `str`, booleans have a conversion function (`bool()`) that converts anything to True/False based on Python's truthiness rules. Understanding this is crucial for control flow in Chapter 17 (`if` statements).

**Q: Can I run this on Windows/Mac/Linux?**

A: Yes! Python 3.14+ works identically on all platforms. The `python type_explorer.py` command works the same everywhere.

---

## Try With AI

Now that you've built a working type explorer, let's collaborate with AI to review it, understand it deeper, and extend it.

**Using Claude Code, Gemini CLI, or ChatGPT (web):**

### Prompt Set

**Prompt 1: Code Review and Type Hints Validation**

Copy this prompt and paste it into your AI tool:

```
I built a Python program called a type explorer that demonstrates
core data types. Here's my program:

[Paste your complete type_explorer.py code here]

Review my code for:
1. Am I using type hints correctly everywhere?
2. What types am I demonstrating (int, float, str, bool, None)?
3. Are there any errors or issues?
4. Is the code well-organized?
```

**Expected outcome:** AI identifies correct type hint usage, confirms all five types are covered, and flags any syntax errors. You'll also get feedback on code clarity.

**Prompt 2: Explore Improvements and Extensions**

```
How could I improve my type explorer program? What additions
would help someone understand types better? Consider:

1. Could I add more examples for each type?
2. Could I demonstrate type conversions (like int("25"))?
3. Could I add error handling?
4. What operations could I show for each type?

What would be your top 3 recommendations for making this program
more educational?
```

**Expected outcome:** AI suggests concrete improvements like adding arithmetic operations for numbers, string methods for strings, or safe conversion examples. You'll get ideas for extending your program.

**Prompt 3: Type Conversion Exploration**

```
I want to understand type conversion better. Show me examples of:

1. Converting a string to an integer: int("25")
2. Converting an integer to a string: str(100)
3. Converting a float to an integer: int(3.7)
4. What happens when conversion fails? (like int("hello"))

For each example, show me:
- The code
- The result
- The type of the result using type()
- When would this be useful in real programs?
```

**Expected outcome:** AI demonstrates type conversion with examples. You'll learn when and why to convert types. You'll also see what errors look like when conversion fails (preparing you for Chapter 21's error handling).

**Prompt 4: Reflection — Why Types Matter (Stretch)**

```
After building and running my type explorer, I'm thinking about
why types matter in Python. Help me understand:

1. What did I learn about types by building this program?
2. How does understanding types help me write better Python code?
3. How will type knowledge help me in Chapter 15 (Operators)?
4. When I work with AI assistants, why is knowing about types important?

Explain as if teaching a friend who just learned types.
```

**Expected outcome:** AI connects type concepts to broader programming principles. You'll see how understanding types is foundational for every Python concept that follows. This builds confidence for Chapter 15 (Operators).

### Safety & Ethics Note

When asking AI to improve your code, always understand what changes it suggests before copying them. Type exploration is foundational—take time to understand each change. If AI suggests something you don't recognize, ask "Explain this line by line" before using it. This builds your judgment about code quality, not just your ability to use AI.

### Next Self-Directed Variation

After working through these prompts:

1. **Build one improvement**: Pick one suggestion from Prompt 2 and add it to your type explorer
2. **Test it**: Run your modified program and see the new behavior
3. **Document it**: Add a comment explaining what your new feature does
4. **Reflect**: Ask your AI: "Does this addition help teach types better? Why or why not?"

This practice turns you from someone following instructions into someone **designing educational code**. That's the capstone skill: creating programs that teach concepts, not just executing pre-written code.

