---
title: "Your First Program"
chapter: 13
lesson: 3
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Understand Program Structure and Execution Flow"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain that programs execute instructions sequentially, from top to bottom, and understand basic program flow"

  - name: "Write and Execute Simple Python Programs with Input/Output"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can write a program that accepts user input and produces output; execute the program; and modify it to change behavior"

  - name: "Ask AI Questions About Code Instead of Memorizing Syntax"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can formulate questions about code and use Claude Code or Gemini to understand what code does before attempting to modify it"

learning_objectives:
  - objective: "Understand what it means to run a Python program and how programs execute sequentially"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of program flow; ability to predict output before running code"

  - objective: "Write, execute, and modify a Python program that uses input and output"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Creation and execution of greeting program; successful modification of program and observation of output changes"

  - objective: "Ask clarifying questions about code using Claude Code or Gemini CLI"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student asks at least one question about the code and uses AI response to understand or modify the program"

cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (Running code, Input/Output pattern) within A2 limit of 7. Progressive scaffolding (simple → complex) prevents cognitive overload ✓"

differentiation:
  extension_for_advanced: "Professional Path Challenge (below)"
  remedial_for_struggling: "Start with just the print() example. Get that working first. Run it multiple times. Change the text. See how output changes. Only move to input() once you're comfortable with print()"

# Generation metadata
generated_by: "lesson-writer v1.0"
source_spec: "specs/part-4-chapter-13/spec.md"
source_plan: "specs/part-4-chapter-13/plan.md"
source_tasks: "specs/part-4-chapter-13/tasks.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Your First Program

## What It Means to Run Code

When you run a Python program, you're telling Python: "Execute these instructions, one line at a time, from top to bottom, in order."

Think of it like a recipe:

1. First, gather ingredients (input)
2. Then, follow steps in order
3. Finally, serve the result (output)

Python does the same thing with code. It reads each line, does what that line says, then moves to the next line.

Here's the key: **Your program controls the conversation with the computer.** You tell Python what to do. Python does it. You tell it the next thing. Python does that. Back and forth. That's how programs work.

## Step 1: The Simplest Program

Let's start with the absolute simplest Python program:

```python
print("Hello, World!")
```

**What it does**: This program prints the text "Hello, World!" to the screen.

**Try it yourself:**

1. Open a text editor (VS Code, Cursor, Notepad, or any plain text editor)
2. Type exactly this:
   ```python
   print("Hello, World!")
   ```
3. Save the file as `hello.py` (the `.py` means it's a Python file)
4. Open a terminal (Command Prompt on Windows, Terminal on macOS/Linux)
5. Navigate to where you saved the file:
   - On Windows: `cd C:\Users\YourName\Desktop` (or wherever you saved it)
   - On macOS/Linux: `cd ~/Desktop` (or wherever you saved it)
6. Run the program:
   ```
   python hello.py
   ```
   (or `python3 hello.py` on macOS/Linux)

**Expected output:**
```
Hello, World!
```

**Did you see "Hello, World!" on the screen? Congratulations! You just ran a Python program.**

This is the foundation of everything. You wrote instructions. Python executed them. You saw the result. That's programming.

## Step 2: Ask the User for Input

Now let's make the program interactive. This program asks the user for their name and stores it using a **type hint** to declare what type of data we expect:

```python
name: str = input("What is your name? ")
```

**What it does**: This program asks the user "What is your name?" and waits for them to type something. The `: str` tells us that `name` will be text (a string). Whatever they type is stored in the variable `name`.

**Try it yourself:**

1. Create a new file called `ask_name.py`
2. Type the code above
3. Save it
4. Run it:
   ```
   python ask_name.py
   ```
5. When you see the question "What is your name?", type your name and press Enter
6. The program ends (because that's all it does)

**What you just did**: Your program paused and waited for you to type something. Then it used what you typed. That's **interaction**. Your program communicated with the user.

## Step 3: Your First Complete Program

Now combine both: ask for the name, then greet them with modern Python syntax:

```python
name: str = input("What is your name? ")
greeting: str = f"Hello, {name}! Welcome to Python."
print(greeting)
```

**What it does**:
- Line 1: Ask the user for their name (with type hint: `str` means text)
- Line 2: Create a greeting using an f-string (the `f` before quotes lets us insert variables easily)
- Line 3: Print the greeting

**Modern Python note:** The `f"Hello, {name}! Welcome to Python."` syntax (called an f-string) is clearer than using `+` to glue strings together. It reads more naturally.

**Try it yourself:**

1. Create a file called `greeting.py`
2. Type the code above exactly
3. Save it
4. Run it:
   ```
   python greeting.py
   ```
5. When asked, type your name and press Enter

**Expected output** (example with name "Alex"):
```
What is your name? Alex
Hello, Alex! Welcome to Python.
```

**What happened:**
- Your program printed a question
- Your program waited for you to respond
- Your program used your response in the greeting
- Your program printed the personalized greeting

This is the fundamental pattern of all programs: **input → processing → output**.

## Step 4: Modify and Experiment

Now the learning happens. Change the program:

**Try modifying the greeting message:**

Change this line:
```python
greeting: str = f"Hello, {name}! Welcome to Python."
```

To this:
```python
greeting: str = f"Hi {name}! Great to see you!"
```

Run the program again. See how the greeting changed?

**Try adding another variable:**

```python
name: str = input("What is your name? ")
age: str = input("How old are you? ")
greeting: str = f"Hello {name}! I'm impressed you're learning Python at age {age}!"
print(greeting)
```

Run it. Now you're asking for two pieces of information and using both in the output.

**The key insight**: Changing code changes behavior. When you understand what each line does, you can modify programs confidently. Type hints (`: str`) make it even clearer what kind of data each variable holds.

## Understanding Type Hints and Modern Python

In modern Python (3.14+), type hints are considered essential for clear code:

```python
name: str = input("What is your name? ")  # name is text
```

The `: str` tells anyone reading this code: "The variable `name` will hold text data." This is a **specification embedded in code**—it documents your intent.

Why does this matter for AI-native development? When you write code with type hints, you're making it easier for:
- **You** to remember what each variable stores
- **Others** reading your code to understand it instantly
- **AI** partners like Claude Code to reason about what your code should do

Type hints are specifications. Clear code is readable code. Readable code is partnership-ready code.

## Optional Challenge: Specification-First Programming

**Here's an optional extension**: Write your own program from a specification.

**Your specification:**
- Ask the user for two pieces of information
- Use both pieces of information in the output
- The program should be personalized

**Example specifications** (pick one or create your own):
1. Ask for their favorite food and favorite color, then respond with a message that uses both
2. Ask for their name and their age, then respond with a message that uses both
3. Ask for the name of their favorite book and favorite author, then respond combining them

**Your task:**
1. Write the specification (plain English) of what your program will do
2. Create a `.py` file with type hints
3. Test it with your own input
4. Ask Claude Code: "Does this program match my specification?"
5. Refine if needed

**Example (if you choose food/color):**

Specification:
```
I want a program that:
1. Asks for the user's favorite food
2. Asks for the user's favorite color
3. Responds with a message that mentions both
```

Code with type hints:
```python
food: str = input("What is your favorite food? ")
color: str = input("What is your favorite color? ")
response: str = f"Interesting! A {color} {food} sounds delicious!"
print(response)
```

**Why this matters**: You're practicing specification-first thinking. You wrote what you wanted before you wrote code. This is exactly how professional AIDD works.

## Try With AI

Use your AI companion (Claude Code or Gemini CLI). You'll deepen your understanding by asking questions about the code.

### Prompt 1: Understand Program Flow
```
Explain what this program does, line by line. What happens first?
What happens second? What happens third?

[Paste your program]
```

**Expected outcome**: AI explains the execution order step-by-step, describing how input flows through the program to create output.

### Prompt 2: Type Hints and Clarity
```
I see `: str` in my code. What does that mean and why would I use it?
How does it help me understand my own code?
```

**Expected outcome**: AI explains type hints as specifications embedded in code, making intent clear for humans and AI alike.

### Prompt 3: Modify with Clarity
```
I want to extend this program to ask for two pieces of information instead of one.
Show me how to do this with type hints. Explain why type hints are important here.

[Paste your current program]
```

**Expected outcome**: AI provides extended code with type hints, explaining how they make the program clearer.

### Prompt 4: Understand Input and Output
```
What's the difference between print() and input()?
When do I use each one? How do they connect to the input → processing → output pattern?
```

**Expected outcome**: AI explains print() displays information to the user; input() collects information from the user.

**Validation Checkpoint**:
- Can you write a program that asks for input and produces personalized output?
- Can you run your program without errors?
- Can you modify your program and see the changes take effect?
- Can you explain what type hints do and why they matter for clarity?

If you answered yes to all four, you've mastered Lesson 3.

**Safety & Ethics Note**: The programs we're writing in this lesson are safe and educational. As you progress, you'll learn about validating user input (making sure data is safe to use). For now, you're learning the fundamentals. Always remember: code that accepts input from users needs special care to be secure. We'll address this in later chapters.

**Next**: You've completed the core lessons! You now understand Python, have it installed, and can write basic programs with clarity and type hints. If you'd like to deepen your understanding of how AI partnership accelerates development, continue to Lesson 4: Thinking Like an AI-First Developer.

<Quiz title="Chapter 4 Quiz" questions={[{"question":"What is the key difference between \u0027external disruption\u0027 and the \u0027internal disruption\u0027 currently happening in software development?","options":{"a":"External disruption is slow, while internal disruption is even slower.","b":"External disruption is when a software company disrupts another industry, while internal disruption is when AI tools transform the software industry itself.","c":"External disruption affects senior developers, while internal disruption affects junior developers.","d":"External disruption is voluntary, while internal disruption is forced upon developers."},"correct_answer":"b","explanation":"The chapter defines the pattern: \u0027External force: Software companies built platforms that competed with traditional retailers... Internal force: The same industry creating the tools is being transformed by them.\u0027"},{"question":"According to the chapter, why is the adoption of AI coding tools happening so much faster than previous technology shifts?","options":{"a":"Because developers are being forced to use them by their managers.","b":"Because there is no external resistance, and developers are adopting them voluntarily for immediate value.","c":"Because the tools are expensive, creating a sense of urgency.","d":"Because previous technology shifts were not very useful."},"correct_answer":"b","explanation":"The text highlights several reasons for the speed, a key one being: \u0027No External Resistance. When AI tools disrupt software development, there\u0027s no external resistance. Developers are adopting these tools voluntarily and enthusiastically...\u0027"},{"question":"What does the chapter mean by the \u0027Recursion Effect\u0027?","options":{"a":"AI coding tools are getting stuck in infinite loops.","b":"Developers are recursively calling the same functions, leading to bugs.","c":"AI coding tools are being used to improve and develop the next version of themselves, creating a rapid improvement cycle.","d":"The process of learning to code is becoming recursive and more difficult."},"correct_answer":"c","explanation":"The chapter explains this \u0027mind-bending\u0027 concept: \u0027AI coding tools are being used to improve AI coding tools... This creates a recursive improvement cycle that has no parallel in previous disruptions.\u0027"},{"question":"How does the impact of AI coding tools on developer roles differ from previous technology shifts like cloud computing or mobile development?","options":{"a":"AI coding tools only affect junior developers.","b":"AI coding tools have a universal impact, affecting every role in the software development value chain simultaneously.","c":"AI coding tools have less impact than previous shifts.","d":"Previous shifts affected all roles, while AI tools only affect a few."},"correct_answer":"b","explanation":"The text states, \u0027AI coding tools affect everyone simultaneously,\u0027 and then lists the impact on junior, mid-level, senior, DevOps, QA, and technical writers, concluding, \u0027There\u0027s nowhere in the software development value chain that remains untouched.\u0027"},{"question":"What is the chapter\u0027s conclusion about the inevitability of AI coding tools?","options":{"a":"The \u0027if\u0027 question is still debated, and their adoption is uncertain.","b":"The \u0027if\u0027 question is already answered; the only remaining question is \u0027how fast?\u0027","c":"The tools are likely a temporary hype or trend.","d":"The adoption will be slow and may take over a decade."},"correct_answer":"b","explanation":"The chapter asserts, \u0027With AI coding, the \u0027if\u0027 question is already answered. The tools exist, they work, they\u0027re being adopted at scale, and they\u0027re improving rapidly. The only remaining question is \u0027how fast?\u0027"}]} />

