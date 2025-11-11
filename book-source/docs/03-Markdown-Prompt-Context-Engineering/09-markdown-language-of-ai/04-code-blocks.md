---
title: "Code Blocks - Showing Examples"
description: "Learning to include code examples and expected output in markdown specifications"
sidebar_label: "Code Blocks - Showing Examples"
sidebar_position: 4
chapter: 9
lesson: 4
duration_minutes: 45
proficiency: "A2"
concepts: 3

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Creating Fenced Code Blocks"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create multi-line code blocks with proper syntax"

  - name: "Using Language Tags"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can add appropriate language tags to code blocks"

  - name: "Using Inline Code"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can format inline code references within text"

learning_objectives:
  - objective: "Create fenced code blocks for showing code and output"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Practice exercise showing expected program output"

  - objective: "Add language tags to code blocks for clarity"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Exercise using python, bash, and text tags"

  - objective: "Use inline code for commands and variable names"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Exercise includes inline code in explanations"

cognitive_load:
  new_concepts: 3
  assessment: "3 concepts (fenced blocks, language tags, inline code) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore syntax highlighting languages; learn to show before/after examples; practice using code blocks to document APIs"
  remedial_for_struggling: "Start with simple text code blocks first; practice identifying the difference between inline and multi-line code; learn one language tag at a time"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-chapter-9-markdown/spec.md"
created: "2025-11-06"
last_modified: "2025-11-07"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.1"
---

# Code Blocks - Showing Examples

When you're writing a specification that says:

> "The program should greet the user and show the current time."

An AI agent could generate code that prints:
- "Hello"
- "Hello World"
- "Hello! Time: 2:30pm"
- "Greetings, human. Current time: 14:30:00"

Which one do you actually want?

Now imagine you show the exact output:

\`\`\`text
Hello! The time is 14:30:00
\`\`\`

Suddenly there's no confusion. The AI knows exactly what format to use.

Code blocks let you show expected output, code examples, and command syntax directly in your specification. This lesson teaches you how to use them effectively.

---

## Concept 1: Fenced Code Blocks (Multiple Lines)

Use **fenced code blocks** when you need to show multiple lines of code or output.

### Basic Syntax

Create a fenced code block with triple backticks (`` ` ``):

```
Line 1 of code or output
Line 2 of code or output
Line 3 of code or output
```

Type three backticks, press Enter, type your code, press Enter, then type three more backticks. Everything between displays exactly as you type it (no markdown formatting applied).

### Example: Showing Expected Output

Here's a specification for a task list app:

**In your README.md:**
```text
When the user views tasks, they should see:
```

Then add a code block showing the expected output:

```
Tasks:
1. Buy groceries [Pending]
2. Call dentist [Complete]
3. Submit report [Pending]
```

The AI agent sees this and knows: "The output must show task number, description, and status on each line."

### Example: Showing Program Code

**In your spec:**
```python
def add(a, b):
    return a + b

result = add(5, 3)
print(result)  # Should print: 8
```

This shows the AI: "This is what the code should look like."

---

## Concept 2: Language Tags (For Clarity)

Add a **language tag** right after the opening backticks to specify what type of code it is.

### Syntax

Type three backticks, then the language name, then your code:

```python
print("Hello, World!")
```

The word `python` is the language tag. It goes right after the opening triple backticks (no space).

### Common Language Tags

Use these tags based on what you're showing:

- **`python`** - Python code
- **`bash`** - Terminal commands
- **`text`** - Plain output (no code)
- **`typescript`** - TypeScript code
- **`json`** - Data formats
- **`yaml`** - Configuration files

### Why Language Tags Matter

Language tags help:
1. **Readers** understand what they're looking at
2. **AI agents** know what language to generate
3. **Code viewers** apply correct syntax highlighting

### Example: Installation Commands

**In your README:**

```bash
pip install requests
python app.py
```

The `bash` tag tells the AI: "These are terminal commands, not Python code."

### Example: Python Code

**Show example code like this:**

```python
def greet(name):
    return f"Hello, {name}!"

print(greet("Alice"))
```

The `python` tag makes it clear this is Python code to implement.

---

## Concept 3: Inline Code (Single Backticks)

Use **inline code** for short code references within regular text - like variable names, commands, or file names.

### Syntax

Use single backticks (`` ` ``) around the code:

Install the package with `pip install requests` command.
The `app.py` file contains the main function.
Set the `DEBUG` variable to `True` for testing.

### When to Use Inline Code

Use single backticks for:
- Command names: `python`, `git`, `npm`
- Variable names: `user_name`, `total_count`
- File names: `README.md`, `app.py`
- Function names: `calculate_total()`, `get_user()`
- Short code snippets within sentences

### Example in a Specification

**In your spec, write:**

1. Install Python 3.9 or higher
2. Run `pip install requests` to install dependencies
3. Create a file named `config.py` with your API key
4. Run the program with `python weather.py`

**See how it works?**
- Commands like `pip install requests` are in backticks
- File names like `config.py` are in backticks
- Function names like `get_weather()` are in backticks

This makes the specification much clearer.

---

## Fenced vs Inline: Which to Use?

### Use Fenced Code Blocks (triple backticks) when:
- Showing multiple lines of code
- Displaying expected program output
- Sharing code examples to implement
- Showing error messages

### Use Inline Code (single backticks) when:
- Mentioning commands in a sentence
- Referring to variable or function names
- Listing file names
- Writing short code snippets in text

### Side-by-Side Examples

**Fenced block (multiple lines):**

```python
def calculate_total(prices):
    return sum(prices)

items = [10, 20, 30]
print(calculate_total(items))
```

**Inline code (within text):**

The `calculate_total()` function takes a list of `prices` and returns the sum.
Call it with `calculate_total([10, 20, 30])` to get `60`.

---

## Practice Exercise: Task Tracker App (Part 3 - Code Blocks)

**Continuing from Lesson 3**: Open your Task Tracker App specification. You'll now **add code blocks** to show expected output and clarify commands.

### Your Task for Lesson 4

Add code blocks to make your specification more concrete:

**Part 1: Add Expected Output Section**

Fill in the "Expected Output" section with a fenced code block showing what the program displays:

```text
## Expected Output

When the user runs `python tracker.py`, they should see:

\`\`\`text
Task Tracker Menu
1. Add Task
2. View Tasks
3. Mark Complete
4. Delete Task
5. Exit

Choose an option: _
\`\`\`

When viewing tasks, the display looks like:

\`\`\`text
Your Tasks:
1. Buy groceries [Pending] - Due: 2025-11-08
2. Call dentist [Pending] - Due: 2025-11-07
3. Submit report [Complete] - Done: 2025-11-06
\`\`\`
```

**Part 2: Update Installation Commands**

Make sure your installation steps use **inline code** for commands:

```text
## Installation

1. Install Python 3.9 or higher from python.org
2. Download the task tracker files from GitHub
3. Navigate to the project folder: `cd task-tracker`
4. Run the program: `python tracker.py`
```

**Part 3: Add Language Tags**

Ensure your code blocks have the `text` language tag (since this is program output, not Python code).

### Validation Checklist

Check your updated specification:

- [ ] Expected output section has at least one fenced code block with `text` tag
- [ ] Code blocks show what the program actually prints (not descriptions)
- [ ] Installation commands use inline code (`` `cd task-tracker` ``, `` `python tracker.py` ``)
- [ ] All code blocks have opening AND closing triple backticks
- [ ] Output is specific (shows actual menu items, not "menu appears")

**Save this file!** You'll add links, images, and emphasis in Lesson 5 to complete it.

---

## Common Mistakes to Avoid

### Mistake 1: Forgetting Closing Backticks

**Wrong:**
```python
print("Hello")
```
(missing closing backticks)

**Correct:**
```python
print("Hello")
```

Always close your code blocks.

### Mistake 2: Using Inline Code for Multiple Lines

**Wrong:**
The code is `def add(a, b): return a + b` (inline code with multiple lines doesn't work)

**Correct:**
The code is:

```python
def add(a, b):
    return a + b
```

Use fenced blocks for multiple lines.

### Mistake 3: No Language Tag When It Matters

**Unclear:**
```
pip install requests
```

**Clear:**
```bash
pip install requests
```

The `bash` tag makes it clear this is a terminal command.

---

## Why This Matters for AI

When you use code blocks correctly, AI agents can:

1. **See exact output format** - "The program prints 3 lines with this structure"
2. **Understand the language** - "This is Python code, not bash commands"
3. **Parse code examples** - "This is example code to implement, not expected output"
4. **Follow command syntax** - "These inline codes are commands to run"

Good code block usage = clearer specifications = better AI-generated code.

---

## Try With AI

Validate your Task Tracker App code blocks with AI feedback — and test execution.

### Setup

Use ChatGPT web (or your AI companion from earlier chapters).

### Exercise

Take your **updated Task Tracker App specification** (now with code blocks) and ask ChatGPT:

**Prompt 1 (Structure Check):**

\`\`\`
I'm learning code blocks in markdown. Can you check if I used the right syntax?

[Paste your specification here]

Tell me:
1. Are my code blocks properly closed?
2. Did I use appropriate language tags?
3. Should any inline code be fenced blocks instead?
\`\`\`

**Prompt 2 (Clarity Check):**

\`\`\`
Based on my specification, can you tell:
1. What should the program output look like?
2. What programming language is this written in?
3. What commands need to be run?
\`\`\`

**Prompt 3 (Implementation + Execution Test):**

\`\`\`
Can you implement a simple version of this Task Tracker App specification?
Generate Python code that shows the main menu and displays sample tasks
like I specified in the Expected Output section.
\`\`\`

### Expected Outcomes

From **Prompt 1**: ChatGPT confirms your code block syntax is correct

From **Prompt 2**: ChatGPT can parse your specification and understand output format, language, and commands

From **Prompt 3**: ChatGPT generates Python code matching your expected output

<Quiz title="Chapter 4 Quiz" questions={[{"question":"What is the key difference between \u0027external disruption\u0027 and the \u0027internal disruption\u0027 currently happening in software development?","options":{"a":"External disruption is slow, while internal disruption is even slower.","b":"External disruption is when a software company disrupts another industry, while internal disruption is when AI tools transform the software industry itself.","c":"External disruption affects senior developers, while internal disruption affects junior developers.","d":"External disruption is voluntary, while internal disruption is forced upon developers."},"correct_answer":"b","explanation":"The chapter defines the pattern: \u0027External force: Software companies built platforms that competed with traditional retailers... Internal force: The same industry creating the tools is being transformed by them.\u0027"},{"question":"According to the chapter, why is the adoption of AI coding tools happening so much faster than previous technology shifts?","options":{"a":"Because developers are being forced to use them by their managers.","b":"Because there is no external resistance, and developers are adopting them voluntarily for immediate value.","c":"Because the tools are expensive, creating a sense of urgency.","d":"Because previous technology shifts were not very useful."},"correct_answer":"b","explanation":"The text highlights several reasons for the speed, a key one being: \u0027No External Resistance. When AI tools disrupt software development, there\u0027s no external resistance. Developers are adopting these tools voluntarily and enthusiastically...\u0027"},{"question":"What does the chapter mean by the \u0027Recursion Effect\u0027?","options":{"a":"AI coding tools are getting stuck in infinite loops.","b":"Developers are recursively calling the same functions, leading to bugs.","c":"AI coding tools are being used to improve and develop the next version of themselves, creating a rapid improvement cycle.","d":"The process of learning to code is becoming recursive and more difficult."},"correct_answer":"c","explanation":"The chapter explains this \u0027mind-bending\u0027 concept: \u0027AI coding tools are being used to improve AI coding tools... This creates a recursive improvement cycle that has no parallel in previous disruptions.\u0027"},{"question":"How does the impact of AI coding tools on developer roles differ from previous technology shifts like cloud computing or mobile development?","options":{"a":"AI coding tools only affect junior developers.","b":"AI coding tools have a universal impact, affecting every role in the software development value chain simultaneously.","c":"AI coding tools have less impact than previous shifts.","d":"Previous shifts affected all roles, while AI tools only affect a few."},"correct_answer":"b","explanation":"The text states, \u0027AI coding tools affect everyone simultaneously,\u0027 and then lists the impact on junior, mid-level, senior, DevOps, QA, and technical writers, concluding, \u0027There\u0027s nowhere in the software development value chain that remains untouched.\u0027"},{"question":"What is the chapter\u0027s conclusion about the inevitability of AI coding tools?","options":{"a":"The \u0027if\u0027 question is still debated, and their adoption is uncertain.","b":"The \u0027if\u0027 question is already answered; the only remaining question is \u0027how fast?\u0027","c":"The tools are likely a temporary hype or trend.","d":"The adoption will be slow and may take over a decade."},"correct_answer":"b","explanation":"The chapter asserts, \u0027With AI coding, the \u0027if\u0027 question is already answered. The tools exist, they work, they\u0027re being adopted at scale, and they\u0027re improving rapidly. The only remaining question is \u0027how fast?\u0027"}]} />

