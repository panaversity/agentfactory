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

## Practice Exercise

Create a specification for a simple calculator app that shows both fenced code blocks and inline code.

### Your Task

Write a specification that includes:

1. **A fenced code block** showing expected output when the user adds two numbers
2. **A fenced code block** with the `python` tag showing example code
3. **Inline code** in the installation instructions

### Template

**Create a file and fill it in:**

```markdown
# Calculator App

## Expected Output

When the user runs `python calculator.py`, they should see:

```

[Your expected output here - show what gets printed]

```

## Example Code

```python
[Your example Python code here]
```

## Installation

1. Install Python 3.9+
2. Run `[your command here]`
3. Execute the program with `[your command here]`
```

### Validation Checklist

Check your specification:

- [ ] Expected output is in a fenced code block
- [ ] Example code is in a fenced code block with `python` tag
- [ ] Installation commands use inline code (single backticks)
- [ ] File names use inline code
- [ ] All code blocks have opening AND closing triple backticks

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

Practice using code blocks with AI feedback.

### Setup

Use ChatGPT web (or your AI companion from earlier chapters).

### Exercise

Write a specification for a greeting app that includes:
- Expected output (fenced block)
- Example code (fenced block with `python` tag)
- Installation command (inline code)

Then ask ChatGPT:

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

**Prompt 3 (Implementation Test):**

\`\`\`
Can you implement this specification? Generate the Python code
that matches my expected output.
\`\`\`

### Expected Outcomes

From **Prompt 1**: ChatGPT confirms your code block syntax is correct

From **Prompt 2**: ChatGPT can parse your specification and understand output format, language, and commands

From **Prompt 3**: ChatGPT generates code that matches your expected output (if your spec was clear)

### Your Task

After ChatGPT generates code, check if it matches your expected output exactly. If not, what was unclear in your specification?

### Reflection

Write a brief reflection (3-5 sentences):
- When would you use fenced blocks vs inline code?
- Did showing expected output help the AI generate correct code?
- How did language tags improve clarity?

### Safety & Verification Note

When asking AI to implement your specification:
- **Check if the generated code matches your output** - Does it print exactly what you specified?
- **Verify commands are safe** - Don't run installation commands without understanding them
- **Test the logic** - Does the code do what you intended, not just what you wrote?

The goal is learning to write clear specifications that produce correct implementations.
