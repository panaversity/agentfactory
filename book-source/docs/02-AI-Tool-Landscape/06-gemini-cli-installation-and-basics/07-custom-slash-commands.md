---
sidebar_position: 7
title: Personalizing Your AI Learning Experience
cefr_level: A2
proficiency: Beginner
teaching_stage: 2
stage_name: "AI Collaboration"
cognitive_load:
  concepts_count: 6
  a2_compliant: true
learning_objectives:
  - id: LO1
    description: "Explain why reusing effective prompts saves time and improves learning consistency"
    bloom_level: "Understand"
    digcomp: "1.3 Managing data, information and digital content"
  - id: LO2
    description: "Create a simple custom slash command using TOML format with description and prompt fields"
    bloom_level: "Apply"
    digcomp: "3.2 Integrating and re-elaborating digital content"
  - id: LO3
    description: "Use argument injection (double curly braces with args) to make commands flexible for different topics"
    bloom_level: "Apply"
    digcomp: "3.2 Integrating and re-elaborating digital content"
  - id: LO4
    description: "Organize multiple commands in folders using namespace notation (category:command)"
    bloom_level: "Apply"
    digcomp: "1.2 Evaluating data, information and digital content"
  - id: LO5
    description: "Test and debug custom commands by checking file location and TOML syntax"
    bloom_level: "Analyze"
    digcomp: "5.2 Identifying needs and technological responses"
  - id: LO6
    description: "Design personal learning commands that match individual study preferences and goals"
    bloom_level: "Create"
    digcomp: "3.2 Integrating and re-elaborating digital content"
---

# Personalizing Your AI Learning Experience

You're learning something new—let's say you're researching "What is Python?" Every time you ask Gemini, you type: "Explain what Python is in simple terms, what it's used for, and why beginners learn it." That's 90+ characters every time. Then tomorrow you want to learn about JavaScript, and you type almost the same thing again. There has to be a better way. In this lesson, you'll learn how custom slash commands turn that 90-character prompt into 7 keystrokes: `/learn Python`

---

## The Problem: Typing the Same Learning Prompts Over and Over

Let's see this in action. You're exploring different topics for your learning journey.

### The Traditional Way: Type the Full Prompt Every Time

**Day 1 - Learning about Python:**
```
Explain what Python is in simple terms. What is it used for? Why do beginners learn it? Give me 3-4 examples of things people build with Python.
```

**Day 2 - Learning about Git:**
```
Explain what Git is in simple terms. What is it used for? Why do beginners learn it? Give me 3-4 examples of how people use Git.
```

**Day 3 - Learning about APIs:**
```
Explain what an API is in simple terms. What is it used for? Why do beginners learn about APIs? Give me 3-4 examples of common APIs.
```

Notice the pattern? **You're typing almost the same structure every time, just changing the topic.**

**Each prompt is 100+ characters.**

You have to:
1. Type (or remember) the structure every time
2. Make sure you ask for what you actually want (simple terms, examples, why it matters)
3. Hope you don't forget to ask for examples or use cases
4. Then add the new topic

**If you research 3 topics a day, you're typing variations of this 21 times a week.**

### What Happens in Practice

**Reality check**:
- Day 1: You type the full detailed prompt
- Day 2: You shorten it to "explain Python"
- Day 3: Gemini's explanation is too technical, you forgot to ask for "simple terms"
- Day 4: You get an explanation but no examples, you forgot to ask for those
- Day 5: You're typing different versions of the same request and getting inconsistent help

**The problems**:
- ❌ **Repetition**: Same prompt structure, different topics
- ❌ **Inconsistency**: Forgetting parts of what makes a good learning prompt
- ❌ **Frustration**: "Why didn't it explain it simply this time?"
- ❌ **Lost knowledge**: That one perfect prompt that really worked? Buried in chat history
- ❌ **Varying quality**: Some explanations are perfect, others miss what you need

---

## The Solution: Custom Slash Commands for Learning

What if you could type `/learn Python` and have Gemini automatically use your perfect learning prompt structure?

**That's what custom slash commands do.**

### Creating Your First Learning Command

Create a file in your home directory:

**Location**: `~/.gemini/commands/learn.toml`

**Content**:
```toml
description = "Learn about a new topic in beginner-friendly terms"
prompt = """
Explain what {{args}} is in simple, beginner-friendly terms.

Please include:
1. What it is (simple definition)
2. What it's used for (real-world purpose)
3. Why beginners learn about it
4. 3-4 concrete examples

Keep the explanation conversational and avoid technical jargon.
"""
```

**What this does**:
1. The `description` appears when you type `/help`
2. The `prompt` is what gets sent to Gemini
3. `{{args}}` gets replaced with whatever you type after `/learn`

### Now the Same Task Becomes Simple

Instead of typing 100+ characters, you type **13 characters**:

```
/learn Python
```

**What happens behind the scenes**:
1. Gemini CLI sees `/learn`
2. Loads `~/.gemini/commands/learn.toml`
3. Replaces `{{args}}` with `Python`
4. Sends the full structured prompt to Gemini
5. Gemini explains Python using your preferred format

**You saved**:
- ⏱️ **Time**: 20 seconds → 2 seconds
- 🧠 **Mental load**: No remembering what makes a good learning prompt
- ✅ **Consistency**: Same high-quality explanation structure every time
- 📚 **Better learning**: Never forget to ask for examples or simple terms

---

## TOML File Structure: How Commands Work

Custom commands are written in **TOML** format (a simple, readable way to save settings—like writing a recipe).

### Required Parts

Every custom command needs two things:

```toml
description = "Brief description of what this command does"
prompt = """
The actual prompt sent to Gemini.
Can be multiple lines.
"""
```

**The `description`**: Shows up when you type `/help` to list all your commands
**The `prompt`**: The actual text sent to Gemini when you run the command

### Making Commands Flexible with Arguments

The magic part is `{'{{args}}'}` (called a "placeholder"):

**Without arguments** - Fixed command:
```toml
description = "Get motivation for learning"
prompt = "Give me an encouraging message about learning to code"
```

Usage: `/motivate` (always says the same thing)

**With arguments** - Flexible command:
```toml
description = "Get motivation for any learning topic"
prompt = "Give me an encouraging message about learning {{args}}"
```

Usage:
- `/motivate Python` → motivation for learning Python
- `/motivate Git` → motivation for learning Git
- `/motivate programming` → motivation for learning programming

The `{'{{args}}'}` placeholder gets **replaced** with whatever you type after the command.

### Real Example: Study Helper

**File**: `~/.gemini/commands/study.toml`

```toml
description = "Create a study plan for any topic"
prompt = """
I want to learn about {{args}}.

Please create a simple 7-day study plan with:
- One clear goal for each day
- Estimated time needed (30-60 minutes per day)
- What to focus on each day
- One practice task per day

Make it beginner-friendly and achievable.
"""
```

**Usage**:
```
/study Python basics
```

**Result**: 7-day beginner study plan specifically for Python basics.

---

## Building Your Personal Learning Command Library

Here are practical commands you can create right now for your learning journey:

### Command 1: Simple Explanation

**File**: `~/.gemini/commands/explain.toml`
```toml
description = "Get a simple explanation of any concept"
prompt = "Explain {{args}} in the simplest way possible, as if I'm hearing about it for the first time. Use an analogy if helpful."
```

**Usage**: `/explain what an API is`

### Command 2: Summary Generator

**File**: `~/.gemini/commands/summarize.toml`
```toml
description = "Summarize key points about a topic"
prompt = """
Give me a quick summary of {{args}}.

Include:
- 3-5 key points
- Why it matters
- One sentence bottom line

Keep it under 100 words.
"""
```

**Usage**: `/summarize how Git works`

### Command 3: Practice Question Generator

**File**: `~/.gemini/commands/quiz.toml`
```toml
description = "Generate practice questions on a topic"
prompt = """
Create 5 beginner-friendly practice questions about {{args}}.

For each question:
- Make it test understanding, not memorization
- Include the answer
- Explain why that's the answer

Focus on practical understanding.
"""
```

**Usage**: `/quiz terminal commands`

### Command 4: Learning Progress Tracker

**File**: `~/.gemini/commands/progress.toml`
```toml
description = "Reflect on what you've learned"
prompt = """
Help me reflect on my learning progress with {{args}}.

Ask me these questions and help me think through each one:
1. What did I understand well?
2. What's still confusing?
3. What should I practice more?
4. What's my next learning step?

Guide me through reflection.
"""
```

**Usage**: `/progress Python variables`

### Command 5: Resource Finder

**File**: `~/.gemini/commands/resources.toml`
```toml
description = "Find beginner-friendly learning resources"
prompt = """
I'm a beginner learning about {{args}}.

Suggest:
1. 2-3 beginner-friendly online resources (free if possible)
2. What makes each resource good for beginners
3. Which one to start with and why
4. What to expect from each

Focus on high-quality, beginner-appropriate resources.
"""
```

**Usage**: `/resources learning Python`

---

## Organizing Your Commands with Folders

As you create more commands, organize them by category.

### Directory Structure Creates Namespaces

```
~/.gemini/commands/
├── learn.toml              → /learn
├── explain.toml            → /explain
├── study/
│   ├── plan.toml           → /study:plan
│   ├── review.toml         → /study:review
│   └── quiz.toml           → /study:quiz
└── research/
    ├── topic.toml          → /research:topic
    ├── compare.toml        → /research:compare
    └── resources.toml      → /research:resources
```

**Rule**: Folder name becomes the first part of the command. Colon (`:`) separates folder from command.

### Example: Study Commands Folder

Create `~/.gemini/commands/study/` folder with three files:

**File 1**: `study/plan.toml`
```toml
description = "Create a study plan"
prompt = "Create a 7-day study plan for learning {{args}}, suitable for beginners studying 30-60 minutes per day."
```
Usage: `/study:plan Python basics`

**File 2**: `study/review.toml`
```toml
description = "Review what you learned"
prompt = "Help me review what I learned about {{args}} by asking me 5 questions to test my understanding."
```
Usage: `/study:review variables and data types`

**File 3**: `study/break.toml`
```toml
description = "Explain why taking a study break matters"
prompt = "I've been studying {{args}} for a while. Explain why taking a break is important for learning, and suggest a good break activity."
```
Usage: `/study:break Python`

Now you have three organized study-related commands!

---

## Trying Your Commands

### Step 1: Create Your First Command

1. Open your terminal
2. Create the commands folder:
   ```bash
   mkdir -p ~/.gemini/commands
   ```

3. Create a file called `learn.toml`:
   ```bash
   # On Windows (Git Bash or WSL)
   notepad ~/.gemini/commands/learn.toml

   # On Mac
   open -e ~/.gemini/commands/learn.toml

   # On Linux
   nano ~/.gemini/commands/learn.toml
   ```

4. Add this content:
   ```toml
   description = "Learn about a topic in simple terms"
   prompt = "Explain {{args}} in simple, beginner-friendly terms with 3 examples."
   ```

5. Save the file

### Step 2: Test It

1. Open Gemini CLI:
   ```bash
   gemini
   ```

2. List your commands:
   ```
   /help
   ```
   You should see `/learn` in the list!

3. Try it:
   ```
   /learn what Python is
   ```

**Expected result**: Gemini explains Python in simple terms with 3 examples, just like your prompt specified.

---

## Common Issues and How to Fix Them

### "Command not found: /learn"

**Problem**: Gemini can't find your command file.

**Check these**:
1. Is the file in the right location?
   ```bash
   ls ~/.gemini/commands/
   ```
   You should see `learn.toml`

2. Is the filename spelled correctly? (must be `learn.toml`, not `Learn.toml` on Mac/Linux)

3. Try restarting Gemini CLI:
   ```bash
   # Exit Gemini
   /exit

   # Start again
   gemini
   ```

### "I typed /learn Python but it didn't work"

**Problem**: TOML syntax might be wrong.

**Check these**:
1. Did you put quotes around strings?
   - ✅ Correct: `description = "Learn something"`
   - ❌ Wrong: `description = Learn something`

2. Did you use three quotes for multi-line prompts?
   - ✅ Correct: `prompt = """Multi-line text here"""`
   - ❌ Wrong: `prompt = "Multi-line text here"`

### "The command runs but arguments show up literally"

**Problem**: Wrong bracket type.

**Check**:
- ✅ Correct: `{'{{args}}'}` (double curly braces)
- ❌ Wrong: `{args}` (single braces)
- ❌ Wrong: `[[args]]` (square brackets)

---

## Try With AI

:::tip Instruction for AI Tools
When practicing these prompts, use **Gemini CLI** as your preferred tool. Gemini's custom slash commands are specifically designed to save your effective learning prompts for reuse.
:::

### Prompt 1: Create Your Personal Learning Command
```
Help me create a custom Gemini CLI command called /understand
that takes a topic as argument and:
1. Explains it in the simplest possible way
2. Uses an analogy I can relate to
3. Gives 2 real-world examples
4. Tells me one practical thing I can do to learn more

Show me:
1. The exact TOML file content
2. Where to save it (~/.gemini/commands/)
3. How to test it works
```

**Expected outcome**: Ready-to-use TOML file you can create and test.

### Prompt 2: Build a Study Session Helper
```
I want to create a command that helps me plan study sessions.

When I type /session [topic], it should:
1. Suggest how long to study for (30-60 min for beginners)
2. Break the topic into 3-4 smaller chunks
3. Recommend a break schedule
4. Give me one focus goal for the session

Write the TOML file for this command.
```

**Expected outcome**: Custom command for planning focused study sessions.

### Prompt 3: Design Your Personal Command Library
```
I'm learning [describe your learning path: programming / web development / data analysis / etc.].

Design 4-5 custom commands I could create to help with:
1. Understanding new concepts quickly
2. Practicing what I learned
3. Tracking my progress
4. Staying motivated

Show me the folder structure and 2-3 example TOML files.
```

**Expected outcome**: Personalized command library matching your learning needs.

### Prompt 4: Debug a Command That's Not Working
```
I created this custom command but it's not working:

[paste your TOML file]

When I run [your command], I get this result: [describe what happens]
But I expected: [describe what you wanted]

What's wrong? How do I fix it?
```

**Expected outcome**: Specific help fixing your actual command.

