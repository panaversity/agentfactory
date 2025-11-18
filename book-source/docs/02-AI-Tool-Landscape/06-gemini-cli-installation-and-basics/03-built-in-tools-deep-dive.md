---
sidebar_position: 3
title: "What Gemini Can Do: Built-In Tools in Action"
cefr_level: A2
proficiency: Beginner
teaching_stage: 2
stage_name: "AI Collaboration"
stage_description: "Students actively use AI tools and understand their capabilities"
cognitive_load:
  concepts_count: 7
  a2_compliant: true
  scaffolding_level: "Heavy"
learning_objectives:
  - id: LO1
    description: "Identify the four built-in tools Gemini uses (Google Search, File Operations, Shell, Web Fetch)"
    bloom_level: "Remember"
  - id: LO2
    description: "Recognize visual indicators showing which tool Gemini is using"
    bloom_level: "Understand"
  - id: LO3
    description: "Explain when Gemini chooses each tool based on query context"
    bloom_level: "Understand"
  - id: LO4
    description: "Apply tools by crafting queries that trigger appropriate tool usage"
    bloom_level: "Apply"
  - id: LO5
    description: "Analyze tool output to verify Gemini used correct tool for the task"
    bloom_level: "Analyze"
  - id: LO6
    description: "Practice using tools through realistic developer scenarios"
    bloom_level: "Apply"
  - id: LO7
    description: "Evaluate when to use Gemini's tools vs manual methods for development tasks"
    bloom_level: "Evaluate"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "1. Information and Data Literacy"
    competency: "1.1 Browsing, searching and filtering data, information and digital content"
  - objective_id: LO2
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Evaluating data, information and digital content"
  - objective_id: LO3
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Evaluating data, information and digital content"
  - objective_id: LO6
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO7
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
---

# What Gemini Can Do: Built-In Tools in Action

You're starting your programming journey and want to learn about Python before diving into coding. You could:
- Open multiple browser tabs
- Search through scattered articles
- Find reliable sources
- Read and compare information
- Synthesize what you've learned

Or you could just ask Gemini:

```
What is Python and what is it used for in 2025?
```

Within seconds, Gemini searches the web, finds reliable sources, and gives you a current answer with citations. **This is the power of built-in tools** - your AI doesn't just answer from memory, it actively reaches out to get current information, read your files, and execute commands.

In this lesson, you'll learn what Gemini can actually do for you as you learn to code.

---

## Tools Activate Automatically

Here's the key insight that changes everything: **You don't invoke tools manually.** You ask naturally, and Gemini decides which tool to use.

When you ask Gemini a question, it analyzes what you need:
- Current information from the web? → Google Search tool activates
- Read a file in your project? → File operations tool activates
- Run a terminal command? → Shell integration activates
- Fetch a specific webpage? → Web fetch tool activates

You'll know a tool is active when you see visual indicators:
- 🔍 **Searching the web...**
- 📁 **Reading file...**
- ⚡ **Running command...**
- 🌐 **Fetching webpage...**

Let's see each tool in action.

---

## Tool 1: Google Search — Getting Current Information

**When Gemini uses this**: You need current information from across the web, and you don't know which specific site has it.

### Example: Learning About Python

You ask Gemini:

```
What is Python and what is it used for in 2025?
```

**What you'll see:**

```
> What is Python and what is it used for in 2025?

🔍 Searching the web...

Python is one of the most popular programming languages today, used for:

**Web Development:**
- Building websites with frameworks like Django and Flask
- Example: Instagram, Spotify, and Pinterest use Python

**Data Science and AI:**
- Analyzing large datasets
- Building machine learning models
- Libraries like NumPy, Pandas, TensorFlow are industry-standard

**Automation:**
- Writing scripts to automate repetitive tasks
- Helpful for managing files, systems, and workflows

**Scientific Computing:**
- Universities use Python for research
- Physics, biology, chemistry simulations

**Why Python is Popular for Beginners:**
- Readable syntax (looks like English)
- Huge community with tons of tutorials
- Runs on Windows, Mac, and Linux
- In high demand for jobs

**Next Steps in Your Learning:**
You'll learn Python starting in Chapter 12 of this book after mastering Bash and Git basics.

Sources:
- python.org (official website)
- Stack Overflow Developer Survey 2024
- GitHub Octoverse Report 2024
```

**What happened**: Gemini recognized you needed current information about Python adoption and use cases, activated Google Search, found reliable sources, and synthesized the answer in beginner-friendly language.

### Example: Understanding Tools Before Learning Them

You ask Gemini:

```
What is Git and why do developers use it?
```

**What you'll see:**

```
> What is Git and why do developers use it?

🔍 Searching the web...

Git is a version control system - a tool that tracks changes to your code.

**What It Does:**
- Saves a history of every change you make to your files
- Lets you go back to previous versions if something breaks
- Allows multiple people to work on the same project without conflicts
- Creates backups of your work

**Real-World Example:**
Imagine you have a document:
- Version 1: "Draft 1"
- Version 2: "Added chapter 3"
- Version 3: "Fixed typos"

Instead of saving "Draft1_final_real_v2.docx", Git automatically tracks all these versions.

**Why Developers Care:**
- Never lose work (every version is saved)
- Collaborate with teammates safely
- Experiment without fear (easy to revert changes)
- Professional standard (most tech jobs require Git)

**You'll Learn Git In:**
Chapter 8 of this book - we'll teach you Bash first (Chapter 7) so you're ready.

Sources:
- git-scm.com (official Git documentation)
- GitHub Learning Lab
- Atlassian Git Tutorials
```

**What happened**: Gemini explained Git concepts for a beginner learning Bash in Chapter 7, helping you understand "what to expect next" before diving into commands.

### When Google Search Activates

Gemini uses Google Search when you ask about:
- **Concepts you're learning**: "What is REST API?" or "What is MongoDB?"
- **Prerequisite knowledge**: "What should I learn before Python?"
- **Career information**: "What programming languages have the most jobs in 2025?"
- **Tool comparisons**: "What's the difference between VS Code and PyCharm?"
- **Learning resources**: "What are the best free Python tutorials for beginners?"

---

## Tool 2: File Operations — Reading Your Files

**When Gemini uses this**: You need Gemini to read or analyze text files in your current directory.

### Example: Reading Your Learning Plan

You ask Gemini:

```
Read my learning-plan.md and tell me what topics I'm studying this week
```

**What you'll see:**

```
> Read my learning-plan.md and tell me what topics I'm studying this week

📁 Reading learning-plan.md...

Your learning plan for this week covers:

**Chapter 6: Gemini CLI (Current)**
- Lesson 1: Installation and setup (completed)
- Lesson 2: Using Gemini for questions (completed)
- Lesson 3: Built-in tools (you're here!)
- Lesson 4: Conversation context (next)

**Chapter 7: Bash Essentials (Next week)**
- Basic commands: pwd, ls, cd
- File operations: mkdir, cp, rm
- Understanding the terminal

**Chapter 8: Git Basics (Following week)**
- Repository creation
- Commits and branches
- Collaborative workflows

**Your Progress:**
You've completed 2 of 4 lessons in Chapter 6. After Gemini CLI, you'll be ready for Bash and Git!

Would you like help preparing for Chapter 7 Bash topics?
```

**What happened**: Gemini read your learning-plan.md file, summarized your progress, and offered supportive guidance aligned with your learning journey.

### Example: Reading Course Notes

You ask Gemini:

```
Read my chapter-6-notes.md and summarize the main concepts I've learned so far
```

**What you'll see:**

```
> Read my chapter-6-notes.md and summarize the main concepts I've learned so far

📁 Reading chapter-6-notes.md...

From your Chapter 6 notes, you've learned:

**Key Concepts:**
1. Gemini CLI is an AI assistant that runs in your terminal
2. You ask questions naturally (no special syntax needed)
3. Gemini has 4 built-in tools that activate automatically:
   - Google Search for current information
   - File Operations for reading local files
   - Shell Integration for running commands
   - Web Fetch for specific URLs

**How It Helps Learning:**
- Instead of searching multiple websites, ask Gemini one question
- Gemini finds current information from reliable sources
- Cites where information comes from

**What You're Ready For:**
- ✅ Understanding Bash concepts before coding
- ✅ Learning Git fundamentals
- ✅ Reading documentation with AI help
- ✅ Researching programming topics independently

Your notes show solid foundational understanding. You're ready to move to the next lesson!
```

**What happened**: Gemini read your notes file, identified key learning points, and gave you feedback on readiness for the next chapter.

### When File Operations Activate

Gemini reads files when you ask about:
- **Your notes**: "Read my study-notes.md and quiz me on key concepts"
- **Documentation**: "Read the README.md file and explain what this project does"
- **Lists and plans**: "Read my TODO.txt and help me prioritize tasks"
- **Configuration examples**: "Read the .gitignore template and explain what each line means"
- **Course materials**: "Read chapter-7-overview.md and prepare me for Bash"

---

## Tool 3: Shell Integration — Running Terminal Commands

**When Gemini uses this**: You need to execute basic terminal commands and get the results.

### Example: Checking Your Current Directory

You ask Gemini:

```
What directory am I currently in? Show me what files are here.
```

**What you'll see:**

```
> What directory am I currently in? Show me what files are here.

⚡ Running: pwd

Current directory: /Users/alex/Documents/learning

⚡ Running: ls

Files and folders in your current directory:
- learning-plan.md
- chapter-6-notes.md
- chapter-7-prep.md
- README.md
- images/ (folder)

You're in your learning directory with organized study materials. Would you like me to:
- Read any of these files?
- Help you organize your study materials?
- Create a new file for upcoming lessons?
```

**What happened**: Gemini ran the `pwd` command to show your location and `ls` to list files, helping you understand your file organization.

### Example: Checking System Information

You ask Gemini:

```
What is today's date? Show me the current time.
```

**What you'll see:**

```
> What is today's date? Show me the current time.

⚡ Running: date

Current date and time:
Monday, January 17, 2025 - 2:30 PM EST

This helps you:
- Know when you started learning today
- Track your progress this week
- Plan your next learning session

You've been learning about Gemini CLI for about 2 hours now - time for a break!
```

**What happened**: Gemini ran the date command to show current time and used it to give you helpful context.

### Example: Seeing What Shell You're Using (Prep for Bash)

You ask Gemini:

```
What shell am I using? Is it Bash?
```

**What you'll see:**

```
> What shell am I using? Is it Bash?

⚡ Running: echo $SHELL

Your current shell: /bin/bash

Great! You're using Bash, which is what you'll learn in Chapter 7.

**What This Means:**
- Bash is the most common shell for beginners
- When you start Chapter 7 (Bash Essentials), you already have it installed
- You'll learn to use commands like: pwd, ls, mkdir, cp, rm

This puts you in a great position to learn Bash fundamentals!
```

**What happened**: Gemini checked your shell environment and connected it to your upcoming learning in Chapter 7, showing how this tool helps you prepare.

### When Shell Integration Activates

Gemini runs commands when you ask about:
- **System info**: "What's my current directory?" or "What shell am I using?"
- **File listings**: "Show me what files are in this folder"
- **Checking setup**: "Is Python installed on my computer?"
- **Getting information**: "What's today's date?" or "Who am I logged in as?"
- **Preparation**: "Check what tools I have before learning Chapter 7"

---

## Tool 4: Web Fetch — Retrieving Specific Pages

**When Gemini uses this**: You know the exact URL and want Gemini to fetch and analyze that specific page.

### Example: Learning Python Fundamentals from Official Docs

You ask Gemini:

```
Fetch the Python.org getting started page and explain what Python can do
```

**What you'll see:**

```
> Fetch the Python.org getting started page and explain what Python can do

🌐 Fetching https://www.python.org/about/...

What is Python?

Python is a programming language that lets you:

**Easy to Learn:**
- Simple, readable syntax (looks like English)
- Great for beginners
- Large community with tons of tutorials

**Powerful & Versatile:**
- Web development (Django, Flask)
- Data science and analysis (NumPy, Pandas)
- Machine learning (TensorFlow, Scikit-learn)
- Automation and scripting
- Scientific computing

**In High Demand:**
- Top 3 most popular programming languages
- Strong job market for Python developers
- Used by companies: Google, Netflix, Spotify, Instagram

**Why Start With Python?**
- Beginner-friendly syntax
- Can build real projects quickly
- Skills transfer to other languages
- Active community means lots of help available

You'll learn Python starting in Chapter 12 of this book!

Source: python.org (Official Python Website)
```

**What happened**: Gemini fetched Python's official website, extracted beginner-friendly information, and connected it to your learning journey.

### Example: Understanding Bash Before You Learn It

You ask Gemini:

```
Fetch the Bash introduction from the GNU Bash manual and explain why it matters for learners
```

**What you'll see:**

```
> Fetch the Bash introduction from the GNU Bash manual and explain why it matters for learners

🌐 Fetching https://www.gnu.org/software/bash/manual/html_node/What-is-Bash_003f...

What is Bash?

Bash is a command interpreter and shell - your interface to communicate with your computer.

**What You'll Do With Bash:**
- Navigate folders (pwd, ls, cd)
- Create and delete files (touch, rm)
- Run programs and scripts
- Automate repetitive tasks
- Manage your development environment

**Why Programmers Care:**
- Universal across Mac, Linux, and most systems
- Professional standard (used in all tech jobs)
- Foundation for understanding systems
- Essential for using Git and development tools

**Your Learning Path:**
- Chapter 7: Bash Essentials (basic commands)
- Chapter 8: Git (uses Bash commands)
- Chapters 9+: All use Bash for tools and automation

**Pro Tip:**
Once you understand Bash, learning Git becomes much easier because Git uses Bash commands!

Source: GNU Bash Manual (Official Documentation)
```

**What happened**: Gemini fetched official Bash documentation, extracted relevant concepts for a beginner, and showed how it connects to your learning path.

### When Web Fetch Activates

Gemini fetches specific pages when you ask about:
- **Official documentation**: "Fetch Python.org to learn about Python"
- **Official tutorials**: "Get the Git official tutorial from git-scm.com"
- **Release notes**: "Fetch the latest Python release notes"
- **Official guides**: "Get the Bash user guide from the GNU website"
- **Course materials**: "Fetch the official Markdown guide from commonmark.org"

---

## How Gemini Decides Which Tool to Use

Understanding the decision logic helps you craft better queries:

| Your Need | Tool Gemini Uses | Example Query |
|-----------|------------------|---------------|
| Current information/concepts | 🔍 Google Search | "What is Python used for?" |
| Read your learning notes | 📁 File Operations | "Read my learning-plan.md" |
| Check system information | ⚡ Shell Integration | "What directory am I in?" |
| Official documentation | 🌐 Web Fetch | "Fetch Python.org getting started page" |
| Learning resources | 🔍 Google Search | "What are best free tutorials for Bash?" |
| Your study notes | 📁 File Operations | "Read chapter-6-notes.md and quiz me" |
| Check your setup | ⚡ Shell Integration | "What shell am I using?" |
| Official guides | 🌐 Web Fetch | "Get Git guide from git-scm.com" |

**The Pattern**:
- **Learning concepts/comparisons/current info** → Google Search
- **Your notes/plans/local files** → File Operations
- **System commands/directory info** → Shell Integration
- **Official documentation/specific URLs** → Web Fetch

---

## Recognizing When Tools Are Active

As you use Gemini, watch for these indicators:

### Visual Indicators

**Status messages appear before results:**
```
🔍 Searching the web...
📁 Reading package.json...
⚡ Running: git status
🌐 Fetching https://docs.python.org/3.13/...
```

### Source Citations

When Gemini uses tools, it cites sources:
```
Sources:
- nodejs.org (official release page)
- GitHub nodejs/node releases
- Retrieved: January 17, 2025
```

### Structured Output

Tool responses often have clear structure:
```
Your project has 8 dependencies:

Production Dependencies:
- react: ^18.2.0
- next: ^14.0.4

Development Dependencies:
- typescript: ^5.3.3
```

### Command Echo

Shell integration shows the exact command run:
```
⚡ Running: node --version

v20.10.0
```

**Why this matters**: Knowing which tool activated helps you verify Gemini did what you expected and learn when each tool is appropriate.

---

## Real-World Learning Scenarios

Here's how tools work together in your learning journey:

### Scenario 1: Starting Your Learning Journey

**You**: "I'm new to programming. What should I learn first, and how does it connect to what I'm learning now?"

**Gemini**:
```
🔍 Searching the web...

Your Learning Path (Beginner-Friendly):

**Part 1-4: Foundations (No Coding Yet)**
✓ Chapter 1-4: AI Development concepts (you completed these)
✓ Chapter 5: Claude Code basics (you completed this)
→ Chapter 6: Gemini CLI (you're here!)

**Part 2: Developer Fundamentals (Next)**
→ Chapter 7: Bash Essentials (terminal commands)
→ Chapter 8: Git Basics (version control)
→ Chapters 9-11: Documentation (Markdown, Prompts, Context)

**Part 3: Programming Starts Here**
→ Chapter 12-17: Python Fundamentals (your first programming language)
→ Chapters 18+: Advanced Python and beyond

**Why This Order?**
1. Learn concepts before code (less overwhelming)
2. Master terminal before programming (essential skill)
3. Learn version control early (professional practice)
4. Start with Python (beginner-friendly language)

**Next Steps:**
Finish Chapter 6, then Chapter 7 teaches you terminal basics before Chapter 12 coding!

Sources:
- Educational programming standards
- Stack Overflow Learning Path reports
```

**Tools used**: Google Search (learning paths), conceptual understanding

---

### Scenario 2: Understanding What You're About to Learn

**You**: "Read my chapter-7-prep.md and tell me what I should expect from Bash"

**Gemini**:
```
📁 Reading chapter-7-prep.md...

Based on your preparation notes, here's what Chapter 7 will teach:

**Chapter 7: Bash Essentials Overview**

**What You'll Learn:**
1. Navigating directories (pwd, ls, cd)
2. Working with files (touch, mkdir, cp, rm)
3. Understanding the file system structure
4. Reading file contents (cat, less)
5. Basic text manipulation (echo, grep)

**Why This Matters:**
- Bash is the foundation for all terminal work
- Essential for using Git (Chapter 8)
- Required for Python development (Chapter 12+)
- Professional standard across all tech jobs

**You're Ready When:**
- You can navigate your file system
- You understand what "current directory" means
- You can create and delete files and folders
- You feel comfortable with terminal basics

**Your Current Status:**
You have Bash installed and ready (verified in Chapter 6)!

**Confidence Check:**
After Chapter 7, you'll have the skills needed for Chapter 8 (Git).

Would you like me to quiz you on any Chapter 6 concepts before moving forward?
```

**Tools used**: File Operations (your prep notes), contextual learning support

---

### Scenario 3: Researching Tools Before Learning Them

**You**: "What is Git? Why is it important before I learn programming?"

**Gemini**:
```
🔍 Searching the web...

Why Git Matters for Every Programmer:

**What Is Git?**
Git is a version control system - it tracks every change you make to code.

**Real Analogy:**
- Google Docs auto-saves versions? Git does that for code
- Git tracks who changed what, when, and why
- You can go back to any previous version anytime

**Why Learn Git Before Programming?**
1. **Professional Standard**: Every programming job uses Git
2. **Collaboration**: Work with teams safely
3. **Backup**: Never lose your code
4. **Experimentation**: Try ideas without fear
5. **Understanding History**: See how projects evolved

**The Professional Workflow:**
1. Write code (Chapter 12+)
2. Save version with Git (Chapter 8 skills)
3. Push to GitHub for backup
4. Collaborate with teammates
5. Review code before merging

**Your Timeline:**
- Chapter 7: Master Bash (terminal skills)
- Chapter 8: Learn Git (version control)
- Chapter 12: Start coding (now Git skills matter!)
- Chapters 13+: Git becomes daily tool

**Key Insight:**
Bash and Git are the "infrastructure" skills that all programmers need before coding. You're learning in the right order!

Sources:
- GitHub Learning resources
- Stack Overflow Developer Survey
- Professional development standards
```

**Tools used**: Google Search (Git importance), learning pathway context

---

### Scenario 4: Verifying Your Setup for Next Chapter

**You**: "Check what files I have for Chapter 7 preparation. List what's in my current directory."

**Gemini**:
```
⚡ Running: pwd

Current directory: /Users/alex/learning/ai-dev

⚡ Running: ls

Your preparation files:
- chapter-6-notes.md (completed Chapter 6 notes)
- chapter-7-prep.md (Chapter 7 preparation guide)
- learning-plan.md (overall learning roadmap)
- README.md (project overview)
- bash-cheatsheet.md (Bash command reference - ready for Chapter 7!)

**Your Setup Status:**
✅ You're organized and ready for Chapter 7!
✅ You have preparation materials for Bash
✅ Your notes show good progress

**What to Do Next:**
1. Review chapter-7-prep.md one more time
2. Start Chapter 7 when ready
3. Keep bash-cheatsheet.md handy while learning

🔍 Quick check - should we verify what Bash version you have installed before Chapter 7?
```

**Tools used**: Shell Integration (verify setup), File Operations (review your files)

---

### Scenario 5: Getting Official Documentation Before Starting

**You**: "Fetch the official Python tutorial from python.org so I know what to expect when I reach Chapter 12"

**Gemini**:
```
🌐 Fetching https://docs.python.org/3/tutorial/index.html...

Python Tutorial Overview (What You'll Learn Eventually):

**Chapter 1: Whetting Your Appetite**
- What Python can do
- Why it's great for beginners

**Chapter 2: Using Python (Chapter 7-type concepts)**
- Running Python interpreter
- Using command-line arguments
- Understanding the environment

**Chapters 3-5: Basic Concepts (Chapter 12-15 will cover)**
- Variables and data types
- Control flow (if/while/for)
- Functions

**Chapters 6-10: Advanced Topics (Chapters 16+)**
- Classes and objects
- Modules and packages
- File handling
- Error handling

**Why This Preview Helps:**
- You know what concepts are coming
- You understand the progression
- Python has a gentle learning curve
- You're already building foundational skills (Bash, Git, terminal)

**Your Position:**
You're in Chapter 6 of a 83-chapter learning journey. You're building skills in the right order to be successful with Python starting in Chapter 12!

Source: python.org Official Tutorial
```

**Tools used**: Web Fetch (official documentation), learning pathway visualization

---

## Try With AI

Now practice using Gemini's tools with these exercises. Use your preferred AI companion (Gemini CLI or Claude Code) - the prompts work with any tool.

### Prompt 1: Practice Google Search — Learning Concepts

```
What is Bash and why do I need to learn it before Python?
```

**Expected outcome**: You'll see 🔍 indicator, Gemini searches the web, returns current information about Bash, its importance, and how it connects to Python programming.

---

### Prompt 2: Practice File Operations — Your Learning Files

```
Read my learning-plan.md and tell me what I should focus on for the next week
```

**Expected outcome**: You'll see 📁 indicator, Gemini reads your learning-plan.md file, understands your current progress, and suggests a focused next step.


---

### Prompt 3: Practice Shell Integration — Your System Setup

```
What directory am I in? Show me what files I have for my learning.
```

**Expected outcome**: You'll see ⚡ indicators for `pwd` and `ls` commands, showing your current directory and files in that directory. Gemini explains what files you have ready for learning.

---

### Prompt 4: Practice Web Fetch — Official Documentation

```
Fetch the Python.org getting started page and tell me what Python is used for
```

**Expected outcome**: You'll see 🌐 indicator, Gemini retrieves Python.org content, extracts beginner-friendly information about Python's uses, and connects it to your learning timeline (Chapter 12).


