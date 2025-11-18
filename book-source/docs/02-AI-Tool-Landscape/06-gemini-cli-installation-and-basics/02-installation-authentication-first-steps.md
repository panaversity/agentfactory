---
sidebar_position: 2
title: Installation, Authentication & First Steps
cefr_level: A2
proficiency: Beginner
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Direct teaching of installation and setup before AI collaboration"
cognitive_load:
  concepts_count: 6
  a2_compliant: true
  scaffolding_level: "Heavy"
learning_objectives:
  - id: LO1
    description: "Install Gemini CLI using appropriate method for their environment"
    bloom_level: "Apply"
  - id: LO2
    description: "Authenticate using Google OAuth to access free tier"
    bloom_level: "Apply"
  - id: LO3
    description: "Navigate the Gemini CLI interface components (input box, status bar, context info)"
    bloom_level: "Understand"
  - id: LO4
    description: "Execute basic slash commands (/help, /tools, /stats, /quit)"
    bloom_level: "Apply"
  - id: LO5
    description: "Use shell mode (!command) to run terminal commands from within Gemini CLI"
    bloom_level: "Apply"
  - id: LO6
    description: "Apply AI-native troubleshooting pattern when encountering installation errors"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO2
    competency_area: "4. Safety"
    competency: "4.2 Protecting personal data and privacy"
  - objective_id: LO3
    competency_area: "1. Information and Data Literacy"
    competency: "1.1 Browsing, searching and filtering data, information and digital content"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO6
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
---

# Installation, Authentication & First Steps

Installing Gemini CLI is like meeting a new colleague—someone who's available 24/7 to help with your work, answer questions, and solve problems. In this lesson, you'll install and launch Gemini CLI, which will then automatically guide you through authentication. You'll be up and running in minutes.


## Prerequisites: What You Need

Make sure you have these before starting:

| Requirement | What It Is | How to Check |
|------------|-----------|-------------|
| **Node.js 20+** | Runtime for JavaScript applications | Open terminal, type: `node --version` |
| **npm** | Package manager (comes with Node.js) | Open terminal, type: `npm --version` |
| **Google account** | For secure authentication | Gmail, YouTube, or any Google account |

### Don't Have Node.js 20+?

1. Visit [nodejs.org](https://nodejs.org/en/download)
2. Download the **LTS version** (Long Term Support—the stable version)
3. Follow the installer steps for your operating system
4. When asked about npm, keep the checkbox checked
5. Restart your computer

### Opening Your Terminal

**Windows:** Search "PowerShell" in your Start menu and open it

**macOS:** Press Cmd+Space, type "Terminal", press Enter

**Linux:** Press Ctrl+Alt+T (most distributions)

---

## Installation Methods

There are three ways to install Gemini CLI, depending on your needs:

### Method 1: Global Installation (Recommended)

This installs Gemini CLI permanently on your system, making it available from any directory:

```bash
npm install -g @google/gemini-cli
```

This command downloads and installs Gemini CLI globally on your computer. You'll see text flowing by—this is normal. Wait for it to complete (usually takes 30-60 seconds).

**When to use**: When you plan to use Gemini CLI regularly across multiple projects.

### Method 2: Run Without Installing (npx)

You can run Gemini CLI without installing it permanently using `npx`:

```bash
npx @google/gemini-cli
```

This downloads and runs the latest version temporarily. Each time you run this command, it checks for the latest version.

**When to use**:
- When you want to try Gemini CLI without committing to installation
- When testing different versions
- On shared/temporary systems where you can't install globally

### Method 3: Install Specific Version

You can install a specific version or release tag:

```bash
# Install latest stable version explicitly
npm install -g @google/gemini-cli@latest

# Install preview/beta version
npm install -g @google/gemini-cli@preview

# Install nightly build (bleeding edge, may be unstable)
npm install -g @google/gemini-cli@nightly
```

**When to use**: When you need a specific version for compatibility or testing purposes.

### Verify Installation

After installation completes (or when using npx), verify it worked:

```bash
# If installed globally
gemini -v

# If using npx
npx @google/gemini-cli --version
```

You should see a version number like `0.4.0` or higher. If you see this, you're ready! ✓

---

## Authentication & First Launch

Now comes the magic—Gemini CLI handles authentication automatically. Simply type:

```bash
gemini
```

When you run this command for the first time, Gemini CLI launches and **automatically guides you through setup**:

### Step 1: Choose Your Theme

Gemini CLI will ask you to select a visual theme for the terminal interface. Choose whichever you prefer—this is just cosmetic. Use arrow keys to select and press Enter.

### Step 2: Choose Authentication Method

You'll see options for authentication:
- **Google login** (free tier: 60 requests/min, 1,000 requests/day - Google AI Studio, 2025)
- **Gemini API Key** (requires API setup)
- **Vertex AI** (requires Google Cloud Project)

**Select "Google login"** for the free tier. This is the beginner-friendly option.

### Step 3: Browser Opens

Your default web browser will automatically open with Google's login page. Simply:
1. Enter your Google account email
2. Enter your password
3. Click "Allow" when Google asks for permission

### Step 4: You're In!

After you authorize, your terminal displays the Gemini CLI interface. You'll see something like this:

```
 ███            █████████  ██████████ ██████   ██████ █████ ██████   █████ █████
░░░███         ███░░░░░███░░███░░░░░█░░██████ ██████ ░░███ ░░██████ ░░███ ░░███
  ░░░███      ███     ░░░  ░███  █ ░  ░███░█████░███  ░███  ░███░███ ░███  ░███
    ░░░███   ░███          ░██████    ░███░░███ ░███  ░███  ░███░░███░███  ░███
     ███░    ░███    █████ ░███░░█    ░███ ░░░  ░███  ░███  ░███ ░░██████  ░███
   ███░      ░░███  ░░███  ░███ ░   █ ░███      ░███  ░███  ░███  ░░█████  ░███
 ███░         ░░█████████  ██████████ █████     █████ █████ █████  ░░█████ █████
░░░            ░░░░░░░░░  ░░░░░░░░░░ ░░░░░     ░░░░░ ░░░░░ ░░░░░    ░░░░░ ░░░░░

Tips for getting started:
1. Ask questions, edit files, or run commands.
2. Be specific for the best results.
3. /help for more information.

╭────────────────────────────────────────────────────────────────────────╮
│ >   Type your message or @path/to/file                                 │
╰────────────────────────────────────────────────────────────────────────╯
 ~/Your/Current/Directory       no sandbox       auto
```

**What you see:**
- **Logo**: The "GEMINI" banner at the top
- **Tips**: Quick start guidance (3 tips)
- **Input Box**: Where you type your messages
- **Status Bar** (bottom):
  - Left: Current directory (your actual location)
  - Middle: Sandbox status (`no sandbox` by default)
  - Right: Mode (`auto` by default)

**Note**: You might also see:
- Context info like "Using: X context files" if you have GEMINI.md files (covered in Lesson 4)
- "X MCP server" if you've configured MCP servers (covered in Lesson 6)
- Update notifications if a newer version is available
- Git branch info if you're in a git repository

**Update Notifications**: You may see a box suggesting updates—this is normal. You can update later with your package manager.

#### 💬 AI Colearning Prompt
> "Why does Gemini CLI use browser-based authentication instead of asking for a password directly in the terminal? What security advantages does this provide?"

---

## Understanding the Gemini CLI Interface

Now that you're inside Gemini CLI, let's understand what you're looking at:

### The Input Box

The main area where you interact:
```
╭────────────────────────────────────────────────────────────────────────╮
│ >   Type your message or @path/to/file                                 │
╰────────────────────────────────────────────────────────────────────────╯
```

- Type your questions or commands here
- Use `@path/to/file` to reference specific files
- Press Enter to send your message

### Status Bar (Bottom)

The status bar shows three important pieces of information:

```
 ~/Documents/development (main*)       no sandbox       auto
```

- **Left**: Current working directory and git branch
  - `~/Documents/development` = your location
  - `(main*)` = git branch (asterisk means uncommitted changes)
- **Middle**: Sandbox mode
  - `no sandbox` = not using containerized environment
  - `docker` or `gvisor` when sandbox is active
- **Right**: Tool execution mode
  - `auto` = Gemini automatically decides when to use tools
  - `manual` = You approve each tool use

### Context Information (When Configured)

Once you configure Gemini CLI (in later lessons), you'll see context information at startup:
```
Using: 2 context files | 1 MCP server
```

- **Context files**: GEMINI.md files (covered in Lesson 4)
- **MCP servers**: Connected external tool servers (covered in Lesson 6)

**For first-time users**: This line won't appear until you add context files or MCP servers. That's normal!

### Basic Slash Commands

Type these commands at the prompt:

- `/help` - See all available commands
- `/tools` - View available tools
- `/stats` - Session statistics
- `/memory show` - Display persistent context
- `/chat save <name>` - Save current conversation
- `/quit` - Exit Gemini CLI

---

## Shell Mode: Running Terminal Commands Inside Gemini

One of Gemini CLI's most powerful features is **shell mode**—the ability to run terminal commands directly from within your Gemini session without exiting.

### How Shell Mode Works

Type `!` followed by any terminal command:

```
!ls -la
```

Gemini executes the command in your current directory and shows you the output.

### Practical Shell Mode Example

Let's say you're working on a project and want to check your current directory structure:

**Inside Gemini CLI, type:**
```
!pwd
```

**Output:**
```
/Users/yourname/projects/my-app
```

**Now list all files:**
```
!ls -la
```

**Output:**
```
total 24
drwxr-xr-x   6 yourname  staff   192 Jan 17 10:30 .
drwxr-xr-x  15 yourname  staff   480 Jan 15 14:20 ..
drwxr-xr-x  12 yourname  staff   384 Jan 17 09:15 .git
-rw-r--r--   1 yourname  staff   245 Jan 16 16:45 README.md
drwxr-xr-x  45 yourname  staff  1440 Jan 17 10:25 node_modules
-rw-r--r--   1 yourname  staff   512 Jan 17 10:30 package.json
```

**Check Node.js version:**
```
!node --version
```

**Output:**
```
v20.10.0
```

### Why Shell Mode Matters

Shell mode lets you:
- ✅ Check your environment (versions, paths, variables)
- ✅ Navigate directories without leaving Gemini
- ✅ Run build scripts, tests, or git commands
- ✅ Verify file operations suggested by Gemini
- ✅ Debug issues by checking actual system state

**Common shell mode use cases:**
- `!git status` - Check git repository state
- `!npm install` - Install dependencies
- `!python --version` - Verify Python installation
- `!cat filename.txt` - Read file contents
- `!mkdir new-folder` - Create directories

### Exiting Shell Mode

After running a shell command, you're automatically returned to normal Gemini chat mode. To exit Gemini CLI entirely, use `/quit`.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize commands like `/help` or `/tools`—you explore conversationally. If you forget a command, just ask: "What commands are available?" Your AI partner tells you. The skill isn't memorization; it's knowing how to ask.

![Gemini CLI Interface](/img/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/l2-gemini-cli-interface.svg)


> **Tip:** This is what your interface should look like after choosing a theme and authenticating. You can now type your first message or use slash commands (`/`) to explore Gemini CLI features.


---

## Your First Task with Gemini

Now that you're inside Gemini CLI, you're ready to put your AI collaborator to work. Simply type your question or request and press Enter:

```
Help me understand what artificial intelligence means
```

Gemini will respond with an explanation. That's it—you're using Gemini CLI!

---

## Understanding the Gemini CLI Session

When you run `gemini`, you're entering an interactive session. Inside this session, you have access to powerful commands and can ask Gemini multiple questions without exiting.

### Session Commands Reference

These slash commands work inside Gemini CLI:

- `/help` - See all available commands and shortcuts
- `/tools` - View all available tools Gemini can use
- `/stats` - See session statistics (tokens used, duration, etc.)
- `/quit` - Exit Gemini CLI and return to your terminal

### How to Exit Gemini

To exit Gemini CLI, simply type:

```
/quit
```

Or press **Ctrl+C twice** to force quit.

#### 🤝 Practice Exercise

> **Ask your AI**: "I'm inside Gemini CLI for the first time. Walk me through: (1) checking what tools are available, (2) seeing session stats, and (3) asking you a test question about machine learning. Then explain what each command does."

**Expected Outcome**: You'll practice using `/tools`, `/stats`, and natural conversation while your AI explains each interaction—building familiarity through guided exploration.

---

## Real-World Workflow: Inside a Gemini Session

Here's what a typical session looks like:

**Step 1: Launch Gemini**
```bash
gemini
```

**Step 2: Inside the session, ask your first question**
```
Explain machine learning to me in simple terms with a real example
```

**Step 3: Gemini responds**
```
Machine learning is a method where computers learn from data...
[Gemini's detailed response]
```

**Step 4: Ask a follow-up question**
```
What are some real-world applications of machine learning?
```

**Step 5: Continue the conversation** (you can ask as many questions as you want)
```
How do companies use machine learning to recommend products?
```

**Step 6: When you're done, exit**
```
/quit
```

---

## When You Hit Problems: AI-Native Troubleshooting

In AI-native development, you don't memorize error solutions—you **ask AI to diagnose and solve problems**. Here's how:

### The AI Troubleshooting Pattern

When you encounter an error during installation or setup:

**Step 1: Copy the complete error message**
- Include the full terminal output, not just the last line
- Capture context: what command you ran, your operating system, Node.js version

**Step 2: Ask your AI assistant**

Use this prompt template:

```
I'm trying to install Gemini CLI and encountered this error:

[Paste complete error message here]

My system:
- OS: [Windows/macOS/Linux]
- Node.js version: [run: node --version]
- npm version: [run: npm --version]

What does this error mean and how do I fix it?
```

**Step 3: Follow AI's diagnosis step-by-step**
- AI will explain what the error means
- AI provides platform-specific solutions
- AI suggests verification steps

**Step 4: If first solution doesn't work, tell AI what happened**

```
I tried [solution AI suggested] but now I'm getting:

[New error message or behavior]

What should I try next?
```

### Real Example: Permission Error

**What you see:**
```
npm ERR! code EACCES
npm ERR! syscall access
npm ERR! path /usr/local/lib/node_modules
```

**Ask your AI Assistant:**

```
I'm getting this error when installing Gemini CLI:

npm ERR! code EACCES
npm ERR! syscall access
npm ERR! path /usr/local/lib/node_modules

My system: macOS 14.2, Node.js v20.10.0

What does this mean and how do I fix it?
```

**AI will explain:**
- This is a permissions issue
- npm doesn't have access to global node_modules directory
- Provide 2-3 solutions ranked by safety
- Walk you through each step

### Why This Approach Works Better

Traditional troubleshooting guides:
- ❌ Cover only known issues at time of writing
- ❌ Become outdated as software versions change
- ❌ Don't adapt to your specific system configuration

AI troubleshooting:
- ✅ Handles new errors not in any documentation
- ✅ Adapts to your specific OS, versions, and environment
- ✅ Explains WHY, not just WHAT to run
- ✅ Iterates with you until problem is solved

#### 🎓 Expert Insight
> The skill isn't memorizing error fixes—it's knowing how to effectively communicate errors to AI. Provide context (OS, versions, what you tried), paste complete error messages, and iterate based on AI feedback. This skill applies to EVERY tool you'll use, not just Gemini CLI.

---



## Try With AI

Now that you've installed Gemini CLI, use your preferred AI companion (Gemini CLI or Claude Code) for these exercises. You can use either the CLI version or web interface—the prompts work with any tool.

### Prompt 1: Learn a New Concept
```
Explain machine learning to me in simple terms with a real example. Then give me 3 real-world applications of machine learning.
```

**Expected outcome**: Clear explanation with concrete examples and practical applications you can understand.

### Prompt 2: Write Professional Content
```
Write a professional email to my manager about my project status. The project is [describe your situation]. Make it concise and positive.
```

**Expected outcome**: Well-structured professional email you can use or adapt.

### Prompt 3: Problem Solving
```
I'm getting an error when I try to install a package. The error message is: [paste error]. What does it mean and how do I fix it?
```

**Expected outcome**: Clear explanation of the error and step-by-step solution.

### Prompt 4: Plan and Organize
```
I want to learn programming. What programming language should I start with and why? Give me a 4-week learning plan with specific goals for each week.
```

**Expected outcome**: Personalized learning recommendation with structured plan you can follow immediately.
