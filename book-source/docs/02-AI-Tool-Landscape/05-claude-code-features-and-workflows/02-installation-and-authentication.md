---
sidebar_position: 2
title: "Installing and Authenticating Claude Code"
duration: "25-30 min"
---

# Installing and Authenticating Claude Code

## From Concept to Reality: Getting Claude Code Running

In Lesson 1, you learned why Claude Code is revolutionary. Now comes the crucial step: **getting it working on your machine.**

This isn't just about following installation commands. It's about crossing the bridge from "interesting concept" to "tool I can actually use." By the end of this lesson, Claude Code will be installed, authenticated, and ready to assist with your development work.

We've designed this lesson to achieve a **95% first-attempt success rate**—meaning you should be up and running without needing external help. We'll cover Windows, macOS, and Linux with multiple installation methods, clear authentication paths, and comprehensive troubleshooting for common issues.

Let's get started.

---

## Prerequisites: What You Need Before Installing

Before we begin, verify you have the following:

**1. Terminal Access**
- **Windows**: Command Prompt, PowerShell, or Windows Terminal
- **macOS**: Terminal app (Applications → Utilities → Terminal)
- **Linux**: Any terminal emulator (GNOME Terminal, Konsole, etc.)

**2. Claude Account** (one of the following):
- **Option A**: Claude.ai subscription (Pro or free tier)
  - Sign up at: https://claude.ai
  - You'll use this account to authenticate Claude Code
- **Option B**: Claude Console account with API credits
  - Create account at: https://console.anthropic.com
  - Requires payment method for API usage

**3. Node.js 18+ (for NPM installation method)**
- Check if installed: `node --version`
- If not installed: https://nodejs.org (download LTS version)

**4. Internet Connection**
- Needed for initial download and authentication
- Claude Code requires connection to communicate with Claude AI

---

## Installation: Choose Your Method

**Recommended**: NPM installation (works on all platforms)

### Method 1: NPM Installation (All Platforms)

**Step 1: Install Claude Code Globally**

Open your terminal and run:

```bash
npm install -g @anthropic-ai/claude-code
```

**What this does**: Downloads and installs Claude Code globally, making it accessible from any directory.

**Common issue on macOS/Linux**: Permission errors

**If you see `EACCES` permission error**:
```bash
sudo npm install -g @anthropic-ai/claude-code
```

**Step 2: Verify Installation**

```bash
claude --version
```

**Expected output** (version number may vary):
```
2.0.35 (Claude Code)
```

---

## Authentication: Connecting Claude Code to Your Account

Once installed, Claude Code needs to authenticate with your Claude account. There are **two authentication paths** depending on which account type you have.

### Which Authentication Method Should I Use?

**Decision Tree**:

```
Do you have a Claude.ai account?
├─ Yes → Use Claude.ai Authentication (Method A)
│        Most common for individual users
│
└─ No, but I have Claude Console API credits
   └─ Use Claude Console Authentication (Method B)
           Common for developers with API access
```

**If you have both**: Use Claude.ai authentication (Method A).

---

### Authentication Method A: Claude.ai Account (Most Common)

**Step 1: Start the Authentication Flow**

In your terminal, run:

```bash
claude
```

**Expected output**:
```
Choose the text style that looks best with your terminal
To change this later, run /theme
❯ 1. Dark mode ✔

 Select login method:

 ❯ 1. Claude account with subscription · Pro, Max, Team, or Enterprise

   2. Anthropic Console account · API usage billing
```

**What happens**: Your default browser opens to the Claude.ai authentication page.

**Step 2: Log In to Claude.ai**

1. If not already logged in, enter your Claude.ai credentials
2. Review the permissions Claude Code is requesting
3. Click "Allow" or "Authorize"

**Step 3: Confirm Authentication**

Return to your terminal. You should see:

```
Logged in as email@gmail.com
Login successful. Press Enter to continue…
```

**Step 4: Test Your Setup**

Run a simple test command:

```bash
claude "Hello! Can you confirm Claude Code is working?"
```

**Expected output**: Claude responds with a greeting confirming the connection works.

---

## Pause and Reflect: You've Crossed the Bridge

**If your installation and authentication succeeded**: Take a moment to appreciate what you've accomplished. Claude Code is now installed and ready to assist you. You've moved from theory to practice.

**Reflection Questions**:
- Which installation method did you choose, and why?
- Did you encounter any issues? If so, which troubleshooting steps helped?
- How does it feel to have an AI assistant accessible directly from your terminal?

**If you're still troubleshooting**: Don't get discouraged. Installation challenges are normal, especially across different platforms and environments. Work through it systematically, and don't hesitate to seek help from the community resources.

## Setting Up CLAUDE.md (Project Memory)

**What is CLAUDE.md?**

CLAUDE.md is a special file that acts as your project's "memory" for Claude Code. It contains:
- Project overview and purpose
- Coding standards and conventions
- Common tasks and how to execute them
- Important context Claude should remember

**Think of it as**: Instructions you'd give a new team member joining your project.

---

### Creating Your First CLAUDE.md

**Step 1: Create the file in your project root**

```bash
claude "Hello! Creatr a CLAUDE.md  file and save my name Muhammad as Project Manager."
```

**Step 2: Ask Claude Code about your name**

```bash
claude "Who is Project Manager."
```

**Expected outcome**: Claude knows Project Manager

---

### Why CLAUDE.md Matters

**Without CLAUDE.md**: You repeat project context in every conversation
- "This is a Python project using Flask..."
- "We follow PEP 8..."
- "Tests are in the /tests folder..."

**With CLAUDE.md**: Claude remembers automatically
- You: "Add tests for the new feature"
- Claude: [already knows testing framework, where tests go, coding standards]

**Time saved**: 2-5 minutes per Claude Code session

---

### When to Update CLAUDE.md

Update it when:
- ✅ You add a new major dependency
- ✅ Coding standards change
- ✅ Project structure changes
- ✅ You notice Claude repeatedly asking for the same context

**CLAUDE.md is iterative**: Start simple, add details as needed. It grows with your project.

---

## Try With AI

**Goal**: Complete a simple task to see immediate value

---

**Ask Claude Code to introduce itself**:

```bash
claude "I just installed you! Give me 3 simple tasks I can ask you to do right now to understand what you're capable of. Make them beginner-friendly."
```

**Expected outcome**: Claude suggests 3 safe, simple tasks you can try

---

<Quiz title="Chapter 2 Quiz" questions={[{"question":"What did Sarah Chen, a solo founder, build in forty-eight hours with the help of Claude Code?","options":{"a":"A simple personal blog.","b":"A complete customer analytics dashboard for 1,200 customers.","c":"A mobile game.","d":"A tutorial on PHP."},"correct_answer":"b","explanation":"The text explicitly states, \u0027In forty-eight hours, she\u0027d built a complete customer analytics dashboard that two months ago would have required a team of five developers and three weeks of work... Her dashboard processed real-time data for 1,200 customers.\u0027"},{"question":"How does the speed of the current AI coding revolution compare to previous transitions in software development?","options":{"a":"It is happening over 10-15 years, similar to past transitions.","b":"It is happening much slower than previous transitions.","c":"It is happening in months, not years.","d":"The speed is exactly the same."},"correct_answer":"c","explanation":"The chapter notes, \u0027Previous transitions... took 10-15 years... The AI coding revolution is happening in months, not years.\u0027"},{"question":"According to the Stack Overflow 2024 Developer Survey cited in the chapter, what percentage of professional developers are already using AI coding tools?","options":{"a":"10%","b":"44%","c":"62%","d":"76%"},"correct_answer":"c","explanation":"The text provides the statistic: \u002776% of professional developers are using or plan to use AI coding tools, with 62% already using them—up from 44% last year (Stack Overflow 2024 Developer Survey).\u0027"},{"question":"What is the chapter\u0027s main answer to questions like \u0027Am I too late?\u0027 or \u0027Will this replace me?\u0027 for developers?","options":{"a":"It is too late for new developers to enter the field.","b":"AI will replace most developers within the next year.","c":"This is the best time in decades to be learning software development because of AI.","d":"Developers should switch to a different career."},"correct_answer":"c","explanation":"The text directly answers, \u0027The answer to all four is the same, and it might surprise you: This is the best time in decades to be learning software development. Not despite AI. Because of it.\u0027"},{"question":"How is the role of a developer evolving in the AI era, according to the chapter?","options":{"a":"From a high-level architect to a low-level coder.","b":"It is not evolving; the tasks remain the same.","c":"From a typist writing code line by line to an orchestrator managing AI agents.","d":"Developers are now primarily focused on hardware and infrastructure."},"correct_answer":"c","explanation":"The chapter states, \u0027Your role as a developer is evolving from typist (writing code line by line) to orchestrator (managing AI agents, making architectural decisions, exercising judgment).\u0027"}]} />

