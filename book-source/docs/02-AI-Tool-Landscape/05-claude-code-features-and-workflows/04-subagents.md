---
sidebar_position: 4
title: "Understanding and Using Subagents"
---

# Understanding and Using Subagents

## The Problem: Context Pollution and Specialization

You've installed Claude Code and run your first commands. But as you use it more, you'll encounter a common challenge: **context pollution**.

Imagine this scenario: You're working on a Python project. You ask Claude Code to help debug a function, then to review your code for style issues, then to generate documentation, then to suggest performance optimizations. By the time you're on your fifth request, Claude Code's context includes all your previous conversations—debugging details, style discussions, documentation drafts. The conversation has become cluttered.

**What if you could press "reset" without losing your work?**

**What if you could have specialized AI assistants for different tasks—a code reviewer, a documentation writer, a debugging expert—each with clear focus and isolated context?**

That's what **subagents** enable. They're one of Claude Code's most powerful features, yet often overlooked by beginners who stick to the main conversation for everything.

In this lesson, you'll learn what subagents are, when to use them, and how to create your first custom subagent—a code reviewer that applies your team's specific standards.

---

## What Are Subagents?

**Definition**: A subagent is a specialized task-specific agent for improved context management. They are configured with custom instructions (called a "system prompt") and isolated context separate from the main conversation.

Think of subagents as specialized team members:

- **Main Claude Code**: Your general assistant—handles one-off questions, exploratory tasks, varied workflows. It will manage the subagents as well.
- **Subagent 1 (Code Reviewer)**: Focused solely on reviewing code for bugs, style, and best practices
- **Subagent 2 (Test Writer)**: Specialized in generating unit tests for functions
- **Subagent 3 (Documentation Generator)**: Expert at writing clear, comprehensive documentation

Each subagent has:
1. **Custom system prompt**: Instructions that define its role, constraints, and expertise
2. **Isolated context**: Conversations in this subagent don't affect the main conversation or other subagents
3. **Tool access configuration**: You can limit which tools (file read/write, command execution) the subagent can use

### Subagents vs. Main Conversation: When to Use Each

| Aspect | Main Conversation | Subagent |
|--------|-------------------|----------|
| **Use Case** | Exploratory work, varied tasks, one-off questions | Repetitive specialized tasks with clear scope |
| **Context** | Accumulates all conversation history | Isolated—only sees subagent's own history |
| **Customization** | General-purpose Claude behavior | Custom system prompt defines specific role |
| **When to Use** | "Help me understand this error" | "Review this code using our team's style guide" |
| **Examples** | Debugging, learning concepts, exploring codebases | Code reviews, test generation, refactoring, documentation |

**Key Insight**: Use subagents when you have **repeatable tasks with clear instructions**. Use the main conversation when you need flexibility and exploration.

---

## Why Subagents Matter: Three Key Benefits

### Benefit 1: Context Preservation

**Problem Without Subagents**: After 10 interactions in the main conversation, Claude Code's context is filled with unrelated discussions. Asking for a code review now means Claude considers all previous context—even if irrelevant.

**Solution With Subagents**: Launch a fresh subagent for each focused task. The code review happens in a clean context, without clutter from previous debugging or documentation work.

### Benefit 2: Specialization and Consistency

**Problem Without Subagents**: You must repeatedly tell Claude Code your code review standards, testing framework preferences, or documentation style with every new session.

**Solution With Subagents**: Encode these instructions once in a subagent's system prompt. Every time you invoke the "code-reviewer" subagent, it already knows your standards.

**Example System Prompt for Code Reviewer**:
```
You are a code reviewer specializing in Python.
Apply these standards:
- PEP 8 style compliance
- Type hints required for all functions
- Docstrings required (Google style)
- Prefer list comprehensions over loops where readable
- Flag any security concerns (SQL injection, hardcoded secrets)
- Suggest performance improvements for O(n²) or worse complexity
```

Every code review from this subagent applies these standards consistently.

### Benefit 3: Reusability Across Projects

**Problem Without Subagents**: You manually recreate your preferred workflows for each new project.

**Solution With Subagents**: Subagents are stored as files in `.claude/agents/`. You can:
- Share subagents with your team via version control
- Reuse the same subagent across multiple projects
- Build a library of specialized assistants

A well-designed subagent becomes **organizational knowledge**—capturing expertise that benefits your entire team.

---

## Subagent Architecture: How They Work

Let's peek under the hood to understand how subagents function.

### File Structure

When you create a subagent named `code-reviewer`, Claude stores a single file under `.claude/agents/` with:
- A name and short description of its purpose
- Optional model selection and color
- A clear checklist of standards it applies
- The expected form of its output (e.g., prioritized issues and actionable suggestions)

### Subagent Lifecycle (Zero Overhead View)

1. Create once via `/agents` and describe the role.
2. Use naturally ("Use the code-reviewer subagent...")—Claude may also auto‑delegate when the match is obvious.
3. Work happens in a clean, isolated context; results return to the main thread.
4. Iterate the description over time as your team practices evolve.

---

## Creating a "Latest News" Subagent

Let's create a **"latest-news" subagent**—a focused researcher that surfaces current headlines, trends, and concise summaries with citations.

### Step 1: Create the Subagent

- Start claude session, from slash commands run `/agents` interface and select "Create new agent"
- Choose Project location so your team can reuse it
- Describe the role succinctly, for example:
  - Purpose: daily news researcher for a chosen domain (e.g., AI, security)
  - Output: 5–7 bulleted headlines with one‑sentence summaries and links; a short trend synopsis; 2 follow‑up questions
  - Constraints: prioritize reputable sources; avoid duplicates; keep total read under 2 minutes

Claude will create the subagent file under `.claude/agents/latest-news.md` (or similar) with your description and settings.

**Expected result**: The agent appears in your list and retains its focused role.

### Step 2: Try the "Latest News" Subagent (Daily Workflow)

- Try now: Ask Claude to use the latest-news subagent for your topic today (for example, AI policy). You should receive headlines with links, a short trend synopsis, and two follow‑up questions.

**If you see targeted, phase‑specific feedback**: ✅ It works. You get clean execution and clear results with minimal prompting.

---

## Delegation Modes

Subagents can be used in two ways:

- Explicit invocation: You request a specific subagent (e.g., "Use the code-reviewer subagent to check my recent changes").
- Automatic delegation: Claude can delegate to a subagent when your task clearly matches that subagent’s description and allowed tools.

Use explicit invocation for predictability. Rely on automatic delegation as a convenience when descriptions are specific. 

---

## ✓ Your Subagent Is Working When:

**Quick verification**:

1. **Subagent is created** - You can list it
2. **Subagent can be invoked** - You can ask it to explain code
3. **It returns helpful explanations** - The response makes sense and helps clarify the code
4. **It stays in character** - Multiple invocations maintain the "collaborative explainer" role

**If this works**: 🎉 **Your collaborative helper is ready! You now have a dedicated partner to help you understand code.**

---

## Subagent Best Practices

**1. Keep System Prompts Focused**
- One clear responsibility per subagent
- Avoid combining unrelated tasks (e.g., "review code AND generate docs")

**2. Use Descriptive Names**
- ✅ `python-code-reviewer`, `pytest-test-generator`
- ❌ `helper`, `tool1`, `bob`

**3. Limit Tool Access Appropriately**
- Read-only subagents for analysis tasks
- Full access only when necessary (e.g., refactoring subagents)

**4. Iterate and Improve**
- After using a subagent, refine its system prompt
- Add examples of good/bad outputs to guide behavior
- Collect feedback from team members

---

## Pause and Reflect: Specialization vs. Flexibility

You've learned how to create specialized AI assistants with subagents. But there's a tradeoff to consider:

**Specialization** provides consistency and focus, but reduces flexibility. A code-reviewer subagent won't help you debug runtime errors or generate documentation—that's by design.

**The main conversation** is flexible and exploratory, but lacks the focused instructions and clean context that subagents provide.

**Reflection Questions**:
- Think about your own daily workflow. What repetitive tasks could benefit from a specialized subagent?
- When would you prefer the flexibility of the main conversation over a focused subagent?
- If you work on a team, what subagents could capture and share your team's unique expertise?

---

## Try With AI

Use Claude Code for this activity. If you already have another AI companion tool set up (e.g., ChatGPT web, Gemini CLI), you may use that instead—the prompts are the same.

### Prompt 1: Decision Tree for Tool Selection

```
I'm confused about when to use subagents vs. the main conversation. Give me a simple decision tree: For each scenario below, should I use a subagent or main conversation? (a) Quick question about Python syntax, (b) Code review following team standards, (c) Debugging a weird error I've never seen, (d) Generating tests for 10 similar functions, (e) Exploring a new codebase I just downloaded.
```

**Expected outcome:** Clear decision framework for when to use subagents

### Prompt 2: First Subagent Design

```
I want to create my FIRST subagent. My most repetitive task is [describe your task: code reviews / writing tests / generating docs / etc.]. Help me design it: (a) What should I name it? (b) What 3-5 instructions should go in the system prompt? (c) What tool access should it have? (d) Give me the exact command to create it.
```

**Expected outcome:** Complete design for your first custom subagent

### Prompt 3: Custom Code Reviewer

```
The lesson shows a code-reviewer subagent with specific standards (PEP 8, type hints, docstrings). I work with [your language/framework]. Create a custom system prompt for a code-reviewer subagent tailored to MY stack. Include 5-7 specific standards I should enforce. Make it copy-paste ready.
```

**Expected outcome:** Copy-paste-ready system prompt for your specific needs

<Quiz title="Chapter 4 Quiz" questions={[{"question":"What is the key difference between \u0027external disruption\u0027 and the \u0027internal disruption\u0027 currently happening in software development?","options":{"a":"External disruption is slow, while internal disruption is even slower.","b":"External disruption is when a software company disrupts another industry, while internal disruption is when AI tools transform the software industry itself.","c":"External disruption affects senior developers, while internal disruption affects junior developers.","d":"External disruption is voluntary, while internal disruption is forced upon developers."},"correct_answer":"b","explanation":"The chapter defines the pattern: \u0027External force: Software companies built platforms that competed with traditional retailers... Internal force: The same industry creating the tools is being transformed by them.\u0027"},{"question":"According to the chapter, why is the adoption of AI coding tools happening so much faster than previous technology shifts?","options":{"a":"Because developers are being forced to use them by their managers.","b":"Because there is no external resistance, and developers are adopting them voluntarily for immediate value.","c":"Because the tools are expensive, creating a sense of urgency.","d":"Because previous technology shifts were not very useful."},"correct_answer":"b","explanation":"The text highlights several reasons for the speed, a key one being: \u0027No External Resistance. When AI tools disrupt software development, there\u0027s no external resistance. Developers are adopting these tools voluntarily and enthusiastically...\u0027"},{"question":"What does the chapter mean by the \u0027Recursion Effect\u0027?","options":{"a":"AI coding tools are getting stuck in infinite loops.","b":"Developers are recursively calling the same functions, leading to bugs.","c":"AI coding tools are being used to improve and develop the next version of themselves, creating a rapid improvement cycle.","d":"The process of learning to code is becoming recursive and more difficult."},"correct_answer":"c","explanation":"The chapter explains this \u0027mind-bending\u0027 concept: \u0027AI coding tools are being used to improve AI coding tools... This creates a recursive improvement cycle that has no parallel in previous disruptions.\u0027"},{"question":"How does the impact of AI coding tools on developer roles differ from previous technology shifts like cloud computing or mobile development?","options":{"a":"AI coding tools only affect junior developers.","b":"AI coding tools have a universal impact, affecting every role in the software development value chain simultaneously.","c":"AI coding tools have less impact than previous shifts.","d":"Previous shifts affected all roles, while AI tools only affect a few."},"correct_answer":"b","explanation":"The text states, \u0027AI coding tools affect everyone simultaneously,\u0027 and then lists the impact on junior, mid-level, senior, DevOps, QA, and technical writers, concluding, \u0027There\u0027s nowhere in the software development value chain that remains untouched.\u0027"},{"question":"What is the chapter\u0027s conclusion about the inevitability of AI coding tools?","options":{"a":"The \u0027if\u0027 question is still debated, and their adoption is uncertain.","b":"The \u0027if\u0027 question is already answered; the only remaining question is \u0027how fast?\u0027","c":"The tools are likely a temporary hype or trend.","d":"The adoption will be slow and may take over a decade."},"correct_answer":"b","explanation":"The chapter asserts, \u0027With AI coding, the \u0027if\u0027 question is already answered. The tools exist, they work, they\u0027re being adopted at scale, and they\u0027re improving rapidly. The only remaining question is \u0027how fast?\u0027"}]} />

