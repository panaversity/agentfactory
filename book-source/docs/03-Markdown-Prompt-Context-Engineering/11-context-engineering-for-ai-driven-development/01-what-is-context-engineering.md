---
title: "What is Context Engineering?"
chapter: 11
lesson: 1
duration_minutes: 15
sidebar_position: 1
description: "Learn what context engineering is and why it's different from prompt engineering in AI-driven development"
keywords: [context engineering, AI development, prompt engineering, AIDD, context management]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Context vs Prompts"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain the difference between context engineering (information environment) and prompt engineering (specific requests) with real-world examples"

  - name: "Recognizing Context Engineering Impact"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Remember"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify whether a development problem is context-related or prompt-related by analyzing AI output quality"

  - name: "Working with AI Code Tools"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can recognize what information AI tools need before generating code (framework, patterns, existing code)"

learning_objectives:
  - objective: "Define context engineering in your own words"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Student explanation using non-technical analogies (office setup, workspace)"

  - objective: "Explain the difference between context and prompt engineering"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Comparison table or explanation distinguishing context (what AI knows) from prompts (what you ask)"

  - objective: "Identify whether a problem is context-related or prompt-related"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Analysis of AI output scenarios to determine if issue stems from missing context or unclear prompt"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (Context Engineering definition, Context vs Prompts distinction, AI information needs) within A1 limit of 5 ✓"

differentiation:
  extension_for_advanced: "Research how different AI tools (Claude Code vs Gemini CLI vs Cursor) handle context differently; compare context window sizes and loading strategies"
  remedial_for_struggling: "Focus on Developer A vs Developer B example as primary learning case; practice with simple 'what context is missing?' scenarios"
---

# What is Context Engineering?

## The Information Environment: Your AI's Workspace

Imagine you're starting a new job. On your first day, you walk into an empty office—no desk setup, no documentation, no colleague to ask questions. Your boss hands you a task: "Build the new customer portal." 

**What would you need before you could even start?**

You'd need:
- Access to existing systems and code
- Company coding standards and design patterns
- Information about what technologies are already in use
- Examples of similar features you can learn from
- Context about the project's goals and constraints

**This is exactly what your AI coding agent needs.** When you work with AI tools like Claude Code or Gemini CLI, they need more than just your request (the prompt). They need the entire **information environment**—the context—to give you useful, project-appropriate responses.

### Real-World Example: Two Developers, Same Task

Let's see context engineering in action.

**Developer A** (Without Context Engineering):
```bash
# Just asks directly without providing context
"Create a user authentication system"
```

**AI's Perspective:**
- What framework are you using? (Flask? FastAPI? Django?)
- What authentication method? (JWT? OAuth? Sessions?)
- What database? (PostgreSQL? MongoDB? SQLite?)
- What's your existing code structure?

**Result:** The AI generates a **generic** authentication system using random choices. It might use Flask when you're using FastAPI, implement sessions when you need JWT, and create patterns that don't match your existing code. Developer A spends hours refactoring to make it fit.

**Developer B** (With Context Engineering):
```bash
# First, AI discovers the context
"I'm building a FastAPI project and need to add user authentication.

Before implementing, discover:
- What framework am I using? (check main entry point)
- Do we have existing auth patterns? (search for authentication code)
- What database models exist? (find user-related models)
- What's our API structure? (analyze route organization)
- What libraries are available? (check dependencies)

Tell me what you found."

# AI responds with discovered patterns
# Then student makes the request
"Create a user authentication system that follows the patterns you discovered"
```

**AI's Perspective:**
- ✓ Framework: FastAPI (confirmed from files)
- ✓ Auth method: OAuth (saw the pattern)
- ✓ Database: (identified from models)
- ✓ Code structure: (understands your organization)
- ✓ Available libraries: (checked requirements.txt)

**Result:** The AI generates authentication code that **perfectly fits** Developer B's project. It follows their existing patterns, uses their chosen libraries, and integrates seamlessly. Minimal changes needed.

**The difference?** Developer B practiced **context engineering**.

---

## What is Context Engineering?

Let's break down this concept using our three-part pattern: **WHAT → WHY → HOW**.

### WHAT: The Definition

**Context Engineering** is the practice of managing the information environment that your AI coding agent operates within. It's about ensuring the AI has access to all the knowledge it needs to make informed decisions.

Think of it like this:

| Concept | Analogy | In AI-Driven Development |
|---------|---------|--------------------------|
| **Prompt** | What you say in a meeting | The specific request or question you give the AI |
| **Context** | All the documents, past emails, and project knowledge everyone brings to the meeting | The files, code patterns, documentation, and history the AI can access |
| **Context Engineering** | Organizing which documents to share before the meeting and when | Strategically managing what information the AI has access to and when |

**Simple Definition:**
> Context Engineering is the art of giving your AI collaborator the right information, at the right time, so it can help you effectively.

### WHY: Why Context Matters More Than You Think

In Chapter 10, you learned prompt engineering—how to phrase your requests clearly. But even the **perfect prompt** fails without proper context.

Here's why:

**The Karpathy Principle** (from AI researcher Andrej Karpathy):

> "The LLM is the CPU, and the context window is the RAM."

Let's translate this for non-programmers:
- Your **AI agent** = The brain doing the thinking
- Your **context** = The working memory holding relevant information
- Your **prompts** = The instructions you give
- **Context engineering** = Managing that memory efficiently

**Example: Why Context Changes Everything**

Same prompt, different contexts:

**Prompt**: "Add error handling to the login function"

**Without Context:**
```python
# AI generates generic error handling
def login(username, password):
    try:
        # Some generic code
        pass
    except Exception as e:
        print(f"Error: {e}")  # Basic, unhelpful
```

**With Context** (AI saw your existing code patterns):
```python
# AI generates error handling matching YOUR style
def login(username: str, password: str) -> LoginResult:
    """Authenticate user with email and password."""
    try:
        user = await self.user_repo.get_by_email(username)
        if not user:
            raise AuthenticationError(
                "Invalid credentials",
                error_code="AUTH_001"
            )
        # ... follows your exact patterns
    except AuthenticationError as e:
        logger.warning(f"Login failed: {e.error_code}")
        raise HTTPException(status_code=401, detail=str(e))
```

**Why the difference?** The AI with context knew:
- You use type hints (`username: str`)
- You have custom exceptions (`AuthenticationError`)
- You have an error code system (`error_code`)
- You use structured logging
- You follow specific HTTP error patterns

**This is the power of context engineering.**

---

## Context Engineering vs Prompt Engineering

You learned prompt engineering in Chapter 10. Now let's see how context engineering complements it.

### Quick Comparison

| Aspect | Prompt Engineering | Context Engineering |
|--------|-------------------|---------------------|
| **Focus** | How you phrase requests | What information is available |
| **Scope** | Single interaction | Entire session/environment |
| **Nature** | Tactical (specific requests) | Strategic (overall setup) |
| **Goal** | Get the best answer to one question | Enable AI to make informed decisions across many interactions |
| **Example** | "Implement JWT authentication with bcrypt" | Loading project structure, coding standards, existing auth patterns |

### Why You Need BOTH

**Prompt Engineering Alone:**
- ❌ Every prompt needs extensive detail
- ❌ AI can't maintain consistency between features
- ❌ You repeat project info constantly
- ❌ AI lacks awareness of your codebase

**Context Engineering Alone:**
- ❌ Even with perfect context, vague prompts yield poor results
- ❌ AI doesn't know which context is relevant now
- ❌ Lots of information, but no clear direction

**Both Together:**
- ✅ Context provides the knowledge base
- ✅ Prompts provide specific instructions
- ✅ AI produces contextually appropriate, well-directed code
- ✅ Consistency across your entire project

### The Relationship

Think of it this way:

```
Context Engineering = The foundation (what AI knows)
     ↓
Prompt Engineering = The instructions (what you ask)
     ↓
Quality AI Output = Perfect fit for your project
```

**Real Example:**

```bash
# Context Engineering (done once at session start)
"Read these files to understand my project:
- README.md (project overview)
- src/config.py (our configuration patterns)
- tests/test_helpers.py (our testing patterns)"

# Prompt Engineering (done for each specific task)
"Following our testing patterns, create tests for the new payment service"
```

**Result:** The AI generates tests that match your existing style automatically, because it has the right **context** and clear **prompts**.

---

## How Context Works in AI-Driven Development

Now let's see how to apply context engineering in your daily work.

### What Goes Into Context?

When you work with an AI coding agent, these things become part of its context:

1. **Files you explicitly load**
   - Your code files
   - Configuration files
   - Documentation

2. **Your conversation history**
   - Previous prompts you've given
   - Code the AI generated
   - Decisions you discussed

3. **Tool outputs**
   - Results from commands the AI ran
   - File listings
   - Test results

4. **System instructions**
   - Rules you've set for the AI
   - Code style preferences
   - Security requirements

### Simple Example: Setting Context

Here's what context engineering looks like in practice with Claude Code:

```bash
# Session Start: AI Discovers Context
claude "I'm building a Python web API. Before we start implementing features,
I need you to understand my project.

Discover and tell me:
1. What framework am I using? (check main files)
2. What's my project structure? (analyze directories)
3. What coding conventions do I follow? (look at existing code patterns)
4. Find example files that show:
   - Our base model pattern
   - Our service layer pattern
   - Our error handling approach

Summarize what patterns you found."

# AI explores and reports back with discovered patterns
```

**Now, every request you make will be answered with your project context in mind.**

```bash
# Your prompt (now context-aware)
claude "Create a new product service following the patterns you discovered"

# AI Response (matches YOUR style automatically)
# - Uses your base model pattern (it discovered)
# - Follows your service pattern (it found)
# - Includes type hints (it saw in examples)
# - Uses async/await (it observed)
# - Applies your error handling (it learned)
```

**You didn't have to repeat all those requirements in your prompt—the AI remembered from context!**

---

## When You DON'T Need Context Engineering

Context engineering is powerful, but **not every task requires it**. Here's when to skip it:

### Skip Context Engineering For:

**Single-file scripts or standalone functions:**
- Writing a utility function that doesn't integrate with other code
- Creating a one-off data processing script
- Quick experiments or prototypes

**Generic coding problems:**
- "Write a function to calculate Fibonacci sequence"
- "Create a regex pattern to validate emails"
- "Generate sample test data"

**Tasks with no project context:**
- Learning exercises from tutorials
- Algorithm practice problems
- Standalone code examples

### Use Context Engineering For:

**Multi-file projects:**
- Adding features to existing codebases
- Projects with established patterns and conventions
- Code that must integrate with other components

**Long-term projects:**
- Work spanning weeks or months
- Team projects where consistency matters
- Production code where quality is critical

### The Quick Test

**Ask yourself:** "Can I describe this entire task in one prompt without referencing existing code?"

- **YES** → You probably don't need context engineering
- **NO** → Context engineering will help significantly

**Example:**
- ❌ "Write a sorting algorithm" → No context needed (generic problem)
- ✅ "Add sorting to our product catalog following our existing patterns" → Context engineering helps (project-specific)

**Remember:** Context engineering has overhead. Use it when the benefits (consistency, integration, quality) justify the setup time.

---

## Pause and Reflect

Before moving on, think about your own experience:

**Question 1:** Think of a time when you asked an AI tool for coding help and got a result that didn't fit your project. What information was the AI missing?

**Question 2:** If you were teaching a new team member about your codebase, what files or documents would you show them first? (These are exactly what you should load as context for your AI!)

**Question 3:** How is context engineering different from just writing better prompts? Can you explain it to someone in one sentence?

---

## Try With AI

**Tool:** Claude Code

Now let's practice understanding context engineering with an AI conversation.

### Prompt 1: Understanding the Concept

```bash
claude "Explain the difference between prompt engineering and context engineering for software development. Give me a simple analogy that a non-programmer would understand."
```

**Expected Outcome:**
- The AI should explain prompt = the question, context = the background knowledge
- You'll likely get an analogy (like asking a question with vs without background information)
- The explanation should clarify why both are needed

**Check your understanding:** Can you now explain the difference to someone else?

---

### Prompt 2: Identifying Context Problems

```bash
claude "Here's a scenario:

I asked an AI: 'Create a login function'

The AI created code using Flask, but my project uses FastAPI. 
It used a session-based authentication, but I need JWT tokens.
It stored passwords in plain text instead of hashing them.

Is this a prompt engineering problem or a context engineering problem? 
Why?"
```

**Expected Outcome:**
- The AI should identify this as primarily a **context** problem
- The AI lacked information about your framework, auth method, and security standards
- Even a perfect prompt couldn't fix missing context

**Reflection:** What context would have prevented these issues?

---

### Prompt 3: Real-World Application

```bash
claude "I'm starting a new software project and I want to work with an AI coding assistant. What context should I provide to the AI at the very start of my first session? Give me a checklist of information the AI would need."
```

**Expected Outcome:**
- A checklist of project context (framework, language, structure, standards)
- Suggestions about documentation to share
- Ideas about existing code patterns to show

**Action:** Save this checklist—you'll use it when you actually start AI-driven development!


