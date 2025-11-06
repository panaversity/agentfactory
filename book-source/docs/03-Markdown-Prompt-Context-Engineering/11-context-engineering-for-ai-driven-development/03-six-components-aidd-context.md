---
title: "The Six Components of AIDD Context"
chapter: 11
lesson: 3
duration_minutes: 25
sidebar_position: 3
description: "Learn the six fundamental components that make context engineering work in AI-driven development"
keywords: [AIDD context components, model selection, development tools, guardrails, orchestration, memory management]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Identifying AIDD Context Components"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Remember"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can list and define the six components (Model Selection, Development Tools, Knowledge & Memory, Audio/Speech, Guardrails, Orchestration)"

  - name: "Recognizing Component Use Cases"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify which component(s) apply to specific development scenarios (e.g., 'which AI tool for this task?' = Model Selection)"

  - name: "Applying Basic Component Selection"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose appropriate tool/component for common tasks (code generation, documentation, testing) with reasoning"

learning_objectives:
  - objective: "Identify the six components that make up AIDD context"
    proficiency_level: "A2"
    bloom_level: "Remember"
    assessment_method: "Complete list of six components with brief definition of each"

  - objective: "Recognize when to use each component in development scenarios"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Scenario-based identification: given a task, student selects relevant component(s)"

  - objective: "Apply basic component selection for common tasks"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student justifies tool choice for 3-5 development tasks with component reasoning"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (6 component types + component-based thinking) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design multi-component workflow for complex project (e.g., microservices with guardrails + orchestration + memory); research enterprise AIDD platforms"
  remedial_for_struggling: "Master first 3 components only (Model Selection, Development Tools, Guardrails) before moving to remaining 3; use decision tree flowcharts"
---

# The Six Components of AIDD Context

## Introduction: The Context Engineering Toolbox

In Lessons 1-2, you learned WHAT context engineering is and HOW context windows work. Now you need to understand the **building blocks** that make context engineering effective in AI-Driven Development.

Think of these six components as **tools in your toolbox**:
- 🔧 Each tool has a specific purpose
- 🔨 You don't use all tools for every job
- 🪛 Knowing which tool to use when = expertise

**The Six Components:**
1. **Model Selection** - Which AI tool for which task?
2. **Development Tools** - How does AI access your code?
3. **Knowledge & Memory** - What does AI remember?
4. **Audio & Speech** - Can you talk to AI? (spoiler: mostly no for coding)
5. **Guardrails** - How to prevent bad code?
6. **Orchestration** - How to coordinate complex workflows?

Let's explore each one for beginners.

---

## Component 1: Model Selection for Development Tasks

### The Problem

You have two main AI coding tools:
- **Claude Code** (200K token context window)
- **Gemini CLI** (1M token context window)

**Question:** Which one should you use?

**Answer:** It depends on the task!

---

### Understanding Model Selection

**Model Selection** = Choosing the right AI tool for your specific development task.

**The Analogy:**

Imagine you need to move furniture:
- **Small item** (chair) → Use your hands
- **Medium item** (dresser) → Use a dolly
- **Large item** (piano) → Call professional movers

**Same with AI tools:**
- **Small task** (fix a bug in one function) → Claude Code
- **Medium task** (refactor a module) → Claude Code
- **Large task** (analyze entire 100-file codebase) → Gemini CLI

---

### When to Use Claude Code

**Strengths:**
- ✅ Excellent at complex reasoning
- ✅ Great for architecture design
- ✅ Strong at refactoring
- ✅ Smart about which files to read

**Best For:**
- Building new features from scratch
- Debugging complex logic
- System design and architecture
- Refactoring code

**Example Scenario:**

```bash
# Perfect for Claude Code: Complex feature with reasoning
claude "Design and implement a payment processing system with:
- Retry logic for failed payments
- Idempotency (no duplicate charges)
- Comprehensive error handling
- Rollback on failures

Walk me through the design first, then implement."
```

**Why Claude Code?** Requires deep reasoning about failure cases, retry strategies, and transaction safety.

---

### When to Use Gemini CLI

**Strengths:**
- ✅ Massive 1M token context (5x larger than Claude)
- ✅ Can load entire large codebases
- ✅ Fast processing of large amounts of code
- ✅ Good for pattern detection across many files

**Best For:**
- Analyzing large codebases (50+ files)
- Generating documentation that requires reading many files
- Finding patterns across entire project
- Large refactoring that touches many files

**Example Scenario:**

```bash
# Perfect for Gemini: Large codebase analysis
gemini chat --session analysis "Read all Python files in the src/ directory (120 files total).

Create a comprehensive report:
1. Overall architecture and structure
2. All database models and relationships
3. All API endpoints grouped by feature
4. Code patterns used consistently
5. Potential issues or inconsistencies

Take your time to read everything before responding."
```

**Why Gemini CLI?** Needs to load 120 files at once - would fill Claude's context window completely.

---

### Quick Decision Guide

| Your Task | Use This Tool | Why |
|-----------|--------------|-----|
| Build a new feature (2-5 files) | Claude Code | Complex reasoning, smart file selection |
| Fix a bug in one function | Claude Code | Deep reasoning, targeted approach |
| Design system architecture | Claude Code | Requires strategic thinking |
| Analyze 50+ file codebase | Gemini CLI | Needs massive context window |
| Generate docs from many files | Gemini CLI | Must read many files at once |
| Find patterns across project | Gemini CLI | Pattern detection across large context |

**Key Principle:** Start with Claude Code. Only switch to Gemini CLI when you need to load more than 20-30 files at once.

---

## Component 2: Development Tools as Context Sources

### The Problem

Your AI needs to access your code. How does that happen?

---

### Understanding Development Tools

**Development Tools** = Ways AI can read your code, run commands, and gather information.

**The Analogy:**

Think of AI as a detective investigating your codebase:
- 🔍 **File System** = Reading documents
- 💻 **Terminal/Bash** = Running tests and experiments
- 📚 **Git** = Checking the history
- 🔎 **Search** = Finding specific clues

---

### Tool 1: File System Access

**What it does:** AI can read files directly from your project.

**Example with Claude Code:**

```bash
# AI reads specific files
claude "Read src/models/user.py and tell me:
1. What fields does the User model have?
2. What relationships exist?
3. What validation is applied?"
```

**What happens:**
- Claude Code reads the file
- Analyzes the content
- Answers your questions

**Beginner Tip:** Be specific about which files to read. Don't say "read everything."

---

### Tool 2: Bash/Terminal Commands

**What it does:** AI can run commands in your terminal to gather information.

**Example with Claude Code:**

```bash
# AI runs command to see existing tests
claude "Run 'pytest --collect-only' to see all existing tests.

Then create tests for the new UserService following the same patterns you see."
```

**What happens:**
- Claude Code executes: `pytest --collect-only`
- Sees your test structure
- Generates tests matching your patterns

**Common Commands AI Can Run:**
- `pytest --collect-only` (see tests)
- `pip list` (see installed packages)
- `ls -la` (see directory structure)
- `git log --oneline -10` (see recent commits)

---

### Tool 3: Git as Context

**What it does:** AI can check git history to understand project evolution.

**Example with Gemini CLI:**

```bash
# AI checks recent changes
gemini chat "Run 'git log --oneline -20' to see recent commits.

Analyze:
1. What features were recently added?
2. Will my new authentication feature conflict with recent work?
3. What patterns can I see in recent commits?"
```

**Why this matters:** Prevents you from breaking recent work or duplicating effort.

---

### Tool 4: Code Search

**What it does:** AI can search for specific patterns across your codebase.

**Example with Claude Code:**

```bash
# AI searches for authentication patterns
claude "Search the codebase for all functions that contain 'authenticate' or 'auth'.

Show me the patterns used so I can implement new authentication consistently."
```

**Why this matters:** Ensures your new code matches existing patterns.

---

### Key Principle

**Don't load files manually.** Let AI use its tools to gather context as needed.

**Bad:**
```bash
# You manually paste code
claude "Here's my user.py file: [paste 200 lines]
Now help me fix..."
```

**Good:**
```bash
# AI uses its tools
claude "Read src/models/user.py and src/services/user_service.py.
Then help me fix the validation bug."
```

---

## Component 3: Knowledge & Memory in AIDD

### The Problem

AI doesn't automatically remember what you did yesterday. You need to manage memory.

---

### Understanding Knowledge & Memory

**Knowledge & Memory** = What AI knows and how it persists across sessions.

**Three Layers:**

1. **Static Knowledge** = Project documentation (README, docs)
2. **Dynamic Memory** = Current conversation history
3. **Code Pattern Memory** = Learning from your existing code

---

### Layer 1: Static Knowledge (Documentation)

**What it is:** Documentation files that don't change often.

**Example with Claude Code:**

```bash
# Load project documentation at session start
claude "Read these files to understand the project:
1. README.md - project overview
2. CONTRIBUTING.md - code standards
3. docs/architecture.md - system design

Confirm you understand the project structure before we start coding."
```

**Why this matters:** AI learns your project conventions without you explaining every time.

**Essential Docs to Load:**
- `README.md` - What the project is
- `CONTRIBUTING.md` - How to write code
- `docs/architecture.md` - How system is designed
- `docs/api-spec.md` - API contracts

---

### Layer 2: Dynamic Memory (Conversation History)

**What it is:** Everything you and AI have discussed in the current session.

**The Problem:** This memory fills up and degrades (context rot from Lesson 2).

**Solution:** Create memory checkpoints.

**Example with Claude Code:**

```bash
# After 90 minutes of work
claude "Create a checkpoint summary in SESSION_CHECKPOINT.md:

## What We Built
[Features completed]

## Key Decisions
[Architecture choices made]

## Current Status
[What's working, what's next]

## Patterns Established
[Code patterns to follow]

Keep it under 500 words."
```

**Next session:**
```bash
# Load the checkpoint
claude "Read SESSION_CHECKPOINT.md to understand what we built yesterday.

Now let's continue with the next feature."
```

---

### Layer 3: Code Pattern Memory (Learning from Examples)

**What it is:** Teaching AI your coding style through examples.

**Example with Gemini CLI:**

```bash
# Show AI your pattern
gemini chat "Read src/services/user_service.py as an example.

This shows our service layer pattern:
- Constructor with dependency injection
- Async methods
- Custom exceptions
- Type hints
- Docstrings

Now implement product_service.py following this EXACT pattern."
```

**Why this matters:** AI generates code matching your style, not generic code.

---

### Key Principle: Persist Important Context

**Don't rely on AI memory alone.** Create files:
- `SESSION_CHECKPOINT.md` - What you built
- `DECISIONS.md` - Architecture decisions
- `PATTERNS.md` - Code patterns to follow

---

## Component 4: Audio & Speech (Brief)

### Understanding Audio/Speech

**Audio & Speech** = Can you talk to AI instead of typing?

**Short Answer for Coding:** Not really useful yet.

**Why?**
- Code requires precise syntax (hard to dictate)
- File paths are easier to type than speak
- Copy-paste is essential (can't copy-paste speech)
- Code review requires visual inspection

**When it might help:**
- Verbal debugging: "Check the authentication function for JWT expiration errors"
- High-level architecture discussions: "Design a microservices architecture for e-commerce"

**For this course:** We'll stick to text-based interactions with Claude Code and Gemini CLI.

---

## Component 5: Guardrails for Code Quality

### The Problem

AI can generate code fast, but how do you ensure it's **good** code?

---

### Understanding Guardrails

**Guardrails** = Rules and constraints you give AI to ensure quality code.

**The Analogy:**

Building a treehouse:
- ❌ **Without guardrails:** "Build a treehouse" → Might be unsafe, unstable, ugly
- ✅ **With guardrails:** "Build a treehouse with safety rails, level platform, and weatherproof roof" → Safe and functional

**Same with code:**
- ❌ **Without guardrails:** "Implement authentication" → Generic, insecure code
- ✅ **With guardrails:** "Implement authentication with security requirements and code standards" → Production-ready code

---

### Guardrail Type 1: Code Style & Convention

**Example with Claude Code:**

```bash
# Explicit style guardrails
claude "Implement user authentication with these code standards:

MUST follow:
- PEP 8 style (Python standards)
- Type hints on all functions
- Google-style docstrings
- No bare except clauses (explicit error handling)
- 80%+ test coverage

Generate: auth_service.py with tests"
```

**What this prevents:**
- ❌ Inconsistent formatting
- ❌ Missing documentation
- ❌ Poor error handling
- ❌ Untested code

---

### Guardrail Type 2: Security Requirements

**Example with Gemini CLI:**

```bash
# Security guardrails
gemini chat "Implement password reset functionality with these security requirements:

MUST include:
- Cryptographically secure random tokens (secrets.token_urlsafe)
- Rate limiting: 5 requests per hour per email
- Token expiry: 1 hour
- Parameterized SQL queries (no SQL injection risk)
- No logging of sensitive data (passwords, tokens)

Show me the implementation plan first."
```

**What this prevents:**
- ❌ Weak token generation
- ❌ Brute force attacks
- ❌ SQL injection vulnerabilities
- ❌ Information leakage in logs

---

### Guardrail Type 3: Architectural Constraints

**Example with Claude Code:**

```bash
# Architectural guardrails
claude "Add payment processing feature with these constraints:

MUST NOT:
- Introduce new dependencies (use existing packages only)
- Bypass authentication middleware
- Break backward compatibility with existing API

MUST:
- Follow layered architecture (API → Service → Repository)
- Use existing error handling patterns
- Match existing logging format"
```

**What this prevents:**
- ❌ Bloated dependencies
- ❌ Security vulnerabilities
- ❌ Breaking existing features
- ❌ Inconsistent architecture

---

### Key Principle: Always Use Guardrails

**Never:**
```bash
# Vague, no guardrails
claude "Add authentication"
```

**Always:**
```bash
# Specific with guardrails
claude "Add JWT authentication following our security standards in CONTRIBUTING.md, using existing auth patterns in src/auth/, with 85%+ test coverage"
```

---

## Component 6: Orchestration & Workflow Management

### The Problem

Building real features requires **multiple steps**. How do you coordinate them?

---

### Understanding Orchestration

**Orchestration** = Breaking complex work into steps and coordinating them.

**The Analogy:**

**Building a house:**
1. Design blueprints (Architecture)
2. Pour foundation (Setup)
3. Frame structure (Core implementation)
4. Install plumbing/electrical (Integration)
5. Finish interior (Polish and testing)

**Same with features:**
1. Design architecture
2. Implement core logic
3. Add tests
4. Write documentation

---

### Orchestration Pattern 1: Single Feature Flow

**Example with Claude Code (multi-session):**

```bash
# Session 1: Design
claude "Design a comment system for our blog:
- Comments on posts
- Nested replies
- User permissions

Create architecture document, don't implement yet."
```

```bash
# Session 2: Implementation (new session, load design)
claude "Read the comment_system_design.md we created.

Implement the Comment model and CommentService following the design."
```

```bash
# Session 3: Testing (new session)
claude "Read src/services/comment_service.py.

Generate comprehensive tests covering:
- Creating comments
- Nested replies
- Permission checks
- Edge cases"
```

```bash
# Session 4: Documentation
claude "Read the comment implementation.

Generate API documentation with examples."
```

**Why separate sessions?**
- Each session has clear focus
- Context stays clean
- Easier to validate each step

---

### Orchestration Pattern 2: Multi-Day Project

**Example with Gemini CLI (named sessions):**

```bash
# Day 1: Foundation
gemini chat --session inventory "Initialize inventory management system.
Create models: Product, Stock, Location"
```

```bash
# Day 2: Core features (continue same session)
gemini chat --session inventory "Implement CRUD operations for inventory items"
```

```bash
# Day 3: Advanced features (continue same session)
gemini chat --session inventory "Add barcode scanning and low-stock alerts to the inventory system"
```

**Why named sessions?**
- Context persists across days
- Can work on multiple projects (different sessions)
- Easy to resume work

---

### Orchestration Pattern 3: Complex Feature Breakdown

**Example with Claude Code:**

```bash
# Master plan
claude "Design an e-commerce checkout system.

Break it into 5 implementable components:
1. Shopping cart service
2. Payment processing
3. Order creation
4. Inventory updates
5. Checkout orchestrator

Create implementation plan for each."
```

Then implement each component separately:

```bash
# Component 1
claude "Implement shopping cart service only (add, remove, update, get cart).
Don't implement payment or orders yet."
```

```bash
# Component 2
claude "Implement payment processing with Stripe API.
Assume cart service exists, don't reimplement it."
```

...and so on.

**Why break it down?**
- Each piece is manageable
- Can validate each component independently
- Prevents context overload

---

## The Six Components in Action

Let's see how all six components work together for a real task.

### Scenario: Add OAuth Social Login

**Task:** Add Google and GitHub login to existing authentication system.

---

### Step-by-Step Using All Components

**Step 1: Model Selection (Component 1)**

```bash
# Use Claude Code (complex reasoning required)
# NOT Gemini (don't need massive context for this)
```

**Step 2: Use Development Tools (Component 2)**

```bash
claude "Before implementing OAuth, use your tools to understand current system:

1. Read src/auth/authentication.py to see current auth approach
2. Run 'pip list' to check if we have OAuth libraries
3. Search codebase for 'authenticate' to find all auth patterns

Summarize what you learn."
```

**Step 3: Load Knowledge & Memory (Component 3)**

```bash
claude "Read these to understand our standards:
- CONTRIBUTING.md (code standards)
- docs/architecture.md (system design)
- DECISIONS.md (past architecture decisions)

Then confirm you understand before we proceed."
```

**Step 4: Skip Audio/Speech (Component 4)**

```bash
# (We're using text, not voice)
```

**Step 5: Apply Guardrails (Component 5)**

```bash
claude "Implement OAuth social login (Google + GitHub) with these guardrails:

Security Requirements:
- Validate OAuth tokens server-side
- Use state parameter to prevent CSRF
- Store only necessary user data
- Implement token refresh logic

Code Standards:
- Follow existing auth patterns in src/auth/
- Type hints on all functions
- Comprehensive error handling
- 85%+ test coverage

Architectural Constraints:
- MUST integrate with existing JWT system
- MUST NOT break existing password auth
- MUST use existing User model

Show me the implementation plan first."
```

**Step 6: Orchestrate Implementation (Component 6)**

```bash
# After review of plan
claude "Approved. Implement OAuth service following the plan."

# Next session: Testing
claude "Generate tests for OAuth implementation"

# Final session: Documentation
claude "Document OAuth setup and usage"
```

---

## Validation Checkpoint

### ✅ Can You Identify Components?

For each scenario, which component(s) are involved?

**Scenario A:** "Should I use Claude Code or Gemini CLI for this task?"
- Component: **1 - Model Selection** ✓

**Scenario B:** "AI ran 'pytest --collect-only' to see my tests"
- Component: **2 - Development Tools** ✓

**Scenario C:** "AI read my README.md to understand the project"
- Component: **3 - Knowledge & Memory** ✓

**Scenario D:** "I told AI: MUST follow PEP 8 style"
- Component: **5 - Guardrails** ✓

**Scenario E:** "I broke feature into 5 sub-tasks"
- Component: **6 - Orchestration** ✓

---

## Try With AI

**Tool:** Claude Code or Gemini CLI (your choice!)

Practice using the six components.

### Exercise 1: Model Selection Practice

```bash
# With Claude Code
claude "I have these three tasks:

Task A: Fix a bug in the login function (1 file affected)
Task B: Analyze 80 Python files to find all database queries
Task C: Design a new microservices architecture

For each task, should I use Claude Code or Gemini CLI? Explain why."
```

**Expected output:**
- Task A: Claude Code (small, focused, requires reasoning)
- Task B: Gemini CLI (large codebase, needs massive context)
- Task C: Claude Code (architecture design, requires complex reasoning)

---

### Exercise 2: Using Development Tools

```bash
# With Gemini CLI
gemini chat "I'm about to add a new feature. Use your tools to gather context:

1. Run 'git log --oneline -10' to see recent work
2. Run 'pip list | grep auth' to check authentication libraries
3. Read src/models/user.py to understand user structure

Report what you learned and how it affects my new feature."
```

**Expected output:**
- Recent commits summary
- List of available auth libraries
- User model structure
- Recommendations based on context

---

### Exercise 3: Applying Guardrails

```bash
# With Claude Code
claude "I want you to implement a file upload feature.

Add these guardrails:
- File size limit: 10MB
- Allowed types: images only (jpg, png, gif)
- Validate file contents (not just extension)
- Store securely (no path traversal vulnerabilities)
- Return clear error messages

First, explain what each guardrail prevents. Then show me the implementation plan."
```

**Expected output:**
- Explanation of each guardrail's purpose
- Implementation plan with security measures
- Validation strategy
