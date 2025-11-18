---
title: "CLAUDE.md Context Files"
sidebar_position: 3
chapter: 5
lesson: 3
duration_minutes: 9

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 2"
layer_progression: "L1 → L2 (AI Collaboration begins)"
layer_1_foundation: "Understanding what persistent context is, CLAUDE.md file structure basics"
layer_2_collaboration: "Co-creating CLAUDE.md with AI through iterative refinement, AI teaches optimal context structure, student teaches project specifics"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Creating Project Context with CLAUDE.md"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can create a complete CLAUDE.md file with 6 standard sections (project overview, file structure, standards, commands, context, getting started), collaborate with AI to refine context, verify auto-loading works, and distinguish appropriate vs inappropriate content for persistent context"

learning_objectives:
  - objective: "Understand how CLAUDE.md provides persistent project context across sessions"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of how CLAUDE.md eliminates repetitive context-providing and comparison to session-based context"
  - objective: "Create a complete CLAUDE.md file with all six standard sections"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Creation of functional CLAUDE.md with project overview, file structure, standards, commands, context, getting started"
  - objective: "Verify that CLAUDE.md auto-loads successfully in new Claude Code sessions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Starting new session and confirming Claude has project context without manual prompting"
  - objective: "Recognize what information belongs in CLAUDE.md vs what doesn't"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Evaluation of content appropriateness for persistent context (e.g., project standards YES, task details NO)"
  - objective: "Understand CLAUDE.md as persistent context that eliminates repetition"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of Three Roles pattern where AI and student iteratively refine CLAUDE.md"

# Cognitive load tracking
cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (persistent context, CLAUDE.md file format, 6 standard sections, auto-loading, context appropriateness, Three Roles collaboration) - within B1 limit of 10 ✓"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Create multi-file CLAUDE.md structure with references, experiment with `.claud ignoreContext`, set up project-specific CLAUDE.md templates for team standardization"
  remedial_for_struggling: "Use pre-built CLAUDE.md template, focus on 3 core sections only (project overview, file structure, getting started), skip advanced context management"

# Generation metadata
generated_by: "content-implementer v1.0.0 (029-chapter-5-refinement)"
source_spec: "specs/029-chapter-5-refinement/spec.md"
created: "2025-01-17"
last_modified: "2025-01-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"

# Legacy compatibility (Docusaurus)
prerequisites:
  - "Lesson 2: Claude Code installed and authenticated"
  - "Understanding of project directory structure basics"
---

# CLAUDE.md Context Files

Imagine this: You've been working with Claude Code on your Python project for weeks. Claude has learned your naming conventions, understood your project structure, and adapted to your coding style. You close the terminal for the evening.

The next morning, you open a new Claude Code session and type a question about your project. Claude responds with generic advice—treating your project like it's starting fresh. You have to re-explain your tech stack, your directory structure, your team's conventions.

**This is context friction.** And it's a productivity killer.

Every session starts with zero context. You either repeat explanations repeatedly, or Claude gives generic answers that don't match your project's reality.

**There's a better way.**

---

## What Is CLAUDE.md?

**CLAUDE.md is a simple markdown file placed in your project root that Claude Code automatically loads at the start of every session.** It contains the persistent context your AI companion needs—without you repeating it.

Think of it as a **persistent project brief** that travels with your code:

- Your project does X, Y, and Z
- You use Python 3.13 with FastAPI and PostgreSQL
- Files go in `src/`, tests in `tests/`, database migrations in `alembic/`
- You prefer type hints, Google-style docstrings, and error handling with custom exceptions
- Key commands to run: `uvicorn main:app --reload`, `pytest`, `alembic upgrade head`

When Claude Code starts a new session, it reads CLAUDE.md automatically. Claude **immediately understands your project** without you saying a word.

#### 💬 AI Colearning Prompt

> "Why is having persistent context in CLAUDE.md more efficient than repeating project details in every session?"

---

## How CLAUDE.md Auto-Loads

Here's the mechanism (you don't need to do anything—it's automatic):

1. **You create** `CLAUDE.md` in your project root (same directory as `package.json`, `pyproject.toml`, `go.mod`, or `.git`)
2. **You start** Claude Code in that directory: `claude`
3. **Claude Code detects** the CLAUDE.md file automatically
4. **Claude reads** the file and loads it into context
5. **Your session begins** with full project understanding—no setup required

Every new session repeats this process. CLAUDE.md is always loaded, always available.

**One-time setup. Automatic benefit forever.**

#### 💬 AI Colearning Prompt
> "Explain how Claude Code loads CLAUDE.md automatically at session start. What's the mechanism that makes this work without manual commands?"

#### 🎓 Expert Insight

> In AI-driven development, context is gold. CLAUDE.md is the cheapest way to give Claude continuous project awareness. Write it once; benefit every session. This is specification-first thinking applied to AI companionship.

---

## What Goes Into CLAUDE.md

CLAUDE.md typically contains 6 sections. Use this structure as your template:

1. Project Overview: What does your project do? What problem does it solve?
2. Technology Stack: Languages, frameworks, databases, key dependencies.
3. Directory Structure: Show the layout so Claude understands where code lives.
4. Coding Conventions: Style, naming, patterns your team follows.
5. Key Commands: Common commands to run the project.
6. Important Notes: Gotchas, dependencies, security considerations.

---

## How to Create Your CLAUDE.md

You could type this all manually. Or—and this is the Claude Code way—**ask Claude to generate it for you.**

Here's the process:

### Step 1: Ask Claude Code to Generate CLAUDE.md

Start Claude Code in your project directory and ask:

```
claude "Help me create a CLAUDE.md file for this project.
What are the main sections I should include, and can you generate a template
based on what you see in the codebase?"
```

Claude will analyze your actual files and propose a CLAUDE.md structure based on your real project.

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a CLAUDE.md for my [Python/Node/Go/etc] [project type] project. Include: Project Overview (2 sentences), Technology Stack (list), Directory Structure (tree), Coding Conventions (list), Key Commands (list), Important Notes (gotchas). Make it specific to what you see in the codebase."
>
> **Expected Outcome**: Claude generates a CLAUDE.md with all sections populated based on your actual project structure.

### Step 2: Review and Refine

Claude's output is a starting point. Read it carefully. Does it match your project? Are conventions accurate? If Claude guessed wrong or missed details, refine it.

### Step 3: Save the File

Save Claude's output as `CLAUDE.md` in your project root (same directory as `package.json`, `pyproject.toml`, etc.)

### Step 4: Verify Auto-Loading

Exit Claude Code (`exit` or close terminal). Open a new terminal session in the same directory:

```bash
claude
```

In the new session, ask Claude a question about your project:

```
"What's the tech stack for this project?"
```

**If Claude mentions your stack without you repeating it—CLAUDE.md loaded successfully.**

---

## Why This Matters: Context as Productivity

Here's what you've accomplished:

- ✅ **One-time creation**: 10-15 minutes to write CLAUDE.md
- ✅ **Automatic benefit**: Every session starts with full context
- ✅ **No friction**: No re-explaining project structure, conventions, or setup
- ✅ **Team alignment**: New team members read CLAUDE.md to understand the project

This is the principle of **"specify once, benefit always"**—define your project context one time, and every future session starts with full understanding.

In later lessons, you'll see how subagents (Lesson 5) and skills (Lesson 6) inherit and extend this CLAUDE.md context—making it the foundation for all Claude Code intelligence.

#### 💬 AI Colearning Prompt

> "How does having persistent context in CLAUDE.md improve the quality of Claude Code's suggestions compared to starting fresh each session?"

---

## Co-Learning: Improving CLAUDE.md Together

Let's see the **Three Roles Framework** in action—where you and Claude Code learn from each other to create a better CLAUDE.md.

### Stage 1: Your First Draft

You create an initial CLAUDE.md:

```markdown
# Project Overview
A Python web app

# Tech Stack
- Python
- FastAPI
```

It's basic, but it's a start.

### Stage 2: AI as Teacher (Claude Teaches You)

Ask Claude to review:

```
"Review my CLAUDE.md. What important sections am I missing?"
```

**Claude responds:**

> "I notice your CLAUDE.md is missing several critical sections:
>
> 1. **Directory Structure** - Where are your route files, models, tests?
> 2. **Coding Conventions** - What style guide? Type hints required?
> 3. **Key Commands** - How do I run the app? Run tests?
> 4. **Database Info** - What database? Migration strategy?
> 5. **API Documentation** - Where's the OpenAPI spec?
>
> For a FastAPI project, these sections help me give better suggestions about imports, file locations, and best practices."

**What just happened?** 🎓 **AI taught you** CLAUDE.md best practices you didn't know. You learned what makes context useful.

### Stage 3: AI as Student (You Teach Claude)

Now you provide domain-specific knowledge Claude doesn't have:

```
"Good points, but our team uses a custom auth pattern. Update CLAUDE.md to reflect:
- All auth goes through middleware/auth.py
- We use custom decorators: @require_auth, @require_admin
- No third-party auth libraries (company policy)
- JWT tokens stored in httpOnly cookies"
```

**Claude responds:**

> "Got it. I'm updating the Coding Conventions section to reflect your team's auth architecture. This helps me:
> - Suggest the correct decorator imports
> - Avoid recommending third-party auth libraries
> - Understand your security model when reviewing code
>
> Here's the updated CLAUDE.md..."

**What just happened?** 🎓 **You taught AI** your team's constraints and patterns. Claude now understands context that doesn't exist in its training data.

### Stage 4: AI as Co-Worker (Converging Together)

You and Claude iterate to refine:

```
"This is better, but the directory structure doesn't show where database migrations live.
How should we organize database-related files?"
```

**Claude responds:**

> "Two common patterns:
> 1. **Separated**: `alembic/` at root for migrations + `app/models/` for schemas
> 2. **Grouped**: `database/` folder containing both migrations and models
>
> Which matches your team's style? Or is there a third pattern you prefer?"

**You respond:**

```
"Option 1—we separate concerns. Migrations are infrastructure, models are app code. Update it."
```

**What just happened?** 🎓 **Neither of you** had the complete picture at start. Through conversation, you **converged** on the right solution—Claude offered options, you selected based on team context.

### The Result: Context Created Through Co-Learning

Your final CLAUDE.md is now:

```markdown
# Project Overview
A Python FastAPI web application for internal business workflows

# Tech Stack
- Python 3.13
- FastAPI 0.104
- PostgreSQL 15
- SQLAlchemy 2.0

# Directory Structure
```
app/
├── models/          # SQLAlchemy models
├── routes/          # API endpoints
├── middleware/      # Custom middleware (auth)
└── tests/           # Pytest test files
alembic/             # Database migrations
```

# Coding Conventions
- Type hints required on all functions
- Google-style docstrings
- Custom auth: @require_auth, @require_admin decorators
- All auth logic in middleware/auth.py
- NO third-party auth libraries (company policy)
- JWT tokens in httpOnly cookies

# Key Commands
- Run app: `uvicorn app.main:app --reload`
- Run tests: `pytest`
- Run migrations: `alembic upgrade head`
- Create migration: `alembic revision --autogenerate -m "description"`

# Important Notes
- Database migrations MUST be reviewed before merge
- All endpoints require authentication except /health
```

**This CLAUDE.md is better** because:
- ✅ Claude taught you what sections to include
- ✅ You taught Claude your team's specific patterns
- ✅ You converged together on the right organization

**This is the Three Roles Framework**—AI collaboration that makes both you and Claude smarter.

---

## Edge Cases and Troubleshooting

### CLAUDE.md Not Loading?

**Symptom**: You created CLAUDE.md, but Claude Code doesn't reference it in new sessions.

**Checklist**:
- ✅ File is named exactly `CLAUDE.md` (case-sensitive)
- ✅ File is in project root (same level as `.git`, `package.json`, etc.)
- ✅ You restarted Claude Code session (new terminal, not same session)
- ✅ File has content (not empty)

**Solution**: If all above are true, restart your terminal completely. Sometimes the session needs a fresh start.

### Unclear What Goes in CLAUDE.md?

**Simple rule**: Ask yourself: *"Does Claude need to know this to give good suggestions?"* If Claude would ask "What's your tech stack?" without CLAUDE.md, then that information belongs in CLAUDE.md.

### Concerns About File Size?

A typical CLAUDE.md is 1-3 KB. Context is cheap; clarity is expensive. A well-organized CLAUDE.md saves repeated explanations every session and improves Claude's suggestions.

---

## Try With AI

Open Claude Code in your project directory:

**Prompt 1: Generate CLAUDE.md**
```
Create a complete CLAUDE.md file with these sections:
- Project Overview (1-2 sentences)
- Technology Stack (bulleted list)
- Directory Structure (tree diagram)
- Coding Conventions (bulleted list)
- Key Commands (bulleted list for running, testing, deploying)
- Important Notes (any gotchas or critical context)

Base it on what you see in the codebase.
```

**Expected Outcome**: Claude generates a full CLAUDE.md ready to save.

**Prompt 2: Refine Based on Your Project**
```
Review the CLAUDE.md. Make these adjustments:
[Add your specific corrections, missing sections, or clarifications]
```

**Expected Outcome**: Claude refines CLAUDE.md based on your feedback.

**Prompt 3: Test Auto-Loading**

Exit Claude Code, start a new session, and ask about your project. Claude should reference your tech stack without you repeating it—proving CLAUDE.md auto-loaded.
