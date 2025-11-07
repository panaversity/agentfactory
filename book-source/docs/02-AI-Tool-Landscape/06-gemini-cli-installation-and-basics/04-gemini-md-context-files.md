---
sidebar_position: 4
title: "GEMINI.md Context Files: Automatic Project Understanding"
duration: "35 min"
---

# GEMINI.md Context Files: Automatic Project Understanding

## The Problem: Re-Explaining Your Project Every Session

Imagine this: You spend 2 hours helping your AI understand your project. You explain the architecture, show examples, define your conventions. Then you close Gemini CLI.

Next day, you reopen it and... you start from scratch. You're re-explaining the same things. The AI doesn't remember what you taught it yesterday.

**This is wasteful.**

Gemini CLI solves this with **GEMINI.md**—a special markdown file that tells Gemini about your project automatically. Every time you work in that directory, Gemini reads GEMINI.md and understands your context instantly. No re-explaining. No wasted time.

This lesson teaches you how to create and use GEMINI.md files with the `/init` command and `/memory` family of commands.

---

## Understanding GEMINI.md: What It Is

**GEMINI.md** is a simple markdown file that:

1. **Lives in your project** (usually at the root: `./GEMINI.md` or `~/.gemini/GEMINI.md`)
2. **Gets loaded automatically** when you work in that directory
3. **Tells Gemini about your project** — architecture, conventions, decisions, context
4. **Persists across sessions** — write it once, use it forever

Think of it like a briefing document for your AI colleague. Before they start working on your project, they read GEMINI.md and understand:
- What the project is
- How it's organized
- What conventions you follow
- What decisions you've made
- What they should and shouldn't do

---

## Two Types of GEMINI.md Files

### Global GEMINI.md (`~/.gemini/GEMINI.md`)

**What it is**: Your personal context that applies to ALL projects

**When to create**: Settings up Gemini CLI for the first time

**What to put here**:
- Your coding style preferences (Python, TypeScript, etc.)
- Your tools and frameworks
- Your workflow preferences
- Your company/team standards (if working for one employer)

**Example**:
```markdown
# My Development Context

## Preferred Languages & Frameworks
- Python 3.11+ (backend, data analysis)
- TypeScript (frontend, Node.js)
- pytest for testing Python
- Jest for testing TypeScript

## My Coding Style
- Type hints required (TypeScript strict mode, Python type hints)
- Functional programming when possible
- Clear, descriptive variable names (no abbreviations)
- Comprehensive error handling

## Tools I Use
- GitHub for version control
- VS Code as editor
- Docker for containerization
- PostgreSQL for databases

## Common Security Rules
- Never commit secrets (.env files)
- API keys in environment variables only
- Rotate tokens every 90 days
```

**How to create**: Use `/init` command:

```bash
You: /init
Gemini: I'll help you set up your GEMINI.md. Answer a few questions:
        What's your preferred programming language?
        What frameworks do you use?
        (etc.)

Gemini: ✓ Created ~/.gemini/GEMINI.md with your preferences
```

---

### Project GEMINI.md (`./GEMINI.md`)

**What it is**: Context specific to THIS ONE PROJECT

**When to create**: Starting a new project or adding Gemini CLI to existing project

**What to put here**:
- Project summary (what are you building?)
- Architecture overview
- Project-specific decisions
- Team/project conventions
- Current status and what you're working on

**Example**:
```markdown
# Authentication Service - GEMINI.md

## Project Summary
Building a JWT-based authentication microservice for SaaS platform.
- Language: TypeScript
- Framework: Express.js
- Database: PostgreSQL
- Context Window: 1,000,000 tokens (Gemini CLI)

## Architecture
- `/src/routes` — API endpoints
- `/src/middleware` — Auth, logging, error handling
- `/src/db` — PostgreSQL queries and migrations
- `/src/tests` — Jest test suite
- `/src/utils` — Helpers (jwt, password hashing, etc.)

## Key Decisions
1. Using JWT for stateless auth (not sessions)
2. Storing refresh tokens in Redis (not database)
3. Password hashing with bcrypt (12 salt rounds)
4. Rate limiting on login endpoint (5 attempts/15 min)

## Current Status
- Core auth endpoints: ✓ Complete
- Email verification flow: 🔄 In progress
- Password reset: ⏳ TODO

## Code Conventions
- Naming: camelCase for functions, PascalCase for classes
- Error messages: Always descriptive (for users and devs)
- Validation: Always validate at route handler before business logic
- Tests: Unit tests (100% for utils), integration tests (all endpoints)

## Important Security Notes
- Passwords never logged or exposed
- Token expiry: access=15min, refresh=7days
- Rate limit: 5 login attempts per 15 minutes per IP
```

---

## How Gemini CLI Uses GEMINI.md

When you work in a directory with Gemini CLI:

1. **Gemini looks for GEMINI.md** in the current directory
2. **Also loads global GEMINI.md** from `~/.gemini/`
3. **Combines them** (project context overrides global)
4. **Uses `/memory show`** to display what's loaded
5. **Uses `/memory refresh`** to reload if you update GEMINI.md

### Example: Different Contexts for Different Projects

```bash
$ cd ~/projects/auth-service && gemini

Gemini CLI: Loading context files...
  - ~/.gemini/GEMINI.md (your global preferences)
  - ./GEMINI.md (project context)

You: /memory show
Gemini CLI:
  Loaded Context:
  - Preferred languages: Python, TypeScript
  - My style: Strict types, functional programming
  - Project: Authentication Service
  - Architecture: Express.js microservice
  - Key decision: JWT stateless auth
```

Later, when you switch projects:

```bash
$ cd ~/projects/frontend-app && gemini

Gemini CLI: Loading context files...
  - ~/.gemini/GEMINI.md (your global preferences)
  - ./GEMINI.md (project context - DIFFERENT from auth-service)

You: /memory show
Gemini CLI:
  Loaded Context:
  - Preferred languages: Python, TypeScript
  - My style: Strict types, functional programming
  - Project: Frontend App
  - Architecture: Next.js 14 with TypeScript
  - Key decision: Server components for data fetching
```

**Gemini automatically knows you switched projects.** No re-explaining needed.

---

## Creating Your First GEMINI.md

### Option 1: Use `/init` (Guided)

```bash
You: /init
Gemini: I'll help you set up GEMINI.md. Let's start:

        What's the name of this project?
        What's the primary purpose?
        What tech stack are you using?
        (etc.)

Gemini: ✓ Created ./GEMINI.md
```

This is the easiest way—Gemini generates a template based on your answers.

### Option 2: Manual Creation

Create `./GEMINI.md` in your project root:

```bash
touch ./GEMINI.md
```

Then add content using this structure:

```markdown
# [Project Name]

## Summary
[One paragraph: What are you building?]

## Tech Stack
- Language: TypeScript
- Framework: Express.js
- Database: PostgreSQL
- Other tools: [testing, auth, etc.]

## Project Structure
[Describe main folders]

## Key Decisions
[Important architectural choices]

## Current Status
[What's done, in progress, or planned]

## Code Conventions
[Your style, naming, patterns]

## Important Notes
[Anything Gemini should know]
```

---

## Commands for Working with GEMINI.md

### `/memory show` — See Loaded Context

Shows all context from GEMINI.md files that are currently loaded.

```bash
You: /memory show
Gemini CLI:
  ~/.gemini/GEMINI.md (loaded):
  - Python 3.11+
  - TypeScript
  - GitHub

  ./GEMINI.md (loaded):
  - Project: Auth Service
  - Architecture: Express.js microservice
```

### `/memory refresh` — Reload Files

If you update GEMINI.md, reload it without restarting Gemini CLI.

```bash
You: [Edit ./GEMINI.md to add new decision]
You: /memory refresh
Gemini: ✓ Reloaded context from GEMINI.md files
```

### `/memory list` — Show File Paths

List which GEMINI.md files are being used.

```bash
You: /memory list
Gemini CLI:
  ~/.gemini/GEMINI.md
  ./GEMINI.md
```

### `/memory add` — Add Temporary Context

Add context for just this session (doesn't save to file).

```bash
You: /memory add we switched to strict TypeScript mode today
Gemini: ✓ Added to current session context
```

---

## Real Example: A Production GEMINI.md

Here's what a real, production-grade GEMINI.md looks like:

```markdown
# User Profile Microservice

## Summary
Handles user profiles in our SaaS platform. CRUD operations, preferences, notifications.

## Tech Stack
- Language: TypeScript 5.2
- Framework: Express.js 4.18
- Database: PostgreSQL 15
- Cache: Redis (for preferences)
- Testing: Jest + Supertest
- Deployment: Docker + Kubernetes

## Project Structure
```
src/
├── routes/          # API endpoints
├── services/        # Business logic
├── db/             # Database queries & migrations
├── middleware/     # Auth, logging, error handling
├── tests/          # Jest test suite
└── utils/          # Helpers
```

## Architecture Decisions

**Microservice Not Monolith**: Profile service is separate from Auth
- Reason: Independent scaling, isolated deployments
- Trade-off: Requires service-to-service authentication

**PostgreSQL Not MongoDB**: ACID guarantees essential for user data
- Reason: Data consistency is critical (users can't lose profile data)
- Trade-off: Less flexible schema evolution

**Redis for Preferences**: Cache for frequently accessed settings
- Reason: Preferences checked on every request
- Trade-off: Must handle cache invalidation on updates

## Current Status
### ✓ Completed
- Profile CRUD (create, read, update, delete)
- Authentication via JWT from Auth service
- Permission checks (users edit only own profiles)
- Unit tests (85% coverage)
- Integration tests (all endpoints)

### 🔄 In Progress
- Profile picture upload
- S3 integration for images

### ⏳ TODO
- Email notifications on profile changes
- Privacy levels (public/private)
- Activity audit log

## Code Conventions
- Naming: camelCase (functions), PascalCase (classes)
- Validation: Always at route handler, before service logic
- Error messages: Always meaningful (for both users and developers)
- Transactions: Middleware handles multi-query operations
- Testing approach: Unit tests (100% for utils), integration tests (endpoints)

## Database Notes
🔒 **PII Stored**: Names, emails, phone numbers (encrypted at rest)
🔒 **Never Store**: Passwords (done by Auth service), payment info
**Indexes**: email, user_id, created_at (for common queries)
**Migrations**: Always backwards compatible

## Important
- User table lives in Auth service, NOT this one
- We replicate user_id in profiles for quick lookups
- Password changes in Auth service invalidate our Redis cache
- Always test with realistic data, not tiny test datasets
```

---

## Best Practices

### ✅ DO

- ✅ Keep it focused on what Gemini needs (not a full design doc)
- ✅ Be specific with examples and code snippets
- ✅ Update it as your project evolves
- ✅ Document decisions AND their trade-offs
- ✅ Add warnings for common pitfalls
- ✅ Use clear structure (headers, lists, code blocks)

### ❌ DON'T

- ❌ Copy entire README verbatim (GEMINI.md is for AI, README is for humans)
- ❌ Write it once and never update (context changes as project evolves)
- ❌ Include secrets or credentials (never!)
- ❌ Make it too long (keep to ~500-1000 words for efficiency)
- ❌ Use unexplained jargon

---

## Updating GEMINI.md As You Work

**GEMINI.md is a living document.** Update it when:

- ✏️ You complete a major feature → Update "Current Status"
- ✏️ You make an important decision → Add to "Key Decisions"
- ✏️ Project structure changes → Update "Project Structure"
- ✏️ You discover a new pattern → Add to "Code Conventions"
- ✏️ You hit a problem → Add to "Important Notes"

**Time investment**: 2 minutes to update GEMINI.md
**Time saved**: 10+ minutes not re-explaining to Gemini next session

---

## Global vs. Project Context

### Use Global GEMINI.md (`~/.gemini/GEMINI.md`) For:
- Personal coding preferences
- Tools you always use
- Standards from your company
- Security policies you follow

### Use Project GEMINI.md (`./GEMINI.md`) For:
- This project's purpose and architecture
- Team decisions for this project
- Project-specific conventions
- What's currently being worked on

### They Work Together:
```bash
$ cd project-x && gemini
Loads: ~/.gemini/GEMINI.md + ./GEMINI.md
Result: "You prefer TypeScript" + "This project is Node.js"
```

---

## Practical Exercises

### Exercise 1: Generate Your First GEMINI.md

Use `/init` to generate one:

```bash
$ cd ~/my-project
$ gemini
You: /init
Gemini: [Guides you through setup]
You: /memory show
(Verify it was created)
```

### Exercise 2: Update Your Project's GEMINI.md

Edit `./GEMINI.md` to add a decision you made today:

```bash
You: [Edit ./GEMINI.md]
You: /memory refresh
Gemini: ✓ Reloaded context
```

### Exercise 3: Check Context Across Projects

Switch to a different project:

```bash
$ cd ../other-project && gemini
You: /memory show
(See how context changed based on ./GEMINI.md in this project)
```

---

## Key Takeaways

- **GEMINI.md** is a briefing document for your AI collaborator
- **Global** (`~/.gemini/GEMINI.md`): Your personal preferences
- **Project** (`./GEMINI.md`): This project's context and decisions
- **Use `/init`** to generate initial GEMINI.md
- **Use `/memory show`** to see what's currently loaded
- **Use `/memory refresh`** if you update GEMINI.md
- **Update regularly** as your project evolves
- **Keep it focused** on what Gemini needs to know

Next lesson: You'll learn how to **save and resume conversations** using `/chat save` and `/chat resume` to persist the work you do across multiple sessions.

