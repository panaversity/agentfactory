---
title: Lesson 6 - Memory Files and Persistent Intelligence
sidebar_position: 6
chapter: 11
lesson: 6
learning_objectives:
  - Design memory file architecture (CLAUDE.md, architecture.md, decisions.md)
  - Implement memory persistence strategy across sessions
  - Create reusable memory-file-architecture skill
  - Recognize when and how to update persistent context
estimated_time: 90 minutes
proficiency_level: B1
generated_by: content-implementer v1.0.0
source_spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
created: 2025-01-18
version: 1.0.0
---

# Lesson 6: Memory Files and Persistent Intelligence

## The Problem You're About to Solve

It's Thursday. You start a new Claude Code session to fix a bug in your FastAPI project.

You describe the issue. Claude begins troubleshooting.

Five minutes in, Claude suggests:

> "Let me refactor this database query using the old ORM pattern you mentioned earlier..."

You pause. **Earlier?** This is a new session. Claude has never seen your project before.

You ask: "How do you know about the ORM pattern?"

> "You mentioned it two sessions ago when we were discussing the data model redesign. You decided against it and switched to SQLAlchemy's query API."

But Claude shouldn't know that. It's a new session. You started fresh.

**What's happening**: Your project evolved across 5+ sessions over two weeks. Architectural decisions were made, patterns established, anti-patterns identified. But each new session, Claude starts with zero context about these decisions.

You find yourself re-explaining:
- "We decided to use SQLAlchemy's query API, not the ORM"
- "Password hashing uses Argon2, not bcrypt"
- "Database migrations are handled by Alembic in the migrations/ folder"
- "We avoid storing sensitive data in logs"

**Re-explanation is waste.** And worse, without context about earlier decisions, Claude might suggest patterns that contradict your architecture.

This lesson teaches you to **design persistent memory files that survive across sessions**, so Claude remembers your project's decisions, patterns, and constraints without re-explanation.

This is Layer 3: **Intelligence Design — Creating Reusable Skills**.

## How Memory Files Work: Persistent Context Across Sessions

### The Core Idea

A **memory file** is a document stored in your project repository that AI tools read at session start to understand your project's context.

Instead of explaining the same architectural decisions repeatedly, you **document them once** in memory files. Every new session, Claude reads these files first, restores context, and continues work with full understanding.

### Three Types of Memory Files

Your project needs **three memory files**, each serving a specific purpose:

**1. CLAUDE.md** (Project Conventions and Patterns)
- **Purpose**: Operational guidelines AI should follow
- **Contains**: Naming conventions, file organization rules, anti-patterns to avoid
- **Updated**: When you discover new patterns or need to correct misunderstandings
- **Example sections**:
  - Database: "We use SQLAlchemy query API, not ORM. Migrations in migrations/ folder via Alembic."
  - Authentication: "Password hashing uses Argon2. Never bcrypt. Token expiry: 24 hours."
  - Logging: "Never log sensitive data (passwords, tokens, PII). Use structured JSON logging."

**2. architecture.md** (System Design)
- **Purpose**: How components relate, which files are important, design patterns
- **Contains**: Component relationships, key files, design decisions, system properties
- **Updated**: When architecture changes (new service, refactored module, new pattern introduced)
- **Example sections**:
  - Overview: "(FastAPI backend, PostgreSQL database, Redis cache, React frontend)"
  - Database Layer: "(models/ contains SQLAlchemy models, migrations/ has schema versions)"
  - Services: "(auth service handles tokens, payment service integrates Stripe, email service sends async)"
  - File Map: "(List key files: app/main.py, app/models/, app/services/)"

**3. decisions.md** (Architectural Decision Record — ADR)
- **Purpose**: Why specific decisions were made, what alternatives were considered
- **Contains**: Decision statements, rationale, alternatives rejected, consequences
- **Updated**: Each time a significant architectural choice is made
- **Format**: One ADR per decision, following standard ADR structure

### Example: Memory Files in Action

**Scenario**: You're starting a new session. Your project folder has these memory files:

```
project/
  CLAUDE.md
  architecture.md
  decisions.md
  app/
  tests/
```

**New Session Start**:

```
You: Claude, I need to fix the payment webhook handler.
It's failing on Stripe signature verification. Diagnosis?

Claude: I'll help. Let me start by reading your project context...
[Claude reads CLAUDE.md, architecture.md, decisions.md]

Claude: I understand your project now. Key context loaded:
- You use PostgreSQL + SQLAlchemy query API (not ORM)
- Payment service handles Stripe integration
- Webhook authentication uses HMAC signature verification, not JWT
- Sensitive data never logged

The issue is likely in the signature verification step.
Let me examine your webhook handler...
```

**With memory files**, Claude:
- Understands your architectural decisions without re-explanation
- Knows which patterns you've chosen and why
- Won't suggest rejected alternatives (like "switch to ORM")
- Can reason about your constraints from day one

**Without memory files**, Claude would start from zero, potentially suggesting patterns you've already rejected.

## Designing Memory File Architecture: Hands-On Exercise

### Exercise 1: Design CLAUDE.md for Your Project

Choose a real project you're working on (or use provided example).

Create a `CLAUDE.md` file with these sections:

```markdown
# CLAUDE.md — Project Conventions and Patterns

## Overview
[One paragraph describing your project, tech stack, purpose]

## Database
- Database: [PostgreSQL/MySQL/SQLite/other?]
- ORM or Query API: [Which do you use? Any constraints?]
- Migrations: [How are schema changes managed?]

## Authentication & Security
- Auth approach: [JWT/Session/OAuth/other?]
- Password hashing: [Argon2/bcrypt/other?]
- Token expiry: [Duration?]
- PII & Sensitive Data: [How handled? What never logged?]

## Naming Conventions
- Files: [snake_case? PascalCase? Kebab-case?]
- Functions/Methods: [snake_case?]
- Classes: [PascalCase?]
- Constants: [SCREAMING_SNAKE_CASE?]

## Anti-Patterns (What NOT to Do)
- [Pattern 1]: [Why to avoid]
- [Pattern 2]: [Why to avoid]

## External Services
- [Service Name]: [Credentials location, integration approach]
```

Save this file in your project root. You'll use it as a template for sessions to come.

### Exercise 2: Design architecture.md for Your Project

Create an `architecture.md` file describing your system:

```markdown
# System Architecture

## Overview
[Diagram description: what services/components exist]

## Components

### Database Layer
- **Technology**: [PostgreSQL/MySQL/other]
- **Models**: [app/models/ contains entity definitions]
- **Migrations**: [migrations/ folder, Alembic/Flyway/other]

### API Layer
- **Framework**: [FastAPI/Flask/Django/Express/other]
- **Entry Point**: [app/main.py or equivalent]
- **Routes**: [app/routes/ or equivalent structure]

### Services
- **Authentication Service**: [Handles tokens, login logic]
- **Payment Service**: [Handles Stripe integration]
- **[Other Services]**: [Descriptions]

## Key Files Map
- `app/main.py`: Application entry point
- `app/models/`: Database entity definitions
- `app/services/`: Business logic
- `migrations/`: Database schema versions
- `tests/`: Test suite

## Design Patterns Used
- [Pattern 1]: [Where used, why chosen]
- [Pattern 2]: [Where used, why chosen]

## System Properties
- **State Management**: [How is state handled?]
- **Caching**: [Is cache used? Where?]
- **Async Operations**: [Background jobs? Queue system?]
```

### Exercise 3: Create decisions.md — Architectural Decision Records

Each architectural decision gets one ADR entry:

```markdown
# Architectural Decisions

## ADR-001: Choose SQLAlchemy Query API Over ORM

**Decision**: Use SQLAlchemy's query API rather than ORM for all database operations.

**Context**:
- Early project had performance issues with ORM lazy loading
- Complex queries needed fine-grained control over SQL execution
- Team had experience with raw SQL optimization

**Rationale**:
- Query API provides explicit control over SQL generation
- Easier to optimize complex queries (joins, aggregations)
- Still maintains schema safety (not raw SQL)

**Alternatives Considered**:
1. Raw SQL queries — Rejected (loses type safety, harder to maintain)
2. Django ORM — Rejected (less explicit control, doesn't match FastAPI workflow)
3. Full SQLAlchemy ORM — Rejected (performance issues in early testing)

**Consequences**:
- Team must understand SQL query optimization
- Each database operation is explicit (not lazy-loaded)
- Migration from ORM to Query API is non-trivial

---

## ADR-002: Use Argon2 for Password Hashing

**Decision**: Password hashing uses Argon2 with recommended parameters.

**Context**:
- Initial implementation used bcrypt
- Security audit recommended Argon2 as modern best practice
- Benchmark tests showed acceptable performance

**Rationale**:
- Argon2 resistant to GPU and ASIC attacks
- Configurable memory and time cost (harder to brute-force)
- Modern standard recommended by OWASP

**Alternatives Considered**:
1. Continue with bcrypt — Rejected (older algorithm, less resistant to hardware attacks)
2. scrypt — Rejected (Argon2 is newer, stronger)

**Consequences**:
- Existing bcrypt hashes must be re-hashed on user login (migration cost)
- Slightly higher CPU cost per hash (acceptable)

---

[Continue for other major architectural decisions]
```

## Persistence Strategy: When to Read, Write, and Update

### Reading Memory Files (Session Initialization)

**Every new session**, your memory files should be read first:

```
Session Start Workflow:
1. User launches Claude Code
2. Claude reads CLAUDE.md from project root
3. Claude reads architecture.md from project root
4. Claude reads decisions.md from project root
5. Claude now has full context about project
6. User asks their question, Claude responds with architectural understanding
```

**Implementation**: Include memory files in your initial Claude Code prompt:

```
I'm working on [project name]. Here's my project context:

[Paste CLAUDE.md]

[Paste architecture.md]

[Paste decisions.md]

Now, [your actual question]
```

Or configure Claude Code to auto-load these files at startup (if your CLI supports it).

### Updating Memory Files (Session Maintenance)

**Update triggers** — When to modify memory files during or after sessions:

| Trigger | Action | File |
|---------|--------|------|
| Discovered new pattern | Document it | CLAUDE.md |
| Architectural change | Record decision | decisions.md |
| System design changes | Update structure | architecture.md |
| New anti-pattern learned | Document what NOT to do | CLAUDE.md |
| Component added | Update component list | architecture.md |

**Example Update Scenario**:

During development, you discover:
- "We should always validate webhook signatures BEFORE processing events"

**Update**: Add to CLAUDE.md:

```markdown
## Webhooks & External Services
- Always validate signatures BEFORE processing webhook events
- Stripe: Verify HMAC signature using webhook secret (never trust header)
- Delay processing until signature verified (no race conditions)
```

### Conflict Resolution: Old vs. New Understanding

**Problem**: Your memory files represent past decisions. What if your understanding changes?

**Resolution Strategy**:

1. **Non-Breaking Changes** (clarifications, refinements):
   - Update memory file directly
   - Version bump not needed
   - Example: Clarifying how pagination works in database queries

2. **Breaking Changes** (reversals of decisions):
   - Add new ADR explaining the change
   - Keep old ADR as historical record
   - Mark old ADR as "superseded"
   - Example: "We're switching from bcrypt to Argon2 for password hashing"

```markdown
## ADR-002: Use Argon2 for Password Hashing
**Status**: SUPERSEDED by ADR-004

[Original decision...]

---

## ADR-004: Migrate to Scrypt for Better Performance (SUPERSEDES ADR-002)

**Decision**: Switch from Argon2 to Scrypt for password hashing.

**Context**:
- Argon2 showed higher CPU cost in production (25% slower logins)
- Scrypt provides similar security with better performance

**Rationale**:
- 15% improvement in login performance
- Still resistant to GPU attacks
- Easier to operate (lower CPU load)

**Supersedes**: ADR-002 (Argon2 decision)

[Consequences, etc...]
```

## Creating the Reusable Skill: memory-file-architecture

You've designed memory files for your project. Now, let's create a **reusable skill** that codifies this pattern for future projects.

### What Goes Into the Skill

The **memory-file-architecture** skill captures:

1. **Templates** (CLAUDE.md, architecture.md, decisions.md structure)
2. **Persistence Strategy** (when to read, write, update)
3. **Update Triggers** (when to modify which file)
4. **Conflict Resolution** (how to handle decision reversals)

### Using the Skill in New Projects

**For your next project**, instead of designing memory files from scratch:

1. Reference the memory-file-architecture skill
2. Copy templates from the skill
3. Customize templates for your project
4. Follow persistence strategy from the skill
5. Apply update triggers from the skill

This is **reusable intelligence** — the pattern you discovered in THIS project applies to ANY future project requiring multi-session continuity.

## Testing Memory Files: Do They Actually Work?

### Validation Exercise

Test your memory files to confirm they work:

**Setup**: Create two sessions (Session A and Session B, separated by 1+ day)

**Session A** (Build context):
- Start with fresh Claude Code session
- Load your project
- Work for 20+ minutes
- Make at least 2 architectural decisions (document in decisions.md)
- Discover 2 patterns (document in CLAUDE.md)

**Between Sessions**: Update memory files with Session A learnings

**Session B** (Test persistence):
- Start new Claude Code session
- Load memory files (CLAUDE.md, architecture.md, decisions.md)
- Ask Claude: "What architectural decisions have I made?"
- Ask Claude: "What patterns should I follow in this project?"
- Observe: Does Claude cite your decisions? Remember your patterns?

**Expected Outcome**: Claude references memory file content WITHOUT you re-explaining.

## Try With AI

**Setup**: Open Claude Code. Create a new project directory with sample code (or use existing project).

**Prompt Set**:

**Prompt 1: Design Memory Files**
```
I'm starting a new AI-assisted development project. Help me create
three memory files (CLAUDE.md, architecture.md, decisions.md) for this codebase:

[Paste your project structure and brief description]

The memory files should capture:
- Project conventions I should follow
- System architecture (components and relationships)
- Major architectural decisions I've made

Generate templates for each file, ready for me to customize.
```

**Prompt 2: Initialize Session with Memory Files**
```
I'm loading my project memory. Here's my context:

[Paste CLAUDE.md]
[Paste architecture.md]
[Paste decisions.md]

Now, I need to [your actual task].
What patterns from my memory files should influence this task?
```

**Prompt 3: Update Decisions**
```
I just made a new architectural decision:
- [Describe the decision and why]

Write a new ADR entry (decisions.md format) for this decision.
Include context, rationale, alternatives considered, and consequences.
```

**Expected Outcomes**:
- Prompt 1: You receive ready-to-customize memory file templates
- Prompt 2: Claude references your project patterns and decisions in its response
- Prompt 3: Claude generates properly formatted ADR following industry standard

**Safety Note**: Memory files should never contain secrets (API keys, passwords, credentials). Store secrets in `.env` files or secret management systems, never in memory files. Reference them by name in memory files ("Uses STRIPE_API_KEY from .env") but never include the actual values.

