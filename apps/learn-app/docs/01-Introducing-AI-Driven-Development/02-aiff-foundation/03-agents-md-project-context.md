---
sidebar_position: 3
title: "AGENTS.md - Project Context for Agents"
description: "Understanding AGENTS.md - how AI agents get project-specific guidance"
keywords: [AGENTS.md, project context, coding conventions, agent instructions, repository]
chapter: 2
lesson: 3
duration_minutes: 12

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Agent Context Requirements"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why AI agents need different context than humans (README vs AGENTS.md)"

  - name: "Designing AGENTS.md Content"
    proficiency_level: "A2"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create an AGENTS.md with build commands, code style, and security guidelines"

  - name: "Applying Hierarchy Rules"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can predict which AGENTS.md applies in a monorepo scenario"

learning_objectives:
  - objective: "Explain why AGENTS.md exists separately from README.md"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Articulation of different audience needs (humans vs AI agents)"

  - objective: "Identify what belongs in an AGENTS.md file"
    proficiency_level: "A2"
    bloom_level: "Remember"
    assessment_method: "Recognition of build commands, style guides, security rules"

  - objective: "Apply the hierarchy rule to determine which AGENTS.md governs a specific file"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Correct identification in monorepo scenarios"

cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (agent vs human context, content categories, hierarchy rule, adoption ecosystem) within A1-A2 limit (5-7 concepts) ✓"

differentiation:
  extension_for_advanced: "Research OpenAI's 88 AGENTS.md files in their repository; analyze patterns across large organizations"
  remedial_for_struggling: "Focus on one section type (build commands); understand deeply before adding style/security"
---

# AGENTS.md - Project Context for Agents

You're deploying your Digital SDR to a new client. Their codebase uses:
- TypeScript with strict mode
- Jest for testing (not Mocha)
- Conventional commits (not freeform messages)
- No console.log in production code

Your Digital FTE doesn't know any of this. It starts generating JavaScript instead of TypeScript. It writes `console.log` everywhere. It creates commits with messages like "fixed stuff."

The client's code review rejects everything.

**AGENTS.md prevents this.**

AGENTS.md is a standard Markdown file that teaches AI agents local rules. When your Digital FTE enters a new project, it reads the AGENTS.md and immediately understands how to behave in that environment—without custom configuration.

---

## Why AGENTS.md Exists: Humans ≠ Agents

Every developer knows README.md. It tells humans:
- What the project does
- How to install it
- How to contribute
- License and credits

But AI agents need different information:

| Humans Need | Agents Need |
|-------------|-------------|
| Project motivation and goals | Build and test commands |
| Getting started tutorial | Code style rules |
| Contribution guidelines | Security constraints |
| Screenshots and demos | File organization patterns |

README.md answers "What is this project?" AGENTS.md answers "How should I behave in this project?"

**AGENTS.md is README for AI agents.**

Without it, your Digital FTE might:
- Use the wrong coding style (tabs vs spaces, naming conventions)
- Skip required tests (doesn't know which command to run)
- Violate security practices (hardcodes API keys)
- Produce work that fails code review (wrong commit format)

With AGENTS.md, your Digital FTE adapts to each client's environment automatically. That's the difference between a demo and a product.

---

## What Goes in AGENTS.md

AGENTS.md is standard Markdown. No special syntax. Include whatever is relevant for an AI agent working in your codebase:

### Build and Test Commands

Tell the agent how to verify its work:

```markdown
## Build Commands

- `pnpm install` - Install dependencies
- `pnpm run build` - Production build
- `pnpm run dev` - Development server with hot reload
- `pnpm test` - Run all tests
- `pnpm test:watch` - Run tests in watch mode
- `pnpm run lint` - Check for lint errors
- `pnpm run typecheck` - Verify TypeScript types
```

**Why this matters:** An agent that can't verify its work produces unreliable output. Build commands let it check before committing.

### Code Style Guidelines

Tell the agent your conventions:

```markdown
## Code Style

- Use TypeScript strict mode for all new code
- Prefer functional components over class components
- Maximum function length: 50 lines
- Use named exports, not default exports
- File names: kebab-case (e.g., `user-profile.tsx`)
- Variable names: camelCase
- Constants: SCREAMING_SNAKE_CASE
```

**Why this matters:** Style violations create noise in code reviews. Agents following your style integrate seamlessly.

### Security Considerations

Tell the agent what to avoid:

```markdown
## Security

- Never hardcode API keys, tokens, or secrets
- Use environment variables for all credentials
- Sanitize all user inputs before database queries
- No `eval()` or `Function()` constructors
- HTTPS only for external API calls
- Log errors, never log sensitive data
```

**Why this matters:** Security mistakes are expensive. Explicit rules prevent common vulnerabilities.

### Architecture and Patterns

Tell the agent your design decisions:

```markdown
## Architecture

- All API routes go in `/src/api/`
- Shared components in `/src/components/shared/`
- Feature-specific components in `/src/features/[feature]/`
- Database queries only through `/src/db/` layer
- No direct database access from API routes
```

**Why this matters:** Consistent architecture makes codebases maintainable. Agents that follow patterns produce code that fits.

### Commit and PR Guidelines

Tell the agent your workflow:

```markdown
## Git Workflow

- Use conventional commits: `feat:`, `fix:`, `docs:`, `refactor:`
- Branch names: `feature/description` or `fix/description`
- One logical change per commit
- PR titles should summarize the change, not describe activity
- Include tests for new functionality
```

---

## The Hierarchy Rule

AGENTS.md supports project-specific granularity through a simple rule:

**The nearest AGENTS.md file takes precedence.**

Consider a monorepo:

```
my-company/
├── AGENTS.md                    ← Root: company-wide rules
├── packages/
│   ├── frontend/
│   │   ├── AGENTS.md            ← Frontend-specific rules
│   │   └── src/
│   │       └── components/
│   │           └── Button.tsx
│   └── backend/
│       ├── AGENTS.md            ← Backend-specific rules
│       └── src/
│           └── routes/
│               └── users.ts
```

**Scenario 1:** Agent edits `packages/frontend/src/components/Button.tsx`
- Finds `packages/frontend/AGENTS.md`
- Those rules apply (nearest file wins)
- Might say: "Use React, prefer hooks, styled-components for CSS"

**Scenario 2:** Agent edits `packages/backend/src/routes/users.ts`
- Finds `packages/backend/AGENTS.md`
- Those rules apply
- Might say: "Use Express, async/await, Prisma for database"

**Scenario 3:** Agent edits a file at root level
- No subdirectory AGENTS.md found
- Falls back to root `AGENTS.md`
- Company-wide rules apply

This hierarchy lets you:
- Define project-wide conventions at the root
- Override with specific rules in subdirectories
- Handle monorepos with different standards per package

---

## Adoption: 60,000+ Projects

AGENTS.md achieved rapid adoption since OpenAI introduced it in August 2025:

**By the numbers:**
- **60,000+** open source projects now include AGENTS.md
- **88** AGENTS.md files in OpenAI's own repository alone
- Every major AI coding agent supports it

**Supported agents:**
- Amp
- Claude Code
- Codex CLI
- Cursor
- Devin
- Factory
- Gemini CLI
- GitHub Copilot
- goose
- Jules
- VS Code

When you add AGENTS.md to your project, all these agents immediately benefit. When you deploy your Digital FTE to a client's project that has AGENTS.md, it immediately adapts.

---

## The Portability Advantage

Remember the Agent Factory vision: selling the same Digital FTE to many clients.

Without AGENTS.md:

| Client | Environment | Your Work |
|--------|-------------|-----------|
| Client A | Python, pytest, Black | Custom configuration |
| Client B | TypeScript, Jest, Prettier | Different custom configuration |
| Client C | Go, standard library, gofmt | Yet another configuration |

With AGENTS.md:

| Client | Environment | Your Work |
|--------|-------------|-----------|
| Client A | Python, pytest, Black | Their AGENTS.md handles it |
| Client B | TypeScript, Jest, Prettier | Their AGENTS.md handles it |
| Client C | Go, standard library, gofmt | Their AGENTS.md handles it |

**Same Digital FTE. Different environments. Zero customization from your side.**

You're not asking clients to change how they work. You're offering an agent that slots into their existing environment seamlessly. That's why AGENTS.md is foundational to the Agent Factory vision.

---

## Creating Effective AGENTS.md

Think about what you'd tell a new developer joining your team. Now filter for what an AI agent actually needs to know.

### Include:
- Commands to run (build, test, lint)
- Code style rules (formatting, naming)
- Security constraints (what to never do)
- Architecture patterns (where code goes)
- Workflow rules (commits, PRs)

### Exclude:
- Project motivation or history (agents don't need context)
- Installation tutorials (agents have different setup)
- Screenshots or demos (agents can't see them)
- Long explanations (agents need rules, not rationales)

### Keep It Actionable:
❌ "We value clean code and good practices."
✅ "Maximum function length: 50 lines. Use descriptive names."

❌ "Security is important to us."
✅ "Never hardcode secrets. Use `process.env` for all credentials."

---

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini, or similar) to practice:

### Prompt 1: Create an AGENTS.md

```
I'm setting up a new TypeScript project with these characteristics:
- Next.js 14 with App Router
- Tailwind CSS for styling
- Prisma for database
- Jest for testing
- GitHub Actions for CI

Help me create an AGENTS.md that covers:
1. Build and test commands
2. Code style guidelines
3. Security considerations
4. File organization patterns

Make it specific enough that an AI agent could follow it precisely.
```

**What you're learning:** Specification writing. Good AGENTS.md files are precise and actionable—skills that transfer to writing specs for Digital FTEs.

### Prompt 2: Test the Hierarchy Rule

```
I have a monorepo with this structure:

```
company/
├── AGENTS.md (says: "Use tabs for indentation")
├── apps/
│   ├── web/
│   │   ├── AGENTS.md (says: "Use Prettier defaults")
│   │   └── src/
│   └── api/
│       └── src/
└── packages/
    └── shared/
        └── src/
```

For each file location below, tell me:
1. Which AGENTS.md applies?
2. What indentation rule would the agent use?

Locations:
- apps/web/src/components/Button.tsx
- apps/api/src/routes/users.ts
- packages/shared/src/utils.ts
```

**What you're learning:** Rule application in hierarchies. Understanding precedence helps you design AGENTS.md structures for complex projects.

### Prompt 3: Diagnose a Problem

```
My Digital SDR is deployed to a client's project, but it keeps making mistakes:
- Generates JavaScript instead of TypeScript
- Uses `console.log` for debugging
- Creates commits with messages like "updates"
- Puts new components in the wrong directory

The client has a README.md but no AGENTS.md.

Help me:
1. Draft an AGENTS.md that would prevent these specific issues
2. Explain to the client why AGENTS.md matters for AI tools
```

**What you're learning:** Diagnosis and remediation. Translating observed problems into preventive rules is key to maintaining Digital FTE quality.
