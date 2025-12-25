---
sidebar_position: 3
title: "Lesson 3: AGENTS.md - Project Context for Agents"
description: "Understanding AGENTS.md - how AI agents get project-specific guidance"
keywords: [AGENTS.md, project context, coding conventions, agent instructions, repository]
---

# Lesson 3: AGENTS.md - Project Context for Agents

## Learning Objectives

By the end of this lesson, you will be able to:

- Explain what AGENTS.md is and its purpose
- Identify typical content that goes in an AGENTS.md file
- Understand the hierarchy rule (nearest file takes precedence)
- Connect AGENTS.md to Digital FTE project understanding

---

## README for Agents

Every developer knows README.md—it tells humans what a project is about, how to set it up, and how to contribute.

But when an AI agent works on your codebase, it needs different information. It doesn't need a friendly introduction. It needs:

- Your coding conventions (tabs or spaces? naming style?)
- Build commands (how to compile, test, deploy)
- Testing requirements (what must pass before commits)
- Security considerations (what to avoid)
- Commit message formats

**AGENTS.md is the README for AI agents.**

It's a dedicated, predictable place to provide the context and instructions that help AI coding agents work effectively on your project.

---

## What Goes in AGENTS.md

AGENTS.md is standard Markdown. There are no required fields—you include whatever is relevant to your project.

Common sections include:

### Build and Test Commands

```markdown
## Build Commands

- `npm run build` - Production build
- `npm run dev` - Development server
- `npm test` - Run all tests
```

### Code Style Guidelines

```markdown
## Code Style

- Use TypeScript strict mode
- Prefer functional components over class components
- Maximum function length: 50 lines
```

### Testing Instructions

```markdown
## Testing

- All new features require unit tests
- Run `npm test` before committing
- Coverage must not decrease
```

### Security Considerations

```markdown
## Security

- Never hardcode API keys
- Use environment variables for secrets
- Sanitize all user inputs
```

### Commit and PR Guidelines

```markdown
## Commits

- Use conventional commit format: `type(scope): message`
- Reference issue numbers when applicable
- Keep commits focused and atomic
```

Think about what you'd tell a new developer joining your team. That's what goes in AGENTS.md—but formatted for an AI agent to parse and follow.

---

## Format: Simple Markdown

AGENTS.md uses plain Markdown. No special syntax. No required schema.

This simplicity is intentional:

- Easy to write and maintain
- Agents parse Markdown naturally
- Humans can read it too
- No tooling required

The only requirement is the filename: `AGENTS.md` (uppercase, in your repository).

---

## The Hierarchy Rule

AGENTS.md supports project-specific granularity through a simple rule:

**The nearest AGENTS.md file takes precedence.**

Consider a monorepo structure:

```
my-project/
├── AGENTS.md                    ← Root: applies to whole project
├── packages/
│   ├── frontend/
│   │   ├── AGENTS.md            ← Frontend-specific rules
│   │   └── src/
│   └── backend/
│       ├── AGENTS.md            ← Backend-specific rules
│       └── src/
```

When an agent edits a file in `packages/frontend/src/`:
1. It first finds `packages/frontend/AGENTS.md`
2. Those rules apply (the nearest file wins)

When an agent edits a file at the root:
1. It uses the root `AGENTS.md`

This hierarchy lets you:
- Define project-wide conventions at the root
- Override with specific rules in subdirectories
- Handle monorepos with different standards per package

---

## Adoption

AGENTS.md has achieved rapid adoption since OpenAI introduced it in August 2025:

- **60,000+ open source projects** now include AGENTS.md
- OpenAI's own main repository has **88 AGENTS.md files**

Supported by every major AI coding agent:
- Claude Code
- Cursor
- GitHub Copilot
- VS Code AI
- Devin
- Zed

When you add AGENTS.md to your project, all these agents immediately benefit.

---

## Connection to Digital FTEs

Your Digital FTEs will work on codebases—yours and your clients'. AGENTS.md ensures they understand local rules.

Without AGENTS.md:
- Your FTE might use the wrong coding style
- It might skip tests or use wrong build commands
- It might violate security practices

With AGENTS.md:
- Your FTE follows project conventions automatically
- It runs correct build and test commands
- It respects security guidelines

**AGENTS.md is how you teach your Digital FTE to respect local rules.**

When you deploy a Digital FTE to a client project, one of the first things it does is read that project's AGENTS.md. This ensures your FTE adapts to each environment rather than imposing its own patterns.

### Why This Matters for Monetization

A Digital FTE that ignores client conventions is worthless. A Digital FTE that reads and follows each client's AGENTS.md is a product you can sell to 100 different organizations—each with different coding standards, build systems, and security requirements—without customization.

**This adaptability transforms a demo into a product.**

Your FTE becomes valuable precisely because it respects each client's existing workflows. You're not asking clients to change how they work. You're offering an agent that slots into their existing environment seamlessly.

---

## Quick Knowledge Check

Before moving on, make sure you can answer:

1. What is AGENTS.md and how is it different from README.md?
2. What kind of content typically goes in an AGENTS.md file?
3. When there are multiple AGENTS.md files, which one applies?
4. How does AGENTS.md help Digital FTEs work on client projects?

---

## Try With AI

Ask your AI assistant:

> "What would you put in an AGENTS.md for a Python web API project using FastAPI?"

Evaluate the response:
- Does it include build/test commands?
- Does it mention coding conventions?
- Does it address security considerations?
- Is it formatted as Markdown sections an agent could parse?

---

## Summary

- **AGENTS.md** is a README for AI agents—project-specific guidance in Markdown format
- **Content** includes: build commands, coding style, testing requirements, security rules
- **Format** is plain Markdown with no required schema
- **Hierarchy rule**: nearest AGENTS.md file takes precedence (supports monorepos)
- **Adoption**: 60,000+ projects, supported by all major AI coding agents
- **Digital FTE connection**: Ensures your FTE respects project-specific conventions

---

## Next Steps

You've learned two AAIF standards: MCP (how agents connect to tools) and AGENTS.md (how agents understand projects). Now let's see what a real agent looks like that implements these standards. [Continue to Lesson 4: goose - A Reference Agent](./04-goose-reference-agent.md).
