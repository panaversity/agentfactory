---
sidebar_position: 5
title: "Lesson 5: Agent Skills - Packaging Expertise"
description: "Understanding Agent Skills - how to encode domain expertise for AI agents"
keywords: [Agent Skills, SKILL.md, progressive disclosure, domain expertise, token efficiency]
---

# Lesson 5: Agent Skills - Packaging Expertise

## Learning Objectives

By the end of this lesson, you will be able to:

- Explain what Agent Skills are and their purpose
- Describe the SKILL.md format
- Understand the progressive disclosure mechanism
- Articulate how MCP and Skills work together

---

## The Matrix Moment

Remember the scene in The Matrix where Trinity needs to fly a helicopter? She doesn't know how. Tank loads the skill: "Pilot program, B-212 helicopter." Seconds later, she can fly.

**Agent Skills work the same way.**

Your domain expertise—the knowledge that makes you valuable—can be encoded into skills that any AI agent can load when needed. Years of experience, packaged and portable.

---

## What Are Agent Skills?

Agent Skills are organized collections of files that package procedural knowledge for agents. In simpler terms:

- **Folders** containing instructions, scripts, and resources
- **Teach agents** how to perform specific tasks
- **Reusable** across projects and agents
- **Open standard** from Anthropic (December 2025)

Think of skills as "how-to guides" written for AI agents. Instead of explaining to a human colleague, you're explaining to an agent—in a format designed for them to parse and execute.

---

## The SKILL.md Format

Every skill needs a `SKILL.md` file at its root. The format is simple:

```markdown
---
name: financial-analysis
description: Analyze financial statements and generate investment reports. Use when reviewing quarterly earnings, comparing company metrics, or preparing investor summaries.
---

# Financial Analysis Skill

## When to Use

- User asks for financial statement analysis
- Quarterly earnings data needs interpretation
- Investment comparison is requested

## How to Execute

1. Gather the relevant financial documents
2. Extract key metrics (revenue, margins, growth rates)
3. Compare against industry benchmarks
4. Generate structured report with recommendations

## Output Format

Reports should include:
- Executive summary (3 sentences max)
- Key metrics table
- Year-over-year comparison
- Risk factors
- Recommendation
```

The key elements:

1. **YAML frontmatter** (`---` block at top)
   - `name`: Identifier for the skill
   - `description`: When the agent should use this skill

2. **Markdown instructions**
   - When to use (trigger conditions)
   - How to execute (step-by-step guidance)
   - Output format (expected results)

That's it. No special syntax. No complex schemas. Just structured Markdown an agent can parse.

---

## Progressive Disclosure: The Key Innovation

Here's the problem skills solve:

**Loading everything upfront wastes tokens.**

If an agent loaded all its skills and all their supporting files at startup, you'd burn through your context window before doing any actual work.

The solution is **progressive disclosure**—loading only what's needed, when it's needed.

### Three Levels

```
Level 1: Startup (~100 tokens per skill)
├── Name: "financial-analysis"
└── Description: "Analyze financial statements..."

Level 2: When Skill Activated
└── Full SKILL.md content

Level 3: When Needed
└── Supporting files (templates, examples, scripts)
```

**Level 1: Startup**

When the agent launches, it sees only names and descriptions. For 50 skills, that's roughly 5,000 tokens—manageable.

**Level 2: Skill Activated**

When the agent decides to use a skill (based on the description matching the task), it loads the full SKILL.md. Now it has the how-to instructions.

**Level 3: On Demand**

If the skill references supporting files (templates, example outputs, helper scripts), those load only when actually needed.

This creates a powerful result: **the amount of knowledge you can bundle into a skill is effectively unbounded**, because only the relevant pieces load at any given time.

---

## Token Efficiency: The Numbers

The efficiency gains are dramatic:

| Approach | Token Cost |
|----------|------------|
| MCP servers alone | 14,000 - 80,000+ tokens |
| Skills + Scripts | ~100 tokens (at startup) |

That's a **80-98% token reduction** compared to loading raw MCP server context.

This efficiency gain is transformative. You can have dozens of skills available without bloating your context.

---

## MCP + Skills: Complementary Standards

MCP and Skills solve different problems:

| Standard | Purpose |
|----------|---------|
| **MCP** | **Connectivity** — how agents talk to tools |
| **Skills** | **Expertise** — what agents know how to do |

They're not competing. They're complementary.

**Example: Stripe Payment Processing**

- **MCP Server**: Connects to Stripe API (create charges, refund, list transactions)
- **Skill**: Knows *how* to handle payment scenarios (retry logic, error recovery, customer communication)

The MCP server gives the agent *access* to Stripe. The skill gives the agent *expertise* in using it properly.

Your most powerful Digital FTEs will combine both: connected via MCP, smart via Skills.

---

## Adoption

The Agent Skills standard launched December 18, 2025, with immediate adoption:

**Agent Support:**
- Claude Code
- VS Code AI
- GitHub Copilot
- Cursor
- goose
- Amp
- OpenCode

**Partner Skills:**
- Canva (design automation)
- Stripe (payment processing)
- Notion (workspace management)
- Zapier (workflow automation)

The ecosystem is growing. Skills you write today work across this entire landscape.

---

## Connection to Digital FTEs

This is where your expertise becomes your product.

The Agent Factory vision from Chapter 1: **YOUR domain expertise → Skills → Digital FTE → Recurring Revenue**

| Your Knowledge | Becomes | Sells As |
|---------------|---------|----------|
| Financial analysis expertise | Financial analysis skill | Financial Digital FTE |
| Legal document review | Contract review skill | Legal assistant FTE |
| Marketing copy patterns | Content creation skill | Marketing FTE |
| Customer service workflows | Support handling skill | Customer service FTE |

Skills are **how you encode what makes you valuable**. They transform tacit knowledge (in your head) into explicit knowledge (in a SKILL.md) that any agent can use.

**Monetization paths:**
- License individual skills
- Sell skill bundles
- Deploy skill-enhanced Digital FTEs
- Consult on skill development

---

## Quick Knowledge Check

Before moving on, make sure you can answer:

1. What is an Agent Skill and what does it contain?
2. What are the key elements of a SKILL.md file?
3. How does progressive disclosure save tokens?
4. What's the difference between what MCP provides and what Skills provide?

---

## Try With AI

Ask your AI assistant:

> "What would a SKILL.md look like for a code review skill?"

Evaluate the response:
- Does it include YAML frontmatter with name and description?
- Does it have clear "When to Use" and "How to Execute" sections?
- Is the description specific enough that an agent would know when to activate it?

---

## Summary

- **Agent Skills** are folders containing instructions that teach agents how to perform specific tasks
- **SKILL.md** format uses YAML frontmatter + Markdown instructions
- **Progressive disclosure** loads only what's needed: name/description at startup, full instructions when activated, supporting files on demand
- **Token efficiency**: ~100 tokens per skill at startup vs 14,000-80,000+ for raw MCP context
- **MCP + Skills** are complementary: MCP provides connectivity, Skills provide expertise
- **Digital FTE connection**: Your domain expertise → Skills → Sellable agents

---

## Next Steps

You've learned how agents connect (MCP), find project context (AGENTS.md), and load expertise (Skills). But how do users interact with agents beyond text chat? Agent interfaces are evolving. [Continue to Lesson 6: MCP-UI & Apps SDK - Agent Interfaces](./06-mcp-ui-and-apps-sdk.md).
