---
sidebar_position: 5
title: "Agent Skills - Packaging Expertise"
description: "Understanding Agent Skills - how to encode domain expertise as portable, monetizable assets"
keywords: [Agent Skills, SKILL.md, progressive disclosure, domain expertise, token efficiency, agentskills.io]
chapter: 2
lesson: 5
duration_minutes: 15

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Progressive Disclosure"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain the three-level loading hierarchy and its token efficiency benefits"

  - name: "Distinguishing MCP from Skills"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can correctly classify capabilities as MCP (connectivity) vs Skills (expertise)"

  - name: "Designing SKILL.md Content"
    proficiency_level: "A2"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create a valid SKILL.md with proper frontmatter, activation conditions, and execution steps"

learning_objectives:
  - objective: "Explain how progressive disclosure achieves 80-98% token reduction"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Articulation of three-level loading hierarchy with token estimates"

  - objective: "Create a valid SKILL.md with proper structure for your domain expertise"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Production of SKILL.md with frontmatter, when-to-use, execution steps"

  - objective: "Map skill monetization paths to your Agent Factory business model"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Connection of four revenue streams to specific domain expertise"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (progressive disclosure, SKILL.md structure, MCP vs Skills, adoption ecosystem, monetization paths) within A1-A2 limit (5-7 concepts) ✓"

differentiation:
  extension_for_advanced: "Clone the anthropics/skills repository; study progressive disclosure implementation in existing skills; analyze partner skills from Canva, Stripe, Notion"
  remedial_for_struggling: "Focus on SKILL.md structure only; understand frontmatter and activation conditions before progressive disclosure"
---

# Agent Skills - Packaging Expertise

You've spent years mastering financial analysis, or legal document review, or sales qualification. That expertise lives in your head—tacit knowledge that makes you valuable but can't scale.

Every time a client asks you to do what you're expert at, you trade time for money. You're the bottleneck.

**Agent Skills let you package that expertise.**

Remember the Matrix? Trinity needs to fly a helicopter. She doesn't know how. Tank loads the skill. Seconds later: "Let's go."

That's what you're building. Your domain expertise—years of pattern recognition, decision frameworks, workflow optimization—encoded into portable skills that any AI agent can load when needed.

The difference between you and Trinity: **you can sell the skill.**

---

## Skills Are Your Core Asset

Return to the Agent Factory vision from Chapter 1:

**YOUR domain expertise → Skills → Digital FTE → Recurring Revenue**

| Your Knowledge | Becomes | Sells As |
|----------------|---------|----------|
| Financial analysis expertise | Financial analysis skill | Financial Digital FTE |
| Legal document review | Contract review skill | Legal assistant FTE |
| Sales qualification methods | Lead scoring skill | Digital SDR |
| Customer service workflows | Support handling skill | Customer service FTE |

Skills transform what's in your head (tacit knowledge) into what's in a file (explicit knowledge). That file becomes an asset—portable, licensable, monetizable.

When Thomson Reuters paid $650 million for CoCounsel, they weren't buying software. They were buying encoded legal expertise. Skills are how you create similar value in any domain.

---

## The SKILL.md Format

Every skill needs a `SKILL.md` file at its root. The format is deliberately simple—just Markdown with YAML frontmatter:

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

**The key elements:**

| Element | Purpose | Example |
|---------|---------|---------|
| **name** | Unique identifier | `financial-analysis` |
| **description** | When to activate (agent reads this) | "Use when reviewing quarterly earnings..." |
| **When to Use** | Activation conditions | User asks, data type present |
| **How to Execute** | Step-by-step workflow | Gather → Extract → Compare → Generate |
| **Output Format** | Expected deliverable structure | Summary + Table + Analysis |

That's it. No special syntax. No compilation. Just structured Markdown that agents parse.

The specification itself lives at agentskills.io/specification—you can read the entire thing in a few minutes.

---

## Progressive Disclosure: The Token Efficiency Secret

Here's the problem skills solve:

**Loading everything upfront wastes tokens.**

If an agent loaded all 50 available skills at startup—full instructions, templates, examples—you'd burn through your context window before doing any actual work.

The solution is **progressive disclosure**: loading only what's needed, when it's needed.

```
Level 1: Agent Startup (~100 tokens per skill)
├── Name: "financial-analysis"
└── Description: "Analyze financial statements..."

Level 2: When Skill Activated (< 5K tokens)
└── Full SKILL.md content (when-to-use, execution steps, output format)

Level 3: When Actually Needed
└── Supporting resources (templates, examples, scripts)
```

**How it works:**

**Level 1 (Always loaded):** Agent sees only names and descriptions. For 50 skills, that's roughly 5,000 tokens—manageable.

**Level 2 (On activation):** When the agent decides to use a skill (based on description matching the current task), it loads the full SKILL.md. Still under 5K tokens.

**Level 3 (On demand):** Supporting files—templates, examples, reference data—load only when the skill explicitly requests them during execution.

**The efficiency gains are dramatic:**

| Approach | Token Cost at Startup |
|----------|----------------------|
| Load everything upfront | 14,000 - 80,000+ tokens |
| Progressive disclosure | ~100 tokens per skill |

**80-98% token reduction.** This means your Digital FTE can have dozens of capabilities available without bloating its context window. More capabilities, same cost.

---

## MCP + Skills: Complementary Standards

MCP and Skills solve different problems. Understanding the distinction is critical for Digital FTE design:

| Standard | Purpose | Physical Metaphor |
|----------|---------|-------------------|
| **MCP** | **Connectivity** — how agents talk to tools | The agent's **hands** |
| **Skills** | **Expertise** — what agents know how to do | The agent's **training** |

**Example: Your Digital SDR Processing Stripe Payments**

| Layer | Technology | What It Does |
|-------|------------|--------------|
| **MCP Server** | Stripe MCP connector | Connects to Stripe API (create charges, refund, list transactions) |
| **Skill** | Payment processing skill | Knows *how* to handle payment scenarios (retry logic, error recovery, customer communication) |

The MCP server gives the agent *access* to Stripe. The skill gives the agent *expertise* in using Stripe properly.

**Without MCP:** Agent can't reach Stripe at all.
**Without Skill:** Agent can reach Stripe but doesn't know payment best practices.
**With both:** Agent handles payments like an experienced professional.

Your most powerful Digital FTEs combine both: connected via MCP, smart via Skills.

---

## Adoption: From Claude to Industry Standard

Agent Skills evolved from internal tool to open standard remarkably fast:

| Date | Milestone |
|------|-----------|
| **October 16, 2025** | Anthropic launches Agent Skills for Claude Code |
| **December 18, 2025** | Anthropic releases Agent Skills as **open standard** at agentskills.io |
| **December 2025** | OpenAI adopts the same SKILL.md format for Codex CLI and ChatGPT |

**Agent support (December 2025):**
- Claude Code, Claude apps (Pro, Max, Team, Enterprise)
- Codex CLI, ChatGPT (OpenAI adoption)
- VS Code, GitHub Copilot (Microsoft adoption)
- Cursor, Amp, goose, OpenCode, Letta

**Partner skills available at launch:**
- Canva (design automation)
- Stripe (payment processing)
- Notion, Figma, Atlassian
- Cloudflare, Ramp, Sentry, Zapier

The Anthropic skills repository contains community and official skills ranging from creative applications (art, music, design) to technical tasks (testing web apps, MCP server generation) to enterprise workflows.

**Skills you write today work across this entire ecosystem.**

---

## Monetization Paths

Skills open four revenue streams for your Agent Factory:

### 1. License Individual Skills
Sell your financial-analysis skill to other agent builders. They pay per-seat or per-use.

**Example:** Your lead qualification skill, licensed to 50 marketing agencies at $200/month each = $10,000/month recurring.

### 2. Skill Bundles
Package related skills for specific verticals.

**Example:** "Complete SDR Skill Set" includes lead qualification, email personalization, objection handling, and meeting scheduling. Premium pricing for the bundle.

### 3. Skill-Enhanced Digital FTEs
Deploy agents with your skills as subscription products—the core Agent Factory model.

**Example:** Your Digital SDR subscription at $1,500/month, powered by skills you've encoded. 100 clients = $150,000/month.

### 4. Skill Development Consulting
Help domain experts encode their expertise into skills.

**Example:** Law firms, financial advisors, healthcare organizations want custom skills but lack encoding expertise. You build skills for them.

---

## Your Agent Factory Blueprint

As you progress through this book, skills connect to everything:

**Part 3 (Claude Code Mastery):** Use existing skills. Experience progressive disclosure in action. Understand what makes skills effective.

**Part 5 (Custom MCP Servers):** Build MCP servers that your skills orchestrate. Skills become the brain; MCP servers become the hands.

**Part 6 (Custom Agents):** Architect Digital FTEs that combine your skills with MCP connectivity. The complete Agent Factory product.

The skills you design in Part 3 become components of the Digital FTEs you sell in Part 6.

---

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini, or similar) to practice skill design:

### Prompt 1: Create Your First Skill

```
I'm a [your profession/domain] expert. Help me design a SKILL.md for my core expertise.

I need:
1. A clear name and description (the agent reads this to decide when to activate)
2. Specific "When to Use" conditions
3. Step-by-step "How to Execute" instructions
4. A structured "Output Format"

Make it specific enough that an AI agent could follow it without asking clarifying questions.
```

**What you're learning:** Skill specification. The ability to extract tacit knowledge from your head and structure it for AI consumption. This is the core Agent Factory skill.

### Prompt 2: Test the MCP vs Skills Distinction

```
I'm building a Digital [your role] that needs both MCP servers and Skills.

For each capability below, tell me:
- Is this MCP (connectivity) or Skill (expertise)?
- Why?

Capabilities:
1. Connect to my company's CRM to read lead data
2. Know the best practices for qualifying B2B leads
3. Send emails through our email platform
4. Write personalized follow-up emails that convert
5. Schedule calendar meetings
6. Decide the optimal time to follow up with each lead
```

**What you're learning:** Architectural classification. Understanding which layer handles which concern prevents over-engineering and under-delivering.

### Prompt 3: Explore Monetization

```
I have expertise in [your domain]. Based on the four skill monetization paths:
1. License individual skills
2. Skill bundles
3. Skill-enhanced Digital FTEs
4. Skill development consulting

Help me think through:
- Which path has the highest leverage for my expertise?
- What would I need to build for each path?
- What's the realistic monthly revenue potential for each?

Push back if my expectations seem unrealistic.
```

**What you're learning:** Business model thinking. Connecting technical capability to revenue requires understanding both the skill and the market.
