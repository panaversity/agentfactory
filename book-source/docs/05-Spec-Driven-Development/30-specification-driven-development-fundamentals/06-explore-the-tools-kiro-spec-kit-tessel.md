---
title: "Which SDD Framework or Tool Fits YOUR Work?"
chapter: 30
lesson: 6
duration: "3-3.5 hours"
skills:
  - name: "Tool Evaluation"
    proficiency: "B1"
    category: "Technical"
  - name: "Decision Making Under Constraints"
    proficiency: "B1"
    category: "Soft"
  - name: "Context Analysis"
    proficiency: "B1"
    category: "Soft"
learning_objectives:
  - "Compare four SDD approaches (Kiro, Spec-Kit, Spec-Kit Plus, Tesel) and understand their evolution (B1)"
  - "Understand how Spec-Kit Plus extends Spec-Kit with ADRs, PHRs, and Intelligence Templates (A2)"
  - "Select the right framework for your situation and understand why this book teaches Spec-Kit Plus (B1)"
---

# Which SDD Framework or Tool Fits YOUR Work?

You've learned the _why_ of specification-driven development. You've written a spec. You understand Constitutions.

Now comes a practical question: **Which SDD framework should YOU use?**

Four major approaches emerged in 2025, each representing a different philosophy about how much structure and intelligence you need. In this lesson, you'll evaluate them against your specific situation and understand why this book teaches **Spec-Kit Plus** — Panaversity's evolution of GitHub's Spec-Kit with AI-native extensions.

---

## Know Your Context

Before comparing frameworks, ground yourself in reality. **These questions determine which tool actually fits.**

### Question 1: Who's Building This?

- **Solo developer** — Just you, learning or building side projects
- **Small team** — 2-5 people, probably in an early-stage company or internal project
- **Medium team** — 5-15 people, established project or startup scaling
- **Large team** — 15+ people across multiple services/domains

### Question 2: What's the Problem Scale?

- **Simple** — One feature, one service.
- **Medium** — Multiple features, system integrations
- **Complex** — Multiple services, architectural decisions or ongoing maintenance.

### Question 3: Are There Regulatory/Compliance Constraints?

- **No constraints** — Build what works. Move fast. Iterate.
- **Light constraints** — "Best practices" (like 80% test coverage)
- **Heavy constraints** — Regulated domain (healthcare, finance, payments, aerospace)
- **Strict constraints** — FDA/SOX/HIPAA/PCI compliance required. Auditable decisions matter.

**Quick self-assessment**: Answer these questions. Don't overthink—your gut feeling matters here. Write them down; you'll refer back to them.

---

## Four SDD Approaches – Designed for Different Needs

Four main approaches emerged to answer this question. Here's how to think about each one.

---

### 1: Kiro — "Start Simple"

**Philosophy**: SDD shouldn't require learning complex processes.

#### Who It's For

✅ **Excellent fit if you are:**

- Solo developer or tiny team (1-3 people)
- Learning SDD for the first time (need low friction)
- Building medium-sized features (not tiny bug fixes, not massive systems)
- Already comfortable with Agile/BDD workflows

❌ **Poor fit if you are:**

- Building complex systems requiring strong architectural enforcement
- On a medium-large team where consistency across projects matters
- In a regulated domain where auditable decisions are critical
- Worried about "how do I enforce that ALL passwords use bcrypt?"

#### The Trade-Off

You get **simplicity and low cognitive load**. You give up **governance and consistency enforcement**.

---

### 2a: Spec-Kit (GitHub's Framework)

**Philosophy**: Strong governance through immutable principles.

Spec-Kit is GitHub's open-source SDD framework released in 2025. It's built around a **Constitution** — a document of immutable rules that apply to **EVERYTHING** in your codebase.

#### The Core Workflow: Constitution → Specify → Plan → Tasks

**1. SPECIFY: What are we building?**

- Clarify requirements and acceptance criteria
- Check for Constitution violations BEFORE code is written
- Output: Specification document + clarification checklist

**2. PLAN: How will we build it?**

- Design architecture and technology decisions
- Verify Constitutional alignment
- Output: Plan document + technical checklist

**3. TASKS: What are the work units?**

- Break into atomic tasks (each 4-8 hours of work)
- Each task traces back to specific requirements
- Output: Task breakdown + task checklist

#### Why Spec-Kit (GitHub) Works

✅ **Strengths:**

- **Strong governance**: Constitution enforces consistency across all features and teams
- **Comprehensive traceability**: Requirements → Plan → Tasks → Code → Tests (auditable)
- **Scales to large teams**: 5 people, 50 people, 500 people—same process works
- **Industry standard**: GitHub's official framework; major companies are adopting it

#### Where Spec-Kit (GitHub) Struggles

❌ **Limitations:**

- **Doesn't track "why" decisions**: Plans document "how," but not "why we chose X over Y"
- **Doesn't log AI interactions**: When an agent generates code, no record of the reasoning
- **No domain templates**: Each team rebuilds domain-specific rules from scratch
- **File proliferation**: A single feature generates 10+ files
- **Steep learning curve**: Phases, checklists, Constitutional structure takes time to learn

#### When to Use Spec-Kit (GitHub)

Choose Spec-Kit if you want to:

- Use GitHub's official open-source framework
- Build your own extensions and customizations
- Avoid any additional proprietary tools

---

### 2b: Spec-Kit Plus (Panaversity's Evolution) — "Spec-Kit + Intelligence"

**What is it?** Spec-Kit Plus is Panaversity's fork and evolution of GitHub's Spec-Kit, designed specifically for **AI-native development** and **professional teams**. It extends GitHub's framework with three critical additions.

#### Horizontal Additions: Decision & Interaction Intelligence

**1. Architectural Decision Records (ADRs)**

- Document the "why" behind choices, not just the "what"
- Example: "Why JWT instead of sessions?" → ADR explains tradeoffs, security implications, and when to reconsider
- **For AI agents**: When facing a choice, agents reference ADRs to understand team philosophy

**2. Prompt History Records (PHRs)**

- Track every AI interaction during development
- Captures: prompt sent → response generated → whether accepted/modified/rejected → why
- Creates audit trail for compliance and learning
- **For AI agents**: Agents learn which prompts produce better results. Teams have compliance documentation.

#### Vertical Additions: Domain Intelligence Templates

Spec-Kit Plus ships with **Intelligence Templates** for specific domains. Each template extends Constitution with domain-specific rules, patterns, and validations.

**Example: Education Template** (what this book uses)

#### The Complete Picture: Spec-Kit Plus

```
GitHub's Spec-Kit (Base Layer)
    ↓
    + ADRs (capture architectural "why")
    + PHRs (capture AI interaction "how")
    + Resusble Intelligence Templates (domain expertise)
    ↓
Spec-Kit Plus (AI-native SDD for professional teams)
```

#### Why Spec-Kit Plus Works

✅ **Strengths:**

- **Everything Spec-Kit has**, plus:
- **Captures reasoning**: ADRs document architectural decisions and tradeoffs
- **Logs AI interactions**: PHRs create audit trail and improve future prompts
- **Domain expertise built-in**: Intelligence Templates save teams from rebuilding domain-specific rules
- **AI-agent friendly**: Agents read Constitution + spec + ADRs + PHRs + templates for complete context
- **Designed for co-learning**: PHRs and ADRs capture how humans and AI reasoned together

#### Where Spec-Kit Plus Is Best

✅ **Excellent fit if you are:**

- On a team (or planning to be) working with AI agents
- Building a system that will last years
- In a regulated domain (healthcare, finance, aerospace)
- Want to learn from AI interactions (improve prompts over time)
- Building in a specific domain (education, healthcare, fintech, etc.)

#### When to Use Spec-Kit Plus

Choose Spec-Kit Plus if:

- You're using this book (all chapters assume Spec-Kit Plus)
- You're on a team collaborating with AI agents
- You need auditable decisions (compliance, regulated domains)
- You want domain-specific expertise without rebuilding from scratch
- You want to track and learn from AI interactions

---

### 3: Tesel — "Emerging Technology"

**Philosophy**: Specs are the only source of truth. Code is generated, never hand-edited.

#### What It Is

Tesel (still in private beta, invite-only) takes spec-driven development to its logical extreme: **specs become the primary artifact, and code is generated**.

**Workflow:**

```
1. Write specification (natural language, structured)
2. Run code generator
3. Code produced (marked: DO NOT EDIT)
4. Later: Update specification (not code)
5. Regenerate code automatically
```

#### The Appeal

- ✅ Specs always match code (never diverge)
- ✅ Code is always current (regenerated from spec)
- ✅ No technical debt from hand-edited code

#### The Reality

Tesel works for specific domains:

- Safety-critical systems (medical devices, aerospace)
- Regulated industries with stable requirements
- Situations where reproducibility and auditability are paramount

It's experimental for most other use cases because:

- **Non-determinism**: Run the generator twice, sometimes get different code
- **Incomplete specs**: Natural language might miss edge cases; you can't manually fix them in code
- **Stability**: Still in beta with limited adoption

**Status**: Invite-only, actively developed. You can track progress at [Tesel's registry](https://tesel.io/).

#### When Tesel Might Work

- Medical device software (FDA requires reproducible processes)
- Financial systems (regulatory audit trails)
- Aerospace/defense (safety-critical)
- If you have stable requirements and don't need to iterate quickly

#### When Tesel Doesn't Work

- Startups with rapidly changing requirements
- Exploratory or experimental projects
- Situations where manual optimization is critical

---

## Understanding the SDD Evolution: Four Tools, One Goal

You've now seen four approaches to SDD. Before we look at real scenarios, let's understand how they relate and why **Spec-Kit Plus** is what you'll actually use in this book.

### The Four Tools on the Spectrum

```
Kiro              Spec-Kit            Spec-Kit Plus     Tesel
(Simple)          (GitHub)            (Panaversity)     (Automatic)
  ↓                 ↓                    ↓                 ↓
Req→Design        Const+Spec+Plan     +ADRs+PHRs         Spec-only
→Tasks            +Tasks              +Templates         Generated
                                                          Code
No governance  Immutable rules  Immutable rules    Generated code
              only             + Decision intel   from specs
                              + Domain intel

Solo/small     Enterprise       All teams + AI     Safety-critical
teams          teams            agents
```

### From Spec-Kit to Spec-Kit Plus: What Changed?

GitHub's **Spec-Kit** gave us the foundation:

- Constitution (immutable rules)
- Three-phase workflow (Specify → Plan → Tasks)
- Strong governance and traceability

But working with AI agents and real professional teams, we discovered critical gaps:

#### Gap 1: Lost Reasoning – Solution: ADRs

**The Problem**: Specs document "what." Plans document "how." But nobody documents "why we chose X over Y."

When an AI agent encounters a choice later (e.g., "Should we use JWT or sessions?"), it doesn't know the team's reasoning.

**The Solution**: **Architectural Decision Records (ADRs)**

- Document the "why" behind architecture choices
- Example: "Why JWT instead of sessions? → ADR explains security tradeoffs, when to reconsider, etc."
- **For AI agents**: Agents reference ADRs to understand team philosophy and make consistent choices

#### Gap 2: Lost Learning – Solution: PHRs

**The Problem**: When an AI agent generates code, how do you know what prompt led to that output? How do you improve next time?

There's no record of the AI's reasoning, no audit trail.

**The Solution**: **Prompt History Records (PHRs)**

- Track every AI interaction: prompt sent → response → whether accepted/modified/rejected → why
- Creates an audit trail (compliance requirement)
- **For AI agents**: Agents learn which prompts work best. Teams improve future interactions.

#### Gap 3: Lost Domain Knowledge – Solution: Intelligence Templates

**The Problem**: A healthcare team writes their own HIPAA rules. A fintech team writes their own PCI-DSS rules. A team building educational software writes their own pedagogy rules.

Each team rebuilds domain-specific knowledge from scratch. Inconsistent patterns. Wasted effort.

**The Solution**: **Intelligence Templates**

- Pre-built domain packages with rules, patterns, and validations
- Example: Education Template (used in this book) includes CEFR proficiency levels, Bloom's taxonomy levels, code testing requirements
- **For AI agents**: Agents know domain patterns. Healthcare teams don't rebuild HIPAA knowledge; Fintech teams don't rebuild PCI-DSS.

### What This Book Teaches: Spec-Kit Plus

```
GitHub's Spec-Kit (Base Layer)
    ↓
    + ADRs (capture architectural "why")
    + PHRs (capture AI interaction "how")
    + Intelligence Templates (domain expertise)
    ↓
Spec-Kit Plus (AI-native SDD for professional teams)
```

**You're not learning abstract theory.** You're learning the practical version that:

- Works with AI agents (Claude Code, Gemini CLI, etc.)
- Scales from solo to 50+ person teams
- Includes domain expertise (Education template for this book, healthcare/fintech/aerospace for others)
- Captures learning (PHRs) and reasoning (ADRs) automatically

---

### Why Spec-Kit Plus for This Book?

**This book teaches Spec-Kit Plus because:**

#### 1. You're Learning to THINK, Not Just Code

Kiro teaches "spec-first thinking."

Spec-Kit Plus teaches "Constitutional thinking"—how to express principles that survive change.

- **Kiro**: "Write a user story for this feature"
- **Spec-Kit Plus**: "What immutable rules must this feature respect? Why?"

Spec-Kit Plus forces you to think about **why** decisions matter. That thinking transfers to any framework.

#### 2. You Might Join a Team

The preface said: "Students & Self-Learners... Developers... Entrepreneurs & Innovators."

Many of you will learn solo and then join teams. Spec-Kit Plus scales from solo → team → large organization.

- Kiro teaches you "how to spec," but not "how to spec at scale"
- Spec-Kit Plus teaches both
- Learning Spec-Kit Plus now = you're ready for real engineering

#### 3. AI Collaboration Gets Better With Discipline

When you work with Claude Code, Gemini CLI, or other AI agents:

- **Kiro**: Agent reads spec, generates code
- **Spec-Kit Plus**: Agent reads Constitution + spec + ADRs + PHRs, generates code that respects ALL organizational principles

Constitution + specs + ADRs + PHRs = **complete knowledge system for AI agents**

This is what professional teams need. It's the difference between:

- "Build a login system" (hope the agent includes password reset)
- "Build a login system that respects our Constitution (bcrypt passwords, 80%+ tests, audit logging)" (agent knows the rules)

#### 4. The Industry Is Standardizing

In 2025, GitHub, major tech companies, and emerging startups chose Spec-Kit-like approaches:

- Clear governance (Constitution)
- Traceability (specs → plans → code)
- Enforcement (AI agents respect principles)

Learning Spec-Kit Plus now means **you're ahead of where the industry is moving**.

---

**Next lesson**: You'll build your first Spec-Kit Plus feature from start to fi`` nish, learning by doing.

**The journey to specification-driven development starts now.**

---
