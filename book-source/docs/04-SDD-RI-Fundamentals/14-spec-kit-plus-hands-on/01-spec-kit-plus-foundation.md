---
title: "Spec-Kit Plus Foundation"
chapter: 14
lesson: 1
duration_minutes: 20
proficiency_level: "A2"
cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (Spec-Kit Plus, Horizontal Intelligence, Vertical Intelligence, workflow phases) within A2 limit of 7 ✓"

learning_objectives:
  - objective: "Explain what Spec-Kit Plus is and why it implements SDD-RI methodology"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Verbal explanation with concrete examples"

  - objective: "Distinguish between Horizontal Intelligence (ADRs/PHRs capturing reasoning across time) and Vertical Intelligence (skills/subagents/tools you create)"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Compare and contrast with concrete examples"

  - objective: "Recognize how intelligence accumulates and compounds across projects"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Predict Project 4 capabilities from Projects 1-3 intelligence"

skills:
  - name: "Understanding Reusable Intelligence Architecture"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"

  - name: "Distinguishing Intelligence Types"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/chapter-14-spec-kit-plus-hands-on/spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "3.0.0"
---

# Spec-Kit Plus Foundation

Before you start building your first project with Spec-Kit Plus, understand WHAT this framework is and HOW it makes Reusable Intelligence practical.

By the end of this lesson, you'll understand the core concept that makes Spec-Kit Plus different: **every project generates two deliverables—working code AND reusable intelligence**. You'll see how this shapes everything you'll learn in this chapter.

---

## What Is Spec-Kit Plus?

Spec-Kit Plus is an **SDD-RI (Specification-Driven Development with Reusable Intelligence) framework** built around one core idea:

**Capture intelligence, not just deliver code.**

### The Two-Output Philosophy

Every feature you build produces two distinct outputs:

**1. Working Code** (ephemeral)
- The deliverable—a feature that works and adds value
- Could be rewritten entirely tomorrow without loss
- Project-specific, technology-locked to current choices

**2. Reusable Intelligence** (permanent)
- Reasoning patterns: "Why did we choose this approach?"
- Decision frameworks: "How do we evaluate similar choices?"
- Prompt templates: "What prompts work for this type of problem?"
- Preserved in `history/` directory for YOUR future reference

**Example**:
```
PROJECT 1: Build Authentication System
  Deliverable: Working Flask + JWT authentication (could be rewritten)
  Intelligence captured (stays in Project 1):
    - ADR: "Chose JWT over sessions because..."
    - PHR: "These specific prompts generated secure, validated code"

PROJECT 2: Build Database System (new project)
  Starts fresh — no automatic access to Project 1's artifacts
  But YOU learned from Project 1:
    - YOU remember the decision patterns that worked
    - YOU apply similar prompt structures that succeeded
    - YOUR experience compounds even without explicit artifact transfer
```

### Framework Components

**Templates**: Standardized documents for capturing thinking
- Specifications (what you're building)
- Plans (how you're building it)
- ADRs (why you chose this approach)
- PHRs (what prompts worked)

**Slash Commands**: AI orchestration tools
- `/sp.specify` - Write clear specifications
- `/sp.clarify` - Identify missing requirements
- `/sp.plan` - Generate implementation plans
- `/sp.tasks` - Break work into atomic units
- `/sp.implement` - Execute tasks with AI

**Directory Structure**: Intelligence preservation
- `specs/` - Specifications for each feature
- `history/adr/` - Architectural Decision Records (your reasoning)
- `history/prompts/` - Prompt History Records (what worked for you)

**Critical distinction**: Spec-Kit Plus is NOT an AI service. It's a methodology framework that works WITH your AI tool (Claude Code, Gemini CLI, etc.). It activates your AI tool's reasoning capabilities through structured workflows.

---

## Horizontal Intelligence: YOUR Learning Across Time

Horizontal Intelligence answers: "What have I learned that makes me better at this?"

YOUR knowledge flows **across time**—each project makes you smarter. The artifacts (ADRs, PHRs) document your learning within each project; the wisdom lives in YOU.

### ADRs (Architectural Decision Records)

An ADR documents the "WHY" behind significant decisions, not just the "WHAT" you built.

**Bad documentation** (loses reasoning):
```
"Used PostgreSQL for user data storage"
```

**ADR (preserves reasoning)**:
```
DECISION: Use PostgreSQL for user data storage (not MongoDB)

CONTEXT:
- Application requires ACID guarantees for financial transactions
- Team has PostgreSQL expertise (no MongoDB learning curve)
- Schema is well-defined (no document-oriented benefits)

REASONS:
1. Transaction integrity: Financial data cannot tolerate eventual consistency
2. Operational knowledge: Team expertise reduces deployment risk
3. Cost: PostgreSQL license vs MongoDB enterprise features

TRADEOFF ACCEPTED:
- Flexibility: SQL schema less flexible than document model (acceptable, schema is stable)
- Scalability: Horizontal scaling more complex (acceptable, vertical scaling sufficient for 5 years)

CONSEQUENCE: Future financial features inherit this decision
```

**Why this matters**: Six months later, new team member asks "Why PostgreSQL?" The ADR explains immediately. Within this project, everyone understands the reasoning behind decisions.

### PHRs (Prompt History Records)

PHRs log AI collaboration sessions, capturing what prompts work vs what fails.

**Example**:
```
PROMPT A (generic):
"Generate a database schema"
→ Result: Incomplete, missing indexes, poor naming

PROMPT B (specific):
"Generate PostgreSQL schema for financial transactions with:
 - Indexes on all foreign keys
 - Constraints for data integrity
 - Audit trail for compliance
 - Clear naming conventions"
→ Result: Production-ready schema

PHR CAPTURES: Prompt B works 85% better than Prompt A
```

**Automatic creation**: PHRs are automatically created during `/sp.specify`, `/sp.plan`, and `/sp.implement` phases within a project. The system logs what prompts worked vs. failed as you work.

**Project-scoped**: PHRs belong to the project where they were created. They do NOT transfer to your next project. The value is in YOUR learning—as you work, you internalize what prompts succeed. That knowledge stays with YOU.

### Intelligence Accumulation: The Compounding Effect

Each project generates intelligence artifacts. Even though artifacts stay in their project, YOUR learning compounds.

**Your Learning Timeline**:
```
PROJECT 1 (Month 1):
  System creates ADRs and PHRs within this project
  Time investment: 60 hours (learning and figuring things out)
  YOU learn: What decisions worked, what prompts succeeded

PROJECT 2 (Month 2):
  Starts fresh — new project, new artifacts
  But YOU are smarter: You remember patterns from Project 1
  Time investment: 35 hours (40% faster because YOU learned)

PROJECT 3 (Month 3):
  YOU recognize patterns even faster
  YOU make decisions more confidently
  Time investment: 25 hours (further acceleration)

PROJECT 10 (Year 2):
  YOU have internalized learnings from 9 projects
  YOU build complex features in days instead of weeks
  YOUR expertise compounds — each project made you better
```

This is **Horizontal Intelligence** because YOUR knowledge flows across time—from past projects to future ones. The artifacts document your learning within each project; the wisdom lives in YOU.

---

## Vertical Intelligence: Components You Create

Vertical Intelligence answers: "What reusable components can I create to handle recurring problems better?"

After sessions that go particularly well, YOU can create reusable intelligence components. These come in different forms:

### Types of Reusable Intelligence

| Component | What It Is | When to Create |
|-----------|------------|----------------|
| **Skill** | Structured prompt with Persona + Questions + Principles | Pattern recurs, needs guided reasoning |
| **Subagent** | Specialized agent with focused expertise | Complex autonomous task (5+ decisions) |
| **Tool/MCP Server** | Custom capability the agent can invoke | Need to connect to external systems |

**Lesson 9** teaches you how to design these components. For now, understand the core idea:

### The Core Idea

```
1. YOU have a great session (e.g., writing a security specification)
2. YOU recognize: "This approach would work for similar problems"
3. YOU create a reusable component (skill, subagent, or tool)
4. AGENT invokes your component in future sessions
```

**All reusable intelligence is:**
- **User-created**: YOU decide when to create them, YOU design them
- **Agent-invoked**: Once created, the agent uses them when appropriate
- **Reusable**: Well-designed components work across similar problems

### Why This Matters

Unlike PHRs (which document what happened), reusable intelligence components are **active tools**:
- PHRs: "Here's what worked" (your learning record)
- Skills/Subagents/Tools: "Here's how to do this type of task" (agent's capabilities)

This is **Vertical Intelligence** because you build specialized components that make AI collaboration more effective over time.

---

## The Two Architectures Work Together

**Horizontal Intelligence** (YOUR learning across time):
- "What did I learn from Project 1?"
- "What prompts worked for me?"
- "Why did I choose this approach?"
- Captured in: ADRs, PHRs

**Vertical Intelligence** (YOUR reusable components):
- "What components have I created?"
- "Which one fits this problem?"
- "How can I improve them?"
- Captured in: Skills, Subagents, Tools

Together, they create **YOUR compounding expertise**:

```
Horizontal Intelligence (ADRs, PHRs):
  YOU document learnings: Project 1 → Project 2 → Project 3 → ...

Vertical Intelligence (Skills, Subagents, Tools):
  After good sessions, YOU create reusable components
  YOU improve them based on experience

Result:
  By Project 10, YOU have documented reasoning (ADRs), proven prompts (PHRs),
  and specialized components—YOUR work becomes 5-10x faster than Project 1
```

---

## Workflow Phases Overview

Throughout this chapter, you'll work with these workflow phases:

**Phase 1: Constitution** (Lesson 3)
- Document project-wide quality standards
- Define success criteria across all work

**Phase 2: Specification** (Lesson 4)
- Write clear, testable requirements using SMART criteria
- Capture intent before implementation

**Phase 3: Clarification** (Lesson 5)
- Identify missing constraints and edge cases
- Refine specifications based on feasibility

**Phase 4: Planning** (Lesson 6)
- Design architecture and technology choices
- Document rationale in ADRs

**Phase 5: Tasks** (Lesson 7)
- Break work into atomic, measurable units
- Define checkpoints and validation

**Phase 6: Implementation** (Lesson 8)
- Execute tasks with AI using `/sp.implement`
- Generate working features

**Phase 7: Intelligence Design** (Lesson 9)
- Learn to create reusable components after good sessions
- Design skills, subagents, or tools based on pattern complexity

**Phase 8: Adoption** (Lessons 10-11)
- Apply Spec-Kit Plus to existing projects
- Experience how documented intelligence accelerates YOUR work

Throughout this workflow, YOU decide when to capture intelligence. The framework shows you HOW—but YOU control WHEN and WHAT to create.

---

## Try With AI

Explore what you learned with these simple prompts:

**Understand the Two Outputs:**

> "Explain the two outputs of every Spec-Kit Plus project: working code and reusable intelligence. Why does capturing the 'why' behind decisions matter more than just delivering code?"

**See How ADRs Help Within a Project:**

> "A new team member joins my project and asks 'Why did we choose PostgreSQL?' I show them the ADR that says 'Chose PostgreSQL because we needed transaction guarantees.' How does this ADR help the team?"

**Understand PHRs:**

> "What's the difference between a generic prompt like 'Generate a database schema' and a specific prompt like 'Generate PostgreSQL schema with indexes on foreign keys and audit trail for compliance'? Why would I want to save what worked?"

**Preview Skills (Lesson 9):**

> "In Spec-Kit Plus, what's a 'skill' and why would I create one after a session goes really well? Just give me a simple overview—I'll learn the details later."
