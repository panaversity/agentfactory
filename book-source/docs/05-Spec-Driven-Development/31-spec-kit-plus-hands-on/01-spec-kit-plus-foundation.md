---
title: "Spec-Kit Plus Foundation: What You're About to Build With"
chapter: 31
lesson: 1
duration_minutes: 20

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Reusable Intelligence Architecture"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain how Spec-Kit Plus captures intelligence through Horizontal (ADRs/PHRs) and Vertical (Subagents) patterns"

  - name: "Distinguishing Intelligence Types"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can differentiate Horizontal Intelligence (reasoning across time) from Vertical Intelligence (delegation hierarchy)"

learning_objectives:
  - objective: "Explain how Spec-Kit Plus captures Reusable Intelligence through ADRs, PHRs, and Subagents"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Verbal explanation with concrete examples"

  - objective: "Recognize the compounding effect of intelligence accumulation across projects"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Compare Project 1 vs Project 10 intelligence availability"

cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (Horizontal Intelligence, Vertical Intelligence) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Design hypothetical intelligence library for complex domain; map compounding effects across 5+ projects"
  remedial_for_struggling: "Focus on concrete ADR/PHR examples from calculator project; use visual diagrams"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-18"
last_modified: "2025-11-18"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.2.0"
---

# Spec-Kit Plus Foundation: What You're About to Build With

Before installation, understand WHAT Spec-Kit Plus is and HOW it makes Reusable Intelligence practical.

By the end of this lesson, you'll understand how Spec-Kit Plus captures and reuses intelligence through its two-architecture pattern, and see exactly what intelligence artifacts you'll build in this chapter.

---

## What Is Spec-Kit Plus?

Spec-Kit Plus is an **SDD-RI framework** built around one core idea: **capture intelligence, not just deliver code**.

Every feature you build generates two outputs:
1. **Working code** (the deliverable - ephemeral, could be rewritten)
2. **Reusable intelligence** (reasoning patterns, decision frameworks, AI prompts that work - permanent)

**The framework provides**:
- Templates for specifications, plans, ADRs (Architectural Decision Records), PHRs (Prompt History Records)
- Slash commands (`/sp.specify`, `/sp.plan`, `/sp.implement`) that orchestrate AI subagents
- Directory structure that separates ephemeral code from permanent intelligence

**Critical distinction**: Spec-Kit Plus is NOT an AI service. It's a methodology framework that works WITH your AI tool (Claude Code, Gemini CLI, etc.).

---

## Horizontal Intelligence: Capturing Reasoning Across Time

Horizontal Intelligence is how Spec-Kit Plus turns decisions into reusable knowledge that persists across projects.

### ADRs (Architectural Decision Records)

Document the "WHY" behind significant decisions, not just the "WHAT" you built.

**Example**:
- **Bad documentation**: "Used JWT for authentication"
- **ADR (Horizontal Intelligence)**: "Chose JWT over sessions because: (1) microservices need stateless auth, (2) mobile clients benefit from token refresh, (3) tradeoff accepted: token revocation complexity"

**When you create ADRs**:
- During Constitution phase (Lesson 3) - Document project quality standards
- During Planning phase (Lesson 6) - Explain architectural choices
- When designing skills/subagents (Lesson 9) - Justify pattern selection

**Storage**: `history/adr/` directory becomes your team's knowledge base

**Reusability**: Six months later, new team member asks "Why this approach?" → ADR answers immediately. AI agents READ ADRs to understand project context before generating code.

### PHRs (Prompt History Records)

Automatically log AI collaboration sessions, capturing what prompts work vs what fails.

**Example**:
- **Prompt A**: "Write a calculator" → Generated insecure code (eval() vulnerability)
- **Prompt B**: "Write a calculator using safe math operations, no eval()" → Generated clean code
- **PHR captures**: Prompt B works, Prompt A fails, reasoning logged

**When PHRs are created**:
- Automatically during `/sp.specify`, `/sp.plan`, `/sp.implement` execution
- You don't manually invoke PHRs - the system creates them

**Storage**: `history/prompts/<feature>/` directory

**Reusability**: Project 2 starts with Project 1's PHRs. Your AI reads: "Last time, prompts emphasizing security constraints worked better than vague prompts." Project 2 specifications are immediately better.

### What Makes This "Intelligence"

This isn't documentation for humans only:
- **AI agents read ADRs** to understand project context before reasoning
- **AI agents learn from PHRs** to avoid repeating past mistakes
- **Intelligence accumulates** - Project 10 is dramatically faster than Project 1

**The compounding effect**:
- Project 1: Create 3 ADRs + 10 PHRs (learning from scratch)
- Project 2: Start with 3 ADRs + 10 PHRs, create 3 new ADRs + 8 new PHRs (total: 6 ADRs, 18 PHRs)
- Project 10: Start with accumulated intelligence from 9 projects, rarely repeat mistakes

---

## Vertical Intelligence: Delegation Through Specialization

Vertical Intelligence is how Spec-Kit Plus distributes work to specialized AI subagents, each designed with the Persona+Questions+Principles (P+Q+P) pattern.

### The Delegation Pattern

```
YOU: "Build a calculator with 5 operations"
  ↓
ORCHESTRATOR: Routes to Specification Subagent
  ↓
SPEC SUBAGENT: Asks clarifying questions, generates complete spec
  ↓
YOU: Review and approve
  ↓
ORCHESTRATOR: Routes to Planning Subagent
  ↓
(Cycle repeats through Plan → Tasks → Implement)
```

### What Makes Subagents "Intelligent"

Each subagent is designed with three components that activate reasoning (not just prediction):

**1. Persona** - Cognitive stance defining how to think
- Example: "You are a requirements analyst who obsesses over edge cases before implementation"
- Not generic: "You are a helpful assistant"

**2. Questions** - Analytical framework guiding reasoning
- Example: "What inputs can break this? What assumptions are hidden? What's the simplest test?"
- Not vague: "Is this good?"

**3. Principles** - Decision criteria for evaluating options
- Example: "Every data input must document boundary conditions (zero, negative, overflow)"
- Not arbitrary: "Make it good"

### Example: Specification Subagent

**Persona**: Requirements analyst who thinks about edge cases before implementation

**Questions**:
- What happens when user inputs zero? Negative numbers? Strings instead of numbers?
- What assumptions am I making about input validation?
- What's the simplest test that proves this specification is complete?

**Principles**:
- SMART criteria enforcement (Specific, Measurable, Achievable, Relevant, Testable)
- Every data input has documented boundary conditions
- Every operation has at least 3 test cases (normal, edge, error)

**Result**: The Specification Subagent doesn't just generate output - it **reasons through problems** using expertise patterns.

### What Makes This "Reusable Intelligence"

- **Not calculator-specific**: The Specification Subagent works for ANY feature needing a spec (authentication, payment processing, file uploads)
- **Not rebuilt every project**: You don't re-teach "how to write specs" - the subagent embeds that expertise
- **Composable**: Lesson 9 teaches you to CREATE your own subagents using the P+Q+P pattern, adding them to your intelligence library

**Why "Vertical"**: Intelligence flows down a hierarchy (You → Orchestrator → Specialists), unlike Horizontal Intelligence which flows across time.

---

## The Intelligence You'll Build in This Chapter

Here's what you'll create across Lessons 2-9:

**Lesson 2: Installation** - Install Spec-Kit Plus framework, verify setup

**Lesson 3: Constitution** - Create project quality standards → **Generate your first ADR** (Horizontal Intelligence artifact)

**Lesson 4: Specification** - Write calculator spec → **Watch Specification Subagent work** (Vertical Intelligence in action)

**Lesson 5: Clarify** - Refine spec with `/sp.clarify` → See AI reasoning improve through iteration

**Lesson 6: Plan** - Generate implementation plan → **Document architectural decisions in ADRs**

**Lesson 7: Tasks** - Decompose plan into atomic work units with checkpoints

**Lesson 8: Implement** - AI generates code → **System creates PHRs automatically**

**Lesson 9: Reusable Intelligence** - Extract patterns from Lessons 3-8 → **Create your own Specification Review Skill + Spec Auditor Subagent** using P+Q+P pattern

### By Lesson 9, You'll Have

1. **Working calculator code** (ephemeral - could be rewritten)
2. **3-5 ADRs** documenting key decisions (permanent - applies to future projects)
3. **10+ PHRs** showing what prompts work (permanent - teaches your AI)
4. **1-2 custom skills/subagents** (permanent - reusable across projects)

### The Compounding Effect in Action

**Project 1 (Calculator)**:
- 5 ADRs (SMART criteria enforcement, error handling patterns, test design principles)
- 12 PHRs (specification prompts that work, planning prompts that fail)
- 2 Skills (Specification Review, Spec Auditor)

**Project 2 (E-commerce Cart)**:
- Start with Project 1's 5 ADRs + 12 PHRs + 2 Skills
- Create 4 new ADRs (state management decisions, database choices)
- Create 8 new PHRs (state machine prompts, transaction safety prompts)
- Create 1 new Skill (State Machine Validator)
- **Total**: 9 ADRs, 20 PHRs, 3 Skills

**Project 10**:
- Start with accumulated intelligence from 9 projects
- Specification phase: 70% faster (reuse existing Spec Auditor + proven prompts)
- Planning phase: 60% faster (ADRs document recurring patterns)
- Implementation: 50% faster (PHRs prevent known failure modes)

This is why Reusable Intelligence (not reusable code) is the primary artifact.

---

## Try With AI: Validate Your Understanding

Now let's test your grasp of how Spec-Kit Plus makes intelligence reusable.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Goal**: Ensure you understand the difference between Horizontal and Vertical Intelligence

:::tip ⚠️ Learning WITH AI (Not Generating FROM AI)

**What this exercise teaches:**
- ❌ **DON'T ask**: "Explain Spec-Kit Plus architecture"
- ✅ **DO ask**: "How do ADRs from Project 1 help me in Project 2? Give concrete example."
- ✅ **DO ask**: "Why does the Specification Subagent use P+Q+P pattern instead of generic prompts?"

**Your role**: Test your mental model, ask clarifying questions, validate understanding
**AI's role**: Distinguish Horizontal vs Vertical Intelligence, explain compounding effects

:::

### Prompt Set (Copy-Paste Ready)

**Prompt 1 - Horizontal Intelligence Understanding**

Copy and paste this into Claude Code:

```
I just learned that Spec-Kit Plus captures Horizontal Intelligence through
ADRs and PHRs. Help me understand the compounding effect:

1. If Project 1 creates 5 ADRs and 12 PHRs, what happens in Project 2?
2. Give a concrete example: If an ADR in Project 1 documents "Why JWT over sessions",
   how does that ADR help me in Project 2 (e-commerce cart)?
3. How do AI agents USE the ADRs and PHRs (not just humans)?

This will help me see why intelligence accumulates across projects.
```

**Prompt 2 - Vertical Intelligence Understanding**

After you receive the response, ask:

```
Now explain Vertical Intelligence (delegation pattern):

1. What's the difference between a generic AI prompt and a subagent designed
   with Persona+Questions+Principles (P+Q+P)?
2. Why is the Specification Subagent REUSABLE across projects (calculator,
   e-commerce, authentication)?
3. In Lesson 9, I'll create my own subagent. What makes it "intelligence"
   instead of just "a prompt"?

I'm trying to understand why P+Q+P activates reasoning, not just prediction.
```

**Prompt 3 - Intelligence Artifacts Preview**

Finally, ask:

```
Looking at Lessons 2-9 in Chapter 31, help me map out what intelligence
I'll actually build:

1. Which lessons create ADRs (Horizontal Intelligence)?
2. Which lessons use existing subagents (Vertical Intelligence)?
3. In Lesson 9, what SPECIFIC intelligence will I create?

This preview will help me understand the chapter progression.
```
