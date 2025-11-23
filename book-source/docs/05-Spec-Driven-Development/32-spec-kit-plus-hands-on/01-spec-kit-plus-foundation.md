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
source_spec: "specs/10-chapter-33-redesign/spec.md"
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

![Architecture diagram showing Spec-Kit-Plus workflow with Constitution, Specification, Planning, Tasks, and Implementation phases](/img/part-5/chapter-32/spec-kit-plus-architecture.png)

![Eight-phase Spec-Kit-Plus workflow showing Constitution → Specify → Clarify → Plan → Tasks → Implement → Validate → Refactor progression](/img/part-5/chapter-32/eight-phase-workflow.png)

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

## Try With AI

Ready to understand how Spec-Kit Plus turns project work into reusable intelligence? Explore these patterns:

**🔍 Explore Intelligence Patterns:**
> "Compare Horizontal Intelligence (ADRs/PHRs) vs Vertical Intelligence (Subagents) in Spec-Kit Plus. Show me a concrete example: if I build a calculator in Project 1 and create 3 ADRs and 10 PHRs, how exactly does that intelligence help me when I build an authentication system in Project 2?"

**🎯 Practice P+Q+P Analysis:**
> "I want to create a subagent for API design review. Help me design it using Persona+Questions+Principles (P+Q+P). What persona should it adopt? What questions should it ask to activate reasoning? What principles should guide its decisions? Show me why this approach works better than a generic prompt."

**🧪 Test Compounding Understanding:**
> "Calculate the compounding effect: If Project 1 creates 5 ADRs + 12 PHRs, Project 2 creates 4 ADRs + 9 PHRs, and Project 3 creates 3 ADRs + 7 PHRs, how much intelligence is available when I start Project 4? Explain why intelligence accumulation makes later projects faster."

**🚀 Apply to Your Domain:**
> "I work in [describe your domain/industry]. Help me design an intelligence architecture using Spec-Kit Plus. What Horizontal Intelligence should I capture (what decisions repeat across projects)? What Vertical Intelligence should I create (what specialized subagents would help)? Outline 3-5 intelligence artifacts I should build first."

---
