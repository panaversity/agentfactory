---
title: "Plan Phase - Architecture Decisions and ADRs"
chapter: 31
lesson: 6
duration_minutes: 120

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.plan Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.plan and interpret generated implementation plan"

  - name: "Understanding Implementation Plan Structure"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands plan components (phases, dependencies, milestones)"

  - name: "Identifying Architecturally Significant Decisions"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes which decisions warrant ADR documentation (long-term impact, alternatives, future questioning)"

  - name: "Writing Architectural Decision Records (ADRs)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write ADR documenting decision title, context, decision, consequences, alternatives"

  - name: "Recognizing Cascade Quality (Plan from Spec)"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student observes how clear specs produce clear plans and how vague specs produce vague plans"

learning_objectives:
  - objective: "Use /sp.plan to generate implementation plan from calculator specification"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.plan and understanding of generated plan"

  - objective: "Identify 2-3 architecturally significant decisions in calculator implementation plan"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Articulation of decisions with long-term impact and multiple alternatives"

  - objective: "Write ADRs documenting calculator architecture decisions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "ADR documentation completeness (title, context, decision, consequences, alternatives)"

  - objective: "Understand plan structure and recognize how spec quality determines plan quality"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of plan components and cascade effect"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Plan command, plan structure, architectural decisions, ADR components, decision significance, consequences analysis, alternatives consideration) within B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Write 3-5 ADRs; analyze which decisions would change if spec requirements changed"
  remedial_for_struggling: "Focus on identifying top 2 ADRs; use provided ADR template without multiple alternatives"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Plan Phase - Architecture Decisions and ADRs

With your specification complete, you now face a new question: How will you actually build it?
This is the essence of the Plan Phase - transforming the ‘What’ of your specification into the ‘How’ of architecture and implementation strategy.

`/sp.plan` generates an implementation plan that breaks your specification into:
- **Architectural components** (core library, error handlers, type system, tests)
- **Implementation phases** (build operations first, then tests, then documentation)
- **Dependencies** (what must be built before what)
- **Design decisions** (which ones matter enough to document)

This lesson teaches you how to work with generated plans and how to capture important architectural decisions using **ADRs (Architectural Decision Records)**.

---

## Understanding the /sp.plan Command

`/sp.plan` analyzes your specification and generates a detailed implementation plan by:

1. **Breaking spec into components** - Which parts of your spec need separate modules?
2. **Ordering dependencies** - What must be built first?
3. **Identifying design decisions** - Where are there multiple valid approaches?
4. **Proposing architecture** - How should code be organized?

**Input**: Your specification (what the calculator must do)

**Output**: Implementation plan with:
- Architecture overview
- Implementation phases
- Component breakdown
- Dependencies and sequencing
- Design decisions highlighted

**The Cascade Effect**: Detailed spec → detailed plan. Vague spec → vague plan.

#### 💬 AI Colearning Prompt
> "Why does the Plan phase generate implementation strategy for ALL operations at once instead of planning each operation separately? What consistency benefits does this provide?"

---

## Generating Your Implementation Plan

Let's generate the plan for your calculator.

### Step 1: Run /sp.plan

In Claude Code, from your calculator-project directory:

```
/sp.plan

Create: architecture sketch, interfaces, data model, error handling, requirements.
Decisions needing: list important choices with options and tradeoffs.
Testing strategy: unit + integration tests based on acceptance criteria.

Technical details:
- Use a simple, functional approach where it makes sense
- Use Python 3.12+ type hints with | union syntax
- Follow TDD: write tests first, then implementation
- Organize code and tests according to your constitution rules
```

**Agent Does:**

- Creates technical implementation plan
- Defines data models and interfaces
- Establishes testing strategy
- Identifies architectural decisions
- Generates quick-start.md and plan.md files

**Why This Matters:**
The plan defines technical architecture for ALL operations at once. This ensures consistency - same type hints, same error handling, same testing approach. Much more efficient than planning each operation separately!

### Step 2: Review Generated Plan

The generated plan should include:

- **Architecture Overview**: How code will be organized (one module? multiple? class-based?)
- **Implementation Phases**: 3-5 phases building from simple to complex
- **Component Breakdown**: Core operations, error handling, type validation, tests, docs
- **Sequencing**: Operations before tests? Validation before operations?
- **Design Decisions**: Where are there choices? (Class vs functions? Exception types? Precision handling?)

---

## Understanding ADRs (Architectural Decision Records) (20 minutes)

Planning exposes **architectural decisions** - choices about how to build that have long-term consequences.

### What Is an ADR?

An **ADR** documents:
- **The Decision**: What choice did you make?
- **The Context**: Why did you need to make this choice?
- **The Alternatives**: What other options existed?
- **The Rationale**: Why did you choose this over alternatives?
- **The Consequences**: What are the long-term impacts?

### When Should You Create an ADR?

**Create an ADR when**:
- The decision has **long-term impact** (affects code structure, not just style)
- **Multiple valid alternatives** existed (not an obvious choice)
- **Future developers will question** the decision
- The decision **constrains future choices** (e.g., choosing a data structure)

**Don't create ADRs for**:
- Style choices (naming conventions, formatting)
- Obvious choices (of course we use Python!)
- Temporary decisions (will revisit in 6 months)
- Out-of-scope decisions (already decided by Constitution)

### Creating ADRs for Your Calculator Plan 

Now let's identify and document the architectural decisions from your calculator plan.

```
/sp.adr review the generate plan and record key Architectural Decisions.
```

It will review the plan and record key architectural decisions in history/adr directory.

#### 🎓 Expert Insight
> In AI-native development, ADRs aren't documentation debt—they're future leverage. When a teammate (or AI agent) asks "Why did we use ValueError instead of custom exceptions?", you point to the ADR. When requirements change and someone questions the architecture, the ADR explains the original constraints and tradeoffs. ADRs save hours of archaeological code reading by preserving the "why" alongside the "what."

#### 🤝 Practice Exercise

> **Ask your AI**: "I've generated an implementation plan for my calculator. Can you review `specs/calculator/plan.md` and help me identify: (1) Which 2-3 decisions are architecturally significant (long-term impact, multiple alternatives, future questioning)? (2) Which decisions are trivial and don't need ADRs? (3) For one significant decision, what alternatives exist and what are the tradeoffs? Then help me draft an ADR for that decision."

**Expected Outcome**: Your AI should identify significant decisions (e.g., exception strategy, function vs class-based design, precision handling) versus trivial ones (e.g., naming conventions from Constitution), explain tradeoffs for each alternative, and help structure an ADR with clear context, decision, consequences, and alternatives sections.

---

## Common Mistakes

### Mistake 1: Documenting Every Small Decision as ADR

**The Error**: Creating ADRs for trivial choices like "Use snake_case for functions" or "Put tests in tests/ folder"

**Why It's Wrong**: ADRs are for **architecturally significant** decisions (long-term impact, multiple alternatives, future questioning). Trivial choices clutter your ADR history.

**The Fix**: Apply the three-part test:
- Does this have long-term consequences?
- Are there multiple viable alternatives?
- Will someone ask "why did we choose this" in 6 months?

If not all three → Skip the ADR.

### Mistake 2: Vague ADR Consequences

**The Error**: ADR says "This approach is better" without explaining tradeoffs

**Why It's Wrong**: Future developers need to understand **why** you chose this and what you gave up.

**The Fix**: Document both positives and negatives:
- ✅ "Pros: Simpler error handling. Cons: Less precise error messages for users."
- ✅ "Alternatives considered: Exception hierarchy (rejected: overkill for 5 operations)"

---

## Try With AI: Validate Your Plan and ADRs

Use your AI companion to review your implementation plan and ADRs.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Context**: Your plan.md and 1-2 ADRs

**Goal**: Confirm plan is sound and ADRs capture key architectural decisions

:::tip ⚠️ Learning WITH AI (Not Generating FROM AI)

**What this exercise teaches:**
- ❌ **DON'T ask**: "Write the implementation code for me"
- ❌ **DON'T ask**: "Generate all the functions"
- ✅ **DO ask**: "Is my plan realistic and well-structured?"
- ✅ **DO ask**: "Do my ADRs capture architecturally significant decisions?"
- ✅ **DO ask**: "What alternatives should I consider for this decision?"

**Your role**: Review plan structure, evaluate ADR quality, make architectural decisions
**AI's role**: Validate plan soundness, suggest improvements, identify missing ADRs

:::

### Prompt Set (Copy-Paste Ready)

**Prompt 1 - Plan Review**

Copy and paste this into Claude Code:

```
I've generated an implementation plan for my calculator.

Looking at my specification, does this plan address all requirements?
Are the phases in the right order? Any missing components?
```

**Prompt 2 - ADR Completeness**

After plan review, ask:

```
I've documented these architectural decisions.

Are these the most important decisions? Did I miss any critical architectural
choices? 
```

**Prompt 3 - Plan to Tasks Readiness**

Finally, ask:

```
Based on my plan and ADRs, am I ready for the Tasks phase?

Tasks will break my plan into atomic work units.
Is my plan detailed enough that tasks will be clear and unambiguous?
```

### Expected Outcomes

- **Plan is sound and complete**
- **ADRs capture key architectural decisions**
- **Specification quality has cascaded into plan and ADR quality**
- **Ready for task breakdown (Lesson 6)**
