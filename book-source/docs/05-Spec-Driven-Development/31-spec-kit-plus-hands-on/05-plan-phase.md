---
title: "Plan Phase - Architecture Decisions and ADRs"
chapter: 31
lesson: 5
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
generated_by: "lesson-writer v3.0.0"
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

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />

