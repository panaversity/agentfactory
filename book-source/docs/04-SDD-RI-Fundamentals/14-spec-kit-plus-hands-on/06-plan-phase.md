---
title: "Plan Phase — Architecture Decisions and ADRs"
chapter: 14
lesson: 6
duration_minutes: 45
proficiency_level: "B1"
cognitive_load:
  new_concepts: 4

learning_objectives:
  - objective: "Execute `/sp.plan` command to generate implementation strategy"
    bloom_level: "Apply"
    assessment_method: "Student successfully runs `/sp.plan` and interprets output"

  - objective: "Understand that plans bridge specifications to executable tasks"
    bloom_level: "Understand"
    assessment_method: "Student explains plan's role in SDD-RI workflow"

  - objective: "Create ADRs for architecturally significant decisions"
    bloom_level: "Apply"
    assessment_method: "Student uses `/sp.adr` to document key decisions"

  - objective: "Recognize how specification quality determines plan clarity"
    bloom_level: "Understand"
    assessment_method: "Student explains cascade effect (clear spec → clear plan)"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
created: "2025-11-26"
last_modified: "2025-11-27"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Plan Phase — Architecture Decisions and ADRs

With your specification complete and clarified, you now face a new question: **How will you actually build it?** This is the essence of the Plan Phase—transforming the "What" of your specification into the "How" of architecture and implementation strategy.

`/sp.plan` generates an implementation plan that breaks your specification into:
- **Architectural components** (sections, research management, quality validation)
- **Implementation phases** (research first, then writing, then polish)
- **Dependencies** (what must be completed before what)
- **Design decisions** (which ones matter enough to document)

This lesson teaches you how to work with generated plans and how to capture important architectural decisions using **ADRs (Architectural Decision Records)**.

---

## Understanding the `/sp.plan` Command

`/sp.plan` analyzes your specification and generates a detailed implementation plan by:

1. **Breaking spec into components** — Which parts of your spec need separate phases?
2. **Ordering dependencies** — What must be built first?
3. **Identifying design decisions** — Where are there multiple valid approaches?
4. **Proposing architecture** — How should work be organized?

**Input**: Your specification (what the paper must accomplish)

**Output**: Implementation plan with:
- Architecture overview
- Implementation phases
- Component breakdown
- Dependencies and sequencing
- Design decisions highlighted

**The Cascade Effect**: Detailed spec → detailed plan. Vague spec → vague plan.

---

## Part A: Generating Your Implementation Plan

Let's generate the plan for your research paper.

### Step 1: Run `/sp.plan`

In your AI tool, from your `my-research-paper` directory:

```
/sp.plan

Create: architecture sketch, section structure, research approach, quality validation.
Decisions needing documentation: list important choices with options and tradeoffs.
Testing strategy: validation checks based on acceptance criteria.

Technical details:
- Use research-concurrent approach (research while writing, not all upfront)
- Follow APA citation style from Constitution
- Organize by phases: Research → Foundation → Analysis → Synthesis
```

**What the agent does:**
- Creates technical implementation plan
- Defines section structure and dependencies
- Establishes validation strategy
- Identifies architectural decisions
- Generates `plan.md` file in your specs directory

**Why This Matters**: The plan defines architecture for ALL sections at once. This ensures consistency—same research approach, same citation handling, same quality validation. Much more efficient than planning each section separately!

### Step 2: Review Generated Plan

The generated plan should include:

**Architecture Overview:** How work will be organized (phases, dependencies)

**Implementation Phases:** 3-5 phases building from research to polish

**Component Breakdown:** Research management, thesis development, evidence integration, writing phases, quality gates

**Sequencing:** Research before findings? Outline before detailed writing?

**Design Decisions:** Where are there choices? (Research-first vs research-concurrent? Section order?)

### Step 3: Verify Plan Completeness

After the agent generates your plan, review it:

```
Show me the generated plan and explain:
1. How does it break down the work into phases?
2. What dependencies does it identify?
3. What design decisions are highlighted?
4. Does it account for all specification requirements?
```

**Agent shows:**
- **Technical Approach** — Overall strategy for building the paper
- **Major Components** — Logical parts that need to be built
- **Dependencies** — What blocks what
- **Phases** — Milestones with deliverables
- **Success Criteria** — How you'll know each phase is complete

---

## Part B: Understanding ADRs (Architectural Decision Records)

Planning exposes architectural decisions—choices about HOW to build that have long-term consequences.

### What Is an ADR?

An ADR documents:
- **The Decision**: What choice did you make?
- **The Context**: Why did you need to make this choice?
- **The Alternatives**: What other options existed?
- **The Rationale**: Why did you choose this over alternatives?
- **The Consequences**: What are the long-term impacts?

### When Should You Create an ADR?

**Create an ADR when:**
- The decision has long-term impact (affects paper structure, not just word choice)
- Multiple valid alternatives existed (not an obvious choice)
- Future readers/collaborators will question the decision
- The decision constrains future choices (e.g., choosing research approach)

**Don't create ADRs for:**
- Style choices (formatting preferences)
- Obvious choices (of course we use APA—Constitution requires it!)
- Temporary decisions (will revisit before submission)
- Out-of-scope decisions (already decided by Constitution)

### Example ADR Decisions for Research Paper

| Decision | ADR Needed? | Why? |
|----------|-------------|------|
| Research-concurrent vs research-first approach | ✅ YES | Affects entire writing workflow |
| Section ordering (Lit Review before Methodology?) | ✅ YES | Affects logical flow and dependencies |
| APA citation style | ❌ NO | Constitution already decided this |
| Font choice | ❌ NO | Trivial, no long-term impact |
| Whether to include ethics discussion | ✅ YES | Scope decision with tradeoffs |

---

## Part C: Creating ADRs for Your Plan

Now let's identify and document the architectural decisions from your plan.

### Step 1: Run `/sp.adr`

```
/sp.adr Review the generated plan and record key Architectural Decisions.

Focus on decisions that:
1. Affect multiple sections or phases
2. Had alternatives we considered
3. Will shape how we write the paper
4. Someone might question later
```

**What the agent does:**
- Reviews your plan.md
- Identifies architecturally significant decisions
- Creates ADR files in `history/adr/` directory
- Documents context, alternatives, rationale, and consequences

### Step 2: Review Generated ADRs

After the agent creates your ADRs, examine them:

```
Show me the ADRs created. For each one, explain:
1. What decision was documented?
2. What alternatives were considered?
3. Why was this choice made over alternatives?
4. What are the consequences (both positive and negative)?
```

**Example ADR Structure:**

```markdown
# ADR-001: Research-Concurrent Writing Approach

## Status
Accepted

## Context
We need to decide when research happens relative to writing.
Two approaches exist: research-first (gather all sources, then write)
vs research-concurrent (research while writing each section).

## Decision
We will use research-concurrent approach.

## Alternatives Considered
1. **Research-first**: Gather all 8+ sources before writing anything
   - Pro: Complete knowledge before writing
   - Con: Delays writing, may gather irrelevant sources

2. **Research-concurrent**: Research each section as we write it
   - Pro: Research stays focused and relevant
   - Con: May discover knowledge gaps mid-writing

## Rationale
Research-concurrent keeps engagement with material high and ensures
sources are directly relevant to sections being written. Risk of
knowledge gaps is mitigated by outline phase identifying key topics.

## Consequences
- Positive: More focused research, faster initial writing
- Negative: May need additional research passes for cross-section references
- Constraint: Must complete detailed outline before starting section writing
```

### Step 3: Verify ADR Completeness

Check that your ADRs:

```
ADR Completeness Checklist:

[ ] Each ADR has clear context (why this decision was needed)
[ ] Alternatives are documented (not just the chosen option)
[ ] Rationale explains WHY this choice over others
[ ] Consequences include both positives and negatives
[ ] Decision is architecturally significant (not trivial)
[ ] ADR would help future collaborator understand the choice
```

---

## Common Mistakes

### Mistake 1: Documenting Every Small Decision as ADR

**The Error**: Creating ADRs for trivial choices like "Use headings for sections" or "Put references at the end"

**Why It's Wrong**: ADRs are for architecturally significant decisions (long-term impact, multiple alternatives, future questioning). Trivial choices clutter your ADR history.

**The Fix**: Apply the three-part test:
1. Does this have long-term consequences?
2. Are there multiple viable alternatives?
3. Will someone ask "why did we choose this" in 6 months?

If not all three → Skip the ADR.

### Mistake 2: Vague ADR Consequences

**The Error**: ADR says "This approach is better" without explaining tradeoffs

**Why It's Wrong**: Future developers need to understand WHY you chose this and WHAT you gave up.

**The Fix**: Document both positives and negatives:
- ✅ "Pros: More focused research. Cons: May need additional passes."
- ✅ "Alternatives considered: Research-first (rejected: delays writing)"

### Mistake 3: Skipping the Plan Phase

**The Error**: Going straight from specification to task writing, skipping planning.

**Why It's Wrong**: You lose sight of overall architecture. Tasks become disconnected. You discover structural problems late.

**The Fix**: Always plan. The `/sp.plan` command makes this quick and automatic.

---

## Connecting Spec → Plan → Tasks

This is important: your specification, plan, and tasks form a clear chain.

**Specification says**: "Write a 3000-5000 word research paper on AI in K-12 education, APA format, 8+ sources, define 3+ concrete applications"

**Plan says**: "Structure: Research Phase → Foundation Writing → Analysis Writing → Synthesis. Phases in order, dependencies mapped. Key decision: research-concurrent approach."

**Tasks** (next lesson) will say: "Task 1: Define thesis (15 min), Task 2: Outline all sections (30 min), Task 3: Research for Lit Review (45 min)..." etc.

Each level adds specificity:
- **Spec**: What is success? (Measurable criteria)
- **Plan**: How will we organize the work? (Architecture + ADRs)
- **Tasks**: What are the 15-30 minute units? (Atomic work)

---

## Try With AI

Ready to generate your implementation plan and document architectural decisions?

**Generate Your Plan:**

> "I have a research paper specification. Run `/sp.plan` to generate an implementation plan. Show me: (1) The technical approach for structuring the paper, (2) Major components and phases, (3) Dependencies between components, (4) Design decisions that need documenting. Create the plan.md file."

**Analyze Plan Quality:**

> "Review my generated plan at specs/[feature-name]/plan.md. Does it: (1) Match the specification's requirements? (2) Show clear dependencies? (3) Break work into logical phases? (4) Identify parallel work opportunities? Where could the plan be clearer?"

**Create ADRs:**

> "Run `/sp.adr` to review my plan and record key Architectural Decisions. Focus on decisions that: (1) Affect multiple sections, (2) Had alternatives we considered, (3) Will shape how we write the paper. For each ADR, document context, alternatives, rationale, and consequences."

**Test Plan-to-Tasks Readiness:**

> "Based on my plan, simulate breaking it into tasks. For each implementation phase, can you create 3-5 atomic tasks? If you struggle to create clear tasks, identify which parts of my plan are too vague and need more detail. This tests if my plan is detailed enough for the Tasks phase."
