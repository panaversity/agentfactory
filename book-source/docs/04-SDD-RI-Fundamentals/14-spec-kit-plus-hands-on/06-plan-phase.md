---
title: "Plan Phase"
chapter: 14
lesson: 6
duration_minutes: 45
proficiency_level: "B1"
cognitive_load:
  new_concepts: 4

learning_objectives:
  - objective: "Understand that plans bridge specifications to executable tasks"
    bloom_level: "Understand"
    assessment_method: "Student explains plan's role in SDD-RI workflow"

  - objective: "Execute `/sp.plan` command to generate implementation strategy"
    bloom_level: "Apply"
    assessment_method: "Student successfully runs `/sp.plan` and interprets output"

  - objective: "Identify key plan components: approach, components, dependencies, milestones"
    bloom_level: "Understand"
    assessment_method: "Student describes what each plan section addresses"

  - objective: "Recognize how specification quality determines plan clarity"
    bloom_level: "Understand"
    assessment_method: "Student explains cascade effect (clear spec → clear plan)"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Plan Phase

Your specification is complete and clarified. Now comes the bridge between intent and execution: **the plan**.

A plan answers the question: **How will we build what the spec defines?**

It's not code. It's not step-by-step instructions. It's a high-level architecture showing:
- What components you'll build
- How those components fit together
- What dependencies exist between parts
- What the major phases are

In this lesson, you'll learn the `/sp.plan` command—the tool that transforms specifications into actionable implementation strategies.

---

## The Role of a Plan

Before learning the command, let's understand what a plan actually does.

### Plan vs Specification vs Tasks

You now understand three distinct artifacts:

**Specification** (You've completed this):
- Answers: **What are we building?**
- Contains: Intent, constraints, success criteria, non-goals
- Example: "3000-5000 word research paper on AI in K-12 education"

**Plan** (You're learning this):
- Answers: **How will we build it?**
- Contains: Approach, major components, dependencies, phases
- Example: "Paper structure: Intro → Lit Review → Methodology → Findings → Discussion → Conclusion"

**Tasks** (You'll learn next lesson):
- Answers: **What are the atomic work units?**
- Contains: Individual tasks, checkpoints, 15-30 minute units
- Example: "Write intro (define AI, education context, thesis), verify references"

**The progression**: Spec defines WHAT → Plan defines HOW → Tasks define atomic WORK UNITS.

### Why Plans Matter

Without a plan, you jump from specification directly to task writing. This creates problems:

1. **Unclear structure**: Tasks don't connect meaningfully
2. **Hidden dependencies**: You discover Task C needs Task A after starting Task C
3. **Rework**: You finish tasks only to realize the architecture doesn't make sense
4. **Communication**: Team members disagree on overall approach

With a plan, everyone (you and AI) agrees on the high-level strategy before diving into atomic work.

### The Cascade Effect

Here's a critical insight: **Clear specs produce clear plans. Vague specs produce vague plans.**

If your specification is detailed and specific (like "3000-5000 word paper with 8+ sources, APA format, specific scope on K-12 impacts"), the plan will clearly show:
- How to structure sections for that word count
- Which sections require research (lit review) before writing (findings)
- How to manage citations throughout
- What the validation checkpoint looks like (word count, source count, format check)

If your specification is vague ("Write a paper about AI"), the plan will be vague:
- Unclear section structure
- Unclear dependencies
- Unclear success criteria

**Quality flows downstream**: Good spec → Good plan → Manageable tasks. Bad spec → Bad plan → Confusing tasks.

---

## Understanding `/sp.plan` Command

The `/sp.plan` command generates an implementation plan from your specification.

### What the Command Does

`/sp.plan` reads your specification and outputs a plan.md file containing:

**Section 1: Technical Approach**
- High-level strategy for solving the problem
- Key decisions about structure
- Technology/tool choices (if applicable)

**Section 2: Major Components**
- What parts you'll build
- How those parts relate to each other
- What each component's responsibility is

**Section 3: Dependencies**
- Which parts must be completed before others
- Parallel work opportunities (parts that can be done simultaneously)
- Blocking relationships

**Section 4: Implementation Phases**
- Milestones—major checkpoints in the work
- Sequence of phases
- What gets completed in each phase

**Section 5: Success Criteria**
- How you'll know the plan is being executed correctly
- Validation points
- Acceptance criteria for each phase

### How `/sp.plan` Works (Simplified)

When you run `/sp.plan`, the command:

1. **Reads your spec.md** (from the current project)
2. **Analyzes the requirements** (What needs to be built?)
3. **Designs an approach** (How should we structure the solution?)
4. **Identifies components** (What are the logical parts?)
5. **Maps dependencies** (What must happen first?)
6. **Creates phases** (What are the major milestones?)
7. **Generates plan.md** (Outputs the complete plan)

The result is a clear, actionable plan that bridges specification to tasks.

---

## Plan Structure for Your Research Paper

Let's see what a plan looks like for your research paper project.

### Example: Research Paper Plan Structure

Based on your specification (3000-5000 word paper on AI in K-12 education, APA format, 8+ sources), here's what a plan addresses:

**Technical Approach:**
```
Structure: Sequential sections building on each other
- Introduction establishes thesis
- Literature review provides context
- Methodology explains your analytical approach
- Findings present key insights
- Discussion interprets implications
- Conclusion synthesizes learning

Dependency insight: Literature review MUST complete before you can write
Findings and Discussion (you need source material to reference)
```

**Major Components:**
```
1. Research Management: Tracking sources, organizing references, APA format
2. Thesis Development: Defining core argument across sections
3. Evidence Integration: Incorporating research findings throughout
4. Writing Phases: Introduction → Foundations → Analysis → Synthesis
5. Quality Gates: Word count verification, source verification, format checks
```

**Dependencies:**
```
Introduction (establishes context) → Literature Review (builds knowledge)
                                     ↓
Methodology (explains approach) → Findings (presents results)
                                     ↓
Discussion (interprets findings) → Conclusion (synthesizes)

Parallel work possible: You can outline all sections while researching for Lit Review
```

**Implementation Phases:**
```
Phase 1: Research & Outline (Lit review research, outline all sections, gather sources)
Phase 2: Foundation Writing (Introduction, Methodology - provide context)
Phase 3: Analysis Writing (Findings, Discussion - present insights)
Phase 4: Synthesis & Polish (Conclusion, cross-reference check, APA validation)
```

---

## Four Key Plan Concepts

### Concept 1: Approach (Your Strategy)

The approach answers: **What's our overall strategy for solving this problem?**

For a research paper, the approach might be:
- "Sequential writing following thesis development" (write intro first to establish thesis, then build on it)
- "Research-first, then synthesis" (gather all sources first, then weave into paper)
- "Iterative refinement" (draft all sections, then revise for consistency)

Your chosen approach shapes everything downstream.

### Concept 2: Components (What You'll Build)

Components are the logical parts of your solution.

For a research paper:
- Introduction (sets up problem, defines scope, proposes thesis)
- Literature Review (establishes knowledge base, identifies gaps)
- Methodology (explains analytical approach)
- Findings (presents key insights from analysis)
- Discussion (interprets findings, addresses implications)
- Conclusion (synthesizes learning, proposes future work)

Each component has a clear purpose.

### Concept 3: Dependencies (What Must Happen First)

Dependencies show which work blocks other work.

For a research paper:
- You can write your Introduction before completing research (outline the problem)
- But you need to complete research before writing Findings (you need source material)
- You need Findings before Discussion (you must present insights before interpreting them)

Some dependencies are mandatory (you can't write Discussion without Findings). Others are optional (you could research everything first, or research incrementally as you write).

### Concept 4: Milestones (Major Checkpoints)

Milestones mark significant progress points.

For a research paper:
- Milestone 1: Complete outline with thesis defined
- Milestone 2: All primary sources identified and organized
- Milestone 3: Introduction and Methodology drafted
- Milestone 4: Findings and Discussion complete (main content done)
- Milestone 5: Conclusion written and cross-references verified

Each milestone is a pause point where you can review progress.

---

## Executing `/sp.plan`

Now let's see how you actually run this command.

### Running the Command

In Claude Code, from your research-paper project directory:

```
/sp.plan
```

The agent will:
1. Read your specification (from specs/research-paper/spec.md)
2. Analyze your requirements
3. Generate a comprehensive plan
4. Output plan.md in your project

You'll see output showing:
- Technical approach being designed
- Components being identified
- Dependencies being mapped
- Phases being created
- Plan file being generated

### Reviewing the Generated Plan

Once generated, your plan.md should include:

**Structure Overview:**
```markdown
## Technical Approach

[High-level strategy for building the paper]

## Components

[Logical parts: Introduction, Lit Review, Methodology, etc.]

## Dependencies

[What must happen before what]

## Implementation Phases

[Phase 1, Phase 2, Phase 3, Phase 4 with deliverables]

## Success Criteria

[How you'll know you're executing the plan correctly]
```

**Key insight**: Every section of your plan flows directly from your specification. If your spec is clear, your plan will be clear. If your spec is vague, the plan will expose that vagueness.

### Example Generated Plan (Research Paper)

Here's what a real plan output looks like:

```markdown
## Technical Approach

Sequential composition building thesis through evidence.
Introduction establishes problem and thesis. Literature
review builds knowledge foundation. Methodology explains
analytical approach. Findings present discoveries. Discussion
interprets implications. Conclusion synthesizes learning.

Key decision: Research-concurrent approach — gather sources
while writing sections, rather than all-upfront research.
This maintains engagement with material.

## Major Components

1. **Research Management System**: Track sources in APA
   format, organize references, maintain bibliographic data

2. **Thesis Development**: Define core argument in
   Introduction, reinforce across all sections, resolve in
   Conclusion

3. **Evidence Integration**: Weave sources throughout Findings
   and Discussion, ensure proper attribution, cross-reference
   back to Methodology

4. **Writing Phases**: Sequential sections building on each
   other, with clear dependency relationships

5. **Quality Validation**: Word count verification, source
   count check, APA format validation, cross-reference
   consistency

## Dependencies

- Introduction (standalone—no blockers)
- Literature Review (requires source gathering—can start immediately)
- Methodology (requires Introduction complete—needs to reference thesis)
- Findings (requires Lit Review complete—needs source material to analyze)
- Discussion (requires Findings complete—must interpret results)
- Conclusion (requires Discussion complete—must synthesize insights)

Parallel opportunity: Outline all sections and begin source gathering immediately

## Implementation Phases

**Phase 1: Research & Structure (Days 1-2)**
- Define thesis clearly
- Outline all sections
- Identify 8+ sources
- Organize in APA format
- Define success metric (3000-5000 words, specific breakdown per section)

**Phase 2: Foundation Writing (Days 3-4)**
- Write Introduction (establish problem, propose thesis)
- Write Methodology (explain analytical approach)
- Define cross-section references

**Phase 3: Analysis Writing (Days 5-6)**
- Write Literature Review (synthesize source knowledge)
- Write Findings (present key insights)
- Write Discussion (interpret implications)

**Phase 4: Synthesis & Polish (Days 7)**
- Write Conclusion (synthesize learning)
- Verify word count and source count
- Cross-reference consistency check
- APA format validation
- Final review
```

---

## Common Mistakes

### Mistake 1: Confusing Plan with Task List

**The Error**: Treating a plan like a task list—listing every small step instead of major components.

**Why It's Wrong**: A plan shows architecture and strategy. A task list shows atomic work units. Mixing them up creates confusion about what layer you're working at.

**The Fix**: Ask: "Is this a major component/phase, or a small task?" If it's small, it's probably a task, not part of the plan.

### Mistake 2: Not Following Dependencies

**The Error**: Creating a plan where Findings come before Literature Review.

**Why It's Wrong**: You can't find insights if you haven't reviewed the research. Plan dependencies should reflect real work flow.

**The Fix**: Ask: "What must I know before starting this component?" If Lit Review is required knowledge, it must come first in the plan.

### Mistake 3: Skipping the Plan

**The Error**: Going straight from specification to task writing, skipping planning.

**Why It's Wrong**: You lose sight of overall architecture. Tasks become disconnected. You discover structural problems late.

**The Fix**: Always plan. The `/sp.plan` command makes this quick and automatic.

---

## Connecting Spec to Plan to Tasks

This is important: your specification, plan, and tasks should form a clear chain.

**Specification says**: "Write a 3000-5000 word research paper on AI in K-12 education, APA format, 8+ sources, define 3+ concrete applications"

**Plan says**: "Structure: Research Phase → Foundation Writing → Analysis Writing → Synthesis. Phases in order, dependencies mapped"

**Tasks** (next lesson) will say: "Task 1: Define thesis (15 min), Task 2: Find 8 sources (45 min), Task 3: Outline all sections (30 min)..." etc.

Each level adds specificity:
- **Spec**: What is success? (Measurable criteria)
- **Plan**: How will we organize the work? (Architecture)
- **Tasks**: What are the 15-30 minute units? (Atomic work)

---

## Try With AI

Ready to generate your implementation plan?

**Setup**: You have a completed and clarified specification for your research paper at `specs/research-paper/spec.md`. Now you'll generate the plan.

**💬 Prompt 1: Generate Your Plan (Core Learning)**

```
I have a research paper specification at specs/research-paper/spec.md.
Run /sp.plan to generate an implementation plan. Show me:

1. The technical approach for structuring the paper
2. Major components (sections, phases)
3. Dependencies between components
4. Implementation phases with sequencing
5. Success criteria for each phase
```

**Expected Outcome**: A plan.md file with clear sections showing HOW you'll build the paper, organized by major components and phases.

**💬 Prompt 2: Analyze Plan Quality (Validation)**

```
Review my generated plan at specs/research-paper/plan.md.
Does it:
- Match the specification's requirements?
- Show clear dependencies (what must come first)?
- Break the work into logical phases?
- Identify parallel work opportunities?

Where could the plan be clearer? What's missing?
```

**Expected Outcome**: Feedback on plan completeness and clarity. If plan is vague, this signals that your specification might need refinement.

**💬 Prompt 3: Trace Requirements to Plan (Mapping)**

```
Take each requirement from my specification:
1. Paper must be 3000-5000 words
2. Must include 8+ academic sources
3. Must cover 3+ concrete AI applications
4. Must follow APA format

For each requirement, show me where it appears in my plan.
Which phase ensures this requirement is met? How will success be verified?
```

**Expected Outcome**: Ability to trace each spec requirement to its corresponding plan component. This ensures nothing fell through the cracks.

**💬 Prompt 4: Identify Critical Decisions (Architect Thinking)**

```
My plan makes these key decisions:
- When to research vs. write (concurrent vs. all-upfront)
- Whether to draft all sections before refining
- How to handle cross-section references

For each decision, what are the tradeoffs? What would happen
if we made a different choice? Which decisions are most
architecturally significant?
```

**Expected Outcome**: Understanding that planning isn't just about structure—it's about making strategic decisions about HOW the work will happen.

---
