---
title: "Planning the How from Specification"
chapter: 31
lesson: 4
duration_minutes: 90

skills:
  - name: "Plan Generation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student uses /sp.plan; reads plan structure; explains plan phases and dependencies"

  - name: "Cascade Understanding"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student traces requirement → plan phase and articulates how spec quality improved plan quality"

  - name: "Dependency Analysis"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student reads plan; identifies which phases must complete before others; explains rationale"

learning_objectives:
  - objective: "Use /sp.plan to generate implementation plans from specifications and understand plan dependencies"
    proficiency_level: "B1"
    bloom_level: "Apply, Analyze"
    assessment_method: "Given a refined specification, student uses /sp.plan, reads plan, explains phase structure and dependencies, traces requirement to plan"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Plan vs. Spec, Plan Structure, Cascade Quality, Dependencies, Milestones, Deliverables, Feedback Loop) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Analyze plans for parallel opportunities; design risk mitigation strategies for dependent phases"
  remedial_for_struggling: "Simplified phase visualization; guided dependency mapping template"
---

# Planning the How from Specification

You have a clear, refined specification. Excellent. Now you need to answer: **How will we build this? What are the phases? What must happen first, what next, what last?**

A specification answers "What?" and "Why?" A plan answers "How?" and "When?"

`/sp.plan` takes your specification and generates an implementation plan: phases, dependencies, milestones, risks. This lesson shows you how to use it and understand the plans it produces.

The key insight: **Clear specifications produce clear plans with explicit dependencies. Vague specifications produce confusing plans with missing phases.**

---

## Specification vs. Plan

| Specification | Plan |
|---|---|
| **Answers**: What to build? Why? | **Answers**: How to build? When? |
| **Describes**: Requirements, acceptance criteria, success measures | **Describes**: Phases, dependencies, milestones |
| **Purpose**: Contract with implementer | **Purpose**: Roadmap for development |
| **Audience**: Everyone (stakeholders, developers, AI) | **Audience**: Development team |
| **Length**: Typically 3-5 pages | **Length**: Typically 5-10 pages (more detailed) |

---

## Generate Plan with /sp.plan 

### Running the Command

Like `/sp.specify`, `/sp.plan` runs **within AI Companion**:

1. Open AI Companion
2. Paste your refined specification (from Lesson 4)
3. Run **Your Prompt:**

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

---

## Try With AI

### Workflow

1. **Run `/sp.plan`** on your refined specification
2. **Review the generated plan**: Read all phases
3. **Ask**: "What are the critical dependencies in this plan?"
4. **Ask**: "Can I parallelize any phases? Which ones and why?"
5. **Trace a requirement**: "How does [requirement from spec] appear in the plan?"