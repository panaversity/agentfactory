---
title: "Explore the Tools: Which SDD Framework Fits YOUR Work?"
chapter: 30
lesson: 6
duration: "3-3.5 hours"
skills:
  - name: "Tool Evaluation"
    proficiency: "B1"
    category: "Technical"
  - name: "Framework Comparison"
    proficiency: "B1"
    category: "Conceptual"
  - name: "Decision Making"
    proficiency: "B1"
    category: "Soft"
learning_objectives:
  - "Compare three SDD frameworks: Kiro, Spec-Kit, and Tessel (B1)"
  - "Evaluate strengths and limitations of each framework (B1)"
  - "Select the right SDD framework for a given project context (B1)"
---

# Explore the Tools: Which SDD Framework Fits YOUR Work?

## Three Philosophies, One Goal

Three main SDD tools emerged in 2024-2025. Each represents a different philosophy about how much structure you need.

**This lesson**: Explore each with your companion and figure out which fits YOUR context.

---

## Tool 1: Kiro — "Keep It Simple"

Ask your companion:

```
Tell me about Kiro. What's its philosophy? What's the workflow?
How is it different from Spec-Kit?
```

Your companion will explain:

> "Kiro's philosophy: SDD shouldn't require learning complex processes.
>
> Workflow: Three documents, three phases
>
> 1. REQUIREMENTS
>    - Write user stories (As a user, I want...)
>    - Add acceptance criteria (GIVEN/WHEN/THEN)
>    - Familiar to Agile teams
>
> 2. DESIGN
>    - One document with architecture sections
>    - Components, data models, decisions
>    - Single file, easy to review
>
> 3. TASKS
>    - Numbered tasks with acceptance criteria
>    - One by one, trackable
>
> Strengths: Low learning curve, Agile-familiar, simple
> Limitations: Less structured enforcement, might feel light for complex systems"

**When to use Kiro**:
- ✅ New to SDD (low learning curve)
- ✅ Medium features (not massive systems)
- ✅ Agile-experienced team
- ✅ Want lightweight approach

**When NOT to use Kiro**:
- ❌ Large team (needs consistency enforcement)
- ❌ Complex system (needs comprehensive planning)
- ❌ Regulated domain (needs strong governance)

---

## Tool 2: Spec-Kit — "Governance First"

Ask your companion:

```
Now tell me about GitHub's Spec-Kit. What's its philosophy?
How does it differ from Kiro?
```

Your companion explains:

> "Spec-Kit's philosophy: Strong governance through immutable principles.
>
> Foundation: CONSTITUTION
> - Document of rules that apply to EVERYTHING
> - Examples: 'All passwords use bcrypt,' 'Test coverage 80%+'
> - Non-negotiable
>
> Workflow: Specify → Plan → Tasks (repeatable)
>
> 1. SPECIFY: What are we building?
>    - Requirements, acceptance criteria, edge cases
>    - Check for Constitution violations BEFORE code
>    - Output: Spec document + checklist
>
> 2. PLAN: How will we build it?
>    - Architecture, technology decisions, risk assessment
>    - Verify Constitutional alignment
>    - Output: Plan document + checklist
>
> 3. TASKS: What are the work units?
>    - Atomic tasks, acceptance criteria
>    - Each task traces to requirements
>    - Output: Task breakdown + checklist
>
> Additional: ADRs (Architecture Decision Records) document 'why'
>
> Strengths: Strong governance, comprehensive, scalable, traceability
> Limitations: Steep learning curve, heavy review burden, can feel like
> overkill for small features"

**When to use Spec-Kit**:
- ✅ Large team (need consistency)
- ✅ Complex system (need traceability)
- ✅ Regulated domain (need governance)
- ✅ Long-lived project (maintenance matters)

**When NOT to use Spec-Kit**:
- ❌ Learning SDD (steep curve)
- ❌ Small features (too heavy)
- ❌ Rapid iteration (Constitution is constraint)

---

## Tool 3: Tessl — "Specs Are Code"

Ask your companion:

```
What about Tessel Framework? What's its ambitious vision?
What does 'spec-as-source' actually mean?
```

Your companion explains:

> "Tessel's vision: Specs are the ONLY artifact developers edit.
> Code is generated, never hand-edited.
>
> Workflow:
> 1. Write specification (human edited)
> 2. Run code generator (automatic)
> 3. Code produced (marked: DO NOT EDIT)
> 4. Later: Update spec (not code)
> 5. Regenerate code (automatic)
>
> Promise: Specs stay current, code always correct, no technical debt
>
> Challenge: Non-determinism (run generator twice → different code?)
> Challenge: Incomplete specs (natural language might miss edge cases)
> Challenge: Performance (generated code might not be optimized)
>
> Status: Private beta 2025, still experimental
>
> When it works: Safety-critical, regulated, stable domains
> When it fails: Rapid iteration, changing requirements, optimization-heavy"

**When Tessel might work**:
- ✅ Medical device software
- ✅ Financial systems
- ✅ Aerospace/defense
- ✅ Regulatory audit requirement

**When Tessel doesn't work**:
- ❌ Startup with changing requirements
- ❌ Performance-critical system
- ❌ Experimental/exploratory work

---

## Decision Matrix: Which Tool for YOU?

Ask your companion to help you answer:

```
Help me figure out which tool fits my context:
- Am I solo or on a team?
- How complex is my system?
- Is this a regulated domain?
- How often do requirements change?
- Do I need strong governance?
```

Your companion will create a matrix:

| Question | Suggests Kiro | Suggests Spec-Kit | Suggests Tessel |
|----------|---|---|---|
| Solo developer? | ✅ | ❌ | ❌ |
| Team of 5? | ✅ | ✅ | ❌ |
| Team of 20+? | ❌ | ✅ | Consider |
| Complex system? | ⚠️ | ✅ | Maybe |
| Regulated domain? | ❌ | ✅ | ✅ |
| Changing requirements? | ✅ | ✅ | ❌ |
| Performance-critical? | ✅ | ✅ | ❌ |

---

## What This Book Teaches: Spec-Kit Plus

Ask your companion:

```
This book mentions Spec-Kit Plus. What's that? How is it different
from regular Spec-Kit?
```

Your companion will explain:

> "Spec-Kit Plus is a variant of GitHub's Spec-Kit, customized for:
>
> 1. AI-native development (works with Claude, Gemini, ChatGPT)
> 2. Pedagogical clarity (designed for learning)
> 3. PHRs (Prompt History Records) - logging AI interactions
> 4. ADRs (Architectural Decision Records) - documenting 'why'
> 5. Skills (domain-specific extensions)
> 6. Subagents (specialized AI agents)
>
> It takes Spec-Kit's governance philosophy and adapts it for
> modern AI-integrated development."

**Why Spec-Kit Plus for this book:**
- Balances structure and simplicity
- Enables scaling from solo to teams
- Works with AI companions
- Teaches professional practice

---

## Your Decision: Which Tool for YOUR Next Project?

**Exercise**: Answer these questions honestly:

1. **Team size**: Am I solo or on a team?
2. **Complexity**: Is my next project small, medium, or complex?
3. **Governance need**: Do I need strong architectural enforcement?
4. **Domain**: Is this regulated or standard?
5. **Requirements**: Stable or changing?

Based on answers:

- **Mostly solo + simple + changing** → Try Kiro (lightweight)
- **Team + medium complexity + stable** → Try Spec-Kit Plus (balanced)
- **Large team + complex + regulated** → Spec-Kit Plus (governance)
- **Medical/finance + stable** → Consider Tessel (spec-as-source)

---

## Key Takeaway

Different tools for different contexts. No single "best" tool.

- **Kiro**: Start simple, scale up if needed
- **Spec-Kit Plus**: Balanced governance and simplicity
- **Tessel**: Ambitious, narrow domains

Pick the tool that matches YOUR context, not the other way around.

---

## Next Steps

You now understand:
- ✅ What SDD is (Lesson 2)
- ✅ How to build a spec (Lesson 3)
- ✅ How to scale specs (Lesson 4)
- ✅ Why specs matter NOW (Lesson 5)
- ✅ Which tools fit your context (Lesson 6)
- Still to learn:
  - The future (Lesson 7: Spec-as-source, MDD lessons)
  - Your commitment (Lesson 8: Manifesto)

---

**Next Lesson**: The future of SDD. Where is all this heading? What comes next?
