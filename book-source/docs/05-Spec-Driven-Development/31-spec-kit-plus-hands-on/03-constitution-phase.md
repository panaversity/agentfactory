---
title: "Constitution Phase - Project-Wide Rules"
chapter: 31
lesson: 3
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Creating Project Constitution"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write project Constitution defining quality, testing, error-handling standards"

  - name: "Distinguishing Global Rules from Feature Requirements"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain difference between Constitution (applies to ALL features) and Specification (applies to ONE feature)"

  - name: "Understanding Cascade Starting Point"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes that Constitution quality determines all downstream quality"

  - name: "Defining Quality Standards"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can define testable quality criteria (code style, type hints, error handling, test coverage)"

  - name: "Git Workflow Integration"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student understands Constitution → Commit pattern (foundation before feature work)"

learning_objectives:
  - objective: "Write a project Constitution defining quality, testing, and error-handling standards for calculator project"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Constitution document review against template completeness and clarity"

  - objective: "Explain why Constitution is created once per project, not per feature"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Written or verbal explanation of Constitution scope"

  - objective: "Understand how Constitution quality cascades through all downstream phases"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Identification of how Constitution rules affect specifications and plans"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Constitution role, Global rules vs feature specs, Cascade starting point, Quality standards, Git workflow) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Write Constitution for 2-3 different hypothetical projects (CLI tool, web API, data pipeline); compare how rules differ by project type"
  remedial_for_struggling: "Use provided Constitution template; fill in only essential sections (quality, error handling, testing) before moving to Lesson 3"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Constitution Phase - Project-Wide Rules

Welcome to the second lesson of hands-on Spec-Kit Plus development. You've installed the framework and configured your AI tool. Now it's time to create the **foundational rules** that will guide every part of your calculator project.

The Constitution is Spec-Kit Plus's answer to a critical question: **What standards apply to every piece of work you do?** Not just for this feature, but for all features. Not just this week, but for the life of the project.

Imagine you and your computer helper are a team building a giant LEGO castle. Before you start, you need to agree on some rules so you don't mess things up.

*   What if you want all the towers to be **square**, but your helper starts building **round** ones?
*   What if you decide the roof must be **blue**, but your helper builds a **red** one?

That would be a mess!

The **Constitution** is your team's **Rulebook**. It lists the most important rules that both you and your computer helper **MUST** follow, no matter what. It makes sure you both build the project the exact same way, every single time.

---

## What Is a Constitution? 

Before writing one, let's understand what a Constitution actually is and why it matters.

### Constitution: Global Rules, One Per Project

A **Constitution** is a document that defines **immutable standards** applying to **all features** in a project. It's distinct from a **Specification**, which applies to **one feature**.

**Constitution applies to**:
- Code quality standards (type hints, docstrings, naming conventions)
- Testing requirements (unit tests, integration tests, coverage targets)
- Error handling patterns (exception hierarchy, error messages, logging)
- Security practices (no hardcoded secrets, input validation, data handling)
- Documentation expectations (README, code comments, docstrings)

**Specification applies to**:
- Specific feature requirements (calculator should add, subtract, multiply, etc.)
- User stories for that feature
- Acceptance criteria for that feature
- Edge cases specific to that feature

**Example**:

```
CONSTITUTION (applies to ALL features):
  ✅ "All functions must have type hints"
  ✅ "All code must be 100% test-covered"

SPECIFICATION (applies only to CALCULATOR feature):
  ✅ "Calculator must support add, subtract, multiply, divide, power"
  ✅ "Power operation handles negative exponents"
  ✅ "Results return float with 6-decimal precision"
```

### Why Constitution Matters: The Cascade

The Constitution is the **starting point of the cascade**:

```
Clear Constitution
    ↓
(ensures that every spec follows quality standards)
    ↓
Clear Specification
    ↓
(ensures that planning is feasible)
    ↓
Clear Plan
    ↓
(ensures that tasks are well-defined)
    ↓
Clear Tasks
    ↓
(enables correct code generation)
    ↓
Working Code
```

**Weak Constitution** produces:
- Specs that are vague about error handling
- Plans that don't account for testing
- Code that's missing type hints
- Integration issues because standards weren't enforced upstream

**Strong Constitution** produces:
- Specs that automatically include quality criteria
- Plans with testing built in
- Code that's consistent and maintainable
- Integration that works because standards were clear from the start

### Constitution is One-Time, Feature Work is Repetitive

This is crucial: You write the Constitution **once per project**. Then, for each feature, you:

1. Write a specification (addressing this feature only)
2. Generate a plan
3. Generate tasks
4. Implement code

But you never rewrite the Constitution for each feature. It's the foundation everything builds on.

**Best Practice Pattern**:

```
1. Initialize project
2. Write Constitution
3. Commit Constitution to git
4. FOR EACH FEATURE:
   - Run /sp.specify (new specification)
   - Run /sp.clarify (refine specification)
   - Run /sp.plan (new plan for this feature)
   - Run /sp.tasks (new tasks for this feature)
   - Run /sp.implement (new code for this feature)
   - Commit feature to git
```

#### 💬 AI Colearning Prompt
> "Why does the Constitution cascade its quality to all downstream phases? What would happen if I wrote a vague Constitution but tried to write a precise specification?"

---

## Reading Existing Constitutions (15 minutes)

Before writing your own, let's look at base Constitution file. Open:

```bash
.specify/memory/constitution.md
```

Remember the Goal: document the non-negotiable principles that every spec, plan, and task must honor.

**The Key Insight**: Constitutions are project-specific. Your calculator Constitution would never mention "data lineage" because that's not relevant. A data pipeline Constitution wouldn't need type hints or power operations.

#### 🎓 Expert Insight
> In AI-native development, the Constitution isn't bureaucracy—it's leverage. Write it once with clear, testable standards ("all functions have type hints"), and every AI-generated spec, plan, and implementation automatically inherits those standards. Vague Constitutions produce vague downstream work. Precise Constitutions produce precise downstream work. The 30 minutes you invest here saves hours of rework later.

---

## Part B: Writing Your Calculator Constitution

Now let's write YOUR Constitution for the calculator project.

### Step 1: Create the Constitution

In your project directory

1. Start agent chat type `/sp.constitution`.
2. Now we have to explain our requirements i.e::
```
/sp.constitution 

Project principles and standards:
- Write tests first (TDD approach)
- Use Python 3.12+ with type hints everywhere
- Keep code clean and easy to read
- Document important decisions with ADRs
- Follow essential OOP principles: SOLID, DRY, KISS

Technical stack:
- Python 3.12+ with UV package manager
- pytest for testing
- Keep all project files in git

Quality requirements:
- All tests must pass
- At least 80% code coverage
- Use dataclasses for data structures
```

3. Agent Does:
  - Creates comprehensive constitution file
  - Sets up project standards and OOP principles
  - Defines coding principles and best practices
  - Establishes technical preferences and quality gates

### Step 2: Improve your Constitution

Think about what "good code" means for a calculator project:

```markdown
Update @.specify/memory/constitution.md to improve Code Quality Standards

- All functions must include type hints on parameters and return types
  - Example: `def add(a: float, b: float) -> float:`
- All functions must include docstrings explaining what they do
  - Example: `"""Add two numbers and return the sum."""`
- Follow PEP 8 naming conventions (lowercase_with_underscores for functions)
- Lines must be under 100 characters
- No magic numbers; use named constants
  - Bad: `if x > 10:`
  - Good: `if x > MAX_POWER_EXPONENT:`
```

**Try writing 3-4 quality standards for your calculator. Think about**:
- How will you know if code is "good"?
- What would "bad" code look like?
- What would make code hard to maintain?

### Step 3: Review and Complete Your Constitution 

**Your Prompt:**

```
Show me the generated constitution file and explain what it contains.
```

**Agent Does:**

- Displays the constitution content
- Explains each section:
  - **Project Principles** - Core development philosophy and OOP principles
  - **Technical Standards** - Code quality, structure, and best practices
  - **Testing Requirements** - TDD and coverage standards
  - **Architecture Decisions** - When to create ADRs and design patterns
  - **Quality Gates** - CI/CD requirements and code quality checks
  
---

## Part C: Commit Constitution to Git (15 minutes)

Here's a critical best practice: **Always commit the Constitution before starting feature work.**

### Why Commit First?

1. **Immutability**: Constitution is foundational; committing it signals "this is our standard"
2. **Clarity**: Everyone (including your AI orchestrator) sees the Constitution as the baseline
3. **Traceability**: Git history shows when and why Constitution was created
4. **Reversibility**: If you need to, you can revert to a previous Constitution (rarely happens, but important)

### Commit Steps

**Your Prompt**

Use the agent to commit and open a PR for the constitution:

```
/sp.git.commit_pr Commit and push the constitution along with current work.
```

**Agent Does:**

- Create a conventional commit for the constitution and push to a new feature branch
- Create a draft PR (or share the compare URL if `gh` auth is missing)


The Constitution is now **the foundation** for all your feature work. Every specification you write, every plan you generate, every task you break down-they all work within the Constitution's constraints.

#### 🤝 Practice Exercise

> **Ask your AI**: "I've created my calculator Constitution with standards for type hints, testing, and error handling. Can you review `.specify/memory/constitution.md` and tell me: (1) Are my quality standards testable or vague? (2) Did I miss any critical standards for a calculator project? Then explain how these standards will cascade to my specifications."

**Expected Outcome**: Your AI should identify whether standards like "all functions have type hints" are testable (✅) versus vague phrases like "good quality" (❌), suggest any missing calculator-specific standards, and explain how the Constitution influences spec-writing.

---

## Common Mistakes

### Mistake 1: Copying Constitution from Another Project Without Customization

**The Error**: "I'll just use the example Constitution as-is for my calculator."

**Why It's Wrong**: Constitutions are project-specific. A data pipeline Constitution mentions "data lineage" - irrelevant for calculators.

**The Fix**: Read example Constitutions for structure, but write rules specific to YOUR project needs.

### Mistake 2: Vague Quality Standards

**The Error**: "Code must be good quality" or "Tests should be comprehensive"

**Why It's Wrong**: "Good" and "comprehensive" are subjective. No one can verify these during code review.

**The Fix**: Use testable criteria:
- ❌ Vague: "Good code quality"
- ✅ Testable: "All functions have type hints; lines under 100 characters; no magic numbers"

---

## Try With AI: Validate Your Constitution

Now let's use your AI companion to review your Constitution and ensure it's clear enough that specs, plans, and code can build on it without ambiguity.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)
**Goal**: Ensure Constitution is clear, testable, and specific enough to guide downstream work

### Prompt Set (Copy-Paste Ready)

**Prompt 1 - Constitution Clarity Check**

Copy and paste this into Claude Code:

```
I've written a Constitution for my calculator project. Please review it for clarity:

1. Are all rules testable? (Can you verify during code review?)
2. Are any rules vague or ambiguous?
3. Do I cover the essential categories (quality, error handling, types, testing)?

Here's my Constitution:
@.specify/memory/constitution.md

Please identify any rules that need clarification.
```

**Prompt 2 - Cascade Validation**

After reviewing clarity, ask:

```
Now imagine I'm writing a specification for the calculator's divide operation.
Looking at my Constitution, what constraints must my spec respect?

- What error handling must the divide spec include?
- What type system rules must divide follow?
- What testing requirements must divide meet?

This helps me understand if my Constitution is specific enough to guide specs.
```

**Prompt 3 - Reality Check**

Finally, ask:

```
Is my Constitution realistic for a calculator project, or am I being too strict?
Any advice on making it more practical without sacrificing quality?
```

### Expected Outcomes

After these prompts, you should understand:

- **Constitution clarity**: Rules are testable and specific (not vague philosophy)
- **Cascade impact**: You can trace how Constitution rules flow into specifications
- **Realism**: Standards are ambitious but achievable
- **Readiness**: You're prepared to write specifications that respect the Constitution