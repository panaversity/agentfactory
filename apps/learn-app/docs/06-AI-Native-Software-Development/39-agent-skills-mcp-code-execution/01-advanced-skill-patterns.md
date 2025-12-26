---
sidebar_position: 1
title: "Advanced Skill Patterns"
description: "Master the core components that make execution skills autonomous: persona design, decision-making frameworks, and behavioral principles that encode domain expertise into code."
keywords: ["skill persona", "decision-making questions", "execution principles", "skill composition", "autonomous behavior", "domain expertise"]
chapter: 39
lesson: 1
duration_minutes: 25

# HIDDEN SKILLS METADATA
skills:
  - name: "Execution Skill Persona Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can articulate why execution skill personas (code orchestrators) differ from advisory skill personas (advice-givers) and design personas that activate appropriate domain reasoning"

  - name: "Decision-Making Framework for Skills"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write testable decision-making questions that encode autonomous behavior, not vague guidance"

  - name: "Principle-Driven Skill Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can articulate principles that constrain skill behavior (safety, efficiency, convergence) and enable autonomous operation"

learning_objectives:
  - objective: "Identify core components of execution skills (persona, questions, principles) and explain how each contributes to autonomous behavior"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Analysis of existing execution skills; design of skill components for student's domain"

  - objective: "Design skill personas that activate appropriate domain expertise and distinguish execution (code orchestration) from advisory (recommendation) roles"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Persona design exercise; comparison with reference patterns"

  - objective: "Write decision-making questions that encode skill autonomy (context analysis, judgment) rather than implementation steps"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Question design exercise; validation against testability criteria"

  - objective: "Articulate principles that guide skill execution (convergence criteria, safety constraints, efficiency heuristics)"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Principle articulation for domain-specific skill"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (persona, persona patterns, questions framework, principles, composition, references, reusability, design decisions) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Analyze how persona/principles scale across different domain problems. Design a skill that depends on another skill (skill composition). Explore how execution skills differ from advisory skills used in earlier chapters."
  remedial_for_struggling: "Focus on one concrete example of advisory vs execution skill. Use the reference skill (fetching-library-docs) as template. Practice writing ONE clear decision question. Articulate ONE principle that guides behavior."
---

# Advanced Skill Patterns

You've already built basic skills in Chapter 5. You understand the Persona + Questions + Principles framework. But there's a critical distinction between skills you've created so far and the skills this chapter will focus on—one that changes how you design for autonomous execution.

Consider this scenario: You've built an advisory skill that says "You are a Python expert. Answer questions about Python best practices." Students ask questions, the skill recommends approaches. Simple advice-giving.

Now imagine a different skill: "You are a Python code orchestrator. Watch for problems requiring multi-step code generation. Estimate execution complexity. Generate code, run it, analyze output. If partial success, refine code and retry until complete." This skill doesn't advise—it acts. It orchestrates code execution autonomously.

This is the jump from **advisory** to **execution** skills. And it requires rethinking every aspect of how you design: persona, questions, and principles become leverage points for autonomous behavior instead of helpful guidance.

## The Execution Shift: Advisory vs Execution Skills

### What Advisory Skills Do

Advisory skills answer questions and offer recommendations. The human makes the final decision.

**Example**: The `fetching-library-docs` skill from Chapter 5 is advisory:
- **Persona**: "You are a Python documentation specialist"
- **Questions**: "What's the user's core need? Which modules are relevant?"
- **Principles**: "Be thorough but concise. Only suggest modules the user asked about."
- **Role**: Recommends which documentation to fetch
- **Human's role**: Evaluates recommendation, decides whether to adopt

The skill provides intelligent guidance. The human is the executor.

### What Execution Skills Do

Execution skills orchestrate code, make decisions, and iterate toward results. They are autonomous agents that act, not just advise.

**Execution Example** (this chapter's focus): A skill that processes data autonomously:
- **Persona**: "You are a data orchestrator: Identify data problems, generate processing code, execute it, validate results. Iterate if partial success."
- **Questions**: "What's the input data structure? What transformations matter? What edge cases exist? Are results complete?"
- **Principles**: "Fail safe: Validate data before processing. Converge deliberately: Iterate max 3 times. Log all transformations."
- **Role**: Generates code, executes it, analyzes results, retries on failure
- **Human's role**: Provides initial specification, monitors execution, validates final output

The skill orchestrates the full execution loop. The human sets direction; the skill handles execution.

### The Critical Difference: Persona

The **persona is the core difference** between advisory and execution skills.

**Advisory Persona** (from Chapter 5):
```
"You are a Python best practices expert"
```

This activates recommendation mode. The skill analyzes context and suggests what you should do. It's passive regarding execution—the human must implement suggestions.

**Execution Persona** (this chapter):
```
"You are a Python execution orchestrator: Watch for code-generation tasks.
For each task: (1) Write code matching spec, (2) Execute code, (3) Analyze output,
(4) If partial success, refine and retry, (5) Log all steps"
```

This activates orchestration mode. The skill sees a task and autonomously manages the full execution cycle. The human watches; the skill operates.

Notice the difference:
- Advisory: "You are an expert..." → Provides recommendations
- Execution: "You are an orchestrator: watch for X, do Y, analyze Z, iterate W..." → Performs workflow

The execution persona explicitly describes the workflow the skill manages. That workflow description becomes the basis for decision-making questions and behavioral principles.

## Designing Questions for Autonomous Decision-Making

Advisory skills ask questions like "What does the user need?" Execution skills ask different questions—questions that encode decision logic about **when, how, and whether** to act.

### Three Types of Questions

**Type 1: Context Analysis Questions** (What's the situation?)
These questions help the skill understand the problem space before acting.

**Good Example**:
```
"Is this a code generation task or a documentation task?
What constraints exist on execution (time, resources, data sensitivity)?
What indicates 'success' for this particular task?"
```

Why these work: They force analysis of context. The skill can't answer them without understanding the problem deeply. The skill's decisions will depend on these answers.

**Bad Example**:
```
"What should I do?"
"Is this important?"
"Should I try my best?"
```

Why these fail: They're vague. No meaningful analysis. The skill ends up guessing.

**Type 2: Convergence Questions** (When should I stop?)
Execution skills iterate. These questions help determine when iteration has achieved enough.

**Good Example**:
```
"Does the output match the specification completely?
Are there edge cases the code doesn't handle?
What is the minimal additional iteration needed to reach completeness?"
```

Why these work: They force evaluation of output quality against specification. The skill can decide whether to iterate or declare success.

**Bad Example**:
```
"Is the output good enough?
Should I keep trying?"
```

Why these fail: "Good enough" is undefined. The skill will keep iterating endlessly or stop prematurely.

**Type 3: Safety/Constraint Questions** (What should I NOT do?)
Execution skills operate autonomously, so constraints are critical.

**Good Example**:
```
"Does this operation access restricted filesystems or APIs?
Are there rate limits I should respect?
What should happen if execution would exceed resource constraints?"
```

Why these work: They identify explicit boundaries. The skill knows what's forbidden before attempting operations.

**Bad Example**:
```
"Is this safe?"
"Should I be careful?"
```

Why these fail: Vague. The skill will operate without clear constraints.

## Principles That Enable Autonomous Behavior

Principles are the behavioral guidelines that govern how a skill operates when it encounters decisions. Unlike questions (which are analytical), principles are directives that constrain choices.

### Structure of Effective Principles

An effective principle has three components:

1. **Constraint** (what's the boundary?)
2. **Reason** (why does this boundary matter?)
3. **Application** (how do you apply this in practice?)

**Example Principle**:

```
Principle: "Fail Safe: Always validate data before processing"

Constraint: Before any code executes against input data, validate structure and content
Reason: Processing malformed data produces incorrect results silently (subtle bugs)
Application: "Every script starts with data validation assertions. If validation fails,
            log and halt rather than attempting recovery"
```

This principle tells the skill: "When you see data, validate it first. Here's why. Here's how you know you've done it right."

### Three Categories of Principles for Execution Skills

**Category 1: Convergence Principles** (How do you know you're done?)

These prevent infinite loops and define success criteria.

**Example**:
```
Principle: "Converge Deliberately: Stop iterating when spec is met or max iterations reached"

Constraint: Skill will retry failed code generation at most 3 times per task
Reason: Infinite iteration indicates a problem that won't be solved by retry (e.g., spec is unclear). Three iterations is enough to fix transient errors; more suggests fundamental issue
Application: After each iteration, check: "Does output match spec? If yes, declare success.
            If no, has this specific error occurred before? If yes, stop and flag for human review"
```

**Category 2: Efficiency Principles** (How do you minimize wasted effort?)

These optimize execution for speed and resource efficiency.

**Example**:
```
Principle: "Filter Ruthlessly: Keep only essential data in context"

Constraint: Don't load all available data. Load only what the current task requires.
Reason: Token limits and execution speed. Irrelevant data fills context without value.
Application: "Before loading data, ask: 'What's the minimum data this step needs?'
            Load that subset. When the subset is processed, load the next subset."
```

**Category 3: Safety Principles** (What should never happen?)

These prevent dangerous operations and define guardrails.

**Example**:
```
Principle: "Sandbox First: Validate before Executing"

Constraint: Never execute generated code against production data on first run
Reason: Generated code may have bugs. Testing against production risks data corruption
Application: "All generated code executes in isolated sandbox first with test data.
            Review execution results. Only after validation, run against real data"
```

## Skill Composition: How Skills Depend on Skills

As skills become more sophisticated, some skills depend on other skills. A data processing skill might depend on a data validation skill. A code execution skill might depend on a syntax error recovery skill.

This creates a **skill dependency graph**. Understanding how to design skills that compose cleanly is critical.

### Dependency Patterns

**Pattern 1: Sequential Dependency** (Skill A must complete before Skill B starts)

```
Specification Flow Skill
    ↓
Data Fetch Skill (uses flow from above)
    ↓
Data Processing Skill (uses fetched data)
    ↓
Validation Skill (confirms processing)
```

Each skill waits for previous to complete, then uses its output.

**Pattern 2: Conditional Dependency** (Skill B only activates if Skill A produces specific result)

```
                    ↓ Success
    Error Detection Skill ─→ Error Recovery Skill
                    ↓ Fail
                   Escalate
```

The error recovery skill only activates if error detection identifies a recoverable error.

**Pattern 3: Referenced Dependency** (Skill B explicitly mentions it depends on Skill A)

In a skill's spec.md:
```yaml
dependencies:
  - skill: "data-validation-skill"
    version: "1.0+"
    when: "Before any code execution against external data"
```

This declaration makes the dependency explicit. Other developers (and the skill itself) know what other skills this skill requires.

### Why Composition Matters

When skills are well-composed:
1. **Clarity**: Other developers understand prerequisites
2. **Reusability**: Skills designed for composition work in multiple projects
3. **Testability**: Each skill can be tested independently
4. **Maintainability**: When one skill changes, composed skills adapt gracefully

When composition is poor:
1. **Hidden dependencies**: Skills fail mysteriously due to missing prerequisites
2. **Rigidity**: Skills only work in one specific context
3. **Integration hell**: Composing skills requires extensive debugging
4. **Maintenance nightmare**: Changing one skill breaks others unexpectedly

## Design Decision: Layer 3 vs Layer 4 Skills

Understanding when a skill belongs in Layer 3 (Intelligence Design - reusable component) vs Layer 4 (Spec-Driven Integration - capstone project) affects how you structure the skill.

### Layer 3 Skill Characteristics

A skill is Layer 3 when:

1. **It's a reusable component**: The pattern applies across 3+ different projects
2. **It solves a recurring problem**: You've encountered this workflow 2+ times
3. **Others will use it**: The skill is documented for broader adoption
4. **It's moderately complex**: 5-7 decision points, not trivial

**Example**: A "write-execute-analyze" loop skill
- Data analysis projects use it
- Code refactoring projects use it
- Report generation projects use it
- Others in your organization will adopt it

Design for **generality and documentation**.

### Layer 4 Skill Characteristics

A skill is Layer 4 when:

1. **It's a capstone orchestration**: Combines 3+ existing skills into a cohesive system
2. **It solves a specific domain problem**: You're building a Digital FTE for a customer
3. **It's production-ready**: Includes error recovery, logging, monitoring
4. **It's potentially monetized**: You'll sell this as a Digital FTE product

**Example**: A "legal-document-analyzer" Digital FTE
- Orchestrates document parsing skill
- Uses semantic analysis skill
- Coordinates with extraction skill
- Produces customer-ready output
- You'll charge customers for it

Design for **domain specificity, production robustness, and customer value**.

## Manual Exercise: Design Your Execution Skill

Using the framework from this lesson, design a skill for your own domain. You won't code it yet—this is design thinking.

### Step 1: Identify the Execution Task

What problem does your skill solve by orchestrating code execution?

**Example answers:**
- "Analyze customer feedback data and generate insights"
- "Process financial transactions and detect anomalies"
- "Refactor legacy code toward modern patterns"

**Your answer:**
```
My execution skill will:
```

### Step 2: Design the Persona

Write a persona that describes the skill as an orchestrator, not an advisor.

Start with: "You are a [domain] orchestrator: [describe the workflow]"

Use this template:
```
You are a [domain] orchestrator: When you see [situation],
(1) [first action],
(2) [second action],
(3) [validation step],
(4) [iteration or completion decision].
```

**Your persona:**
```
You are a _____________ orchestrator:
```

### Step 3: Write Five Decision-Making Questions

Write five questions the skill must answer to operate autonomously. Each question should be:
- **Concrete** (not vague—"Is this important?" is too vague)
- **Answerable** (the skill can actually figure out the answer)
- **Decision-triggering** (the answer determines what the skill does next)

**Your questions:**
```
1.
2.
3.
4.
5.
```

### Step 4: Articulate Three Principles

For each principle, state the constraint, reason, and application:

```
Principle 1: [Name]
Constraint: [What's the boundary?]
Reason: [Why does this matter?]
Application: [How do you know you've done it right?]
```

**Your principles:**
```
Principle 1:
Principle 2:
Principle 3:
```

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini, or similar) to validate and refine your skill design.

### Prompt 1: Validate Persona Clarity

```
I've designed a skill with this persona:
[paste your persona from Step 2]

Does this persona describe an execution workflow (orchestration) or just advisory guidance?
Specifically, what workflow does this persona describe? Can you extract the steps?
If the persona is vague, suggest how to make it more concrete with explicit workflow steps.
```

**What you're learning**: How to articulate execution workflows clearly so skills (and humans) understand exactly what the skill will do.

### Prompt 2: Refine Decision Questions

```
I've written these decision-making questions for my skill:
[paste your five questions from Step 3]

For each question:
1. Is this question concrete enough that the skill can actually answer it?
2. Does the answer lead to a specific action, or is it just informational?
3. If the question is vague, rewrite it to be more concrete

Then ask: "Which of your original questions are most critical to the skill's autonomous behavior?"
```

**What you're learning**: How to distinguish vague questions ("Is this important?") from decision questions that actually guide execution ("Does this output match the spec?").

### Prompt 3: Strengthen Principles

```
I've written these principles to guide my skill's execution:
[paste your three principles from Step 4]

For each principle, evaluate:
1. Is the constraint clear and testable?
2. Does the reason explain WHY this constraint matters?
3. Is the application concrete enough that someone could validate compliance?

If any principle is unclear, help me rewrite it with a concrete constraint, clear reason, and testable application.
```

**What you're learning**: How to encode behavioral guidelines that actually constrain skill behavior, not vague aspirations.

### Safety Note

As you design skills that orchestrate code execution, remember: the skill will be running code autonomously. Design your principles with safety in mind. What data should never be accessed? What operations should never be attempted? What limits (time, resources, retries) prevent runaway execution? These constraints become the difference between a reliable Digital FTE and a system that fails unpredictably.
