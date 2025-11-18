---
title: "Designing Reusable Intelligence from SDD Workflows"
chapter: 31
lesson: 9
duration_minutes: 150

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Pattern Recognition for Intelligence Encoding"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify which workflow patterns from Lessons 1-7 justify encoding as reusable intelligence (frequency, complexity, organizational value)"

  - name: "Skill Design Using Persona + Questions + Principles"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can design a skill with Persona (cognitive stance), Questions (reasoning prompts), and Principles (decision frameworks)"

  - name: "Subagent Persona Definition"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can define a subagent persona for specification review with 5+ decision points"

  - name: "Intelligence Component File Structure"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can create intelligence component file following .claude/skills/ structure with proper metadata"

  - name: "Reuse vs Create Decision Framework"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can determine when to reuse existing intelligence vs create new component based on context specificity and decision complexity"

learning_objectives:
  - objective: "Identify recurring patterns from Lessons 1-7 that justify intelligence encoding (constitution creation, specification review, edge case analysis)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Pattern identification exercise with frequency and complexity analysis"

  - objective: "Design a skill using Persona + Questions + Principles pattern for specification quality review"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Skill component completeness and reasoning activation quality"

  - objective: "Define a subagent persona for specification auditing with autonomous decision-making capability"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Subagent persona clarity and decision point enumeration (5+ required)"

  - objective: "Create intelligence component file with proper structure and metadata"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "File structure validation against .claude/skills/ standards"

  - objective: "Apply reuse vs create framework to determine when to build new intelligence"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Decision justification for 3 example scenarios"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (Pattern recognition framework, Reusable intelligence paradigm, P+Q+P pattern structure, Reasoning vs prediction modes, Skill file structure, Subagent autonomy distinction, Intelligence library organization, Reuse decision criteria) - at upper B1 limit but manageable with scaffolding ✓"

differentiation:
  extension_for_advanced: "Create second skill for plan quality validation; design subagent for cross-phase consistency checking; build intelligence library with 3+ components"
  remedial_for_struggling: "Use provided skill template with examples; focus on single skill creation before subagent design; defer file structure details to later practice"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Designing Reusable Intelligence from SDD Workflows

You've completed the SDD workflow (Lessons 1-7): Constitution → Specify → Clarify → Plan → Tasks → Implement. You can now build projects using specification-first methodology.

But here's what separates AI-native developers from AI-assisted developers: **The ability to transform workflow patterns into reusable intelligence.**

In this lesson, you'll apply the core paradigm: **Reusable Intelligence** is the new unit of value, not reusable code. You'll extract patterns from Lessons 1-7 and encode them as skills and subagents—creating an intelligence library that compounds with every project.

You'll transform tacit knowledge into explicit, reusable components—the next evolution beyond workflow execution.

---

## From Workflow Execution to Intelligence Accumulation

### What You've Built So Far

**Lessons 1-7 taught you process**:
- How to create Constitutions (quality standards)
- How to write specifications (intent documentation)
- How to clarify ambiguities (specification gaps)
- How to generate plans (implementation strategy)
- How to decompose tasks (work breakdown)
- How to orchestrate implementation (AI-driven execution)

**What's missing**: Reusable components that make the next project 10x faster.

### The Paradigm Shift: Specs + Intelligence > Code

The Specification-Driven Development with Reusable Intelligence (SDD-RI) approach changes what we consider reusable:

> **Traditional Development**: Code libraries are the units of reuse. Developers share functions, classes, frameworks.
>
> **AI-Native Development**: Specifications, Agent Architectures, and Skills are the units of reuse. Developers share intelligence.

**In practice**:
- **Project 1**: You write constitution, specification, run workflow (10 hours)
- **Project 2** (without intelligence): You write new constitution, new spec, run workflow (9 hours—slightly faster)
- **Project 2** (with intelligence): You invoke `spec-review` skill, reuse constitution template, orchestrate with subagent (3 hours—7x faster)

**The difference**: Accumulated intelligence compounds. Every pattern you encode accelerates future work.

---

## Identifying Patterns Worth Encoding

Not every workflow step justifies creating reusable intelligence. Use this decision framework:

### Decision Framework: When to Encode Intelligence

Ask three questions about the workflow pattern:

**1. Frequency**: Will this pattern recur across 3+ projects?
- ✅ YES: Specification quality review (every project needs specs)
- ✅ YES: Edge case identification (every feature has edge cases)
- ❌ NO: Calculator-specific math operations (one-off for this project)

**2. Complexity**: Does this pattern involve 5+ decision points?
- ✅ YES: Specification review (scope, clarity, SMART criteria, edge cases, constraints, non-goals = 6+ decisions)
- ❌ NO: Running `/sp.specify` command (1 decision: invoke or not)

**3. Organizational Value**: Will encoding this pattern improve team capability?
- ✅ YES: Constitution templates (standardizes team quality)
- ✅ YES: Specification review skills (catches common errors)
- ❌ NO: Personal file organization preferences (individual style)

**Rule**: If 2+ answers are YES → Encode as reusable intelligence

### Pattern Analysis from Lessons 1-7

Let's analyze what you learned:

| **Pattern** | **Frequency** | **Complexity** | **Org Value** | **Encode?** |
|-------------|---------------|----------------|---------------|-------------|
| Constitution creation | ✅ Every project | ✅ 7+ decisions | ✅ Team standards | **YES** |
| Specification review | ✅ Every feature | ✅ 6+ decisions | ✅ Quality gate | **YES** |
| Edge case identification | ✅ Every feature | ✅ 5+ decisions | ✅ Bug prevention | **YES** |
| `/sp.specify` invocation | ✅ Every feature | ❌ 1 decision | ❌ Trivial | NO |
| Git branch creation | ✅ Every feature | ❌ 1-2 decisions | ❌ Standard practice | NO |
| SMART criteria validation | ✅ Every spec | ✅ 5 decisions | ✅ Clarity enforcement | **YES** |

**Candidates for intelligence encoding**:
1. **Constitution Template Skill**: Guide for creating project-specific quality standards
2. **Specification Review Skill**: Quality audit checklist for specs
3. **Edge Case Analysis Skill**: Framework for identifying edge cases
4. **SMART Criteria Validator Subagent**: Autonomous acceptance criteria review

**In this lesson, you'll build**: Specification Review Skill + SMART Validator Subagent foundation

---

## Skill Design: Persona + Questions + Principles

Effective intelligence uses the **Persona + Questions + Principles (P+Q+P)** pattern.

This pattern **activates reasoning mode** (context-specific thinking) instead of **prediction mode** (pattern retrieval).

### The P+Q+P Pattern

**Persona**: Establishes cognitive stance (how to think about the problem)
- Not: "You are an expert" (vague, triggers generic responses)
- But: "Think like a specification reviewer who audits for SMART criteria the way a code reviewer checks for security vulnerabilities—systematically, with checklists, identifying gaps before they cause failures"

**Questions**: Forces context-specific analysis (what to analyze)
- Not: "Is this specification good?" (yes/no, no reasoning)
- But: "Are acceptance criteria Specific (exact metrics)? Measurable (objective tests)? Achievable (implementable)? Relevant (map to user needs)? For each criterion, identify vague language and suggest SMART alternatives."

**Principles**: Provides decision frameworks (how to make judgments)
- Not: "Use best practices" (meaningless without definition)
- But: "SMART Criteria Framework: Specific (no ambiguous terms like 'good', 'fast', 'secure'), Measurable (numeric thresholds or objective tests), Achievable (implementable with available resources), Relevant (directly serves user need from spec), Time-bound (if applicable)"

### Building Your First Skill: Specification Review

**Step 1: Define the Persona**

Open your AI companion and collaborate:

```
I want to create a skill for reviewing specification quality.
The skill should help me audit specs for SMART criteria, edge case coverage,
and Constitution alignment.

Let's design the persona together. I'm thinking:
"Think like a specification reviewer who audits for completeness..."

Help me finish this persona. What cognitive stance activates the right
kind of analytical thinking for specification review?
```

**What Good Looks Like**:

```markdown
## Persona

You are a specification quality auditor who thinks about requirements
the way a security engineer thinks about attack surfaces:

- Systematically checking for gaps
- Identifying what's NOT specified (implicit assumptions)
- Validating that acceptance criteria are testable
- Ensuring edge cases are documented before implementation

Your goal: Catch specification ambiguities that would cause
implementation failures or misalignment with user needs.
```

**Step 2: Formulate Analytical Questions**

```
Now let's design the questions this skill should ask when reviewing a spec.

Questions should force context-specific analysis, not yes/no answers.

Here are my ideas:
1. "Are acceptance criteria SMART or vague?"
2. "What edge cases are missing?"

Help me expand this to 5-7 questions that activate thorough analysis.
```

**What Good Looks Like**:

```markdown
## Analytical Questions

Before approving a specification, analyze:

1. **SMART Criteria Validation**:
   - Are acceptance criteria Specific (exact metrics, no vague terms)?
   - Are they Measurable (objective tests, not subjective judgment)?
   - Are they Achievable (implementable with available resources)?
   - Are they Relevant (directly serve user needs from spec overview)?

2. **Edge Case Coverage**:
   - What boundary conditions exist (min/max values, empty inputs, nulls)?
   - What error states can occur (network failures, invalid inputs, race conditions)?
   - What assumptions could be violated (type mismatches, concurrent access)?

3. **Scope Clarity**:
   - Are in-scope features clearly listed?
   - Are out-of-scope items explicitly documented?
   - Can implementer distinguish included vs excluded features?

4. **Constitution Alignment**:
   - Does spec respect project quality standards?
   - Are required constraints (type hints, testing, documentation) specified?
   - Does spec reference relevant constitution sections?

5. **Completeness Check**:
   - Does spec have Overview, Scope, Requirements, Acceptance Criteria, Constraints?
   - Can another developer implement from this spec without guessing?
   - Are success metrics defined (how to validate implementation)?
```

**Step 3: Articulate Decision Principles**

```
Finally, let's define the decision frameworks that guide the review.

These should be concrete rules, not abstract advice.

Example: "Vague Language Detection: Flag any acceptance criterion
containing 'good', 'fast', 'secure', 'clean', 'professional' without
numeric thresholds or objective tests."

Help me create 5 principle statements for specification review.
```

**What Good Looks Like**:

```markdown
## Decision Principles

Apply these frameworks when reviewing specifications:

1. **SMART Criteria Enforcement**:
   - Specific: No ambiguous language ("works well" → "returns results in <100ms")
   - Measurable: Numeric thresholds or objective tests ("fast" → "p95 latency <200ms")
   - Achievable: Implementable with stated resources/constraints
   - Relevant: Maps to user need or business goal from overview
   - Time-bound: If applicable (not always required)

2. **Edge Case Minimum Standard**:
   - Every data input: Document min/max/empty/null behavior
   - Every operation: Document error states and handling
   - Every assumption: Document what happens if assumption violated
   - Minimum 3 edge cases per feature

3. **Explicit Over Implicit**:
   - Non-goals section required (what we're NOT building)
   - Assumptions section required (what we're taking for granted)
   - If spec doesn't say it, implementation shouldn't assume it

4. **Testability Requirement**:
   - Every acceptance criterion must map to automated test
   - If criterion can't be tested objectively → it's too vague
   - Success metrics define "done" (not implementation completion)

5. **Constitution as Contract**:
   - Spec cannot contradict constitution constraints
   - If spec requires something constitution forbids → escalate conflict
   - Reference specific constitution sections for requirements
```

---

## Creating Your Skill File

Now let's turn your P+Q+P components into a reusable skill file.

### Step 1: Create the Skill Directory

```bash
# In your calculator-project directory
mkdir -p .claude/skills
```

### Step 2: Create the Skill File

```bash
# Create specification-review skill
touch .claude/skills/specification-review.md
```

### Step 3: Write the Skill Content

Open `.claude/skills/specification-review.md` and collaborate with your AI:

```
I want to create a skill file for specification review using the
P+Q+P pattern we designed.

The file should follow the standard skill structure:
- Metadata header (name, version, description, when to use)
- Persona section
- Questions section
- Principles section
- Usage examples
- Self-check validation

Here's what we designed:
[Paste your Persona + Questions + Principles from above]

Help me format this as a complete skill file.
```

**What Good Looks Like**:

Your AI will generate a complete skill file following this structure:

```markdown
# Skill: Specification Quality Review

**Metadata**: Version, created date, category, complexity level

## Description
Brief explanation of what the skill does and when to use it

## When to Use This Skill
- Apply when: [3-4 scenarios]
- Skip when: [2-3 scenarios]

## Persona
[Your cognitive stance from earlier - e.g., "specification quality auditor
who thinks about requirements the way a security engineer thinks about
attack surfaces"]

## Analytical Questions
[Your 5-7 questions from earlier - SMART validation, edge cases, scope,
constitution, completeness]

## Decision Principles
[Your 5 decision frameworks - SMART enforcement, edge case standards,
explicit over implicit, testability, constitution alignment]

## Usage Example
**Scenario**: [Concrete use case]
**Invocation**: [How to call this skill]
**Expected Output**: [What quality response looks like]

## Self-Check Validation
[5 checkboxes to verify skill was applied correctly]
```

**Key sections to verify**:
- ✅ Persona establishes clear cognitive stance (not generic "expert")
- ✅ Questions force analysis (not yes/no answers)
- ✅ Principles provide decision criteria (not vague guidance)
- ✅ Usage example shows realistic input/output
- ✅ Self-check enables validation

---

## Subagent Foundations: From Skills to Autonomous Agents

Skills provide **guidance** (checklists, frameworks). Subagents provide **autonomous reasoning** (make decisions without human intervention).

**Decision Rule**:
- **2-4 decision points** → Skill (human applies framework)
- **5+ decision points** → Subagent (autonomous execution)

### When to Create Subagents

Your specification review skill has **7+ decision points** (SMART validation × 5 criteria + edge cases + scope + constitution + completeness). This justifies a subagent.

**Subagent capabilities**:
- Read specification files autonomously
- Apply SMART criteria framework without prompting
- Generate review reports automatically
- Integrate into CI/CD pipelines

### Defining Subagent Persona

A subagent persona is more detailed than a skill persona because it needs to operate autonomously.

**Collaborate with your AI**:

```
I want to design a subagent for automated specification review.

The subagent should:
1. Read spec.md files from specs/ directory
2. Apply the Specification Quality Review skill autonomously
3. Generate review reports with specific findings
4. Provide pass/fail verdict with justification

Help me design the subagent persona. It should include:
- Role definition (what it does)
- Cognitive stance (how it thinks)
- Decision authority (what it can decide vs escalate)
- Reporting format (how it communicates findings)
```

**What Makes Subagents Different from Skills**:

A subagent file builds on the skill structure but adds three critical sections:

**1. Role Definition** (NEW):
```markdown
**Name**: spec-auditor
**Autonomy Level**: High (makes pass/fail decisions autonomously)
**Invocation**: Automatic (after `/sp.specify`) or manual (`/review-spec`)
```

**2. Decision Authority** (EXPANDED from Principles):
```markdown
**Can PASS**: Specifications meeting all SMART + edge case + completeness criteria
**Can CONDITIONAL PASS**: Specifications with 1-2 minor gaps (listed fixes required)
**Can FAIL**: Specifications with 3+ vague criteria or missing critical edge cases
**Must ESCALATE**: Constitution conflicts or ambiguous quality outside criteria
```

**3. Reporting Format** (NEW):
```markdown
Generate structured reports:
=== AUDIT REPORT ===
File: specs/[feature]/spec.md
--- SMART CRITERIA ---
Status: [PASS | PARTIAL | FAIL]
Findings: [specific line references + SMART alternatives]
--- VERDICT ===
Overall: [PASS | CONDITIONAL | FAIL | ESCALATE]
Required Actions: [numbered list with line refs]
```

**Key differences from skills**:
- Skills provide framework (human applies it)
- Subagents make decisions autonomously (human reviews verdict)
- Subagents need explicit pass/fail criteria
- Subagents generate structured reports

---

## Building Your Intelligence Library

You've now created:
1. **Specification Review Skill** (guidance framework, 7 decision points)
2. **Spec Auditor Subagent Foundation** (autonomous reviewer, high decision authority)

### Organizing Your Intelligence

**Standard directory structure**:
```
calculator-project/
├── .claude/
│   ├── skills/
│   │   ├── specification-review.md    ← Your skill
│   │   ├── edge-case-analysis.md      ← Future skill
│   │   └── constitution-template.md   ← Future skill
│   ├── subagents/
│   │   ├── spec-auditor.md            ← Your subagent
│   │   └── plan-validator.md          ← Future subagent
│   └── workflow-config.yml            ← Automation rules
├── specs/
│   └── calculator/
│       └── spec.md
└── README.md
```

### Intelligence Reuse Strategy

**Skill reuse** (apply guidance to new contexts):
```
# Project 2: REST API
I'm reviewing an API specification.

Apply the Specification Quality Review skill from
.claude/skills/specification-review.md

Context: This is a REST API spec, not a library.
Adapt edge case analysis for HTTP contexts (404s, timeouts, malformed JSON).
```

**Subagent reuse** (autonomous application):
```
# Project 2: REST API
/review-spec user-authentication

# AI automatically applies spec-auditor subagent
# Generates audit report adapted to API context
```

**Intelligence composition** (combine multiple components):
```
# Project 3: CLI tool
Apply these skills in sequence:
1. Constitution Template → generate project quality standards
2. Specification Review → audit CLI tool spec
3. Edge Case Analysis → identify CLI-specific edge cases

Then invoke spec-auditor subagent for final validation.
```

---

## Validation: Does Your Intelligence Activate Reasoning?

Bad intelligence triggers **prediction mode** (generic responses). Good intelligence triggers **reasoning mode** (context-specific analysis).

### Test Your Specification Review Skill

**Ask your AI companion**:

```
Apply the Specification Quality Review skill to this vague acceptance criterion:

"Calculator should handle errors gracefully"

Walk me through your analysis using the Persona, Questions, and Principles
from the skill.
```

**Good response (reasoning mode)**:

```
Applying Specification Quality Review skill:

PERSONA: Thinking like a specification auditor checking for testability gaps...

QUESTIONS ANALYSIS:
1. SMART Criteria Validation:
   - Specific? NO - "gracefully" is undefined
   - Measurable? NO - "handle errors" has no objective test
   - Achievable? UNKNOWN - no error types specified
   - Relevant? UNCLEAR - which errors? all operations?

2. Edge Case Coverage:
   - What error states? (division by zero, type mismatch, overflow?)
   - What does "gracefully" mean? (raise exception, return None, log?)

PRINCIPLES APPLICATION:
- SMART Criteria Enforcement: "gracefully" is vague language (like "good", "fast")
- Testability Requirement: Cannot write automated test for "graceful"

SMART ALTERNATIVE:
"Calculator raises ValueError with descriptive message for:
 - Division by zero: 'Cannot divide by zero'
 - Invalid types: 'Inputs must be int or float'
 - Overflow: 'Result exceeds maximum float value'"

This version is:
- Specific: Exact error type (ValueError), exact messages
- Measurable: Can test with pytest.raises(ValueError, match="Cannot divide")
- Achievable: Standard Python exception handling
- Relevant: Covers error handling requirement from overview
```

**Bad response (prediction mode)**:

```
The criterion "handle errors gracefully" is too vague.
Make it more specific like "handle errors properly."
```

**If you get prediction mode responses**: Your skill's Persona/Questions/Principles need strengthening. Revise with more concrete frameworks.

---

## Common Mistakes

### Mistake 1: Creating Skills for Trivial Decisions

**The Error**: Creating a skill for "How to run /sp.specify command"

**Why It's Wrong**: 1 decision point (invoke or not) doesn't justify intelligence encoding. Skills are for 2-4+ decision workflows.

**The Fix**: Only encode patterns with frequency + complexity + organizational value.

### Mistake 2: Vague Personas

**The Error**: "You are an expert specification reviewer"

**Why It's Wrong**: "Expert" is generic, triggers prediction mode ("use best practices").

**The Fix**: Specific cognitive stance with analogy:
- ❌ "You are an expert"
- ✅ "Think like a security engineer checking attack surfaces—systematically, with checklists, identifying gaps"

### Mistake 3: Yes/No Questions

**The Error**: "Is this specification good?"

**Why It's Wrong**: Binary questions don't activate analysis. AI responds "yes" or "no" without reasoning.

**The Fix**: Open-ended analytical questions:
- ❌ "Is this specification good?"
- ✅ "Which acceptance criteria are vague vs SMART? For each vague criterion, what specific measurable alternative would make it testable?"

### Mistake 4: Over-Specific Skills

**The Error**: Creating "Calculator-Specification-Review" skill that only works for calculators

**Why It's Wrong**: Intelligence should be reusable across projects. Over-specificity reduces organizational value.

**The Fix**: Generalize patterns:
- ❌ "Calculator-Specification-Review"
- ✅ "Specification-Quality-Review" (works for APIs, CLIs, libraries, etc.)

---

## Try With AI

Ready to create reusable intelligence that works across projects? Build your intelligence library:

**🔍 Explore Skill vs Subagent:**
> "Explain the difference between a Skill and a Subagent in Spec-Kit Plus. For specification quality review, show me: (1) What would a Skill look like (Persona + Questions + Principles)? (2) What would a Subagent look like (autonomous with decision authority)? (3) When would I use each? (4) Can they work together? Give me concrete examples of both for the same task."

**🎯 Practice Creating a Skill:**
> "Help me create a reusable Skill for specification quality review. Guide me through the P+Q+P pattern: (1) What Persona should it adopt (cognitive stance with analogy)? (2) What 5-7 Analytical Questions should it ask to review specs? (3) What 5 Decision Principles should guide its evaluation? Structure it as a complete skill file following `.claude/skills/` format with metadata, usage example, and self-check validation."

**🧪 Test Intelligence Reuse:**
> "I created a Specification Review skill for my calculator project. Now I'm starting a REST API for user authentication. Apply my skill to audit this API acceptance criterion: 'API should return proper status codes'. Show me how the P+Q+P pattern adapts to API context vs library context. Does the skill identify vagueness? What API-specific questions does it ask? How does it demonstrate reusability?"

**🚀 Apply to Your Domain:**
> "I work on [describe your project type]. Help me identify what reusable intelligence I should create: (1) What decisions do I repeat across projects? (2) What reviews could be systematic (skills)? (3) What workflows could be autonomous (subagents)? (4) Design ONE skill or subagent using P+Q+P that would save me time on my next 3 projects. Explain its structure and provide the complete file."

---

