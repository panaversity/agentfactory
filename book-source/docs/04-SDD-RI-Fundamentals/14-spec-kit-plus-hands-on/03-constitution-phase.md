---
title: "Constitution Phase — Project-Wide Quality Standards"
chapter: 14
lesson: 3
duration_minutes: 30
proficiency_level: "A2"
cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (constitution purpose, cascade effect, testable standards, git commit) within A2 limit of 7 ✓"
learning_objectives:
  - objective: "Use `/sp.constitution` command to define project-wide quality standards"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successfully run /sp.constitution and generate constitution file"
  - objective: "Understand how constitution guides all downstream specification and implementation phases"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explain the cascade effect with concrete examples"
  - objective: "Write testable quality criteria that apply across all features in a project"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Identify vague vs testable standards in constitution"
  - objective: "Distinguish between constitution (global rules) and specification (feature-specific requirements)"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Correctly categorize example rules as constitution vs specification"
generated_by: "content-implementer v1.0.0"
source_spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
created: "2025-11-26"
last_modified: "2025-11-27"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "3.0.0"
---

# Constitution Phase — Project-Wide Quality Standards

You've installed Spec-Kit Plus and initialized your project. Now it's time to create the foundational rules that will guide every part of your research paper project.

The Constitution answers a critical question: **What standards apply to every piece of work you do?** Not just for this paper, but for all papers. Not just this deadline, but for your professional reputation.

Think of it like team rules before building a big LEGO project:
- What if you want all towers square, but your helper builds round ones?
- What if you decide the roof must be blue, but your helper builds red?

The Constitution is your team's **Rulebook**. It lists the most important rules that both you and your AI companion MUST follow, no matter what.

---

## What Is a Constitution?

### Constitution: Global Rules, One Per Project

A **Constitution** defines **immutable standards** applying to **all work** in a project. It's distinct from a **Specification**, which applies to **one feature**.

**Constitution applies to (research paper project)**:
- Citation standards for ALL papers (APA format, source verification)
- Writing clarity for ALL papers (Flesch-Kincaid grade level)
- Academic integrity for ALL papers (plagiarism checking)
- Source requirements for ALL papers (peer-reviewed minimum)

**Specification applies to (one specific paper)**:
- THIS paper's thesis statement
- THIS paper's specific research questions
- THIS paper's word count and deadline
- THIS paper's acceptance criteria

**Example**:

```
CONSTITUTION (applies to ALL papers):
  ✅ "All papers must cite primary sources"
  ✅ "All claims must be verified against sources"
  ✅ "APA citation format required"
  ✅ "Zero plagiarism tolerance"

SPECIFICATION (applies only to THIS paper):
  ✅ "Thesis: AI-native development requires spec-first thinking"
  ✅ "Target length: 5,000 words"
  ✅ "Minimum 12 peer-reviewed sources"
  ✅ "Due date: December 15"
```

### Why Constitution Matters: The Cascade

The Constitution is the **starting point of the cascade**:

```
Clear Constitution
    ↓
(ensures every spec respects quality standards)
    ↓
Clear Specification
    ↓
(ensures planning accounts for quality gates)
    ↓
Clear Plan
    ↓
(ensures tasks include verification)
    ↓
Clear Tasks
    ↓
(enables AI to generate properly cited writing)
    ↓
Published Research Paper
```

**Weak Constitution** produces:
- Specs that don't specify citation requirements
- Plans that skip plagiarism checking
- Writing with uncited claims
- Papers that fail fact-checking

**Strong Constitution** produces:
- Specs that automatically include source quality requirements
- Plans with built-in verification steps
- Writing that's properly cited
- Papers that pass publication standards

### Constitution is One-Time, Feature Work is Repetitive

You write the Constitution **once per project**. Then, for each paper:

```
1. Initialize project
2. Write Constitution (quality standards for ALL papers)
3. Commit Constitution to git
4. FOR EACH PAPER:
   - Run /sp.specify (new specification for this paper)
   - Run /sp.clarify (refine specification)
   - Run /sp.plan (new plan for this paper)
   - Run /sp.tasks (new tasks for this paper)
   - Run /sp.implement (write paper with AI)
   - Commit paper to git
```

---

## Part A: Reading the Base Constitution

Before writing your own, look at the base Constitution file that `specifyplus init` created:

```bash
# Open the constitution file
cat .specify/memory/constitution.md
```

**What you'll see**: A starter template with placeholder sections for principles, standards, and constraints.

**The Key Insight**: Constitutions are project-specific. Your research paper Constitution would never mention "type hints" because that's for code. A software project Constitution wouldn't need "citation format" because that's for papers.

---

## Part B: Writing Your Research Paper Constitution

Now let's write YOUR Constitution for the research paper project.

### Step 1: Run `/sp.constitution`

Open your AI tool and run the constitution command with your project requirements:

```
/sp.constitution

Project: Research paper on AI-native software development

Core principles:
- Accuracy through primary source verification
- Clarity for academic audience (computer science background)
- Reproducibility (all claims cited and traceable)
- Rigor (peer-reviewed sources preferred)

Key standards:
- All factual claims must be traceable to sources
- Citation format: APA style
- Source types: minimum 50% peer-reviewed articles
- Plagiarism check: 0% tolerance before submission
- Writing clarity: Flesch-Kincaid grade 10-12

Constraints:
- Word count: 5,000-7,000 words
- Minimum 15 sources
- Format: PDF with embedded citations

Success criteria:
- All claims verified against sources
- Zero plagiarism detected
- Passes fact-checking review
```

**What the agent does**:
- Creates a comprehensive Constitution file at `.specify/memory/constitution.md`
- Defines testable quality standards
- Documents all constraints and success criteria
- Shows you the generated Constitution

### Step 2: Review Your Constitution

After the agent generates your Constitution, **review it carefully**.

**Your Prompt**:

```
Show me the generated constitution file and explain what it contains.
```

**Agent shows**:
- **Core Principles** — Your research philosophy
- **Quality Standards** — Testable criteria for all papers
- **Source Requirements** — Citation and verification rules
- **Constraints** — Length, format, deadlines
- **Success Criteria** — How to know if standards are met

### Step 3: Improve Your Constitution

Think about what "good research" means for YOUR project. Ask the agent:

```
Review my Constitution at .specify/memory/constitution.md and improve it:

1. Are all standards testable (not vague)?
   - ❌ Vague: "Papers should be well-written"
   - ✅ Testable: "Flesch-Kincaid grade 10-12; active voice 75%+ of time"

2. Did I cover essential categories?
   - Citation accuracy
   - Source verification
   - Writing clarity
   - Plagiarism checking
   - Review process

3. Are any standards unrealistic?

Suggest 2-3 concrete improvements.
```

**What the agent does**:
- Identifies vague standards and makes them testable
- Suggests missing categories
- Flags unrealistic constraints
- Updates the Constitution file

---

## Part C: Commit Constitution to Git

Here's a critical best practice: **Always commit the Constitution before starting feature work.**

### Why Commit First?

1. **Immutability**: Constitution is foundational; committing signals "this is our standard"
2. **Clarity**: Everyone (including your AI) sees Constitution as the baseline
3. **Traceability**: Git history shows when and why Constitution was created
4. **Reversibility**: You can revert if needed (rare, but important)

### Commit Steps

**Your Prompt**:

```
/sp.git.commit_pr Commit the constitution to a feature branch
```

**Agent Does**:
- Creates a conventional commit for the constitution
- Pushes to a new feature branch
- Creates a draft PR (or shares compare URL)

The Constitution is now the foundation for all your paper work. Every specification you write, every plan you generate—they all work within the Constitution's constraints.

---

## How Constitution Guides Downstream Phases

Now that you've created a Constitution, let's see how it cascades through every other phase.

### Constitution → Specification Phase

**Your Constitution says**:
```
- All papers must cite primary sources
- Minimum 50% peer-reviewed sources
- APA citation format
- Zero plagiarism tolerance
```

**Your Specification for Paper #1 must respect this**:
```
This specification inherits Constitution standards:
- Thesis: "AI-native development requires spec-first thinking"
- Length: 5,000 words
- Sources: minimum 15 total, minimum 8 peer-reviewed
- Format: APA style (inherited from Constitution)
- Success criteria: All claims cited, Flesch-Kincaid 10-12
```

Notice: You DON'T re-specify citation format or plagiarism checking. The Constitution already requires it.

### Constitution → Plan Phase

**Your Constitution says**:
```
- All claims must be verified against sources
- Plagiarism check required before submission
```

**Your Plan must account for this**:
```
Phase 1: Research and source identification
Phase 2: Detailed outline with source assignments
Phase 3: Draft writing with inline citations
Phase 4: Fact-checking pass (verify all claims)
Phase 5: Plagiarism scanning (0% tolerance)
Phase 6: Final review and submission
```

The plan INCLUDES verification steps because Constitution REQUIRES them.

### Constitution → Implementation

When you write the paper with AI, Constitution standards guide every interaction:

```
You: "Write the Introduction section. Use these sources: [list].
Follow the Constitution: APA citations, verify all claims."

AI: "I'll write the introduction with:
- In-text APA citations for each claim
- Plain language targeting grade 10-12 reading level
- Verification of claims against provided sources"

[AI writes introduction with citations]

You: "The third claim needs a primary source, not secondary.
Constitution requires 50% primary sources."

AI: "You're right. Let me find the primary research on that topic
and revise the citation."

[AI revises with primary source]
```

Constitution REQUIREMENTS shape every interaction.

---

## Common Mistakes

### Mistake 1: Copying Constitution Without Customization

**The Error**: "I'll use the example Constitution as-is."

**Why It's Wrong**: Constitutions are project-specific. A code project Constitution mentions "type hints"—irrelevant for papers.

**The Fix**: Read examples for structure, but write rules specific to YOUR project.

### Mistake 2: Vague Quality Standards

**The Error**: "Papers must be good quality" or "Sources should be credible"

**Why It's Wrong**: "Good" and "credible" are subjective. No one can verify these.

**The Fix**: Use testable criteria:
- ❌ Vague: "Good writing quality"
- ✅ Testable: "Flesch-Kincaid grade 10-12; all claims cited; zero plagiarism"

### Mistake 3: Forgetting to Commit Constitution

**The Error**: Create Constitution, then start spec without committing.

**Why It's Wrong**: Constitution becomes "whatever I remember" instead of "documented standard."

**The Fix**: Always commit Constitution BEFORE starting `/sp.specify`.

---

## Try With AI

Validate your Constitution and understand how quality rules cascade:

**Test Constitution Completeness:**

> "Review my Constitution at `.specify/memory/constitution.md`. Check: (1) Are all standards testable and specific? (2) Did I cover citation, source verification, writing clarity, plagiarism checking? (3) Are any standards unrealistic? Suggest 2-3 improvements."

**Explore Cascade Effect:**

> "I wrote a Constitution with standards for APA citations, source verification, and zero plagiarism tolerance. Explain how these rules cascade: How do they affect my Specification? My Plan? My Implementation? Give me a concrete example."

**Test Specification Alignment:**

> "Based on my Constitution rules, what constraints MUST a specification for my first paper section include? Walk through: citation requirements, source types, verification steps, and quality standards."
