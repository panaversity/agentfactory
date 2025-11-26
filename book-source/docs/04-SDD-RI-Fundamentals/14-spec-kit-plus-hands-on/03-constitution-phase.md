---
title: "Constitution Phase — Project-Wide Quality Standards"
chapter: 14
lesson: 3
duration_minutes: 30
proficiency_level: "A2"
cognitive_load:
  new_concepts: 4
learning_objectives:
  - "Use `/sp.constitution` command to define project-wide quality standards"
  - "Understand how constitution guides all downstream specification and implementation phases"
  - "Write testable quality criteria that apply across all features in a project"
  - "Distinguish between constitution (global rules) and specification (feature-specific requirements)"
generated_by: "content-implementer v1.0.0"
source_spec: "specs/chapter-14-spec-kit-plus/lesson-3-spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Constitution Phase — Project-Wide Quality Standards

Welcome to hands-on work with `/sp.constitution`. In Chapter 13, you learned that a Constitution is a document defining immutable standards applying to **all features** in a project. Now you'll create one for a research paper project.

The Constitution answers a critical question: **What standards apply to every piece of work I do?** Not just for this paper, but for every paper in your academic career. Not just for this deadline, but for your professional reputation.

Before diving in, let's understand what a Constitution actually is and why it matters for **any** project—papers, code, data pipelines, books.

---

## What Is a Constitution?

### Constitution: Global Rules, One Per Project

A **Constitution** is a document that defines **immutable standards** applying to **all features** in a project. It's distinct from a **Specification**, which applies to **one feature**.

**Constitution applies to (research paper project)**:
- Quality standards for all papers (citation accuracy, source verification, writing clarity)
- Audience-level consistency (all papers written for same proficiency level)
- Academic integrity requirements (plagiarism checking, source documentation)
- Structural standards (formatting, section organization, reference style)
- Revision and review processes (how feedback loops work)

**Specification applies to (one specific research paper)**:
- This paper's thesis statement
- Specific research questions to explore
- Sources to investigate (this paper's sources, not all papers)
- Target length and deadline
- Specific acceptance criteria for this paper's completion

**Example**:

```
CONSTITUTION (applies to ALL research papers):
  ✅ "All papers must cite primary sources, not just secondary sources"
  ✅ "All claims must be verified against authoritative sources"
  ✅ "Writing must be clear and direct (Flesch-Kincaid grade 10-12)"
  ✅ "All papers reviewed for plagiarism before submission"
  ✅ "All references formatted in APA style"

SPECIFICATION (applies only to SPECIFIC RESEARCH PAPER):
  ✅ "This paper explores AI development methodology"
  ✅ "Target length: 5,000 words"
  ✅ "Minimum 12 peer-reviewed sources required"
  ✅ "Due date: December 15"
  ✅ "Thesis: AI-native development requires thinking in specifications, not code"
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
(ensures tasks include review and verification)
    ↓
Clear Tasks
    ↓
(enables AI to generate writing that's accurate and well-cited)
    ↓
Published Research Paper
```

**Weak Constitution** produces:
- Specs that don't specify citation requirements (leading to uncited claims)
- Plans that don't include plagiarism checking
- Writing that lacks source verification
- Papers that fail fact-checking
- Integration issues because quality standards weren't enforced upstream

**Strong Constitution** produces:
- Specs that automatically include source quality and citation requirements
- Plans with built-in fact-checking and verification steps
- Writing that's automatically verified against sources
- Papers that pass publication standards
- Integration that works because quality was clear from the start

### Constitution Is One-Time, Project Work Is Repetitive

This is crucial: You write the Constitution **once per project**. Then, for each paper you write, you:

1. Write a specification (addressing this paper only)
2. Generate a plan
3. Generate tasks
4. Implement the paper with AI collaboration

But you never rewrite the Constitution for each paper. It's the foundation everything builds on.

**Best Practice Pattern**:

```
1. Initialize research project
2. Write Constitution (quality standards for ALL papers in this project)
3. Commit Constitution to git
4. FOR EACH PAPER:
   - Run /sp.specify (new specification for this paper)
   - Run /sp.clarify (refine specification)
   - Run /sp.plan (new plan for this paper)
   - Run /sp.tasks (new tasks for this paper)
   - Run /sp.implement (write paper with AI collaboration)
   - Commit paper to git
```

#### 💬 Explore This Concept

> "Why does a strong Constitution cascade to all downstream work? What would happen if I wrote a vague Constitution like 'papers should be good' but tried to write a precise specification for a specific research paper?"

---

## Part A: Understanding `/sp.constitution` Command

Before writing your Constitution, let's understand what the `/sp.constitution` command does.

### What `/sp.constitution` Does

The `/sp.constitution` command **interactively guides you** to create a Constitution by asking targeted questions about:

1. **Project intent**: What are you building? What matters most?
2. **Quality principles**: What non-negotiable standards apply to ALL work?
3. **Constraints**: What limitations or requirements always apply?
4. **Success criteria**: How will you know if quality standards are met?
5. **Technical requirements**: What tools, formats, or processes are required?

**The command does NOT**:
- Write your Constitution for you (you make decisions)
- Assume your project type (it asks)
- Copy templates blindly (it personalizes to YOUR project)

### Constitution Structure

When you run `/sp.constitution`, the agent creates a document with sections like:

```markdown
# Project Constitution

## Section 1: Core Principles
[Your project's non-negotiable values]

## Section 2: Quality Standards
[Testable criteria for evaluating work]

## Section 3: Technical Requirements
[Tools, formats, processes required]

## Section 4: Constraints
[Limitations that always apply]

## Section 5: Success Criteria
[How to verify quality standards are met]
```

Each section contains **specific, testable** statements—not vague aspirations.

#### 🎓 Expert Insight

> In specification-driven development, the Constitution isn't bureaucracy—it's leverage. Write it once with clear, testable standards ("all papers must cite primary sources and pass plagiarism checking before submission"), and every specification and plan you create automatically inherits those standards. Vague Constitutions produce vague downstream work. Precise Constitutions produce precise downstream work.

---

## Part B: Create Your Research Paper Constitution

Now you'll create a Constitution for a research paper project using `/sp.constitution`.

### Step 1: Plan Your Constitution (5 minutes)

Before running the command, think about your research paper project. Answer these questions:

**About your project**:
- What is the research paper about?
- Who is the audience (academic, general public, specific discipline)?
- What's most important: accuracy? Clarity? Innovation? Accessibility?

**About your quality standards**:
- What citation standard matters (APA, Chicago, MLA)?
- Must sources be primary or secondary or both?
- How will you verify claims are accurate?
- What writing clarity is expected?

**About constraints**:
- How long should papers be (word count range)?
- How many sources are required?
- Are there format requirements (margins, fonts, spacing)?

### Step 2: Run `/sp.constitution` Command

Open your agent (Claude Code or similar) and run:

```bash
/sp.constitution
```

**Then describe your research paper project**. Example prompt:

```
/sp.constitution

Project: Research paper on AI-native software development

Core principles:
- Accuracy through primary source verification
- Clarity for academic audience (computer science background)
- Reproducibility (all claims cited and traceable)
- Rigor (only peer-reviewed sources or authoritative technical documentation)

Key standards:
- All factual claims must be traceable to sources
- Citation format: APA style
- Source types: minimum 50% peer-reviewed articles
- Plagiarism check: 0% tolerance before submission
- Writing clarity: Flesch-Kincaid grade 10-12

Constraints:
- Word count: 5,000-7,000 words
- Minimum 15 sources
- Deadlines: draft by Dec 1, final by Dec 15
- Format: PDF with embedded citations

Success criteria:
- All claims verified against sources
- Zero plagiarism detected
- Passes fact-checking review
- Meets word count and source requirements
```

**What the agent does**:
- Asks clarifying questions if needed
- Creates a comprehensive Constitution file
- Defines testable quality standards
- Documents all constraints and success criteria
- Shows you the generated Constitution

### Step 3: Review and Improve Your Constitution

After the agent generates your Constitution, **review it carefully**:

**Check these aspects**:

1. **Are standards testable?** (NOT vague like "good writing")
   - ❌ Vague: "Papers should be well-written"
   - ✅ Testable: "Papers use active voice 75% of time; Flesch-Kincaid grade 10-12"

2. **Are constraints explicit?** (NOT assumptions)
   - ❌ Assumption: "Sources should be recent"
   - ✅ Explicit: "Primary sources published within 10 years; seminal papers published within 30 years"

3. **Are success criteria measurable?** (NOT subjective)
   - ❌ Subjective: "Paper is well-researched"
   - ✅ Measurable: "15+ sources; 50%+ peer-reviewed; all claims cited"

4. **Does it cover all essential categories?**
   - Quality standards (citation accuracy, source types, writing clarity)
   - Constraints (length, format, deadline)
   - Academic integrity (plagiarism checking, source verification)
   - Review process (how feedback loops work)

**If anything is vague or missing, ask the agent**:

```
Review my Constitution and suggest improvements:
(1) Which standards are testable vs vague?
(2) What critical standards am I missing?
(3) Are constraints realistic for this research project?
Suggest 2-3 concrete improvements.
```

### Step 4: Commit Constitution to Git

Once your Constitution is complete and reviewed, commit it:

```bash
/sp.git.commit_pr Commit research paper constitution to feature branch
```

**Why commit first?**
1. **Immutability**: Constitution is foundational; committing signals "this is our standard"
2. **Clarity**: Everyone sees Constitution as baseline for all paper work
3. **Traceability**: Git history shows when and why Constitution was created
4. **Reversibility**: You can revert to previous Constitution if needed (rarely)

---

## Part C: How Constitution Guides Downstream Phases

Now that you've created a Constitution, let's see how it cascades through every other phase.

### Constitution → Specification Phase

When you write a specification for a **specific research paper**, your Constitution automatically influences it:

**Your Constitution says**:
```
- All papers must cite primary sources
- Minimum 50% peer-reviewed sources
- APA citation format
- Flesch-Kincaid grade 10-12
- Zero plagiarism tolerance
```

**Your Specification for Paper #1 must respect this**:
```
This specification inherits Constitution standards:
- Thesis: "Explain ML model interpretability for non-technical audience"
- Length: 5,000 words
- Sources: minimum 15 total, minimum 8 peer-reviewed, minimum 5 primary
- Format: APA style (inherited from Constitution)
- Deadline: December 15
- Success criteria: Thesis supported by cited sources, Flesch-Kincaid 10-12
```

Notice: You DON'T re-specify citation format or plagiarism checking. The Constitution already requires it.

### Constitution → Plan Phase

When you create a plan for research, your Constitution shapes the planning:

**Your Constitution says**:
```
- All claims must be verified against sources
- Plagiarism check required before submission
- Fact-checking review process in place
```

**Your Plan must account for this**:
```
Phase 1: Research and source identification (find sources meeting Constitution standards)
Phase 2: Detailed outline with source assignments (map claims to sources)
Phase 3: Draft writing (write with inline citations as per Constitution)
Phase 4: Fact-checking pass (verify all claims against sources)
Phase 5: Plagiarism scanning (check against Constitution's 0% tolerance)
Phase 6: Final review and submission
```

Notice: The plan INCLUDES verification steps because Constitution REQUIRES them.

### Constitution → Task Phase

When you break work into tasks, Constitution constraints apply:

**Your Constitution says**:
```
- Maximum 7,000 words
- Minimum 15 sources (50%+ peer-reviewed)
- APA format
- Deadline: December 15
```

**Your Tasks must respect these**:
```
Task 1: Find 15+ sources (prioritize peer-reviewed)
Task 2: Create outline with source assignments
Task 3: Write Section 1 (Introduction, ~1,200 words with citations)
Task 4: Write Section 2 (Literature Review, ~2,000 words with citations)
Task 5: Write Section 3 (Analysis, ~2,000 words with citations)
Task 6: Write Conclusion (~800 words)
Task 7: Format all citations in APA style
Task 8: Run plagiarism check (must be 0% before submission)
Task 9: Fact-check all claims
Task 10: Final review and submission
```

Every task respects Constitution constraints (word count, sources, format, plagiarism check).

### Constitution → Implementation

When you write the paper with AI collaboration, Constitution standards guide every interaction:

```
You: "Write the Introduction section for my research paper.
Use these sources: [list].
Follow the Constitution: APA citations, Flesch-Kincaid 10-12,
verify all claims against sources."

AI: "I'll write the introduction accounting for:
- In-text APA citations for each claim
- Plain language targeting grade 10-12 reading level
- Verification of claims against provided sources
- ~1,200 words to fit your Constitution word-count budget"

[AI writes introduction with citations and source verification]

You: "This is good, but the third claim about ML interpretability
needs a primary source, not just secondary. The Constitution requires
50% primary sources. Can you verify this claim?"

AI: "You're right. Let me check the sources... The primary research
on LIME (Local Interpretable Model-Agnostic Explanations) is at [source].
I'll revise to cite this primary paper instead."

[AI revises with primary source citation]
```

Notice: Constitution REQUIREMENTS (primary sources, APA style, verification) shape every interaction.

#### 🤝 Reflection Exercise

> Think about YOUR Constitution for the research paper project. How would these standards change the way you write the paper? What would be different if your Constitution required "primary sources only" vs "primary sources not required"? How would that change your Plan? Your Tasks? Your Implementation?

---

## Common Mistakes

### Mistake 1: Writing Too General a Constitution

**The Error**: "All papers must be good quality and well-researched"

**Why It's Wrong**: "Good" and "well-researched" are subjective. No one can verify these or test them during review.

**The Fix**: Use testable criteria:
- ❌ Vague: "Papers should be good quality"
- ✅ Testable: "Papers must pass plagiarism check; all claims verified against sources; Flesch-Kincaid grade 10-12; APA citations"

### Mistake 2: Constitution Too Specific to One Paper

**The Error**: "This Constitution applies only to my AI research paper"

**Why It's Wrong**: You can't reuse it for future papers. Constitution should guide ALL papers in your project.

**The Fix**: Write Constitution that applies to ANY paper:
- ❌ Too specific: "AI research papers must cite these 5 AI researchers"
- ✅ General: "All papers must cite primary sources; minimum 50% peer-reviewed; APA citations"

### Mistake 3: Forgetting to Commit Constitution

**The Error**: Create Constitution, then start writing spec without committing

**Why It's Wrong**: Constitution becomes "whatever I remember" instead of "documented standard." No traceability.

**The Fix**: Always commit Constitution first:
```
1. Create Constitution
2. Commit Constitution (git add + git commit)
3. THEN start /sp.specify for specific papers
```

---

## Try With AI

Ready to validate your Constitution and understand how quality standards cascade through your research work? Explore these prompts with your AI companion:

**Setup**: You've created a Constitution for your research paper project with standards for citation accuracy, source verification, and writing clarity.

**Explore Cascade Effect:**
> "I created a Constitution for my research paper project with standards for: (1) all claims must cite primary sources, (2) writing clarity must be Flesch-Kincaid grade 10-12, (3) zero plagiarism tolerance before submission. Explain how these Constitution rules cascade into downstream phases: How do they affect the Specification I write for a specific paper? How do they affect the Plan I create? How do they affect the Tasks I break down? Give me a concrete example."

**Test Constitution Completeness:**
> "Review my Constitution at `.specify/memory/constitution.md`. Check for: (1) Are all quality standards testable and specific (not vague)? (2) Did I cover essential categories (citation standards, source types, writing clarity, plagiarism checking, review process)? (3) Are any standards unrealistic or conflicting? Suggest 2-3 concrete improvements to make it clearer and more practical for research papers."

**Validate Against Specification:**
> "Based on my Constitution, what requirements MUST a specification for a specific research paper include? Walk through: (1) Citation and source requirements inherited from Constitution, (2) Plagiarism checking and fact-verification requirements, (3) Writing clarity and format standards. This shows if my Constitution is specific enough to guide paper specifications."

**Apply to Your Domain:**
> "Help me draft a Constitution for [describe your research project: climate policy analysis / medical literature review / AI ethics research]. What quality standards, citation requirements, source verification, plagiarism checking, and writing clarity standards should I define? How would Constitution rules differ for different research domains?"

---
