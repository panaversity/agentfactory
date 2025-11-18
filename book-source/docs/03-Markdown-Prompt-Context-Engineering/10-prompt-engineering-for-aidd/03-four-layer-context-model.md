# Lesson 3: The 4-Layer Context Model — First AI Collaboration

---

## What Should This Lesson Enable You to Do?

By the end of this lesson, you should be able to:

- **Understand the 4-layer context model** (Project → Code → Constraints → Analyst) and why each layer matters
- **Provide structured context to AI agents** so they can reason accurately about complex codebases
- **Recognize that AI output quality depends on input context quality**, not just AI capability
- **Apply the 4-layer model in bidirectional collaboration** with AI (teaching and learning together)
- **Validate AI's reasoning** against your knowledge of the codebase and constraints

**Success criteria**: You can provide 4-layer context to an AI agent for a real codebase analysis and evaluate whether its reasoning is accurate based on domain knowledge.

---

## The Strategic Context Problem

### The Scenario

You are a **Developer** evaluating whether to adopt a new Python library for your team's project. The library handles markdown parsing, and your team needs to generate documentation from Python docstrings. However, the library's documentation is extensive (20+ pages), and you have **30 minutes** to assess whether it fits your needs before the team meeting.

You open the library's documentation with Claude Code and start asking questions. But here's the problem:

> **AI agent sees 20 documentation pages, 50+ code examples. It has ZERO context about:**
> - What problem you're solving (docstring → markdown conversion)
> - What your current workflow looks like (manual docs vs. automated)
> - What "good fit" means for your project (simple API, extensibility, maintenance)
> - What questions matter most (ease of use? performance? compatibility?)

**Without context, AI will:**
- Summarize all features instead of assessing fit for your use case
- Miss workflow integration concerns specific to your team
- Suggest features that don't apply to your constraints

**With context, AI will:**
- Focus on what matters to you (docstring conversion workflow)
- Identify specific integration points with your toolchain
- Provide decision-ready assessment in 30 minutes

**This lesson teaches you to provide that context.**

---

## Concept 1: The Four Layers of Context

### Why Context Matters

Before you can effectively ask an AI agent to analyze code, you need to answer a meta-question: **What should the AI know to answer my question well?**

That answer has **four layers**.

### Layer 1: Project Context (The "Why")

**What goes here**: The business/strategic context. Why are you asking this question?

Example:
```
We're evaluating this markdown library for our documentation generation workflow.
Our product is a Python CLI tool for internal developers.
We have 50+ docstrings to convert, need automated updates on each release.
We need to know if this library's API fits our docstring → markdown workflow.
```

**Effect**: With this context, the AI focuses on workflow suitability rather than generic feature coverage—treating this as a tooling decision, not academic analysis.

### Layer 2: Code Context (The "What")

**What goes here**: Information about the documentation itself. What are you analyzing?

Example:
```
Documentation: Python markdown library, 20 pages, 50+ code examples
Core features: Markdown parsing, table support, extensions API, syntax highlighting
Input formats: Plain text, reStructuredText, Python docstrings
Output formats: HTML, Markdown, PDF
Key use cases: Documentation generation, blog rendering, note-taking apps
```

**Effect**: The AI can now ask targeted questions about scope and structure ("How does docstring parsing work?" instead of "What are the features?").

### Layer 3: Constraints Context (The "How")

**What goes here**: What limits, requirements, and technical dependencies exist?

Example:
```
Our workflow: Python 3.11+, Sphinx documentation, GitHub Actions CI/CD
Team experience: Python-fluent, familiar with Sphinx, no markdown library experience
Integration points: Must work with existing Sphinx pipeline, output to docs/ folder
Performance: Process 50 docstrings in < 10 seconds (CI/CD bottleneck)
Constraints: Cannot rewrite existing docstrings (Google-style format)
Timeline: 30 minutes to assessment for team meeting
```

**Effect**: The AI can now assess workflow fit based on specific requirements ("Does this library fit OUR workflow?" instead of "Is this a good library in general?").

### Layer 4: Analyst Context (The "Who")

**What goes here**: Who is asking the question, and what do they need to know?

Example:
```
Asking as: Mid-level developer (technical decision-maker for tooling)
Decision needed: Adopt this library or continue manual docs? Time pressure: moderate
Background: I understand Python, Sphinx, docstrings, markdown basics
I do NOT understand: Advanced markdown extensions, library internals
What I need: Integration steps, example workflow (docstring → markdown), performance data
Decision threshold: If integration < 1 day AND performance acceptable, we adopt. If > 3 days, we keep manual process.
```

**Effect**: The AI can now explain at the right level (not too basic, not too expert), focus on what matters for this decision, and flag issues relevant to this specific decision-maker.

### The Four Layers Visualized

```
Layer 4: Analyst Context     ← WHO is asking? (Developer, 30-min timeline, moderate pressure)
         ↓
Layer 3: Constraints Context ← HOW are we constrained? (Sphinx, CI/CD, Google-style docstrings)
         ↓
Layer 2: Code Context        ← WHAT are we analyzing? (Markdown library docs, 20 pages, 50 examples)
         ↓
Layer 1: Project Context     ← WHY are we asking? (Documentation automation decision)
```

Each layer narrows the focus and makes the analysis more specific to your situation.

---

## Concept 2: Iterative Context Refinement (Socratic Discovery)

### The Problem: Incomplete Context

When you first provide context, it's often incomplete. **That's normal.** The way to fix it is through collaboration.

### Socratic Questions to Refine Context

As you work with an AI agent analyzing code, these questions help refine context iteratively:

**Question 1**: "What did I NOT tell you that would change your analysis?"

This forces the AI to articulate what it's uncertain about. You then provide that missing context.

**Question 2**: "What assumptions are you making about our integration constraints?"

This reveals where the AI might be guessing. You correct those assumptions.

**Question 3**: "How would your assessment change if [constraint]?"

This tests whether the AI truly understands your situation.

**Question 4**: "Is there a scenario where this architecture wouldn't work for us?"

This forces edge-case thinking. Often, your team's experience reveals hidden constraints.

---

## Concept 3: Three Roles in Context Collaboration

### This is Where AI Becomes a Learning Partner

So far, you've learned **how to provide context**. Now comes the powerful part: **using context to collaborate with AI**.

In this collaboration, AI plays three roles. **For the first time in this chapter, we're actually using an AI agent.** Watch how the bidirectional learning works.

---

## Three Roles Demonstration: Evaluating a Markdown Library

### Setup

Imagine you've provided the 4-layer context above to Claude Code. You show it the markdown library's documentation and ask: "Does this library fit our documentation workflow?"

The AI will play three roles as you work together.

---

### Role 1: AI as Teacher (Suggests Pattern You Didn't Know)

**You ask Claude Code**:
```
Analyze this markdown library's documentation. Does it fit our docstring → markdown workflow?
```

**AI responds** (simplified):
```
I notice the library has a docstring parser extension that specifically handles
Google-style docstrings (which you mentioned using). This is important because:
1. Native support (no preprocessing needed)
2. Preserves type hints (converts to markdown tables automatically)
3. Handles complex signatures (multi-line, defaults, decorators)

Most markdown libraries treat docstrings as plain text, requiring you to
write custom parsers. This library has built-in support, which I see in
the "Extensions" section of their docs.

What you may not have realized: The extension includes Sphinx compatibility mode,
meaning you can keep your existing Sphinx configuration and just swap the backend.
```

**What you discovered**: The Google-style docstring extension—a feature you weren't aware of—directly solves your workflow problem. This is **AI as Teacher**.

**Evidence**: The AI identified a specific feature (docstring parser extension) and explained why it matters for YOUR specific workflow (preserves Sphinx config, handles Google-style). This is reasoning, not just summarizing features.

---

### Role 2: AI as Student (Adapts to Your Feedback)

**You respond** (teaching AI):
```
Good point on the docstring extension. BUT — I checked our CI/CD pipeline,
and we're running on GitHub Actions with strict 10-second timeout.
Their documentation mentions "streaming parser" but doesn't specify performance.

Will 50 docstrings process in < 10 seconds?
```

**AI responds** (adapted to your correction):
```
Ah, thank you for that constraint. That changes the recommendation significantly.

Looking at the performance section (page 15), I see:
- Streaming parser handles 1000 docstrings in ~2 seconds (their benchmark)
- But that's for simple docstrings (no tables, no complex types)
- Your Google-style docstrings with type hints are likely 3-5x slower

New assessment: 50 docstrings should take 1-2 seconds (well under 10-second limit).
But if you add table generation or syntax highlighting extensions, could reach 8-9 seconds.

What I initially missed: CI/CD timeout is a hard constraint. I was focusing on
features, not performance. Your 10-second limit is critical—recommend testing
with realistic docstrings before full adoption.
```

**What changed**: The assessment recalibrated from "feature fit" to "performance fit" based on the CI/CD constraint you provided. This is **AI as Student**.

**Evidence**: The AI initially focused on features (docstring extension). You introduced the performance gap. The AI incorporated your constraint and recalibrated its recommendation. This is bidirectional learning.

---

### Role 3: AI as Co-Worker (Convergence Through Iteration)

**You continue** (convergence loop):
```
If performance is borderline (8-9 seconds with extensions), can we optimize?
What if we disable syntax highlighting in CI/CD and only enable it for local builds?
```

**AI responds**:
```
That's a smart approach. Here's how selective extension loading would work:

1. Configuration file (markdown.yaml) controls which extensions load
2. CI/CD mode: docstring parser only (fastest, 1-2 seconds)
3. Local builds: full extensions (syntax highlighting, diagrams) for review
4. This keeps CI fast while maintaining rich local docs

With selective loading: CI stays under 5 seconds, local builds have full features.

I initially didn't consider this approach because I was thinking of
all-or-nothing adoption. You suggested the configuration pattern that
optimizes for both speed (CI) and richness (local).

So our assessment evolved:
- Initial: "Good fit, has docstring extension" (feature-focused)
- Second: "Borderline, performance might exceed 10 seconds" (constraint-aware)
- Final: "Strong fit with selective extension loading" (optimized solution)
```

**What emerged**: The final solution combined insights from both sides:
- The docstring parser extension (discovered through AI's documentation analysis)
- The 10-second CI constraint (critical context you provided)
- The selective loading approach (designed through collaborative iteration)

**This is AI as Co-Worker**: Convergence through iteration toward a better solution.

**Evidence**: The final workflow emerged from bidirectional learning, not from AI's initial knowledge or your initial question.

---

## Concept 4: Validating AI's Reasoning

### The Critical Skill: "Is the AI Right?"

After AI provides analysis based on context, **you must validate it**. Just because AI has context doesn't mean its reasoning is correct.

### Validation Framework

**Step 1: Check for Factual Accuracy**
```
AI claims: "The docstring parser extension handles Google-style docstrings"
Your check: Scan the docs. Find Extensions section. Is this feature documented? ✓
```

**Step 2: Check for Constraint Understanding**
```
AI claims: "50 docstrings process in 1-2 seconds with selective loading"
Your check: Does the AI understand your 10-second CI timeout correctly?
Did it account for GitHub Actions overhead? ✓/✗
```

**Step 3: Check for Assumption Clarity**
```
AI claims: "Selective extension loading assumes you can configure per-environment"
Your check: Is that assumption valid? Can you control CI config separately from local? ✓
```

**Step 4: Ask Edge Cases**
```
Your question: "What if we add 200 more docstrings next quarter?
Will performance still be acceptable with selective loading?"
AI response: [Should address scaling, configuration limits, performance degradation]
```

### When to Trust AI's Reasoning

✅ Trust AI when:
- You can verify it against the documentation
- It explains reasoning, not just summarizing features
- It accounts for YOUR constraints (not generic feature list)
- It flags uncertainty ("I'm not sure if...")

❌ Don't trust AI when:
- It generalizes without referencing specific documentation sections
- It ignores constraints you mentioned
- It presents alternatives without tradeoffs
- It claims certainty about undocumented behavior

---

## Concept 5: Applying 4-Layer Context to Your Work

### The Template You'll Use

Every time you ask an AI agent to analyze documentation or evaluate a tool, provide this context upfront:

```markdown
## Layer 1: Project Context

**Decision needed**: [What are you deciding?]
**Use case**: [Why does this matter?]
**Timeline**: [How much time do you have?]
**Stakes**: [What's the impact if you choose wrong?]

## Layer 2: Documentation Context

**Content type**: [What are you analyzing?]
**Scope**: [How much content?]
**Key features**: [What matters most?]
**Current understanding**: [What do you know so far?]

## Layer 3: Constraints Context

**Technical constraints**: [What must be true?]
**Team constraints**: [What skills do you have?]
**Integration points**: [What must connect?]
**Performance requirements**: [What are the limits?]

## Layer 4: Analyst Context

**Your role**: [Why are you the asker?]
**Your expertise**: [What do you know?]
**Your knowledge gaps**: [What do you NOT understand?]
**Your decision threshold**: [What would make you decide?]
```

### Exercise: Provide Context for Your Scenario

**Scenario**: You're evaluating a documentation generator tool (MkDocs) for your project.

**Your task**: Write a 4-layer context that an AI agent could use to analyze whether MkDocs fits your needs.

**Hint**:
- Layer 1: What decision are you making?
- Layer 2: What are you analyzing? (Documentation tool features?)
- Layer 3: What constraints exist? (Workflow? Team? Integration?)
- Layer 4: What expertise do you have?

(Example answer below)

---

## Synthesis: Context as the Bridge

### The Journey from Question to Intelligence

1. **Without context**: "Evaluate this library" → Generic feature list
2. **With Layer 1**: "We're deciding on documentation automation" → Decision-focused evaluation
3. **With Layer 2**: "20 pages docs, 50 examples, docstring features" → Targeted analysis
4. **With Layer 3**: "Must integrate with Sphinx, 10-second CI limit" → Constraint-aware assessment
5. **With Layer 4**: "Mid-level dev, Python-fluent, 30 minutes" → Right-level explanation

**Each layer transforms vague requests into specific intelligence.**

### Why This Matters: Context Multiplies Capability

You don't need smarter AI. You need **better context**. And context comes from YOU.

The magic isn't that AI is intelligent. It's that **when you provide structured context, both you and AI think more clearly.**

---

## Self-Assessment: Context-Building Exercise

### Exercise 1: Audit Your Context

Think of a time you asked an AI agent a question about code and got an unhelpful answer.

**Diagnosis**: Which layer of context were you missing?
- Layer 1 (Project): Did the AI know WHY you were asking?
- Layer 2 (Code): Did the AI understand WHAT you were analyzing?
- Layer 3 (Constraints): Did the AI know your HOW constraints?
- Layer 4 (Analyst): Did the AI understand WHO was asking?

**Reflection**: How would providing that missing layer have changed the answer?

---

### Exercise 2: Three Roles Recognition

Read this dialogue:

**You**: "Does this authentication code have security vulnerabilities?"

**AI**: "It uses bcrypt for password hashing, which is strong. However, it doesn't implement rate limiting on login attempts, which is a vulnerability."

**Question**: Which role did AI play?
- AI as Teacher? (Yes/No - explain)
- AI as Student? (Yes/No - explain)
- AI as Co-Worker? (Yes/No - explain)

**Answer**:
- **AI as Teacher**: Yes. You asked about "security vulnerabilities" (vague), and the response introduced the specific concept of rate limiting as a security pattern.
- **AI as Student**: Not in this exchange (no feedback loop yet).
- **AI as Co-Worker**: Not yet (need iteration).

---

## Try With AI

**This is the FIRST lesson in Chapter 10 where you use AI tools.** You've built foundation (Lessons 1-2) and learned the framework (this lesson). Now apply it.

### Hands-On Exercise: Provide Context and Analyze

**Your task**: Use Claude Code (or your preferred AI tool) to practice 4-layer context with a real (or sample) codebase.

**Step 1: Prepare Your Context (15 minutes)**

Use the template from Concept 5 to write 4-layer context for a codebase you know. Examples:
- A Python project you've worked on
- An open-source repository you want to evaluate
- A contractor's code you're assessing

**Step 2: Provide Context to AI (5 minutes)**

Open Claude Code or your AI tool. Paste your 4-layer context. Then ask one question:
```
Based on this context, does this codebase architecture work for our use case?
```

**Step 3: Validate AI's Reasoning (10 minutes)**

Read the AI's response. Check:
- Does it reference your specific constraints?
- Does it show understanding of your decision timeline?
- Does it explain REASONING, not just features?

**Step 4: Iterate (10 minutes)**

Ask a follow-up question that teaches the AI something:
```
But wait—our team doesn't know [technology].
Does that change your assessment?
```

Observe how AI adapts.

**Step 5: Reflect**

Write 3-5 sentences answering:
- How did providing 4-layer context change the AI's response compared to asking without context?
- Which layer mattered most for your specific question?
- What did you learn from the AI? What did the AI learn from you?

---

## Expected Outcomes

After this exercise, you should notice:

✅ **AI responses are more specific** (focused on YOUR constraints, not generic)

✅ **AI asks clarifying questions** (because context was structured, AI knows what's important)

✅ **You catch errors** (because you're validating reasoning, not trusting blindly)

✅ **Iteration improves both sides** (you teach AI about your domain, AI teaches you about patterns)

**This is the beginning of strategic AI collaboration.**

---

## Lesson Metadata

- **Stage**: 2 (AI Collaboration — Three Roles demonstration ✅)
- **Modality**: Specification-first + Socratic dialogue + AI application
- **Concepts**: 7 (4-layer context, project/code/constraints/analyst context, iterative refinement, Three Roles, validation, context quality impacts output)
- **Cognitive Load**: 7 ≤ 10 (B1 tier Stage 2 limit ✅)
- **AI Tools Used**: Claude Code or Gemini CLI (FIRST usage in chapter)
- **Duration**: 60-70 minutes
- **Three Roles Demonstrated**: ✅ Teacher, ✅ Student, ✅ Co-Worker (all explicit)
- **Version**: 1.0.0
- **Constitution**: v6.0.0 Compliance (Stage 2 three-roles requirement met)
- **Generated**: 2025-01-18
- **Feature**: 025-chapter-10-redesign
- **Source Spec**: `specs/025-chapter-10-redesign/plan.md` (Lesson 3 structure)

---

## Answer Key: Exercises

### Exercise from Concept 5 (4-Layer Context for MkDocs evaluation)

```markdown
## Layer 1: Project Context

**Decision needed**: Adopt documentation generator for Python project
**Use case**: Technical documentation (API reference, tutorials, guides) for open-source library (5K GitHub stars)
**Timeline**: 2 weeks to decision, 3 weeks to migration from existing Sphinx setup
**Stakes**: If we choose wrong tool, 6 months of contributor docs become incompatible, community trust damaged

## Layer 2: Documentation Context

**Content type**: MkDocs documentation (static site generator)
**Scope**: 30+ page docs, Markdown-based, includes code examples, API reference, tutorials
**Key features**: Markdown support, theme customization, search, versioning, plugin ecosystem
**Current understanding**: Sphinx user (3 years), new to MkDocs, heard it's "simpler"

## Layer 3: Constraints Context

**Technical**: Must support Markdown, Google-style docstrings, GitHub Pages deployment
**Team**: 3 core maintainers (Python-fluent), 20 occasional contributors (varying skill levels)
**Integration**: Must work with existing CI/CD (GitHub Actions), support versioned docs (v1.0, v2.0)
**Performance**: Build time < 30 seconds (contributors test locally frequently)

## Layer 4: Analyst Context

**Your role**: Open-source maintainer deciding on documentation tooling
**Your expertise**: 5 years Python, 3 years Sphinx/reStructuredText, basic Markdown knowledge
**Knowledge gaps**: MkDocs plugin ecosystem, migration effort from Sphinx, Markdown limitations vs. reST
**Decision threshold**: If migration < 1 week AND contributor experience improves (Markdown easier), adopt MkDocs

---

**Your question to AI**: Given this context, should we migrate from Sphinx to MkDocs for our open-source project?
```

This context guides AI to give you a decision-ready assessment, not a generic MkDocs feature list.
