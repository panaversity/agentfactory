---
title: "Specify Phase"
chapter: 14
lesson: 4
duration_minutes: 45
proficiency_level: "A2-B1"

learning_objectives:
  - objective: "Execute the `/sp.specify` command to convert vague requirements into clear specifications"
    bloom_level: "Apply"
    assessment_method: "Student produces valid spec.md with all required sections"

  - objective: "Write specification with clear Intent, Constraints, Success Evals, and Non-Goals"
    bloom_level: "Apply"
    assessment_method: "Specification review checklist confirms all sections present and SMART"

  - objective: "Distinguish what from how (specification vs implementation)"
    bloom_level: "Understand"
    assessment_method: "Student identifies implementation details leaked into spec and corrects them"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (Specification definition, `/sp.specify` command, SMART success criteria, Non-Goals scope boundaries) within A2-B1 limit ✓"

differentiation:
  extension_for_advanced: "Write specification for different types of documents (blog post, technical guide, presentation outline); compare how success criteria differ by document type"
  remedial_for_struggling: "Use provided specification template; fill in essential sections (Intent, Success Evals) before moving to Clarify phase"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Specify Phase

Welcome to the fourth lesson of hands-on SDD-RI development. You've established your project's global rules in the Constitution phase. Now it's time to specify what you're actually building: **a clear, measurable description of one feature**.

In this lesson, you'll learn the `/sp.specify` command—the tool that transforms vague ideas into actionable specifications. You'll practice by specifying a research paper, then apply the same approach to any project.

---

## The Core Skill: Writing Clear Specifications

Before showing you the command, let's understand what a specification actually is.

### What Is a Specification?

A **specification** answers one question: **What are we building?**

It is NOT a plan (how), NOT a task list (what steps), NOT code. It is a description of intent, constraints, success criteria, and scope boundaries.

**Example: Research Paper Project**

Instead of vague ideas:
> "Write a paper on artificial intelligence and education"

A specification makes it concrete:
> - **Intent**: 3000-5000 word research paper on AI's impact on K-12 classroom efficiency
> - **Success**: Paper defines 3+ concrete AI applications, includes 8+ academic sources, reader understands economic ROI
> - **Constraints**: APA format, markdown source, no images (text only for MVP)
> - **Not Building**: Literature review of entire AI field, implementation guide, code examples

Now you know exactly what "done" looks like.

### Why Specifications Matter

Specifications are the foundation of specification-driven development. Here's why:

1. **Clarity for AI**: A clear spec produces clear outputs. A vague spec produces confusion.
2. **Measurable Success**: You can verify success objectively, not subjectively ("looks good" is not measurable).
3. **Scope Management**: Non-Goals prevent creep ("Can we add images?" → "No, not in this spec").
4. **Reduced Rework**: Clear intent prevents misunderstandings that lead to rewrites.

---

## Understanding the `/sp.specify` Command

The `/sp.specify` command is a Spec-Kit Plus tool that guides you through writing a specification.

### Command: `/sp.specify`

**What it does**: Converts a vague feature idea into a structured specification document

**When to use it**: When you have an idea and need to clarify exactly what success looks like before planning implementation

**Output**: A `spec.md` file with Intent, Constraints, Success Evals, and Non-Goals sections

### Command Syntax

In your Claude Code terminal:

```bash
/sp.specify "Your feature idea or problem to solve"
```

**Example**:
```bash
/sp.specify "Research paper on AI's impact on K-12 classroom efficiency"
```

The command then guides you through:
1. **Clarifying Intent**: What problem does this feature solve?
2. **Defining Success Evals**: How do we know it worked? (Measurable outcomes)
3. **Listing Constraints**: What are the limits? (Format, length, scope)
4. **Acknowledging Non-Goals**: What are we NOT building? (Explicit boundaries)

---

## The Four Required Sections

Every specification has these sections. They're the backbone of clear communication.

### Section 1: Intent

**Purpose**: Describes what problem this feature solves and who benefits

**Questions to answer**:
- What are we building?
- Why are we building it?
- Who will use it?
- What problem does it solve?

**Example (Research Paper)**:
```markdown
## Intent

Write a 3000-5000 word research paper exploring artificial intelligence's impact
on classroom efficiency in K-12 education. Target audience: education administrators
and technology decision-makers. Purpose: Provide evidence-based analysis of AI's
role in reducing teacher workload and improving student outcomes.
```

**Strong intent**: Specific, contextual, audience-aware
**Weak intent**: "Write a paper about AI and education"

### Section 2: Constraints

**Purpose**: Defines the boundaries, technical requirements, and limitations

**Questions to answer**:
- What's the scope? (Length, time, format)
- What tools/platforms do we use?
- What are the technical requirements?
- What limits do we respect?

**Example (Research Paper)**:
```markdown
## Constraints

- Word count: 3000-5000 words
- Format: Markdown source file, APA citation style
- Sources: Academic peer-reviewed journals only (minimum 8 sources)
- Deadline: Written within 2 weeks
- No visual elements (text-based only for MVP)
- Time investment: Max 20 hours research and writing
```

**Strong constraints**: Specific, testable, realistic
**Weak constraints**: "Make it professional", "Use good sources"

### Section 3: Success Evals

**Purpose**: Defines measurable success criteria—how you'll know this is "done"

**Questions to answer**:
- What does success look like?
- How do we verify it objectively?
- What's measurable vs subjective?

**Example (Research Paper)**:
```markdown
## Success Evals

- ✓ Paper is between 3000-5000 words
- ✓ Paper identifies 3+ concrete AI applications in K-12 classrooms
- ✓ At least 8 citations from peer-reviewed academic sources
- ✓ Each major claim is supported by evidence
- ✓ Reader can explain ROI (return on investment) of classroom AI after reading
- ✓ Paper is valid Markdown with proper APA formatting
- ✓ Paper completed within 2-week timeframe
```

**Strong evals**: Specific, observable, measurable
**Weak evals**: "Paper is well-written", "Sources are credible"

### Section 4: Non-Goals

**Purpose**: Explicitly states what's OUT of scope to prevent creep and misunderstanding

**Questions to answer**:
- What are we NOT building?
- What features are deferred?
- What could be confusing to clarify?

**Example (Research Paper)**:
```markdown
## Non-Goals

- Not a comprehensive literature review (focused analysis only)
- Not a how-to guide for implementing AI (conceptual analysis only)
- Not a comparison of specific AI tools (focus on application patterns)
- No implementation code examples
- No discussion of ethical concerns (separate paper)
- Not a quantitative study (qualitative analysis based on existing research)
```

**Strong non-goals**: Clear, specific, prevent common misunderstandings
**Weak non-goals**: "No extra stuff"

---

## The SMART Criteria for Success Evals

Success Evals must be SMART: Specific, Measurable, Achievable, Relevant, Time-bound.

### SMART Checklist

**Is it Specific?**
- ❌ "Paper is high quality"
- ✅ "Paper identifies 3+ AI applications with specific classroom examples"

**Is it Measurable?**
- ❌ "Reader understands the topic"
- ✅ "Reader can explain 3 concrete AI use cases in education"

**Is it Achievable?**
- ❌ "Paper citations come from top 1% most cited journals"
- ✅ "At least 8 citations from peer-reviewed journals"

**Is it Relevant?**
- ❌ "Paper has creative formatting" (doesn't matter for this research paper)
- ✅ "Paper uses APA formatting correctly"

**Is it Time-bound?**
- ❌ "Paper is completed eventually"
- ✅ "Paper is completed within 2 weeks"

---

## Common Specification Mistakes

### Mistake 1: Leaking Implementation Into Specification

**The Error**:
```markdown
## Specification: Research Paper

The paper will be written using Claude AI to:
1. Research the topic
2. Outline the structure
3. Generate each section based on the outline
4. Ask Claude to review and refine

Use these prompts with Claude...
```

**Why It's Wrong**: This is IMPLEMENTATION (HOW). Specification is WHAT. Planning phase (next lesson) is HOW.

**The Fix**:
```markdown
## Intent

5000-word research paper on AI in K-12 education that explains
economic impact and classroom efficiency gains.

## Constraints

- Word count: 5000 ± 500 words
- Sources: Peer-reviewed academic journals only (8+ minimum)
- Format: APA style, markdown source file

## Success Evals

- Paper word count is between 4500-5500 words
- At least 8 academic citations
- Paper structure includes introduction, 3+ findings, conclusion
- Each finding is supported by evidence

## Non-Goals

- No implementation guide
- No discussion of technology challenges
```

[Implementation details go in `/sp.plan`, not here]

### Mistake 2: Subjective Success Criteria

**The Error**:
```markdown
## Success Evals

- Paper is high-quality
- Sources are credible
- Writing is clear
- Reader finds it valuable
```

**Why It's Wrong**: You cannot verify these objectively. Different people will disagree.

**The Fix**:
```markdown
## Success Evals

- Paper cites 8+ sources from peer-reviewed journals
- Average citation date is within past 10 years
- Each paragraph includes at least one citation or supporting evidence
- Paper word count is 5000 ± 500 words
- Paper uses APA formatting correctly
```

### Mistake 3: Vague Intent

**The Error**:
```markdown
## Intent

Write a paper on AI and education.
```

**Why It's Wrong**: Too vague. What aspect of AI? What level of education? What's the focus?

**The Fix**:
```markdown
## Intent

Analyze AI's measurable impact on K-12 classroom efficiency (teacher time savings,
student engagement improvements) based on existing research. Audience: school
administrators evaluating AI adoption. Goal: Provide evidence-based recommendations
for implementation.
```

---

## Writing Your Research Paper Specification

Now it's your turn. Use this process to write a specification for your research paper.

### Step 1: Clarify Your Topic (5 minutes)

Before writing the spec, think about:

1. **What's your research paper about?** (Specific topic, not just "AI")
2. **Who needs this paper?** (Audience context)
3. **What problem does it solve?** (Why does this paper matter?)
4. **How will you know it's done?** (Success indicators)

Take notes. You're creating the raw material for your specification.

### Step 2: Use `/sp.specify` Command

In your Claude Code terminal:

```bash
/sp.specify "Research paper on [your topic]. Target audience: [who]. Goal: [why]."
```

**Example**:
```bash
/sp.specify "Research paper on AI's impact on K-12 classroom efficiency. Target: school administrators. Goal: provide evidence-based ROI analysis."
```

The command will guide you through writing Intent, Constraints, Success Evals, and Non-Goals.

### Step 3: Review Your Specification

After the command generates your spec, check these:

```
Specification Checklist:

[ ] Intent is clear (someone unfamiliar with your project can understand the goal)
[ ] Constraints are specific and testable (not vague "do good work")
[ ] Success Evals are SMART (Specific, Measurable, Achievable, Relevant, Time-bound)
[ ] Non-Goals are explicit (prevents scope creep)
[ ] No "how" leaked in (specification describes what, not how to build)
[ ] Specification is 1-2 pages (concise, not essay-length)
```

### Step 4: Refine Based on Feedback

Ask your AI companion:

```
I've written this research paper specification:

[paste your spec.md]

Can you review it for:
1. Is the intent clear enough for someone to write from?
2. Are the success evals measurable? Can I verify them without ambiguity?
3. What constraints am I missing?
4. Did I accidentally leak implementation details?
5. How would you describe this paper to someone who hasn't read the spec?
```

---

## Specification vs Plan vs Implementation

Here's how to know which phase you're in:

| Phase | Answers | Example |
|-------|---------|---------|
| **Specify** | WHAT are we building? | "5000-word research paper with 3+ AI applications, 8+ sources" |
| **Plan** | HOW will we build it? | "Research step 1→2→3, outline structure, write section by section, get feedback, refine" |
| **Implement** | DO the work | "Search databases, read sources, write intro paragraph, save to file" |

**If you're writing about HOW → You're in Plan or Implementation, not Specification**

---

## Why Research Paper as Example?

The research paper is simple but teaches the same specification skills you'll use for anything:

- **Like a software feature**: Both need clear intent, measurable success, explicit boundaries
- **Like a documentation project**: Both benefit from concrete constraints and SMART criteria
- **Like a deployment**: Both require specification before execution

The framework (Intent → Constraints → Success Evals → Non-Goals) works for any project.

---

## Try With AI

Ready to master the `/sp.specify` command? Practice writing specifications with your AI companion:

**Explore Specification Structure:**
> "I'm learning to write specifications using the `/sp.specify` command. Before I run the command, help me understand: (1) What's the difference between Intent and Non-Goals? (2) How do I write success evals that are measurable instead of subjective? (3) What kind of constraints matter? (4) Can you give me examples of strong vs weak specifications?"

**Practice With Your Topic:**
> "I want to write a research paper on [your topic]. Help me clarify the specification by asking: (1) Who exactly is the audience for this paper? (2) What specific problem does it solve? (3) How will we measure success—what will prove it's a good paper? (4) What aspects of the topic are we NOT covering? Guide me through these before I run `/sp.specify`."

**Test Specification Quality:**
> "After I run `/sp.specify` and get my specification, review it. Tell me: (1) Is the intent clear enough for someone to work from? (2) Are success criteria measurable (not vague)? (3) What constraints am I missing? (4) Did I leak any implementation details? (5) Are the non-goals explicit?"

**Apply to Your Feature:**
> "Help me think through my research paper specification. I'm planning to write about [topic]. The paper needs to [goal]. My audience is [who]. But I'm not sure about: (1) How long should it be? (2) What format and sources? (3) How do I know when it's done? (4) What should I definitely NOT include? Walk me through writing a strong specification."
