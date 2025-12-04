---
title: "Capstone — Intelligence Acceleration"
chapter: 14
lesson: 11
duration_minutes: 60
proficiency_level: "B1"
cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (Skill invocation in implementation, Measuring acceleration, Intelligence compounding, Portfolio value of skills) within B1 limit of 10 ✓"
learning_objectives:
  - objective: "Invoke reusable skills in /sp.implement to execute new tasks faster"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful skill reference in implementation command for second paper section"

  - objective: "Measure intelligence acceleration by comparing effort metrics across projects"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Documented comparison of Lesson 8 (from scratch) vs Lesson 11 (with skill)"

  - objective: "Recognize intelligence compounding patterns across projects"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Articulation of why Project 2 is faster than Project 1, and why Project 3 accelerates further"

  - objective: "Articulate the portfolio value of reusable intelligence skills"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Explanation of how skills represent organizational asset value"

generated_by: "content-implementer v1.0.0"
source_spec: "Chapter 14 Lesson 11 Rewrite: Research Paper Capstone with Skill Reuse"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Capstone — Intelligence Acceleration

You've completed the full SDD-RI workflow across Lessons 1-10. You've written specifications, executed implementation, and created reusable intelligence (the `section-writer` skill from Lesson 9).

Now comes the proof: **Can you complete a second project faster using the intelligence you've built?**

This capstone answers that question decisively. You'll write a second paper section—using the `section-writer` skill created in Lesson 9—and measure how much faster execution becomes. This demonstrates the core principle of SDD-RI: **accumulated intelligence compounds**.

By the end of this capstone, you'll have:
- A complete research paper with multiple sections (portfolio-ready)
- Evidence that your skills accelerated the work
- Understanding of why intelligence accumulation matters more than code libraries
- The ability to tackle future projects with exponentially less effort

---

## The Acceleration Principle

Before diving into the capstone, let's measure what's about to happen.

### Project Timeline: First Section (Lesson 8)

Lesson 8 took you from nothing to a complete, validated paper section (introduction):

| Phase | Lesson | Time Estimate | What You Did |
|-------|--------|----------------|-------------|
| Constitution | 03 | 30 min | Defined research paper quality standards |
| Specification | 04 | 45 min | Wrote paper introduction spec |
| Clarify | 05 | 30 min | Refined spec with research requirements |
| Plan | 06 | 45 min | Designed writing strategy and outline |
| Tasks | 07 | 30 min | Broke down into atomic writing units |
| Implement | 08 | 60 min | Generated, refined, and validated introduction |
| **Total** | | **~3.5 hours** | **Built from scratch** |

You had to:
- Learn how to structure research paper specifications
- Discover how to validate claims against sources
- Troubleshoot writing clarity and academic tone
- Write complete implementation plan
- Handle iteration when first drafts needed refinement

### Project Timeline: Second Section (Lesson 11 - This Capstone)

This lesson takes you from specification to published section using the `section-writer` skill you created:

| Phase | What You'll Do | Time Estimate |
|-------|----------------|----------------|
| Specification | Write main body section spec | 15 min |
| Implement | Execute with `section-writer` skill | 30 min |
| Validation | Verify against spec criteria | 15 min |
| Reflection | Document acceleration | 10 min |
| **Total** | | **~70 minutes** |

The difference: **You're reusing intelligence.**

### Why This Is Faster

Compare the effort:

**Without skill** (hypothetical—writing second section from scratch):
- Discover writing patterns from first section
- Learn what "quality section" means in your context
- Determine how to integrate research sources consistently
- Troubleshoot tone and structure issues
- Estimate: 2-3 hours (similar to first section)

**With skill** (your reality—using `section-writer` skill):
- Reference your `section-writer` skill (encodes writing patterns)
- Let `/sp.implement` invoke that skill
- Validate output
- Done in 70 minutes

**The math**:
- **Lesson 8**: 3.5 hours to build writing intelligence
- **Lesson 11**: 70 minutes using that intelligence
- **Savings**: 2 hours 20 minutes (65% time reduction)
- **But here's the key**: The next paper section will take 45 minutes. The third will take 40 minutes. Intelligence compounds exponentially.

---

## Step 1: Write Your Second Section Specification

Your specification for the main body section should be **intentionally shorter** than your introduction specification (from Lesson 4). This demonstrates specification complexity reduction through skill composition.

### Why This Spec Is Simpler

**Lesson 4 (Introduction) spec required:**
- Research paper structure overview
- Claim definition and validation approach
- Source integration strategy
- Multiple decision points (all handled by you the first time)

**Lesson 11 (Main Body) spec requires:**
- Intent (develop key claims with evidence)
- Constraints (academic standards, source requirements)
- Success Criteria (clarity, evidence completeness)

The upstream complexity is **encapsulated in your skill**. Your specification just needs to say *what* success looks like, not *how* the skill works internally.

### Create Your Main Body Section Specification

Create a file at `specs/paper-project/main-body-section.md` with this structure:

```markdown
# Specification: Research Paper Main Body Section

## Intent

Write the main body section of the research paper, developing the three key claims established in the introduction.

**Success means**: Reader understands each claim, sees supporting evidence, and recognizes logical progression.

**Target scope**: 800-1200 words across 3 subsections (one per key claim)

**Foundation**: Introduction (Lesson 8) establishes context; this section builds the argument.

## Constraints

- **Structure**: 3 subsections (one per key claim from introduction)
- **Evidence**: Each claim supported by minimum 2 research sources
- **Tone**: Consistent with introduction (academic, professional)
- **Clarity**: Accessible to educated reader unfamiliar with topic
- **Quality standards**: Follow constitution (Chapter 03) quality metrics

## Success Evals

- ✅ All three claims developed with clear reasoning
- ✅ Each claim supported by minimum 2 credible sources
- ✅ Logical progression between sections
- ✅ Academic tone consistent throughout
- ✅ Word count within 800-1200 word target
- ✅ Sources properly cited and integrated
- ✅ Reader understands argument progression

## Non-Goals

- No counter-argument discussion (saved for conclusion)
- No external research beyond existing sources
- No section reorganization (three claims must map to outline)
- No graphics or visual elements

## Edge Cases Handled by Skill

The `section-writer` skill from Lesson 9 handles these:
- Maintaining tone consistency with introduction
- Integrating diverse sources coherently
- Structuring long claims into digestible subsections
- Ensuring evidence completeness
- Validating academic standards
```

**Stop and verify**: Your specification is complete when:
- ✅ Intent is clear (developing the three claims from introduction)
- ✅ Constraints are explicit (structure, evidence requirements, tone, quality standards)
- ✅ Success criteria are measurable (claim development, source count, word count, tone consistency)
- ✅ Non-goals prevent scope creep (no counter-arguments, no graphics, no reorganization)

---

## Step 2: Understand Your Skill and How It Compounds

Now comes the leverage: you're going to invoke `/sp.implement` with explicit skill references. This tells the AI assistant: "Use the intelligence I've already built."

### The `section-writer` Skill From Lesson 9

In Lesson 9, you created the `section-writer` skill at `.claude/skills/section-writer/SKILL.md`:

```markdown
---
name: "section-writer"
description: "Write clear, evidence-supported research paper sections with proper citations and logical flow. Use when user asks to write a section, needs help with academic writing, or wants to structure research content."
version: "1.0.0"
---

# Section Writing Skill

## When to Use This Skill

- User asks to "write a section" or "draft content" for a research paper
- User mentions thesis, evidence integration, or academic writing
- User needs help structuring research content with citations

## How This Skill Works

1. **Outline key points**: Identify 3-5 main points the section must make
2. **Write opening paragraph**: Establish context, hook interest, connect to thesis
3. **Develop each point**: Topic sentence → evidence/citation → significance → transition
4. **Write closing**: Summarize insights, connect to thesis, set up next section
5. **Quality check**: Verify claims cited, flow smooth, thesis connected

## Quality Criteria

A section is ready when:
- Someone unfamiliar with topic can follow the logic
- All claims supported by evidence
- No sentences require re-reading to understand
- Opening hooks interest, closing connects to thesis
```

When AI discovers this skill (through the description), it applies this reasoning framework, which accelerates the entire writing process.

### Understanding Skill Invocation

Skills are automatically discovered through their descriptions. When you run `/sp.implement` and mention writing a section, AI finds your `section-writer` skill:

```
/sp.implement

I need to write the main body section for my research paper.
Use my section-writer skill to guide the process.

Specification: specs/paper-project/main-body-section.md
```

**How skill discovery works**:
1. AI scans `.claude/skills/` for available skills
2. Your `section-writer` description says "Use when user asks to write a section"
3. AI automatically loads and applies that skill's workflow

**The AI assistant will**:
1. Read your specification
2. Discover and load your `section-writer` skill (from `.claude/skills/section-writer/SKILL.md`)
3. Follow the skill's "How This Skill Works" steps
4. Apply the skill's quality criteria
5. Validate output against success evals

**What you're NOT doing**:
- ❌ Writing from scratch (no discovery phase)
- ❌ Learning academic structure again (already in skill)
- ❌ Figuring out evidence integration (skill handles this)
- ❌ Validating against quality standards (skill applies them)

Your skill has already encoded this knowledge.

---

## Step 3: Execute with /sp.implement and Skills

When you're ready (your specification complete, your introduction from Lesson 8 available, research sources gathered), run this command:

```
/sp.implement

Write the main body section of my research paper.
Use the section-writer skill from .claude/skills/.

Specification: specs/paper-project/main-body-section.md

Key claims to develop:
1. [Your first claim from introduction]
2. [Your second claim from introduction]
3. [Your third claim from introduction]
```

### What Happens During Execution

The AI assistant will:

1. **Load Your Specification**
   - Read intent, constraints, success criteria
   - Identify that this section develops three claims from introduction
   - Extract quality standards from constitution (Chapter 03)

2. **Discover and Apply Your Skill**
   - Find `section-writer` skill in `.claude/skills/section-writer/SKILL.md`
   - Follow the skill's process (outline → opening → develop → closing → quality check)
   - Apply the skill's quality criteria
   - Check academic tone consistency with introduction

3. **Execute the Writing**
   - Write main body section with 3 subsections (one per claim)
   - Integrate research sources (minimum 2 per claim)
   - Maintain word count (800-1200 words)
   - Ensure logical progression

4. **Validate Success**
   - Check that all three claims are developed
   - Verify evidence completeness
   - Confirm tone consistency
   - Validate word count and structure
   - Confirm success evals are met

### Your Role During Implementation

You don't watch passively. You're part of the execution:

1. **Review Before Finalization** — The assistant will show you the draft section and ask: "Does this meet your specification?" You verify that claims are developed clearly and evidence is compelling.

2. **Provide Feedback If Needed** — If something doesn't match your vision (tone feels off, a claim needs more support), the assistant will refine based on your input.

3. **Validate The Outcome** — Once finalized, you read the section and verify it integrates well with your introduction and advances your argument.

---

## Step 4: Validate Your Main Body Section

Once the section is written, run this validation checklist:

### Pre-Implementation Checklist (Before Invoking Skill)

- [ ] **Specification is complete** — All required sections filled (Intent, Constraints, Success Evals, Non-Goals)
- [ ] **Introduction from Lesson 8 exists** — You have the validated introduction section
- [ ] **Skill is referenced** — `/sp.implement` command includes `--skills section-writer`
- [ ] **Research sources gathered** — You have credible sources for claims
- [ ] **Claim clarity** — You can articulate the three claims your main body will develop

### Post-Implementation Validation (After Section Is Written)

- [ ] **All three claims developed** — Each claim has clear reasoning and support
- [ ] **Evidence completeness** — Each claim has minimum 2 credible sources cited
- [ ] **Tone consistency** — Matches introduction (academic, professional)
- [ ] **Word count appropriate** — Falls within 800-1200 word range
- [ ] **Logical progression** — Reader can follow argument from claim to claim
- [ ] **Sources integrated** — Citations feel natural, not forced
- [ ] **Integrates with introduction** — Section builds on introduction without repetition

### Success Criteria Verification

Map your main body section to the specification success evals:

| Success Eval | Evidence | Status |
|-------------|----------|--------|
| All three claims developed with clear reasoning | Section clearly explains each claim | ✅ / ❌ |
| Each claim supported by minimum 2 sources | Citations visible in text | ✅ / ❌ |
| Logical progression between sections | Natural flow from claim to claim | ✅ / ❌ |
| Academic tone consistent throughout | Matches introduction tone | ✅ / ❌ |
| Word count within target | 800-1200 words | ✅ / ❌ |
| Sources properly cited | References formatted correctly | ✅ / ❌ |
| Reader understands progression | Argument is clear and persuasive | ✅ / ❌ |

**Success condition**: All evals marked ✅

---

## Step 5: Reflection — Why Was This Faster?

Now comes the insight: pause and measure what just happened.

### Comparing Two Experiences

**Lesson 8 (First Section from Scratch)**

You had to:
1. Learn research paper specification structure
2. Discover what "quality academic writing" means in context
3. Write detailed implementation plan
4. Troubleshoot tone and clarity issues
5. Learn how to validate claims against sources
6. Iterate when first draft wasn't quite right
7. Test integration with paper structure

**Result**: Comprehensive understanding of academic writing, but time-consuming (3.5 hours).

**Lesson 11 (Second Section with Skill)**

You only had to:
1. Write specification (15 min)
2. Reference existing skill
3. Run `/sp.implement` with skill reference (30 min)
4. Validate output (15 min)

**Result**: Main body section written, faster, using intelligence you built (70 minutes).

### The Acceleration Hypothesis

Why was this faster? Document your observations:

```markdown
## Intelligence Acceleration - Reflection

### Time Comparison
- **Lesson 8 (first section from scratch)**: ~3.5 hours
- **Lesson 11 (second section with skill)**: ~70 minutes
- **Time savings**: ~2 hours 20 minutes (65% reduction)

### Why It Was Faster
1. **Specification was simpler** — Upstream complexity handled by skill
2. **Skill encoded patterns** — No discovery needed (academic structure, tone standards, evidence validation)
3. **Composition worked** — Reused writing intelligence instead of inventing approach
4. **Fewer decisions** — Skill Persona + Questions meant fewer "how should I structure this?" moments

### What Came from Skill vs from Scratch
- **From `section-writer` skill**: Academic writing patterns, claim development structure, evidence integration, tone consistency, quality validation
- **From scratch** (minimal): Only how this specific main body maps to your paper outline

### Next Paper (Hypothetical)
If you wrote another research paper next month:
- **Without skill**: 3+ hours per section (similar to first time)
- **With refined skill**: 45 minutes per section (skill improved, patterns reinforced)

### The Compounding Effect
- **Paper 1**: 3.5 hours per section (build intelligence)
- **Paper 2**: 1 hour per section (use intelligence)
- **Paper 3**: 45 min per section (intelligence refined)
- **Paper 5**: 30 min per section (mastered, patterns automated)

**The math of intelligence accumulation**: Each new project takes progressively less time because you're not rediscovering patterns—you're applying known solutions at increasing velocity.

### Skills as YOUR Assets
- **Code library**: Reusable technical solutions (functions, classes, frameworks)
- **Intelligence skills**: YOUR reasoning frameworks and domain knowledge, saved as P+Q+P files
- **Your advantage**: You're building a library of *thinking*, not just code

This is how YOU scale your expertise: by encoding patterns as reusable intelligence that YOU invoke across future projects.
```

**Stop here and reflect**: What surprised you about the time difference? What did the skill actually buy you? What would change if you needed to write 10 more sections using this approach? Write 2-3 sentences in your own reflection document.

---

## Step 6: The Portfolio Value of Your Skills

You now have two documented skills:

1. **`section-writer` skill** — Encodes academic writing patterns, claim development, evidence integration, tone consistency, quality validation
2. **Future skills** — Any patterns you encode from this project

### What Makes These Skills Valuable

**To yourself**:
- Next paper takes 50% less time
- Next ten papers compound benefits (each faster)
- Future writing projects start with foundation, not zero

**To a team**:
- New team members inherit your writing standards
- Specifications become templates others can reuse
- Reasoning frameworks spread consistency

**To clients or organizations**:
- Writing velocity multiplies across projects
- Quality standards are codified, not dependent on individual skill
- Knowledge doesn't leave when people do

### Measuring Skill Value

You can articulate the value of your `section-writer` skill in concrete terms:

**Time savings**:
- First paper: 3.5 hours per section
- Subsequent papers: 1.25 hours per section (65% reduction)
- 10-paper project: Save ~23 hours using this one skill

**Quality consistency**:
- Patterns are encoded (not rediscovered with each paper)
- Standards are applied consistently (not dependent on mood or fatigue)
- Knowledge accumulates (second paper better than first)

**Organizational knowledge**:
- Specification becomes template
- Skill becomes reference implementation
- Patterns become organization standard

This is why SDD-RI matters: not because it produces code faster, but because it compounds intelligence. Each pattern you encode makes your team exponentially more capable.

---

## Chapter 14 Complete Validation Checklist

You've completed Chapter 14. Verify you have:

### ✅ Projects Completed

- [ ] **First paper section (introduction)** — Completed in Lesson 8, validated against spec
- [ ] **Second paper section (main body)** — Completed in Lesson 11 using skill reuse
- [ ] **Skills created** — `section-writer` skill (Lesson 9), possibly others

### ✅ Documentation Complete

- [ ] **Constitution file** — `specs/paper-project-constitution.md` defines quality standards
- [ ] **Introduction specification** — `specs/paper-project/introduction.md` completed in Lesson 4
- [ ] **Main body specification** — `specs/paper-project/main-body-section.md` completed in this lesson
- [ ] **Specifications refined** — Both specs passed `/sp.clarify` validation (Lesson 5)
- [ ] **Implementation plans** — Detailed approach for each section
- [ ] **Task breakdowns** — Atomic work units for each phase

### ✅ Skills Documented

- [ ] **`section-writer` skill** — Persona + Questions + Principles for academic writing
- [ ] **Skill metadata** — Proficiency levels, decision frameworks, reasoning principles documented
- [ ] **Skill reuse applied** — Second section explicitly used skill from Lesson 9

### ✅ Intelligence Demonstrated

- [ ] **Acceleration measured** — Lesson 8 timeline (3.5 hours) vs Lesson 11 (70 minutes) documented
- [ ] **Skill reuse applied** — `/sp.implement` explicitly referenced `section-writer` skill
- [ ] **Composition proven** — Main body section used writing intelligence without reinvention
- [ ] **Compounding recognized** — Reflection on how future papers accelerate beyond current one

### ✅ Portfolio-Ready Project

- [ ] **Paper sections complete** — At minimum: introduction + main body (core argument)
- [ ] **Skills documented** — Could explain them to another developer
- [ ] **Specification files** — Show your thinking (specification primacy)
- [ ] **Timeline evidence** — Can demonstrate intelligence acceleration to others

**All boxes checked?** You've completed the full SDD-RI cycle. You have:
- **Theory** (Chapter 13)
- **Practice** (Chapter 14, Lessons 1-10)
- **Application** (Chapter 14, Lesson 11 - this capstone with skill reuse)
- **Reusable Intelligence** (Documented skills from Lessons 3, 6, 9)
- **Measurable Acceleration** (Evidence that Project 2 is faster than Project 1)

You're now positioned to approach any new writing project with composition-first thinking instead of build-from-scratch thinking.

---

## Try With AI

**Setup**: Open your AI companion (Claude Code, Gemini, or ChatGPT). You're going to reflect on your skill library and plan future projects.

**Prompt Set**:

```
Prompt 1 (Portfolio Recognition):
"I completed a research paper using SDD-RI workflow and created a reusable `section-writer` skill.
In plain language: What kinds of future writing projects could I tackle faster using this skill?
What if I combined this skill with a `research-validator` skill (to fact-check claims)?
What would my skill library look like after 5 more papers?"

Prompt 2 (Acceleration Measurement):
"I spent 3.5 hours writing my first paper section (Lesson 8).
I spent 70 minutes writing my second section using the section-writer skill (Lesson 11).
That's a 65% time savings.

What should I be measuring to prove that intelligence accumulation is real?
If I write 5 more papers using this skill: What would my velocity look like?
How would I show this improvement to a potential employer or client?"

Prompt 3 (Skill Compounding):
"If I followed the SDD-RI approach for 10 writing projects over the next year:
- What would my skill library look like?
- How would my writing velocity compound?
- At what point would I be writing papers faster than competitors?
- What's the long-term ROI of building reusable intelligence vs writing one-off papers?"
```

**Expected Outcomes**:
- **Prompt 1**: AI should help you envision future projects where your skills provide leverage (client papers, research projects, content creation, technical writing)
- **Prompt 2**: AI should highlight what makes intelligence valuable (pattern reuse, decision framework sharing, quality consistency, reproducibility)
- **Prompt 3**: AI should demonstrate exponential returns from intelligence accumulation (skills compound across projects, velocity multiplies with each iteration)

**Safety Note**: When you discuss your writing project or future client work, remember that you've built this using specifications and skills as the foundation. Credit the methodology. You didn't use AI to write the entire paper—you used AI to *accelerate* structured thinking with documented specifications and reusable intelligence.

**Optional Stretch**: Now that you have a reusable skill, find a real writing project (research paper for a client, technical documentation, proposal, blog series). Write a specification for that project. Estimate how long it would take you to execute using your `section-writer` skill. Then, using your skill, actually write that project. This is how writers monetize their intelligence accumulation: by solving client problems faster than competitors.

---

**Chapter 14 is now complete.** You've demonstrated full SDD-RI mastery—from specification to implementation to reusable intelligence to accelerated execution.

You're ready for more complex projects. Your skills compound. Your intelligence library grows. Your development velocity increases exponentially.

This is the power of SDD-RI.
