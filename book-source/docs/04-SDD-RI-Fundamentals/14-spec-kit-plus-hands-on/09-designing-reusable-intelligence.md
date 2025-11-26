---
title: "Designing Reusable Intelligence"
chapter: 14
lesson: 9
duration_minutes: 45
proficiency_level: "B1"
cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (Pattern reusability criteria, P+Q+P framework, Skill file structure, Skill vs subagent distinction) at B1 level with paper writing scaffolding"
learning_objectives:
  - objective: "Identify recurring patterns that justify reusable intelligence encoding (frequency, complexity, organizational value)"
    bloom_level: "Analyze"
  - objective: "Design a skill using Persona + Questions + Principles (P+Q+P) pattern"
    bloom_level: "Create"
  - objective: "Determine when to create skill vs subagent based on decision point count"
    bloom_level: "Evaluate"
  - objective: "Apply P+Q+P framework to transform domain workflows into reasoning-activated intelligence"
    bloom_level: "Apply"
generated_by: "content-implementer v1.0.0"
source_spec: "Chapter 14 Lesson 9 Rewrite: P+Q+P Framework for Skill Design"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Designing Reusable Intelligence

You've completed the SDD workflow (Lessons 01-08): Constitution → Specify → Clarify → Plan → Tasks → Implement. You wrote specifications, refined requirements, planned architecture, decomposed work into tasks, and executed implementation with AI collaboration.

But here's what separates AI-native developers from AI-assisted developers: **The ability to transform workflow patterns into reusable intelligence.**

In this lesson, you'll learn the **Persona + Questions + Principles (P+Q+P)** framework—the pattern that converts tacit knowledge into explicit, reasoning-activated components. By the end, you'll be able to create reusable skills that compound your capability with every project.

---

## From Workflow Execution to Intelligence Accumulation

### What You've Built So Far

**Lessons 04-08 taught you a complete project workflow**:
- How to write clear specifications (Lesson 04)
- How to refine specifications with edge cases (Lesson 05)
- How to plan architecture and design decisions (Lesson 06)
- How to decompose work into tasks (Lesson 07)
- How to orchestrate AI implementation (Lesson 08)

**What's missing**: Reusable components that make your next research paper project 10x faster.

### The Paradigm Shift: Reusable Intelligence > Reusable Code

The Specification-Driven Development with Reusable Intelligence (SDD-RI) approach changes what we consider reusable:

> **Traditional Development**: Code libraries are the units of reuse. Developers share functions, classes, frameworks.
>
> **AI-Native Development**: Specifications, Skills, and Agent Architectures are the units of reuse. Developers share intelligence.

**In practice**:
- **Project 1 (Lessons 04-08)**: You execute the complete workflow to write your first research paper (8-10 hours)
- **Project 2 (without intelligence)**: You write new specification, run workflow again (7-9 hours—slightly faster through practice)
- **Project 2 (with `section-writer` skill)**: You invoke skill, reuse writing patterns, orchestrate with trained frameworks (2-3 hours—3-4x faster)

**The difference**: Accumulated intelligence compounds. Every pattern you encode accelerates future work.

---

## Identifying Patterns Worth Encoding

Not every workflow step justifies creating reusable intelligence. Use this decision framework:

### Decision Framework: When to Encode Intelligence

Ask three questions about the workflow pattern:

**1. Frequency**: Will this pattern recur across 3+ projects?
- ✅ YES: Writing quality sections (every paper needs clear, well-supported sections)
- ✅ YES: Research validation (every claim needs sources)
- ✅ YES: Outline refinement (every paper needs structural iteration)
- ❌ NO: One-time outline for this specific research topic

**2. Complexity**: Does this pattern involve 5+ decision points?
- ✅ YES: Section writing (quality standards, source integration, clarity, engagement, validation = 5+ decisions)
- ✅ YES: Research validation (finding sources, evaluating credibility, citations, cross-checking facts)
- ✅ YES: Outline refinement (logical flow, scope control, balance, audience readiness)
- ❌ NO: Single typing action (1 decision: execute or not)

**3. Organizational Value**: Will encoding this pattern improve your capability?
- ✅ YES: `section-writer` skill (faster writing, consistent quality)
- ✅ YES: `research-validator` skill (reliable fact-checking, prevents errors)
- ✅ YES: `outline-refiner` skill (better structure, clearer thinking)
- ❌ NO: Personal formatting preferences (individual style, limited team value)

**Rule**: If 2+ answers are YES → Encode as reusable intelligence

### Pattern Analysis from Chapter 14 Project

Let's analyze what you've learned building your research paper:

| **Pattern** | **Frequency** | **Complexity** | **Org Value** | **Encode?** |
|-------------|---------------|----------------|---------------|-------------|
| Writing quality sections | ✅ Every paper | ✅ 6+ decisions | ✅ Speed future papers | **YES** |
| Research validation | ✅ Every source | ✅ 5+ decisions | ✅ Accuracy | **YES** |
| Outline refinement | ✅ Every draft | ✅ 5+ decisions | ✅ Structure | **YES** |
| Formatting citations | ✅ Every source | ❌ 2-3 decisions | ❌ Tool-dependent | NO |
| Spell checking | ✅ Every draft | ❌ 1 decision | ❌ Automated already | NO |

**Candidates for intelligence encoding**:
1. **Section-Writer Skill**: Guide for writing clear, well-supported sections (core writing pattern)
2. **Research-Validator Skill**: Framework for validating sources and facts
3. **Outline-Refiner Skill**: Process for improving structure and logical flow

**In this lesson, you'll build**: `section-writer` skill + understand the broader skill composition pattern

---

## The P+Q+P Pattern: Activating Reasoning Mode

Effective reusable intelligence uses the **Persona + Questions + Principles (P+Q+P)** pattern.

This pattern **activates reasoning mode** (context-specific thinking) instead of **prediction mode** (pattern retrieval from training data).

### Understanding P+Q+P

**Persona**: Establishes cognitive stance (how to think about the problem)
- **Not**: "You are a writing expert" (vague, triggers generic responses)
- **But**: "Think like a research writer who ensures clear communication the way a journalist ensures readers understand stories—with evidence, clarity, and audience awareness"
- **Why it works**: Specific stance with analogy activates domain-specific reasoning, not generic expertise mode

**Questions**: Forces context-specific analysis (what to analyze)
- **Not**: "Is this section good?" (yes/no, no reasoning)
- **But**: "Does the section clearly explain the concept? Is every claim supported by evidence? Could someone unfamiliar with the topic understand the argument? What iteration would improve clarity?"
- **Why it works**: Open-ended questions force analysis across multiple dimensions, activating careful thinking

**Principles**: Provides decision frameworks (how to make judgments)
- **Not**: "Use best practices" (meaningless without definition)
- **But**: "Clarity Principle: Use simple words (8th-grade reading level), explain technical terms, connect ideas with transitions. Evidence Principle: Every claim needs a source. New information requires citation, familiar knowledge may not."
- **Why it works**: Concrete rules enable consistent judgment, remove ambiguity

### How P+Q+P Differs from Checklists

**Checklist** (prediction mode):
```
- Does section have introduction? ✅
- Does section have examples? ✅
- Does section have conclusion? ✅
→ Checklist complete, section ready
```

**P+Q+P** (reasoning mode):
```
Persona: "Think like a journalist ensuring readers understand the story"

Questions:
- Does opening hook the reader's interest?
- Does explanation flow logically?
- Does evidence convince or just inform?
- What confusion would a reader still have?
- What iteration would deepen understanding?

Principles:
- Clarity: One idea per paragraph
- Evidence: Support claims with sources
- Engagement: Match audience knowledge level
```

**Why it's better**: Checklist asks "Is it done?" P+Q+P asks "Is it excellent?" One checks boxes. The other activates thinking.

---

## Building Your First Skill: Section-Writer

### Step 1: Define the Persona

Collaborate with your AI to develop the cognitive stance:

**Your thinking** (to share with AI):
```
I want to create a skill for writing research paper sections. The skill should
guide both the writing process and quality validation.

Think about how a good research writer approaches sections differently than
someone just trying to fill page count. They think about:
- Clear communication of complex ideas
- Integrating sources without disrupting flow
- Building logical progression
- Meeting audience expectations

What cognitive stance activates the right thinking for research writing?
```

**What a Strong Persona Looks Like**:

```markdown
## Persona

You are a research writing specialist who thinks about section writing
the way a journalist thinks about storytelling:

- Systematically structuring ideas before writing
- Ensuring clarity matches audience knowledge level
- Supporting claims with credible evidence
- Building logical progression that leads to understanding
- Iterating based on clarity feedback

Your goal: Write research sections that communicate complex ideas clearly,
maintain reader engagement, and validate quality before final submission.
Every iteration deepens understanding.
```

**Key characteristics**:
- Specific analogy (journalist, not "expert")
- Concrete practices (structuring, ensuring clarity, supporting claims)
- Clear purpose (communicate ideas, maintain engagement)
- Emphasis on iteration and quality

### Step 2: Formulate Analytical Questions

Questions force analysis across multiple dimensions:

**Your thinking** (to share with AI):
```
Now let's design questions that activate deep analysis of research writing.

These should force context-specific thinking, not just yes/no answers.

I'm thinking about:
1. Clarity (does reader understand?)
2. Evidence (is it supported?)
3. Logical flow (do ideas connect?)
4. Audience awareness (is it appropriate?)
5. Completeness (does it answer the research question?)

Help me expand this to 5-7 questions that activate thorough section analysis.
```

**What Strong Analytical Questions Look Like**:

```markdown
## Analytical Questions

Before finalizing a section, analyze:

1. **Clarity & Understanding**:
   - Does the opening establish what this section explains?
   - Could someone unfamiliar with this topic understand the argument?
   - Are technical terms defined when first introduced?
   - Do transitions connect ideas logically?

2. **Evidence & Support**:
   - Is every major claim supported by evidence or citation?
   - Are sources credible and relevant?
   - Does evidence feel integrated or tacked on?
   - Are contradictory sources acknowledged?

3. **Logical Progression**:
   - Does the section build understanding step-by-step?
   - Does each paragraph connect to the previous one?
   - Does the conclusion follow logically from evidence?
   - Is anything explained out of order?

4. **Audience Awareness**:
   - Is the language appropriate for the intended reader?
   - Are assumptions about background knowledge correct?
   - Would the audience find this section engaging or tedious?
   - Is the depth appropriate (not oversimplified, not over-technical)?

5. **Completeness & Relevance**:
   - Does the section answer questions the research raises?
   - Is all content relevant to the thesis?
   - Is anything important left out?
   - Would removing anything hurt understanding?
```

**Key characteristics**:
- Multiple dimensions (clarity, evidence, flow, audience, completeness)
- Open-ended (not yes/no)
- Action-oriented (could, does, would)
- Considers both writer and reader perspectives

### Step 3: Articulate Decision Principles

Principles provide concrete decision rules:

**Your thinking** (to share with AI):
```
Finally, let's define concrete principles for section writing and quality validation.

These should be decision rules that guide actions, not abstract advice.

I'm thinking:
1. Clarity principles (word choice, sentence length, examples)
2. Evidence integration principles (citations, quote use, fact claims)
3. Structure principles (paragraph focus, transitions, flow)
4. Iteration principles (what to improve first, when section is ready)

Help me create 4-5 principle statements that provide concrete guidance.
```

**What Strong Decision Principles Look Like**:

```markdown
## Decision Principles

Apply these frameworks when writing and validating research sections:

### Principle 1: Clarity & Readability

- **Vocabulary**: Use simple words (8th-grade reading level where possible)
- **Sentence length**: Average 15-20 words (shorter for complex ideas, longer for transitions)
- **Technical terms**: Define before use on first occurrence
- **Examples**: Use concrete examples to explain abstract concepts
- **Transitions**: Connect sentences with "because," "therefore," "in contrast," etc.

### Principle 2: Evidence Integration

- **Citation requirement**: Every factual claim needs a source or must be common knowledge
- **Quote use**: Quotes support claims but don't replace explanation. Explain the significance.
- **Source quality**: Prefer peer-reviewed, recent sources. Acknowledge limitations of sources used.
- **Contradictions**: If sources disagree, acknowledge and explain, don't hide it

### Principle 3: Logical Structure

- **Paragraph focus**: One main idea per paragraph. Topic sentence states it clearly.
- **Paragraph length**: Typically 4-8 sentences. Each sentence supports the main idea.
- **Flow between paragraphs**: Last sentence hints at next topic. Reader should see connections.
- **Section structure**: Introduction (what and why), body (evidence and explanation), conclusion (implications)

### Principle 4: Iteration & Quality Gates

- **Quality Gate 1**: Every paragraph has a clear topic sentence
- **Quality Gate 2**: Every factual claim is supported by evidence or cited source
- **Quality Gate 3**: Section reads smoothly without stopping for confusion
- **Quality Gate 4**: Section connects to thesis (reader sees relevance)
- **Quality Gate 5**: Section stands alone (could be read without full paper context)

- **Iteration priority**: Fix clarity first (Gates 1-3), then evidence (Gate 2), then thesis connection (Gate 4)
- **Section ready**: All 5 gates pass
- **Section needs iteration**: 2+ gates fail—refine and revalidate
```

**Key characteristics**:
- Concrete (specific word limits, sentence structures, gate definitions)
- Measurable (can check if principle is met)
- Prioritized (clarity first, evidence second, etc.)
- Provides clear pass/fail criteria

---

## Skill File Structure & Location

Now that you've designed Persona, Questions, and Principles, here's where and how to store this as reusable intelligence:

### Skill File Location

Skills live in your AI companion's `.claude/skills/` directory:

```
.claude/
├── skills/
│   ├── section-writer.md
│   ├── research-validator.md
│   ├── outline-refiner.md
│   └── [other-skills].md
```

### Skill File Format

Your skill file follows this structure:

```markdown
# Skill: Section Writing for Research Papers

**Name**: section-writer
**Category**: Research & Academic Writing
**Complexity**: High (6+ decision points)
**First Use**: Chapter 14 Lesson 09
**Reusable Across**: Any research paper, thesis, technical documentation

## Description

This skill provides a complete framework for writing clear, well-supported research
sections using the P+Q+P pattern. It combines persona-based thinking, analytical
questions, and concrete principles to ensure sections communicate effectively.

When sections need consistent quality, clear communication, and reliable evidence
integration across multiple papers.

## When to Use This Skill

- Apply when: Writing sections for research papers
- Apply when: Need to ensure clarity and evidence support
- Apply when: Want consistent quality across multiple papers
- Apply when: Refining writing through iteration
- Skip when: Writing is one-off (low reuse value)
- Skip when: Using different writing context (creative fiction)

## Persona

[Insert your persona definition from Step 1]

## Analytical Questions

[Insert your questions from Step 2]

## Decision Principles

[Insert your principles from Step 3]

## Usage Example

**Scenario**: You're writing a section on "Machine Learning in Healthcare" for your paper.

**Step 1: Write the section** (using Lesson 04 planning pattern)
```
Intent: Explain how ML is used in healthcare diagnosis.
Success criteria: Reader understands 3 ML applications, their benefits, limitations.
Evidence: 4-5 credible sources.
Audience: Technical but not ML-specialists.
```

**Step 2: Apply the Persona** (activate this skill)
```
I'm thinking like a research writer ensuring clear communication.
My opening hooks interest. Evidence flows naturally. Terminology is clear.
```

**Step 3: Analyze with Questions** (force deep analysis)
- Does the opening establish what this section explains?
- Could someone unfamiliar with ML understand this?
- Is every application supported by evidence?
- Does the section answer the thesis question?

**Step 4: Validate Against Principles** (measure quality)
- Quality Gate 1: Every paragraph has clear topic sentence? ✅
- Quality Gate 2: Every claim supported by source? ✅
- Quality Gate 3: Section reads smoothly? ✅
- Quality Gate 4: Connected to thesis? ✅
- Quality Gate 5: Stands alone? ✅

All gates pass → Section ready. No gates fail → Complete.

## Self-Check Validation

Before considering your skill complete, verify:

- ✅ Persona establishes clear cognitive stance specific to research writing
- ✅ Analytical questions force deep analysis (not yes/no)
- ✅ Quality gates are objective and measurable (not subjective)
- ✅ Decision principles provide concrete rules (not vague guidance)
- ✅ Usage example demonstrates end-to-end workflow
- ✅ Skill is reusable across papers (not topic-specific)
- ✅ Someone on your team could use this skill immediately
```

### Skill vs Subagent: When to Create Which

As you identify more patterns, you'll wonder: **Should I create a skill or a subagent?**

**Decision Framework**:

**Create a SKILL (2-4 decision points)** when:
- Pattern has few decision points (outline review, citation formatting)
- Pattern applies to similar contexts (all research papers)
- Pattern uses simple reasoning frameworks
- **Example**: Section-writer (6 decisions—on the border but still skill-based)

**Create a SUBAGENT (5+ decision points, autonomous reasoning)** when:
- Pattern has many complex decision points (research validation with 7+ decisions)
- Pattern requires autonomous judgment (when to reject sources, how to resolve contradictions)
- Pattern should run independently with minimal human intervention
- **Example**: Research-validator might be subagent (evaluates source credibility, resolves contradictions autonomously)

**In practice for Chapter 14**:
- `section-writer`: Skill (6 decisions, writer controls iteration)
- `research-validator`: Likely subagent (8+ decisions about credibility, autonomously validates claims)
- `outline-refiner`: Likely skill (5 decisions, collaborative refinement)

---

## Common Mistakes When Creating Skills

### Mistake 1: Creating Skills for Trivial Patterns

**The Error**: Creating a skill for "How to open a text editor"

**Why It's Wrong**: 1 decision point (open or not) doesn't justify intelligence encoding. Skills need 5+ decision workflows.

**The Fix**: Only encode patterns with frequency + complexity + organizational value. Research validation (8+ decisions) justifies skill. Opening editor (1 decision) does not.

### Mistake 2: Vague Personas

**The Error**: "You are a research writing expert"

**Why It's Wrong**: "Expert" is generic, triggers prediction mode (generic advice).

**The Fix**: Specific cognitive stance with analogy:
- ❌ "You are a research expert"
- ✅ "Think like a journalist ensuring readers understand stories—with evidence, clarity, and audience awareness"

### Mistake 3: Yes/No Questions

**The Error**: "Is this section good?"

**Why It's Wrong**: Binary questions don't activate analysis. AI responds "yes" or "no" without reasoning.

**The Fix**: Open-ended analytical questions:
- ❌ "Is this section good?"
- ✅ "Does the narrative clearly explain the concept? Is every claim supported? Could an unfamiliar reader understand? What iteration would improve clarity?"

### Mistake 4: Over-Specific Skills

**The Error**: Creating "Machine-Learning-Healthcare-Section-Writer" that only works for ML papers

**Why It's Wrong**: Intelligence should be reusable across contexts. Over-specificity reduces organizational value.

**The Fix**: Generalize patterns:
- ❌ "ML-Healthcare-Section-Writing"
- ✅ "Section-Writer" (works for any research paper topic)

### Mistake 5: Skills Without Quality Gates

**The Error**: "Write section and you're done"

**Why It's Wrong**: No way to verify quality. Low-quality sections damage papers.

**The Fix**: Define objective quality gates (not subjective):
- ❌ "Section should be clear"
- ✅ "Gate 1: Topic sentence present, Gate 2: Evidence provided, Gate 3: Smooth reading, Gate 4: Thesis-connected"

---

## Skill Reuse in Practice

### Project 1: Research Paper (Lessons 04-08)

You execute the complete workflow:
- Write specification for paper structure
- Plan research sections
- Tasks for execution
- Implement with AI collaboration
- **Total**: 8-10 hours

### Project 2: New Research Paper (With `section-writer` Skill)

With your skill, dramatically faster:
1. Write paper specification (30 min using Lesson 04 pattern)
2. Invoke `section-writer` skill (select persona + questions from template)
3. Write sections with skill guidance (3 hours for 5 sections = 36 min per section)
4. **Total**: 4 hours (vs 8-10 hours without skill—50% faster)

**Skill provides**:
- Persona (cognitive stance for research writing)
- Analytical framework (5+ questions to analyze deeply)
- Quality gates (objective validation criteria)
- Decision principles (concrete rules for clarity, evidence, flow)

### Project 3: Multi-Section Major Writing (With Multiple Skills)

With accumulated skills, orchestrate sequences:
1. Use `section-writer` skill to write sections (2 hours)
2. Use `research-validator` skill to check facts (1 hour)
3. Use `outline-refiner` skill to improve structure (30 min)
4. **Total**: 3.5 hours

**Intelligence compounds**: Skill 1 (section writing) + Skill 2 (validation) + Skill 3 (structure) = Professional paper in less time than single section without skills.

---

## Try With AI

Ready to create reusable intelligence for your research writing workflow? Design your skill with your AI partner:

**Explore the P+Q+P Pattern:**
> "Explain the Persona + Questions + Principles pattern for skill design. Show me: (1) What Persona activates the right thinking for research writing (hint: think like a journalist, not a writing expert)? (2) What 5-7 Analytical Questions force deep analysis of writing quality? (3) What 5 Decision Principles provide concrete rules for clarity, evidence, and flow? Compare this to just having a checklist—why does P+Q+P activate reasoning mode better?"

**Design the Persona for Section Writing:**
> "Help me create a Persona for section writing skill. I want it to establish cognitive stance specific to research writing, not just generic 'expert'. Guide me: (1) What analogy best captures research writing thinking? (2) What cognitive stance should it adopt? (3) How does this persona activate different thinking than 'writing expert'? (4) Write the complete Persona section for my section-writer skill."

**Create Analytical Questions:**
> "Generate 5-7 Analytical Questions for my section-writer skill. These should force context-specific analysis of research sections. Questions should cover: clarity (does reader understand?), evidence (is it supported?), flow (do ideas connect?), audience awareness (is it appropriate?), and completeness (does it answer the research question?). For each question, explain why it's open-ended vs yes/no, and what reasoning it activates."

**Build Your Complete Skill File:**
> "Help me create a complete section-writer skill file following standard skill structure. Include: (1) Metadata (name, category, complexity, first use, reusable across); (2) Persona (cognitive stance for research writing); (3) Analytical Questions (5-7 open-ended); (4) Decision Principles (5 concrete frameworks for clarity, evidence, flow, iteration, gates); (5) Complete usage example (specification → persona → questions → validation); (6) Self-check validation (7 checkpoints). Format as production-ready file someone on my team could use immediately."

**Apply to Your Own Workflow:**
> "I want to create a reusable skill for [your domain/workflow]. Help me identify: (1) Does this pattern recur across 3+ contexts? (2) Does it involve 5+ decision points? (3) Would encoding it improve team capability? If yes to 2+, help me design the P+Q+P skill: What Persona activates the right thinking? What 5-7 Questions force deep analysis? What 5 Decision Principles provide concrete rules?"
