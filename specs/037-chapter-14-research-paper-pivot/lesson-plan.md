# Chapter 14: Spec-Kit Plus Hands-On (Research Paper Project) — Detailed Lesson Plan

**Generated**: 2025-11-26
**Feature Branch**: `037-chapter-14-research-paper-pivot`
**Constitutional Version**: 6.0.1
**Pedagogical Framework**: 4-Layer Teaching Method (L1→L2→L3→L4)
**Target Proficiency**: A2-B1 (Beginner transitioning to Intermediate)
**Total Chapter Duration**: 8-12 hours (8 lessons × 60-90 minutes)

---

## Executive Summary

Chapter 14 is a **project-based capstone for Part 4** that transforms SDD-RI theory (Chapter 13) into hands-on practice. Students will execute the complete **Spec-Kit Plus workflow** (`/sp.constitution` → `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`) by building a research paper artifact using only markdown and AI collaboration. The chapter culminates in creating **reusable intelligence (skills)** using the **Persona + Questions + Principles** framework, demonstrating how intelligence accumulates across projects.

**Project**: Students research and write a 3,000-5,000 word markdown research paper on an AI-related topic of their choice, documenting their workflow and extracting reusable patterns.

**Learning Arc**:
1. **Foundation (L1-2)**: Introduction to the project structure + Constitution creation
2. **Application (L3-5)**: Execute SDD-RI workflow (Specify → Plan → Tasks → Implement)
3. **Integration (L6-7)**: Identify patterns + Create reusable skill
4. **Mastery (L8)**: Capstone - Apply skill to second task, demonstrate acceleration

---

## Pedagogical Arc Overview

| Phase | Layer | Lessons | Focus | Output |
|-------|-------|---------|-------|--------|
| **Foundation** | L1-L2 | 1-2 | Manual + AI intro | Constitution + Project Plan |
| **Application** | L2-L3 | 3-5 | Execute workflow | Spec, Plan, Tasks, Paper Sections |
| **Integration** | L3 | 6-7 | Identify + Create | Reusable Skill (P+Q+P) |
| **Mastery** | L4 | 8 | Spec-driven capstone | Skill Applied + Reflection |

**Teaching Modality Variation** (from Chapter 13):
- **Chapter 13**: Problem-discovery through error analysis (learning WHY SDD-RI matters)
- **Chapter 14**: Hands-on project execution (learning HOW to apply SDD-RI)

**Three Roles Integration** (Invisible Framework):
Throughout the chapter, students experience AI in three collaborative roles—but these roles are never labeled explicitly. Instead, students discover them through narrative and action:
- **AI as Teacher**: "AI suggested a section structure you hadn't considered"
- **AI as Student**: "You taught AI your specific constraints (audience, length)"
- **AI as Co-Worker**: "Together you iterated toward a final version"

---

## Cognitive Load Analysis

| Lesson | New Concepts | Count | Tier | Validation |
|--------|--------------|-------|------|------------|
| L1: Introduction | Research paper, SDD-RI workflow, artifact | 3 | A2 | ✅ 3 ≤ 7 |
| L2: Constitution | Standards, constraints, quality gates, tone | 4 | A2 | ✅ 4 ≤ 7 |
| L3: Specification | Intent, success criteria, constraints, non-goals | 4 | A2-B1 | ✅ 4 ≤ 7 |
| L4: Planning | Section breakdown, AI strategy, timing | 4 | B1 | ✅ 4 ≤ 7 |
| L5: Tasks & Implement | Atomic units, checkpoints, iteration | 3 | B1 | ✅ 3 ≤ 7 |
| L6: Pattern Recognition | Recurring workflow, pattern identification, reusability | 3 | B1 | ✅ 3 ≤ 7 |
| L7: Skill Creation | Persona, Questions, Principles, skill structure | 4 | B1 | ✅ 4 ≤ 7 |
| L8: Capstone | Spec-first thinking, skill reuse, acceleration | 3 | B1-C1 | ✅ 3 ≤ 10 |

**All lessons respect CEFR cognitive load limits.** A2 lessons ≤7 concepts, B1 lessons ≤10 concepts.

---

## Intelligence Accumulation Tracking

**What reusable intelligence emerges at each stage?**

| Stage | Pattern Recognition | Potential Skills | Captured When |
|-------|-------------------|------------------|---------------|
| L1-2 | Constitution as quality gates | (Foundation, no skills yet) | — |
| L3-4 | Specification + Planning cycle | "research-section-specification" | L6 reflection |
| L5 | Iterative refinement loop | "ai-writing-iteration" | L6 reflection |
| L6-7 | Pattern crystallization | 1 Student-Created Skill (P+Q+P) | L7 exercise |
| L8 | Skill reuse acceleration | Applied skill demonstrates value | L8 capstone |

**By end of chapter**: Students have created 1 reusable skill (2-4 decisions) demonstrating they understand how intelligence compounds.

---

## Lesson Plans

---

## Lesson 1: Welcome to Your Research Paper Project

**Duration**: 45-60 minutes
**Stage**: L1 (Manual Foundation)
**Bloom's Level**: Understand + Apply
**Concepts Introduced** (count: 3):
1. Research paper as a hands-on artifact for learning SDD-RI
2. Spec-Kit Plus workflow overview (five phases)
3. Project constraints (scope, timeline, deliverables)

### Learning Objectives

1. Understand what you'll build over this chapter (research paper artifact)
2. Recognize the five phases of Spec-Kit Plus workflow
3. Choose an AI-related research topic that excites you
4. Commit to a manageable scope (3-5 sections, 3,000-5,000 words)

### Lesson Flow

**Opening Hook** (5 min)
- **Narrative**: "In Chapter 13, you learned the theory of SDD-RI—that specifications define your work, and reusable intelligence multiplies your power. Now it's time to prove it. Over the next week or two, you're going to write a research paper using ONLY these tools: your AI companion, markdown, and your ability to think clearly about what you're building. By the end, you'll have both a publishable-quality document AND a reusable skill you can apply to future writing projects."
- **Question**: "What's one question about AI that you've wanted to explore deeply?"

**Context: The Research Paper as Artifact** (10 min)
- Research papers are the perfect vehicle for learning SDD-RI because:
  - They're entirely text-based (no dependencies on Python, MCP, external APIs)
  - They require careful specification (what topics to cover, what NOT to cover)
  - They benefit from AI collaboration without becoming AI-generated (humans decide structure, AI helps develop ideas)
  - They produce something you can keep, share, or publish

- **Example**: A student choosing "How AI-Driven Development Changes Software Engineering" will:
  - **Week 1**: Create constitution, spec, and plan (decide WHAT to write)
  - **Week 2**: Write sections using AI as thinking partner, not ghostwriter
  - **Week 3**: Reflect on your process, extract a reusable skill
  - **End product**: 4,000-word paper you authored (with AI as collaborator) + a reusable skill you can apply to any future research task

**The Spec-Kit Plus Workflow** (15 min)

Students will execute these five phases (not as magic commands, but as structured thinking):

1. **Constitution Phase** (Lesson 2): Define project-wide standards
   - "What quality standards will guide all my work?"
   - "What tone, length, and audience do I target?"

2. **Specification Phase** (Lesson 3): Define WHAT you're building
   - "What topics MUST my paper cover?"
   - "What will success look like?"
   - "What am I deliberately NOT covering?"

3. **Planning Phase** (Lesson 4): Design HOW you'll build it
   - "What are my sections?"
   - "In what order should I write them?"
   - "How will I use AI effectively for each section?"

4. **Tasks Phase** (Lesson 5): Break work into atomic units
   - "What's the smallest piece I can complete in one session?"
   - "What validation checkpoint proves I'm done?"

5. **Implementation Phase** (Lesson 5+): Execute with AI collaboration
   - "Now I write sections using my plan as a guide"
   - "AI is my thinking partner, not my ghostwriter"

**Topic Selection & Scope** (20 min)

Students choose a topic. **Recommended**: "How AI-Driven Development Changes Software Engineering" (directly related to what they're learning).

**Alternative topics** (all AI-related, all achievable in scope):
- The Role of Specifications in AI-Native Development
- Comparing Traditional TDD with AI-Driven Development
- How Prompting Differs from Programming
- The Future of Software Jobs in the Agentic Era

**Scope guidance**:
- **Too broad**: "The History of Artificial Intelligence" (centuries of content)
- **Too narrow**: "Claude's Context Window Advantages" (3,000 words max)
- **Just right**: "How AI-Driven Development Changes Software Engineering" (3-5 focused sections)

Each student commits to:
- **Final length**: 3,000-5,000 words (about 6-10 pages)
- **Structure**: 3-5 main sections + introduction + conclusion
- **Timeline**: Completable in 2-3 weeks at 2-4 hours/week

### Artifacts Produced

- **Decision**: Chosen research topic
- **Artifact**: `research-paper.md` (created, empty for now)
- **Artifact**: Project notebook: "Chapter 14 Project Timeline"

### Prerequisites from Earlier Lessons

- Chapter 10 (Markdown): How to structure markdown documents
- Chapter 13 (SDD-RI Theory): What specifications are, why they matter
- Chapters 5-6 (AI Tools): Comfortable with your AI companion

### Try With AI

**Activity: Topic Exploration with Your AI Companion** (20 min)

Don't start writing yet. Start *thinking* with your AI.

**Prompt to your AI**:
```
I'm researching a paper on [your chosen topic].
Help me explore this topic by asking me clarifying questions.
I want to understand:
1. What specific angle interests me most
2. What key questions the paper should answer
3. What I already know vs what I need to learn

Ask me 5-7 questions to help me refine my thinking.
```

**What happens** (experience, not framework language):
- Your AI suggests angles you hadn't considered (AI as teacher)
- You describe what matters to you, forcing clarity (you as teacher)
- Together, you sharpen your topic focus (convergence)

**Reflection**:
- "What angle did AI suggest that you hadn't thought of?"
- "How did explaining your thinking to AI change your thinking?"
- "Does your topic feel more focused now than when you started?"

Save this conversation as `01-topic-exploration.md` for reference.

---

## Lesson 2: Create Your Constitution

**Duration**: 60-75 minutes
**Stage**: L2 (AI Collaboration with invisible Three Roles)
**Bloom's Level**: Create + Analyze
**Concepts Introduced** (count: 4):
1. Constitution as project-wide quality standards
2. Quality gates (rules that guide all decisions)
3. Target audience and tone
4. Success criteria visible in finished work

### Learning Objectives

1. Understand what a constitution is (decision framework for your entire project)
2. Write your first constitution defining standards for YOUR research paper
3. Identify 3-5 quality gates that will guide your work
4. Establish target audience and tone upfront

### Lesson Flow

**Why Constitution Matters** (10 min)

Chapter 13 introduced constitutions as governance frameworks. Now you're creating one for your specific project.

**Without a constitution**, you might:
- Start writing, then realize halfway through "What tone should this have?"
- Write some sections academically, others casually (inconsistent)
- Over-write one section, under-write another (no quality standard)
- Get lost in details and forget your core goal

**With a constitution**, you have decision rules that make every choice faster:
- "Will I cite sources? How many per section?" (decided once)
- "What's my target audience: undergrads, professionals, general readers?" (decided once)
- "How technical can I get? Should I explain jargon?" (decided once)

**Structure of a Constitution** (15 min)

A research paper constitution has these sections:

1. **Intent**: Why are you writing this? (audience + purpose)
   - Example: "To explain how AI-driven development changes the software industry for developers transitioning from traditional workflows"

2. **Quality Gates**: Rules that guide all sections
   - Examples:
     - "All claims about AI capabilities are cited or grounded in my experience"
     - "Technical terms are explained in plain language for a general developer audience"
     - "Sections are 600-800 words (except intro/conclusion: 300-500)"
     - "Tone: Informative but conversational (not textbook, not blog)"

3. **Non-Goals**: What you're deliberately NOT doing
   - "This is not a technical tutorial" (I'm not teaching coding)
   - "This is not a marketing pitch" (I'm presenting balanced perspective)
   - "This is not a deep academic paper" (I'm keeping it accessible)

4. **Success Criteria**: How you'll know you're done
   - "All sections follow the same structure"
   - "No jargon without explanation"
   - "Logical flow from intro → argument → conclusion"

**Example Constitution** (for reference, not copying):

```markdown
# Research Paper Constitution: How AI-Driven Development Changes Software Engineering

## Intent
Audience: Software developers with 2-5 years experience, transitioning from traditional
coding to AI-native development. Purpose: Help them understand the paradigm shift and
see themselves as agents of change, not casualties of AI.

## Quality Gates
1. Technical accuracy: All claims about AI are either cited or grounded in demonstrated capability
2. Clarity for developers: Jargon explained, code examples when helpful but not required
3. Section consistency: Introduction (200-300 words), Body sections (600-800 each),
   Conclusion (300-400)
4. Conversational tone: Informative but personable; avoid academic lecturing and marketing hype
5. Balanced perspective: Acknowledge both benefits and challenges of AI-driven development

## Non-Goals
- NOT a technical tutorial (readers won't code anything)
- NOT a manifesto or marketing document (balanced perspective only)
- NOT a comprehensive academic paper (accessible, not specialized)
- NOT promotion of any particular tool or platform

## Success Criteria
- Reader can articulate 3-5 ways AI-driven development changes their work
- Reader feels the author understands their perspective (not being sold to)
- Reader could summarize main points to a colleague
- Writing is consistent throughout (same tone, structure, depth)
```

### Writing Your Constitution: Guided Activity** (35 min)

Follow this structure:

**Part 1: Intent** (8 min)
- Answer: "Who am I writing for? What do I want them to understand?"
- Write 2-3 sentences describing your ideal reader
- Write 1-2 sentences on your goal for them

**Part 2: Quality Gates** (15 min)
- Think about your paper sections. What standards should they all meet?
- Example quality gates (choose/adapt some):
  - Length per section (how many words?)
  - Technical level (technical details or plain language?)
  - Citation style (how much evidence needed?)
  - Tone (formal, conversational, technical?)
  - Structure (every section has intro/body/conclusion?)
  - Clarity (how much jargon explanation needed?)
- Write 3-5 quality gates as simple rules

**Part 3: Non-Goals** (6 min)
- What are you NOT trying to do?
- What could readers mistakenly think is your goal? Say explicitly it's not.
- Write 2-4 non-goals

**Part 4: Success Criteria** (6 min)
- How will you know your paper succeeds?
- Write 3-4 observable criteria (things you can check at the end)

### Artifacts Produced

- **Artifact**: `constitution.md` (completed constitution document)

### Prerequisites from Earlier Lessons

- Chapter 13 (SDD-RI Theory): Understanding constitutions as decision frameworks
- Chapter 10 (Markdown): Formatting markdown documents

### Three Roles Demonstrated (Invisibly)

**What happens when you write this WITH your AI companion**:

1. **AI as Teacher**: Your AI suggests quality gates you didn't think of
   - "Have you considered a consistency check? Like: every section should start with a question?"
   - "Most successful research papers have a consistent structure. Here's what works..."

2. **You as Teacher**: You teach AI what matters to you
   - "Actually, I don't want citations. I'm writing from my experience as a developer."
   - "My audience doesn't have deep AI knowledge, so I need simpler language."

3. **Convergence**: Together you refine the constitution
   - AI suggests: "Sections should be 700 words"
   - You refine: "Only body sections. Intro should be shorter."
   - Final result: "Introduction (250 words), Body sections (700 words), Conclusion (300)"

### Try With AI

**Activity: Co-Author Your Constitution** (25 min)

Don't write it alone. Write it WITH your AI.

**Prompt to your AI**:
```
I'm writing a research paper on [your topic].
Help me create a constitution that will guide all my decisions.

Here's what I'm thinking:
- Audience: [describe them]
- Goal: [what should readers understand]
- Tone: [what kind of voice]
- Approximate length: [total words, and per-section]

Ask me clarifying questions to help me refine this constitution.
Then, help me write it in clear sections (Intent, Quality Gates, Non-Goals, Success Criteria).
```

**What you'll experience**:
- Your AI asks clarifying questions that force your thinking deeper
- You refine your answers as you explain them
- Your AI suggests structure and language for the constitution itself
- You adapt AI's suggestions to match your vision

**Reflection**:
- "What aspects did AI help you think more clearly about?"
- "How is this constitution different from what you would have written alone?"
- "Do you feel this will actually guide your future decisions?"

Save the conversation as `02-constitution-conversation.md`.

---

## Lesson 3: Specification Phase — Define WHAT You're Building

**Duration**: 75-90 minutes
**Stage**: L2 (AI Collaboration with invisible Three Roles)
**Bloom's Level**: Analyze + Create
**Concepts Introduced** (count: 4):
1. Specification as intent definition (WHAT, not HOW)
2. Success evaluation (observable criteria for completion)
3. Constraints (what you won't do)
4. Non-goals (deliberate exclusions)

### Learning Objectives

1. Understand why specification comes BEFORE planning (defining WHAT before HOW)
2. Write a complete specification for your research paper
3. Distinguish between success criteria (measurable) and vague wishes
4. Make deliberate trade-off decisions (what's in scope, what's out)

### Lesson Flow

**Why Specification Before Planning** (15 min)

In Chapter 13, you learned the theory. Now you experience WHY it matters:

**Scenario 1: Planning Without Specification (Fails)**
```
Your plan: "Write 5 sections, each 700 words"
You start writing...
After 3 sections, you realize:
- Your sections are overlapping (same points repeated)
- You're writing at different depth levels (one section is shallow, one is advanced)
- You have 5,000 words but only answered 2 of the 3 key questions you cared about

You're lost. You have to rewrite. Hours wasted.
```

**Scenario 2: Specification Before Planning (Succeeds)**
```
Your specification says:
- "The paper will answer: (1) What changed? (2) Why does it matter? (3) What now?"
- "Success criteria: Reader can explain these 3 questions in their own words"

You plan: "Section 1 answers 'what changed', Section 2 explains 'why', Section 3 addresses 'what now'"

You write with clarity. Every section knows its job. No overlap, no lostness.
You finish on time with exactly what you intended.
```

**The specification makes planning and writing 10x faster because you know exactly what you're building.**

**Specification Template** (20 min)

A complete specification has four parts:

**Part 1: Intent**
- What problem does your paper solve for readers?
- What do you want them to understand?
- Example: "Help software developers understand how AI changes their work, giving them hope and direction instead of fear"

**Part 2: Success Evals** (Measurable criteria)
- What observable behaviors prove your paper succeeded?
- These should be checkable: "Reader can articulate X", "Paper contains Y sections", "Content includes Z depth"
- Example evals:
  - "Reader can explain 3 ways AI-driven development differs from traditional development"
  - "Paper contains 3-5 main sections plus intro/conclusion"
  - "Each section has clear argument with examples"

**Part 3: Constraints** (What you DO define)
- What are your hard boundaries?
- Examples:
  - "5 sections maximum"
  - "3,000-4,000 words total"
  - "All claims either cited or grounded in demonstrated capability"
  - "Written for intermediate developers, not researchers"

**Part 4: Non-Goals** (What you deliberately don't do)
- What would be in scope but you're choosing NOT to do?
- Examples:
  - "NOT a how-to guide (no code tutorials)"
  - "NOT a academic deep-dive (keeping it accessible)"
  - "NOT promotion of specific tools (balanced perspective)"
  - "NOT a controversy piece (avoiding cultural debates)"

**Writing Your Specification** (35 min)

Work through each part:

**Intent** (5 min):
- Reread your constitution. What's your big goal?
- Write 2-3 sentences on what problem your paper solves

**Success Evals** (10 min):
- Imagine your paper is finished. How would you know it succeeded?
- Write 3-5 measurable criteria (things you can check)
- Make them observable: reader can articulate X, paper contains Y, text shows Z depth

**Constraints** (10 min):
- What are your hard boundaries?
- Length: How many words total?
- Sections: How many main sections?
- Depth: What level of technical detail?
- Evidence: How much must you cite/explain?
- Write 3-5 constraints as absolute rules

**Non-Goals** (10 min):
- What's deliberately out of scope?
- What could readers think you're trying to do (but you're not)?
- Write 2-4 non-goals

### Artifacts Produced

- **Artifact**: `spec.md` (completed specification)

### Prerequisites from Earlier Lessons

- Lesson 2 (Constitution): Your quality standards provide context
- Chapter 13 (SDD-RI Theory): Understanding specifications as intent blueprints

### Three Roles Demonstrated (Invisibly)

1. **AI as Teacher**: AI suggests success criteria you hadn't considered
   - "How will you know your readers understood your main argument?"
   - "What's an observable behavior that proves your intent worked?"

2. **You as Teacher**: You clarify your actual intent through conversation
   - "Actually, I don't care if they memorize details. I want them to feel empowered."
   - "My real goal is they see themselves as capable of this transition."

3. **Convergence**: Together you refine until spec is precise
   - AI: "Success criteria should be measurable..."
   - You: "Right, like: reader can articulate 3 ways AI changes their work"
   - Together: Refined success criteria that are both measurable and meaningful

### Try With AI

**Activity: Co-Author Your Specification** (20 min)

**Prompt to your AI**:
```
I'm creating a specification for my research paper: [topic]

Here's what I'm trying to accomplish:
- Intent: [your goal for readers]
- Core ideas: [main arguments you'll make]
- Audience: [who will read this]

Help me create a rigorous specification by:
1. Suggesting success evaluation criteria (how will readers know they learned something?)
2. Identifying constraints I should be explicit about (length, technical depth, etc.)
3. Pointing out what I might be implicitly trying to do but should explicitly exclude

Then, help me write this into a formal specification document with:
Intent | Success Evals | Constraints | Non-Goals
```

**What you'll experience**:
- AI asks questions that clarify your actual intent
- You refine your thinking as you explain it
- AI suggests success criteria you hadn't considered
- Together you build a spec that's both clear and rigorous

**Reflection**:
- "How is this specification different from what you would have written alone?"
- "Do you feel this spec actually defines what you're building?"
- "Could someone else read this spec and understand your vision?"

Save the conversation as `03-specification-conversation.md`.

---

## Lesson 4: Planning Phase — Design HOW You'll Build It

**Duration**: 60-75 minutes
**Stage**: L2 (AI Collaboration with invisible Three Roles)
**Bloom's Level**: Analyze + Create
**Concepts Introduced** (count: 4):
1. Section breakdown (logical divisions of your topic)
2. Section order (dependency and narrative flow)
3. AI collaboration strategy (how to use AI for each section)
4. Milestone tracking (knowing you're making progress)

### Learning Objectives

1. Break your paper into 3-5 manageable sections
2. Sequence sections in logical order (builds understanding progressively)
3. Decide how you'll collaborate with AI for each section
4. Create a milestone plan (when you'll finish each section)

### Lesson Flow

**Why Planning Matters** (10 min)

Your specification says WHAT. Your plan says HOW.

**Specification**: "Paper will answer: (1) What changed? (2) Why does it matter? (3) What now?"

**Plan translates this into sections**:
- **Section 1: "The Shift"** → Explains what changed in software development due to AI
- **Section 2: "Why This Matters"** → Explores impact on developers, productivity, careers
- **Section 3: "The Future of Software Work"** → Addresses what developers should do now
- **Plus**: Introduction (frames the conversation), Conclusion (brings it home)

Without this breakdown, you face a blank page. With this plan, each writing session knows exactly what it's building.

**Planning Template** (15 min)

Your plan should answer:

1. **What are your main sections?** (3-5 sections)
   - Title each section
   - Write 1-2 sentences on what it covers
   - Note any dependencies (does Section 2 require understanding from Section 1?)

2. **In what order?** (narrative flow)
   - Does this build understanding progressively?
   - Could a section be understood before an earlier one?
   - Or does each section require the previous one?

3. **How long is each?** (word budget)
   - Total words: 3,000-5,000
   - Intro: 250-300 words
   - Body sections: 600-800 words each
   - Conclusion: 300-400 words
   - Allocate your word budget across sections

4. **How will you collaborate with AI on each?** (AI strategy)
   - For some sections, you might: "I'll write the first draft, AI helps me refine"
   - For others: "AI helps me brainstorm ideas, then I write"
   - Others: "AI writes, I heavily edit for my voice"
   - **Key**: You decide the strategy upfront, don't figure it out while writing

5. **When will you finish each?** (milestone plan)
   - If you have 2 weeks, when do sections start/finish?
   - What's the checkpoint for each? (rough draft? final?)
   - When do you reflect and create your skill?

**Example Plan** (reference structure):

```markdown
# Research Paper Plan: How AI-Driven Development Changes Software Engineering

## Section Breakdown

### Introduction: The Moment We're In
- Length: 250 words
- Purpose: Hook reader, establish stakes
- What's changing: Software development is shifting from code-centric to spec-centric
- AI Strategy: I'll write this, AI helps me refine opening for clarity

### Section 1: What Changed
- Length: 700 words
- Purpose: Explain the paradigm shift clearly
- Core idea: From memorizing syntax to crafting specifications
- AI Strategy: I'll outline main points, AI helps flesh out examples and transitions

### Section 2: Why This Matters for Developers
- Length: 750 words
- Purpose: Help readers see personal relevance
- Core idea: This isn't job loss—it's liberation from busywork
- AI Strategy: AI helps brainstorm how this affects different developer roles,
  I pick what resonates

### Section 3: Building Intelligence, Not Code
- Length: 700 words
- Purpose: Introduce reusable intelligence concept
- Core idea: Skills and specifications become more valuable than code libraries
- AI Strategy: I'll write this with AI as thinking partner, iterating until clear

### Conclusion: Your Role in This Transition
- Length: 350 words
- Purpose: Call to action, empowerment
- Core idea: Readers are equipped to thrive in AI-native era
- AI Strategy: I'll draft, AI helps me find powerful closing language

## Total: 3,000 words (intro + 3 sections + conclusion)

## Milestones
- Week 1: Constitution + Specification + Plan (DONE)
- Week 2: Intro + Section 1 + Section 2
- Week 3: Section 3 + Conclusion + First revision
- Week 4: Final revision + Reflection + Skill extraction
```

**Creating Your Plan** (30 min)

Walk through each element:

**Sections** (10 min):
- List your 3-5 main sections
- Write what each covers
- Note any dependencies
- Allocate words per section (should add to ~3,000-5,000)

**Order** (5 min):
- Does the order make sense?
- Does each section build on previous understanding?
- Can you rearrange any sections for better flow?

**AI Collaboration Strategy** (10 min):
- For each section, decide: "Will I write first and refine? Or brainstorm with AI first?"
- Be specific about what AI does vs what you do
- Avoid: "AI writes it all" or "I write it alone"

**Milestone Plan** (5 min):
- Estimate how much time you have
- When will each section be drafted?
- When will you finish?
- When will you reflect and extract your skill?

### Artifacts Produced

- **Artifact**: `plan.md` (completed plan document)

### Prerequisites from Earlier Lessons

- Lesson 3 (Specification): Know what you're building before planning how
- Chapter 13 (SDD-RI Theory): Understanding plans as implementation roadmaps

### Three Roles Demonstrated (Invisibly)

1. **AI as Teacher**: AI suggests section order or structure you hadn't considered
   - "Have you thought about building understanding progressively? Maybe put the foundation first?"
   - "What if you reversed these sections? Does the narrative work better?"

2. **You as Teacher**: You clarify your specific writing style and process
   - "Actually, I like to write first and refine. I don't want AI writing for me."
   - "For technical sections, I want AI helping me explain clearly to non-experts."

3. **Convergence**: Together you refine the plan until it's actionable
   - AI: "How much do you want each section to be?"
   - You: "About 700 words, but shorter intro"
   - Final: "Intro 250, Sections 700 each, Conclusion 350"

### Try With AI

**Activity: Co-Author Your Plan** (20 min)

**Prompt to your AI**:
```
I have a specification for my research paper: [topic]

My success criteria are:
[List from spec]

I'm thinking of breaking the paper into these sections:
[List your proposed sections]

Help me refine this plan by:
1. Suggesting if the order makes sense for building understanding
2. Recommending word allocation (how long should each section be?)
3. For each section, asking: how should I collaborate with you? (I write first?
   We brainstorm? You draft and I refine?)
4. Creating a milestone timeline (how long should each section take?)

Then help me write this into a formal plan document.
```

**What you'll experience**:
- AI asks about narrative flow, making you think about progression
- You explain your writing preferences, clarifying your process
- AI suggests collaboration strategies you hadn't considered
- Together you build a realistic, detailed plan

**Reflection**:
- "How will this plan make your writing process clearer?"
- "What collaboration strategies surprised you?"
- "Do you feel ready to start writing?"

Save the conversation as `04-plan-conversation.md`.

---

## Lesson 5: Tasks & Implementation — Write Your Paper

**Duration**: 120-180 minutes (3-4 hours across multiple sessions)
**Stage**: L2-L3 (AI Collaboration, beginning to crystallize patterns)
**Bloom's Level**: Create + Evaluate
**Concepts Introduced** (count: 3):
1. Atomic tasks (smallest completion unit)
2. Validation checkpoints (how you know you're done)
3. Iterative refinement (revision as conversation with AI)

### Learning Objectives

1. Execute your plan: write your paper section by section
2. Use AI as a collaborative thinking partner, not a ghostwriter
3. Validate each section against your specification and constitution
4. Refine through iteration until you're satisfied

### Lesson Flow

**From Plan to Execution** (10 min)

You now have:
- ✅ Specification (what you're building)
- ✅ Plan (how you'll build it)
- Now: Execute the plan, section by section

**Key principle**: You're writing this paper. AI is helping you think. You decide quality, structure, voice. This is not: "Tell AI to write my paper." This is: "Tell AI to help me think about my topic, then I craft the writing."

**Execution Pattern for Each Section** (repeated for each section):

1. **Brainstorm** (15 min)
   - Prompt your AI with your section goal
   - Ask: "What are the key ideas here? What am I missing?"
   - Collect ideas without writing yet

2. **Draft** (30-45 min)
   - You write a first draft (rough, don't worry about perfection)
   - Use AI notes from brainstorming
   - Your voice, your structure

3. **Refine with AI** (20-30 min)
   - Share your draft with AI
   - Ask: "Is this clear? What's confusing? How could I strengthen this?"
   - Iterate: AI suggests → you adapt → repeat

4. **Validate** (10 min)
   - Check against your spec: Does this section do what it promised?
   - Check against your constitution: Does it match tone, length, quality gates?
   - If yes → move to next section
   - If no → iterate

5. **Mark Complete** (5 min)
   - Move section to `paper.md` (your final document)
   - Note in your tasks.md that section is complete

**Example: Section 1 Task Breakdown**

```markdown
## Task: Write Section 1 - "What Changed"

**Goal**: Help readers understand the paradigm shift from code-centric to spec-centric development

**Length**: 700 words

**Acceptance Criteria**:
- Explains what changed (code writing → specification writing)
- Provides 2-3 examples of how this changes the developer's work
- Tone matches constitution (conversational, not academic)
- Clear argument (not just information dumping)

**Execution Checkpoint**:
1. Brainstorm with AI: "What are the key ideas in this section?"
2. Draft: Write your understanding in your own words
3. Refine: Show AI draft, ask for feedback on clarity and examples
4. Validate: Section meets all acceptance criteria
5. Complete: Move to final paper.md

**Estimated Time**: 1.5 hours
```

**Writing Your Paper** (140-170 min across multiple sessions)

Work through your plan:
- Write one section at a time
- Use the execution pattern for each
- Validate against spec and constitution
- Build your paper incrementally

**Iteration and Refinement**

You will revise. This is expected and healthy.

**First draft** might be:
```
"AI changes development. Developers no longer write code manually.
They use AI to write code. This is a big change. It means different skills matter now."
```

**After refining with AI**, might become:
```
"The shift from code-centric to spec-centric development is fundamental.
Where developers once spent hours typing correct syntax, they now spend those hours
crafting precise specifications that AI agents implement. This changes what skills matter:
attention to detail in specification writing now outweighs keyboard speed.
The developer becomes an architect of intent, not a typist of syntax."
```

Same idea. Much clearer. AI helped you articulate what you meant.

### Artifacts Produced

- **Artifact**: `paper.md` (growing document, sections added as completed)
- **Artifact**: `tasks.md` (tracking what's done, what's next)
- **Artifacts**: Section conversation files (`05-section-1.md`, `05-section-2.md`, etc.)

### Prerequisites from Earlier Lessons

- Lesson 4 (Plan): Know your section goals and AI collaboration strategy
- Lesson 3 (Specification): Know what success looks like
- Lesson 2 (Constitution): Know your quality standards

### Three Roles Demonstrated (Invisibly)

**Throughout the writing process**:

1. **AI as Teacher**: AI helps you understand your topic more deeply
   - "What do you think is the deepest reason this matters?"
   - Suggests examples and metaphors you hadn't considered

2. **You as Teacher**: You teach AI your perspective and constraints
   - "You're explaining it too technically. My audience needs simpler language."
   - "I want this to feel hopeful, not threatening."

3. **Convergence**: Iterative refinement toward clear, authentic expression
   - Iteration 1: AI suggests → You adapt
   - Iteration 2: You try different structure → AI asks clarifying questions
   - Iteration 3: Together you land on something that's both clear AND your voice

### Try With AI

**Activity: Write One Section Using the Execution Pattern** (60-90 min per section)

For your first section, work through this fully:

**Phase 1: Brainstorm** (15 min)
```
Prompt your AI:
I'm writing a section on [section topic].
My goal is to help readers understand [section purpose].

What are the key ideas here? What should I not miss?
What are good examples of [topic]?
What common misunderstandings should I address?

Tell me the main ideas; I'll write them in my own words.
```

**Phase 2: Draft** (30-45 min)
- Rough draft, in your voice, using AI's ideas
- Don't aim for perfection; aim for first attempt
- Save as `05-section-X-draft.md`

**Phase 3: Refine** (20-30 min)
```
Prompt your AI:
Here's my draft of [section]:
[Paste your draft]

Review this for:
1. Is the main argument clear?
2. What's confusing?
3. Are my examples helpful?
4. How could I strengthen the conclusion?

Don't rewrite it. Ask me questions or suggest improvements.
```

**Phase 4: Validate** (10 min)
- Does it meet your spec's acceptance criteria?
- Does it match your constitution (tone, length, quality)?
- If yes → ready for final paper
- If no → clarify what's missing, revise

**Phase 5: Complete** (5 min)
- Move to `paper.md`
- Mark in tasks.md as complete

**Repeat for each section.**

**Reflection**:
- "How did writing with AI help you think more deeply?"
- "Where did you teach AI something about your perspective?"
- "How different is this from writing alone?"

---

## Lesson 6: Pattern Recognition — Identify What's Reusable

**Duration**: 60-75 minutes
**Stage**: L3 (Intelligence Design, beginning crystallization)
**Bloom's Level**: Analyze + Evaluate
**Concepts Introduced** (count: 3):
1. Recurring patterns (what happened in multiple sections)
2. Reusability criteria (does this apply beyond this project?)
3. Skill identification (what gets extracted for future use)

### Learning Objectives

1. Reflect on your writing process and identify patterns
2. Recognize which patterns are project-specific vs universally applicable
3. Select one pattern worthy of converting to a reusable skill
4. Articulate why this pattern has value beyond this one project

### Lesson Flow

**Why Patterns Matter** (10 min)

Chapter 13 taught: "Reusable intelligence is the future."

Now you experience why.

**Your writing process probably involved repeating things**:
- Brainstorming with AI (same pattern each time)
- Structuring sections (similar approach each section)
- Refining for clarity (iterative conversation)
- Checking against spec (validation loop)

If you write 5 more papers or documents, you'll repeat these patterns again.

**Without a skill**, you'll repeat them from scratch each time.

**With a skill** (reusable intelligence), you'll:
- Ask your AI to apply the pattern upfront
- Use it as a reminder of what works
- Compound your learning across projects

**Pattern Recognition Activity** (40 min)

Reflect on your writing process:

**Question 1: What patterns did you repeat?** (10 min)

Look at your section conversation files. What did you do multiple times?

Examples (your patterns might differ):
- "I always started by brainstorming with AI, asking 'What are the key ideas?'"
- "I wrote a rough draft, then showed it to AI for feedback on clarity"
- "I checked each section against my spec: Does it answer the question?"
- "I refined paragraphs with AI: 'Is this clear to someone unfamiliar with this?'"
- "When AI made suggestions, I adapted them to my voice"

List 3-5 patterns you repeated across sections.

**Question 2: Which patterns are universal?** (10 min)

For each pattern, ask: "If I were writing a completely different document (not a research paper), would this pattern still work?"

Examples:
- Pattern: "Brainstorm with AI, asking 'What are the key ideas?'"
  - Universal? **YES** (works for any complex topic)
- Pattern: "Check for academic tone"
  - Universal? **NO** (specific to research papers)
- Pattern: "Iteratively refine for clarity with AI feedback"
  - Universal? **YES** (works for any writing)

Mark which patterns are universal.

**Question 3: Which pattern will you turn into a skill?** (15 min)

Choose ONE pattern that:
- ✅ Is universal (applies beyond this project)
- ✅ Saved you time or improved quality
- ✅ You'd want to reuse in future projects
- ✅ Has enough complexity to be worth capturing (2-4 decision points)

Examples:
- "research-section-brainstorm": How to brainstorm clearly with AI before writing
- "write-and-refine-loop": How to draft, show AI, iterate toward clarity
- "specification-validation": How to check your work against your spec

**Why this pattern?** Write 2-3 sentences explaining:
- What problem does this pattern solve?
- How did it help you in this project?
- How would it help in future projects?

### Artifacts Produced

- **Artifact**: `06-pattern-reflection.md` (answers to reflection questions)
- **Selected Skill**: Decision on which pattern to turn into reusable intelligence

### Prerequisites from Earlier Lessons

- Lesson 5 (Implementation): Completed sections reveal your actual process
- Lesson 4 (Planning): Planned strategy vs. actual execution (what changed?)

### Try With AI

**Activity: Reflect on Your Process With Your AI Companion** (30 min)

Don't just reflect alone. Reflect WITH your AI.

**Prompt to your AI**:
```
I've just finished writing a research paper on [topic].

Looking back at the process, I notice I repeated certain patterns:
[List 3-5 patterns you noticed]

Help me identify which of these are:
1. Specific to this project (academic writing, this topic)
2. Universal (would apply to any complex writing or research task)

For the universal patterns, help me decide: which one should I turn into a
reusable skill for future projects?

What makes a pattern worth capturing? What's not worth turning into a skill?
```

**What you'll experience**:
- AI helps you see patterns you didn't fully articulate
- AI asks questions that deepen your reflection
- AI helps you distinguish project-specific from universal patterns
- Together you identify which pattern has the most reusable value

**Reflection**:
- "What surprised you about your own process?"
- "Did AI help you see patterns you missed?"
- "Are you excited about the skill you've chosen?"

Save the conversation as `06-reflection-conversation.md`.

---

## Lesson 7: Skill Creation — Build Reusable Intelligence

**Duration**: 90-120 minutes
**Stage**: L3 (Intelligence Design, crystallization complete)
**Bloom's Level**: Create
**Concepts Introduced** (count: 4):
1. Persona (who will use this skill)
2. Questions (what reasoning does the skill activate)
3. Principles (what frameworks guide the skill)
4. Skill structure (SKILL.md format for reusability)

### Learning Objectives

1. Understand Persona + Questions + Principles framework
2. Write your first reusable skill using this framework
3. Test your skill to ensure it's usable and activates reasoning
4. Prepare your skill for reuse in Lesson 8 capstone

### Lesson Flow

**What Makes a Reusable Skill?** (15 min)

A **skill** is different from a template.

**Template** (not reusable):
```
Title: __________
Introduction: __________
Main points: __________
Conclusion: __________
```
This is just a form to fill in. It doesn't make you think better.

**Skill** (reusable intelligence):
```
## Who Uses This

Developers writing research papers on AI and software development.

## What This Activates

This skill activates reasoning about:
- How to translate your deep knowledge into clear writing
- How to validate your work against rigorous criteria
- How to iterate toward clarity with AI as thinking partner

## The Pattern

[Persona + Questions + Principles guide you through the reasoning]
```

This is intelligence you can apply again and again.

**Persona + Questions + Principles Framework** (20 min)

Your skill has three parts:

**Part 1: Persona**
- Who would use this skill?
- What are they trying to do?
- What challenges do they face?
- Write 3-5 sentences on the user

Example:
```
Software developers writing research papers or technical articles.
They have deep knowledge but struggle to translate it into clear writing.
They want to avoid sounding academic or jargon-heavy. They want readers
to understand complex ideas without years of background. This skill helps
them clarify their thinking and validate their work.
```

**Part 2: Questions**
- What questions should a user ask themselves when using this skill?
- These questions activate reasoning, not just fill-in-the-blank
- 4-6 questions that guide the user's thinking
- Example questions:
  - "What's the one idea I'm trying to convey in this section?"
  - "What does my reader already know? What's new to them?"
  - "Have I used jargon without explaining it?"
  - "If I read this cold, would I understand the argument?"
  - "What would make this section more compelling?"

**Part 3: Principles**
- What decision frameworks guide the skill?
- These are NOT rules ("always do X"); they're frameworks ("when X, consider Y")
- 3-5 principles based on your experience
- Example principles:
  - "Specificity beats generality: Replace 'technology changes things' with 'AI shifts development from code writing to specification crafting'"
  - "Examples before abstractions: Show the concrete case first, then generalize"
  - "Voice matters: Write like you're explaining to a smart friend, not lecturing"
  - "Iteration improves clarity: Your first draft won't be clear; that's normal. Refine with feedback."

**Writing Your Skill** (60-80 min)

Create a file: `SKILL-[skill-name].md`

Structure:

```markdown
# Skill: [Your Skill Name]

## Persona

[Who uses this? What are they trying to do? Why?]

## Intent

[What does this skill help someone accomplish?]

## Core Questions

1. [Question 1]
2. [Question 2]
3. [Question 3]
4. [Question 4]
5. [Question 5]

## Guiding Principles

**Principle 1: [Name]**
[Framework description]

**Principle 2: [Name]**
[Framework description]

**Principle 3: [Name]**
[Framework description]

## How to Use This Skill

[Step-by-step: How does someone actually use this skill in practice?]

[Include 1-2 examples from your research paper]

## When This Skill Works Best

[What contexts is this designed for?]

## Example Usage

[Walk through an example: persona + questions + principles → better output]
```

**Example Skill** (reference structure):

```markdown
# Skill: Research Section Brainstorm

## Persona

Software developers writing research papers or technical articles.
They have deep domain knowledge but struggle to translate it into
clear, structured writing. They want to brainstorm effectively with AI
before writing, so their drafts are organized and purposeful instead of
scattered or meandering.

## Intent

This skill helps you brainstorm a research section with AI in a way that
produces organized, actionable ideas you can write from—not a vague pile
of thoughts.

## Core Questions

1. What's the ONE central idea of this section?
2. What doesn't my reader know yet that they need to know?
3. What's a concrete example that makes this idea real?
4. What's the most important question this section answers?
5. How does this section build on what came before?

## Guiding Principles

**Principle 1: Clarity Over Completion**
Before collecting all ideas, get one idea crystal clear.
A reader would rather understand one thing deeply than
feel lost in a swamp of possibilities.

**Principle 2: Concrete Before Abstract**
Start with the real example, then generalize. "AI writes code" is vague.
"Where you spent 3 hours typing syntax, you now spend that time crafting
the specification AI will implement" is concrete.

**Principle 3: Questions Unlock Thought**
Don't ask AI to "write about X". Ask AI "What are the key insights about X?"
The question determines what thinking gets activated.

## How to Use This Skill

1. Open your section outline
2. For that section, ask yourself: "What's the ONE central idea?"
3. Ask your AI: [Core Question 1]
4. Ask: [Core Question 2]
5. Ask: [Core Question 3]
6. Collect AI's answers
7. Now you have organized thoughts to write from

## Example From Research Paper

**Section Goal**: Explain how AI changes what developers do

**Using the skill**:
- Q1 (Central idea): "Developers shift from writing code to specifying intent"
- Q2 (What's new): "Most developers don't yet realize this is possible"
- Q3 (Concrete example): "Instead of typing loops, you describe what data transforms and AI writes the loop"
- Q4 (Key question): "What does this freedom enable?"
- Q5 (Builds on): "We established that AI can write code. Now: what does that change?"

**Result**: Organized brainstorm → clear section draft
```

### Artifacts Produced

- **Artifact**: `SKILL-[skill-name].md` (completed skill definition)

### Prerequisites from Earlier Lessons

- Lesson 6 (Pattern Recognition): Identified the pattern to extract
- Lesson 5 (Implementation): Actual experience using the pattern
- Chapter 13 (SDD-RI Theory): Understanding how skills encapsulate intelligence

### Try With AI

**Activity: Co-Author Your Skill With Your AI Companion** (40 min)

**Prompt to your AI**:
```
I've completed a research paper using this repeating pattern:
[Describe your pattern in detail]

I want to turn this into a reusable skill that other writers could use.

Help me create a skill by:
1. Defining the persona (who would use this? what are they doing?)
2. Identifying 5-6 core questions that activate reasoning
3. Extracting 3-4 guiding principles from my experience
4. Writing a step-by-step guide for using the skill
5. Including examples from my paper

The skill should help someone repeat this pattern with their own content,
not copy my paper.
```

**What you'll experience**:
- AI helps you articulate your implicit knowledge
- You refine your thinking as you explain it
- AI suggests questions you hadn't formulated
- Together you build a skill that's both clear and useful

**Reflection**:
- "Does this skill actually activate reasoning, or is it just a template?"
- "Would you use this skill for your next project?"
- "Could someone else understand and apply this skill?"

Save the conversation as `07-skill-creation-conversation.md`.

---

## Lesson 8: Capstone — Demonstrate Skill Acceleration

**Duration**: 120-180 minutes (2-3 hours)
**Stage**: L4 (Spec-Driven Integration)
**Bloom's Level**: Evaluate + Create
**Concepts Introduced** (count: 3):
1. Skill reuse (applying intelligence from previous project)
2. Intelligence acceleration (measurable speed/quality improvement)
3. Metacognition (reflecting on your own learning)

### Learning Objectives

1. Apply your created skill to a second task or section
2. Experience measurable acceleration (faster, better quality than first iteration)
3. Articulate how reusable intelligence compounds your capability
4. Reflect on what you've learned about SDD-RI methodology

### Lesson Flow

**Why Capstone Matters** (10 min)

You've now:
- ✅ Executed the complete Spec-Kit Plus workflow
- ✅ Created a reusable skill
- Now: Prove that the skill actually works

This is where theory becomes experience. This is where you see why SDD-RI matters.

**The Acceleration Hypothesis**

If intelligence truly compounds, then:
- **First project** (Research Paper): Full workflow from scratch
  - Constitution: 30 min
  - Specification: 45 min
  - Planning: 45 min
  - Implementation: 6-8 hours
  - Reflection + Skill: 2 hours
  - **Total: ~10 hours**

- **Second project** (Capstone): Using created skill
  - Specification: 20 min (you know how now)
  - Planning: 20 min (you know how now)
  - Implementation with skill: 2-3 hours (skill accelerates the work)
  - **Total: ~3 hours** (70% faster)

The skill accelerates you because:
- You don't reinvent the approach
- You ask better questions upfront
- Your AI companion applies your accumulated intelligence
- You make fewer false starts

**The Capstone Task Options** (10 min)

Choose ONE:

**Option 1: Write a Second Research Section**
- Different topic (e.g., "The Economics of AI-Driven Development")
- Same skill (research section brainstorm)
- 700-800 words
- Produce section in 2-3 hours using your skill

**Option 2: Write a Different Kind of Piece**
- Blog post, article, or document on a topic you care about (AI-related)
- Apply your skill to a new context
- Verify: Does it accelerate the work even in a different format?
- 1,000-1,500 words

**Option 3: Create a Second Skill**
- Identify a DIFFERENT pattern from your research paper process
- Apply Persona + Questions + Principles to create it
- Test it with one application
- Demonstrates you understand the pattern extraction process

**Recommended**: **Option 1** (most direct demonstration of acceleration)

**The Capstone Process** (100-150 min)

**Phase 1: Specification** (20 min)

What are you building with this capstone?

If you chose **Option 1** (Second Research Section):
```
Specification: Write a 700-word section on [new topic]
Intent: Explore a related AI question
Success Criteria:
- Section stands alone (could be read without context)
- Follows your constitution (tone, length, quality gates)
- Demonstrates the skill accelerated your work

Timeline: Complete in 2-3 hours (vs. 6-8 hours for first section)
```

**Phase 2: Preparation** (15 min)

Get your skill ready.

- Open your `SKILL-[name].md`
- Reread it. Make sure you understand your own skill.
- Update it if you learned anything during your research paper process

**Phase 3: Execution Using Your Skill** (90-120 min)

Execute your capstone using your skill.

For **Option 1** (Section), follow this:

```
Step 1: Review your skill (5 min)
- Open your skill definition
- Reread the questions and principles

Step 2: Brainstorm with your skill (20 min)
- Ask your AI to apply your skill
- "I created a research section brainstorm skill.
   Here's the skill definition: [paste]
   I want to write a new section on [topic].
   Apply the skill questions to help me brainstorm."

Step 3: Draft (30 min)
- Write your section draft, using brainstorm notes
- Same approach as first section, but faster

Step 4: Refine (30 min)
- Show AI your draft
- Ask for feedback on clarity (same iteration pattern)
- Refine 2-3 times until satisfied

Step 5: Validate (10 min)
- Check against spec: Does it meet your goal?
- Check against constitution: Tone, length, quality?
```

**Documenting Your Experience** (15 min)

As you work, note:
- **Time tracking**: How long did each phase take?
- **Comparison**: Was this faster than your first section? By how much?
- **Quality**: Is this section as good as, better than, or different from first section?
- **Observations**: What did the skill help you do better?

### Artifacts Produced

- **Artifact**: Capstone output (second section, article, or new skill—depending on your choice)
- **Artifact**: `08-capstone-reflection.md` (your experience, observations, learning)

### Prerequisites from Earlier Lessons

- Lesson 7 (Skill Creation): Your created skill
- Lesson 5 (Implementation): Understanding the writing/iteration process
- All previous lessons: Complete SDD-RI methodology experience

### Try With AI

**Activity: Execute Capstone With Your Skill** (100-120 min)

**For Option 1 (Second Section)**:

**Phase 1: Prep Your Skill** (5 min)
```
Prompt your AI:
I created a skill during my research paper project: [skill name]

Here's my skill definition:
[Paste your SKILL-[name].md]

I want to use this skill to write a new section on [topic].

First, review my skill. Ask me: "Does this skill still feel right?
Would you change anything based on what you learned writing the first paper?"
```

**Phase 2: Brainstorm** (20 min)
```
Prompt your AI:
Now apply my skill to help me brainstorm this new section.

Using my skill's core questions, help me think through:
1. [Core Question 1]
2. [Core Question 2]
... etc

Organize your answers so I can write from them.
```

**Phase 3: Draft** (30 min)
- Write your section draft (you write, not AI)
- Use AI's brainstorm notes
- Your voice, your structure

**Phase 4: Refine** (30 min)
```
Prompt your AI:
Here's my draft section: [paste]

Review it for:
1. Clarity (is the argument clear?)
2. Examples (are they helpful?)
3. Strength (what would make it better?)
4. Alignment (does it match my constitutional tone/length?)

Ask me questions or suggest improvements.
```

**Phase 5: Reflect** (15 min)

Answer in `08-capstone-reflection.md`:

```markdown
# Capstone Reflection

## Experience

**Time invested**: [total hours]

**Comparison to first section**:
- First section took approximately [X hours]
- This capstone took approximately [Y hours]
- **Acceleration**: [X - Y] hours faster ([percentage]%)

## What the Skill Helped

[Describe: What did your skill help you do better?]
Examples:
- Brainstorm more systematically
- Ask better questions upfront
- Avoid false starts
- Clarify your thinking faster

## Quality Comparison

**First section**:
- Strengths: [what worked well]
- Challenges: [what was hard]

**Capstone section**:
- Strengths: [what's better]
- What improved: [comparison]

## Learning: Why SDD-RI Matters

**Articulate the value you've experienced**:
- Why does specification upfront matter?
- How did reusable intelligence help you?
- What would you do differently in your next project?
- How would you teach someone else the methodology?

## What You'd Do Differently Next Time

[Based on your capstone experience, what would you change?]
```

**What You'll Experience**:

1. **Speed**: You'll notice this is faster. The skill compressed repetitive thinking.
2. **Quality**: You might notice it's better—the skill activates reasoning you didn't do consciously first time.
3. **Confidence**: You're applying methodology without constantly questioning yourself.
4. **Meta-awareness**: You'll start seeing the pattern everywhere—even in tasks outside writing.

**Final Reflection**:
- "Did the skill actually accelerate your work?"
- "Would you use this skill again?"
- "How does this prove the value of reusable intelligence?"
- "What's the difference between writing the first section vs. the capstone section?"

---

## Intelligence Accumulation Mapping

By the end of Chapter 14, what reusable intelligence have you created?

| Artifact | Layer | Reusability | Use Case |
|----------|-------|-------------|----------|
| **Constitution** | L1 | Project-specific | Your research paper standards |
| **Specification** | L2 | Template (pattern) | Future writing projects |
| **Plan** | L2 | Template (pattern) | Any complex writing project |
| **Skill** (P+Q+P) | L3 | Universal | ANY project needing this pattern |

**The skill is the primary intelligence artifact.** Constitution, Spec, and Plan are project artifacts. The Skill is intelligence that compounds.

---

## Chapter Completion Checklist

- [ ] Lesson 1: Chose research topic, committed to scope
- [ ] Lesson 2: Created constitution defining quality standards
- [ ] Lesson 3: Wrote specification defining what you're building
- [ ] Lesson 4: Created plan breaking project into sections
- [ ] Lesson 5: Completed research paper (3+ sections, 3,000-5,000 words)
- [ ] Lesson 6: Identified reusable pattern from your process
- [ ] Lesson 7: Created skill using Persona + Questions + Principles
- [ ] Lesson 8: Applied skill to capstone task, experienced acceleration

**All 8 checkboxes complete = You've mastered SDD-RI methodology.**

---

## Time Estimates

| Phase | Activity | Duration |
|-------|----------|----------|
| **Lesson 1** | Topic selection + project framing | 45-60 min |
| **Lesson 2** | Constitution creation | 60-75 min |
| **Lesson 3** | Specification writing | 75-90 min |
| **Lesson 4** | Planning + section breakdown | 60-75 min |
| **Lesson 5** | Implementation (writing paper sections) | 3-4 hours |
| **Lesson 6** | Pattern reflection + analysis | 60-75 min |
| **Lesson 7** | Skill creation (P+Q+P) | 90-120 min |
| **Lesson 8** | Capstone + reflection | 120-180 min |
| **Total** | Complete chapter | 8-12 hours |

**Recommended pacing**: 2-3 weeks at 3-4 hours/week

---

## Success Criteria Summary

**By end of Chapter 14, you will have**:

1. ✅ Executed `/sp.constitution` → `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow
2. ✅ Produced a complete research paper (3,000-5,000 words, 3-5 sections)
3. ✅ Created ONE reusable skill using Persona + Questions + Principles framework
4. ✅ Demonstrated skill reuse in capstone (measurable acceleration)
5. ✅ Can articulate the difference between SDD (workflow) and SDD-RI (workflow + reusable intelligence)
6. ✅ Understand why intelligence compounds and how skills accelerate future work

**These are the observable behaviors that prove mastery of Chapter 14.**

---

## Anti-Convergence Notes

This lesson plan deliberately varies from Chapter 13's pedagogy:
- **Chapter 13**: Problem-discovery (understanding WHY through error analysis)
- **Chapter 14**: Hands-on execution (understanding HOW through project building)

This variation prevents convergence on generic "lecture + exercise" patterns by forcing active, discovery-based learning through project creation.

**Three Roles Framework** remains INVISIBLE throughout—students experience AI as collaborative partner through narrative ("AI suggested", "You refined", "Together you converged") rather than studying framework labels.

**All lessons respect cognitive load limits** (A2: max 7 concepts, B1: max 7-10 concepts) and maintain clear pedagogical progression through 4 layers.

---

**Document Version**: 1.0
**Created**: 2025-11-26
**Constitutional Alignment**: v6.0.1 (Reasoning-Activated)
**Ready for Implementation**: Yes
