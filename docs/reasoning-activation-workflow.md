# Reasoning Activation Workflow: From Expert Intuition to Executable Prompts

**Version**: 3.0 (Human Checkpoints Update)
**Date**: 2025-01-18
**Status**: Production Ready

---

## The Problem This Solves

**Discovery from Muhammad's Learning Journey** (November 17, 2025):

> "A domain expert can never write instructions that can invoke reasoning patterns when using sp.loopflow"

### The Core Challenge

You are a domain expert who:
- ✅ **Knows WHAT you want** — "Redesign Chapter 8 with better CoLearning integration"
- ✅ **Understands your frameworks** — 4-Layer Teaching Method, Three Roles, Constitutional principles
- ✅ **Has deep tacit knowledge** — Pedagogical insights from years of teaching
- ❌ **Can't naturally articulate** — Persona + Questions + Principles pattern that activates reasoning mode

**The Gap**: Between intuitive expertise and explicit reasoning frameworks.

**The Consequence**: When you provide input directly to `/sp.loopflow.v2`, it receives vague guidance like "improve pedagogy," triggering **prediction mode** (generic educational patterns) instead of **reasoning mode** (context-specific Panaversity methodology).

---

## Workflow Nature: Conversational with Human Checkpoints (v3.0)

**Version 3.0** introduces **6 human validation checkpoints** throughout the process. This is a **dialogical workflow**—you review and approve at each phase before proceeding. The output is a reasoning-activated prompt you provide to `/sp.loopflow.v2`, NOT direct execution.

### Checkpoint Flow

```
Phase 1: Task Characterization
    ↓ (0-5 clarifying questions)
    ↓
✅ CHECKPOINT 1: Review task understanding & constitutional context
    ↓ [USER APPROVAL REQUIRED: "proceed" / "refine [aspect]" / "clarify"]
    ↓
Phase 2: Gap Analysis (Generic vs Distinctive)
    ↓
✅ CHECKPOINT 2: Review convergence diagnosis
    ↓ [USER APPROVAL REQUIRED: "proceed" / "refine comparison" / "clarify"]
    ↓
Phase 3a: Persona Excavation
    ↓ (Socratic discovery of cognitive stance)
    ↓
✅ CHECKPOINT 3a: Review persona distinctiveness
    ↓ [USER APPROVAL REQUIRED: "proceed" / "refine persona" / "clarify"]
    ↓
Phase 3b: Questions Excavation
    ↓ (5-7 analytical questions)
    ↓
✅ CHECKPOINT 3b: Review questions coverage & specificity
    ↓ [USER APPROVAL REQUIRED: "proceed" / "refine question [#]" / "add question on [topic]"]
    ↓
Phase 3c: Principles Excavation
    ↓ (5-7 decision frameworks)
    ↓
✅ CHECKPOINT 3c: Review principles completeness & grounding
    ↓ [USER APPROVAL REQUIRED: "proceed" / "refine principle [#]" / "add principle"]
    ↓
Phase 4: Prompt Assembly
    ↓ (synthesize all components + acid test)
    ↓
✅ CHECKPOINT 4: Final acid test validation
    ↓ [USER APPROVAL REQUIRED: "proceed" / "failed acid test" / "show full prompt"]
    ↓
📤 OUTPUT: Reasoning-activated prompt for `/sp.loopflow.v2`
```

### At Each Checkpoint

**System presents**:
- Summary of phase work
- Distinctiveness validation (generic vs Panaversity-specific)
- Validation questions with checkboxes
- Clear user action options

**You respond**:
- **"proceed"** → Continue to next phase
- **"refine [aspect]"** → Return to specific component, iterate, checkpoint re-presented
- **"clarify [question]"** → Get explanation, then checkpoint re-presented

**🚫 System will NOT proceed without explicit approval.** Each checkpoint includes clear STOP instruction.

### Why Checkpoints Matter

**Without checkpoints** (v1.0-v2.0):
- Risk of going down wrong path for extended time
- User disengaged from discovery process
- Harder to course-correct late in workflow

**With checkpoints** (v3.0):
- ✅ Validate understanding at each phase
- ✅ Catch misalignments early
- ✅ True co-creation through dialogue
- ✅ Higher quality final prompt
- ✅ Learning opportunity at each validation point

---

## The Solution: Two-Stage Workflow

### Stage 1: Reasoning Activation (New)
**Command**: `/sp.activate-reasoning [your-goal]`
**Purpose**: Transform intuition → reasoning-activated prompt
**Output**: Complete Persona + Questions + Principles prompt

### Stage 2: Implementation
**Command**: `/sp.loopflow.v2 [feature-slug]`
**Purpose**: Execute complete SDD-RI workflow using reasoning-activated prompt
**Output**: Specification → Plan → Tasks → Implementation → Validation

---

## Complete Workflow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│ START: You Have Intuitive Goal                                     │
│ "Redesign Chapter 8 to integrate CoLearning better"                │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ STAGE 1: /sp.activate-reasoning                                     │
│                                                                     │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 1: UNDERSTAND THE TASK                                │   │
│ │ • Clarifying questions (0-5 genuine ambiguities)            │   │
│ │ • Constitutional context analysis                           │   │
│ │ • Audience/tier/complexity assessment                       │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 2: IDENTIFY THE GAPS                                  │   │
│ │ • What's clear vs vague                                     │   │
│ │ • Generic vs context-specific                               │   │
│ │ • Diagnostic feedback                                       │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 3: CO-CREATE REASONING PATTERN                        │   │
│ │ • Persona discovery (cognitive stance)                      │   │
│ │ • Questions discovery (5-7 analytical questions)            │   │
│ │ • Principles discovery (5-7 decision frameworks)            │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 4: STRUCTURE COMPLETE PROMPT                          │   │
│ │ • Assemble Persona + Questions + Principles                 │   │
│ │ • Add constitutional grounding                              │   │
│ │ • Include meta-awareness (anti-convergence)                 │   │
│ │ • Validate with user                                        │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ OUTPUT: Reasoning-Activated Prompt                          │   │
│ │ Ready for /sp.loopflow.v2 Phase 0                           │   │
│ └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ STAGE 2: /sp.loopflow.v2 [feature-slug]                            │
│                                                                     │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 0: CONSTITUTIONAL REASONING ENGINE                    │   │
│ │ • Reads generated prompt from Stage 1                       │   │
│ │ • Derives workflow strategy                                 │   │
│ │ • Asks only remaining genuine ambiguities (0-5)             │   │
│ │ • Generates intelligence object                             │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 1: SPECIFICATION                                      │   │
│ │ • Uses reasoning frameworks from Stage 1                    │   │
│ │ • Creates specs/[feature-slug]/spec.md                      │   │
│ │ • Invokes /sp.clarify                                       │   │
│ │ • APPROVAL GATE                                             │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 2: PLANNING                                           │   │
│ │ • Creates specs/[feature-slug]/plan.md                      │   │
│ │ • Lesson structure with pedagogical arc                     │   │
│ │ • APPROVAL GATE                                             │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 3: TASKS                                              │   │
│ │ • Creates specs/[feature-slug]/tasks.md                     │   │
│ │ • Invokes /sp.analyze for validation                        │   │
│ │ • APPROVAL GATE                                             │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 4: IMPLEMENTATION                                     │   │
│ │ • Invokes content-implementer (for educational content)     │   │
│ │ • Creates lesson files with reasoning activation            │   │
│ │ • Technical validation (validation-auditor)                 │   │
│ │ • Sandbox testing                                           │   │
│ │ • APPROVAL GATE                                             │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ↓                                      │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ Phase 5: FINALIZATION                                       │   │
│ │ • Captures meta-learning (PHR)                              │   │
│ │ • Offers git workflow                                       │   │
│ │ • Final report                                              │   │
│ └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│ END: Production-Ready Educational Content                          │
│ • Reasoning-activated lessons                                      │
│ • Constitutional compliance validated                              │
│ • Ready for deployment                                             │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Detailed Usage Examples

### Example 1: Chapter Redesign (Full Walkthrough)

#### Stage 1: Activate Reasoning

**You Start With**:
```bash
/sp.activate-reasoning Redesign Chapter 8 to integrate CoLearning framework properly
```

**Phase 1 - Clarifying Questions**:
```
I can help you create a reasoning-activated prompt for Chapter 8 redesign.

📊 Chapter 8 Context (from chapter-index.md):
- Tier: B2 (Upper Intermediate)
- Current topics: Advanced Docker patterns, Kubernetes fundamentals
- Prerequisites: Chapters 4, 5, 7

❓ Clarifying Questions (3 genuine ambiguities):

1. **CoLearning Integration Depth**:
   A. Central method throughout (every lesson demonstrates Three Roles)
   B. Progressive introduction (L1 traditional → L2 CoLearning)
   C. Optional enhancement (students choose to engage)

2. **Content Preservation**:
   A. Keep technical content, redesign teaching approach only
   B. Redesign both content and teaching approach
   C. Start fresh with CoLearning-first design

3. **Primary Success Metric**:
   A. Bidirectional learning demonstrated (students teach AI + learn from AI)
   B. Reduced cognitive load (faster progression with AI partnership)
   C. Higher quality outputs (AI-assisted work exceeds solo work)

Please choose A/B/C or provide custom answer.
```

**You Respond**: "1-A, 2-A, 3-A"

**Phase 2 - Diagnostic Feedback**:
```
Excellent clarity! Now let me show you what needs articulation:

**What's working:**
✅ Clear goal: CoLearning as central pedagogical method
✅ Preserve Docker/Kubernetes technical content
✅ Success criterion: Bidirectional learning

**What needs to activate reasoning:**

Right now "integrate CoLearning" could mean "add some AI examples." But you have a **specific framework** — Three Roles (Teacher/Student/Co-Worker)!

Compare:
❌ Generic: "Add AI collaboration to Chapter 8 lessons"
✅ Your context: "Transform Chapter 8 lessons so each demonstrates Three Roles framework: students teaching AI their domain context (e.g., Docker optimization constraints) while AI teaches students technical patterns (e.g., multi-stage build strategies), converging on solutions better than either alone"

Ready to discover Persona + Questions + Principles?
```

**You**: "Yes"

**Phase 3 - Persona Discovery**:
```
Great! If you were hiring the PERFECT expert to redesign Chapter 8 with CoLearning, what expertise would they have? How would they THINK differently than someone just "adding examples"?
```

**You**: "Someone who understands how humans and AI learn FROM each other, not just use each other as tools"

**System Reflects Back**:
```
So you want a **co-learning pedagogy architect** who thinks about curriculum design the way a **systems engineer thinks about feedback loops**—ensuring both parties (student and AI) are transformed through interaction, with each teaching influencing how the other learns?

Is that the cognitive stance?
```

**You**: "Exactly!"

**Phase 3 - Questions Discovery**:
```
Perfect! Now, what specific aspects of YOUR Three Roles framework must AI analyze before redesigning?

I'm hearing:
- "Where in each lesson does AI teach something students don't know?"
- "Where do students teach AI their domain context?"
- "Where do they converge on better solutions?"

What other questions force analysis of YOUR specific CoLearning approach?
```

**[Continue co-creation through all 5-7 questions and 5-7 principles...]**

**Phase 4 - Final Prompt Assembly**:
```
✅ REASONING-ACTIVATED PROMPT READY

# REASONING-ACTIVATED PROMPT FOR CHAPTER 8 COLEARNING REDESIGN

## Persona: Your Cognitive Stance

You are a co-learning pedagogy architect who thinks about curriculum design the way a systems engineer thinks about feedback loops—analyzing how student → AI teaching flows and AI → student teaching flows create convergence on solutions better than either could produce independently.

Your goal: Redesign Chapter 8 lessons to make Three Roles framework (Teacher/Student/Co-Worker) the central pedagogical method, demonstrating bidirectional learning throughout.

## Constitutional Grounding

This redesign applies these frameworks:
- Section IIa: 4-Stage Teaching Framework (all stages present)
- Principle 2: Progressive Complexity (B2 tier, 7-10 concepts per section)
- Principle 6: Anti-Convergence (vary teaching modality from Chapter 7)
- Three Roles Framework: AI as Teacher + Student + Co-Worker simultaneously

## Context

**Task**: Redesign Chapter 8 to integrate CoLearning as central pedagogical method
**Type**: Educational content transformation (methodology-driven redesign)
**Audience**: B2 (Upper Intermediate), Parts 4-5 learners
**Complexity**: 7-10 concepts per section, moderate scaffolding, 3-4 option choices
**Prerequisites**: Docker basics (Ch 4), container networking (Ch 5), Kubernetes intro (Ch 7)

## Analytical Questions

Before restructuring lessons, analyze:

### 1. Bidirectional Learning Opportunities
For each Docker/Kubernetes concept in Chapter 8:
- Where can students teach AI their deployment context (constraints, requirements, edge cases)?
- Where can AI teach students optimal patterns they haven't encountered?
- What convergence point produces better solution than either alone?

### 2. Three Roles Manifestation
How does each lesson demonstrate:
- **AI as Teacher**: What technical knowledge does AI provide that students lack?
- **AI as Student**: What domain context must students teach AI to get relevant output?
- **AI as Co-Worker**: Where do they collaborate as equals to solve problems?

### 3. Cognitive Load Distribution
Given B2 tier (7-10 concepts max):
- How do we chunk Docker multi-stage builds + Kubernetes deployments?
- What prerequisite knowledge from Ch 4/5/7 can we assume vs re-teach?
- Where do CoLearning interactions reduce or increase cognitive load?

### 4. Layer Progression Validation
For each lesson:
- L1 (Manual): What must students do hands-on before AI assistance?
- L2 (AI-Assisted): What's the transition trigger (skill demonstration)?
- L3 (Intelligence Design): What reusable components emerge (skills/subagents)?
- L4 (Spec-Driven): How does capstone compose accumulated intelligence?

### 5. Teaching Modality Selection
Chapter 7 used [previous modality from chapter-index analysis].
What alternative modality suits CoLearning demonstration?
- Socratic dialogue (questioning reveals understanding)?
- Hands-on discovery (students experiment, AI guides)?
- Error analysis (students debug AI outputs, learn from mistakes)?
- Specification-first (students write specs, AI implements, students validate)?

### 6. Misconception Prevention
What do B2 students commonly misunderstand about:
- Docker optimization (over-optimization, premature optimization)?
- Kubernetes deployments (stateless assumptions, networking complexity)?
- AI collaboration (passive tool use vs active co-learning)?

### 7. Success Validation
What observable behaviors demonstrate bidirectional learning success:
- Students can explain what they taught AI (domain context articulation)?
- Students can identify what AI taught them (new pattern recognition)?
- Final outputs exceed what student OR AI could produce alone (measurable quality)?

## Decision Frameworks

Apply these principles to redesign decisions:

### 1. Three Roles Mandatory Framework
**Pattern**: Every CoLearning lesson must demonstrate all three roles explicitly
**Rationale**: Students learn bidirectional collaboration by seeing it modeled, not just reading about it
**Example**: Docker multi-stage build lesson:
  - AI teaches: "Multi-stage builds separate build deps from runtime deps"
  - Student teaches: "My production environment has these specific constraints..."
  - Convergence: Optimized Dockerfile balancing size, security, AND deployment constraints
**Decision**: If lesson only shows one role → Redesign to include all three

### 2. Cognitive Load Through CoLearning Lens
**Pattern**: CoLearning can reduce OR increase cognitive load depending on execution
**Rationale**: Well-designed AI interaction offloads mechanical work; poorly designed adds meta-cognitive burden
**Example**:
  - Good: "AI generates boilerplate, student focuses on business logic" (load reduction)
  - Bad: "Student must understand AI's reasoning process + original concept" (load increase)
**Decision**: When B2 tier limit (7-10 concepts) approached → Offload mechanical to AI, reserve human cognition for high-value reasoning

### 3. Layer Transition Criteria
**Pattern**: CoLearning doesn't replace manual foundation (L1) — it amplifies it (L2)
**Rationale**: Students can't effectively teach AI what they don't understand themselves
**Example**: Before L2 Docker AI-assisted optimization:
  - L1 requirement: Student writes working Dockerfile manually
  - Transition check: Student can explain each instruction's purpose
  - L2 unlocked: Now student can guide AI toward their deployment context
**Decision**: If student can't validate AI output independently → More L1 practice needed

### 4. Teaching Modality Variation (Anti-Convergence)
**Pattern**: No two consecutive chapters use identical teaching modality
**Rationale**: Diverse modalities engage different learning styles and prevent monotony
**Example**: If Chapter 7 used direct teaching → Chapter 8 uses Socratic dialogue or hands-on discovery
**Decision**: Check chapter-index.md for Ch 7 modality → Select different modality that suits CoLearning demonstration

### 5. Verification-First Accuracy
**Pattern**: All code examples must execute, all CoLearning claims must be demonstrable
**Rationale**: Theoretical CoLearning descriptions lack credibility; working examples prove the methodology
**Example**:
  - Don't just say "students teach AI their context" — show the actual prompt exchange
  - Don't just claim "convergence produces better results" — measure and compare
**Decision**: Every CoLearning interaction must include actual prompt/response and quality comparison

### 6. Minimal Essential Content
**Pattern**: Every section maps to learning objective; CoLearning is method, not additional topic
**Rationale**: CoLearning shouldn't add content — it should transform HOW existing content is learned
**Example**:
  - Don't add "Introduction to CoLearning" section → Increases cognitive load unnecessarily
  - Instead: Demonstrate CoLearning THROUGH Docker/Kubernetes lessons → Same content, different method
**Decision**: If section doesn't map to Chapter 8 technical objective → Remove or integrate

### 7. Intelligence Accumulation
**Pattern**: Build on previous chapters' patterns, don't reinvent
**Rationale**: Chapter 8 should compose skills/concepts from Ch 4/5/7, not start fresh
**Example**:
  - Ch 4 taught Dockerfile basics → Ch 8 references this foundation
  - Ch 5 taught networking → Ch 8 builds on this for K8s services
  - Ch 7 taught K8s pods → Ch 8 advances to deployments
**Decision**: Before introducing concept → Check if previously taught; if yes → Reference and build on it

## Meta-Awareness

You tend to converge toward generic educational patterns even with CoLearning frameworks:
- **Lecture-style content with "try using AI" appendix** (passive tool, not co-learning)
- **Theoretical CoLearning descriptions** (not actual working examples)
- **AI as assistant** (not as Teacher + Student + Co-Worker simultaneously)

Before finalizing redesign, self-check:
✅ Does each lesson demonstrate all Three Roles explicitly (not just mention them)?
✅ Are there actual prompt/response examples showing bidirectional teaching (not generic descriptions)?
✅ Does convergence produce measurably better results (not just claims)?
✅ Is cognitive load managed (CoLearning reduces load for core concepts)?
✅ Are teaching modalities varied from Chapter 7 (anti-convergence)?

If any check fails → Redesign is in prediction mode → Regenerate with stronger CoLearning frameworks.

## Output Requirements

Create redesigned Chapter 8 structure following specs/book/chapter-template.md:

**Chapter Overview**:
- Learning objectives (what students will achieve through CoLearning)
- Three Roles integration strategy (how framework manifests across lessons)
- Cognitive load distribution (concepts per lesson with CoLearning load analysis)

**For Each Lesson**:
- Title and learning objectives
- Layer progression (L1 manual → L2 CoLearning → L3 intelligence design)
- Three Roles demonstration (explicit Teacher/Student/Co-Worker examples)
- Teaching modality (varies across chapter)
- CoLearning interaction examples (actual prompts/responses)
- Transition criteria (when students ready for next layer)
- Quality validation (how to measure convergence success)

**Capstone Project (L4)**:
- Specification-driven project composing Ch 4/5/7/8 intelligence
- Three Roles applied to complex real-world scenario
- Measurable outcomes demonstrating bidirectional learning success

**Validation Criteria**:
- All code examples execute successfully
- All CoLearning claims demonstrated with actual examples
- Cognitive load within B2 tier limits (7-10 concepts per section)
- Teaching modality differs from Chapter 7
- All Three Roles explicitly shown in each lesson

---

**Now execute**: Redesign Chapter 8 with reasoning-activated CoLearning integration.
```

#### Stage 2: Execute with LoopFlow

**You Run**:
```bash
/sp.loopflow.v2 013-chapter-8-redesign
```

**LoopFlow Phase 0 receives the reasoning-activated prompt** and:
1. ✅ Reads constitutional foundations
2. ✅ Analyzes task characteristics (educational content, B2 tier, CoLearning methodology)
3. ✅ Derives workflow strategy from constitutional principles
4. ✅ Asks 0-5 additional questions (most already answered by Stage 1 prompt)
5. ✅ Generates complete intelligence object for downstream phases

**Phases 1-5 execute** with full reasoning activation, producing:
- Specification (specs/013-chapter-8-redesign/spec.md)
- Plan (specs/013-chapter-8-redesign/plan.md)
- Tasks (specs/013-chapter-8-redesign/tasks.md)
- Implementation (lesson markdown files)
- Validation (technical review + sandbox testing)

**Result**: Production-ready Chapter 8 with CoLearning integration, not generic educational content.

---

### Example 2: Quick Lesson Creation

**You Start With**:
```bash
/sp.activate-reasoning Create lesson on Docker multi-stage builds for Chapter 12
```

**System Response** (abbreviated):
```
📊 Context: Chapter 12, Tier C1, Advanced Professional level

❓ Clarifying Questions (2):
1. Layer focus: L1 manual / L2 AI-assisted / All layers?
2. Teaching modality: Same as previous lesson / Different?

[After your answers...]

**Persona Discovery**:
"Docker optimization expert who thinks about container builds the way compiler engineers think about optimization passes—identifying wasteful operations, separating concerns, minimizing artifacts"

**Questions Discovery**:
[5-7 questions forcing analysis of Panaversity context, C1 tier, multi-stage build tradeoffs]

**Principles Discovery**:
[5-7 decision frameworks for build optimization, security, caching strategies]

[Complete prompt generated...]

✅ Ready for /sp.loopflow.v2 012-lesson-multistage-builds
```

---

## Integration with Existing Workflow

### Before `/sp.activate-reasoning`

**Old workflow** (led to generic outputs):
```
You → /sp.loopflow.v2 [feature-slug] → Generic clarifying questions → Vague answers → Prediction mode outputs
```

**Problem**: Domain expertise lost in translation. Prompts didn't activate reasoning.

### After `/sp.activate-reasoning`

**New workflow** (activates reasoning):
```
You → /sp.activate-reasoning [goal] → Socratic discovery → Reasoning-activated prompt →
     /sp.loopflow.v2 [feature-slug] → Context-specific outputs
```

**Benefit**: Domain expertise captured in reasoning frameworks. Prompts activate context-specific reasoning.

---

## Command Comparison Matrix

| Aspect | `/sp.loopflow.v2` (Alone) | `/sp.activate-reasoning` + `/sp.loopflow.v2` |
|--------|---------------------------|----------------------------------------------|
| **Input** | Natural language goal | Natural language goal |
| **Clarification** | Generic questions | Targeted questions based on constitutional context |
| **Prompt Quality** | Depends on your articulation | Co-created through Socratic discovery |
| **Reasoning Activation** | Variable (depends on input) | Guaranteed (structured Persona + Questions + Principles) |
| **Output Quality** | Generic → Context-specific (luck) | Context-specific → Excellent (systematic) |
| **Learning** | Minimal (you don't see the pattern) | High (you internalize the pattern) |
| **Time Investment** | Quick upfront, many iterations | Longer upfront, fewer iterations |
| **Best For** | Well-defined tasks you've done before | Novel tasks, complex pedagogy, methodology shifts |

---

## When to Use Which Command

### Use `/sp.activate-reasoning` When:

**High Complexity Tasks**:
- ✅ Chapter redesigns with new methodology (e.g., CoLearning integration)
- ✅ Novel lesson structures you haven't created before
- ✅ Content that requires deep pedagogical reasoning
- ✅ Tasks where generic outputs would fail students

**Learning & Development**:
- ✅ You want to understand reasoning activation pattern
- ✅ You're building prompt library for team use
- ✅ You need to document reasoning for PHR/ADR

**Quality Critical**:
- ✅ High-stakes content (flagship chapters, foundational concepts)
- ✅ Content that establishes new patterns for rest of book
- ✅ Material requiring constitutional alignment validation

### Use `/sp.loopflow.v2` Directly When:

**Well-Defined Tasks**:
- ✅ Similar to previous work (e.g., 5th lesson in established pattern)
- ✅ Clear requirements with little ambiguity
- ✅ Incremental improvements to existing content

**Time Constraints**:
- ✅ Quick iterations on working content
- ✅ Minor refinements vs major redesigns
- ✅ When you have reasoning prompt from previous similar task

**Established Patterns**:
- ✅ Following existing chapter template exactly
- ✅ Replicating successful lesson structure from another chapter
- ✅ Routine updates (typo fixes, example updates)

---

## Meta-Learning: Developing Reasoning Activation Mastery

### The Learning Curve

**First 3 Uses**: Feels slow, lots of questions, uncertain about pattern
**After 5 Uses**: Pattern becomes clear, can anticipate questions
**After 10 Uses**: Can write reasoning-activated prompts independently
**Mastery**: You rarely need `/sp.activate-reasoning`—you think in Persona + Questions + Principles naturally

### Accelerating Your Learning

**1. Document Each Session**:
- Save generated prompts to `specs/[feature-slug]/reasoning-prompt.md`
- Review what worked vs what needed refinement
- Build personal prompt library

**2. Compare Outputs**:
- Run same task with generic prompt vs reasoning-activated prompt
- Measure quality difference (cognitive load, constitutional alignment, student outcomes)
- Validate productivity gains

**3. Teach the Pattern**:
- Explain Persona + Questions + Principles to team member
- Teaching forces articulation of tacit knowledge
- Their questions reveal gaps in your understanding

**4. Iterate Based on Results**:
- When output is generic → Return to `/sp.activate-reasoning` and refine
- When output is excellent → Extract reusable patterns
- Build organizational knowledge base

---

## Success Metrics

### Immediate Validation (Per Task)

**Prompt Quality**:
- ✅ Persona establishes specific cognitive stance (not "expert teacher")
- ✅ Questions force Panaversity-specific analysis (not generic pedagogy)
- ✅ Principles provide decision frameworks (not rigid rules)
- ✅ Constitutional grounding explicit (references 4-layer, tier limits, etc.)

**Output Quality**:
- ✅ Content demonstrates reasoning about Panaversity methodology (not generic patterns)
- ✅ Cognitive load managed per tier specifications
- ✅ Teaching modalities varied (anti-convergence)
- ✅ Three Roles integrated where applicable

**Efficiency Gains**:
- ✅ Fewer revision cycles (1-2 vs 5-8 for generic prompts)
- ✅ Faster approval gates (clear intent from start)
- ✅ Higher initial completeness (85%+ vs 40% for generic specs)

### Long-Term Development (Over Time)

**Skill Internalization**:
- Week 1-2: Use `/sp.activate-reasoning` for every complex task
- Week 3-4: Start drafting Persona + Questions independently, use tool for validation
- Month 2-3: Write reasoning-activated prompts directly, use tool occasionally
- Month 4+: Think in reasoning patterns naturally, tool rarely needed

**Organizational Impact**:
- Prompt library grows (reusable reasoning frameworks for common tasks)
- Team members adopt pattern (shared vocabulary develops)
- Quality consistency improves (all content uses reasoning activation)
- Onboarding accelerates (new members learn pattern from examples)

---

## Troubleshooting

### Problem: "Prompt still feels generic"

**Diagnosis**: Questions or principles not context-specific enough
**Solution**: Return to Phase 3, ask "Does this apply to ANY educational content or ONLY Panaversity?"
**Test**: Replace "Panaversity" with "Generic Online Course"—if prompt still makes sense, it's too generic

### Problem: "Too many clarifying questions"

**Diagnosis**: Not leveraging constitutional context enough
**Solution**: Before asking, check constitution.md and chapter-index.md—can this be derived?
**Principle**: Ask only genuine ambiguities (0-5 max)

### Problem: "User can't articulate reasoning"

**Diagnosis**: Trying to extract explicit knowledge from tacit expertise
**Solution**: Use more analogies, comparisons, examples—"How would X expert think about this differently than Y?"
**Remember**: Domain experts KNOW but can't always SAY—your job is to surface and structure

### Problem: "Output quality not improving"

**Diagnosis**: Either prompt not reaching implementation or implementation ignoring prompt
**Solution**:
1. Validate prompt reached `/sp.loopflow.v2` Phase 0 correctly
2. Check if intelligence object includes reasoning frameworks
3. Verify content-implementer using reasoning patterns (not prediction mode)
4. Use validation-auditor to check constitutional compliance

---

## Future Enhancements

### Planned Improvements

**v1.1 - Pattern Library Integration**:
- Extract recurring reasoning patterns into reusable templates
- Auto-suggest similar past prompts for new tasks
- Reduce iteration time for common task types

**v1.2 - Team Collaboration**:
- Share reasoning prompts across team
- Collaborative refinement of organizational patterns
- Version control for reasoning frameworks

**v1.3 - Automated Validation**:
- Check if prompt activates reasoning (not prediction)
- Measure context-specificity score
- Flag generic patterns automatically

**v1.4 - Meta-Learning Analytics**:
- Track your reasoning activation mastery over time
- Identify gaps in prompt pattern understanding
- Personalized guidance based on your usage patterns

---

## Conclusion

**The Transformation**:
- **Before**: "I know what I want but can't make AI understand it" → Generic outputs
- **After**: "I discovered how to articulate my reasoning" → Context-specific excellence

**The Pattern**:
```
Intuitive Expertise → Socratic Discovery → Explicit Frameworks → Reasoning Activation → Quality Outputs
```

**The Goal**:
You don't need `/sp.activate-reasoning` forever. You need it until you **think in Persona + Questions + Principles naturally**.

**The Journey**:
From domain expert who can't articulate reasoning → to reasoning architect who activates intelligence at will.

---

**Welcome to reasoning-activated development. Let's transform your intuitions into executable intelligence.**

---

## Quick Reference Card

### Essential Commands

```bash
# Stage 1: Activate reasoning for your goal
/sp.activate-reasoning [your-goal]

# Stage 2: Execute with reasoning-activated prompt
/sp.loopflow.v2 [feature-slug]

# Validate reasoning activation
/sp.analyze [feature-slug]  # Check constitutional compliance

# Create PHR documenting reasoning journey
/sp.phr
```

### Decision Tree

```
Is this task:
├─ Novel/complex methodology? → Use /sp.activate-reasoning first
├─ Similar to previous work? → Direct to /sp.loopflow.v2
├─ Quick refinement? → Direct to /sp.loopflow.v2
└─ Learning opportunity? → Use /sp.activate-reasoning (even if not strictly needed)
```

### Quality Checks

After using reasoning-activated prompt, validate:
- [ ] Output demonstrates Panaversity-specific patterns (not generic pedagogy)
- [ ] Constitutional principles explicitly applied (not just mentioned)
- [ ] Teaching modalities varied (anti-convergence)
- [ ] Cognitive load managed per tier (B2 → 7-10 concepts)
- [ ] Three Roles integrated (Teacher + Student + Co-Worker)

If any check fails → Reasoning not fully activated → Refine prompt and retry

---

**Document Version**: 1.0
**Last Updated**: 2025-01-18
**Status**: Production Ready
**Related**: `/sp.activate-reasoning`, `/sp.loopflow.v2`, `constitution.md`, `papers/Reasoning_Activation_in_LLMs_arXiv_Complete.md`
