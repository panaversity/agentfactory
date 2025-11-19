# Chapter 13 Plan Summary: Reasoning-Activated Lesson Design

**Created**: 2025-01-18
**Status**: Plan Complete (Ready for `/sp.tasks` and implementation)
**Document**: Full plan at `plan.md`

---

## I. Pedagogical Architecture (Reasoning Framework Applied)

### Narrative Arc Model
Rather than organizing Chapter 13 by topics ("Lesson 1: Python Basics", "Lesson 2: Variables", etc.), the plan uses a **story-based narrative arc** that unfolds like a journey:

```
Opening Hook: "Imagine teaching a robot to make a sandwich"
             ↓
First Victory: "Robot speaks" (print function)
             ↓
Rising Action: "Robot remembers" (variables)
             ↓
Building Complexity: "Robot organizes knowledge" (type hints)
             ↓
Peak Engagement: "Robot listens" (user input)
             ↓
Resolution: "Robot introduces itself" (capstone synthesis)
```

**Why This Structure?**
- Narrative engagement > topic lists for absolute beginners
- Story thread connects lessons (robot metaphor throughout)
- Rising action matches cognitive load progression (light → moderate)
- Capstone resolution satisfies learning journey (not just "here's what you learned")

### Anti-Convergence Detection Applied
This plan explicitly avoids the generic "uniform 9-lesson template" pattern by:
1. **Analyzing concept density**: 9 core concepts across 6 lessons = 1.5 avg/lesson (justified, not arbitrary)
2. **Justifying lesson count**: 6 lessons for A2 beginners (not 9) because heavy scaffolding requires breathing room
3. **Verifying stage progression**: All lessons Stage 1 (manual) with gentle Stage 2 introduction in "Try With AI"
4. **Enforcing teaching modality variation**: Story-based (Ch 13) vs Analogy-based (Ch 14) ≠ replicating same approach

---

## II. Concept Density Analysis (Constitution Principle 2)

### Core Concepts Identified
From approved spec, extracted 9 distinct concepts requiring mastery:

1. **What programming is** (giving instructions)
2. **Why learn Python** (practical, widely-used)
3. **Running Python code** (terminal execution)
4. **Print function** (output to screen)
5. **Variables concept** (memory labels)
6. **Creating variables** (assignment syntax)
7. **Data types concept** (different kinds of data)
8. **Type hints** (specifying data types)
9. **User input** (reading from user)

### Distribution Across Lessons

| Lesson | Concepts | Count | Proficiency | Load |
|--------|----------|-------|-------------|------|
| L1: Programming Intro | What programming is, Why Python, Terminal primer | 3 | A2 | Light |
| L2: First Program | Running Python, Print function | 2 | A2 | Light |
| L3: Variables | Memory concept, Variables, Creating variables | 3 | A2 | Light-Moderate |
| L4: Type Hints | Data types, Type hint syntax, Why types matter | 3 | A2-B1 | Light-Moderate |
| L5: User Input | Input function, Integration of L1-L4 | 2 | A2-B1 | Moderate-Peak |
| L6: Capstone | 0 new concepts (synthesis only) | 0 | A2-B1 | Integration |
| **TOTAL** | **9 core concepts** | **13 across 6 lessons** | **A2-B1** | **2.2 avg** |

### Justification for 6-Lesson Structure
- **A2 tier constraint**: Max 5-7 concepts per lesson, heavy scaffolding required
- **Story breathing room**: Narrative needs pacing (not compressed)
- **Confidence-building**: Early lessons intentionally light
- **Capstone foundation**: L1-L5 must establish before synthesis
- **NOT arbitrary template**: Every lesson justified by cognitive load analysis

---

## III. Stage Progression Strategy (Constitution Section IIa)

### Chapter 13 = Pure Stage 1 (Manual Foundation)
All lessons L1-L6 prioritize **manual practice** over AI collaboration:
- Students manually write code (not copy-paste from AI)
- Students manually test and verify (not trust AI output blindly)
- Students manually debug (learn error-reading skills)

### "Try With AI" as Intentional Bridge (NOT Stage 2)
"Try With AI" sections in each lesson are:
- **AI as practice partner**: Generates scenarios for students to code
- **AI as validating listener**: Students explain concepts to AI
- **AI as clarifier**: Answers student questions about concepts
- **NOT**: AI as solution generator ("Ask AI to write the code")

### Stage 2 Deferred to Chapter 14+
Chapter 13 establishes foundation. Stage 2 (AI collaboration with Three Roles) begins in Chapter 14 with deeper teaching.

---

## IV. Teaching Modality Variation (Anti-Convergence)

### Chapter 14 Approach: Analogy-Based Concept-Before-Syntax
- **Hook**: Kitchen jar analogy (organizing data like ingredients)
- **Pattern**: "This type is for...", "Use string when...", classification framework
- **Modality**: Systematic, organized, declarative ("Here's what you need to know")

### Chapter 13 Approach: Story-Based Narrative Journey
- **Hook**: Robot sandwich story (programming = giving instructions)
- **Pattern**: "Robot learns to speak → remember → organize → listen → introduce itself"
- **Modality**: Narrative, experiential, discovery-based ("Here's a journey you're on")

### Key Distinction
| Dimension | Ch 13 | Ch 14 |
|-----------|-------|-------|
| **Metaphor** | Robot learning | Kitchen organization |
| **Flow** | Narrative arc | Classification system |
| **Engagement** | Story-driven | Concept-driven |
| **Student perspective** | "I'm on a learning journey" | "I'm learning a taxonomy" |

These are **genuinely different modalities**, not duplicate approaches.

---

## V. Cognitive Load Compliance (CEFR A2 Tier)

### A2 Tier Constraints Applied
- **Concept limit**: Max 5-7 NEW concepts per lesson ✅
- **Scaffolding intensity**: Heavy handholding required ✅
- **Option presentation**: Max 2 choices (prevent paralysis) ✅
- **Language**: Define EVERY term, use analogies liberally ✅
- **Pacing**: 30-45 minutes per lesson (not rushed) ✅

### Validation Distribution
| Lesson | New Concepts | Limit | Status |
|--------|--------------|-------|--------|
| L1 | 3 | 7 | Within ✅ |
| L2 | 2 | 7 | Within ✅ |
| L3 | 3 | 7 | Within ✅ |
| L4 | 3 | 7 | Within ✅ |
| L5 | 2 | 7 | Within ✅ |
| L6 | 0 | 7 | Within ✅ |

**All lessons comply with A2 cognitive load limits.**

---

## VI. WHAT-WHY-HOW Progression (Spec Requirement)

### Applied Consistently Across All Lessons

**Pattern**: Explain concept PURPOSE before teaching syntax

#### Example: Lesson 3 (Variables)
1. **WHAT is a variable?** → Definition: labeled container for data
2. **WHY use variables?** → Purpose: computers need to remember things
3. **Real-world analogy** → Pet name labels (label = variable name, pet = data)
4. **Building mental model** → Computer memory diagram showing labeled storage
5. **NOW show syntax** → `name = "Alice"` appears AFTER understanding
6. **HOW to use it** → Code examples using variables with print()

Variables are **NOT taught** as arbitrary syntax. Students understand INTENT first.

#### Applied to All Lessons
- L1: WHAT is programming (instructions) → WHY learn Python (practical)
- L2: WHAT is print() (tell computer to display) → WHY use it (communicate with users)
- L3: WHAT is variable (memory label) → WHY needed (remember data)
- L4: WHAT are types (data categories) → WHY matter (different operations)
- L5: WHAT is input() (ask user) → WHY use (interactive programs)
- L6: Synthesis (no new concepts, all practice)

---

## VII. Story Cohesion Map

### Narrative Thread (Robot Metaphor)
```
L1: "Teaching a robot to make sandwiches" = Programming defined as instruction-giving
L2: "Robot speaks" = Robot's first words (Hello World)
L3: "Robot remembers" = Robot has memory for data (variables)
L4: "Robot organizes knowledge" = Classifying data by type
L5: "Robot listens" = Two-way communication (user input)
L6: "Robot introduces itself" = Capstone uses all skills to create introduction card
```

**Continuity Check**: Each lesson references previous (no disconnected topics)
- L2 assumes understanding from L1 (programming is instructions)
- L3 builds on L2 (robot now needs memory for data)
- L4 builds on L3 (robot needs to organize its memory)
- L5 builds on L4 (robot can classify input data)
- L6 uses all (robot uses all skills to introduce itself)

---

## VIII. Intelligence Accumulation Strategy

### Chapter 13 Does NOT Create Stage 3 Intelligence
**Reason**: Chapter 13 is foundation-only. No recurring patterns justifying reusable skills/subagents.

### What Gets Built For
- **Chapter 14 (Data Types)**: Type hints introduced in Ch 13 L4, deep-dived in Ch 14
- **Chapter 15 (Operators)**: Variables from Ch 13, operators manipulate them
- **Chapter 17 (Control Flow)**: Boolean type from Ch 13, used in conditionals
- **Chapter 20 (Functions)**: Function design builds on variable/parameter concepts

### When Skills Are Created
- **Chapter 14+**: Type classification task recurs 2+ → "type-decision-tree" skill created
- **Chapter 17+**: Conditional patterns recur → "conditional-logic" skill created
- **Chapter 20+**: Function design patterns recur → "function-signature-designer" skill

---

## IX. Success Metrics

### Specification Alignment
All 28+ functional requirements from spec addressed:
- FR-001 to FR-010: Story-based, WHAT-WHY-HOW, A2 cognitive load
- FR-011 to FR-015: Stage 1 foundation, teaching modality variation, scaffolding hidden
- FR-016 to FR-020: Code tested, claims cited, validation gates

### Success Criteria Mapping
Each lesson maps to multiple success criteria:

| Lesson | Maps to EV/SC |
|--------|---------------|
| L1 | EV-001 (explain what Python is) |
| L2 | EV-002 (write & execute Hello World) |
| L3 | EV-003 (create variables), EV-004 (explain why) |
| L4 | EV-003 (type hints), EV-004 (why types matter) |
| L5 | Extends EV-002 (now interactive) |
| L6 | EV-005 (capstone), EV-006 (code executes) |

**All success criteria covered by lesson plan.**

---

## X. Anti-Pattern Prevention

### Explicitly Avoided
- ❌ Lecture-style: "Here are 10 facts about Python" → Replaced with narrative
- ❌ Topic taxonomy: "Lesson 1: History", "Lesson 2: Philosophy" → Replaced with story arc
- ❌ Syntax-first: "Here's how to write variables" → Replaced with WHAT-WHY-HOW
- ❌ Scaffolding exposure: "Stage 1 Focus" in student text → Hidden (pedagogy invisible)
- ❌ Arbitrary lesson count: "9 lessons" without justification → Justified by concept density (6 lessons)

### Validation Gates Built In
Plan includes explicit sections for:
- Concept density analysis (every lesson justified)
- Cognitive load audit (stay within A2 limits)
- Story cohesion verification (narrative continuous)
- WHAT-WHY-HOW pattern check (always before syntax)
- Teaching modality differentiation (story vs analogy verified)

---

## XI. Implementation Roadmap

### Phase 1: Content Creation (Per Lesson)
Each lesson follows template:
1. **Narrative hook**: Story element connecting to previous lesson
2. **Concept explanation** (WHAT-WHY): No code yet, build understanding
3. **Code introduction** (HOW): Show syntax after understanding established
4. **Practice exercises**: Simple to complex, scaffolded
5. **"Try With AI"** section: Practice prompts, not solution-generation

### Phase 2: Validation Gates (After Each Lesson)
Before moving to next lesson:
- [ ] Story-based modality clearly evident
- [ ] WHAT-WHY-HOW pattern followed
- [ ] Code executes successfully (test logs attached)
- [ ] Cognitive load within limits
- [ ] "Try With AI" prompts appropriate
- [ ] Success criteria mapping verified

### Phase 3: Final Validation (After All Lessons)
Comprehensive audit:
- [ ] Pedagogical effectiveness (story cohesion, engagement indicators)
- [ ] Constitutional compliance (all 7 principles applied)
- [ ] Factual accuracy (all code tested, claims cited)
- [ ] Cognitive load assessment (A2 tier limits confirmed)
- [ ] Anti-pattern detection (no scaffolding exposure, no meta-commentary)

---

## XII. Key Decisions & Justifications

| Decision | Justification | Source |
|----------|---------------|--------|
| **6 lessons** (not 9) | 9 core concepts, A2 heavy scaffolding requires breathing room | Concept density analysis |
| **Story-based modality** | Narrative engagement > topic lists for beginners; demystifies programming | Pedagogy + spec preference |
| **Robot metaphor** | Single consistent analogy throughout (not multiple metaphors) | Anti-convergence + cohesion |
| **L6 capstone synthesis** | 0 new concepts, proves mastery of L1-L5 | Bloom's progression |
| **"Try With AI" not Stage 2** | Chapter 13 is Stage 1 foundation; Stage 2 deferred to Ch 14+ | Constitutional progression |
| **Type hints in L4** | Prepares for Ch 14 deep dive; normalizes practice early | Intelligence accumulation |
| **No operators/strings** | Stay within Chapter 13 scope; Ch 15/16 their responsibility | Non-goals compliance |
| **Common mistakes sections** | A2 learners make predictable errors; friendly guidance builds confidence | Beginner-friendly pedagogy |

---

## XIII. Files Produced

### Specification Artifacts
- **`plan.md`**: Full implementation plan (1,266 lines)
  - Technical context + pedagogical arc
  - Lesson-by-lesson breakdown (6 lessons detailed)
  - Cognitive load distribution + story mapping
  - Success criteria + quality assurance checklist
  - YAML frontmatter template + timeline estimates

### Next Outputs (After Plan Approval)
- **`tasks.md`**: Generated by `/sp.tasks` (breakdown into actionable tasks)
- **6 lesson files**: Generated by `/sp.implement` (actual content)
- **`README.md`**: Chapter overview + learning journey map

---

## XIV. Success Definition

### Plan Succeeds When:
✅ Lesson count justified by concept density (not arbitrary)
✅ Stage progression enforced (L1-L5 Stage 1, gentle L2 intro in Try With AI)
✅ Cognitive load respects A2 tier (all lessons ≤7 concepts)
✅ WHAT-WHY-HOW applied consistently (no syntax-first)
✅ Story-based modality clearly distinguishes from Ch 14 analogy approach
✅ Teaching modality variation is intentional, not accidental
✅ Anti-convergence safeguards active (narrative-driven, not lecture-driven)

### Implementation Succeeds When:
✅ Students read L1 and feel programming is demystified (not intimidated)
✅ 90%+ write Hello World within 15 minutes of L2
✅ 80%+ understand variables as memory labels (not just syntax)
✅ 75%+ explain why types matter (not just "I learned types")
✅ 70%+ complete capstone independently (synthesis achieved)
✅ Validation passes all checks (pedagogical + constitutional + factual)

---

## XV. Document Navigation

**For Implementation Details**, read full `plan.md`:
- **Pedagogical Arc Overview** (p.3): Visual flow diagram
- **Lesson-by-Lesson Breakdown** (p.14-80): Each lesson fully detailed
- **Cognitive Load Distribution** (p.89): Concept audit table
- **Quality Assurance Checklist** (p.110): 30+ validation gates
- **YAML Frontmatter Template** (p.125): For lesson metadata
- **Implementation Sequence** (p.100): Step-by-step order

---

**Status**: Plan complete and ready for `/sp.tasks` task generation and implementation phase.

