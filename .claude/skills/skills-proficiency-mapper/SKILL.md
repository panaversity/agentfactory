---
name: "skills-proficiency-mapper"
description: "Map skills to lessons with international standards proficiency levels (CEFR A1-C2), validate proficiency progressions, and generate assessment rubrics aligned to cognitive complexity (Bloom's taxonomy) and digital competence (DigComp). Grounded in 40+ years of research spanning language learning, cognitive science, and digital competence frameworks."
  - Grep
  - Bash
  - Task
version: "2.0.0"
research-foundation:
  - "CEFR (Common European Framework of Reference): 40+ years of language learning research"
  - "Bloom's Taxonomy: 70+ years of cognitive complexity research (2001 revision)"
  - "DigComp 2.1: Latest digital competence framework (2022)"
improvements-v2:
  - "Added skill coherence validation (detect fragmentation, regressions, missing prerequisites)"
  - "Added skill dependency mapping (prevent A2 skills without A1 prerequisites)"
  - "Added naming consistency checks (use distinct verbs: assess vs. evaluate vs. prioritize)"
  - "Added skill connectivity verification (ensure skills aren't isolated)"
  - "Added proficiency progression validation (A1→A2→B1, never regress)"
  - "Integrated Master Skill Registry concept (single source of truth)"
  - "Added 5 validation tests for cross-chapter consistency"
---

# Skills-Proficiency-Mapper Skill

## Purpose

Map learning skills to lessons with **international standards-based proficiency levels**, validate that proficiency progression follows learning science principles, and generate assessment rubrics that measure understanding at each proficiency level.

This skill transforms generic learning objectives into **research-grounded, measurable competencies** with clear proficiency targets that enable:
- Student skill tracking with internationally recognized levels
- Competency-based assessment design
- Institutional accreditation alignment
- Employer credentialing (portable credentials)
- Differentiation and extension activities

---

## When to Use This Skill

### Primary Activation Triggers

**During Chapter Planning**:
- "What skills should students develop in this chapter?"
- "Map the proficiency progression for these lessons"
- "Which skills should reach B1 level by chapter end?"
- "Create a skills inventory for this chapter"

**During Lesson Design**:
- "What proficiency level should this lesson target?"
- "Validate that this lesson teaches at B1 level"
- "Design scaffolding to move from A2 to B1 proficiency"
- "Create assessment rubrics for this proficiency level"

**During Assessment Design**:
- "What does A1, A2, B1 understanding look like for this skill?"
- "Design a rubric measuring B1-level skill mastery"
- "Create test items appropriate to proficiency level"
- "Validate assessment measures intended proficiency"

**During Differentiation**:
- "Design extension activities for B1+ students"
- "Create remedial activities for A1-level strugglers"
- "Build scaffolding appropriate to proficiency jump from A1→A2"

---

## Foundational Frameworks: 40 Years of Research

### 1. CEFR: Proficiency Levels (40+ years)

**Origin**: Common European Framework of Reference for Languages
**Timeline**: 1980s research, 1996 publication, 2020 digital update
**Research Base**: Validated across 40+ languages, millions of learners
**Why Credible**: Officially used by 40+ countries; de facto standard for 100+ countries

**Proficiency Levels**:

| Level | Name | Description | Bloom's Equivalent | DigComp Category |
|-------|------|-------------|-------------------|------------------|
| **A1** | Foundation | Recognize and understand basic concepts | Remember, Understand | Foundation |
| **A2** | Basic | Apply in simple, familiar contexts | Understand, Apply | Foundation-Intermediate |
| **B1** | Intermediate | Apply to real, unfamiliar problems | Apply, Analyze | Intermediate-Competent |
| **B2** | Advanced | Analyze and evaluate complex situations | Analyze, Evaluate | Competent-Expert |
| **C1** | Proficient | Expert independent use; teach others | Evaluate, Create | Expert |
| **C2** | Expert | Thought leadership and innovation | Create | Expert |

**How to Recognize Each Level**:

- **A1**: Student can recognize skill when shown, understand basic definition, follow example
- **A2**: Student can perform skill in simple, guided context; apply to textbook scenarios
- **B1**: Student can apply skill independently to real problems; handle routine variations
- **B2**: Student can analyze when and how to apply skill; evaluate alternative approaches
- **C1**: Student can design and teach the skill to others; solve novel complex problems
- **C2**: Student innovates in the domain; advances the field

---

### 2. Bloom's Taxonomy: Cognitive Complexity (70+ years)

**Origin**: Educational psychology research
**Timeline**: 1956 original taxonomy, 2001 revision with updated language
**Research Base**: Most widely-adopted cognitive framework in education globally

**Levels** (from simplest to complex):

1. **Remember** (Recall facts/concepts)
   - "Define A1-level understanding"
   - Action verbs: define, duplicate, list, memorize, recall, repeat, reproduce
   - Assessment: Multiple choice, fill-in-the-blank, recognition tasks

2. **Understand** (Explain meaning, summarize, interpret)
   - "Explain why A2-level is important"
   - Action verbs: classify, describe, discuss, explain, identify, locate, recognize, report, select, translate
   - Assessment: Short-answer, essay, concept maps, analogies

3. **Apply** (Use information in new situations)
   - "Use this skill to solve a new problem"
   - Action verbs: apply, change, choose, demonstrate, dramatize, practice, schedule, sketch, solve, use
   - Assessment: Case studies, problem-solving, scenario-based questions

4. **Analyze** (Distinguish between components, identify relationships)
   - "Compare approach A vs. B; identify tradeoffs"
   - Action verbs: appraise, compare, contrast, criticize, differentiate, evaluate, examine, explain, infer, relate
   - Assessment: Analysis essays, decision matrices, fault diagnosis

5. **Evaluate** (Judge based on criteria, justify choices)
   - "Assess which approach is best; defend your choice"
   - Action verbs: appraise, argue, defend, judge, select, support, value, evaluate
   - Assessment: Debate, critique, written justification, design reviews

6. **Create** (Produce new or original work; assemble parts into new whole)
   - "Design a novel solution combining these principles"
   - Action verbs: assemble, construct, create, design, develop, formulate, write, generate, organize, plan
   - Assessment: Design projects, original compositions, entrepreneurial ventures

**CEFR-Bloom's Alignment**:

```
A1-A2 (Foundation-Basic):        Remember ↔ Understand
B1 (Intermediate):                Apply ↔ Analyze
B2-C1 (Advanced-Proficient):      Evaluate ↔ Create
C2 (Expert):                       Create (mastery, innovation)
```

---

### 3. DigComp 2.1: Digital Competence Framework (Latest 2022)

**Origin**: European Union initiative
**Timeline**: Evolved 2013→2016→2022 (current)
**Research Base**: OECD, UNESCO adoption; reflects modern digital realities

**Five Competence Areas**:

1. **Information & Data Literacy**: Find, evaluate, organize, manage information
2. **Communication & Collaboration**: Communicate, share, collaborate, participate via digital media
3. **Digital Content Creation**: Create, edit, produce digital content; manage intellectual property
4. **Safety**: Protect self, data, devices, privacy, copyright, wellbeing
5. **Problem-Solving**: Solve technical problems, use tools creatively, troubleshoot

**Proficiency Levels** (matching CEFR):
- Foundation, Intermediate, Competent, Highly Competent, Expert

**Mapping SDD Skills to DigComp**:
- Specification Writing → Content Creation (Area 3)
- Team Coordination → Communication & Collaboration (Area 2)
- Context Architecture → Problem-Solving (Area 5)
- Commitment Planning → Safety & Professional Ethics (Area 4)

---

## How This Skill Works

### Input Requirements

**Minimum Input**:
- Lesson title or description
- Expected proficiency level (A1-C2)
- Skills to teach (names and categories)

**Full Input**:
- Chapter context (prerequisites, where this fits)
- Target audience (beginner, intermediate, advanced, professional)
- Available time/length
- Related skills from prior lessons
- Assessment method preferences

### Process Overview

#### Step 1: Validate Proficiency Level Appropriateness

**Question**: Is the intended proficiency level realistic for:
- The time available (A1 takes less time than B1)?
- The prerequisites taught (can't teach B1 without A2 foundation)?
- The audience complexity tier (Parts 1-3=A1-A2, Parts 4-5=A2-B1, Parts 6-8=B1-B2, Parts 9-13=B2-C1)?

**Output**: "✓ B1-level proficiency is appropriate for 2.5-hour lesson with A2 prerequisites"

#### Step 2: Map Skills to Lesson

**Question**: Which specific skills does this lesson teach? At what proficiency level?

**Process**:
1. Review existing skill inventory (registry.md from Part 5, or create new)
2. List skills explicitly:
   - Name (e.g., "Specification Writing")
   - Category (Technical / Conceptual / Soft)
   - Proficiency level (A1, A2, B1, etc.)
   - Measurable indicator (what "B1-level Specification Writing" looks like)

**Output**:
```yaml
skills:
  - name: "Specification Writing"
    proficiency: "B1"
    category: "Technical"
    measurable_at_this_level: "Student can write complete spec with Intent, I/O, Requirements, NFRs, Acceptance Criteria for real problem"

  - name: "Requirements Clarity"
    proficiency: "B1"
    category: "Soft"
    measurable_at_this_level: "Student can explain requirement ambiguities and propose clarifications"
```

#### Step 3: Design Proficiency Progression

**Question**: How does this lesson move students from current level to target level?

**Process**:
- **Scaffolding**: Break target proficiency into sub-steps
  - A1→A2: Show worked example, identify components, practice with support
  - A2→B1: Solve guided problem, solve semi-independent, solve fully independent
  - B1→B2: Analyze tradeoffs, critique approaches, design alternatives

- **Cognitive Load Management**:
  - A1: Max 5 new concepts; focus on understanding one thing deeply
  - A2: Max 7 new concepts; start applying to simple contexts
  - B1: Max 10 new concepts; apply to real problems with variations
  - B2+: No limit; synthesis and evaluation expected

**Output**:
```markdown
## Proficiency Progression: A2→B1

### Stage 1: Foundation Review (15 min)
- Review A2-level (simple spec writing with support)
- Check comprehension of core concepts
- *Cognitive load: 0 new concepts*

### Stage 2: Introduce B1-level (30 min)
- Worked example: Real specification with all components
- Analyze what makes it work (Intent? Complete? Testable?)
- *Cognitive load: 3 new concepts (NFRs, Testability, Real Context)*

### Stage 3: Guided Practice (30 min)
- Semi-realistic problem with scaffolding
- Student writes spec with checklist support
- Instructor feedback on clarity
- *Cognitive load: 2 new concepts (Independence, Real Problems)*

### Stage 4: Independent Application (45 min)
- Student selects real problem (from capstone, job, etc.)
- Writes specification from scratch
- Self-assesses against rubric
- *Cognitive load: 0 new concepts; all practice*
```

#### Step 4: Create Assessment Rubric

**Question**: How do we measure if student reached target proficiency?

**Process**:
- Design rubric specific to proficiency level
- Create descriptors for each level (exceeds target, meets target, developing, not yet)
- Use Bloom's levels to structure criteria

**Output**:
```markdown
## B1-Level Specification Writing Rubric

| Criterion | Exceeds (B2) | Meets (B1) | Developing (A2) | Not Yet (A1) |
|-----------|------------|-----------|-----------------|------------|
| **Clarity of Intent** | Explains problem+solution+why | States problem clearly | Problem stated vaguely | No clear problem statement |
| **Completeness** | All components + edge cases | All 5 core components | Missing 1-2 components | Missing 3+ components |
| **Testability** | All criteria measurable | Acceptance criteria testable | Some criteria unmeasurable | No acceptance criteria |
| **Real Context** | Applied to novel situation | Applied to familiar context | Applied to textbook scenario | Not applied to context |
| **Refinement** | Iterates based on feedback | Incorporates feedback | Resists feedback | Doesn't accept feedback |
```

#### Step 5: Design Level-Specific Assessments

**Question**: What kinds of assessments measure each proficiency level?

**Mapping**:
- **A1**: Recognition, definition, identification (multiple choice, matching)
- **A2**: Explanation, simple application (short answer, guided problem)
- **B1**: Application to real problems (case study, design, analysis)
- **B2**: Evaluation, tradeoff analysis (critique, comparison)
- **C1+**: Creation, innovation (novel design, thought leadership)

**Output**:
```markdown
## Assessment Methods by Proficiency Level

### For A1 (Foundation):
- [ ] Multiple choice: Identify definition of "Specification"
- [ ] Matching: Match spec components to definitions
- [ ] Fill-in-blank: "A spec describes [___] in [___] way"

### For A2 (Basic):
- [ ] Short answer: "What 5 components should every spec include?"
- [ ] Guided task: Write 1-paragraph spec for given problem
- [ ] Explanation: Why does accepting criteria matter?

### For B1 (Intermediate):
- [ ] Real problem: Write complete spec for capstone/work project
- [ ] Analysis: Compare two specs; identify gaps
- [ ] Design: Sketch a specification process for your team

### For B2+ (Advanced):
- [ ] Critique: Evaluate spec quality; propose improvements
- [ ] Tradeoffs: Compare spec-first vs. spec-as-source; defend choice
- [ ] Innovation: Design novel spec component for specific domain
```

---

## Templates Provided

### Template 1: Skill-Mapping Template

```yaml
# Skill Mapping Template
skill_name: ""
category: "Technical|Conceptual|Soft"

proficiency_by_lesson:
  lesson_1:
    level: "A1"
    definition: "Student can recognize/understand..."
    learnable_indicator: "Student demonstrates by..."
    assessment_method: "Test item type..."

  lesson_2:
    level: "A2"
    definition: "Student can apply in simple contexts..."

  lesson_3:
    level: "B1"
    definition: "Student can apply to real problems..."

progression_notes: |
  How skill develops across lessons:
  - Cognitive load management?
  - Scaffolding appropriate?
  - Prerequisites satisfied?

interdependencies: |
  What other skills must be learned first?
  What skills depend on this one?
```

### Template 2: Proficiency Progression

```markdown
# Proficiency Progression: [SKILL] from [A1] to [B1]

## Current State (A1)
- Student can: [Recognition/Understanding level]
- Time to teach: 15 min
- Cognitive load: 0-2 new concepts

## Target State (B1)
- Student can: [Application to real problems]
- Time to teach: 90 min total
- Cognitive load: 3-5 new concepts

## Progression Steps

### Step 1: Foundation Review (15 min)
Learning objective: Review A1-level understanding
Cognitive load: 0 new concepts

### Step 2: Introduce Target Concepts (30 min)
Learning objectives: [List 3-5 B1-level objectives]
Cognitive load: [3-5 new concepts]

### Step 3: Guided Practice (30 min)
Scaffolding: [How is student supported?]

### Step 4: Independent Application (45 min)
Student demonstrates B1 proficiency through: [Task description]
```

### Template 3: Assessment Rubric

```markdown
# Assessment Rubric: [SKILL] at [PROFICIENCY LEVEL]

| Criterion | Exceeds | Meets | Developing | Not Yet |
|-----------|---------|-------|-----------|---------|
| [Criterion 1] | [C2] | [B1] | [A2] | [A1] |
| [Criterion 2] | [C2] | [B1] | [A2] | [A1] |

## Scoring Guide

### Exceeds (C1-C2):
- [Description of advanced proficiency]

### Meets (B1):
- [Description of target proficiency]
- Students at this level can independently...

### Developing (A2):
- [Description of basic proficiency]
- Students need support for...

### Not Yet (A1):
- [Description of foundation level]
- Students still need to...

## Leveled Assessment Questions

### A1-Level Assessment
- Question type: [Recognition/definition]
- Example: [Sample question]

### A2-Level Assessment
- Question type: [Simple application]
- Example: [Sample question]

### B1-Level Assessment
- Question type: [Real problem application]
- Example: [Sample question]

### B2+-Level Assessment
- Question type: [Analysis/evaluation/creation]
- Example: [Sample question]
```

---

## Research References

### CEFR Resources
- European Commission: CEFR Digital Companion (2020)
- Council of Europe: Common European Framework of Reference (2001, 2020)
- Usage: 40+ countries as official standard, 100+ countries unofficially

### Bloom's Taxonomy
- Anderson, L.W. & Krathwohl, D.R. (eds.) - "A Taxonomy for Learning, Teaching, and Assessing: A Revision of Bloom's Taxonomy of Educational Objectives" (2001)
- Usage: Most widely-adopted framework in education globally

### DigComp
- Carretero, Vuorikari & Punie - "DigComp 2.1: The Digital Competence Framework for Citizens" (2022)
- EU, OECD, UNESCO adoption

### Cognitive Load Theory
- Sweller, J. - "Cognitive Load During Problem Solving" (1988+)
- Paas & Sweller - "Cognitive Architecture and Instructional Design" (2014)

### Scaffolding & Worked Examples
- Renkl, A. - "Learning from worked examples in mathematics: Student and teacher perspectives" (2014)
- Wood, Bruner, Ross - "The Role of Tutoring in Problem Solving" (1976)

---

---

## NEW (v2.0): Skill Coherence Validation Framework

### Why Coherence Matters

**Problem**: In a 55-chapter book with 200+ lessons, skills can become fragmented across chapters. Without validation:
- Same skill named differently in different chapters (fragmentation)
- Skills appear at A2 without A1 prerequisites (broken progressions)
- Proficiency regresses (A2 → A1 later = incoherent)
- Skills never deepen (A1 in Ch1, never again = isolated)
- Dependencies aren't explicit (students don't understand why skill appears now)

**Solution**: Five validation tests that catch coherence issues BEFORE they accumulate.

### The Five Validation Tests

#### Test 1: Uniqueness
**Question**: Is this skill unique or am I duplicating an existing skill with a different name?

**How to Check**:
1. Open Master Skill Registry for your book
2. Search for skills with similar concepts in the same domain
3. Check if the concept is already defined

**Red Flags**:
- ❌ "Evaluating Career Relevance" vs "Assessing Personal Role Transition" — Same concept, different verbs
- ❌ Skill appears in registry with different name in different chapters
- ❌ Multiple skills with very similar proficiency paths

**Fix**: Use canonical name from registry; extend existing skill if deepening it

#### Test 2: Naming Convention & Semantic Clarity
**Question**: Is the skill name clear and distinct from similar skills? Can different people read the name and understand the same thing?

**How to Check**:
1. Does the skill use one of these verbs consistently?
   - **Recognizing** = Identifying patterns/signals (A1)
   - **Understanding** = Grasping concepts/frameworks (A1-A2)
   - **Assessing** = Determining current state/readiness (A2)
   - **Evaluating** = Judging quality/fitness for purpose (A2)
   - **Personalizing** = Adapting frameworks to individual context (A2)
   - **Prioritizing** = Making strategic choices among options (A2)

2. Is the skill name specific enough?
   - ✓ "Understanding AI Stack Architecture" (specific)
   - ❌ "Understanding X" (too generic)

3. Can two people read the name and understand the same concept?
   - ✓ Test: Write the name to another instructor; do they understand what students learn?

**Red Flags**:
- ❌ Same verb used 20+ times without semantic distinction
- ❌ Skill name so vague it could mean different things to different people
- ❌ Similar skills with different verbs that mean the same thing

**Fix**: Create verb clarity guide; rename for specificity; ensure "Evaluating X" is truly evaluation (judging quality), not simple application

#### Test 3: Proficiency Progression Validity
**Question**: If the skill appears in multiple chapters, does proficiency increase appropriately (A1→A2→B1), never regress?

**How to Check**:
1. Search for all appearances of the skill across chapters
2. Verify proficiency levels form a non-decreasing sequence

**Red Flags**:
- ❌ Ch2, L2 (A2) → Ch2, L3 (A1) = Regression
- ❌ Ch1, L1 (A1) → Ch3, L2 (A1) → Ch2, L5 (A2) = Out of order
- ❌ Skill appears at A2 but never appears at A1 (where was A1 foundation?)

**Rule**: Proficiency should stay same or increase. Never regress.

**Expected Patterns**:
- ✓ Ch1, L1 (A1) → Ch3, L2 (A1 or A2) = Good
- ✓ Ch2, L3 (A1) → Ch4, L5 (A2) → Part 2, Ch6, L2 (B1) = Good
- ❌ Ch2, L3 (A2) → Ch2, L4 (A1) = Bad (regress)

**Fix**: Reorder lessons or correct proficiency assignment

#### Test 4: Prerequisite Satisfaction
**Question**: Have all prerequisite skills been taught BEFORE this skill?

**How to Check**:
1. Find skill in Master Skill Registry
2. Look at Dependencies field
3. For each dependency:
   - Is it taught in an earlier chapter/lesson?
   - Can students access prerequisite before dependent skill?

**Red Flags**:
- ❌ A2 skill with no A1 prerequisite taught anywhere
- ❌ Skill appears in Ch3 but its prerequisite isn't taught until Ch4
- ❌ Skill lists prerequisite but prerequisite isn't actually in lessons

**Rule**: All prerequisites must be taught before skill introduces them.

**Example - Correct**:
```
Ch3, L2: "Understanding Competitive Layer Strategy" (A1)
  ↓ prerequisite for
Ch3, L3: "Recognizing Economic Scalability Patterns" (A2)
  ↓ prerequisite for
Ch3, L5: "Understanding Vertical Market Strategy" (A1)
```
Each skill's prerequisites are taught earlier. ✓

**Example - Incorrect**:
```
Ch3, L5: "Evaluating Market Risk & Timing" (A2)
Prerequisites (from registry):
  - "Recognizing Market Dynamics" (A1) — NOT TAUGHT ANYWHERE
```
Missing A1 prerequisite. ❌

**Fix**: Either add missing A1 prerequisite or document explicit prerequisite in YAML

#### Test 5: Skill Connectivity & Progression Track
**Question**: Does this skill connect to other skills, or is it isolated? Does it fit into a progression track?

**How to Check**:
1. Open Master Skill Registry
2. Look for the skill's progression track (Development, Business, Career, Systems)
3. Verify:
   - Skill appears in multiple chapters (not just one)
   - Later chapters reference/build on this skill
   - Skill supports development of other skills

**Red Flags**:
- ❌ Skill appears only in one lesson, never mentioned again
- ❌ Skill is A2 but no B1 or deeper version planned
- ❌ No other skills reference or build on this skill

**Acceptable Exceptions**:
- ✓ Context-setting skills (introduce topic, set stage for later) — appear once is OK
- ✓ Scaffold skills (temporary, support concept, not long-term skill) — appear once is OK
- ❌ Important domain skills (core to methodology) — should progress across chapters

**Example - Well Connected**:
```
Progression Track 2: Business Models & Strategy

"Recognizing AI Opportunity Timing" (A1) in Ch1, L1
  ↓ deepens in
"Understanding Competitive Layer Strategy" (A1) in Ch3, L2
  ↓ builds to
"Evaluating Market Risk & Timing" (A2) in Ch3, L5
  ↓ applies to
"Understanding Vertical Market Strategy" (A1) in Ch3, L5
```
Clear progression; each builds on previous; connected. ✓

**Example - Isolated Skill**:
```
"Recognizing CS Education Gaps" (A1) in Ch1, L8
[never appears again]
[no other skills reference it]
[doesn't build to B1 or deeper version]
```
Isolated; fine for context-setting, but important concepts should progress. ⚠️

**Fix**: Either integrate into progression track or acknowledge as context-setting

### Coherence Validation Workflow

**When to Run**: Before finalizing any chapter's skills metadata

**Process**:
1. Extract all skills from chapter
2. For each skill, run all 5 tests
3. Document results in coherence report
4. Fix issues before marking chapter complete
5. Update Master Skill Registry

**Output**: Coherence validation checklist showing:
- [ ] Test 1: Uniqueness ✓/❌
- [ ] Test 2: Naming convention ✓/❌
- [ ] Test 3: Proficiency progression ✓/❌
- [ ] Test 4: Prerequisites ✓/❌
- [ ] Test 5: Connectivity ✓/❌

**Automated Checks** (can be built into tools):
- Compare skill names against registry (Test 1)
- Verify proficiency levels in registry never regress (Test 3)
- Check prerequisites exist and are taught earlier (Test 4)
- Count skill appearances across chapters (Test 5)

---

## When to Activate This Skill

**Primary triggers** (use this skill when you need):
- Map skills to lessons with proficiency levels
- Validate proficiency progression is appropriate
- Design assessments for specific proficiency levels
- Create rubrics for proficiency-based grading
- Manage cognitive load across chapter/course
- **Validate skill coherence across chapters (NEW v2.0)**
- **Check for skill fragmentation and missing prerequisites (NEW v2.0)**
- **Ensure skills don't regress in proficiency (NEW v2.0)**
- Align to international competence standards

**Not appropriate for** (skills better suited elsewhere):
- General lesson planning (use chapter-planner)
- Code example creation (use code-example-generator)
- Exercise design (use exercise-designer)
- Technical clarity review (use technical-clarity)

---

## Output Examples

### Example 1: Skills for Chapter 30, Lesson 1

```yaml
# Chapter 30, Lesson 1: Vague Code Problem

skills:
  - name: "Problem Identification"
    proficiency: "A2"
    category: "Conceptual"
    definition: "Student can identify vagueness and ambiguity in requirements"
    assessed_by: "Student identifies 3+ types of vagueness in example specs"
    cognitive_load_new_concepts: 1

  - name: "AI Communication Clarity"
    proficiency: "A2"
    category: "Soft"
    definition: "Student can explain why precision matters when communicating with AI"
    assessed_by: "Student explains consequences of vague specs to AI"
    cognitive_load_new_concepts: 1

  - name: "Specification Understanding"
    proficiency: "A1"
    category: "Conceptual"
    definition: "Student can recognize what a specification is"
    assessed_by: "Student identifies 5 components in spec example"
    cognitive_load_new_concepts: 2

proficiency_progression:
  - from: "None"
    to: "A1"
    mechanism: "Introduction + recognition tasks"

  - from: "A1"
    to: "A2"
    mechanism: "Worked examples + identification practice"

cognitive_load_check: "Total new concepts = 4 (within A2 limit of 7) ✓"
```

---

**Version**: 2.0.0
**Last Updated**: November 1, 2025
**Research Foundation**: 40+ years CEFR, 70+ years Bloom's Taxonomy, 2022 DigComp 2.1
**New in v2.0**: Skill Coherence Validation Framework with 5 validation tests (Uniqueness, Naming, Progression, Prerequisites, Connectivity)
**Status**: Enhanced for book-scale coherence validation; ready for Part 2+ chapters and automated validation workflows
