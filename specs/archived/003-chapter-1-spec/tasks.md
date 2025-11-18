# Tasks: Chapter 1 — The AI Development Revolution

**Input**: Design documents from `specs/003-chapter-1-spec/`
- **spec.md**: Chapter requirements, user stories, success criteria
- **plan.md**: 5-lesson architecture with content outlines
- **context source**: `@context/02_chap1_spec/readme.md`

**Implementation Model**: chapter-writer subagent implements all 5 lessons based on plan.md. Human reviews after each lesson completes. Simple, focused task list.

**Organization**: Tasks grouped by lesson + integration/review phases.

---

## Phase 1: Setup (Directory & Structure)

- [X] T001 Create chapter directory: `book-source/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/`
- [X] T002 Create lessons subdirectory structure
- [X] T003 Create assets directory for examples and analogies

---

## Phase 2: Foundational Assets (Text Examples & Analogies)

**These support all 5 lessons**:

- [X] T004 Compile vibe coding example: "I built my email filter in one afternoon"
- [X] T005 Compile enterprise example: Legacy code migration 2x speedup, hiring acceleration
- [X] T006 Compile startup example: Cursor beating GitHub Copilot
- [X] T007 Create analogies file: Conductor, coach, architect (with brief explanations)

---

## Phase 3: Lesson 1 — The Moment We're In (3-4 min, 300-400 words)

**Goal**: Hook readers with urgent, compelling statement of disruption

- [X] T008 Write Lesson 1: "Software has disrupted every industry. Now software itself is being disrupted."
  - Hook: Provocative opening (50-100 words)
  - Proof: Fastest-growing startup sector in history (100-150 words)
  - Stakes: You're entering profession at pivotal moment (75-100 words)
  - Transition: "But here's what most people get wrong..." (50-75 words)
  - Total: 300-400 words
- [X] T009 Add reflection prompt: "Does this feel real to you?"
- [X] T010 Add learning outcome summary

**Checkpoint: Lesson 1 complete & ready for human review**

---

## Phase 4: Lesson 2 — The $3 Trillion Disruption (4-5 min, 400-500 words)

**Goal**: Ground disruption in economic reality

- [X] T011 Write Lesson 2: "The $3 trillion software development economy is being disrupted"
  - The number: $3 trillion annual value (50-75 words)
  - The calculation: 30M × $100k = $3T (75-100 words)
  - Scale perspective: France's GDP comparison (75-100 words)
  - Speed: Trillion-dollar markets shift structurally, not slowly (75-100 words)
  - Why it matters: This is real disruption, not hype (75-100 words)
  - Total: 400-500 words
- [X] T012 Embed text-based $3T scale explanation (with calculation)
- [X] T013 Add comprehension check: "Can you explain why $3T matters?"
- [X] T014 Add learning outcome summary

**Checkpoint: Lesson 2 complete & ready for human review**

---

## Phase 5: Lesson 3 — Your New Role: Agent Orchestrator (3-4 min, 400-500 words)

**Goal**: Clarity on who readers are becoming + why it makes them MORE valuable

- [X] T015 Write Lesson 3: "Your role is shifting from code writer to agent orchestrator"
  - The paradigm shift: Code writer → Orchestrator (75-100 words)
  - Dimension 1 - Specification Writer: Clear requirements (75-100 words)
  - Dimension 2 - System Architect: Design system topology (75-100 words)
  - Dimension 3 - Agent Director: Guide toward solutions (75-100 words)
  - Dimension 4 - Quality Arbiter: Evaluate and decide (75-100 words)
  - Why it matters: Judgment is irreplaceable (50-75 words)
  - Total: 400-500 words
- [X] T016 Embed table: Four dimensions with one-line descriptions
- [X] T017 Add reflection prompt: "Which dimension resonates with you?"
- [X] T018 Add learning outcome summary

**Checkpoint: Lesson 3 complete & ready for human review**

---

## Phase 6: Lesson 4 — Why This Is Your Moment (3-4 min, 300-400 words)

**Goal**: Opportunity framing + psychological reframing of anxiety

- [X] T019 Write Lesson 4: "This is the best time in history to build with AI"
  - Market expansion: Software production accelerating (75-100 words, use vibe coding + hiring examples)
  - Incumbent vulnerability: Even Microsoft faces startup competition (75-100 words, use Cursor example)
  - Barriers lowering: Easiest to build in 3-4 decades (50-75 words)
  - Your advantage: Understanding AI = seeing problems others miss (50-75 words)
  - Closing: "This is your moment" (50-75 words)
  - Total: 300-400 words
- [X] T020 Weave in enterprise and startup examples naturally
- [X] T021 Add sentiment check prompt: "Do you see this as opportunity or threat?"
- [X] T022 Add learning outcome summary

**Checkpoint: Lesson 4 complete & ready for human review**

---

## Phase 7: Lesson 5 — How You'll Learn (2-3 min, 300-400 words)

**Goal**: Set expectations for pedagogical model throughout book

- [X] T023 Write Lesson 5: "You'll work WITH agents, not FROM them"
  - The distinction: WITH vs. FROM (75-100 words)
  - What WITH means: You direct, evaluate, decide (75-100 words)
  - Your responsibility: Write specs, guide agents, make decisions (50-75 words)
  - How this book works: Next chapters teach tools and first program (50-75 words)
  - Closing: "You're about to become an agent orchestrator. Let's start." (50-75 words)
  - Total: 300-400 words
- [X] T024 Embed text-based WITH vs. FROM comparison (markdown table)
- [X] T025 Add understanding prompt: "What does 'working WITH agents' mean?"
- [X] T026 Add self-assessment: "Am I ready for Chapter 2?"
- [X] T027 Add learning outcome summary

**Checkpoint: Lesson 5 complete & ready for human review**

---

## Phase 8: Chapter Integration & Assessments

**Purpose**: Connect all 5 lessons and create chapter-level assessments

- [X] T028 Add chapter-level navigation: Links between lessons in intro/conclusion
- [X] T029 Create chapter README with:
  - Main learning outcomes (consolidate from 5 lessons)
  - How each lesson contributes to chapter goal
  - Connection to Chapter 2
  - Domain skills used
  - Success metrics summary
- [X] T030 Create chapter-level glossary: All terms defined
- [X] T031 Create chapter comprehension check (3 questions):
  1. "Why is the $3 trillion figure important?" (Bloom's: Understand)
  2. "What are the four dimensions of an agent orchestrator?" (Bloom's: Remember/Understand)
  3. "What does 'agent-native education' mean to you?" (Bloom's: Understand)
- [X] T032 Create chapter reflection exercise: "Identify one thing you currently do that's 'orchestration' work"
- [X] T033 Add accessibility review: Clear language, no jargon, analogies accessible

**Checkpoint: Chapter integration complete**

---

## Phase 9: Final Review & Polish

- [ ] T034 Human review of chapter flow and pacing (15-20 minute reading time)
- [ ] T035 Verify all claims grounded in context document
- [ ] T036 Check that all domain skills are visibly applied
- [ ] T037 Verify Constitution alignment (Principles 1, 5, 8, 9)
- [ ] T038 Final proofreading and clarity pass
- [ ] T039 Create chapter summary: Word count, reading time, lessons breakdown

**Final Checkpoint: Chapter 1 ready for publication**

---

## Task Summary

**Total Tasks**: 39
**Phases**: 3 (Setup + Lessons + Integration + Review)
**Recommended Workflow**:
1. Run Setup (T001-T003) + Foundational Assets (T004-T007) first
2. Invoke chapter-writer subagent for Lessons 1-5 (T008-T027)
3. Human review after each lesson before proceeding to next
4. Complete Integration (T028-T033) once all lessons approved
5. Final Review (T034-T039)

**Human Review Checkpoints**: 5 (after each lesson) + 1 final

---

## Lesson-Writer Agent Assignments

**Each lesson should be assigned to content-implementer agent with specific guidelines below.**

### Lesson 1 Assignment (T008-T010)

**Agent**: content-implementer
**Input**: plan.md (Lesson 1 section)
**Output**: `01-lesson-1.md` in chapter directory
**Word Budget**: 300-400 words | **Time**: 3-4 minutes

**Guidelines for content-implementer**:
- Hook readers immediately with provocative statement
- Show proof first (fastest-growing startup sector exists NOW)
- Make stakes personal ("You're entering profession at pivotal moment")
- Use emotional engagement, not just information
- Transition to next lesson naturally ("But here's what most people get wrong...")
- Ground every claim in context document — no speculation
- Add reflection prompt at end: "Does this feel real to you?"
- Add learning outcome summary

**Success Criteria**: Reader feels urgent invitation to continue, not overwhelmed

---

### Lesson 2 Assignment (T011-T014)

**Agent**: content-implementer
**Input**: plan.md (Lesson 2 section) + context document (sections 2.1)
**Output**: `02-lesson-2.md` in chapter directory
**Word Budget**: 400-500 words | **Time**: 4-5 minutes

**Guidelines for content-implementer**:
- Present $3T number, let it land
- Make concrete: 30M developers × $100k = $3T (show calculation)
- Use France's GDP comparison for scale perspective (relatable)
- Emphasize SPEED of market shift (months not decades)
- Cite directly from context document (quotations welcome)
- Include text-based $3T scale explanation with visual calculation
- Add comprehension check: "Can you explain why $3T matters?"
- Add learning outcome summary

**Success Criteria**: Reader understands this is real disruption grounded in economics, not hype

---

### Lesson 3 Assignment (T015-T018)

**Agent**: content-implementer
**Input**: plan.md (Lesson 3 section) + plan.md (Four Dimensions table)
**Output**: `03-lesson-3.md` in chapter directory
**Word Budget**: 400-500 words | **Time**: 3-4 minutes

**Guidelines for content-implementer**:
- Frame as shift FROM code writer (execution) TO orchestrator (direction + judgment)
- Explain each dimension clearly:
  - Specification Writer: Clear requirements
  - System Architect: System design
  - Agent Director: Guiding iteration
  - Quality Arbiter: Evaluation + decisions
- Validate these are skills readers already have (just applied differently)
- Emphasize: "Syntax changes; judgment endures"
- Create table showing four dimensions with one-line descriptions
- Add reflection prompt: "Which dimension resonates with you?"
- Add learning outcome summary

**Success Criteria**: Reader sees themselves in this role and recognizes existing skills are MORE valuable

---

### Lesson 4 Assignment (T019-T022)

**Agent**: content-implementer
**Input**: plan.md (Lesson 4 section) + foundational assets (examples)
**Output**: `04-lesson-4.md` in chapter directory
**Word Budget**: 300-400 words | **Time**: 3-4 minutes

**Guidelines for content-implementer**:
- Counter displacement anxiety with evidence (not just reassurance)
- Show market expansion: vibe coding boom + enterprise hiring surge
- Include real examples naturally (vibe coding, Cursor vs. GitHub Copilot, hiring data)
- Explain incumbent struggle (Microsoft faces startup competition)
- Emphasize barriers lowering (easiest to build in 3-4 decades)
- Personal agency: "Understanding AI capabilities = seeing problems others miss"
- Create urgency (productive, not panic-inducing)
- Add sentiment check: "Do you see this as opportunity or threat?"
- Add learning outcome summary

**Success Criteria**: Reader shifts from fear → hope and sees this as personal opportunity

---

### Lesson 5 Assignment (T023-T027)

**Agent**: content-implementer
**Input**: plan.md (Lesson 5 section) + spec.md (pedagogical approach section)
**Output**: `05-lesson-5.md` in chapter directory
**Word Budget**: 300-400 words | **Time**: 2-3 minutes

**Guidelines for content-implementer**:
- Clarify distinction: Learning WITH agents (you direct) vs. FROM agents (you consume)
- Explain what WITH means: You write specs, direct agents, evaluate, decide
- Emphasize reader's ACTIVE role (empower, don't diminish)
- Show connection: Clear specs → Better agent output
- Explain how this book works: Chapter 2 teaches tools, Chapter 3 first program
- Close with momentum: "You're about to become an agent orchestrator. Let's start."
- Create markdown table: WITH vs. FROM comparison
- Add understanding check: "What does 'working WITH agents' mean?"
- Add self-assessment: "Am I ready for Chapter 2?"
- Add learning outcome summary

**Success Criteria**: Reader understands pedagogy, feels empowered, ready to proceed to Chapter 2

---

## Implementation Notes

**For content-implementer subagent**:
- Follow lesson outlines exactly (word counts, time budgets)
- Ground ALL claims in context document (no speculation, cite sources)
- Use Show-Then-Explain: examples first → then concepts
- Define every term on first use (heavy scaffolding)
- Use analogies naturally (Conductor, Coach, Architect — see plan.md)
- Heavy scaffolding for diverse audiences (no gatekeeping)
- Respect 15-20 minute total reading time (don't overwhelm)
- Format with Markdown: headers, bold, examples clearly distinguished
- Include YAML frontmatter with lesson metadata

**YAML Frontmatter Template**:
```yaml
---
sidebar_position: [lesson_number]
title: "[Lesson Title]"
duration: "[X-Y minutes]"
learning_objective: "[Bloom's aligned objective]"
---
```

**For human reviewer**:
- After each lesson completes: Review for clarity, accuracy, engagement, pedagogy, grounding
- Ask: "Does this hook readers and make them want the next lesson?"
- Check: All claims grounded in context document
- Verify: Word count, reading time realistic
- Assess: Domain skills visibly applied
- Approve or request revisions before proceeding to next lesson

