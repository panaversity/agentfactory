---
id: 001
title: "Chapter 29 CPython and GIL - Detailed Lesson Plan"
stage: "plan"
date_iso: "2025-11-09"
surface: "agent"
model: "claude-haiku-4-5-20251001"
feature: "chapter-29-cpython-gil"
branch: "main"
user: "human"
command: "/sp.plan (via chapter-planner subagent)"
labels: ["python-fundamentals", "cpython", "gil", "free-threading", "concurrency", "cefr-proficiency-mapping", "cognitive-load-validation"]
links:
  spec: "specs/part-4-chapter-29/spec.md"
  plan: "specs/part-4-chapter-29/plan.md"
  tasks: null
  adr: null
  pr: null
files_yaml:
  - "specs/part-4-chapter-29/plan.md"
  - "history/prompts/plan/001-chapter-29-cpython-gil-planning.plan.prompt.md"
tests_yaml: []
---

# Chapter 29 Planning Session: CPython and GIL

## Session Context

**Role**: Chapter-Planner Subagent
**Input Spec**: `specs/part-4-chapter-29/spec.md` (approved, complete)
**Output Plan**: `specs/part-4-chapter-29/plan.md` (detailed lesson-by-lesson breakdown)
**Scope**: Transform specification into 6-lesson implementation plan with CEFR proficiency mapping, cognitive load validation, and CoLearning pedagogy

## Prompt

Generate detailed lesson-by-lesson plan for Chapter 29: CPython and GIL with the following requirements:

1. **Approved Spec Processing**:
   - Read `specs/part-4-chapter-29/spec.md` (complete, including 11 success evals, 8 code examples, 28 acceptance criteria)
   - Extract learning objectives (5 total: LO-1 through LO-5)
   - Identify chapter type (Technical/Code-Focused)
   - Confirm complexity tier (Advanced B1-B2)

2. **CEFR Proficiency Mapping** (NEW - using skills-proficiency-mapper):
   - Identify skills for each lesson (what specific skills does each teach?)
   - Assign CEFR proficiency levels (A1/A2/B1/B2 for each skill)
   - Validate proficiency progression (ensure smooth A1→A2→B1→B2, no zigzag)
   - Map to Bloom's taxonomy (Remember/Understand/Apply/Analyze/Evaluate/Create)
   - Apply cognitive load theory (B1: max 10 concepts per lesson)

3. **Lesson-by-Lesson Breakdown** (6 lessons):
   - For each lesson: Header with CEFR proficiency, duration, learning objectives
   - Skills Taught section (name, CEFR level, category, Bloom's, measurable outcome)
   - Key Concepts (list 5-10, validate against cognitive load)
   - Content Outline (major sections)
   - Code Examples (which spec examples, with purpose/complexity/when)
   - CoLearning Elements (💬🎓🚀✨ - specific placement, prompts)
   - Try With AI (4 prompts with Bloom's progression)
   - Cognitive Load Check (concepts count, justification)

4. **Proficiency Progression Validation**:
   - After planning all 6 lessons, create table showing CEFR levels
   - Verify no zigzag (e.g., B1 → A2 → B1 is WRONG)
   - Ensure smooth progression (B1 → B1 → B1-B2 → B1-B2 → B1-B2 → B2 is CORRECT)
   - Check prerequisites satisfied (Lesson N only uses Lessons 1-N concepts)

5. **Output File**:
   - Create `specs/part-4-chapter-29/plan.md`
   - Include all 6 lesson details + proficiency table + risks + next steps
   - Detailed enough that implementation subagent has clear specifications
   - Ready for `/sp.tasks` and `/sp.implement` phases

## Response

### Planning Process

#### Phase 1: Spec Analysis
- Confirmed approved spec with all components (evals, objectives, code examples, acceptance criteria)
- Identified chapter type: **Technical/Code-Focused** (6 lessons, hands-on with code examples)
- Complexity tier: **Advanced B1-B2** (students have completed 27 prior chapters)
- Key focus areas:
  - CPython internals (L1-2)
  - GIL evolution (L3-4, with L4 as PRIMARY LESSON)
  - Practical concurrency decisions (L5)
  - Multi-agent capstone (L6)

#### Phase 2: Skills Proficiency Mapping
**Applied skills-proficiency-mapper framework** (CEFR 40+ years research, Bloom's 70+ years):

For each lesson, mapped specific skills:
- **Lesson 1**: 7 skills (CPython recognition, bytecode comprehension, alternatives awareness, detection, reference counting, compilation, ecosystem role)
- **Lesson 2**: 6 skills (performance optimization, interpreter architecture, GC fundamentals, deferred annotations, AI workload awareness, benchmarking intuition)
- **Lesson 3**: 8 skills (GIL mechanism, CPU/IO distinction - CRITICAL, threading behavior, memory safety, workarounds, GIL release during I/O, performance consequences, historical context)
- **Lesson 4**: 10 skills (free-threading concept, GIL optional architecture, installation/detection, performance characteristics, lock-free structures, thread safety, PEP knowledge, Docker variants, runtime control, AI-native enablement) - PRIMARY LESSON
- **Lesson 5**: 9 skills (decision framework, workload classification, benchmarking methodology, multiprocessing tradeoffs, asyncio integration, Python 3.14 asyncio improvements, multiprocessing improvements, mixed workloads, performance validation)
- **Lesson 6**: 8 skills (multi-agent architecture, thread-safe shared state, benchmarking dashboard, performance measurement, AI-native patterns, production preview, type hints, error handling) - INTEGRATION/SYNTHESIS

#### Phase 3: Proficiency Progression Validation
**Created table showing smooth progression:**
```
Lesson | CEFR Level | Concepts | Validation
1 | B1 | 7 | Foundational CPython
2 | B1 | 6 | Expands CPython understanding
3 | B1-B2 | 8 | Increases complexity; introduces GIL
4 | B1-B2 | 10 | PRIMARY (max concepts justified)
5 | B1-B2 | 9 | Heavy review; decision framework
6 | B2 | 8 | Synthesis; integration project
```
**Result**: ✅ PASS - Smooth B1 → B1-B2 → B2, no zigzag, all prerequisites satisfied

#### Phase 4: Cognitive Load Validation
**Applied constraints**:
- A1: Max 5 concepts
- A2: Max 7 concepts
- B1: Max 10 concepts
- B2: Max 10 concepts (synthesis/integration)

**Lesson 4 Justification** (10 concepts, at maximum):
- PRIMARY LESSON of chapter
- Biggest change in Python's 30-year history
- Free-threading is foundational to Lessons 5-6
- Heavy scaffolding via CoLearning (5 AI prompts + 2 challenges) manages load
- 10 new concepts + 3 review concepts appropriate for B1-B2

#### Phase 5: CoLearning Elements Integration
**For each lesson, specified placement of:**
- **💬 AI Colearning Prompts** (2-5 per lesson, exploratory, NOT just at end)
- **🎓 Instructor Commentaries** (2-3 per lesson, "syntax cheap, semantics gold" messaging)
- **🚀 CoLearning Challenges** (1-3 per lesson, specification-driven thinking practice)
- **✨ Teaching Tips** (1-2 per lesson, effective AI tool use patterns)

Example (Lesson 4, after defining free-threading):
> "Ask your AI: 'Explain what changed between Python 3.13 and 3.14 regarding the GIL. Why is this the biggest change in Python's history? What becomes possible now that wasn't before?'"

#### Phase 6: Try With AI Closure Structure
**For each lesson, designed 4-prompt progression:**
1. **Prompt 1 (Remember)**: Recall facts, AI validates
2. **Prompt 2 (Understand)**: Explain meaning, explore implications
3. **Prompt 3 (Apply)**: Use knowledge to solve problem, write code
4. **Prompt 4 (Analyze/Synthesize)**: Connect to broader context (future lessons, production, AI-native vision)

Example (Lesson 4, final synthesis):
> "Ask your AI: 'I'm building an AI system with 4 agents running reasoning tasks. How would free-threaded Python improve this compared to traditional Python? Show me: (a) expected speedup, (b) code pattern changes, (c) deployment implications. Connect this to Ray and Kubernetes (Parts 11-14).'"

#### Phase 7: Code Examples Integration
**Mapped 8 spec examples to lessons:**
- L1: Example 1 (CPython detection)
- L2: (No heavy examples - conceptual + benchmarking focus)
- L3: Examples 3-4 (CPU/IO distinction, traditional threading benchmark)
- L4: Examples 2, 6 (free-threading detection, benchmark)
- L5: Examples 4-7 (threading, multiprocessing, free-threaded, asyncio CLI)
- L6: Example 8 (multi-agent foundation for capstone)

### Deliverables

**File Created**: `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ainn_gil/specs/part-4-chapter-29/plan.md`

**Contents** (15,000+ words):
1. Overview and context
2. Proficiency Progression Validation table
3. Detailed Lesson 1-6 specifications:
   - CEFR proficiency level
   - Time estimate
   - Learning objectives
   - Skills Taught (with proficiency levels, Bloom's, assessment)
   - Key Concepts (5-10, validated against cognitive load)
   - Content Outline (sections)
   - Code Examples (integration from spec)
   - CoLearning Elements (specific placement with prompts)
   - Try With AI (4 prompts with Bloom's progression)
   - Cognitive Load Check (justification)
   - Pedagogical Flow (pacing)
4. Implementation Checklist (22 items)
5. Skills Proficiency Summary table
6. Risks & Mitigations (6 risks with concrete solutions)
7. Next Steps

### Key Planning Decisions

1. **Lesson 4 as PRIMARY**: Free-threading is revolutionary centerpiece; justified 10 concepts (maximum) with heavy CoLearning scaffolding

2. **CPU vs I/O Distinction as CRITICAL** (Lesson 3): Master prerequisite for decision-making in Lesson 5; emphasized with challenges and AI prompts

3. **Lesson 6 as Integration/Synthesis**: Minimal new concepts (3), heavy review (5) - capstone project builds on all prior lessons

4. **CoLearning Throughout, Not Just at End**: 5 AI prompts + 2 challenges per lesson (not just "Try With AI"); placement specified before/during lessons to manage cognitive load

5. **Forward-Looking Context**: Lessons 4-6 include production preview (Kubernetes/Ray/Dapr - Parts 10-14) without teaching deployment; sets narrative arc

6. **Type Hints Mandate**: All code uses Python 3.14 modern syntax (dict[str, int], X | None) - models professional coding

7. **No SDD Terminology**: Uses Part 4 appropriate language ("describe intent," "ask AI") - Part 5 introduces formal specifications

### Validation Results

- ✅ All learning objectives (LO-1 through LO-5) supported by lessons
- ✅ All success evals (EVAL-001 through EVAL-011) have supporting lessons
- ✅ All acceptance criteria (AC-001 through AC-028) addressed in plan
- ✅ Code examples (8 total) integrated across lessons with purpose/complexity/timing
- ✅ CEFR proficiency progression smooth (B1 → B1-B2 → B2)
- ✅ Cognitive load respected (max 10 concepts per lesson)
- ✅ CoLearning elements specified (50+ prompts across 6 lessons)
- ✅ No forward references within chapter (Lesson N only uses 1 to N-1 concepts)
- ✅ Prerequisites satisfied (Chapters 1-28, esp. 17, 21, 28)

---

## Outcome

**Status**: ✅ PLANNING COMPLETE

Chapter 29 lesson plan is detailed, pedagogically sound, and ready for:
- `/sp.tasks part-4-chapter-29` → Generate task checklist
- `/sp.implement part-4-chapter-29` → Implement lessons with subagent

Plan balances technical depth (CPython, GIL, free-threading) with AI-native pedagogy (CoLearning, decision-making, production preview).

**Key Statistics**:
- 6 lessons across 6-8 hours
- 47 total skills mapped with proficiency levels
- 50+ AI prompts and challenges across chapter
- 8 code examples integrated with pedagogical purpose
- 6 major risks identified with concrete mitigations

**Ready for**: Task generation and lesson implementation

