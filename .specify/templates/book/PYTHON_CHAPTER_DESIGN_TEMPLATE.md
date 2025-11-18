# Python Chapter Design Template: AIDD Applied to Programming

**Purpose**: Framework for designing Python chapters (12-29) by teaching programming through AIDD principles learned in Part 1-2.

**Foundation**: Students already know AIDD concepts (Part 1-3, Chapters 1-11). Python chapters teach **how to apply AIDD thinking to real code**—specification-first problem-solving, validation-first testing, AI partnership for implementation.

**NOT covered here**: Formal Spec-Driven Development (SDD)—that comes later in Part 6, Chapter 30-33. We're using AIDD mindset, not SDD formality.

---

## CRITICAL DESIGN RULES (FIXED)

### Rule 1: USER INTENT IS AUTHORITY

**Never override user input:**
- User says "beginner" → Make A1-A2 (NOT A2-B1)
- User says "just variables" → Only variables (NOT + functions + loops)
- User says "5 concepts max" → Exactly 5 (NOT "5 + bonus")

**Always ask, always honor. Do NOT assume.**

---

### Rule 2: NO FORWARD REFERENCES (SDD Only — AIDD Already Taught)

**Never mention untaught concepts (SDD onwards):**
- ❌ NO Chapter 30+ references
- ❌ NO "Spec-Driven Development" (not yet taught)
- ❌ NO methodology beyond AIDD

**DO reference AIDD (Ch 1-11, already taught):**
- ✅ "Apply the specification-first thinking from Chapter 4..."
- ✅ "Recall the nine pillars of AIDD from Chapter 4..."
- ✅ "Use validation-first thinking when testing your code..."

**Students learned AIDD in Part 1-2.** Python chapters should reinforce it by applying AIDD concepts to actual programming.

---

### Rule 3: MINIMAL SCOPE

**Depth > breadth.**

- Beginner chapters: 5 concepts max, 3 lessons
- Intermediate chapters: 7 concepts max, 4-5 lessons
- Advanced chapters: 10 concepts max, 5-6 lessons

**If user asks for "just variables," create JUST variables.** Don't add "but you should also know about loops."

---

### Rule 4: MINIMAL FILES

**Create ONLY:**
- ✅ spec.md
- ✅ plan.md
- ✅ tasks.md

**Never create:**
- ❌ index.md
- ❌ _templates/ directory
- ❌ _assets/ directory
- ❌ _code-examples/ directory
- ❌ lesson-template.md
- ❌ capstone-rubric.md

---

### Rule 5: SIMPLE TEACHING STRUCTURE

**Every lesson (DO NOT force 5-part structure):**

1. **What it is** - Plain explanation (2-3 sentences)
2. **Code Idea** - Minimal example showing concept
3. **Try It** - Student builds something with concept
4. **Why It Matters** - Real-world connection

**No philosophy lectures. No methodology references. Just clear teaching.**

---

## Chapter Design Workflow: Three Sources of Intelligence

### How Context, User Intent, and Python Expertise Work Together

**Priority Order:**
1. **Chapter Title** (anchor) — defines what we're teaching
2. **User Intent** (audience/scope/outcome) — must be honored completely
3. **Context** (if provided) — extract relevant pedagogical insights
4. **Python Docs** (authoritative) — verify syntax and best practices
5. **AI Intelligence** (pedagogical design) — create optimal learning approach

---

### Phase 0: Context Gathering (ASK USER - HONOR ANSWERS)

**Extract from arguments:**
- Chapter number (12-29)
- Chapter title (from chapter-index.md) — **this is our anchor**
- Context (if provided: teaching materials, code examples, existing pedagogical work)

**If context is provided:**
1. Read the chapter title FIRST
2. Extract pedagogical insights that fit the title
3. Note useful code examples and patterns
4. Understand learning outcomes already identified
5. Check prerequisites and connections
6. **Ignore what doesn't fit the chapter scope**

**ASK USER (honor their answers completely — never override):**

1. **Target audience?** (pick ONE)
   - Absolute beginner (no coding experience)
   - Beginner (some coding)
   - Intermediate (comfortable with basics)

2. **Core focus for this chapter?** (pick ONE - don't add extras)
   - Just variables and basic types
   - Variables + basic operations
   - Variables, types, and simple functions
   - [From context, or describe your focus]

3. **What can students BUILD after?** (testable outcome)
   - Store and display name/age
   - Simple calculation script
   - Function that processes data
   - [From context, or your specific project]

4. **Additional context to incorporate?**
   - Existing lesson drafts or notes
   - Code examples you want to use
   - Specific Python 3.13+ features
   - Real-world projects students should build

**CRITICAL WORKFLOW**:
- Chapter title is the anchor (never deviate)
- User's stated intent is authoritative (always honor)
- Context provides relevant insights (extract, don't override)
- Python docs ensure accuracy (verify with official sources)
- AI intelligence creates pedagogy (design optimal learning)

**Do NOT**: Let context override user's explicit answers. Do NOT assume intermediate when user said beginner. Do NOT add concepts beyond user's stated scope.

---

## Using Three Sources of Intelligence

### 1. Python Official Docs (Authoritative)
- Verify Python 3.13+ syntax and features
- Check official best practices for language features
- Understand how the language actually works
- Never teach outdated or non-standard approaches

### 2. AI Intelligence (Pedagogical Design)
- Create explanations suitable for target audience
- Find real-world examples and use cases
- Design progression from simple to complex
- Generate code examples that clearly demonstrate concepts
- Create exercises that build toward the final project
- Anticipate common misconceptions and address them

### 3. AIDD Thinking (Learning Model)
- **Specification-first**: Explain the problem before the code
- **Validation-first**: How students test their understanding
- **AI-partnership**: What they ask their AI to explore
- **Real outcomes**: What they actually build (not just exercises)

**Integration**: Python docs provide accuracy. AI intelligence provides pedagogy. AIDD thinking provides the learning model.

---

### Phase 1: Specification (NO FORWARD REFERENCES)

**Create `specs/part-5-chapter-${N}/spec.md` with this structure:**

```yaml
---
chapter: N
title: "[From chapter-index.md - EXACT]"
part: 5
target_audience: "[From user answer - EXACT, don't override]"
core_focus: "[From user answer - EXACT, don't add extras]"
complexity: [beginner|intermediate|advanced]
python_version: "3.13+"
---

## Success Criteria (From User Input)

### Students Can BUILD
[From user answer to "What can they build?"]

### Students Understand
[3-5 concepts ONLY - matching user's "core focus" answer]
[VALIDATE: Total concepts ≤ 5 for beginner, ≤7 for intermediate, ≤10 for advanced]

### NO FORWARD REFERENCES (SDD Only — AIDD Already Taught)
✅ Validate: Does this mention Chapters 30+ or "Spec-Driven Development"? NO
✅ Validate: Does this mention AIDD (Ch 1-11, already taught)? YES, ENCOURAGED when relevant
✅ Validate: Does this mention methodology not yet taught (beyond AIDD)? NO
**Note**: Python chapters can and should reference AIDD concepts (specification-first thinking, validation-first, etc.) since students learned them in Part 1-2. Example: "Apply the specification-first thinking from Chapter 4 when designing your functions..."

## Prerequisites
[Only chapters BEFORE this one]

## Learning Objectives
[3-5 SMART objectives using ONLY concepts taught in chapters 1-N]

## Content Structure

### Lesson 1: [Topic from core focus]
Duration: 15-20 min (beginner), 25-30 min (intermediate)
Concepts: [1-2 from list above]
Code examples: [1-2 simple]
Student activity: [Build something with Concept 1]

### Lesson 2: [Topic from core focus]
[Same structure]

### Lesson 3: [Topic from core focus - ONLY if in scope]
[Same structure - DON'T add lessons beyond user scope]

## TOTAL CONCEPTS TAUGHT
[Count them. Verify ≤ tier limit]

## Acceptance Criteria
- [ ] Scope matches user's "core focus" answer EXACTLY
- [ ] Target audience matches user answer (NOT overridden)
- [ ] Zero forward references (no Chapter 30+, no SDD, no methodology)
- [ ] Concept count ≤ tier limit (5/7/10)
- [ ] Each lesson 15-35 min
- [ ] Python 3.13+ syntax
- [ ] Students can actually build the promised project
```

**Key principle:**
- ❌ NEVER: Override user scope, add extra concepts, reference untaught methodology
- ✅ ALWAYS: Honor user answers, stay focused, self-contained chapter

---

### Phase 2: Planning (Lesson Breakdown)

**Create `specs/part-5-chapter-${N}/plan.md` with:**

```yaml
lessons:
  - lesson_number: 1
    title: [Title]
    duration: 20-25 minutes
    concepts: [list, with, count]
    cognitive_load: X / [5|7|10]

    skills_taught:
      - skill: [Name]
        proficiency_level: [A1|A2|B1|B2]
        bloom_level: [Remember|Understand|Apply|Analyze|Evaluate|Create]
        assessment: [How to measure]

    code_examples:
      - example_id: "1.1"
        purpose: [Pedagogical purpose]
        ai_prompt_template: "[Student asks AI...]"
        validation_criteria: [How to verify output]

    try_with_ai_activity:
      - specify: "[Student tells AI...]"
      - receive: "[AI generates...]"
      - validate: "[Student checks...]"
      - extend: "[Student deepens...]"
```

**Skills Metadata (Hidden from Students, Visible to Institutions):**
- CEFR proficiency levels (A1-C2)
- Bloom's cognitive complexity
- DigComp digital competence areas
- MEASURABLE at this level (what can student actually DO?)

**Progressive Complexity:**
- Lessons 1-2: A1 (recognition, basic application)
- Lessons 3-4: A2 (application with guidance)
- Lesson 5+: B1 (independent application to new problems)

---

### Phase 3: Tasks (Implementation Checklist)

**Create `specs/part-5-chapter-${N}/tasks.md`:**

```yaml
tasks:
  - task_id: "T001"
    content: "Create chapter directory with README"
    depends_on: []

  - task_id: "T002"
    content: "[US1] Implement Lesson 1: [Title] with 5-part structure"
    depends_on: ["T001"]
    acceptance_criteria:
      - [ ] Lesson follows: Concept → Code → Think With AI → Reasoning
      - [ ] YAML frontmatter with skills metadata
      - [ ] Code examples run in Python 3.13+
      - [ ] "Try With AI" activity specified
      - [ ] Cognitive load checked (≤5 concepts)

  - task_id: "T003"
    content: "Validate all AI prompts with Claude Code / Gemini CLI"
    depends_on: ["T002"]

  - task_id: "T00X"
    content: "Run validation-auditor validation"
    depends_on: [all lesson tasks]
```

---

## Lesson File Template

Each lesson follows this structure:

### YAML Frontmatter (Hidden Skills Metadata)

```yaml
---
chapter: N
lesson: L
title: "[Lesson Title]"
duration_minutes: 20-35

# HIDDEN SKILLS METADATA (For institutional accreditation)
skills:
  - name: "[Skill Name]"
    proficiency_level: "[A1|A2|B1|B2]"
    category: "[Technical|Conceptual|Soft]"
    bloom_level: "[Remember|Understand|Apply|Analyze|Evaluate|Create]"
    digcomp_area: "[Information Literacy|Communication & Collaboration|Digital Content Creation|etc]"
    measurable_at_this_level: "Student can... [specific, observable behavior]"

learning_objectives:
  - objective: "[SMART objective]"
    proficiency_level: "[A1|A2|B1]"
    bloom_level: "[...]"
    assessment_method: "..."

cognitive_load:
  new_concepts: X
  assessment: "X new concepts (list) within [limit] ✓"

differentiation:
  extension_for_advanced: "[For A2 students pushing toward B1]"
  remedial_for_struggling: "[For A1 students needing support]"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/part-5-chapter-${N}/spec.md"
version: "1.0.0"
---
```

### Content Structure

**Part A: Opening Hook**
```
"[Real-world scenario showing why this concept matters]"
```

**Part B: Concept Explanation**
```
## 1. [Concept Name] — [Why It Matters]

**What it is:**
[2-3 sentences, plain language]

[Philosophy/Why for AI development]

### 💻 Code Idea

\`\`\`python
# Minimal code showing concept
# Comments explain WHAT it does, not HOW syntax works
\`\`\`

### 🤖 Think With Your AI

> **Understand:** "Question that builds understanding"
>
> **Extend:** "Question that explores edge cases or variants"
>
> **Apply:** "Question linking to AI development"

### 🧠 The Reasoning Pattern

[Why this concept matters for AI reasoning, not just coding]
```

**Part C: "Try With AI" Activity**
```
## Try With Your AI Co-Teacher

**Activity Name**: [What you're building]

1. **Specify**: "Tell Claude Code: [Your specification]"
2. **Receive**: Claude generates code
3. **Validate**: Check output against intent
4. **Extend**: Modify/improve the code

[Alternative: If no code yet, dialogue-driven exploration]
```

**Part D: Checkpoint/Reflection**
```
## What You Learned

[Synthesize the concept]
[Connect to next lesson]
```

---

## Python Standards (Mandatory)

### Version & Syntax
- **Python 3.13+** (never 3.10 or earlier)
- f-strings ONLY (no % or .format())
- Modern type hints: `list[int]`, `dict[str, int]`, `X | None` (not `List`, `Dict`, `Optional`)
- match/case statements (Chapter 17+)
- Walrus operator `:=` (Chapter 17+)

### Type Hints Progression
```
Chapter 13 (Intro):      NO type hints (too early)
Chapter 14-16:           Show in examples, not required
Chapter 17-26:           Gradually increase
Chapter 27+ (Pydantic):  MANDATORY in all examples
```

### Security Non-Negotiables
- ❌ NO `eval()`, `exec()` on user input
- ❌ NO `shell=True` in subprocess
- ❌ NO hardcoded secrets/passwords
- ❌ NO SQL string concatenation
- ✅ Always: Environment variables, input validation, modern patterns

---

## Cognitive Load Management

### By Complexity Tier

| Tier | Chapters | CEFR | Concepts/Section | Examples | REPL? |
|------|----------|------|-----------------|----------|-------|
| **Beginner** | 12-16 | A1-A2 | 5 max | 1-2 per section | No; use "Ask AI" |
| **Intermediate** | 17-23 | A2-B1 | 7 max | 2-3 per section | No; use "Ask AI" |
| **Advanced** | 24-29 | B1-B2 | 10 max | 3-5 per section | No; use "Ask AI" |

### Counting Concepts

Each lesson has a **cognitive load budget**:
- 1 concept = core idea + vocabulary
- 5 concepts (beginner) = e.g., functions, parameters, return types, scope, recursion
- 7 concepts (intermediate) = e.g., above + closures, decorators, generators
- 10 concepts (advanced) = e.g., above + metaclasses, descriptors, context managers

**Formula**: New concepts this lesson < Tier limit

---

## Cross-Chapter Coherence

### Book Scaffolding

Every chapter references:
- **Prerequisites** (chapters that must come first)
- **Enables** (chapters that depend on this)
- **Concepts thread** (how concepts connect across chapters)

**Example for Chapter 13:**
```
Prerequisite: Chapter 12 (UV project structure)
Enables: Chapter 14 (Data Types), Chapter 17 (Control Flow)
Concept thread: Variables → Data Types → Control Flow → Functions
```

### Real-World Thread

Build ONE coherent project across all lessons:
- **Lesson 1**: Concept foundation
- **Lesson 2**: Mechanism understanding
- **Lesson 3-5**: Applied examples
- **Capstone**: Working, real project

**Example for Chapter 13:**
- Lesson 1: "Why Python for AI?"
- Lesson 2: "How Python executes" (bytecode)
- Lesson 3: "Modern syntax and types"
- Lesson 4: "Everything is an object" (OOP foundation)
- Lesson 5: "Duck typing" (flexibility)
- Lesson 6: "Dynamic + optional types" (balance)
- **Capstone**: Build an LLM Response Validator using all concepts

---

## Assessment & Validation

### Lesson-Level Assessment

Each "Try With AI" activity:
- ✅ **Specify**: Student expresses intent clearly
- ✅ **Receive**: AI generates code
- ✅ **Validate**: Student checks against intent
- ✅ **Extend**: Student modifies/improves

### Chapter-Level Validation

**Technical Reviewer Checklist:**
```
- [ ] All code examples run in Python 3.13+
- [ ] Type hints are syntactically correct
- [ ] No security vulnerabilities
- [ ] Each lesson cognitive load ≤ tier limit
- [ ] Reading level Grade 7+ with explanations
- [ ] Duration per lesson 20-35 minutes
- [ ] "Try With AI" activities testable
- [ ] Chapter coherence (thread + progression)
- [ ] Constitution alignment (spec-first, validation-first)
- [ ] Cross-references functional
```

---

## Implementation Orchestration

When you run `/sp.python-chapter ${N}`:

```
Step 1: Parse chapter number → Validate (12-29) → Get chapter title

Step 2: Ask context questions
   └─ Existing materials? (transform if yes)
   └─ Target audience? (shapes evals)
   └─ Real problems? (informs capstone)

Step 3: Auto-run /sp.specify
   └─ Create spec.md with evals-first approach
   └─ Apply Python 3.13+ standards
   └─ User approves spec

Step 4: Auto-run /sp.plan
   └─ Break into 5-6 lessons
   └─ Map skills (CEFR/Bloom's/DigComp)
   └─ Design "Try With AI" prompts
   └─ User approves plan

Step 5: Auto-run /sp.tasks
   └─ Generate implementation checklist
   └─ Include validation tasks
   └─ User approves tasks

Step 6: Deliver
   └─ specs/part-5-chapter-${N}/
      ├── spec.md (with evals)
      ├── plan.md (with skills metadata)
      └── tasks.md (with acceptance criteria)
   └─ Ready for content-implementer implementation
```

---

## Differences from Traditional Programming Books

| Traditional | AI-Native |
|-------------|-----------|
| "Learn syntax" | "Learn concepts" |
| "Memorize patterns" | "Think with AI partner" |
| "REPL first, validation" | "Specify intent, AI executes, validate" |
| "Read code examples" | "Ask AI to generate, observe, extend" |
| "Solo problem-solving" | "Collaborative reasoning" |
| "Beginner-unfriendly" | "Beginner-accessible (AI co-teacher)" |

---

## Quality Gates (Non-Negotiable)

Before publishing ANY chapter:

### Technical Quality
- ✅ Every code example tested (Python 3.13+)
- ✅ No security anti-patterns
- ✅ Type hints correct and helpful
- ✅ Cognitive load validated

### Pedagogical Quality
- ✅ Every concept → Code → Think With AI → Reasoning
- ✅ "Try With AI" activities specified (testable)
- ✅ Learning objectives measurable
- ✅ Assessment methods clear

### Constitution Alignment
- ✅ Spec-first approach (evals → spec → plan → tasks)
- ✅ Validation-first mindset (checks, not just code)
- ✅ AI-native philosophy (colearning, not memorization)
- ✅ Accessibility (Grade 7+ reading, explained terms)

---

## Usage Examples

### Example 1: Chapter 13 (Introduction to Modern Python)

```
$ /sp.python-chapter 13

→ Chapter 13: Introduction to Modern Python
→ Complexity: Beginner (A1-A2, 5 concepts/section)
→ Question 1: Existing materials? (← Reference old lesson, transform)
→ Question 2: Target audience? (← AI developers, career path)
→ Question 3: Real problems? (← Build LLM validator)

[Auto-run /sp.specify]
[Create: specs/part-5-chapter-13/spec.md with evals-first]

[User approves spec]

[Auto-run /sp.plan]
[Create: specs/part-5-chapter-13/plan.md with lesson breakdown + skills metadata]

[User approves plan]

[Auto-run /sp.tasks]
[Create: specs/part-5-chapter-13/tasks.md with implementation checklist]

[User approves tasks]

✅ Complete package ready for content-implementer subagent
```

### Example 2: Chapter 17 (Control Flow)

```
$ /sp.python-chapter 17

→ Chapter 17: Control Flow
→ Complexity: Intermediate (A2-B1, 7 concepts/section)
→ Concepts: if/elif/else, for loops, while loops, break/continue, match/case, comprehensions, generators
→ Capstone: Build a request router (if/elif → match/case; loops for batch processing)
→ AI-native frame: "How does an AI agent route requests? How does it iterate?"

[Full workflow with intermediate complexity limits]
```

---

## CRITICAL VALIDATION CHECKLIST

**Before finalizing spec.md, verify ALL of these:**

```
□ Target audience MATCHES user answer (no override)
□ Core focus MATCHES user answer exactly
□ Scope does NOT add beyond what user asked
□ No chapters 30+ mentioned anywhere
□ No "Spec-Driven Development" mentioned
□ AIDD references (Ch 1-11) used when relevant to reinforce prior learning
□ No methodology/pedagogy names mentioned (except AIDD for reinforcement)
□ Concept count COUNTED and ≤ tier limit (5/7/10)
□ Prerequisites ONLY chapters before this one (Ch 1-11 known; Ch 12+ if building on Python)
□ Learning objectives testable and realistic
□ Students CAN actually build promised project
□ Only 3 files will be created (spec/plan/tasks)
□ No index.md, templates, assets directories
```

**If ANY check fails → ASK USER FOR CLARIFICATION, don't assume.**

**Note on AIDD references**: Python chapters should reinforce AIDD principles by applying them. For example:
- "Let's apply the specification-first thinking from Chapter 4 to write function specifications"
- "Use the validation-first approach from AIDD when testing your code"
- "Remember the nine pillars of AIDD—they apply to functions too"

---

## FIXES IN THIS TEMPLATE

✅ **Problem 1: Part mismatch** - Validates chapter location from authoritative source
✅ **Problem 2: Forward references** - Zero mentions of Chapter 30+, SDD, methodology
✅ **Problem 3: Content overload** - Respects "core focus" answer, doesn't add extras
✅ **Problem 4: File clutter** - Creates ONLY spec/plan/tasks, no scaffolding
✅ **Problem 5: Audience mismatch** - Asks user, honors their answer, NO overrides
✅ **Problem 6: Self-referential methodology** - No circular dependencies, self-contained

---

## Final Note

**This template enforces:**
- User intent is authority (honor answers, no overrides)
- No forward references (self-contained chapters)
- Minimal scope (depth > breadth)
- Minimal files (3 only, no clutter)
- Simple teaching structure (no forced patterns)
- Critical validation (11-point checklist)

**Result**: Beginner-appropriate, focused, self-contained chapters without circular dependencies. Any AI orchestrator can design consistent, high-quality Python chapters using this framework.

