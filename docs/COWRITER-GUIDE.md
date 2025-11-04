# CoWriter Guide: Vertical Intelligence-Powered SDD Orchestration

**Version**: 1.0.0
**Last Updated**: 2025-11-04
**For**: Domain Experts & Content Collaborators
**Prerequisites**: Basic git knowledge, access to Claude Code or similar AI coding assistant

---

## Welcome, CoWriter! 👋

This guide shows you how to collaborate with AI orchestrators using the **Vertical Intelligence-Powered SDD (Spec-Driven Development) Orchestration Layer** to create high-quality educational content.

**What is Vertical Intelligence?**
A synchronized 4-layer system where every layer (constitution → output-styles → subagents → content) references the same authoritative sources, eliminating contradictions and ensuring consistent, high-quality outputs.

**What You'll Learn**:
- How to collaborate with AI as a co-reasoning partner (not just a code generator)
- How to use evals-first methodology for quality content
- How to leverage specialized subagents for planning, writing, and validation
- How to onboard domain experts for different book parts

---

## Table of Contents

1. [Quick Start: Your First Chapter](#quick-start-your-first-chapter)
2. [Understanding the SDD Workflow](#understanding-the-sdd-workflow)
3. [Working with the Orchestration Layer](#working-with-the-orchestration-layer)
4. [Onboarding Domain Experts](#onboarding-domain-experts)
5. [Chapter Generation Process](#chapter-generation-process)
6. [Quality Assurance with Validation](#quality-assurance-with-validation)
7. [Common Workflows & Examples](#common-workflows--examples)
8. [Troubleshooting & Best Practices](#troubleshooting--best-practices)
9. [Reference: Available Tools](#reference-available-tools)

---

## Quick Start: Your First Chapter

### Step 0: Verify Your Environment

```bash
# Check you're in the project root
pwd
# Should show: /path/to/tutorgpt-build/bbb

# Check current branch
git status
# Should show: On branch main

# Create feature branch
git checkout -b chapter-X-your-topic
```

### Step 1: Define Success First (Evals-First ✨ NEW)

**Before writing any spec**, define how you'll measure success:

```markdown
# Chapter X Success Evals

## Business Goals
- Students can [specific action] after this chapter
- 80%+ completion rate on "Try With AI" exercises
- 75%+ score on end-of-chapter quiz

## Measurable Outcomes
- Student can write valid [artifact] without template (quiz)
- Student can identify [concept] in real-world scenarios (test)
- Student can explain [principle] in their own words (assessment)

## Quality Metrics
- Technical accuracy: 100% (all code examples tested)
- Pedagogical effectiveness: 90%+ (learning objectives met)
- Constitution alignment: 100% (passes technical-reviewer)
```

**Why First?**
Professional AI-native development starts with evals (Anthropic/OpenAI standard). Evals connect to **business goals**, not arbitrary metrics like "10ms response time."

### Step 2: Create Chapter Specification

**Use the constitution as your guide**:

```bash
# Reference constitution for principles
cat .specify/memory/constitution.md

# Reference chapter-index for current structure
cat specs/book/chapter-index.md

# Reference output-styles for formatting
cat .claude/output-styles/chapters.md
cat .claude/output-styles/lesson.md
```

**Create your spec**:

```bash
mkdir -p specs/part-X-chapter-Y
touch specs/part-X-chapter-Y/spec.md
```

**Spec Template** (minimum required sections):

```markdown
# Chapter X: [Title]

## Success Evals (Phase 0.5)
[From Step 1 - copy your evals here]

## Learning Objectives
1. [Bloom's taxonomy verb] + [concept] + [context]
2. [Remember/Understand/Apply/Analyze/Evaluate/Create]

## Prerequisites
- Chapter [X]: [concept needed]
- Skills: [list any required skills]

## Content Outline
### Section 1: [Topic]
- Key concepts
- Code examples (if technical)

### Section 2: [Topic]
...

## Acceptance Criteria
- [ ] All learning objectives have corresponding content
- [ ] All code examples tested and working
- [ ] "Try With AI" section in every lesson
- [ ] Passes technical-reviewer validation
```

**Get Human Approval**: Show your spec to the domain expert before proceeding!

### Step 3: Generate Plan with chapter-planner

**Invoke the chapter-planner subagent**:

```bash
# Using slash command (if available)
/sp.plan

# Or manually invoke via your AI assistant:
# "Use chapter-planner subagent to create a detailed lesson plan
#  for specs/part-X-chapter-Y/spec.md"
```

**chapter-planner will**:
- Break spec into 3-7 lessons with clear learning objectives
- Map skills to CEFR proficiency levels (A1/A2/B1/B2/C1)
- Identify code examples needed
- Create task checklist with acceptance criteria
- Reference constitution v3.0.1 and output-styles automatically

**Output**: `specs/part-X-chapter-Y/plan.md`

**Review and approve** the plan before implementation!

### Step 4: Generate Tasks

```bash
# Using slash command (if available)
/sp.tasks

# Or manually invoke:
# "Use the plan to generate actionable tasks in tasks.md"
```

**Output**: `specs/part-X-chapter-Y/tasks.md`

### Step 5: Implement with lesson-writer

**Iterative lesson-by-lesson implementation**:

```markdown
# For each lesson in the plan:

1. Invoke lesson-writer: "Write Lesson 1 based on plan.md"
2. Review generated content
3. Validate with technical-reviewer
4. Approve or request revisions
5. Move to next lesson
```

**lesson-writer will**:
- Generate content matching output-styles templates
- Include 7 metadata fields in YAML frontmatter
- Add skills proficiency metadata (CEFR levels)
- Follow "Try With AI" closure policy
- Reference chapter-index.md for cross-references

**Output**: Markdown files in `book-source/docs/Part-X/chapter-Y/`

### Step 6: Validate with technical-reviewer

```bash
# Validate the entire chapter
# "Use technical-reviewer to validate chapter-Y for publication readiness"
```

**technical-reviewer checks**:
- Constitution v3.0.1 alignment (evals-first, spec-first)
- Output-styles compliance (structure, YAML metadata)
- Chapter-index.md accuracy (chapter number, title, status)
- Code quality (Python 3.13+, type hints, tested)
- Pedagogical effectiveness (learning objectives met)

**Output**: Validation report with APPROVE/REVISE/RETURN recommendation

### Step 7: Commit and Create PR

```bash
# Add all chapter files
git add book-source/docs/Part-X/chapter-Y/
git add specs/part-X-chapter-Y/

# Commit with descriptive message
git commit -m "feat: Add Chapter X - [Title]

- Implemented [N] lessons
- All code examples tested
- Passes technical-reviewer validation
- CEFR proficiency levels mapped

🤖 Generated with Claude Code
Co-Authored-By: Claude <noreply@anthropic.com>"

# Push and create PR
git push -u origin chapter-X-your-topic
gh pr create --title "Chapter X: [Title]" --body "..."
```

---

## Understanding the SDD Workflow

### The Evals → Spec → Plan → Implement → Validate Loop

```
Phase 0.5: Evals Definition ✨ NEW
    ↓
    Define success criteria (business-goal-aligned)
    ↓
Phase 1: Specification
    ↓
    Write chapter spec (approved by human)
    ↓
Phase 2: Planning
    ↓
    chapter-planner generates detailed lesson plan
    ↓
Phase 2.5: Tasks
    ↓
    Generate actionable task checklist
    ↓
Phase 3: Implementation
    ↓
    lesson-writer generates content (iterative, lesson-by-lesson)
    ↓
Phase 4: Validation
    ↓
    technical-reviewer validates quality
    ↓
Phase 5: Publication (human final review)
```

### Why This Order?

**Evals First** (NEW in constitution v3.0.1):
- Professional AI-native pattern (Anthropic/OpenAI standard)
- Prevents "we built it, but does it work?" problem
- Connects to business goals (employability, skill acquisition)

**Spec Before Code**:
- Prevents scope creep and wasted effort
- Enables AI to understand intent before execution
- Human approves direction before investment

**Plan Before Implement**:
- Breaks complex chapters into manageable lessons
- Identifies dependencies and skill progression
- Maps proficiency levels (CEFR) for competency-based learning

**Validate Before Publish**:
- Catches errors before they reach students
- Ensures constitution alignment
- Provides actionable feedback for improvement

---

## Working with the Orchestration Layer

### The 4 Layers of Vertical Intelligence

```
Layer 1: Constitution (.specify/memory/constitution.md)
    ↓ References
Layer 2: Output-Styles (.claude/output-styles/)
    ↓ References
Layer 3: Subagents (.claude/agents/)
    ↓ Generates
Layer 4: Content (book-source/docs/)
```

**Key Principle**: Every layer references the layers above it. Changes propagate downward automatically.

### Authoritative Sources (Single Source of Truth)

| What | Where | Who Uses It |
|------|-------|-------------|
| Current chapter status | `specs/book/chapter-index.md` | All subagents |
| File structure rules | `specs/book/directory-structure.md` | lesson-writer |
| Content formatting | `.claude/output-styles/chapters.md` | chapter-planner, lesson-writer |
| Lesson structure | `.claude/output-styles/lesson.md` | lesson-writer, technical-reviewer |
| Skills proficiency | `.claude/skills/skills-proficiency-mapper/` | chapter-planner |
| Project principles | `.specify/memory/constitution.md` (v3.0.1) | All subagents |

**What This Means for You**:
- Never hardcode chapter numbers or filenames in specs
- Reference chapter-index.md: "See chapter-index.md for current chapter list"
- Use relative references: "Prerequisites: See Chapter [X] (reference chapter-index.md)"
- Trust subagents to use correct structure (they reference output-styles)

### Available Subagents

#### 1. chapter-planner
**When to Use**: After spec approved, before implementation
**Input**: Approved chapter spec
**Output**: Detailed lesson-by-lesson plan, task checklist
**What It Does**:
- Breaks spec into 3-7 lessons
- Maps skills to CEFR proficiency levels
- Creates task checklist with acceptance criteria
- References constitution v3.0.1 and output-styles

**Invocation**:
```bash
/sp.plan
# Or: "Use chapter-planner to create lesson plan for [spec path]"
```

#### 2. lesson-writer
**When to Use**: After plan approved, during implementation
**Input**: Approved lesson plan, lesson number
**Output**: Complete lesson markdown with YAML frontmatter
**What It Does**:
- Generates content matching output-styles
- Includes 7 metadata fields in YAML
- Adds skills proficiency metadata
- Validates content against proficiency levels
- Ends with "Try With AI" section

**Invocation**:
```bash
# For each lesson
"Use lesson-writer to implement Lesson [N] based on plan.md"
```

#### 3. technical-reviewer
**When to Use**: After lesson/chapter complete, before publication
**Input**: Complete chapter directory
**Output**: Validation report with APPROVE/REVISE/RETURN
**What It Does**:
- Validates constitution v3.0.1 alignment
- Checks output-styles compliance
- Verifies chapter-index.md accuracy
- Tests code examples
- Assesses pedagogical effectiveness

**Invocation**:
```bash
"Use technical-reviewer to validate chapter-[X] for publication readiness"
```

---

## Onboarding Domain Experts

### Scenario: You Need a Domain Expert for Part 7 (Advanced Python)

**Your Role**: CoWriter/Orchestrator
**Expert Role**: Subject Matter Expert (SME)
**AI Role**: Co-reasoning partner and implementation engine

### Step 1: Share This Guide

**Send the expert**:
1. This guide (COWRITER-GUIDE.md)
2. Constitution (`.specify/memory/constitution.md`)
3. Part 7 overview from chapter-index.md
4. Example chapter from Part 5 (shows the end result)

**Key Points to Emphasize**:
- They define **WHAT** to teach (evals, learning objectives, concepts)
- AI handles **HOW** to structure (lesson plans, formatting, metadata)
- Iterative process: expert reviews and approves at each phase

### Step 2: Discovery Session (1-2 hours)

**Questions to Ask**:

**Business Goals**:
- What should students be able to DO after completing Part 7?
- How will we measure success? (completion rate, skill acquisition, employment outcomes)
- What skill level should they reach? (CEFR: A1/A2/B1/B2/C1)

**Prerequisite Knowledge**:
- What must students know before starting Part 7?
- Which chapters from Parts 1-6 are prerequisites?
- Any external knowledge required? (math, computer science concepts)

**Content Scope**:
- What topics MUST be covered? (core curriculum)
- What topics are OPTIONAL? (nice-to-have extensions)
- What topics are OUT OF SCOPE? (too advanced, wrong audience)

**Pedagogical Approach**:
- Conceptual understanding first, or hands-on first?
- Real-world projects, or isolated exercises?
- How much theory vs. practice? (ratio)

### Step 3: Define Success Evals Together

**Collaborate on evals document**:

```markdown
# Part 7: Advanced Python - Success Evals

## Business Goals
- Students can build production-ready Python applications
- 85%+ land junior Python developer roles within 3 months
- 80%+ complete all 8 chapters in Part 7

## Skill Acquisition (CEFR B2 Target)
- Can analyze complex code and identify design patterns
- Can evaluate multiple solutions and choose appropriate one
- Can create original applications solving real-world problems
- Can debug complex issues using systematic approaches

## Measurable Outcomes
- Quiz: 80%+ identify appropriate design patterns (10 scenarios)
- Project: Build REST API with auth, validation, error handling
- Code review: Provide constructive feedback on peer code (rubric)
- Portfolio: 3 completed projects demonstrating B2-level skills

## Quality Standards
- All code examples pass mypy strict mode (100%)
- All code examples have 80%+ test coverage
- All chapters pass technical-reviewer (constitution v3.0.1)
- All lessons end with "Try With AI" section
```

**Key**: Expert defines the evals; you validate they're measurable

### Step 4: First Chapter Walkthrough

**Choose one representative chapter** from Part 7 (e.g., Chapter 20: Decorators)

**Walk through the full SDD loop together**:

1. **Evals** (30 min): Define success criteria together
2. **Spec** (1 hour): Expert describes content, you write spec
3. **Review Spec** (30 min): Expert approves or requests changes
4. **Generate Plan** (AI, 10 min): Use chapter-planner subagent
5. **Review Plan** (30 min): Expert approves lesson structure
6. **Generate Lesson 1** (AI, 10 min): Use lesson-writer subagent
7. **Review Lesson 1** (30 min): Expert provides feedback
8. **Iterate** (AI + Expert): Refine until expert approves
9. **Validate** (AI, 10 min): Use technical-reviewer subagent
10. **Publish** (You): Commit, create PR, merge

**Total Time**: ~4 hours for first chapter (faster for subsequent chapters)

### Step 5: Establish Collaboration Patterns

**After first chapter, agree on**:

**Communication**:
- How often to sync? (daily standups, weekly reviews)
- Async feedback? (GitHub PR comments, shared doc)
- Escalation path? (what decisions require your input vs. expert handles)

**Review Process**:
- Expert reviews specs before planning? (recommended: YES)
- Expert reviews plans before implementation? (recommended: YES)
- Expert reviews each lesson? (recommended: YES for first 2 chapters, then spot-check)
- Expert performs final review? (recommended: ALWAYS)

**Approval Authority**:
- Expert can approve: content accuracy, pedagogical approach, examples
- You approve: formatting, metadata, constitutional alignment, publication timing

**Tool Access**:
- Does expert use Claude Code directly? (preferred)
- Or expert provides content, you orchestrate? (slower but viable)
- Git access? (recommended: expert has branch access, you review PRs)

### Step 6: Scale Up Production

**Once expert is comfortable** (after 2-3 chapters together):

**Parallel Work**:
- Expert works on specs for Chapters 21-23
- You orchestrate implementation for Chapter 20
- AI subagents handle formatting, metadata, validation

**Quality Gates**:
- Spec review: You + Expert (must approve before planning)
- Plan review: Expert spot-checks (trusts chapter-planner)
- Content review: Expert reviews final output (before validation)
- Validation: technical-reviewer (automated quality check)
- Publication: You merge after expert approval

**Velocity**:
- Week 1: 1 chapter (learning the process)
- Week 2: 2 chapters (parallel spec + implement)
- Week 3+: 3-4 chapters/week (expert on specs, you orchestrate, AI generates)

---

## Chapter Generation Process

### Detailed Workflow with Examples

#### Example: Chapter 15 - List Comprehensions

**Phase 0.5: Evals Definition**

```markdown
# Chapter 15 Success Evals

## Business Goals
- Students can use list comprehensions for data transformation tasks
- 90%+ complete "Try With AI" exercises
- 75%+ score on list comprehension quiz

## Measurable Outcomes
- Quiz: Transform 5 for-loops into list comprehensions (80%+ correct)
- Exercise: Filter and transform dataset with comprehensions (passes tests)
- Code review: Identify when list comp is better than for-loop (5/5 correct)

## Quality Standards
- All examples tested on Python 3.13+
- Type hints on all functions
- Covers simple, filtered, nested, and dict comprehensions
```

**Phase 1: Specification**

```markdown
# Chapter 15: List Comprehensions - Spec

## Success Evals
[Copy from Phase 0.5]

## Learning Objectives
1. Understand the syntax and purpose of list comprehensions (Understand)
2. Apply list comprehensions to transform and filter data (Apply)
3. Evaluate when list comprehensions improve code readability (Evaluate)
4. Create nested and dictionary comprehensions for complex data structures (Create)

## Prerequisites
- Chapter 12: Lists and Tuples (must complete)
- Chapter 13: For Loops and Iteration (must complete)

## Content Outline

### Lesson 1: Introduction to List Comprehensions
- What are list comprehensions? (syntax overview)
- Why use them? (readability, performance)
- Simple transformations: [x*2 for x in range(10)]

### Lesson 2: Filtering with Comprehensions
- Adding conditions: [x for x in data if x > 0]
- Multiple conditions
- Common use cases (filter, map combined)

### Lesson 3: Nested Comprehensions and Dict Comprehensions
- Nested lists: [[i*j for j in range(5)] for i in range(5)]
- Dict comprehensions: {k: v for k, v in pairs}
- When to avoid nesting (readability threshold)

### Lesson 4: Real-World Applications
- Data cleaning with comprehensions
- JSON transformation
- Performance comparison (comp vs for-loop)

## Code Examples
1. Simple transformation (list of squares)
2. Filtering even numbers
3. Nested comprehension (matrix operations)
4. Dict comprehension (key-value swapping)
5. Real dataset transformation (CSV processing)

## Acceptance Criteria
- [ ] All 5 code examples tested and working
- [ ] Each lesson has "Try With AI" section
- [ ] CEFR proficiency levels: L1=A2, L2=B1, L3=B1, L4=B2
- [ ] Passes technical-reviewer validation
```

**Human Approval**: Domain expert reviews and approves spec ✅

**Phase 2: Planning**

**You invoke**: `"Use chapter-planner to create lesson plan for specs/part-4-chapter-15/spec.md"`

**chapter-planner generates**: `specs/part-4-chapter-15/plan.md`

```markdown
# Chapter 15: List Comprehensions - Lesson Plan

## Lesson 1: Introduction to List Comprehensions
**Proficiency Level**: A2 (Beginner Intermediate)
**Duration**: 30 minutes

**Learning Objectives**:
- Recognize list comprehension syntax (Remember - A2)
- Understand when to use list comprehensions vs for-loops (Understand - A2)

**Key Topics**:
- Syntax: [expression for item in iterable]
- Equivalence to for-loops
- Readability benefits

**Code Examples**:
- Example 1: Squares [x**2 for x in range(10)]
- Example 2: String transformation [s.upper() for s in names]

**Try With AI**:
- Tool: Learner's AI companion (post-tool-onboarding)
- Prompts: 4 prompts transforming for-loops to comprehensions
- Expected: Student identifies which version is more readable

**Skills Metadata**:
- Skill: List Comprehension Recognition
- CEFR: A2
- Bloom's: Remember, Understand
- Measurable: Student can identify list comprehension pattern in code

[Lessons 2-4 follow similar structure]
```

**Human Approval**: Domain expert reviews lesson structure ✅

**Phase 2.5: Tasks**

**You invoke**: `/sp.tasks`

**Output**: `specs/part-4-chapter-15/tasks.md`

```markdown
# Tasks: Chapter 15 - List Comprehensions

- [ ] T001 Create chapter README.md with overview and learning outcomes
- [ ] T002 Write Lesson 1: Introduction (syntax, simple examples)
- [ ] T003 Create 2 code examples for Lesson 1 (test and validate)
- [ ] T004 Design "Try With AI" for Lesson 1 (4 prompts)
- [ ] T005 Write Lesson 2: Filtering (conditions, multiple filters)
...
```

**Phase 3: Implementation (Iterative)**

**Lesson 1**:

**You invoke**: `"Use lesson-writer to implement Lesson 1 based on plan.md"`

**lesson-writer generates**: `book-source/docs/04-Python-Fundamentals/15-list-comprehensions/01-introduction-to-list-comprehensions.md`

**Domain expert reviews** content → Approves ✅

**Lesson 2**:

**You invoke**: `"Use lesson-writer to implement Lesson 2 based on plan.md"`

**Domain expert reviews** → Requests changes: "Add example with string filtering"

**You refine**: `"Add example: [word for word in text.split() if len(word) > 5]"`

**Domain expert reviews** → Approves ✅

[Continue for Lessons 3-4]

**Phase 4: Validation**

**All lessons complete**. You invoke: `"Use technical-reviewer to validate chapter-15 for publication readiness"`

**technical-reviewer generates**: Validation report

**Result**: APPROVE (all 5 code examples tested, YAML frontmatter correct, constitution aligned)

**Phase 5: Publication**

```bash
git add book-source/docs/04-Python-Fundamentals/15-list-comprehensions/
git add specs/part-4-chapter-15/
git commit -m "feat: Add Chapter 15 - List Comprehensions

- Implemented 4 lessons with 5 code examples
- CEFR proficiency: A2→B2 progression
- Passes technical-reviewer validation

🤖 Generated with Claude Code
Co-Authored-By: Claude <noreply@anthropic.com>"

git push -u origin chapter-15-list-comprehensions
gh pr create --title "Chapter 15: List Comprehensions" --body "..."
```

---

## Quality Assurance with Validation

### Using technical-reviewer Effectively

**When to Validate**:
- ✅ After completing each lesson (spot-check)
- ✅ After completing entire chapter (comprehensive)
- ✅ Before creating PR (final check)
- ✅ After addressing feedback (re-validation)

**What Gets Validated**:

**1. Constitution v3.0.1 Alignment**:
- Evals-first methodology applied?
- Spec-first workflow followed?
- AI positioned as co-reasoning partner?
- Validation-first safety demonstrated?

**2. Output-Styles Compliance**:
- Structure matches chapters.md template?
- YAML frontmatter has 7 metadata fields?
- Lessons end with "Try With AI" section?
- File naming follows conventions?

**3. Chapter-Index Accuracy**:
- Chapter number and title match?
- Directory path correct?
- Prerequisites accurately referenced?

**4. Technical Quality**:
- Code examples tested and working?
- Python 3.13+ features used correctly?
- Type hints on all functions?
- No hardcoded secrets or credentials?

**5. Pedagogical Effectiveness**:
- Learning objectives met?
- Concepts scaffold progressively?
- Proficiency levels appropriate (CEFR)?
- "Try With AI" exercises meaningful?

### Interpreting Validation Results

**APPROVE**: Ready for publication
- All criteria passed
- No critical or major issues
- Minor issues (if any) are optional fixes

**REVISE & RESUBMIT**: Fix issues before publication
- Critical issues: Must fix (blocking publication)
- Major issues: Should fix (quality concerns)
- Minor issues: Nice to fix (polish)

**RETURN**: Fundamental problems, restart from spec/plan
- Spec misalignment: Chapter doesn't match approved spec
- Constitution violations: Contradicts core principles
- Quality failures: Multiple critical issues across lessons

### Addressing Validation Feedback

**Example Validation Report Extract**:

```markdown
## Critical Issues (Must Fix)

1. **Lesson 3 incomplete** - Line 143: Ends abruptly without "Try With AI"
   - Fix: Add "Try With AI" section with 4 prompts
   - Time: 30 minutes

2. **Missing YAML metadata** - All lessons lack sidebar_position field
   - Fix: Add sidebar_position: N to each lesson's frontmatter
   - Time: 10 minutes

## Major Issues (Should Fix)

3. **README case mismatch** - Chapter uses README.md (should be readme.md)
   - Fix: Rename file to lowercase
   - Time: 2 minutes
```

**Your Response**:

1. Fix critical issues immediately (blocking)
2. Decide on major issues (consult domain expert if needed)
3. Re-validate after fixes
4. Commit when validation passes

---

## Common Workflows & Examples

### Workflow 1: New Chapter from Scratch

```
1. Discovery: Meet with domain expert, define evals (2 hours)
2. Spec: Write chapter spec collaboratively (1 hour)
3. Approval: Expert reviews spec (30 min)
4. Plan: Invoke chapter-planner (10 min)
5. Approval: Expert reviews plan (30 min)
6. Tasks: Generate task checklist (5 min)
7. Implement: Iterate lesson-by-lesson with lesson-writer (4-6 hours)
   - For each lesson:
     a. Generate with lesson-writer (10 min)
     b. Expert reviews (20 min)
     c. Refine if needed (10 min)
     d. Approve (expert)
8. Validate: Run technical-reviewer (10 min)
9. Fix: Address validation issues (30 min - 2 hours)
10. Publish: Commit, PR, merge (20 min)

Total: 8-12 hours (includes expert time)
```

### Workflow 2: Redesigning Existing Chapter

**Context**: Chapter created with old subagents, needs alignment

```
1. Audit: Run technical-reviewer on existing chapter (10 min)
2. Review: Analyze validation report, categorize issues (30 min)
3. Decide: Redesign vs. repair? (consult expert)
   - Redesign: Start from Workflow 1 (spec → plan → implement)
   - Repair: Fix issues incrementally (below)

If Repair:
4. Fix Structure: Update YAML frontmatter, add metadata (1 hour)
5. Fix Content: Address closure policy violations (2 hours)
6. Fix References: Update chapter-index cross-references (30 min)
7. Re-validate: Run technical-reviewer again (10 min)
8. Publish: Commit fixes, PR, merge (20 min)

Total: 4-5 hours (repair), 8-12 hours (redesign)
```

### Workflow 3: Batch Chapter Generation (Expert-Led)

**Context**: Expert creates specs for multiple chapters, you orchestrate

```
Week 1:
- Monday: Discovery session with expert (2 hours)
- Tuesday: Expert writes specs for Chapters 20-22 (6 hours)
- Wednesday: You review specs, provide feedback (2 hours)
- Thursday: Expert revises specs (2 hours)
- Friday: You generate plans with chapter-planner (30 min total)

Week 2:
- Monday: Expert reviews plans (2 hours)
- Tues-Fri: You orchestrate implementation:
  - Chapter 20: Implement → Expert reviews → Validate (2 days)
  - Chapter 21: Implement → Expert reviews → Validate (2 days)

Week 3:
- Mon-Wed: Chapter 22: Implement → Expert reviews → Validate (3 days)
- Thu-Fri: Expert spot-checks all 3 chapters, approves publication (4 hours)

Total: 3 weeks for 3 chapters (parallelized)
```

---

## Troubleshooting & Best Practices

### Common Issues & Solutions

#### Issue 1: "Subagent Generated Wrong Structure"

**Symptoms**:
- Files in wrong directory
- YAML frontmatter missing fields
- README.md instead of readme.md

**Root Cause**: Subagent didn't reference output-styles correctly

**Solution**:
```bash
# Regenerate using explicit reference
"Use lesson-writer to generate Lesson 1.
IMPORTANT: Reference .claude/output-styles/lesson.md for correct YAML frontmatter structure."
```

**Prevention**: Verify subagents are updated (Week 2 Day 6-7 updates)

#### Issue 2: "technical-reviewer Fails Validation"

**Symptoms**:
- Critical issues found
- Content doesn't match spec
- Constitution violations

**Root Cause**: Spec unclear, or subagent misunderstood intent

**Solution**:
```bash
# Step 1: Read validation report carefully
cat specs/part-X-chapter-Y/validation-report.md

# Step 2: Identify root cause
# - Spec issue? Update spec and regenerate
# - Implementation issue? Provide more specific prompt
# - Minor issue? Fix manually

# Step 3: Re-validate after fixes
"Use technical-reviewer to re-validate chapter-Y"
```

**Prevention**: Get human approval on spec AND plan before implementation

#### Issue 3: "Expert Says 'This Isn't What I Meant'"

**Symptoms**:
- Content doesn't match expert's vision
- Examples are off-target
- Pedagogical approach wrong

**Root Cause**: Spec didn't capture expert's intent

**Solution**:
```bash
# Step 1: Collaborative spec refinement session
# - Screen share
- Walk through spec together
- Expert describes vision in detail
- You update spec in real-time

# Step 2: Use examples from similar chapters
# - "Show me a chapter that has the right style"
# - Reference that chapter in spec

# Step 3: Start with Lesson 1 only
# - Don't generate all lessons at once
# - Get expert approval on L1 before L2
# - Build shared understanding iteratively
```

**Prevention**: Show expert examples before starting, align on style

### Best Practices

#### 1. Evals-First Always ✨

**DO**:
```markdown
## Success Evals (Before Spec)
- 80%+ students can [measurable action]
- Quiz pass rate 75%+
- "Try With AI" completion 90%+
```

**DON'T**:
```markdown
## Learning Objectives
- Students will learn decorators (no measurable outcome)
- Students will understand patterns (vague)
```

#### 2. Iterative Implementation

**DO**: Generate and review lesson-by-lesson
```bash
# Lesson 1
lesson-writer → expert reviews → approve → commit

# Lesson 2
lesson-writer → expert reviews → approve → commit

# Lesson 3
...
```

**DON'T**: Generate all lessons at once
```bash
# All 7 lessons
lesson-writer (all) → expert reviews 50 pages → overwhelmed
```

**Why**: Easier to catch and fix issues early; builds shared understanding

#### 3. Reference, Don't Hardcode

**DO**:
```markdown
## Prerequisites
- See chapter-index.md for list of implemented chapters
- Chapter 12: Lists (check chapter-index.md for actual filename)
```

**DON'T**:
```markdown
## Prerequisites
- Chapter 12: lists-and-tuples (might be wrong)
- 16 chapters exist (will become stale)
```

**Why**: Single source of truth prevents drift

#### 4. Trust but Verify

**DO**: Review subagent outputs
```bash
# After lesson-writer
1. Open generated file
2. Check YAML frontmatter (7 fields present?)
3. Check structure (matches output-styles?)
4. Check "Try With AI" section (present at end?)
5. Spot-check code examples (look reasonable?)
```

**DON'T**: Blindly commit subagent output
```bash
git add . && git commit -m "chapter done" # risky
```

**Why**: Subagents are 95% compliant, not 100%

#### 5. Validate Early and Often

**DO**:
```bash
# After Lesson 1
technical-reviewer (spot-check)

# After all lessons
technical-reviewer (comprehensive)

# After fixes
technical-reviewer (re-validation)
```

**DON'T**:
```bash
# Write all 7 lessons
# ...never validate...
# Create PR → CI fails → 30 issues to fix
```

**Why**: Catching issues early saves time

---

## Reference: Available Tools

### Subagents

| Subagent | Purpose | Input | Output |
|----------|---------|-------|--------|
| chapter-planner | Transform spec into lesson plan | Approved spec | plan.md |
| lesson-writer | Generate lesson content | Approved plan + lesson # | Lesson .md file |
| technical-reviewer | Validate chapter quality | Complete chapter dir | Validation report |
| proof-validator | Fact-check and proofread | Complete chapter | Proof report |

### Skills

| Skill | Purpose | When to Use |
|-------|---------|-------------|
| skills-proficiency-mapper | Map skills to CEFR levels | During planning |
| content-evaluation-framework | Evaluate pedagogical quality | After implementation |
| code-example-generator | Generate tested code examples | During lesson writing |
| exercise-designer | Create practice exercises | During lesson writing |
| assessment-builder | Create quizzes and tests | After chapter complete |

### Slash Commands

| Command | Purpose | When to Use |
|---------|---------|-------------|
| /sp.specify | Create/update feature spec | Starting new chapter |
| /sp.plan | Generate lesson plan | After spec approved |
| /sp.tasks | Generate task checklist | After plan approved |
| /sp.implement | Execute implementation | During lesson writing |
| /sp.validate | Run validation checks | Before publication |

### Templates

| Template | Location | Purpose |
|----------|----------|---------|
| Chapter spec | `.specify/templates/chapter-spec.md` | Spec structure |
| Lesson plan | Output from chapter-planner | Plan structure |
| Chapter README | `.claude/output-styles/chapters.md` | Overview page |
| Lesson file | `.claude/output-styles/lesson.md` | Lesson structure |

### Validation Checklists

**Pre-Implementation**:
- [ ] Evals defined (business-goal-aligned)
- [ ] Spec approved by domain expert
- [ ] Plan approved by domain expert
- [ ] CEFR proficiency levels mapped

**During Implementation**:
- [ ] Each lesson reviewed by expert before next
- [ ] Code examples tested on Python 3.13+
- [ ] "Try With AI" section in every lesson
- [ ] YAML frontmatter has 7 metadata fields

**Post-Implementation**:
- [ ] technical-reviewer validation passes
- [ ] All critical issues resolved
- [ ] Expert performs final review
- [ ] Ready for commit and PR

---

## Support & Resources

### Getting Help

**Questions about the orchestration layer?**
- Read constitution: `.specify/memory/constitution.md`
- Check output-styles: `.claude/output-styles/`
- Review validation reports: `specs/001-fix-vertical-intelligence/validation/`

**Questions about content?**
- Consult domain expert
- Reference similar chapters in book
- Check chapter-index.md for prerequisites

**Technical issues?**
- GitHub Issues: [project repo]/issues
- Slack: #ai-book-development channel
- Email: cowriter-support@[domain]

### Additional Resources

**Phase 1 Completion Report**:
`specs/001-fix-vertical-intelligence/phase-1-completion-summary.md`

**Validation Examples**:
- Chapter 31 validation: `specs/001-fix-vertical-intelligence/validation/chapter-31-specifyplus-hands-on.md`
- Chapter 32 validation: `specs/001-fix-vertical-intelligence/VALIDATION-REPORT-CHAPTER-32.md`

**Chapter Redesign Guide**:
`specs/001-fix-vertical-intelligence/ESSENTIAL-TASKS-FOR-OLD-CHAPTERS.md`

---

## Changelog

### Version 1.0.0 (2025-11-04)
- Initial release after Phase 1 completion
- Added Evals-First methodology (constitution v3.0.1)
- Documented subagent updates (Week 2 Day 6-7)
- Included validation examples (Chapters 31 & 32)

---

## License & Attribution

This guide is part of the AI-Native Software Development book project.

**Generated with**: Claude Code (AI Orchestrator)
**Reviewed by**: Domain Expert (Human)
**Maintained by**: CoWriter Team

---

**🎉 You're Ready to Start!**

Pick a chapter, define success evals, write a spec, and let the orchestration layer handle the rest. Welcome to AI-native content creation! 🚀
