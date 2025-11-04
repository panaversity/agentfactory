# CoWriter Guide: AI-Native Content Creation with SpecKit Plus

**Version**: 2.0.0
**Last Updated**: 2025-11-04
**For**: Domain Experts & Content Collaborators
**Prerequisites**: Access to Claude Code or AI companion, basic understanding of AI collaboration

---

## Welcome, CoWriter! 👋

This guide shows you how to create high-quality educational content using **AI-native Spec-Driven Development** with the SpecKit Plus methodology.

**What Makes This AI-Native?**
- **No manual bash commands** - AI orchestrator handles branch creation, file generation
- **Evals-first thinking** - Define success criteria before writing specs
- **Slash command workflow** - `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`
- **Iterative clarification** - AI asks questions, refines with you at each phase
- **Vertical intelligence** - Constitution → output-styles → subagents → content (all synchronized)

**Reference Material**:
- **Chapter 31**: Complete SpecKit Plus hands-on workflow (book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/)
- **Constitution v3.0.1**: Project principles and evals-first philosophy (.specify/memory/constitution.md)
- **Chapter Index**: Current book structure (specs/book/chapter-index.md)

---

## Table of Contents

1. [Quick Start: Your First Chapter](#quick-start-your-first-chapter)
2. [Understanding AI-Native SDD](#understanding-ai-native-sdd)
3. [The Four-Phase Workflow](#the-four-phase-workflow)
4. [Working with Subagents](#working-with-subagents)
5. [Quality Validation](#quality-validation)
6. [Onboarding Domain Experts](#onboarding-domain-experts)
7. [Common Patterns & Examples](#common-patterns--examples)
8. [Troubleshooting](#troubleshooting)

---

## Quick Start: Your First Chapter

### Phase 0: Think First (Evals-First Methodology)

**Before opening AI companion**, clarify in your mind or notes:

**1. Business Goals** (What matters?)
- What can students DO after this chapter?
- What real-world problems can they solve?
- How does this connect to their career goals?

**2. Success Evals** (How do we measure?)
- Quiz: Student scores 75%+ on end-of-chapter assessment
- Exercise: Student completes "Try With AI" without help
- Application: Student can apply concept to unfamiliar problem

**3. Quality Standards** (What's acceptable?)
- Technical accuracy: 100% (all code tested)
- Constitution alignment: 100% (passes technical-reviewer)
- Pedagogical effectiveness: 90%+ (learning objectives met)

**Why This Matters**:
Professional AI development (Anthropic/OpenAI standard) starts with **Evals → Spec → Implement → Validate**. Not "write code, hope it works."

Evals connect to **business goals** (student employability, skill acquisition), not arbitrary technical metrics ("10ms response time").

---

### Phase 1: `/sp.specify` - Create Specification

**Open Claude Code** (or your AI companion) and run:

```
/sp.specify

I want to create Chapter X on [topic].

Business Goal: Students learn [specific skill] to [solve real-world problem].

Core Ideas:
- [Key concept 1]
- [Key concept 2]
- [Key concept 3]

Success Evals:
- Students can [measurable outcome 1]
- Students can [measurable outcome 2]
- Assessment: [how we measure mastery]

Prerequisites: Chapter Y (students already know [prior knowledge])

Audience: [Beginner/Intermediate/Advanced/Professional] (Parts 1-3/4-5/6-8/9-13)
```

**What Happens Next**:

1. **AI asks clarifying questions**:
   - "What's in scope vs. out of scope?"
   - "What are the specific learning objectives?"
   - "What complexity tier? (A1/A2 for beginners, B1 for intermediate, etc.)"
   - "What code examples will demonstrate these concepts?"

2. **You answer and iterate** until spec is clear

3. **AI auto-creates feature branch** (no manual `git checkout -b`)

4. **AI generates `spec.md`** in `specs/part-X-chapter-Y/spec.md`

5. **You review and approve** - this is a checkpoint!

**Optional Refinement**:
```
/sp.clarify

Review the spec and identify any underspecified areas.
Ask me 3-5 targeted questions to improve clarity.
```

AI will ask questions to fill gaps you might have missed.

**Ready to Move Forward?**
When spec has all 6 sections complete (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria), proceed to planning.

**Reference**: See Chapter 31, Lesson 3 (book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/03-building-specs-with-sp-specify.md) for complete examples.

---

### Phase 2: `/sp.plan` - Generate Implementation Plan

**With approved spec**, run:

```
/sp.plan

Create implementation plan for this chapter.

Focus on:
- Break chapter into lesson sequence
- Map CEFR proficiency levels (A1/A2/B1/B2/C1)
- Identify all code examples needed
- Apply correct complexity tier for target audience
- Note dependencies and risks

Technical details:
- Use skills from .claude/skills/ (dynamic discovery)
- Reference book-source materials where appropriate
- Follow output-styles formatting (.claude/output-styles/chapters.md, lesson.md)
```

**What Happens Next**:

1. **AI reads your approved spec**

2. **AI generates lesson breakdown**:
   - Lesson 1: [Topic] - 60 minutes
   - Lesson 2: [Topic] - 90 minutes
   - Lesson 3: [Topic] - 120 minutes

3. **AI maps skills proficiency**:
   - Lesson 1: "Specification Writing" - A2 - Technical - "Student writes complete spec with 6 components"
   - Lesson 2: "Iterative Clarity" - B1 - Conceptual - "Student recognizes specs are iterative, provides refinement rationale"

4. **AI identifies code examples** with specifications:
   - Example 1: Basic [concept] demonstration (15 lines, beginner-friendly)
   - Example 2: Real-world [application] (40 lines, intermediate complexity)

5. **AI writes `plan.md`** in `specs/part-X-chapter-Y/plan.md`

6. **You review plan structure** - another checkpoint!

**Questions to Ask AI**:
- "What are the critical dependencies in this plan?"
- "Can lessons be parallelized or must they be sequential?"
- "How does [requirement from spec] appear in the plan?"

**Reference**: See Chapter 31, Lesson 4 (book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/04-planning-sp-plan.md).

---

### Phase 3: `/sp.tasks` - Decompose Into Atomic Work Units

**With approved plan**, run:

```
/sp.tasks

Break plan into atomic tasks.

Requirements:
- Each task: 1-4 hours of work
- Clear acceptance criteria per task
- Explicit dependencies between tasks
- Priority levels (MUST/SHOULD/NICE-TO-HAVE)
- Group into phases with human review checkpoints

Example structure:
- T001: Research and outline Lesson 1 content
- T002: Write Lesson 1 - Intro section
- T003: Create code example 1 (with specification)
- T004: Write "Try With AI" exercise for Lesson 1
- T005: Human review checkpoint - Lesson 1
- [Continue for all lessons...]
```

**What Happens Next**:

1. **AI breaks plan into numbered tasks** (T001, T002, T003...)

2. **AI groups tasks by phase**:
   - **Phase 1: Lesson 1 Implementation** (T001-T005)
   - **Phase 2: Lesson 2 Implementation** (T006-T010)
   - **Phase 3: Lesson 3 Implementation** (T011-T015)

3. **AI establishes dependencies**:
   - T003 depends on T002 (can't write example before concept)
   - T005 is checkpoint (human review before proceeding)

4. **AI writes `tasks.md`** in `specs/part-X-chapter-Y/tasks.md`

5. **You review task breakdown** - verify granularity makes sense

**Manual Refinement** (if needed):
```
Update @specs/part-X-chapter-Y/tasks.md so that:
- Add explicit human review checkpoint after each lesson
- Include validation steps (run technical-reviewer) before completion
- Group related tasks for better flow
```

**Reference**: See Chapter 31, Lesson 5 (book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/05-decomposing-tasks-sp-tasks.md).

---

### Phase 4: `/sp.implement` - Execute Implementation

**With approved tasks**, run:

```
/sp.implement

Let's implement this chapter following the task checklist.

Rules:
- Work through tasks sequentially
- Complete one lesson at a time
- Pause for human review after each lesson
- Apply all domain skills from .claude/skills/
- Follow output-styles formatting
- Include all metadata (7 YAML fields)
- Reference book-source materials
- Create "Try With AI" exercises
- Write end-of-lesson assessments

After each lesson:
- Mark tasks complete in tasks.md
- Run technical-reviewer validation
- Get human approval before next lesson
```

**What Happens Next**:

1. **AI works through tasks in order**:
   - T001: Research Lesson 1 outline → Presents outline
   - **You review and approve** ✓
   - T002: Write Lesson 1 intro → Generates content
   - **You review and approve** ✓
   - T003: Create code example → Shows specification → AI prompt → generated code → validation
   - **You review and approve** ✓
   - [Continue through all tasks...]

2. **Checkpoint: End of Lesson 1**:
   - AI marks T001-T005 complete in tasks.md
   - You review complete lesson content
   - Run validation: `technical-reviewer` subagent
   - If passes: Proceed to Lesson 2
   - If fails: Iterate based on feedback

3. **Repeat for all lessons** until chapter complete

4. **Final Integration**:
   - All lessons written and validated
   - Cross-references checked
   - Docusaurus build tested
   - Ready for publication

**Validation at Each Step**:
```
Run technical-reviewer on Lesson 1 to validate:
- Technical accuracy
- Code quality
- Constitution alignment
- Pedagogical effectiveness
```

**Reference**: See Chapter 31, Lesson 6 (book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/06-implementation-sp-implement.md).

---

### Phase 5: Commit and Create PR

**After all validation passes**, use the Git workflow command:

```
/sp.git.commit_pr

Wrap up Chapter X implementation.
All lessons complete, validated, and ready for review.
```

**What Happens**:
- AI creates comprehensive commit message
- AI pushes to remote branch
- AI creates pull request with summary
- You review PR description
- Merge when ready

**You're Done!** Chapter complete and ready for publication.

---

## Understanding AI-Native SDD

### Traditional vs. AI-Native Workflow

**Traditional (Manual, Old Thinking)**:
```bash
# Step 1: Create branch manually
git checkout -b chapter-X-topic

# Step 2: Create directories manually
mkdir -p specs/part-X-chapter-Y
mkdir -p book-source/docs/part-X-chapter-Y

# Step 3: Create files manually
touch specs/part-X-chapter-Y/spec.md
touch specs/part-X-chapter-Y/plan.md
touch specs/part-X-chapter-Y/tasks.md

# Step 4: Write specification (by hand, from scratch)
# Step 5: Write plan (by hand, from scratch)
# Step 6: Write tasks (by hand, from scratch)
# Step 7: Implement content (by hand, line by line)
# Step 8: Validate (manually, inconsistently)
# Step 9: Commit (manually write commit messages)
# Step 10: Create PR (manually write description)
```

**Problems**:
- High cognitive load (remembering directory structure, file conventions)
- Error-prone (typos in file paths, missing files)
- Inconsistent (different developers format differently)
- Slow (many manual steps)
- Skippable (easy to skip spec/plan and jump to coding)

---

**AI-Native (SpecKit Plus, NEW Thinking)**:
```
# Step 1: Think first (Evals-First)
# Define success criteria in your head or notes

# Step 2: /sp.specify
# Share business idea with AI
# AI clarifies, iterates, auto-creates branch, generates spec.md

# Step 3: /sp.plan
# AI reads spec, generates lesson plan, maps proficiency levels

# Step 4: /sp.tasks
# AI breaks plan into atomic tasks with dependencies

# Step 5: /sp.implement
# AI executes tasks, you review at checkpoints

# Step 6: Validate
# Run technical-reviewer, fix issues

# Step 7: /sp.git.commit_pr
# AI handles git workflow, creates PR
```

**Benefits**:
- Low cognitive load (AI handles structure, conventions)
- Error-resistant (AI follows constitution, output-styles)
- Consistent (same AI, same templates, same formatting)
- Fast (AI generates, you validate and approve)
- Enforced (workflow prevents skipping steps)

---

### The Vertical Intelligence Stack

**What Is Vertical Intelligence?**
A synchronized 4-layer architecture where every layer references the same authoritative sources:

```
┌─────────────────────────────────────┐
│ Layer 1: Constitution v3.0.1       │ ← Single source of truth
│ (.specify/memory/constitution.md)  │   (17 core principles, evals-first, domain skills)
└─────────────────────────────────────┘
              ↓ References
┌─────────────────────────────────────┐
│ Layer 2: Output Styles              │ ← Formatting standards
│ (.claude/output-styles/)            │   (chapters.md, lesson.md)
└─────────────────────────────────────┘
              ↓ References
┌─────────────────────────────────────┐
│ Layer 3: Subagents                  │ ← Specialized assistants
│ (.claude/agents/)                   │   (chapter-planner, lesson-writer,
│                                     │    technical-reviewer)
└─────────────────────────────────────┘
              ↓ Generates
┌─────────────────────────────────────┐
│ Layer 4: Content                    │ ← Book chapters, lessons
│ (book-source/docs/)                 │   (MDX files with YAML metadata)
└─────────────────────────────────────┘
```

**Why This Matters**:
- **No contradictions** - All layers read same constitution
- **Automatic updates** - Change constitution once, all subagents update
- **Consistent quality** - Same standards applied by all subagents
- **Traceability** - Every decision traces back to constitution principle

**Contrast with Traditional**:
- Traditional: Each developer has own interpretation of standards
- Vertical Intelligence: Single source of truth, synchronized references

---

## The Four-Phase Workflow

### Phase 1: Specify (WHAT to build)

**Purpose**: Define what you're creating and why it matters

**Inputs**: Business idea, success evals, target audience

**Process**:
1. Run `/sp.specify` with your core idea
2. AI asks clarifying questions
3. You answer and iterate
4. AI generates spec.md
5. You approve spec

**Outputs**:
- Feature branch (auto-created)
- `specs/part-X-chapter-Y/spec.md` (6 sections complete)

**Quality Gate**: Spec is ready when all 6 sections are complete, testable, and approved by human.

**Reference**: Constitution v3.0.1 Core Principle #1 (Evals-First Development)

---

### Phase 2: Plan (HOW to build)

**Purpose**: Break WHAT into HOW - lesson sequence, proficiency levels, code examples

**Inputs**: Approved spec.md

**Process**:
1. Run `/sp.plan`
2. AI reads spec
3. AI generates lesson breakdown with CEFR proficiency levels
4. AI identifies code examples needed
5. AI writes plan.md
6. You review plan structure

**Outputs**:
- `specs/part-X-chapter-Y/plan.md` (lesson-by-lesson architecture)
- Skills proficiency metadata (CEFR levels, Bloom's taxonomy, DigComp areas)

**Quality Gate**: Plan is ready when lesson sequence is logical, proficiency progression is sound, and dependencies are clear.

**Subagent Used**: `chapter-planner` (.claude/agents/chapter-planner.md)

---

### Phase 3: Tasks (Atomic work units)

**Purpose**: Decompose HOW into specific, testable work units

**Inputs**: Approved plan.md

**Process**:
1. Run `/sp.tasks`
2. AI breaks plan into numbered tasks (T001, T002, etc.)
3. AI establishes dependencies
4. AI groups into phases with checkpoints
5. AI writes tasks.md
6. You verify granularity

**Outputs**:
- `specs/part-X-chapter-Y/tasks.md` (atomic checklist)

**Quality Gate**: Tasks are ready when each task is 1-4 hours, has clear acceptance criteria, and dependencies are explicit.

**Manual Refinement**: Common to adjust task breakdown after initial generation (add checkpoints, regroup tasks).

---

### Phase 4: Implement (Execute and validate)

**Purpose**: Generate content, validate quality, iterate based on feedback

**Inputs**: Approved tasks.md

**Process**:
1. Run `/sp.implement`
2. AI works through tasks sequentially
3. You review after each lesson (checkpoint)
4. Run `technical-reviewer` validation
5. Iterate if issues found
6. Move to next lesson when validation passes
7. Final integration when all lessons complete

**Outputs**:
- Lesson files: `book-source/docs/part-X-chapter-Y/lesson-N.md`
- YAML metadata: 7 generation fields + skills proficiency
- Code examples: Specification → AI prompt → generated code → validation steps
- Exercises: "Try With AI" interactive exercises
- Assessments: End-of-lesson quizzes/tests

**Quality Gate**: Chapter is ready when all lessons pass technical-reviewer validation and human final review.

**Subagents Used**:
- `lesson-writer` (.claude/agents/lesson-writer.md) - generates content
- `technical-reviewer` (.claude/agents/technical-reviewer.md) - validates quality

---

## Working with Subagents

### What Are Subagents?

**Subagents** are specialized AI assistants with isolated context and specific responsibilities. Think of them as expert consultants:

- **chapter-planner**: Architect (designs lesson structure)
- **lesson-writer**: Content creator (writes lessons following plan)
- **technical-reviewer**: Quality auditor (validates correctness)

**Why Subagents vs. Main AI?**
- **Isolation**: Subagent context doesn't pollute main conversation
- **Specialization**: Each subagent has domain expertise
- **Consistency**: Same subagent applies same standards across all chapters
- **Scalability**: Multiple subagents can work in parallel (future)

**How Subagents Work**:
1. Main AI invokes subagent with task
2. Subagent has access to: Constitution, output-styles, skills, specs
3. Subagent executes task using domain skills
4. Subagent returns result to main AI
5. Main AI presents result to you for review

---

### chapter-planner Subagent

**Location**: `.claude/agents/chapter-planner.md`

**When to Use**: After spec is approved, before writing content

**Responsibilities**:
- Read approved spec.md
- Break chapter into lesson sequence
- Map CEFR proficiency levels to each lesson
- Apply concept-scaffolding (graduated complexity)
- Identify all code examples needed (with specifications)
- Check prerequisites via chapter-index.md
- Generate plan.md with lesson-by-lesson architecture
- Suggest ADRs for significant decisions

**Invocation**:
```
/sp.plan
```
(Main AI automatically invokes chapter-planner subagent)

**Skills Used** (from .claude/skills/):
- learning-objectives
- concept-scaffolding
- book-scaffolding
- skills-proficiency-mapper
- code-example-generator

**Outputs**:
- `specs/part-X-chapter-Y/plan.md`
- Skills proficiency metadata for each lesson

**Quality Checks**:
- Validates spec alignment with constitution v3.0.1
- Ensures evals defined before implementation
- Verifies proficiency progression (A1 → A2 → B1)
- Applies correct complexity tier for target audience

**Reference**: `.claude/agents/chapter-planner.md` (updated for constitution v3.0.1)

---

### lesson-writer Subagent

**Location**: `.claude/agents/lesson-writer.md`

**When to Use**: During `/sp.implement` phase

**Responsibilities**:
- Read approved plan.md and tasks.md
- Implement lesson content following specifications
- Apply all 14 domain skills from constitution
- Generate code examples: Specification → AI prompt → code → validation
- Create "Try With AI" exercises
- Write end-of-lesson assessments
- Include 7 YAML generation metadata fields
- Validate skills proficiency alignment (CEFR levels)
- Follow output-styles formatting

**Invocation**:
```
/sp.implement
```
(Main AI automatically invokes lesson-writer subagent for each lesson)

**Skills Used** (from .claude/skills/):
- learning-objectives
- concept-scaffolding
- code-example-generator
- exercise-designer
- assessment-builder
- technical-clarity
- ai-augmented-teaching
- book-scaffolding

**YAML Metadata Generated**:
```yaml
---
title: "Lesson Title"
chapter: 31
lesson: 1
duration_minutes: 90

# Skills proficiency
skills:
  - name: "Skill Name"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can [specific measurable outcome]"

# Learning objectives
learning_objectives:
  - objective: "Clear, testable objective"
    proficiency_level: "A2"
    bloom_level: "Understand, Apply"
    assessment_method: "How we measure this"

# Generation metadata (7 fields)
generated_by: "lesson-writer-v3.0.1"
source_spec: "specs/part-X-chapter-Y/spec.md"
created: "2025-11-04"
last_modified: "2025-11-04"
git_author: "AI + CoWriter Name"
workflow: "speckit-sdd"
version: "1.0.0"
---
```

**Quality Checks**:
- Content matches CEFR proficiency level from plan
- Cognitive load within limits (A1: max 5 concepts, A2: max 7, B1: max 10)
- Code examples all tested and validated
- All domain skills applied contextually
- Output-styles formatting followed

**Reference**: `.claude/agents/lesson-writer.md` (updated for constitution v3.0.1)

---

### technical-reviewer Subagent

**Location**: `.claude/agents/technical-reviewer.md`

**When to Use**: After lesson implementation, before moving to next lesson

**Responsibilities**:
- Validate technical accuracy (all code tested)
- Check code quality (Python 3.13+, TypeScript strict mode)
- Verify pedagogical effectiveness (learning objectives met)
- Assess constitution alignment (evals-first, spec-first, validation steps)
- Review output-styles compliance (formatting, structure)
- Check chapter-index references (correct chapter number, prerequisites)
- Validate skills proficiency alignment (CEFR levels appropriate)

**Invocation**:
After each lesson:
```
Run technical-reviewer on Chapter X, Lesson Y.

Validate:
- Technical correctness
- Code quality
- Constitution v3.0.1 alignment
- Pedagogical effectiveness
- Output-styles compliance
```

**Validation Report**:
```markdown
# Validation Report: Chapter X, Lesson Y

## Overall Verdict: PASS / REVISE / FAIL

## Issues Found:
### CRITICAL (must fix):
- Issue description with line reference

### MAJOR (should fix):
- Issue description with line reference

### MINOR (nice to fix):
- Issue description with line reference

## Recommendations:
- Actionable fix for each issue

## Constitution Alignment: 95%
- Principle #1 (Evals-First): ✓ PASS
- Principle #2 (Spec-First): ✓ PASS
- [etc.]
```

**Quality Thresholds**:
- PASS: 0 critical, 0-2 major issues
- REVISE: 1-3 critical OR 3+ major issues
- FAIL: 4+ critical issues

**Reference**: `.claude/agents/technical-reviewer.md` (updated for constitution v3.0.1)

---

## Quality Validation

### Why Validation Is Non-Negotiable

**Constitution Principle #14**: "Validation-First Safety - ALL AI-generated code MUST be validated before inclusion in book content."

**The Risk**: AI is powerful but imperfect. It can:
- Misunderstand spec ambiguities
- Generate code with edge case bugs
- Miss security issues
- Optimize in wrong direction

**Your Responsibility**: You validate not because AI is evil, but because AI is literal. Professional development requires systematic validation.

---

### Three-Tier Validation Strategy

#### Tier 1: Continuous Validation (During Implementation)

**When**: After each lesson implementation

**How**:
```
Run technical-reviewer on Chapter X, Lesson Y
```

**What's Checked**:
- Technical accuracy
- Code quality
- Constitution alignment
- Pedagogical effectiveness

**Action**:
- PASS: Proceed to next lesson
- REVISE: Fix issues, re-validate
- FAIL: Return to planning, refine spec

---

#### Tier 2: Chapter-Level Validation (After All Lessons)

**When**: All lessons in chapter complete

**How**:
```
Run technical-reviewer chapter-level validation for Chapter X.

Check:
- Lesson flow and coherence
- Cross-references correct
- Prerequisites met
- Proficiency progression sound
- All code examples working
```

**What's Checked**:
- Individual lesson quality (from Tier 1)
- Chapter coherence (lessons connect logically)
- Cross-reference accuracy (links to other chapters)
- Proficiency progression (A1 → A2 → B1)

**Action**:
- PASS: Proceed to publication
- REVISE: Fix issues, re-validate chapter
- FAIL: Significant rework needed

---

#### Tier 3: Human Final Review (Before Publication)

**When**: Chapter passes technical-reviewer validation

**What You Do**:
1. **Editorial polish**: Voice, tone, flow
2. **Cross-reference validation**: Links to other chapters work
3. **Docusaurus build test**: Chapter renders correctly
4. **Visual inspection**: Formatting, images, code blocks

**Tools**:
```bash
# Build and serve locally
npm run start

# Open browser to chapter
# Visually inspect formatting, images, code blocks
# Test all interactive elements
```

**Action**:
- Approved: Merge PR, publish
- Needs polish: Minor edits, re-review
- Major issues: Return to implementation

---

### Validation Checklist Template

Use this checklist for every chapter:

```markdown
# Chapter X Validation Checklist

## Tier 1: Lesson-Level Validation
- [ ] Lesson 1: technical-reviewer PASS
- [ ] Lesson 2: technical-reviewer PASS
- [ ] Lesson 3: technical-reviewer PASS
- [Continue for all lessons...]

## Tier 2: Chapter-Level Validation
- [ ] All lessons coherent and logically connected
- [ ] Cross-references to other chapters correct
- [ ] Prerequisites from chapter-index.md met
- [ ] Proficiency progression sound (CEFR levels)
- [ ] All code examples tested and working
- [ ] "Try With AI" exercises functional
- [ ] End-of-lesson assessments complete

## Tier 3: Human Final Review
- [ ] Editorial polish (voice, tone, flow)
- [ ] Docusaurus build successful
- [ ] Visual inspection (formatting, images, code blocks)
- [ ] All links working (internal + external)
- [ ] Metadata correct (YAML frontmatter)

## Ready for Publication
- [ ] All validation tiers passed
- [ ] PR created and reviewed
- [ ] Final approval obtained
```

---

## Onboarding Domain Experts

### Who Are Domain Experts?

**Domain experts** are subject matter specialists you onboard to write specific book parts:

- **Part 1-3** (Beginner): Education specialists, technical writers with pedagogy background
- **Part 4-5** (Intermediate): Software engineers with teaching experience
- **Part 6-8** (Advanced): Senior engineers, architects
- **Part 9-13** (Professional): Infrastructure engineers, DevOps specialists, production experts

---

### Onboarding Process

#### Step 1: Share This Guide

**Send**:
- This guide (docs/COWRITER-GUIDE.md)
- Constitution v3.0.1 (.specify/memory/constitution.md)
- Chapter 31 (book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/)

**Ask them to read**:
1. Constitution Core Principles (especially #1: Evals-First)
2. This guide Quick Start section
3. Chapter 31 for hands-on workflow

---

#### Step 2: Environment Setup

**Prerequisites**:
- AI companion access (Claude Code, Gemini CLI, or similar)
- Git installed and configured
- Repository cloned locally

**Verify setup**:
```bash
# Check repository access
cd /path/to/tutorgpt-build/bbb
git status

# Check AI companion available
# (Claude Code: Cmd+Shift+P > "Claude Code: Start")
# (Gemini CLI: gemini --version)
```

---

#### Step 3: Practice Run (Sandbox Chapter)

**Before real chapter**, do practice run:

```
/sp.specify

Practice Chapter: Introduction to [Your Domain Expertise Area]

Business Goal: Demonstrate the SpecKit Plus workflow with a throwaway chapter.

Core Ideas:
- [Topic 1]
- [Topic 2]

This is just practice - we'll delete this chapter after walkthrough.
```

**Walkthrough**:
1. Complete all 4 phases (/sp.specify → /sp.plan → /sp.tasks → /sp.implement)
2. Run technical-reviewer validation
3. See how iteration and refinement works
4. Delete practice chapter

**Goal**: Build confidence with workflow before real work.

---

#### Step 4: Assign Real Chapters

**Provide context**:
```markdown
# Your Assignment: Part X Chapters

You'll be writing:
- Chapter Y: [Topic] (3 lessons, beginner level)
- Chapter Z: [Topic] (4 lessons, intermediate level)

Prerequisites:
- Students have completed Chapters A, B, C
- Students know: [specific prior knowledge]

Complexity Tier: [Beginner/Intermediate/Advanced/Professional]
- Cognitive load limits: [5/7/10/no limit] new concepts per lesson
- CEFR proficiency: [A1-A2 / B1 / B2 / C1]

Reference Material:
- chapter-index.md: See current book structure
- Your chapters fit here: [specific location in book flow]
```

---

#### Step 5: Check-In Cadence

**Schedule regular check-ins**:

**After Spec Phase**:
- Review spec.md together
- Verify evals align with business goals
- Check scope is appropriate
- Approve before planning

**After Plan Phase**:
- Review lesson structure
- Verify proficiency progression
- Check code example specifications
- Approve before implementation

**After Each Lesson**:
- Review lesson content
- Run technical-reviewer together
- Discuss any issues found
- Approve before next lesson

**After Chapter Complete**:
- Chapter-level validation
- Human final review
- Merge PR

---

## Common Patterns & Examples

### Pattern 1: Creating a Beginner Chapter (Parts 1-3)

**Complexity Tier**: Beginner (A1-A2 proficiency)

**Constraints**:
- Max 2 options to choose from
- Max 5 new concepts per lesson section
- Max 1 new skill per lesson
- No forward references without explanation
- AI agent handles 3+ options

**Example Specification**:
```
/sp.specify

Chapter 12: Python Package Management for Beginners

Business Goal: Students learn how to install and manage Python packages using AI agent assistance.

Core Ideas:
- What are packages and why they matter
- Two main tools: pip and uv (AI chooses which to use)
- How to install packages for a project
- How to verify packages installed correctly

Success Evals:
- Student can explain what packages are (assessment)
- Student can ask AI to install a package (exercise)
- Student can verify installation worked (hands-on)

Audience: Complete beginners (never coded before)
Complexity: A1-A2 (beginner)

Cognitive Load:
- Lesson 1: 4 new concepts (packages, dependencies, installation, verification)
- Lesson 2: 3 new concepts (requirements file, virtual environment, AI workflow)
```

**Key Principles**:
- **Concept-first pattern**: WHAT → WHY → HOW → PRACTICE
- **AI-as-partner framing**: "Your agent knows which tool to use"
- **Error literacy**: Show what errors mean, how to ask AI for help
- **No memorization required**: Students understand concepts, AI executes

**Reference**: Constitution Principle #12 (Graduated Complexity)

---

### Pattern 2: Creating an Intermediate Chapter (Parts 4-5)

**Complexity Tier**: Intermediate (B1 proficiency)

**Constraints**:
- 3-4 options allowed (show tradeoffs)
- Max 7 new concepts per lesson section
- Expect independent problem-solving with AI assistance
- Introduce architectural thinking

**Example Specification**:
```
/sp.specify

Chapter 28: API Design Patterns with AI Code Generation

Business Goal: Students learn to design clean APIs and use AI to generate boilerplate code.

Core Ideas:
- REST vs. GraphQL vs. RPC (tradeoffs discussed)
- API design principles (consistency, versioning, error handling)
- Using AI to generate API scaffolding from specification
- Validating generated API code

Success Evals:
- Student can compare 3 API patterns and choose appropriately (assessment)
- Student can write API specification that AI understands (exercise)
- Student can validate generated API code (hands-on)

Audience: Intermediate (know one programming language, learning architecture)
Complexity: B1 (intermediate)

Cognitive Load:
- Lesson 1: 6 new concepts (REST, GraphQL, RPC, tradeoffs, when to use each, specification writing)
- Lesson 2: 7 new concepts (endpoints, routes, middleware, error handling, validation, testing, deployment)
```

**Key Principles**:
- **Tradeoffs discussion**: Multiple valid approaches, explain when to use each
- **Specification-first**: Write API spec before generating code
- **Validation emphasis**: Students verify AI-generated code meets requirements
- **Real-world context**: Connect to production systems

**Reference**: Constitution Principle #12 (Graduated Complexity - Tier 2)

---

### Pattern 3: Creating a Professional Chapter (Parts 9-13)

**Complexity Tier**: Professional (B2-C1 proficiency)

**Constraints**:
- No artificial option limits (show ecosystem realistically)
- No concept count limits (synthesis and integration expected)
- Production-ready expectations (security, scale, cost, operations)
- Business context (ROI, SLAs, risk management)

**Example Specification**:
```
/sp.specify

Chapter 47: Production Kubernetes Deployment with AI-Generated Infrastructure Code

Business Goal: Students deploy production-ready Kubernetes clusters using AI to generate infrastructure-as-code with proper security, observability, and cost controls.

Core Ideas:
- Kubernetes production requirements (HA, security, observability, cost)
- Infrastructure-as-code (Terraform, Pulumi, Crossplane - compare and choose)
- Using AI to generate IaC from architectural requirements
- Validating generated infrastructure (security audit, cost estimation, reliability review)
- Deployment strategies (blue-green, canary, rolling updates)
- Operational readiness (monitoring, alerting, runbooks)

Success Evals:
- Student can write infrastructure requirements specification (hands-on)
- Student can use AI to generate complete IaC (exercise)
- Student can perform security audit on generated code (hands-on)
- Student can deploy to production with proper observability (capstone)

Audience: Professional (production experience required)
Complexity: B2-C1 (professional)

Production Validation Checklist:
- Security: Secrets management, IAM roles, network policies, TLS
- Reliability: Health checks, graceful shutdown, retry logic, circuit breakers
- Observability: Structured logging, metrics, tracing, alerting
- Cost: Resource requests appropriate, autoscaling configured, no over-provisioning
```

**Key Principles**:
- **No scaffolding**: Assumes professional competence
- **Real-world complexity**: Security, scale, cost, operations
- **System thinking**: Not just code, entire production system
- **Business context**: ROI, SLAs, risk management
- **Comprehensive validation**: Security audit, cost estimation, reliability review

**Reference**: Constitution Principle #12 (Graduated Complexity - Tier 4)

---

### Pattern 4: Handling Validation Failures

**Scenario**: technical-reviewer finds critical issues after lesson implementation

**What Happened**:
```markdown
# Validation Report: Chapter 31, Lesson 2

## Overall Verdict: REVISE

## Issues Found:
### CRITICAL:
- Code example at line 145 has unhandled edge case (division by zero)
- Specification at line 67 is ambiguous ("user can see history" - format unclear)

### MAJOR:
- CEFR proficiency mismatch: Content teaches B1 concepts but marked as A2
- Missing validation steps in code example workflow
```

**How to Fix**:

**Step 1: Analyze root cause**
```
Which issue is upstream?
- Code bug → Implementation issue (fix in lesson)
- Spec ambiguity → Specification issue (fix spec.md, regenerate)
- Proficiency mismatch → Planning issue (fix plan.md, regenerate)
```

**Step 2: Fix at appropriate layer**

**If implementation issue** (code bug):
```
Fix the code example at line 145 to handle division by zero.

Add validation steps:
1. Show specification for error handling
2. Show AI prompt: "Generate division function with zero check"
3. Show generated code with error handling
4. Show validation: Test with zero, verify raises appropriate error
```

**If specification issue** (ambiguity):
```
Update @specs/part-5-chapter-31/spec.md:

OLD: "User can see calculation history"
NEW: "User can see calculation history as JSON array, max 100 entries, sorted newest first"

Then regenerate affected lessons with updated spec.
```

**If planning issue** (proficiency mismatch):
```
Update @specs/part-5-chapter-31/plan.md:

Change Lesson 2 proficiency from A2 to B1.
Rationale: Content requires analysis of tradeoffs (B1 cognitive level).

Adjust content to match B1 complexity (up to 10 concepts allowed).
```

**Step 3: Re-validate**
```
Run technical-reviewer on updated Chapter 31, Lesson 2
```

**Step 4: Verify fix**
```
# Validation Report: Chapter 31, Lesson 2 (Revised)

## Overall Verdict: PASS

## Issues Fixed:
✓ Code example handles division by zero
✓ Specification updated with clear format
✓ Proficiency level corrected to B1
✓ Validation steps added to code workflow

## Recommendations:
- Consider adding similar validation examples to other lessons
```

**Reference**: Constitution Principle #14 (Validation-First Safety)

---

## Troubleshooting

### Issue 1: "AI doesn't follow output-styles formatting"

**Symptom**: Generated content doesn't match .claude/output-styles/

**Root Cause**: Subagent not referencing output-styles correctly

**Fix**:
```
Regenerate lesson following output-styles exactly.

Reference:
- .claude/output-styles/chapters.md (chapter structure)
- .claude/output-styles/lesson.md (lesson structure)

Ensure:
- YAML frontmatter matches template
- Section headers match style guide
- Code blocks formatted per standards
```

**Prevention**: Always include output-styles reference in `/sp.implement` command.

---

### Issue 2: "Subagent outputs not written to files"

**Symptom**: AI shows generated content in chat but doesn't create files

**Root Cause**: Subagent returned content without writing

**Fix**:
```
Write the generated content to file:
- Path: book-source/docs/part-X-chapter-Y/lesson-N.md
- Ensure all YAML metadata included
- Verify file created successfully
```

**Prevention**: After subagent execution, verify files exist:
```bash
ls -la book-source/docs/part-X-chapter-Y/
```

---

### Issue 3: "CEFR proficiency levels seem wrong"

**Symptom**: Lesson marked A2 but content is too complex (or too simple)

**Root Cause**: Proficiency mapping error during planning

**Fix**:
```
Review @specs/part-X-chapter-Y/plan.md proficiency levels.

CEFR guidelines:
- A1: Recognition only (identify, recall)
- A2: Recognition + simple application with scaffolding
- B1: Application to real, unfamiliar problems independently
- B2: Analysis, evaluation, design decisions
- C1: Advanced synthesis, critique, expert judgment

Adjust lesson proficiency level and regenerate content to match.
```

**Reference**: skills-proficiency-mapper skill (.claude/skills/skills-proficiency-mapper/)

---

### Issue 4: "Chapter-index.md out of sync"

**Symptom**: Chapter numbers or titles don't match chapter-index.md

**Root Cause**: chapter-index.md not updated after planning

**Fix**:
```
Update @specs/book/chapter-index.md:

For Chapter X:
- Status: 📋 Planned → ✅ Implemented
- Title: [Update if changed]
- File path: [Verify correct]
```

**Prevention**: Always reference chapter-index.md during planning phase.

---

### Issue 5: "AI skips evals-first step"

**Symptom**: Spec created without defining success criteria first

**Root Cause**: Not following constitution v3.0.1 Principle #1

**Fix**:
```
STOP implementation.

Return to Phase 0: Think First (Evals-First).

Define before proceeding:
1. Business goals (what matters?)
2. Success evals (how do we measure?)
3. Quality standards (what's acceptable?)

Then regenerate spec.md with evals integrated.
```

**Prevention**: Always start with "Think First" phase before `/sp.specify`.

---

### Issue 6: "Branch not auto-created"

**Symptom**: `/sp.specify` doesn't create feature branch

**Root Cause**: Git workflow command not integrated in slash command

**Fix**:
```bash
# Manually create branch
git checkout -b part-X-chapter-Y-topic

# Then run /sp.specify
```

**Note**: Auto-branch creation may vary by AI companion configuration.

---

## Reference: Available Tools

### Slash Commands

| Command | Purpose | Phase | Output |
|---------|---------|-------|--------|
| `/sp.specify` | Create specification | 1 - Specify | spec.md |
| `/sp.clarify` | Identify gaps, ask clarifying questions | 1 - Specify | Updated spec.md |
| `/sp.plan` | Generate implementation plan | 2 - Plan | plan.md |
| `/sp.tasks` | Decompose plan into atomic tasks | 3 - Tasks | tasks.md |
| `/sp.implement` | Execute implementation | 4 - Implement | Lesson files |
| `/sp.adr` | Create Architecture Decision Record | Any | history/adr/NNN-title.md |
| `/sp.phr` | Create Prompt History Record | Any | history/prompts/.../ID-title.md |
| `/sp.git.commit_pr` | Commit and create pull request | After implement | Git commit + PR |

---

### Subagents

| Subagent | Location | Purpose | Invoked By |
|----------|----------|---------|-----------|
| **chapter-planner** | .claude/agents/chapter-planner.md | Plan lesson structure, proficiency levels | `/sp.plan` |
| **lesson-writer** | .claude/agents/lesson-writer.md | Generate lesson content | `/sp.implement` |
| **technical-reviewer** | .claude/agents/technical-reviewer.md | Validate quality | Manual after lessons |

---

### Domain Skills

Skills are located in `.claude/skills/` and dynamically discovered:

| Skill | Purpose | Used By |
|-------|---------|---------|
| **learning-objectives** | Define clear, testable objectives | chapter-planner, lesson-writer |
| **concept-scaffolding** | Apply graduated complexity | chapter-planner, lesson-writer |
| **code-example-generator** | Create code examples with specs | lesson-writer |
| **exercise-designer** | Design "Try With AI" exercises | lesson-writer |
| **assessment-builder** | Create end-of-lesson quizzes | lesson-writer |
| **technical-clarity** | Write clear technical explanations | lesson-writer |
| **book-scaffolding** | Maintain book structure, cross-references | chapter-planner, lesson-writer |
| **ai-augmented-teaching** | Frame AI as co-reasoning partner | lesson-writer |
| **skills-proficiency-mapper** | Map CEFR/Bloom's/DigComp levels | chapter-planner |

---

### File References

| File | Purpose | Used By |
|------|---------|---------|
| `.specify/memory/constitution.md` | Project principles, evals-first, domain skills | All subagents |
| `specs/book/chapter-index.md` | Book structure, chapter status | chapter-planner |
| `.claude/output-styles/chapters.md` | Chapter formatting standards | lesson-writer |
| `.claude/output-styles/lesson.md` | Lesson formatting standards | lesson-writer |
| `book-source/` | Source material to reference | lesson-writer |

---

## Summary

**This is AI-Native Content Creation**:
1. **Think first** (Evals-First): Define success criteria before specifications
2. **Use slash commands** (/sp.specify, /sp.plan, /sp.tasks, /sp.implement)
3. **AI orchestrates** (branch creation, file generation, formatting)
4. **You validate** (review, approve, iterate at checkpoints)
5. **Subagents specialize** (planning, writing, validation)
6. **Constitution synchronizes** (single source of truth, vertical intelligence)

**You are now ready** to create high-quality educational content using the SpecKit Plus methodology!

**Questions?** Reference:
- Chapter 31: Complete hands-on workflow (book-source/docs/05-Spec-Kit-Plus-Methodology/31-specifyplus-hands-on/)
- Constitution v3.0.1: Project principles (.specify/memory/constitution.md)
- This guide: AI-native patterns and troubleshooting

**Happy creating!** 🚀
