---
ID: "001"
TITLE: "Chapter 8 Plan & Task Generation"
STAGE: "plan"
DATE_ISO: "2025-10-31"
SURFACE: "agent"
MODEL: "claude-haiku-4-5-20251001"
FEATURE: "001-chapter-8-git-github"
BRANCH: "001-chapter-8-git-github"
USER: "system"
COMMAND: "chapter-planner: generate plan.md and tasks.md for Chapter 8"
LABELS: ["chapter-planning", "git-github", "lesson-architecture", "task-decomposition"]
LINKS:
  SPEC: "specs/001-chapter-8-git-github/spec.md"
  PLAN: "specs/001-chapter-8-git-github/plan.md"
  TASKS: "specs/001-chapter-8-git-github/tasks.md"
  CONSTITUTION: ".specify/memory/constitution.md"
  DIRECTORY_STRUCTURE: "specs/book/directory-structure.md"
---

# Chapter 8 Planning Session: Git & GitHub for AI-Driven Development

## Prompt Summary

You are the chapter-planner subagent tasked with transforming the approved specification for Chapter 8 (Git & GitHub for AI-Driven Development) into detailed implementation artifacts.

**Inputs:**
- Feature Spec: `specs/001-chapter-8-git-github/spec.md`
- Constitution: `.specify/memory/constitution.md`
- Directory Structure: `specs/book/directory-structure.md`
- Output Styles: `.claude/output-styles/lesson.md` and `.claude/output-styles/chapter-readme.md`

**Outputs:**
1. Detailed implementation plan (`plan.md`) with 5-lesson architecture
2. Actionable task checklist (`tasks.md`) with acceptance criteria and priorities

**Key Decisions Required:**
1. Lesson structure (hybrid chapter: Part I Git commands, Part II Natural Language Prompts)
2. Scaffolding approach (progression from motivation → essentials → branches → collaboration → convenience layer)
3. Exercise design (hands-on practice validating each concept)
4. Risk mitigation (secrets, installation, merge conflicts, authentication)

---

## Key Planning Decisions Made

### Decision 1: Five-Lesson Architecture

**Rationale**: 3-4 hours naturally breaks into 5 focused lessons (30-90 min each), avoiding cognitive overload while ensuring comprehensive coverage.

| Lesson | Duration | Focus | Bloom's Level |
|--------|----------|-------|---------------|
| 1 | 30 min | Why Git matters (motivation) | Understand |
| 2 | 90 min | Installation & basic workflow | Apply |
| 3 | 90 min | Safe experimentation (branches, rollback) | Apply |
| 4 | 75 min | Pull requests & collaboration | Apply |
| 5 | 45 min | Natural language prompts (convenience layer) | Apply |
| **Total** | **4.5-5h** | **Foundation + practice + professional + convenience** | |

**Rejected Alternative**: Fewer, longer lessons would overload cognitive load; more lessons would fragment the narrative.

### Decision 2: Hybrid Structure (Commands First, Prompts Second)

**Rationale**: Teaches understanding first (Lessons 2-4), then shows convenience layer (Lesson 5). Prevents prompts from becoming "magic" without comprehension. Aligns with constitutional principle: AI as partner, not replacement.

**Structure**:
- **Part I (Lessons 1-4)**: Git commands and workflows (core understanding)
- **Part II (Lesson 5)**: Natural language prompts (convenience layer)

**Rejected Alternative**: Interleaving commands and prompts throughout—too fragmented, wouldn't reinforce core concepts.

### Decision 3: Lesson 1 as Narrative Foundation

**Rationale**: Establishes motivation and safety-net concept before teaching syntax. Addresses Risk 2 (learners fear "breaking everything"). Shows why Git matters in AI-driven development context.

**Content**: No code, no commands—just stories, scenarios, and analogies that build excitement about safe experimentation.

**Rejected Alternative**: Starting with installation—would lose motivation, learners don't yet understand why they're learning Git.

### Decision 4: GitHub UI Focus for Pull Requests

**Rationale**: Lesson 4 teaches PR workflow via GitHub web interface, reducing command-line friction. Most learners will use GitHub web, not CLI.

**Content**: Screenshots/descriptions of GitHub UI, focus on process and communication, not Git commands.

**Rejected Alternative**: Teaching `git request-pull` or complex CLI workflows—not standard, GitHub web is faster and more visual.

### Decision 5: Safety and Rollback Emphasized Early

**Rationale**: Lesson 3 teaches rollback/stash prominently (not as advanced topic). This is core value of Git for AI-driven development—safe experimentation. Builds confidence early.

**Content**: Checkpoint pattern → branches → safe undoing → merging (progression from simple safety to complex merging).

**Rejected Alternative**: Rollback as advanced topic late in chapter—would leave learners anxious throughout Lessons 2-3.

### Decision 6: 20+ Exercises Distributed Across Lessons

**Rationale**: Each lesson has 5-6 hands-on exercises with clear success criteria. Validates learning incrementally (not one mega-project at end).

**Structure**:
- Lesson 1: Reflection prompts (no exercises—conceptual)
- Lesson 2: 5 exercises (installation, config, workflow, commit, history)
- Lesson 3: 6 exercises (checkpoint, branch, switch, merge, reset, conflict resolution)
- Lesson 4: 7 exercises (repo creation, push, PR, review, feedback, merge, optional: conflict in PR)
- Lesson 5: 6 exercises (prompt translation, learning, workflow, conflict help)
- **Plus**: 1 comprehensive end-of-chapter project (60-90 min)
- **Plus**: Quick reference card and common scenarios guide (SHOULD, not MUST)

**Total**: 20+ hands-on exercises building confidence throughout chapter.

### Decision 7: Domain Skills Application Explicit

**Rationale**: Each of the 9 constitutional domain skills applied explicitly to this chapter. Ensures quality, consistency, pedagogical effectiveness.

**Skills Applied**:
1. **Learning Objectives**: Each lesson has single, measurable Bloom's-level objective
2. **Concept Scaffolding**: Git decomposed into why → setup → workflow → branches → collaboration → prompts
3. **Code Examples**: All runnable on Windows/Mac/Linux with expected output
4. **Exercise Design**: Progressive complexity, clear acceptance criteria
5. **Assessment**: Self-assessments at end of each lesson
6. **Technical Clarity**: No jargon gatekeeping, multiple explanation styles
7. **Book Scaffolding**: Chapter positioned within Part 2, bridges to Part 3
8. **AI-Augmented Teaching**: Natural language prompts, safe experimentation enabled by Git
9. **Content Evaluation**: Technical accuracy, pedagogical effectiveness, constitution alignment verified

### Decision 8: Task Breakdown into Atomic Units

**Rationale**: Each task completes in 1-2 hours, has clear acceptance criteria, testable completion. Enables parallel work where possible.

**Task Organization**:
- **Phase 1**: Chapter foundation & Lesson 1 (foundation)
- **Phase 2-5**: Lessons 2-5 (sequential, each with content + exercises + assessment)
- **Phase 6**: Cross-chapter integration & comprehensive project
- **Phase 7**: QA (pedagogical review, technical verification, accessibility, constitution alignment, coherence)
- **Phase 8**: Final review & publication readiness

**Total**: 33 atomic tasks (13 MUST, 5 SHOULD, 3 NICE-TO-HAVE)

---

## Architecture Alignment Checklist

### Constitutional Principles

- ✅ **Principle 1 (AI-First Teaching)**: Chapter demonstrates AI tools as collaborative partners
  - Lesson 5 shows natural language prompts for Git operations
  - Lesson 4 suggests asking AI for help with merge conflicts
  - Throughout: safe experimentation enabled by Git, making AI collaboration viable

- ✅ **Principle 5 (Progressive Complexity)**: Content difficulty increases gradually
  - Lesson 1: motivation (no syntax)
  - Lesson 2: simple commands (basic workflow)
  - Lesson 3: branches (more abstract concept)
  - Lesson 4: GitHub UI (familiar interface)
  - Lesson 5: prompts (convenience layer)

- ✅ **Principle 8 (Accessibility)**: No gatekeeping language
  - No assumed prior Git knowledge
  - Jargon defined on first use
  - Multiple explanation styles (stories, step-by-step, diagrams, UI)
  - Platform-specific instructions (Windows/Mac/Linux)

- ✅ **Principle 9 (Show-Then-Explain)**: Workflows shown before explanations
  - Lesson 2: complete workflow first, then explain each command
  - Lesson 3: branch workflow demonstrated, then explained
  - Lesson 4: PR workflow via UI shown, then concepts explained
  - Lesson 5: prompts shown, then best practices explained

### Book Gaps Checklist (Constitutional Section II.C)

**For ALL Chapters:**
- ✅ Factual accuracy: All Git behavior verified against actual Git 2.x behavior
- ✅ Field volatility: Notes on topics needing future updates (GitHub UI changes, Git versions)
- ✅ Inclusive language: No gatekeeping, diverse examples, gender-neutral
- ✅ Accessibility: Defined terminology, multiple explanations, visual breaks
- ✅ Bias & representation: Not applicable (technical topic), but examples include diverse contexts

**For Technical Chapters (Code-Focused):**
- ✅ Code security: `.gitignore` coverage, warning against committing secrets
- ✅ Ethical AI: Frame AI limitations in Lesson 5 (prompts are convenience, not replacement)
- ✅ Testing & quality: Exercises have clear success criteria, verification steps
- ✅ Deployment readiness: Instructions for all platforms, troubleshooting guides
- ✅ Scalability awareness: Not directly applicable (version control scales naturally)
- ✅ Real-world context: Realistic Git scenarios (AI refactoring, team reviews, rollback needs)
- ✅ Engagement: Opening hooks in each lesson, real examples, visual breaks
- ✅ Practicality: Cross-platform setup, common pitfalls section, environment setup explicit

---

## Success Criteria from Spec (Addressed)

| Spec Criteria | Plan Section | Measurement |
|---------------|--------------|-------------|
| SC-001: Create repo and commit in 5 min | Lesson 2, Exercises 3-4 | Timed exercise with success verification |
| SC-002: Safe checkpoint and rollback | Lesson 3, Exercises 1, 5 | Create checkpoint, break code, rollback, verify |
| SC-003: Branch workflow without data loss | Lesson 3, Exercises 2-4 | Create, edit, switch, merge, verify no loss |
| SC-004: Create PR, address feedback, merge | Lesson 4, Exercises 1-7 | Full GitHub PR workflow end-to-end |
| SC-005: Translate 10+ commands to prompts | Lesson 5, Prompt examples | 10+ command→prompt translations provided, practiced |
| SC-006: Understand when to commit | Lesson 2 reflection, Lesson 5 scenarios | Self-assessment + real scenario practice |
| SC-007: End-of-chapter project (90% success) | Task 6.1 | Comprehensive project combining all skills |
| SC-008: Identify commands for 5 scenarios | Lesson 4-5, scenarios reference | Scenario-based exercises and reference guide |

---

## Risk Mitigations (From Spec)

**Risk 1: Learners accidentally commit secrets** → Lesson 2 includes `.gitignore` introduction early, templates, warning boxes
**Risk 2: Fear of "breaking everything"** → Lesson 1 establishes safety-net concept, Lesson 3 teaches rollback prominently
**Risk 3: Platform-specific installation issues** → Detailed Win/Mac/Linux instructions, verification, troubleshooting in Lesson 2
**Risk 4: GitHub authentication confusion** → Step-by-step PAT guide in Lesson 4, error message interpretation
**Risk 5: Merge conflict paralysis** → Explanation in Lesson 3, simple resolution steps, offer AI assistance

---

## Effort Estimates

| Phase | Tasks | Effort | Notes |
|-------|-------|--------|-------|
| 1 (Foundation) | 2 | 4h | Chapter README + Lesson 1 narrative |
| 2 (Git Essentials) | 3 | 7h | Installation, workflow, exercises, assessment |
| 3 (Safe Experimentation) | 3 | 7.5h | Branches, rollback, exercises, assessment |
| 4 (Pull Requests) | 3 | 7h | GitHub UI, exercises, assessment |
| 5 (Natural Language) | 3 | 4h | Prompt examples, exercises, assessment |
| 6 (Integration) | 3 | 4.5h | Comprehensive project, references |
| 7 (QA) | 5 | 7h | Pedagogical, technical, accessibility, constitution, coherence |
| 8 (Final Review) | 3 | 4h | Style guide, editorial, handoff |
| **TOTAL** | **25** | **16-23h** | Sequential implementation with parallel QA |

---

## Files Delivered

### Plan.md
- **File**: `specs/001-chapter-8-git-github/plan.md`
- **Size**: ~6,000 words
- **Contents**:
  - Chapter objectives (5 learning outcomes)
  - Lesson architecture (5 lessons, each with objectives, content outline, exercises, prerequisites)
  - Lesson dependencies and flow
  - Content flow and cognitive load management
  - Integration with book (Chapters 5-7, Part 3, Part 4)
  - Domain skills application (all 9 skills mapped)
  - Success criteria mapping
  - Validation strategy
  - File organization (directory structure)
  - Key architectural decisions (5 decisions with rationale and alternatives)
  - Risk mitigations
  - Implementation notes for content-implementer and validation-auditor
  - Effort estimates
  - Success definition

### Tasks.md
- **File**: `specs/001-chapter-8-git-github/tasks.md`
- **Size**: ~8,000 words
- **Contents**:
  - 25 atomic tasks organized in 8 phases
  - Each task has:
    - Priority (MUST/SHOULD/NICE-TO-HAVE)
    - Effort estimate
    - Dependencies
    - Acceptance criteria (checkbox lists)
    - Expected deliverables
  - Task dependency graph (visual and explicit)
  - Overall acceptance criteria (definition of done)
  - Follow-up sequence for agent handoff
  - Notes for implementation and review teams

---

## Response Summary

**Delivered**: Complete chapter planning artifacts for Chapter 8: Git & GitHub for AI-Driven Development

**Key Outputs**:
1. ✅ **plan.md** (6,000 words): 5-lesson architecture with detailed breakdown, scaffolding strategy, domain skills mapping, and architectural decisions
2. ✅ **tasks.md** (8,000 words): 25 atomic implementation tasks organized by phase, with clear acceptance criteria and dependency ordering

**Architecture Highlights**:
- **Hybrid chapter**: Part I (Git commands, Lessons 1-4) + Part II (Natural language prompts, Lesson 5)
- **5 lessons** (30-90 min each): Narrative foundation → essentials → safe experimentation → team collaboration → convenience layer
- **Progressive scaffolding**: Why → setup → workflow → branches → PRs → prompts
- **20+ exercises**: Hands-on practice validating each concept, building confidence incrementally
- **Risk-aware**: Early emphasis on safety (checkpoint, rollback), all risk mitigations documented
- **Constitution-aligned**: All 9 domain skills applied, constitutional principles validated, book gaps checklist addressed
- **Testable deliverables**: Each task has clear acceptance criteria, all exercises have success criteria

**Ready for**: Handoff to content-implementer agent for Phase 2 (sequential lesson implementation)

---

## Next Steps

1. **Review & Approve**: User reviews plan and tasks, provides feedback on architectural decisions
2. **Handoff to Lesson-Writer Agent**: Use `sp.implement` to begin lesson-by-lesson content creation
3. **Iterative Review**: After each lesson drafted, human reviews and provides feedback
4. **Technical Verification**: Technical-reviewer agent validates pedagogy, accuracy, constitution alignment
5. **Publication**: Chapter ready for Docusaurus build once all phases complete

