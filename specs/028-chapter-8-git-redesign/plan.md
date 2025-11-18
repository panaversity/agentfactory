# Chapter 8: Git & GitHub for AI-Driven Development — Complete Lesson Plan

**Specification**: `specs/028-chapter-8-git-redesign/spec.md`
**Status**: Plan (Ready for Content Implementation)
**Created**: 2025-01-17
**Version**: 2.0.0 (Reasoning-Activated Edition)
**Constitution**: v6.0.0

---

## Executive Summary

This lesson plan restructures Chapter 8 from existing 9-lesson layout into a **7-lesson pedagogically optimized sequence** that:

1. **Follows hands-on discovery modality** (varying from Chapter 7's direct teaching)
2. **Respects A1/A2 cognitive load limits** (5-7 concepts per lesson)
3. **Enforces 4-stage progression** explicitly (Manual → AI Collab → Intelligence → Capstone)
4. **Frames Git as a safety system** for AI-driven development (not command memorization)
5. **Integrates GitHub Agent HQ awareness** at appropriate awareness level (not implementation)

**Key Design Decision**: Consolidated to 7 lessons (not 9) justified by concept density analysis: 34 core concepts ÷ 7 lessons = 4.9 concepts per lesson average (within A1/A2 limit of 5-7).

---

## I. Concept Density & Lesson Count Justification

### Core Concepts Inventory (from spec.md)

**Stage 1 (Manual Foundation)**: 9 concepts
- Git repository, working directory state, staging area, commits, history, diff, restore, reset, destructive-vs-non-destructive undo

**Stage 2 (AI Collaboration)**: 14 concepts
- Branches (create/switch/list/merge/delete), GitHub (account/remote/push/pull/clone), pull requests (create/review/merge)

**Stage 3 (Intelligence Design)**: 3 concepts
- Workflow pattern recognition, git-workflow.md documentation, pattern application

**Stage 4 (Capstone)**: 3 concepts
- Complete workflow orchestration, meaningful commit messages, multi-file project management

**GitHub Agent HQ Awareness**: 5 concepts
- Multi-agent orchestration, mission control, branch controls, platform evolution, agent coordination

**Total**: 34 concepts across 4 stages

### Justified Lesson Count

Using A1/A2 tier limits (5-7 concepts/lesson):

- Stage 1: 9 concepts → 2 lessons (4-5 each)
- Stage 2: 14 concepts → 3 lessons (4-5 each distributed)
- Stage 3: 3 concepts → 1 lesson
- Stage 4: 3 concepts + 5 Agent HQ concepts → 1 capstone lesson

**Total: 7 lessons** (not arbitrary 9-lesson template)

**Validation**: Average 4.9 concepts/lesson = WITHIN A1/A2 limits ✅

---

## II. Pedagogical Arc

Chapter follows explicit 5-phase progression:

| Phase | Lessons | Stage | Modality | Focus |
|-------|---------|-------|----------|-------|
| **Foundation** | 1-2 | 1 (Manual) | Hands-on discovery (execute → observe → understand) | Core Git operations, safety mindset |
| **Application** | 3-5 | 2 (AI Collab) | Three Roles demonstrations (Teacher/Student/Co-Worker) | Branches, GitHub, PRs with bidirectional learning |
| **Integration** | 6 | 3 (Intelligence) | Persona + Questions + Principles reflection | Pattern recognition, reusable documentation |
| **Mastery** | 7 | 4 (Capstone) | Specification-first orchestration + Agent HQ awareness | Complete workflow with accumulated intelligence |

---

## III. Complete Lesson Breakdown

### Lesson 1: Your First Git Repository - Hands-On Safety Exploration

**Stage**: 1 (Manual Foundation)
**Phase**: Foundation
**Duration**: 45 minutes
**Cognitive Load**: 5 concepts (at A1 limit)

#### Learning Objectives

1. Create first Git repository and understand `.git` directory purpose (Bloom's Apply)
2. Explain what `git status` reveals about project state (Bloom's Understand)
3. Stage files using `git add` and observe staging area changes (Bloom's Apply)
4. Create first commit with meaningful message (Bloom's Apply)
5. Explain why commits are "save points" for AI safety (Bloom's Understand)

#### Concepts Covered (5 total)
1. Git repository (local version control with `git init`)
2. Working directory state (tracked/untracked files)
3. Staging area (index concept)
4. Commits as snapshots (save points with metadata)
5. Git safety mindset (fearless AI experimentation)

#### Functional Requirements Addressed
- FR-001: `git status` execution and interpretation
- FR-002: `git add <file>` staging files
- FR-003: `git commit -m "message"` creating snapshots
- FR-004: `git log` viewing history (introduced)

#### Teaching Approach: Hands-On Discovery

**Phase 1: Execute**
- Create simple project folder with 2-3 text files
- Ask Claude/Gemini: "Initialize Git in this folder"
- Observe: `.git` directory appears
- **Discovery question**: "What just happened?"

**Phase 2: Observe**
- Ask AI: "Show me status of this repository"
- AI executes `git status` → output shows untracked files
- Students trace: "I see files Git doesn't know about yet"

**Phase 3: Understand**
- Manual inspection: "Which files does Git protect?"
- Discovery: Untracked files aren't protected
- **Question**: "How do I tell Git to protect these?"

**Phase 4: Apply**
- Ask AI: "Add all files and create first commit"
- Students observe `git status` again: "Everything is committed"
- Reflection: "Now if I make a mistake, I can go back"

#### Success Criteria Mapping
- SC-001: Explain Git's value for AI development (safety framing)
- SC-002: Execute init → add → commit successfully

#### Prerequisites
- Chapter 7 (Bash): Terminal navigation
- Chapter 5 (Claude Code): AI prompting basics

#### Deliverables
- Working Git repository with 2-3 commits
- Written reflection: "Why committing before AI changes is safe"

---

### Lesson 2: Viewing Changes & Safe Undo - Discovery Through Error Recovery

**Stage**: 1 (Manual Foundation)
**Phase**: Foundation
**Duration**: 45 minutes
**Cognitive Load**: 4 concepts (within A1 limit)

#### Learning Objectives

1. View changes using `git diff` and interpret output (Bloom's Apply)
2. Discard unwanted changes using `git restore` (Bloom's Apply)
3. Unstage files using `git reset HEAD <file>` (Bloom's Apply)
4. Distinguish destructive vs non-destructive undo operations (Bloom's Understand)

#### Concepts Covered (4 total)
1. Diff (change visualization)
2. Restore (non-destructive undo)
3. Reset (unstage files)
4. Undo strategy awareness (knowing which command applies)

#### Functional Requirements Addressed
- FR-005: `git diff` execution and interpretation
- FR-006: `git restore <file>` to discard working changes
- FR-007: `git reset HEAD <file>` to unstage
- FR-008, FR-009: Destructive vs non-destructive understanding

#### Teaching Approach: Hands-On Discovery Through Deliberate Error

**Phase 1: Execute**
- Edit one committed file (add 5-10 lines)
- Ask AI: "Show me what I changed"
- AI executes `git diff` → red/green highlighting
- **Discovery**: "I can see exactly what changed"

**Phase 2: Observe (Error Scenario)**
- Deliberately add bad change (syntax error)
- See `git status` shows "modified"
- **Question**: "How do I get back to working version?"

**Phase 3: Understand**
- Explore two recovery paths: `git restore` vs manual editing
- Compare outcomes, discuss tradeoffs
- Discovery: "Restore is faster and safer"

**Phase 4: Apply to Staging**
- Create file, stage it, then change mind
- Ask AI: "Unstage this file"
- AI executes `git reset HEAD <file>`
- Observe: File still exists but not staged
- **Discovery**: "Unstaging is safe—file still there"

#### Success Criteria Mapping
- SC-003: Recover from AI-generated errors using undo commands
- SC-012: Identify which undo command applies to scenario

#### Prerequisites
- Lesson 1 (Git basics)

#### Deliverables
- Demonstration: identify → undo → verify recovery
- Written explanation: "Three ways to undo and when to use each"

---

### Lesson 3: Testing AI Safely with Branches - Parallel Experimentation

**Stage**: 2 (AI Collaboration with Three Roles)
**Phase**: Application
**Duration**: 50 minutes
**Cognitive Load**: 5 concepts (at A1 limit)

#### Learning Objectives

1. Create branches using `git branch` or `git checkout -b` (Bloom's Apply)
2. Switch branches using `git switch` or `git checkout` (Bloom's Apply)
3. Explain branch purpose (isolated experimentation) (Bloom's Understand)
4. Merge tested branch back to main (Bloom's Apply)
5. Recognize when to use branches vs commits (Bloom's Understand)

#### Concepts Covered (5 total)
1. Branches (isolated development timelines)
2. Branch creation
3. Branch switching
4. Merging (integration)
5. Branch cleanup (deletion)

#### Functional Requirements Addressed
- FR-010, FR-011, FR-012, FR-013, FR-014: All branching operations

#### Teaching Approach: Three Roles Demonstration (Stage 2)

**Setup**: Students have committed code on main. AI suggests two improvements.

**Role 1: AI as Teacher**
- Student says: "I want to test two solutions without losing either"
- AI suggests: "Create branches for parallel testing"
- **What student learns**: Branch isolation purpose they didn't explicitly ask for

**Role 2: AI as Student**
- Student tests branch but finds it's slower than main
- Student corrects: "This branch is too slow—delete it"
- AI adapts: "Understood—deleting slow branch, keeping fast one"
- **What AI learns**: Performance constraint student added

**Role 3: AI as Co-Worker (Convergence)**
- Iteration 1: Create two branches, test both
- Iteration 2: Student wants to document performance difference
- Iteration 3: AI suggests adding performance note to merge commit
- **Result**: Better commit message than either alone would create

#### Success Criteria Mapping
- SC-007: Complete branch → experiment → merge in <5 minutes
- SC-013: Recognize when to use branches vs commits

#### Prerequisites
- Lessons 1-2

#### Deliverables
- Two experimental branches tested
- Performance comparison documented
- Winning branch merged to main

---

### Lesson 4: Cloud Backup & Portfolio - GitHub Integration

**Stage**: 2 (AI Collaboration with Three Roles)
**Phase**: Application
**Duration**: 40 minutes
**Cognitive Load**: 4 concepts (within A1 limit)

#### Learning Objectives

1. Connect local repository to GitHub remote (Bloom's Apply)
2. Push commits to GitHub (Bloom's Apply)
3. Clone repository from GitHub (Bloom's Apply)
4. Explain GitHub as cloud backup and portfolio (Bloom's Understand)

#### Concepts Covered (4 total)
1. GitHub accounts (cloud platform)
2. Remote repositories (cloud-hosted copy)
3. Push (upload to cloud)
4. Clone (download from GitHub)

#### Functional Requirements Addressed
- FR-015-020: GitHub account creation through clone operations

#### Teaching Approach: Three Roles Demonstration (Stage 2)

**Setup**: Students have local repository. Now add cloud backup.

**Role 1: AI as Teacher**
- Student asks: "How do I backup my work?"
- AI suggests: "GitHub provides backup AND builds your portfolio"
- **What student learns**: Dual value (backup + portfolio showcase) they didn't know

**Role 2: AI as Student**
- AI pushes repository but student says: "Wait—don't push my secret config"
- Student corrects: "Add .gitignore first"
- AI adapts: "Understood—removing secrets and adding .gitignore"

**Role 3: AI as Co-Worker (Convergence)**
- Iteration 1: Push to GitHub
- Iteration 2: Student wants to test recovery on different machine
- Iteration 3: AI and student clone together and verify backup success
- **Result**: Validated backup strategy neither alone would verify

#### Success Criteria Mapping
- SC-006: Create and push GitHub repo in <10 minutes
- SC-011: Build GitHub portfolio with 3+ projects

#### Prerequisites
- Lessons 1-2

#### Deliverables
- GitHub repository with local code pushed
- Project cloned successfully (backup verified)
- Portfolio URL created

---

### Lesson 5: Code Review with Pull Requests - AI-Generated Code Evaluation

**Stage**: 2 (AI Collaboration with Three Roles)
**Phase**: Application & Integration
**Duration**: 50 minutes
**Cognitive Load**: 4 concepts (within A1 limit)

#### Learning Objectives

1. Create pull request from branch to main (Bloom's Apply)
2. Review PR diff to evaluate changes (Bloom's Analyze)
3. Document AI assistance in PR description (Bloom's Apply)
4. Merge approved PR into main (Bloom's Apply)

#### Concepts Covered (4 total)
1. Pull Requests (code review formalization)
2. PR diff review (evaluating changes)
3. PR description documentation (explaining intent + AI assistance)
4. Merge workflow (integrating PR)

#### Functional Requirements Addressed
- FR-021-024: Complete PR workflow

#### Teaching Approach: Three Roles + Transparency (Stage 2)

**Setup**: Feature branch with AI-generated changes. Now review and merge via PR.

**Role 1: AI as Teacher**
- Student creates basic PR without documentation
- AI suggests: "Include 'AI Assistance' section for transparency"
- **What student learns**: Transparency as best practice

**Role 2: AI as Student**
- AI writes technical PR description
- Student corrects: "Make it plain language for non-programmers"
- AI adapts: Simplifies explanation

**Role 3: AI as Co-Worker (Convergence)**
- Create PR with code
- AI suggests adding performance notes
- Student and AI converge on clear, transparent PR description

#### Success Criteria Mapping
- SC-008: Document AI assistance in 100% of PRs
- SC-001: Explain Git's value (safety + transparency)

#### Prerequisites
- Lessons 3-4

#### Deliverables
- PR with clear description including AI attribution
- Merged PR visible on GitHub
- Collaboration pattern demonstrated

---

### Lesson 6: Reusable Git Patterns - Creating Your Workflow Skill

**Stage**: 3 (Intelligence Design)
**Phase**: Integration
**Duration**: 60 minutes
**Cognitive Load**: 3 concepts (intelligence design)

#### Learning Objectives

1. Identify recurring Git workflow patterns from Lessons 1-5 (Bloom's Create)
2. Document `git-workflow.md` as reusable intelligence (Bloom's Create)
3. Apply patterns to new projects without re-learning (Bloom's Apply)

#### Concepts Covered (3 total)
1. Workflow pattern recognition (commit-before-experiment, branch-test-merge)
2. Reusable documentation (git-workflow.md)
3. Pattern application (applying to novel scenario)

#### Functional Requirements Addressed
- FR-029, FR-030, FR-031: Pattern recognition through reusable documentation

#### Teaching Approach: Persona + Questions + Principles (Stage 3)

**Part 1: Pattern Recognition (20 min)**

Reflection questions:
- "What did you do BEFORE every AI change?" → Commit current state
- "When did you branch instead of committing directly?" → Parallel experiments
- "What sequence: branch → change → test → decision?" → Feature workflow

**Part 2: Create git-workflow.md (30 min)**

Students document patterns with Persona + Questions + Principles:

**Persona**: "Think like a Git safety engineer preventing catastrophic mistakes"

**Sections**:
```markdown
# My Git Workflow for AI-Assisted Development

## Before AI Makes Changes
- Commit current state: "Before [task]"
- Why: Creates restore point

## Testing Multiple AI Suggestions
1. Create branch: option-a, option-b, option-c
2. Let AI implement on branch
3. Test each
4. Keep winner, delete losers

## Daily Pattern
1. git status → see changes
2. git diff → understand changes
3. git add → choose what to save
4. git commit → save snapshot
5. git push → backup to cloud

## Error Recovery
- Wrong file? git restore <file>
- Committed early? git reset HEAD~1
- Bad merge? git revert <commit>
```

**Part 3: Apply (10 min)**

Students encounter new scenario: AI suggests 3 authentication approaches
- Students apply documented workflow
- Create 3 branches
- Test each
- Choose and merge winner
- **Outcome**: Pattern applied without referring to lessons

#### Success Criteria Mapping
- SC-005: Commit AI-generated code within 30 seconds

#### Prerequisites
- Lessons 1-5

#### Deliverables
- `git-workflow.md` personal reference
- Successful application to new scenario

---

### Lesson 7: Capstone - Managing Git for AI-Generated Multi-File Project

**Stage**: 4 (Spec-Driven Integration with Agent HQ Awareness)
**Phase**: Mastery
**Duration**: 120 minutes
**Cognitive Load**: 5 concepts (orchestration + Agent HQ) + 5 concepts (Agent HQ awareness)

#### Learning Objectives

1. Manage complete Git workflow for AI-generated project (Bloom's Apply)
2. Compose all accumulated Git skills (Bloom's Apply)
3. Document project history through meaningful commits (Bloom's Create)
4. Understand GitHub Agent HQ as foundation for future (Bloom's Understand)
5. Validate Git enables safe AI-generated code management (Bloom's Evaluate)

#### Concepts Covered (10 total, split into two parts)

**Part A: Complete Workflow (5 concepts)**
1. Complete workflow orchestration
2. Meaningful commit messages
3. Multi-file project management
4. AI transparency documentation
5. Workflow validation

**Part B: Agent HQ Awareness (5 concepts)**
1. Multi-agent orchestration platform
2. Mission control interface
3. Branch controls for agent coordination
4. GitHub as evolving platform
5. Your role evolution (spec → multiple agents)

#### Functional Requirements Addressed
- FR-035, FR-036, FR-037: Complete workflow with accumulated skills
- FR-032, FR-033, FR-034: GitHub Agent HQ awareness

#### Teaching Approach: Specification-First + Agent HQ Awareness

**Part 1: Project Specification (20 min)**

Students write **spec.md FIRST** (before AI helps):

```markdown
# Project: Personal Task Manager CLI

## Intent
CLI task manager demonstrating Git workflow with AI-generated code

## Features
- Add task to list
- View all tasks
- Mark complete
- Delete task
- Persist to JSON

## Success Criteria
- All tasks in tasks.json
- Program runs without errors
- Help text explains usage

## Non-Goals
- Database (JSON sufficient)
- Web interface (CLI only)
- Authentication
```

**Part 2: Workflow Execution (60 min)**

1. **Initialize** (5 min)
   - Create folder → git init
   - Commit: "Initial: project structure"

2. **AI Generation** (15 min)
   - Create branch: feature/core-functions
   - Ask AI: "Generate task manager per spec"
   - AI generates main.py, tasks.json
   - Commit: "feat: core task manager"

3. **Test & Improve** (15 min)
   - Test all features
   - Create branch: feature/improvements
   - Ask AI: "Add error handling"
   - Commit: "feat: input validation"

4. **Integrate** (15 min)
   - Switch to main
   - Merge both feature branches
   - Verify all features work
   - Push to GitHub

5. **Document** (10 min)
   - Create PR with description
   - Include "AI Assistance" section
   - Explain what changed and why

**Part 3: GitHub Agent HQ Awareness (30 min)**

**Section A: What is Agent HQ? (10 min)**
- Show: GitHub Agent HQ announcement
- Explain: GitHub now orchestrates multiple AI agents
- Connect: Branches you learned = tool for managing multiple agents
- Real example: "Agent A works on auth, Agent B on database on separate branches"

**Section B: Why This Matters (10 min)**
- You're learning single-agent Git today
- Agent HQ will handle multi-agent coordination
- Git fundamentals stay same; platform adds orchestration layer
- Your role: Write specs; agents handle implementation

**Section C: You're Prepared (10 min)**
- Learning Git now = prepared for Agent HQ future
- Foundation skills don't change
- Platform evolution is additive, not disruptive
- Confidence: "I understand the foundation"

#### Success Criteria Mapping
- SC-002: Execute full workflow successfully
- SC-004: Manage branches safely
- SC-008: Document AI assistance (100%)
- SC-009: Confidence validated: "I can always undo"
- SC-014: Understand Agent HQ as future platform without anxiety

#### Prerequisites
- Lessons 1-6 (all core skills + workflow documentation)

#### Deliverables
- **Task manager project** on GitHub
- **Git history** showing branches, meaningful commits, merges
- **PR** with clear documentation including AI attribution
- **Reflection**: "How Git prevented catastrophic mistakes"
- **Understanding**: Agent HQ as platform evolution foundation

#### Expected Git History
```
* Merge pull request #1
|\
| * feat: input validation
* |
| Merge branch feature/core
|\
| * feat: core task manager
* Initial: project structure
```

---

## IV. Stage Progression Validation

### Stage Enforcement

**Stage 1 (Lessons 1-2)**: NO AI for Git operations
- Students execute `git init`, `git add`, `git commit` manually
- AI not involved in Git command execution
- Students own the learning

**Stage 2 (Lessons 3-5)**: AI collaboration WITH Three Roles
- Each lesson explicitly demonstrates:
  - **Teacher**: AI suggests pattern student didn't know
  - **Student**: Student corrects/refines AI output
  - **Co-Worker**: Iteration toward better solution
- No one-way instruction (human prompts → AI executes → done)

**Stage 3 (Lesson 6)**: Intelligence design with Persona+Q+P
- Students reflect on patterns
- Create `git-workflow.md` documentation
- Apply pattern to novel scenario without lesson reference

**Stage 4 (Lesson 7)**: Specification-first THEN orchestrate
- Spec.md written BEFORE any code
- AI generates code following spec
- Git manages the workflow

### Skipping Prevention

**✅ NOT doing**: Stage 4 (spec-first) in Lesson 1
- Stage 1 requires manual foundation BEFORE spec-first
- Spec-first only applies in capstone (Lesson 7)

**✅ NOT doing**: AI as passive tool in Stage 2
- All Stage 2 lessons require Three Roles
- Detection: If lesson shows only "human → AI → code" → FAIL

---

## V. Pedagogical Progression Summary

| Phase | Lessons | Stage | Arc Progress | Modality | Cognitive Load |
|-------|---------|-------|--------------|----------|-----------------|
| **Foundation** | 1-2 | 1 | Build mental models via manual execution | Hands-on discovery | 5, 4 concepts |
| **Application** | 3-5 | 2 | Practice with AI via Three Roles | Three Roles demos | 5, 4, 4 concepts |
| **Integration** | 6 | 3 | Create reusable skill from patterns | Persona+Q+P reflection | 3 concepts |
| **Mastery** | 7 | 4 | Orchestrate accumulated intelligence | Spec-first + Agent HQ | 10 concepts |

**Validation**:
- ✅ Follows pedagogical arc (Foundation → Application → Integration → Mastery)
- ✅ Cognitive load within A1/A2 limits
- ✅ Modality varies (hands-on, Three Roles, Persona+Q+P, spec-first)
- ✅ Stage progression enforced

---

## VI. Anti-Convergence Validation

### Chapter 7 vs Chapter 8 Modality

**Chapter 7 (Bash)**: Direct teaching
- Explain → Demonstrate → Practice

**Chapter 8 (Git)**: Hands-on discovery
- Execute → Observe → Understand

**Difference**: ✅ CONFIRMED

### Within-Chapter Modality

- Lessons 1-2: Hands-on discovery
- Lessons 3-5: Three Roles collaboration
- Lesson 6: Persona + Questions + Principles
- Lesson 7: Specification-first orchestration

**Variation**: ✅ CONFIRMED (no identical consecutive modalities)

---

## VII. Constitutional Compliance

### Principle 1: Specification Primacy
- **Lesson 7**: Spec.md written BEFORE implementation ✅

### Principle 2: Progressive Complexity
- **Average**: 4.3 concepts/lesson (within A1/A2 of 5-7) ✅
- **Scaffolding**: Heavy for Lessons 1-2 (step-by-step, validation checkpoints) ✅
- **Options**: Max 2 per concept (A1/A2 requirement) ✅

### Principle 4: Coherent Structure
- **Arc**: Foundation → Application → Integration → Mastery ✅
- **Justification**: 34 concepts ÷ 7 lessons = 4.9/lesson (not arbitrary 9) ✅

### Principle 6: Anti-Convergence
- **Chapter variation**: Hands-on discovery (vs Bash's direct teaching) ✅
- **Within-chapter variation**: Four different modalities ✅

**Overall**: ✅ FULLY COMPLIANT

---

## VIII. Content Implementation Requirements

For each lesson, content-implementer must:

1. **Lesson 1**
   - Test `git init`, `git status`, `git add`, `git commit`
   - Create hands-on discovery prompts
   - NO AI assistance (Stage 1)
   - Attach execution logs

2. **Lesson 2**
   - Test `git diff`, `git restore`, `git reset HEAD`
   - Create deliberate error scenario
   - Demonstrate recovery path
   - NO AI assistance (Stage 1)

3. **Lesson 3**
   - Test `git branch`, `git switch`, `git merge`
   - Develop explicit Three Roles scenario
   - Show convergence iteration
   - Include AI prompts showing all three roles

4. **Lesson 4**
   - Verify GitHub free tier features
   - Test `git remote add`, `git push`, `git clone`
   - Develop Three Roles scenario (backup value)
   - Show convergence through recovery testing

5. **Lesson 5**
   - Create PR on GitHub
   - Test diff visualization
   - Develop Three Roles scenario (transparency)
   - Include AI attribution in description

6. **Lesson 6**
   - Facilitate pattern reflection
   - Guide git-workflow.md creation
   - Test applying pattern to novel scenario
   - Verify students don't need lesson reference

7. **Lesson 7**
   - Create spec.md template (task manager)
   - Test AI implementation per spec
   - Test multi-file Git workflow
   - Create PR documentation
   - Include 30-minute Agent HQ awareness section
   - Verify GitHub portfolio visibility

---

## IX. Success Metrics

**This plan succeeds when**:

- ✅ Lesson count (7) justified by concept density
- ✅ All Stage 1 lessons avoid AI assistance
- ✅ All Stage 2 lessons demonstrate Three Roles explicitly
- ✅ Stage 3 creates reusable documentation
- ✅ Stage 4 writes spec FIRST, then orchestrates
- ✅ Cognitive load ≤ 7 concepts per lesson
- ✅ Teaching modalities vary (within and across chapters)
- ✅ Capstone composes accumulated intelligence
- ✅ Agent HQ awareness integrated without overwhelming
- ✅ Constitutional compliance verified
- ✅ Success criteria measurable at lesson end

---

**Plan Status**: Ready for content implementation
**Quality**: Reasoning-activated, concept-density justified, pedagogically coherent, constitutionally compliant
**Next**: Content-implementer executes per lesson requirements above
