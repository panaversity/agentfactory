---
title: "Parallel Implementation & Integration: Proving Decomposition Works"
chapter: 32
lesson: 3
duration_minutes: 120

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Parallel Implementation Execution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run `/sp.implement` simultaneously across multiple branches, manage state across sessions without blocking, and coordinate parallel workflows"

  - name: "Integration & Conflict Resolution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can merge parallel branches in dependency order, resolve conflicts strategically, understand what conflicts reveal about decomposition quality, and validate integration quality"

  - name: "Decomposition Thinking: Part 3 (Validation)"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands concrete proof that good decomposition = clean merges; poor decomposition = integration pain; uses merge conflicts as feedback to improve system design"

learning_objectives:
  - objective: "Execute `/sp.implement` in parallel across 3 feature branches without blocking"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student demonstrates non-blocking parallel execution across multiple worktrees"

  - objective: "Merge feature branches in dependency order while strategically resolving conflicts"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student merges branches, analyzes conflicts, and explains what conflicts reveal about decomposition"

  - objective: "Analyze merge conflicts as feedback on decomposition quality; understand how this pattern scales to 7-9 agents"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student articulates decomposition lessons from integration experience and explains how pattern applies at larger scale"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (parallel execution, non-blocking coordination, merge strategies, conflict analysis, decomposition feedback, dependency management, integration testing) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Run 4-5 features in parallel instead of 3; introduce merge conflict strategies (ours vs theirs, manual resolution with strategic rebasing); simulate 10-agent team scenario"
  remedial_for_struggling: "Work through one merge conflict example in detail; use simple 2-feature setup before attempting 3; pair with instructor for dependency analysis"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/002-chapter-32-redesign/spec.md"
created: "2025-11-06"
last_modified: "2025-11-06"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Parallel Implementation & Integration: Proving Decomposition Works

## The Critical Test: From Plans to Working Code

**Can you actually build all three features in parallel, integrate them cleanly, and ship a working system?**

This lesson proves whether your decomposition was actually good. Here's the profound insight: **Clean merges mean excellent decomposition. Many merge conflicts mean your system design needs rethinking.** You're not struggling with git—you're learning about system architecture.

This is where everything changes. Most teams decompose poorly, then waste weeks in integration hell. You're going to see why, experience it firsthand, and learn the patterns to avoid it. And here's what's remarkable: **The patterns you learn with 3 features and 3 sessions scale directly to 5-7 agents running in parallel on production systems.** Same principles. Different scale.

Let's build it.

---

## Merge Strategy: Dependency Order

Before implementing, identify your merge order based on dependencies.

**Key principle**: You must merge features in dependency order. If Feature B depends on Feature A, Feature A must merge first. If they're truly independent, order doesn't matter, but the *act of merging in a deliberate order* forces you to think about hidden dependencies.

Let's say you decomposed an Assignment Grader into:

- **Feature 001**: Upload feature (students upload assignments, file validation)
- **Feature 002**: Grade feature (apply rubric, calculate scores, store grades)
- **Feature 003**: Feedback feature (generate feedback based on grades, display to students)

What's the dependency order?

- Feature 001 (Upload) has no dependencies—merge it first
- Feature 002 (Grade) needs Upload (to grade uploaded assignments)—merge second
- Feature 003 (Feedback) needs both Upload *and* Grade (to generate feedback based on assignment grades)—merge last

**Your merge order**: Feature 001 → Feature 002 → Feature 003

---

## Running Parallel Implementation

This is where the 2.5x speed advantage manifests.

### The Setup: Three Worktrees, Three Sessions

Instead of this sequential approach:

```
Start Feature 001 → 45 min → Finish Feature 001
Start Feature 002 →         45 min → Finish Feature 002
Start Feature 003 →                  45 min → Finish Feature 003
Total: ~2 hours
```

You do this in parallel:

```
Session 1: Start Feature 001 → 45 min → Finish Feature 001 ┐
Session 2: Start Feature 002 → 45 min → Finish Feature 002 ├→ Total: ~50 min
Session 3: Start Feature 003 → 45 min → Finish Feature 003 ┘
```

The clock time is ~50 minutes (dominated by the longest feature), not 2+ hours.

### Execute in 3 Terminals

You already have 3 worktrees from Lessons 1-2. Now run implementations in parallel:

**Terminal 1** (Feature 001 - Upload):
```bash
cd worktree-001-upload
claude
# Then: /sp.implement
```

**Terminal 2** (Feature 002 - Grade):
```bash
cd worktree-002-grade
claude
# Then: /sp.implement
```

**Terminal 3** (Feature 003 - Feedback):
```bash
cd worktree-003-feedback
claude
# Then: /sp.implement
```

**Timeline** (~90 minutes total for all 3):
- T+0: All 3 sessions running
- T+60-75 min: Features completing
- T+90 min: All complete

**vs Sequential**: 270 minutes (**3x speedup**)

---

## Integration: Merge in Dependency Order

All 3 features are complete. Merge them into `main`:

### Merge Sequence

**Step 1: Merge Feature 001**
```bash
git checkout main
git merge feature-001-upload
pytest tests/  # Verify
```

**Step 2: Merge Feature 002**
```bash
git merge feature-002-grade
pytest tests/  # Verify
```

**Step 3: Merge Feature 003**
```bash
git merge feature-003-feedback
pytest tests/  # Verify all features work together
```

**Expected**: Clean merges if decomposition was good.

---

## Handling Merge Conflicts

If git says "CONFLICT", two features modified the same file section.

**Example**:
```python
<<<<<<< HEAD
def validate(amount):
    return amount > 0
=======
def validate(amount):
    if amount <= 0:
        raise ValueError("Invalid")
    return True
>>>>>>> feature-002
```

**Resolution**:
1. **Decide which to keep** (or combine both)
2. **Edit the file** to resolve conflict
3. **Mark resolved**:
   ```bash
   git add filename.py
   git commit -m "Merge feature-002, resolved validation conflict"
   ```

---

## Time Tracking & Reflection

**Record your results**:

Sequential baseline: 270 min (90 min × 3 features)
Your parallel execution: _____ min
**Speedup**: _____x

**Reflection**:
1. **What enabled parallel work?** (Clear specs? Independent files?)
2. **Did you have merge conflicts?** (What did they reveal?)
3. **What would you decompose differently next time?**

---

## Try With AI

Use Claude Code to analyze your merge experience:

```
I merged 3 feature branches and had [N] conflicts in these files:
- [list files]

What does this tell me about my decomposition quality?
What should I decompose differently next time?
```

**Expected**: AI identifies decomposition issues and suggests improvements.

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />

