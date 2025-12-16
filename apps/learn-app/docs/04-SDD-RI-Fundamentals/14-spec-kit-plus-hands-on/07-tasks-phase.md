---
title: "Tasks Phase - Atomic Work Units and Checkpoints"
chapter: 14
lesson: 7
duration_minutes: 30
proficiency_level: "B1"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.tasks Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.tasks to decompose plan into atomic work units"

  - name: "Understanding Atomic Task Definition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands atomic task = 15-30 minute unit with single acceptance criterion"

  - name: "Recognizing Task Dependencies"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands which tasks must complete before others (dependency graph)"

  - name: "Human-Controlled Checkpoint Pattern"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands Agent→Review→Commit→Continue workflow and human's role in each checkpoint"

learning_objectives:
  - objective: "Use /sp.tasks to decompose research paper plan into atomic work units"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.tasks and understanding of task breakdown"

  - objective: "Understand checkpoint pattern for task execution: Agent Complete → Human Review → Human Approve → Continue"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of checkpoint pattern and human's control role"

  - objective: "Trace requirement lineage from specification through plan to tasks"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Ability to follow a requirement from spec to task unit"

  - objective: "Identify tasks with clear acceptance criteria and explicit dependencies"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Ability to review task list and validate quality"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (/sp.tasks command, atomic units, dependencies, checkpoint pattern) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design task breakdown for 2-3 different projects (paper, research proposal, literature review); compare how task structure varies by project complexity"
  remedial_for_struggling: "Focus on understanding what makes a 'good' task (atomic, testable, 15-30 min) without detailed dependency analysis"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/036-chapter-14-sdd-ri-hands-on/spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Tasks Phase - Atomic Work Units and Checkpoints

You now have:
- ✅ A clear research paper specification (intent, success criteria, scope)
- ✅ A detailed implementation plan (research approach, structure, timeline)
- ✅ Documented architecture decisions (citation style, source strategy, outline format)

Next: Break the plan into **atomic work units** (tasks) that you'll execute. Each task is 15-30 minutes, has one acceptance criterion, and produces a verifiable output.

This lesson teaches the **checkpoint pattern**—the critical workflow practice that keeps YOU in control. The pattern is:

```
Agent: "Here's Section 1 research complete"
You: "Review... sources are credible!"
You: "Commit to git"
You: "Tell me what's next"
Agent: "Section 2 outline refinement starting"
```

NOT:

```
Agent: "Here's everything at once" (no human control)
```

The difference is huge. Checkpoints keep you in control and catch issues early before wasting time on downstream tasks.

---

## What Are Tasks?

A **task** is a unit of work that:
- Takes 15-30 minutes to complete
- Has a single, clear acceptance criterion
- Depends on specific other tasks
- Produces one verifiable output (file, section, validated state)

### Task Properties

**Size**: 15-30 minutes
- Too small (under 10 minutes) = too many micro-tasks, checkpoint overhead
- Too large (over 45 minutes) = hard to review, hard to fix if wrong
- Just right (15-30) = meaningful progress, reviewable scope

**Criterion**: Single, testable
- "Research section 1 sources and verify credibility" ✅
- "Research section 1 AND outline section 1 AND find images" ❌ (three things)
- "Work on research stuff" ❌ (untestable)

**Independence**: Can be reviewed individually
- Doesn't require other tasks to be done first
- Or clearly depends on specific other tasks

**Clarity**: Defines exact acceptance criterion that signals completion
- ✅ "Section 1 has 5+ credible sources (peer-reviewed or authoritative), each with full citation"
- ❌ "Section 1 is researched"

---

## The Checkpoint Pattern (CRITICAL)

This is **the most important concept** in this lesson. The checkpoint pattern is how you maintain control of the workflow.

### Pattern Definition

```
Loop:
  1. Agent: "I've completed Phase X (description of output)"
  2. Human: "Review the work (output visible and testable)"
  3. Human: "APPROVE" → Commit to git
  4. Human: "Tell me next phase"
```

### Why Checkpoints Matter

**Without Checkpoints** (dangerous):
```
You: "Write my research paper"
Agent: "Done! I've completed 15 tasks, researched all sections,
        synthesized 50 sources, written full paper, formatted
        everything. All automated. You're welcome."
You: "Wait, which sources did you use? Is section 3 accurate?
      How do I verify what you wrote?"
Agent: "Already committed. Sorry! Check it now?"
```

**With Checkpoints** (controlled):
```
You: "Start research paper workflow"
Agent: "Phase 1 (Section 1 Research) complete:
        ✓ 5 credible sources identified
        ✓ Notes summarizing key points
        ✓ Citations formatted
        Ready for review."
You: "Read sources... all high-quality. Commit. What's next?"
Agent: "Phase 2 (Section 1 Outline) - Key points ordered..."
You: "Found issue with point sequence. Fixing..."
Agent: "Phase 3 (Section 2 Research) - Starting literature review..."
You: "Paper structure looks good so far. Paper complete!"
```

### Your Role in Each Checkpoint

**Step 1: Human Reviews**
- See the actual output (written section, bibliography, research notes)
- Ask: "Does this match the plan?"
- Ask: "Are there accuracy issues I should fix before continuing?"
- Ask: "Is this ready for the next phase?"

**Step 2: Human Decides**
- Approve ("Looks good, commit")
- Reject ("Fix this issue before continuing")
- Request clarification ("Where did you get this source?")

**Step 3: Human Directs**
- "What's the next phase?"
- You initiate next phase
- Agent doesn't autonomously continue

---

## Task Structure for Research Paper

Your research paper project breaks into **4 phases with 10 atomic tasks**. Here's the breakdown:

### Phase 1: Research Foundation (3 tasks, 45-60 minutes)

These tasks establish credible sources and research notes BEFORE writing.

**Task 1.1: Research Section 1 - Find Credible Sources**
- **Duration**: 20 minutes
- **Depends on**: Nothing
- **What to do**: Identify 5+ credible sources for Section 1 (topic introduction)
- **Acceptance**: "5+ sources identified; each is peer-reviewed OR from authoritative domain expert; full citations recorded"
- **Output**: Bibliography file with 5+ sources and notes on why each is credible

**Task 1.2: Research Section 1 - Synthesize Key Points**
- **Duration**: 15 minutes
- **Depends on**: Task 1.1
- **What to do**: Read sources and extract key points for Section 1
- **Acceptance**: "3-5 key points documented; each has source attribution; points relate directly to introduction goal"
- **Output**: Research notes file with key points and source citations

**Task 1.3: Create Outline Structure**
- **Duration**: 15 minutes
- **Depends on**: Task 1.2
- **What to do**: Draft outline for all sections with main points and sub-points
- **Acceptance**: "Outline has all 4+ sections; each section has 2-3 main points; structure flows logically"
- **Output**: Markdown outline file showing section hierarchy

### Phase 2: Content Research and Organization (4 tasks, 60-90 minutes)

These tasks research remaining sections and organize findings.

**Task 2.1: Research Section 2 - Find Credible Sources**
- **Duration**: 20 minutes
- **Depends on**: Task 1.3
- **What to do**: Identify 5+ credible sources for Section 2 (main topic)
- **Acceptance**: "5+ sources identified; each is peer-reviewed OR from domain expert; full citations recorded"
- **Output**: Bibliography update with Section 2 sources and credibility notes

**Task 2.2: Research Section 2 - Synthesize Key Points**
- **Duration**: 15 minutes
- **Depends on**: Task 2.1
- **What to do**: Read sources and extract key points for Section 2
- **Acceptance**: "4-6 key points documented; source attributed; points advance main argument"
- **Output**: Research notes update with Section 2 key points

**Task 2.3: Research Section 3 and Beyond - Find Sources**
- **Duration**: 20 minutes
- **Depends on**: Task 2.2
- **What to do**: Research remaining sections (conclusion, implications) and gather sources
- **Acceptance**: "All remaining sections have 3+ credible sources each; citations recorded"
- **Output**: Complete bibliography with all sections covered

**Task 2.4: Organize All Research Notes by Section**
- **Duration**: 15 minutes
- **Depends on**: Task 2.3
- **What to do**: Consolidate all research notes; organize by section; verify coverage
- **Acceptance**: "All sections have research notes; notes are organized by topic; no gaps identified"
- **Output**: Organized research notes file; verified coverage checklist

### Phase 3: Writing and Synthesis (2 tasks, 60-90 minutes)

These tasks transform research into written paper.

**Task 3.1: Write and Synthesize Content**
- **Duration**: 45 minutes
- **Depends on**: Task 2.4
- **What to do**: Write paper sections using research notes; synthesize findings
- **Acceptance**: "All sections written; each section 300+ words; citations embedded; argument flows"
- **Output**: Complete draft paper with citations

**Task 3.2: Format and Verify Citations**
- **Duration**: 20 minutes
- **Depends on**: Task 3.1
- **What to do**: Apply citation style; verify all sources cited; check bibliography completeness
- **Acceptance**: "All citations follow APA format; bibliography complete; in-text citations present"
- **Output**: Formatted paper with verified citations; complete bibliography

### Phase 4: Review and Finalization (1 task, 30 minutes)

These tasks validate final paper quality.

**Task 4.1: Review Paper Quality and Finalize**
- **Duration**: 30 minutes
- **Depends on**: Task 3.2
- **What to do**: Read complete paper; verify accuracy, clarity, argument coherence; finalize
- **Acceptance**: "Paper reads smoothly; argument is clear; sources are credible; no typos; ready to submit"
- **Output**: Final paper ready for submission; quality checklist completed

---

## Checkpoint Sequence

Here's how the workflow actually progresses with human checkpoints:

### Checkpoint 1: After Phase 1 (Research Foundation)
```
AGENT: "Research foundation complete:
        ✓ Section 1 sources identified (5 peer-reviewed articles)
        ✓ Key points synthesized from sources
        ✓ Outline structure created (4 sections, logical flow)

        Ready for Phase 2: Content Research"

YOU: "Review sources... all high-quality academic journals.
      Read outline... structure makes sense.
      Looks good! Committing Phase 1."

YOU: (git commit)

YOU: "Proceed to Phase 2"
```

### Checkpoint 2: After Phase 2 (Content Research)
```
AGENT: "Content research complete:
        ✓ Sections 2-4 sources identified (15+ total sources)
        ✓ Key points extracted and organized by section
        ✓ Research notes consolidated and verified for gaps

        Ready for Phase 3: Writing"

YOU: "Review research notes... coverage is comprehensive.
      Check bibliography... all sources have full citations.
      Committing Phase 2."

YOU: (git commit)

YOU: "Proceed to Phase 3"
```

### Checkpoint 3: After Phase 3 (Writing)
```
AGENT: "Writing complete:
        ✓ All sections written (2,500+ words total)
        ✓ Research synthesized into narrative
        ✓ Citations formatted in APA style
        ✓ Bibliography complete

        Ready for Phase 4: Finalization"

YOU: "Read draft... argument is clear and well-supported.
      Check citations... all formatted correctly.
      Committing Phase 3."

YOU: (git commit)

YOU: "Proceed to Phase 4"
```

### Checkpoint 4: After Phase 4 (Finalization)
```
AGENT: "Review and finalization complete:
        ✓ Paper reads smoothly, no formatting issues
        ✓ All sources properly cited
        ✓ Argument coherent from introduction to conclusion
        ✓ Quality gates: All pass

        PROJECT COMPLETE: Paper ready for submission"

YOU: "Final read-through... excellent quality.
      All requirements met. Committing final version.
      Project complete!"

YOU: (final git commit)
```

---

## Task Dependency Graph

Here's how your tasks depend on each other:

```
Phase 1 (Research Foundation): Sequential path (each depends on previous)
┌────────────────────────────────────────────────────────────┐
│  Task 1.1: Research Section 1 Sources                      │
│      ↓                                                      │
│  Task 1.2: Synthesize Section 1 Key Points                 │
│      ↓                                                      │
│  Task 1.3: Create Outline Structure                        │
│      ↓ [CHECKPOINT 1]                                      │
└────────────────────────────────────────────────────────────┘

Phase 2 (Content Research): Sequential path (each depends on previous)
┌────────────────────────────────────────────────────────────┐
│  Task 2.1: Research Sections 2+ Sources (depends on 1.3)   │
│      ↓                                                      │
│  Task 2.2: Synthesize Sections 2+ Key Points               │
│      ↓                                                      │
│  Task 2.3: Research Final Sections Sources                 │
│      ↓                                                      │
│  Task 2.4: Organize All Research by Section                │
│      ↓ [CHECKPOINT 2]                                      │
└────────────────────────────────────────────────────────────┘

Phase 3 (Writing): Linear path (each depends on previous)
┌────────────────────────────────────────────────────────────┐
│  Task 3.1: Write and Synthesize Content (depends on 2.4)   │
│      ↓                                                      │
│  Task 3.2: Format and Verify Citations                     │
│      ↓ [CHECKPOINT 3]                                      │
└────────────────────────────────────────────────────────────┘

Phase 4 (Finalization): Final review (depends on Phase 3)
┌────────────────────────────────────────────────────────────┐
│  Task 4.1: Review and Finalize (depends on 3.2)            │
│      ↓ [CHECKPOINT 4 - PROJECT COMPLETE]                   │
└────────────────────────────────────────────────────────────┘

Legend: Each task must complete before next starts (strict dependency)
        Checkpoints occur after each phase group
```

---

## Lineage Traceability

Can you trace a task back to specification? Try this one:

```
Specification: "Write 2,500+ word research paper on AI in education
               with academic rigor and clear argument structure"
  ↓
Plan: "Phase 1: Establish research foundation with credible sources;
       Phase 2: Organize research by section;
       Phase 3: Synthesize into written paper;
       Phase 4: Verify quality and finalize"
  ↓
Task 2.1: "Research Section 2 - Find Credible Sources"
  ↓
Acceptance Criterion: "5+ sources identified; each peer-reviewed OR
                       from domain expert; full citations recorded"
```

If you can trace this lineage for each task, your task breakdown is well-connected to your specification.

---

## Common Mistakes

### Mistake 1: Tasks Too Large (45+ Minutes)

**The Error**: "Task: Complete entire section research and writing (2+ hours)"

**Why It's Wrong**: Large tasks hide complexity, delay feedback, and make checkpoints meaningless. You can't validate progress until the entire section completes.

**The Fix**: Break into atomic units (15-30 minutes each):
- ❌ Large: "Research and write section 1"
- ✅ Atomic: "Find sources" (20 min), "Synthesize points" (15 min), "Outline structure" (15 min), "Write section" (45 min)

### Mistake 2: Combining Research and Writing

**The Error**: Task includes "research section, synthesize findings, write content, format citations" all as one task

**Why It's Wrong**: If you find issues with source credibility mid-task, you can't easily restart research without redoing writing. Mixing research + writing confuses where quality issues originate.

**The Fix**: Separate research from writing:
- Task 2.1: "Research Section 2 Sources" (automation/research)
- Task 2.2: "Synthesize Section 2 Points" (analysis)
- **CHECKPOINT**: Human reviews research quality before continuing
- Task 3.1: "Write and Synthesize Content" (composition)

### Mistake 3: Vague Acceptance Criteria

**The Error**: "Task: Section 1 is researched" (what does "researched" mean?)

**Why It's Wrong**: You won't know if the task is done or if there's a hidden gap.

**The Fix**: Make acceptance criteria specific and testable:
- ✅ "Section 1 has 5+ peer-reviewed sources AND each source has full citation AND notes summarize key points"
- ✅ "Paper has 2,500+ words AND all sources cited AND bibliography complete AND no formatting errors"

---

## What Makes /sp.tasks Powerful

The `/sp.tasks` command analyzes your specification and plan, then **generates a complete task breakdown** that includes:

1. **Atomic Unit Definition** - Each task is 15-30 minutes with one acceptance criterion
2. **Dependency Ordering** - Tasks ordered so dependencies are clear
3. **Checkpoint Placement** - Human review points between phases
4. **Lineage Traceability** - You can trace each task back to specification
5. **Acceptance Criteria** - Each task has specific, testable completion condition

You don't write tasks from scratch. **`/sp.tasks` writes them for you** based on your specification and plan. Your job is to **understand the task structure, validate it's atomic, and execute it with checkpoints**.

---

## Try With AI

Ready to validate your task breakdown and understand how `/sp.tasks` works? Test your understanding:

**Explore Task Atomicity:**
> "I'm using `/sp.tasks` to break my research paper into atomic work units. Review my task list: (1) Is each task atomic (does ONE thing with ONE acceptance criterion)? (2) Are they sized right (15-30 minutes, not hours or minutes)? (3) Can each be reviewed independently? (4) Identify any tasks that should be split further or combined. (5) Which tasks would you add or remove?"

**Practice Checkpoint Validation:**
> "Walk me through the checkpoint pattern for my research paper workflow. For each checkpoint (after Phases 1, 2, 3, 4): (1) What should I review for? (2) What makes a 'good' output at this checkpoint? (3) What issues could arise that I should catch before continuing? (4) Create a checklist I can use at each checkpoint to decide 'ready to proceed'."

**Analyze Dependencies:**
> "Examine my task dependencies: (1) Are they logically correct? (2) Which tasks could theoretically run in parallel? (3) What's the critical path (minimum sequence to completion)? (4) If one task failed (e.g., couldn't find enough credible sources), which downstream tasks would be affected? (5) How would I recover and restart?"

**Understanding /sp.tasks Command:**
> "Explain what `/sp.tasks` does: (1) What INPUT does it take (spec + plan)? (2) What OUTPUT does it generate (task list structure)? (3) How does it ensure tasks are atomic? (4) How does it order tasks by dependency? (5) When would you run `/sp.tasks` in a Spec-Kit Plus workflow?"

---
