## Tasks Phase - Atomic Work Units and Checkpoints

The `/sp.tasks` command transforms a specification and plan into a structured breakdown of atomic work units (15-30 minute tasks) with explicit dependencies and human-controlled checkpoints. This lesson teaches task atomicity (single acceptance criterion per task), the four-phase checkpoint pattern (Agent Complete → Human Review → Human Approve → Continue), and how to maintain human control throughout execution by requiring explicit approval at each phase boundary before proceeding to the next phase.

### Mental Models

- **Atomic Task Unit**: A 15-30 minute unit of work with a single, testable acceptance criterion that produces one verifiable output. Size constraint prevents tasks too small (micro-management) or too large (hidden complexity, delayed feedback, hard to fix mid-stream).

- **Checkpoint Pattern**: Agent → Review → Approve → Continue loop that keeps humans in control. Without checkpoints, agents can complete all tasks autonomously and you lose visibility. With checkpoints, you see each phase output, validate quality, catch issues early, and explicitly authorize the next phase before proceeding.

- **Dependency Graph**: Visual representation of task ordering showing which tasks must complete before others can start (strict sequential path for research paper: Phase 1 → Phase 2 → Phase 3 → Phase 4, with sub-task dependencies within each phase).

- **Lineage Traceability**: Ability to trace any task backward through the chain: Specification → Plan → Task → Acceptance Criterion. If traceability breaks, the task may be solving the wrong problem or may not be needed for specification fulfillment.

### Key Patterns

- **Four-Phase Task Structure**: Research Foundation (sources + outline) → Content Research (gather all materials) → Writing and Synthesis (transform research into prose) → Review and Finalization (quality validation). Each phase ends with human checkpoint.

- **Separation of Concerns**: Research tasks (find sources, extract key points) separate from writing tasks. This prevents quality issues in research from cascading through written content without human review between phases.

- **Sequential Dependency within Phase**: Each task in a phase depends on the previous task completing first. Task 1.1 (find sources) → Task 1.2 (synthesize points) → Task 1.3 (create outline), enabling strict checkpoints at phase boundaries.

- **Clear Acceptance Criteria**: "5+ sources identified; each peer-reviewed OR from domain expert; full citations recorded" (testable) vs. "Section 1 is researched" (untestable). Testable criteria are measurable and allow human validation.

- **Checkpoint Sequence**: Four explicit human approval points (after Phase 1, Phase 2, Phase 3, and Phase 4 completion) where human reviews output, validates against plan, and explicitly authorizes progression to next phase.

### Common Mistakes

- **Tasks Too Large (45+ minutes)**: Hiding complexity behind overly large tasks defeats the checkpoint pattern. Task "Research and write Section 1" (2+ hours) prevents human feedback until too much work has been done. Fix: Split into atomic units (find sources, synthesize points, outline, write content).

- **Combining Research and Writing**: Including "research section, synthesize findings, write content, format citations" in a single task prevents early quality validation of sources. If sources are poor, human discovers this too late (after writing has already started). Fix: Separate research tasks from writing tasks with a checkpoint between phases.

- **Vague Acceptance Criteria**: Criteria like "Section 1 is researched" leave hidden ambiguity about when task truly completes. Does "researched" mean 3 sources? 5? Peer-reviewed? Full citations? Fix: Make criteria specific and measurable: "5+ peer-reviewed sources, full citations, notes summarizing key points."

- **Missing Dependencies**: Treating all tasks as independent or assuming parallel execution when tasks actually have strict sequence requirements. Correct understanding of dependencies prevents attempting Task 2.1 before Task 1.3 completes, which would invalidate the outline structure needed for targeted research.

- **Skipping Checkpoints**: Letting agent autonomously complete multiple phases without human review ("Tell me everything") removes human control and prevents early issue detection. The checkpoint pattern explicitly requires human approval between phases.

### Progression Context

- **Builds on**: Lesson 6 (Plan Phase) taught how to structure implementation approach through phases, timelines, and strategies. This lesson decomposes that plan into executable atomic work units with clear sequencing and human approval gates.

- **Leads to**: Lesson 8 (Execution Phase) teaches how to actually execute the tasks with the `/sp.implement` command, monitoring progress through the checkpoint pattern, and handling task failures or refinements discovered during human review.

- **Within SDD-RI Workflow**: `/sp.tasks` is the bridge between planning (intent, approach) and execution (atomic work units, checkpoints). Spec → Plan → Tasks → Implement → Validate forms the complete workflow.

- **Requires**:
  - Understanding of specification structure (Chapter 12, Lesson 1)
  - Understanding of plan structure and phases (Lesson 6)
  - Ability to trace lineage from specification through plan to tasks (validation skill)
  - Familiarity with acceptance criteria as measurable completion signals (Lesson 6)

- **Prepares for**:
  - Executing tasks using `/sp.implement` with checkpoint validation (Lesson 8)
  - Creating skills and subagents that orchestrate task sequences autonomously (Chapter 16+)
  - Recognizing when tasks are too large or unclear and requesting refinement from `/sp.tasks`
