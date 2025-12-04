# Implement Phase — Execute Tasks with AI Collaboration

Implementation transforms your specification and plan into reality by executing tasks with measurable validation. This lesson teaches that implementation is orchestrated execution with human decision-making at checkpoints—not autonomous code generation. The core skill is validating task completion against specification success criteria at critical phase boundaries, using the `/sp.implement` command to coordinate AI execution with your judgment.

### Mental Models

- **Implementation ≠ Autonomous Execution**: Implementation is you directing strategy, AI handling execution, both validating results. You maintain control through checkpoints, not by walking away and hoping the AI completes the project autonomously.

- **Specification as Acceptance Standard**: Your spec defines "done" before implementation begins. Success criteria (word counts, requirements, quality measures) become objective measures of task completion. A task meets spec when its output satisfies all predefined criteria; iteration refines tasks that don't meet criteria.

- **Checkpoint Pattern (Control Mechanism)**: Work executes in phases separated by checkpoints. At each checkpoint, you answer one question—"Does this meet the specification?"—then decide: commit and proceed, iterate the current task, or revise the plan if checkpoint reveals structural issues. Checkpoints prevent cascading failures from undetected task deficiencies.

- **Iteration as Solution Pattern**: When tasks don't meet spec, iteration (not abandonment) solves the problem. AI refines output based on your feedback guided by the spec. Iteration transforms task failures into productive learning—spec shows what's missing, you guide adjustment, AI re-executes, you re-validate.

### Key Patterns

- **Spec→Prompt→Execute→Validate Cycle**: Tasks execute within a structured cycle where you specify requirements from tasks.md, AI proposes execution approach, you refine direction, AI executes, you validate against spec success criteria, cycle repeats or proceeds based on checkpoint decision.

- **Checkpoint Decisions (Three Options)**: At each checkpoint you choose: (1) "Commit and Proceed"—all tasks meet spec, move to next phase; (2) "Iterate on This Task"—specific task needs refinement, provide targeted feedback based on spec gaps; (3) "Revise the Plan"—checkpoint revealed structural issue requiring plan adjustment (rare, reserved for systematic problems).

- **Serial Task Execution**: Tasks execute in dependency order (Task 1 → Task 2 → Task 3 → Checkpoint). Each task builds on previous output. You validate after each task block to ensure quality before next block uses output as foundation.

- **Parallel Task Execution**: Independent tasks execute simultaneously (Task 1.1, 1.2, 1.3 in parallel → Checkpoint). Reduces time when tasks have no dependencies. Requires checkpoint validation to ensure parallel outputs don't create integration conflicts.

- **Iterative Refinement Pattern**: Task executes, checkpoint review identifies gap, task re-executes with refinement, review validates against spec, process repeats until specification met. Used for complex work where polished first draft exists but needs targeted improvement.

- **AI Collaboration During Execution**: AI doesn't just execute blindly—it collaborates with suggestion→refinement→execution→validation cycle. AI proposes approach (suggesting pattern you might not discover), you refine based on project constraints, AI adapts and executes revised version, you validate outcome.

### Common Mistakes

- **Approving Without Review**: Checkpoint appears and you say "Looks good" without validating against specification success criteria. Skipping review defeats checkpoint purpose—specs guide validation; validation ensures spec-aligned quality. Always spend 2-5 minutes validating each checkpoint.

- **Accepting "Close Enough"**: Spec requires 500-700 words, task delivers 780 words. You move on without iteration because it's "close enough." This is how requirements creep occurs. Specs exist to prevent creep—enforce them at checkpoints through iteration when tasks exceed or fall short of criteria.

- **Not Iterating When First Pass Fails**: Task doesn't meet spec, and you either give up or accept the failed output. Iteration is normal and productive. When a task misses spec, identify the specific gap using specification criteria, provide targeted feedback to AI, re-execute with refinement, validate again. This convergence is how implementation works.

- **Attempting Autonomous Execution**: You run `/sp.implement` and let the agent execute all tasks without checkpoint review. This eliminates human judgment and risks cascading failures. Instead, wait at each checkpoint, validate against spec, make informed decision, then proceed.

- **Missing Dependencies in Validation**: Checkpoint review checks individual task quality but doesn't verify that downstream tasks can safely build on this output. Always ask: "Can the next task safely use this output? Are there integration risks?" This prevents downstream failures.

- **Conflating Iteration with Plan Revision**: When a task doesn't meet spec, iteration refines that task. When checkpoint reveals a systematic planning problem (word budget overflow, structural mismatch), plan revision adjusts downstream tasks. Confusing these patterns leads to excessive plan changes instead of targeted task refinement.

### Progression Context

- **Builds on**: Lesson 7 (Plan Phase) where you created atomic tasks with dependencies and success criteria. That plan becomes the roadmap `/sp.implement` executes. You needed clear task definitions and dependencies before implementation can proceed efficiently.

- **Leads to**: Validation and completion workflows where you systematically verify that final implementation meets specification success evals (Lesson 9, Validation Phase). Implementation executes tasks; validation verifies that accumulated task outputs fulfill the original specification intent.
