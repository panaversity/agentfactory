## Lesson Title

A plan bridges specification (WHAT are we building?) to tasks (WHAT are the atomic work units?) by answering HOW we'll build it. The plan identifies major components, their dependencies, phases of work, and the overall architecture. The `/sp.plan` command automates this translation: it reads a clear specification and generates a comprehensive plan showing approach, components, dependencies, and implementation phases. Critically, plan quality cascades from specification quality—clear specs produce clear plans, while vague specs expose themselves as inadequate plans.

### Mental Models

- **Three-Layer Artifact Hierarchy**: Specification defines WHAT (intent, constraints, success criteria), Plan defines HOW (architecture, components, sequencing), Tasks define atomic WORK UNITS (15-30 minute units). Each layer increases specificity. Plans sit between intent and execution, preventing the jump from abstract requirements directly to granular tasks.

- **Cascade Effect**: Specification quality determines plan quality. A detailed, specific specification produces a clear plan with well-defined components and dependencies. A vague specification produces a vague plan that exposes missing constraints. Plans act as a quality checkpoint—if your plan is unclear, your specification needs refinement.

- **Component-Dependency Graph**: Plans map logical parts (components) and their blocking relationships (dependencies). Some dependencies are mandatory (you must research before analyzing findings), others are optional (you could parallelize work). This graph makes dependencies explicit before task execution.

- **Architecture vs. Execution**: Plans are architectural (high-level strategy, major components, phases). Tasks are operational (individual, atomic units). Confusing these layers creates either overly detailed plans (every small step listed) or incompletely detailed tasks (missing connections to overall strategy).

### Key Patterns

- **Spec → Plan → Tasks Progression**: Each SDD-RI phase adds specificity. Specification establishes success criteria (measurable outcomes). Plan establishes architecture (how to organize the work to meet those criteria). Tasks establish atomic units (individual work broken into 15-30 minute pieces).

- **Approach as Strategic Choice**: Every plan embeds architectural decisions about HOW to solve the problem. For a research paper: research-first vs. research-concurrent approach; sequential sections vs. iterative refinement; all-upfront outlining vs. outline-as-you-go. These decisions shape dependencies and phases downstream.

- **Dependency Identification**: Plans must identify which work blocks other work. Some dependencies are logical (you can't write Discussion without Findings), others reflect knowledge flow (you need Methodology to contextualize Findings). Making these explicit prevents rework.

- **Phase-Based Milestones**: Implementation phases mark major checkpoints (Phase 1: Research & Structure, Phase 2: Foundation Writing, Phase 3: Analysis Writing, Phase 4: Synthesis & Polish). Milestones allow review and course-correction without atomic task-level visibility.

- **Quality Gate Identification**: Plans identify validation points—how you'll know the work is proceeding correctly. For a research paper: source count verification, word count verification, APA format validation. Quality gates prevent downstream surprises.

### Common Mistakes

- **Confusing Plan with Task List**: Treating plans as detailed task checklists instead of high-level architecture. Plans show major components and phases, not every small step. If something is small, it belongs in the task layer, not the plan layer.

- **Breaking Logical Dependencies**: Creating plans where downstream work precedes required knowledge. Example: Findings before Literature Review (you can't present insights without having reviewed research). Dependencies should reflect actual work flow causality.

- **Skipping Planning Entirely**: Jumping from specification directly to tasks, losing sight of overall architecture. This causes tasks to become disconnected, structural problems emerge late, and rework multiplies. Always plan—`/sp.plan` makes it quick.

- **Misaligning Plan to Specification**: Creating plans that don't directly trace back to specification requirements. A good plan ensures every spec requirement maps to a specific plan component or phase. If a spec requirement has no plan equivalent, the spec needs clarification or the plan is incomplete.

- **Over-Specificity in Plan**: Including implementation details (exact variable names, function signatures, specific syntax) that belong in tasks or code. Plans should be approach and architecture, not implementation.

### Progression Context

- **Builds on**: Lesson 5 (Clarifying specifications) - students have now completed a clear, detailed specification and understand specification quality. They recognize that vague specs lead to vague plans. They understand the SDD-RI workflow: Spec → Plan → Tasks.

- **Leads to**: Lesson 7 (Tasks phase) - students will take the plan just created and break it into atomic, 15-30 minute work units. The plan's components and phases will structure the task breakdown. Students will learn that plans prevent orphaned tasks and ensure every task contributes to a major phase objective.
