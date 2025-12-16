# Constitution Phase — Project-Wide Quality Standards

This lesson teaches students how to create a Constitution using `/sp.constitution`—a document that defines project-wide quality standards applying to ALL work in a project. Students run the command, review the generated constitution, improve it with testable criteria, then commit it to git. The key distinction: Constitution applies to ALL papers (global rules), Specification applies to ONE paper (feature-specific requirements). Students learn that weak constitutions produce vague downstream work, while strong constitutions cascade quality through every phase.

### Mental Models

- **Constitution vs Specification**: Constitution = global rules for ALL features ("all papers must cite primary sources, APA format, zero plagiarism"). Specification = rules for ONE feature ("this paper's thesis is X, due December 15"). Constitution written once; Specification written per feature.

- **The Cascade Effect**: Clear Constitution → Clear Specification → Clear Plan → Clear Tasks → Quality Implementation. Each phase respects Constitution constraints without re-specifying them. Weak Constitution produces specs without citation requirements, plans without verification steps, papers with uncited claims.

- **Testability Principle**: Standards must be measurable, not subjective. ❌ Vague: "Papers should be well-written." ✅ Testable: "Flesch-Kincaid grade 10-12; active voice 75%+ of time; all claims cited."

### Key Patterns

- **Run `/sp.constitution` with project requirements**: Provide core principles, key standards, constraints, and success criteria. Agent generates comprehensive constitution file at `.specify/memory/constitution.md`.

- **Review for testability**: After generation, check: Are all standards testable (not vague)? Are essential categories covered (citation, verification, clarity, plagiarism)? Are any standards unrealistic?

- **Commit before feature work**: Always `git commit` the Constitution BEFORE running `/sp.specify`. This establishes Constitution as documented standard with full traceability.

- **Constitution-First Workflow**: Initialize project → Write Constitution (once) → Commit to git → FOR EACH PAPER: `/sp.specify` → `/sp.clarify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` → commit.

### Common Mistakes

- **Copying Constitution without customization**: Using example Constitution as-is. A code project Constitution mentions "type hints"—irrelevant for papers. Read examples for structure, write rules specific to YOUR project.

- **Vague quality standards**: "Papers must be good quality" or "Sources should be credible." These are subjective and unverifiable. Use testable criteria: "Flesch-Kincaid grade 10-12; all claims cited; zero plagiarism."

- **Forgetting to commit Constitution**: Creating Constitution then starting `/sp.specify` without committing. Constitution becomes "whatever I remember" instead of documented standard. Always commit BEFORE starting feature work.

### Progression Context

- **Builds on**: Lesson 2 (Installation and Setup) where students installed Spec-Kit Plus and initialized their project. The constitution file from `specifyplus init` is a starter template—this lesson expands it.

- **Leads to**: Lesson 4 (Specification Phase) where students run `/sp.specify` to write requirements for their first paper. The Specification must respect Constitution standards without re-specifying them.
