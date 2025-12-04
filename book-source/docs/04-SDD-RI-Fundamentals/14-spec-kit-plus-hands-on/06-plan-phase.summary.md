# Plan Phase — Architecture Decisions and ADRs

This lesson teaches students to transform specifications into implementation plans using `/sp.plan`, then document key architectural decisions using `/sp.adr`. Students run the plan command, review the generated architecture (components, phases, dependencies), and create ADRs for decisions that have long-term impact, multiple alternatives, and will be questioned later. The lesson emphasizes the cascade effect: clear spec → clear plan. Plans answer HOW we'll build what the spec defines, bridging intent to executable tasks.

### Mental Models

- **Cascade Effect**: Specification quality determines plan quality. Detailed spec → clear plan with well-defined components. Vague spec → vague plan that exposes missing requirements. Plans act as quality checkpoint.

- **Three-Layer Hierarchy**: Spec defines WHAT (success criteria), Plan defines HOW (architecture), Tasks define WORK UNITS (15-30 min pieces). Each layer adds specificity. Plans prevent jumping from abstract requirements to granular tasks.

- **ADR as Decision Documentation**: Architectural Decision Records capture WHY choices were made, not just WHAT was decided. Include context, alternatives considered, rationale, and consequences (both positive and negative).

- **ADR Significance Test**: Create ADR only if: (1) long-term consequences, (2) multiple viable alternatives existed, (3) someone will question it later. If not all three → skip the ADR.

### Key Patterns

- **Run `/sp.plan` with context**: Provide architecture sketch, section structure, research approach, quality validation strategy. Agent generates plan.md with components, dependencies, phases.

- **Review generated plan**: Check that plan maps to all spec requirements, shows clear dependencies, breaks work into logical phases, identifies parallel opportunities.

- **Run `/sp.adr` to document decisions**: Focus on decisions affecting multiple sections, with considered alternatives, that shape how work proceeds. Agent creates ADRs in `history/adr/`.

- **ADR Structure**: Status → Context (why decision needed) → Decision → Alternatives Considered (with pros/cons) → Rationale (why this over others) → Consequences (positives AND negatives).

- **Spec → Plan → Tasks chain**: Each level adds specificity. Spec: measurable success criteria. Plan: architecture + ADRs. Tasks: 15-30 minute atomic units.

### Common Mistakes

- **Documenting trivial decisions as ADRs**: Creating ADRs for "use headings for sections" or style choices. Apply three-part test: long-term consequences? multiple alternatives? future questioning?

- **Vague ADR consequences**: "This approach is better" without tradeoffs. Document both positives AND negatives. Show what you gave up.

- **Skipping plan phase**: Jumping from spec to tasks loses architecture visibility. Tasks become disconnected, structural problems emerge late.

### Progression Context

- **Builds on**: Lesson 5 (Clarify Phase) where students refined specifications. Students now have complete, detailed specs ready for planning.

- **Leads to**: Lesson 7 (Tasks Phase) where students break the plan into atomic 15-30 minute work units. Plan's components and phases structure the task breakdown.
