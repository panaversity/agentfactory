### Core Concept
Single agents hit limits. Four patterns cover how multiple specialists work together: Coordinator (parallel), Sequential (pipeline), Iterative Refinement (feedback loops), and Human-in-the-Loop (approval gates). Match the pattern to your problem structure.

### Key Mental Models
- **Pattern → Problem Structure**: Independent subtasks → Coordinator; ordered steps → Sequential; quality-critical → Iterative Refinement; high-stakes → HITL
- **Patterns Combine**: Real systems use multiple patterns—Coordinator for routing, Sequential within workflows, HITL for escalations
- **Wrong Pattern → Failure Modes**: Mismatched patterns don't just underperform, they create systematic failures

### Critical Patterns
- **Coordinator**: One routes, many execute in parallel. Use when subtasks are independent.
- **Sequential**: Each output feeds the next. Use when order matters (can't test before writing code).
- **Iterative Refinement**: Generator + Critic loop until quality threshold met. Design max iterations and escalation paths.
- **Human-in-the-Loop**: Agent pauses for human approval. Design clear triggers (e.g., refunds >$500).

### Common Mistakes
- Using Coordinator when results depend on each other (breaks parallelization)
- Using Sequential when tasks could parallelize (wastes time)
- Iterative Refinement without max iterations (infinite loops) or quality threshold (never terminates)
- HITL gates too aggressive (humans review everything) or too permissive (risky actions proceed unchecked)

### Connections
- **Builds on**: 5-Step Loop (Lesson 3)—each agent runs its own loop within the pattern
- **Leads to**: Agent Ops (Lesson 5)—how to evaluate and debug multi-agent systems
