## Core Concept
Memory files (CLAUDE.md, architecture.md, decisions.md) are persistent documentation stored in your project that survive across sessions, eliminating re-explanation of architectural patterns and enabling seamless continuity in multi-day development.

## Key Mental Models
- **Persistent context vs. session context**: Session context disappears, persistent context survives restarts
- **Three-layer memory architecture**: Conventions (CLAUDE.md) + System structure (architecture.md) + Decision rationale (decisions.md) serve complementary purposes
- **Update triggers**: Different files updated for different events (patterns discovered → CLAUDE.md, architecture changes → architecture.md, decisions made → decisions.md)
- **Conflict resolution through versioning**: Breaking changes spawn new ADRs; non-breaking changes update in place
- **Memory file as session primer**: Load files at session start to restore full architectural context without verbal re-explanation

## Critical Patterns
- **CLAUDE.md construction**: Project conventions, naming rules, anti-patterns, security constraints (what NOT to do)
- **architecture.md structure**: Component relationships, key files map, design patterns used, system properties
- **ADR (Architectural Decision Record) pattern**: Decision + Context + Rationale + Alternatives Considered + Consequences = testable reasoning trail
- **Persistence workflow**: Create once → load every session → update when understanding evolves
- **Supersession pattern**: Old ADR marked "SUPERSEDED" when decision reversal; new ADR added (preserves history)

## AI Collaboration Keys
- AI reads memory files at session start → understands your architectural decisions without re-explanation
- Memory files prevent "suggestion contradiction" (Claude suggesting bcrypt when project decided Argon2)
- AI can reason about constraints from day one because memory files encode project-specific knowledge
- Reduces context pollution (AI less likely to confuse patterns from different projects)

## Common Mistakes
- Creating memory files but not loading them in new sessions (defeats persistence)
- Treating memory files as read-only (should update after every session with new decisions)
- Confusing ADR versioning (trying to edit old decision rather than marking superseded + adding new)
- Over-documenting (including trivial decisions clogs file; focus on architectural significance)
- Failing to update when understanding changes (stale memory files become misleading)

## Connections
- **Builds on**: Lesson 4 (checkpoints as temporary memory) extended to permanent, project-wide memory
- **Leads to**: Lesson 7 (memory files are primary context source before considering tool selection), Lesson 9 (capstone orchestrates memory file reading)
- **Related skill**: Skills/memory-file-architecture (reusable pattern for any multi-session project)
