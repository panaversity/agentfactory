## Core Concept
Specification-driven design shifts focus from **how to build** (code-first) to **what to build** (intent-first). A complete specification answers all critical questions (When? What? How measured?) without prescribing implementation, enabling someone unfamiliar with the problem to build the correct system independently.

## Key Mental Models
- **Intent vs. implementation separation**: Specification describes WHAT (behaviors, constraints, success criteria); implementation describes HOW (technologies, algorithms, code)
- **Specification completeness criteria**: Can someone build this without asking clarifying questions? If no → specification is incomplete
- **Falsifiability principle**: Success criteria must be measurable and testable (not "works well" but "80%+ sessions under 70% utilization")
- **Component responsibility pattern**: Each component has single responsibility (Separation of Concerns) + specified inputs/outputs/algorithm
- **Progressive disclosure of complexity**: Intent (1-2 paragraphs) → Success Criteria (5-10 measurable outcomes) → Functional Requirements (testable behaviors) → System Architecture (components + interactions) → Algorithms (decision logic)

## Critical Patterns
- **Vague vs. implementation-ready specs**: Vague specs (e.g., "helps manage context") leave implementers guessing; ready specs specify observable behaviors with triggers and outputs
- **Six-component orchestration pattern**: Context Monitor (what's broken), Checkpoint Engine (quick fix), Task Similarity Analyzer (prevention), Memory File Manager (persistence), Tool Selector (resource optimization), Orchestrator (coordinator)
- **Algorithm specification without code**: Decision logic written in prose/pseudocode, not Python/JavaScript (e.g., "IF utilization > 80% AND session > 60min THEN create checkpoint")
- **Completeness checklist validation**: Intent clarity, measurable success, testable requirements, architectural completeness, algorithm specification, multi-agent coordination, length (3-5 pages)
- **Peer review validation**: Can peer build system without asking questions? Identifies ambiguities before implementation begins

## AI Collaboration Keys
- Specification-driven design is fundamentally human-first (human defines intent/constraints, AI implements)
- Specification separates intent from implementation → enables AI tool switching (Gemini explores feasibility, Claude implements details)
- Clear specs prevent AI interpretation ambiguities ("What counts as high utilization?" is answered in spec, not inferred)
- Peer review of spec before implementation catches misunderstandings early (cheaper than implementation rework)

## Common Mistakes
- Including implementation details in spec ("use Redis for caching" vs. "system must load context in <2 seconds")
- Writing vague success criteria ("tool is reliable" vs. "zero context-related failures in 100+ test sessions")
- Failing to specify algorithm decision logic (leaves componentbuilders guessing at boundary conditions)
- Ignoring error cases (spec doesn't address "what if memory file is corrupted?")
- Spec too short (incomplete) or too long (loses focus with filler)

## Connections
- **Builds on**: All Lessons 1-8 teach individual techniques; this lesson orchestrates them into coherent system design
- **Capstone validation**: Specification demonstrates mastery—must demonstrate understanding of all 8 lesson concepts integrated
- **Workflow integration**: Spec-first approach is Layer 4 teaching framework (know intent before coding; implement from spec)
- **Reusability pattern**: Spec becomes contract between designer and implementer; enables team collaboration and code handoff
