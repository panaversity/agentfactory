---
description: Execute the implementation planning workflow with context-aware agent routing. Routes to chapter-planner for content work, general-purpose for engineering/platform work.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Core Directive

**Context-Aware Routing**: This command analyzes the specification to determine work type and routes to the appropriate planning agent:
- **Content Work** (lessons, modules, chapters) → `chapter-planner` subagent
- **Engineering Work** (features, APIs, components) → `general-purpose` subagent
- **Platform Work** (auth, RAG, infrastructure) → `general-purpose` subagent

**WHY**: Different work types require different planning expertise. Educational content needs pedagogical arc planning. Engineering needs architectural decomposition. Platform needs infrastructure sequencing.

**Agent Discovery**: Before routing, check `.claude/agents/` for current agent inventory. Agent names below are examples—always verify what's actually available.

## Outline

1. **Setup**: Run `.specify/scripts/bash/setup-plan.sh --json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH.

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template.

3. **Classify Work Type**: Analyze spec to determine routing:

   ```
   CLASSIFICATION SIGNALS:

   CONTENT (→ chapter-planner):
   - spec mentions: lesson, module, chapter, exercise, learning objectives
   - spec references: pedagogical, teaching, students, proficiency
   - spec includes: 4-layer framework, hardware tiers for content

   ENGINEERING (→ general-purpose):
   - spec mentions: feature, endpoint, API, component, service
   - spec references: architecture, database, frontend, backend
   - spec includes: technical requirements, integrations

   PLATFORM (→ general-purpose):
   - spec mentions: auth, RAG, deployment, CI/CD, infrastructure
   - spec references: Better-Auth, Qdrant, Cloud Run, Neon
   - spec includes: system architecture, scaling, security
   ```

4. **Route to Appropriate Planner**:

   ### For CONTENT Work (chapter-planner)

   ```
   Use Task tool with:
   - subagent_type: "chapter-planner"
   - prompt: Include spec path, constitution reference, and:
     - Pedagogical arc requirement (Foundation → Mastery)
     - Layer progression mapping (L1 → L2 → L3 → L4)
     - Hardware tier requirements per lesson
     - Teaching modality variation from previous content
     - Cognitive load limits by proficiency tier
   ```

   **chapter-planner Output**:
   - Lesson structure with learning objectives
   - Stage identification per lesson (1/2/3/4)
   - Teaching modality selection with rationale
   - Hardware tier requirements and fallbacks
   - Intelligence creation opportunities (skills/subagents)
   - Capstone composition strategy

   ### For ENGINEERING/PLATFORM Work (general-purpose)

   ```
   Use Task tool with:
   - subagent_type: "general-purpose"
   - prompt: Include spec path, constitution reference, and:
     - Technical architecture requirements
     - Component decomposition
     - Dependency ordering
     - Test strategy
     - Integration points
   ```

   **general-purpose Output**:
   - Technical architecture decisions
   - Component breakdown with dependencies
   - Implementation sequence
   - Test coverage plan
   - Integration strategy

5. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate appropriate artifacts based on work type
   - Re-evaluate Constitution Check post-design

6. **Stop and report**: Command ends after planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Work-Type-Specific Artifacts

### CONTENT Work Artifacts

| Artifact | Purpose |
|----------|---------|
| `plan.md` | Lesson structure, pedagogical progression |
| `research.md` | Technical accuracy verification |
| Optional: `data-model.md` | If content includes data structures |

**Content Plan Requirements**:
- Lesson count justified by concept density
- Pedagogical arc: Foundation → Application → Integration → Validation → Mastery
- Hardware tiers marked per lesson (Tier 1 always supported)
- Teaching modality varied from previous content
- Stage progression explicit (L1/L2/L3/L4)
- Intelligence creation identified (skills/subagents)

### ENGINEERING/PLATFORM Work Artifacts

| Artifact | Purpose |
|----------|---------|
| `plan.md` | Architecture, components, sequence |
| `research.md` | Technology decisions |
| `data-model.md` | Entities and relationships |
| `contracts/` | API specifications |
| `quickstart.md` | Integration scenarios |

**Engineering Plan Requirements**:
- Component decomposition with clear boundaries
- Dependency graph for build order
- Test strategy (unit, integration, e2e)
- Integration points identified
- Error handling approach

## Hardware Tier Awareness (Content Only)

For content planning, ensure each lesson specifies:

```markdown
### Lesson N: [Title]

**Hardware Tier**: Tier [1-4]
**Fallback**: [Yes/No - if Yes, describe Tier 1 alternative]
**Simulation-First**: [Yes/No - for robotics/physical content]
```

**Tier Requirements**:
- Tier 1 (Laptop/Cloud): Browser, MockROS, Pyodide
- Tier 2 (RTX GPU): Local Isaac Sim, Gazebo
- Tier 3 (Jetson Edge): Real sensors, edge deployment
- Tier 4 (Physical Robot): Unitree Go2/G1

## Cross-Book Intelligence Check

Before finalizing plan, assess:

```
INTELLIGENCE ACCUMULATION:
- Patterns that apply to future books: [list]
- Patterns specific to THIS book only: [list]
- Skills to create (platform-level): [list]
- Skills to create (domain-level): [list]
- Knowledge files needed: [list]
```

## Key Rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications
- Route to chapter-planner for ANY educational content
- Route to general-purpose for engineering/platform work
- Always check hardware tier requirements for content

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record).

1) Determine Stage: `plan`

2) Generate Title and Determine Routing:
   - Route: `history/prompts/<feature-name>/`

3) Create and Fill PHR:
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage plan --feature <name> --json`
   - Fill placeholders with plan summary

4) Validate + report
