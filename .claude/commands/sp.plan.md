---
description: Execute the implementation planning workflow using the plan template to generate design artifacts.
handoffs: 
  - label: Create Tasks
    agent: sp.tasks
    prompt: Break the plan into tasks
    send: true
  - label: Create Checklist
    agent: sp.checklist
    prompt: Create a checklist for the following domain...
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
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/bash/update-agent-context.sh claude`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
