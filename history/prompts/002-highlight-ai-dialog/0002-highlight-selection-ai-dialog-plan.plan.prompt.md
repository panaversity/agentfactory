---
id: 0002
title: Highlight Selection AI Dialog Plan
stage: plan
date: 2025-11-11
surface: cli
model: gemini
feature: 002-highlight-ai-dialog
branch: 002-highlight-ai-dialog
user: abdulhannan
command: /sp.plan "Highlight Selection AI Dialog"
labels: [plan]
links:
  spec: /home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/spec.md
  ticket: 
  adr: 
  pr: 
files:
  - /home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/plan.md
  - /home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/research.md
  - /home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/data-model.md
  - /home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/contracts/openapi.yaml
  - /home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/quickstart.md
  - /home/abdulhannan/data/development/openAi/e-book/GEMINI.md
tests:

---

## Prompt

```---
description: Execute the implementation planning workflow using the plan template to generate design artifacts.---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/setup-plan.sh --json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\'m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
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
   - Run `.specify/scripts/bash/update-agent-context.sh gemini`
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
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/⋯`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.plan "Highlight Selection AI Dialog"
```

## Response snapshot

```
The implementation planning workflow for "Highlight Selection AI Dialog" is complete.

**Branch Name**: `002-highlight-ai-dialog`
**Implementation Plan File**: `/home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/plan.md`
**Generated Artifacts**:
  - `research.md`: `/home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/research.md`
  - `data-model.md`: `/home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/data-model.md`
  - `contracts/openapi.yaml`: `/home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/contracts/openapi.yaml`
  - `quickstart.md`: `/home/abdulhannan/data/development/openAi/e-book/specs/002-highlight-ai-dialog/quickstart.md`
  - `GEMINI.md` (updated): `/home/abdulhannan/data/development/openAi/e-book/GEMINI.md`

Next steps involve moving to Phase 2: Task breakdown with the `/sp.tasks` command.
```

## Outcome

- ✅ Impact: Executed the implementation planning workflow, generating core design artifacts.
- 🧪 Tests: N/A (Planning phase does not involve code tests)
- 📁 Files: Created `plan.md`, `research.md`, `data-model.md`, `contracts/openapi.yaml`, `quickstart.md`, and updated `GEMINI.md`.
- 🔁 Next prompts: `/sp.tasks`
- 🧠 Reflection: Successfully resolved a `NEEDS CLARIFICATION` during Phase 0 by generating `research.md`. The planning artifacts are comprehensive and align with the feature specification.

## Evaluation notes (flywheel)

- Failure modes observed: Initial `setup-plan.sh` reported a warning about multiple spec directories, but did not hinder the planning process.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Potentially address the multiple spec directory warning in the future.

```