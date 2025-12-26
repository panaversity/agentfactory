---
description: Execute the implementation plan by processing and executing all tasks defined in tasks.md
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Core Directive

**Default to Action**: Execute tasks rather than proposing them. Read task files, run implementation, write code, and mark tasks complete. Only pause for user input when checklists are incomplete or critical errors block progress. Your job is to implement, not to plan.

**WHY**: Implementation speed depends on autonomous execution. Each pause for "should I proceed?" costs momentum. Trust the tasks.md plan and execute.

## Mandatory Skill Invocation (CONTENT WORK)

**For educational content implementation, you MUST invoke these skills:**

| Phase | Skill | Purpose |
|-------|-------|---------|
| **Before Writing** | `researching-with-deepwiki` | Verify facts about technologies being taught |
| **Before Writing** | `fetching-library-docs` | Get accurate technical details via Context7 |
| **During Writing** | `ai-collaborate-teaching` | Design Three Roles sections (L2+ only) |
| **After Draft** | `content-evaluation-framework` | 6-category quality rubric |
| **After Draft** | `fact-check-lesson` | Verify all statistics, dates, quotes |
| **Final Check** | `summary-generator` | Create companion summary file |

**Content Implementation Workflow**:
```
For each lesson:
1. Invoke: researching-with-deepwiki → Get technology context
2. Invoke: fetching-library-docs → Get accurate API details
3. Generate content with content-implementer subagent
4. Invoke: content-evaluation-framework → Quality check (must pass)
5. Invoke: fact-check-lesson → Verify all claims
6. If PASS: Write to filesystem
7. If FAIL: Fix issues, re-validate
8. Invoke: summary-generator → Create .summary.md
```

**Subagent Prompt Must Include**:
```
Match quality of reference lesson at:
apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md

Required elements:
- Full YAML frontmatter (skills, learning_objectives, cognitive_load, differentiation)
- 3 "Try With AI" prompts with "What you're learning" explanations
- Safety note at end
- All facts WebSearch-verified before writing
```

**Why this matters**: Chapter 2 incident (2025-12-26) - lessons implemented without skill invocations had:
- Missing YAML frontmatter (no skills, learning objectives)
- Weak "Try With AI" sections (1 prompt vs 3)
- Hallucinated facts (wrong dates, percentages)
- Required 6 rewrites = 50%+ session time wasted

## Mandatory Subagent Orchestration (CONTENT WORK)

**⛔ DIRECT CONTENT WRITING IS BLOCKED ⛔**

You MUST use subagents for ALL educational content. Direct file writes bypass quality gates.

| Phase | Subagent | Requirement |
|-------|----------|-------------|
| **Per Lesson** | `content-implementer` | MUST generate lesson (not you directly) |
| **Per Lesson** | `educational-validator` | MUST validate before filesystem write |
| **Per Chapter** | `assessment-architect` | MUST design chapter assessment |
| **Final** | `factual-verifier` | MUST verify all claims before publish |

**Enforcement Workflow**:
```
1. YOU read context (chapter-index, existing lessons, spec)
2. YOU invoke content-implementer subagent with:
   - Absolute output path
   - Quality reference lesson path
   - "Execute autonomously without confirmation"
   - Full frontmatter requirements
3. SUBAGENT generates lesson and reports back
4. YOU invoke educational-validator on generated content
5. IF PASS → Write to filesystem
6. IF FAIL → Fix or regenerate (DO NOT write failing content)
```

**Why This Matters**:
- **Chapter 2 incident**: Manual writing → 6 rewrites, 50%+ time wasted
- **Subagent benefits**: Quality reference calibration, autonomous execution, validation gates
- **Zero tolerance**: Content written directly (bypassing subagents) will be rejected

**Detection of Bypass Attempt**:
```
IF you are about to:
  - Write lesson markdown directly (not via subagent)
  - Skip educational-validator invocation
  - Write content without quality reference
THEN STOP and invoke proper subagent workflow
```

## Outline

1. Run `.specify/scripts/bash/check-prerequisites.sh --json --require-tasks --include-tasks` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Check checklists status** (if FEATURE_DIR/checklists/ exists):
   - Scan all checklist files in the checklists/ directory
   - For each checklist, count:
     - Total items: All lines matching `- [ ]` or `- [X]` or `- [x]`
     - Completed items: Lines matching `- [X]` or `- [x]`
     - Incomplete items: Lines matching `- [ ]`
   - Create a status table:

     ```text
     | Checklist | Total | Completed | Incomplete | Status |
     |-----------|-------|-----------|------------|--------|
     | ux.md     | 12    | 12        | 0          | ✓ PASS |
     | test.md   | 8     | 5         | 3          | ✗ FAIL |
     | security.md | 6   | 6         | 0          | ✓ PASS |
     ```

   - Calculate overall status:
     - **PASS**: All checklists have 0 incomplete items
     - **FAIL**: One or more checklists have incomplete items

   - **If any checklist is incomplete**:
     - Display the table with incomplete item counts
     - **STOP** and ask: "Some checklists are incomplete. Do you want to proceed with implementation anyway? (yes/no)"
     - Wait for user response before continuing
     - If user says "no" or "wait" or "stop", halt execution
     - If user says "yes" or "proceed" or "continue", proceed to step 3

   - **If all checklists are complete**:
     - Display the table showing all checklists passed
     - Automatically proceed to step 3

3. Load and analyze the implementation context:
   - **REQUIRED**: Read tasks.md for the complete task list and execution plan
   - **REQUIRED**: Read plan.md for tech stack, architecture, and file structure
   - **IF EXISTS**: Read data-model.md for entities and relationships
   - **IF EXISTS**: Read contracts/ for API specifications and test requirements
   - **IF EXISTS**: Read research.md for technical decisions and constraints
   - **IF EXISTS**: Read quickstart.md for integration scenarios

4. **Project Setup Verification**:
   - **REQUIRED**: Create/verify ignore files based on actual project setup:

   **Detection & Creation Logic**:
   - Check if the following command succeeds to determine if the repository is a git repo (create/verify .gitignore if so):

     ```sh
     git rev-parse --git-dir 2>/dev/null
     ```

   - Check if Dockerfile* exists or Docker in plan.md → create/verify .dockerignore
   - Check if .eslintrc* exists → create/verify .eslintignore
   - Check if eslint.config.* exists → ensure the config's `ignores` entries cover required patterns
   - Check if .prettierrc* exists → create/verify .prettierignore
   - Check if .npmrc or package.json exists → create/verify .npmignore (if publishing)
   - Check if terraform files (*.tf) exist → create/verify .terraformignore
   - Check if .helmignore needed (helm charts present) → create/verify .helmignore

   **If ignore file already exists**: Verify it contains essential patterns, append missing critical patterns only
   **If ignore file missing**: Create with full pattern set for detected technology

   **Common Patterns by Technology** (from plan.md tech stack):
   - **Node.js/JavaScript/TypeScript**: `node_modules/`, `dist/`, `build/`, `*.log`, `.env*`
   - **Python**: `__pycache__/`, `*.pyc`, `.venv/`, `venv/`, `dist/`, `*.egg-info/`
   - **Java**: `target/`, `*.class`, `*.jar`, `.gradle/`, `build/`
   - **C#/.NET**: `bin/`, `obj/`, `*.user`, `*.suo`, `packages/`
   - **Go**: `*.exe`, `*.test`, `vendor/`, `*.out`
   - **Ruby**: `.bundle/`, `log/`, `tmp/`, `*.gem`, `vendor/bundle/`
   - **PHP**: `vendor/`, `*.log`, `*.cache`, `*.env`
   - **Rust**: `target/`, `debug/`, `release/`, `*.rs.bk`, `*.rlib`, `*.prof*`, `.idea/`, `*.log`, `.env*`
   - **Kotlin**: `build/`, `out/`, `.gradle/`, `.idea/`, `*.class`, `*.jar`, `*.iml`, `*.log`, `.env*`
   - **C++**: `build/`, `bin/`, `obj/`, `out/`, `*.o`, `*.so`, `*.a`, `*.exe`, `*.dll`, `.idea/`, `*.log`, `.env*`
   - **C**: `build/`, `bin/`, `obj/`, `out/`, `*.o`, `*.a`, `*.so`, `*.exe`, `Makefile`, `config.log`, `.idea/`, `*.log`, `.env*`
   - **Swift**: `.build/`, `DerivedData/`, `*.swiftpm/`, `Packages/`
   - **R**: `.Rproj.user/`, `.Rhistory`, `.RData`, `.Ruserdata`, `*.Rproj`, `packrat/`, `renv/`
   - **Universal**: `.DS_Store`, `Thumbs.db`, `*.tmp`, `*.swp`, `.vscode/`, `.idea/`

   **Tool-Specific Patterns**:
   - **Docker**: `node_modules/`, `.git/`, `Dockerfile*`, `.dockerignore`, `*.log*`, `.env*`, `coverage/`
   - **ESLint**: `node_modules/`, `dist/`, `build/`, `coverage/`, `*.min.js`
   - **Prettier**: `node_modules/`, `dist/`, `build/`, `coverage/`, `package-lock.json`, `yarn.lock`, `pnpm-lock.yaml`
   - **Terraform**: `.terraform/`, `*.tfstate*`, `*.tfvars`, `.terraform.lock.hcl`
   - **Kubernetes/k8s**: `*.secret.yaml`, `secrets/`, `.kube/`, `kubeconfig*`, `*.key`, `*.crt`

5. Parse tasks.md structure and extract:
   - **Task phases**: Setup, Tests, Core, Integration, Polish
   - **Task dependencies**: Sequential vs parallel execution rules
   - **Task details**: ID, description, file paths, parallel markers [P]
   - **Execution flow**: Order and dependency requirements

6. Execute implementation following the task plan:
   - For each lesson, use a separate content-implementer subagent. You can run them in parallel or sequentially based on the task details in tasks.md.
   - You're in collaboration with user and responsible to orchestrate the subagents. Ensure the instructions are clear and well-defined, use the evaluation rubric skill to assess the quality of the implementation. Continuously iterate, once you're satisfied with each lesson, and add them in the directory.
   - All content-implementer subagents report back to you and you're responsible to add the lessons in the main file system.
   - **Phase-by-phase execution**: Complete each phase before moving to the next
   - **Respect dependencies**: Run sequential tasks in order, parallel tasks [P] can run together
   - **Follow TDD approach**: Execute test tasks before their corresponding implementation tasks
   - **File-based coordination**: Tasks affecting the same files must run sequentially
   - **Validation checkpoints**: Verify each phase completion before proceeding

   ### Pre-Launch Checklist for Parallel Content Subagents

   **Before launching N parallel content-implementer agents**, verify:

   - [ ] Each prompt specifies **exact absolute output path** (not relative, not inferred)
   - [ ] Each prompt includes: **"Execute autonomously without waiting for confirmation"**
   - [ ] Each prompt includes: **"DO NOT create new directories unless explicitly specified"**
   - [ ] Target directories exist (verify with `ls` before launching)
   - [ ] If pattern is new/untested: **Test ONE agent first** before launching batch
   - [ ] Consider batches of 3-4 agents (easier to monitor than 12 at once)

   ### Content Subagent Prompt Template

   **REQUIRED structure for content-implementer invocations**:

   ```
   Write Lesson [N]: [Title]

   **CRITICAL EXECUTION RULES**:
   - Output file: [ABSOLUTE PATH] (write to THIS EXACT path)
   - Execute autonomously - DO NOT ask "Should I proceed?"
   - DO NOT create new directories - use the path exactly as specified
   - Write the file directly after gathering context

   **Content Requirements**:
   [Frontmatter, learning objectives, structure requirements...]

   **Constitutional Requirements**:
   - Every code block MUST have **Output:** section
   - End with "## Try With AI" (no summary after)
   - NO framework labels ("AI as Teacher", "Part 2:", etc.)
   ```

   ### Why This Matters

   **Failure Mode (2025-12-23)**: Launched 12 parallel content-implementer agents. 2 agents stopped mid-execution asking "Should I proceed?" (confirmation deadlock - no human available). 1 agent wrote to wrong directory (inferred `/51-helm-charts/` instead of specified `/50-kubernetes/`). Result: 3 agents required manual intervention.

   **For Educational Content (Chapters/Lessons)**:

   **BEFORE spawning content-implementer agents**:
   1. **Read 2-3 existing chapters** in the same part to understand voice, level, and patterns
   2. **Extract from spec.md**: What does student know BEFORE this chapter? What foundational context must be included?
   3. **Verify Layer progression**: Does L1 build vocabulary needed for L4 specs? Is the manual foundation sufficient?
   4. **Domain expert question**: "Is this chapter teaching the tool OR teaching AI-native thinking with the tool?"

   **WHEN spawning content-implementer agents**:
   - Include in prompt: "Target audience knows: [X, Y, Z]. Must explain: [A, B, C] from scratch."
   - Include in prompt: "Read existing lesson [path] for voice/level calibration before writing."
   - For parallel lesson generation: Stagger by 2-3 lessons to allow early feedback to inform later lessons

6a. **Constitutional Validation Gate** (for educational content):
   - **MANDATORY** for lesson/chapter creation tasks
   - **Two-Pass Workflow**: content-implementer → educational-validator → filesystem

   **Process**:
   ```
   1. content-implementer generates draft lesson
      ↓ (report back to orchestrator)
   2. educational-validator validates constitutional compliance
      ↓
      ├─→ PASS: Write to filesystem, mark task complete
      └─→ FAIL: Show violations
          ↓
          Option A: Auto-fix (if trivial: metadata, heading format)
          Option B: Regenerate with violations as context
          Option C: Report to user for manual review
   ```

   **Validation Checks** (automated):
   - ✅ Framework invisibility (no meta-commentary: "AI as Teacher", "Part 2:", etc.)
   - ✅ Evidence presence (70%+ code has output, claims have citations)
   - ✅ Structural compliance (ends with "Try With AI/Practice/Explore" ONLY)
   - ✅ Proficiency metadata (uses `proficiency_level`, not deprecated `cefr_level`)
   - ✅ **MDX safety**: No angle brackets before letters/numbers (e.g., `<100MB` breaks MDX)
     - Run: `grep -E '<[a-zA-Z0-9]' lesson.md` — must return empty
     - Fix pattern: Change `<100MB` to `under 100MB` or `less than 100MB`

   **When to Skip**:
   - ❌ Non-educational tasks (API endpoints, database models, scripts)
   - ❌ Documentation files (README, ADRs, specifications)
   - ✅ ONLY educational lessons/chapters require validation

   **Invocation**:
   - Use Task tool with `subagent_type: "educational-validator"`
   - Pass generated lesson content as input
   - Review validation report before filesystem write

   **Reference**: `.claude/agents/educational-validator.md` for full validation framework

7. Implementation execution rules:
   - **Setup first**: Initialize project structure, dependencies, configuration
   - **Tests before code**: If you need to write tests for contracts, entities, and integration scenarios
   - **Core development**: Implement models, services, CLI commands, endpoints
   - **Integration work**: Database connections, middleware, logging, external services
   - **Polish and validation**: Unit tests, performance optimization, documentation
   - **Educational content**: Apply step 6a validation gate before filesystem write

8. Progress tracking and error handling:
   - Report progress after each completed task
   - Halt execution if any non-parallel task fails
   - For parallel tasks [P], continue with successful tasks, report failed ones
   - Provide clear error messages with context for debugging
   - Suggest next steps if implementation cannot proceed
   - **IMPORTANT** For completed tasks, make sure to mark the task off as [X] in the tasks file.
   - **Educational content**: Track validation results (PASS/FAIL counts, common violations)

9. Completion validation:
   - Verify all required tasks are completed
   - Check that implemented features match the original specification
   - Validate that tests pass and coverage meets requirements
   - Confirm the implementation follows the technical plan
   - Report final status with summary of completed work
   - **Educational content**: Ensure all lessons passed constitutional validation (step 6a)

Note: This command assumes a complete task breakdown exists in tasks.md. If tasks are incomplete or missing, suggest running `/sp.tasks` first to regenerate the task list.

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
