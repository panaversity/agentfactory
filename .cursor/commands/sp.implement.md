---
description: Execute the implementation plan with context-aware agent routing. Routes to content-implementer for authoring tasks, general-purpose for engineering, and specialized agents for platform work.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Core Directive

**Context-Aware Implementation**: This command analyzes tasks.md to determine work type and routes to the appropriate implementation agent:
- **Content Work** (lessons, modules, chapters) → `content-implementer` subagent
- **Engineering Work** (features, APIs, components) → `general-purpose` subagent
- **Platform Work** (RAG, auth, infrastructure) → specialized agents (`rag-builder`, `scaffolder`, or `general-purpose`)

**WHY**: Different work types require different implementation expertise. Educational content needs pedagogical validation. Engineering needs test-driven development. Platform needs integration testing.

**Agent Discovery**: Before routing, check `.claude/agents/` for current agent inventory. Agent names below are examples—always verify what's actually available.

## Outline

1. **Prerequisites Check**: Run `.specify/scripts/bash/check-prerequisites.sh --json --require-tasks --include-tasks` and parse FEATURE_DIR and AVAILABLE_DOCS.

2. **Load Implementation Context**:
   - **REQUIRED**: Read `tasks.md` for task list
   - **REQUIRED**: Read `plan.md` for architecture/structure
   - **IF EXISTS**: Read `data-model.md`, `contracts/`, `research.md`, `quickstart.md`

3. **Classify Work Type**: Analyze tasks.md to determine routing:

   ```
   CLASSIFICATION SIGNALS:

   CONTENT (→ content-implementer):
   - tasks mention: lesson, module, chapter, exercise, assessment
   - tasks reference: learning objectives, proficiency, teaching
   - tasks include: hardware tier markers, layer progression

   ENGINEERING (→ general-purpose):
   - tasks mention: implement, endpoint, component, service, test
   - tasks reference: API, database, frontend, backend
   - tasks include: technical implementation steps

   PLATFORM (→ specialized):
   - RAG tasks → rag-builder
   - Scaffolding tasks → scaffolder
   - Auth/deployment → general-purpose
   ```

4. **Route to Appropriate Implementer**:

   ### For CONTENT Work (content-implementer)

   For each lesson task, spawn a `content-implementer` subagent:

   ```
   Use Task tool with:
   - subagent_type: "content-implementer"
   - prompt: Include:
     - Lesson specification from tasks.md
     - Plan context (pedagogical arc, stage)
     - Constitution reference
     - Hardware tier requirements
     - Teaching modality for this lesson
     - Previous lesson context (if not first)
   ```

   **content-implementer Responsibilities**:
   - Generate lesson following 4-layer framework
   - Apply hardware tier gates (`<HardwareGate>`, `<CloudFallback>`)
   - Implement Three Roles (INVISIBLE to students)
   - Create skills/subagents for Stage 3 lessons
   - End with "Try With AI" section only

   **Constitutional Validation Gate** (MANDATORY for content):
   ```
   1. content-implementer generates draft lesson
      ↓
   2. educational-validator validates constitutional compliance
      ↓
      ├─→ PASS: Write to filesystem, mark task complete
      └─→ FAIL: Show violations, regenerate or fix
   ```

   ### For ENGINEERING Work (general-purpose)

   ```
   Use Task tool with:
   - subagent_type: "general-purpose"
   - prompt: Include:
     - Task specification from tasks.md
     - Plan context (architecture, dependencies)
     - Test requirements
     - Integration points
   ```

   **general-purpose Responsibilities**:
   - Implement code following spec
   - Write tests (TDD approach)
   - Handle error cases
   - Document APIs

   ### For PLATFORM Work (specialized agents)

   ```
   RAG Implementation:
   Use Task tool with:
   - subagent_type: "general-purpose" (or custom rag-builder if exists)
   - prompt: RAG-specific requirements from tasks

   Auth Implementation:
   Use Task tool with:
   - subagent_type: "general-purpose"
   - prompt: Better-Auth integration requirements

   Infrastructure Implementation:
   Use Task tool with:
   - subagent_type: "general-purpose"
   - prompt: Deployment/CI/CD requirements
   ```

5. **Execution Flow**:

   ```
   FOR each task in tasks.md:
     1. Determine task type (content/engineering/platform)
     2. Route to appropriate subagent
     3. Receive implementation from subagent
     4. Validate (constitutional for content, tests for engineering)
     5. Write to filesystem if valid
     6. Mark task complete in tasks.md
   ```

   **Parallel Execution**: Tasks marked [P] can run in parallel. Use multiple Task tool calls in single message.

6. **Validation by Work Type**:

   ### Content Validation
   ```
   Use Task tool with:
   - subagent_type: "educational-validator"
   - prompt: Validate lesson against constitutional compliance:
     - Framework invisibility (no meta-commentary)
     - Evidence presence (code outputs, citations)
     - Structural compliance (ends with "Try With AI" ONLY)
     - Proficiency metadata correct
   ```

   ### Engineering Validation
   ```
   Run test suite:
   - Unit tests pass
   - Integration tests pass
   - Coverage meets requirements
   - Linting/type checking pass
   ```

   ### Platform Validation
   ```
   - Integration tests pass
   - Deployment validation in sandbox
   - Security review for auth/data handling
   ```

7. **Progress Tracking**:
   - Report progress after each completed task
   - Mark tasks complete: `- [X]` in tasks.md
   - Track validation results (PASS/FAIL counts)
   - Halt on blocking failures, continue on parallel task failures

## Hardware Tier Implementation (Content Only)

For content implementation, ensure each lesson includes:

```jsx
// For Tier 2+ content
<HardwareGate minTier={2}>
  This exercise requires NVIDIA RTX GPU...
  [GPU-specific content]
</HardwareGate>

// Always provide Tier 1 fallback
<CloudFallback tier={1}>
  If you don't have local GPU, use Omniverse Cloud...
  [Cloud-based alternative]
</CloudFallback>
```

**Tier Requirements**:
- Tier 1 MUST work for ALL content (browser/cloud path)
- Tier 2+ content MUST have fallback
- Robotics content MUST use simulation-first approach

## Content Anti-Pattern Checklist

Before writing any lesson to filesystem, verify:

```
✅ Ends with "Try With AI" as ONLY final section
✅ No "What's Next", "Summary", "Key Takeaways" sections
✅ No "Stage 1/2/3/4" labels in student-facing text
✅ No "Three Roles" headers or meta-commentary
✅ Hardware tier gates present for Tier 2+ content
✅ Safety considerations for robotics content
✅ Code examples have output/test logs
✅ Claims have citations
```

**Validation Command**:
```bash
# Check for forbidden patterns
grep -E "What's Next|Key Takeaways|Summary|Stage [0-9]|Three Roles" lesson.md
# Expected: Zero matches

# Check for missing hardware gates
grep -l "RTX\|GPU\|Isaac Sim" lesson.md | xargs grep -L "HardwareGate"
# Expected: Zero matches (all GPU content has gates)
```

## Cross-Book Intelligence Creation

During implementation, watch for:

```
INTELLIGENCE OPPORTUNITIES:
- Pattern used 2+ times → Consider creating skill
- Complex workflow (5+ decisions) → Consider creating subagent
- Reusable across books → Add to .claude/skills/ (platform-level)
- Domain-specific pattern → Document in spec or constitution
```

For Stage 3 lessons, explicitly create:
- Authoring skill: `.claude/skills/authoring/<skill-name>/SKILL.md` (content creation)
- Engineering skill: `.claude/skills/engineering/<skill-name>/SKILL.md` (platform/tooling)
- Authoring agent: `.claude/agents/authoring/<agent-name>.md` (content workflows)
- Engineering agent: `.claude/agents/engineering/<agent-name>.md` (platform workflows)

## Completion Validation

Before marking implementation complete:

1. **All tasks marked `[X]` in tasks.md**
2. **Validation results**:
   - Content: All lessons passed educational-validator
   - Engineering: All tests pass
   - Platform: Integration tests pass

3. **Cross-book intelligence**:
   - Skills/subagents created for recurring patterns
   - Knowledge files updated if new domain info

4. **Final report**:
   ```
   IMPLEMENTATION COMPLETE:
   - Tasks completed: N/N
   - Content lessons: [N] passed validation
   - Engineering tasks: [N] tests passing
   - Platform tasks: [N] integrations verified
   - Intelligence created: [list skills/subagents]
   ```

## Key Rules

- Route to content-implementer for ANY educational content
- Route to general-purpose for engineering/platform work
- Constitutional validation MANDATORY for content
- Test validation MANDATORY for engineering
- Mark tasks complete ONLY after validation passes
- Hardware tier fallbacks REQUIRED for Tier 2+ content

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record).

1) Determine Stage: `green` (implementation)

2) Generate Title and Determine Routing:
   - Route: `history/prompts/<feature-name>/`

3) Create and Fill PHR:
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage green --feature <name> --json`
   - Fill placeholders with implementation summary

4) Validate + report
