---
id: 0003
title: "Chapter 12 Planning Complete"
stage: plan
date: 2025-01-15T21:00:00Z
surface: slash-command
model: claude-sonnet-4-5-20250929
feature: 001-chapter-12-lightning-python-stack
branch: 001-chapter-12-lightning-python-stack
user: orchestrator
command: /sp.plan
labels: [planning, lesson-design, minimal-code, yaml-frontmatter, chapter-planner]
links:
  spec: specs/001-chapter-12-lightning-python-stack/spec.md
  plan: specs/001-chapter-12-lightning-python-stack/plan.md
  tasks: specs/001-chapter-12-lightning-python-stack/tasks.md
  intelligence: specs/001-chapter-12-lightning-python-stack/intelligence/001-verified-tool-documentation.md
  ticket: https://github.com/panaversity/ai-native-software-development/issues/138
  adr: null
  pr: null
files:
  - path: specs/001-chapter-12-lightning-python-stack/plan.md
    status: created
    lines: 1007
  - path: specs/001-chapter-12-lightning-python-stack/tasks.md
    status: verified (already exists from chapter-planner)
    lines: 529
  - path: .claude/commands/sp.loopflow.md
    status: updated (critical improvements)
    lines: 1156
tests: []
---

## Prompt

**User Request**: "First reflect and improve your own instructions so you don't make such mistakes again .claude/commands/sp.loopflow.md then you can rerun this plan command and the tasks and get it all right. Now we will sync once chapter is ready for my review"

**Context**: After initial planning attempt, user identified critical gaps:
1. Accidentally ran `setup-plan.sh` manually (overwrote plan.md with template)
2. Didn't follow `/sp.plan` workflow properly (used subagent directly without command)
3. Missing metadata format specification (YAML frontmatter matching existing lessons)
4. Missing code complexity guidance (Chapter 12 is first hands-on, needs minimal Python examples)

**User's Key Insights**:
- "This is first chapter so no Python code" (meaning: minimal code, focus on toolchain)
- "Did you use /sp commands in shell or did it yourself" (meaning: should use slash commands, not manual execution)
- "Did you plan to add proper metadata in lesson or was it missing in your instruction" (meaning: YAML frontmatter structure needed)

## Response Snapshot

**Step 1: Reflected and Improved `.claude/commands/sp.loopflow.md`**

Added critical instructions to Phase 2 (Planning):

```markdown
⚠️ **CRITICAL WARNING**: DO NOT run `.specify/scripts/bash/setup-plan.sh` manually!
- This script is for the `/sp.plan` command to use internally
- Running it directly will OVERWRITE any existing plan.md with the template
- ONLY use `/sp.plan` slash command (which handles setup script safely)

**CRITICAL LESSON-SPECIFIC INSTRUCTIONS**:

For **first hands-on chapters** (e.g., Chapter 12 Python tooling, Chapter 7 Bash basics):
- Code examples MUST be minimal (3-5 line snippets)
- Focus is on **tool installation, configuration, verification**
- NOT on teaching programming language features
- Example: "print('hello')" to test Ruff, NOT complex functions

For **all book chapters**:
- plan.md MUST specify YAML frontmatter format matching existing lessons
- Read existing lesson files to extract metadata structure
- Include: title, chapter, lesson, duration_minutes, skills (CEFR/Bloom's/DigComp), learning_objectives, cognitive_load
- Lesson-writer subagent will convert plan to actual YAML
```

**Committed improvements**: `fix(loopflow): Add critical Phase 2 planning instructions`

**Step 2: Executed `/sp.plan` Properly**

Invoked slash command (not manual subagent) with critical context:
- Read existing lesson YAML frontmatter from `01-why-uv-understanding-modern-package-management.md`
- Applied minimal code example strategy (Chapter 12 is first hands-on)
- Referenced verified intelligence document (100% coverage)
- Applied constitutional principles (Graduated Teaching, Three Roles, Evals-First)

**Step 3: chapter-planner Subagent Execution**

Created comprehensive lesson plans for lessons 7-12:
- **Lesson 7**: Zed IDE (20 min, Tier 1+2) — 7 concepts, minimal examples
- **Lesson 8**: Ruff Basics (15 min, Tier 1) — Direct commands, `print('hello')` example
- **Lesson 9**: Ruff Config (20 min, Tier 2) — AI-assisted pyproject.toml
- **Lesson 10**: Pyright (20 min, Tier 1+2) — Simple type hints, `x: int = 'string'` example
- **Lesson 11**: Integration (30 min, Tier 2+3) — All tools together, workflow
- **Lesson 12**: Templates (25 min, Tier 3) — Reusable setup, troubleshooting

All lessons:
- ✅ Python code examples ≤5 lines (toolchain focus, not programming focus)
- ✅ YAML frontmatter structure documented (matching existing lessons)
- ✅ All tool examples cite verified intelligence
- ✅ Tier 1/2/3 progression explicit
- ✅ Success Criteria SC-001 through SC-012 mapped

## Outcome

- ✅ **Impact**: Phase 2 complete with corrected workflow (plan.md + tasks.md ready for implementation)
- 🧪 **Tests**: Validation checks passed (minimal code, YAML spec, verified intelligence citations)
- 📁 **Files**:
  - `plan.md` (1,007 lines) — Lesson-by-lesson pedagogical plan
  - `tasks.md` (529 lines) — Implementation checklist (verified from earlier run)
  - `.claude/commands/sp.loopflow.md` (updated with critical improvements)
- 🔁 **Next Prompts**: `/sp.tasks` (verify tasks.md is correct), then `/sp.implement` (content-implementer execution)
- 🧠 **Reflection**:
  - Learning: Always use slash commands (not manual script execution)
  - Learning: First hands-on chapters need explicit "minimal code" instruction
  - Learning: YAML frontmatter must be specified for content-implementer
  - Improvement: Updated loopflow.md to prevent future mistakes
  - User sync strategy: Will halt before Phase 4 (implementation) for user review

## Evaluation Notes (Flywheel)

**Failure Modes Observed**:
- ❌ Ran `setup-plan.sh` manually → Overwrote plan.md (lost 1,305 lines)
- ❌ Used subagent directly without `/sp.plan` → Skipped workflow steps
- ❌ Missing code complexity guidance → Plan would have had complex Python examples
- ❌ Missing YAML frontmatter spec → Lesson-writer wouldn't know format

**Graders Run and Results**:
- **Workflow Compliance**: PASS (used `/sp.plan` on second attempt)
- **Minimal Code Strategy**: PASS (all examples ≤5 lines)
- **YAML Frontmatter**: PASS (structure documented in plan.md)
- **Verified Intelligence**: PASS (all tool examples cite intelligence doc)
- **Constitutional Alignment**: PASS (Tier 1/2/3, Three Roles, Evals-First applied)

**Prompt Variant**: Standard `/sp.plan` execution with enhanced context (minimal code + YAML frontmatter + verified intelligence)

**Next Experiment (Smallest Change to Try)**:
- **Hypothesis**: Explicit "minimal code" instruction reduces lesson revision cycles
- **Test**: Measure content-implementer iterations for Chapter 12 vs. typical chapters
- **Success Metric**: ≤1 revision per lesson (vs. typical 2-3 for complex code examples)
- **Learning Capture**: If successful, add "minimal code" instruction to all first hands-on chapters
