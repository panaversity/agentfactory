---
id: 0001
title: book-restructure-planning
stage: plan
date: 2025-01-24
surface: claude-code
model: claude-sonnet-4-5
feature: 030-book-restructure-sdd-before-python
branch: 030-book-restructure-sdd-before-python
user: mjs
command: /sp.plan
labels: [book-infrastructure, git-operations, asset-migration]
links:
  spec: specs/030-book-restructure-sdd-before-python/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/030-book-restructure-sdd-before-python/plan.md
    status: created
    lines: 426
tests: []
---

## Prompt

User executed `/sp.plan 030-book-restructure-sdd-before-python` after completing comprehensive specification with edge case analysis.

**Context**:
- Specification complete with 28 functional requirements covering directory moves, asset migration (21 slides, ~18 images), plugin configuration updates, and 315+ content reference updates
- User confirmed two critical decisions: (1) all slides are local-only (no cloud storage), (2) cross-reference updates belong in Phase 1 (restructuring-related)
- Edge case analysis identified plugin path hardcoding as critical issue requiring updates to docusaurus.config.ts and remark-interactive-python/index.js

**Task**: Generate implementation plan following /sp.plan workflow: Technical Context, Constitution Check, Project Structure, Phase 0 (Research), Phase 1 (Design & Contracts), and Phase 2 preview.

## Response snapshot

Created comprehensive implementation plan (426 lines) for book restructuring infrastructure task:

**Key Components**:

1. **Technical Context**: Documented Bash/TypeScript/Markdown stack, Docusaurus 3.9.2, git operations with 33 chapters, 33 slides, ~30 image dirs, 315+ content references

2. **Constitution Check**: Determined constitution DOES NOT APPLY—this is infrastructure/documentation work, not educational content creation. Applied git best practices and SDD principles instead.

3. **Project Structure**: Documented current/future directory layouts showing Part 4-6 reorganization, asset paths, and plugin configuration files

4. **Phase 0 Research Tasks** (6 items):
   - Git history preservation with `git mv`
   - Reverse-order renaming pattern rationale
   - Docusaurus sidebar autogeneration mechanics
   - Interactive Python plugin path resolution
   - Custom plugin chapter number extraction verification
   - Content reference pattern analysis

5. **Phase 1 Design** (4 deliverables):
   - **data-model.md**: ChapterRenumbering and AssetRenumbering entity schemas with complete mappings
   - **contracts/**: Pre/post migration file structure schemas + validation rules (git history, no loss, plugin updates, build success)
   - **quickstart.md**: Rollback procedures (`git reset --hard`) + validation commands
   - **Agent context update**: Document git mv, reverse-order patterns, Docusaurus/plugin config for future features

6. **Phase 2 Preview**: Estimated ~78 tasks across 5 phases (Directory Prep → Chapter Moves → Asset Migration → Metadata/Content Updates → Validation)

**Critical Success Factors Documented**:
- Reverse-order execution for both file moves AND content updates
- Atomic git operations for simple rollback
- Plugin configuration as critical path dependency
- Automated search-and-replace for 315+ references

**Risk Mitigation**:
- Mid-operation failures → git rollback
- Plugin path misses → grep validation + manual browser test
- Double-replacement risk → reverse-order execution
- Build breaks → validation step before completion

## Outcome

- ✅ Impact: Implementation plan complete and ready for /sp.tasks command. Plan provides clear technical context, research roadmap, design contracts, and 78-task execution preview with rollback safety.
- 🧪 Tests: No tests—planning phase defines validation contracts for implementation
- 📁 Files: Created specs/030-book-restructure-sdd-before-python/plan.md (426 lines)
- 🔁 Next prompts: User should run `/sp.tasks 030-book-restructure-sdd-before-python` to generate actionable tasks.md from this plan
- 🧠 Reflection: Infrastructure task correctly identified as constitution-exempt. Reverse-order execution pattern essential for overlapping numeric ranges—documented explicitly in multiple sections.

## Evaluation notes (flywheel)

- Failure modes observed: None—plan generated smoothly after spec completion
- Graders run and results (PASS/FAIL): Manual review—plan structure matches template, all required sections filled
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): Proceed to /sp.tasks to generate atomic task breakdown from this plan
