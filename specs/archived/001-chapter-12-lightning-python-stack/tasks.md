# Chapter 12: The Lightning Python Stack — Implementation Tasks

**Chapter Type**: Technical (Tool-by-Tool Integration)
**Status**: Ready for Development
**Feature Branch**: 001-chapter-12-lightning-python-stack
**Owner**: (To be assigned)
**Estimated Total Effort**: 40-50 story points (approximately 60-70 hours)
**Deliverable**: Lessons 7-12 content files + updated chapter metadata

---

## Context

This task checklist implements **Lessons 7-12** (6 new lessons) for Chapter 12: "The Lightning Python Stack" expansion.

**Reference**:
- Specification: `specs/001-chapter-12-lightning-python-stack/spec.md`
- Lesson Plan: `specs/001-chapter-12-lightning-python-stack/plan.md`
- Verified Intelligence: `specs/001-chapter-12-lightning-python-stack/intelligence/001-verified-tool-documentation.md`
- Constitutional Alignment Audit: `specs/001-chapter-12-lightning-python-stack/CONSTITUTIONAL-ALIGNMENT-AUDIT.md`

**Key Constraints**:
- ✅ DO NOT modify lessons 1-6 (existing uv content preserved)
- ✅ ALL examples use verified tool documentation (Context7 + sandbox tested)
- ✅ Cognitive load ≤7 concepts per section (B1 tier, Intermediate audience)
- ✅ Graduated Teaching applied (Tier 1 → 2 → 3 progression)
- ✅ Each lesson ends with "Try With AI" section (no separate "Key Takeaways" or "What's Next")
- ✅ All code examples include source citations (e.g., `# Source: Ruff docs v0.14.5`)

---

## Task List by Phase

### Phase 1: Content Structure & Lesson Outlines (Must-Have Priority)

- [ ] **MUST**: Create chapter metadata updates (readme.md, title change to "The Lightning Python Stack")
  - Acceptance: README includes all 12 lessons in table of contents; new chapter title reflects uv + Zed + Ruff + Pyright
  - Reference: `specs/001-chapter-12-lightning-python-stack/plan.md` (Executive Summary)
  - Effort: 2h

- [ ] **MUST**: Lesson 7 outline and structure (Zed IDE for Python Development)
  - Acceptance: Outline approved; matches plan.md Section "Lesson 7: Zed IDE for Python Development"
  - Includes: 7 key concepts, 2-3 exercises, Tier 1+2 teaching tiers, end-of-lesson "Try With AI"
  - Reference: `.claude/output-styles/lesson.md` for lesson structure template
  - Effort: 2h

- [ ] **MUST**: Lesson 8 outline (Ruff Linter & Formatter Basics)
  - Acceptance: Outline matches plan.md Section "Lesson 8"
  - Includes: 7 concepts, 2 exercises, Tier 1 teaching, "Try With AI" prompts
  - Effort: 1.5h

- [ ] **MUST**: Lesson 9 outline (Advanced Ruff Configuration)
  - Acceptance: Outline matches plan.md; includes pyproject.toml config examples, Zed integration
  - Includes: 7 concepts, 2 exercises, Tier 2 teaching, "Try With AI" prompts
  - Effort: 1.5h

- [ ] **MUST**: Lesson 10 outline (Pyright Type Checker Integration)
  - Acceptance: Outline matches plan.md; includes type hints, checking modes, Zed integration
  - Includes: 7 concepts, 2 exercises, Tier 1+2 teaching, "Try With AI" prompts
  - Effort: 2h

- [ ] **MUST**: Lesson 11 outline (Complete Workflow Integration)
  - Acceptance: Outline matches plan.md; covers manual workflow → IDE integration → Tier 3 orchestration
  - Includes: 7 concepts, 3 exercises (single-file, IDE, troubleshooting), "Try With AI" prompts
  - Effort: 2h

- [ ] **MUST**: Lesson 12 outline (Professional Template & Troubleshooting)
  - Acceptance: Outline matches plan.md; includes template structure, awareness-level tools, checklist
  - Includes: 7 concepts, 3 exercises (create template, pytest config, troubleshooting), "Try With AI" prompts
  - Effort: 2h

---

### Phase 2: Content Writing & Code Examples

- [ ] **MUST**: Write Lesson 7 full content (Zed IDE for Python Development)
  - Acceptance: Content covers: installation (all platforms), opening projects, integrated terminal, LSP config, AI tools integration
  - Code examples: 1 settings.json example (verified from Context7 Section 4)
  - Exercises: 2 hands-on (install & open project; enable Pyright LSP)
  - "Try With AI": 3 prompts (Tier 1, 2, 2) with expected outcomes
  - Duration estimate validation: ~20 minutes to read and practice
  - Reference: `001-verified-tool-documentation.md` Section 4 (Zed Editor Integration)
  - Effort: 4h

- [ ] **MUST**: Write Lesson 8 full content (Ruff Linter & Formatter Basics)
  - Acceptance: Content covers: what Ruff is, installation via uv, formatting demo, linting demo, rule codes
  - Code examples: 3 verified (from `001-verified-tool-documentation.md` Section 2)
    - Messy Python file → formatted file (before/after)
    - Python file with logic issues → Ruff check output
    - Rule code interpretation (F401, F841)
  - Exercises: 2 hands-on (format messy code; add issues & run linting)
  - "Try With AI": 3 prompts (Tier 1, 1, 2)
  - Duration validation: ~15 minutes
  - Reference: `001-verified-tool-documentation.md` Section 2.2-2.3 (verified in sandbox)
  - Effort: 3.5h

- [ ] **MUST**: Write Lesson 9 full content (Advanced Ruff Configuration)
  - Acceptance: Content covers: pyproject.toml strategy, [tool.ruff] configuration, Zed format-on-save integration
  - Code examples: 2 verified configs (from `001-verified-tool-documentation.md` Section 2.4)
    - pyproject.toml [tool.ruff] section (permissive + strict variants)
    - Zed settings.json with Ruff LSP
  - Exercises: 2 hands-on (configure Ruff in pyproject.toml; enable format-on-save)
  - "Try With AI": 3 prompts (Tier 2, 2, 3) with escalating complexity
  - Duration validation: ~20 minutes
  - Reference: `001-verified-tool-documentation.md` Section 2.4 (configuration verified in sandbox)
  - Effort: 4h

- [ ] **MUST**: Write Lesson 10 full content (Pyright Type Checker Integration)
  - Acceptance: Content covers: why type checking matters, type hint syntax, Pyright installation, type checking modes, Zed integration
  - Code examples: 3 verified (from `001-verified-tool-documentation.md` Section 3)
    - Function without type hints → function with type hints
    - Type error example (return type mismatch)
    - Pyright configuration (strict mode)
  - Exercises: 2 hands-on (add type hints & detect errors; enable strict mode)
  - "Try With AI": 3 prompts (Tier 1, 1, 2)
  - Duration validation: ~20 minutes
  - Reference: `001-verified-tool-documentation.md` Section 3.2-3.3 (verified in sandbox)
  - Effort: 4h

- [ ] **MUST**: Write Lesson 11 full content (Complete Workflow Integration)
  - Acceptance: Content covers: workflow sequence (format → lint → type check), manual execution, IDE integration, Tier 3 orchestration, troubleshooting
  - Code examples: 4 verified (from plan.md Section "Lesson 11")
    - Test file with all three issue types (formatting, linting, type)
    - Step-by-step output from each tool
    - Troubleshooting checklist (5 common issues with fixes)
  - Exercises: 3 hands-on (manual workflow, IDE integration, troubleshooting)
  - "Try With AI": 3 prompts (Tier 2, 2, 3 orchestration)
  - Duration validation: ~30 minutes
  - Effort: 5h

- [ ] **MUST**: Write Lesson 12 full content (Professional Template & Troubleshooting)
  - Acceptance: Content covers: template motivation, pyproject.toml template, Zed config, awareness-level tools (pytest, pre-commit, GitHub Actions), troubleshooting checklist
  - Code examples: 5 verified
    - Complete pyproject.toml template
    - Zed settings.json template
    - .pre-commit-config.yaml
    - GitHub Actions workflow (.github/workflows/test.yml)
    - Project directory structure diagram
  - Exercises: 3 hands-on (create template, extend with pytest, troubleshoot broken tool)
  - "Try With AI": 3 prompts (Tier 3, 1, 2)
  - Duration validation: ~25 minutes
  - Effort: 5h

---

### Phase 3: Code Examples & Verification

- [ ] **MUST**: Verify all Zed installation commands (Lesson 7)
  - Acceptance: Installation commands tested on Windows, macOS, Linux
  - Reference: `001-verified-tool-documentation.md` Section 4.1
  - Effort: 1h

- [ ] **MUST**: Verify all Ruff commands and outputs (Lessons 8-9)
  - Acceptance: All `ruff format` and `ruff check` examples match actual tool output
  - Reference: `001-verified-tool-documentation.md` Section 2.2-2.3 (all tested in sandbox)
  - Effort: 1h

- [ ] **MUST**: Verify all Pyright commands and type examples (Lesson 10)
  - Acceptance: Type error messages match exact Pyright output
  - Reference: `001-verified-tool-documentation.md` Section 3.2 (verified in sandbox)
  - Effort: 1h

- [ ] **SHOULD**: Create working project template for Lesson 12
  - Acceptance: Template can be cloned; `uv sync` installs all dependencies; all tools work
  - Tested on: macOS (primary), Windows WSL, Ubuntu (secondary)
  - Deliverable: Git repository with working template
  - Effort: 3h

- [ ] **SHOULD**: Record demo videos (optional, nice-to-have)
  - Acceptance: Short clips (3-5 min) showing: Zed installation, format-on-save, Ruff check output
  - Tools: ScreenFlow (macOS) or equivalent
  - Reference: Chapter 9 for video embedding style
  - Effort: 4h (NICE-TO-HAVE, defer if time constrained)

---

### Phase 4: Exercise Design & Acceptance Criteria

- [ ] **MUST**: Design and validate Lesson 7 exercises
  - Exercise 1: Install Zed and open uv project
    - Acceptance: Zed opens; integrated terminal available; `uv` commands work
    - Validation: User can run `uv add requests` in Zed terminal
    - Effort: 1h

  - Exercise 2: Enable Pyright LSP; verify type checking
    - Acceptance: Type error appears as red squiggly in editor
    - Validation: Edit file with `x: int = "string"`; see type error inline
    - Effort: 1h

- [ ] **MUST**: Design and validate Lesson 8 exercises
  - Exercise 1: Format messy Python file
    - Acceptance: `uv run ruff format main.py` produces clean output
    - Validation: Spacing normalized, quotes standardized
    - Effort: 1h

  - Exercise 2: Lint file with issues; interpret output
    - Acceptance: `uv run ruff check` reports F401 (unused import), F841 (unused variable)
    - Validation: Student identifies and understands each error
    - Effort: 1h

- [ ] **MUST**: Design and validate Lesson 9 exercises
  - Exercise 1: Configure Ruff in pyproject.toml
    - Acceptance: Configuration is valid TOML; rules apply as specified
    - Validation: `uv run ruff check` respects line-length, quote-style
    - Effort: 1h

  - Exercise 2: Enable format-on-save in Zed
    - Acceptance: Saving Python file in Zed auto-formats
    - Validation: Spacing and quotes auto-correct on save
    - Effort: 1h

- [ ] **MUST**: Design and validate Lesson 10 exercises
  - Exercise 1: Add type hints; run Pyright
    - Acceptance: Pyright passes without errors
    - Validation: Type-safe function runs correctly
    - Effort: 1h

  - Exercise 2: Introduce type errors; fix them
    - Acceptance: Pyright detects type errors; fixes eliminate errors
    - Validation: Strict mode catches edge cases (None handling, etc.)
    - Effort: 1h

- [ ] **MUST**: Design and validate Lesson 11 exercises
  - Exercise 1: Run manual workflow (format → lint → type check)
    - Acceptance: All three tools pass with no errors
    - Validation: Each tool's output correct and actionable
    - Effort: 1h

  - Exercise 2: Use IDE integration (automatic workflow)
    - Acceptance: All tools run on save in Zed
    - Validation: All diagnostics visible inline
    - Effort: 1h

  - Exercise 3: Troubleshoot broken tool
    - Acceptance: Using checklist, identify and fix issue
    - Validation: Tool works after fix
    - Effort: 1h

- [ ] **MUST**: Design and validate Lesson 12 exercises
  - Exercise 1: Create and use project template
    - Acceptance: Template can be cloned; new project created; tools work
    - Validation: `uv sync` succeeds; `ruff check`, `pyright` pass
    - Effort: 1.5h

  - Exercise 2: Extend template with pytest
    - Acceptance: pytest discovers and runs tests
    - Validation: `uv run pytest` finds and executes starter test
    - Effort: 1h

  - Exercise 3: Diagnose and fix tool integration failure
    - Acceptance: Using troubleshooting checklist, resolve issue
    - Validation: Tool works correctly
    - Effort: 1h

---

### Phase 5: "Try With AI" Section Design

- [ ] **MUST**: Design Lesson 7 "Try With AI" prompts
  - Prompt A (Tier 1): "Explain what the integrated terminal does and why I didn't need to activate a virtual environment."
    - Expected output: Explanation of Zed environment pre-activation
    - Effort: 0.5h

  - Prompt B (Tier 2): "Configure my Zed settings.json to show type errors inline using Pyright."
    - Expected output: Working Pyright LSP configuration
    - Effort: 0.5h

  - Prompt C (Tier 2): "How do I use Claude Code to explain a confusing Python function while editing it?"
    - Expected output: Step-by-step guide to Claude Code integration
    - Effort: 0.5h

- [ ] **MUST**: Design Lesson 8 "Try With AI" prompts
  - Prompt A (Tier 1): "I ran `ruff format`. Explain what changed and why consistent formatting matters."
    - Expected output: Summary of changes, benefits explanation
    - Effort: 0.5h

  - Prompt B (Tier 1): "Explain these errors: F401 (unused import), F841 (assigned but unused). Why does Ruff catch them?"
    - Expected output: Clear error explanation, why they matter
    - Effort: 0.5h

  - Prompt C (Tier 2): "Can I use `ruff check --fix` to auto-fix all errors, or should I manually review?"
    - Expected output: Explanation of auto-fix limitations, review best practices
    - Effort: 0.5h

- [ ] **MUST**: Design Lesson 9 "Try With AI" prompts
  - Prompt A (Tier 2): "Generate the [tool.ruff] section with double quotes, line length 100, strict linting (E, F, B, I, D)."
    - Expected output: Valid TOML configuration
    - Effort: 0.5h

  - Prompt B (Tier 2): "Set up Zed to run Ruff formatter automatically on save."
    - Expected output: Zed settings.json with format-on-save
    - Effort: 0.5h

  - Prompt C (Tier 3): "Apply Ruff formatting to 50 files. What command do I run?"
    - Expected output: Orchestration command + explanation
    - Effort: 0.5h

- [ ] **MUST**: Design Lesson 10 "Try With AI" prompts
  - Prompt A (Tier 1): "Write a function with type hints. Explain what each one means."
    - Expected output: Function with annotations, explanation of syntax
    - Effort: 0.5h

  - Prompt B (Tier 1): "I got a Pyright error: 'str cannot be assigned to int'. What went wrong?"
    - Expected output: Type mismatch explanation, fix options
    - Effort: 0.5h

  - Prompt C (Tier 2): "Configure Pyright in strict mode in my pyproject.toml."
    - Expected output: Valid Pyright configuration with strict mode
    - Effort: 0.5h

- [ ] **MUST**: Design Lesson 11 "Try With AI" prompts
  - Prompt A (Tier 2): "Walk me through running Ruff format, Ruff check, and Pyright in order. Explain what each fixes."
    - Expected output: Step-by-step workflow guide with explanations
    - Effort: 0.5h

  - Prompt B (Tier 2): "My Pyright errors don't show in Zed. Diagnose and fix the LSP configuration."
    - Expected output: Diagnosis (LSP not running) and fix
    - Effort: 0.5h

  - Prompt C (Tier 3): "Run Ruff and Pyright on my 20-file project. Show me all issues."
    - Expected output: Orchestration of tools across project; summary
    - Effort: 0.5h

- [ ] **MUST**: Design Lesson 12 "Try With AI" prompts
  - Prompt A (Tier 3): "Create a Python project template with uv, Ruff, Pyright, Zed, pytest, pre-commit all pre-configured."
    - Expected output: Complete template structure with all files
    - Effort: 0.5h

  - Prompt B (Tier 1): "Explain what pytest, pre-commit, and GitHub Actions do. I see they're configured but don't understand them yet."
    - Expected output: Explanation of each tool's purpose; awareness-level understanding
    - Effort: 0.5h

  - Prompt C (Tier 2): "pytest says 'No tests found'. Walk me through the troubleshooting checklist."
    - Expected output: Step-by-step diagnosis and fix
    - Effort: 0.5h

---

### Phase 6: Peer Review & Validation

- [ ] **MUST**: Pedagogical peer review for all 6 lessons
  - Acceptance: Reviewer confirms learning objectives met, content clear, exercises effective
  - Reference: Content evaluation framework (Principle 8 accessibility + Principle 9 show-spec-validate)
  - Effort: 3h (0.5h per lesson)

- [ ] **MUST**: Technical accuracy review
  - Acceptance: Verify all tool commands and configurations match verified intelligence document
  - Reference: `001-verified-tool-documentation.md` (100% verification coverage, 27/27 claims)
  - Effort: 2h

- [ ] **MUST**: Accessibility and inclusivity check
  - Acceptance: No ableist language, jargon explained, examples diverse, error literacy clear
  - Reference: Principle 8 (Accessibility and Inclusivity)
  - Effort: 1h

- [ ] **SHOULD**: Cross-chapter coherence check
  - Acceptance: Verified that lessons 7-12 flow from lessons 1-6 without gaps
  - Reference: Lesson plan "Content Flow & Dependencies"
  - Effort: 1h

- [ ] **SHOULD**: Cognitive load verification
  - Acceptance: Each lesson ≤7 new concepts; no sudden jumps in complexity
  - Reference: Principle 12 (Cognitive Load Consciousness, B1 tier)
  - Effort: 1h

---

### Phase 7: Chapter Metadata & Integration

- [ ] **MUST**: Update chapter readme.md
  - Acceptance: Title changed to "The Lightning Python Stack"; all 12 lessons listed in TOC
  - Includes: Updated learning objectives covering lessons 7-12
  - Reference: plan.md "Chapter Objective"
  - Effort: 1h

- [ ] **MUST**: Update chapter YAML frontmatter
  - Acceptance: Chapter 12 metadata reflects new scope (lessons 7-12 added)
  - Effort: 0.5h

- [ ] **SHOULD**: Expand and update quiz (07_chapter_12_quiz.md)
  - Acceptance: Quiz covers lessons 7-12 in addition to existing lessons 1-6
  - Questions: Multiple choice + scenario-based for each new lesson
  - Answer key provided
  - Effort: 3h

- [ ] **SHOULD**: Create chapter summary document
  - Acceptance: One-page summary of all 12 lessons and how they connect
  - Useful for: Quick reference, review before final project
  - Effort: 1h

---

## Quality Assurance Checklist

### Content Quality Gates

- [ ] **All tool claims sourced**: Every tool command/configuration cites `001-verified-tool-documentation.md`
- [ ] **Cognitive load OK**: Each lesson ≤7 concepts; no section overloaded
- [ ] **Graduated teaching clear**: Tier 1/2/3 explicitly labeled in content
- [ ] **Three roles modeled**: Each lesson includes examples of AI as Teacher/Student/Co-Worker
- [ ] **Accessibility verified**: Jargon explained on first use, examples inclusive, error literacy present
- [ ] **Constitutional alignment**: All lessons align with Principles 1, 9, 13, 18
- [ ] **Exercises testable**: Each exercise has clear acceptance criteria
- [ ] **Duration realistic**: Actual reading/practice time matches estimated duration
- [ ] **No memorization expected**: Tier 2+ sections delegate complexity to AI (not expect memorization)
- [ ] **Validation-first present**: All exercises include verification steps

### Technical Gates

- [ ] **All Zed commands tested** on macOS, Windows (WSL recommended), Linux
- [ ] **All Ruff examples verified** against Ruff 0.14.5+ (sandbox tested)
- [ ] **All Pyright examples verified** against Pyright 1.1.407+ (sandbox tested)
- [ ] **All configurations valid**: TOML/JSON syntax correct; no hallucinated options
- [ ] **Cross-platform documented**: Installation instructions for all major platforms
- [ ] **Error messages accurate**: Examples show real tool output, not paraphrased

### Pedagogical Gates

- [ ] **Show-Spec-Validate pattern present**: Examples follow spec → prompt → code → explanation → validation
- [ ] **Real-world relevance clear**: Lessons explain why professionals use this stack
- [ ] **Prerequisites met**: All lessons reference completed chapters correctly
- [ ] **Forward connections clear**: Future chapters (Chapter 42, 54) mentioned appropriately (awareness-level)
- [ ] **Learning objectives measurable**: Students can demonstrate understanding via exercises
- [ ] **Homework/practice clear**: Each lesson has 2-3 exercises with defined outcomes

### File Output Gates

- [ ] **All lesson files in correct directory**: `/book-source/docs/04-Python-Fundamentals/12-python-uv-package-manager/`
- [ ] **File naming convention**: `07-lesson-name.md`, `08-lesson-name.md`, etc.
- [ ] **Frontmatter complete**: YAML includes title, chapter, lesson, duration, skills, learning_objectives
- [ ] **Internal links valid**: Cross-references to other chapters use correct paths
- [ ] **Code blocks properly formatted**: Python examples use ```python with syntax highlighting
- [ ] **No trailing whitespace**: Files clean and ready for git

---

## Acceptance Criteria (Definition of Done)

**All Chapters Must Meet**:
- ✅ All MUST tasks completed (no SHOULD/NICE-TO-HAVE blocking delivery)
- ✅ Content quality passes peer review
- ✅ All exercises have been tested and work as documented
- ✅ All tool examples verified against `001-verified-tool-documentation.md`
- ✅ Accessibility requirements met (Principle 8)
- ✅ Each lesson ends with "Try With AI" section; no "Key Takeaways" or "What's Next" sections
- ✅ Learning objectives measurable and use appropriate Bloom's levels
- ✅ Cognitive load appropriate for B1 (≤7 concepts per section)
- ✅ Constitutional alignment verified (Principles 1, 9, 13, 18)

**Lessons 7-12 Specific**:
- ✅ All 6 lessons (7-12) implemented with full content
- ✅ All tool configurations from verified intelligence applied
- ✅ Graduated Teaching (Tier 1→2→3) progression clear across lessons
- ✅ Three Roles (Teacher/Student/Co-Worker) modeled in AI examples
- ✅ Cross-platform support documented (Windows/macOS/Linux)
- ✅ Smooth transition from lessons 1-6 (uv) to lessons 7-12 (complete stack)
- ✅ Chapter metadata updated (readme, YAML frontmatter, quiz)

---

## Follow-Up Tasks & Risks

### Identified Risks

- **Risk 1: Tool Version Creep**
  - **Impact**: Zed/Ruff/Pyright release breaking changes; examples become outdated
  - **Likelihood**: Medium (tools actively developed)
  - **Mitigation**: Version pins in all examples; update triggers documented (Section VI.B Constitution)
  - **Action**: Set review date Q3 2025 (3-month refresh cycle)

- **Risk 2: Platform-Specific Issues**
  - **Impact**: Installation fails on Windows/Linux; students stuck
  - **Likelihood**: Low (Zed/Ruff/Pyright all cross-platform)
  - **Mitigation**: Test on three platforms; document workarounds
  - **Action**: Request tester for Windows/Linux before publication

- **Risk 3: LSP Configuration Complexity**
  - **Impact**: Students struggle with Zed settings.json; LSP doesn't start
  - **Likelihood**: Medium (configuration is notoriously finicky)
  - **Mitigation**: Detailed troubleshooting checklist (Lesson 11); AI examples show how to ask for help
  - **Action**: Include pre-made settings.json in template (Lesson 12)

### Next Steps (After Phase 7)

1. **Phase 8: Validation** (Technical Review)
   - Invoke `validation-auditor` subagent to verify code examples
   - Run sandbox tests on all commands
   - Verify against Success Criteria (SC-001 through SC-012 from spec.md)

2. **Phase 9: Proof Validation** (Final QA)
   - Invoke `factual-verifier` for final quality gate
   - Check all files written to correct locations
   - Verify chapter builds in Docusaurus

3. **Phase 10: Publication**
   - Human final review
   - Merge to main branch
   - Build and test full book
   - Publish to web

---

## Summary: Task Estimates by Category

| Category | Tasks | Est. Hours | Effort |
|----------|-------|-----------|--------|
| **Outlines & Structure** | 7 | 14 | P0 (Must-Have) |
| **Content Writing** | 6 lessons | 25.5 | P0 (Must-Have) |
| **Code Examples** | Verification | 4 | P0 (Must-Have) |
| **Exercise Design** | 18 exercises | 18 | P0 (Must-Have) |
| **"Try With AI"** | 18 prompts | 9 | P0 (Must-Have) |
| **Peer Review** | 4 reviews | 7 | P0 (Must-Have) |
| **Chapter Metadata** | readme, quiz | 4.5 | P0 (Must-Have) |
| **Optional Deliverables** | Videos, template | 7 | P1 (Should-Have) |
| **TOTAL** | — | **88.5h** | 40-50 SP |

**Critical Path** (non-parallelizable): Outlines → Content Writing → Review → Metadata (6 weeks with 1 FTE)

---

## References

- **Specification**: `specs/001-chapter-12-lightning-python-stack/spec.md`
- **Lesson Plan**: `specs/001-chapter-12-lightning-python-stack/plan.md` ← READ FIRST for lesson structure
- **Verified Intelligence**: `specs/001-chapter-12-lightning-python-stack/intelligence/001-verified-tool-documentation.md`
- **Constitutional Alignment Audit**: `specs/001-chapter-12-lightning-python-stack/CONSTITUTIONAL-ALIGNMENT-AUDIT.md` (94/100 score)
- **Output Styles**: `.claude/output-styles/lesson.md` (lesson structure template)
- **Chapter Index**: `specs/book/chapter-index.md` (for chapter positioning in book)

