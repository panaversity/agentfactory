# Feature Specification: Chapter 12 — The Lightning Python Stack Expansion

**Feature Branch**: `001-chapter-12-lightning-python-stack`
**Created**: 2025-01-15
**Status**: Draft
**Input**: Chapter 12 revision/expansion: Transform from uv-only to comprehensive Lightning Python Stack (uv + Zed + Ruff + Pyright). EXPAND existing 6 uv lessons, ADD 4-6 new lessons for Zed IDE, Ruff linter/formatter, and Pyright type checker. Tool-by-tool teaching sequence with AI-first integration.

## Executive Summary

**Current State**: Chapter 12 ("Python UV: Fastest Python Package Manager") has 6 lessons focused exclusively on uv package manager basics.

**Target State**: Expand Chapter 12 to "The Lightning Python Stack — uv, Zed, Ruff & Pyright" by preserving existing uv lessons (1-6) and adding 4-6 new lessons (7-12) covering Zed IDE, Ruff linter/formatter, and Pyright type checker as an integrated modern Python development toolchain.

**Audience**: Intermediate learners (B1 tier, Part 4: Python Fundamentals)
**Prerequisites**: Chapters 7 (Bash), 8 (Git), 11 (Context Engineering)
**Teaching Approach**: Tool-by-tool sequence with AI-first integration and graduated teaching (direct commands → AI companion → AI orchestration)

---

## Success Criteria (Evals-First) *(mandatory)*

### Measurable Outcomes

**Reader Comprehension & Skill Acquisition**:

- **SC-001**: 80%+ of readers can explain why each tool (uv, Zed, Ruff, Pyright) exists and when to use it (quiz assessment)
- **SC-002**: 75%+ of readers successfully set up complete Lightning Python Stack on their platform without external help (hands-on exercise completion)
- **SC-003**: 85%+ of readers can create a new Python project with all tools integrated and verify quality gates work (integrated workflow test)
- **SC-004**: 70%+ of readers can troubleshoot common integration issues using AI assistance (troubleshooting exercise)

**AI Partnership Effectiveness**:

- **SC-005**: 80%+ of readers demonstrate understanding when to use direct commands vs. AI assistance (scenario-based assessment)
- **SC-006**: Readers use AI for complex configurations (pyproject.toml, Zed settings, Ruff rules) rather than memorizing syntax (workflow observation)

**Content Quality**:

- **SC-007**: Chapter maintains B1 cognitive load limits (≤7 new concepts per section)
- **SC-008**: No lesson duration inflation (realistic estimates: simple operations 1-5 min, complex setups 15-30 min)
- **SC-009**: All tool-specific examples use verified facts (no hallucinated configuration options)
  - **Verification**: All claims verified via Phase 0.5 Deep Research (Context7 MCP + sandbox testing)
  - **Evidence**: [001-verified-tool-documentation.md](intelligence/001-verified-tool-documentation.md)
  - **Coverage**: 100% (27/27 tool-specific claims verified against official documentation)

**Integration Success**:

- **SC-010**: Chapter smoothly transitions from uv (lessons 1-6) to integrated toolchain (lessons 7-12) without conceptual gaps
- **SC-011**: Readers understand progression: foundational uv → professional IDE (Zed) → code quality automation (Ruff/Pyright)

**Awareness-Level Coverage**:

- **SC-012**: Readers know pytest, pre-commit, MkDocs exist and have working config examples, but depth deferred to later chapters (awareness check)

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Professional Python Setup for First-Time Learners (Priority: P1)

**Scenario**: A beginner Python developer completing Part 4 fundamentals wants to set up a modern, professional Python development environment that scales with their learning journey.

**Why this priority**: This is the CORE value proposition—giving intermediate learners (B1) immediate access to professional-grade tooling without overwhelming complexity. Success here = successful chapter.

**Independent Test**: Reader can create a new Python project with uv, open it in Zed, run Ruff formatter, and verify type checking with Pyright—all within 30 minutes following chapter instructions. No external documentation needed.

**Acceptance Scenarios**:

1. **Given** reader has completed Chapter 11 (Context Engineering), **When** they start Chapter 12 lesson 1, **Then** they understand WHY modern Python development uses integrated toolchains (not just pip)
2. **Given** reader has uv installed (lesson 2), **When** they add Zed IDE (new lesson 7), **Then** they can open uv projects in Zed with integrated terminal
3. **Given** reader has written Python code, **When** they add Ruff (new lesson 8-9), **Then** code is auto-formatted on save and linting errors appear inline
4. **Given** reader uses type hints (Python 3.13+), **When** they add Pyright (new lesson 10), **Then** type errors are caught instantly in Zed editor
5. **Given** reader completes integration lesson (new lesson 11), **When** they create a new project from scratch, **Then** all tools work together automatically

---

### User Story 2 - AI-Assisted Configuration Mastery (Priority: P2)

**Scenario**: A reader wants to customize their development toolchain (Ruff rules, Pyright strictness, Zed keybindings) but doesn't want to memorize complex configuration syntax.

**Why this priority**: Demonstrates constitutional Principle 13 (Graduated Teaching) and Principle 1 (AI-First Teaching). This differentiates our approach from traditional docs.

**Independent Test**: Reader can describe desired configuration changes in natural language to Claude Code/Gemini CLI and get working pyproject.toml or Zed settings.json modifications. Validates AI-driven workflow.

**Acceptance Scenarios**:

1. **Given** reader wants stricter Ruff linting, **When** they ask AI "Add import sorting and security checks to Ruff", **Then** AI updates pyproject.toml [tool.ruff] section correctly
2. **Given** reader wants Pyright strict mode, **When** they specify "Enable strict type checking for this project", **Then** AI configures typeCheckingMode = "strict" in pyproject.toml
3. **Given** reader wants Zed to show type errors inline, **When** they request "Configure Pyright LSP in Zed", **Then** AI generates correct Zed settings.json LSP configuration
4. **Given** reader encounters configuration error, **When** they paste error to AI, **Then** AI diagnoses issue and suggests fix

---

### User Story 3 - Understanding Tool-by-Tool Progression (Priority: P2)

**Scenario**: A reader wants to understand how each tool (uv, Zed, Ruff, Pyright) contributes to the development workflow and why professionals use all four together.

**Why this priority**: Conceptual understanding prevents "cargo cult" tool adoption. Reader learns strategic thinking, not just commands.

**Independent Test**: Reader can explain to a peer: "uv manages packages, Zed provides AI-first editing, Ruff enforces code quality, Pyright catches type errors—together they automate quality gates."

**Acceptance Scenarios**:

1. **Given** reader completes lesson 6 (uv team collaboration), **When** they start lesson 7 (Zed intro), **Then** they understand: "uv created the project, Zed is where I EDIT code with AI help"
2. **Given** reader writes messy Python code, **When** they run Ruff (lesson 8), **Then** they see before/after comparison showing consistent formatting
3. **Given** reader adds type hints, **When** they enable Pyright (lesson 10), **Then** they catch bugs BEFORE running code (type error examples shown)
4. **Given** reader completes all lessons, **When** they review chapter summary, **Then** they can map each tool to its purpose in professional workflow

---

### User Story 4 - Cross-Platform Setup Success (Priority: P3)

**Scenario**: A reader on Windows/macOS/Linux wants all tools (uv, Zed, Ruff, Pyright) to work identically on their platform.

**Why this priority**: Essential for inclusivity (Principle 8) but lower priority than core functionality. Platform differences are well-documented in official docs.

**Independent Test**: Reader follows chapter instructions on their platform and reaches identical end state (working toolchain) as readers on other platforms.

**Acceptance Scenarios**:

1. **Given** reader is on Windows, **When** they install Zed, **Then** instructions include Windows-specific download link and setup notes
2. **Given** reader is on macOS, **When** they configure Ruff in Zed, **Then** LSP settings use macOS path conventions
3. **Given** reader is on Linux, **When** they install via package manager, **Then** alternative installation methods are documented
4. **Given** reader encounters platform-specific error, **When** they search chapter, **Then** platform-specific troubleshooting section exists

---

### User Story 5 - Awareness-Level Exposure to Advanced Tools (Priority: P3)

**Scenario**: A reader wants to know what professional Python developers use beyond the core stack (pytest, pre-commit, MkDocs, CI/CD) without deep-diving into each tool now.

**Why this priority**: Satisfies curiosity and provides "next steps" roadmap without scope creep. ChatGPT conversation mentioned these tools—include awareness, defer depth.

**Independent Test**: Reader can name pytest (testing), pre-commit (hooks), MkDocs (docs) and explain "we'll learn these in later chapters, but here's a working config."

**Acceptance Scenarios**:

1. **Given** reader completes Ruff/Pyright lessons, **When** they reach "Professional Workflow" section, **Then** they see pytest config example with note "Chapter 42 teaches TDD in depth"
2. **Given** reader wants automated quality checks, **When** they see pre-commit config, **Then** they understand "runs Ruff/Pyright automatically before git commit" (concept, not full tutorial)
3. **Given** reader wants to document their project, **When** they see MkDocs mention, **Then** they know it exists and can explore later
4. **Given** reader wonders about CI/CD, **When** they see GitHub Actions snippet, **Then** they understand "automated checks on every push" (awareness, depth in Part 7)

---

### Edge Cases

**Tool Versioning Issues**:
- What happens when Zed, Ruff, or Pyright release breaking changes after chapter publication?
  - **Mitigation**: Include update triggers ("Review when Zed 1.x → 2.x") and version pins in examples (Ruff 0.6.9, Pyright 1.1.389)

**Integration Conflicts**:
- How does system handle conflicting LSP configurations (e.g., Pyright vs. Pylance)?
  - **Mitigation**: Document "Disable Pylance if using Pyright" and show LSP conflict resolution in Zed

**AI Tool Availability**:
- What if reader doesn't have Claude Code or Gemini CLI?
  - **Mitigation**: Show examples with BOTH tools + fallback: "or use any AI assistant with this prompt template"

**Existing Chapter 12 Users**:
- What happens to readers who completed "old" Chapter 12 (uv-only)?
  - **Mitigation**: Update chapter intro: "If you completed Chapter 12 before [date], lessons 7-12 are NEW content—start there"

**Platform-Specific Tool Gaps**:
- What if Zed doesn't support reader's platform (e.g., older Linux distros)?
  - **Mitigation**: Provide VS Code + Ruff/Pyright as alternative IDE integration example

---

## Requirements *(mandatory)*

### Functional Requirements

**Content Preservation**:

- **FR-001**: Chapter MUST preserve existing lessons 1-6 (uv content) without modification
- **FR-002**: Chapter metadata MUST be updated (new title, expanded learning objectives, updated readme.md)

**New Content Addition**:

- **FR-003**: Chapter MUST add 4-6 new lessons (numbered 7-12) covering Zed, Ruff, Pyright, and integration
- **FR-004**: New lessons MUST follow tool-by-tool sequence: uv (existing 1-6) → Zed IDE (7) → Ruff (8-9) → Pyright (10) → Integration (11-12)
- **FR-005**: Each new lesson MUST include: learning objectives, direct commands section, AI collaboration section, hands-on exercise, troubleshooting tips

**Zed IDE Coverage** (Lessons 7 + Integration):

- **FR-006**: Lesson 7 MUST teach: Zed installation (all platforms), opening uv projects, integrated terminal, basic keybindings
- **FR-007**: Zed content MUST demonstrate AI integration: Claude Code in Zed, Gemini CLI in Zed terminal, AI-assisted editing
- **FR-008**: Zed content MUST show LSP configuration for Python (Pyright integration preview)

**Ruff Coverage** (Lessons 8-9):

- **FR-009**: Lesson 8 MUST teach: Ruff purpose (linter + formatter replaces Black/isort), installation via uv, basic usage (`ruff format`, `ruff check`)
- **FR-010**: Lesson 9 MUST teach: Ruff configuration in pyproject.toml, rule selection, Zed integration (format-on-save, inline diagnostics)
- **FR-011**: Ruff examples MUST use verified configuration syntax (line-length, select/ignore rules, isort settings)

**Pyright Coverage** (Lesson 10):

- **FR-012**: Lesson 10 MUST teach: Pyright purpose (type checker for Python 3.13+), installation via uv, type checking modes (basic/strict)
- **FR-013**: Pyright content MUST demonstrate: catching type errors before runtime, type hint best practices, Zed LSP integration
- **FR-014**: Pyright examples MUST show real type errors (str vs int, None handling, generic types)

**Integration Coverage** (Lessons 11-12):

- **FR-015**: Lesson 11 MUST show complete workflow: uv init → Zed open → write code → Ruff format → Pyright check → all working together
- **FR-016**: Lesson 12 MUST provide pyproject.toml template with all tools configured (uv dependencies, Ruff settings, Pyright settings)
- **FR-017**: Integration lessons MUST include troubleshooting common issues (LSP not starting, Ruff not running on save, type errors cryptic)

**AI-First Teaching** (All New Lessons):

- **FR-018**: Each lesson MUST apply Graduated Teaching (Principle 13): direct commands for basics, AI companion for complex configs, AI orchestration for scale
- **FR-019**: Each lesson MUST include "Try with AI" section with 2-3 focused prompts (not 8+ verbose examples)
- **FR-020**: Lessons MUST show AI usage strategy clearly: Tier 1 (direct), Tier 2 (AI companion), Tier 3 (AI orchestration)

**Awareness-Level Content**:

- **FR-021**: Chapter MUST mention pytest, pre-commit, MkDocs with working config snippets
- **FR-022**: Awareness content MUST defer depth: "We'll learn [tool] in Chapter X, but here's a working config"
- **FR-023**: Config snippets MUST be copy-pasteable and functional (no broken examples)

**Constitutional Compliance**:

- **FR-024**: All lessons MUST respect B1 cognitive load limits (≤7 new concepts per section)
- **FR-025**: All tool examples MUST use modern Python standards (3.13+, type hints mandatory)
- **FR-026**: All durations MUST be realistic (installation 5-10 min, basic usage 10-15 min, integration 20-30 min)
- **FR-027**: All code examples MUST use verified facts (no hallucinated config options or commands)

### Key Entities *(content structure)*

**Lesson Structure**:
- Lesson number (7-12 for new content)
- Title (tool-focused: "Zed IDE for Python Development", "Ruff: Fast Python Linting and Formatting")
- Learning objectives (knowledge, skills, AI partnership)
- Duration estimate (realistic, not inflated)
- Prerequisites (previous lessons in chapter)
- Sections: Introduction, Direct Commands, AI Collaboration, Hands-On Exercise, Troubleshooting, Summary

**Tool Documentation**:
- Tool name (Zed, Ruff, Pyright)
- Purpose (why it exists, what problem it solves)
- Installation method (platform-specific)
- Basic usage (direct commands students execute)
- Configuration (AI-assisted for complexity)
- Integration points (how it connects with other tools in stack)

**Configuration Files**:
- pyproject.toml (uv dependencies, Ruff settings, Pyright settings)
- Zed settings.json (LSP configuration, keybindings, format-on-save)
- .pre-commit-config.yaml (awareness-level, copy-pasteable)
- Example GitHub Actions workflow (awareness-level, copy-pasteable)

---

## Assumptions

1. **Existing Chapter 12 Quality**: Assume lessons 1-6 (uv content) are pedagogically sound and don't require revision (confirmed by user: EXPAND, not replace)

2. **Tool Stability**: Assume Zed, Ruff, and Pyright APIs are stable enough for educational content (verified versions as of 2025-01-15):
   - **uv**: 0.5.18 (stable, weekly releases)
   - **Ruff**: 0.14.5 (stable, monthly releases)
   - **Pyright**: 1.1.407 (very stable, weekly releases)
   - **Zed**: Latest stable (beta, bi-weekly releases)
   - **Verification**: All commands and configurations tested in sandbox ([evidence](intelligence/001-verified-tool-documentation.md))
   - **Update Triggers**: Re-verify when major version bumps occur (Ruff 0.15+, Pyright 1.2+, uv 0.6+, Zed 1.0 GA)

3. **Reader Prerequisites**: Assume readers have completed Chapters 7 (Bash), 8 (Git), 11 (Context Engineering) and understand basic terminal commands

4. **AI Tool Access**: Assume readers have access to at least ONE AI CLI tool (Claude Code OR Gemini CLI OR equivalent) for hands-on exercises

5. **Platform Coverage**: Assume primary platforms are macOS, Windows (with WSL), and mainstream Linux distros (Ubuntu, Fedora). Edge platforms (BSD, Android Termux) get "community-supported" note.

6. **Python Version**: Assume readers are using Python 3.13+ (per Principle 3: Modern Python Standards)

7. **Internet Access**: Assume readers have internet for tool downloads and AI API calls (offline mode not covered)

8. **Time Investment**: Assume readers allocate 3-5 hours for complete chapter (6 existing lessons + 6 new lessons, ~30 min each)

9. **Learning Retention**: Assume readers practice hands-on exercises (not just read) for skill acquisition measured in Success Criteria

10. **Chapter Sequencing**: Assume Chapter 12 readers will encounter pytest (Chapter 42), FastAPI (Chapter 41), and Docker (Chapter 51) in later chapters, so awareness-level mentions are appropriate

---

## Scope Boundaries

### In Scope

✅ **Core Tools**: uv (existing), Zed IDE, Ruff linter/formatter, Pyright type checker
✅ **Integration**: All four tools working together in pyproject.toml and Zed
✅ **AI-Driven Workflows**: Show AI assistance for complex configurations (not memorization)
✅ **Cross-Platform Setup**: Installation and configuration for Windows/macOS/Linux
✅ **Awareness-Level Mentions**: pytest, pre-commit, MkDocs, GitHub Actions (configs shown, depth deferred)
✅ **Troubleshooting**: Common integration issues and AI-assisted debugging
✅ **Chapter Metadata Update**: New title, learning objectives, readme.md

### Out of Scope

❌ **Replacing Existing Lessons**: Lessons 1-6 (uv) remain unchanged (user confirmed EXPAND strategy)
❌ **Deep Pytest Coverage**: Testing frameworks taught in Chapter 42 (TDD)
❌ **Deep Pre-Commit Coverage**: Git hooks taught with CI/CD in Part 7 (Chapters 54+)
❌ **Deep MkDocs Coverage**: Documentation generation is Part 7 content
❌ **Alternative IDEs**: VS Code, PyCharm, Cursor get brief mentions but not full tutorials
❌ **Advanced Ruff Features**: Custom plugins, AST analysis, performance tuning (beyond B1 tier)
❌ **Advanced Pyright Features**: Stub files, type narrowing edge cases, generics deep-dive (advanced tier content)
❌ **Docker Integration**: Containerization is Part 7 (Chapter 51+)
❌ **CI/CD Deep Dive**: GitHub Actions, GitLab CI taught in Chapter 54
❌ **Quiz Revision**: Existing quiz (07_chapter_12_quiz.md) update deferred to implementation phase

### Deferred to Later Phases

🔄 **Phase 0.5 (Deep Research)**: Verify Zed, Ruff, Pyright configuration syntax against official docs (triggered after spec approval due to tool-specific claims)
🔄 **Lesson File Naming**: Determine exact lesson numbering (7-12 or 7-11 depending on integration split)
🔄 **Quiz Expansion**: Update existing quiz to cover Zed/Ruff/Pyright (implementation phase)
🔄 **Readme.md Rewrite**: Update chapter readme with new title and objectives (implementation phase)

---

## Dependencies

**Technical Dependencies**:
- Python 3.13+ (Chapter 13 prerequisite)
- uv package manager (lessons 1-6 teach installation)
- Bash/terminal access (Chapter 7 prerequisite)
- Git (Chapter 8 prerequisite)
- Internet access (tool downloads, AI API calls)

**Content Dependencies**:
- Chapter 7 (Bash Essentials): Terminal commands, PATH, environment variables
- Chapter 8 (Git and GitHub): Version control basics, .gitignore
- Chapter 11 (Context Engineering): AI collaboration patterns, effective prompting

**Tool Dependencies**:
- uv → Zed (Zed opens uv projects)
- uv → Ruff (installed via `uv add --dev ruff`)
- uv → Pyright (installed via `uv add --dev pyright`)
- Zed → Ruff/Pyright (LSP integration in Zed requires tools installed)

**Future Chapter References** (awareness mentions):
- Chapter 42: Test-Driven Agent Development (pytest deep dive)
- Chapter 54: CI/CD & Infrastructure-as-Code (pre-commit, GitHub Actions)
- Part 7: Production deployment (Docker, Kubernetes)

---

## Risks & Mitigation

**Risk 1: Tool Version Volatility**
- **Impact**: Zed/Ruff/Pyright APIs change, breaking chapter examples
- **Likelihood**: Medium (tools are stable but evolving)
- **Mitigation**: ✅ RESOLVED via Phase 0.5 verification
  - Verified versions: uv 0.5.18, Ruff 0.14.5, Pyright 1.1.407
  - All commands tested in sandbox ([evidence](intelligence/001-verified-tool-documentation.md))
  - Update triggers documented (re-verify on major version bumps)
  - Configuration syntax confirmed against official documentation (Context7 MCP)

**Risk 2: Cognitive Load Overflow**
- **Impact**: Adding 6 new tools/concepts overwhelms B1 readers
- **Likelihood**: Medium (4 tools in one chapter is a lot)
- **Mitigation**: Tool-by-tool progression (not all-at-once), graduated teaching (direct → AI), clear "why each tool" explanations, realistic durations

**Risk 3: Configuration Complexity**
- **Impact**: Readers get lost in pyproject.toml / Zed settings.json syntax
- **Likelihood**: High (config files are notoriously confusing)
- **Mitigation**: ✅ PARTIALLY RESOLVED via Phase 0.5 verification
  - AI-driven configuration (students describe intent, AI writes TOML/JSON)
  - All configuration examples verified in sandbox ([evidence](intelligence/001-verified-tool-documentation.md#4-configuration-verification))
  - Provide complete working templates with source citations
  - "Copy-paste this" sections for common configurations

**Risk 4: Platform-Specific Issues**
- **Impact**: Zed/tools don't work on reader's platform, chapter becomes unusable
- **Likelihood**: Low (Zed supports major platforms)
- **Mitigation**: Platform-specific installation notes, VS Code alternative for unsupported platforms, community troubleshooting section

**Risk 5: Existing Chapter 12 Integration**
- **Impact**: New lessons 7-12 don't flow smoothly from uv lessons 1-6
- **Likelihood**: Medium (adding content to finished chapter is tricky)
- **Mitigation**: Transition section between lesson 6 → 7 ("Now that you have uv, let's add professional IDE..."), chapter intro explains expansion, smooth narrative arc

**Risk 6: Hallucinated Configuration**
- **Impact**: Examples use fake Ruff rules or Pyright options that don't exist
- **Likelihood**: Medium (AI can hallucinate config syntax)
- **Mitigation**: ✅ RESOLVED via Phase 0.5 verification
  - 100% verification coverage (27/27 tool-specific claims verified)
  - All configuration examples tested in sandbox ([evidence](intelligence/001-verified-tool-documentation.md))
  - Context7 MCP fetched 24,000 tokens of official documentation (Ruff, Pyright, Zed)
  - Zero hallucinations detected in verified configurations
  - All lesson examples will include source citations (e.g., `# Source: Ruff docs v0.14.5`)

**Risk 7: Scope Creep**
- **Impact**: Chapter tries to cover too much (pytest, Docker, CI/CD), loses focus
- **Likelihood**: Medium (ChatGPT conversation mentioned many tools)
- **Mitigation**: Clear scope boundaries (awareness-level only for pytest/pre-commit/MkDocs), user confirmed "awareness + deep dive later" strategy

---

## Open Questions

1. **Lesson Split for Integration**: Should "integration" be ONE lesson (11) or TWO lessons (11: workflow, 12: complete template)?
   - **Recommendation**: TWO lessons. Lesson 11 = step-by-step workflow, Lesson 12 = complete template + troubleshooting. Prevents single lesson from being too long.

2. **Zed vs. VS Code Emphasis**: How much time to spend on "Zed is AI-first" vs. "VS Code works too"?
   - **Recommendation**: Zed as primary (2 lessons), VS Code alternative in sidebar note. Aligns with Principle 11 (Tool Diversity) without diluting focus.

3. **ChatGPT Config Snippets**: The ChatGPT conversation provides pyproject.toml, pre-commit, GitHub Actions, Dockerfile. Include all?
   - **Recommendation**: Include pyproject.toml (core), pre-commit (awareness), GitHub Actions (awareness), Dockerfile (defer to Chapter 51). Matches scope boundaries.

4. **Quiz Update Scope**: Existing quiz is 80K+ tokens (07_chapter_12_quiz.md). Update now or defer?
   - **Recommendation**: Defer to implementation phase. Spec focuses on lesson content; quiz update is mechanical expansion.

5. **Verified Intelligence Trigger**: Do tool-specific claims require Phase 0.5 (Deep Research)?
   - **Original Recommendation**: YES. Spec has many tool-specific claims (Ruff rules, Pyright modes, Zed LSP config).
   - **STATUS**: ✅ COMPLETED (2025-01-15)
   - **Results**: 100% verification coverage, zero hallucinations detected
   - **Evidence**: [001-verified-tool-documentation.md](intelligence/001-verified-tool-documentation.md)

---

## Next Steps

**Phase 0.5: Deep Research** — ✅ COMPLETED (2025-01-15)
- Context7 MCP fetched 24,000 tokens of official documentation (Ruff, Pyright, Zed)
- All configuration examples verified in sandbox
- Verified intelligence document created: [001-verified-tool-documentation.md](intelligence/001-verified-tool-documentation.md)
- Spec updated with source citations and verification evidence

**After Phase 0.5 Completion**:

1. **Phase 1: /sp.clarify** (OPTIONAL - SKIP IF SPEC SUFFICIENT)
   - Review spec for underspecified areas
   - Ask targeted clarifications (max 5 questions) if needed
   - **Current Assessment**: Spec appears complete, clarifications may not be needed

2. **Phase 2: /sp.plan** (NEXT STEP)
   - Invoke chapter-planner subagent to create detailed lesson plans
   - Generate `specs/001-chapter-12-lightning-python-stack/plan.md`
   - Map lessons 7-12 with teaching tiers and proficiency levels

4. **Phase 3: /sp.tasks**
   - Generate implementation task checklist
   - Create `specs/001-chapter-12-lightning-python-stack/tasks.md`

5. **Phase 4: /sp.implement**
   - Invoke lesson-writer subagent to write lessons 7-12
   - Update chapter readme.md and metadata
   - Verify all files written to `book-source/docs/04-Python-Fundamentals/12-python-uv-package-manager/`

6. **Phase 5: Validation**
   - Invoke technical-reviewer for quality gate
   - Run sandbox tests for all commands and configs
   - Verify against Success Criteria (SC-001 through SC-012)
