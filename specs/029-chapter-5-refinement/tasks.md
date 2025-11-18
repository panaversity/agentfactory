# Tasks: Chapter 5 (Claude Code) Refinement

**Input**: Design documents from `/specs/029-chapter-5-refinement/`
**Prerequisites**: spec.md (success criteria), plan.md (lesson-by-lesson strategy), research.md (current state analysis)

**Organization**: Tasks grouped by implementation phase following risk-managed sequencing (LOW → MEDIUM → HIGH risk)

**Estimated Total Effort**: 14-19 hours distributed across 5 phases

---

## Format: `[ID] [P?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- Include exact file paths in descriptions
- All lesson files are in: `book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/`

---

## Phase 1: Metadata Enhancement (LOW RISK) — 2-3 hours

**Purpose**: Add constitutional-compliant metadata (stage, prerequisites, concept_count) to all 9 lessons

**Risk**: LOW (YAML additions, no content changes)

**Checkpoint**: All lessons have complete metadata, YAML validates successfully

### Metadata Tasks (All can run in parallel)

- [X] T001 [P] Add metadata (stage: L1, prerequisites, concept_count: 5) to 01-origin-story.md
- [X] T002 [P] Add metadata (stage: L1, prerequisites, concept_count: 8) to 02-installation-and-authentication.md
- [X] T003 [P] Add metadata (stage: L2, prerequisites, concept_count: 6) to 03-claude-md-context-files.md
- [X] T004 [P] Add metadata (stage: L2, prerequisites, concept_count: 6) to 04-mcp-integration.md
- [X] T005 [P] Add metadata (stage: L2, prerequisites, concept_count: 6) to 05-subagents-and-orchestration.md
- [X] T006 [P] Add metadata (stage: L2, prerequisites, concept_count: 7) to 06-agent-skills.md
- [X] T007 [P] Add metadata (stage: L2, prerequisites, concept_count: 5) to 07-hooks-and-extensibility.md
- [X] T008 [P] Add metadata (stage: L3, prerequisites, concept_count: 5) to 08-settings-hierarchy.md
- [X] T009 [P] Add metadata (stage: L4, prerequisites, concept_count: 8) to 09-plugins-putting-it-all-together.md

### Validation

- [X] T010 Validate YAML syntax on all 9 lesson files (use YAML parser or Docusaurus build test)
- [X] T011 Verify stage labels (L1, L2, L3, L4) match plan.md lesson mapping
- [X] T012 Verify concept_count values within tier limits (≤7 for A2 lessons, ≤10 for B1 lessons)

**Completion Criteria**:
- ✅ All 9 lessons have `stage`, `prerequisites`, `concept_count` fields
- ✅ YAML parses without errors
- ✅ Stage labels consistent with plan.md
- ✅ Concept counts within constitutional limits

---

## Phase 2: Easy Content Refinement (LOW-MEDIUM RISK) — 3-4 hours

**Purpose**: Add methodology framing ("Why This Matters"), enhance Three Roles demos, update README

**Risk**: LOW-MEDIUM (new sections, minimal existing content changes)

**Checkpoint**: Lessons with easy enhancements complete, README has Stage Progression section

### Lesson 1 (Already complete from Phase 1)

- [X] T013 Verify Lesson 1 needs no additional content changes (metadata-only per plan.md)

### Lesson 2: Add Methodology Framing

- [X] T014 Add "Why This Matters: Terminal Integration for AI Workflows" section (80-120 words) to 02-installation-and-authentication.md after intro, before Prerequisites
- [X] T015 Light content pruning in 02-installation-and-authentication.md to reduce from 289 to ~270-280 lines (trim explanatory prose, keep CLI commands intact)
- [X] T016 Verify line count ≤280 for 02-installation-and-authentication.md

### Lesson 3: Preserve Excellence

- [X] T017 Verify Three Roles demonstration in 03-claude-md-context-files.md is preserved exactly (reference-quality example)
- [X] T018 Optional: Add 1 sentence to "Why This Matters" connecting to Lessons 5+ (subagents/skills inherit CLAUDE.md context) in 03-claude-md-context-files.md
- [X] T019 Trim any redundancy in 03-claude-md-context-files.md if line count exceeds 350 after metadata

### Lesson 7: Add Three Roles Step

- [X] T020 Add Step 5 "Co-Design a Hook for Your Workflow" to 07-hooks-and-extensibility.md after Step 4 (explicit AI collaboration prompt showing three roles)
- [X] T021 Verify line count ≤300 for 07-hooks-and-extensibility.md

### Lesson 8: Strengthen Methodology

- [X] T022 Replace weak Expert Insight with explicit "Why Settings Hierarchy Matters for Team Intelligence" section (120-150 words) in 08-settings-hierarchy.md after "The Three Settings Levels", before "Precedence"
- [X] T023 Verify line count ≤350 for 08-settings-hierarchy.md

### README: Stage Progression

- [X] T024 Add "Stage Progression: L1→L2→L3→L4" section to README.md after "What You'll Learn" (see plan.md for full content)
- [X] T025 Add visual ASCII diagram showing L1→L2→L3→L4 progression to README.md
- [X] T026 Add "How to Use This Chapter" guidance section to README.md
- [X] T027 Verify README stage mapping matches all 9 lesson metadata `stage` fields

**Completion Criteria**:
- ✅ Lesson 2 has "Why This Matters" section (methodology-focused, not feature sales)
- ✅ Lesson 3 Three Roles example preserved exactly
- ✅ Lesson 7 has Three Roles co-design step
- ✅ Lesson 8 has strengthened "Why This Matters" section
- ✅ README has Stage Progression section with visual diagram
- ✅ All line counts within targets

---

## Phase 3: Medium Refinement (MEDIUM RISK) — 3-4 hours

**Purpose**: Add "Why This Matters" + Three Roles examples to Lessons 4 & 5

**Risk**: MEDIUM (new sections + content streamlining)

**Checkpoint**: Lessons 4 & 5 complete with enhanced methodology framing and Three Roles demos

### Lesson 4: MCP Integration

- [X] T028 Add "Why This Matters: Safe External Integration" section (80-120 words) to 04-mcp-integration.md after "Think of MCP Like This", before "What Is MCP?"
- [X] T029 Add Three Roles co-learning cycle to "Try With AI" section in 04-mcp-integration.md (explicit prompt showing AI explores, student refines, convergence on answer)
- [X] T030 Verify line count ≤350 for 04-mcp-integration.md (currently 225, new sections add ~100 lines → ~325 total)

### Lesson 5: Subagents & Orchestration

- [X] T031 Add "Why This Matters: Specialized Expertise" section (80-120 words) to 05-subagents-and-orchestration.md after "What Are Subagents?", before "The Plan Subagent"
- [X] T032 Add Step 2.5 "Co-learn Subagent Design" to 05-subagents-and-orchestration.md after Step 2 (Choose location) — explicit Three Roles cycle for subagent design
- [X] T033 Streamline "More Subagent Ideas" section in 05-subagents-and-orchestration.md from examples-heavy to concept-focused (save 30-40 lines to maintain line count ≤400)
- [X] T034 Verify line count ≤400 for 05-subagents-and-orchestration.md (target after additions + streamlining)

**Completion Criteria**:
- ✅ Lesson 4 has "Why This Matters" section + Three Roles example
- ✅ Lesson 5 has "Why This Matters" section + Three Roles co-design step
- ✅ Lesson 5 "More Subagent Ideas" streamlined
- ✅ All line counts within targets
- ✅ Methodology framing clear (workflow impact, paradigm connection)

---

## Phase 4: Critical Reductions (HIGH RISK) — 4-5 hours

**Purpose**: Reduce Lessons 6 & 9 significantly while preserving learning objectives

**Risk**: HIGH (39% and 47% reductions require surgical editing)

**Checkpoint**: Lessons 6 & 9 reduced to target line counts, learning objectives intact, methodology framing added

⚠️ **CRITICAL**: These tasks require careful judgment. Use plan.md guidance (sections to compress/cut) to preserve essential content while achieving line targets.

### Lesson 6: Agent Skills (547 → 330-340 lines, 39% reduction)

- [ ] T035 Compress "How Skills Differ from Subagents" section in 06-agent-skills.md from ~100 lines to ~40 lines (keep comparison table, remove redundant paragraphs)
- [ ] T036 Simplify "Three-Level Architecture" section in 06-agent-skills.md from ~150 lines to ~80 lines (brief summary per level, replace detailed example with reference to `.claude/skills/` directory)
- [ ] T037 Streamline "When Claude Code Invokes Skills" section in 06-agent-skills.md from ~80 lines to ~50 lines (keep patterns, use bullet points vs verbose explanations)
- [ ] T038 Consolidate "Try With AI" prompts in 06-agent-skills.md from ~60 lines to ~40 lines (keep 2 most essential prompts)
- [ ] T039 Trim "Hands-On: Create Your First Skill" in 06-agent-skills.md from ~60 lines to ~45 lines (keep step names, reduce descriptive prose)
- [ ] T040 Remove redundant sections in 06-agent-skills.md (~40 lines) — identify duplicate concepts between skills vs subagents
- [ ] T041 Add "Why This Matters: Reusable Organizational Capability" section (~100 words) to 06-agent-skills.md after skill definition, before "How Skills Differ"
- [ ] T042 Add one Three Roles example (~80 words) to 06-agent-skills.md hands-on section (ask Claude to review skill design, Claude suggests improvement, student adapts)
- [ ] T043 Verify line count 330-340 for 06-agent-skills.md (net change: -217 lines cut + 180 lines added = -37 net)
- [ ] T044 Cross-check 06-agent-skills.md against spec learning objectives — verify all essential content preserved

### Lesson 9: Plugins (744 → 350-400 lines, 47% reduction)

- [ ] T045 Eliminate exhaustive marketplace catalog in 09-plugins-putting-it-all-together.md from ~150 lines to ~40 lines (keep 2-3 representative examples, note "many more available")
- [ ] T046 Simplify plugin discovery section in 09-plugins-putting-it-all-together.md from ~80 lines to ~40 lines (one simple `/plugin` command, let users explore)
- [ ] T047 Consolidate installation methods in 09-plugins-putting-it-all-together.md from ~60 lines to ~35 lines (UI method as primary, CLI as alternative)
- [ ] T048 Replace "Test Your Installed Skills" in 09-plugins-putting-it-all-together.md from ~100 lines to ~50 lines (one concrete example, encourage exploration)
- [ ] T049 Streamline "Recognizing When Plugins Solve Your Needs" in 09-plugins-putting-it-all-together.md from ~80 lines to ~40 lines (single clear decision framework)
- [ ] T050 Trim "Understanding Plugin Architecture" in 09-plugins-putting-it-all-together.md from ~60 lines to ~30 lines (single diagram + brief text)
- [ ] T051 Remove redundancy in 09-plugins-putting-it-all-together.md (~40 lines) — consolidate repeated plugin definitions
- [ ] T052 Reduce "Try With AI" prompts in 09-plugins-putting-it-all-together.md from ~80 lines to ~40 lines (keep 2 most essential prompts)
- [ ] T053 Add "Why This Matters: Composition as Organizational Practice" section (~120 words) to 09-plugins-putting-it-all-together.md early, after plugin definition
- [ ] T054 Verify lesson 9 skill reference material preserved (advanced skill creation guide, MCP building guide — user-approved keep)
- [ ] T055 Verify line count 350-400 for 09-plugins-putting-it-all-together.md (net change: -344 lines cut + 120 lines added = -224 net)
- [ ] T056 Cross-check 09-plugins-putting-it-all-together.md against spec learning objectives — verify capstone integrates L1-L8 concepts effectively

**Completion Criteria**:
- ✅ Lesson 6 reduced to 330-340 lines (from 547)
- ✅ Lesson 9 reduced to 350-400 lines (from 744)
- ✅ Both lessons have "Why This Matters" sections (methodology-focused)
- ✅ Lesson 6 has Three Roles example
- ✅ Skill creation walkthrough in Lesson 6 still clear and actionable
- ✅ Lesson 9 skill reference material preserved
- ✅ All learning objectives from spec.md preserved in both lessons
- ✅ Capstone (L9) effectively integrates all chapter concepts

---

## Phase 5: Validation & Verification (MEDIUM RISK) — 4-5 hours

**Purpose**: Sandbox-test all code examples, commands, and workflows; verify metadata consistency

**Risk**: MEDIUM (may discover broken examples requiring fixes)

**Checkpoint**: All code verified working, SANDBOX-AUDIT-REPORT.md complete, metadata consistency validated

### Sandbox Testing (Parallelizable by lesson)

- [ ] T057 [P] Sandbox test: Execute all 4 installation methods from 02-installation-and-authentication.md in clean environment, verify `claude --version` output
- [ ] T058 [P] Sandbox test: Execute CLAUDE.md generation prompt from 03-claude-md-context-files.md in test project, verify file creation and auto-loading
- [ ] T059 [P] Sandbox test: Execute MCP add commands from 04-mcp-integration.md (Playwright, Context7), verify `claude mcp list` shows registered servers
- [ ] T060 [P] Sandbox test: Execute `/agents` workflow from 05-subagents-and-orchestration.md, verify interface and agent creation
- [ ] T061 [P] Sandbox test: Create test SKILL.md in `.claude/skills/` per 06-agent-skills.md, verify Claude discovery and invocation
- [ ] T062 [P] Sandbox test: Create test `.claude/settings.json` with hook per 07-hooks-and-extensibility.md, restart Claude Code, verify hook execution
- [ ] T063 [P] Sandbox test: Create `~/.claude/settings.json`, `.claude/settings.json`, `.claude/settings.local.json` per 08-settings-hierarchy.md, verify precedence behavior
- [ ] T064 [P] Sandbox test: Execute `/plugin marketplace add` and `/plugin` commands from 09-plugins-putting-it-all-together.md, install example-skills plugin, test skill discovery

### Verification & Reporting

- [ ] T065 Create SANDBOX-AUDIT-REPORT.md documenting all test results (execution logs, pass/fail status, any fixes required)
- [ ] T066 Verify all CLI commands produce expected output (cross-reference against `claude --help` and official docs)
- [ ] T067 Identify all feature claims across 9 lessons, verify citations to official Claude Code docs (https://code.claude.com/docs)
- [ ] T068 Verify metadata consistency: All 9 lessons have complete frontmatter (no undefined, no empty arrays, no placeholders)
- [ ] T069 Verify stage labels in lesson metadata match README Stage Progression mapping
- [ ] T070 Verify prerequisite chains are correct and non-circular (each lesson's prerequisites list concepts from earlier lessons)
- [ ] T071 Verify concept_count values match actual distinct concepts in lesson body (±1 margin acceptable)
- [ ] T072 Run final line count check on all 9 lessons, verify targets met (L1-5,7-8 ≤400, L6 ≤340, L9 ≤400)
- [ ] T073 Verify theory-to-practice ratio ≥60% hands-on for all lessons (manual count of practice blocks vs explanatory paragraphs)
- [ ] T074 Verify "Why This Matters" sections exist and are methodology-focused (not feature sales pitches) in lessons 2, 4, 5, 6, 8, 9
- [ ] T075 Verify Three Roles Framework demonstrated in lessons 3 (preserved), 4, 5, 6, 7 (enhanced)

**Completion Criteria**:
- ✅ SANDBOX-AUDIT-REPORT.md shows 100% code examples passed
- ✅ All CLI commands verified accurate
- ✅ All feature claims cited from official docs
- ✅ Metadata 100% complete across all 9 lessons
- ✅ Stage progression consistent (README ↔ lesson metadata)
- ✅ All line counts within targets
- ✅ Theory-to-practice ratio ≥60% for all lessons
- ✅ All success criteria from spec.md (SC1-SC7, FR1-FR7, AC1-AC7) verified met

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Metadata)**: No dependencies - can start immediately
- **Phase 2 (Easy Content)**: Should complete Phase 1 first (establishes metadata foundation)
- **Phase 3 (Medium Content)**: Can run in parallel with Phase 2 (different lessons)
- **Phase 4 (Critical Reductions)**: Should complete Phases 2-3 first (build confidence before high-risk edits)
- **Phase 5 (Validation)**: Depends on Phases 1-4 completion (validate final state)

### Task Dependencies Within Phases

**Phase 1**: T001-T009 all parallel → T010-T012 validation (sequential after T001-T009)

**Phase 2**:
- T014-T016 (Lesson 2) can run parallel with T017-T019 (Lesson 3)
- T020-T021 (Lesson 7) can run parallel with T022-T023 (Lesson 8)
- T024-T027 (README) can run parallel with any lesson tasks

**Phase 3**:
- T028-T030 (Lesson 4) can run parallel with T031-T034 (Lesson 5)

**Phase 4**:
- T035-T044 (Lesson 6) can run parallel with T045-T056 (Lesson 9)
- CAUTION: Both are HIGH RISK, may be safer to do sequentially to avoid context-switching errors

**Phase 5**:
- T057-T064 (sandbox tests) all parallel
- T065-T075 (verification) sequential after sandbox tests complete

### Parallel Opportunities

#### Phase 1: All Metadata Tasks
```bash
# Launch all 9 metadata additions together:
T001: "Add metadata to 01-origin-story.md"
T002: "Add metadata to 02-installation-and-authentication.md"
T003: "Add metadata to 03-claude-md-context-files.md"
T004: "Add metadata to 04-mcp-integration.md"
T005: "Add metadata to 05-subagents-and-orchestration.md"
T006: "Add metadata to 06-agent-skills.md"
T007: "Add metadata to 07-hooks-and-extensibility.md"
T008: "Add metadata to 08-settings-hierarchy.md"
T009: "Add metadata to 09-plugins-putting-it-all-together.md"
```

#### Phase 2: Easy Refinements
```bash
# Launch lesson groups in parallel:
T014-T016: "Lesson 2 enhancements"
T017-T019: "Lesson 3 verification"
T020-T021: "Lesson 7 Three Roles"
T022-T023: "Lesson 8 methodology"
T024-T027: "README Stage Progression"
```

#### Phase 5: Sandbox Tests
```bash
# Launch all 8 sandbox tests together:
T057: "Test Lesson 2 installation commands"
T058: "Test Lesson 3 CLAUDE.md generation"
T059: "Test Lesson 4 MCP commands"
T060: "Test Lesson 5 subagent workflow"
T061: "Test Lesson 6 skill creation"
T062: "Test Lesson 7 hook execution"
T063: "Test Lesson 8 settings precedence"
T064: "Test Lesson 9 plugin commands"
```

---

## Implementation Strategy

### MVP First (Phases 1-2)

1. Complete Phase 1: Metadata Enhancement (2-3 hours)
2. Complete Phase 2: Easy Content Refinement (3-4 hours)
3. **STOP and VALIDATE**: Test Docusaurus build, verify metadata, check line counts
4. **DECISION POINT**: Proceed to Phases 3-4 (critical reductions) or stop if satisfactory

### Incremental Delivery

1. Complete Phase 1 → Metadata foundation ready
2. Complete Phase 2 → Easy enhancements done (Lessons 1-3, 7-8, README)
3. Complete Phase 3 → Medium refinements done (Lessons 4-5)
4. Complete Phase 4 → Critical reductions done (Lessons 6, 9) — HIGHEST RISK
5. Complete Phase 5 → Full validation → Publication ready

### Sequential Strategy (Safer for HIGH RISK phases)

With single implementer (recommended for quality):

1. Complete Phases 1-3 (metadata + easy/medium content)
2. Tackle Lesson 6 reduction alone (90 min focused work)
3. Validate Lesson 6 against learning objectives
4. Tackle Lesson 9 reduction alone (120 min focused work)
5. Validate Lesson 9 against learning objectives
6. Run comprehensive validation (Phase 5)

---

## Rollback Points

If critical reductions (Phase 4) fail to preserve learning objectives:

**Lesson 6 Rollback**:
- Revert to post-Phase-1 state (metadata-only changes)
- Alternative strategy: Aim for less aggressive reduction (547 → 400 lines instead of 330)
- Accept 400-line limit as acceptable (still within constitutional range)

**Lesson 9 Rollback**:
- Revert to post-Phase-1 state (metadata-only changes)
- Alternative strategy: Preserve marketplace catalog, cut other sections less aggressively
- Accept 500-line limit if essential (with justification in ADR)

**Validation Failures (Phase 5)**:
- If code examples fail: Fix commands/workflows, re-test
- If metadata invalid: Fix YAML syntax, re-validate
- If concept counts wrong: Recount, update metadata
- Document all fixes in SANDBOX-AUDIT-REPORT.md

---

## Success Metrics

**Quantitative** (from spec.md):
- ✅ 9/9 lessons have complete metadata (100% coverage)
- ✅ Lesson 6 ≤340 lines, Lesson 9 ≤400 lines
- ✅ Average lesson length 250-350 lines
- ✅ Theory-to-practice ratio ≥60% hands-on
- ✅ All concept counts within tier limits (A2: ≤7, B1: ≤10)
- ✅ 100% code examples sandbox-tested and verified
- ✅ 100% feature claims cited from official docs

**Qualitative** (from spec.md):
- ✅ README stage progression explicit with visual diagram
- ✅ "Why This Matters" sections methodology-focused (not features)
- ✅ Three Roles examples demonstrate authentic co-learning
- ✅ Reduced lessons (6, 9) maintain learning objective clarity
- ✅ Chapter arc (L1→L2→L3→L4) clear and traceable

**Constitutional Compliance**:
- ✅ Specification Primacy: Methodology before features
- ✅ Progressive Complexity: A2-B1 tier limits enforced
- ✅ Factual Accuracy: All code tested, all claims cited
- ✅ Coherent Structure: L1→L2→L3→L4 progression visible
- ✅ Intelligence Accumulation: Lessons build toward capstone
- ✅ Anti-Convergence: Surgical targeting (not uniform refinement)
- ✅ Minimal Content: Non-goals honored (no new lessons, no comprehensive docs)

---

## Total Task Count

**Phase 1**: 12 tasks (9 parallel metadata + 3 validation)
**Phase 2**: 15 tasks (lesson enhancements + README)
**Phase 3**: 7 tasks (Lessons 4 & 5 medium refinement)
**Phase 4**: 22 tasks (Lessons 6 & 9 critical reductions)
**Phase 5**: 19 tasks (sandbox testing + verification)

**TOTAL**: 75 tasks distributed across 5 phases

**Parallel Opportunities**:
- Phase 1: 9 tasks parallel
- Phase 2: 5 task groups parallel
- Phase 3: 2 task groups parallel
- Phase 5: 8 tasks parallel

**Critical Path**: Phase 1 → Phase 4 (Lesson 6) → Phase 4 (Lesson 9) → Phase 5 (validation)

**Estimated Duration**: 14-19 hours (per plan.md)

---

## Notes

- **[P] tasks**: Different files, no dependencies on incomplete tasks
- **Metadata template** (Phase 1): See plan.md for exact YAML structure per lesson
- **"Why This Matters" template** (Phases 2-4): 80-150 words, methodology-focused (workflow impact + paradigm connection + real-world context)
- **Three Roles template** (Phases 2-4): AI teaches → Student teaches → Convergence loop (explicit demonstration)
- **Line count verification**: Use `wc -l` on each lesson file
- **Theory-to-practice ratio**: Count explanatory paragraphs vs. practice blocks (🤝 exercises, 💬 AI prompts, code "Try this")
- **Commit strategy**: Commit after each phase completion (not individual tasks) to enable clean rollbacks
- **Validation gates**: Do NOT proceed to next phase if current phase validation fails
- **Risk tolerance**: If Phase 4 (critical reductions) feels unsafe, revert and accept higher line counts with ADR justification
