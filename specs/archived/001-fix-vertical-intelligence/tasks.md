---
description: "Task list for fixing vertical intelligence core misalignment"
---

# Tasks: Fix Vertical Intelligence Core Misalignment

**Input**: Design documents from `/specs/001-fix-vertical-intelligence/`
**Prerequisites**: plan.md (required), spec.md (required)
**Branch**: `001-fix-vertical-intelligence`

**Tests**: No automated tests in this infrastructure fix. Validation through AI analysis + human checkpoints.

**Organization**: Tasks are organized in two phases following the implementation plan, with incremental validation checkpoints.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

---

## Phase 1: Emergency Synchronization (2 Weeks)

### Week 1: Documentation Layer Fixes

#### Day 1-2: Constitution Update (FR-001 to FR-004)

- [X] T001 Add header section explaining "Current Reality" vs "Future State" distinction in `.specify/memory/constitution.md`
- [X] T002 Update Current Reality section: document 14 chapters across Parts 1/2/3/5 (not 55 chapters) - delegated to specs/book/chapter-index.md to avoid constant updates
- [X] T003 Update Current Reality section: document semantic skills activation model (current implementation) - already in v3.0.0, plugin-based architecture
- [X] T004 Move aspirational content (55 chapters, 13 parts, plugin architecture) to clearly-marked Future State section
- [X] T005 Update terminology throughout constitution: keep "lessons" terminology as-is (no change needed)
- [X] T006 Document two-level chapter structure explicitly: delegated to output-styles templates
- [X] T007 Update book statistics: clarify counts - delegated to specs/book/chapter-index.md for current status
- [X] T008 Add cross-references linking Current Reality sections to corresponding Future State sections
- [X] T009 Run AI validation scan comparing old vs new constitution terminology, generate evidence report in `specs/001-fix-vertical-intelligence/validation/layer-constitution-validation.md`
- [X] T010 Human review: Domain expert approved constitution changes including v3.0.1 evals-first philosophy and authorized Phase 1 merge ✓

#### Day 3-4: Output Styles Rewrite (FR-005 to FR-008)

- [X] T011 Rewrite `.claude/output-styles/chapters.md`: correct structure to Part (Title-Case) → Chapter (lowercase) → readme.md (lowercase) ✓
- [X] T012 Add actual example from Chapter 1 to chapters.md: readme.md + 8 lesson files with descriptive names (01-moment_that_changed_everything.md style) ✓
- [X] T013 Update all terminology in chapters.md: kept "lessons" terminology (per user feedback), "13 parts aspirational" (not "7 parts") ✓
- [X] T014 Update `.claude/output-styles/lesson.md`: include actual YAML frontmatter example from Chapter 1, Lesson 1 (01-moment_that_changed_everything.md) ✓
- [X] T015 Document two-level structure in lesson.md: Chapter readme.md (overview + "What You'll Learn") vs Lesson files (YAML + content + "Try With AI") ✓
- [X] T016 Add metadata fields in YAML frontmatter: generated_by, source_spec, created, last_modified, git_author, workflow, version (changed from HTML comments to YAML per user feedback) ✓
- [X] T017 Add policy note for lesson authors: Within chapter, each lesson must end with single "Try With AI" section (no "Key Takeaways" or "What's Next") - already documented ✓
- [X] T018 Run AI validation comparing output styles against actual Chapter 1 structure, report discrepancies in `specs/001-fix-vertical-intelligence/validation/layer-output-styles-validation.md` ✓
- [X] T019 Human review: Domain expert approved template corrections with 3 adjustments: (1) descriptive filenames confirmed good, (2) metadata moved to YAML frontmatter instead of HTML comments, (3) added 3 additional metadata fields (last_modified, version) ✓

#### Day 5: Chapter Index Verification (FR-013 to FR-015)

**Note**: Originally planned to create PROJECT-STRUCTURE-REALITY.md, but **specs/book/chapter-index.md already exists** and serves this purpose. Tasks refocused on verification and enhancement.

- [X] T020 Verify `specs/book/chapter-index.md` accurately reflects current chapter status - found 14 implemented chapters (not 5) ✓
- [X] T021 Add completion status markers (✅ = exists, 📋 = planned) - already present, corrected from 5 to 14 implemented ✓
- [X] T022 Verify directory structure documentation in `specs/book/directory-structure.md` is current - verified accurate ✓
- [X] T023 Verify output-styles templates reference correct file structure examples - verified chapters.md and lesson.md reference chapter-index.md correctly ✓
- [X] T024 Update output-styles if needed to match actual content structure - completed in T011-T019 ✓
- [X] T025 Verify constitution cross-references to chapter-index.md are correct - verified in T001-T009 ✓
- [X] T026 Verify subagent instructions reference chapter-index.md for current status - deferred to T029-T042 (Week 2) ✓
- [X] T027 Run AI validation verifying chapter-index.md matches actual book-source/docs/ structure - validation report created with 19 corrections ✓
- [X] T028 Human review: Domain expert confirmed chapter-index.md accurately represents reality + added constitutional update for evals-first development (v3.0.1) ✓

### Week 2: Execution Layer Fixes + End-to-End Validation

#### Day 6-7: Subagent Instruction Alignment (FR-009 to FR-012)

- [X] T029 Update `.claude/agents/chapter-planner.md`: remove hardcoded skill counts ("8 skills", "9 skills") - removed hardcoded counts, added dynamic discovery from `.claude/skills/`, added evals-first validation ✓
- [X] T030 Add reference to specs/book/chapter-index.md in chapter-planner.md for current chapter status - added verification step with current count (14 chapters) ✓
- [X] T031 Update chapter-planner.md output specification: reference output-styles templates for structure - corrected `chapter-readme.md` → `chapters.md`, added `lesson.md` reference ✓
- [X] T032 Verify terminology in chapter-planner.md matches actual usage (keep "lessons" terminology) - verified mixed "sections/lessons" is correct per chapter type ✓
- [X] T033 Add instruction to chapter-planner.md: map skills with CEFR proficiency levels per skills-proficiency-mapper - already comprehensive (lines 107-144), no changes needed ✓
- [X] T034 Update `.claude/agents/content-implementer.md`: reference output-styles for structure examples - already present (line 83), no additional updates needed ✓
- [X] T035 Add reference to specs/book/chapter-index.md and book-source/docs/ for structure examples - already present (lines 150, 182), added book-source verification in step 5 ✓
- [X] T036 Update content-implementer.md to generate metadata comments showing: Generated by (subagent + version), Source spec (path), Created (date), Git author, Workflow (command) - added 7 metadata fields in YAML frontmatter (lines 88, 97, 197-200) ✓
- [X] T037 Clarify two-level output in content-implementer.md: reference output-styles/chapters.md and lesson.md for structure - references already present throughout ✓
- [X] T038 Update `.claude/agents/validation-auditor.md`: remove hardcoded skill count references - removed "All 9" and "8 domain skills", added dynamic discovery (lines 44, 501) ✓
- [X] T039 Add validation checks to validation-auditor.md: verify against output-styles templates and chapter-index.md - added validation steps (lines 89-90) ✓
- [X] T040 Add reference to specs/book/chapter-index.md in validation-auditor.md as current status reference - added chapter context validation (line 89) ✓
- [X] T041 Run AI scan of each subagent file, flag remaining contradictions in `specs/001-fix-vertical-intelligence/validation/layer-subagents-validation.md` - comprehensive validation report created with 12/12 updates documented ✓
- [X] T042 Human review: Domain expert approved subagent alignment for shipping despite old chapters needing redesign ✓

#### Day 8-9: Cross-Layer Validation Script (FR-016 to FR-018)

- [ ] T043 Create `scripts/validation/validate-vertical-intelligence.py` script file with header and imports
- [ ] T044 Implement constitution terminology scanner: extract key terms (sections vs lessons, readme case, part counts)
- [ ] T045 Implement output styles terminology scanner: extract same key terms from templates
- [ ] T046 Implement subagent instructions terminology scanner: extract key terms from all agent files
- [ ] T047 Implement content sampler: randomly select 3 existing chapters and extract actual structure/terminology
- [ ] T048 Implement cross-layer comparison logic: compare extracted terms across all 4 layers
- [ ] T049 Implement report generation: create markdown report with file:line references for any misalignments
- [ ] T050 Implement consistency score calculator: compute percentage match across layers (target: 100%)
- [ ] T051 Add CLI interface to validation script: accept options for verbosity, sample size, output format
- [ ] T052 Test validation script with intentional misalignment: verify it catches inserted inconsistencies
- [ ] T053 Human review: Domain expert reviews validation report format and approves script

#### Day 10: Test Chapter Generation + Metadata Tracking (End-to-End Validation)

- [ ] T054 Create test chapter specification in `specs/test-chapter/spec.md` with typical chapter requirements
- [ ] T055 Invoke corrected `/sp.plan` workflow with test chapter spec, capture plan.md output
- [ ] T056 Invoke corrected content-implementer workflow with test plan, capture readme.md and section files
- [ ] T057 Run AI analysis comparing generated test chapter files vs PROJECT-STRUCTURE-REALITY.md specification
- [ ] T058 Verify file naming correctness: descriptive section names (not generic), lowercase readme.md
- [ ] T059 Verify YAML frontmatter structure matches expected format from actual examples
- [ ] T060 Verify terminology consistency: all files use "sections" vocabulary, not "lessons"
- [ ] T061 Verify two-level architecture: chapter readme.md has overview + "What You'll Learn", sections have YAML + content + "Try With AI"
- [ ] T062 Verify metadata comments present in all generated files: Generated by, Source spec, Created, Git author, Workflow
- [ ] T063 Calculate Terminology Consistency Score: percentage of terms matching across 4 layers (target: 100%)
- [ ] T064 Calculate Structure Match Score: percentage of generated files matching PROJECT-STRUCTURE-REALITY.md spec (target: 100%)
- [ ] T065 Calculate Zero-Correction Rate: count manual fixes needed (target: 0)
- [ ] T066 Calculate Workflow Success Rate: workflow completed without contradictory instruction errors (target: 100%)
- [ ] T067 Generate comprehensive metrics report in `specs/001-fix-vertical-intelligence/validation/end-to-end-validation.md`
- [ ] T068 Human review: Domain expert examines test chapter output, confirms zero manual corrections needed, approves metrics as baseline

**Phase 1 Quick Validation** (completed before merge):
- [X] Quick validation check: Run validation-auditor on Chapters 31 & 32 to verify updated subagents work correctly
- [X] Results: 95% compliant with constitution v3.0.1, subagents functional, ready for writer handoff
- [X] Report: `specs/001-fix-vertical-intelligence/validation/quick-subagent-check.md`

**Phase 1 Checkpoint**: T001-T042 complete. SC-001 to SC-004 validated. Ready for merge and writer handoff.

**Note**: T043-T068 (validation script, test chapter generation) deferred to separate iteration. Writers will redesign existing chapters using updated subagents.

---

## Phase 2: Adaptive Intelligence Infrastructure (4 Weeks)

### Week 3: Sub-Agent Evaluation Framework (FR-019 to FR-022)

#### Day 11-12: Benchmark Suite Creation

- [ ] T069 [P] Create 50 test prompts for chapter-planner in `specs/benchmarks/chapter-planner/`: 20 normal, 15 edge, 10 incomplete, 5 ambiguous
- [ ] T070 [P] Create 50 test prompts for content-implementer in `specs/benchmarks/content-implementer/`: 20 normal, 15 edge, 10 incomplete, 5 ambiguous
- [ ] T071 [P] Create 50 test prompts for validation-auditor in `specs/benchmarks/validation-auditor/`: 20 normal, 15 edge, 10 incomplete, 5 ambiguous
- [ ] T072 [P] Create expected outputs (gold standard) for chapter-planner test prompts
- [ ] T073 [P] Create expected outputs (gold standard) for content-implementer test prompts
- [ ] T074 [P] Create expected outputs (gold standard) for validation-auditor test prompts
- [ ] T075 Document benchmark suite structure and usage in `specs/benchmarks/README.md`

#### Day 13-14: Evaluation Infrastructure

- [ ] T076 Create `scripts/evaluation/evaluate-subagent.py` script with argument parser for subagent selection
- [ ] T077 Implement test prompt loader: read all 50 prompts for specified subagent
- [ ] T078 Implement subagent invocation logic: run subagent against each prompt, capture output
- [ ] T079 Implement output comparator: compare generated output vs expected gold standard
- [ ] T080 Implement accuracy calculator: percentage of outputs matching expected structure/content (target: >95%)
- [ ] T081 Implement consistency checker: run same prompt 3 times, measure quality variance (target: >98% consistency)
- [ ] T082 Implement completeness validator: percentage of outputs with all required sections/files (target: >90%)
- [ ] T083 Implement cost metrics tracker: log tokens consumed, wall-clock time, API cost per operation
- [ ] T084 Implement failure logger: capture error patterns, identify which prompt types cause failures
- [ ] T085 Implement evaluation report generator: markdown report with pass/fail per success criterion
- [ ] T086 Test evaluation infrastructure: run against one subagent with 10 sample prompts

#### Day 15: Cost & Failure Analysis

- [ ] T087 Create `scripts/evaluation/analyze-failures.py` script for failure pattern analysis
- [ ] T088 Implement error pattern cataloguer: group failures by type (syntax, incomplete, contradictory)
- [ ] T089 Implement prompt type analyzer: identify which categories (normal/edge/incomplete/ambiguous) have highest failure rates
- [ ] T090 Implement retry strategy recommender: define which errors are recoverable and suggest retry approaches
- [ ] T091 Create cost dashboard script `scripts/evaluation/cost-dashboard.py`
- [ ] T092 Implement per-operation cost reporter: tokens per chapter/section/plan with averages
- [ ] T093 Implement time tracker: wall-clock duration per workflow invocation type
- [ ] T094 Implement ROI analyzer: cost trend over time vs quality metrics improvement
- [ ] T095 Generate initial benchmark evaluation report: run all 150 tests (50 × 3 subagents), document baseline metrics

### Week 4: User Feedback Integration (FR-023 to FR-025)

#### Day 16-17: Analytics Instrumentation

- [ ] T096 Design "Try With AI" tracking schema: exercise_id, started_timestamp, completed_timestamp, time_spent, success_indicator, optional_comment
- [ ] T097 Create tracking script template for "Try With AI" sections: minimal JavaScript/HTML for client-side logging
- [ ] T098 Implement feedback submission endpoint/script: capture success indicator ("Yes"/"No"/"Partial") and optional comment (max 280 chars)
- [ ] T099 Create `scripts/analytics/collect-feedback.py` for parsing log files
- [ ] T100 Implement completion metrics aggregator: calculate completion rates per chapter/section
- [ ] T101 Implement success rate calculator: percentage of "Yes" responses per section
- [ ] T102 Implement data storage: save analytics to CSV/JSON format (no PII, aggregate only)
- [ ] T103 Add privacy compliance: ensure no PII collection, provide opt-out mechanism
- [ ] T104 Test instrumentation: add tracking to 3 test sections, verify data collection works

#### Day 18: Community Input Channels

- [ ] T105 [P] Create GitHub issue template `.github/ISSUE_TEMPLATE/chapter-feedback.yml` for structured chapter feedback
- [ ] T106 [P] Create GitHub issue template `.github/ISSUE_TEMPLATE/exercise-problem.yml` for "Try With AI" exercise issues
- [ ] T107 Create embedded survey framework: design post-chapter survey with 5-star rating and optional comments (max 500 chars)
- [ ] T108 Create analytics dashboard script `scripts/analytics/dashboard.py`
- [ ] T109 Implement engagement metrics display: completion rate, average time spent, ratings per chapter
- [ ] T110 Implement problem area identifier: flag sections with low success rate or high confusion feedback
- [ ] T111 Implement weekly summary report generator: automated digest of key metrics and trends
- [ ] T112 Deploy community input channels: merge templates to main branch, announce to contributors

#### Day 19: A/B Testing Framework

- [ ] T113 Create `scripts/ab-testing/setup-test.py` for A/B test configuration
- [ ] T114 Implement test definition schema: 2 pedagogical variations (A and B) for same concept with descriptions
- [ ] T115 Implement random assignment logic: assign readers to variant A or B (50/50 split)
- [ ] T116 Implement variation tracker: log which variant each reader saw
- [ ] T117 Implement outcome metrics collector: completion rate, time-to-understanding, user ratings per variant
- [ ] T118 Implement statistical significance calculator: determine winner when p<0.05 and n>50
- [ ] T119 Implement auto-promotion logic: replace losing variant with winner when statistically significant
- [ ] T120 Document A/B testing process in `specs/ab-testing/README.md` with example test
- [ ] T121 Create example A/B test: "Explain decorators with restaurant metaphor (A) vs factory metaphor (B)"

### Week 5: Adaptive Intelligence System (FR-026 to FR-028)

#### Day 20-21: Drift-Detection Agent

- [ ] T122 Create `scripts/drift-detection/detect-drift.py` script for weekly misalignment scanning
- [ ] T123 Implement recent commit scanner: analyze git log for past 7 days, identify new/modified chapters
- [ ] T124 Implement constitution-content comparator: check if new chapters follow PROJECT-STRUCTURE-REALITY.md standards
- [ ] T125 Implement terminology drift detector: scan recent commits for new uses of "lessons", "README.md" (uppercase)
- [ ] T126 Implement structure validator: verify new chapters have correct two-level architecture (readme.md + sections)
- [ ] T127 Implement alert report generator: create markdown report with file:line references for violations
- [ ] T128 Implement notification system: send alert report via email or create GitHub issue
- [ ] T129 Configure weekly cron job or GitHub Action to run drift detection automatically
- [ ] T130 Test drift detection: introduce intentional violations in test commits, verify detection works

#### Day 22: Auto-Migration System

- [ ] T131 Create `scripts/migration/generate-migration.py` for version transition support
- [ ] T132 Implement version comparator: analyze constitution differences between specified versions (e.g., v3.0 → v4.0)
- [ ] T133 Implement impact analyzer: identify which existing chapters need updates based on version changes
- [ ] T134 Implement change requirement generator: list specific changes needed per affected chapter
- [ ] T135 Implement acceptance criteria creator: define validation checks per change type
- [ ] T136 Implement migration script generator: create executable bash/python script for applying changes
- [ ] T137 Implement dry-run mode: preview all changes without applying them
- [ ] T138 Implement migration task checklist generator: create markdown checklist from analysis
- [ ] T139 Test auto-migration: run script for hypothetical v3.0 → v4.0 transition, review generated tasks

#### Day 23: AI Tool Evolution Monitoring

- [ ] T140 [P] Create `scripts/monitoring/track-tool-updates.py` for AI tool release monitoring
- [ ] T141 [P] Implement release notes fetcher: monitor Claude Code, Gemini CLI, OpenAI Codex release pages/APIs
- [ ] T142 [P] Implement changelog parser: extract key changes from release notes (new features, breaking changes, performance improvements)
- [ ] T143 [P] Implement benchmark tester: run new AI tool version against existing benchmark suite (from Week 3)
- [ ] T144 [P] Implement metrics comparator: compare accuracy, cost, time between old and new versions
- [ ] T145 [P] Implement upgrade recommender: suggest adoption if quality improves >10% OR cost reduces >20%
- [ ] T146 [P] Implement breaking change detector: flag changes requiring subagent instruction updates
- [ ] T147 [P] Implement upgrade report generator: create markdown report with recommendation and rationale
- [ ] T148 Schedule monthly monitoring: configure cron job or GitHub Action to check for tool updates

### Week 6: Scalability Infrastructure (FR-029 to FR-031)

#### Day 24-25: Multi-Model Orchestration

- [ ] T149 Create `scripts/orchestration/model-selector.py` for optimal AI model routing
- [ ] T150 Define task type taxonomy: planning, code generation, validation, content writing, analysis
- [ ] T151 Implement model mapping: associate each task type with optimal AI model (Claude for planning, Gemini for code, etc.)
- [ ] T152 Implement load balancer: distribute requests across multiple API endpoints to avoid rate limits
- [ ] T153 Implement fallback handler: if primary model unavailable, route to secondary model
- [ ] T154 Implement cost optimizer: select most cost-efficient model meeting quality threshold for each task
- [ ] T155 Update subagent invocation wrapper to use model selector
- [ ] T156 Test multi-model orchestration: run diverse tasks, verify correct model selection

#### Day 26: Load Simulation

- [ ] T157 Create `scripts/scalability/load-test.py` for parallel workflow testing
- [ ] T158 Create 10 diverse test chapter specifications for load simulation
- [ ] T159 Implement parallel workflow launcher: invoke chapter-planner → content-implementer for all 10 chapters simultaneously
- [ ] T160 Implement resource monitor: track tokens consumed, wall-clock time, API costs during load test
- [ ] T161 Implement error collector: log any failures, timeouts, or degraded quality during parallel execution
- [ ] T162 Implement bottleneck identifier: analyze which operations serialize unnecessarily
- [ ] T163 Implement parallelization recommender: identify tasks that could run concurrently
- [ ] T164 Implement 55-chapter simulation: extrapolate from 10-chapter results to estimate full book scope feasibility
- [ ] T165 Generate scalability report: document findings, recommendations, and capacity projections

#### Day 27: Cost Control Mechanisms

- [ ] T166 Create `scripts/cost-control/budget-monitor.py` for spending tracking
- [ ] T167 Implement budget threshold setter: configure limits per operation type (chapter, section, plan)
- [ ] T168 Implement real-time usage tracker: monitor token consumption against thresholds
- [ ] T169 Implement alert system: send notification (email or Slack) when usage exceeds threshold
- [ ] T170 Implement cost reporter: generate daily/weekly per-chapter and per-section cost reports
- [ ] T171 Implement optimization recommender: identify prompt compression opportunities, caching potential
- [ ] T172 Implement model switching suggester: recommend cheaper models when quality difference is minimal (<5%)
- [ ] T173 Implement ROI dashboard: visualize cost trend over time vs quality metrics improvement
- [ ] T174 Set initial budget thresholds based on Phase 1 baseline metrics
- [ ] T175 Test cost controls: simulate high-usage scenarios, verify alerts trigger correctly

**Phase 2 Checkpoint**: Verify SC-008 to SC-011 achieved over 3-month validation period.

---

## Phase 2 Maintenance & Integration (Ongoing)

### Quarterly Benchmark Maintenance

- [ ] T176 Quarterly benchmark review: Update 50 test prompts per subagent based on real production failures from past 3 months, refresh gold standards to reflect evolved best practices in `specs/benchmarks/[subagent-name]/`

### Phase 2 Integration Testing

- [ ] T177 [P] End-to-end integration test: Trigger intentional drift (add chapter violating standards), verify drift-detection catches within 7 days, cost controls flag budget threshold breach, user feedback dashboard aggregates test responses
- [ ] T178 [P] Cross-system integration test: Run benchmark evaluation → sub-agent fails threshold → drift-detection flags regression → auto-migration generates remediation plan → verify full adaptive loop works
- [ ] T179 Multi-model load test: Generate 3 chapters using model selector (Claude for planning, Gemini for code examples, Claude for validation), verify orchestration routes correctly, cost optimization selects appropriate models, load balancing prevents rate limits

### Success Verification Checkpoints

- [ ] T180 After T068 (Day 10 validation): Verify SC-001 through SC-007 all pass, confirm Phase 1 success before proceeding to Phase 2
- [ ] T181 After T095 (Week 3 complete): Verify SC-008 achieved (>95% accuracy on 150 benchmark tests), review failure patterns, confirm quality thresholds met
- [ ] T182 After T121 (Week 4 complete): Verify SC-009 operational (user feedback instrumentation in >80% sections, channels established, beta test collected 20+ responses)
- [ ] T183 After T148 (Week 5 complete): Verify SC-010 functional (drift-detection runs weekly, auto-migration generates valid scripts, tool monitoring tracks updates)
- [ ] T184 After T175 (Week 6 complete): Verify SC-011 demonstrated (10-chapter parallel test succeeds, cost controls active, multi-model orchestration works)
- [ ] T185 Three-month validation checkpoint: Measure zero undetected drift over 3 months, 50+ user feedback responses per month sustained, costs trending downward

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 Week 1**: Constitution update → Output styles correction → PROJECT-STRUCTURE-REALITY.md creation (sequential within week, some parallelization possible)
- **Phase 1 Week 2**: Depends on Week 1 completion (subagents reference updated docs)
- **Phase 1 Day 10**: Depends on Days 1-9 completion (test chapter uses all corrected layers)
- **Phase 2 Week 3**: Depends on Phase 1 completion (benchmarks test corrected subagents)
- **Phase 2 Week 4**: Can start after Week 3 (independent user feedback system)
- **Phase 2 Week 5**: Week 5 Day 20-21 depends on Week 3 completion (drift detection validates against benchmark baseline)
- **Phase 2 Week 6**: Can start after Week 3 (uses benchmark suite for load testing)

### Within Phase 1

- T001-T010 (Constitution) must complete before T011-T019 (Output Styles) - subagents read constitution
- T011-T019 (Output Styles) must complete before T020-T028 (PROJECT-STRUCTURE-REALITY) - serves as examples source
- T020-T028 (PROJECT-STRUCTURE-REALITY) must complete before T029-T042 (Subagents) - subagents reference it
- T001-T042 (All docs) must complete before T054-T068 (Test Chapter) - test validates all layers

### Within Phase 2

- Week 3 (Benchmark Suite) must complete before Week 5 (Drift Detection) - uses benchmarks as baseline
- Week 3 (Evaluation) must complete before Week 6 Day 26 (Load Simulation) - uses benchmark suite for testing
- Week 4 (User Feedback) is independent - can run in parallel with Weeks 5-6

### Parallel Opportunities

**Phase 1**:
- T002-T003 (constitution sections) can run in parallel
- T011-T015 (output styles files) can be updated in parallel
- T021-T026 (PROJECT-STRUCTURE-REALITY sections) can be written in parallel
- T029-T040 (subagent files) can be updated in parallel

**Phase 2 Week 3**:
- T069, T070, T071 (benchmark creation for 3 subagents) can run in parallel
- T072, T073, T074 (gold standard creation for 3 subagents) can run in parallel

**Phase 2 Week 4**:
- T105, T106 (GitHub issue templates) can be created in parallel

**Phase 2 Week 6**:
- T149-T156 (Multi-model orchestration) can overlap with T166-T173 (Cost control) - different systems

---

## Parallel Example: Constitution Update (Day 1-2)

```bash
# Can work on these sections concurrently:
Task: "Update Current Reality section: document 14 chapters..." (T002)
Task: "Update Current Reality section: document semantic skills..." (T003)
Task: "Update terminology throughout constitution..." (T005)
```

---

## Implementation Strategy

### MVP: Phase 1 Only (Unblock Authors)

1. Complete Week 1 (Documentation Layer): Constitution, Output Styles, PROJECT-STRUCTURE-REALITY.md
2. Complete Week 2 (Execution Layer): Subagents, Validation Script, Test Chapter
3. **STOP and VALIDATE**: Run end-to-end test chapter generation (Day 10)
4. If SC-001 to SC-007 all pass → Merge to main, unblock authors
5. Phase 1 delivers immediate value: authors can create content without contradictions

### Incremental Delivery

1. Phase 1 completion → **Deploy**: Authors have working, aligned system
2. Phase 2 Week 3 → **Add**: Evaluation framework gives quality metrics
3. Phase 2 Week 4 → **Add**: User feedback system enables data-driven improvement
4. Phase 2 Week 5 → **Add**: Adaptive intelligence prevents future drift
5. Phase 2 Week 6 → **Add**: Scalability infrastructure enables full 55-chapter scope
6. Each phase adds intelligence without breaking previous work

### Staged Rollout (Minimize Disruption)

**Phase 1 Merge** (Days 11-12 after Day 10 validation):
1. Merge constitution changes first (lowest risk)
2. Merge output styles (medium risk, templates only)
3. Merge PROJECT-STRUCTURE-REALITY.md (no risk, new file)
4. Merge subagent instructions (higher risk, test immediately after)
5. Merge validation script (no risk, new file)
6. Monitor for 2 days (Days 13-14), ready to hotfix

**Phase 2 Rollout** (incremental):
1. Week 3: Internal evaluation framework (no user impact)
2. Week 4: User feedback (optional, non-intrusive)
3. Week 5: Adaptive intelligence (background processes)
4. Week 6: Scalability (infrastructure optimization, transparent)

---

## Success Metrics

### Phase 1 (SC-001 to SC-007)

- ✅ Terminology Consistency Score = 100% across all 4 layers
- ✅ Structure Match Score = 100% (generated files match PROJECT-STRUCTURE-REALITY.md)
- ✅ Zero-Correction Rate = 0 manual fixes needed for test chapter
- ✅ Workflow Success Rate = 100% (no contradictory instruction errors)
- ✅ Author satisfaction: zero complaints about increased difficulty/confusion
- ✅ Existing 14 chapters validated, migration complete in <2 hours total

### Phase 2 (SC-008 to SC-011)

- ✅ Sub-agent Accuracy: >95% on 150 benchmark tests
- ✅ Sub-agent Consistency: >98% across repeat runs
- ✅ Sub-agent Completeness: >90% generate all required outputs
- ✅ User Feedback: 50+ responses per month collected
- ✅ Drift Detection: Zero undetected misalignments over 3 months
- ✅ Load Simulation: 10 chapters generated in parallel successfully
- ✅ Cost Control: Budget alerts working, costs trending down over time

---

## Notes

- **Validation approach**: Hybrid AI-human model (AI analyzes and presents evidence, human reviews and confirms)
- **Incremental checkpoints**: After each layer (constitution, output styles, subagents) - human go/no-go decision
- **Test-driven validation**: Day 10 test chapter proves workflow works end-to-end before merge
- **Data-driven**: All Phase 2 systems include quantitative metrics (>95%, >98%, >90%, 50+/month targets)
- **Rollback safety**: Keep backup of old constitution/subagent instructions for easy rollback if needed
- **Communication**: Announce changes to authors during Days 11-12 merge window
- **Metadata tracking**: All generated content includes comments showing source spec, tool, git author, workflow
- **Lesson author policy**: Within each chapter, lessons must end with single "Try With AI" section (no "Key Takeaways" or "What's Next")
