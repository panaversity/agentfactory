# Implementation Plan: Fix Vertical Intelligence Core Misalignment

**Branch**: `001-fix-vertical-intelligence` | **Date**: 2025-11-04 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-fix-vertical-intelligence/spec.md`

## Summary

**Problem**: The vertical intelligence system (constitution → output styles → subagents → content) has critical misalignment. Every layer describes a different version of the project, causing contradictory instructions and degraded content quality.

**Solution**: Two-phase approach:
- **Phase 1** (2 weeks): Emergency synchronization of all 4 layers to describe same current reality
- **Phase 2** (4 weeks): Build adaptive intelligence infrastructure with evaluation, feedback loops, and drift detection

**Technical Approach**:
- Data-driven validation with quantitative metrics (>95% accuracy, >98% consistency, >90% completeness)
- Hybrid AI-human validation (AI analyzes and presents evidence, human confirms)
- Incremental layer-by-layer fixes with checkpoints
- Test-driven validation using generated test chapters

## Technical Context

**Language/Version**: Markdown (documentation), Python 3.13+ (validation scripts), Bash (automation)
**Primary Dependencies**:
- Claude Code (AI orchestrator)
- Existing subagents: chapter-planner, lesson-writer, technical-reviewer, proof-validator
- Existing skills: skills-proficiency-mapper, content-evaluation-framework, book-scaffolding

**Storage**: File-based (markdown, YAML frontmatter)
**Testing**:
- Validation scripts for terminology consistency
- Test chapter generation for end-to-end workflow validation
- Benchmark suite (50 prompts per subagent) for Phase 2

**Target Platform**: macOS/Linux development environment with git
**Project Type**: Infrastructure/Documentation (book authoring system fix)
**Performance Goals**:
- Phase 1: Zero manual corrections needed for generated content
- Phase 2: >95% sub-agent accuracy, 50+ user responses/month, zero drift over 3 months

**Constraints**:
- Backward compatibility with existing 14 chapters
- Minimal disruption to active authors
- 2-week timeline for Phase 1 (urgent)
- Budget <$500 for Phase 2 instrumentation

**Scale/Scope**:
- 4 layers to synchronize (constitution, output styles, subagents, content)
- 14 existing chapters to validate
- 31 functional requirements to implement
- 11 success criteria to achieve

## Constitution Check

*GATE: Infrastructure fixes must preserve constitution quality standards*

✅ **Preserves Core Principles**: Specification-first workflow, validation-first safety, graduated complexity
✅ **Maintains Quality Standards**: Python 3.13+, type hints, docstrings, PEP 8/257 compliance unchanged
✅ **Respects Architecture**: 13-part book structure aspirational state documented separately from current 4-part reality
✅ **Backward Compatible**: Existing 14 chapters remain functional during and after fix
✅ **No Complexity Violations**: This is fixing existing infrastructure, not adding new layers

**Post-Design Re-check Required**: After Phase 1 completion, verify constitution accurately describes current reality

## Two-Phase Implementation Strategy

### Phase 1: Emergency Synchronization (2 Weeks)

**Goal**: Unblock authors by synchronizing all 4 layers to describe same current reality

**Week 1: Documentation Layer Fixes**

**Day 1-2: Constitution Update** (FR-001 to FR-004)
- Add header explaining "Current Reality" vs "Future State" sections
- Update Current Reality: 14 chapters, 4 parts (1/2/3/5), semantic skills activation
- Move aspirational content to clearly-marked Future State section
- Update terminology throughout: "sections" not "lessons", "readme.md" not "README.md"
- Update book statistics: clarify 14 exist, 41 planned
- Document two-level chapter structure explicitly (Chapter README + Section files)

**Validation**: AI scans updated constitution, generates evidence report comparing old vs new terminology. Human reviews and approves.

**Day 3-4: Output Styles Rewrite** (FR-005 to FR-008)
- Rewrite `.claude/output-styles/chapters.md`:
  - Correct structure: Part (Title-Case) → Chapter (lowercase) → readme.md (lowercase)
  - Use actual example from Chapter 1 (readme.md + 8 section files with descriptive names)
  - Update all terminology: "sections" not "lessons", "13 parts aspirational"
- Update `.claude/output-styles/lesson.md`:
  - Include actual YAML frontmatter from 01-moment_that_changed_everything.md
  - Document two-level structure (Chapter README overview vs Section detailed content)
  - Add metadata comment requirements example

**Validation**: AI compares output styles against actual Chapter 1 structure, reports discrepancies. Human spot-checks 3 sample files.

**Day 5: PROJECT-STRUCTURE-REALITY.md Creation** (FR-013 to FR-015)
- Create `.claude/PROJECT-STRUCTURE-REALITY.md` as single source of truth
- Include sections:
  - Actual directory tree from book-source/docs/
  - Terminology definitions (Part → Chapter → Section, not "lessons")
  - Canonical example: Chapter 1 structure (readme.md + 8 sections)
  - Link to constitution sections describing Future State
  - Migration status tracker (which parts exist, which planned)

**Validation**: AI verifies all examples in doc match actual file structure. Human confirms canonical example represents best practice.

**Week 2: Execution Layer Fixes + End-to-End Validation**

**Day 6-7: Subagent Instruction Alignment** (FR-009 to FR-012)
- Update `chapter-planner.md`:
  - Remove hardcoded skill counts ("8 skills", "9 skills")
  - Reference PROJECT-STRUCTURE-REALITY.md for structure
  - Specify generating: chapter readme.md (overview) + section plans (YAML + content)
  - Use correct terminology throughout ("sections" not "lessons")
- Update `lesson-writer.md`:
  - Add note that despite filename, it writes "sections"
  - Reference PROJECT-STRUCTURE-REALITY.md for examples
  - Update to generate metadata comments (spec source, tool, git author, workflow)
  - Clarify two-level output: readme.md vs section files
- Update `technical-reviewer.md`:
  - Remove skill count references
  - Add validation checks for: terminology consistency, file naming, two-level structure
  - Reference PROJECT-STRUCTURE-REALITY.md as validation standard

**Validation**: AI scans each subagent file, flags remaining contradictions. Human reviews and confirms alignment.

**Day 8-9: Cross-Layer Validation Script** (FR-016 to FR-018)
- Create `scripts/validation/validate-vertical-intelligence.py`:
  - Scan constitution for terminology (sections, readme case, part counts)
  - Scan output styles for same terms
  - Scan subagent instructions for same terms
  - Compare against sample of actual content (pick 3 random chapters)
  - Generate report with file:line references for misalignments
  - Calculate consistency scores (target: 100%)

**Validation**: Run script, verify it catches intentionally-introduced misalignments. Human reviews report format.

**Day 10: Test Chapter Generation + Metadata Tracking** (End-to-End Validation)
- Create test chapter spec (`specs/test-chapter/spec.md`)
- Invoke corrected chapter-planner → lesson-writer workflow
- Capture all outputs: plan.md, tasks.md, readme.md, section files
- **AI Analysis**:
  - Compare generated files vs PROJECT-STRUCTURE-REALITY.md spec
  - Check file naming (descriptive, lowercase readme)
  - Verify YAML frontmatter matches expected structure
  - Verify terminology (sections not lessons)
  - **Verify metadata comments present** in all generated files:
    - Source spec used
    - Tool/subagent that created it
    - Git author
    - Workflow command
- Generate data-driven metrics:
  - Terminology Consistency Score: % terms matching across 4 layers
  - Structure Match Score: % files matching spec
  - Zero-Correction Rate: manual fixes needed (target: 0)
  - Workflow Success Rate: completed without errors (target: 100%)

**Validation**: Human reviews test chapter output, confirms no manual corrections needed. Approves metrics as baseline.

**Phase 1 Deliverables**:
- ✅ Updated constitution with Current/Future State separation
- ✅ Rewritten output styles matching actual implementation
- ✅ Aligned subagent instructions with consistent terminology
- ✅ PROJECT-STRUCTURE-REALITY.md as ground truth
- ✅ Validation script operational
- ✅ Test chapter proving workflow works correctly
- ✅ Metadata tracking implemented in subagents
- ✅ Baseline metrics established (consistency, structure match, zero-correction, workflow success)

### Phase 2: Adaptive Intelligence Infrastructure (4 Weeks)

**Goal**: Build systems that define "good" and maintain quality over time

**Week 3: Sub-Agent Evaluation Framework** (FR-019 to FR-022)

**Day 11-12: Benchmark Suite Creation**
- Create 50 test prompts per subagent (chapter-planner, lesson-writer, technical-reviewer):
  - 20 normal cases (standard chapter specs, typical content)
  - 15 edge cases (single-section chapters, heavy code examples, conceptual-only)
  - 10 incomplete specs (missing prerequisites, vague learning objectives)
  - 5 ambiguous requirements (conflicting success criteria, unclear audience)
- Store in `specs/benchmarks/[subagent-name]/`
- Create expected outputs for each prompt (gold standard)

**Day 13-14: Evaluation Infrastructure**
- Create `scripts/evaluation/evaluate-subagent.py`:
  - Run subagent against all 50 prompts
  - Compare output vs expected (structure, completeness, terminology)
  - Calculate metrics:
    - Accuracy: % outputs matching expected structure/content (target: >95%)
    - Consistency: % same prompt produces same quality across 3 runs (target: >98%)
    - Completeness: % outputs with all required sections/files (target: >90%)
  - Track cost metrics: tokens, time, API cost per operation
  - Log failures: capture error patterns, identify retry candidates
- Generate evaluation report with pass/fail per success criterion

**Day 15: Cost & Failure Analysis**
- Create `scripts/evaluation/analyze-failures.py`:
  - Catalogue common error patterns from benchmark failures
  - Identify which prompt types cause failures
  - Define retry strategies for recoverable errors
- Create cost dashboard showing:
  - Tokens consumed per chapter/section/plan
  - Wall-clock time per workflow invocation
  - API cost per operation type
  - ROI analysis (cost vs quality improvement over time)

**Validation**: Run benchmark suite, verify >95% accuracy achieved. Human reviews failure patterns, approves retry strategies.

**Week 4: User Feedback Integration** (FR-023 to FR-025)

**Day 16-17: Analytics Instrumentation**
- Modify "Try With AI" sections to include tracking:
  - Add hidden script logging: exercise started timestamp, completed timestamp, time spent
  - Add success indicator form: "Did this work? [Yes/No/Partial]" (optional submission)
  - Add feedback mechanism: "Optional comment (max 280 chars)"
- Create `scripts/analytics/collect-feedback.py`:
  - Parse log files for completion metrics
  - Aggregate success rates per chapter/section
  - Store in simple CSV/JSON format (no PII, aggregate only)

**Day 18: Community Input Channels**
- Create GitHub issue templates:
  - `chapter-feedback.yml`: Structured feedback form per chapter
  - `exercise-problem.yml`: Report issues with "Try With AI" exercises
- Create embedded survey framework:
  - After chapter completion: "Rate this chapter: [1-5 stars]" (optional)
  - Optional: "What worked well?" / "What was confusing?" (max 500 chars)
- Create analytics dashboard (`scripts/analytics/dashboard.py`):
  - Show engagement per chapter (completion rate, time spent, ratings)
  - Show problem areas (low success rate, high confusion feedback)
  - Generate weekly summary report

**Day 19: A/B Testing Framework**
- Create `scripts/ab-testing/setup-test.py`:
  - Define test: 2 pedagogical variations for same concept
  - Randomly assign readers to variant A or B
  - Track: completion rate, time-to-understanding, user ratings
  - Auto-promote winning variant when statistically significant (p<0.05, n>50)
- Example test: "Explain decorators with restaurant metaphor (A) vs factory metaphor (B)"

**Day 19.5: Beta Test (Risk Mitigation)**
- **Purpose**: Validate user feedback instrumentation works in production before full deployment
- **Scope**: Apply instrumentation to 2 test chapters (select from Parts 1-2, completed chapters)
- **Beta Test Activities**:
  - Add tracking code to "Try With AI" sections in 2 chapters
  - Deploy to staging/preview environment (or mark as "beta feedback" in production)
  - Invite 10-15 beta testers (contributors, early readers) to complete chapters
  - Collect 2 weeks of beta feedback data
  - Analyze: completion tracking works, success indicators captured, comments stored correctly
  - Verify: no privacy issues, no performance degradation, opt-out mechanism functions
- **Success Criteria**: Collect 20+ feedback responses, zero data collection errors, dashboard displays beta data accurately
- **Contingency**: If beta test reveals issues, pause Week 4 rollout, fix instrumentation, re-test before proceeding

**Validation**: Instrument 3 test chapters, collect 10+ feedback responses. Human reviews dashboard, confirms data accuracy. **Beta test with real users validates production readiness before full rollout.**

**Week 5: Adaptive Intelligence System** (FR-026 to FR-028)

**Day 20-21: Drift-Detection Agent**
- Create `scripts/drift-detection/detect-drift.py`:
  - Run weekly (cron job or GitHub Actions)
  - Compare constitution descriptions vs actual content structure:
    - Scan recent commits (past 7 days) for new chapters
    - Check if new content follows PROJECT-STRUCTURE-REALITY.md standards
    - Detect terminology drift (new uses of "lessons", "README.md")
  - Generate alert report with file:line references for violations
  - Send to designated reviewer (email or GitHub issue)

**Day 22: Auto-Migration System**
- Create `scripts/migration/generate-migration.py`:
  - Input: current version (v3.0), target version (v4.0)
  - Analyze constitution changes between versions
  - Generate migration script identifying:
    - Which chapters need updates (impact analysis)
    - Specific changes required per chapter
    - Acceptance criteria per change
  - Create migration task checklist (markdown format)
  - Include dry-run validation mode (preview changes without applying)

**Day 23: AI Tool Evolution Monitoring**
- Create `scripts/monitoring/track-tool-updates.py`:
  - Monitor release notes for: Claude Code, Gemini CLI, OpenAI Codex
  - Test new versions against benchmark suite (from Week 3)
  - Compare metrics: accuracy, cost, time vs current version
  - Recommend upgrade if quality improves >10% OR cost reduces >20%
  - Flag breaking changes requiring subagent updates
  - Generate upgrade recommendation report

**Validation**: Run drift detection on last 30 days of commits, verify it catches known issues. Human reviews migration script output for test v3→v4 transition.

**Week 6: Scalability Infrastructure** (FR-029 to FR-031)

**Day 24-25: Multi-Model Orchestration**
- Create `scripts/orchestration/model-selector.py`:
  - Define task types: planning, code generation, validation, content writing
  - Map optimal model per task (e.g., Claude for planning, Gemini for code)
  - Implement load balancing across API endpoints (avoid rate limits)
  - Add fallback strategies if primary model unavailable
  - Optimize costs by routing to most efficient model for task type
- Update subagent invocation to use model selector

**Day 26: Load Simulation**
- Create `scripts/scalability/load-test.py`:
  - Generate 10 test chapters simultaneously (parallel workflow invocations)
  - Measure: tokens consumed, time elapsed, API costs, errors encountered
  - Identify bottlenecks: which operations serialize unnecessarily?
  - Identify parallelization opportunities: which tasks can run concurrently?
  - Verify validation can handle full 55-chapter scope (simulated)
- Generate scalability report with recommendations

**Day 27: Cost Control Mechanisms**
- Create `scripts/cost-control/budget-monitor.py`:
  - Set budget thresholds per operation type
  - Alert when token usage exceeds thresholds (email or Slack)
  - Generate cost per chapter/section reports (daily/weekly)
  - Provide optimization recommendations:
    - Prompt compression opportunities
    - Caching potential (reuse generated plans)
    - Model switching suggestions (use cheaper model if quality sufficient)
  - ROI analysis dashboard: cost trend over time vs quality metrics

**Validation**: Run load simulation, verify 10 chapters generate successfully in parallel. Human reviews cost controls, sets appropriate thresholds.

**Phase 2 Deliverables**:
- ✅ Benchmark suite (50 prompts × 3 subagents = 150 tests)
- ✅ Evaluation infrastructure with >95% accuracy target
- ✅ Cost tracking and failure analysis operational
- ✅ User feedback collection (analytics, GitHub templates, surveys)
- ✅ A/B testing framework for pedagogical optimization
- ✅ Drift-detection agent (weekly runs, catches issues <7 days)
- ✅ Auto-migration system (generates scripts for version transitions)
- ✅ AI tool monitoring (tracks updates, recommends upgrades)
- ✅ Multi-model orchestration for cost optimization
- ✅ Load simulation proving 10x parallel capacity
- ✅ Cost control mechanisms with budget alerts

## Validation Strategy

**Hybrid AI-Human Model** (matching spec clarification):

### Incremental Layer Validation (During Phase 1)

After each layer fixed (constitution, output styles, subagents):

1. **AI Analysis**:
   - Runs validation checks for that layer
   - Scans for terminology consistency
   - Cross-references with actual content samples
   - Generates evidence report with specific file:line quotes

2. **Human Checkpoint**:
   - Reviews validation report
   - Confirms terminology changes correct
   - Approves evidence that layer describes actual implementation
   - Provides go/no-go decision for next layer

3. **Audit Trail**:
   - Each validation saved as markdown report
   - Stored in `specs/001-fix-vertical-intelligence/validation/layer-{name}-validation.md`

### Test-Driven End-to-End Validation (Phase 1 completion)

1. **Generate Test Chapter**:
   - Use corrected workflow with test spec
   - Capture all outputs (plan, tasks, readme, sections)

2. **Automated Analysis** (AI):
   - Compare generated vs PROJECT-STRUCTURE-REALITY.md
   - Check: file naming, YAML structure, terminology, two-level architecture, metadata comments
   - Calculate data-driven metrics (consistency score, structure match, zero-correction rate, workflow success)

3. **Human Review**:
   - Examines test chapter output
   - Validates content quality
   - Confirms zero manual corrections needed
   - Approves or requests iteration

### Success Validation Matrix

| Success Criterion | AI Validation Method | Human Confirmation | Data Metric |
|-------------------|---------------------|-------------------|-------------|
| SC-001: No contradictory instructions | Invoke workflow, capture logs, scan for errors | Author reviews workflow execution | Error count = 0 |
| SC-002: Content matches structure | Compare files vs spec, list discrepancies | Author spot-checks 3 files | Match rate = 100% |
| SC-003: Zero misalignments | Run validation script, report inconsistencies | Author reviews top 5 findings | Misalignment count = 0 |
| SC-004: Accurate information | Query AI about project state, compare to reality | Author validates 5 key facts | Accuracy rate = 100% |
| SC-005: Consistent terminology | Scan 4 layers for terms, flag mixing | Author confirms term choices | Consistency score = 100% |
| SC-006: Two-level structure documented | Check docs mention both levels with examples | Author validates examples match | Documentation completeness = 100% |
| SC-007: Minimal migration effort | Analyze 14 chapters, estimate changes | Author reviews migration plan | Estimated time < 2 hours |
| SC-008: Sub-agents pass benchmarks | Run 150 benchmark tests, calculate metrics | Author reviews failure patterns | Accuracy >95%, Consistency >98%, Completeness >90% |
| SC-009: User feedback operational | Check instrumentation in sections, test submission | Author validates dashboard shows data | 50+ responses/month |
| SC-010: Adaptive intelligence prevents drift | Run detection on 30-day history, verify catches known issues | Author reviews alert reports | Zero undetected drift over 3 months |
| SC-011: Scalability demonstrated | Run 10-chapter parallel test, measure resources | Author reviews load test results | Success without degradation |

## Rollout Strategy

**Phase 1 Rollout** (minimize disruption):

1. **Feature Branch Work** (Days 1-9):
   - All changes on `001-fix-vertical-intelligence` branch
   - No impact to main branch or active authors

2. **Validation Period** (Day 10):
   - Generate test chapter, collect metrics
   - If any SC-001 to SC-007 fails: fix and re-test
   - Do not proceed to rollout until all Phase 1 SC pass

3. **Staged Merge** (Day 11-12):
   - Merge constitution changes first (lowest risk)
   - Merge output styles (medium risk, templates only)
   - Merge subagent instructions (higher risk, test immediately after)
   - Merge validation script (no risk, new file)
   - Merge PROJECT-STRUCTURE-REALITY.md (no risk, new file)

4. **Author Communication**:
   - Announce changes in project Slack/Discord/email
   - Provide migration guide for any in-progress work
   - Offer support channel for questions

5. **Monitor Period** (Days 13-14):
   - Watch for new content generation
   - Check for error reports or confusion
   - Ready to hotfix if issues found

**Phase 2 Rollout** (incremental feature additions):

1. **Week 3-4**: Evaluation infrastructure (internal only, no user impact)
2. **Week 4**: User feedback (optional, non-intrusive)
3. **Week 5**: Adaptive intelligence (background processes, no user-facing changes)
4. **Week 6**: Scalability (infrastructure optimization, transparent to users)

## Risks & Mitigation

### Phase 1 Risks

**Risk 1: Existing authors have in-progress chapters on main branch**
- **Likelihood**: Medium
- **Impact**: High (merge conflicts, wasted work)
- **Mitigation**:
  - Announce freeze period for main branch during merge (Day 11-12)
  - Provide migration script for in-progress work
  - Offer 1:1 support for migrating in-progress chapters

**Risk 2: Test chapter validation fails, revealing deeper issues**
- **Likelihood**: Medium
- **Impact**: High (Phase 1 timeline extends)
- **Mitigation**:
  - Build buffer into timeline (Day 10 is validation, Days 11-12 for fixes)
  - Have fallback: if critical issues found, pause rollout, fix, re-validate

**Risk 3: Subagent updates break existing functionality**
- **Likelihood**: Low
- **Impact**: Critical (authors blocked)
- **Mitigation**:
  - Test subagent updates with 3 diverse chapter specs before merge
  - Keep backup of old subagent instructions (easy rollback)
  - Deploy during low-activity period (weekend)

### Phase 2 Risks

**Risk 4: Benchmark suite doesn't cover real-world edge cases**
- **Likelihood**: Medium
- **Impact**: Medium (false confidence in sub-agent quality)
- **Mitigation**:
  - Continuously expand benchmark suite as new edge cases discovered
  - Include real failed prompts from production as test cases
  - Review benchmark quarterly, update as book evolves

**Risk 5: User feedback collection has low adoption**
- **Likelihood**: Medium
- **Impact**: Low (Phase 2 success but less data-driven iteration)
- **Mitigation**:
  - Make feedback optional and quick (<30 seconds)
  - Explain value proposition (help improve book)
  - Incentivize participation (e.g., contributor credit)

**Risk 6: Drift detection generates false positives**
- **Likelihood**: Medium
- **Impact**: Low (reviewer fatigue, ignored alerts)
- **Mitigation**:
  - Tune detection thresholds based on first 2 weeks
  - Allow whitelist for intentional variations
  - Generate weekly digest (not alert per commit)

## Dependencies

**External Dependencies**:
- Claude Code API access (required for all phases)
- GitHub repository with Actions enabled (required for drift detection cron)
- Python 3.13+ runtime (required for validation/evaluation scripts)

**Internal Dependencies** (must complete before phase starts):
- Phase 1 Day 6 requires Days 1-5 complete (subagents reference updated constitution/styles)
- Phase 1 Day 10 requires Days 1-9 complete (test chapter uses all corrected layers)
- Phase 2 Week 3 requires Phase 1 complete (benchmarks test corrected subagents)
- Phase 2 Week 5 requires Week 3 complete (drift detection validates against benchmark baseline)

**Human Resource Dependencies**:
- Domain expert (author) available for human validation checkpoints (estimated 2-3 hours/week Phase 1, 1 hour/week Phase 2)
- Technical reviewer to approve script implementations (estimated 1 hour/week both phases)

## Resource Breakdown by Role

**Phase 1 (80 hours total, 2 weeks)**:
- **Domain Expert/Author**: 20 hours
  - Constitution review and approval: 2 hours
  - Output styles validation: 2 hours
  - PROJECT-STRUCTURE-REALITY review: 2 hours
  - Subagent alignment approval: 3 hours
  - Test chapter human review: 4 hours
  - Validation checkpoints: 7 hours (10 minutes per checkpoint × 42 tasks with checkpoints)
- **AI Orchestrator (Claude Code)**: 50 hours
  - Documentation updates: 20 hours
  - Validation script development: 15 hours
  - Test chapter generation: 5 hours
  - AI analysis reports: 10 hours
- **Technical Reviewer**: 10 hours
  - Script implementation review: 5 hours
  - End-to-end validation: 3 hours
  - Migration coordination: 2 hours

**Phase 2 (160 hours total, 4 weeks)**:
- **Domain Expert/Author**: 30 hours
  - Benchmark suite review: 5 hours
  - Gold standard validation: 8 hours
  - User feedback design review: 4 hours
  - A/B test approval: 3 hours
  - Drift detection configuration: 4 hours
  - Load simulation review: 3 hours
  - Cost threshold setting: 3 hours
- **AI Orchestrator (Claude Code)**: 110 hours
  - Benchmark creation (150 prompts): 25 hours
  - Evaluation infrastructure: 20 hours
  - User feedback instrumentation: 20 hours
  - Adaptive intelligence systems: 25 hours
  - Scalability infrastructure: 20 hours
- **Technical Reviewer**: 20 hours
  - Script/infrastructure review: 12 hours
  - Integration testing: 5 hours
  - Security/privacy review: 3 hours

**Total Effort**: 240 hours over 6 weeks (40 hours/week assuming single full-time contributor, or 20 hours/week with 2 parallel contributors)

## Tools Appendix

**Required Development Tools**:
- **Python 3.13+**: For validation scripts, evaluation framework, analytics
  - Setup: `python --version` to verify, install from python.org if needed
  - Virtual environment: `python -m venv .venv && source .venv/bin/activate`
  - Dependencies: Will be specified in requirements.txt during implementation
- **Git**: For version control, branch management, commit history analysis
  - Setup: `git --version` to verify, install via system package manager if needed
- **Claude Code CLI**: Primary AI orchestrator for content generation and analysis
  - Already in use (assumed available)
- **GitHub CLI (gh)**: For issue template management, PR creation, Actions setup
  - Setup: `brew install gh` (macOS) or `apt install gh` (Linux)
  - Auth: `gh auth login`

**Required Infrastructure**:
- **GitHub Actions**: For automated drift detection cron jobs, weekly monitoring
  - Setup: Create `.github/workflows/drift-detection.yml` during Phase 2 Week 5
  - Free tier sufficient for weekly runs
- **GitHub Issue Templates**: For community feedback collection
  - Setup: Create `.github/ISSUE_TEMPLATE/*.yml` during Phase 2 Week 4
- **Analytics Storage**: Simple file-based (CSV/JSON) for user feedback data
  - Location: `analytics/` directory (gitignored, not committed to repo)
  - No external database required

**Optional Tools** (recommended for Phase 2):
- **Slack/Discord Webhooks**: For drift detection alerts, cost threshold notifications
  - Setup: Configure webhook URLs in environment variables
  - Fallback: Email notifications or GitHub Issues if webhooks unavailable
- **Jupyter Notebooks**: For ROI analysis visualization, cost trend charting
  - Setup: `pip install jupyter matplotlib pandas`
  - Optional: Can use simple terminal output instead

**Environment Configuration**:
```bash
# .env file (create during implementation, add to .gitignore)
DRIFT_DETECTION_SLACK_WEBHOOK=https://hooks.slack.com/...
COST_ALERT_THRESHOLD_TOKENS=1000000
ANALYTICS_STORAGE_PATH=./analytics
BENCHMARK_SUITE_PATH=./specs/benchmarks
```

**Pre-Implementation Checklist**:
- [ ] Python 3.13+ installed and verified
- [ ] Git configured with user name and email
- [ ] GitHub CLI installed and authenticated
- [ ] Claude Code CLI access confirmed
- [ ] Repository cloned and branch created (`001-fix-vertical-intelligence`)
- [ ] Development environment ready (editor, terminal, documentation access)

## Success Definition

**Phase 1 Success** (all must pass to proceed to Phase 2):
- ✅ SC-001 to SC-007 all achieve 100% targets
- ✅ Test chapter generated without manual corrections
- ✅ Existing 14 chapters validated against new standards (migration complete)
- ✅ No author complaints about increased difficulty or confusion

**Phase 2 Success** (measured over 3-month validation period):
- ✅ SC-008: All subagents pass >95% of benchmark tests
- ✅ SC-009: Collect 50+ user feedback responses per month
- ✅ SC-010: Zero undetected drift incidents over 3 months
- ✅ SC-011: Load simulation proves 10-chapter parallel capability
- ✅ Cost per chapter tracked and optimized (downward trend)

**Long-Term Success Indicators**:
- Authors report faster chapter creation (measured via analytics)
- Fewer GitHub issues related to structure/terminology confusion
- Increased reader engagement (measured via feedback and completion tracking)
- Ability to scale to full 55-chapter scope without quality degradation

## Next Steps

After plan approval:

1. **Review & Approve Plan**: Domain expert reviews, confirms approach, approves to proceed
2. **Generate Tasks**: Run `/sp.tasks` to break down plan into actionable checklist with dependencies
3. **Begin Phase 1 Day 1**: Start constitution update following plan above
4. **Track Progress**: Use task checklist to monitor completion, update regularly
5. **Phase 1 Checkpoint**: Day 10 validation determines proceed/iterate decision
6. **Phase 2 Go/No-Go**: After Phase 1 success validation, approve Phase 2 start

---

**Plan Status**: Ready for review and `/sp.tasks` generation
**Estimated Total Effort**:
- Phase 1: 80 hours (2 weeks × 40 hours)
- Phase 2: 160 hours (4 weeks × 40 hours)
- Total: 240 hours over 6 weeks
