# Tasks: Chapter 7 Bash Essentials - Constitutional Compliance Refinement

**Input**: Design documents from `/specs/030-chapter-7-bash-refinement/`
**Prerequisites**: spec.md (approved), plan.md (approved)

**Tests**: No test tasks - this is educational content refinement, not software development

**Organization**: Tasks are grouped by refinement phase (Critical → Minor → Enhancement → Validation) to enable incremental validation and early regression detection.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different lesson files, no dependencies)
- Include exact file paths in descriptions
- Each task maps to specific constitutional principle

## Path Conventions

- Lesson files: `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/[NN]-[lesson-name].md`
- All paths relative to repository root

---

## Phase 1: Critical Structural Fixes (Priority 1)

**Purpose**: Fix constitutional violations that break Principle 7 (Minimal Content - single closure section)

**⚠️ CRITICAL**: These lessons violate the single closure requirement - content appears after "Try With AI" sections

### Lesson 2: Safety-First Pattern (Critical Violation)

- [ ] T001 Read `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-safety-first-pattern.md` lines 410-506 to understand current structure and identify content to move
- [ ] T002 Locate final "## Try With AI: Real Examples from Gemini" section (currently at line 413)
- [ ] T003 Cut lines 420-503 exactly (Real Example 1, Real Example 2, Real Example 3, Your Task, Key Insight paragraphs) - preserve text verbatim, no paraphrasing
- [ ] T004 Paste cut content INTO "## Try With AI: Real Examples from Gemini" section as subsections (### Real Example 1, ### Real Example 2, ### Real Example 3, ### Your Task: Compare Real vs. Textbook)
- [ ] T005 Add Three Roles marker after "Real Example 2" (line ~461): "**Observation**: Notice how Gemini asked YOU to clarify what 'old' means—this demonstrates AI as student, learning project-specific context from you."
- [ ] T006 Remove redundant "---" separators after content move, ensure exactly one final "---\n\n" at end of lesson
- [ ] T007 Verify diff shows only structural move (lines relocated but text unchanged) + one Three Roles marker addition (~30 words)
- [ ] T008 Run validation: `tail -50 book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-safety-first-pattern.md | grep -E "^## " | tail -1` should return "## Try With AI: Real Examples from Gemini"
- [ ] T009 Run validation: `awk '/^## Try With AI/,0' book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-safety-first-pattern.md | grep -E "^## (What's Next|Key Takeaways|Summary|Safety Note)"` should return zero matches

**Constitutional Mapping**: Principle 7 (Minimal Content) - single closure section only

**Estimated Duration**: 90 minutes

---

### Lesson 8: Real Project Troubleshooting (Critical Violation)

- [ ] T010 Read `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-project-troubleshooting.md` lines 560-610 to understand current structure and identify content to move
- [ ] T011 Locate final "## Try With AI: Orchestration at Scale (Advanced)" section (line 564)
- [ ] T012 Cut lines 603-605 exactly (closing paragraphs: "You can now supervise..." and "This is the foundation...") - preserve text verbatim
- [ ] T013 Paste cut content INTO "## Try With AI: Orchestration at Scale (Advanced)" section after "Key Insight" paragraph (before line 601's "---")
- [ ] T014 Add Three Roles marker after orchestration examples (~line 598): "**Observation**: Through this dialogue, you and AI converged on bash workflows neither of you had initially—you specified the goal, AI composed the pipeline, you verified safety. This is orchestration-scale co-learning."
- [ ] T015 Remove redundant "---" separators (lines 607-610), ensure exactly one final "---\n\n" at end of lesson
- [ ] T016 Verify diff shows only structural move (lines relocated but text unchanged) + one Three Roles marker addition (~40 words)
- [ ] T017 Run validation: `tail -50 book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-project-troubleshooting.md | grep -E "^## " | tail -1` should return "## Try With AI: Orchestration at Scale (Advanced)"
- [ ] T018 Run validation: `awk '/^## Try With AI/,0' book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-project-troubleshooting.md | grep -E "^## (What's Next|Key Takeaways|Summary|Safety Note)"` should return zero matches

**Constitutional Mapping**: Principle 7 (Minimal Content) - single closure section only

**Estimated Duration**: 90 minutes

---

**Checkpoint 1: Critical Fixes Validated** 🚧

After completing T001-T018, STOP and validate:

```bash
# Verify both critical lessons fixed
for lesson in 02-safety-first-pattern.md 08-real-project-troubleshooting.md; do
  echo "=== $lesson ==="
  tail -50 "book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/$lesson" | grep -E "^## " | tail -1
done
# Expected: Both return "## Try With AI"
```

**Pass Criteria**: Both lessons show "## Try With AI" as final section header

**If validation fails**: Review diff, check for unintended text changes, rollback if needed

---

## Phase 2: Minor Whitespace Normalization (Priority 2)

**Purpose**: Fix minor formatting violations (excessive trailing newlines violate Principle 7 single closure pattern)

**Note**: Lessons 3-7 end correctly with "Try With AI" but have excessive trailing whitespace

### Lesson 3: Understanding Navigation

- [ ] T019 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-understanding-navigation.md`
- [ ] T020 [P] Locate final "---" (around line 289-292)
- [ ] T021 [P] Remove all newlines after final "---" except exactly one, resulting in pattern: `---\n\n[EOF]`
- [ ] T022 [P] Verify: `tail -3 book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-understanding-navigation.md | cat -A` shows `---$\n$\n`

**Duration**: 15 minutes

---

### Lesson 4: Understanding File Operations

- [ ] T023 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-understanding-file-operations.md`
- [ ] T024 [P] Locate final "---" (around line 235-238)
- [ ] T025 [P] Remove all newlines after final "---" except exactly one, resulting in pattern: `---\n\n[EOF]`
- [ ] T026 [P] Verify: `tail -3 book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-understanding-file-operations.md | cat -A` shows `---$\n$\n`

**Duration**: 15 minutes

---

### Lesson 5: Configuration and Secrets

- [ ] T027 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-configuration-secrets.md`
- [ ] T028 [P] Locate final "---" (around line 245-248)
- [ ] T029 [P] Remove all newlines after final "---" except exactly one, resulting in pattern: `---\n\n[EOF]`
- [ ] T030 [P] Verify: `tail -3 book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-configuration-secrets.md | cat -A` shows `---$\n$\n`

**Duration**: 15 minutes

---

### Lesson 6: Packages and Dependencies

- [ ] T031 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-packages-dependencies.md`
- [ ] T032 [P] Locate final "---" (around line 296-299)
- [ ] T033 [P] Remove all newlines after final "---" except exactly one, resulting in pattern: `---\n\n[EOF]`
- [ ] T034 [P] Verify: `tail -3 book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-packages-dependencies.md | cat -A` shows `---$\n$\n`

**Duration**: 15 minutes

---

### Lesson 7: Pipes and Complex Commands

- [ ] T035 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-pipes-complex-commands.md`
- [ ] T036 [P] Locate final "---" (around line 311-314)
- [ ] T037 [P] Remove all newlines after final "---" except exactly one, resulting in pattern: `---\n\n[EOF]`
- [ ] T038 [P] Verify: `tail -3 book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-pipes-complex-commands.md | cat -A` shows `---$\n$\n`

**Duration**: 15 minutes

---

**Checkpoint 2: Minor Fixes Validated** 🚧

After completing T019-T038, STOP and validate:

```bash
# Verify whitespace normalization
for lesson in 03-understanding-navigation.md 04-understanding-file-operations.md 05-configuration-secrets.md 06-packages-dependencies.md 07-pipes-complex-commands.md; do
  echo "=== $lesson ==="
  tail -3 "book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/$lesson" | cat -A
done
# Expected: All show "---$\n$\n" pattern
```

**Pass Criteria**: All lessons show exactly one blank line after final "---"

---

## Phase 3: Three Roles Clarity Enhancement (Priority 3)

**Purpose**: Make Three Roles (AI as Teacher/Student/Co-Worker) visible to A2 learners through light narration

**Constitutional Mapping**: Section IIa (Stage 2 - Three Roles framework), User Decision (Clear But Natural)

**Approach**: Add 2-3 observational markers per Stage 2 lesson (Lessons 3-8) showing roles through natural narrative

### Lesson 3: Understanding Navigation (Stage 2)

- [ ] T039 Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-understanding-navigation.md`
- [ ] T040 After navigation comparison table (~line 260), add marker: "\n\n**Observation**: Notice how you and your AI use identical commands (`cd`, `..`) but in different locations. This is co-learning—you understand AI's navigation because you've practiced the same pattern yourself. When AI suggests a path, you can verify it makes sense.\n"
- [ ] T041 After safety verification prompt (~line 287), add marker: "\n\n**Key Insight**: By navigating yourself first, you built the mental model to supervise AI's navigation. You're not blindly trusting—you're collaborating from understanding.\n"
- [ ] T042 Verify word count increase <100 words total, tone is observational (not didactic)

**Duration**: 45 minutes

---

### Lesson 4: Understanding File Operations (Stage 2)

- [ ] T043 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-understanding-file-operations.md`
- [ ] T044 [P] After copy comparison table (~line 205), add marker: "\n\n**Observation**: Here you taught AI the safety verification pattern—before any file operation, check what exists. AI learns project-specific safety habits from your requirements.\n"
- [ ] T045 [P] After safety checklist prompt (~line 232), add marker: "\n\n**Key Insight**: Through this dialogue, you and AI established a safety protocol together. You specified the questions, AI internalized the pattern. This is collaborative safety culture.\n"
- [ ] T046 [P] Verify word count increase <80 words total, preserves surgical correction approach

**Duration**: 45 minutes

---

### Lesson 5: Configuration and Secrets (Stage 2)

- [ ] T047 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-configuration-secrets.md`
- [ ] T048 [P] After configuration comparison table (~line 180), add marker: "\n\n**Observation**: Notice how AI suggested both temporary and persistent approaches—it taught you configuration options. Then you chose based on your project needs—teaching AI your context. This bidirectional learning is the pattern.\n"
- [ ] T049 [P] After verification prompt (~line 220), add marker: "\n\n**Key Insight**: Configuration isn't about memorizing `export` syntax—it's about understanding persistence and verification through conversation with AI.\n"
- [ ] T050 [P] Verify word count increase <90 words total

**Duration**: 45 minutes

---

### Lesson 6: Packages and Dependencies (Stage 2)

- [ ] T051 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-packages-dependencies.md`
- [ ] T052 [P] After package installation comparison (~line 200), add marker: "\n\n**Observation**: AI knows which package manager to use (`pip`, `npm`, `brew`) because it understands your project context. You verify it worked through testing. Together you ensure dependencies are correctly installed.\n"
- [ ] T053 [P] After verification prompt (~line 275), add marker: "\n\n**Key Insight**: Package management through AI collaboration means you focus on understanding what's being installed (and whether it worked), not memorizing package manager flags.\n"
- [ ] T054 [P] Verify word count increase <90 words total

**Duration**: 45 minutes

---

### Lesson 7: Pipes and Complex Commands (Stage 2)

- [ ] T055 [P] Open `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-pipes-complex-commands.md`
- [ ] T056 [P] After pipeline comparison table (~line 287), add marker: "\n\n**Observation**: You specified WHAT you wanted (count Python files), AI built the HOW (pipeline). Then you traced the data flow to understand it. This is how complex workflows become understandable through AI collaboration.\n"
- [ ] T057 [P] After pipeline modification prompt (~line 310), add marker: "\n\n**Key Insight**: Through iterative refinement (trace → understand → modify → verify), you and AI converge on pipelines that solve your exact need. You're not memorizing syntax—you're reasoning about data transformations.\n"
- [ ] T058 [P] Verify word count increase <100 words total

**Duration**: 45 minutes

---

### Lesson 8: Real Project Troubleshooting (Already Enhanced in Phase 1)

- [ ] T059 Verify Lesson 8 already has Three Roles marker from T014 (orchestration-scale co-learning)
- [ ] T060 Review marker placement and tone for A2 appropriateness
- [ ] T061 If needed, adjust marker to be more observational: "**Observation**: Through this orchestration dialogue, you and AI converged on bash workflows neither of you had initially. You specified the goal, AI composed the pipeline, you verified safety—this is professional-scale co-learning."

**Duration**: 30 minutes

---

**Checkpoint 3: Three Roles Enhancement Validated** 🚧

After completing T039-T061, STOP and validate:

1. **Manual Review**: Read Lessons 3-8, verify Three Roles visible in 2+ examples per lesson
2. **Tone Check**: Markers use observational language ("Notice how...", "Here you..."), not didactic ("You must understand...")
3. **Word Count**: Each lesson increased by <100 words (~2-3 sentences per marker)
4. **A2 Appropriateness**: Language is beginner-friendly, no jargon without explanation

**Pass Criteria**: Reviewer identifies all three roles (Teacher/Student/Co-Worker) in at least 2 examples per lesson without roles being explicitly labeled

---

## Phase 4: Final Constitutional Validation

**Purpose**: Verify 100% constitutional compliance (up from 87.5% baseline)

### Full Compliance Audit

- [ ] T062 Run Check 1 (Final Section) on all 8 lessons: `for lesson in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/0*.md; do echo "=== $(basename $lesson) ==="; tail -50 "$lesson" | grep -E "^## " | tail -1; done` - Expected: All return "## Try With AI"
- [ ] T063 Run Check 2 (No Forbidden Sections) on all 8 lessons: `for lesson in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/0*.md; do echo "=== $(basename $lesson) ==="; awk '/^## Try With AI/,0' "$lesson" | grep -E "^## (What's Next|Key Takeaways|Summary|Safety Note)"; done` - Expected: All return zero matches
- [ ] T064 Run Check 3 (No Internal Scaffolding) on all 8 lessons: `grep -n "Stage [0-9]\|Layer [0-9]\|Three Roles (Framework|in Action)" book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/0*.md` - Expected: Zero matches
- [ ] T065 Document validation results: Create `specs/030-chapter-7-bash-refinement/VALIDATION-REPORT.md` with 24/24 checks passing (8 lessons × 3 checks each)

**Constitutional Checks**:
- Check 1: All lessons end with "Try With AI" only (8/8 = 100%)
- Check 2: No forbidden sections after "Try With AI" (8/8 = 100%)
- Check 3: No internal scaffolding exposed (8/8 = 100%)

**Total**: 24/24 checks = 100% compliance ✅

**Duration**: 45 minutes

---

### Content Preservation Verification

- [ ] T066 [P] Run diff analysis on Lesson 2: Verify structural move only (lines relocated but text unchanged) + Three Roles addition (~30 words)
- [ ] T067 [P] Run diff analysis on Lessons 3-7: Verify whitespace cleanup only (no content changes)
- [ ] T068 [P] Run diff analysis on Lesson 8: Verify structural move + Three Roles addition (~70 words total from T014 + T061)
- [ ] T069 Run word count comparison: `for lesson in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/0*.md; do wc -w "$lesson"; done` and compare to baseline - Expected: <10% variation per lesson
- [ ] T070 Manual review: Spot-check 3 dialogue examples across lessons - verify they're unchanged (preservation validation)

**Expected Diff Patterns**:
- Lessons 2, 8: Structural moves (content relocated) + Three Roles markers
- Lessons 3-7: Whitespace cleanup + Three Roles markers
- Lesson 1: No changes (already compliant)

**Red Flags** (investigate if found):
- Rewritten dialogue examples
- Removed bash commands
- Changed section headers
- Word count >10% variation

**Duration**: 60 minutes

---

### Bash Command Accuracy Testing

- [ ] T071 Extract all bash code blocks from all 8 lessons: `grep -A 5 '```bash' book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/0*.md > /tmp/chapter-7-commands.txt`
- [ ] T072 Review extracted commands for executability in test environment (macOS Bash 5.x, Linux Bash 4.0+)
- [ ] T073 Execute sample commands in sandbox: `pwd`, `ls -la`, `cd`, `mkdir`, `cp`, `export`, `pip install`, pipeline examples
- [ ] T074 Document any commands that fail (expected: zero failures except pedagogical errors in troubleshooting lesson)
- [ ] T075 If failures found: Verify they're intentional (Lesson 8 troubleshooting examples) or fix accuracy issues

**Pass Criteria**: 100% of commands either execute successfully or fail intentionally as documented

**Duration**: 60 minutes

---

**Checkpoint 4: Final Validation Complete** 🎯

After completing T062-T075, confirm:

- ✅ SC-001: All 8 lessons pass constitutional validation (24/24 checks)
- ✅ SC-002: Three Roles visible in 2+ examples per Stage 2 lesson
- ✅ SC-003: Content preservation verified (<10% word count variation)
- ✅ SC-004: All bash commands accurate and executable
- ✅ SC-005: Compliance score = 100% (up from 87.5%)

**Final Deliverable**: `specs/030-chapter-7-bash-refinement/VALIDATION-REPORT.md` documenting all checks passing

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Critical Fixes)**: No dependencies - start immediately
- **Checkpoint 1**: BLOCKS Phase 2 - must validate critical fixes before proceeding
- **Phase 2 (Minor Fixes)**: Depends on Checkpoint 1 passing
- **Checkpoint 2**: BLOCKS Phase 3 - must validate whitespace before enhancement
- **Phase 3 (Enhancement)**: Depends on Checkpoint 2 passing
- **Checkpoint 3**: BLOCKS Phase 4 - must validate Three Roles clarity before final validation
- **Phase 4 (Validation)**: Depends on Checkpoint 3 passing

### Within Each Phase

**Phase 1** (Sequential):
- T001-T009 (Lesson 2) must complete before T010-T018 (Lesson 8)
- Rationale: Learn correction pattern on first critical lesson, apply to second

**Phase 2** (Parallel):
- T019-T038 can ALL run in parallel (different lesson files, independent corrections)
- Marked [P] for parallel execution

**Phase 3** (Parallel):
- T039-T042 (Lesson 3), T043-T046 (Lesson 4), T047-T050 (Lesson 5), T051-T054 (Lesson 6), T055-T058 (Lesson 7) can run in parallel
- T059-T061 (Lesson 8) sequential (verifying existing marker from Phase 1)

**Phase 4** (Mixed):
- T062-T065 (constitutional checks) sequential (validates ALL lessons together)
- T066-T070 (diff analysis) marked [P] for parallel
- T071-T075 (bash testing) sequential (extraction → review → execution → documentation)

### Parallel Opportunities

**Maximum parallelization** (with 5 team members):
- Phase 1: Sequential (2 critical lessons, learn-then-apply pattern)
- Phase 2: 5 parallel tasks (Lessons 3-7 simultaneously)
- Phase 3: 5 parallel tasks (Lessons 3-7 simultaneously, Lesson 8 quick verification)
- Phase 4: Partial parallelization (diff analysis parallel, other tasks sequential)

**Single implementer**: Execute sequentially, validate at each checkpoint

---

## Implementation Strategy

### Surgical Refinement Approach

1. **Phase 1**: Fix critical violations (Lessons 2, 8) → Validate → Commit
2. **Phase 2**: Fix minor violations (Lessons 3-7) → Validate → Commit
3. **Phase 3**: Enhance Three Roles clarity → Validate → Commit
4. **Phase 4**: Final validation suite → Document → Commit

**Git Workflow** (phase-based commits):
- Commit 1: "fix: Chapter 7 Lessons 2, 8 structural corrections (Principle 7 compliance)"
- Commit 2: "fix: Chapter 7 Lessons 3-7 whitespace normalization"
- Commit 3: "enhance: Chapter 7 Three Roles clarity for A2 learners"
- Commit 4: "validate: Chapter 7 100% constitutional compliance (24/24 checks)"

### Validation-Gated Progression

- **Checkpoint 1** (after Phase 1): Critical fixes MUST pass before minor fixes begin
- **Checkpoint 2** (after Phase 2): Whitespace MUST be clean before enhancement begins
- **Checkpoint 3** (after Phase 3): Three Roles MUST be clear before final validation
- **Checkpoint 4** (after Phase 4): All checks MUST pass before declaring success

**Rationale**: Early validation gates prevent cascading errors, enable rollback at phase boundaries

### Content Preservation Safeguards

- **Exact text moves**: Cut/paste verbatim, no paraphrasing
- **Diff validation**: Every task includes expected diff pattern
- **Word count limits**: <10% variation per lesson
- **Manual spot-checks**: Review 3+ dialogue examples for preservation

**If preservation violated**: Rollback immediately, investigate, re-execute task

---

## Notes

- [P] tasks = different lesson files, no dependencies, can run in parallel
- Each task includes verification step (bash command or manual check)
- Constitutional principle mapped for each correction type
- Checkpoints enable rollback at phase boundaries
- Surgical approach preserves pedagogical strengths while fixing violations
- Three Roles narration uses A2-appropriate observational tone
- Validation-gated progression catches regressions early

---

## Success Criteria Summary

**Compliance Metrics**:
- Baseline: 87.5% (7 violations: 2 critical, 5 minor)
- Target: 100% (0 violations)
- Measurement: 24/24 constitutional checks passing (8 lessons × 3 checks)

**Quality Metrics**:
- Three Roles visible: 2+ examples per Stage 2 lesson (Lessons 3-8)
- Content preservation: <10% word count variation
- Bash accuracy: 100% commands executable (or pedagogically failing)
- A2 tone: Observational narration, no condescension

**Deliverables**:
- 8 refined lesson files (constitutional compliance)
- VALIDATION-REPORT.md (24/24 checks documented)
- 4 git commits (phase-based, with clear messages)
- PHR (meta-learning captured)

---

**Total Task Count**: 75 tasks
- Phase 1 (Critical): 18 tasks (2 lessons × 9 tasks each)
- Phase 2 (Minor): 20 tasks (5 lessons × 4 tasks each)
- Phase 3 (Enhancement): 23 tasks (6 lessons, varying complexity)
- Phase 4 (Validation): 14 tasks (constitutional + preservation + accuracy)

**Estimated Duration**: 10-14 hours (matches plan estimate)
- Phase 1: 3 hours
- Phase 2: 1.5 hours
- Phase 3: 4.5 hours
- Phase 4: 3 hours
- Buffer: 2 hours for adjustments

**Parallel Speedup**: With 5 team members, Phases 2-3 can reduce to ~30-45 min each (from 1.5hr and 4.5hr)
