# Feature Specification: Chapter 7 Bash Essentials - Constitutional Compliance Refinement

**Feature ID**: `030-chapter-7-bash-refinement`
**Created**: 2025-01-17
**Status**: Draft - Awaiting Approval
**Type**: Educational Content Refinement (Surgical)
**Orchestrator**: LoopFlow v2.0 (Reasoning-Activated)

---

## Constitutional Grounding

**Constitution Version**: v6.0.0 (Reasoning Activation Redesign)
**Applicable Principles**:
- **Principle 7 (Minimal Content)**: Single closure section ("Try With AI" only), no forbidden sections, no internal scaffolding
- **Section IIa (4-Stage Framework)**: Three Roles demonstrated through narrative (not headers)
- **Principle 2 (Progressive Complexity)**: A2 tier (5-7 concepts, heavy scaffolding, max 2 options)
- **Principle 6 (Anti-Convergence)**: Dialogue-first hands-on discovery (varies from Ch 6 direct teaching)

**Audience Tier**: A2 (Aspiring/Beginner, Part 2, Chapter 7)

**Quality Standard**: Constitutional Compliance First (surgical corrections, not complete rewrite)

---

## Evals Section (Success Criteria)

### Pre-Refinement Baseline

**Current State** (from audit 2025-01-17):
- **Compliance Score**: 87.5% (7/8 lessons substantially compliant)
- **Critical Violations**: 2 (Lessons 2, 8 - content outside final section)
- **Minor Violations**: 5 (Lessons 3-7 - excessive trailing whitespace)
- **Strengths**: No forbidden closures, no scaffolding exposed, Three Roles present

### Post-Refinement Success Criteria

**SC-001: 100% Constitutional Compliance**
- **Measure**: All 8 lessons pass constitutional audit with zero violations
- **Test**: Run grep validation checks from Constitution:1020-1028
  ```bash
  # Check 1: Final section is "Try With AI"
  tail -50 lesson.md | grep -E "^## " | tail -1
  # Expected: "## Try With AI"

  # Check 2: No forbidden sections after "Try With AI"
  awk '/^## Try With AI/,0' lesson.md | grep -E "^## (What's Next|Key Takeaways|Summary|Safety Note)"
  # Expected: Zero matches

  # Check 3: No internal scaffolding
  grep -E "Stage [0-9]|Layer [0-9]|Three Roles (Framework|in Action)" lesson.md
  # Expected: Zero matches
  ```
- **Acceptance**: All 8 lessons return expected results

**SC-002: Three Roles Clarity Enhancement (A2-Appropriate)**
- **Measure**: Each Stage 2 lesson includes 2-3 light narrative markers showing Three Roles
- **Example**: "Notice how AI suggested a pattern you hadn't considered (teaching you), then you refined it with project-specific context (teaching AI), and together you converged on the optimal solution"
- **Test**: Manual review confirms roles visible but not heavy-handed
- **Acceptance**: Reviewer identifies all three roles in at least 2 examples per lesson without explicit headers

**SC-003: Content Preservation**
- **Measure**: Existing pedagogical strengths preserved (dialogue-first pattern, real examples, safety integration)
- **Test**: Compare pre/post word counts - variations <10% except for added Three Roles narration
- **Acceptance**: Core content unchanged, only structural corrections + light narration additions

**SC-004: Bash Command Accuracy**
- **Measure**: All bash examples execute correctly in test environment
- **Test**: Extract all code blocks, execute in clean bash shell, verify outputs match lesson descriptions
- **Acceptance**: 100% executable (or clearly marked as hypothetical where appropriate)

**SC-005: No New Violations Introduced**
- **Measure**: Refinement doesn't introduce new constitutional violations
- **Test**: Post-refinement audit shows improvement or maintenance of compliance, never regression
- **Acceptance**: Compliance score ≥ 87.5% (never decreases)

---

## Intent Section

### What We're Building

**Surgical constitutional compliance refinement** of Chapter 7 (Bash Essentials) to align with Constitution v6.0.0 while preserving the chapter's pedagogical strengths.

### Why This Matters

**Problem**: Chapter 7 was implemented (Nov 2024) before Constitution v6.0.0 (Jan 2025). Audit reveals 87.5% compliance with structural violations:
1. Content appearing after final "Try With AI" sections (Lessons 2, 8)
2. Excessive trailing whitespace (Lessons 3-7)
3. Three Roles present but could be clearer for A2 learners

**Impact**: Constitutional violations create inconsistency across book chapters and may confuse A2-tier learners about pedagogical patterns.

**Solution**: Fix structural violations (Priority 1) + enhance Three Roles clarity (Priority 2) while preserving existing strengths (dialogue-first pattern, real AI examples, integrated safety).

### Who This Serves

**Primary**: Future readers of Chapter 7 (A2-tier learners encountering bash through AI collaboration)
**Secondary**: Content maintainers ensuring constitutional consistency across all chapters
**Tertiary**: Validation auditors verifying compliance

### Success Definition

**Before**: 87.5% compliance, 7 violations (2 critical, 5 minor), Three Roles present but subtle
**After**: 100% compliance, 0 violations, Three Roles clear with light narration, all strengths preserved

---

## Requirements (Mandatory)

### Functional Requirements

**FR-001: Structural Violation Corrections (Priority 1)**
- **Lesson 2** (`02-safety-first-pattern.md`): Move lines 420-503 (Real Examples 1-3) INTO final "Try With AI" section
- **Lesson 8** (`08-real-project-troubleshooting.md`): Move lines 603-605 (closing paragraphs) INTO final "Try With AI" section before "---"
- **Lessons 3-7**: Remove excessive trailing newlines (keep exactly 1 blank line after final "---")

**FR-002: Three Roles Clarity Enhancement (Priority 2)**
- Add 2-3 light narrative markers per lesson showing AI as Teacher/Student/Co-Worker
- Markers should be 1-2 sentences maximum, embedded naturally in existing examples
- **Example pattern**: "Notice how [AI action] (this demonstrates [Role]) and [Student action] (this demonstrates [Role])"
- **Placement**: Within existing "Try With AI" sections or as inline narration in dialogue examples
- **Tone**: Observational, not didactic ("Notice how..." not "This is Stage 2 showing Three Roles")

**FR-003: Whitespace Normalization**
- All lessons end with: `[final "Try With AI" section content]\n\n---\n\n` (exactly one blank line before and after "---")
- Remove trailing newlines beyond this pattern

**FR-004: Constitutional Validation**
- All 8 lessons must pass grep validation checks (SC-001 tests)
- No internal scaffolding language in student-facing text
- Single closure section per lesson

### Non-Functional Requirements

**NFR-001: Content Preservation**
- Existing dialogue examples unchanged (except for added Three Roles narration)
- Real AI examples (Gemini CLI, Claude Code) preserved verbatim
- Safety integration patterns maintained
- Pedagogical arc intact (Foundation → Application → Integration → Mastery)

**NFR-002: A2-Appropriate Language**
- Three Roles narration uses beginner-friendly language
- No jargon without explanation
- Observational tone, not academic

**NFR-003: Verification Standard**
- All bash commands remain executable
- All file paths relative (no hardcoded absolute paths)
- Platform differences addressed in examples where present

---

## Constraints

### Technical Constraints

**TC-001: File Structure**
- 8 lesson files preserved (01-08)
- Filenames unchanged
- Directory location unchanged: `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/`

**TC-002: Markdown Compatibility**
- Docusaurus-compliant markdown (existing YAML frontmatter preserved)
- Code blocks with language tags maintained
- Internal links preserved

### Pedagogical Constraints

**PC-001: Cognitive Load**
- A2 tier: max 5-7 concepts per section (existing sections compliant, preserve this)
- Three Roles narration adds ≤1 concept per lesson (keeps within limits)

**PC-002: Stage Progression**
- Lessons 1-2: Stage 1 (Manual foundation) - preserve
- Lessons 3-8: Stage 2 (AI collaboration with Three Roles) - enhance clarity
- No Stage 3 or 4 content (Chapter 7 is foundational)

**PC-003: Teaching Modality**
- Dialogue-first hands-on discovery (already implemented, preserve)
- Varies from Chapter 6 direct teaching (anti-convergence compliant)

---

## Non-Goals (Explicitly Out of Scope)

**NG-001: Complete Content Rewrite**
- We are NOT rewriting lessons from scratch
- Existing pedagogy is sound, only structural/clarity fixes needed

**NG-002: Bash Content Expansion**
- We are NOT adding new bash commands or concepts
- Scope limited to compliance + clarity, not feature expansion

**NG-003: New Example Creation**
- We are NOT creating new dialogue examples
- Existing examples (Gemini CLI, Claude Code) are sufficient

**NG-004: README.md Redesign**
- Chapter README already compliant (checked in audit)
- Scope limited to 8 lesson files (01-08)

**NG-005: Validation Framework Changes**
- Constitution v6.0.0 validation checks are fixed
- We implement compliance, not redefine standards

---

## Success Criteria (Acceptance Tests)

### Test 1: Structural Compliance (Automated)

**Given**: All 8 refined lesson files
**When**: Run constitutional validation grep checks
**Then**:
- Check 1 (final section): All return "## Try With AI"
- Check 2 (no forbidden sections): All return zero matches
- Check 3 (no scaffolding): All return zero matches

**Pass Criteria**: 8/8 lessons pass all 3 checks

---

### Test 2: Three Roles Visibility (Manual Review)

**Given**: Lessons 3-8 (Stage 2 content)
**When**: Reviewer reads each lesson looking for Three Roles demonstration
**Then**:
- Reviewer identifies at least 2 examples per lesson where:
  1. AI teaches student (suggests pattern/optimization learner didn't know)
  2. Student teaches AI (corrects context/constraints AI missed)
  3. Co-worker convergence (iterative refinement toward solution)

**Pass Criteria**: Reviewer finds all 3 roles in ≥2 examples per lesson, without roles being explicitly labeled as headers

---

### Test 3: Bash Command Execution (Sandbox)

**Given**: All code blocks extracted from refined lessons
**When**: Execute in clean bash environment (macOS Bash 5.x, Linux Bash 4.0+)
**Then**:
- Commands execute without errors OR
- Errors are pedagogically intentional (troubleshooting lessons)

**Pass Criteria**: 100% of commands either execute successfully or fail intentionally as documented in lesson

---

### Test 4: Content Preservation (Diff Analysis)

**Given**: Pre-refinement and post-refinement lesson files
**When**: Run `diff` or word count comparison
**Then**:
- Core dialogue examples unchanged (except added narration)
- Word count variation <10% per lesson (allows for Three Roles additions)
- No pedagogical content removed

**Pass Criteria**: Diff shows only:
1. Structural moves (content INTO "Try With AI" sections)
2. Whitespace cleanup
3. Three Roles narration additions (≤200 words per lesson)

---

### Test 5: Regression Prevention

**Given**: Refined lessons validated
**When**: Run full constitutional audit (same as pre-refinement audit)
**Then**: Compliance score = 100% (up from 87.5%)

**Pass Criteria**: Zero violations, all checks green

---

## Assumptions

**A-001: Audit Accuracy**
- Audit report (2025-01-17) accurately identifies all violations
- No undiscovered violations exist beyond those documented

**A-002: Constitutional Stability**
- Constitution v6.0.0 remains stable during refinement
- No new principles added mid-refinement

**A-003: A2 Tier Appropriateness**
- Light narration (2-3 sentences per lesson) is appropriate cognitive load for A2 learners
- More explicit Three Roles guidance benefits beginners

**A-004: Existing Example Quality**
- Real AI dialogue examples (Gemini CLI, Claude Code) are authentic and don't require fact-checking
- Examples remain current (bash commands haven't changed)

**A-005: Single-Pass Refinement**
- Violations fixable in single refinement pass
- No iterative debugging needed beyond initial validation

---

## Dependencies

**D-001: Constitutional Validation Tools**
- Grep commands for validation (Constitution:1020-1028)
- Bash shell for command execution testing

**D-002: Existing Lesson Files**
- 8 lesson files (01-08) in current state
- README.md for context (already compliant)

**D-003: Audit Report**
- Constitutional audit completed (2025-01-17)
- Violations documented with line numbers

**D-004: Docusaurus Build Environment**
- For verifying markdown compatibility post-refinement

---

## Risks & Mitigations

### Risk 1: Unintended Content Changes

**Risk**: While fixing structure, accidentally alter pedagogical content
**Probability**: Low
**Impact**: Medium (confuses learners if examples change)
**Mitigation**:
- Use surgical text moves (cut/paste exact content)
- Validate with diff that only structural changes occurred
- Manual review confirms examples unchanged

---

### Risk 2: Three Roles Over-Explanation

**Risk**: Adding narration makes A2 content feel condescending or over-explained
**Probability**: Low
**Impact**: Medium (reduces engagement)
**Mitigation**:
- Limit narration to 1-2 sentences per example
- Use observational tone ("Notice how...") not instructional ("You must understand...")
- User decision: "Clear But Natural" option selected to balance visibility with immersion

---

### Risk 3: Platform-Specific Bash Issues

**Risk**: Commands tested on one platform (macOS) may not work on Linux/Windows
**Probability**: Low (bash commands are standardized)
**Impact**: Low (existing lessons already address platform differences)
**Mitigation**:
- Test in multiple environments (macOS Bash 5.x, Linux Bash 4.0+)
- Preserve existing platform notes in lessons

---

### Risk 4: Grep Validation False Positives

**Risk**: Automated checks miss edge cases or flag acceptable content
**Probability**: Very Low
**Impact**: Low (manual review catches issues)
**Mitigation**:
- Manual review supplements automated checks
- Validation auditor performs final quality gate

---

## Implementation Approach

### Phase 1: Structural Corrections (Priority 1)

**Lesson 2** (`02-safety-first-pattern.md`):
1. Locate final "## Try With AI: Real Examples from Gemini" section (line 413)
2. Move content from lines 420-503 INTO that section as subsections
3. Remove trailing "---" separators after move
4. Add single final "---\n\n" at end

**Lesson 8** (`08-real-project-troubleshooting.md`):
1. Locate final "## Try With AI: Orchestration at Scale (Advanced)" section (line 564)
2. Move closing paragraphs (lines 603-605) INTO that section before final "---"
3. Remove redundant "---" separators
4. Normalize whitespace to single blank line after final "---"

**Lessons 3-7**:
1. Verify final section is "## Try With AI: [title]"
2. Remove trailing newlines beyond `---\n\n` pattern
3. Ensure exactly one blank line before and after final "---"

---

### Phase 2: Three Roles Clarity Enhancement (Priority 2)

**For each Stage 2 lesson (Lessons 3-8)**:

1. **Identify existing dialogue examples** showing AI collaboration
2. **Add light narration** (1-2 sentences) showing roles:
   - **AI as Teacher**: "Notice how AI suggested [optimization/pattern] you hadn't considered"
   - **AI as Student**: "Here, you refined AI's output by teaching it [context/constraint]"
   - **Co-Worker**: "Through [N] iterations, you and AI converged on [solution neither had initially]"
3. **Placement**:
   - Inside existing "Try With AI" sections as inline observations, OR
   - After dialogue examples as brief analysis paragraphs
4. **Frequency**: 2-3 markers per lesson (not every example)

**Example Addition** (Lesson 3):
```markdown
### Practice: Navigation with Your AI

**Prompt:** "Show me how you navigate through folders..."

[Existing dialogue example]

**What to Compare:**

| Navigation Step | You Do This | Your AI Does This |
|---|---|---|
| Navigate down | `cd Documents` | `cd book-source` |

**Observation**: Notice how you and your AI use identical commands (`cd`, `..`) but in different locations. This is co-learning—you understand AI's navigation because you've practiced the same pattern. When AI suggests a path, you can verify it makes sense.
```

---

### Phase 3: Validation

1. **Automated checks**: Run grep validation (SC-001)
2. **Manual review**: Verify Three Roles visible but natural
3. **Bash execution**: Test all code blocks in sandbox
4. **Diff analysis**: Confirm content preservation
5. **Full audit**: Re-run constitutional compliance audit

---

## Timeline Estimate

**Total**: 2-3 days (surgical refinement, not full rewrite)

- **Phase 0 (Research/Audit)**: ✅ Complete (5 hours)
- **Phase 1 (Specification)**: ✅ In Progress (3 hours)
- **Phase 2 (Planning)**: 2 hours (lesson-by-lesson correction strategy)
- **Phase 3 (Tasks)**: 1 hour (violation checklist)
- **Phase 4 (Implementation)**: 8-12 hours (8 lessons × 1-1.5 hours each)
- **Phase 5 (Validation)**: 4 hours (automated + manual review + sandbox)
- **Phase 6 (Finalization)**: 2 hours (meta-learning capture, git workflow)

**Total Estimated Hours**: 25-29 hours (spread across 2-3 days)

---

## Definition of Done

- [ ] All 8 lessons pass automated constitutional validation (SC-001)
- [ ] Three Roles visible in 2+ examples per Stage 2 lesson (SC-002)
- [ ] Content preservation verified via diff analysis (SC-003)
- [ ] All bash commands execute correctly or fail pedagogically (SC-004)
- [ ] Post-refinement audit shows 100% compliance (SC-005)
- [ ] Technical reviewer validates pedagogical soundness
- [ ] Validation auditor confirms constitutional compliance
- [ ] Sandbox testing completes without unexpected errors
- [ ] Git commit created with refined lessons
- [ ] PHR (Prompt History Record) created capturing refinement reasoning

---

## Intelligence Object (Complete Context)

```json
{
  "meta": {
    "orchestrator_version": "2.0",
    "reasoning_mode": "constitutional_derivation",
    "task_type": "educational_content_refinement",
    "refinement_scope": "surgical_compliance_corrections",
    "user_decisions": {
      "scope": "Constitutional Compliance First (Option A)",
      "three_roles": "Clear But Natural (Option B)",
      "preservation": "Preserve Structure, Fix Content (Option A)"
    }
  },

  "audit_findings": {
    "date": "2025-01-17",
    "compliance_score": "87.5%",
    "critical_violations": 2,
    "minor_violations": 5,
    "strengths": [
      "No forbidden closure sections",
      "No internal scaffolding exposed",
      "Three Roles present through narrative",
      "Safety naturally integrated",
      "Dialogue-first pattern working"
    ],
    "violations_by_lesson": {
      "lesson_1": "fully_compliant",
      "lesson_2": "critical_structural_violation",
      "lesson_3": "minor_whitespace_issue",
      "lesson_4": "minor_whitespace_issue",
      "lesson_5": "minor_whitespace_issue",
      "lesson_6": "minor_whitespace_issue",
      "lesson_7": "minor_whitespace_issue",
      "lesson_8": "critical_structural_violation"
    }
  },

  "constitutional_frameworks": {
    "teaching_framework": {
      "stages_needed": [1, 2],
      "stage_1_lessons": ["01", "02"],
      "stage_2_lessons": ["03", "04", "05", "06", "07", "08"],
      "three_roles_requirement": "Clear But Natural narration in Stage 2 lessons"
    },
    "cognitive_load": {
      "tier": "A2",
      "limit": 7,
      "current_compliance": true
    },
    "minimal_content": {
      "single_closure": "Try With AI",
      "violations": ["Lesson 2", "Lesson 8"],
      "fix_approach": "Move content INTO final sections"
    }
  },

  "refinement_strategy": {
    "priority_1": "Structural corrections (Lessons 2, 8)",
    "priority_2": "Three Roles clarity (Lessons 3-8)",
    "priority_3": "Whitespace normalization (Lessons 3-7)",
    "content_preservation": "Core examples unchanged, only add light narration"
  }
}
```

---

## Next Steps

1. **Specification Review & Approval** — Validate refinement scope with user
2. **Planning Phase** (`/sp.plan`) — Lesson-by-lesson correction checklist
3. **Tasks Phase** (`/sp.tasks`) — Violation-specific fix tasks
4. **Implementation** (`content-implementer`) — Execute corrections with intelligence context
5. **Validation** (`validation-auditor` + `factual-verifier`) — Constitutional + bash accuracy checks
6. **Finalization** — Git commit + PHR capture

---

**This specification activates reasoning mode by providing complete constitutional context, audit findings, and surgical refinement strategy. Content-implementer will receive full intelligence object ensuring corrections align with Constitution v6.0.0 while preserving Chapter 7's pedagogical strengths.**
