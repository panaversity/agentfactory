# /sp.constitution-sync: Intelligent Constitution Propagation

**Version**: 1.0.0
**Created**: 2025-01-10
**Philosophy**: AI-centric per-lesson intelligence (scripts provide data, AI decides intervention)

---

## Purpose

Detect constitution changes and apply the **minimum necessary intervention** per lesson to bring existing chapters into compliance with updated/new constitutional rules.

**Key Innovation**: No mode flags—AI intelligently decides per-lesson what intervention is needed (surgical edit, enhanced regeneration, full regeneration, or skip).

---

## Syntax

```bash
/sp.constitution-sync [chapter-number]
```

**Examples**:
```bash
/sp.constitution-sync 14
/sp.constitution-sync 12
```

---

## Distinction: Error Analysis vs Constitution-Sync

| Aspect | `/sp.error-analysis` | `/sp.constitution-sync` |
|--------|---------------------|------------------------|
| **Purpose** | **Diagnose** workflow issues | **Fix** constitutional violations |
| **Trigger** | After workflow execution (reactive) | After constitution change (proactive) |
| **Input** | Executed artifacts (traces) | Constitution delta + existing chapters |
| **Output** | Report + recommendations | Updated chapters |
| **Action** | Read-only analysis | Write operations (edits/regen) |
| **Timing** | Post-mortem | Maintenance |
| **Analogy** | Medical diagnosis | Treatment application |

**Relationship**: Error analysis can debug constitution-sync itself (if sync has issues, run error analysis on sync execution to diagnose).

---

## The Intelligence: Per-Lesson Decision Tree

```
For each lesson in chapter:

┌─ Does spec/plan need changes?
│  ├─ YES → Update spec/plan first
│  │        Then reassess lessons
│  └─ NO → Continue to lesson analysis
│
├─ Analyze lesson compliance
│  ├─ Read lesson content
│  ├─ Check against ALL constitution rules
│  └─ Classify violations
│
└─ Decision Matrix:

    IF lesson is 90%+ compliant (minor formatting issues):
       → SURGICAL EDIT (str_replace)
       Examples: Add missing 💬 prompt, remove post-section
       Time: ~3-5 minutes per lesson

    ELSE IF lesson is 60-89% compliant (structural issues):
       → ENHANCED REGENERATION (preserve good parts)
       - Extract: Well-written explanations, quality examples
       - Regenerate: With constitution rules + extracted quality
       - Result: New lesson that preserves best content
       Time: ~10-15 minutes per lesson

    ELSE IF lesson is <60% compliant (fundamental problems):
       → FULL REGENERATION
       - Use spec/plan as source of truth
       - Generate fresh with current constitution
       Time: ~15-20 minutes per lesson

    ELSE (100% compliant):
       → NO CHANGE
       Time: ~1 minute (validation only)
```

---

## Execution Flow: 5-Phase Intelligent Sync

### PHASE 0: Constitution Delta Analysis (Context Discovery)

**Your Role**: AI as **Student** — Learn what changed in constitution before evaluating content.

#### Step 0.1: Read Current Constitution

```bash
Read .specify/memory/constitution.md
```

**Extract**:
- Version number (from header)
- Sync Impact Report (from HTML comment at top)
- All 18 core principles (Principle 1-18)
- Core philosophy (8 tenets including Co-Learning, Evals-First, Spec-Driven, etc.)
- Graduated teaching pattern (Principle 13)
- Three-Role AI Partnership (Principle 18)
- Lesson closure policy (from output styles)
- CoLearning elements requirements (from output styles)

#### Step 0.2: Categorize Constitutional Rules

**Organize rules by impact type** (for prioritization):

**High-Impact Rules** (affects all lessons):
- CoLearning elements throughout (💬🎓🚀✨)
- Three-Role AI Partnership (AI as Teacher/Student/Co-Worker)
- Lesson closure pattern ("Try With AI" only, no post-sections)
- Pedagogical ordering (no forward references)

**Medium-Impact Rules** (affects some lessons):
- Graduated Teaching Pattern (Book → AI Companion → AI Orchestration)
- Cognitive load limits (A1: max 5 concepts, A2: max 7, B1: max 10)
- Tone and style (conversational, exploration-focused)
- Specification-first pedagogy (show spec → AI prompt → code → validation)

**Low-Impact Rules** (rarely violated if lesson exists):
- Python 3.13+ standards
- Type hints mandatory
- Test-first mindset
- Reading level (Grade 7 baseline)

#### Step 0.3: Constitution Delta Report

**Output**:

```markdown
## Constitution Delta Analysis

**Constitution Version**: 3.1.2

**What Changed** (from Sync Impact Report):
- [List changes from HTML comment at top of constitution]

**Impacted Rules for This Chapter**:
- **High Impact**: [List rules that affect all lessons]
- **Medium Impact**: [List rules that affect some lessons]
- **Low Impact**: [List rules rarely violated]

**Expected Violations** (hypothesis):
- [Based on change history, predict likely violations]
```

---

### PHASE 1: Spec/Plan Compliance Check (Foundation Validation)

**Your Role**: AI as **Co-Worker** — Ensure foundational artifacts are compliant before lesson assessment.

#### Step 1.1: Locate Artifacts

**Run helper script**:

```bash
.specify/scripts/bash/constitution-sync/artifact-locator.sh [chapter-number]
```

**Expected output (JSON)**:
```json
{
  "chapter": 14,
  "artifacts": {
    "spec": "specs/part-4-chapter-14/spec.md",
    "plan": "specs/part-4-chapter-14/plan.md",
    "lessons": [
      "book-source/docs/04-Part-4/14-data-types/01-intro.md",
      "book-source/docs/04-Part-4/14-data-types/02-numeric.md"
    ]
  },
  "status": "complete"
}
```

#### Step 1.2: Check Spec Compliance

**Read spec.md**:
- Learning objectives align with constitution?
- Prerequisites explicit?
- Success criteria measurable?
- Complexity tier appropriate for part?
- Evals defined with business-goal connection?

**Decision**:
- ✅ **COMPLIANT**: Proceed to plan check
- ⚠️ **NEEDS UPDATE**: Note required changes (will prompt user)

#### Step 1.3: Check Plan Compliance

**Read plan.md**:
- Lesson breakdown matches spec?
- Skills proficiency metadata present (CEFR levels)?
- CoLearning elements referenced?
- Graduated Teaching Pattern followed?
- Cognitive load validated per lesson?

**Decision**:
- ✅ **COMPLIANT**: Proceed to per-lesson analysis
- ⚠️ **NEEDS UPDATE**: Note required changes (will prompt user)

#### Step 1.4: Apply Spec/Plan Fixes (If Needed)

**If spec or plan needs updates**:

```markdown
**Spec/Plan Updates Required**:

Spec.md:
  - [List specific issues and fixes needed]

Plan.md:
  - [List specific issues and fixes needed]

**Recommendation**: Update spec/plan first, then re-run /sp.constitution-sync
```

**Ask user**: "Should I update spec/plan now, or would you like to review and update manually?"

**If user approves**: Apply fixes, document changes, proceed to Phase 2.

---

### PHASE 2: Per-Lesson Intelligence (The Core)

**Your Role**: AI as **Teacher** — Diagnose compliance, decide intervention, explain reasoning.

**For EACH lesson** in chapter, run this analysis:

---

#### Step 2.1: Run Quantitative Scripts (Data Gathering)

**Script 1: Compliance Metrics**

```bash
.specify/scripts/bash/constitution-sync/compliance-metrics.sh [chapter-number] [lesson-file]
```

**Expected output (JSON)**:
```json
{
  "lesson": "01-intro.md",
  "colearning_elements": {
    "💬_prompts": 0,
    "🎓_commentaries": 0,
    "🚀_challenges": 0,
    "✨_tips": 0
  },
  "lesson_closure": {
    "try_with_ai_exists": true,
    "post_sections": ["## What's Next"]
  },
  "pedagogical_ordering": {
    "forward_references_found": 2,
    "flagged_lines": [234, 456]
  },
  "tone_analysis": {
    "conversational_score": 8,
    "exploration_language": "present",
    "ai_partnership": "mentioned"
  },
  "code_quality": {
    "python_version": "3.13+",
    "type_hints": "present",
    "security": "no_issues"
  }
}
```

**Script 2: Forward Reference Detection**

```bash
.specify/scripts/bash/constitution-sync/detect-forward-references.sh [chapter-number]
```

**Expected output (JSON)**:
```json
{
  "lesson": "01-intro.md",
  "violations_detected": [
    {
      "line": 234,
      "code_snippet": "name.upper()",
      "issue": "String method not introduced",
      "severity_flag": "CRITICAL",
      "context": "name = 'alice' print(name.upper())"
    }
  ]
}
```

---

#### Step 2.2: AI Intelligence Layer (Contextual Judgment)

**Now YOU (AI agent) read the lesson file and apply intelligence**:

**Read Lesson Content**:
```bash
Read [lesson-file-path]
```

**Analyze Against Constitution**:

**Rule-by-Rule Check**:

1. **CoLearning Elements** (High Impact):
   - Are 💬 AI Colearning Prompts present? (Expected: 1-4 per lesson)
   - Are 🎓 Instructor Commentaries present? (Expected: 2-4 per lesson)
   - Are 🚀 CoLearning Challenges present? (Expected: 1-4 per lesson)
   - Are ✨ Teaching Tips present? (Expected: 1-3 per lesson)
   - **Quality assessment**: Read actual prompts—are they exploration-focused or just "Ask AI to write X"?

2. **Lesson Closure Pattern** (High Impact):
   - Does lesson end with "Try With AI" section?
   - Are there ANY sections after "Try With AI"? (❌ VIOLATION if yes)
   - Sections to flag: "Key Takeaways", "What's Next", "Summary", "Recap", "Completion Checklist"

3. **Pedagogical Ordering** (High Impact):
   - Script flagged forward references—are they ACTUAL violations?
   - Read context around flagged lines (±5 lines)
   - **Judge severity**:
     - ❌ **CRITICAL**: Concept not introduced at all (beginner can't understand)
     - ⚠️ **MAJOR**: Concept mentioned but not explained (confusing)
     - ✅ **ACCEPTABLE**: Concept introduced inline before use

4. **Three-Role AI Partnership** (High Impact):
   - AI as Teacher: Does AI suggest patterns student doesn't know?
   - AI as Student: Does AI adapt to student feedback?
   - AI as Co-Worker: Is language collaborative vs. command-driven?

5. **Graduated Teaching Pattern** (Medium Impact):
   - Tier 1 (Book teaches): Foundational concepts explained directly?
   - Tier 2 (AI handles): Complex syntax delegated to AI?
   - Tier 3 (AI orchestrates): Scaling operations automated?

6. **Cognitive Load** (Medium Impact):
   - Count new concepts in lesson
   - A1: max 5, A2: max 7, B1: max 10
   - Is limit respected?

7. **Tone & Style** (Medium Impact):
   - Conversational vs. documentation style?
   - Exploration language ("Let's explore", "What happens if...") vs. prescriptive?
   - AI partnership emphasized vs. tool-driven?

8. **Specification-First Pedagogy** (Medium Impact):
   - Code examples show: spec → AI prompt → code → validation?
   - Or just: "Here's code, copy it"?

---

#### Step 2.3: Compliance Scoring (Quantitative)

**Calculate compliance percentage**:

```python
# Scoring weights (constitution-aligned priorities)
WEIGHTS = {
    "colearning_elements": 25,  # High impact
    "lesson_closure": 20,       # High impact
    "pedagogical_ordering": 20, # High impact
    "three_role_partnership": 15, # High impact
    "graduated_teaching": 10,   # Medium impact
    "tone_style": 5,            # Medium impact
    "code_quality": 5,          # Low impact (rarely violated)
}

# Example scoring
score = (
    (colearning_score / 4) * 25 +  # 4 element types
    (closure_score / 1) * 20 +     # Binary: compliant or not
    (ordering_score / 1) * 20 +    # No critical violations?
    (partnership_score / 3) * 15 + # 3 roles present?
    (graduated_score / 3) * 10 +   # 3 tiers present?
    (tone_score / 10) * 5 +        # Subjective 0-10 scale
    (code_score / 1) * 5           # Python 3.13+, type hints
)

# Final compliance: 0-100%
```

**Example Output**:

```markdown
### Lesson 1: 01-variables-and-type-hints.md

**Compliance Score: 72% (Structural issues, good content)**

**Breakdown**:
- CoLearning Elements: 0% (0/4 types present) ❌
- Lesson Closure: 100% ("Try With AI" present, no post-sections) ✅
- Pedagogical Ordering: 100% (no forward references) ✅
- Three-Role Partnership: 40% (AI mentioned but not embedded) ⚠️
- Graduated Teaching: 70% (Tiers 1-2 present, light on Tier 2) ⚠️
- Tone & Style: 90% (conversational, exploration-focused) ✅
- Code Quality: 100% (Python 3.13+, type hints) ✅

**Critical Issues**: Missing all CoLearning elements
**Major Issues**: Three-Role Partnership not demonstrated
**Minor Issues**: Tier 2 (AI Companion) underutilized

**Content Quality**: ✅ Excellent (explanations clear, examples solid, structure logical)
```

---

#### Step 2.4: Decision Matrix (Intelligent Intervention)

**Based on compliance score + content quality + violation severity**:

```
IF compliance ≥ 90% AND no critical issues:
   → SURGICAL EDIT
   Rationale: Minor fixes, preserve quality
   Time: 3-5 minutes

ELSE IF 60% ≤ compliance < 90% AND content quality good:
   → ENHANCED REGENERATION
   Rationale: Structural issues + quality content worth preserving
   Time: 10-15 minutes

ELSE IF compliance < 60% OR critical pedagogical violations:
   → FULL REGENERATION
   Rationale: Fundamental issues require rewrite
   Time: 15-20 minutes

ELSE IF compliance = 100%:
   → NO CHANGE
   Rationale: Already compliant
   Time: 1 minute (validation only)
```

**CRITICAL JUDGMENT CALLS** (AI intelligence required):

**When to choose SURGICAL EDIT**:
- ✅ Content quality is excellent
- ✅ Violations are **structural** (missing elements, post-sections)
- ✅ Can insert/remove without disrupting flow
- ✅ No pedagogical ordering violations
- ❌ NOT if: Tone issues pervasive, forward references, poor quality

**When to choose ENHANCED REGENERATION**:
- ✅ Content quality is good (worth preserving)
- ✅ Violations are **mixed** (structural + some content issues)
- ✅ Core examples and explanations are solid
- ⚠️ BUT: Some sections need rewriting (tone, ordering, partnerships)
- ❌ NOT if: Fundamental conceptual issues, complete misalignment

**When to choose FULL REGENERATION**:
- ✅ Compliance < 60%
- ✅ Critical pedagogical violations (forward refs block learning)
- ✅ Tone issues throughout (documentation style, not conversational)
- ✅ Learning objectives misinterpreted
- ❌ ALWAYS if: Spec/plan changed significantly (lesson outdated)

**When to choose NO CHANGE**:
- ✅ Compliance = 100%
- ✅ All constitutional rules met
- ✅ Content quality excellent
- ⏱️ Validation only (re-run technical-reviewer to confirm)

---

#### Step 2.5: Execute Intervention (Per-Lesson Action)

---

##### SURGICAL EDIT Execution

**Strategy**:
1. Identify natural insertion points (after concepts introduced)
2. Generate contextual CoLearning elements
3. Insert via Edit tool (preserve surrounding content)
4. Remove post-sections (if any)
5. Validate: Run technical-reviewer

**Example Process**:

```markdown
**Lesson 1: Surgical Edit Decision**

**Insertions Needed**: 12 CoLearning elements
**Deletions Needed**: 1 post-section ("What's Next")

**Execution**:

Insert 1: After line 89 (concept: variables introduced)
  ├─ Element: 💬 AI Colearning Prompt
  ├─ Context: Just explained what variables are
  └─ Generated:
      #### 💬 AI Colearning Prompt
      > "Ask your AI: Why does Python let me reassign variables to
      > different types (x = 5, then x = 'hello')? What are the
      > tradeoffs of this flexibility?"

Insert 2: After line 156 (concept: type hints introduced)
  ├─ Element: 🎓 Instructor Commentary
  ├─ Context: Type hints just explained
  └─ Generated:
      #### 🎓 Instructor Commentary
      > In AI-native development, type hints aren't about making
      > Python "strongly typed"—they're about communicating intent
      > to your AI pair-programmer. When you write `name: str`,
      > you're saying "I intend this to be a string," and your AI
      > understands that intent. Syntax is cheap; clarity is gold.

[... 10 more insertions ...]

Delete 1: Lines 678-695 ("What's Next" section after "Try With AI")
  ├─ Violation: Post-section after lesson closure
  └─ Action: Remove entire section

Applying edits via Edit tool...
✅ 12 insertions complete
✅ 1 deletion complete

Validation: Running technical-reviewer...
✅ PASS (CoLearning elements: 4/4 types present, closure compliant)
```

**Use the Edit tool for each insertion/deletion**:

```
For each insertion:
  old_string: [exact text from line N]
  new_string: [exact text from line N] + [generated CoLearning element]

For each deletion:
  old_string: [entire post-section text]
  new_string: [empty string]
```

---

##### ENHANCED REGENERATION Execution

**Strategy**:
1. Extract quality content (examples, explanations)
2. Identify sections needing rewrite (violations)
3. Regenerate with constitution + preserved quality
4. Best of both worlds

**Example Process**:

```markdown
**Lesson 2: Enhanced Regeneration Decision**

**Compliance**: 65% (structural + pedagogical issues)
**Content Quality**: Good (examples solid, some tone issues)

**Extraction Phase**:

Extracting quality content...
  ✅ Example 1 (integer operations): Excellent, preserve
  ✅ Example 3 (float precision): Excellent, preserve
  ✅ Explanation of numeric types: Good tone, preserve
  ⚠️ Section on "Type Conversions": Has pedagogical violations, regenerate
  ⚠️ Paragraphs 5-8: Documentation tone, regenerate

**Regeneration Phase**:

Generating enhanced lesson...
  ├─ Using preserved examples (3 examples)
  ├─ Using preserved explanations (2 sections)
  ├─ Regenerating problematic sections (with constitution)
  ├─ Adding missing CoLearning elements (throughout)
  └─ Fixing pedagogical ordering (no forward references)

**Strategy for regeneration**:
- Read spec.md learning objectives (source of truth)
- Read plan.md lesson structure (intended flow)
- Use lesson-writer skill with constitution constraints
- Embed preserved examples at appropriate points
- Add CoLearning elements throughout

Generated: 02-integers-and-floats.md (enhanced version)

**Diff Summary**:
  Preserved: 60% of original content (quality maintained)
  Regenerated: 40% (compliance fixed)
  Added: 8 CoLearning elements
  Fixed: 2 pedagogical ordering violations
  Fixed: 4 tone/style issues

Validation: Running technical-reviewer...
✅ PASS (All rules compliant, quality preserved)
```

**Use lesson-writer skill invocation**:

```
Invoke lesson-writer with:
- Spec: specs/part-4-chapter-14/spec.md
- Plan: specs/part-4-chapter-14/plan.md (Lesson 2 objectives)
- Preserved Content: [Extracted examples and explanations]
- Constitution Constraints: [List violated rules to enforce]
- Output: book-source/docs/04-Part-4/14-data-types/02-numeric.md
```

---

##### FULL REGENERATION Execution

**Strategy**:
1. Use spec/plan as authoritative source
2. Generate fresh lesson with constitution
3. No preservation (fundamental issues)

**Example Process**:

```markdown
**Lesson 4: Full Regeneration Decision**

**Compliance**: 35% (fundamental issues)
**Critical Violations**:
- Pedagogical ordering (uses iteration before loops taught)
- Tone (documentation style throughout)
- Concept misinterpretation (teaches usage, should teach awareness)

**Regeneration Phase**:

Reading learning objectives from spec.md...
  → "Understand that Python has collection types (list, dict, tuple, set)"
  → "Recognize collection syntax in code"
  → "Know when to ask AI about collections vs use primitive types"

Reading plan.md lesson structure...
  → Collections awareness ONLY (no deep dive)
  → Tier 1: What they are
  → Tier 2: Basic syntax recognition
  → Tier 3: NOT covered (defer to Ch 18-19)

Generating new lesson from scratch...
  ├─ Constitution-compliant structure
  ├─ CoLearning elements throughout
  ├─ NO iteration (just show static examples)
  ├─ NO methods (just show syntax)
  ├─ Conversational tone
  └─ AI partnership emphasized

**Strategy for regeneration**:
- Read spec.md + plan.md (source of truth)
- Use lesson-writer skill with strict constraints
- Enforce: NO forward references
- Enforce: Collections AWARENESS (not deep dive)
- Add: CoLearning elements throughout
- Tone: Conversational, exploration-focused

Generated: 04-collections-awareness.md (new version)

**Key Changes**:
  - Removed: 15+ code examples with iteration/methods
  - Added: 6 simple examples (static collections)
  - Added: 12 CoLearning elements
  - Fixed: All pedagogical ordering violations
  - Restructured: From reference to learning narrative

Validation: Running technical-reviewer...
✅ PASS (Fundamental issues resolved)
```

**Use lesson-writer skill invocation**:

```
Invoke lesson-writer with:
- Spec: specs/part-4-chapter-14/spec.md
- Plan: specs/part-4-chapter-14/plan.md (Lesson 4 objectives)
- Constitution Constraints: [All relevant rules, strict enforcement]
- Output: book-source/docs/04-Part-4/14-data-types/04-collections-awareness.md
```

---

##### NO CHANGE Execution

**Strategy**: Validation only

```markdown
**Lesson 3: No Change Decision**

**Compliance**: 100%
**Content Quality**: Excellent

**Validation**:

Running technical-reviewer...
✅ PASS (All constitutional rules met)

**Action**: None (lesson already compliant)
**Time**: 1 minute
```

---

### PHASE 3: Chapter-Level Validation (Cross-Lesson Consistency)

**Your Role**: AI as **Co-Worker** — Ensure chapter-level coherence after per-lesson changes.

#### Step 3.1: Spec/Plan Consistency

**Check**:
- All spec learning objectives covered in lessons?
- All plan concepts present in lessons?
- CEFR progression maintained (A1 → A2 → B1)?

**If issues found**: Flag for user review (may need plan update).

#### Step 3.2: Cross-Lesson Consistency

**Check**:
- No forward references across lessons (Lesson N only uses concepts from 1 to N)?
- Prerequisite chain intact (L1 → L2 → L3 → L4 → L5)?
- Terminology consistent across lessons?
- CoLearning elements balanced across lessons?

**If issues found**: Identify problematic lessons, may need additional surgical edits.

#### Step 3.3: Constitution Compliance (Chapter-Wide)

**Check**:
- All 18 constitutional principles verified at chapter level
- All lessons have CoLearning elements (4 types)?
- All lessons follow lesson closure pattern?
- All lessons demonstrate Three-Role Partnership?

**Output**:

```markdown
## Chapter-Level Validation

**Spec/Plan Consistency**: ✅ PASS
  ✓ All spec objectives covered in lessons
  ✓ All plan concepts present in lessons
  ✓ CEFR progression maintained

**Cross-Lesson Consistency**: ✅ PASS
  ✓ No forward references across lessons
  ✓ Prerequisite chain intact (L1→L2→L3→L4→L5)
  ✓ Terminology consistent

**Constitution Compliance**: ✅ PASS
  ✓ All 18 principles verified
  ✓ CoLearning elements present in all lessons
  ✓ Lesson closure pattern: 5/5 compliant
  ✓ Pedagogical ordering: 5/5 compliant

**Final Verdict**: ✅ PASS (Chapter ready for publication)
```

---

### PHASE 4: Summary Report (Narrative Co-Learning)

**Your Role**: AI as **Teacher** — Explain what happened, why decisions were made, what user should know.

#### Report Template

Use the standardized template at `.specify/templates/constitution-sync-report-template.md` to structure your output.

**Template Structure**:
- **Frontmatter**: Metadata (ID, chapter, constitution version, status)
- **Executive Summary**: Total time, lessons processed, summary
- **Constitution Changes Applied**: What changed, impact on chapter
- **Phase 0-3 Details**: Delta analysis, spec/plan checks, per-lesson intelligence, chapter validation
- **Why This Was Optimal**: Comparison to all-surgical and all-regenerate approaches
- **Recommendations**: Next steps, decision points
- **Appendix**: Detailed metrics, time breakdown, intervention distribution

#### Report Location

Save completed reports to:
```
history/constitution-sync/YYYY-MM-DD-chapter-N-sync-vX.X.X.md
```

**Naming examples**:
- `history/constitution-sync/2025-11-10-chapter-14-sync-v3.1.2.md`
- `history/constitution-sync/2025-11-10-chapter-12-sync-v3.1.2.md`

#### Report Structure Example

```markdown
# ✅ CONSTITUTION SYNC COMPLETE: Chapter [N]

## Executive Summary

**Total Time**: [X] minutes
**Approach**: Intelligent hybrid (per-lesson decisions)
**Constitution Version**: [version]

## Constitution Changes Applied

**What Changed**:
- [List changes from delta analysis]

**Impact on This Chapter**:
- [Explain which rules affected this chapter and why]

## Per-Lesson Breakdown

| Lesson | Compliance | Decision | Time | Changes |
|--------|-----------|----------|------|---------|
| Lesson 1 | 72% | Surgical Edit | 8 min | 15 insertions |
| Lesson 2 | 65% | Enhanced Regen | 12 min | 40% regenerated, 60% preserved |
| Lesson 3 | 100% | No Change | 1 min | Validation only |
| Lesson 4 | 35% | Full Regen | 18 min | Complete rewrite |
| Lesson 5 | 92% | Surgical Edit | 2 min | 1 deletion (18 lines) |

**Total**: 5 lessons, 46 minutes

## Why This Was Optimal

**vs All Surgical Edit**:
- Would have missed Lesson 4 fundamental issues
- Quality: Lower (forced surgery on deep problems)
- Time: ~40 min (saved 6 min) BUT lower quality

**vs All Regenerate**:
- Would have regenerated excellent Lessons 1, 3, 5
- Quality: Risk losing existing excellence
- Time: ~2-3 hours (saved 1.5-2.5 hours!)

**Intelligent Hybrid**:
- Each lesson got exactly what it needed
- Quality: Maximized (preserve good, fix bad)
- Time: 46 minutes (optimal)

## Constitution Compliance

✅ All 18 principles verified compliant
✅ CoLearning elements (100% coverage)
✅ Lesson closure pattern (100% compliant)
✅ Pedagogical ordering (no forward references)
✅ Three-Role Partnership (demonstrated throughout)

## Recommendation

Chapter [N] is now constitution-compliant and ready for publication.

**Next Steps**:
1. Review changes (git diff to see what was modified)
2. Test Docusaurus build (ensure no broken links)
3. Commit changes:
   ```bash
   git add .
   git commit -m "Constitution sync: Chapter [N] (intelligent hybrid)"
   ```
4. Process next chapter:
   ```bash
   /sp.constitution-sync [N+1]
   ```

## Decision Points

**Would you like to**:
1. Review changes before committing?
2. Process next chapter immediately?
3. Batch process multiple chapters?
4. Generate detailed diff report?

I'm here to execute your decision.
```

---

## Helper Scripts (Quantitative Support)

The command uses these helper scripts (AI runs them, interprets outputs):

### 1. `artifact-locator.sh`

**Purpose**: Find all artifacts for a chapter

**Usage**:
```bash
.specify/scripts/bash/constitution-sync/artifact-locator.sh [chapter-number]
```

**Output** (JSON):
```json
{
  "chapter": 14,
  "artifacts": {
    "spec": "specs/part-4-chapter-14/spec.md",
    "plan": "specs/part-4-chapter-14/plan.md",
    "lessons": [...]
  },
  "status": "complete"
}
```

---

### 2. `compliance-metrics.sh`

**Purpose**: Quantitative metrics per lesson

**Usage**:
```bash
.specify/scripts/bash/constitution-sync/compliance-metrics.sh [chapter-number] [lesson-file]
```

**Output** (JSON):
```json
{
  "lesson": "01-intro.md",
  "colearning_elements": {
    "💬_prompts": 0,
    "🎓_commentaries": 0,
    "🚀_challenges": 0,
    "✨_tips": 0
  },
  "lesson_closure": {
    "try_with_ai_exists": true,
    "post_sections": []
  },
  "pedagogical_ordering": {
    "forward_references_found": 0,
    "flagged_lines": []
  },
  "tone_analysis": {
    "conversational_score": 8,
    "exploration_language": "present"
  }
}
```

---

### 3. `detect-forward-references.sh`

**Purpose**: Flag potential pedagogical ordering violations

**Usage**:
```bash
.specify/scripts/bash/constitution-sync/detect-forward-references.sh [chapter-number]
```

**Output** (JSON):
```json
{
  "lesson": "01-intro.md",
  "violations_detected": [
    {
      "line": 234,
      "code_snippet": "name.upper()",
      "issue": "String method not introduced",
      "severity_flag": "CRITICAL",
      "context": "name = 'alice' print(name.upper())"
    }
  ]
}
```

---

## AI Intelligence at Core

**Philosophy**: Scripts provide **data points** (counts, patterns, flags). AI provides **intelligence** (context, severity, recommendations).

**Example**:

**Script says**: "Found `.upper()` at line 234"

**AI reads context**:
```python
# Line 230-240
name = "alice"
print(name.upper())  # No introduction of methods
```

**AI judges**: "CRITICAL - beginner doesn't know what methods are"

**vs.**

**Script says**: "Found `.upper()` at line 234"

**AI reads context**:
```markdown
### String Methods (New Concept)

Python strings have built-in methods (actions they can do).
One useful method is `.upper()` which converts to uppercase:

\`\`\`python
name = "alice"
print(name.upper())  # Prints: ALICE
\`\`\`
```

**AI judges**: "ACCEPTABLE - concept introduced inline before use"

**This is why AI intelligence is at core**—only AI can read context and make pedagogical judgments.

---

## Performance Comparison

| Scenario | All Surgery | All Regen | Intelligent Hybrid |
|----------|------------|-----------|-------------------|
| **Time** | 40 min | 2-3 hours | **46 min** |
| **Quality** | ⚠️ Misses deep issues | ⚠️ Loses existing quality | ✅ **Optimal** |
| **Risk** | ⚠️ Some lessons still broken | ⚠️ May introduce new issues | ✅ **Minimal** |
| **Efficiency** | ⚠️ Surgical on unfixable | ⚠️ Regen on excellent | ✅ **Right tool per lesson** |

**Why Intelligent Hybrid Wins**:
- Surgical edit on minor issues (fast, preserve quality)
- Enhanced regen on mixed issues (best of both worlds)
- Full regen on fundamental issues (deep fix when needed)
- Skip on compliant lessons (no unnecessary work)

---

## Success Metrics

**Quality Gates**:
- ✅ All 18 constitutional principles verified compliant
- ✅ CoLearning elements present in all lessons (4 types)
- ✅ Lesson closure pattern correct (no post-sections)
- ✅ Pedagogical ordering maintained (no forward references)
- ✅ Three-Role Partnership demonstrated

**Efficiency Metrics**:
- ✅ Per-lesson time: 1 min (skip) to 20 min (full regen)
- ✅ Chapter time: 30-60 min average (vs. 2-3 hours all-regen)
- ✅ Quality preserved where possible (vs. blind regeneration)

**Output Quality**:
- ✅ Intelligent decisions (context-aware, not mechanical)
- ✅ Narrative report (explains reasoning, not just actions)
- ✅ Strategic partnership (decision points, not dictation)

---

## Maintenance & Evolution

### Adding New Constitutional Rules

When constitution is updated with new rules:

1. **Update command file**: Add new rule to Phase 0 (delta analysis) and Phase 2 (per-lesson checks)
2. **Update helper scripts**: If quantitative detection possible (e.g., count new element type)
3. **Test on existing chapter**: Verify new rule detected and fixed correctly
4. **Document change**: Update SYSTEM-OVERVIEW.md with new rule handling

### Updating Detection Heuristics

When false positives/negatives occur:

1. **Observe patterns**: Which violations are missed or incorrectly flagged?
2. **Refine scripts**: Update detection patterns in helper scripts
3. **Update AI judgment logic**: Improve context interpretation in command file
4. **Test on problematic chapters**: Verify improvements

---

## Credits & Philosophy

**Design Principles**:
1. **AI intelligence at core** (scripts support, don't decide)
2. **Per-lesson intelligence** (right intervention per lesson)
3. **Content quality preservation** (when possible)
4. **Constitutional alignment** (all 18 principles)
5. **Narrative partnership** (explain reasoning, offer choices)

**Inspired By**:
- /sp.error-analysis (AI-centric analysis pattern)
- Constitution v3.1.2 (18 principles, co-learning paradigm)
- Intelligent hybrid approach (optimize per-lesson, not per-chapter mode)

**Built With**:
- AI agent intelligence (context-aware judgments)
- Bash helper scripts (quantitative metrics)
- JSON structured data (parseable outputs)
- Markdown narrative reports (human-readable)

---

**This system intelligently brings existing chapters into constitutional compliance without unnecessary regeneration or quality loss.**
