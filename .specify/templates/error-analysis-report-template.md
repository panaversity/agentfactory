---
id: {{ID}}
title: "Error Analysis: Chapter {{CHAPTER_NUMBER}}"
type: {{TYPE}}  # triage | full-analysis
date: {{DATE_ISO}}
chapter: {{CHAPTER_NUMBER}}
constitution_version: {{CONSTITUTION_VERSION}}
analyst: {{AI_MODEL}}
severity: {{SEVERITY}}  # PASS | DEGRADED | FAIL
---

# Error Analysis Report: Chapter {{CHAPTER_NUMBER}}

**Analysis Type**: {{TYPE}}
**Date**: {{DATE_ISO}}
**Constitution Version**: {{CONSTITUTION_VERSION}}
**Analyst**: {{AI_MODEL}}

---

## Executive Summary

**Verdict**: {{VERDICT}}
**Overall Severity**: {{SEVERITY}}
**Time to Analyze**: {{ANALYSIS_TIME}}
**Recommended Action**: {{RECOMMENDED_ACTION}}

**One-Line Summary**:
{{ONE_LINE_SUMMARY}}

---

## Context Discovered

### Chapter Metadata
- **Chapter Number**: {{CHAPTER_NUMBER}}
- **Part**: {{PART_NUMBER}}
- **Target Audience**: {{TARGET_AUDIENCE}}  # A1, A2, B1, etc.
- **Total Lessons**: {{LESSON_COUNT}}

### Artifacts Located
- **Spec**: {{SPEC_PATH}}
- **Plan**: {{PLAN_PATH}}
- **Tasks**: {{TASKS_PATH}}
- **Lessons**: {{LESSON_PATHS}}
- **PHR**: {{PHR_PATH}}
- **Validation Report**: {{VALIDATION_REPORT_PATH}}

---

## Phase 1: Quick Triage (5-Minute Assessment)

### Quantitative Metrics (from triage-metrics.sh)

```json
{{TRIAGE_METRICS_JSON}}
```

### AI Intelligent Judgment

**CoLearning Elements Status**:
{{COLEARNING_STATUS}}

**Lesson Closure Violations**:
{{CLOSURE_VIOLATIONS}}

**Forward Reference Flags**:
{{FORWARD_REFERENCE_FLAGS}}

**Verdict**: {{TRIAGE_VERDICT}}  # PASS | DEGRADED | FAIL

**Reasoning**:
{{TRIAGE_REASONING}}

---

## Phase 2: Deep Analysis (AI Intelligence at Core)

<!-- Only included in full-analysis type -->

### Check 1: Co-Learning Authenticity

**Question**: Are we doing genuine co-learning or synthetic compliance?

**Detailed Metrics** (from colearning-metrics.sh):
```json
{{COLEARNING_DETAILED_JSON}}
```

**AI Reading & Judgment**:

#### 💬 AI Colearning Prompts Analysis
- **Count**: {{PROMPT_COUNT}} (Expected: {{PROMPT_EXPECTED}})
- **Quality Assessment**:
  {{PROMPT_QUALITY_ANALYSIS}}

**Sample Prompts Read**:
1. {{PROMPT_SAMPLE_1}}
2. {{PROMPT_SAMPLE_2}}
3. {{PROMPT_SAMPLE_3}}

**Authenticity Score**: {{PROMPT_AUTHENTICITY_SCORE}} / 10
**Issues Found**: {{PROMPT_ISSUES}}

#### 🎓 Instructor Commentaries Analysis
- **Count**: {{COMMENTARY_COUNT}} (Expected: {{COMMENTARY_EXPECTED}})
- **Quality Assessment**:
  {{COMMENTARY_QUALITY_ANALYSIS}}

**Authenticity Score**: {{COMMENTARY_AUTHENTICITY_SCORE}} / 10
**Issues Found**: {{COMMENTARY_ISSUES}}

#### 🚀 CoLearning Challenges Analysis
- **Count**: {{CHALLENGE_COUNT}} (Expected: {{CHALLENGE_EXPECTED}})
- **Quality Assessment**:
  {{CHALLENGE_QUALITY_ANALYSIS}}

**Authenticity Score**: {{CHALLENGE_AUTHENTICITY_SCORE}} / 10
**Issues Found**: {{CHALLENGE_ISSUES}}

#### ✨ Teaching Tips Analysis
- **Count**: {{TIP_COUNT}} (Expected: {{TIP_EXPECTED}})
- **Quality Assessment**:
  {{TIP_QUALITY_ANALYSIS}}

**Authenticity Score**: {{TIP_AUTHENTICITY_SCORE}} / 10
**Issues Found**: {{TIP_ISSUES}}

**Check 1 Verdict**: {{CHECK_1_VERDICT}}  # ✅ PASS | ⚠️ DEGRADED | ❌ FAIL

---

### Check 2: Pedagogical Ordering (No Forward References)

**Question**: Can a beginner understand this without prior knowledge?

**Forward Reference Detection** (from detect-forward-references.sh):
```json
{{FORWARD_REFERENCE_JSON}}
```

**AI Contextual Reading**:

{{FORWARD_REFERENCE_ANALYSIS}}

**Per-Lesson Breakdown**:

| Lesson | Violations Found | Severity | AI Judgment |
|--------|------------------|----------|-------------|
{{PEDAGOGICAL_TABLE_ROWS}}

**Check 2 Verdict**: {{CHECK_2_VERDICT}}  # ✅ PASS | ⚠️ DEGRADED | ❌ FAIL

---

### Check 3: Constitutional Embodiment

**Question**: Is the lesson taught according to book constitution?

**Constitutional Rules Checked**:

#### Rule: Three-Role AI Partnership (Principle 18)
- **AI as Teacher**: {{AI_AS_TEACHER_STATUS}}
- **AI as Student**: {{AI_AS_STUDENT_STATUS}}
- **AI as Co-Worker**: {{AI_AS_COWORKER_STATUS}}
- **Verdict**: {{THREE_ROLE_VERDICT}}

#### Rule: Graduated Teaching Pattern (Principle 13)
- **Tier 1 (Book teaches)**: {{TIER_1_STATUS}}
- **Tier 2 (AI Companion)**: {{TIER_2_STATUS}}
- **Tier 3 (AI Orchestration)**: {{TIER_3_STATUS}}
- **Verdict**: {{GRADUATED_TEACHING_VERDICT}}

#### Rule: Cognitive Load Consciousness (Principle 12)
- **Target Level**: {{TARGET_CEFR_LEVEL}}
- **Max Concepts Allowed**: {{MAX_CONCEPTS}}
- **Concepts Per Lesson**: {{CONCEPTS_PER_LESSON}}
- **Verdict**: {{COGNITIVE_LOAD_VERDICT}}

#### Rule: Lesson Closure Pattern
- **"Try With AI" Present**: {{TRY_WITH_AI_PRESENT}}
- **Post-Sections Found**: {{POST_SECTIONS_FOUND}}
- **Verdict**: {{LESSON_CLOSURE_VERDICT}}

#### Rule: Specification-First Pedagogy (Principle 3)
- **Examples Show**: Spec → Prompt → Code → Validation?
- **Assessment**: {{SPEC_FIRST_ASSESSMENT}}
- **Verdict**: {{SPEC_FIRST_VERDICT}}

**Check 3 Verdict**: {{CHECK_3_VERDICT}}  # ✅ PASS | ⚠️ DEGRADED | ❌ FAIL

---

### Check 4: Real Learner Value (Beginner Simulation)

**Question**: Are we creating real value for new learners vs. "AI learning sugar-quoting"?

**Beginner Empathy Simulation**:

{{BEGINNER_SIMULATION_NARRATIVE}}

**Emotional Journey Tracking**:

| Lesson | Confusion Points | Aha Moments | Frustration Level | Learning Joy |
|--------|------------------|-------------|-------------------|--------------|
{{EMOTIONAL_JOURNEY_TABLE_ROWS}}

**Value Assessment**:
- **Actionable Learning**: {{ACTIONABLE_LEARNING_SCORE}} / 10
- **Cognitive Accessibility**: {{COGNITIVE_ACCESSIBILITY_SCORE}} / 10
- **Real-World Relevance**: {{REAL_WORLD_RELEVANCE_SCORE}} / 10
- **AI Partnership Value**: {{AI_PARTNERSHIP_VALUE_SCORE}} / 10

**Check 4 Verdict**: {{CHECK_4_VERDICT}}  # ✅ PASS | ⚠️ DEGRADED | ❌ FAIL

---

## Phase 3: Root Cause Analysis & Pattern Detection

### Systemic Patterns Identified

**Pattern 1**: {{PATTERN_1_NAME}}
- **Occurrences**: {{PATTERN_1_COUNT}} lessons
- **Root Cause**: {{PATTERN_1_ROOT_CAUSE}}
- **Impact**: {{PATTERN_1_IMPACT}}

**Pattern 2**: {{PATTERN_2_NAME}}
- **Occurrences**: {{PATTERN_2_COUNT}} lessons
- **Root Cause**: {{PATTERN_2_ROOT_CAUSE}}
- **Impact**: {{PATTERN_2_IMPACT}}

**Pattern 3**: {{PATTERN_3_NAME}}
- **Occurrences**: {{PATTERN_3_COUNT}} lessons
- **Root Cause**: {{PATTERN_3_ROOT_CAUSE}}
- **Impact**: {{PATTERN_3_IMPACT}}

### Root Cause Summary

{{ROOT_CAUSE_SUMMARY}}

### ROI of Systemic Fixes

**If we fix systemically (update workflow/templates)**:
- **Time Saved**: {{TIME_SAVED_ESTIMATE}}
- **Chapters Benefited**: {{CHAPTERS_BENEFITED}}
- **Recurrence Prevention**: {{RECURRENCE_PREVENTION}}

---

## Phase 4: Actionable Recommendations

### Critical Fixes (Must Do Immediately)

{{CRITICAL_FIXES}}

<!-- Example format:
1. **Remove all post-sections after "Try With AI"** (5 lessons affected)
   - Priority: CRITICAL
   - Time: 30 minutes
   - Impact: Constitutional compliance
-->

### Major Improvements (Should Do Soon)

{{MAJOR_IMPROVEMENTS}}

### Systemic Fixes (Prevent Recurrence)

{{SYSTEMIC_FIXES}}

<!-- Example format:
1. **Update content-implementer agent prompt**
   - Add explicit check: "No sections after Try With AI"
   - Add validation: CoLearning element minimum counts
   - Time: 1 hour
   - Prevents: 80% of current issues
-->

### Decision Points for User

**Would you like to**:
1. {{DECISION_OPTION_1}}
2. {{DECISION_OPTION_2}}
3. {{DECISION_OPTION_3}}
4. {{DECISION_OPTION_4}}

I'm here to execute your decision.

---

## Appendix: Detailed Metrics

### CoLearning Elements by Lesson

```json
{{COLEARNING_PER_LESSON_JSON}}
```

### Forward References by Lesson

```json
{{FORWARD_REFERENCES_PER_LESSON_JSON}}
```

### Constitutional Compliance Scorecard

| Principle | Status | Score | Notes |
|-----------|--------|-------|-------|
{{COMPLIANCE_SCORECARD_ROWS}}

---

## Meta: This Analysis Process

**Three-Role Framework Applied**:
- **AI as Teacher**: Diagnosed issues, suggested fixes
- **AI as Student**: Learned chapter context, read all content
- **AI as Co-Worker**: Partnered on recommendations, offered choices

**Co-Learning Convergence**:
- **Iteration 1**: Quantitative metrics (scripts)
- **Iteration 2**: Qualitative judgment (AI reading)
- **Iteration 3**: Root cause analysis (pattern detection)
- **Iteration 4**: Actionable recommendations (user partnership)

**This report embodies the co-learning paradigm it evaluates.**

---

**Report generated by**: {{AI_MODEL}}
**Report ID**: {{ID}}
**Location**: `history/error-analysis/{{FILENAME}}`
