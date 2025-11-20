# /sp.error-analysis System Overview

**Version**: 1.0.0
**Created**: 2025-01-10
**Philosophy**: AI-centric analysis with quantitative support

---

## The Big Picture

`/sp.error-analysis` is an **AI-driven chapter quality analysis system** that evaluates chapters through three critical lenses:

1. **Co-Learning Authenticity**: Genuine bidirectional learning vs. synthetic compliance
2. **Constitutional Embodiment**: Does chapter practice what we preach?
3. **Real Learner Value**: Would a beginner actually learn from this?

**Key Innovation**: AI agent intelligence at core, scripts provide supporting metrics.

---

## Architecture

```
/sp.error-analysis (Command)
    ↓
AI Agent (Intelligence Layer)
    ├─ Reads Constitution (context)
    ├─ Reads Spec/Plan (intent)
    ├─ Reads Lessons (implementation)
    └─ Runs Scripts (quantitative data)
         ↓
Helper Scripts (Data Layer)
    ├─ artifact-locator.sh (find files)
    ├─ triage-metrics.sh (quick counts)
    ├─ colearning-metrics.sh (element breakdown)
    └─ detect-forward-references.sh (violation flags)
         ↓
JSON Output (structured data)
    ↓
AI Interpretation (qualitative judgment)
    ├─ Reads flagged content in context
    ├─ Applies pedagogical principles
    ├─ Judges severity (critical/major/minor)
    └─ Recommends strategic fixes
         ↓
Narrative Report (co-learning partnership)
    ├─ Executive summary
    ├─ Detailed findings
    ├─ Root cause analysis
    └─ Actionable recommendations
```

---

## Components

### 1. Command File

**Location**: `.claude/commands/sp.error-analysis.md`

**Responsibilities**:
- Orchestrate analysis phases
- Provide context-engineered instructions for AI
- Define evaluation criteria (Three-Role Framework, Graduated Teaching, etc.)
- Structure narrative report format

**Phases**:
- **Phase 0**: Context Immersion (AI learns chapter intent)
- **Phase 1**: Quick Triage (5-min assessment)
- **Phase 2**: Deep Analysis (AI reads + judges quality)
- **Phase 3**: Root Cause Analysis (pattern detection)
- **Phase 4**: Actionable Recommendations (co-learning partnership)

---

### 2. Helper Scripts

**Location**: `.specify/scripts/bash/error-analysis/`

**Philosophy**: Provide **data points**, not **decisions**.

**Scripts**:
1. `artifact-locator.sh` - Find all chapter artifacts
2. `triage-metrics.sh` - Quick quantitative assessment
3. `colearning-metrics.sh` - Detailed element breakdown
4. `detect-forward-references.sh` - Flag potential violations

**Output**: Structured JSON for AI parsing

---

### 3. Output Reports

**Location**: `history/error-analysis/chapter-{N}-{DATE}.md`

**Format**: Narrative co-learning report (NOT template fill-in)

**Structure**:
- Context Summary (what AI learned about chapter)
- Triage Verdict (PASS/DEGRADED/FAIL)
- Deep Analysis (3 checks: Co-Learning, Pedagogical Ordering, Constitutional Embodiment)
- Beginner Simulation (empathy-based value assessment)
- Root Cause Analysis (patterns detected)
- Actionable Recommendations (prioritized fixes with effort estimates)
- Decision Points (partner with user on strategy)

---

## Workflow Example

### User Invokes
```bash
/sp.error-analysis 14
```

### AI Agent Executes

**Phase 0: Context Immersion**
```
1. Read constitution (principles to apply)
2. Run artifact-locator.sh → Find all files
3. Read spec.md → Understand goals
4. Read plan.md → Understand strategy
5. Skim Chapter 13 → What student knows
6. Skim Chapter 15 → Where we're going
```

**Output**: Context Summary
```markdown
## Context Summary: Chapter 14

**Goal**: Teach 5 core data types to beginners
**Tier**: Beginner (max 5 concepts/lesson)
**Audience**: No prior programming
**Success Evals**: 80%+ can create typed variables

**Key Principles to Apply**:
- Principle 12: Cognitive load (max 5 concepts)
- Principle 13: Graduated teaching (book → AI companion)
- Principle 18: Three-role framework
```

---

**Phase 1: Triage**
```
1. Run triage-metrics.sh → Get quick counts
2. Read metrics + skim 2 lessons
3. Apply intelligence: Are violations critical?
```

**Output**: Triage Verdict
```markdown
## Triage Verdict

**Assessment**: FAIL

**One-Sentence**: Pedagogical ordering violations (`.upper()`, `isinstance()`)
prevent beginners from following examples.

**Critical Violations**: 2
- lesson-1.md:234 - `.upper()` method (Chapter 15 concept)
- lesson-2.md:123 - `isinstance()` + `def` (double forward reference)

**Recommendation**: Full analysis required
```

---

**Phase 2: Deep Analysis**

**Check 1: Co-Learning Authenticity**
```
1. Run colearning-metrics.sh → Get element counts
2. Read 3-5 example prompts → Judge quality
3. Evaluate Three-Role Framework:
   - AI as Teacher? (suggests patterns)
   - AI as Student? (adapts to feedback)
   - AI as Co-Worker? (collaborative language)
4. Evaluate Convergence Cycles:
   - One-shot prompts? (❌ synthetic)
   - Multi-iteration refinement? (✅ genuine)
```

**Output**: Co-Learning Verdict
```markdown
**Verdict**: DEGRADED (Synthetic Compliance)

**Quantitative**: 8 prompts (below 10-20 expected)
**Qualitative**: 80% are lookup-style ("Ask AI to explain")

**What's Missing**: Convergence cycles. Only 1 of 5 lessons shows
student ← AI → refine → converge pattern.

**Impact**: Students learn to use AI as search engine, not learning partner.
```

---

**Check 2: Pedagogical Ordering**
```
1. Run detect-forward-references.sh → Get violation flags
2. For each flag:
   - Read surrounding context (±5 lines)
   - Judge: CRITICAL/MAJOR/MINOR/FALSE_POSITIVE
3. Confirm violations with evidence
```

**Output**: Pedagogical Ordering Verdict
```markdown
**Verdict**: FAIL (2 critical violations)

**Violation 1: `.upper()` (Line 234)**
- Script flagged: String method not introduced
- AI read context: No explanation of methods
- Severity: CRITICAL (beginner doesn't know "." syntax)
- Impact: Student cannot understand example

**Violation 2: `isinstance()` (Line 123)**
- Script flagged: Type checking before functions
- AI read context: Uses `def` + `isinstance()` together
- Severity: CRITICAL (double forward reference)
- Impact: Student cannot run this code
```

---

**Check 3: Constitutional Embodiment**
```
1. Read Constitution Principle 13 (Graduated Teaching Pattern)
2. Map chapter content to tiers:
   - Tier 1 (Book teaches): Core data types ✅
   - Tier 2 (AI companion): Division edge cases ✅
   - Tier 1 (Book teaches): Type conversion ❌ (delegated to AI)
3. Identify misalignments
```

**Output**: Constitutional Embodiment Verdict
```markdown
**Verdict**: 70% Aligned (Partial)

**Misalignments**:
- Type conversion delegated to AI (should be book-taught)
- Floating point precision delegated to AI (foundational concept)

**Impact**: Students build weaker mental models (don't internalize foundations)
```

---

**Check 4: Beginner Simulation**
```
1. AI pretends to be complete beginner
2. Read Lesson 1 as if first time
3. Track:
   - Clarity moments ("Aha!")
   - Confusion moments ("Wait, what?")
   - Cognitive load spikes (too much at once)
   - Learning moments (genuine insight)
   - Emotional journey (confidence → frustration → breakthrough?)
```

**Output**: Real Learner Value Verdict
```markdown
**Verdict**: 4/10 (Below Acceptable)

**Emotional Journey**:
Minute 0-6: Confident (clear examples)
Minute 7-9: Confused (`.upper()` mystery)
Minute 13-15: Frustrated (`.len()` + f-string combo)
Minute 16-18: Breakthrough! (dictionary pattern from AI)

**Value Ratio**: 30% learning, 70% information

**Recommendation**: Fix critical issues (forward refs) + add 2 convergence
cycles → 85% completion target
```

---

**Phase 3: Root Cause Analysis**
```
1. Synthesize findings across all checks
2. Identify recurring patterns:
   - Pattern 1: Context pollution (content-implementer has future chapters)
   - Pattern 2: Weak convergence guidance (prompt doesn't enforce cycles)
   - Pattern 3: Tier boundary confusion (Tier 1 vs Tier 2 unclear)
3. Recommend systemic fixes (prevents recurrence)
```

**Output**: Root Cause Analysis
```markdown
## Root Cause Analysis

### Pattern 1: Context Pollution
- **Evidence**: All violations reference future chapters (15, 16, 20)
- **Root Cause**: Lesson-writer context includes Chapters 1-55
- **Fix**: Pre-validation gate (filter to Chapters 1-N)
- **Effort**: 4-6 hours
- **Impact**: 90% reduction in pedagogical violations

### Pattern 2: Weak Convergence Guidance
- **Evidence**: 80% one-shot prompts
- **Root Cause**: Prompt lacks convergence cycle template
- **Fix**: Add iteration structure to content-implementer prompt
- **Effort**: 1 hour
- **Impact**: 80% of lessons will have convergence

### Pattern 3: Tier Boundary Confusion
- **Evidence**: 30% foundational concepts delegated to AI
- **Root Cause**: No Tier 1/2 decision matrix
- **Fix**: Add rubric to content-implementer prompt
- **Effort**: 30 minutes
- **Impact**: 90% correct tier assignments
```

---

**Phase 4: Actionable Recommendations**
```
1. Prioritize fixes (critical → major → systemic)
2. Estimate effort (hours)
3. Predict impact (% improvement)
4. Calculate ROI (time saved)
5. Partner with user on strategy
```

**Output**: Recommendations + Decision Points
```markdown
## Recommended Action Plan

### Critical Fixes (Must Do)
1. Remove `.upper()` and `isinstance()` (1.5 hours)
2. Move foundational concepts from Tier 2 → Tier 1 (1 hour)
3. Fix lesson closure violation (5 min)

**Total**: 2.5 hours (minimum to publish)

---

### Systemic Fixes (Prevent Recurrence)
1. Pre-validation gate (4-6 hours)
2. Convergence cycle template (1 hour)
3. Tier decision matrix (30 min)

**Total**: 5.5-7.5 hours
**ROI**: Saves 109.5 hours across 42 remaining chapters (14.6x return)

---

## Decision Points

**What would you like to do?**
1. Quick fix (2.5h) → Publish with conditional pass
2. Recommended fix (3.5h) → Publish with pass quality
3. Ideal fix (7h) → Publish as model chapter
4. Systemic first (7.5h) → Fix content-implementer, regenerate Chapter 14

I'm here to execute your decision. What's the strategic priority?
```

---

## Key Differentiators

### vs. Traditional Linters

| Traditional Linter | /sp.error-analysis |
|--------------------|-------------------|
| Rule-based scoring | Context-aware judgment |
| Mechanical pass/fail | Narrative partnership |
| "Count < threshold = error" | "Read context, judge severity" |
| No pedagogical understanding | Applies learning science |
| No empathy simulation | Simulates beginner experience |

### vs. Manual Review

| Manual Review | /sp.error-analysis |
|---------------|-------------------|
| Time-intensive (hours) | Quick triage (5 min) + deep dive (20 min) |
| Inconsistent criteria | Constitutional principles |
| No pattern detection | Identifies systemic issues |
| Limited scope | Comprehensive (co-learning + pedagogy + value) |

### vs. Other AI Tools

| Generic AI Assistant | /sp.error-analysis |
|---------------------|-------------------|
| No project context | Constitution-aware |
| Generic feedback | Pedagogically grounded |
| No systemic analysis | Root cause + pattern detection |
| Advises, doesn't partner | Co-learning partnership |

---

## Success Metrics

**Quality Gates**:
- ✅ Co-learning authenticity (genuine bidirectional learning)
- ✅ Constitutional embodiment (practices what we preach)
- ✅ Real learner value (beginner can learn + gain confidence)

**Output Quality**:
- ✅ Narrative report (not template)
- ✅ Context-aware judgments (not mechanical scoring)
- ✅ Actionable recommendations (effort + impact + ROI)
- ✅ Strategic partnership (decision points, not dictation)

**System Health**:
- ✅ Scripts provide data (quantitative support)
- ✅ AI provides intelligence (qualitative judgment)
- ✅ Reports demonstrate co-learning (AI as Teacher/Student/Co-Worker)

---

## Maintenance & Evolution

### Adding New Checks

1. **Identify pedagogical principle** (from constitution)
2. **Define evaluation criteria** (what does "good" look like?)
3. **Add check to Phase 2** (in command file)
4. **Create helper script** (if quantitative data needed)
5. **Update SYSTEM-OVERVIEW.md** (document new check)

### Updating Heuristics

1. **Observe false positives** (script flags incorrectly)
2. **Refine detection patterns** (in helper scripts)
3. **Update severity thresholds** (critical vs. major vs. minor)
4. **Test on existing chapters** (verify improvement)

### Constitutional Updates

When constitution changes:
1. **Update command file** (Phase 0 references)
2. **Update evaluation criteria** (Phase 2 checks)
3. **Update expected ranges** (helper scripts if needed)
4. **Regenerate reports** (for affected chapters)

---

## Future Enhancements

### Short-Term (Next 3 Months)
- [ ] Add tone analysis script (conversational vs. documentation style)
- [ ] Add accessibility checks (reading level, jargon count)
- [ ] Add comparative analysis (`--compare-to` flag implementation)
- [ ] Add trend tracking (store results, show improvement over time)

### Medium-Term (Next 6 Months)
- [ ] Automated fix suggestions (generate edit patches)
- [ ] Batch analysis (analyze multiple chapters in parallel)
- [ ] Visual dashboards (quality metrics over time)
- [ ] Integration with validation workflow (run before validation-auditor)

### Long-Term (Next Year)
- [ ] Predictive analysis (flag issues during spec phase)
- [ ] Learning from fixes (improve detection heuristics automatically)
- [ ] Chapter quality scoring (0-100 scale with constitutional breakdown)
- [ ] Cross-chapter consistency checks (terminology, patterns, progression)

---

## Credits & Philosophy

**Design Principles**:
1. **AI intelligence at core** (scripts support, don't decide)
2. **Co-learning partnership** (AI as Teacher/Student/Co-Worker)
3. **Constitutional alignment** (evaluate against project principles)
4. **Pedagogical grounding** (learning science, not opinion)
5. **Actionable outputs** (effort estimates, ROI calculations, decision points)

**Inspired By**:
- Andrew Ng's Error Analysis Methodology (trace examination + HLP benchmarking)
- Constitution v3.1.2 (18 principles, 9 pillars, co-learning paradigm)
- SpecKit Plus SDD (spec → plan → implement → validate loop)
- Real beginner feedback (empathy simulation grounded in user research)

**Built With**:
- AI agent orchestration (Claude Sonnet 4.5)
- Bash helper scripts (quantitative support)
- JSON structured data (parseable output)
- Markdown narrative reports (human-readable analysis)

---

**This system embodies the co-learning paradigm it evaluates: AI teaches (diagnoses), AI learns (context), AI co-works (partners on solutions).**
