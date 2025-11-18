# Agent Integration Guide: Reasoning-Activated Subagents

**Date**: 2025-01-17
**Version**: 2.0 (Reasoning Mode)
**Status**: Implementation Ready

---

## Quick Reference: When to Use Which Agent

```
┌─────────────────────────────────────────────────────────────────┐
│                    AGENT SELECTION DECISION TREE                 │
└─────────────────────────────────────────────────────────────────┘

QUESTION: What am I trying to validate/create?

├─ "Is this spec ready for planning?"
│  → USE: spec-architect
│  → WHEN: After /sp.specify, before /sp.plan
│  → OUTPUT: Specification quality analysis with refinement recommendations
│
├─ "Does this learning progression make pedagogical sense?"
│  → USE: pedagogical-designer
│  → WHEN: After /sp.plan (chapter-planner), before content-implementer
│  → OUTPUT: Layer progression validation, cognitive load analysis
│
├─ "Are these assessments aligned with objectives?"
│  → USE: assessment-architect (COMING SOON)
│  → WHEN: During lesson design, before implementation
│  → OUTPUT: Assessment specifications with Bloom's/CEFR mapping
│
├─ "Are these code examples production-quality?"
│  → WHEN: During lesson writing, before validation
│  → OUTPUT: Tested code examples with sandbox validation
│
├─ "Is this content ready for publication?"
│  → USE: validation-auditor (replaces validation-auditor + factual-verifier)
│  → WHEN: After content-implementer, before publication
│  → OUTPUT: Multi-dimensional quality report (technical + pedagogical + factual)
│
├─ "Are these factual claims accurate and cited?"
│  → USE: factual-verifier (COMING SOON)
│  → WHEN: Anytime during content creation/review
│  → OUTPUT: Citation audit with source verification
│
└─ "Does this task require comprehensive intelligence gathering?"
   → USE: super-orchestra
   → WHEN: Gap spans multiple sources, market-defining output required
   → OUTPUT: Deep research + planning + implementation + positioning
```

---

## Integration into Workflow Commands

### Integration Point 1: /sp.loopflow (Universal Orchestrator)

**Current State**: Uses chapter-planner, content-implementer, validation-auditor

**Updated Integration**:

```markdown
## PHASE 1: SPECIFICATION + CLARIFICATION GATE

[Existing /sp.specify invocation]
[Existing /sp.clarify invocation]

### NEW: Spec Quality Validation

After /sp.clarify completes, YOU MUST invoke spec-architect:

Task(
  subagent_type="spec-architect",
  description="Validate specification quality",
  prompt="Analyze specs/[feature-slug]/spec.md for testability, completeness, ambiguity, and traceability. Ensure evals-first pattern followed."
)

IF verdict = "NEEDS CLARIFICATION" OR "INCOMPLETE":
  Apply refinements recommended by spec-architect
  Re-run /sp.clarify with updated spec
  REPEAT until verdict = "READY"

BLOCK: User must approve spec before proceeding to Phase 2.
```

**Updated Phase 2**:

```markdown
## PHASE 2: PLANNING + ADR GATE

[Existing /sp.plan invocation]

### NEW: Pedagogical Progression Validation (Book Chapters Only)

IF task_type = "book_chapter":

  Task(
    subagent_type="pedagogical-designer",
    description="Validate learning progression",
    prompt="Analyze specs/[feature-slug]/plan.md for Layer 1-4 progression, cognitive load limits (CEFR), dependency ordering, and mental model building."
  )

  IF verdict = "NEEDS ADJUSTMENT" OR "REQUIRES RESTRUCTURING":
    Apply fixes recommended by pedagogical-designer
    Update plan.md
    REPEAT until verdict = "SOUND PROGRESSION"

[Existing ADR gate]

BLOCK: User must approve plan before proceeding to Phase 3.
```

**Updated Phase 4**:

```markdown
## PHASE 4: IMPLEMENTATION + VALIDATION GATE

[Existing /sp.implement invocation]

### STEP 3: Multi-Dimensional Validation (CONSOLIDATED)

YOU MUST invoke validation-auditor (replaces validation-auditor + factual-verifier):

Task(
  subagent_type="validation-auditor",
  description="Comprehensive quality validation",
  prompt="Validate [feature-slug] across 4 dimensions:

  1. Technical Correctness (code runs, types present, security practices)
  2. Pedagogical Effectiveness (objectives align, scaffolding sound, Three Roles demonstrated)
  3. Factual Accuracy (claims cited, examples current, volatile topics flagged)
  4. Accessibility (terminology defined, examples diverse, language inclusive)

  Output publication readiness verdict with CRITICAL/MAJOR/MINOR issue classification."
)

Wait for validation report:
- APPROVE → Continue to sandbox validation
- REVISE & RESUBMIT → Fix major issues, re-validate spot-check
- RETURN FOR REVISION → Critical issues or widespread gaps, consult before re-validation

IF NOT APPROVE:
  Apply fixes for issues identified
  Re-run validation-auditor
  REPEAT until APPROVE

[Existing sandbox validation]
```

---

### Integration Point 2: /sp.specify (Specification Creator)

**Enhancement**: Add spec-architect validation to /sp.specify itself

```markdown
## Internal Quality Check (New Step)

After generating spec.md but BEFORE returning to user:

1. Self-validate using spec-architect reasoning framework:
   - Are acceptance criteria measurable (no subjective terms)?
   - Are constraints and non-goals explicit?
   - Is evals-first pattern followed (evals section exists FIRST)?
   - Are ambiguous terms defined?

2. IF self-validation detects issues:
   - Auto-fix if straightforward (e.g., add missing evals section)
   - Flag for /sp.clarify if complex (e.g., vague requirements)

3. Add spec quality metadata to spec.md:
   ```yaml
   spec_quality:
     testability: [score/10]
     completeness: [score/10]
     self_validated: true
     clarification_needed: [yes/no]
   ```

This pre-validation reduces /sp.clarify iterations.
```

---

### Integration Point 3: /sp.plan (Plan Creator)

**Enhancement**: Add pedagogical-designer validation for book chapters

```markdown
## Pedagogical Quality Check (New Step for Book Chapters)

After generating plan.md but BEFORE returning to user:

1. Self-validate using pedagogical-designer reasoning framework:
   - Layer 1-4 progression followed? (No Layer violations)
   - Cognitive load within CEFR limits? (Count concepts per section)
   - Dependencies properly ordered? (Prerequisites before dependents)
   - Mental model foundation built? (Layer 1 manual practice)

2. Add pedagogical metadata to plan.md:
   ```yaml
   pedagogical_quality:
     layer_progression: [compliant/violations_found]
     cognitive_load_status: [within_limits/overload_detected]
     dependency_ordering: [correct/violations_found]
     max_concepts_per_section: [number]
     target_cefr_level: [A1/A2/B1/B2/C1/C2]
   ```

3. IF violations detected:
   - Auto-fix if simple (e.g., reduce concept count in section)
   - Flag for human review if structural (e.g., Layer progression violation)

This pre-validation ensures pedagogically sound plans.
```

---

### Integration Point 4: /sp.python-chapter (Python Chapter Workflow)

**Current State**: Chains /sp.specify → /sp.plan → /sp.tasks → /sp.implement

**Updated Integration** (add spec-architect + pedagogical-designer gates):

```markdown
## PHASE 1: SPECIFICATION

[Invoke /sp.specify as before]

### NEW: Specification Quality Gate

Task(
  subagent_type="spec-architect",
  description="Validate Python chapter spec",
  prompt="Analyze spec for Python chapter. Ensure:
  - Python-specific evals measurable (students can write/debug code)
  - Constraints include Python 3.13+, type hints, PEP 8
  - Non-goals prevent scope creep (no web frameworks in basics chapter)
  - Code examples testable in sandbox"
)

BLOCK: Spec must pass spec-architect before planning.

## PHASE 2: PLANNING

[Invoke /sp.plan as before]

### NEW: Pedagogical Progression Gate

Task(
  subagent_type="pedagogical-designer",
  description="Validate Python chapter learning progression",
  prompt="Analyze lesson sequence for Python chapter. Ensure:
  - Layer 1: Manual Python coding (no AI yet)
  - Layer 2: AI-assisted Python (prompt engineering for code)
  - Layer 3: Create reusable Python patterns (skills/subagents)
  - Layer 4: Spec-driven Python project (compose accumulated intelligence)

  Validate cognitive load for target tier (A2/B1/C2).
  Check prerequisite ordering (variables → lists → loops → comprehensions)."
)

BLOCK: Plan must pass pedagogical-designer before tasks.

[Continue with /sp.tasks → /sp.implement as before]
```

---

### Integration Point 5: /fact-check-lesson (Factual Accuracy Command)

**Current State**: Standalone command for fact-checking lessons

**Enhanced Integration** (use factual-verifier agent):

```markdown
## Fact-Checking Workflow

When /fact-check-lesson is invoked:

Task(
  subagent_type="factual-verifier",
  description="Verify factual claims in lesson",
  prompt="Read lesson file: [path]

  Identify all factual claims:
  - Statistics (must have source + date)
  - Technical specifications (versions, APIs, commands)
  - Historical dates/events
  - Examples (must be current, not outdated)

  Verify each claim:
  - Check inline citations present ([Source, Year])
  - Validate source authority (primary > secondary > tertiary)
  - Flag volatile topics for maintenance (AI tools, APIs)
  - Verify examples current (not 2019 Python 2 code in 2025)

  Output citation audit report with:
  - Verified claims: [count]
  - Unverified claims: [list with recommendations]
  - Maintenance triggers: [volatile topics requiring annual review]"
)

Output factual-verifier report.

IF unverified claims > 0:
  BLOCK: Must add citations before publication
ELSE:
  PASS: Factual accuracy validated
```

---

## Agent Workflow Patterns

### Pattern 1: Sequential Validation (Spec → Plan → Tasks)

```
User invokes: /sp.loopflow "Chapter 15: Python Decorators"

Phase 0: Deep Search
→ Reads constitution, chapter-index, existing specs
→ Derives: Part 4 (Intermediate B1), prerequisites (Chapter 12-14)

Phase 1: Specification
→ /sp.specify creates spec.md
→ /sp.clarify asks 3 questions, updates spec
→ spec-architect validates (READY verdict)
→ User approves spec

Phase 2: Planning
→ /sp.plan creates plan.md
→ pedagogical-designer validates (SOUND PROGRESSION)
→ User approves plan

Phase 3: Tasks
→ /sp.tasks creates tasks.md
→ /sp.analyze checks consistency
→ User approves tasks

Phase 4: Implementation
→ /sp.implement → content-implementer creates content
→ validation-auditor validates (APPROVE)
→ Sandbox testing (PASS)
→ User approves implementation

Phase 5: Finalization
→ Chapter index updated
→ PHR created
→ Git workflow optional
```

**Agent Flow**:
```
spec-architect → pedagogical-designer → validation-auditor → factual-verifier
(gates prevent bad specs) (gates prevent bad progression) (gates prevent bad quality) (gates prevent false claims)
```

---

### Pattern 2: Concurrent Validation (Parallel Quality Checks)

```
User invokes: validation-auditor on completed lesson

validation-auditor executes 4 parallel sub-checks:

1. Technical Correctness Thread:
   → Extract all code blocks
   → Sandbox test each example
   → Verify type hints, PEP 8
   → Check security practices

2. Pedagogical Effectiveness Thread:
   → Call pedagogical-designer
   → Validate Layer progression
   → Check cognitive load
   → Verify Three Roles demonstrated

3. Factual Accuracy Thread:
   → Call factual-verifier
   → Verify all claims cited
   → Check source authority
   → Flag volatile topics

4. Accessibility Thread:
   → Scan for gatekeeping terms
   → Check example diversity
   → Verify terminology defined
   → Validate inclusive language

Aggregate results:
→ CRITICAL issues from any thread → RETURN FOR REVISION
→ MAJOR issues only → REVISE & RESUBMIT
→ MINOR issues only → APPROVE with polish suggestions
```

**Agent Flow**:
```
                    validation-auditor (orchestrator)
                           |
        ┌──────────────────┼──────────────────┬──────────────┐
        │                  │                  │              │
pedagogical-designer  [code sandbox]  factual-verifier  [accessibility scan]
        │                  │                  │              │
        └──────────────────┴──────────────────┴──────────────┘
                           │
                   Aggregate Report
                (APPROVE / REVISE / RETURN)
```

---

### Pattern 3: Iterative Refinement (Fix-Validate Loop)

```
Lesson created by content-implementer

→ validation-auditor reports: REVISE & RESUBMIT
  Issues:
  - MAJOR: Cognitive overload in Section 2 (10 concepts, A2 limit is 7)
  - MAJOR: Missing source citation for statistics
  - MINOR: Typo in code comment

→ Apply fixes:
  1. pedagogical-designer suggests: Split Section 2 into two sections (5 + 5 concepts)
  2. factual-verifier finds source: [World Bank, 2023]
  3. Fix typo

→ Re-run validation-auditor
  → Reports: APPROVE (all issues resolved)

→ Proceed to publication
```

**Agent Flow**:
```
content-implementer → validation-auditor (FAIL)
                       ↓
            pedagogical-designer (suggests fix)
            factual-verifier (finds source)
                       ↓
                Apply fixes
                       ↓
                validation-auditor (PASS)
                       ↓
                 Publication
```

---

## Migration Plan from Old Agents

### Phase 1: Consolidate (Immediate)

**Merge validation-auditor + factual-verifier → validation-auditor**

```bash
# Backup old agents
mv .claude/agents/validation-auditor.md .claude/agents/_backup/
mv .claude/agents/factual-verifier.md .claude/agents/_backup/

# Validation-auditor already created (combines both)
# Update all references in workflow commands

# Files to update:
- .claude/commands/sp.loopflow.md (Phase 4 validation)
- .claude/commands/sp.python-chapter.md (validation step)
- .claude/commands/sp.implement.md (if exists)
```

**Update Command References**:

Find/Replace across `.claude/commands/`:
```
OLD: validation-auditor
NEW: validation-auditor

OLD: factual-verifier
NEW: validation-auditor (for factual accuracy, use factual-verifier)
```

---

### Phase 2: Add New Agents (Week 1)

**Create agent definition files** (2 completed, 2 remaining):

- [x] `spec-architect.md` (DONE)
- [x] `pedagogical-designer.md` (DONE)
- [ ] `assessment-architect.md` (TODO)
- [ ] `factual-verifier.md` (TODO)
- [x] `validation-auditor.md` (TODO - consolidate from validation-auditor + factual-verifier)

**Integration sequence**:

1. **Week 1 Day 1-2**: Create `assessment-architect.md`
   - Reasoning framework for evaluation design
   - Bloom's taxonomy alignment
   - CEFR proficiency mapping
   - Integration: Add to lesson planning workflow

   - Reasoning framework for production code quality
   - Sandbox testing requirements
   - Cross-platform validation
   - Integration: Add to lesson implementation workflow

3. **Week 1 Day 5**: Create `factual-verifier.md`
   - Reasoning framework for source accuracy
   - Citation verification protocols
   - Maintenance trigger flagging
   - Integration: Extract from validation-auditor, make standalone

4. **Week 1 Day 6-7**: Create `validation-auditor.md`
   - Consolidate validation-auditor + factual-verifier
   - Multi-dimensional quality framework
   - Orchestrate sub-validations (pedagogical, factual, code, accessibility)
   - Integration: Replace all old agent references

---

### Phase 3: Test Reasoning Activation (Week 2)

**Validation Test Cases**:

Test each agent with edge cases to confirm reasoning mode (not prediction mode):

**Test 1: spec-architect Reasoning**
```
Input: Vague spec with "make it secure, user-friendly, and fast"
Expected: Identifies ambiguity, proposes measurable alternatives
  "secure" → "bcrypt 12+ rounds, rate limiting 5/hour, no plain text"
  "user-friendly" → "95%+ users complete setup <3 min"
  "fast" → "API response time <200ms p95"

PASS: Agent reasons about testability
FAIL: Agent accepts vague terms
```

**Test 2: pedagogical-designer Reasoning**
```
Input: Lesson sequence teaching decorators before higher-order functions
Expected: Detects dependency violation, proposes correct ordering
  Violation: "Decorators require HOF understanding"
  Fix: "Teach HOF in Lesson N-1, decorators in Lesson N"

PASS: Agent reasons about dependencies
FAIL: Agent accepts inverted ordering
```

**Test 3: validation-auditor Reasoning**
```
Input: Code example with hardcoded API key
Expected: Flags CRITICAL security violation
  Issue: "Line 42: Hardcoded secret in plain text"
  Recommendation: "Use environment variables, add to .gitignore"

PASS: Agent reasons about security implications
FAIL: Agent accepts hardcoded secrets
```

---

### Phase 4: Workflow Integration (Week 3)

**Update all workflow commands** to use new agents:

1. `/sp.loopflow` (highest priority - universal orchestrator)
2. `/sp.python-chapter` (Python-specific workflow)
3. `/sp.specify` (add spec-architect self-validation)
4. `/sp.plan` (add pedagogical-designer self-validation)
5. `/fact-check-lesson` (integrate factual-verifier)

**Integration Checklist**:
- [ ] All old agent references removed (validation-auditor, factual-verifier)
- [ ] New agents invoked at appropriate gates (spec-architect after /sp.specify, etc.)
- [ ] Approval gates enforce quality (BLOCK until agent verdict = READY/PASS)
- [ ] Error handling for agent failures (retry logic, escalation to user)
- [ ] Documentation updated (README, agent usage examples)

---

## Success Metrics

### Reasoning Mode Validation

**Test**: Present agent with novel situation not in documentation

**spec-architect Test**:
```
Input: "Add authentication for a quantum computing API"
Expected: Applies principles (testability, constraints) to novel domain
  - Identifies quantum-specific constraints (qubit entanglement security?)
  - Proposes testable criteria (quantum state verification methods?)
  - Flags ambiguities unique to quantum domain

PASS: Agent reasons from principles
FAIL: Agent says "I don't have rules for quantum APIs"
```

**pedagogical-designer Test**:
```
Input: "Teach quantum entanglement to A2 learners"
Expected: Applies cognitive load limits to novel subject
  - Recognizes A2 limit (7 concepts max)
  - Identifies foundational prerequisites (what's a qubit?)
  - Proposes analogies for abstract concepts
  - Structures progression (classical → quantum)

PASS: Agent adapts principles to new domain
FAIL: Agent rejects because "not in database"
```

---

### Clear Responsibility Boundaries

**Test**: Given ambiguous task, only ONE agent should be obvious choice

**Scenario**: "Validate lesson code examples"

**Before (Overlap)**:
```
Options:
- validation-auditor (checks code quality)
- factual-verifier (checks factual accuracy of code)
→ UNCLEAR: Which to use?
```

**After (Clear Boundaries)**:
```
Decision tree:
1. Are examples created yet?
   YES → Continue

2. Do examples need validation?
   YES → validation-auditor (validates correctness + pedagogy + accuracy)

3. Are factual claims in examples cited?
   NO → factual-verifier (verifies sources for claims)

```

---

### Layer Integration

**Test**: Check agents explicitly map to 4-Layer methodology

**Mapping Verification**:
```
Layer 2 (AI Collaboration):
- content-implementer (executes lesson writing with AI assistance)
- validation-auditor (validates quality with human approval)

Layer 3 (Intelligence Design):
- pedagogical-designer (creates learning progression intelligence)
- assessment-architect (creates evaluation design intelligence)

Layer 4 (Spec-Driven Orchestration):
- spec-architect (validates specification quality before planning)
- super-orchestra (orchestrates deep research + multi-agent workflows)

Cross-Layer:
- factual-verifier (validates sources at any layer)

→ PASS: All agents map to layers
→ FAIL: Agents exist independently of framework
```

---

## Agent Usage Examples (Quick Start)

### Example 1: Validate Spec Before Planning

```bash
# After creating spec with /sp.specify
Task(
  subagent_type="spec-architect",
  description="Validate chapter spec",
  prompt="Analyze specs/chapter-15/spec.md for testability, completeness, ambiguity. Ensure evals-first pattern followed."
)

# Agent returns:
# Verdict: NEEDS CLARIFICATION
# Issue: FR-003 "make it fast" is unmeasurable
# Recommendation: Replace with "API response <200ms p95"

# Apply fix, re-run spec-architect
# Verdict: READY FOR PLANNING
# Proceed to /sp.plan
```

---

### Example 2: Validate Learning Progression

```bash
# After creating plan with /sp.plan
Task(
  subagent_type="pedagogical-designer",
  description="Validate lesson sequence",
  prompt="Analyze specs/chapter-15/plan.md for Layer 1-4 progression, cognitive load (B1 tier), dependency ordering."
)

# Agent returns:
# Verdict: NEEDS ADJUSTMENT
# Issue: Lesson 1 uses "tell your AI" prompts (Layer 1 violation)
# Recommendation: Move AI prompts to Lesson 3 (Layer 2), teach manual decorators in Lesson 1

# Apply fix, re-run pedagogical-designer
# Verdict: SOUND PROGRESSION
# Proceed to /sp.tasks
```

---

### Example 3: Multi-Dimensional Validation

```bash
# After lesson implementation
Task(
  subagent_type="validation-auditor",
  description="Comprehensive quality check",
  prompt="Validate docs/part-4/chapter-15/lesson-1-decorators.md across all dimensions (technical, pedagogical, factual, accessibility)."
)

# Agent returns:
# Verdict: REVISE & RESUBMIT
# CRITICAL: None
# MAJOR:
#   - Cognitive overload in Section 2 (10 concepts, B1 limit is 10) → borderline
#   - Missing citation for "80% of Python developers use decorators"
# MINOR:
#   - Typo in code comment line 42

# Fix issues, re-run validation-auditor
# Verdict: APPROVE
# Proceed to publication
```

---

## Troubleshooting

### Issue: Agent Gives Generic "Add More Detail" Feedback

**Problem**: Agent operating in prediction mode (not reasoning mode)

**Solution**: Check agent prompt includes Persona + Questions + Principles structure

```markdown
❌ BAD (Prediction Mode):
"Review the spec and provide feedback"

✅ GOOD (Reasoning Mode):
"You are a specification architect who thinks about requirements like
a compiler designer thinks about grammars.

Before approving, analyze:
1. Testability: Are acceptance criteria measurable?
2. Completeness: What's missing?
3. Ambiguity: Where could interpretations diverge?

Principles:
- Intent over implementation (WHAT before HOW)
- Measurable success (objective pass/fail)
- Explicit constraints (boundaries matter)"
```

---

### Issue: Multiple Agents Give Overlapping Feedback

**Problem**: Unclear responsibility boundaries

**Solution**: Consolidate agents or clarify distinct domains

```
Before (Overlap):
- validation-auditor checks code quality
- factual-verifier checks code accuracy
→ Both validate code (overlap)

After (Distinct):
- validation-auditor VALIDATES code (technical + pedagogical + factual)
- factual-verifier VERIFIES claims in code
→ Clear: create → validate → verify
```

---

### Issue: Agent Rejects Novel Situations

**Problem**: Agent has rigid rules (if/then) instead of principles

**Solution**: Redesign agent with decision frameworks

```markdown
❌ BAD (Rigid Rules):
"IF Python code THEN check PEP 8"
"IF JavaScript code THEN reject (only Python allowed)"

✅ GOOD (Principles):
"Apply code quality principles:
- Clarity over cleverness (readable by target audience)
- Production patterns (not toy examples)
- Cross-platform tested (works on Windows/Mac/Linux)

For Python: PEP 8, type hints
For JavaScript: ESLint, TypeScript
For any language: Security, error handling, testability"
```

---

## Next Steps

**Immediate** (This Week):
- [ ] Test spec-architect and pedagogical-designer on existing specs/plans
- [ ] Validate reasoning activation with edge cases
- [ ] Update /sp.loopflow Phase 1 and Phase 2 with new agents

**Week 1**:
- [ ] Create assessment-architect.md
- [ ] Create factual-verifier.md
- [ ] Create validation-auditor.md (consolidate old agents)

**Week 2**:
- [ ] Test all agents with edge cases
- [ ] Validate reasoning mode (not prediction mode)
- [ ] Fix any agent design issues

**Week 3**:
- [ ] Update all workflow commands (.claude/commands/)
- [ ] Remove old agent references
- [ ] Documentation updates
- [ ] End-to-end workflow testing

**Success Criteria**:
- All agents use Persona + Questions + Principles structure
- Clear responsibility boundaries (no overlap)
- Explicit 4-Layer integration
- Reasoning mode validated (agents handle novel situations)

---

**Document Status**: Implementation Guide (Ready for Execution)
**Approval**: Review agent definitions, approve migration plan
**Contact**: Update workflows incrementally, test after each integration
