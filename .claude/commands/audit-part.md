---
description: Audit a book part against Panaversity teaching method frameworks
---

You are executing a comprehensive Part audit using the framework defined in `.claude/prompts/part-audit-prompt.md`.

## Your Task

**READ FIRST** (mandatory context gathering):
1. `.specify/experiment-prompts/part-audit-prompt.md` - Full audit framework
2. `.specify/memory/constitution.md` - Constitutional governance
3. `book-source/docs/chapter-index.md` - Part structure and tiers
4. Relevant teaching papers from `papers/` directory

**TARGET PART**: The user will specify which part to audit (e.g., "Part 4" or path to part directory)

**EXECUTION PROTOCOL**:

### Phase 1: Intelligence Gathering (30 min)
- Read constitution, extract 7 principles + 4-layer framework
- Read chapter-index, determine proficiency tier (A1-C2)
- Map part structure (chapters, lessons, prerequisites)
- Read teaching papers for pedagogical patterns

### Phase 2: Systematic Evaluation (2-3 hours)

Evaluate across 6 dimensions with weighted scoring:

**Dimension 1: Constitutional Compliance (30% weight)**
- Specification Primacy (spec before code?)
- Progressive Complexity (cognitive load matches tier?)
- Factual Accuracy (code tested, claims cited?)
- Pedagogical Structure (Foundation→Mastery arc?)
- Intelligence Accumulation (reuses earlier concepts?)
- Anti-Convergence (varied teaching modalities?)
- Minimal Content (only "Try With AI" endings?)

**Dimension 2: 4-Layer Framework (25% weight)**
- Layer 1: Manual Foundation present?
- Layer 2: Three Roles demonstrated? (AI as Teacher/Student/Co-Worker)
  - **CRITICAL**: Check for meta-commentary violations!
  - Run: `grep -i "What to notice\|AI.*teach\|AI.*learn" [lessons]`
- Layer 3: Skills created using P+Q+P pattern?
- Layer 4: Spec-driven capstone with intelligence composition?
- Stage transitions clear?

**Dimension 3: Pedagogical Coherence (20% weight)**
- Prerequisites managed correctly?
- Concepts scaffold incrementally?
- Pedagogical arc coherent across chapters?
- Learning objectives align with content?

**Dimension 4: Consistency & Quality (15% weight)**
- Writing quality consistent?
- Code tested and production-grade?
- Formatting/structure consistent?

**Dimension 5: Metadata & Technical (5% weight)**
- Metadata complete in chapter-index.md?
- Internal/external links working?

**Dimension 6: Reasoning Activation (5% weight)**
- Questions activate reasoning (not just recall)?
- Skills use P+Q+P pattern correctly?

### Phase 3: Synthesis & Reporting (30 min)

**OUTPUT FORMAT**:

```markdown
# Part Audit Report: [Part Name]

**Overall Score**: X/48 (X%)
**Quality Tier**: [Excellent 85-100% | Good 70-84% | Needs Work 50-69% | Insufficient <50%]
**Recommendation**: [PUBLISH | REVISE | REWORK]

## Executive Summary

**Top 3 Critical Issues**:
1. [P0 blocker issue with location]
2. [P0 blocker issue with location]
3. [P1 major issue with location]

**Top 3 Strengths**:
1. [What works exceptionally well]
2. [What works exceptionally well]
3. [What works exceptionally well]

## Dimension Scores

| Dimension | Score | Grade |
|-----------|-------|-------|
| Constitutional Compliance | X/21 | [%] |
| 4-Layer Implementation | X/12.5 | [%] |
| Pedagogical Coherence | X/8 | [%] |
| Consistency & Quality | X/4.5 | [%] |
| Metadata & Technical | X/1 | [%] |
| Reasoning Activation | X/1 | [%] |

## Detailed Findings

### Dimension 1: Constitutional Compliance

**Strengths**:
- [Specific examples of what's working]

**Weaknesses**:
- [Specific violations with chapter/lesson locations]

**Critical Issues**:
- [P0 issues blocking publication]

**Recommendations**:
- [Actionable fixes with evidence]

[Repeat for all 6 dimensions]

## Priority Issue Summary

| Priority | Issue | Location | Recommendation |
|----------|-------|----------|----------------|
| P0 | [Critical blocker] | Chapter X, Lesson Y | [Specific fix] |
| P1 | [Major issue] | Chapter X | [Specific fix] |
| P2 | [Polish item] | Multiple chapters | [Specific fix] |

## Evidence Appendix

### Validation Command Outputs

```bash
# Meta-commentary check
grep -i "What to notice\|AI.*teach" part-04/*/lesson-*.md
# Results: [paste output]

# Ending sections check
grep -E "^## (What's Next|Key Takeaways)" part-04/*/lesson-*.md
# Results: [paste output]

# Code testing check
grep -A 20 '```python' part-04/*/lesson-*.md | grep -i 'output:\|result:'
# Results: [paste output]
```

### Code Quality Examples
[Include specific good/bad code snippets with explanations]

### Link Validation
[Results of link checking]
```

## Important Notes

1. **Be specific**: Don't say "improve quality" - say "Lesson 3 shows code before specification (line 47), violating Principle 1"

2. **Provide evidence**: Include grep outputs, line numbers, specific examples

3. **Prioritize ruthlessly**:
   - P0 = Blocks publication (constitutional violations, broken code, meta-commentary)
   - P1 = Should fix before publication (quality issues, missing Layer 3)
   - P2 = Nice to have (formatting, polish)

4. **Reason, don't just check**: Ask "Does this achieve educational goals?" not just "Does this follow rules?"

5. **Time budget**: Spend 3-4 hours for thorough audit. Don't rush.

## Self-Monitoring

Before submitting report, ask yourself:
- Are findings specific enough to guide revision?
- Did I separate critical issues from polish items?
- Did I explain WHY something is a problem, not just WHAT?
- Would this report enable informed Publish/Revise/Rework decision?

---

**Execute this audit with reasoning mode activated. Your goal: provide actionable intelligence that enables quality decision-making.**
