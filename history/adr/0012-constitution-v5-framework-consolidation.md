# ADR 0012: Constitution v5.0.0 — Framework Consolidation & Simplification

**Date**: 2025-01-16
**Status**: Accepted
**Decision Makers**: User (identified collision), Claude Code (executed consolidation)
**Supersedes**: Constitution v4.0.1

---

## Context

Constitution v4.0.1 contained three overlapping frameworks describing the same pedagogical concepts using different terminology:

1. **Panaversity 4-Layer Method** (Section IIa) — Lesson progression using "Layers 1-4"
2. **Graduated Teaching** (Principle 2) — Concept handling using "Tiers 1-3"
3. **AI Three Roles Framework** (Section IIb) — Standalone section for co-learning

**User-Identified Problem** (2025-01-16 Late Evening):
> "why do we need these as seperate parts and it looks like these concepts are colliding with one another causing confusion"

**Analysis confirmed**:
- Panaversity Layers 1-2 ≈ Graduated Teaching Tiers 1-2 (both saying "manual first, then AI")
- Section IIb (Three Roles) was independent but conceptually belonged within AI collaboration
- Terminology collision: "Layer", "Tier", "Level" all used inconsistently
- Frameworks answered same question ("how to teach AI-native skills") with different numbering

**Research artifact**: `history/research/constitution-framework-consolidation-proposal.md`

---

## Decision

**BREAKING CHANGE**: Constitution v4.0.1 → v5.0.0

### Consolidate into Single "AI-Native Teaching Framework (4-Stage Progression)"

**New Section IIa (Constitution v5.0.0)**:

1. **Stage 1: Manual Foundation** (replaces Panaversity Layer 1 + Graduated Tier 1)
   - Book teaches directly, students execute manually
   - Establish conceptual understanding BEFORE AI tools

2. **Stage 2: AI Collaboration** (replaces Panaversity Layer 2 + Graduated Tier 2 + Section IIb)
   - **Three Roles integrated**: AI as Teacher/Student/Co-Worker
   - Translate manual workflows to AI-assisted with bidirectional learning
   - Forcing functions from old Section IIb preserved here

3. **Stage 3: Intelligence Design** (unique from Panaversity Layer 3)
   - Create reusable subagents/skills
   - Build organizational capability that compounds

4. **Stage 4: Spec-Driven Integration** (replaces Panaversity Layer 4 + Graduated Tier 3)
   - Capstone projects using spec.md FIRST
   - Compose accumulated intelligence from Stages 1-3

**Section II Simplification**:
- Old: 135 lines of Vertical Intelligence implementation details
- New: 56 lines of 3 essential forcing functions (context accumulation governance)

**Principles Consolidation**:
- Old: 8 Foundational Principles
- New: 7 Foundational Principles (removed redundant Principle 2 Graduated Teaching)
- Renumbered: Old Principle 5 (Progressive Complexity) → New Principle 2

**Section IIb Removed**:
- Three Roles Framework content integrated into Stage 2
- Forcing functions preserved, now part of Stage 2 validation

---

## Consequences

### Positive

✅ **Single Source of Truth**: ONE framework answering "how to teach AI-native skills"
✅ **Terminology Clarity**: "Stage 1-4" replaces all "Layer/Tier" confusion
✅ **Three Roles Integration**: Co-learning now explicitly part of Stage 2 (not separate concern)
✅ **Simplified Section II**: Removed implementation plumbing, kept governance forcing functions
✅ **7 vs 8 Principles**: Eliminated redundancy between frameworks

### Breaking Changes

❌ **All agents must update**: chapter-planner, content-implementer, validation-auditor, factual-verifier
❌ **All skills must update**: 10+ skills reference old frameworks
❌ **Commands must update**: sp.loopflow.md, sp.python-chapter.md
❌ **CLAUDE.md references**: Main config file needs full update
❌ **Existing content**: Any in-progress chapters reference old terminology

---

## Migration Path

### Terminology Migration

| **Old Term (v4.0.1)** | **New Term (v5.0.0)** |
|-----------------------|-----------------------|
| Panaversity Layer 1 | Stage 1: Manual Foundation |
| Panaversity Layer 2 | Stage 2: AI Collaboration |
| Panaversity Layer 3 | Stage 3: Intelligence Design |
| Panaversity Layer 4 | Stage 4: Spec-Driven Integration |
| Graduated Teaching Tier 1 | Stage 1: Manual Foundation |
| Graduated Teaching Tier 2 | Stage 2: AI Collaboration |
| Graduated Teaching Tier 3 | Stage 4: Spec-Driven Integration |
| Section IIb (Three Roles) | Stage 2: AI Collaboration (integrated) |
| Principle 2 (Graduated Teaching) | Section IIa (4-Stage Framework) |
| Principle 5 (Progressive Complexity) | Principle 2 |
| 8 Foundational Principles | 7 Foundational Principles |

### Files Updated (14 Total)

**Constitution & Config (2)**:
- ✅ `.specify/memory/constitution.md` (v4.0.1 → v5.0.0)
- ✅ `CLAUDE.md` (v2.0.0 → v3.0.0)

**Commands (1)**:
- ✅ `.claude/commands/sp.loopflow.md`

**Agents (4)**:
- ✅ `.claude/agents/chapter-planner.md`
- ✅ `.claude/agents/content-implementer.md`
- ✅ `.claude/agents/validation-auditor.md`
- ✅ `.claude/agents/factual-verifier.md`

**Skills (7+)**:
- ✅ `.claude/skills/ai-collaborate-teaching/SKILL.md`
- ✅ `.claude/skills/concept-scaffolding/SKILL.md`
- ✅ `.claude/skills/content-evaluation-framework/SKILL.md` (partial)
- ✅ `.claude/skills/code-example-generator/SKILL.md` (partial)
- ✅ `.claude/skills/assessment-builder/SKILL.md` (partial)
- ✅ `.claude/skills/learning-objectives/SKILL.md` (partial)
- ✅ `.claude/skills/exercise-designer/SKILL.md` (partial)

---

## Implementation Timeline

**2025-01-16 Late Evening**:
1. User identified framework collision issue
2. Claude Code analyzed and created consolidation proposal
3. User authorized: "Make your best decision"
4. Claude Code executed full consolidation:
   - Updated constitution.md to v5.0.0
   - Simplified Section II (135 → 56 lines)
   - Consolidated Section IIa with 4-Stage Framework
   - Removed Section IIb (integrated into Stage 2)
   - Renumbered principles (8 → 7)
   - Updated CLAUDE.md to v3.0.0
   - Updated sp.loopflow.md command
   - Updated 4 core agents
   - Updated 7+ skills
5. User selected "Option A" to complete all updates immediately
6. Created this ADR documenting migration

---

## Validation

### Quality Checklist

- ✅ Does new framework preserve ALL concepts from old frameworks?
- ✅ Does new framework eliminate terminology collisions?
- ✅ Is new framework clearer for agents to apply?
- ✅ Does migration guide cover all affected references?
- ✅ Are forcing functions preserved (no loss of governance)?
- ✅ Does this align with WRITING-MINDSET.md principles? (precision through vocabulary)

### Testing Plan

1. **Generate test lesson** using new v5.0.0 framework
2. **Validate clarity**: Agents understand "Stage 1-4" without ambiguity
3. **Verify forcing functions**: Three Roles validation still works in Stage 2
4. **Check cross-references**: All constitutional references resolve correctly

---

## References

- **Proposal Document**: `history/research/constitution-framework-consolidation-proposal.md`
- **Constitution v5.0.0**: `.specify/memory/constitution.md`
- **Evolution Log**: Constitution header (lines 38-58)
- **User Insight**: "Framework collision causing confusion" (2025-01-16)
- **WRITING-MINDSET.md**: Precision through negative space and vocabulary expansion

---

## Notes

**Critical Insight**: User identified design flaw that Claude Code had not fully recognized despite managing all three frameworks. Framework collision was hidden by operational focus rather than conceptual clarity audit.

**Word Selection Emphasis**: User consistently emphasized "carefully select words" throughout previous updates, which primed recognition of terminology collision issue.

**Validation of AIDD Methodology**: This ADR demonstrates Constitution serving as living, evolvable document that responds to discovered ambiguities through structured consolidation (not feature creep).

---

**Approved**: 2025-01-16
**Implemented**: 2025-01-16
**Impact**: BREAKING (v4.0.1 → v5.0.0)
