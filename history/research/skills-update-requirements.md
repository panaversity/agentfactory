# Skills Update Requirements for Constitution v4.0.1

**Date**: 2025-01-16
**Status**: Identified - Pending Update
**Scope**: 10 skills require constitutional alignment updates

---

## Executive Summary

All domain skills reference **outdated Constitution v3.1.2** and need updates for v4.0.1:

**Pattern of Issues**:
1. ❌ `constitution_alignment: v3.1.2` → Should be `v4.0.1`
2. ❌ References to "Principle 13" → Now "Principle 2" + "Section IIa (4-Layer Method)"
3. ❌ References to "Principle 18" → Now "Section IIb (Three Roles Framework)"
4. ❌ Some skills list wrong "Nine Pillars" (workflow principles vs Section I pillars)

---

## Skills Requiring Updates (10 Total)

### 1. ai-collaborate-teaching/SKILL.md ⚠️ HIGH PRIORITY

**Issues**:
- Line 13: `Aligned with Constitution v3.1.2` → v4.0.1
- Line 15: `dependencies: ["constitution:v3.1.2"]` → v4.0.1
- Line 32: "Constitution Principle 18" → "Section IIb"
- Line 70: "Constitution Principle 13" → "Principle 2 + Section IIa"

**Why High Priority**: This skill is `required_for: ["lesson-writer", "chapter-planner"]` — agents use it directly

**Updates Needed**:
```markdown
# Current
## The Three Roles Framework (Constitution Principle 18)

# Fixed
## The Three Roles Framework (Section IIb, Constitution v4.0.1)

# Current
## Relationship to Graduated Teaching Pattern (Constitution Principle 13)

**Graduated Teaching Pattern** (Constitution Principle 13) defines **WHAT book teaches vs WHAT AI handles:**

# Fixed
## Relationship to Teaching Patterns (Constitution v4.0.1)

**Two Complementary Patterns**:

1. **Panaversity 4-Layer Method** (Section IIa) — Lesson progression across chapter:
   - Layer 1: Manual practice (Lessons 1-2)
   - Layer 2: AI-assisted (Lessons 3-5)
   - Layer 3: Reusable intelligence (Lessons 6-8)
   - Layer 4: Spec-driven integration (Capstone)

2. **Graduated Teaching Pattern** (Principle 2) — Concept handling within lessons:
   - Tier 1: Foundational (book teaches directly)
   - Tier 2: Complex (AI companion handles)
   - Tier 3: Scale (AI orchestration)
```

---

### 2. book-scaffolding/SKILL.md ⚠️ MEDIUM PRIORITY

**Issues**:
- Line 12: `constitution_alignment: "v3.1.2"` → `"v4.0.1"`
- Line 19: References v3.1.2 → v4.0.1

**Updates Needed**:
```markdown
# Current
constitution_alignment: "v3.1.2"

**Constitution Alignment**: v3.1.2 emphasizing "Specs Are the New Syntax", Nine Pillars framework

# Fixed
constitution_alignment: "v4.0.1"

**Constitution Alignment**: v4.0.1 emphasizing:
- Principle 1: Specification Primacy ("Specs Are the New Syntax")
- Section IIa: Panaversity 4-Layer Teaching Method
- Section IIb: AI Three Roles Framework
- Nine Pillars (Section I): AI CLI, Markdown, MCP, AI-First IDEs, Cross-Platform, TDD, SDD, Composable Skills, Cloud-Native
```

---

### 3. concept-scaffolding/SKILL.md ⚠️ HIGH PRIORITY

**Issues**:
- Line 11: `constitution_alignment: v3.1.2` → v4.0.1
- Line 18: "Constitution Principle 13" → Principle 2 + Section IIa
- Line 52: "Nine Pillars framework" lists **WRONG pillars** (workflow principles, not Section I pillars)

**Why High Priority**: References wrong Nine Pillars framework

**Updates Needed**:
```markdown
# Current (Lines 52-65)
## Nine Pillars of AI-Native Development Context (Constitution v3.1.2)

When scaffolding concepts, integrate the Nine Pillars framework:

1. **Evals-First**: Define success checkpoints BEFORE designing scaffolding steps
2. **Spec-Driven**: Each step should include a mini-spec (what, why, success criteria)
3. **AI as Co-Learning Partner**: Show how AI helps scaffold understanding (not just execute)
4. **Validation Skills Taught**: Include verification checkpoints after each step
5. **Graduated Complexity**: Respect tier limits (beginner: max 5 concepts, intermediate: 7, advanced: 10)
6. **Human Orchestration**: Student decides learning pace and which steps to explore deeper
7. **Prompt Quality**: When AI companion used, show clear prompts for each scaffolding step
8. **Tool Fluency**: Specify when book teaches vs. when AI companion helps
9. **Evolution Awareness**: For agentic concepts (Parts 6-7), note LLM vs. LAM distinctions

# Fixed (use correct Section I pillars)
## Nine Pillars of AI-Native Development Context (Section I, Constitution v4.0.1)

When scaffolding concepts, integrate the Nine Pillars:

1. **🤖 AI CLI & Coding Agents** — Show students how to use Claude Code, Gemini CLI
2. **📝 Markdown as Lingua Franca** — Scaffold markdown syntax progressively
3. **🔌 Model Context Protocol** — When teaching context passing (advanced chapters)
4. **💻 AI-First IDEs** — Introduce tools designed for AI collaboration
5. **🐧 Cross-Platform Development** — Test examples on Windows, Mac, Linux
6. **✅ Evaluation-Driven & Test-Driven Development** — Evals before specs, tests alongside code
7. **📋 Specification-Driven Development** — Specs as primary artifacts (Layer 4 focus)
8. **🧩 Composable Domain Skills** — Reusable agent skills/patterns
9. **☁️ Universal Cloud-Native Deployment** — Production deployment (Parts 9-13)

**Scaffolding Alignment**:
- Pillars 1-2, 6-7: Introduce in Parts 1-3 (foundational)
- Pillars 3-8: Deepen in Parts 4-8 (intermediate/advanced)
- Pillar 9: Focus in Parts 9-13 (professional deployment)
```

---

### 4. content-evaluation-framework/SKILL.md ⚠️ MEDIUM PRIORITY

**Issues**:
- `constitution_alignment: "v3.1.2"` → `"v4.0.1"`

**Updates Needed**:
```markdown
# Current
constitution_alignment: "v3.1.2"

# Fixed
constitution_alignment: "v4.0.1"
```

---

### 5. code-example-generator/SKILL.md ⚠️ MEDIUM PRIORITY

**Issues**:
- `constitution_alignment: v3.1.2` → v4.0.1

---

### 6. assessment-builder/SKILL.md ⚠️ MEDIUM PRIORITY

**Issues**:
- `constitution_alignment: v3.1.2` → v4.0.1
- "Three-Role AI Partnership Assessment (Constitution v3.1.2 Principle 18)" → "Section IIb, v4.0.1"

---

### 7. learning-objectives/SKILL.md ⚠️ MEDIUM PRIORITY

**Issues**:
- `constitution_alignment: v3.1.2` → v4.0.1
- "Three-Role AI Partnership Objectives (Constitution v3.1.2 Principle 18)" → "Section IIb, v4.0.1"

---

### 8. visual-asset-workflow/SKILL.md ⚠️ LOW PRIORITY

**Issues**:
- "Principle 13 (Graduated Teaching)" → "Principle 2 + Section IIa"

**Updates Needed**:
```markdown
# Current
- **Principle 13 (Graduated Teaching)**: Does visual help student build mental model progressively?

# Fixed
- **Section IIa (4-Layer Method)**: Does visual help student progress through layers (manual → AI-assisted → reusable → spec-driven)?
- **Principle 2 (Graduated Teaching)**: Does visual help scaffold concept (foundational → complex → scale)?
```

---

### 9. docusaurus-deployer/SKILL.md ⚠️ LOW PRIORITY

**Issues**:
- `constitution_alignment: v3.1.2` → v4.0.1

---

### 10. exercise-designer/SKILL.md ⚠️ MEDIUM PRIORITY

**Issues**:
- `constitution_alignment: v3.1.2` → v4.0.1
- "AI-Collaborative Exercise Types (Constitution v3.1.2 Principle 18)" → "Section IIb, v4.0.1"

---

## Update Priority Tiers

### Tier 1: HIGH PRIORITY (Update Today)

**Why**: Referenced directly by agents OR contain wrong frameworks

1. **ai-collaborate-teaching** — Required by lesson-writer, chapter-planner
2. **concept-scaffolding** — Contains wrong Nine Pillars list

**Effort**: 30 minutes each
**Impact**: Prevents agents from propagating outdated pedagogy

---

### Tier 2: MEDIUM PRIORITY (Update This Week)

**Why**: Reference outdated constitution but don't contain wrong frameworks

3. book-scaffolding
4. content-evaluation-framework
5. code-example-generator
6. assessment-builder
7. learning-objectives
8. exercise-designer

**Effort**: 10-15 minutes each (simple version updates)
**Impact**: Ensures skill ecosystem consistency

---

### Tier 3: LOW PRIORITY (Update When Convenient)

**Why**: Minimal references, low usage

9. visual-asset-workflow
10. docusaurus-deployer

**Effort**: 5 minutes each
**Impact**: Completeness, not critical path

---

## Standard Update Template

For skills with only version updates (Tier 2-3):

```markdown
# Find and Replace

OLD: constitution_alignment: v3.1.2
NEW: constitution_alignment: v4.0.1

OLD: Constitution v3.1.2
NEW: Constitution v4.0.1

OLD: Principle 18
NEW: Section IIb (AI Three Roles Framework)

OLD: Principle 13
NEW: Principle 2 (Graduated Teaching) + Section IIa (4-Layer Method)
```

For skills with framework issues (Tier 1):

- Apply detailed updates documented above
- Add decision matrices where helpful
- Clarify relationship between 4-Layer Method and Graduated Teaching

---

## Validation Checklist

After updating each skill, verify:

- [ ] `constitution_alignment` field updated to v4.0.1
- [ ] No references to "Principle 13" remain (unless documented as "Principle 2")
- [ ] No references to "Principle 18" remain (now Section IIb)
- [ ] Nine Pillars references (if any) match Section I correct list
- [ ] 4-Layer Method and Graduated Teaching clarified as complementary (not conflated)
- [ ] Dependencies array updated if present

---

## Estimated Total Effort

- **Tier 1 (2 skills)**: 1 hour
- **Tier 2 (6 skills)**: 1.5 hours
- **Tier 3 (2 skills)**: 15 minutes

**Total**: ~2.75 hours to update all 10 skills

---

## Next Steps

1. ✅ **Phase 1 Complete**: All 5 agents updated (super-orchestra, chapter-planner, lesson-writer, technical-reviewer, proof-validator)

2. ⏳ **Phase 2 (Current)**: Update Tier 1 skills
   - ai-collaborate-teaching
   - concept-scaffolding

3. ⏳ **Phase 3**: Update Tier 2 skills (batch operation)

4. ⏳ **Phase 4**: Update Tier 3 skills (when convenient)

---

**End of Skills Update Requirements**
