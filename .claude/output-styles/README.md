---
title: Output Styles Framework - 4-Layer Reasoning-Activated Pedagogy
version: 2.0.0
created: 2025-01-17
---

# Output Styles Framework: Reasoning-Activated Pedagogy

## Overview

This framework provides **reasoning-activated output styles** for educational content creation aligned with the 4-Layer Teaching Method. Each style activates specific cognitive modes in both agents and learners appropriate to the learning layer.

**Core principle**: Output styles are not structural templates—they are pedagogical frameworks that activate reasoning modes.

---

## Framework Architecture

```
.claude/output-styles/
├── README.md                      # This file - framework overview
├── pedagogical/                   # Reasoning-activated styles (NEW)
│   ├── layer-1-foundation.md     # Direct teaching mode
│   ├── layer-2-collaboration.md  # AI partnership mode
│   ├── layer-3-intelligence.md   # Reusable pattern mode
│   ├── layer-4-orchestration.md  # Spec-driven mode
│   └── decision-tree.md          # Style selection guide
├── structural/                    # Content templates
│   ├── lesson-template.md        # YAML + structure
│   ├── chapter-readme-template.md # Chapter overview
│   ├── part-readme-template.md   # Part overview
│   └── file-organization.md      # Docusaurus config
└── workflows/                     # Process styles
    └── super-orchestra.md        # Deep research mode
```

---

## The 4-Layer Pedagog ical Framework

### Layer 1: Foundation Style (Manual Practice Mode)

**Purpose**: Build mental models through direct instruction before AI assistance

**When to use**:
- First exposure to concept
- Stable knowledge (won't change in 2+ years)
- Mental model required for quality evaluation
- Student needs debugging intuition

**Reasoning activation**: "Can I explain this? Can I execute independently? Can I recognize errors?"

**File**: `pedagogical/layer-1-foundation.md`

**Content characteristics**:
- Explanatory voice with analogies
- Step-by-step walkthroughs with explicit reasoning
- Self-validation criteria throughout
- Manual practice exercises
- AI role minimal or absent

---

### Layer 2: Collaboration Style (AI Partnership Mode)

**Purpose**: Develop prompting, validation, and collaboration skills through Three Roles framework

**When to use**:
- Student completed Layer 1 (has foundation)
- Multi-step task with complexity
- AI can suggest optimizations
- Student needs output evaluation skills

**Reasoning activation**: "What can AI teach me? How do I refine AI's understanding? What emerges through iteration?"

**File**: `pedagogical/layer-2-collaboration.md`

**Content characteristics**:
- Collaborative voice ("Let's explore together")
- Three Roles demonstrated (AI as Teacher, Student, Co-Worker)
- Iterative convergence loops shown explicitly
- Evaluation frameworks (Correctness, Clarity, Alignment)
- CoLearning elements (💬🎓🤝) integrated throughout

**Mandatory requirements**:
- ✅ AI teaches student (pattern they didn't know)
- ✅ Student teaches AI (provides constraints/context)
- ✅ Convergence loop (iteration toward better solution)

---

### Layer 3: Intelligence Design Style (Reusable Pattern Mode)

**Purpose**: Transform tacit knowledge into explicit, reusable intelligence using Persona+Questions+Principles pattern

**When to use**:
- Pattern encountered 2+ times (recurring workflow)
- 5+ decision points (sufficient complexity)
- Cross-project value (applies to 3+ projects)
- Encoding cost justified by reuse

**Reasoning activation**: "What's the general pattern? What decision framework guides application? How do I make this reusable?"

**File**: `pedagogical/layer-3-intelligence.md`

**Content characteristics**:
- Abstraction voice (specific to general)
- Persona + Questions + Principles structure
- Skill vs Subagent decision framework
- Reusability validation (3+ technologies)
- Right altitude balance (specific yet flexible)

**Output**: Actual skill or subagent files in `.claude/skills/` or `.claude/subagents/`

---

### Layer 4: Orchestration Style (Spec-Driven Mode)

**Purpose**: Design systems through specifications that compose accumulated intelligence at scale

**When to use**:
- Student has 3+ reusable components
- Project requires orchestration
- 10+ coordinated operations
- Specification capability validated

**Reasoning activation**: "What specification is sufficient? How do I compose intelligence? How do I validate spec↔implementation alignment?"

**File**: `pedagogical/layer-4-orchestration.md`

**Content characteristics**:
- Architectural voice (system design thinking)
- Specification framework (Intent, Constraints, Non-goals, Acceptance)
- Component composition mapping
- Orchestration patterns (sequential, concurrent, iterative)
- Validation protocols

**Output**: Complete specification + orchestrated implementation using existing components

---

## Style Selection Guide

**Use the decision tree**: `pedagogical/decision-tree.md`

### Quick Reference

```
First exposure? → Layer 1
Foundation solid + complex task? → Layer 2
Pattern recurring (2+) + complexity (5+)? → Layer 3
Components exist (3+) + orchestration needed? → Layer 4
```

### Common Patterns

**Single lesson progression**:
```
Section 1: Introduction → Layer 1 (foundation)
Section 2: Practice → Layer 1 (manual)
Section 3: AI Collaboration → Layer 2 (optimization)
Section 4: Pattern Recognition → Layer 3 (if reusable)
```

**Chapter progression** (8-9 lessons):
```
Lessons 1-2: Layer 1 (Manual foundation)
Lessons 3-5: Layer 2 (AI collaboration)
Lessons 6-7: Layer 2 + Layer 3 (Collaboration + Intelligence design)
Lesson 8: Layer 3 validation (Test components)
Lesson 9: Layer 4 (Spec-driven capstone)
```

---

## Complexity Tier Integration

**All layers scale with tier** (A1-A2 / B1-B2 / C1-C2):

| Tier | L1 Load | L2 Style | L3 Scope | L4 Detail |
|------|---------|----------|----------|-----------|
| **A2** (Aspiring) | ~5-7 concepts | Highly guided | Simple skills (2-3 decisions) | High detail |
| **B1** (Intermediate) | ~7-10 concepts | Moderately guided | Moderate skills (3-4 decisions) | Decision frameworks |
| **C2** (Advanced) | No limits | Peer collaboration | Complex subagents (5+ decisions) | Architectural intent |

**See each layer file for tier-specific scaffolding patterns**

---

## Structural Templates

**Location**: `structural/`

These provide the **mechanical structure** (file organization, YAML, formatting), while pedagogical styles provide the **reasoning frameworks**.

### Lesson Template

**File**: `structural/lesson-template.md`

**Provides**:
- YAML frontmatter structure
- File naming conventions
- Docusaurus configuration
- Section organization
- CoLearning element placement rules

**Use with**: Pedagogical layer styles determine CONTENT, template determines STRUCTURE

### Chapter/Part Templates

**Files**:
- `structural/chapter-readme-template.md`
- `structural/part-readme-template.md`

**Provides**:
- Introduction patterns
- "What You'll Learn" formatting
- Navigation structure
- Consistency standards

---

## Workflow Styles

**Location**: `workflows/`

### Super Orchestra Session

**File**: `workflows/super-orchestra.md`

**When to use**:
- Comprehensive intelligence gathering required
- Market-defining quality targeted
- Deep research + planning + iteration
- Meta-learning capture needed

**Characteristics**:
- Evidence-based communication
- Intelligence source documentation
- Iteration logging
- Positioning validation

---

## How to Use This Framework

### For Agents Creating Content

**Step 1: Recognize Layer**
1. Analyze task using decision tree (`pedagogical/decision-tree.md`)
2. Identify which layer(s) apply
3. Check tier from chapter-index.md (A2/B1/C2)

**Step 2: Apply Pedagogical Style**
1. Read appropriate layer file (`pedagogical/layer-N-*.md`)
2. Follow reasoning frameworks (not rigid templates)
3. Apply tier-appropriate scaffolding
4. Validate against layer-specific checklist

**Step 3: Apply Structural Template**
1. Use `structural/lesson-template.md` for file structure
2. Apply YAML frontmatter patterns
3. Follow Docusaurus conventions
4. Ensure CoLearning elements placed correctly

**Step 4: Validate**
1. Does content match layer reasoning patterns?
2. Are tier adjustments applied?
3. Does structure follow templates?
4. Are transition signals clear?

### For Humans Reviewing Content

**Layer validation questions**:
- **Layer 1**: Clear explanations? Manual practice? Self-validation criteria?
- **Layer 2**: Three Roles demonstrated? Convergence loops shown?
- **Layer 3**: Persona+Questions+Principles? Reusable across 3+ technologies?
- **Layer 4**: Specification complete? Components composed (not reinvented)?

**Quality signals**:
- Content activates reasoning (not prediction)
- Examples production-relevant (not toy apps)
- Teaching modality varied from previous chapter
- Cognitive load appropriate for tier

---

## Reasoning Activation vs Prediction Mode

### What Makes Styles "Reasoning-Activated"

**Prediction mode** (statistical pattern matching):
- Generic instructions: "Make it secure"
- High-frequency patterns: "Use HTTPS, sanitize inputs"
- Fast sampling from training data
- No context-specific analysis

**Reasoning mode** (context-specific analysis):
- Cognitive frameworks: "Think like security auditor analyzing attack surfaces"
- Structured inquiry: "What surfaces exist? What vectors apply? How prioritize?"
- Decision principles: "Defense in depth, fail secure, least privilege"
- Active problem-solving

**This framework's styles activate reasoning mode** through:
1. **Persona**: Establishes cognitive stance
2. **Questions**: Forces context analysis
3. **Principles**: Provides decision frameworks
4. **Anti-convergence**: Self-monitoring against generic patterns

---

## Research Foundation

**Based on**:
- `papers/compass_artifact_wf-411b5e9e-2fa8-4d2a-9086-5d63431afb98_text_markdown.md`
- Anthropic Skills blog post + Context Engineering guide
- Persona + Questions + Principles pattern (activates reasoning 83% better)
- Constitutional AI (Bai et al. 2022) - process-based training
- Chain-of-Thought (Wei et al. 2022) - reasoning activation

**Key insight**: LLMs default to distributional convergence (generic outputs). Strategic prompting activates reasoning by:
- Providing cognitive frameworks (not rigid instructions)
- Structuring analytical process (questions > commands)
- Enabling flexible application (principles > rules)

---

## Version History

### v2.0.0 (2025-01-17) - Reasoning-Activated Framework

**BREAKING CHANGE**: Complete redesign of output styles

**What changed**:
- Added 4 pedagogical layer styles (NEW)
- Created decision tree for style selection (NEW)
- Reorganized into pedagogical/ + structural/ + workflows/
- All styles use Persona+Questions+Principles pattern
- Tier-appropriate scaffolding integrated

**What's new**:
- Reasoning activation as core principle
- Layer-specific cognitive frameworks
- Three Roles framework (Layer 2)
- Intelligence design patterns (Layer 3)
- Spec-driven orchestration (Layer 4)

**Migration impact**:
- Agents must recognize layers before selecting style
- Content must match layer reasoning patterns
- Tier adjustments required for all layers

### v1.0.0 (Previous) - Structural Templates

**Original architecture**:
- Generic lesson template
- Chapter/part templates
- File organization guide
- Super orchestra workflow

**Limitation**: Templates specified WHAT to include but not HOW to activate reasoning in learners

---

## Integration Points

**This framework integrates with**:

- **Constitution** (`.specify/memory/constitution.md` v6.0.0)
  - Principles provide decision frameworks
  - 4-Stage Framework maps to 4 layers
  - Transition criteria defined

- **CLAUDE.md** (v4.0.0)
  - Layer recognition (Section I)
  - Cognitive architecture (Sections II-V)
  - Self-monitoring (Section X)

- **Skills Library** (`.claude/skills/`)
  - All skills redesigned with Persona+Questions+Principles
  - Skills support each layer's cognitive mode
  - Progressive disclosure architecture

---

## Success Criteria

**This framework succeeds when**:

**Agents**:
- ✅ Automatically recognize which layer applies
- ✅ Apply layer-appropriate reasoning frameworks
- ✅ Produce content that activates reasoning (not prediction)
- ✅ Self-monitor for convergence patterns

**Learners**:
- ✅ Build solid mental models (Layer 1)
- ✅ Collaborate effectively with AI (Layer 2)
- ✅ Design reusable intelligence (Layer 3)
- ✅ Orchestrate at scale through specifications (Layer 4)

**Content**:
- ✅ Zero specification violations (spec before code)
- ✅ Zero untested code (all examples validated)
- ✅ Zero hallucinations (all claims verified)
- ✅ 100% pedagogical structure (layers → tiers → transitions)

---

## Quick Start

**Creating new lesson content**:

1. **Identify layer**: Use `pedagogical/decision-tree.md`
2. **Read layer style**: Open appropriate `pedagogical/layer-N-*.md` file
3. **Apply tier adjustments**: Check chapter-index.md for A2/B1/C2
4. **Use structural template**: Follow `structural/lesson-template.md` for file format
5. **Validate**: Check layer-specific checklist

**Example workflow**:
```bash
# 1. Recognize: First exposure to Python functions → Layer 1
# 2. Read: pedagogical/layer-1-foundation.md
# 3. Tier: Chapter 15 = A2 tier (check chapter-index.md)
# 4. Apply: Layer 1 patterns with A2 scaffolding (5-7 concepts, heavy guidance)
# 5. Structure: Use structural/lesson-template.md for YAML + organization
# 6. Validate: Layer 1 checklist (explanations? walkthroughs? self-validation?)
```

---

## Support & Questions

**For style selection**: See `pedagogical/decision-tree.md`

**For layer details**: See individual layer files in `pedagogical/`

**For structure**: See templates in `structural/`

**For constitution alignment**: See `.specify/memory/constitution.md`

**For agent guidance**: See `CLAUDE.md` Sections I-V

---

**This framework transforms output styles from structural templates into reasoning activation tools that produce distinctive, pedagogically sound educational content aligned with the 4-layer teaching method.**

---

**Version**: 2.0.0
**Created**: 2025-01-17
**Authors**: Panaversity AI-Native Development Team
**License**: Proprietary
