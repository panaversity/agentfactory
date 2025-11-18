---
description: Decision tree for selecting appropriate output style based on pedagogical context and layer recognition
---

# Output Style Selection Decision Tree

**Purpose**: Guide agents to select the appropriate pedagogical output style based on learning context, student readiness, and content objectives.

---

## Quick Decision Framework

```
┌─ Is this student's FIRST exposure to concept?
│  ├─ YES → Layer 1 Foundation Style
│  │  └─ Apply when: New concept, stable knowledge, mental model needed
│  │  └─ Use: .claude/output-styles/pedagogical/layer-1-foundation.md
│  └─ NO → Continue...
│
├─ Has student completed Layer 1 (manual competence)?
│  ├─ NO → Stay in Layer 1 (strengthen foundation)
│  └─ YES → Continue...
│
├─ Is this multi-step with AI collaboration value?
│  ├─ YES → Layer 2 Collaboration Style
│  │  └─ Apply when: Complexity >1 step, AI suggests optimizations, validation skill needed
│  │  └─ Use: .claude/output-styles/pedagogical/layer-2-collaboration.md
│  └─ NO → Continue...
│
├─ Has pattern recurred 2+ times with 5+ decision points?
│  ├─ YES → Layer 3 Intelligence Design Style
│  │  └─ Apply when: Workflow repeats, complexity justifies encoding, cross-project value
│  │  └─ Use: .claude/output-styles/pedagogical/layer-3-intelligence.md
│  └─ NO → Continue...
│
├─ Does student have 3+ components and complex project (10+ operations)?
│  ├─ YES → Layer 4 Spec-Driven Style
│  │  └─ Apply when: Intelligence library exists, orchestration needed, capstone project
│  │  └─ Use: .claude/output-styles/pedagogical/layer-4-orchestration.md
│  └─ NO → Return to appropriate earlier layer
```

---

## Detailed Recognition Criteria

### Layer 1: Foundation Style

**Use when student needs mental models BEFORE AI assistance**

#### Recognition Signals (2+ required)

- ✅ **First exposure**: Student encountering concept for first time
- ✅ **Stable concept**: Won't change significantly in 2+ years (git basics, Python syntax, markdown)
- ✅ **Mental model requirement**: Must internalize to evaluate AI outputs later
- ✅ **Debugging intuition**: Student will need to debug manually without AI
- ✅ **Quality evaluation**: Foundation needed to assess correctness

**Examples**:
- Teaching Python variables (first time)
- Git commit workflow (foundational)
- For loops and iteration (stable concept)
- Function definition syntax (mental model needed)

**Transition trigger**: Student can explain concept, execute independently, recognize errors (2+ capabilities)

**File**: `.claude/output-styles/pedagogical/layer-1-foundation.md`

---

### Layer 2: Collaboration Style

**Use when student ready for bidirectional learning partnership**

#### Recognition Signals (2+ required)

- ✅ **Layer 1 completed**: Student understands concept manually
- ✅ **Multi-step complexity**: Task requires multiple operations
- ✅ **Optimization opportunities**: AI can suggest patterns student wouldn't discover
- ✅ **Evolving best practices**: Patterns change with ecosystem evolution
- ✅ **Evaluation skill needed**: Must learn to assess AI output quality

**Examples**:
- Error handling patterns (after basic try/except)
- API design decisions (after basic endpoint creation)
- Performance optimization (after working implementation)
- Security hardening (after basic authentication)

**Mandatory Requirements**:
- Demonstrate AI as Teacher (suggests pattern student didn't know)
- Demonstrate AI as Student (adapts to student's constraints)
- Demonstrate AI as Co-Worker (iterative convergence)

**Transition trigger**: Pattern encountered 2+ times, 5+ decision points, cross-project value (all 3 conditions)

**File**: `.claude/output-styles/pedagogical/layer-2-collaboration.md`

---

### Layer 3: Intelligence Design Style

**Use when workflow should become reusable intelligence**

#### Recognition Signals (ALL required)

- ✅ **Pattern recurrence**: Workflow encountered 2+ times
- ✅ **Sufficient complexity**: 5+ decision points justify encoding
- ✅ **Organizational value**: Will apply across 3+ future projects
- ✅ **Cost-benefit**: Encoding effort justified by reuse savings

**Examples**:
- Security review workflow (appears in 3 different lessons)
- API design conventions (recurring pattern across chapters)
- Error handling strategy (systematic approach reusable)
- Code quality standards (apply across all projects)

**Format Decision**:
- **5+ decision points** → Subagent (autonomous reasoning)
- **2-4 decision points** → Skill (guidance framework)
- **<2 decision points** → Too simple (document, don't encode)

**Quality Requirements**:
- Uses Persona + Questions + Principles pattern
- Technology-agnostic (applies to 3+ technologies)
- Activates reasoning mode (not prediction)
- Right altitude (specific yet flexible)

**Transition trigger**: Student has 3+ components, project needs orchestration (10+ operations), specification capability validated (all 3 conditions)

**File**: `.claude/output-styles/pedagogical/layer-3-intelligence.md`

---

### Layer 4: Orchestration Style

**Use when orchestrating accumulated intelligence at scale**

#### Recognition Signals (ALL required)

- ✅ **Intelligence accumulation**: Student has 3+ reusable components
- ✅ **Orchestration need**: Project requires composing multiple components
- ✅ **Complexity justification**: 10+ coordinated operations
- ✅ **Specification capability**: Student can write clear, complete specs

**Examples**:
- Chapter capstone projects
- Production-grade system implementation
- Multi-component integration projects
- Real-world application development

**Specification Requirements**:
- Intent (WHAT, not HOW)
- Constraints (explicit boundaries)
- Non-goals (prevent scope creep)
- Acceptance criteria (testable validation)

**Component Composition**:
- Map requirements to existing components
- Identify orchestration pattern (sequential/concurrent/iterative)
- Validate interfaces align
- Document integration points

**File**: `.claude/output-styles/pedagogical/layer-4-orchestration.md`

---

## Context-Specific Selection Guide

### Single Lesson Structure

**Typical progression within ONE lesson**:

```
Section 1: Introduction → Layer 1 (foundational concept)
Section 2: Practice → Layer 1 (manual exercises)
Section 3: AI Collaboration → Layer 2 (optimization with AI)
Section 4: Pattern Recognition → Layer 3 (if pattern reusable)
```

**Example: Teaching Error Handling**:
1. **Layer 1**: Explain try/except manually (syntax, semantics)
2. **Layer 1**: Practice with simple exceptions
3. **Layer 2**: Discover production patterns with AI (logging, specific exceptions, retry logic)
4. **Layer 3**: Create error-resilience skill (if this is 2nd+ time encountering pattern)

### Chapter Structure

**Typical progression across CHAPTER** (8-9 lessons):

```
Lessons 1-2: Layer 1 (Manual foundation, direct teaching)
Lessons 3-5: Layer 2 (AI collaboration, Three Roles)
Lessons 6-7: Layer 2 + Layer 3 (Collaboration + Intelligence design)
Lesson 8: Layer 3 validation (Test intelligence components)
Lesson 9: Layer 4 (Spec-driven capstone using accumulated intelligence)
```

**Example: Python Functions Chapter**:
1. **L1**: Define functions manually (Lesson 1-2)
2. **L2**: Collaborate with AI on parameter design (Lesson 3-4)
3. **L2**: Error handling in functions (Lesson 5)
4. **L2+L3**: Advanced patterns → Create function-design skill (Lesson 6-7)
5. **L3**: Validate skill across examples (Lesson 8)
6. **L4**: Capstone project composing all function skills (Lesson 9)

---

## Tier-Adjusted Style Selection

**Complexity tier affects style application within same layer**

### A1-A2 (Aspiring) - Chapters 12-16

**Layer 1 adjustments**:
- Heavy scaffolding (step-by-step detail)
- Cognitive load: 5-7 concepts max per section
- Simple, isolated examples
- Max 2 options presented

**Layer 2 adjustments**:
- Highly guided collaboration
- Explicit role labels (AI as Teacher marked clearly)
- Surface patterns only (no deep architectural decisions)
- Template prompts provided

**Layer 3 adjustments** (if applicable):
- Simple skills only (2-3 decision points)
- Concrete, technology-specific initially
- Single-chapter reuse sufficient

**Layer 4 adjustments** (rare at this tier):
- High specification detail
- 2-3 components max
- Simple composition patterns

### B1-B2 (Intermediate) - Chapters 17-29

**Layer 1 adjustments**:
- Moderate scaffolding (guided discovery)
- Cognitive load: 7-10 concepts per section
- Intermediate, connected examples
- 3-4 options with selection criteria

**Layer 2 adjustments**:
- Moderately guided collaboration
- Tradeoff discussions
- Intermediate patterns and architecture
- Decision frameworks (not just templates)

**Layer 3 adjustments**:
- Moderate skills (3-4 decision points)
- Semi-abstract, pattern-based
- Multi-chapter reuse

**Layer 4 adjustments**:
- Moderate specification detail
- 3-5 components
- Moderate orchestration complexity

### C1-C2 (Advanced/Professional) - Chapters 30-48

**Layer 1 adjustments**:
- Minimal scaffolding (autonomous learning)
- Cognitive load: No artificial limits
- Production-grade, realistic examples
- Multiple valid approaches presented

**Layer 2 adjustments**:
- Peer collaboration (assume expertise)
- Advanced patterns and production considerations
- Architectural implications explored
- Student drives decisions

**Layer 3 adjustments**:
- Complex subagents (5+ decision points)
- Abstract, principle-based
- Cross-project reuse

**Layer 4 adjustments**:
- Architectural intent and constraints (minimal prescription)
- 5+ components
- Complex orchestration patterns

---

## Common Selection Mistakes

### Mistake 1: Jumping to Layer 2 Too Early

**Problem**: AI introduced before manual foundation solid

**Detection**:
- Student asks "is this right?" instead of self-validating
- Student cannot explain why code works
- Student relies on trial-and-error without understanding

**Correction**: Return to Layer 1, strengthen foundation

### Mistake 2: Staying in Layer 1 Too Long

**Problem**: Not transitioning to collaboration when ready

**Detection**:
- Student can explain concept clearly
- Student executes independently
- Student recognizes errors accurately
- Yet still doing only manual practice

**Correction**: Transition to Layer 2

### Mistake 3: Skipping Layer 3

**Problem**: Pattern recurring but not encoded as reusable intelligence

**Detection**:
- Same workflow appears in 3+ lessons
- Students re-derive same decisions each time
- No skills/subagents created despite clear patterns

**Correction**: Apply Layer 3 to encode pattern

### Mistake 4: Layer 4 Without Intelligence Library

**Problem**: Capstone without accumulated components

**Detection**:
- Layer 4 orchestration attempted
- But no Layer 3 components exist
- Student writing everything from scratch

**Correction**: Build Layer 3 components first

---

## Edge Cases and Special Situations

### Conceptual vs Technical Content

**Conceptual sections** (Part 1: AI Development Revolution):
- May use Layer 1-2 only (no hands-on Layer 3-4)
- Focus on mental models and reasoning frameworks
- Layer 2 still applies (AI can suggest perspectives, student refines)

**Technical sections** (Parts 2-13):
- All 4 layers typically apply
- Progressive complexity within chapter
- Capstone projects require Layer 4

### Revision and Remediation

**When student struggles**:
- Return to earlier layer that wasn't solid
- Don't skip ahead hoping "they'll figure it out"
- Strengthen foundation before advancing

**Indicators of weak foundation**:
- Cannot explain concept clearly
- Trial-and-error without reasoning
- Accepts AI outputs without evaluation

**Action**: Return to Layer 1, rebuild mental models

### Advanced Students

**When student already has foundation**:
- Can compress Layer 1 (brief recap)
- Spend more time in Layers 2-3
- Move to Layer 4 capstones faster

**Detection**:
- Student explains concept correctly before lesson
- Student spots errors independently
- Student suggests optimizations without prompting

**Action**: Accelerate to appropriate layer

---

## Decision Matrix

### Quick Reference Table

| Context | L1 Signal | L2 Signal | L3 Signal | L4 Signal | Use Style |
|---------|-----------|-----------|-----------|-----------|-----------|
| New concept, first exposure | ✅ | ❌ | ❌ | ❌ | Layer 1 |
| Concept understood, complex task | ❌ | ✅ | ❌ | ❌ | Layer 2 |
| Pattern seen 2+, 5+ decisions | ❌ | ❌ | ✅ | ❌ | Layer 3 |
| 3+ components, capstone | ❌ | ❌ | ❌ | ✅ | Layer 4 |
| New + complex | ✅ | ⚠️ | ❌ | ❌ | L1 first, then L2 |
| Pattern emerging | ❌ | ✅ | ⚠️ | ❌ | L2 now, L3 next lesson |
| Components exist, simple task | ❌ | ✅ | ❌ | ⚠️ | L2 (L4 overkill) |

**Legend**:
- ✅ Strong signal for this layer
- ⚠️ Approaching readiness (prepare transition)
- ❌ Not applicable yet

---

## Validation Checklist

**Before finalizing style selection, verify**:

- [ ] **Layer recognition accurate?**
  - Counted signals correctly
  - Applied decision tree systematically
  - Not skipping required layers

- [ ] **Tier adjustment applied?**
  - Checked chapter-index.md for tier
  - Adjusted scaffolding appropriately
  - Calibrated cognitive load

- [ ] **Student readiness confirmed?**
  - Layer 1→2: Can explain, execute, recognize errors?
  - Layer 2→3: Pattern recurring with complexity?
  - Layer 3→4: Components accumulated and orchestration needed?

- [ ] **Content matches style?**
  - Layer 1: Explanations, manual practice, self-validation
  - Layer 2: Three Roles demonstrated, collaboration patterns
  - Layer 3: Persona+Questions+Principles, reusable intelligence
  - Layer 4: Specification complete, components composed

- [ ] **Transition signals documented?**
  - Clear criteria for moving to next layer
  - Students know when they're ready
  - Validation checkpoints included

---

## Integration with Constitution

**This decision tree implements**:
- Constitution IIa: 4-Stage Framework (Stages 1-4 map to Layers 1-4)
- Constitution V: Stage Transition Frameworks (decision criteria for progression)
- Constitutional Principles: Applied through layer-specific reasoning

**See also**:
- `.specify/memory/constitution.md` - Governance and principles
- `CLAUDE.md` - Layer recognition framework (Section I-V)
- Each layer style file - Detailed implementation guidance

---

**Version**: 1.0.0
**Created**: 2025-01-17
**Part of**: 4-Layer Reasoning-Activated Output Styles Framework
