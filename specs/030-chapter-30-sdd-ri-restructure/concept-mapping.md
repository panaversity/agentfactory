# Chapter 30 Concept Mapping: SDD-RI Cognitive Load Analysis

**Purpose**: Validate that B1 tier cognitive load limits are respected across all 8 lessons while ensuring complete SDD-RI methodology coverage.

**B1 Tier Constraints**:
- Max 7-10 concepts per section
- Max 1-2 new concepts per lesson (scaffolding rule)
- Moderate scaffolding required
- Progressive disclosure (simple → complex)

---

## Complete Concept Inventory (12 Total)

### SDD Concepts (6) — Lessons 1-5

1. **Vagueness Cost** (L1)
   - Definition: Quantifiable time, rework, and debugging overhead from unclear requirements
   - Why it matters: Specifications prevent vagueness-induced waste
   - Complexity: Low (concrete examples)

2. **Intent vs Implementation** (L1)
   - Definition: WHAT system does (intent) vs HOW it does it (implementation)
   - Why it matters: Specifications capture intent, code captures implementation
   - Complexity: Low (clear dichotomy)

3. **Spec.md Structure** (L2)
   - Definition: Intent, Requirements, Constraints, Non-Goals, Acceptance Tests sections
   - Why it matters: Structured format ensures completeness
   - Complexity: Medium (multiple components)

4. **Evals-First Principle** (L2)
   - Definition: Define success criteria before implementation
   - Why it matters: Cannot validate without predefined criteria
   - Complexity: Medium (requires foresight)

5. **AI as Spec Partner** (L3)
   - Definition: Iterative refinement through bidirectional learning (Three Roles)
   - Why it matters: AI collaboration produces better specs than solo work
   - Complexity: Medium (requires practice)

6. **Specification Primacy** (L4)
   - Definition: Code is OUTPUT of specification, not INPUT
   - Why it matters: Ensures implementation matches intent
   - Complexity: Medium (workflow sequence)

7. **Spec Quality Spectrum** (L5)
   - Definition: Over-specification (too prescriptive) ↔ Under-specification (too vague)
   - Why it matters: Quality evaluation framework
   - Complexity: Medium (judgment required)

8. **Tooling Tradeoffs** (L5)
   - Definition: Formal tools (SpecKit) vs lightweight templates (markdown)
   - Why it matters: Context-appropriate tooling choice
   - Complexity: Low (decision framework provided)

### RI Concepts (4) — Lessons 6-8

9. **Reusable Intelligence (RI)** (L6)
   - Definition: Skills, Subagents, Intelligence Libraries that encode recurring patterns
   - Why it matters: Specs become reusable across projects
   - Complexity: Medium (new abstraction layer)

10. **Encoding Decision Framework** (L6)
    - Definition: When to encode patterns (Frequency, Complexity, Organizational Value)
    - Why it matters: Not all patterns justify encoding cost
    - Complexity: Medium (requires pattern recognition)

11. **Persona + Questions + Principles (P+Q+P)** (L7)
    - Definition: Reasoning activation pattern (not prediction mode)
    - Why it matters: Produces context-specific outputs, not generic templates
    - Complexity: High (requires understanding reasoning vs prediction)

12. **Constitutions** (L8)
    - Definition: Organizational governance documents encoding principles (not rules)
    - Why it matters: Coordination at scale without micromanagement
    - Complexity: Medium (synthesis of all prior concepts)

---

## Concept-to-Lesson Mapping

### Lesson 1: Why Specifications Matter

**New Concepts**: 2
- Vagueness Cost
- Intent vs Implementation

**Cumulative Concepts**: 2

**Cognitive Load Analysis**:
- **Concept Density**: 2 concepts in 45-60 minutes
- **Complexity**: Both Low (concrete, relatable)
- **Scaffolding**: Heavy (real-world examples, case studies)
- **Practice Opportunity**: Identify vagueness in sample docs
- **B1 Compliance**: ✅ Within limit (2 concepts, simple)

**Dependencies**:
- **Prerequisites**: None (foundational lesson)
- **Enables**: L2 (understanding intent → understanding spec structure)

---

### Lesson 2: Anatomy of a Specification

**New Concepts**: 2
- Spec.md Structure
- Evals-First Principle

**Cumulative Concepts**: 4

**Cognitive Load Analysis**:
- **Concept Density**: 2 concepts in 60-75 minutes
- **Complexity**: Medium (structured format has multiple components)
- **Scaffolding**: Moderate (template provided, annotated example)
- **Practice Opportunity**: Write first manual spec
- **B1 Compliance**: ✅ Within limit (2 concepts, scaffolded)

**Dependencies**:
- **Prerequisites**: L1 (Intent vs Implementation)
- **Enables**: L3 (manual competence → AI collaboration)

**Chunking Strategy**:
- Spec.md Structure and Evals-First taught together (related concepts)
- Reduces load via natural grouping

---

### Lesson 3: Writing Specs with AI

**New Concepts**: 1
- AI as Spec Partner (Three Roles Framework)

**Cumulative Concepts**: 5

**Cognitive Load Analysis**:
- **Concept Density**: 1 concept in 60-75 minutes
- **Complexity**: Medium (requires understanding bidirectional learning)
- **Scaffolding**: Moderate (guided practice with prompts)
- **Practice Opportunity**: Collaborative spec refinement
- **B1 Compliance**: ✅ Within limit (1 concept, ample practice time)

**Dependencies**:
- **Prerequisites**: L2 (can write manual spec → can evaluate AI-generated spec)
- **Enables**: L4 (AI collaboration → spec-to-code workflow)

**Constitutional Requirement**:
- MUST demonstrate Three Roles (AI as Teacher/Student/Co-Worker)
- Content validated against constitution Section IIa Stage 2

---

### Lesson 4: From Spec to Code

**New Concepts**: 1
- Specification Primacy (workflow sequence)

**Cumulative Concepts**: 6

**Cognitive Load Analysis**:
- **Concept Density**: 1 concept in 60-75 minutes
- **Complexity**: Medium (workflow execution)
- **Scaffolding**: Moderate (step-by-step workflow guidance)
- **Practice Opportunity**: Generate code from spec, validate alignment
- **B1 Compliance**: ✅ Within limit (1 concept, practical application)

**Dependencies**:
- **Prerequisites**: L3 (has refined spec → can use it for code generation)
- **Enables**: L5 (spec-to-code experience → quality evaluation)

**Workflow Taught**:
1. spec.md (from L3) → AI prompt → code generation → acceptance test → validation loop

---

### Lesson 5: Spec Quality & Tooling

**New Concepts**: 2
- Spec Quality Spectrum
- Tooling Tradeoffs

**Cumulative Concepts**: 8

**Cognitive Load Analysis**:
- **Concept Density**: 2 concepts in 60-75 minutes
- **Complexity**: Low-Medium (evaluation framework + decision tree)
- **Scaffolding**: Moderate (comparison table, decision criteria)
- **Practice Opportunity**: Evaluate sample specs, choose tooling
- **B1 Compliance**: ✅ Within limit (2 concepts, decision frameworks provided)

**Dependencies**:
- **Prerequisites**: L1-4 (complete SDD experience)
- **Enables**: L6 (mastery of single specs → reusability patterns)

**Transition Setup**:
- Lesson ending: "You can write great specs. Next: making them reusable across projects."
- Forward reference to RI concepts (Lesson 6)

**Chunking Strategy**:
- Quality and Tooling taught together (both are evaluation/selection skills)

---

### Lesson 6: Introduction to Reusable Intelligence

**New Concepts**: 2
- Reusable Intelligence (RI)
- Encoding Decision Framework

**Cumulative Concepts**: 10

**Cognitive Load Analysis**:
- **Concept Density**: 2 concepts in 60-75 minutes
- **Complexity**: Medium (new abstraction layer)
- **Scaffolding**: Moderate (decision framework, real examples from project)
- **Practice Opportunity**: Identify RI opportunities in student's work
- **B1 Compliance**: ✅ Within limit (2 concepts, builds on L1-5 foundation)

**Dependencies**:
- **Prerequisites**: L1-5 (SDD mastery → can recognize reusability opportunities)
- **Enables**: L7 (RI concept → RI implementation)

**Pedagogical Milestone**:
- **"Aha Moment"**: Reveals full SDD-RI methodology
- Late reveal strategy (Option 2b) executed here

**Examples Shown**:
- Real Skill from `.claude/skills/` directory
- Real Subagent from `.claude/agents/` directory
- Demonstrates Skills vs Subagents distinction (2-4 decisions vs 5+)

**Chunking Strategy**:
- RI definition and Encoding decision taught together (concept + application)

---

### Lesson 7: Designing Skills and Subagents

**New Concepts**: 1
- Persona + Questions + Principles (P+Q+P)

**Cumulative Concepts**: 11

**Cognitive Load Analysis**:
- **Concept Density**: 1 concept in 75-90 minutes
- **Complexity**: High (most complex concept in chapter)
- **Scaffolding**: Heavy (workshop format, step-by-step guidance)
- **Practice Opportunity**: Create complete Skill document
- **B1 Compliance**: ✅ Within limit (1 concept, extended time, heavy scaffolding)

**Dependencies**:
- **Prerequisites**: L6 (RI concept → RI implementation)
- **Enables**: L8 (P+Q+P understanding → organizational patterns)

**Workshop Structure** (manages high complexity):
1. **Theory** (15 min): Reasoning vs Prediction modes
2. **Pattern Breakdown** (20 min): P+Q+P components explained
3. **Guided Practice** (30 min): Write Persona, Questions, Principles
4. **Validation** (15 min): Test skill, verify reasoning activation

**Constitutional Reference**:
- Teaches pattern from constitution.md Section 0 (Persona + Questions + Principles)
- Meta-pedagogical: Teaching the teaching framework

**Load Mitigation**:
- Single concept (P+Q+P) but highest complexity
- Extended time (75-90 min vs 60-75 typical)
- Workshop format provides scaffolding
- Hands-on creation cements understanding

---

### Lesson 8: Organizational Patterns & Governance

**New Concepts**: 1
- Constitutions (governance documents)

**Cumulative Concepts**: 12

**Cognitive Load Analysis**:
- **Concept Density**: 1 concept in 60-75 minutes
- **Complexity**: Medium (synthesis concept)
- **Scaffolding**: Moderate (case study analysis)
- **Practice Opportunity**: Design simple constitution
- **B1 Compliance**: ✅ Within limit (1 concept, synthesis of prior learning)

**Dependencies**:
- **Prerequisites**: L1-7 (complete SDD-RI understanding)
- **Enables**: Part 6 (organizational patterns → agent building)

**Synthesis Nature**:
- Constitutions apply ALL prior concepts at organizational scale
- Shows how Principles (L1), Specs (L2-5), Skills (L6-7) compose into governance

**Case Study**:
- This project's `.specify/memory/constitution.md` as working example
- Students see real constitution that governed this book's creation

**Forward References**:
- Chapter 31: SpecKit Plus hands-on
- Chapter 32: AI Orchestration (agent teams)
- Chapter 33: Tessl vision (spec-as-source)
- Part 6: AI Native Development (agent building)

---

## Progressive Disclosure Validation

### Concept Complexity Progression

| Lesson | Concepts | Complexity Level | Scaffolding | Time (min) |
|--------|----------|------------------|-------------|------------|
| L1 | 2 | Low | Heavy | 45-60 |
| L2 | 2 | Medium | Moderate | 60-75 |
| L3 | 1 | Medium | Moderate | 60-75 |
| L4 | 1 | Medium | Moderate | 60-75 |
| L5 | 2 | Low-Med | Moderate | 60-75 |
| L6 | 2 | Medium | Moderate | 60-75 |
| L7 | 1 | High | Heavy | 75-90 |
| L8 | 1 | Medium | Moderate | 60-75 |

**Progressive Disclosure Check**:
- ✅ Early lessons (L1-2): Low-Medium complexity, Heavy scaffolding
- ✅ Middle lessons (L3-6): Medium complexity, Moderate scaffolding
- ✅ Late lessons (L7-8): High complexity (L7) with Heavy scaffolding, synthesis (L8)

**Complexity managed via**:
- Scaffolding increases for complex concepts (L7)
- Time extends for complex concepts (L7: 75-90 min)
- Synthesis concepts come last (L8 after all components taught)

---

## Chunking Strategy Analysis

### Related Concepts Grouped Together

**Chunk 1 (L1)**: Problem → Principle
- Vagueness Cost (problem) + Intent vs Implementation (principle)
- Natural pairing: problem statement + solution framework

**Chunk 2 (L2)**: Structure → Evaluation
- Spec.md Structure (what) + Evals-First (how to validate)
- Natural pairing: format + quality criteria

**Chunk 3 (L3)**: Collaboration Pattern
- AI as Spec Partner (single concept, ample practice time)
- Standalone: Complex enough to warrant full lesson

**Chunk 4 (L4)**: Workflow Execution
- Specification Primacy (single concept, practical application)
- Standalone: Workflow execution needs practice time

**Chunk 5 (L5)**: Evaluation → Selection
- Spec Quality Spectrum (evaluate) + Tooling Tradeoffs (select)
- Natural pairing: both are meta-skills about choosing approaches

**Chunk 6 (L6)**: Concept → Application
- Reusable Intelligence (what) + Encoding Decision (when)
- Natural pairing: definition + decision framework

**Chunk 7 (L7)**: Implementation Pattern
- P+Q+P (single concept, high complexity)
- Standalone: Needs heavy scaffolding and extended time

**Chunk 8 (L8)**: Organizational Synthesis
- Constitutions (single concept, synthesizes all prior)
- Standalone: Capstone synthesis concept

**Chunking Effectiveness**: ✅ Related concepts naturally grouped, reduces cognitive load

---

## Cognitive Load Risk Analysis

### High-Risk Lessons (Potential Overload)

**Lesson 5: Spec Quality & Tooling** (2 concepts, Medium complexity)
- **Risk**: Quality evaluation + tooling selection both require judgment
- **Mitigation**: Decision frameworks provided for both, comparison tables reduce load
- **Outcome**: Low risk (frameworks scaffold decision-making)

**Lesson 6: Introduction to RI** (2 concepts, Medium complexity, new abstraction)
- **Risk**: RI is new paradigm (reusable specs), Encoding decision requires pattern recognition
- **Mitigation**: Real examples from project, builds on L1-5 foundation (specs mastered)
- **Outcome**: Medium risk (monitor student understanding, add examples if needed)

**Lesson 7: P+Q+P Pattern** (1 concept, High complexity)
- **Risk**: Most complex concept in chapter (reasoning vs prediction modes)
- **Mitigation**: Extended time (75-90 min), heavy scaffolding (workshop), hands-on practice
- **Outcome**: Medium risk (complexity justified by importance, scaffolding reduces risk)

### Load Distribution Over Time

**Total Chapter Concepts**: 12
**Total Chapter Time**: ~525 minutes (8.75 hours)
**Average Concept Density**: 1.5 concepts per lesson
**Average Time per Concept**: ~44 minutes

**Load Spread**:
- **Early (L1-2)**: 4 concepts in 120 min = 30 min/concept (high time → low load)
- **Middle (L3-6)**: 6 concepts in 270 min = 45 min/concept (moderate)
- **Late (L7-8)**: 2 concepts in 135 min = 67 min/concept (low density → deep practice)

**Progressive Load**: ✅ Early lessons have more time per concept (scaffolding), late lessons focus on depth

---

## B1 Tier Compliance Checklist

### Per-Lesson Validation

- ✅ **L1**: 2 concepts, Low complexity, Heavy scaffolding = **PASS**
- ✅ **L2**: 2 concepts, Medium complexity, Moderate scaffolding = **PASS**
- ✅ **L3**: 1 concept, Medium complexity, Moderate scaffolding = **PASS**
- ✅ **L4**: 1 concept, Medium complexity, Moderate scaffolding = **PASS**
- ✅ **L5**: 2 concepts, Low-Med complexity, Decision frameworks = **PASS**
- ✅ **L6**: 2 concepts, Medium complexity, Real examples = **PASS**
- ✅ **L7**: 1 concept, High complexity, Heavy scaffolding + extended time = **PASS**
- ✅ **L8**: 1 concept, Medium complexity, Synthesis (builds on prior) = **PASS**

### Chapter-Level Validation

- ✅ **Total Concepts**: 12 concepts across 8 lessons = 1.5 avg per lesson (within B1 limit)
- ✅ **Max Concepts Per Lesson**: 2 (within B1 limit of 1-2 per lesson)
- ✅ **Progressive Disclosure**: Complexity increases gradually with scaffolding
- ✅ **Chunking Applied**: Related concepts grouped naturally
- ✅ **Practice Opportunities**: Every lesson includes hands-on practice
- ✅ **Scaffolding Calibration**: Heavy (early) → Moderate (middle) → Heavy (L7 complex) → Moderate (L8 synthesis)

**Overall B1 Compliance**: ✅ **PASS** — All cognitive load limits respected

---

## Prerequisite Chain Validation

### Internal Prerequisites (Within Chapter 30)

**L1 → L2**: Intent vs Implementation → Spec Structure
- Student must understand intent before learning to structure specs ✅

**L2 → L3**: Manual spec writing → AI collaboration
- Student must have manual competence before AI collaboration (Stage 1 → 2) ✅

**L3 → L4**: AI-refined spec → Spec-to-code workflow
- Student must have refined spec to use for code generation ✅

**L4 → L5**: Spec-to-code experience → Quality evaluation
- Student must have executed workflow before evaluating quality ✅

**L5 → L6**: SDD mastery → RI introduction
- Student must recognize recurring patterns (requires 2+ specs written) ✅

**L6 → L7**: RI concept → RI implementation
- Student must understand Skills/Subagents before creating them ✅

**L7 → L8**: P+Q+P pattern → Organizational patterns
- Student must understand reasoning activation before governance (Constitutions use P+Q+P) ✅

**Dependency Chain**: ✅ No circular dependencies, clear linear progression

### External Prerequisites (From Prior Chapters)

**From Parts 1-3**:
- AI tool literacy (prompting, context engineering) → Enables L3 (AI collaboration)
- Markdown proficiency → Enables L2 (spec.md writing)
- Git/GitHub workflows → Enables L4 (code validation)

**From Part 4 (Python)**:
- Programming fundamentals → Enables L4 (code generation validation)
- Experience with AI code assistance → Enables L3 (AI as spec partner)

**All Prerequisites Satisfied**: ✅ Students completing Parts 1-4 have necessary foundation

### Outgoing Prerequisites (For Part 6+)

**Chapter 30 Prepares Students For**:
- Chapter 34 (AI Agents): L6-7 (Skills/Subagents concepts)
- Chapter 42 (Test-Driven Development): L2 (Evals-first principle)
- Chapter 44 (Agent Design Patterns): L7 (P+Q+P reasoning activation)
- Chapter 61 (Governance): L8 (Constitutions pattern)

**Prerequisite Fulfillment**: ✅ All Part 6+ dependencies satisfied by Ch 30

---

## Concept Coverage Completeness

### SDD Methodology (Foundation)

- ✅ **Why specs matter**: L1 (Vagueness cost)
- ✅ **What is a spec**: L2 (Structure, Evals-first)
- ✅ **How to write specs**: L3 (AI collaboration)
- ✅ **How to use specs**: L4 (Spec-to-code workflow)
- ✅ **How to evaluate specs**: L5 (Quality, Tooling)

**SDD Coverage**: ✅ Complete (all foundational concepts taught)

### RI Methodology (Extension)

- ✅ **What is RI**: L6 (Skills, Subagents, Libraries)
- ✅ **When to encode**: L6 (Decision framework)
- ✅ **How to design RI**: L7 (P+Q+P pattern)
- ✅ **Organizational scale**: L8 (Constitutions)

**RI Coverage**: ✅ Complete (all extension concepts taught)

### SDD-RI Integration

- ✅ **Sequential dependency**: SDD (L1-5) taught before RI (L6-8)
- ✅ **Conceptual unity**: Presented as unified methodology (not separate)
- ✅ **Transition clarity**: L5 → L6 transition explicit ("making specs reusable")
- ✅ **Synthesis**: L8 shows organizational composition of all concepts

**Methodology Coverage**: ✅ Complete and integrated

---

## Success Metrics

### Cognitive Load Management
- ✅ No lesson exceeds 2 concepts (B1 limit respected)
- ✅ Average 1.5 concepts per lesson (optimal pacing)
- ✅ Complex concepts get extended time (L7: 75-90 min)
- ✅ Scaffolding calibrated to complexity (Heavy for L1, L7)

### Pedagogical Progression
- ✅ Foundation → Application → Integration → Validation arc clear
- ✅ Stage 1 (Manual) → 2 (AI Collab) → 3 (Intelligence Design) mapped
- ✅ Progressive disclosure (simple → complex with scaffolding)
- ✅ No cognitive overload at any point

### Prerequisite Satisfaction
- ✅ All internal dependencies linear (no circular)
- ✅ All external prerequisites satisfied by Parts 1-4
- ✅ All Part 6+ prerequisites provided by Ch 30

### Concept Coverage
- ✅ SDD methodology complete (6 concepts)
- ✅ RI extension complete (4 concepts)
- ✅ SDD-RI integration explicit (12 total concepts)

---

## Risk Mitigation Summary

**Identified Risks**:
1. L6 (RI intro): New abstraction layer
2. L7 (P+Q+P): Highest complexity concept

**Mitigations Applied**:
1. L6: Real examples from project, builds on mastered SDD foundation
2. L7: Extended time (75-90 min), workshop format, heavy scaffolding

**Residual Risk**: **Low** — All high-risk lessons have appropriate mitigations

---

**This concept mapping validates that the 8-lesson expansion respects B1 cognitive load limits while providing complete SDD-RI methodology coverage. Progressive disclosure, chunking, and scaffolding strategies ensure learning effectiveness without overwhelming students.**
