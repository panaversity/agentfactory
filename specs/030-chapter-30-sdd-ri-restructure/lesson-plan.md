# Chapter 30 Lesson Plan: SDD-RI Unified Methodology (8 Lessons)

**Chapter**: 30 — Understanding Spec-Driven Development with Reusable Intelligence
**Part**: 5 — Spec-Driven Development
**Audience Tier**: B1 (Intermediate)
**Total Lessons**: 8 (expanded from 5)
**Pedagogical Pattern**: Practical SDD-RI Learning Through Examples
**Approach**: Students write specs, see real examples, practice workflows — but DON'T use frameworks yet (Chapter 31+)

---

## Pedagogical Architecture

### Overall Chapter Arc

**Foundation (L1-5)**: SDD Fundamentals — Practical specification skills
- L1: Vague code problem identification
- L2: Historical context (Why specs matter NOW)
- L3: Spec anatomy (structure and quality awareness)
- L4: Collaborative spec writing with AI
- L5: Shared governance rules (Constitutions)

**Integration (L6-8)**: RI Extension — Reusable Intelligence thinking
- L6-7: Introduction to Skills/Subagents design
- L8: Framework landscape awareness (bridge to Chapter 31)

### What Chapter 30 IS

✅ **Practical learning**: Students write real specs, see real examples, practice real workflows
✅ **Hands-on exercises**: Build calculator spec, create constitutions, identify RI patterns
✅ **Grounded in examples**: Every concept demonstrated with concrete artifacts
✅ **Foundation building**: Master SDD-RI thinking before using frameworks

### What Chapter 30 IS NOT

❌ **Framework implementation**: NOT using Spec-Kit/tools to generate code from specs (Chapter 31+)
❌ **Code generation focus**: NOT "spec → code → deploy" workflow (implementation phase)
❌ **Purely conceptual**: NOT just theory without practice

### Cognitive Load Distribution

**B1 Tier Limit**: 7-10 concepts per section, 1-2 concepts per lesson

**Total Chapter Concepts**: 14 concepts
- **SDD Concepts (L1-5)**: 8 concepts
- **RI Concepts (L6-7)**: 3 concepts
- **Framework Awareness (L8)**: 3 concepts

**Load per lesson**: 1.75 concepts average (well within B1 capacity)

### Stage Progression Mapping

- **Stage 1 (Manual)**: L1-3 (build mental model of specifications)
- **Stage 2 (AI Collaboration)**: L4-5 (write specs with AI, governance)
- **Stage 3 (Intelligence Design)**: L6-7 (create reusable components)
- **Stage 3→4 Bridge**: L8 (framework awareness, prepare for implementation)

---

## Lesson-by-Lesson Structure

### Lesson 1: Why Specifications Matter

**Stage**: 1 (Manual Foundation)
**Teaching Modality**: Problem-based discovery (show cost of vagueness)
**Duration**: 45-60 minutes

**Learning Objectives**:
- Diagnose vagueness in requirements and quantify real costs (time, rework, debugging)
- Articulate the difference between intent and implementation
- Explain why AI agents need specifications (not just prompts)

**Concepts Introduced** (2):
1. **Vagueness Cost**: Time waste, rework cycles, technical debt from unclear requirements
2. **Intent vs Implementation**: WHAT the system does vs HOW it does it

**Content Structure**:
- **Problem**: Real-world case study of project failure due to vague requirements
- **Analysis**: Break down where vagueness entered, how it propagated
- **Principle**: Specifications capture intent before implementation
- **Practice**: Identify vagueness in sample requirement documents

**AI Role**: Minimal (student practices manual analysis)

**Cognitive Load**: 2 concepts, simple examples, heavy scaffolding = **Low (A2-B1)**

**Prerequisites**:
- Completed Part 4 (Python Fundamentals)
- Understands basic software development concepts
- Has experienced working with AI coding assistants (from Parts 1-3)

**Outputs**:
- Student can identify vague vs precise requirements
- Student understands why "just prompt the AI" fails for complex projects

---

### Lesson 2: Why Do Specs Matter NOW? The AI Moment

**Stage**: 1 (Manual Foundation)
**Teaching Modality**: Historical context + motivational analysis
**Duration**: 60-75 minutes
**Status**: 🔄 NEEDS COMPRESSION (from existing L5, currently 2.5-3 hours)

**Learning Objectives**:
- Explain why SDD emerged NOW in 2025 (not in 1970s, 2000s, or 2010s)
- Connect AI literal-mindedness to specification necessity
- Understand cost-benefit shift: specs save time with AI agents

**Concepts Introduced** (3):
1. **Three Convergent Forces**: AI capability + iteration savings + AI literal-mindedness
2. **Historical Context**: Why previous spec approaches failed (Formal Methods, MDD, Agile backlash)
3. **AI Necessity**: AI agents need specs (humans can improvise, AI cannot)

**Content Structure**:
- **Part 1 (15 min)**: The Question
  - "Why is SDD standard NOW, not 20 years ago?"
  - Three convergent forces explanation

- **Part 2 (20 min)**: Historical Timeline (COMPRESSED)
  - 1970s Formal Methods: Too rigid (PhD-level math required)
  - 2000s Model-Driven Development: Tool lock-in, models diverged from code
  - 2010s Agile Backlash: Lost institutional knowledge
  - ONE SENTENCE per era (not detailed deep dive)

- **Part 3 (25 min)**: Why SDD Succeeds NOW
  - AI literal-mindedness (can't infer intent like humans)
  - Cost-benefit math: Before (12 hours) vs With SDD (4 hours)
  - Why MDD failed but SDD succeeds (natural language, LLMs, no lock-in)

- **Part 4 (10 min)**: Practice Exercise
  - Reflect: "When did I iterate 5+ times due to vague requirements?"
  - Calculate time savings for own projects

**AI Role**: Reflective partner (student asks: "Why did MDD fail but SDD succeeds?")

**Cognitive Load**: 3 concepts, historical context, motivational = **Medium (B1)**

**Prerequisites**:
- Lesson 1 completed (understands vague code problem)

**Outputs**:
- Understanding of WHY specs matter NOW (motivation for learning structure)
- Recognition that AI makes specs necessary, not optional
- **Transition to L3**: "Now you know WHY specs matter. L3 shows WHAT a good spec looks like."

**Implementation Note**: Compress existing L5 (264 lines) to ~150-180 lines by:
- Keeping three convergent forces (CRITICAL)
- Compressing historical timeline (1 sentence per era)
- Keeping AI literal-mindedness insight
- Keeping cost-benefit math
- Removing detailed MDD analysis, market validation timeline

---

### Lesson 3: Anatomy of a Specification

**Stage**: 1 (Manual Foundation)
**Teaching Modality**: Direct teaching with annotated examples + quality awareness
**Duration**: 60-75 minutes
**Status**: ✅ EXISTING (add quality evaluation integration)

**Learning Objectives**:
- Identify the core sections of a production-ready spec (Intent, Constraints, Acceptance Criteria)
- Write a simple spec.md file manually (no AI yet)
- Evaluate spec quality using clarity, testability, and completeness checklist

**Concepts Introduced** (2):
1. **Spec.md Structure**: Intent, Requirements, Constraints, Non-Goals, Acceptance Tests
2. **Evals-First Principle**: Define success criteria before implementation

**Content Structure**:
- **Part 1**: Anatomy Walkthrough (section-by-section breakdown)
- **Part 2**: Annotated Example (production spec from real project)
- **Part 3**: Quality Awareness (NEW)
  - Good spec vs bad spec comparison
  - Quality checklist: Clear Intent? Testable success criteria? Explicit constraints?
  - Common pitfalls: Vague language, missing edge cases, over-specification
- **Part 4**: Practice (students write spec for simple Python feature)
- **Part 5**: Self-Validation (use quality checklist on own spec)

**AI Role**: None (manual practice builds foundation)

**Cognitive Load**: 2 concepts + quality awareness, structured template = **Medium (B1)**

**Prerequisites**:
- Lesson 2 completed (understands WHY specs matter)

**Outputs**:
- Student's first manually-written spec.md file
- Quality self-assessment capability
- **Transition to L4**: "You can write a spec manually. L4 shows how to write better specs WITH AI."

**Quality Integration**: Add "Good vs Bad Spec" comparison section showing:
- ❌ Vague: "System should be fast" → ✅ Clear: "API response time <200ms for 95th percentile"
- ❌ Untestable: "UI should look good" → ✅ Testable: "All buttons have 8px padding, primary color #007bff"

---

### Lesson 4: Build Your First Spec Together (AI Collaboration)

**Stage**: 2 (AI Collaboration)
**Teaching Modality**: Three Roles Framework (AI as Teacher/Student/Co-Worker)
**Duration**: 60-75 minutes
**Status**: ✅ EXISTING (add quality iteration)

**Learning Objectives**:
- Use AI to generate initial specification draft
- Refine AI-generated specs through iterative dialogue
- Demonstrate AI as Teacher (suggests missing requirements), Student (learns constraints), Co-Worker (converges on solution)

**Concepts Introduced** (1):
1. **AI as Spec Partner**: Iterative refinement through collaboration

**Content Structure**:
- **Role 1 — AI as Teacher**: AI suggests edge cases student didn't consider
- **Role 2 — AI as Student**: Student corrects AI's generic assumptions with domain constraints
- **Role 3 — AI as Co-Worker**: Convergence loop produces better spec than either alone
- **Practice**: Write calculator spec collaboratively (reuses L3 quality checklist)
- **Quality Iteration** (NEW): Draft → Evaluate against quality checklist → Refine → Validate

**AI Role**: Active collaboration (Three Roles demonstrated)

**Cognitive Load**: 1 concept (AI collaboration pattern), builds on L1-3 foundation = **Medium (B1)**

**Prerequisites**:
- Lesson 3 completed (can write basic spec manually + knows quality checklist)
- Understands spec structure well enough to evaluate AI output

**Outputs**:
- Collaboratively-refined calculator spec.md
- Experience with bidirectional learning (student ↔ AI)
- Quality-validated spec (using L3 checklist)
- **Transition to L5**: "You can write specs with AI. L5 shows how to ensure consistency across TEAM specs."

**Constitutional Requirement**: MUST demonstrate all three roles (Teacher/Student/Co-Worker)

**Quality Integration**: After collaborative draft, add quality review step:
- Check against L3 checklist (Clear Intent? Testable? Complete?)
- Identify vagueness collaboratively with AI
- Refine until quality criteria met

---

### Lesson 5: Your Team Needs Shared Rules (Constitutions)

**Stage**: 2 (AI Collaboration)
**Teaching Modality**: Governance patterns introduction
**Duration**: 60-75 minutes
**Status**: ✅ EXISTING LESSON (add quality consistency emphasis)

**Learning Objectives**:
- Understand specification consistency across teams and projects
- Define a Constitution (governance document) that applies globally
- Apply constitutional rules to ensure quality at scale

**Concepts Introduced** (2):
1. **Constitutions**: Global governance rules for all specifications
2. **Governance Patterns**: Memory Banks, ADRs, PHRs (organizational knowledge capture)

**Content Structure**:
- **Problem**: Inconsistent specs across team (different formats, conflicting rules)
- **Solution**: Constitutions as meta-governance (rules about specs)
- **Examples**: Memory Banks (Kiro framework), Constitutions (Spec-Kit framework)
- **Practice**: Create simple team constitution (password hashing rules, error handling standards)
- **Quality at Scale** (NEW): Constitution ensures quality consistency
  - Every spec meets minimum quality bar
  - Team standards encode lessons learned
  - Prevents recurring quality issues

**AI Role**: Governance advisor (helps design constitution)

**Cognitive Load**: 2 concepts (governance patterns), builds on L1-4 = **Medium (B1)**

**Prerequisites**:
- Lesson 4 completed (has written individual specs with AI)
- Understands spec structure and quality (can create governance rules)

**Outputs**:
- Simple team constitution document
- Understanding of governance as quality enforcement + reusable intelligence foundation
- **Transition to L6**: "You can write consistent specs. But you notice patterns repeating (auth, validation, error handling). L6 shows how to make those patterns REUSABLE."

**Why Critical**: Constitutions are CORE to SDD-RI methodology and prerequisite for L6-7 (Skills/Subagents inherit constitutional thinking)

**Quality Integration**: Constitution as quality enforcement mechanism:
- Global rules ensure every spec meets standards
- Example: "All API specs must define error responses" (prevents quality gaps)
- Team learns from mistakes, encodes in Constitution

---

### Lesson 6: Introduction to Reusable Intelligence (RI)

**Stage**: 3 (Intelligence Design)
**Teaching Modality**: Example-driven learning with real Skills/Subagents
**Duration**: 60-75 minutes
**Status**: 🆕 NET-NEW (Create from scratch)

**Learning Objectives**:
- Transform recurring spec patterns into reusable components (Skills/Subagents)
- Distinguish between Skills (2-4 decisions, guidance) and Subagents (5+ decisions, autonomous reasoning)
- Apply decision framework: When to encode pattern vs write fresh spec

**Concepts Introduced** (2):
1. **Reusable Intelligence (RI)**: Skills, Subagents as encoded specification patterns
2. **Skills vs Subagents Decision Framework**: Complexity, frequency, organizational value

**Content Structure**:
- **Part 1 (20 min)**: Real Skill Example
  - **Skill**: "API Error Handling Pattern" (from `.claude/skills/`)
  - When to use: Any feature with error responses
  - What it encodes: Status codes, error message format, logging requirements
  - Reusable across: 10+ API features
  - Practice: Identify which L1-4 specs could use this skill

- **Part 2 (20 min)**: Real Subagent Example
  - **Subagent**: "Security Auditor" (from `.claude/agents/`)
  - When to use: Any feature handling sensitive data
  - What it reviews: Input validation, encryption, authentication, authorization
  - Reusable across: All security-sensitive features
  - Practice: Would security auditor improve L1-4 specs?

- **Part 3 (20 min)**: Decision Framework + Practice
  - Pattern characteristics mapping:
    - 2-4 decisions, horizontal (applies broadly) → **Skill**
    - 5+ decisions, vertical (domain-specific) → **Subagent**
    - Organizational rule (applies to ALL specs) → **Constitution** (already learned in L4)
  - Practice: Map patterns from L5 analysis to Skills/Subagents/Constitutions

**AI Role**: Pattern identifier (helps categorize which patterns are which type)

**Cognitive Load**: 2 concepts with concrete examples = **Medium (B1)**

**Prerequisites**:
- Lesson 5 completed (identified recurring patterns)
- Understands Constitutions (L4) as foundation

**Outputs**:
- Pattern identification worksheet: Which L1-5 patterns are Skills vs Subagents?
- Understanding of RI as methodology extension (not just tooling)

**Pedagogical Note**: "Aha moment" — students realize SDD-RI is complete methodology, not just specs

---

### Lesson 7: Designing Skills and Subagents (P+Q+P Pattern)

**Stage**: 3 (Intelligence Design)
**Teaching Modality**: Design walkthrough with annotated examples
**Duration**: 60-75 minutes
**Status**: 🆕 NET-NEW (Create from scratch)

**Learning Objectives**:
- Design reusable intelligence using Persona + Questions + Principles (P+Q+P) pattern
- Understand why P+Q+P activates reasoning (not just pattern matching)
- Create 2 designed components: 1 Skill + 1 Subagent (documented, not implemented)

**Concepts Introduced** (1):
1. **Persona + Questions + Principles (P+Q+P)**: Reasoning activation pattern for RI design

**Content Structure**:
- **Part 1 (20 min)**: Design Walkthrough — "Input Validation Skill"
  ```markdown
  ### Persona
  You are a defensive programming specialist focused on preventing invalid input bugs.

  ### Questions to Ask
  1. What are valid input types? (string, int, float, list, dict)
  2. What are acceptable value ranges? (min/max, allowed values)
  3. What happens with invalid input? (raise ValueError, return None, default value)
  4. Should validation be strict (reject) or lenient (coerce)?

  ### Principles
  - Validate at system boundaries (API entry points, user input)
  - Fail fast with clear error messages
  - Use type hints + runtime validation for critical inputs
  - Document validation rules in spec.md
  ```

- **Part 2 (20 min)**: Design Walkthrough — "Performance Optimization Subagent"
  ```markdown
  ### Persona
  You are a performance engineer who reviews specifications for efficiency concerns.

  ### Questions to Ask (5+)
  1. What are expected data volumes? (10 records, 1M records, 1B records)
  2. What are latency requirements? (real-time <100ms, batch <1min, offline <1hr)
  3. Are there N+1 query risks? (database access patterns)
  4. Should results be cached? (frequency of identical requests)
  5. What are memory constraints? (loading all data vs streaming)

  ### Principles
  - Optimize for actual use case (premature optimization is waste)
  - Measure before optimizing (profile, don't guess)
  - Document performance requirements in Constraints section
  - Use appropriate data structures (O(1) lookup vs O(n) scan)
  ```

- **Part 3 (20 min)**: Practice — Student Designs 2 Components
  - Design "Security Review Subagent" using P+Q+P pattern
  - Design "API Design Skill" for consistent endpoint patterns
  - Critique: Which design is better and why?

**Why P+Q+P Pattern**:
- **Persona**: Activates domain expertise (security mindset, performance mindset)
- **Questions**: Prompts reasoning (not just pattern matching)
- **Principles**: Ensures consistency across applications

**AI Role**: Design critic (evaluates P+Q+P quality, suggests improvements)

**Cognitive Load**: 1 concept, 2 walkthroughs + practice = **Medium (B1)**

**Prerequisites**:
- Lesson 6 completed (understands Skills vs Subagents)
- Has pattern from L5-6 analysis to design

**Outputs**:
- 2 designed components (Skill + Subagent) documented using P+Q+P template
- Understanding that RI design ≠ RI implementation (design now, implement in Chapter 31+)

**Note**: Students design RI components but DON'T implement them in code (that's Chapter 31+ framework work)

---

### Lesson 8: SDD-RI Framework Landscape & What's Next

**Stage**: 3 → 4 Bridge (prepare for framework implementation in Chapter 31+)
**Teaching Modality**: Framework awareness + tool justification
**Duration**: 60-75 minutes (3-3.5 hours compressed)
**Status**: 🔄 ADAPT FROM EXISTING L6 (Framework Comparison)

**Learning Objectives**:
- Compare four SDD approaches (Kiro, Spec-Kit, Spec-Kit Plus, Tesel) and understand their evolution
- Understand context assessment for framework selection (team size, compliance, scale)
- Understand why this book teaches Spec-Kit Plus and prepare for Chapter 31 implementation

**Concepts Introduced** (3):
1. **Framework Landscape**: Four major SDD approaches with different philosophies
2. **Decision Context**: Team size, problem scale, regulatory constraints determine framework fit
3. **Spec-Kit Plus Extensions**: ADRs, PHRs, Intelligence Templates (AI-native additions)

**Content Structure** (from existing L6, adapted):

- **Part 1 (15 min)**: Know Your Context
  - Context assessment questions:
    - Who's building? (Solo, small team, medium, large)
    - Problem scale? (Simple, medium, complex)
    - Compliance constraints? (None, light, heavy, strict)
  - Purpose: Framework awareness, not premature decision-making

- **Part 2 (30 min)**: Four SDD Approaches (COMPRESSED from existing L6)
  - **Kiro**: Simple, low-friction (solo/small teams)
  - **Spec-Kit (GitHub)**: Strong governance, Constitution-based (enterprise)
  - **Spec-Kit Plus (Panaversity)**: Spec-Kit + ADRs/PHRs/Intelligence Templates (AI-native)
  - **Tesel**: Specs-as-source extreme (beta, safety-critical)
  - Decision matrix: Which framework for which situation

- **Part 3 (15 min)**: Why This Book Teaches Spec-Kit Plus
  - **NOT "choose your framework"** but "here's why WE chose Spec-Kit Plus"
  - ADRs capture reasoning (you've designed Skills/Subagents with reasoning in L7)
  - PHRs log AI interactions (co-learning pattern from L4)
  - Intelligence Templates (domain expertise, you've learned RI concepts in L6-7)
  - Frame as: **Our pedagogical choice for teaching SDD-RI**

- **Part 4 (10 min)**: Chapter 31 Preview & Bridge
  - What you'll build: Real features using Spec-Kit Plus
  - How frameworks implement SDD-RI concepts you've learned
  - Expectation: Chapter 30 = understand, Chapter 31 = implement
  - Synthesis:
    ```
    Chapter 30 Complete:
    - SDD Fundamentals (L1-5)
    - RI Concepts (L6-7)
    - Framework Awareness (L8)

    Chapter 31 Next:
    - Implement using Spec-Kit Plus
    - Build real features
    - Practice complete SDD-RI workflow
    ```

**AI Role**: Framework advisor (explains landscape, not makes choice FOR student)

**Cognitive Load**: 3 concepts (framework types, context factors, tool rationale) = **Medium (B1)**

**Prerequisites**:
- Lessons 1-7 completed (full SDD-RI conceptual understanding)
- Designed Skills/Subagents (L7)
- Understands Constitutions (L5)

**Outputs**:
- Framework landscape awareness (industry context)
- Understanding of Spec-Kit Plus rationale (why this book uses it)
- Preparation for Chapter 31 implementation
- **NOT**: Framework selection decision (that's for their real projects later)

**Framing**: "Industry landscape awareness before implementation" (NOT "choose your framework NOW")

**Implementation Note**: Adapt existing L6 (359 lines) by:
- Preserving context assessment (lines 32-60)
- Compressing framework comparison (lines 63-244)
- Reframing as "landscape awareness" not "student choice"
- Adding Chapter 31 bridge (NEW)

---

## Cognitive Load Validation

### Per-Lesson Concept Count

| Lesson | New Concepts | Cumulative | Load Tier |
|--------|--------------|------------|-----------|
| L1 | 2 (Vagueness, Intent vs Impl) | 2 | Low (A2-B1) |
| L2 | 3 (Three forces, Historical context, AI necessity) | 5 | Medium (B1) |
| L3 | 2 (Spec structure, Evals-first) | 7 | Medium (B1) |
| L4 | 1 (AI collaboration) | 8 | Medium (B1) |
| L5 | 2 (Constitutions, Governance) | 10 | Medium (B1) |
| L6 | 2 (RI, Skills vs Subagents) | 12 | Medium (B1) |
| L7 | 1 (P+Q+P pattern) | 13 | Medium (B1) |
| L8 | 3 (Framework landscape, Context, Extensions) | 16 | Medium (B1) |

**Average**: 2.0 concepts per lesson
**B1 Compliance**: ✅ All lessons within 1-3 concept limit (L2 and L8 have 3 due to motivational/synthesis nature)

### Progressive Disclosure Strategy

**Foundation (L1-5)**: 10 concepts, practical specification skills
- L1: Problem identification (vague code)
- L2: Historical motivation (why NOW)
- L3: Spec anatomy + quality awareness
- L4: Collaborative spec writing with AI
- L5: Governance patterns (Constitutions)

**Integration (L6-8)**: 6 concepts, RI thinking + framework awareness
- L6-7: Skills/Subagents introduction + design
- L8: Framework landscape + Chapter 31 bridge

**Load progression**: Motivation (L1-L2) → Structure (L3-L5) → Intelligence (L6-L7) → Implementation Prep (L8) ✅

---

## Anti-Convergence Validation

**Previous Chapter 29**: CPython and GIL
- **Modality**: Technical deep dive, direct teaching
- **Examples**: Python internals, GIL mechanics

**Chapter 30**: Spec-Driven Development
- **Modality**: Specification-first teaching (meta: teaching specs by writing specs)
- **Examples**: Production-relevant features (not toy apps)

**Variation Achieved**: ✅ Different teaching modality, different example domain

---

## Prerequisites and Dependencies

### Incoming Prerequisites (from Parts 1-4)

**From Part 1-3**:
- AI tool literacy (Claude Code, prompting basics)
- Markdown proficiency
- Git/GitHub workflows

**From Part 4 (Python)**:
- Programming fundamentals
- Experience writing code with AI assistance
- Understanding of testing and validation

### Outgoing Prerequisites (for Part 6+)

**Chapter 30 enables Part 6** (AI Native Software Development):
- Understanding of specifications as executable contracts
- Skills and Subagents concepts (RI foundation)
- Persona + Questions + Principles activation pattern
- Intelligence accumulation mindset

**Specific Part 6 dependencies**:
- Chapter 34 (Introduction to AI Agents) assumes L6-7 (Skills/Subagents)
- Chapter 42 (Test-Driven Agent Development) assumes L2 (Evals-first)
- Chapter 44 (Building Effective Agents) assumes L7 (P+Q+P pattern)

---

## Teaching Modality Breakdown

| Lesson | Primary Modality | Secondary Modality |
|--------|------------------|-------------------|
| L1 | Problem-based discovery | Case study analysis |
| L2 | Historical context + motivation | Timeline analysis |
| L3 | Direct teaching + quality awareness | Annotated examples |
| L4 | Three Roles Framework | Iterative refinement |
| L5 | Governance patterns | Practical constitution design |
| L6 | Example-driven learning | Decision framework |
| L7 | Design walkthrough | Hands-on P+Q+P practice |
| L8 | Framework awareness + tool justification | Landscape comparison |

**Variety**: ✅ No two consecutive lessons use identical modality
**Practical Focus**: ✅ All lessons include hands-on exercises with real artifacts
**Strategic Flow**: ✅ Motivation (L2) → Structure (L3-L5) → Intelligence (L6-L7) → Implementation Prep (L8)

---

## Success Criteria (Lesson Plan Quality)

### Pedagogical Arc Completeness
- ✅ Foundation phase (L1-4): SDD fundamentals with practical specification skills
- ✅ Integration phase (L5-8): RI extension with Skills/Subagents design
- ✅ Practical focus: Students write specs, see examples, practice workflows
- ✅ Clear boundary: Understanding SDD-RI (Chapter 30) vs Implementing with frameworks (Chapter 31+)

### Constitutional Compliance
- ✅ Principle 2 (Progressive Complexity): B1 tier, 1-2 concepts per lesson
- ✅ Principle 4 (Coherent Structure): Clear pedagogical arc
- ✅ Principle 5 (Intelligence Accumulation): Prepares for Part 6
- ✅ Principle 6 (Anti-Convergence): Varied teaching modalities
- ✅ Section IIa (4-Stage Framework): Stages 1-3 mapped to lessons

### Stage Transition Validation
- ✅ Stage 1→2 (L3→4): Students can write basic spec manually before AI collaboration
- ✅ Stage 2→3 (L5→6): Students understand governance before learning RI concepts
- ✅ Stage 3→4 (L8→Ch31): Students ready for framework implementation (Spec-Kit Plus)

### Learning Objectives Clarity
- ✅ Every lesson has 3 measurable learning objectives
- ✅ Objectives build progressively (each lesson enables next)
- ✅ Final lesson synthesizes all prior objectives into complete SDD-RI understanding

### Practical Learning Validation
- ✅ Students create artifacts: specs, constitutions, Skills/Subagents designs
- ✅ Real examples: Production-relevant features (not toy apps)
- ✅ Hands-on practice: Every lesson includes exercises
- ✅ NOT framework implementation: Chapter 30 = understanding, Chapter 31+ = tooling

---

## Implementation Notes

### Completed (L1)
**Status**: ✅ DONE
- L1: Three Roles Framework validated (95% compliance)

### Remaining Work (L2-L8)

**L2: Why Specs Matter NOW** (COMPRESSION NEEDED)
- Source: Existing L5 (264 lines, 2.5-3 hours)
- Target: 150-180 lines, 60-75 minutes
- Keep: Three convergent forces, AI literal-mindedness, cost-benefit math
- Compress: Historical timeline (1 sentence per era)
- Remove: Detailed MDD analysis, market validation timeline
- **Estimated effort**: 2-3 hours

**L3: Anatomy of Specification** (MINOR UPDATE)
- Source: Existing L2 (100% compliant)
- Add: Quality awareness section (good vs bad spec comparison)
- Add: Quality checklist introduction
- **Estimated effort**: 1-2 hours

**L4: Build Your First Spec Together** (MINOR UPDATE)
- Source: Existing L3 (100% compliant)
- Add: Quality iteration step (draft → checklist → refine)
- Already has co-learning reflection
- **Estimated effort**: 1 hour

**L5: Your Team Needs Shared Rules** (MINOR UPDATE)
- Source: Existing L4 (Constitution lesson)
- Add: Quality at scale emphasis
- Add: Constitution as quality enforcement
- **Estimated effort**: 1 hour

**L6: Introduction to Reusable Intelligence** (KEEP AS PLANNED)
- Content: Real Skills/Subagents examples
- Practice: Pattern identification
- **Estimated effort**: 4-5 hours (NET-NEW)

**L7: Designing Skills and Subagents** (KEEP AS PLANNED)
- Content: P+Q+P pattern walkthrough
- Practice: Design 2 components
- **Estimated effort**: 4-5 hours (NET-NEW)

**L8: Framework Landscape** (ADAPTATION NEEDED)
- Source: Existing L6 (359 lines, 3-3.5 hours)
- Target: ~200 lines, 60-75 minutes
- Preserve: Context assessment, framework comparison
- Reframe: "Landscape awareness" not "student choice"
- Add: Chapter 31 bridge
- **Estimated effort**: 2-3 hours

**Total Remaining**: 15-20 hours of implementation

### Validation Checklist (Per Lesson)
- [ ] Learning objectives measurable and achievable
- [ ] Concept count within B1 limit (1-2 per lesson)
- [ ] Teaching modality identified and varied from adjacent lessons
- [ ] AI role appropriate for stage (Manual → Collaboration → Co-design)
- [ ] Prerequisites satisfied by prior lessons
- [ ] Outputs feed into next lesson or part
- [ ] Practical examples (students create artifacts, not just read theory)
- [ ] Clear boundary: Understanding (Chapter 30) vs Framework implementation (Chapter 31+)

---

## References

**Constitutional Grounding**:
- `.specify/memory/constitution.md` v6.0.0
  - Section IIa: 4-Stage Teaching Framework
  - All 7 Foundational Principles

**Domain Structure**:
- `specs/book/chapter-index.md` — Part 5 structure
- Chapter 30 existing content: `book-source/docs/05-Spec-Driven-Development/30-specification-driven-development-fundamentals/`

**Skills Library** (referenced in L6-7):
- `.claude/skills/` — Example skills for student analysis

**Strategic Decision**:
- `specs/030-chapter-30-sdd-ri-restructure/adr-001-expand-chapter-30-to-include-ri.md` — ADR documenting expansion rationale

---

## Summary

**This lesson plan operationalizes the strategic decision to teach SDD-RI as unified methodology through 8 progressively-structured lessons with practical learning focus.**

### Key Principles

1. **Practical Learning Through Examples**: Students write specs, see real examples, practice workflows
2. **Clear Scope Boundary**: Chapter 30 = Understanding SDD-RI | Chapter 31+ = Framework implementation
3. **Artifact Creation**: Students produce tangible outputs (specs, constitutions, Skills/Subagents designs)
4. **B1 Cognitive Load**: 1-2 concepts per lesson, example-driven teaching
5. **Constitutional Alignment**: 4-Stage progression, Three Roles Framework, Intelligence Accumulation

### What Students Can Do After Chapter 30

✅ Write clear specifications manually and with AI collaboration
✅ Identify vague vs precise requirements in any spec
✅ Recognize recurring patterns across multiple specifications
✅ Design reusable intelligence using P+Q+P pattern
✅ Create governance rules (constitutions for consistency)
✅ Understand complete SDD-RI methodology (ready for framework implementation in Chapter 31+)

### What Students CANNOT Do Yet (Chapter 31+ Content)

❌ Use Spec-Kit framework to generate code from specs
❌ Build actual Skills/Subagents in code
❌ Deploy SDD-RI pipelines with CI/CD integration
❌ Implement Spec-as-Source workflows at scale

**Chapter 30 builds the mental model. Chapter 31+ provides the implementation tools.**
