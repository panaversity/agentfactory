# Part 5: Spec-Kit Plus Methodology — Implementation Plan

**Branch**: `008-part-5-sdd` | **Date**: 2025-11-01 | **Spec**: `/specs/008-part-5-sdd/spec.md`

**Plan Status**: Complete — Ready for Task Generation & Lesson Writing

---

## Summary

Part 5 pivots students from technical skills (Parts 1-4) to professional software development methodology. Three interleaved chapters teach Specification-Driven Development (SDD) through progressive, hands-on projects:

- **Chapter 25** (Fundamentals): Concept-first + AI collaboration discovery — students learn *why* SDD matters
- **Chapter 26** (Hands-On): SpecifyPlus framework + mini-projects — students learn *how* to use SDD
- **Chapter 27** (Real-World): Capstone project + team simulation — students learn *when* to scale SDD

**Pedagogical Model**: Theory and practice run in parallel. Every concept introduced in one chapter is immediately practiced in the next. Students move from abstract understanding (Ch 25) → safe practice (Ch 26 calculator) → real-world application (Ch 27 capstone: grading system with parallel AI agents).

**Key Outcome**: Students complete Part 5 capable of writing specifications for any project, planning implementation with SpecifyPlus, and coordinating AI agents through clear specifications. They are ready for Parts 6+ (Agentic AI, Deployment, Databases).

---

## Learning Architecture

### Timeline & Effort Estimation

| Chapter | Focus | Lessons | Hours | Weeks |
|---------|-------|---------|-------|-------|
| **Ch 25** | SDD Fundamentals | 5 lessons | 8-10h | 1.5 weeks |
| **Ch 26** | Hands-On SpecifyPlus | 7 lessons + 2 mini-projects | 12-15h | 2 weeks |
| **Ch 27** | Real-World Workflows & Capstone | 6 lessons + capstone | 12-15h | 2 weeks |
| **Part 5 Total** | | 18 lessons + 3 projects | 32-40h | 5-6 weeks |

**Complexity Tier**: Advanced (Parts 6-8) — readers are Python-proficient, AI-tool-literate, ready for architectural thinking.

### Core Learning Arc

```
Chapter 25: UNDERSTAND
  Why is SDD critical for AI-native development?
  How do clear specs reduce iteration cycles?

Chapter 26: PRACTICE
  How do I write testable specifications?
  How do I use SpecifyPlus commands?
  How do I coordinate specs with AI implementation?

Chapter 27: MASTER
  How do I scale specs to team coordination?
  How do I decompose systems for parallel development?
  How do I manage spec-driven teams?
```

### Prerequisites & Foundation

**Must Complete Before Part 5**:
- Part 1: AI-native mindset established
- Part 2: AI tools introduced (Claude Code, Gemini CLI)
- Part 3: Prompting and context engineering
- Part 4: Python 3.13+ proficiency with type hints

**Enables for Parts 6-13**:
- Part 6 (Agents): Agents require clear behavior specifications
- Part 10 (Deployment): Docker/Kubernetes are specifications
- Part 11 (Databases): Schemas are specifications
- Part 13 (Stateful Agents): Workflows are specifications

---

## Chapter 25: Specification-Driven Development Fundamentals

**Pedagogical Model**: Concept-first + Spec-First Discovery AI collaboration

**Duration**: 8-10 hours | **5 Lessons** | **Advanced complexity**

### Learning Objectives (Bloom's Taxonomy)

By chapter end, students will:
1. **Understand** the SDD loop (Spec → Plan → Tasks → Implementation → Validation)
2. **Analyze** tradeoffs between spec-driven and code-first approaches
3. **Apply** SDD concepts to real project evaluation
4. **Evaluate** specification quality using acceptance criteria
5. **Create** rough specifications and iteratively refine with AI

### Lesson Breakdown

| Lesson | Title | Hours | Learning Objective | Key Activity |
|--------|-------|-------|-------------------|--------------|
| 1 | What Is Specification-Driven Development? | 2h | Understand SDD as methodology + why it matters | Real-world crisis case study |
| 2 | The Cost of Specifications vs. Code | 2h | Analyze cost/benefit tradeoffs | Cost model comparison |
| 3 | Spec vs. Code Discovery (AI Collaboration) | 2.5h | Experience why clear specs matter | Vague spec → AI struggles → Refined spec → AI excels |
| 4 | The SDD Loop Formalized | 2h | Map spec to 5-phase workflow | Walk through API spec project |
| 5 | SDD and AI-Native Development Connection | 1.5h | Connect SDD to broader professional practice | Synthesis + commitment |

**Each lesson concludes with "Try With AI" section** using ChatGPT web (accessible, established tool for Part 5). Prompts guide reflection, not hands-on coding.

---

## Chapter 26: Writing and Planning Specifications with SpecifyPlus

**Pedagogical Model**: Hands-on SpecifyPlus framework + progressive project complexity

**Duration**: 12-15 hours | **7 Lessons + 2 Mini-Projects** | **Advanced complexity**

### Learning Objectives

By chapter end, students will:
1. **Apply** SpecifyPlus framework to write clear, testable specifications
2. **Use** core SpecifyPlus commands fluently (`/sp.specify`, `/sp.plan`, `/sp.tasks`)
3. **Analyze** incomplete specifications and identify missing requirements
4. **Create** acceptance criteria checklists that enable validation
5. **Perform** iterative spec refinement with AI assistance

### Lesson & Project Breakdown

| # | Title | Hours | Type | Learning Focus |
|---|-------|-------|------|-----------------|
| 1 | Specification Structure — The Anatomy | 2h | Lesson | Components of good specs |
| 2 | SpecifyPlus Framework & Setup | 2h | Lesson + Hands-On | Installation, navigation, commands |
| 3 | Writing Acceptance Criteria | 2h | Lesson | Testable, observable, measurable done-ness |
| 4 | /sp.specify and /sp.plan Commands | 2h | Lesson + Hands-On | Using commands to create and refine specs |
| 5 | /sp.tasks Command | 2h | Lesson | Atomic, testable task decomposition |
| 6 | Spec Refinement and Iteration | 2h | Lesson + Peer Review | AI-assisted spec improvement |
| 7 | Putting It All Together | 2h | Lesson + Integration | End-to-end SDD workflow |
| MP1 | Python Calculator (Mini-Project 1) | 3h | Hands-On | Complete SDD cycle in safe project |
| MP2 | Grading System Spec (Mini-Project 2) | 3h | Hands-On | Real-world spec writing + preparation for Ch 27 |

**Mini-Project 1 (Calculator)**:
- Safe, bounded scope (familiar to all students)
- Complete SDD cycle: spec → plan → tasks → implement → validate
- Students experience: Vague spec → Refine → AI implementation → Test → Validate
- Outcome: Fluency with SpecifyPlus and SDD workflow

**Mini-Project 2 (Grading System)**:
- Real-world scope (YC-validated market problem)
- Write complete specification (no implementation yet)
- Foundation for Chapter 27 capstone
- Outcome: Understanding of real-world specification complexity

---

## Chapter 27: Real-World Spec-Kit Workflows & Team Collaboration

**Pedagogical Model**: Full capstone project with multi-agent team simulation

**Duration**: 12-15 hours | **6 Lessons + Capstone** | **Advanced complexity**

### Learning Objectives

By chapter end, students will:
1. **Analyze** how specifications enable team coordination (1, 5, 10+ person teams)
2. **Design** feature decompositions for parallel development
3. **Coordinate** multiple AI agents working on different features simultaneously
4. **Evaluate** integration points and validate system coherence
5. **Synthesize** SDD methodology across full-stack development

### Lesson & Capstone Breakdown

| # | Title | Hours | Focus |
|---|-------|-------|-------|
| 1 | SDD at Scale — From Solo to Teams | 2h | Conceptual: How specs enable async collaboration |
| 2 | Feature Decomposition for Parallel Work | 2h | Design: Breaking systems into independent features |
| 3 | Team Workflows & Spec-Driven Practices | 2h | Practical: Code review, versioning, change control |
| 4 | Scaling to 10+ Person Teams | 2h | Advanced: Async coordination, governance, ownership |
| 5 | Real-World Integration — SDD in Production | 2h | Synthesis: APIs, databases, deployment, agents |
| 6 | SDD Philosophy & Professional Practice | 1.5h | Reflection: Career implications, commitment |
| Capstone | Grading System with Parallel AI Agents | 6-8h | Hands-On: Multi-feature implementation + integration |

**Capstone Project Walkthrough**:

**Part 1: Master Spec & Decomposition** (2-3 hours)
- Review and refine grading system spec from Ch 26
- Decompose into 2 independent features:
  - Feature 1: Grading Engine (input: submission + rubric; output: grade + explanation)
  - Feature 2: Feedback Generation (input: submission + grade + rubric; output: personalized feedback)
- Write feature specs with clear interfaces
- Document how features integrate

**Part 2: Parallel AI Agent Implementation** (2-3 hours)
- Create 2 SpecifyPlus project instances
  - Project 1: AI Companion A (Claude Code) building Grading Engine
  - Project 2: AI Companion B (Gemini CLI) building Feedback Generator
- Run `/sp.plan` independently on each feature
- Run `/sp.tasks` on each plan
- Both AI companions implement their features in parallel
- Student validates each implementation against its specification
- Integrate implementations and run end-to-end tests
- Key learning: Clear specs enable parallel work; integration is straightforward

**Part 3: Reflection & Professional Practice** (1-2 hours)
- Reflect on spec impact: "How did spec quality affect implementation quality?"
- Reflect on parallelization: "How did clear specs enable two teams to work independently?"
- Reflect on AI coordination: "How did you coordinate AI agents through specifications?"
- Write synthesis: "My Specification-Driven Development Manifesto" (500 words)
- Identify connections to Parts 6+ (agents, deployment, databases)

**Success Criteria**:
- [ ] Master spec complete and feature decomposition sound
- [ ] Both feature implementations pass acceptance criteria tests
- [ ] Integration tests validate system coherence
- [ ] Student articulates how specs enabled parallel development
- [ ] Student is ready for Part 6 (agent systems are specification-driven)

---

## Technical Context

**Project Type**: Educational (chapters, not code product)

**Content Scope**: 18 lessons across 3 chapters, 3 hands-on projects (calculator spec-impl, grading system spec, capstone implementation)

**Tools/Frameworks**:
- **SpecifyPlus**: SDD framework (spec.md, plan.md, tasks.md, commands)
- **AI Companions**: Claude Code, Gemini CLI, ChatGPT web
- **Python**: 3.13+ with type hints (calculator, grading system implementations)
- **Testing**: pytest for validation of generated code

**Output Artifacts**:
- 18 lesson markdown files (chapter-specific structure)
- 2 mini-project specifications + implementations
- 1 capstone project with parallel features + integration
- Part 5 README.md with learning paths and navigation

---

## Constitution Alignment

**Core Principles Applied**:

✅ **Principle 2 (Spec-Kit Plus Methodology)**:
- Part 5 teaches SDD as THE core methodology
- All projects use Spec-Kit Plus structure: spec.md → plan.md → tasks.md → implementation
- Students practice specification-writing WITH AI assistance

✅ **Principle 9 (Show-Spec-Validate Pedagogy)**:
- Every lesson shows specification before code
- Code examples include the spec that produced them
- Validation steps demonstrated throughout

✅ **Principle 10 (Real-World Project Integration)**:
- Chapter 26 calculator is bounded but complete
- Chapter 27 capstone uses YC-validated grading system problem (real market)
- All projects include documentation, testing, deployment awareness

✅ **Principle 12 (Cognitive Load - Advanced Tier)**:
- No artificial simplification for advanced audience
- 10+ concepts per lesson section (synthesis expected)
- Multiple options shown realistically (no limiting)
- Architectural thinking and tradeoffs included

✅ **Principle 14 (Planning-First Development)**:
- All projects start with specification
- AI generation ONLY after spec approved
- Iteration happens at spec level (refine spec, regenerate code)

✅ **Principle 15 (Validation-Before-Trust)**:
- Every "Try With AI" section includes code reading + understanding
- Capstone includes comprehensive testing and validation
- Students practice validating AI-generated code against specifications

---

## Factual Claims Requiring Verification

**Market Validation**:
- "Grading systems are YC-validated problem" — Verify: Gradewiz, Edexia, Frizzle, Mimir all exist and solving this problem
- Maintenance trigger: Annual review of startup status

**Rework Statistics**:
- "35-50% rework without specs vs. 15-20% with specs" — Source needed (academic research or industry survey)
- Maintenance trigger: Update when better data available

**Company Examples**:
- Stripe API specification enabling 15+ SDKs
- Anthropic Claude API documentation
- AWS 200+ teams coordinating via service contracts
- Maintenance trigger: Verify current accuracy before publication

**Tool Commands**:
- SpecifyPlus command syntax (`/sp.specify`, `/sp.plan`, `/sp.tasks`)
- Maintenance trigger: Major SpecifyPlus version releases

---

## Architecture Decisions

**Decision 1: Interleaved vs. Sequential Chapters**
- **Selected**: Interleaved (theory + practice each chapter)
- **Rationale**: Maintains motivation; students practice immediately after learning
- **Alternative Rejected**: Sequential (all theory then all practice) = loss of engagement

**Decision 2: AI Companions for Chapter 27**
- **Selected**: Parallel instances (2 AI companions, 2 features)
- **Rationale**: Demonstrates real team coordination challenges; specs solve real problems
- **Alternative Rejected**: Single feature = misses team lessons

**Decision 3: Capstone Scope**
- **Selected**: Real grading system (market-validated problem)
- **Rationale**: Motivation + relevance to real students' lives (many have teacher connections)
- **Alternative Rejected**: Toy project (low motivation, low relevance)

---

## Validation Strategy

### Chapter 25 (Fundamentals)

**Formative**:
- End-of-lesson reflection prompts
- AI conversation artifacts

**Summative**:
- Specification quality evaluation (compare vague vs. clear specs)
- Synthesis: "How does SDD connect to AI-native development?"

**Success Criteria**:
- Student explains SDD loop and five phases
- Student understands cost/benefit tradeoffs
- Student has experienced spec-first discovery with AI

### Chapter 26 (Hands-On)

**Formative**:
- Mini-Project 1 (Calculator): Instructor review at each SDD phase
- Mini-Project 2 (Grading): Spec review and refinement feedback

**Summative**:
- Calculator: Complete implementation passing all tests
- Grading: Specification ready for Chapter 27 implementation

**Success Criteria**:
- Student writes complete specs with all 6 components
- Student uses `/sp.specify`, `/sp.plan`, `/sp.tasks` fluently
- Mini-projects demonstrate full SDD workflow

### Chapter 27 (Real-World)

**Formative**:
- Lessons 1-6: Discussion and reflection
- Capstone Part 1: Spec review and feature decomposition check

**Summative**:
- Capstone Part 2: Both features implemented, tests passing
- Capstone Part 3: Synthesis manifesto and professional reflection

**Success Criteria**:
- Master spec and feature decomposition sound
- Both features implement specs; integration works
- Student articulates team coordination through specs
- Student ready for Part 6 (agents are spec-driven)

---

## Next Steps

This plan is ready for:

1. **Task Generation** — Run `/sp.tasks` to create detailed task checklists
2. **Lesson Writing** — Distribute to content-implementer subagent for content creation
3. **Review Cycles** — Technical and pedagogical review before publication

---

**Version**: 1.0 | **Date**: 2025-11-01 | **Status**: Ready for Task Generation
