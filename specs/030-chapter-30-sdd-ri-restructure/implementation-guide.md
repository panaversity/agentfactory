# Implementation Guide: Chapter 30 Restructuring to 8-Lesson SDD-RI

**Status**: Ready for implementation
**Target**: `book-source/docs/05-Spec-Driven-Development/30-specification-driven-development-fundamentals/`
**Change Type**: Expansion (5 lessons → 8 lessons)
**Implementation Effort**: Medium (3 net-new lessons, 5 minor updates)

---

## Overview

**What Changed**: Chapter 30 expands from 5 lessons (SDD only) to 8 lessons (unified SDD-RI methodology)

**Why Changed**: Book teaches "SDD-RI" but Chapter 30 only covered "SDD", creating prerequisite gap for Part 6+ (agent building)

**Impact**:
- **Lessons 1-5**: Minor updates (existing SDD content)
- **Lessons 6-8**: Net-new implementation (RI concepts)
- **Chapter README**: Update to reflect 8-lesson structure
- **Chapter-index.md**: Update Chapter 30 description to include RI

---

## Implementation Phases

### Phase 1: Preparation (Pre-Implementation)

**Tasks**:
- [ ] Read strategic decision documents:
  - `specs/030-chapter-30-sdd-ri-restructure/adr-001-expand-chapter-30-to-include-ri.md`
  - `specs/030-chapter-30-sdd-ri-restructure/lesson-plan.md`
  - `specs/030-chapter-30-sdd-ri-restructure/concept-mapping.md`
- [ ] Review constitutional grounding (`.specify/memory/constitution.md` v6.0.0)
- [ ] Backup existing Chapter 30 content
- [ ] Create implementation branch: `030-chapter-30-sdd-ri-expansion`

**Validation**:
- All strategic context understood
- Constitutional principles internalized
- Safe to proceed with changes

---

### Phase 2: Existing Content Updates (Lessons 1-5)

**Scope**: Minor updates to existing lessons, no major rewrites

#### Lesson 1: Why Specifications Matter

**Current State**: Exists
**Changes Required**: Minor (validate cognitive load, add production examples)

**Update Checklist**:
- [ ] Validate concept count (max 2: Vagueness Cost, Intent vs Implementation)
- [ ] Ensure examples are production-relevant (not toy apps)
- [ ] Add real-world case study if missing
- [ ] Verify B1 scaffolding (heavy for foundational lesson)
- [ ] No forward reference needed (L1 is foundation)

**Validation**:
- Cognitive load: 2 concepts ✅
- Teaching modality: Problem-based discovery ✅
- Examples: Production-relevant ✅

#### Lesson 2: Anatomy of a Specification

**Current State**: Exists
**Changes Required**: Minor (validate structure, ensure evals-first emphasis)

**Update Checklist**:
- [ ] Validate concept count (max 2: Spec.md Structure, Evals-First)
- [ ] Ensure spec.md structure matches project template
- [ ] Verify evals-first principle emphasized (success criteria before code)
- [ ] Check annotated example is complete and clear
- [ ] Practice exercise: Students write manual spec

**Validation**:
- Cognitive load: 2 concepts ✅
- Teaching modality: Direct teaching with annotated examples ✅
- Spec structure aligns with `.specify/templates/` ✅

#### Lesson 3: Writing Specs with AI

**Current State**: Exists
**Changes Required**: **Critical** — Must demonstrate Three Roles Framework

**Update Checklist**:
- [ ] **CRITICAL**: Validate Three Roles demonstrated:
  - [ ] AI as Teacher: AI suggests edge case student didn't consider
  - [ ] AI as Student: Student corrects AI's generic assumption
  - [ ] AI as Co-Worker: Convergence loop produces better spec
- [ ] Ensure iterative refinement shown (not one-shot prompting)
- [ ] Practice exercise: Students collaborate with AI on intermediate feature
- [ ] Verify B1 cognitive load (1 concept: AI collaboration pattern)

**Constitutional Requirement** (Section IIa, Stage 2):
> Every Stage 2 lesson must include:
> 1. At least ONE instance where AI teaches student
> 2. At least ONE instance where student teaches AI
> 3. At least ONE convergence loop

**Validation**:
- Three Roles explicitly demonstrated ✅
- Bidirectional learning shown (not passive tool use) ✅
- Cognitive load: 1 concept ✅

#### Lesson 4: From Spec to Code

**Current State**: Exists
**Changes Required**: Minor (validate workflow sequence, ensure primacy clear)

**Update Checklist**:
- [ ] Validate workflow sequence: spec.md → prompt → code → validation
- [ ] Ensure specification primacy explicit (code is OUTPUT, not INPUT)
- [ ] Include debugging exercise (intentional spec-code mismatch)
- [ ] Validation loop demonstrated (iterate until acceptance criteria met)
- [ ] Verify B1 cognitive load (1 concept: Specification Primacy workflow)

**Validation**:
- Workflow sequence clear ✅
- Debugging practice included ✅
- Cognitive load: 1 concept ✅

#### Lesson 5: Spec Quality & Tooling Landscape

**Current State**: Exists
**Changes Required**: **Critical** — Must add transition to L6 (RI intro)

**Update Checklist**:
- [ ] Validate concept count (2: Quality spectrum, Tooling tradeoffs)
- [ ] Ensure over/under-specification examples clear
- [ ] Include tooling comparison (SpecKit Plus, Tessl, custom markdown)
- [ ] Decision framework for tooling selection provided
- [ ] **CRITICAL**: Add lesson ending transition:
  ```markdown
  ## What's Next

  You've mastered writing individual specifications. But what happens when you write similar specs repeatedly across projects?

  In the next lesson, we'll explore **Reusable Intelligence (RI)**—the patterns and practices that make specifications composable and reusable, not just one-off documents.
  ```

**Validation**:
- Quality evaluation framework clear ✅
- Tooling decision criteria provided ✅
- Transition to L6 explicit ✅
- Cognitive load: 2 concepts ✅

---

### Phase 3: Net-New Content Creation (Lessons 6-8)

**Scope**: Full lesson implementation (from scratch)

#### Lesson 6: Introduction to Reusable Intelligence

**Current State**: Does not exist
**Implementation Required**: **Full lesson creation**

**Content Specification**:

**Learning Objectives**:
1. Recognize when patterns recur enough to justify encoding as reusable intelligence
2. Distinguish between Skills (2-4 decisions) and Subagents (5+ decisions)
3. Apply encoding decision framework (Frequency, Complexity, Organizational Value)

**Concepts Introduced** (2):
- Reusable Intelligence (RI): Skills, Subagents, Intelligence Libraries
- Encoding Decision Framework: When to make specs reusable

**Content Structure**:
1. **Problem Introduction** (10 min)
   - "I'm writing similar specs repeatedly—there must be a better way"
   - Show 3 similar specs side-by-side (highlight duplication)

2. **Solution: Reusable Intelligence** (15 min)
   - Definition: RI = Skills + Subagents + Intelligence Libraries
   - Skills: 2-4 decision guidance documents
   - Subagents: 5+ decision autonomous reasoning agents
   - Intelligence Libraries: Organizational knowledge accumulation

3. **Decision Framework** (20 min)
   - **Frequency**: Pattern recurs 2+ times → consider encoding
   - **Complexity**: 2-4 decisions → Skill, 5+ → Subagent
   - **Organizational Value**: Applies across 3+ projects → worth encoding
   - Show decision tree diagram

4. **Real Examples** (15 min)
   - Skill example: `.claude/skills/learning-objectives/skill.md`
   - Subagent example: `.claude/agents/content-implementer/`
   - Walk through structure, explain components

5. **Practice Exercise** (20 min)
   - Students analyze their past work (from L1-5)
   - Identify recurring patterns
   - Apply decision framework: Which patterns justify encoding?
   - Document RI opportunities (preparation for L7)

**AI Role**: Co-designer (help identify patterns worth encoding)

**Teaching Modality**: Discovery-based (problem → pattern → solution)

**Cognitive Load Management**:
- 2 concepts (RI definition + decision framework)
- Real examples from this project reduce abstraction
- Builds on L1-5 foundation (specs mastered)

**Validation Criteria**:
- [ ] Problem motivation clear (duplication pain)
- [ ] RI definition concrete (not abstract)
- [ ] Skills vs Subagents distinction clear (decision count)
- [ ] Real examples from project shown
- [ ] Practice exercise actionable
- [ ] Transition to L7: "Next: How to design these components"

**Files to Create**:
- `06-introduction-to-reusable-intelligence.md`

---

#### Lesson 7: Designing Skills and Subagents

**Current State**: Does not exist
**Implementation Required**: **Full lesson creation (workshop format)**

**Content Specification**:

**Learning Objectives**:
1. Apply Persona + Questions + Principles pattern to activate reasoning mode
2. Create a functional Skill document for recurring workflow
3. Distinguish reasoning mode (context-specific) from prediction mode (generic templates)

**Concepts Introduced** (1):
- Persona + Questions + Principles (P+Q+P): Reasoning activation pattern

**Content Structure** (Workshop Format):

1. **Theory: Reasoning vs Prediction** (15 min)
   - Prediction mode: Pattern retrieval from training data (generic outputs)
   - Reasoning mode: Context analysis using frameworks (specific outputs)
   - Why it matters: Generic skills fail, reasoning skills adapt

2. **P+Q+P Pattern Breakdown** (20 min)
   - **Persona**: Cognitive stance that activates right thinking
     - Bad: "You are an expert" (vague)
     - Good: "Think like a DevOps engineer balancing speed vs image size"
   - **Questions**: Context-specific analysis prompts
     - Bad: "Is this secure?" (prediction trigger)
     - Good: "What attack surfaces exist? What threat vectors apply? How to prioritize?" (reasoning trigger)
   - **Principles**: Decision frameworks (not rigid rules)
     - Bad: "Use best practices" (meaningless)
     - Good: "Defense in depth: Never single control. Fail secure: Errors deny access."

3. **Guided Workshop** (40 min)
   - Students select RI opportunity from L6 analysis
   - **Step 1** (10 min): Write Persona
     - What cognitive stance activates right reasoning?
     - Review examples, draft, refine with AI
   - **Step 2** (15 min): Write Questions
     - What analysis forces context-specific thinking?
     - Create 5-7 questions that avoid generic answers
   - **Step 3** (15 min): Write Principles
     - What decision frameworks guide application?
     - Document with examples (not abstract rules)

4. **Validation & Testing** (15 min)
   - Test skill on new context (not original)
   - Check: Does output show reasoning (context analysis) or prediction (generic template)?
   - If prediction mode → Strengthen persona/questions
   - Iterate until reasoning activation confirmed

**AI Role**: Co-designer (help refine P+Q+P structure)

**Teaching Modality**: Co-design workshop (hands-on creation)

**Cognitive Load Management**:
- 1 concept (P+Q+P) but highest complexity in chapter
- Extended time (75-90 min vs 60-75 typical)
- Workshop format provides scaffolding
- Hands-on creation cements understanding

**Constitutional Reference**:
- Teaches pattern from `.specify/memory/constitution.md` Section 0
- Meta-pedagogical: Teaching the teaching framework

**Validation Criteria**:
- [ ] Reasoning vs Prediction distinction clear
- [ ] P+Q+P pattern explained with examples (good vs bad)
- [ ] Workshop scaffolded (step-by-step guidance)
- [ ] Students create complete Skill document
- [ ] Validation step included (test for reasoning activation)
- [ ] Transition to L8: "Skills at organizational scale"

**Files to Create**:
- `07-designing-skills-and-subagents.md`
- `assets/skill-template.md` (starter template for students)

---

#### Lesson 8: Organizational Patterns & Governance

**Current State**: Does not exist
**Implementation Required**: **Full lesson creation (case study format)**

**Content Specification**:

**Learning Objectives**:
1. Understand how Constitutions embed governance at scale
2. Evaluate spec-as-source vision (Tessl framework)
3. Apply organizational patterns: Intelligence Libraries, Agent Teams, Manager Patterns

**Concepts Introduced** (1):
- Constitutions: Domain-level governance documents encoding principles (not rules)

**Content Structure**:

1. **Case Study: This Project's Constitution** (25 min)
   - Read `.specify/memory/constitution.md` (excerpt)
   - Analyze structure:
     - Section 0: Persona (Educational Systems Architect)
     - 7 Foundational Principles (frameworks, not rules)
     - Stage Progression criteria
     - Self-monitoring prompts
   - Key insight: Constitution enables coordination without micromanagement
   - How it governs this chapter's design (meta-example)

2. **Intelligence Libraries at Scale** (15 min)
   - Skills Library: `.claude/skills/` (16 skills)
   - Subagent Library: `.claude/agents/` (8 agents)
   - Spec Pattern Library: `specs/` (accumulated specifications)
   - Show how library compounds organizational capability

3. **Spec-as-Source Vision** (15 min)
   - Tessl framework overview (Chapter 33 deep dive preview)
   - Specifications as primary source (code regenerated from specs)
   - Organizational shift: From code-first to intent-first
   - Why SDD-RI matters for this future

4. **Practice: Design Simple Constitution** (25 min)
   - Students identify 3-5 principles for their team/project
   - Convert principles to decision frameworks (not rules)
   - Add self-monitoring prompts
   - Document as simple constitution.md

**AI Role**: Governance advisor (help design constitution)

**Teaching Modality**: Case study analysis + organizational design

**Cognitive Load Management**:
- 1 concept (Constitutions) synthesizes all prior concepts
- Case study reduces abstraction (see real constitution)
- Organizational scale (not individual practice)

**Forward References**:
- Chapter 31: SpecKit Plus hands-on (tooling deep dive)
- Chapter 32: AI Orchestration (agent teams, manager patterns)
- Chapter 33: Tessl framework (spec-as-source implementation)
- Part 6: AI Native Development (agent building with RI foundation)

**Validation Criteria**:
- [ ] Constitution case study thorough (show Section 0 + Principles)
- [ ] Intelligence Libraries explained (Skills, Subagents, Specs)
- [ ] Spec-as-source vision introduced (not deep dive, preview Ch 33)
- [ ] Practice exercise actionable (design simple constitution)
- [ ] Forward references clear (prepare for Chapters 31-33, Part 6)
- [ ] Chapter ending: Synthesis of SDD-RI journey

**Files to Create**:
- `08-organizational-patterns-and-governance.md`
- `assets/constitution-template.md` (starter template for students)

---

### Phase 4: Chapter-Level Updates

#### Update Chapter README

**File**: `book-source/docs/05-Spec-Driven-Development/30-specification-driven-development-fundamentals/README.md`

**Changes Required**:

```markdown
# Chapter 30: Understanding Spec-Driven Development with Reusable Intelligence

Spec-Driven Development with Reusable Intelligence (SDD-RI) bridges human creativity and AI precision: it turns vague ideas into executable contracts so coding agents can operate within clear boundaries and shared truths. By starting with specs—and making them reusable—we ensure AI coding agents build systems that are correct by design, traceable by memory, and governed by principle.

As AI-driven development matures, one thing is clear: its future will be written not in code, but in specifications and reusable intelligence.

Throughout this chapter you'll collaborate with your AI companion to co-create specifications, make them reusable through Skills and Subagents, and apply organizational patterns that scale to enterprise development.

## How to Work Through This Chapter

**You'll need**: Your AI coding assistant open alongside this book.

**The workflow**: When you see "Tell your companion:" or "Ask your companion:" prompts:
1. **Copy the prompt** and paste it into your AI tool
2. **Review the response** - does it make sense? Does it raise questions?
3. **Refine through dialogue** - ask follow-up questions, request clarifications
4. **Capture the results** - save collaborative specs and Skills in your project

This isn't passive reading—it's active collaboration. By the end, you'll have specifications, Skills, and even a simple Constitution you created with AI.

## What You'll Learn

By the end of this chapter you will be able to:

* **Diagnose vagueness**: Identify and quantify the real costs (time, rework, debugging, debt) caused by unclear requirements.
* **Write production-ready specs**: Produce executable specs with intent, requirements, acceptance criteria, and test scenarios.
* **Use AI as a spec partner**: Apply prompts and dialogues that reveal missing requirements, edge cases, and testable criteria.
* **Create reusable intelligence**: Design Skills and Subagents that encode recurring patterns using the Persona + Questions + Principles activation framework.
* **Apply organizational patterns**: Understand Constitutions, Intelligence Libraries, and how spec-as-source vision scales SDD-RI across teams.
* **Evaluate tools & patterns**: Choose appropriate tooling strategy for your team's scale and constraints.

## Chapter Structure (8 Lessons)

### Part 1: SDD Fundamentals (Lessons 1-5)
Foundation in writing individual specifications with AI collaboration.

### Part 2: Reusable Intelligence (Lessons 6-8)
Extension to organizational-scale patterns: Skills, Subagents, and Governance.
```

**Validation**:
- [ ] Chapter description reflects 8-lesson structure
- [ ] SDD-RI terminology throughout (not just "SDD")
- [ ] Learning objectives updated to include RI concepts
- [ ] Chapter structure shows 2-part progression (SDD → RI)

#### Update Chapter-Index.md

**File**: `specs/book/chapter-index.md`

**Changes Required**:

Find Chapter 30 entry (line ~87):
```markdown
| 30 | Understanding Spec-Driven Development | `30-specification-driven-development-fundamentals/` | ✅ Implemented |
```

Update to:
```markdown
| 30 | Understanding Spec-Driven Development with Reusable Intelligence (SDD-RI) | `30-specification-driven-development-fundamentals/` | ✅ Redesigned (8 lessons: SDD fundamentals + RI patterns) |
```

**Add note in Part 5 description**:
```markdown
## Part 5: Spec-Driven Development (Chapters 30-33)

Professional development workflow: specifications, reusable intelligence, and organizational patterns

**Note**: Chapter 30 expanded to 8 lessons (Jan 2025) to include Reusable Intelligence (RI) concepts—Skills, Subagents, and Constitutions—as prerequisites for Part 6 (AI Native Development).
```

**Validation**:
- [ ] Chapter 30 title updated to "SDD-RI"
- [ ] Status reflects redesign (not just "Implemented")
- [ ] Part 5 description includes RI concepts
- [ ] Prerequisite chain to Part 6 explicit

---

### Phase 5: Quality Validation

**Validation Checklist** (run after implementation):

#### Cognitive Load Validation
- [ ] Each lesson: Max 2 concepts ✅
- [ ] Complex lesson (L7): Extended time + heavy scaffolding ✅
- [ ] Progressive disclosure: Simple → Complex with scaffolding ✅
- [ ] Total chapter: 12 concepts across 8 lessons = 1.5 avg ✅

#### Pedagogical Arc Validation
- [ ] Foundation (L1-2): Manual practice ✅
- [ ] Application (L3-5): AI collaboration ✅
- [ ] Integration (L6-7): Reusability patterns ✅
- [ ] Validation (L8): Organizational synthesis ✅

#### Constitutional Compliance
- [ ] Principle 2 (Progressive Complexity): B1 limits respected ✅
- [ ] Principle 4 (Coherent Structure): Clear pedagogical arc ✅
- [ ] Principle 5 (Intelligence Accumulation): Prepares for Part 6 ✅
- [ ] Principle 6 (Anti-Convergence): Varied modalities ✅
- [ ] Section IIa (4-Stage Framework): Stages 1-3 mapped ✅
- [ ] Three Roles Framework: Demonstrated in L3 ✅

#### Content Quality Validation
- [ ] L1-5: Existing content updated (minor changes)
- [ ] L6-8: Net-new content implemented (full lessons)
- [ ] All code examples: Tested and verified
- [ ] All claims: Cited from authoritative sources
- [ ] Examples: Production-relevant (not toy apps)
- [ ] Practice exercises: Actionable and scaffolded

#### Prerequisite Chain Validation
- [ ] Internal dependencies: Linear (L1→L2→...→L8)
- [ ] External dependencies: Parts 1-4 satisfied
- [ ] Outgoing dependencies: Part 6+ prerequisites provided

#### Anti-Pattern Detection
- [ ] No lesson endings with "What's Next" / "Key Takeaways" / "Summary"
- [ ] No standalone "Safety Note" sections
- [ ] No "Stage 1/2/3" labels in student-facing text
- [ ] No "Three Roles Framework" as section header
- [ ] Single closure: "Try With AI" (L1-7), synthesis (L8)

---

## Implementation Timeline

**Estimated Effort**:
- Phase 1 (Preparation): 1 hour
- Phase 2 (L1-5 updates): 3-5 hours
- Phase 3 (L6-8 creation): 10-15 hours
- Phase 4 (Chapter updates): 1 hour
- Phase 5 (Validation): 2-3 hours

**Total**: 17-25 hours (2-3 working days)

**Critical Path**:
1. L3 (Three Roles validation) — must demonstrate bidirectional learning
2. L6 (RI introduction) — foundational for L7-8
3. L7 (P+Q+P workshop) — most complex lesson, requires careful scaffolding

---

## Success Criteria

**Implementation succeeds when**:
- ✅ All 8 lessons created/updated
- ✅ Cognitive load validation passes (B1 compliance)
- ✅ Constitutional compliance verified
- ✅ Prerequisite chain complete (Part 6 ready)
- ✅ Quality validation passes (content + pedagogy)
- ✅ Anti-pattern checks pass (no forbidden sections)

**Implementation fails if**:
- ❌ Any lesson exceeds 2 concepts (cognitive overload)
- ❌ L3 doesn't demonstrate Three Roles (constitutional violation)
- ❌ L6-8 missing or incomplete (Part 6 prerequisites unsatisfied)
- ❌ Examples are toy apps (not production-relevant)
- ❌ Anti-patterns present (standalone Safety Notes, "What's Next" sections)

---

## References

**Strategic Documents**:
- `specs/030-chapter-30-sdd-ri-restructure/adr-001-expand-chapter-30-to-include-ri.md`
- `specs/030-chapter-30-sdd-ri-restructure/lesson-plan.md`
- `specs/030-chapter-30-sdd-ri-restructure/concept-mapping.md`

**Constitutional Grounding**:
- `.specify/memory/constitution.md` v6.0.0 (all 7 principles)

**Examples for L6-7**:
- `.claude/skills/` (Skills library)
- `.claude/agents/` (Subagent library)

**Case Study for L8**:
- `.specify/memory/constitution.md` (governance example)

---

**This implementation guide provides step-by-step instructions for restructuring Chapter 30 from 5-lesson SDD to 8-lesson unified SDD-RI methodology while maintaining B1 cognitive load compliance and constitutional alignment.**
