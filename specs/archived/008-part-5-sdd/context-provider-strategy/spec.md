# Feature Specification: Context Architecture for AI Agents in SDD

**Feature ID**: context-provider-strategy
**Status**: ✅ IMPLEMENTED
**Date**: 2025-11-01

---

## Intent

Add a **new lesson to Chapter 32** (Lesson 6) that teaches how to architect AI agents to stay current with evolving APIs and libraries. This addresses the gap between agent knowledge cutoffs and real-world API changes.

**Problem Solved**:
- Developers write specs saying "use latest API" but agents have knowledge cutoffs
- APIs evolve faster than agent knowledge
- Generated code can be stale on day 1

**Solution**:
- Teach context architecture as a SPECIFICATION DECISION
- Present three major strategies: Context7, Tessel Registry, Hybrid approaches
- Provide decision framework for choosing the right approach
- Integrate into capstone project planning

---

## Scope

### In Scope
✅ Context7 (MCP-based live documentation)
✅ Tessel Registry (spec-driven library discovery)
✅ Hybrid approaches (RAG, embedded, memory, version pinning)
✅ Decision matrix for choosing strategy
✅ Real example: Stripe integration with Context7
✅ Integration into capstone project

### Out of Scope
❌ Implementation details of Context7 or Tessel (reference only)
❌ Building custom RAG systems (overview only)
❌ Comparing other context providers not mentioned
❌ Deep dive into MCP protocol (already taught in Part 2)

---

## Key Concepts

1. **Knowledge Cutoff Problem**
   - Agent knowledge is frozen at training date
   - APIs evolve continuously after that date
   - Specs say "latest" but agents can't deliver "latest"

2. **Context Architecture**
   - Strategic decision about how agents access current knowledge
   - Part of specification, not implementation detail
   - Affects code quality, maintenance, and correctness

3. **Three Strategies**
   - Context7: Live documentation via MCP
   - Tessel: Spec-driven machine-readable specs
   - Hybrid: RAG, embedded, memory, versioning

4. **Decision Framework**
   - How often does API change?
   - How critical is correctness?
   - Is library Tessel-enabled?
   - Do you have offline requirements?
   - What's your infrastructure appetite?

---

## Learning Outcomes

By completing this lesson, students will:

1. **Understand** why agent knowledge cutoffs matter for specifications
2. **Identify** when context architecture is a critical design decision
3. **Evaluate** three major strategies for keeping agents current
4. **Choose** appropriate context architecture for their capstone
5. **Implement** context strategy reference in their capstone specification

---

## Content Structure

### Part 1: The Problem (15 minutes)
- Real scenario: Stripe integration failing because API changed
- Why this is a specification problem
- How context architecture is part of SDD

### Part 2: Strategy 1 - Context7 (45 minutes)
- What is Context7 (MCP-based live docs)
- How it works with Claude Code
- Real spec example with Context7 reference
- When to use, when not to use
- Hands-on: Write Stripe spec using Context7

### Part 3: Strategy 2 - Tessel Registry (45 minutes)
- What is Tessel (from Chapter 30 recap)
- Machine-readable library specs
- How code generators use Tessel specs
- When to use (safety-critical systems)
- Reflection: Tessel future promise

### Part 4: Strategy 3 - Hybrid Approaches (45 minutes)
- Option A: Embedded documentation
- Option B: RAG (vector search)
- Option C: Agent memory
- Option D: Version pinning
- Trade-offs for each approach

### Part 5: Decision Framework (30 minutes)
- Matrix: Which strategy for which scenario
- Questions to ask your context
- Exercise: Assess your capstone's needs
- Commitment: Choose strategy for capstone

---

## Connection to Constitution

**Principles this teaches**:
1. **Principle 1: Specifications Precede Implementation**
   - Context architecture IS a specification decision

2. **Principle 3: Validation-First Safety**
   - Validating that agents have current knowledge

3. **Principle 6: AI as Co-Reasoning Partner**
   - Designing the agent's knowledge environment

4. **Principle 14: Planning-First Development**
   - Planning includes: How will agents stay current?

---

## Integration Points

**Prerequisites**:
- Chapter 30: Understanding SDD (spec-as-source concept)
- Chapter 31: Hands-on spec writing
- Chapter 32, Lessons 1-5: Team scaling (context for production systems)
- Part 2, Lesson 5: MCP servers (context for understanding Context7)

**Follows**:
- Lesson 5: SDD In the Wild (real companies use specs)

**Precedes**:
- Lesson 7: Write Professional Commitment (incorporate context strategy)
- Capstone Part 1: Decompose Spec (with context architecture included)

---

## Deliverables

✅ **Lesson file**: `06-how-agents-stay-current-context-architecture-for-living-specs.md`
✅ **Chapter 32 README updated**: New lesson documented
✅ **Part 5 README updated**: Time estimates and topic list
✅ **Part 5 total**: 51-64 hours (updated from 45-59)

---

## Teaching Method

**AI-Native Pedagogy**:
- Problem-centered (knowledge cutoff failing specs)
- Discovery through dialogue (ask your companion questions)
- Real examples (Stripe, FastAPI)
- Decision-making emphasis (choose YOUR approach)
- Integration into capstone (apply immediately)

---

## Success Criteria

✅ Students understand why context architecture matters
✅ Students can identify when it's a critical decision
✅ Students can compare and contrast three strategies
✅ Students can choose appropriate approach for their context
✅ Students incorporate context strategy into capstone plan
✅ Lesson integrates seamlessly before capstone

---

## Implementation Status

**Status**: ✅ COMPLETE

**Files Created**:
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/32-real-world-spec-kit-workflows/06-how-agents-stay-current-context-architecture-for-living-specs.md`

**Files Updated**:
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/32-real-world-spec-kit-workflows/README.md`
- `/book-source/docs/05-Spec-Kit-Plus-Methodology/README.md`

**Chapter 32 Structure After Update**:
1. Lesson 1: Watch Your Companions Coordinate
2. Lesson 2: Design Team Workflows
3. Lesson 3: Trace SDD Through Your Company
4. Lesson 4: See How Specs Flow Through Everything
5. Lesson 5: SDD In the Wild
6. **[NEW] Lesson 6: How Agents Stay Current** ← You are here
7. Lesson 7: Write Your Professional Commitment
8-10. Capstone Parts 1-3

---

## Next Steps

1. Review and validate lesson content
2. Optionally create corresponding spec document for this lesson
3. Plan testing with real examples (Context7 instance, Tessel registry)
4. Consider creating supplementary materials (decision tree, architecture diagram)
5. Update book index with new lesson

---

## Questions Answered

**Q: Why before capstone?**
A: Students need this architectural knowledge to design their capstone specifications thoughtfully. They should make explicit choices about context architecture in Lesson 6, then implement them in the capstone.

**Q: Why is this part of SDD?**
A: Specifications are only useful if agents can act on them with current knowledge. Context architecture is the mechanism that keeps that promise.

**Q: How is this different from Part 2 (MCP)?**
A: Part 2 teaches HOW to use MCP servers. This lesson teaches WHY context architecture matters and HOW to choose between strategies.

**Q: Will this be relevant in 2026+?**
A: Yes. As APIs evolve and agents improve, the question of "how do agents stay current" only becomes more important. This framework will remain relevant.
