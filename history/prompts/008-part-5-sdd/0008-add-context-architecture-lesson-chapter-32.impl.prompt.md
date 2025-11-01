---
id: 0008
title: "Add Context Architecture Lesson to Chapter 32"
stage: impl
date: 2025-11-01
surface: agent
model: claude-haiku-4-5-20251001
feature: 008-part-5-sdd
branch: 008-part-5-sdd
user: mjs
command: Manual implementation (new lesson + documentation)
labels: [chapter32, context-architecture, agent-knowledge, context7, tessel, context-strategy]
links:
  spec: specs/context-provider-strategy/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - docs/05-Spec-Kit-Plus-Methodology/32-real-world-spec-kit-workflows/06-how-agents-stay-current-context-architecture-for-living-specs.md
  - docs/05-Spec-Kit-Plus-Methodology/32-real-world-spec-kit-workflows/README.md
  - docs/05-Spec-Kit-Plus-Methodology/README.md
  - specs/context-provider-strategy/spec.md
tests:
  - Lesson 6 written (3-3.5 hours, ~5,600 words)
  - Chapter 32 README updated with lesson description
  - Part 5 README updated with duration and topics
  - Specification document created
  - All files reviewed for content accuracy and structure
---

## Prompt

Add a new Lesson 6 to Chapter 32 (Real-World Spec-Kit Workflows) teaching how to architect AI agents to stay current with evolving APIs. This addresses the gap between agent knowledge cutoffs and real-world API changes. Present Context7 (live docs), Tessel Registry (spec-driven), and hybrid approaches with decision framework. Position BEFORE the capstone so students make conscious choices about context architecture when designing their capstone projects.

## Response snapshot

**Added Lesson 6: How Agents Stay Current — Context Architecture for Living Specs**

**Deliverable**:
- New lesson file: `06-how-agents-stay-current-context-architecture-for-living-specs.md`
- Duration: 3-3.5 hours, ~5,600 words
- Strategic positioning: Before capstone (Lesson 6 of 7 lessons)

**Lesson Structure**:
- **Part 1**: The Problem (15 min) — APIs evolve faster than agent knowledge cutoffs
- **Part 2**: Strategy 1 - Context7 (45 min) — Live documentation via MCP
- **Part 3**: Strategy 2 - Tessel Registry (45 min) — Spec-driven library discovery
- **Part 4**: Strategy 3 - Hybrid Approaches (45 min) — RAG, embedded, memory, versioning
- **Part 5**: Decision Framework (30 min) — Choose the right strategy for your context

**Key Content**:
- Real example: Stripe integration failing because API changed
- Three major strategies with pros/cons
- Decision matrix for choosing approach
- Real Stripe example using Context7
- Hands-on exercises and reflection
- Integration into capstone planning

**Pedagogical Approach**:
- Problem-centered opening (knowledge cutoff failing specs)
- Ask companion questions
- Real examples (Stripe, FastAPI)
- Decision-making emphasis
- Integration into capstone

**Connection to SDD**:
- Context architecture is a SPECIFICATION DECISION
- Completes the story: specifications must stay CURRENT
- Bridges Part 2 (MCP) → Part 5 (SDD) → Part 6+ (Agent Architecture)
- Teaches that choosing how agents access knowledge is strategic

**Strategic Value**:
- First book content to teach context architecture as part of SDD
- Integrates Context7, Tessel, hybrid approaches into curriculum
- Addresses real problem every AI developer faces
- Differentiates from other educational content
- Prepares students for production-ready agent systems

**Documentation Updates**:
- Chapter 32 README updated with Lesson 6 description
- Part 5 README updated: Chapter 32 now 18-20 hours (added 3-3.5h)
- Part 5 total: 51-64 hours (updated from 45-59)
- New specification document created for context strategy

**Status**: ✅ Complete, integrated, ready for validation

**Future Opportunities**:
- Advanced topics: RAG implementation, agent memory systems
- Specific tutorials: Setting up Context7, using Tessel registry
- Part 6 (Agent Architecture): Deeper dive into context strategies

