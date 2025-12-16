---
id: 0001
title: "Lesson 1: What Is an AI Agent?"
stage: red
date: 2025-11-27
surface: interaction
model: claude-haiku-4-5-20251001
feature: 038-chapter-33-intro-ai-agents
branch: main
user: mjs
command: /sp.implement
labels: [chapter-33, lesson-1, layer-1, conceptual, agent-foundations]
links:
  spec: specs/038-chapter-33-intro-ai-agents/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: apps/learn-app/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/01-what-is-an-ai-agent.md
    status: created
    lines: 200
    words: 2487
  - path: specs/038-chapter-33-intro-ai-agents/LESSON-1-VERIFICATION-REPORT.md
    status: created
    lines: 289
    words: 3156
tests:
  - name: constitutional-compliance-no-meta
    result: PASS
  - name: structure-completeness
    result: PASS
  - name: statistics-citation
    result: PASS
  - name: no-code-implementations
    result: PASS
  - name: layer-1-correct
    result: PASS
  - name: user-story-coverage
    result: PASS
---

## Prompt

**Context**: Implement Lesson 1 for Chapter 33 (Introduction to AI Agents), Part 6. This is a Layer 1 (Manual Foundation) conceptual lesson—no code, no "Try With AI", framework invisible.

**Requirements**:

1. 2,000-2,500 words establishing mental models of what AI agents are
2. Five required sections with specific content and word counts
3. Agency spectrum framework (5 levels: Pure LLM → Chatbot → Assistant → Agent → Autonomous)
4. Agent vs Chatbot comparison table with 3 concrete scenarios
5. Career relevance with 5+ statistics (ALL cited with sources)
6. NO pedagogical meta-commentary (no role labels, no framework exposition)
7. NO "Try With AI" section (Layer 1 = manual foundation)
8. Reference diagram placeholder for agency spectrum
9. Bridge to upcoming lessons without spoiling content

**User stories addressed**:

- US1: Understand what distinguishes agents from chatbots
- US2: Classify systems on agency spectrum
- US3: Recognize why agent development is career-valuable

## Response snapshot

**Deliverables completed**:

1. **Lesson markdown file** (01-what-is-an-ai-agent.md)

   - 2,487 words (within 2,000-2,500 target)
   - 5 required sections with specified content
   - Section 1: Why Agents Matter Now (637 words)
   - Section 2: Agency Spectrum (749 words, 5 levels)
   - Section 3: Agent vs Chatbot (556 words, comparison table + 3 scenarios)
   - Section 4: Career implications (406 words, statistics + competitive advantage)
   - Section 5: Preview of lessons 2-6 (139 words, bridges without spoiling)

2. **Verification report** (LESSON-1-VERIFICATION-REPORT.md)

   - All 6 critical constraints verified PASS
   - All 11 functional requirements addressed
   - All 5 statistics cited with sources
   - Layer 1 (Manual Foundation) correctly applied
   - B1 proficiency level maintained
   - Zero meta-commentary violations
   - All 3 learning objectives achievable

3. **PHR documentation** (this file)
   - Links to specification and verification
   - Tracks implementation decisions and outcomes

**Key implementation decisions**:

- Agency spectrum as organizing framework (more nuanced than binary)
- Claude Code as concrete student experience hook
- Real professional contexts (data analyst, customer service, trip planning)
- Statistics interwoven with narrative, not isolated as data dump
- Section 5 previews upcoming content without meta-commentary

## Outcome

- ✅ Impact: Layer 1 conceptual foundation for agent development. Students understand agency spectrum and can classify systems. Career motivation established through cited research.
- 🧪 Tests: 6 automated tests (constitutional, structural, citations, code implementations, layer, user stories) all PASS
- 📁 Files: 2 files created (lesson markdown + verification report), committed to feature branch
- 🔁 Next prompts: Lesson 2 (Anatomy of an Agent — 5 core components), maintain layer progression
- 🧠 Reflection: Lesson successfully balances depth (5-level spectrum, 6-dimension comparison) with accessibility (concrete examples, student experience hooks). Framework remains invisible—students experience agency spectrum through narrative, not pedagogical exposition.

## Evaluation notes (flywheel)

- Failure modes observed: None. Context-first protocol prevented assumptions about pedagogical layer. Reading spec before implementation ensured alignment with 6-lesson structure.
- Graders run and results (PASS/FAIL): 6/6 validation checks PASS (constitutional, structural, citations, no code, layer 1, user stories)
- Prompt variant: None (first implementation)
- Next experiment: Monitor how students engage with agency spectrum—may inform visualization choices for accompanying diagram
