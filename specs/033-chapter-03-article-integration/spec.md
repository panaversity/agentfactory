# Feature Specification: Chapter 3 Lesson 6 Article Integration

**Feature Branch**: `033-chapter-03-article-integration`
**Created**: 2025-12-06
**Status**: In Progress
**Input**: Integrate "AI Is Quietly Creating Millionaires" (Entrepreneur.com, Simeon Ivanov) into Chapter 3 Lesson 6 to reinforce why domain expertise creates competitive moat

---

## User Scenarios & Testing

### User Story 1 - Students Connect Domain Expertise to Economic Value (Priority: P1)

Students understand that domain expertise isn't just "nice to have"—it's the economic difference between a feature and a billion-dollar business.

**Why this priority**: Foundation for understanding why all three requirements are necessary. Without connecting expertise to economic defensibility, students don't understand stakes.

**Independent Test**: Student can explain in 2-3 sentences WHY domain expertise creates defensible advantage (not just HOW to build it). Passes if student articulates economic moat concept, not technical implementation.

**Acceptance Scenarios**:

1. **Given** student reads "Why Intelligence Is the New Competitive Asset" subsection, **When** asked "Why does a competitor struggle to replicate your domain intelligence?", **Then** student explains: "Months to build knowledge base + embedded in every decision + competitive moat because they must replicate the entire system"

2. **Given** previous lessons (Lesson 3: super orchestrator economics), **When** student synthesizes, **Then** student connects: "Super orchestrators win through accumulated intelligence, not effort—that intelligence is what we need to be defensible"

---

### User Story 2 - Summary Reflects Article Insights (Priority: P2)

Update 06-three-requirements.summary.md to include article perspective, helping students review quickly.

**Why this priority**: Summary is student's quick reference. If article insights missing, students don't carry forward understanding of intelligence-as-asset.

**Independent Test**: Summary includes intelligence-as-moat concept and connects to fine-tuned models/vertical intelligence defensibility.

**Acceptance Scenarios**:

1. **Given** student reviews summary after lesson, **When** checking "Why three requirements matter", **Then** finds statement about intelligence as competitive asset

2. **Given** student preparing for capstone project selection, **When** rereading summary, **Then** understands "intelligence = moat" shapes their vertical choice

---

## Requirements

### Functional Requirements

- **FR-001**: New subsection "Why Intelligence Is the New Competitive Asset" placed after "Path 2: Vertical Reusable Intelligence" section and before "Choosing Your Path"
- **FR-002**: Subsection is 300-500 words (target: 400 words) with A1-A2 language (no jargon, accessible explanations)
- **FR-003**: Subsection must introduce 0-2 NEW concepts beyond existing 5 (keeps total at or below 7, respecting A2 cognitive load limit)
- **FR-004**: Article insights extracted and reframed in educational tone (remove entrepreneurial hype, retain economic insight)
- **FR-005**: Subsection connects explicitly to Lesson 3 super orchestrator economics (reference or summarize)
- **FR-006**: Subsection maintains Layer 1 (Manual Foundation) pedagogical approach—no "Try With AI" prompts within subsection
- **FR-007**: Summary file (06-three-requirements.summary.md) updated with intelligence-as-moat insight in "Key Mental Models" section

### Content Requirements

**Article Source**: "AI Is Quietly Creating Millionaires" (Entrepreneur.com, Simeon Ivanov)
**Core Thesis to Integrate**:
- Intelligence (not effort) is the new wealth-creation mechanism
- Systems-based wealth ("systems build empires") vs effort-based wealth
- Early adopter advantage through accumulated knowledge
- Domain expertise becomes defensible competitive asset

**Concepts from Article**:
1. Intelligence as new asset class (replacing effort/manual work as wealth driver)
2. Defensibility through accumulated systems (takes months to replicate)
3. Early mover advantage in applying AI to vertical domains
4. Fine-tuned models and vertical intelligence as embodied competitive advantage

**Reframing Requirement**: Article is written for entrepreneurs (wealth-building focus). Must reframe for students in educational institution (learning-focused):
- **From**: "Build an AI system and become a millionaire"
- **To**: "Understanding why domain expertise is non-negotiable for building defensible solutions"

---

## Success Criteria

### Measurable Outcomes (Educational)

- **SC-001**: Cognitive load audit: New subsection introduces ≤2 new concepts. Total lesson remains at 5-7 concepts (A2 limit). Measured by concept count review.

- **SC-002**: A1-A2 Language Compliance: No unexplained jargon (model fine-tuning explained, skill library concepts grounded in lesson, "competitive moat" concept explained in plain language). Measured by vocabulary audit.

- **SC-003**: Connection to Prior Learning: Subsection explicitly references Lesson 3 super orchestrators. Student who read Lesson 3 sees continuity; new reader can understand context. Measured by explicit cross-reference check.

- **SC-004**: Constitutional Compliance:
  - Zero meta-commentary (no "What to notice", no "AI learns from you" exposition)
  - Layer 1 approach maintained (direct explanation + examples, no hidden pedagogy)
  - Specification primacy (establishes WHAT and WHY before implementation details)
  - Measured by grep validation + human review

- **SC-005**: Article Integration Authenticity: Article insights reflected (intelligence as asset, defensibility through system complexity, early adopter advantage), not just token mentions. Measured by conceptual alignment review.

- **SC-006**: Summary Consistency: Updated summary contains intelligence-as-moat language that mirrors lesson's new subsection. Measured by content comparison.

### Non-Goals (What We're NOT Doing)

- NOT teaching model fine-tuning technical details (that's Part 8 territory, not Part 1)
- NOT adding new sections about specific AI techniques—leveraging existing framework
- NOT redesigning lesson structure—targeted subsection addition only
- NOT creating "Try With AI" exercises within new subsection (Layer 1 stays manual/conceptual)

---

## Context & Constraints

**Chapter Context**:
- Part 1: Introducing AI-Driven Development (conceptual foundation)
- Chapter 3: Strategic market opportunity (Snakes & Ladders → Super Orchestrators → Three Requirements)
- Lesson 6: Synthesizes strategy with execution requirements

**Lesson-Level Constraints**:
- Proficiency: A1-A2 (beginner audience, no programming prerequisites)
- Duration: Currently 12 minutes. Addition should NOT exceed 2 minutes (keep total ~14 min)
- Learning Objectives: Already defined—addition must serve existing objectives, not add new ones
- Cognitive Load: Already at 5 new concepts. Addition can add ≤2 without exceeding A2 limit of 7
- Pedagogical Layer: Layer 1 (Manual Foundation)—explains directly, builds mental models, no hidden scaffolding

**Article Source Constraint**:
- Must be publicly verifiable (Entrepreneur.com, author Simeon Ivanov, published 2025)
- Can quote directly (with attribution) or paraphrase with citation
- Must reframe entrepreneurial angle in educational tone without losing core insight

---

## Architectural Alignment

This subsection serves as **capstone insight for three-requirements framework** before students choose between two paths to domain expertise. It establishes:

1. **Why it matters** (intelligence as competitive asset—connecting to economics from Lesson 3)
2. **Why both paths work** (fine-tuned models AND vertical intelligence both defensible, both require months to replicate)
3. **Why missing any requirement fails** (follows from understanding that intelligence is not easily replicated)

This is **not a new requirement**—it's the WHY underlying Requirement 1 (domain expertise) that was previously assumed.

---

## Acceptance Checklist

Before implementation approval, verify:

- [ ] Spec is complete (no NEEDS CLARIFICATION placeholders)
- [ ] Constraints explicitly stated (what we ARE and ARE NOT doing)
- [ ] Success criteria are measurable (can be objectively verified)
- [ ] Article source is verifiable
- [ ] Concept reframing from entrepreneurial→educational is clear
- [ ] Non-goals defined (prevents scope creep)
- [ ] Ready for planning phase

