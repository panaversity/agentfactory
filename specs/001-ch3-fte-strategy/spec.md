# Feature Specification: Chapter 3 Digital FTE Strategy Restructure

**Feature Branch**: `001-ch3-fte-strategy`
**Created**: 2025-12-25
**Status**: Draft
**GitHub Issue**: #386
**Input**: Restructure Chapter 3 from "Billion Dollar AI" to "The Digital FTE Strategy" to align with Agent Factory vision

---

## Assumed Knowledge

**What students know BEFORE this chapter**:
- Agent Factory paradigm and core vision (Chapter 1, Lessons 1-10)
- General Agents vs Custom Agents distinction (Chapter 1, Lesson 3)
- Digital FTE concept and value proposition (Chapter 1, Lesson 10)
- From Coder to Orchestrator role shift (Chapter 1, Lesson 4)
- Spec-Driven Development preview (Chapter 1, Lesson 9)
- OODA loop and reasoning architecture (Chapter 1, Lesson 3)
- Vibe Coding vs SDD distinction (Chapter 1, Lesson 10)
- AIFF Foundation standards (Chapter 2 - concurrent)

**What this chapter must explain from scratch**:
- How to position YOUR specific expertise as a Digital FTE product
- Competitive strategy layers (Snakes & Ladders framework)
- The FTE revenue advantage vs human labor economics
- Agent Skills and SKILL.md as monetizable assets
- Four monetization models (Subscription, Success Fee, License, Marketplace)
- PPP (Piggyback-Protocol-Pivot) market entry strategy
- Three requirements for Digital FTE success
- When NOT to use AI agents (guardrails and pitfalls)

**Proficiency Level**: A2 (Building on A1 foundations from Chapter 1)

---

## User Scenarios & Testing

### User Story 1 - Learn Monetization Strategy (Priority: P1)

A domain expert who completed Chapter 1 wants to understand HOW to monetize their expertise through Digital FTEs. They need clear revenue models with examples.

**Why this priority**: This is the core value proposition of Chapter 3 - turning conceptual understanding into actionable business strategy.

**Independent Test**: Reader can articulate which monetization model fits their domain and explain why, with a concrete pricing example.

**Acceptance Scenarios**:

1. **Given** a reader who understands the Digital FTE concept, **When** they complete Lesson 5 (Four Monetization Models), **Then** they can select an appropriate model for their domain with justified reasoning
2. **Given** a reader with sales background, **When** they read the FTE Advantage lesson, **Then** they can construct a client pitch comparing Digital FTE to human FTE costs
3. **Given** a reader in healthcare/finance, **When** they review the License model section, **Then** they understand why data sovereignty drives this choice

---

### User Story 2 - Build Competitive Strategy (Priority: P2)

A developer-entrepreneur wants to understand competitive dynamics in the AI agent market. They need frameworks for positioning and market entry.

**Why this priority**: Strategy frameworks (Snakes & Ladders, PPP) differentiate this chapter from generic AI content.

**Independent Test**: Reader can map their current position on the competitive layers and identify their next strategic move.

**Acceptance Scenarios**:

1. **Given** a reader at Layer 1 (commodity), **When** they study the Snakes & Ladders framework, **Then** they can identify which "ladder" moves them to Layer 2-3
2. **Given** a reader planning market entry, **When** they complete PPP Strategy lesson, **Then** they can outline a Piggyback → Protocol → Pivot path for their domain
3. **Given** a reader evaluating opportunities, **When** they apply Three Requirements framework, **Then** they can assess domain fit systematically

---

### User Story 3 - Understand Agent Skills as Assets (Priority: P2)

A technical reader wants to understand how Agent Skills (SKILL.md) become portable, sellable intellectual property.

**Why this priority**: Agent Skills are the bridge between conceptual understanding and practical implementation.

**Independent Test**: Reader can describe the SKILL.md format and explain progressive disclosure benefits.

**Acceptance Scenarios**:

1. **Given** a reader familiar with manual prompting, **When** they study the Vertical Intelligence lesson, **Then** they understand why SKILL.md creates reusable IP
2. **Given** a reader building agents, **When** they see the SKILL.md format specification, **Then** they understand folder structure and token efficiency
3. **Given** a reader considering licensing, **When** they connect skills to monetization, **Then** they see skills as licensable "expertise packs"

---

### User Story 4 - Learn Guardrails and Risk Management (Priority: P3)

A cautious reader wants to understand when NOT to deploy AI agents and how to avoid common pitfalls.

**Why this priority**: Risk awareness prevents costly mistakes and builds credibility with enterprise clients.

**Independent Test**: Reader can identify red flags in a proposed agent deployment scenario.

**Acceptance Scenarios**:

1. **Given** a scenario with undefined success criteria, **When** reader applies the "When NOT to Use" framework, **Then** they correctly flag it as unsuitable for automation
2. **Given** a list of common pitfalls, **When** reader reviews their own project plans, **Then** they can identify at least 2 potential risks
3. **Given** enterprise compliance requirements, **When** reader studies security layers, **Then** they understand minimum requirements for production deployment

---

### Edge Cases

- What happens when a reader skipped Chapter 1? → Lesson 1 must provide callback references without repeating content
- How do we handle readers only interested in technical skills (not business)? → Vertical Intelligence lesson serves as bridge
- What if reader's domain doesn't fit any monetization model cleanly? → Include "hybrid approaches" discussion

---

## Requirements

### Functional Requirements

**Structural Requirements**:
- **FR-001**: Chapter folder MUST be renamed from `03-billion-dollar-ai/` to `03-digital-fte-strategy/`
- **FR-002**: Chapter MUST contain exactly 11 lessons (expanded from current 8; monetization split into Part 1 & Part 2 per plan.md cognitive load analysis)
- **FR-003**: All lesson files MUST follow existing naming convention (##-lesson-name.md)
- **FR-004**: All .summary.md companion files MUST be regenerated for updated lessons
- **FR-005**: README.md MUST be updated with new chapter structure and learning objectives

**Content Requirements - Existing Lessons (Update)**:
- **FR-010**: Lesson 1 MUST be reframed from "Billion Dollar Question" to "Your Expertise as Product"
- **FR-011**: Lesson 1 MUST remove content redundant with Chapter 1 (market opportunity framing)
- **FR-012**: Lesson 1 MUST add explicit callback to Chapter 1 context
- **FR-013**: Lesson 2 (Snakes and Ladders) MUST retain unique framework with minor terminology alignment
- **FR-014**: Lesson 3 MUST be reframed from "Super Orchestrators" to "The FTE Advantage"
- **FR-015**: Lesson 3 MUST remove Digital FTE comparison table (now in Chapter 1, Lesson 10)
- **FR-016**: Lesson 3 MUST add sales presentation tips from Agent Factory slides (Page 72-73)
- **FR-017**: Lesson 4 (Vertical Intelligence) MUST incorporate Agent Skills/SKILL.md content
- **FR-018**: Lesson 4 MUST include SKILL.md format specification from slides (Page 48)
- **FR-019**: Lesson 4 MUST explain progressive disclosure and token efficiency (Page 50)
- **FR-020**: Lesson 6 (PPP Strategy) MUST retain unique framework with minor edits
- **FR-021**: Lesson 7 (Three Requirements) MUST be renumbered and retain content

**Content Requirements - New Lessons**:
- **FR-030**: NEW Lesson 5 MUST cover Monetization Models Part 1 (Subscription, Success Fee) from slides Part 11
- **FR-031**: NEW Lesson 6 MUST cover Monetization Models Part 2 (License, Marketplace) from slides Part 11
- **FR-032**: Lessons 5-6 MUST include decision matrix for model selection (complete matrix in Lesson 6)
- **FR-033**: Lessons 5-6 MUST include real examples (Digital SDR case study in Lesson 5)
- **FR-034**: NEW Lesson 9 MUST cover "When NOT to Use AI Agents"
- **FR-035**: Lesson 9 MUST include security and compliance framework from slides Part 15
- **FR-036**: Lesson 9 MUST include common pitfalls table (6 pitfalls with fixes)
- **FR-037**: Lesson 9 MUST include "shadow mode" deployment strategy

**Content Requirements - Synthesis Lessons**:
- **FR-040**: Lesson 10 (Pause and Reflect) MUST be updated with monetization-focused prompts
- **FR-041**: Lesson 11 (Quiz) MUST add 15 new questions covering new content areas
- **FR-042**: Quiz MUST cover: monetization models, Agent Skills, guardrails, competitive strategy

**Quality Requirements**:
- **FR-050**: All lessons MUST follow 4-Layer Teaching Method invisibly
- **FR-051**: No meta-commentary exposing pedagogical framework
- **FR-052**: Each lesson MUST include "Try With AI" section
- **FR-053**: Zero content overlap with Chapter 1 lessons

### Key Entities

- **Lesson**: A markdown file containing educational content (~2000-3000 words)
- **Summary**: Companion .summary.md file with key takeaways
- **Chapter README**: Overview file with structure and learning objectives
- **Quiz**: Assessment file with questions and answer key

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: Chapter contains exactly 11 lessons with correct numbering and naming
- **SC-002**: Zero content redundancy with Chapter 1 (verified by diff comparison)
- **SC-003**: All 4 monetization models clearly explained with at least 1 example each
- **SC-004**: SKILL.md format specification included with folder structure diagram
- **SC-005**: "When NOT to Use" lesson includes minimum 6 specific scenarios
- **SC-006**: Quiz contains minimum 15 new questions on Chapter 3 content
- **SC-007**: All 11 lessons have corresponding .summary.md files
- **SC-008**: All internal cross-references updated (no broken links)
- **SC-009**: README reflects new chapter structure with accurate lesson titles
- **SC-010**: Build passes with no errors (`pnpm nx build learn-app`)

---

## Scope & Boundaries

### In Scope
- Rename chapter folder
- Update 6 existing lessons
- Create 2 new lessons
- Update quiz with new questions
- Update chapter README
- Regenerate all .summary.md files

### Out of Scope
- Changes to Chapter 1 or Chapter 2
- Changes to other parts of the book
- Adding code examples (this is a conceptual/strategy chapter)
- Modifying sidebars or navigation beyond this chapter
- Changes to the preface

---

## Dependencies

- **Chapter 1 (Agent Factory Paradigm)**: COMPLETED - provides foundational concepts
- **Chapter 2 (AIFF Foundation)**: IN PROGRESS (#385) - independent, no blocking dependency
- **Agent Factory Slides**: Source material for new content (available as markdown)

---

## Assumptions

1. Existing lesson quality is acceptable as baseline (we're updating, not rewriting from scratch)
2. Summary files can be regenerated using existing summary-generator skill
3. The Agent Factory markdown document contains all needed source content
4. No changes to Docusaurus configuration are required
5. Sidebar positions will auto-update based on file ordering

---

## Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Broken cross-references after rename | Medium | Search codebase for old paths before rename |
| Content overlap with Chapter 2 AIFF | Low | Chapter 2 covers standards; Chapter 3 covers strategy |
| Scope creep into other chapters | Medium | Strict adherence to spec boundaries |
| Summary regeneration quality | Low | Manual review of generated summaries |
