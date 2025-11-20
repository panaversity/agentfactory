# Feature Specification: Part 6 README (AI Native Software Development)

**Feature Branch**: `026-part-6-readme`
**Created**: 2025-01-18
**Status**: Draft
**Input**: Create Part 6 README (AI Native Software Development, 16 chapters). Target: developers who completed Parts 1-5 (AIDD fundamentals, Python, Spec-Driven Dev). Goals: (1) position Part 6 as transition from "learning to code" to "building production agents", (2) preview chapter progression (Agents SDK → MCP → FastAPI → TDD/Evals → Design Patterns → Databases), (3) clarify 4-layer teaching methodology applies throughout, (4) set expectations for capstone projects at chapter ends. Tone: bridge between education and production engineering.

---

## Evals (Success Criteria)

### Measurable Outcomes

- **SC-001**: **Clarity Test** - 3 independent reviewers (unfamiliar with Part 6 content) can accurately answer: "What will I learn in Part 6?" after reading README (80%+ accuracy threshold)

- **SC-002**: **Tone Verification** - README balances educational and professional language (verified by keyword analysis: neither educational nor professional language falls below 40% of total positioning language)

- **SC-003**: **Prerequisite Clarity** - Readers can identify required prerequisites (Parts 4-5) vs recommended background (Parts 1-3) without ambiguity (verifiable via comprehension test)

- **SC-004**: **Chapter Grouping Logic** - Readers can explain WHY chapters are sequenced (Agents SDK → MCP → FastAPI → TDD → Patterns → Databases) in their own words after reading (pedagogical logic clarity test)

- **SC-005**: **Expectation Alignment** - Capstone project scope is concrete enough that readers can distinguish realistic scope from unrealistic scope (binary test: "Will capstones be production-deployed?" correct answer: No, local deployment demonstrations)

- **SC-006**: **Factual Accuracy** - Zero discrepancies between README chapter references (numbers, titles, themes) and chapter-index.md (automated verification)

- **SC-007**: **Minimal Content Compliance** - README contains ONLY essential sections (Overview, Goals, Chapter Progression, Methodology Note, Capstone Expectations, Getting Started). No forbidden sections (Why This Matters, Success Stories, FAQ, What's Next After Part 6)

- **SC-008**: **Readability** - README is scannable in under 3 minutes for experienced developers (timed reading test, comprehension validation)

### Qualitative Outcomes

- **QC-001**: README avoids hype language ("amazing", "powerful", "cutting-edge") and over-promising ("build production systems in weeks")

- **QC-002**: README maintains student confidence by framing prerequisites as assets built ("you've learned X") not barriers ("you must have X before proceeding")

- **QC-003**: 4-layer methodology is described naturally through student experience ("chapters introduce concepts manually, then guide AI collaboration, then build reusable components, then apply them in capstone projects") without exposing internal scaffolding labels ("Stage 1/2/3/4")

---

## Intent

### What

A positioning document (README.md) for Part 6 of "AI Native Software Development" book that bridges students from foundational learning (Parts 1-5: AIDD fundamentals, Python, Spec-Driven Development) to production agent development (Part 6: 16 chapters on Agents SDK, MCP, FastAPI, TDD/Evals, Design Patterns, Databases).

### Why

**Problem**: Students completing Parts 1-5 face cognitive transition from educational environment (learning programming fundamentals) to professional environment (building production-ready AI agents). Without clear positioning, they may:
- Experience imposter syndrome ("Am I ready for production patterns?")
- Misunderstand chapter progression logic (why Agents SDK before FastAPI?)
- Have unrealistic expectations (capstones = production deployment)
- Fail to recognize methodology continuity (4-layer teaching continues)

**Solution**: README provides essential context that:
1. Validates readiness ("you've built Python skills and spec-driven methodology")
2. Explains pedagogical progression (why chapters sequence this way)
3. Sets realistic capstone scope (learning demonstrations, not production deployment)
4. Signals continued teaching support (4-layer methodology persists)

### Who

**Primary Audience**: Developers who completed Parts 1-5 and are about to start Part 6

**Audience Characteristics**:
- **Skill Level**: B2 (intermediate Python, comfortable with AI collaboration, familiar with spec-driven development)
- **Transition State**: Moving from "learning to code with AI" to "building production agents"
- **Needs**: Clarity about what changes, what stays same, what's expected
- **Concerns**: "Am I ready?", "Will this be overwhelming?", "What will I actually build?"

**Secondary Audience**: Course administrators, instructors previewing Part 6 curriculum

---

## User Scenarios & Testing

### User Story 1 - Validate Readiness and Transition Clarity (Priority: P1)

A developer named Sarah completed Parts 1-5 (chapters 1-33). She opens Part 6 README to understand if she's ready and what to expect.

**Why this priority**: Core positioning function. If students can't determine readiness and understand transition, README fails primary purpose.

**Independent Test**: Sarah can answer "What skills from Parts 1-5 enable me for Part 6?" and "What's different in Part 6 vs Parts 1-5?" after reading Overview and Goals sections only.

**Acceptance Scenarios**:

1. **Given** Sarah completed Part 5 (chapters 30-33 on spec-driven development), **When** she reads README Overview section, **Then** she understands Part 6 builds on Python (Part 4) and spec-driven methodology (Part 5) as core prerequisites

2. **Given** Sarah is concerned about difficulty jump, **When** she reads tone and positioning language, **Then** she recognizes continuity ("bridge" tone: educational support continues, complexity increases gradually) without feeling gatekept

3. **Given** Sarah wants to know what's new, **When** she scans Goals section, **Then** she identifies concrete outcomes: "implement production agent patterns", "practice testing strategies", "understand design tradeoffs" (professional language mixed with educational support)

---

### User Story 2 - Understand Chapter Progression Logic (Priority: P2)

Sarah wants to understand WHY chapters progress as: Agents SDK → MCP → FastAPI → TDD/Evals → Design Patterns → Databases (not random sequence).

**Why this priority**: Pedagogical transparency builds trust and helps students see learning arc, but secondary to readiness validation.

**Independent Test**: Sarah can explain chapter grouping logic in her own words without memorizing chapter list (tests comprehension, not recall).

**Acceptance Scenarios**:

1. **Given** Sarah sees chapter titles 34-49 grouped by theme, **When** she reads Chapter Progression section, **Then** she understands grouping logic: "Agent Frameworks (34-37) establish concepts → Integration Patterns (38-40) connect components → Implementation (41: FastAPI) enables building → Quality Practices (42-43) validate correctness → Advanced Patterns (44-46) handle complexity → Data Layer (47-49) adds persistence"

2. **Given** Sarah wonders why FastAPI comes after MCP, **When** she reads progression rationale, **Then** she recognizes pedagogical sequence: "Foundation (what are agents) → Integration (how agents connect) → Implementation (how to build with FastAPI) → Quality (how to test) → Patterns (how to scale) → Data (how to persist)"

3. **Given** Sarah previews part 6 scope, **When** she reviews chapter themes, **Then** she recognizes continuity with Part 5 (spec-driven development now applied to agent systems)

---

### User Story 3 - Set Realistic Capstone Expectations (Priority: P3)

Sarah wants to know what "capstone projects at chapter ends" actually means for Part 6.

**Why this priority**: Expectation management prevents disappointment, but less critical than readiness validation and progression logic.

**Independent Test**: Sarah can distinguish between realistic capstone scope (local agent demonstrations) vs unrealistic expectations (production-deployed systems serving users).

**Acceptance Scenarios**:

1. **Given** Sarah reads Capstone Expectations section, **When** she forms mental model of capstones, **Then** she understands scope: "compose chapter learnings into complete agent implementations with testing and local deployment" (not production-at-scale)

2. **Given** Sarah has spec-driven methodology from Part 5, **When** she previews capstone approach, **Then** she recognizes methodology continuity: "capstones apply spec-first development to agent projects" (same methodology, new domain)

3. **Given** Sarah wants concrete examples, **When** she reviews capstone description, **Then** she understands deliverables: "functional agents with tests, running locally, demonstrating chapter patterns" (tangible but learning-focused)

---

### Edge Cases

- **Partial Part 5 Completion**: What if reader completed Parts 1-4 but skipped Part 5 (spec-driven development)? README should signal spec-driven methodology is prerequisite (referenced throughout Part 6).

- **Returning After Break**: What if reader completed Parts 1-5 months ago and needs refresher? README should reference prerequisite parts explicitly so reader can review if needed.

- **Experienced Developers Skipping Earlier Parts**: What if professional developer starts at Part 6 without earlier parts? README should clarify assumed knowledge (Python fundamentals, AI collaboration basics, spec-driven workflow) so they assess gaps honestly.

---

## Requirements

### Functional Requirements

#### Content Requirements

- **FR-001**: README MUST include Overview section positioning Part 6 as transition from "learning to code with AI" (Parts 1-5) to "building production agents" (Part 6)

- **FR-002**: README MUST include Goals section listing measurable learning outcomes using bridge tone (mix educational + professional language: "learn to implement production patterns", "understand design tradeoffs", "practice testing strategies")

- **FR-003**: README MUST include Chapter Progression section grouping chapters 34-49 by theme with pedagogical rationale (why this sequence serves learning)

- **FR-004**: README MUST include Methodology Note section explaining 4-layer teaching framework continues (described naturally without exposing Stage 1/2/3/4 labels: "chapters introduce concepts manually, then guide AI collaboration, then build reusable components, then apply them in capstone projects")

- **FR-005**: README MUST include Capstone Expectations section defining realistic scope: "compose chapter learnings into complete agent implementations with testing and local deployment" (NOT production-deployed systems)

- **FR-006**: README MUST include Getting Started section with next action (how to begin Part 6)

#### Tone Requirements

- **FR-007**: README MUST use bridge tone balancing educational language (40%+ occurrence: "learn", "explore", "practice", "understand") with professional language (40%+ occurrence: "implement", "design", "deploy", "test", "optimize")

- **FR-008**: README MUST frame prerequisites as enablers ("Part 6 builds on Python skills from Part 4 and spec-driven methodology from Part 5") NOT gates ("You must complete Parts 1-5 before attempting Part 6")

- **FR-009**: README MUST avoid hype language (zero occurrences of: "amazing", "powerful", "cutting-edge", "revolutionary", "game-changing")

- **FR-010**: README MUST avoid over-promising (zero occurrences of: "build production systems in weeks", "ready for deployment", "enterprise-grade in days")

#### Accuracy Requirements

- **FR-011**: README MUST reference chapter numbers and titles exactly as they appear in `specs/book/chapter-index.md` (chapters 34-49, Part 6 section, lines 94-115)

- **FR-012**: README MUST accurately represent chapter themes based on chapter-index.md titles (no speculation about unwritten chapter content)

- **FR-013**: README MUST correctly identify prerequisite parts: Parts 4-5 (required), Parts 1-3 (recommended background)

#### Structure Requirements

- **FR-014**: README MUST contain ONLY these sections: Overview, Goals, Chapter Progression, Methodology Note, Capstone Expectations, Getting Started (Minimal Content principle)

- **FR-015**: README MUST NOT include these sections: "Why This Matters", "Success Stories", "FAQ", "What's Next After Part 6", standalone "Safety Note" (violate minimal content, see Non-Goals section for rationale)

- **FR-016**: README MUST be scannable in under 3 minutes (estimated 600-900 words max, clear section headers, bullet points for scannability)

- **FR-017**: README MUST address edge cases by explicitly stating prerequisite parts (Parts 4-5 required, Parts 1-3 recommended) so readers with partial completion or gaps can assess readiness honestly

- **FR-018**: README MUST use terminology consistent with Parts 1-5 (e.g., "spec-driven development" not "specification-first methodology", "4-layer teaching" not "four-stage progression") to maintain conceptual continuity

### Key Entities

*(Documentation artifact, no data entities involved)*

---

## Success Criteria

### Measurable Outcomes

*(See Evals section above for comprehensive success criteria)*

### Acceptance Tests

**AT-001**: **Clarity Test** - 3 independent reviewers read README and answer: "What will I learn in Part 6?" - Correct answers must include: "production agent development", "Agents SDK, MCP, FastAPI", "TDD/testing for agents", "design patterns", "database integration" (80%+ accuracy required)

**AT-002**: **Prerequisite Test** - Reviewers identify: "What parts must I complete before Part 6?" - Correct answer: Parts 4-5 (Python + spec-driven development) required, Parts 1-3 recommended

**AT-003**: **Tone Analysis** - Keyword count: Neither educational terms (learn, explore, practice, understand) nor professional terms (implement, design, deploy, test, optimize) falls below 40% of total positioning language

**AT-004**: **Hype Scan** - Zero occurrences of: "amazing", "powerful", "cutting-edge", "revolutionary", "game-changing"

**AT-005**: **Over-promising Scan** - Zero occurrences of: "production systems in weeks", "ready for deployment", "enterprise-grade in days"

**AT-006**: **Factual Accuracy** - Cross-reference all chapter numbers (34-49) against chapter-index.md - 100% match required

**AT-007**: **Section Compliance** - README contains exactly: Overview, Goals, Chapter Progression, Methodology Note, Capstone Expectations, Getting Started - no forbidden sections

**AT-008**: **Capstone Scope Test** - Reviewers answer: "Will capstone projects be deployed to production serving real users?" - Correct answer: No (local deployment demonstrations)

**AT-009**: **Methodology Language Scan** - Zero occurrences of scaffolding labels as section headers: "Stage 1/2/3/4", "Layer 1/2/3/4", "Three Roles Framework"

**AT-010**: **Readability Test** - Word count: 600-900 words - confirms scannability in under 3 minutes

---

## Constraints

### Tone Constraints

- **ALLOWED**: Bridge tone mixing educational support with professional expectations
  - Educational: "learn", "explore", "practice", "understand", "discover"
  - Professional: "implement", "design", "deploy", "test", "optimize", "build"
  - Example: "Learn to implement production patterns", "Understand design tradeoffs"

- **FORBIDDEN**:
  - Hype language: "amazing", "powerful", "cutting-edge", "revolutionary"
  - Over-promising: "build production systems in weeks", "production-ready in days"
  - Gatekeeping: "You must have X before Y", "Only attempt if you completed Z"
  - Condescension: "Don't worry, it's easy!", "Even beginners can handle this!"

### Content Constraints

- **ALLOWED**:
  - Chapter numbers 34-49 with exact titles from chapter-index.md
  - Grouping chapters by pedagogical theme (not exhaustive enumeration)
  - Prerequisite references to Parts 1-5 (framed as assets, not barriers)
  - 4-layer methodology described through student experience (not scaffolding labels)

- **FORBIDDEN**:
  - Speculation about chapter content not defined in chapter-index.md
  - Listing all 16 chapters individually (use thematic grouping instead)
  - Exposing internal scaffolding language: "Stage 1/2/3/4", "Layer 1/2/3/4", "Three Roles Framework" as section headers
  - Navigation redundancy: "What's Next", "Coming Soon", "Future Parts"

### Scope Constraints

- **Length**: 600-900 words maximum (scannable in under 3 minutes)
- **Sections**: Exactly 6 sections (Overview, Goals, Chapter Progression, Methodology Note, Capstone Expectations, Getting Started)
- **Audience Assumptions**: Readers completed Parts 1-5 or have equivalent Python + spec-driven development experience

---

## Non-Goals

### What We're NOT Doing

1. **NOT creating comprehensive syllabus**:
   - Why excluded: README is positioning document, not detailed course catalog
   - Where to find: Detailed syllabi exist in chapter-index.md and individual chapter READMEs

2. **NOT marketing Part 6 with hype**:
   - Why excluded: Educational content requires clarity over excitement, facts over promotion
   - Alternative: Tone focuses on concrete outcomes and realistic scope

3. **NOT providing complete prerequisite review**:
   - Why excluded: README references prerequisites but doesn't re-teach them
   - Where to find: Students revisit Part 4 (Python) and Part 5 (Spec-Driven Dev) if refresher needed

4. **NOT listing every chapter individually**:
   - Why excluded: Exhaustive enumeration overwhelms, thematic grouping illuminates progression
   - Alternative: Group by theme (Frameworks, Integration, Quality, Patterns, Data)

5. **NOT explaining Part 6 chapters in detail**:
   - Why excluded: Chapter-specific content belongs in chapter READMEs, not Part README
   - Where to find: Each chapter 34-49 has own README with detailed learning objectives

6. **NOT addressing "What comes after Part 6"**:
   - Why excluded: Forward navigation adds no learning value, students see structure from table of contents
   - Alternative: Students discover Parts 7-12 when ready, focus on current part only

---

## Assumptions

### About Readers

1. **Prerequisite Completion**: Readers completed Parts 1-5 OR have equivalent experience (Python fundamentals, AI collaboration basics, spec-driven development methodology)

2. **Motivation**: Readers are self-motivated developers choosing to advance to production patterns (not forced or uncertain about continuing)

3. **Context Awareness**: Readers know course structure from table of contents (don't need "what's next" navigation within README)

### About Content

1. **Chapter Accuracy**: chapter-index.md is authoritative source for chapter numbers, titles, and themes (chapters 34-49)

2. **Teaching Methodology**: 4-layer teaching framework (manual foundation → AI collaboration → intelligence design → spec-driven integration) is established methodology from earlier parts

3. **Capstone Pattern**: Capstone projects at chapter ends is established pattern from earlier parts (readers familiar with format)

### About Positioning

1. **Transition Signal**: Clear positioning as "bridge" between education and professional development serves students better than abrupt shift

2. **Readiness Validation**: Explicitly validating reader readiness (not gatekeeping) reduces imposter syndrome and builds confidence

3. **Scope Realism**: Setting realistic capstone expectations (local demonstrations, not production deployment) prevents disappointment and maintains trust

---

## Open Questions

*(None - user clarifications received for tone [bridge: educational + professional] and chapter preview depth [grouped by theme])*
