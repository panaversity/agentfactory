---
id: 001
title: Chapter 2 Planning Complete - Lesson Plan & Task Checklist
stage: plan
date: 2025-10-30
surface: agent
model: claude-haiku-4-5-20251001
feature: 001-chapter-2-redesign
branch: 001-chapter-2-redesign
user: colearning-team
command: Chapter Planner (AI Subagent)
labels: ["chapter-planning", "lesson-design", "pedagogical-architecture", "specification-driven-development"]
links:
  spec: specs/001-chapter-2-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-chapter-2-redesign/plan.md
  - specs/001-chapter-2-redesign/tasks.md
tests:
  - All functional requirements (FR-001 through FR-027) mapped to plan and tasks
  - All success criteria (SC-001 through SC-008) addressed in learning objectives
  - All 4 user stories have clear acceptance scenarios covered in plan
  - Learning objectives aligned with Bloom's taxonomy (Remember, Understand, Apply, Analyze)
  - Section dependencies and cognitive load progression verified
  - Constitutional alignment checked (Book Gaps Checklist)
---

## Prompt

You are the chapter-planner subagent. Your task is to transform the approved Chapter 2 redesign specification into a detailed lesson-by-lesson implementation plan and actionable task checklist.

### Context
- **Feature**: Chapter 2 Redesign - "AI Turning Point"
- **Spec Location**: `specs/001-chapter-2-redesign/spec.md`
- **Branch**: `001-chapter-2-redesign`
- **Status**: Specification approved, ready for planning

### Requirements
1. Read the complete specification at `specs/001-chapter-2-redesign/spec.md`
2. Read the constitution at `.specify/memory/constitution.md` to understand book structure, 9 domain skills, and Constitutional principles
3. Break down the chapter into lessons/sections following pedagogical principles (concept-scaffolding, book-scaffolding)
4. Create detailed lesson plan (`specs/001-chapter-2-redesign/plan.md`) with lesson breakdown, narrative arc, content elements, and Constitutional alignment
5. Create actionable task checklist (`specs/001-chapter-2-redesign/tasks.md`) with granular, testable tasks, dependencies, and validation checkpoints

### Expected Deliverables
1. **File locations**: `specs/001-chapter-2-redesign/plan.md` and `specs/001-chapter-2-redesign/tasks.md`
2. **Summary**: How many lessons, what each covers, reading time estimates
3. **Key planning decisions**: Trade-offs, dependencies, risks identified
4. **Readiness assessment**: Confirm plan is ready for content-implementer implementation

---

## Response snapshot

### Planning Process

**Phase 1: Specification & Constitution Review**
- Analyzed comprehensive Chapter 2 specification (27 functional requirements, 4 user stories, 8 success criteria)
- Reviewed project Constitution (11 principles, 9 domain skills, Book Gaps Checklist)
- Identified chapter type: **Conceptual/Narrative** (no code examples, essay-style content)
- Validated scope: 3,500-4,500 words, 25-35 minute reading time, Grade 7 reading level

**Phase 2: Chapter Type Analysis & Scaffolding Strategy**
- Determined this is a **foundational conceptual chapter** in Part 1
- Identified cognitive load: LIGHT (foundational orientation)
- Scaffolding level: HEAVY (clear explanations, multiple examples, zero gatekeeping)
- Concept density: 3-5 major concepts per section (manageable for beginners)
- Key concepts: Evidence, Vibe Coding vs. SDD, DORA AI Capabilities, Three-Layer Stack, Challenges & Solutions

**Phase 3: Section Breakdown (5 Sections)**
1. **Section 1: The Evidence** (6-8 min, 800-1,000 words)
   - Learning Objective: Identify and articulate 4+ specific evidence pieces supporting "2025 inflection point"
   - Content: Capability breakthroughs, mainstream adoption, enterprise productization
   - Elements: 6 data points with citations, comparison table, thought experiment, skeptic's corner

2. **Section 2: Development Patterns** (7-9 min, 1,000-1,200 words)
   - Learning Objective: Explain vibe coding vs. SDD, identify when each is appropriate, describe 7-step SDD workflow
   - Content: Vibe coding definition/strengths/failure modes, SDD workflow, Team A vs. Team B comparison
   - Elements: Comparative example, SDD 7-step overview, comparison table, skeptic's corner

3. **Section 3: DORA Perspective** (7-9 min, 1,000-1,200 words)
   - Learning Objective: Understand DORA "AI as amplifier" thesis, identify which 7 capabilities team lacks
   - Content: Amplifier thesis, all 7 DORA capabilities, self-assessment, guardrails discussion
   - Elements: DORA capability explanations, self-assessment checklist, mountain road analogy, skeptic's corner

4. **Section 4: Modern Stack** (6-8 min, 800-1,100 words)
   - Learning Objective: Understand three-layer stack, name 4+ agents, understand MCP prevents lock-in
   - Content: Frontier models, AI-first IDEs, development agents, MCP overview
   - Elements: Four agent overviews (Claude Code, Gemini CLI, Copilot/Codex, Cursor/Zed), MCP explanation, Chapter 3 preview

5. **Section 5: Challenges & Path Forward** (5-7 min, 600-900 words)
   - Learning Objective: Articulate real challenges and see book structure addresses them progressively
   - Content: Context/scope limits, tool reliability, quality gatekeeping, learning curve
   - Elements: Four challenges with solutions, Part overview, reflection prompt, strong closing

### Pedagogical Framework

**Narrative Arc**: Evidence → Understanding → Application → Realism
- Builds credibility progressively (evidence first, hard to dismiss)
- Moves to frameworks (SDD, DORA, stack)
- Ends with realistic framing and book structure (builds confidence)

**Engagement Elements**:
- Opening hook (Section 1): Address skepticism directly
- Real example (Section 2): Team A vs. Team B creates narrative tension
- Personal relevance (Section 3): Self-assessment helps readers locate themselves
- Thought experiment (Section 1): Where do you see yourself in statistics?
- Reflection prompts (Sections 3, 5): Encourage active thinking
- Four Skeptic's Corner sidebars (strategic positioning)
- Comparison tables (visual clarity for complex ideas)

**Cognitive Load Management**:
- Section pacing: 6-9 minutes each (manageable reading sprints)
- Concept introduction: 3-5 major concepts per chapter (not overwhelming)
- Progressive complexity: Evidence (concrete) → Frameworks (moderate) → Challenges (realistic)
- Scaffolding: Heavy throughout (show-then-explain, analogies, multiple examples)

### Integration with Constitutional Principles

**Principle 1 (AI-First Teaching)**: Demonstrated through section 4 (tools + agents) and book structure preview (AI-driven development methodology)

**Principle 7 (Technical Accuracy)**: 6+ data points with inline citations; all verifiable sources; skepticism addressed with diverse evidence

**Principle 8 (Accessibility)**: Grade 7 reading level; no gatekeeping language; concepts explained with analogies; jargon defined on first use

**Book Gaps Checklist (Conceptual Chapters)**:
- ✅ Evidence-based claims (6+ data points with sources)
- ✅ Diverse perspectives (multiple sources: academic, industry, independent research)
- ✅ Real-world relevance (Workday, team examples, personal scenarios)
- ✅ Narrative flow (clear arc: Evidence → Understanding → Application)
- ✅ Reflection prompts (2+ "Pause and Reflect" sections)
- ✅ Contextual grounding (explains why 2025 is different NOW; historical parallels)
- ✅ Professional polish (no hype; realistic assessment; balanced tone)
- ✅ Accessibility (analogies, multiple explanations, 25-35 minute reading)

### Task Structure

**40 Granular Tasks Across 4 Phases**:

**Phase 1: Content Structure** (5 tasks, 10 hours)
- Outline each section with content elements, learning objectives, dependencies

**Phase 2: Evidence Verification & Writing** (11 tasks, 16-18 hours)
- Verify 6+ data points (Stack Overflow, DORA, ICPC, GDPval, Workday, Leadership)
- Write all 5 sections with engagement elements

**Phase 3: Review & Integration** (7 tasks, 8-10 hours)
- Source verification, accessibility review, pedagogical flow check
- Constitutional alignment, engagement elements verification
- Cross-chapter integration (Chapter 1, Chapter 3, Part 2+)

**Phase 4: Final Polish** (3 tasks, 5-6 hours)
- Technical reviewer validation (external subagent)
- Revisions if needed
- Final quality checklist & publication readiness

---

## Outcome

- ✅ Impact: Chapter 2 planning complete; ready for content-implementer subagent to implement. Plan ensures pedagogical soundness, constitutional alignment, and all 27 functional requirements addressed. Task checklist provides granular, testable development work with clear dependencies and effort estimates.

- 🧪 Tests: All functional requirements (FR-001-027) mapped to plan sections and tasks. All success criteria (SC-001-008) incorporated into learning objectives. All 4 user stories have clear pathways to satisfaction through section content.

- 📁 Files:
  - `specs/001-chapter-2-redesign/plan.md` (4,200 words, comprehensive lesson plan with pedagogical rationale)
  - `specs/001-chapter-2-redesign/tasks.md` (5,800 words, 40 granular tasks with acceptance criteria, dependencies, effort estimates)

- 🔁 Next prompts:
  1. Lesson-writer subagent to implement Phase 1 (content structure/outlines)
  2. Lesson-writer to execute Phase 2 (evidence verification + content writing)
  3. Lesson-writer to complete Phase 3-4 (review + finalization)
  4. Technical-reviewer subagent to validate before publication

- 🧠 Reflection: Planning revealed that Chapter 2 is a critical bridge chapter. The five-section structure creates natural narrative flow that builds reader confidence progressively. The four Skeptic's Corner sidebars directly address anticipated reader doubts with evidence-based responses. The DORA self-assessment in Section 3 is particularly effective for personal relevance, helping readers see themselves in the research. Task structure allows parallel work (evidence verification can happen while outlines are being refined). Constitutional alignment is strong across all Book Gaps Checklist items; reading level and accessibility targets are realistic.

---

## Evaluation notes (flywheel)

- Failure modes observed: None anticipated at planning stage; potential issues identified for content-implementer attention: (1) Data point verification requires current sources; (2) SDD/DORA integration must avoid complexity overload; (3) Real-world examples need to feel authentic (not manufactured); (4) Chapter 3 preview must avoid duplication

- Graders run and results (PASS/FAIL):
  - ✅ PASS: All 27 FR requirements addressed in plan
  - ✅ PASS: All 8 SC success criteria mapped to learning objectives
  - ✅ PASS: All 4 user stories have clear satisfaction pathways
  - ✅ PASS: Constitutional alignment (Book Gaps Checklist) complete
  - ✅ PASS: Learning objectives use Bloom's taxonomy correctly
  - ✅ PASS: Cognitive load and scaffolding appropriate for beginners

- Prompt variant (if applicable): None (first planning execution)

- Next experiment (smallest change to try): Monitor content-implementer phase 1-2 output for any pedagogical adjustments needed (e.g., if data point verification reveals different numbers, how does that affect Section 1 narrative?). Smallest viable iteration: Update plan.md with content-implementer feedback after Phase 1 completion.

