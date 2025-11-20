# Feature Specification: Book Structure and Architect Plan

**Feature Branch**: `002-book-structure`
**Created**: 2025-10-29
**Status**: Draft
**Input**: User description: "Setup Book Structure and Architect overall plan. This will lay the foundation for your peers and writers to start planning and then design the actual content for our book."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Content Creator Can Find Assignment and Understand Scope (Priority: P1)

A book content creator (human author or AI agent) joins the project and needs to understand what to write. They should be able to quickly locate their assigned chapter, understand its scope, learning objectives, and prerequisites—without ambiguity about what belongs in that chapter versus neighboring chapters.

**Why this priority**: This is the foundational capability. Without clear chapter assignments and scope, writers will create overlapping content, gaps, or scope creep. This directly enables all downstream writing work.

**Independent Test**: A new writer can:
1. Consult the central chapter index
2. Identify a specific chapter assignment (title, number, filename, part)
3. Understand the chapter's scope and key topics
4. Know what chapters precede and follow it
5. Begin writing without requesting clarification

**Acceptance Scenarios**:

1. **Given** a writer is assigned Chapter 14 (Functions, Types, and Type Hints), **When** they consult the chapter index, **Then** they immediately see: chapter number, exact title, filename format, part assignment (Part 4), and the 3-4 key topics this chapter must cover.

2. **Given** a writer is creating Chapter 6 (Claude Code: Features and Workflows), **When** they check dependencies, **Then** they can see which chapters must come before it (Part 2 chapters 1-5) and which come after (Part 2 chapters 7-9).

3. **Given** a writer is unsure about scope (e.g., "Should Chapter 15 cover generators or is that later?"), **When** they consult the chapter index, **Then** the chapter's key topics clearly answer the question without needing human intervention.

---

### User Story 2 - Content Creator Understands Quality Standards and Non-Negotiable Rules (Priority: P1)

Writers need a clear, authoritative source for quality expectations: what the book values, what's required, what's forbidden. They should understand the 11 core principles, the 8 domain skills that must be applied, and the non-negotiable rules (ALWAYS DO / NEVER DO).

**Why this priority**: Quality standards prevent downstream rework. Without clear expectations, writers may produce content that doesn't align with the book's philosophy (e.g., teaching Python without AI, using deprecated syntax, skipping tests).

**Independent Test**: A new writer can:
1. Read the constitution and understand the project's core philosophy
2. Find the 11 core principles and why each matters
3. Locate the 8 mandatory domain skills and how to apply them
4. See the non-negotiable rules (ALWAYS DO / NEVER DO)
5. Review output style guides (chapters, lessons, code examples, exercises)
6. Begin writing content confident they understand expectations

**Acceptance Scenarios**:

1. **Given** a writer is creating code examples, **When** they consult quality standards, **Then** they see: "All code MUST use Python 3.13+ with mandatory type hints on every function. Every example must be tested before publication." They know this is non-negotiable.

2. **Given** a writer is starting Chapter 5, **When** they review the constitution, **Then** they understand: "Principle 9 requires show-then-explain pedagogy" and "Principle 8 forbids gatekeeping language like 'simple' or 'obvious'". They design their content accordingly.

3. **Given** a writer is uncertain about how to integrate AI into Chapter 15, **When** they consult Principle 1 (AI-First Teaching Philosophy), **Then** they understand: "Every code example shows both the prompt/request and the AI-generated result" and they adjust their lesson accordingly.

---

### User Story 3 - Architect Can Plan Content Work Systematically (Priority: P1)

The project lead/architect needs a structured overview of all 32 chapters across 7 parts, showing which chapters are prerequisites for others, which topics depend on prior chapters, and what the overall learning progression looks like. This enables:
- Parallel chapter writing (chapters with no dependencies can be assigned independently)
- Identifying gaps or overlaps in scope
- Sequencing content review in logical order
- Spotting where scaffolding is needed

**Why this priority**: This enables efficient project management. Without this view, the architect can't assign work effectively or predict bottlenecks. This is essential for coordinating 26+ chapters across a team.

**Independent Test**: The architect can:
1. View the complete 7-part structure with all 32 chapters
2. Identify prerequisite chains (e.g., Chapter 14 must come before 15-16)
3. See which chapters can be written in parallel
4. Understand the scaffolding strategy (Parts 1-3 heavy scaffolding, Parts 4-7 lighter)
5. Spot dependencies across parts (e.g., Part 5 depends on concepts from Parts 1-4)
6. Plan resource allocation and timeline

**Acceptance Scenarios**:

1. **Given** the architect reviews the book structure, **When** they look at Part 4 chapters (14-21), **Then** they see: Chapter 14 is foundational, Chapter 17 (Testing) is prerequisite for later projects, Chapter 21 synthesizes all of Part 4. They can plan dependencies.

2. **Given** the architect wants to parallelize Chapter 1 and Chapter 6 writing, **When** they check dependencies, **Then** they see that Chapter 6 only requires Chapters 1-5 as prerequisites, so both can be worked on independently as long as Part 1 is complete first.

3. **Given** the architect is planning timeline, **When** they review scaffolding levels, **Then** they see: Chapters 1-5 (heavy scaffolding) may need more review cycles; Chapters 22+ (lighter scaffolding) can move faster.

---

### User Story 4 - Output Styles Are Centralized and Reusable (Priority: P2)

Writers follow consistent output formats so the book has a unified voice and structure. Output styles (chapter template, lesson template, code example format, exercise format) are centralized, documented, and reusable across the entire book. When output styles change, all chapters update automatically by referencing the central style, not by editing each chapter.

**Why this priority**: Consistency is essential for reader experience. Centralized styles reduce maintenance burden and enable AI/human collaboration on content without style conflicts.

**Independent Test**:
1. A writer creates Chapter 10 by following `.claude/output-styles/chapters.md`
2. A second writer creates Chapter 25 using the same template
3. Both chapters have identical structure, section names, and formatting conventions
4. If the chapter style is updated, both chapters automatically improve without being re-edited
5. The template is clear enough that both human and AI writers produce consistent results

**Acceptance Scenarios**:

1. **Given** two writers start Chapter 5 and Chapter 20 independently, **When** they both follow `.claude/output-styles/chapters.md`, **Then** their chapters have identical structure: same section names, same learning objectives format, same code example styling, same exercise format.

2. **Given** a decision is made to add a new section "Key Takeaways" to all chapters, **When** the output style is updated, **Then** writers can reference the new style and apply it; old chapters can be updated by referencing the new template without manual editing of each chapter.

---

### User Story 5 - Domain Skills Are Applied Consistently (Priority: P2)

The 8 mandatory domain skills (learning-objectives, concept-scaffolding, code-example-generator, exercise-designer, assessment-builder, technical-clarity, book-architecture, ai-augmented-teaching) are documented, accessible, and applied to every chapter. Writers know which skills apply to their chapter and how to invoke them.

**Why this priority**: The 8 skills codify best practices. Consistent application ensures pedagogical quality across all 32 chapters. This is essential for reader learning outcomes.

**Independent Test**:
1. A writer starts Chapter 8 and consults `.claude/skills/`
2. They invoke the `learning-objectives` skill to define measurable outcomes
3. They invoke the `code-example-generator` skill to create examples
4. They invoke the `exercise-designer` skill to create practice activities
5. The chapter demonstrates quality across all applied skills

**Acceptance Scenarios**:

1. **Given** a writer is creating Chapter 16 (OOP), **When** they reference `.claude/skills/learning-objectives`, **Then** they create Bloom's taxonomy-aligned outcomes (remember, understand, apply, analyze levels) before writing content.

2. **Given** Chapter 12 needs complex scaffolding (debugging with AI), **When** the writer invokes the `concept-scaffolding` skill, **Then** they break the topic into 4-5 progressive steps, each building on the previous.

3. **Given** Chapter 19 (APIs and Data), **When** the writer uses the `code-example-generator` skill, **Then** all examples include type hints, docstrings, and passing tests—not just working code.

---

### Edge Cases

- What happens when a chapter requires concepts from non-adjacent parts (e.g., Part 6 agent examples needing Part 4 Python concepts)? → Dependencies are documented; cross-part references are explicit.
- What happens when an output style changes mid-project (e.g., new section required)? → Template is updated; writers reference the new version; old chapters can be updated by referencing the template.
- What happens when a domain skill needs updating (e.g., new Python 3.13 feature discovered)? → Skill is updated; writers reference the new version; old content validated for compliance.

---

## Cognitive Load Management & Pedagogical Architecture *(critical for success)*

**Core Principle**: "Cognitive Load is the Main Barrier for Beginners." The specification MUST ensure readers are never overwhelmed by managing information progressively and providing scaffolding at each stage.

### The Beautiful Progression: Foundation → Tools → Communication → Code → Methodology → Agents → Integration

The 7-part structure follows a precise cognitive progression that prevents overwhelming beginners:

```
Part 1: "Understanding the AI Revolution"    ← Orientation: Build confidence with AI
  ↓ (Part 1 complete, foundation ready)
Part 2: "Meeting the Tools"                   ← Tool literacy: Understand your options
  ↓ (Tools chosen and familiar)
Part 3: "Learning to Communicate"            ← Skill building: Master AI interaction
  ↓ (AI collaboration skills ready)
Part 4: "Learning to Code"                    ← Language mastery: Professional Python
  ↓ (Can write production code)
Part 5: "Learning Methodology"               ← Professional practices: Scale projects
  ↓ (Can structure complex projects)
Part 6: "Building AI Agents"                  ← Advanced application: Autonomous systems
  ↓ (Understand agents thoroughly)
Part 7: "Building MCP Servers"                ← Integration: Extend capabilities
```

### Why This Sequence Works for Beginners

| Part | Cognitive Load | Scaffolding | Learning Outcomes | Why This Ordering |
|------|----------------|-------------|-------------------|-------------------|
| **1** | **Light** | **Heavy** | Comfort with AI, Python basics, tool orientation | Builds confidence first; no programming anxiety yet |
| **2** | **Moderate** | **Heavy** | Understanding tools, making informed choices | Learners choose familiar tool before deep work |
| **3** | **Moderate** | **Heavy** | Effective communication, context management | AI collaboration skill transfers to all future work |
| **4** | **High** | **Moderate** | Production-quality Python, types, testing | Most demanding part; learner ready for challenge; still has AI support |
| **5** | **Moderate** | **Moderate** | Professional workflow, specs, planning | Learner consolidates Python skills; methodology is lighter cognitive load |
| **6** | **Moderate-High** | **Light** | Agentic patterns, autonomous systems | Learner can handle complexity; previous methodology training helps design |
| **7** | **Moderate** | **Light** | Integration, protocol, extension | Final specialization; learner is confident; MCP is narrowly scoped |

### Scaffolding Strategy: "The Teacher Helps Manage Cognitive Load"

The specification requires explicit scaffolding guidance per part:

**Heavy Scaffolding (Parts 1-3: 13 chapters)**
- Content creator provides: Detailed examples, step-by-step walkthroughs, lots of guided practice
- Cognitive approach: "Show → Explain → Practice with guidance"
- Expected review cycles: 2-3 iterations (more feedback loops)
- Learner independence: Low (heavy guidance expected)

**Moderate Scaffolding (Parts 4-5: 13 chapters)**
- Content creator provides: Clear examples, some guided practice, some independent exercises
- Cognitive approach: "Show → Explain → Practice with some independence"
- Expected review cycles: 1-2 iterations
- Learner independence: Medium (gradual release of responsibility)

**Light Scaffolding (Parts 6-7: 6 chapters)**
- Content creator provides: Working examples, challenges, some advanced reading
- Cognitive approach: "Show → Explain → Independent practice"
- Expected review cycles: Self-review sufficient (learner reviews their own work)
- Learner independence: High (learner expected to synthesize and apply)

### Progressive Concept Disclosure: How Many Ideas Per Chapter?

The specification requires concept density management:

**Part 1 Concept Density**: 3-4 concepts per chapter
- Example Ch 2: "AI Tools" → Concept 1: What is AI? Concept 2: Types of AI tools Concept 3: Capabilities vs. Limitations
- Rationale: Beginners need time to digest; more concepts → cognitive overload

**Part 2 Concept Density**: 3-5 concepts per chapter
- Example Ch 6: "Claude Code" → Concept 1: Features Concept 2: Workflows Concept 3: Strengths Concept 4: Limitations Concept 5: When to use
- Rationale: More familiar with AI basics; can handle slightly more

**Part 3 Concept Density**: 4-5 concepts per chapter
- Example Ch 11: "Context Management" → Concept 1: Context windows Concept 2: Memory management Concept 3: Tokens Concept 4: State design Concept 5: Practical strategies
- Rationale: Building skill, ready for more

**Part 4 Concept Density**: 5-7 concepts per chapter (MAXIMUM—requires tightly focused chapters)
- Example Ch 16: "Data Structures" → Concept 1: Types (list, dict, set) Concept 2: Performance Concept 3: Type hints Concept 4: Common patterns Concept 5: Testing Concept 6: When to use each
- Rationale: Most complex part; must not exceed 7 or cognitive load spikes
- **Mitigation**: Longest part (8 chapters) spreads Python learning; earlier chapters have less density

**Parts 5-7 Concept Density**: 4-6 concepts per chapter
- Rationale: Learner is experienced; can handle complexity but not overwhelmed by new domain

### Learning Outcomes Per Part (Foundation for Writer Guidance)

Specification MUST articulate what readers achieve per part so writers scope chapters correctly:

**Part 1 Outcomes**:
- ✅ Understand AI as collaborative partner (not replacement for thinking)
- ✅ Set up Python and development environment
- ✅ Write first Python programs with AI assistance
- ✅ Understand debug/iterate with AI workflow

**Part 2 Outcomes**:
- ✅ Understand distinct tools: Claude Code, Gemini CLI, GitHub Copilot
- ✅ Know strengths of each tool
- ✅ Make informed tool choices for tasks
- ✅ Understand tool limitations and fallback strategies

**Part 3 Outcomes**:
- ✅ Write effective, specific prompts
- ✅ Manage context and token usage
- ✅ Debug AI-generated code systematically
- ✅ Handle AI errors and refine requests

**Part 4 Outcomes**:
- ✅ Write Python with complete type annotations
- ✅ Use modern Python 3.13+ features
- ✅ Implement comprehensive testing
- ✅ Handle errors professionally
- ✅ Work with APIs and external data
- ✅ Design clean, maintainable code
- ✅ Build realistic, complete projects

**Part 5 Outcomes**:
- ✅ Understand Spec-Driven Development (SDD) methodology
- ✅ Write specifications for projects
- ✅ Create plans and decompose into tasks
- ✅ Apply Spec-Kit workflow to real projects
- ✅ Coordinate teams using specs

**Part 6 Outcomes**:
- ✅ Understand agentic AI concepts and patterns
- ✅ Design autonomous agent workflows
- ✅ Build functional agents with tools
- ✅ Orchestrate multi-agent systems
- ✅ Debug and improve agent behavior

**Part 7 Outcomes**:
- ✅ Understand Model Context Protocol (MCP) architecture
- ✅ Integrate existing MCP servers
- ✅ Build custom MCP servers
- ✅ Design tools that extend AI capabilities

### Reading Path Support: Different Learners, Different Journeys

The specification MUST support multiple reading paths so learners can match their needs:

**Linear Path (Recommended for Beginners)**
- Read all 32 chapters sequentially
- Serves: Absolute beginners with no programming background
- Rationale: Part 1-3 foundation essential; Part 4 large but manageable with prior preparation

**Fast Track (Experienced Developers)**
- Skip Parts 1-2 (already understand development); start Part 3 (prompting is unique to AI)
- Read: Ch 10-32 (23 chapters)
- Rationale: Experienced developers don't need basic setup; prompting and Spec-Kit are new

**Agent + MCP Focus**
- Foundation + Python + Methodology + Agents + MCP
- Read: Part 1 + Part 4 (core Python) + Part 5 (methodology) + Part 6-7 (agents/MCP)
- Skip: Parts 2-3 (tool details less critical if focused on building)

**Python Fundamentals Path**
- For developers wanting Python + testing without agents
- Read: Parts 1, 3, 4, 5 (skip 2, 6, 7)

### Preventing Cognitive Overload: The Architect's Role

The specification requires the architect plan to include:

1. **Concept Density Validation**: No chapter exceeds 7 core concepts
2. **Difficulty Progression Smoothness**: No difficulty jump > 20% between consecutive chapters
3. **Bloom's Level Balance**: Each chapter targets mix of levels (Remember 20%, Understand 30%, Apply 30%, Analyze 20%)
4. **Exercise Frequency**: Guided practice every 3-4 pages of explanation
5. **Prerequisite Chains**: No learner surprises (prerequisites always stated)
6. **Scaffolding Consistency**: Heavy scaffolding in Parts 1-3, medium in 4-5, light in 6-7

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Book structure document MUST specify all 32 chapters organized into 7 parts with exact titles, filenames, and key topics for each chapter.
- **FR-002**: Each chapter entry MUST include chapter number, title, filename pattern, part assignment, and 3-5 key topics.
- **FR-003**: Constitution document MUST codify the 11 core principles, clearly explaining the WHY and WHAT for each principle.
- **FR-004**: Constitution MUST list the 8 mandatory domain skills with descriptions of how each applies to book development.
- **FR-005**: Constitution MUST document non-negotiable rules as ALWAYS DO / NEVER DO lists with clear rationale.
- **FR-006**: Constitution MUST specify the 7-part structure (parts 1-7) with chapter counts, focus areas, and learning outcomes for each part.
- **FR-007**: Output styles document MUST define chapter structure (sections, content flow, formatting, frontmatter format).
- **FR-008**: Output styles document MUST define lesson structure (teaching flow, section names, assessment patterns).
- **FR-009**: Output styles document MUST define code example standards (type hints, docstrings, testing requirements).
- **FR-010**: Output styles document MUST define exercise/assignment structure (setup, scenario, reflection).
- **FR-011**: Architect plan document MUST identify dependencies between chapters (prerequisites, knowledge scaffolding).
- **FR-012**: Architect plan MUST specify which chapters can be written in parallel vs. which must be sequential.
- **FR-013**: Architect plan MUST define the scaffolding strategy (which parts have heavy guidance vs. light guidance).
- **FR-014**: Architect plan MUST document the quality gates and acceptance criteria for each content stage.

### Key Entities

- **Chapter**: A single unit of instruction (32 total) with title, number, filename, part assignment, key topics, learning objectives, prerequisites, and acceptance criteria.
- **Part**: A grouping of 3-8 chapters focused on a coherent learning outcome (7 total).
- **Domain Skill**: A specialized pedagogical capability (8 total) applied across all chapters (learning-objectives, concept-scaffolding, code-example-generator, exercise-designer, assessment-builder, technical-clarity, book-architecture, ai-augmented-teaching).
- **Output Style**: A reusable template for formatting content consistently (chapters.md, lesson.md, code-example.md, exercise.md).
- **Quality Standard**: A non-negotiable rule or principle guiding content creation (11 principles + non-negotiable rules).
- **Dependency Chain**: A relationship showing which chapters must be completed before others can be written or understood.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chapter Index is complete with all 32 chapters specified, filenames assigned, and part assignments clear. A writer can locate any chapter in <30 seconds and understand its scope.

- **SC-002**: Constitution document contains all 11 core principles with clear rationale and enforcement guidance. Each principle is referenced by name and number for citation.

- **SC-003**: Constitution document lists the 8 mandatory domain skills with descriptions, when to apply each, and how to invoke. Writers reference skills by name when creating content.

- **SC-004**: Constitution specifies non-negotiable rules in ALWAYS DO / NEVER DO format with rationale. At least 8 ALWAYS rules and 6 NEVER rules are documented.

- **SC-005**: Seven-part structure is clearly defined with part names, chapter counts (5+4+4+8+5+3+3=32), focus areas, and learning outcomes. Each part has a clear educational goal.

- **SC-006**: Output styles document for chapters (chapters.md) specifies: section structure, formatting conventions, Docusaurus frontmatter format, word count targets, and quality checklist. A writer can create Chapter N by following this template.

- **SC-007**: Output styles document for lessons (lesson.md) specifies: lesson flow, section names, learning objective integration, code example placement, and exercise linking. A writer can structure a lesson by following this template.

- **SC-008**: Output styles document for code examples (code-example.md) specifies: type hint requirements, docstring standards, testing expectations, and validation checklist. All examples follow these standards.

- **SC-009**: Output styles document for exercises (exercise.md) specifies: scenario format, AI exercise structure, reflection questions, and difficulty levels. Exercises are consistent across all chapters.

- **SC-010**: Architect plan document identifies 5+ prerequisite chains (e.g., Chapter 14 → 15-16, Chapter 6 → Part 2 completion before Part 3). Dependencies are clear enough to parallelize work.

- **SC-011**: Architect plan specifies which chapters can be written in parallel (e.g., Chapters 6 and 10 can start once Part 1 is complete) and which must be sequential. Parallelization increases efficiency by 30%+.

- **SC-012**: Architect plan documents scaffolding levels: Parts 1-3 (heavy scaffolding, 2-3 review cycles), Parts 4-5 (moderate, 1-2 cycles), Parts 6-7 (light, self-review sufficient). Writers understand expectations per part.

- **SC-013**: Quality gates are defined for each content stage: spec → outline → draft → review → publication. Acceptance criteria for each gate are measurable and verifiable.

- **SC-014**: All planning artifacts (constitution, chapter index, architect plan, output styles, domain skills) are accessible in the `.specify/memory/` and `.claude/` directories with clear navigation.

- **SC-015**: Cognitive load management plan specifies: (1) scaffolding levels per part (heavy/moderate/light), (2) concept density limits per part (3-7 concepts/chapter), (3) learning outcomes per part, (4) multiple reading paths, (5) difficulty progression rules (no jump >20%), (6) Bloom's level targets per chapter. Writers use this to prevent overwhelming learners.

- **SC-016**: Architect plan includes validation checklist for cognitive load: no chapter exceeds 7 concepts, no difficulty jump >20% between consecutive chapters, Bloom's distribution is balanced (Remember 20%, Understand 30%, Apply 30%, Analyze 20%), guided practice every 3-4 pages, prerequisites always stated, scaffolding consistent with part level.

---

## Implementation Constraints

- **Scope**: This feature focuses on STRUCTURE and PLANNING, not content creation. No chapters are written in this feature. Output styles are templates, not complete chapters.

- **Architecture**: The book structure follows the 7-part, 32-chapter model defined in the Constitution (Section III). This is non-negotiable and cannot be changed without amending the Constitution.

- **Quality**: All artifacts must align with the Constitution (Section I-VII) before completion. Constitutional alignment is verified before handoff.

- **Dependencies**: This feature is a prerequisite for all downstream chapter-writing features. No chapters can be written until this structure is approved.

---

## Assumptions

1. **Constitution is current**: The `.specify/memory/constitution.md` (v2.0.0, 2025-10-28) accurately reflects the project's vision, principles, and requirements. No major changes are anticipated.

2. **Chapter Index is authoritative**: The existing `specs/book/chapter-index.md` is the single source of truth for chapter assignments. The architect plan will reference and reinforce this index, not replace it.

3. **Output styles are reusable**: Templates (chapters.md, lesson.md, code-example.md, exercise.md) are generic and reusable across all 32 chapters. They are not chapter-specific.

4. **8 domain skills are complete**: The 8 mandatory domain skills (in `.claude/skills/`) are fully defined and ready to apply. Writers can invoke them without further development.

5. **AI + Human collaboration**: Writers include both AI agents and human authors. Structure and output styles must be clear enough for both to follow consistently.

6. **Python 3.13+ is locked**: Python version 3.13+ is the standard for all code examples. No legacy pattern support is required.

7. **32-chapter scope is final**: The book is scoped at exactly 32 chapters in 7 parts. Adding/removing chapters requires constitutional amendment.

---

## Out of Scope

- ❌ Writing actual chapter content (that's downstream in `/sp.plan` and `/sp.implement` commands)
- ❌ Creating code examples (done by code-example-generator skill)
- ❌ Designing exercises (done by exercise-designer skill)
- ❌ Creating assessments (done by assessment-builder skill)
- ❌ Docusaurus build and deployment (separate feature)
- ❌ GitHub publishing and CI/CD setup (separate feature)

---

## Success Metrics Summary

| Artifact | Success Measure | Target |
|----------|-----------------|--------|
| Chapter Index | Complete, unambiguous, searchable | 32 chapters, all specified |
| Constitution | Coherent, principle-focused, non-negotiable rules clear | 11 principles + ALWAYS/NEVER rules |
| Cognitive Load Management | Scaffolding strategy clear, concept density limits defined, reading paths documented | Parts 1-3 heavy, Parts 4-5 moderate, Parts 6-7 light; 3-7 concepts/chapter; 4 reading paths |
| Architect Plan | Dependency chains clear, parallelization opportunities identified, cognitive validation rules defined | 5+ chains, 30%+ parallelization, validation checklist |
| Output Styles | Reusable, clear, sufficient for consistent implementation | 4 templates (chapter, lesson, code, exercise) |
| Domain Skills | Documented, referenceable, applicable to all chapters | 8 skills with clear invocation guidance |

---

## Next Steps (Planning)

Once this specification is approved:

1. **Create Architect Plan** (`/sp.plan` on this feature) → Detailed analysis of chapter dependencies, scaffolding strategy, quality gates, timeline estimates
2. **Generate Task List** (`/sp.tasks`) → Actionable items for finalizing artifacts and validating alignment
3. **Begin Chapter Planning** (new feature `/sp.specify` for individual chapters like "Part 1 Planning") → Create detailed specs for each chapter as writers come on board

---

## Part 1 Narrative: "The Billion-Dollar Opportunity: From Coder to Super Orchestrator"

**Part 1 establishes the foundational mindset shift** that transforms how developers think about their role in the AI era.

### Core Message

The game has changed. Developers are no longer "coders"—they are **super orchestrators** who:
- Design system architectures that solve billion-dollar business challenges
- Write specifications that AI agents execute flawlessly
- Orchestrate subagents to build production-grade solutions
- Create reusable vertical intelligence instead of disposable code
- Dominate niche markets through deep integrations and agentic solutions

### Five Chapter Arc

1. **Welcome to AI-Driven Development** - Orient readers to the paradigm shift: why orchestration beats coding
2. **Understanding AI Tools** - Explain the 9 revolutions that make this moment possible (frontier LLMs, mainstream adoption, coding agents, executable specs, MCP, AI-native IDEs, cloud infrastructure, composable architectures, universal platforms)
3. **Setting Up Your Environment** - Practical setup: Claude Code, Gemini CLI, development tools
4. **Your First Program with AI** - Get hands-on: Write spec → AI generates → AI tests → AI deploys
5. **Debugging and Iterating with AI** - Build confidence: How to work effectively with AI agents, handling failures, iterating quickly

### Part 1 Learning Outcomes

By the end of Part 1, readers will:
- Understand the shift from "coder" to "super orchestrator" mindset
- Recognize the competitive opportunity in vertical agentic solutions
- Know the 9 technological revolutions enabling this shift
- Have Spec-Kit Plus tools installed and verified
- Have executed their first spec-driven development cycle
- Be confident working iteratively with AI agents

### Pedagogical Strategy

**Cognitive Load: LIGHT** (Parts 2-7 build on this foundation)
**Scaffolding: HEAVY** (Show-then-explain, plenty of guided examples, zero gatekeeping language)
**Review Cycles: 2-3**

Key techniques:
- Use the "Coder to Super Orchestrator" narrative to maintain coherence
- Emphasize that orchestration is MORE powerful than traditional coding, not a degradation
- Show real ARR numbers and startup examples to build motivation
- Heavy scaffolding: every concept shown first, then explained
- No prerequisites assumed—this is the book's entry point

---

## Glossary

- **Cognitive Load**: The amount of mental effort required to understand and process information. The specification prevents overwhelming beginners by managing concept density, scaffolding, and progressive disclosure.
- **Specification-Driven Development (SDD)**: A methodology where clear requirements precede implementation. Used throughout this project.
- **Scaffolding**: Pedagogical support for learners—heavy early (guided examples), lighter later (independent problem-solving).
- **Concept Density**: The number of core concepts taught per chapter (target: 3-7, varies by part). Higher density → more cognitive load.
- **Progressive Disclosure**: Teaching technique where information is revealed gradually in manageable chunks, building from simple to complex.
- **Domain Skill**: A specialized capability applied across multiple chapters (e.g., learning-objectives skill applies to all chapters).
- **Output Style**: A reusable template for formatting content consistently (e.g., all chapters follow the same structure).
- **Constitutional Alignment**: Ensuring all work conforms to the Constitution's principles and non-negotiable rules.
- **Artifact**: A tangible deliverable (document, template, specification, plan).
