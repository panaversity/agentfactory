# Feature Specification: Chapter 10 Redesign - Prompt Engineering Methodology for Developers

**Feature Branch**: `025-chapter-10-redesign`
**Created**: 2025-01-18 (Revised with correct developmental sequencing)
**Status**: Draft
**Input**: User description: "Rewrite Chapter 10 (Prompt Engineering for AI-Driven Development) using reasoning activation frameworks from research papers. Teach complete prompt engineering methodology to developers who haven't learned to code yet (Chapter 10 of 83), using non-coding practice vehicles (markdown, documentation, conceptual exploration) that transfer perfectly when Python starts (Part 4)."

**Context**: Part 3, Chapter 10 of "AI Native Software Development" book (B1 tier, intermediate audience post-tool-onboarding). Students have completed Chapters 1-9 (orientation, tools, markdown) but have NOT yet learned Python (Part 4, Chapters 12-29) or any programming. This chapter teaches COMPLETE prompt engineering methodology conceptually and methodologically, preparing students to apply these patterns when coding begins.

## Success Criteria *(mandatory - evals-first)*

### Measurable Outcomes

**Learning Effectiveness** (methodology mastery, not coding):

- **SC-001**: 80% of students can apply Persona + Questions + Principles pattern to generate effective prompts for documentation exploration tasks
- **SC-002**: 75% of students demonstrate evals-driven iteration (60% baseline → 95%+ refined) on markdown generation assignments
- **SC-003**: Students complete prompt refinement tasks 3x faster after learning systematic methodology vs trial-and-error baseline
- **SC-004**: 70% of students can articulate when to use specification-first vs exploratory prompting (decision framework internalized)

**Constitutional Compliance**:

- **SC-005**: 100% of lessons respect B1 cognitive load limits (7-10 concepts per section max) — validated through concept density audit
- **SC-006**: 100% of technical claims verified with citations from RESEARCH-REPORT.md sources — validated through factual-verifier agent
- **SC-007**: Chapter demonstrates all 4 teaching stages (Manual → AI-Assisted → Intelligence Design → Spec-Driven) — validated through pedagogical-designer review
- **SC-008**: Zero unverified statistics or hallucinated framework claims — validated through fact-checking against research sources

**Platform Specificity** (using tools students already know):

- **SC-009**: Students can correctly select appropriate Claude Code tool (Read vs WebFetch vs Grep) for 5 distinct documentation exploration scenarios
- **SC-010**: Students can write functional Gemini CLI custom command (TOML format) for markdown generation workflow automation

**Methodology Transfer** (preparing for Part 4):

- **SC-011**: 80% of students can explain how debugging protocol learned with markdown transfers to Python debugging (conceptual understanding validated)
- **SC-012**: Students can map 4-layer context model to system architecture exploration (mental model assessment)
- **SC-013**: 75% of students create reusable prompt skill using Persona + Questions + Principles pattern that works across multiple domains

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Developer Learns Framework Through Strategic Prompting (Priority: P1)

**Scenario**: Student needs to understand FastAPI framework conceptually (mental models, design decisions, architectural patterns) before writing any code in Part 4. Uses AI to explore official documentation systematically rather than surface-level "explain FastAPI" queries.

**Why this priority**: Core value proposition - teaching specification-first, evals-driven prompting methodology that students will use throughout entire book. Demonstrates how strategic prompting builds deep understanding vs passive information consumption.

**Independent Test**: Can be fully tested by providing FastAPI documentation URL and validation that students can produce accurate conceptual architecture diagram, identify 3 key design decisions, and explain framework philosophy WITHOUT writing any code.

**Acceptance Scenarios**:

1. **Given** FastAPI documentation URL, **When** student uses specification-first prompting (defines WHAT understanding they need before exploring HOW framework works), **Then** student produces mental model diagram showing request lifecycle, dependency injection, and async handling
2. **Given** initial vague prompt ("explain FastAPI") producing 60% quality overview, **When** student applies evals-driven iteration with explicit success criteria, **Then** student achieves 95%+ conceptual accuracy validated against official docs
3. **Given** recurring documentation exploration pattern (encountered 2+ times), **When** student encodes pattern as reusable prompt skill using Persona + Questions + Principles, **Then** skill transfers to exploring Django, Flask, or any framework documentation

---

### User Story 2 - Developer Generates Production-Quality Markdown (Priority: P2)

**Scenario**: Student creates technical specification document (markdown format) for hypothetical project. First attempt is vague and incomplete (60% quality). Through systematic prompt refinement using Jake Heller's methodology (define success → measure baseline → iterate → production quality), achieves 95%+ quality matching professional standards.

**Why this priority**: Hands-on practice with complete methodology (specification-first, evals-driven iteration, systematic accuracy) using substrate students can execute now (markdown). Builds muscle memory for prompt refinement that transfers to code generation later.

**Independent Test**: Student starts with vague prompt ("create project spec"), measures against rubric (60% baseline), applies systematic refinement prompts, produces final document rated 95%+ by rubric covering completeness, clarity, and professionalism.

**Acceptance Scenarios**:

1. **Given** project idea and specification rubric, **When** student writes specification-first prompt (defines WHAT document should contain before generating), **Then** AI produces structured outline covering all required sections
2. **Given** 60% baseline draft with vague requirements, **When** student applies constraint-driven refinement prompts (explicit criteria: testable, unambiguous, measurable), **Then** requirements achieve 95%+ rubric score
3. **Given** need for consistent formatting across multiple specs, **When** student creates Gemini CLI custom TOML command encoding best-practice prompt pattern, **Then** command generates production-quality specs repeatably

---

### User Story 3 - Developer Debugs Systematic Protocol (Priority: P2)

**Scenario**: Student encounters markdown rendering issue (lists not displaying correctly). Instead of generic "help fix this", applies structured debugging protocol: isolate symptom → form hypothesis → test systematically → validate fix. Learns methodology that transfers perfectly to Python debugging in Part 4.

**Why this priority**: Demonstrates debugging as systematic process, not random trial-and-error. Using markdown (students' current capability) to learn protocol that applies to ANY debugging scenario (code, configuration, deployment).

**Independent Test**: Student debugs markdown rendering problem using structured protocol, produces root cause analysis, validates fix, explains protocol steps. Assessment verifies methodology understanding (can articulate protocol), not just solution.

**Acceptance Scenarios**:

1. **Given** broken markdown with lists not rendering, **When** student uses AI with structured debugging prompts (symptom → hypothesis → targeted tests), **Then** student identifies root cause (incorrect indentation) and validates fix
2. **Given** initial generic prompt ("fix my markdown") producing scattered suggestions, **When** student applies specification-first debugging (WHAT needs investigation before HOW to fix), **Then** AI performs systematic analysis with hypothesis testing
3. **Given** debugging protocol pattern (isolation → hypothesis → test → validate), **When** student encodes as reusable prompt skill, **Then** skill transfers to bash script debugging, git workflow issues, or any troubleshooting

---

### User Story 4 - Developer Designs System Architecture Conceptually (Priority: P3)

**Scenario**: Student explores microservices architecture pattern through AI-guided investigation. Uses Socratic prompting and 4-layer context model to build deep mental models about service boundaries, communication patterns, data consistency, and trade-offs WITHOUT implementing any code.

**Why this priority**: Supporting use case demonstrating strategic thinking about systems design. Prepares students for Part 6 (agent building) where architectural decisions become critical. Uses conceptual exploration (students' current capability) to build transferable mental models.

**Independent Test**: Student produces architecture decision document covering: service decomposition rationale, communication patterns with trade-offs, data consistency strategy, and failure mode analysis. Validated against system design rubric (no code required).

**Acceptance Scenarios**:

1. **Given** microservices concept, **When** student uses Socratic prompting (analytical questions guiding discovery), **Then** student articulates 3 service boundary decision frameworks and trade-offs
2. **Given** complex architecture with multiple subsystems, **When** student applies 4-layer context model (conceptual → logical → physical → operational layers), **Then** student produces coherent architecture exploration covering all layers
3. **Given** need to compare architectural approaches (microservices vs monolith), **When** student uses specification-first analysis (WHAT criteria matter before comparing HOW each works), **Then** student produces decision framework with context-specific recommendations

---

### Edge Cases

- **What happens when documentation exceeds context window limits (1M tokens)?**
  Student must learn context chunking strategies (explore section-by-section, use progressive refinement, leverage subagent patterns for distributed analysis)

- **How does system handle ambiguous prompts that could mean multiple things?**
  Student learns to write falsifiable specifications (explicit success criteria) that remove ambiguity. If AI interpretation differs from intent, specification was underspecified.

- **What if AI provides outdated information from training data?**
  Student validates using WebFetch tool to check official documentation, cross-references multiple sources, explicitly requests version-specific information in prompts.

- **How to handle conceptual topics with no "right answer" (architectural trade-offs)?**
  Student learns to prompt for decision frameworks (criteria, trade-offs, context dependencies) rather than absolute recommendations. AI becomes reasoning partner, not oracle.

## Requirements *(mandatory)*

### Functional Requirements

**Prompt Engineering Methodology** (complete curriculum):

- **FR-001**: Chapter MUST teach Persona + Questions + Principles pattern as core prompt design methodology (from reasoning activation research)
- **FR-002**: Chapter MUST demonstrate specification-first thinking (WHAT before HOW) applied to all prompt engineering examples
- **FR-003**: Chapter MUST teach Jake Heller's evals-driven iteration framework (define success → measure baseline 60-70% → systematic refinement → production quality 90%+)
- **FR-004**: Chapter MUST cover 4-layer context engineering model (conceptual → logical → physical → operational) for complex prompting tasks
- **FR-005**: Chapter MUST teach systematic debugging protocol (isolate → hypothesize → test → validate) using non-coding examples

**Pedagogical Methodology** (constitutional compliance):

- **FR-006**: Chapter MUST apply 4-Stage Teaching Framework (Manual foundation → AI-assisted practice → Intelligence design → Spec-driven integration) per Constitution Section IIa
- **FR-007**: Lessons MUST demonstrate specification-first teaching modality (show WHAT prompts achieve before HOW to write them)
- **FR-008**: Lessons MUST incorporate Socratic dialogue (analytical questions guiding discovery, not direct instruction)
- **FR-009**: Chapter MUST include Three Roles CoLearning pattern (AI as Teacher, Student, Co-Worker) in Stage 2 lessons per Constitution
- **FR-010**: Each lesson MUST respect B1 cognitive load limits (7-10 concepts per section max) per Constitution Principle 2

**Practice Vehicle Selection** (developmental sequencing):

- **FR-011**: Chapter MUST use ONLY non-coding practice examples since students haven't learned Python yet (Chapter 10 of 83, coding starts Chapter 12+)
- **FR-012**: All hands-on exercises MUST use substrates students can execute NOW: markdown generation, documentation exploration, bash commands, git workflows, conceptual system design
- **FR-013**: Chapter MUST explicitly connect each prompt pattern to future coding applications ("This debugging protocol works identically with Python errors in Part 4")
- **FR-014**: Examples MUST demonstrate complete methodology (not toy examples) while avoiding code: architecture exploration via documentation, specification writing, conceptual debugging, framework learning

**Platform-Specific Coverage** (using tools students already know):

- **FR-015**: Chapter MUST include minimum 2 lessons dedicated to Claude Code tool ecosystem (Read, WebFetch, Grep for documentation exploration and markdown workflows)
- **FR-016**: Chapter MUST include minimum 1 lesson dedicated to Gemini CLI command syntax and custom TOML commands for markdown generation automation
- **FR-017**: Chapter MUST cover project memory files (CLAUDE.md, GEMINI.md) for persistent context management in non-coding projects
- **FR-018**: Chapter MUST explain when/how to use subagent patterns (Explore, Plan modes) for complex conceptual investigation tasks

**Intelligence Accumulation** (reusable skills):

- **FR-019**: Chapter MUST create minimum 3 reusable prompt skills (Layer 3 intelligence): documentation-exploration, markdown-generation, debugging-protocol
- **FR-020**: Each reusable skill MUST use Persona + Questions + Principles pattern (not technology-locked templates)
- **FR-021**: Chapter MUST reference prerequisite concepts from Chapters 7 (Bash), 8 (Git), 9 (Markdown) as integrated examples
- **FR-022**: Final lesson (Stage 4) MUST demonstrate capstone composition of accumulated prompt skills, NOT standalone new content

**Factual Accuracy** (research grounding):

- **FR-023**: Chapter MUST remove all unverified statistical claims from previous version ("55% more productive", "70% first try" accuracy rates)
- **FR-024**: All Claude Code tool capabilities (Read, Write, Edit, Grep, Glob, Bash, WebFetch, WebSearch) MUST be documented with verified examples from RESEARCH-REPORT.md
- **FR-025**: All Gemini CLI commands (@filename, !command, custom TOML) MUST be documented with verified examples from RESEARCH-REPORT.md
- **FR-026**: All prompt engineering best practices MUST cite authoritative sources (Anthropic docs, Google docs, research papers) - no invented frameworks

**Minimal Essential Content** (clarity over comprehensiveness):

- **FR-027**: Chapter MUST define explicit non-goals (what we're NOT teaching: advanced RAG, fine-tuning, model training, production deployment)
- **FR-028**: Every section MUST map to specific learning objective (validated through content audit - tangential material removed)
- **FR-029**: Chapter MUST have single "Try With AI" closure section per lesson (NO "Key Takeaways", "Summary", "What's Next" sections per Constitution)

### Key Entities *(conceptual, not data)*

- **Prompt Pattern**: Reusable template structure (Persona + Questions + Principles) that activates reasoning in AI agents for specific task types
- **Evaluation Criteria**: Explicit success metrics defining "what good looks like" for prompt outputs (enables evals-driven iteration)
- **Context Layer**: One of four abstraction levels (conceptual → logical → physical → operational) for systematic information organization
- **Debugging Protocol**: Structured methodology (isolate → hypothesize → test → validate) transferable across any troubleshooting domain
- **Reusable Prompt Skill**: Encoded intelligence object (Persona + Questions + Principles + Domain context) that works across multiple problem instances

## Constraints *(optional)*

### Scope Constraints

- Students at Chapter 10 have completed Chapters 1-9 (tools, markdown) but NOT learned Python or any programming language
- All hands-on practice MUST use non-coding substrates: markdown, documentation, bash, git, conceptual design
- Chapter prepares for Part 4 (Python) and Part 6 (Agents) but does NOT teach coding or implementation
- Platform coverage limited to Claude Code and Gemini CLI (tools students have already installed in Chapters 5-6)

### Constitutional Constraints

- B1 tier cognitive load: maximum 7-10 concepts per section (intermediate audience with moderate scaffolding)
- Teaching modality MUST vary from Chapter 9 (previous chapter used direct teaching; this chapter uses specification-first + Socratic dialogue)
- Stage progression MUST be explicit: Manual foundation → AI-assisted → Intelligence design → Spec-driven integration
- No Stage labels in student-facing content (planning artifact only)

### Time Constraints

- Chapter duration: 6-8 hours total instruction time (assumes students work through at moderate pace)
- Lesson duration: 50-80 minutes each (based on concept density and cognitive load)
- Capstone project: 120-150 minutes (spec-driven composition of accumulated prompt skills)

### Research Grounding Constraints

- All platform capabilities MUST be verified in RESEARCH-REPORT.md (no assumptions from training data)
- No hallucinated statistics or unverified frameworks
- All best practices MUST cite authoritative sources
- Version-specific features MUST note version number and documentation URL

## Non-Goals *(optional)*

### Explicitly Out of Scope

- **Advanced prompt techniques**: Not teaching RAG, fine-tuning, few-shot learning, chain-of-thought prompting (Part 6 material)
- **Production deployment**: Not covering API integration, cost optimization, rate limiting, monitoring (Part 7 material)
- **Code generation**: Not teaching how to prompt for Python code (students don't know Python yet - Part 4 covers this)
- **Generic ChatGPT prompting**: Not covering web interface tricks, casual conversational prompting, or non-developer use cases
- **Prompt libraries/repositories**: Not creating exhaustive template collections (focus on methodology, not templates)
- **Automated prompt optimization**: Not covering meta-prompting, self-critique loops, or automated refinement (advanced topics)

### Deferred to Later Chapters

- **Python-specific prompting**: Deferred to Part 4 chapters where Python is taught
- **Agent architecture prompting**: Deferred to Part 6 (agent building)
- **System integration prompting**: Deferred to Part 7 (cloud-native development)
- **Specialized domains**: Security prompting, performance optimization, accessibility - integrated throughout book, not isolated here

### Assumptions *(optional)*

- Students have reliable internet access (required for Claude Code/Gemini CLI tools)
- Students can execute bash commands and git operations (prerequisite from Chapters 7-8)
- Students understand markdown syntax (prerequisite from Chapter 9)
- Students have installed Claude Code OR Gemini CLI (prerequisite from Chapters 5-6)
- Students can access official documentation websites (for WebFetch exercises)
- Students are comfortable with English technical documentation (book language assumption)

## Dependencies *(optional)*

### Internal Dependencies (Prerequisites)

- **Chapter 5**: Claude Code installation and basic usage (required for tool-specific exercises)
- **Chapter 6**: Gemini CLI installation and npm basics (required for TOML custom commands)
- **Chapter 7**: Bash essentials (required for shell command prompting and script automation examples)
- **Chapter 8**: Git/GitHub workflows (required for version control prompting examples)
- **Chapter 9**: Markdown mastery (required for markdown generation exercises)

### Internal Dependencies (Downstream)

- **Part 4 (Chapters 12-29)**: Python learning will leverage prompt patterns from this chapter (methodology transfers to code generation)
- **Part 5 (Chapters 30-33)**: Spec-Driven Development will compose prompt skills from this chapter (specification writing + AI orchestration)
- **Part 6 (Chapters 34-49)**: Agent building will apply debugging protocol and reusable skills (scaling prompt methodology)

### External Dependencies (Resources)

- **RESEARCH-REPORT.md**: Verified Claude Code/Gemini CLI capabilities (source of truth for platform features)
- **Anthropic Documentation**: Official Claude Code tool reference (for Read, WebFetch, Grep examples)
- **Google Gemini CLI Docs**: Official command syntax and TOML format (for custom command examples)
- **FastAPI Documentation**: Example framework for documentation exploration exercises (accessible, well-documented, relevant to Part 4)
- **Sample Markdown Projects**: Example repositories for markdown debugging and generation practice

### Validation Dependencies

- **Constitution v6.0.0**: Reasoning-activated pedagogy framework (governs all content decisions)
- **Chapter-index.md**: B1 tier specification and prerequisite mapping (defines cognitive load limits)
- **Lesson-template.md**: CoLearning lesson structure (ensures consistent Three Roles pattern)

## Acceptance Criteria *(optional)*

### Content Completeness

- ✅ All 8 lessons implement 4-Stage Teaching Framework progression (Manual → AI-assisted → Intelligence → Spec-driven)
- ✅ Every lesson respects B1 cognitive load (7-10 concepts max, explicitly tracked)
- ✅ All hands-on exercises use non-coding substrates (markdown, documentation, bash, git, conceptual design)
- ✅ Three reusable prompt skills created with Persona + Questions + Principles structure
- ✅ Capstone project composes accumulated skills (not standalone)

### Methodology Coverage

- ✅ Persona + Questions + Principles pattern taught and practiced across multiple lessons
- ✅ Specification-first thinking demonstrated in minimum 3 different contexts
- ✅ Evals-driven iteration (60% → 95%+ journey) shown with concrete rubrics
- ✅ 4-layer context model applied to complex documentation exploration task
- ✅ Systematic debugging protocol practiced with markdown/bash/git examples

### Platform Specificity

- ✅ Claude Code: 14 tools documented with verified capabilities (Read, Write, Edit, Grep, Glob, Bash, WebFetch, WebSearch, TodoWrite, plus 5 others)
- ✅ Gemini CLI: @filename, !command, custom TOML commands demonstrated with working examples
- ✅ Project memory files: CLAUDE.md and GEMINI.md templates created and explained
- ✅ Subagent patterns: When to use Explore/Plan modes for complex tasks

### Factual Accuracy

- ✅ Zero unverified statistics (removed "55% productive", "70% first try" from previous version)
- ✅ All tool capabilities cited from RESEARCH-REPORT.md
- ✅ All best practices cite authoritative sources (Anthropic, Google, research papers)
- ✅ Version-specific features note version number

### Constitutional Compliance

- ✅ Teaching modality varies from Chapter 9 (specification-first + Socratic vs direct teaching)
- ✅ Three Roles CoLearning demonstrated in Stage 2 lessons (AI as Teacher/Student/Co-Worker)
- ✅ No Stage labels in student-facing content (planning artifact only)
- ✅ Single "Try With AI" closure per lesson (no summaries/key takeaways)
- ✅ All sections map to learning objectives (no tangential content)

### Transfer Preparation

- ✅ Each prompt pattern explicitly connects to future coding applications (Part 4 preview)
- ✅ Debugging protocol transfers from markdown to Python clearly explained
- ✅ Documentation exploration methodology prepares for framework learning (Part 6)
- ✅ Reusable skills architecture demonstrates intelligence accumulation principle

## Risks & Mitigations *(optional)*

### Risk 1: Students Frustrated by "No Coding" Examples

**Likelihood**: Medium
**Impact**: High (could reduce engagement)
**Mitigation**:
- Frame every example with "This exact methodology works for Python debugging in Part 4"
- Use realistic scenarios (documentation exploration, specification writing) that feel professional, not contrived
- Show actual value of methodology through measurable improvement (60% → 95%+ on rubrics)
- Include testimonials/previews of how prompt skills accelerate Part 4 learning

### Risk 2: Methodology Feels Abstract Without Code

**Likelihood**: Medium
**Impact**: Medium (harder to internalize patterns)
**Mitigation**:
- Use concrete, hands-on exercises with immediate feedback (markdown renders correctly or doesn't)
- Provide rubrics for every exercise so students can measure their own improvement
- Create "before/after" examples showing dramatic quality improvement from methodology
- Use familiar domains (git, bash, markdown) so cognitive load goes to methodology, not substrate

### Risk 3: Platform Capabilities Change (Claude Code/Gemini CLI Updates)

**Likelihood**: High (tools actively developed)
**Impact**: Low (examples may break but methodology stays valid)
**Mitigation**:
- Version-pin examples with documentation URLs (e.g., "as of Claude Code 0.8, Read tool supports PDFs - see docs.anthropic.com/...")
- Focus on methodology over tool-specific tricks (Persona + Questions + Principles works regardless of tool version)
- Create maintenance checklist for chapter (verify examples quarterly against latest tool versions)
- Use stable core features (Read, WebFetch) over cutting-edge capabilities

### Risk 4: Cognitive Load from New Methodology

**Likelihood**: Low (B1 tier, 7-10 concepts max)
**Impact**: Medium (could overwhelm if not managed)
**Mitigation**:
- Progressive disclosure: Persona in Lesson 1, Questions in Lesson 2, Principles in Lesson 3, full pattern in Lesson 4+
- Concrete before abstract: Practice with markdown generation before teaching meta-framework
- Scaffolded exercises: Simple (specification-first) → Moderate (+ Socratic) → Complex (full Persona + Questions + Principles)
- Explicit concept tracking: Each lesson lists exact concepts introduced (students can self-monitor load)

## Timeline & Milestones *(optional)*

### Implementation Phases

**Phase 1: Infrastructure** (Est. 4 hours)
- Create chapter directory structure
- Curate sample documentation (FastAPI, Django, Flask for exploration exercises)
- Create CLAUDE.md / GEMINI.md templates
- Write chapter README with strategic framing
- Validation: Directory structure correct, samples accessible

**Phase 2: Stage 1 Foundation** (Est. 8 hours)
- Implement Lesson 1: Understanding AI Capabilities (Manual exploration, NO AI use)
- Implement Lesson 2: Specification-First Thinking (Manual practice with rubrics)
- Create "before/after" example set showing methodology value
- Validation: Lessons have zero AI tool usage, concepts ≤7, exercises completable

**Phase 3: Stage 2 AI-Assisted** (Est. 12 hours)
- Implement Lesson 3: Persona + Questions + Principles Pattern (Three Roles demonstration)
- Implement Lesson 4: Claude Code Tool Ecosystem (Three Roles with Read/WebFetch/Grep)
- Implement Lesson 5: Gemini CLI Workflows (Three Roles with @filename/!command/TOML)
- Validation: Three Roles explicitly demonstrated (AI as Teacher/Student/Co-Worker), tools verified against RESEARCH-REPORT.md

**Phase 4: Stage 3 Intelligence Design** (Est. 10 hours)
- Implement Lesson 6: Creating Reusable Prompt Skills (documentation-exploration, markdown-generation, debugging-protocol)
- Implement Lesson 7: Project Memory Architecture (CLAUDE.md/GEMINI.md usage patterns)
- Create skill templates with Persona + Questions + Principles structure
- Validation: Skills reusable across domains (not technology-locked), memory files functional

**Phase 5: Stage 4 Spec-Driven Integration** (Est. 6 hours)
- Implement Lesson 8: Capstone - Systematic Documentation Exploration
- Design spec-first project: "Evaluate framework for hypothetical project" (FastAPI vs Django vs Flask)
- Create evaluation rubric and decision framework template
- Validation: Capstone composes Lessons 1-7 skills (not standalone), produces professional deliverable

**Phase 6: Validation & Polish** (Est. 4 hours)
- Run validation-auditor (constitutional compliance)
- Run factual-verifier (RESEARCH-REPORT.md grounding)
- Execute all code examples (bash, TOML commands)
- Cross-reference prerequisites (Chapters 7-9)
- Validation: Zero critical issues, all examples tested, claims verified

**Total Estimated Time**: 44 hours (design + implementation + validation)

### Milestones

- **M1** (Infrastructure Complete): Directory structure + templates ready
- **M2** (Foundation Lessons): Lessons 1-2 implement Stage 1 (Manual) correctly
- **M3** (AI-Assisted Lessons): Lessons 3-5 demonstrate Three Roles explicitly
- **M4** (Intelligence Created): Lessons 6-7 produce 3 reusable skills
- **M5** (Capstone Ready): Lesson 8 composes accumulated intelligence
- **M6** (Validation Passed): All constitutional + factual checks pass

---

**Status**: Ready for `/sp.clarify` (if needed) or `/sp.plan` (lesson structure design)
