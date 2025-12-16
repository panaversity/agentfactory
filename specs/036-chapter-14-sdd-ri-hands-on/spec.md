# Feature Specification: Chapter 14 — SDD-RI Hands-On with Spec-Kit Plus

**Feature Branch**: `036-chapter-14-sdd-ri-hands-on`
**Created**: 2025-11-25
**Status**: Draft
**Constitutional Version**: 6.0.1
**Part**: 4 (SDD-RI Fundamentals)
**Chapter**: 14 (Position in book structure)
**Proficiency Tier**: A2-B1 (Beginner transitioning to Intermediate)

---

## Constitutional Grounding

This specification applies these constitutional frameworks:

| Framework                               | Application                                                        |
| --------------------------------------- | ------------------------------------------------------------------ |
| **Section IIa: 4-Layer Progression**    | Full L1→L2→L3→L4 through lessons                                   |
| **Principle 2: Progressive Complexity** | A2-B1 = max 7 concepts/section, moderate scaffolding               |
| **Principle 3: Factual Accuracy**       | Playwright MCP + Gemini patterns verified                          |
| **Principle 6: Anti-Convergence**       | Ch 13 = problem-discovery → Ch 14 = **hands-on project execution** |
| **Principle 7: Minimal Content**        | Focus on workflow, not comprehensive tool tutorials                |

**Teaching Modality Variation**: Chapter 13 used problem-discovery through "vague code" failure analysis. Chapter 14 MUST use **project-based hands-on learning** where students execute the complete SDD-RI workflow by building a real product.

---

## User Scenarios & Testing _(mandatory)_

### User Story 1 - Execute Complete SDD-RI Workflow (Priority: P1)

As a student who completed Chapter 13 (SDD-RI theory), I need to apply the Spec-Kit Plus workflow to a real project so that I internalize the methodology through hands-on practice.

**Why this priority**: Core learning objective—theory without practice doesn't transfer.

**Independent Test**: Student can execute `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` for a video generation project from start to finish.

**Acceptance Scenarios**:

1. **Given** a student who completed Chapter 13, **When** they follow Chapter 14 lessons, **Then** they can write a specification for a product demo video that meets Spec-Kit Plus quality criteria (evals, intent, constraints, non-goals)
2. **Given** a completed specification, **When** the student runs `/sp.plan`, **Then** they produce a plan that uses Gemini.google.com + Playwright MCP as the implementation approach
3. **Given** a completed plan, **When** the student runs `/sp.tasks` → `/sp.implement`, **Then** they have a video file downloaded locally on their machine

---

### User Story 2 - Create Reusable Intelligence (Priority: P1)

As a student learning SDD-RI, I need to identify recurring patterns in my workflow and convert them to reusable skills so that I understand how intelligence accumulates across projects.

**Why this priority**: Core differentiator of SDD-RI over plain SDD.

**Independent Test**: Student creates at least 2 skills from their video generation workflow.

**Acceptance Scenarios**:

1. **Given** a completed video generation workflow, **When** the student reflects on patterns, **Then** they identify at least 2 recurring workflows suitable for skill creation
2. **Given** identified patterns, **When** the student applies P+Q+P (Persona + Questions + Principles) framework, **Then** they produce valid skill definitions using Spec-Kit Plus structure
3. **Given** created skills (`generate-video`, `upload-youtube`), **When** the student uses them in the capstone, **Then** the skills activate reasoning (not just template filling)

---

### User Story 3 - Complete Capstone Project (Priority: P2)

As a student, I need to apply all Part 4 learning to an end-to-end project (video creation + YouTube upload) so that I have a portfolio piece demonstrating SDD-RI mastery.

**Why this priority**: Synthesis validates learning transfer.

**Independent Test**: Student has a video uploaded to YouTube using their created skills.

**Acceptance Scenarios**:

1. **Given** completed video generation project + skills, **When** the student tackles YouTube upload, **Then** they reuse their skills instead of starting from scratch
2. **Given** the capstone project, **When** the student completes it, **Then** they can explain the SDD-RI workflow and how skills accelerated their second loop
3. **Given** the completed capstone, **When** evaluated against Chapter 13 learning objectives, **Then** the student demonstrates practical application of SDD, SDD-RI, and Spec-Kit Plus

---

### Edge Cases

**Playwright MCP Availability**:

- What if Playwright MCP isn't installed? (Solution: Lesson 2 covers installation verification)
- What if Gemini UI changes? (Solution: Document selectors in maintainable way, note volatility)

**Authentication**:

- What if session expires mid-workflow? (Solution: Session persistence guidance + recovery steps)
- What if student uses different Google account? (Solution: Generic selectors, not account-specific)

**Video Quality**:

- What if Gemini doesn't produce desired video quality? (Solution: Emphasize iteration loop—SDD handles this through refinement)
- What if video download fails? (Solution: Error handling in spec, retry patterns)

**Skills Creation**:

- What if student creates over-engineered skills? (Solution: Complexity checklist: 2-4 decisions = skill, 5+ = subagent)
- What if skills don't generalize? (Solution: P+Q+P framework guides abstraction)

---

## Requirements _(mandatory)_

### Functional Requirements

**Chapter Structure**:

- **FR-001**: Chapter MUST have 7-9 lessons following pedagogical arc (Foundation → Application → Integration → Validation → Mastery)
- **FR-002**: Each lesson MUST have clear learning objectives mapped to Bloom's taxonomy levels
- **FR-003**: Chapter MUST end with capstone project (Layer 4: Spec-Driven Integration)

**SDD-RI Workflow Coverage**:

- **FR-004**: Chapter MUST demonstrate complete `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow
- **FR-005**: Students MUST write at least one specification from scratch (not copy-paste)
- **FR-006**: Students MUST execute at least one planning phase with AI collaboration
- **FR-007**: Students MUST experience task execution with checkpoint validation

**Reusable Intelligence Creation**:

- **FR-008**: Chapter MUST teach skill identification (recognizing 2+ recurring patterns)
- **FR-009**: Chapter MUST teach P+Q+P framework application for skill design
- **FR-010**: Students MUST create at least 2 skills: `generate-video` and `upload-youtube`
- **FR-011**: Skills MUST follow Spec-Kit Plus structure (SKILL.md format)

**Technical Implementation**:

- **FR-012**: Project MUST use Playwright MCP for browser automation
- **FR-013**: Project MUST target Gemini.google.com for video generation (free tier)
- **FR-014**: Project MUST include YouTube upload automation
- **FR-015**: Authentication MUST use session persistence (student logs in once, Playwright persists)

**Pedagogical Requirements**:

- **FR-016**: Chapter MUST vary teaching modality from Chapter 13 (use hands-on project, not problem-discovery)
- **FR-017**: Chapter MUST demonstrate Three Roles (AI as Teacher, Student, Co-Worker) INVISIBLY (no framework labels)
- **FR-018**: Lessons MUST follow cognitive load limits (max 7 concepts per section for A2-B1)
- **FR-019**: Each lesson MUST end with "Try With AI" section (no "What's Next" or "Key Takeaways")

**Business Context**:

- **FR-020**: Project MUST have clear business framing (product demo videos for a SaaS)
- **FR-021**: Specification MUST include business success metrics (not just technical completion)

### Key Entities

- **Specification (spec.md)**: Feature specification following Spec-Kit Plus template with evals, intent, constraints, non-goals
- **Plan (plan.md)**: Implementation plan detailing technical approach, lessons, pedagogical structure
- **Tasks (tasks.md)**: Atomic work units with dependencies and acceptance criteria
- **Skill (SKILL.md)**: Reusable intelligence component following P+Q+P pattern
- **Capstone Project**: End-to-end demonstration composing all chapter learnings

---

## Success Criteria _(mandatory)_

### Measurable Outcomes

**Workflow Mastery**:

- **SC-001**: 100% of students can execute `/sp.specify` command and produce valid specification
- **SC-002**: 100% of students can execute `/sp.plan` and produce implementation plan
- **SC-003**: 100% of students can execute `/sp.tasks` → `/sp.implement` cycle
- **SC-004**: 90% of students successfully generate a video using the workflow

**Reusable Intelligence**:

- **SC-005**: 100% of students can identify at least 2 reusable patterns from their workflow
- **SC-006**: 90% of students create valid skill definitions following P+Q+P
- **SC-007**: 80% of students successfully reuse skills in capstone (YouTube upload)

**Capstone Completion**:

- **SC-008**: 80% of students complete full capstone (video generation + YouTube upload)
- **SC-009**: 90% of students can explain SDD-RI workflow and skill value proposition
- **SC-010**: All capstone projects demonstrate Layer 4 (spec-driven orchestration)

**Quality Standards**:

- **SC-011**: All code examples in lessons execute successfully (verified in sandbox)
- **SC-012**: All technical claims cite authoritative sources (Playwright docs, Gemini docs)
- **SC-013**: All lessons pass constitutional compliance (no framework labels, proper endings)
- **SC-014**: Chapter varies teaching modality from Chapter 13 (verified by reviewer)

---

## Constraints _(if applicable)_

### Technical Constraints

- MUST use Playwright MCP (not raw Playwright) for browser automation
- MUST target Gemini.google.com (free, no API key required)
- MUST NOT require students to share credentials (session persistence approach)
- MUST work on macOS, Linux, and Windows (Playwright cross-platform)
- MUST handle Gemini UI changes gracefully (selector abstraction)

### Pedagogical Constraints

- MUST follow 4-Layer progression (L1 → L2 → L3 → L4)
- MUST vary modality from Chapter 13 (project-based, not problem-discovery)
- MUST NOT expose pedagogical scaffolding (no "Stage 2", "Three Roles" labels)
- MUST maintain A2-B1 cognitive load limits (max 7 concepts/section)
- MUST end lessons with "Try With AI" only (no "What's Next", "Summary", etc.)

### Process Constraints

- MUST reuse Chapter 13 SDD concepts (no re-teaching fundamentals)
- MUST build on accumulated intelligence (reference Chapter 13 patterns)
- MUST create skills that generalize beyond this specific project

---

## Assumptions _(if applicable)_

- Students have completed Chapter 13 and understand SDD-RI theory
- Students have Claude Code or similar AI companion installed and configured
- Students have Playwright MCP available (covered in earlier chapters or lesson 2)
- Students have Google accounts for Gemini and YouTube access
- Gemini.google.com video generation feature is available (as of 2025)
- Session persistence works reliably for Playwright browser automation

---

## Open Questions

1. **Gemini Video Generation**: What specific Gemini feature generates videos? (Research needed during planning)
2. **Playwright MCP Selectors**: How stable are Gemini.google.com selectors? (Research needed)
3. **YouTube API vs UI**: Should upload use YouTube API or browser automation? (Decision: Browser automation for consistency with Gemini approach)

---

## Dependencies _(if applicable)_

### Prerequisites

- **Chapter 13**: SDD-RI fundamentals (theory foundation)
- **Playwright MCP**: Browser automation tool (installation covered in Ch 14 or prerequisite)
- **Google Account**: For Gemini and YouTube access
- **Claude Code / AI Companion**: For executing Spec-Kit Plus commands

### Downstream Impacts

- **Chapter 15**: AI Product & Business Intelligence Capstone (builds on Ch 14 skills)
- **Part 5 Python**: Can reference SDD-RI workflow established here
- **Skills Library**: `generate-video` and `upload-youtube` skills become reusable

---

## Non-Goals _(important)_

**This chapter does NOT teach**:

- ❌ Comprehensive Playwright tutorial (only what's needed for project)
- ❌ Gemini API usage (uses browser UI, not API)
- ❌ YouTube API (uses browser automation)
- ❌ Advanced video editing (Gemini generates, we don't edit)
- ❌ Subagent creation (skills only for A2-B1 tier)
- ❌ Multiple project types (focuses on product demo videos only)

**Why these are excluded**:

- Playwright depth belongs in dedicated tooling chapters
- API usage adds complexity without pedagogical value for SDD-RI
- Advanced editing is tangential to specification-driven workflow
- Subagents are C1+ complexity (this is A2-B1)

---

## References _(if applicable)_

- **Constitution**: `.specify/memory/constitution.md` (v6.0.1)
- **Chapter 13**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/13-specification-driven-development-fundamentals/`
- **Spec-Kit Plus Commands**: `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`
- **Skill Template**: `.claude/skills/skill-creator/SKILL.md`
- **Playwright MCP**: Official documentation (research during planning)

---

## Additional Context _(if helpful)_

### Pedagogical Arc for Chapter 14

**Lessons Structure** (preliminary):

1. **Foundation** (L1-2): Setup + First Spec

   - L1: Project introduction + business context (product demo videos)
   - L2: Write your first specification with AI collaboration

2. **Application** (L3-5): Execute SDD-RI Workflow

   - L3: Planning phase with Playwright MCP approach
   - L4: Task breakdown and implementation start
   - L5: Complete video generation + download

3. **Integration** (L6-7): Intelligence Creation

   - L6: Identify reusable patterns in your workflow
   - L7: Create skills using P+Q+P framework

4. **Mastery** (L8-9): Capstone
   - L8: YouTube upload using created skills
   - L9: Reflection + complete project presentation

### Three Roles Integration (Invisible)

**In each lesson, students experience**:

- **AI as Teacher**: AI suggests patterns student didn't consider (e.g., better spec structure)
- **AI as Student**: Student teaches AI their constraints (e.g., "must use free tools")
- **AI as Co-Worker**: Iterative refinement toward production-ready output

**Framework stays INVISIBLE**: Students experience these roles through narrative ("AI suggested...", "You refined...", "Together you converged...") NOT through labels ("AI as Teacher", "Three Roles Framework").

### Skills to Create

**`generate-video` skill**:

- Persona: Video content producer
- Questions: What's the product? What's the key message? Target audience?
- Principles: Visual clarity, concise messaging, brand consistency

**`upload-youtube` skill**:

- Persona: Content publisher
- Questions: Video title? Description? Tags? Visibility?
- Principles: SEO optimization, thumbnail selection, scheduling considerations

---

## Evals Section (Success Criteria Synthesis)

### What Observable Behaviors Demonstrate Success?

1. **Workflow Execution**: Student completes full `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` cycle
2. **Artifact Quality**: Specification meets Spec-Kit Plus quality criteria (evals, intent, constraints, non-goals present)
3. **Video Output**: Student has downloaded video file locally
4. **Skill Creation**: Student produces 2 valid skills following P+Q+P pattern
5. **Capstone Completion**: Student uploads video to YouTube using created skills
6. **Transfer Demonstration**: Student can explain how skills accelerate future projects

### How Do We Measure Learning?

| Learning Objective           | Assessment Method     | Success Threshold                |
| ---------------------------- | --------------------- | -------------------------------- |
| Execute SDD workflow         | Project completion    | Video downloaded                 |
| Write production-ready specs | Spec review checklist | 4/4 sections present             |
| Create reusable skills       | Skill validation      | P+Q+P pattern visible            |
| Apply skills in new context  | Capstone completion   | YouTube upload works             |
| Explain SDD-RI value         | Reflection prompt     | Articulates acceleration benefit |

### What Tests Validate These Criteria?

- **Automated**: Spec.md contains required sections (grep validation)
- **Automated**: SKILL.md follows P+Q+P structure
- **Manual**: Video plays correctly (student self-validates)
- **Manual**: YouTube upload successful (student self-validates)
- **Manual**: Reflection demonstrates understanding (instructor review)
