# Feature Specification: Chapter 9 Markdown Redesign - Specification Language Approach

**Feature Branch**: `034-chapter-9-markdown-redesign`
**Created**: 2025-11-18
**Status**: Draft
**Input**: User description: "Rewrite Chapter 9 (Markdown - The Language of AI Communication) with critical constraint: NO programming code examples (students haven't learned any language yet)"

**Constitutional Frameworks Applied**:
- Section IIa: 4-Stage Teaching Framework (Manual → AI Collaboration → Intelligence Design → Spec-Driven Integration)
- Principle 1 (Specification Primacy): Markdown IS the specification language itself
- Principle 2 (Progressive Complexity): A2 tier, 5-7 concepts per section
- Principle 6 (Anti-Convergence): Specification-first modality (varies from Chapter 8 hands-on discovery)

## Evals (Success Criteria) *(mandatory)*

### Observable Student Behaviors

**SC-001**: Students can write clear feature specifications in markdown that AI agents parse correctly without requiring programming knowledge

**SC-002**: 80% of students successfully complete capstone project (task management system specification) using accumulated markdown skills from all 5 lessons

**SC-003**: Students can distinguish specification examples (WHAT system does) from implementation code (HOW it's built) when shown both

**SC-004**: Students demonstrate Three Roles collaboration: AI suggests specification structure → Student refines requirements → Converge on clarity (measured through "Try With AI" exercises)

**SC-005**: Zero code examples present in any lesson (100% compliance with no-code constraint across all 5 lessons)

### Measurable Assessment Criteria

**AC-001**: Each lesson's "Try With AI" section produces valid specification artifacts (feature lists, acceptance criteria, or system requirements) that follow markdown syntax correctly

**AC-002**: Capstone specification (Lesson 5) includes all required components: headings for organization, lists for requirements, code blocks for expected outputs, links to reference materials—all without implementation code

**AC-003**: Students can explain markdown's role as Intent Layer in SDD-RI methodology (assessed through comprehension questions)

**AC-004**: Cognitive load maintained at A2 tier: each section introduces maximum 7 concepts, with heavy scaffolding and validation checkpoints

## Intent *(mandatory)*

### WHAT We're Building

**Chapter 9 Redesign**: Transform "Markdown - The Language of AI Communication" from generic markdown tutorial (with code examples) to specification language curriculum (without code examples).

**Core Transformation**:
- **FROM**: Markdown as documentation formatting tool for developers (current README.md approach with Python/Bash examples in Lesson 4)
- **TO**: Markdown as structured specification language for communicating intent to AI agents (pre-programming beginners)

**5-Lesson Structure** (following 4-stage progression):

1. **Lesson 1 (Stage 1 - Manual Foundation)**: Introduction to markdown as specification language + basic headings for document structure
2. **Lesson 2 (Stage 1-2 Transition)**: Lists for organizing features and requirements + introduction to AI-assisted spec creation
3. **Lesson 3 (Stage 2 - AI Collaboration)**: Code blocks for specifications (NOT code) + expected outputs + Three Roles demonstrated
4. **Lesson 4 (Stage 3 - Intelligence Design)**: Links, images, and creating reusable specification templates
5. **Lesson 5 (Stage 4 - Spec-Driven Integration)**: Complete task management system specification capstone composing all skills

### WHY This Matters

**Foundation for Part 5 (Spec-Driven Development)**:
- Chapter 9 is Part 3 (Markdown, Prompt & Context Engineering) - students haven't learned programming yet
- Part 4 teaches Python (Chapters 12-29)
- Part 5 teaches Spec-Driven Development (Chapters 30-33)
- **Sequencing requirement**: Markdown must be taught WITHOUT assuming code literacy, as it precedes programming instruction

**Markdown as Intent Layer**:
- In SDD-RI methodology, markdown is the primary language for specifications (spec.md files)
- Students must master specification writing (markdown) BEFORE implementation (code)
- This chapter teaches "how to tell AI agents WHAT to build" not "how to code"

**Pedagogical Innovation**:
- Demonstrates specification-first thinking from day one (even before students write code)
- Shows that clear specifications → clear AI outputs (fundamental principle for AI-native development)
- Builds mental model: Structure communication (markdown) → AI parses structure → Implementation emerges

### WHO This Is For

**Primary Audience**: A2 tier complete beginners in Part 3 (foundational)

**Student Profile**:
- Completed Chapters 1-8 (AI tool landscape, Bash basics, Git fundamentals)
- Has NOT learned Python or any programming language yet
- Understands AI tools (Claude Code, Gemini CLI) from Chapter 5-6
- Can execute basic terminal commands (Bash from Chapter 7)
- Knows version control concepts (Git from Chapter 8)

**Prerequisites Knowledge**:
- Can create and edit text files
- Understands concept of "communicating with AI agents through prompts"
- Familiar with structured thinking (from context engineering in upcoming Chapter 11)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understand Markdown as Specification Language (Priority: P1)

**Student journey**: A complete beginner needs to understand WHY markdown matters for AI communication and HOW it differs from unstructured text.

**Why this priority**: Foundation for all subsequent lessons. Without understanding markdown's role as specification language (not formatting tool), students will converge toward generic README-writing instead of specification thinking.

**Independent Test**: Student can explain in own words: "Markdown helps AI agents understand my requirements because it's structured" and can identify difference between specification example and implementation code when shown both.

**Acceptance Scenarios**:

1. **Given** student reads Lesson 1, **When** asked "Why use markdown instead of plain text?", **Then** student answers using specification concepts (structure, AI parsing, clarity) not formatting concepts (bold text, pretty README)

2. **Given** student completes Lesson 1 manual exercise, **When** student writes simple feature specification using headings, **Then** markdown syntax is correct and headings create meaningful hierarchy (Feature > Sub-feature structure)

3. **Given** student sees two examples (specification vs code), **When** asked to identify which is which, **Then** student correctly identifies specification example (describes WHAT) vs implementation code (shows HOW)

---

### User Story 2 - Write Structured Requirements Using Lists (Priority: P1)

**Student journey**: Student needs to organize features and requirements using markdown lists so AI agents can parse discrete requirements vs continuous prose.

**Why this priority**: Lists are fundamental for specification writing. Without mastering lists, students cannot write clear requirement enumeration (bullet lists for options, numbered lists for sequential steps).

**Independent Test**: Student can convert unstructured feature description into structured markdown lists (bullets for feature options, numbers for implementation steps) without code examples.

**Acceptance Scenarios**:

1. **Given** unstructured feature description, **When** student converts to markdown lists, **Then** bullet lists used for alternative options and numbered lists used for sequential requirements

2. **Given** student writes feature specification, **When** AI agent reads markdown lists, **Then** AI correctly identifies discrete requirements (not continuous paragraph interpretation)

3. **Given** student completes Lesson 2 AI collaboration exercise, **When** AI suggests list structure, **Then** student evaluates suggestion and refines requirements (demonstrates AI as Teacher role)

---

### User Story 3 - Use Code Blocks for Specifications (Not Code) (Priority: P1)

**Student journey**: Student learns that code blocks (triple backticks) can show specifications, expected outputs, and system requirements WITHOUT implementation code.

**Why this priority**: Critical transformation from current Lesson 4 (which uses Python/Bash code blocks). Students must understand code blocks serve specification purposes (showing expected behavior, API descriptions, acceptance criteria) not code syntax.

**Independent Test**: Student can use code blocks to show expected system output, feature requirements, or API endpoint descriptions without writing implementation code. Demonstrates Three Roles collaboration (AI suggests structure, student refines content, converge on clarity).

**Acceptance Scenarios**:

1. **Given** feature requiring specific output format, **When** student uses code block, **Then** block shows expected output specification (e.g., "System displays: Welcome, [Username]!") not implementation code

2. **Given** API endpoint specification needed, **When** student uses code block, **Then** block describes endpoint behavior (request format, response structure) without implementation language (no Python, JavaScript, etc.)

3. **Given** student collaborates with AI on specification, **When** AI suggests code block structure, **Then** student teaches AI domain constraints (corrects generic examples with project-specific requirements - demonstrates AI as Student role)

4. **Given** student and AI iterate on specification, **When** convergence loop completes, **Then** final specification emerged that neither had at start (demonstrates AI as Co-Worker role)

---

### User Story 4 - Create Reusable Specification Templates (Priority: P2)

**Student journey**: Student recognizes recurring specification patterns (feature template, acceptance criteria template, user story template) and encodes them as reusable markdown templates.

**Why this priority**: Stage 3 (Intelligence Design) requires creating reusable components. Students transition from writing individual specs to designing reusable intelligence that compounds across projects.

**Independent Test**: Student creates at least one reusable template (feature specification template OR acceptance criteria template OR user story template) that includes links to reference materials and can be applied to multiple projects.

**Acceptance Scenarios**:

1. **Given** student has written specifications in Lessons 1-3, **When** student identifies recurring pattern (e.g., feature description structure), **Then** student creates reusable template with markdown placeholders

2. **Given** reusable template created, **When** student applies template to new feature, **Then** only content changes (template structure remains consistent - demonstrates intelligence reusability)

3. **Given** student adds links/images to template, **When** template used in future projects, **Then** links provide references to authoritative sources (documentation, examples, standards) not implementation tutorials

---

### User Story 5 - Write Complete System Specification (Capstone) (Priority: P1)

**Student journey**: Student composes all accumulated markdown skills (headings, lists, code blocks, links, images) into complete task management system specification using specification-first approach.

**Why this priority**: Stage 4 (Spec-Driven Integration) validates that students can orchestrate accumulated intelligence. Capstone demonstrates mastery of markdown as specification language without code implementation.

**Independent Test**: Student produces complete task management system specification including: system overview (headings), feature list (lists), expected behavior (code blocks for outputs), reference materials (links). Specification is clear enough that AI agent could implement system from spec alone (tested through AI feedback on specification clarity).

**Acceptance Scenarios**:

1. **Given** capstone project prompt (task management system), **When** student writes specification, **Then** spec includes all required components: headings (system organization), lists (features/requirements), code blocks (expected outputs), links (references)

2. **Given** completed specification, **When** AI agent reads spec, **Then** AI can identify discrete features to implement (demonstrates specification parsability)

3. **Given** specification validation exercise, **When** student receives AI feedback, **Then** student can refine specification based on feedback (demonstrates specification iteration without implementation)

4. **Given** capstone completion, **When** assessed against success criteria, **Then** specification contains ZERO implementation code (100% compliance with no-code constraint)

---

### Edge Cases

**Cognitive Load Boundary**: What happens when concept density exceeds A2 tier limit (7 concepts per section)?
- **Response**: Section must be split into multiple subsections or lessons restructured to distribute concepts across more lessons

**AI Collaboration Failure**: How does student handle when AI suggests implementation code instead of specification?
- **Response**: Lesson includes explicit guidance: "If AI provides code, prompt: 'Please show specification (WHAT system does) not code (HOW it's implemented)'"

**Specification Ambiguity**: What happens when student writes vague specification that AI misinterprets?
- **Response**: Validation checkpoints in each lesson where AI provides feedback on specification clarity (student learns through iteration)

**Template Over-Engineering**: How does system prevent students from creating overly specific templates (violates reusability)?
- **Response**: Lesson 4 includes decision framework: "Template applies to 3+ projects? Reusable. Template specific to one technology? Over-engineered."

**Capstone Complexity Overwhelm**: What happens when capstone project scope exceeds beginner capacity?
- **Response**: Task management system scoped to A2 tier: 3-4 core features (create task, view tasks, mark complete, delete task) with clear boundaries in lesson prompt

## Requirements *(mandatory)*

### Functional Requirements

**Chapter Structure Requirements**:

- **FR-001**: Chapter MUST contain exactly 5 lessons following 4-stage pedagogical progression (Manual → AI Collaboration → Intelligence Design → Spec-Driven Integration)
- **FR-002**: Each lesson MUST maintain A2 tier cognitive load (maximum 7 concepts per section with heavy scaffolding)
- **FR-003**: Chapter MUST use specification-first teaching modality (students write specs for hypothetical systems) to vary from Chapter 8 hands-on discovery

**No-Code Constraint Requirements**:

- **FR-004**: ALL lessons MUST use specification examples ONLY (feature lists, acceptance criteria, expected outputs, API descriptions, system requirements)
- **FR-005**: ZERO implementation code examples permitted (no Python, Bash, JavaScript, or any programming language syntax)
- **FR-006**: Code blocks (triple backticks) MUST demonstrate specification usage (showing expected behavior, not implementation code)

**Stage 1 (Manual Foundation) Requirements**:

- **FR-007**: Lesson 1 MUST teach markdown basics through manual writing (students create markdown by hand in text editor, no AI assistance for foundation)
- **FR-008**: Lesson 1 MUST establish mental model: Markdown = structured communication for AI agents (not formatting tool)
- **FR-009**: Lesson 1 MUST teach headings (`#` symbols) for document hierarchy with specification examples (Feature > Sub-feature > Requirement structure)

**Stage 2 (AI Collaboration) Requirements**:

- **FR-010**: Lessons 2-3 MUST demonstrate Three Roles framework through narrative (AI as Teacher + Student + Co-Worker) WITHOUT using "Stage 2" or "Three Roles" labels in student-facing text
- **FR-011**: Each Stage 2 lesson MUST include at least ONE instance where AI teaches student (suggests pattern student didn't know)
- **FR-012**: Each Stage 2 lesson MUST include at least ONE instance where student teaches AI (corrects or refines AI output with domain constraints)
- **FR-013**: Each Stage 2 lesson MUST include at least ONE convergence loop (iterative refinement toward optimal specification)
- **FR-014**: Lesson 2 MUST teach lists (bullet `-` and numbered `1.`) for organizing features/requirements with AI-assisted spec generation
- **FR-015**: Lesson 3 MUST teach code blocks for specifications with Three Roles demonstrated (AI suggests structure, student refines requirements, converge on clarity)

**Stage 3 (Intelligence Design) Requirements**:

- **FR-016**: Lesson 4 MUST teach links `[text](url)` and images `![alt](url)` for references and diagrams in specifications
- **FR-017**: Lesson 4 MUST guide students to create reusable specification templates (feature template OR acceptance criteria template OR user story template)
- **FR-018**: Reusable templates MUST use Persona + Questions + Principles pattern (activate reasoning, not prediction)
- **FR-019**: Lesson 4 MUST NOT include Mermaid diagrams (user decision: skip for cognitive load management)

**Stage 4 (Spec-Driven Integration) Requirements**:

- **FR-020**: Lesson 5 (capstone) MUST require complete task management system specification composing all markdown skills from Lessons 1-4
- **FR-021**: Capstone specification MUST include: system overview (headings), feature list (lists), expected behavior (code blocks for outputs), reference materials (links)
- **FR-022**: Capstone scope MUST be appropriate for A2 tier: 3-4 core features (create task, view tasks, mark complete, delete task)
- **FR-023**: Capstone MUST validate specification completeness through AI feedback (student receives feedback on clarity without implementation)

**Lesson Ending Protocol Requirements**:

- **FR-024**: Each lesson MUST end with ONLY "Try With AI" section (no "What's Next", "Key Takeaways", "Summary", or standalone "Safety Note" sections)
- **FR-025**: Safety notes (if needed) MUST be embedded INSIDE "Try With AI" section (maximum 1-2 sentences)

**CommonMark Compliance Requirements**:

- **FR-026**: All markdown syntax examples MUST comply with CommonMark standard (validated through post-spec research phase)
- **FR-027**: When GitHub Flavored Markdown (GFM) differs from CommonMark, lessons MUST teach CommonMark syntax with footnote explaining GFM variant

**SDD-RI Methodology Requirements**:

- **FR-028**: Chapter MUST explicitly position markdown as "Intent Layer" of Spec-Driven Development with Reusable Intelligence (SDD-RI)
- **FR-029**: Lessons MUST demonstrate how AI agents parse markdown structure: headings = hierarchy, lists = requirements, code blocks = specifications
- **FR-030**: Capstone MUST show specification → AI implementation workflow (student writes spec, AI provides feedback, specification is refined WITHOUT writing implementation code)

### Key Entities *(N/A - no data models in educational content specification)*

## Success Criteria *(mandatory)*

### Measurable Outcomes

**SC-001**: Students can write clear feature specifications in markdown that AI agents parse correctly (80% capstone success rate)

**SC-002**: Each lesson's "Try With AI" section produces valid specification artifacts (100% completion rate for students who attempt exercises)

**SC-003**: Students can distinguish specification examples (WHAT) from implementation code (HOW) when shown both (90% correct identification rate)

**SC-004**: Zero code examples present in any lesson (automated grep validation shows 0 matches for code patterns)

**SC-005**: Students demonstrate Three Roles collaboration in Lessons 2-3 exercises (observable through "Try With AI" submissions)

**SC-006**: Cognitive load maintained at A2 tier (concept count per section ≤ 7, validated through frontmatter metadata)

**SC-007**: Capstone specification includes all required components (headings, lists, code blocks, links) in 80% of student submissions

## Constraints *(mandatory)*

### Critical Constraints

**FORBIDDEN**:
- ❌ Programming code examples (Python, Bash, JavaScript, TypeScript, or any implementation language syntax)
- ❌ Internal scaffolding labels in student-facing text ("Stage 1/2/3/4", "Layer 1/2/3/4", "Three Roles Framework" as section headers)
- ❌ Lesson sections after "Try With AI" ("What's Next", "Key Takeaways", "Summary", standalone "Safety Note")
- ❌ Code-centric examples (showing HOW to implement vs WHAT system does)
- ❌ Mermaid diagram syntax (user decision: skip for cognitive load management)
- ❌ Generic markdown tutorials (README formatting, GitHub documentation writing)

### Required Constraints

**REQUIRED**:
- ✅ Exactly 5 lessons following 4-stage progression (Manual → AI Collaboration → Intelligence Design → Spec-Driven Integration)
- ✅ Specification examples ONLY (feature lists, acceptance criteria, expected outputs, API endpoint descriptions, system requirements)
- ✅ A2 tier cognitive load compliance (5-7 concepts per section, maximum 7 with heavy scaffolding)
- ✅ Specification-first teaching modality (students write specs for hypothetical systems throughout chapter)
- ✅ Three Roles demonstrated in Lessons 2-3 (AI as Teacher + Student + Co-Worker through narrative, not labels)
- ✅ Capstone project: Task management system specification (3-4 core features, composing all skills)
- ✅ CommonMark standard compliance (with GFM variants noted where applicable)
- ✅ "Try With AI" as ONLY final section in each lesson
- ✅ Markdown positioned as Intent Layer of SDD-RI methodology

### Complexity Tier Constraints (A2 - Beginner)

**Cognitive Load**:
- Maximum 7 concepts per section (5-7 range with heavy scaffolding)
- Progressive disclosure: simple syntax first (headings in L1, lists in L2), complexity later (templates in L4, capstone in L5)
- Maximum 2 options presented at decision points (reduce beginner paralysis)

**Scaffolding**:
- Step-by-step examples with validation checkpoints
- Explicit "Try With AI" exercises with clear prompts provided
- Visual examples showing markdown source and rendered output side-by-side
- Frequent comprehension checks ("Can you explain why we use headings for hierarchy?")

### Anti-Convergence Constraints

**Teaching Modality Variation**:
- Chapter 8 (Git) used hands-on discovery modality (execute commands, observe results)
- Chapter 9 MUST use specification-first modality (write specs, receive AI feedback, refine specifications)
- Different from typical markdown tutorials (not README formatting or documentation writing)

## Non-Goals *(mandatory)*

**NOT Teaching**:
- ❌ Programming language syntax or coding concepts (students haven't learned Python yet - that's Part 4)
- ❌ Comprehensive markdown reference (teaching minimal sufficient markdown for specification writing)
- ❌ README file formatting for GitHub repositories (focus is AI communication, not developer documentation)
- ❌ Advanced markdown features (tables, footnotes, definition lists) unless directly serve specification purposes
- ❌ Markdown rendering engines or implementation details (students don't need to know how markdown parsers work)
- ❌ HTML embedded in markdown (outside specification scope)
- ❌ Markdown extensions beyond CommonMark (GFM noted as variant, not taught as primary)

**NOT Including**:
- ❌ Mermaid diagram syntax (user decision: skip for cognitive load management - can be introduced later if needed)
- ❌ Code syntax highlighting languages beyond `text` tag (students don't know programming languages to highlight)
- ❌ LaTeX/mathematical notation in markdown (outside beginner scope)
- ❌ Markdown linting or validation tools (too technical for A2 tier)

**NOT Demonstrating**:
- ❌ Implementation of specifications (students write specs, AI provides feedback on clarity, but NO code implementation in this chapter)
- ❌ GitHub-specific markdown features (issue references, @mentions, task lists) unless serve specification purposes
- ❌ Markdown editors or IDEs (students use simple text editor to build manual foundation)

**Deferred to Later Chapters**:
- Part 4 (Python Fundamentals, Chapters 12-29): Implementation of specifications
- Chapter 30 (Spec-Driven Development Fundamentals): Advanced specification techniques
- Chapter 31 (Spec-Kit Plus): Professional specification tooling

## Acceptance Tests *(mandatory)*

### Content Quality Tests

**AT-001**: Manual review of all 5 lesson markdown files confirms ZERO code examples present (Python, Bash, JavaScript, or any programming language syntax)
- **Pass criteria**: Grep search for common code patterns (`def `, `function `, `import `, `const `, `#!/bin/bash`) returns zero matches in student-facing lesson text
- **Test method**: `grep -rE "(def |function |import |const |#!/bin/bash)" book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/*.md`

**AT-002**: Each lesson file ends with "## Try With AI" as ONLY final section (no forbidden sections after)
- **Pass criteria**: `tail -50 <lesson>.md | grep -E "^## " | tail -1` returns "## Try With AI" for all 5 lessons
- **Test method**: Automated check per constitutional validation script (see constitution.md:1020-1028)

**AT-003**: No internal scaffolding labels present in student-facing text
- **Pass criteria**: Grep search for internal labels returns zero matches
- **Test method**: `grep -rE "(Stage [0-9]|Layer [0-9]|Three Roles (Framework|in Action))" <lesson-files>` returns zero results

**AT-004**: All code blocks use specification context (not implementation code)
- **Pass criteria**: Manual review confirms code blocks show expected outputs, API descriptions, feature requirements, system behavior—NOT code syntax
- **Sample validation**: Each code block preceded by context like "Expected system output:", "API endpoint specification:", "Feature requirements:"

### 4-Stage Progression Tests

**AT-005**: Lesson 1 demonstrates Stage 1 (Manual Foundation) - students write markdown by hand without AI assistance
- **Pass criteria**: Lesson includes explicit instruction: "Open text editor, create file, write markdown manually" with NO "Try With AI" prompt for basic syntax
- **Validation**: "Try With AI" section focuses on validation (student manually created markdown, AI checks syntax) not generation

**AT-006**: Lessons 2-3 demonstrate Stage 2 (AI Collaboration) with Three Roles present
- **Pass criteria**: Each lesson includes narrative examples where:
  - AI suggests pattern student didn't know (AI as Teacher)
  - Student corrects AI with domain constraints (AI as Student)
  - Iteration produces better result than either alone (AI as Co-Worker)
- **Validation**: Presence of phrases like "AI suggested...", "You refined by...", "Together you converged on..."

**AT-007**: Lesson 4 demonstrates Stage 3 (Intelligence Design) - students create reusable templates
- **Pass criteria**: Lesson explicitly guides template creation using Persona + Questions + Principles pattern
- **Validation**: Template deliverable defined with structure: Persona (role), Questions (analysis prompts), Principles (decision frameworks)

**AT-008**: Lesson 5 demonstrates Stage 4 (Spec-Driven Integration) - capstone composes all skills
- **Pass criteria**: Capstone requirements explicitly list: use headings (L1), lists (L2), code blocks (L3), links (L4) in complete specification
- **Validation**: Rubric includes checklist for each markdown skill presence in final specification

### A2 Tier Cognitive Load Tests

**AT-009**: Each lesson section introduces maximum 7 concepts
- **Pass criteria**: Concept count per section documented in lesson frontmatter (`concepts: N` where N ≤ 7)
- **Test method**: Automated frontmatter validation script checks concept count compliance

**AT-010**: Heavy scaffolding present in all lessons
- **Pass criteria**: Each lesson includes:
  - Step-by-step examples with numbered instructions
  - Validation checkpoints ("Check your work" sections)
  - Visual comparisons (markdown source vs rendered output)
  - Clear success criteria for each exercise
- **Validation**: Manual review confirms scaffolding elements present

**AT-011**: Maximum 2 options presented at decision points (beginner paralysis prevention)
- **Pass criteria**: When lessons present choices (e.g., "Use bullet list OR numbered list"), maximum 2 options given with selection criteria
- **Validation**: Manual review of decision points confirms binary or limited choice presentation

### Capstone Project Tests

**AT-012**: Lesson 5 capstone produces complete task management system specification
- **Pass criteria**: Student deliverable includes:
  - System overview using headings (hierarchical organization)
  - Feature list using lists (enumerated requirements)
  - Expected behavior using code blocks (output specifications, NOT code)
  - Reference materials using links (authoritative sources)
- **Validation**: Capstone rubric checklist covers all required components

**AT-013**: Capstone scope appropriate for A2 tier (3-4 core features, not overwhelming)
- **Pass criteria**: Lesson prompt explicitly limits scope: "Your task management system should include: create task, view tasks, mark complete, delete task. NOT including: user authentication, task sharing, advanced filtering."
- **Validation**: Non-goals section in capstone prompt prevents scope creep

**AT-014**: Capstone demonstrates specification clarity through AI feedback
- **Pass criteria**: "Try With AI" exercise includes: "Share your specification with AI. AI will provide feedback on clarity. Refine specification based on feedback (do NOT ask AI to implement code)."
- **Validation**: Exercise rubric includes specification iteration without implementation

### CommonMark Compliance Tests

**AT-015**: All markdown syntax examples comply with CommonMark standard
- **Pass criteria**: Post-spec research phase validates syntax against https://commonmark.org/
- **Test method**: Research validation documented in research-findings.md shows CommonMark compliance

**AT-016**: GFM (GitHub Flavored Markdown) variants noted where applicable
- **Pass criteria**: When syntax differs between CommonMark and GFM, lesson includes footnote: "Note: GitHub uses [variant] - we teach CommonMark standard"
- **Validation**: Search for GFM variant callouts in lessons where differences exist (e.g., task lists, tables)

### SDD-RI Methodology Tests

**AT-017**: Chapter explicitly positions markdown as Intent Layer of SDD-RI
- **Pass criteria**: Lesson 1 includes section explaining: "In Spec-Driven Development with Reusable Intelligence (SDD-RI), markdown is the Intent Layer—where you communicate WHAT to build to AI agents"
- **Validation**: Keyword search for "Intent Layer" and "SDD-RI" in Lesson 1

**AT-018**: Lessons demonstrate how AI agents parse markdown structure
- **Pass criteria**: Examples show:
  - Headings = hierarchical organization (Feature > Sub-feature > Requirement)
  - Lists = enumerated requirements (bullet = options, numbered = sequence)
  - Code blocks = specifications (expected outputs, NOT implementation)
  - Links = references to authoritative sources
- **Validation**: Each markdown element lesson includes "How AI Reads This" explanation section

**AT-019**: Capstone shows specification → AI feedback workflow (no implementation)
- **Pass criteria**: Lesson 5 exercise: Student writes spec → AI provides feedback on clarity → Student refines spec → Validation complete (STOP before implementation)
- **Validation**: Exercise explicitly states: "Do NOT ask AI to implement code. Focus on specification clarity."

### Anti-Pattern Detection Tests

**AT-020**: Lessons do NOT converge toward generic markdown tutorials
- **Pass criteria**: Manual review confirms language focuses on "specification articulation" not "README formatting"
- **Red flags**: Phrases like "make your GitHub repository look professional", "format your documentation", "create pretty README files"
- **Expected language**: Phrases like "communicate requirements to AI", "structure your specifications", "AI agents parse this as..."

**AT-021**: Lessons do NOT treat code blocks as "for code only"
- **Pass criteria**: Code block lesson (Lesson 3) explicitly teaches: "Code blocks show specifications like expected outputs, API descriptions, system requirements—NOT implementation code"
- **Validation**: Example code blocks labeled with specification context (not code syntax highlighting)

## Assumptions *(if applicable)*

**Student Prerequisites**:
- Students have completed Chapters 1-8 (assumed proficiency with basic terminal commands, file creation, AI tool interaction)
- Students can type markdown syntax in plain text editor (no WYSIWYG markdown editors assumed)
- Students have access to AI agents (Claude Code or Gemini CLI) for "Try With AI" exercises

**Teaching Environment**:
- Lessons delivered through Docusaurus documentation site (assumed from book structure)
- Students can view markdown source and rendered output side-by-side (Docusaurus provides this)
- Lessons accessed sequentially (students complete L1 before L2, etc.)

**Cognitive Load Baseline**:
- A2 tier students can handle 5-7 concepts per section with heavy scaffolding
- Students have working memory capacity for 7±2 items (Miller's Law applied to instructional design)
- Progressive disclosure effective: introduce simple concepts first, build complexity later

**Capstone Project**:
- Task management system is familiar domain for beginners (most students understand todo lists conceptually)
- 3-4 core features appropriate for A2 tier (more features would overwhelm cognitive capacity)
- Specification clarity can be validated through AI feedback without implementation (AI can assess if requirements are unambiguous)

**CommonMark vs GFM**:
- Teaching CommonMark as standard is appropriate for beginners (GFM is extension, not baseline)
- Students will encounter GFM later when using GitHub (Part 5 Spec-Driven Development)
- Noting GFM variants prepares students for differences without overwhelming them

**Research Validation**:
- Post-spec research phase (5 hours budgeted) sufficient to validate CommonMark syntax and AI parsing mechanics
- WebFetch tool access to CommonMark.org and markdown guides assumed
- Existing specs in `specs/` directory provide patterns for specification examples

## Dependencies & Prerequisites *(if applicable)*

### Student Prerequisites (Must Complete Before Chapter 9)

**Chapter 1-4 (Part 1: Introducing AI-Driven Development)**:
- Understanding of AI-Driven Development paradigm
- Familiarity with AI agents as tools for software creation

**Chapter 5-6 (Part 2: AI Tool Landscape - Agent Tools)**:
- Proficiency with Claude Code OR Gemini CLI (required for "Try With AI" exercises)
- Understanding of AI agent prompting basics

**Chapter 7 (Bash Essentials)**:
- Ability to create and edit text files using terminal commands
- Basic file navigation (cd, ls, cat)

**Chapter 8 (Git and GitHub)**:
- Understanding of version control concepts (provides context for specification-first thinking: spec before code)
- Familiarity with hands-on discovery modality (Chapter 9 varies from this with specification-first)

### Content Dependencies (Chapter 9 Enables)

**Chapter 10 (Prompt Engineering for AIDD)**:
- Markdown skills enable students to structure prompts using markdown formatting
- Specification writing (Chapter 9) + prompt engineering (Chapter 10) = effective AI communication

**Chapter 11 (Context Engineering)**:
- Markdown specifications serve as context for AI agents
- Students learn to provide structured context (markdown specs) to improve AI outputs

**Part 4 (Python Fundamentals, Chapters 12-29)**:
- Students write specifications (Chapter 9 markdown) BEFORE writing code (Part 4 Python)
- Specification-first mental model established before implementation begins

**Part 5 (Spec-Driven Development, Chapters 30-33)**:
- Chapter 9 provides markdown foundation for professional specification writing
- spec.md files in Part 5 use markdown skills from Chapter 9

### Tool Dependencies

**Required Tools**:
- Text editor (any: VS Code, Sublime Text, Nano, Vim - students choose based on Chapter 7 Bash familiarity)
- AI agent access (Claude Code OR Gemini CLI - from Chapters 5-6)

**Optional Tools** (NOT required but enhance experience):
- Markdown preview plugin for text editor (helps students see rendered output)
- Split-screen setup (markdown source on left, preview on right)

### Research Dependencies (Post-Spec Phase)

**CommonMark Standard Validation**:
- Access to https://commonmark.org/ (WebFetch tool)
- CommonMark specification document (reference for syntax validation)

**AI Parsing Mechanics**:
- Research on how AI agents parse markdown structure (headings, lists, code blocks)
- Understanding of markdown as machine-readable format (not just human-readable)

**Specification Language Research**:
- Examples of professional specifications written in markdown (from existing `specs/` directory)
- Patterns for feature requirements, acceptance criteria, user stories in markdown format

## Open Questions / Needs Clarification *(if applicable)*

**None**: User provided clear decisions on key ambiguities:
- ✅ Capstone scope: Task management system (not blog platform, chatbot, or e-commerce)
- ✅ Mermaid diagrams: Skip for now (cognitive load management)

**Resolved Through Constitutional Reasoning**:
- Teaching modality: Specification-first (derived from anti-convergence principle - Chapter 8 used hands-on discovery)
- Lesson count: 5 lessons (derived from 4-stage framework + current structure analysis)
- Cognitive load limits: 5-7 concepts per section (derived from A2 tier specification in chapter-index.md)
- No-code constraint enforcement: Specification examples only (derived from task requirement: students haven't learned programming yet)

**Research Phase Will Validate**:
- CommonMark vs GFM syntax differences (post-spec research)
- Specific examples of how AI agents parse markdown structure (post-spec research)
- Optimal markdown patterns for specifications (analysis of existing specs in `specs/` directory)
