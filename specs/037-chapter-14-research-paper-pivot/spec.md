# Feature Specification: Chapter 14 — Master Spec-Kit Plus

**Feature Branch**: `037-chapter-14-research-paper-pivot`
**Created**: 2025-11-26
**Status**: Draft
**Constitutional Version**: 6.0.1
**Part**: 4 (SDD-RI Fundamentals)
**Chapter**: 14
**Proficiency Tier**: A2-B1 (Beginner transitioning to Intermediate)

---

## Core Focus

**This chapter teaches Spec-Kit Plus** — the workflow, the commands, the phases.

**Research paper is the VEHICLE, not the lesson.** Students practice each Spec-Kit Plus phase by applying it to a simple research paper project that requires only markdown + AI companion.

| What Students Learn | What Students Practice On |
|---------------------|---------------------------|
| `/sp.constitution` command | Research paper quality standards |
| `/sp.specify` command | Research paper specification |
| `/sp.clarify` command | Refining paper requirements |
| `/sp.plan` command | Research paper structure |
| `/sp.tasks` command | Breaking paper into sections |
| `/sp.implement` command | Writing paper sections |
| Skill creation (P+Q+P) | `section-writer` skill |
| Intelligence acceleration | Reusing skill for second section |

---

## Constitutional Grounding

| Framework | Application |
|-----------|-------------|
| **Section IIa: 4-Layer Progression** | L1→L2→L3→L4 through lessons |
| **Principle 2: Progressive Complexity** | A2-B1 = max 7 concepts/section |
| **Principle 3: Factual Accuracy** | Spec-Kit Plus commands verified |
| **Principle 6: Anti-Convergence** | Ch13 problem-discovery → Ch14 hands-on workflow |
| **Principle 7: Minimal Content** | Focus on Spec-Kit Plus, not writing tutorials |

---

## Lesson Structure (Matches Existing)

Each lesson teaches ONE Spec-Kit Plus concept/phase:

| # | Lesson Title | Spec-Kit Plus Focus | Practice Vehicle |
|---|--------------|---------------------|------------------|
| 01 | Spec-Kit Plus Foundation | What is Spec-Kit Plus, why use it | Concepts only |
| 02 | Installation and Setup | Configure Claude Code/Gemini for Spec-Kit Plus | Setup project folder |
| 03 | Constitution Phase | `/sp.constitution` command | Paper quality standards |
| 04 | Specify Phase | `/sp.specify` command | Paper specification |
| 05 | Clarify Phase | `/sp.clarify` command | Refine paper requirements |
| 06 | Plan Phase | `/sp.plan` command | Paper implementation plan |
| 07 | Tasks Phase | `/sp.tasks` command | Atomic paper sections |
| 08 | Implement Phase | `/sp.implement` command | Write first section |
| 09 | Designing Reusable Intelligence | P+Q+P skill creation | `section-writer` skill |
| 10 | Brownfield Adoption | Adding Spec-Kit Plus to existing projects | Concepts + examples |
| 11 | Capstone | Skill reuse, intelligence acceleration | Second section using skill |
| 12 | Quiz | Assessment | N/A |

**Key Insight**: Lessons 03-08 each focus on ONE command. The research paper provides concrete practice without adding cognitive load.

---

## User Scenarios & Testing

### User Story 1 - Master Spec-Kit Plus Workflow (Priority: P1)

As a student who completed Chapter 13 (SDD-RI theory), I need to learn each Spec-Kit Plus phase/command so that I can apply specification-driven development to any project.

**Why this priority**: Core learning objective—students must know HOW to use each command.

**Independent Test**: Student can explain and execute each Spec-Kit Plus command (`/sp.constitution`, `/sp.specify`, `/sp.clarify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`).

**Acceptance Scenarios**:

1. **Given** Lesson 03, **When** the student completes it, **Then** they understand what `/sp.constitution` does and can create one
2. **Given** Lesson 04, **When** the student completes it, **Then** they understand what `/sp.specify` does and can write a specification
3. **Given** Lesson 05, **When** the student completes it, **Then** they understand what `/sp.clarify` does and can refine requirements
4. **Given** Lessons 06-08, **When** the student completes them, **Then** they can execute `/sp.plan` → `/sp.tasks` → `/sp.implement` cycle

---

### User Story 2 - Create Reusable Intelligence (Priority: P1)

As a student learning SDD-RI, I need to understand how to create skills using P+Q+P framework so that I can build reusable intelligence.

**Why this priority**: Core differentiator of SDD-RI—intelligence accumulation.

**Independent Test**: Student creates at least 1 skill using Persona + Questions + Principles.

**Acceptance Scenarios**:

1. **Given** Lesson 09, **When** the student completes it, **Then** they can explain P+Q+P framework
2. **Given** practice with research paper, **When** the student identifies a pattern, **Then** they create a `section-writer` skill
3. **Given** a created skill, **When** used in Lesson 11 capstone, **Then** it accelerates the second writing task

---

### User Story 3 - Experience Intelligence Acceleration (Priority: P2)

As a student, I need to apply my skill to a second task so that I experience how reusable intelligence accelerates future work.

**Why this priority**: Capstone validates SDD-RI value proposition.

**Acceptance Scenarios**:

1. **Given** Lesson 11 capstone, **When** student writes second section, **Then** they reuse their skill
2. **Given** skill reuse, **When** comparing effort, **Then** student articulates acceleration benefit

---

## Requirements

### Functional Requirements

**Chapter Structure**:
- **FR-001**: Chapter MUST have 12 lessons matching existing structure
- **FR-002**: Lessons 03-08 MUST each focus on ONE Spec-Kit Plus command
- **FR-003**: Each lesson MUST teach Spec-Kit Plus first, use research paper as practice

**Lesson Focus (Critical)**:
- **FR-004**: Lessons MUST NOT teach research paper writing techniques
- **FR-005**: Lessons MUST NOT teach academic methodology
- **FR-006**: Lessons MUST focus on Spec-Kit Plus workflow, using paper as simple example
- **FR-007**: Every "Try With AI" section MUST practice a Spec-Kit Plus command

**Tool Constraints**:
- **FR-008**: Chapter MUST NOT require Python (starts Part 5, Chapter 16)
- **FR-009**: Chapter MUST NOT require MCP servers (Chapter 38)
- **FR-010**: Chapter MUST NOT require external APIs or web services
- **FR-011**: Chapter MUST use only: AI companion, markdown, terminal, git

**Spec-Kit Plus Coverage**:
- **FR-012**: Lesson 03 MUST teach `/sp.constitution`
- **FR-013**: Lesson 04 MUST teach `/sp.specify`
- **FR-014**: Lesson 05 MUST teach `/sp.clarify`
- **FR-015**: Lesson 06 MUST teach `/sp.plan`
- **FR-016**: Lesson 07 MUST teach `/sp.tasks`
- **FR-017**: Lesson 08 MUST teach `/sp.implement`
- **FR-018**: Lesson 09 MUST teach P+Q+P skill creation
- **FR-019**: Lesson 11 MUST demonstrate skill reuse and acceleration

---

## Success Criteria

### Command Mastery
- **SC-001**: 100% of students can explain what each command does
- **SC-002**: 100% of students can execute each command with AI companion
- **SC-003**: 90% of students produce valid artifacts for each phase

### Intelligence Creation
- **SC-004**: 90% of students create valid P+Q+P skill
- **SC-005**: 80% of students successfully reuse skill in capstone

### Conceptual Understanding
- **SC-006**: 90% of students can explain SDD-RI vs plain SDD
- **SC-007**: 90% of students can articulate why specification quality matters

---

## Constraints

### Technical Constraints
- MUST use ONLY: AI companion (Claude Code/Gemini CLI), terminal, git, markdown
- MUST NOT require Python, MCP, npm, or external services
- MUST work with natural language if slash commands unavailable

### Pedagogical Constraints
- MUST focus on Spec-Kit Plus commands, not research paper writing
- MUST keep research paper as simple practice vehicle
- MUST maintain A2-B1 cognitive load (max 7 concepts/section)
- MUST end lessons with "Try With AI" practicing Spec-Kit Plus

---

## Non-Goals

**This chapter does NOT teach**:
- ❌ Research paper writing methodology
- ❌ Academic citation practices
- ❌ Advanced writing techniques
- ❌ Python programming
- ❌ MCP server development
- ❌ External API integrations

**Why**: The chapter teaches Spec-Kit Plus. Research paper is just the practice vehicle.

---

## Why Research Paper as Vehicle

**Problem with original design (Video Generation)**:
- Required MCP (not taught until Chapter 38)
- Required Python (not taught until Chapter 16)
- Required external APIs (Gemini, YouTube)
- Mixed Spec-Kit Plus learning with tool learning

**Research paper solves this**:
- Uses only markdown + AI companion (already known)
- No external dependencies
- Students focus on Spec-Kit Plus workflow
- Simple enough that paper content doesn't distract from commands

---

## Evals Section

### Observable Behaviors

1. **Command Execution**: Student runs each Spec-Kit Plus command correctly
2. **Artifact Production**: Student produces constitution, spec, plan, tasks, implementation
3. **Skill Creation**: Student creates P+Q+P skill
4. **Acceleration Recognition**: Student articulates how skill reuse is faster

### Assessment Methods

| Objective | Method | Threshold |
|-----------|--------|-----------|
| Execute `/sp.specify` | Command produces valid spec | 4/4 sections present |
| Execute `/sp.plan` | Command produces valid plan | Structure matches template |
| Create skill | P+Q+P visible | All 3 components present |
| Demonstrate acceleration | Capstone reflection | Articulates time/effort savings |
