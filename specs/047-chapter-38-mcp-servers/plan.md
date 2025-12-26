# Implementation Plan: Chapter 38 - Building Custom MCP Servers

**Feature Branch**: `047-chapter-38-mcp-servers`
**Planning Date**: 2025-12-26
**Status**: APPROVED FOR IMPLEMENTATION
**Spec**: `/specs/047-chapter-38-mcp-servers/spec.md`
**Constitution**: v7.0.0 (Agent Factory paradigm)

---

## I. Executive Summary

**Chapter 38 teaches students to BUILD MCP servers** (producer perspective) following Chapter 37's conceptual foundation (consumer perspective). Students progress through 10 lessons (+ assessment) implementing @mcp.tool, @mcp.resource, @mcp.prompt decorators, learning authentication patterns, testing via MCP Inspector, and packaging for distribution.

**Pedagogical Arc**: Manual Foundation (Lessons 1-2) → AI Collaboration with Three Roles (3-7) → Intelligence Design: Reusable Server Framework Skill (Lesson 8) → Spec-Driven Capstone: Domain-Specific Digital FTE (Lesson 9) → Assessment (Lesson 10)

**Key Outcome**: Students produce a complete, distributable MCP server that encodes domain expertise into reusable tools, resources, and prompts—a Digital FTE component.

---

## II. Technical Context

**Language/Version**: Python 3.11+ (B1-B2 developers, Part 5 mastery)
**Primary Dependencies**: FastMCP SDK 0.1.0+, uvicorn, httpx, Pydantic
**Testing**: MCP Inspector (browser-based), pytest (for student code)
**Target Platform**: Local development + distribution via PyPI/uv
**Project Type**: Educational content (lessons) + skill artifact (MCP Server Builder)

**Technical Constraints**:
- Chapter assumes Chapter 37 (MCP Fundamentals) mastery: Students understand Host → Client → Server, transport layers, primitives conceptually
- NOT re-teaching what primitives ARE (FR-015, FR-016, FR-017)
- Focus exclusively on HOW TO BUILD servers, not theory
- All code examples must execute successfully (testing requirement)
- No hallucinated APIs—all examples verified against official FastMCP docs

---

## III. Constitution Alignment

**Governing Framework**: Constitution v7.0.0 (Agent Factory Paradigm)

### Key Principle: Specification Primacy
- All Layer 4 content (Lesson 9-10) shows spec FIRST, implementation SECOND
- No Vibe Coding anti-pattern (code without specification)
- Students learn: "Specs are the new syntax"

### Key Principle: 4-Layer Progression
- Layer 1 (Manual): Lessons 1-2 (no AI assistance, build mental model)
- Layer 2 (AI Collaboration): Lessons 3-7 (Three Roles framework enforced)
- Layer 3 (Intelligence Design): Lesson 8 (create reusable MCP Server Builder skill)
- Layer 4 (Spec-Driven): Lesson 9 (produce Digital FTE via specification orchestration)

### Key Principle: Three Roles (Layer 2 Only)
- **Required in Lessons 3-7**: Each lesson demonstrates AI as Teacher + Student + Co-Worker
- **NOT as meta-commentary**: No "What to notice: AI is teaching you" (forbidden)
- **Instead**: Action prompts + self-reflection that create experience without labeling

### Key Principle: Digital FTE Outcome
- Chapter capstone (Lesson 9) must produce SELLABLE agent component
- Not toy project—genuine domain-specific server
- Viable for distribution and reuse

---

## IV. Concept Density Analysis (Pedagogical Foundation)

### Core Concepts (Count: 13)

1. FastMCP project structure (uv init, pyproject.toml)
2. @mcp.tool decorator pattern
3. Type hints → JSON schema conversion
4. Pydantic Field for parameter descriptions
5. @mcp.resource decorator (static URIs)
6. URI templates and parameter handling (templated resources)
7. MIME type specification
8. @mcp.prompt decorator pattern
9. Message structure (user/assistant)
10. Environment variable handling (os.environ)
11. Startup validation pattern
12. MCP Inspector for testing
13. pyproject.toml distribution configuration

### Complexity Assessment
- **Tier**: B1-B2 (Intermediate, independent developers)
- **Complexity**: Standard (decorators, patterns—straightforward for Python developers)
- **Prerequisites**: Part 5 Python mastery + Chapter 37 conceptual understanding
- **Justification**: 13 concepts distributed across 9 implementation lessons (Lessons 1-8 + 9 capstone) = ~1.5 concepts per lesson average, well within B1-B2 cognitive limits

### Justified Lesson Count: 10 lessons
- Lessons 1-2 (Layer 1, Manual): 5 + 6 concepts = 11 total
- Lessons 3-7 (Layer 2, AI Collab): 7 + 8 + 7 + 8 + 6 = 36 concept-lessons (distributed)
- Lesson 8 (Layer 3, Intelligence): 3 reusable (encapsulates Lessons 1-7)
- Lesson 9 (Layer 4, Capstone): ~13 orchestrated through spec-driven discipline
- Lesson 10 (Assessment): Validation

**Reasoning**: B1-B2 tier allows 7-10 concepts per lesson. Average ~7 across core lessons. Capstone (Lesson 9) orchestrates known concepts through specification lens, not new learning.

---

## V. Lesson Structure & Layer Progression

### Lesson 1: Project Setup with FastMCP (Layer 1: Manual Foundation)
- **Duration**: 45 minutes
- **Concepts**: 5 (uv init workflow, pyproject.toml, FastMCP initialization, uvicorn, Inspector connection)
- **Modality**: Hands-on discovery (students execute CLI commands manually)
- **Output**: Working FastMCP server project, no tools yet
- **Layer 1 Requirement**: ZERO AI assistance (builds mental model before AI collaboration)

### Lesson 2: Your First Tool (@mcp.tool) (Layer 1: Manual Foundation)
- **Duration**: 50 minutes
- **Concepts**: 6 (decorator syntax, type hints, docstrings, schema generation, return values, error handling)
- **Modality**: Sequential walkthrough with observation
- **Output**: Tool with proper type hints, visible in Inspector schema
- **Layer 1 Requirement**: ZERO AI assistance (manual implementation, manual testing via Inspector)

### Lesson 3: Pydantic Field & Parameter Descriptions (Layer 2: AI Collaboration)
- **Duration**: 55 minutes
- **Concepts**: 7 (Pydantic Field, description metadata, validation constraints, defaults, Field vs plain types)
- **Modality**: Spec-first, then AI collaboration with Three Roles
- - Part 1: AI as Teacher (suggests validation patterns)
- - Part 2: Student teaches AI (domain constraints)
- - Part 3: Convergence (iterate toward better validation)
- **Output**: Tool with rich parameter descriptions, defensive validation
- **Three Roles**: EXPLICITLY demonstrated through prompts and iterations (no meta-commentary)

### Lesson 4: Implementing Resources (@mcp.resource) (Layer 2: AI Collaboration)
- **Duration**: 55 minutes
- **Concepts**: 8 (resource decorator, static vs templated URIs, parameter extraction, MIME types, resource design patterns)
- **Modality**: Contrast-based (tools vs resources), then AI collaboration
- **Output**: Both static and templated resources working in Inspector
- **Three Roles**: Demonstrated through design conversation

### Lesson 5: Implementing Prompts (@mcp.prompt) (Layer 2: AI Collaboration)
- **Duration**: 55 minutes
- **Concepts**: 7 (prompt decorator, parameters, message structures, role assignment, expertise encoding)
- **Modality**: Expertise encoding focus, then AI collaboration on prompt quality
- **Output**: Parameterized prompt template returning message structure
- **Three Roles**: Demonstrated through prompt engineering iterations

### Lesson 6: Server Authentication & Environment Variables (Layer 2: AI Collaboration)
- **Duration**: 60 minutes
- **Concepts**: 8 (os.environ, .env files, startup validation, fail-fast pattern, secure credential handling, logging to stderr)
- **Modality**: Security-first specification, then AI collaboration on patterns
- **Output**: Server with validated environment variables, secure API integration
- **Three Roles**: Demonstrated through security pattern development

### Lesson 7: Testing & Debugging Your Server (Layer 2: AI Collaboration)
- **Duration**: 50 minutes
- **Concepts**: 6 (Inspector usage, server logs/stderr, common errors, debugging workflow, iteration patterns)
- **Modality**: Hands-on debugging with AI as partner
- **Output**: Ability to diagnose and fix server issues using Inspector
- **Three Roles**: Demonstrated through debugging conversation

### Lesson 8: Packaging & Publishing (Layer 2: AI Collaboration)
- **Duration**: 45 minutes
- **Concepts**: 6 (pyproject.toml metadata, entry points, build/install workflow, distribution via PyPI/uv)
- **Modality**: Configuration walkthrough, AI as metadata collaborator
- **Output**: Packaged server installable via pip/uv in clean environment
- **Three Roles**: Demonstrated through packaging refinement

### Lesson 9: Create Reusable Server Framework Skill (Layer 3: Intelligence Design)
- **Duration**: 60 minutes
- **Concepts**: 3 reusable (skill design pattern, decision framework for primitives, reusability criteria)
- **Artifact**: `.claude/skills/mcp-server-builder/SKILL.md`
- **Structure**: Persona + Questions + Principles
- - Persona: MCP Server Architect
- - Questions: How to choose tool vs resource vs prompt for requirement?
- - Principles: Security, testing, distribution patterns
- **Output**: Reusable framework for designing domain-specific servers

### Lesson 10: Capstone - Domain-Specific MCP Server (Layer 4: Spec-Driven Integration)
- **Duration**: 120 minutes
- **Approach**: Specification FIRST (intent, constraints, success criteria), then orchestrate using Lesson 8 skill
- **Domains** (student chooses ONE):
  - Project Management (create task, update status, get task list)
  - Data Analysis (query database, generate report, get schema)
  - Customer CRM (log interaction, update contact, get profile)
  - Code Quality (analyze code, generate suggestions, style guide)
  - Email Campaign (schedule campaign, track opens, template list)
- **Spec Requirements**: Must include user stories, primitive mapping, constraints, success criteria
- **Implementation**: AI implements using server-builder skill; student validates spec alignment
- **Output**: Distributable Digital FTE (complete server, installable, functional)
- **Three Roles**: Demonstrated throughout spec writing → implementation → validation cycles

### Lesson 11: Chapter Quiz & Assessment (Validation)
- **Duration**: 60 minutes
- **Format**:
  - Part 1: Concept recognition (10 multiple choice)
  - Part 2: Code analysis (5 scenarios with bugs to find/fix)
  - Part 3: Design challenge (1 open-ended: design server for domain)
- **Validation**: Demonstrates mastery of all 13 core concepts

---

## VI. Skill Opportunity (Layer 3 Intelligence Artifact)

### Primary Artifact: MCP Server Builder Skill

**File**: `.claude/skills/mcp-server-builder/SKILL.md`

**Purpose**: Reusable framework for designing and building domain-specific MCP servers

**Scope**: Generic to any domain (project mgmt, CRM, data analysis, etc.)

**Structure**:

1. **Persona**
   - Identity: MCP Server Architect
   - Expertise: Designing servers that expose tools, resources, prompts
   - Reasoning: Context-dependent primitive selection

2. **Questions** (Decision Framework)
   - What is the domain expertise being exposed?
   - What are the 3-5 core operations users need?
   - For each operation: ACTION (tool) vs READ DATA (resource) vs GUIDANCE (prompt)?
   - What external APIs/authentication needed?
   - How will server be packaged and distributed?

3. **Principles**
   - Tool (Action): Use when operation DOES something
   - Resource (Read): Use when operation READS data without modifying
   - Prompt (Guidance): Use when encoding expertise/instructions
   - Security: Credentials via env vars. Fail fast if missing. Never log secrets.
   - Testing: Use Inspector for development. Test error cases.
   - Distribution: Proper pyproject.toml. Clear entry points. Installable.

**Reusability**: Used in Part 6+ chapters requiring MCP servers; applied to capstone projects; transferable to student's own projects

**Connection to Capstone**: Lesson 9 (capstone) applies this skill through spec-driven approach

---

## VII. Success Criteria Mapping (Evals-First)

### Functional Requirements (FR) Coverage

| FR | Lesson | Validation |
|----|--------|-----------|
| FR-001: FastMCP setup with uv | L1 | Student creates project, installs dependencies |
| FR-002: @mcp.tool decorator | L2 | Tool appears in Inspector, callable |
| FR-003: Type hints → JSON schema | L2-L3 | Inspector shows schema from type hints |
| FR-004: Pydantic Field descriptions | L3 | Parameter descriptions visible in Inspector |
| FR-005: @mcp.resource static/templated | L4 | Both static and templated resources work |
| FR-006: MIME type specification | L4 | Resources return correct content-type |
| FR-007: @mcp.prompt decorator | L5 | Prompt appears in Inspector, returns messages |
| FR-008: Message structure (user/assistant) | L5 | Prompt returns proper message format |
| FR-009: Environment variables | L6 | API key accessed via os.environ |
| FR-010: Startup validation | L6 | Server fails fast if env var missing |
| FR-011: Logging to stderr | L6 | Logs don't corrupt JSON-RPC protocol |
| FR-012: MCP Inspector for development | L7 | Student uses Inspector for testing/debugging |
| FR-013: pyproject.toml distribution | L8 | Package installable via pip/uv |
| FR-014: Capstone domain-specific server | L10 | All primitives work together in domain context |

### Success Criteria (SC) Coverage

| SC | Lesson | Measurable Test |
|----|--------|-----------------|
| SC-001: Create server <10 min | L1 | Measure time from uv init to Inspector connection |
| SC-002: Tool with types + descriptions | L2-L3 | Tool schema visible with descriptions in Inspector |
| SC-003: Static + templated resources | L4 | Both resource types callable, return correct data |
| SC-004: Parameterized prompt template | L5 | Prompt accepts params, returns messages |
| SC-005: Environment variable validation | L6 | Server starts only when env var present |
| SC-006: Package for distribution | L8 | Installation succeeds in clean environment |
| SC-007: Capstone with multiple primitives | L10 | All user stories → working primitives |
| SC-008: NO overlap with Chapter 37 | All | No re-explanation of concepts from Chapter 37 |

### Educational Outcomes (EO) Coverage

| EO | Lessons | Demonstrates |
|----|---------|--------------|
| EO-001: Translate requirements to primitives | L4, L10 | Domain analysis → tool/resource/prompt selection |
| EO-002: Decorator-based development | L2-L8 | Apply patterns consistently across all decorators |
| EO-003: Debug independently | L7 | Use Inspector, interpret logs, fix issues |
| EO-004: Distribute servers | L8, L10 | Package and install server successfully |

**Validation**: All evals addressed across 10 lessons ✓

---

## VIII. Cognitive Load Distribution

### Per-Lesson Concept Count & CEFR B1-B2 Compliance

| Lesson | Concepts | Count | B1-B2 Limit | Status |
|--------|----------|-------|------------|--------|
| 1 | uv, FastMCP, pyproject.toml, uvicorn, Inspector | 5 | ≤10 | ✓ Within |
| 2 | Decorator, type hints, docstrings, schema, return, errors | 6 | ≤10 | ✓ Within |
| 3 | Pydantic Field, description, validation, constraints | 7 | ≤10 | ✓ Within |
| 4 | @resource, static/templated URIs, parameters, MIME | 8 | ≤10 | ✓ Within |
| 5 | @prompt, parameters, messages, user/assistant, expertise | 7 | ≤10 | ✓ Within |
| 6 | os.environ, validation, fail-fast, logging, security | 8 | ≤10 | ✓ Within |
| 7 | Inspector, logs, errors, debugging, iteration | 6 | ≤10 | ✓ Within |
| 8 | pyproject.toml, entry points, build, install, versioning | 6 | ≤10 | ✓ Within |
| 9 (L3) | Skill design pattern, decision framework, reusability | 3 | ≤10 | ✓ Within |
| 10 (L4) | ~13 orchestrated concepts through spec-driven discipline | - | Justified | ✓ Orchestrated |
| 11 | Assessment (no new concepts) | - | - | ✓ Validation |

**Cognitive Load Validation**: All lessons respect B1-B2 limits. Lesson 10 (capstone) exceeds per-concept count but justified by spec-driven approach (managing complexity through specification instead of new learning).

---

## IX. Content Quality Requirements (Mandatory Checklist)

### YAML Frontmatter (Every Lesson)
```yaml
---
sidebar_position: X
chapter: 38
lesson: X
title: "..."
description: "..."
duration_minutes: X

# HIDDEN SKILLS METADATA
skills:
  - name: "Skill Name"
    proficiency_level: "B1|B2"
    category: "Technical|Conceptual"
    bloom_level: "Remember|Understand|Apply|Analyze|Evaluate|Create"
    digcomp_area: "..."
    measurable_at_this_level: "..."

learning_objectives:
  - objective: "..."
    proficiency_level: "B1|B2"
    bloom_level: "..."
    assessment_method: "..."

cognitive_load:
  new_concepts: X
  concepts_list: [...]
  assessment: "X concepts (at B1-B2 limit of 7-10) ✓"

differentiation:
  extension_for_advanced: "..."
  remedial_for_struggling: "..."
---
```

### Content Requirements (Every Lesson)
- [ ] **Compelling Narrative Opening** (real-world hook, 2-3 paragraphs before first section)
- [ ] **Deep Evidence** (tables, diagrams, examples with numbers)
- [ ] **Three "Try With AI" Prompts** (Lessons 3-8) OR **Manual Practice** (Lessons 1-2)
  - Each prompt targets different skill
  - Each has "**What you're learning:**" explanation
  - Prompts are copyable (code blocks)
  - Final prompt connects to reader's domain
- [ ] **No Meta-Commentary** (FORBIDDEN: "AI is teaching you", "What to notice", Layer labels)
- [ ] **All Code Examples Tested** (execution logs attached)
- [ ] **All Claims Fact-Checked** (APIs verified against FastMCP docs, not hallucinated)
- [ ] **Specification Before Implementation** (Layer 2-4: spec intent shown first, code after)

### Forbidden Patterns (Constitutional)
- ❌ Meta-commentary: "What to notice: AI is teaching you" → FORBIDDEN
- ❌ Framework exposition: "This is Layer 2 AI Collaboration" → FORBIDDEN
- ❌ Role labels: "AI as Teacher/Student/Co-Worker" → FORBIDDEN
- ❌ Code without spec: Implementation shown before intent → FORBIDDEN (teaches Vibe Coding)
- ❌ Untested code examples: Included without execution logs → FORBIDDEN
- ❌ Hallucinated APIs: Invented parameters not in official docs → FORBIDDEN

### Three Roles Demonstration (Lessons 3-8: Layer 2 Only)
Every Layer 2 lesson MUST include:
1. **AI as Teacher**: Lesson shows AI suggesting pattern student didn't know
2. **AI as Student**: Lesson shows student correcting/refining AI output
3. **Convergence**: Lesson shows iteration toward better solution

**Implementation**: Action prompts + self-reflection questions (NOT explicit role labels)

---

## X. Chapter Pedagogical Modality (Anti-Convergence Requirement)

### Chapter 37 Pattern (Reference)
- Narrative + conceptual explanation
- Architecture diagrams
- Conceptual prompts exploring understanding

### Chapter 38 Pattern (MUST DIFFER)
- Hands-on discovery (manual exploration)
- Specification-first (intent before code)
- Three Roles collaboration (AI teaches, student teaches, convergence)
- Error analysis (debug real problems)
- Capstone integration (orchestrate through spec)

**Variation Achieved**: Chapter 37 narrative/conceptual. Chapter 38 practical/implementation with hands-on discovery. ✓

---

## XI. Dependencies & Prerequisites

### Prerequisites (Must Be Completed Before)
- **Chapter 37**: MCP Fundamentals (conceptual understanding of Host → Client → Server, primitives, transport)
- **Part 5**: Python async/await, decorators, type hints, Pydantic mastery

### Tools Required
- Python 3.11+
- uv package manager
- Node.js + npm (for MCP Inspector)
- Text editor (VS Code recommended)
- FastMCP SDK (installed via uv)

### Research Depth
- Market-defining chapter (comprehensive)
- Research required: FastMCP SDK official docs, best practices
- Quality standard: Superior to official FastMCP tutorial

---

## XII. Validation Criteria

**This plan succeeds when**:
- [x] All 13 core concepts covered across lessons
- [x] Each lesson respects B1-B2 cognitive load limits (≤10 concepts)
- [x] Layers 1→2→3→4 progression enforced without skipping
- [x] All FR/SC/EO success criteria mapped to specific lessons
- [x] No overlap with Chapter 37 (verified against FR-015, FR-016, FR-017 constraints)
- [x] MCP Server Builder skill encapsulates Lessons 1-8 patterns
- [x] Capstone demonstrates Digital FTE production through spec-driven approach
- [x] Three Roles demonstrated in every Layer 2 lesson (3-8)
- [x] Teaching modality differs from Chapter 37 (hands-on vs narrative)

---

## XIII. Implementation Timeline

| Phase | Deliverable | Status |
|-------|-------------|--------|
| **PLANNING** (This Document) | Chapter 38 plan.md | ✓ COMPLETE |
| **IMPLEMENTATION** | Lessons 1-11 (content writing) | Upcoming |
| **QUALITY ASSURANCE** | Technical review, fact-checking | Upcoming |
| **PUBLICATION** | Merged to main, in Docusaurus | Upcoming |

**Next Action**: Content-Implementer executes lessons 1-11 following this plan.

---

## XIV. References

- **Specification**: `/specs/047-chapter-38-mcp-servers/spec.md`
- **Constitution**: `/.specify/memory/constitution.md` (v7.0.0)
- **Chapter Index**: `/specs/book/chapter-index.md`
- **Quality Reference**: `/apps/learn-app/docs/02-AI-Tool-Landscape/09-git-and-github/01-your-first-git-repository.md`
- **FastMCP Docs**: Official FastMCP SDK documentation
- **Skill Template**: `.claude/skills/mcp-builder/` (reference for Lesson 9 skill creation)
