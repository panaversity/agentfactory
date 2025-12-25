# Feature Specification: Chapter 2 - AIFF Foundation & Agent Standards

**Feature Branch**: `036-chapter-2-aiff-foundation`
**Created**: 2025-12-25
**Status**: Approved
**GitHub Issue**: [#385](https://github.com/panaversity/ai-native-software-development/issues/385)
**Input**: Create Chapter 2 covering AAIF founding projects (MCP, AGENTS.md, goose) plus Agent Skills and MCP-UI/Apps SDK.

---

## Executive Summary

This specification defines Chapter 2 (AIFF Foundation & Agent Standards) which teaches the **infrastructure layer** enabling Digital FTE production. The chapter bridges from Chapter 1's paradigm vision to practical standards understanding.

**Key Design Decisions**:
- **AAIF-first organization**: Cover 3 founding projects (MCP, AGENTS.md, goose) as governed ecosystem
- **Agent Skills as complement**: Anthropic's open standard that works WITH MCP (not part of AAIF yet)
- **MCP-UI focus (not generic "Agent UI")**: Specific standard, with Apps SDK as practical starting point
- **No A2A/ACP/Commerce**: Agent-to-agent protocols belong in advanced chapters
- **Layer 1 only**: Conceptual foundation; no implementation tutorials
- **Constitutional alignment**: Every concept connects to Digital FTE production

**Constitutional Grounding**:
- Part 1, Chapter 2, A1 proficiency tier
- Layer 1 (Manual Foundation) - book teaches directly, student builds mental models
- 3-5 concepts per section maximum

---

## Constitutional Intelligence Object

```json
{
  "task_characterization": {
    "type": "educational_content_creation",
    "domain": "agentic_ai_infrastructure_standards",
    "audience": "A1_beginner",
    "part": 1,
    "chapter": 2,
    "existing_structure": "placeholder_readme_only",
    "implementation_status": "not_started",
    "novel_approach": "aaif_ecosystem_unified_view"
  },
  "constitutional_frameworks": {
    "teaching_layer": "1_manual_foundation",
    "cognitive_tier": "A1",
    "concept_limit_per_section": "3-5",
    "scaffolding": "very_heavy",
    "options_max": 2,
    "teaching_modality": "conceptual_with_analogies",
    "anti_convergence": "chapter_1_paradigm_vision_chapter_2_infrastructure_standards",
    "ai_role": "minimal_or_absent"
  },
  "digital_fte_contribution": {
    "why_this_matters": "Students cannot build Digital FTEs without understanding MCP (tool connectivity), Skills (expertise encoding), and AGENTS.md (project context)",
    "layer_4_connection": "MCP enables Act power, Skills enable custom expertise, AGENTS.md provides project-specific context",
    "accumulation": "Foundation for Parts 2-6 implementation chapters"
  },
  "research_sources": {
    "aaif_announcement": "https://www.linuxfoundation.org/press/linux-foundation-announces-the-formation-of-the-agentic-ai-foundation",
    "mcp_spec": "https://modelcontextprotocol.io/specification/2025-11-25",
    "agent_skills": "https://agentskills.io/specification",
    "agents_md": "https://agents.md",
    "goose": "https://block.github.io/goose/",
    "apps_sdk": "https://developers.openai.com/apps-sdk",
    "mcp_ui": "https://github.com/MCP-UI-Org/mcp-ui",
    "mcp_apps_extension": "https://blog.modelcontextprotocol.io/posts/2025-11-21-mcp-apps/"
  },
  "verification_date": "2025-12-25"
}
```

---

## Lesson Structure (6 Lessons)

| # | Title | Duration | Focus | Key Concepts |
|---|-------|----------|-------|--------------|
| 1 | **AAIF: The Agentic AI Foundation** | 12 min | Ecosystem governance | Linux Foundation, 3 founding projects, why open standards matter |
| 2 | **MCP: Connecting Agents to Tools** | 18 min | Core protocol | Resources/Tools/Prompts, Host→Client→Server, "USB for AI" |
| 3 | **AGENTS.md: Project Context for Agents** | 12 min | Repository instructions | Format, hierarchy rules, 60K+ adoption |
| 4 | **goose: A Reference Agent** | 12 min | Real implementation | Block's MCP-native agent, what agents look like in practice |
| 5 | **Agent Skills: Packaging Expertise** | 15 min | Procedural knowledge | SKILL.md format, progressive disclosure, MCP complement |
| 6 | **MCP-UI & Apps SDK: Agent Interfaces** | 15 min | Interactive UI | Apps SDK (today), MCP-UI extension (emerging standard) |

**Total Time**: ~84 minutes + quiz

---

## User Scenarios & Testing

### User Story 1 - Understanding the AAIF Ecosystem (Priority: P1)

**Scenario**: Developer completed Chapter 1 and heard buzzwords (MCP, Skills, AGENTS.md). Wants to understand how they fit together and who governs them.

**Journey**:
1. Student reads Lesson 1: Discovers AAIF under Linux Foundation (December 2025)
2. Learns 3 founding projects: MCP (Anthropic), AGENTS.md (OpenAI), goose (Block)
3. Understands governance: neutral home, open source, avoiding proprietary lock-in
4. Sees platinum members: AWS, Anthropic, Block, Bloomberg, Cloudflare, Google, Microsoft, OpenAI
5. Connects to Digital FTE: "Open standards mean my Digital FTEs work across platforms"

**Why P1**: Core orientation - must understand ecosystem before diving into individual standards.

**Acceptance Scenarios**:
1. **Given** student completes Lesson 1, **When** asked "What is AAIF?", **Then** student explains Linux Foundation initiative for agentic AI interoperability
2. **Given** student understands AAIF, **When** asked to name founding projects, **Then** student lists MCP, AGENTS.md, goose with correct company origins

---

### User Story 2 - Understanding MCP Architecture (Priority: P1)

**Scenario**: Student needs to understand MCP deeply enough to recognize how agents connect to tools - the foundation for Digital FTE "Act" power.

**Journey**:
1. Reads Lesson 2: MCP is "universal protocol for connecting AI to tools/data"
2. Learns three primitives: Resources (data to read), Tools (functions to call), Prompts (templates)
3. Understands architecture: Host (LLM app) → Client (connector) → Server (tool provider)
4. Grasps analogy: "MCP is USB - standardized connection, any device plugs in"
5. Learns transports: stdio for local, HTTP/SSE for remote
6. Connects to Digital FTE: "My Digital FTEs will use MCP servers to access CRM, databases, APIs"

**Why P1**: MCP is the foundational protocol - enables all tool integration in later chapters.

**Acceptance Scenarios**:
1. **Given** student completes MCP lesson, **When** asked "What problem does MCP solve?", **Then** student explains standardization of AI-to-tool connections
2. **Given** student understands primitives, **When** shown list (search, file access, templates), **Then** student correctly categorizes as Tools, Resources, or Prompts
3. **Given** student understands architecture, **When** shown MCP command, **Then** student identifies it as adding tool connectivity

---

### User Story 3 - Understanding AGENTS.md (Priority: P2)

**Scenario**: Student wants to understand how AI agents get project-specific context when working in codebases.

**Journey**:
1. Reads Lesson 3: AGENTS.md is "README for agents"
2. Understands purpose: coding conventions, build commands, testing requirements
3. Learns format: standard Markdown, no required fields, any relevant sections
4. Understands hierarchy: root AGENTS.md + subdirectory files (nearest takes precedence)
5. Sees adoption: 60,000+ open source projects, supported by all major agents
6. Connects to Digital FTE: "When my Digital FTE works on a codebase, AGENTS.md tells it local rules"

**Why P2**: Simpler than MCP, specific purpose. Important but less foundational.

**Acceptance Scenarios**:
1. **Given** student completes lesson, **When** asked "Where does AGENTS.md go?", **Then** student answers "Repository root, optionally subdirectories"
2. **Given** student understands hierarchy, **When** asked which file applies in subdirectory, **Then** student explains "nearest one takes precedence"

---

### User Story 4 - Understanding goose as Reference Agent (Priority: P2)

**Scenario**: Student has learned abstract standards (MCP, AGENTS.md) and wants to see what a real agent looks like in practice.

**Journey**:
1. Reads Lesson 4: goose is Block's open source agent framework
2. Learns key features: local-first, MCP-native, extensible, any LLM
3. Understands enterprise adoption: thousands of Block employees use it daily
4. Sees 50-75% time savings reported on common tasks
5. Recognizes goose as reference implementation: "This is what MCP looks like in a real agent"
6. Connects to Digital FTE: "goose shows how General Agents work in practice"

**Why P2**: Concrete example after abstract standards. Validates that MCP/AGENTS.md are real.

**Acceptance Scenarios**:
1. **Given** student completes lesson, **When** asked "What is goose?", **Then** student explains "Block's open source agent that uses MCP"
2. **Given** student understands goose, **When** asked "Why is goose in AAIF?", **Then** student explains "Reference implementation, contributes to open standards"

---

### User Story 5 - Understanding Agent Skills (Priority: P2)

**Scenario**: Student completed MCP lesson (connectivity) and wants to understand how to encode domain expertise into portable packages.

**Journey**:
1. Reads Lesson 5: Skills are "folders of procedural knowledge that teach agents workflows"
2. Learns SKILL.md format: YAML frontmatter (name, description) + Markdown instructions
3. Understands progressive disclosure: metadata loads at startup, instructions load on-demand
4. Grasps token efficiency: only ~100 tokens initially, rest loads when needed
5. Understands MCP relationship: "MCP = connectivity to tools, Skills = expertise for using them"
6. Sees adoption: VS Code, Cursor, goose, GitHub, Claude Code
7. Connects to Digital FTE: "My domain expertise becomes Skills that I package into Digital FTEs"

**Why P2**: Critical for monetization - Skills are how expertise becomes sellable.

**Acceptance Scenarios**:
1. **Given** student completes lesson, **When** asked "What's in SKILL.md?", **Then** student describes frontmatter + instructions format
2. **Given** student understands progressive disclosure, **When** asked "How do Skills save tokens?", **Then** student explains "Load description first, full content only when needed"
3. **Given** student understands ecosystem, **When** asked "MCP vs Skills?", **Then** student explains "MCP connects to tools, Skills teach how to use them"

---

### User Story 6 - Understanding Agent Interfaces (Priority: P3)

**Scenario**: Student wants to know how agent UIs are evolving beyond chat, with focus on what they can use today.

**Journey**:
1. Reads Lesson 6: Agent interfaces evolving from chat to interactive components
2. Learns Apps SDK: build apps inside ChatGPT with custom UI, uses MCP as foundation
3. Understands MCP-UI: emerging standard for servers to return interactive UI
4. Sees collaboration: Anthropic + OpenAI + MCP-UI community standardizing together
5. Practical guidance: "Use Apps SDK today, know unified standard is coming"
6. Connects to Digital FTE: "My Digital FTEs might have custom interfaces in ChatGPT"

**Why P3**: UI is evolving rapidly, less stable than other standards. Awareness-level.

**Acceptance Scenarios**:
1. **Given** student completes lesson, **When** asked "What can I use today for agent UI?", **Then** student answers "OpenAI Apps SDK"
2. **Given** student understands MCP-UI, **When** asked "What's MCP-UI?", **Then** student explains "Extension for MCP servers to return interactive UI components"

---

### Edge Cases

- **Outdated knowledge**: Student knows MCP from 2024 → Explain AAIF governance (Dec 2025), new spec features
- **Confusion with GPTs**: Student confuses Agent Skills with OpenAI GPTs → Clarify Skills is open standard, GPTs is OpenAI-specific
- **Implementation urge**: Student wants to build MCP server NOW → Redirect to conceptual foundation first, implementation in Part 2+
- **Standards fatigue**: "Why so many standards?" → Explain complementary roles, unified governance under AAIF
- **goose confusion**: Student thinks goose replaces Claude Code → Clarify both are General Agents, goose is reference implementation

---

## Requirements

### Functional Requirements

#### Lesson 1: AAIF - The Agentic AI Foundation

- **FR-001**: Student MUST understand AAIF as Linux Foundation initiative (December 2025)
- **FR-002**: Student MUST identify three founding projects: MCP (Anthropic), AGENTS.md (OpenAI), goose (Block)
- **FR-003**: Student MUST understand AAIF purpose: neutral governance, open standards, avoiding proprietary lock-in
- **FR-004**: Student MUST recognize platinum members: AWS, Anthropic, Block, Bloomberg, Cloudflare, Google, Microsoft, OpenAI
- **FR-005**: Student MUST connect open standards to Digital FTE portability

#### Lesson 2: MCP - Connecting Agents to Tools

- **FR-006**: Student MUST understand MCP as universal agent-to-tool protocol
- **FR-007**: Student MUST identify three primitives: Resources (data), Tools (functions), Prompts (templates)
- **FR-008**: Student MUST understand architecture: Host → Client → Server
- **FR-009**: Student MUST recognize transport mechanisms: stdio (local), HTTP/SSE (remote)
- **FR-010**: Student MUST understand MCP origin (Anthropic Nov 2024) and AAIF donation (Dec 2025)
- **FR-011**: Student MUST connect MCP to Digital FTE "Act" power (Five Powers)

#### Lesson 3: AGENTS.md - Project Context for Agents

- **FR-012**: Student MUST understand AGENTS.md as project-level agent instructions
- **FR-013**: Student MUST recognize format: standard Markdown, no required fields
- **FR-014**: Student MUST understand hierarchy: nearest file takes precedence
- **FR-015**: Student MUST identify typical content: conventions, build commands, testing
- **FR-016**: Student MUST understand OpenAI origin (Aug 2025), 60K+ adoption
- **FR-017**: Student MUST connect AGENTS.md to Digital FTE project context

#### Lesson 4: goose - A Reference Agent

- **FR-018**: Student MUST understand goose as Block's open source agent framework
- **FR-019**: Student MUST recognize key features: local-first, MCP-native, any LLM
- **FR-020**: Student MUST understand goose as AAIF founding project
- **FR-021**: Student MUST see goose as reference implementation of MCP integration
- **FR-022**: Student MUST understand 50-75% time savings reported at Block
- **FR-023**: Student MUST connect goose to General Agents path (Two Paths Framework)

#### Lesson 5: Agent Skills - Packaging Expertise

- **FR-024**: Student MUST understand Skills as folders of procedural knowledge
- **FR-025**: Student MUST recognize SKILL.md format: YAML frontmatter + instructions
- **FR-026**: Student MUST understand progressive disclosure mechanism
- **FR-027**: Student MUST grasp token efficiency (~100 tokens initial, rest on-demand)
- **FR-028**: Student MUST understand Skills complement MCP (connectivity vs expertise)
- **FR-029**: Student MUST recognize Anthropic origin (Dec 2025 open standard), industry adoption
- **FR-030**: Student MUST connect Skills to encoding domain expertise for Digital FTEs

#### Lesson 6: MCP-UI & Apps SDK - Agent Interfaces

- **FR-031**: Student MUST understand agent interfaces evolving beyond chat
- **FR-032**: Student MUST recognize Apps SDK: build apps inside ChatGPT, uses MCP
- **FR-033**: Student MUST understand Apps SDK as production-ready today
- **FR-034**: Student MUST recognize MCP-UI: emerging standard for interactive components
- **FR-035**: Student MUST understand Anthropic + OpenAI + MCP-UI collaboration
- **FR-036**: Student MUST connect UI evolution to Digital FTE interface possibilities

### Key Entities

- **AAIF (Agentic AI Foundation)**: Linux Foundation initiative providing neutral governance for agentic AI standards
- **MCP (Model Context Protocol)**: Universal protocol for connecting AI agents to external tools and data
- **AGENTS.md**: Repository-level markdown file providing project-specific guidance for AI agents
- **goose**: Block's open source, MCP-native agent framework (AAIF founding project)
- **Agent Skills**: Anthropic's open standard for reusable procedural knowledge packages
- **SKILL.md**: Entry-point file for Agent Skills containing frontmatter + instructions
- **Apps SDK**: OpenAI's framework for building apps inside ChatGPT
- **MCP-UI**: Community extension enabling MCP servers to return interactive UI components

---

## Success Criteria

### Measurable Outcomes

**Conceptual Understanding:**

- **SC-001**: 90%+ of students can name AAIF and its three founding projects
- **SC-002**: 85%+ of students correctly identify which standard solves which problem
- **SC-003**: 80%+ of students can explain MCP primitives (Resources, Tools, Prompts)
- **SC-004**: 85%+ of students understand MCP + Skills complementary relationship

**Vocabulary Competence:**

- **SC-005**: Students correctly use terms: MCP server, SKILL.md, Host/Client/Server
- **SC-006**: Students distinguish Agent Skills (open standard) from OpenAI GPTs (proprietary)
- **SC-007**: Students can explain AAIF to someone unfamiliar

**Digital FTE Connection:**

- **SC-008**: 90%+ students articulate how standards enable Digital FTE production
- **SC-009**: 85%+ students understand these standards used in Parts 2-6
- **SC-010**: 80%+ students envision their expertise as future Skills

**Pedagogical Quality:**

- **SC-011**: Chapter passes constitutional validation (A1 cognitive load, Layer 1)
- **SC-012**: All facts verified against official sources (December 2025)
- **SC-013**: Cognitive load ≤ 5 concepts per section
- **SC-014**: Each lesson includes "Try With AI" section for engagement

---

## Constraints

### Pedagogical Constraints

- **C-001**: Content MUST target A1 beginner audience (Part 1, conceptual foundation)
- **C-002**: Layer 1 ONLY: Book teaches directly, minimal AI role
- **C-003**: Cognitive load MUST NOT exceed 3-5 concepts per section
- **C-004**: NO code examples requiring execution - awareness level only
- **C-005**: Every concept MUST connect to Agent Factory / Digital FTE vision

### Technical Constraints

- **C-006**: All facts MUST be verified against official sources (December 2025)
- **C-007**: MCP details MUST reference 2025-11-25 specification
- **C-008**: Agent Skills MUST reference agentskills.io open standard
- **C-009**: AAIF details MUST reference Linux Foundation announcement

### Scope Constraints

- **C-010**: NO MCP server implementation tutorials (deferred to Part 2+)
- **C-011**: NO Agent Skills creation tutorials (deferred to Part 3+)
- **C-012**: NO AGENTS.md writing tutorials (deferred to Part 2+)
- **C-013**: NO A2A/ACP/Commerce protocols (belongs in advanced chapters)
- **C-014**: Focus on WHAT and WHY, not HOW to build

---

## Non-Goals

### Explicitly Out of Scope

**Implementation Tutorials:**
- Building MCP servers (Part 2: Tool Landscape)
- Creating Agent Skills (Part 3: Markdown & Prompting)
- Writing AGENTS.md files (Part 2 setup chapters)
- Apps SDK development (advanced parts)

**Deep Protocol Details:**
- MCP JSON-RPC internals
- Transport layer implementation
- SDK implementation details
- OAuth/authentication mechanisms

**Competing Protocols:**
- A2A (Google Agent-to-Agent) - different problem space
- ACP (Agent Communication Protocol) - advanced multi-agent
- OpenAI Commerce - payment protocols
- These belong in advanced multi-agent chapters

---

## Dependencies

### Prerequisites
- **Chapter 1**: Agent Factory Paradigm, Two Paths Framework, Digital FTE vision, Five Powers

### Downstream (What This Enables)
- **Part 2 (Tool Landscape)**: MCP understanding for `gemini mcp add`, `claude mcp` commands
- **Part 3 (Markdown & Prompting)**: Skills understanding for SKILL.md creation
- **Part 6 (AI Native Development)**: Full MCP/Skills implementation

---

## Success Indicators

### Specification Complete: ✅
- [x] User stories prioritized (P1-P3)
- [x] Functional requirements mapped (FR-001 through FR-036)
- [x] Success criteria defined (SC-001 through SC-014)
- [x] Constraints explicit
- [x] Non-goals explicit (no A2A/ACP/Commerce)
- [x] 6-lesson structure approved

### Next: Planning Phase

---

**Status**: Specification approved. Proceeding to plan generation.
