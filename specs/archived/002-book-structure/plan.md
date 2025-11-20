# Implementation Plan: Book Structure and Architect Plan

**Branch**: `002-book-structure` | **Date**: 2025-10-29 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-book-structure/spec.md`

**Note**: This is the architectural plan for establishing the complete book structure, directory organization, and Docusaurus configuration for "CoLearning Python & Agentic AI: The AI-Driven Way."

## Summary

Establish the complete book structure (32 chapters across 7 parts) with:
1. Directory organization aligned with Docusaurus requirements
2. Part-based folder hierarchy in `book-source/docs/`
3. Chapter templates and metadata for all 32 chapters
4. Docusaurus configuration for navigation and sidebar
5. Preview structure for reference and quality validation
6. Clear path for writers to understand where each chapter goes

## Technical Context

**Language/Version**: Markdown (content) + YAML (frontmatter) + Python 3.13+ (code examples)
**Primary Dependencies**: Docusaurus v3.x (build/hosting), Python 3.13+ (for code examples in chapters)
**Storage**: File-based (Markdown files in `book-source/docs/[part]/[chapter].md`)
**Testing**: Docusaurus build validation, Markdown linting, code example validation (via pytest in chapter examples)
**Target Platform**: Web-based technical book (rendered by Docusaurus)
**Project Type**: Technical documentation/educational content (32-chapter book with code examples)
**Performance Goals**: Docusaurus build completes in <60 seconds, site renders instantly (static HTML)
**Constraints**:
- All 32 chapters must follow consistent Markdown structure (per `.claude/output-styles/chapters.md`)
- All code examples must use Python 3.13+ with type hints (per Constitution Principle 3)
- Chapter filenames must match chapter index exactly
- Directory structure must match Docusaurus expectations
**Scale/Scope**: 32 chapters, 7 parts, ~600-800 KB total content, ~200+ code examples

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Version**: v2.0.0 (Principle-driven governance)
**Source**: `.specify/memory/constitution.md`

### Alignment Validation ✅

| Principle | Status | Verification |
|-----------|--------|--------------|
| **Principle 1: AI-First Teaching** | ✅ PASS | Every chapter designed to show AI prompts + results; not manual-coding focused |
| **Principle 2: Spec-Kit Methodology** | ✅ PASS | Part 5 (5 chapters) dedicated to Spec-Kit; all projects from Part 3 onward use Spec-Kit structure |
| **Principle 3: Modern Python 3.13+** | ✅ PASS | Technical context specifies Python 3.13+ for all code examples; type hints mandatory |
| **Principle 4: Test-First Mindset** | ✅ PASS | Chapter 17 (Part 4) dedicated to testing; testing integrated throughout |
| **Principle 5: Progressive Complexity** | ✅ PASS | Cognitive load management (see spec) ensures Parts 1-3 heavy scaffolding, Parts 4-7 lighter |
| **Principle 6: Consistent Structure** | ✅ PASS | All chapters follow `.claude/output-styles/chapters.md` template; reusable across all 32 chapters |
| **Principle 7: Technical Accuracy** | ✅ PASS | Chapter index references verified; all tool versions current (Python 3.13+, latest Claude/Gemini) |
| **Principle 8: Accessibility** | ✅ PASS | No assumed knowledge; all jargon explained; diverse examples; inclusive language |
| **Principle 9: Show-Then-Explain** | ✅ PASS | Output style templates enforce: working code → explanation → application |
| **Principle 10: Real-World Projects** | ✅ PASS | Part 4 Chapter 21, Part 5 chapters focus on realistic projects; GitHub publication planned |
| **Principle 11: Tool Diversity** | ✅ PASS | Part 2 (4 chapters) covers Claude Code, Gemini CLI, GitHub Copilot; honest comparison throughout |

### Domain Skills Alignment ✅

All 8 mandatory skills will be applied across chapters:
- ✅ **learning-objectives** → All chapters define Bloom's-aligned outcomes
- ✅ **concept-scaffolding** → Parts 1-3 heavy scaffolding; Parts 4-7 lighter
- ✅ **code-example-generator** → All examples follow standard (types, docstrings, tests)
- ✅ **exercise-designer** → All chapters include exercises aligned to objectives
- ✅ **assessment-builder** → Quizzes and reviews per chapter
- ✅ **technical-clarity** → Output styles enforce clarity; no gatekeeping
- ✅ **book-architecture** → 7-part structure validated for pedagogical flow
- ✅ **ai-augmented-teaching** → Every chapter demonstrates learning WITH AI

### Non-Negotiable Rules ✅

**ALWAYS DO** (All implemented in output styles):
- ✅ Use type hints on every function (Constitution IV)
- ✅ Test all code before publication (Constitution IV)
- ✅ Explain WHY, not just HOW (Constitution IV)
- ✅ Provide working code examples (Constitution IV)
- ✅ Use Python 3.13+ modern syntax (Constitution IV)
- ✅ Include "Common Mistakes" section in every chapter (Constitution IV)
- ✅ Include "AI Exercise" starting Ch 3 (Constitution IV)

**NEVER DO** (All prevented by output styles + review gates):
- ❌ Vague gatekeeping language ("easy", "simple") (Constitution IV)
- ❌ Untested or broken code (Constitution IV)
- ❌ Assume reader knowledge (Constitution IV)
- ❌ Use deprecated Python syntax (Constitution IV)
- ❌ Skip type hints for "simple" functions (Constitution IV)
- ❌ Hardcode secrets, tokens, API keys (Constitution IV)

**GATE STATUS**: ✅ **ALL PASS** - Plan aligns with Constitution v2.0.0

---

## Per-Part Detailed Architecture

This section outlines the detailed pedagogical and content architecture for each of the 7 parts. Each part will be implemented using the SpecKit SDD loop with subagents (chapter-planner, content-implementer, validation-auditor).

### PART 1: Introducing AI-Driven Development (5 chapters)

**Grounding Idea**: "From Coder to Super Orchestrator"
**Resource**: https://github.com/panaversity/spec-kit-plus/tree/main/docs-plus#the-billion-dollar-opportunity-from-coder-to-super-orchestrator

**Part Purpose**: Orient beginners to AI-driven development paradigm; build confidence with AI tools

**Cognitive Load**: Light | **Scaffolding**: Heavy | **Review Cycles**: 2-3

**Part Learning Outcomes**:
- ✅ Understand the paradigm shift from "manual coder" to "AI-orchestrator"
- ✅ Comfort using AI as collaborative partner (not replacement for thinking)
- ✅ Set up development environment (Python, IDE, AI tools)
- ✅ Write and execute first Python program with AI assistance
- ✅ Master debug/iterate cycle with AI feedback

**Chapter Breakdown** (5 chapters):
1. Welcome to AI-Driven Development — Paradigm shift, AI as collaborator, super orchestrator concept
2. Understanding AI Tools — Types of AI tools, Claude/Gemini/Copilot overview, capabilities vs limitations
3. Setting Up Your Environment — Python 3.13 installation, IDE setup, AI tool configuration
4. Your First Program with AI — "Hello AI" workflow, prompting for code, understanding output, iteration
5. Debugging and Iterating with AI — Error interpretation, refinement techniques, feedback loops

**[PLACEHOLDER]**: Exact story structure from "Coder to Super Orchestrator" for Chapter 1

---

### PART 2: AI Tool Landscape (4 chapters)

**Grounding Idea**: AI Tools as Collaborators (not magic)

**Part Purpose**: Tool literacy; understanding strengths/weaknesses of different AI platforms

**Cognitive Load**: Moderate | **Scaffolding**: Heavy | **Review Cycles**: 2-3

**Part Learning Outcomes**:
- ✅ Understand Claude Code (IDE integration, strengths, ideal use cases)
- ✅ Understand Gemini CLI (command-line workflows, when to prefer)
- ✅ Understand GitHub Copilot (inline completion, strengths, integration)
- ✅ Make informed tool choices based on task and context
- ✅ Know fallback strategies when preferred tool unavailable

**Chapter Breakdown** (4 chapters):
6. Claude Code: Features and Workflows — IDE integration, project context, real-world workflows
7. Gemini CLI: Installation and Basics — Command-line setup, prompting via CLI, model selection
8. GitHub Copilot and Code Editors — Integration with VS Code/Cursor/Zed, inline suggestions
9. Choosing the Right Tool for the Task — Comparison matrix, decision framework, tradeoffs

---

### PART 3: Prompt & Context Engineering (4 chapters)

**Grounding Framework**:
- CH 10: The Architect Toolkit (prompting foundations)
- CH 11: Six-Part Prompting Framework
- CH 12: Multimodal Input (images, videos)
- CH 13: Advanced Techniques

**Part Purpose**: Master AI communication; understand context management

**Cognitive Load**: Moderate | **Scaffolding**: Heavy | **Review Cycles**: 2-3

**Part Learning Outcomes**:
- ✅ Write effective, specific prompts using structured frameworks
- ✅ Manage context windows and token budgets
- ✅ Work with multimodal inputs (text, images, video)
- ✅ Debug AI-generated code systematically
- ✅ Refine and iterate prompts based on output quality

**Chapter Breakdown** (4 chapters):
10. The Architect Toolkit: Prompting Foundations — Prompt structure, clarity, specificity, good/bad examples
11. Six-Part Prompting Framework — [PLACEHOLDER: Define exact 6-part structure], context, task, examples
12. Multimodal and Advanced Input — Images, videos, function calling, tool invocation
13. Advanced Prompt Techniques — Few-shot learning, chain-of-thought, prompt chaining

---

### PART 4: Modern Python with Type Hints (8 chapters)

**Grounding Source**: Modern AI Python colabs
**Resource**: https://github.com/panaversity/learn-modern-ai-python/tree/main/00_python_colab

**Part Purpose**: Production-quality Python programming with type safety

**Cognitive Load**: High | **Scaffolding**: Moderate | **Review Cycles**: 1-2

**Part Learning Outcomes**:
- ✅ Write Python with complete type annotations (Python 3.13+ features)
- ✅ Use modern syntax (match/case, structural patterns, etc.)
- ✅ Implement comprehensive testing (unit, integration)
- ✅ Handle errors professionally
- ✅ Work with APIs and external data
- ✅ Design clean, maintainable code (patterns, best practices)
- ✅ Build realistic, complete projects

**Chapter Breakdown** (8 chapters):
14. Functions, Types, and Type Hints — Function definitions, type annotations, protocols, generics
15. Data Structures with Type Safety — Lists, dicts, sets, typing module, custom collections
16. Object-Oriented Programming (Modern) — Classes, inheritance, protocols, dataclasses, patterns
17. Testing and Quality Assurance — pytest, unit tests, fixtures, mocking, coverage
18. Error Handling and Debugging — Exception hierarchy, custom exceptions, debugging strategies
19. Working with APIs and Data — HTTP requests, JSON, data validation, API patterns
20. Clean Code and Design Patterns — SOLID principles, design patterns, refactoring
21. Building Your First Real Project — Project structure, integration, deployment, portfolio

**[PLACEHOLDER]**: Which real project to use for Chapter 21

---

### PART 5: Spec-Kit Methodology (5 chapters)

**Grounding Framework**: Specification-Driven Development (SDD) from Constitution

**Part Purpose**: Learn systematic project organization and planning

**Cognitive Load**: Moderate | **Scaffolding**: Moderate | **Review Cycles**: 1-2

**Part Learning Outcomes**:
- ✅ Understand Spec-Driven Development (SDD) methodology
- ✅ Write effective specifications (requirements, scope, success criteria)
- ✅ Create comprehensive plans (architecture, phases, milestones)
- ✅ Decompose projects into tasks
- ✅ Coordinate teams using shared specifications

**Chapter Breakdown** (5 chapters):
22. Specification-Driven Development Fundamentals — Why SDD matters, spec vs plan vs tasks, iterative workflow
23. Writing Effective Specifications — User scenarios, functional requirements, success criteria
24. Planning and Tasking — Architecture decisions, phases, task breakdown, dependencies
25. Real-World Spec-Kit Workflows — Case studies, practical patterns, common mistakes
26. Scaling Spec-Kit for Teams — Communication patterns, code review, version control

**[PLACEHOLDER]**: Case studies from real projects to include in Chapter 25

---

### PART 6: Agentic AI Fundamentals (3 chapters)

**Grounding Idea**: Autonomous AI systems that accomplish goals
**[PLACEHOLDER]**: Grounding material and framework not yet specified

**Part Purpose**: Design and build autonomous AI agents

**Cognitive Load**: Moderate-High | **Scaffolding**: Light | **Review Cycles**: 1

**Part Learning Outcomes**:
- ✅ Understand agentic AI concepts and patterns
- ✅ Design agent workflows and decision trees
- ✅ Build agents with tool integration
- ✅ Orchestrate multi-agent systems
- ✅ Debug and improve agent behavior

**Chapter Breakdown** (3 chapters):
27. Introduction to Agentic AI — What are agents, agent vs chatbot, patterns, architecture, examples
28. Building Your First Agent — Simple agents, tool integration, execution loops, function calling
29. Agent Orchestration and Real-World Applications — Multi-agent coordination, error recovery, production patterns

**[PLACEHOLDER]**: Agent frameworks to teach (LangChain vs AutoGen vs other?), complete agent examples

---

### PART 7: MCP Fundamentals (3 chapters)

**Grounding Idea**: Model Context Protocol for tool integration
**[PLACEHOLDER]**: Grounding material and framework not yet specified

**Part Purpose**: Extend AI capabilities through standardized protocols

**Cognitive Load**: Moderate | **Scaffolding**: Light | **Review Cycles**: 1

**Part Learning Outcomes**:
- ✅ Understand MCP architecture and standards
- ✅ Integrate existing MCP servers into applications
- ✅ Build custom MCP servers and tools
- ✅ Design tools that extend AI capabilities

**Chapter Breakdown** (3 chapters):
30. Introduction to Model Context Protocol — MCP architecture, why standardization, ecosystem
31. Integrating MCP into Your Applications — Using existing MCP servers, configuration, invocation
32. Building Custom MCP Servers — Server development, tool definition, SDK usage, best practices

**[PLACEHOLDER]**: Complete MCP server example, which applications/frameworks to cover

---

## Subagent-Driven Implementation Workflow

For each part, follow this 4-phase SpecKit SDD loop:

**Phase 1: SPEC** (You + Main Claude)
- Approve overall part architecture and learning outcomes
- Clarify [PLACEHOLDER] items
- Output: `specs/part-X/part-X-spec.md`

**Phase 2: PLAN** (chapter-planner subagent)
- For each chapter: Create detailed lesson breakdown
- Output: `specs/part-X/chapter-Y-plan.md` + `chapter-Y-tasks.md`

**Phase 3: IMPLEMENT** (content-implementer subagent, iterative)
- For each lesson: Write content and get approval
- Output: `docs/part-X/chapter-Y.mdx`

**Phase 4: VALIDATE** (validation-auditor subagent)
- Test all code, verify pedagogy, check Constitution alignment
- Output: Validation report

---

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Book Content Structure

**Decision**: File-based Markdown structure in `book-source/docs/` organized by Part (7 parts) → Chapters (32 total)

```text
book-source/
├── docs/
│   ├── intro.md                                          # Introduction/landing (existing)
│   │
│   ├── 01-Introducing-AI-Driven-Development/            # Part 1: 5 chapters
│   │   ├── 01-Welcome-to-AI-Driven-Development.md
│   │   ├── 02-Understanding-AI-Tools.md
│   │   ├── 03-Setting-Up-Your-Environment.md
│   │   ├── 04-Your-First-Program-with-AI.md
│   │   └── 05-Debugging-and-Iterating-with-AI.md
│   │
│   ├── 02-AI-Tool-Landscape/                            # Part 2: 4 chapters
│   │   ├── 01-Claude-Code-Features-and-Workflows.md
│   │   ├── 02-Gemini-CLI-Installation-and-Basics.md
│   │   ├── 03-GitHub-Copilot-and-Code-Editors.md
│   │   └── 04-Choosing-the-Right-Tool-for-the-Task.md
│   │
│   ├── 03-Prompt-and-Context-Engineering/              # Part 3: 4 chapters
│   │   ├── 01-Writing-Effective-Prompts.md
│   │   ├── 02-Context-Management-and-Memory.md
│   │   ├── 03-Debugging-AI-Generated-Code.md
│   │   └── 04-Advanced-Prompt-Techniques.md
│   │
│   ├── 04-Modern-Python-with-Type-Hints/               # Part 4: 8 chapters
│   │   ├── 01-Functions-Types-and-Type-Hints.md
│   │   ├── 02-Data-Structures-with-Type-Safety.md
│   │   ├── 03-Object-Oriented-Programming-Modern.md
│   │   ├── 04-Testing-and-Quality-Assurance.md
│   │   ├── 05-Error-Handling-and-Debugging.md
│   │   ├── 06-Working-with-APIs-and-Data.md
│   │   ├── 07-Clean-Code-and-Design-Patterns.md
│   │   └── 08-Building-Your-First-Real-Project.md
│   │
│   ├── 05-Spec-Kit-Methodology/                         # Part 5: 5 chapters
│   │   ├── 01-SDD-Fundamentals.md
│   │   ├── 02-Writing-Effective-Specifications.md
│   │   ├── 03-Planning-and-Tasking.md
│   │   ├── 04-Real-World-Spec-Kit-Workflows.md
│   │   └── 05-Scaling-Spec-Kit-for-Teams.md
│   │
│   ├── 06-Agentic-AI-Fundamentals/                      # Part 6: 3 chapters
│   │   ├── 01-Introduction-to-Agentic-AI.md
│   │   ├── 02-Building-Your-First-Agent.md
│   │   └── 03-Agent-Orchestration-and-Real-World-Applications.md
│   │
│   └── 07-MCP-Fundamentals/                             # Part 7: 3 chapters
│       ├── 01-Introduction-to-Model-Context-Protocol.md
│       ├── 02-Integrating-MCP-into-Your-Applications.md
│       └── 03-Building-Custom-MCP-Servers.md
│
├── sidebars.js                                           # Docusaurus sidebar configuration
├── docusaurus.config.js                                  # Docusaurus main configuration
└── static/                                               # Images, assets, code examples
    └── examples/                                         # Runnable code examples per chapter
```

**Structure Rationale**:
- ✅ **Part-based folders** match Docusaurus nav hierarchy
- ✅ **Chapter numbers** (01-, 02-, etc.) ensure correct ordering
- ✅ **Descriptive names** make chapters self-explanatory
- ✅ **Flat chapter structure** prevents nesting > 2 levels (Docusaurus best practice)
- ✅ **Matches chapter-index.md** exactly (single source of truth)
- ✅ **Supports multiple reading paths** (sidebar can reorder without file moves)

**Structure Decision**:
- **Documentation**: Stored in feature spec directory (`specs/002-book-structure/`)
- **Content**: Organized in `book-source/docs/` by Part (7 folders) → Chapter files (32 total)
- **Configuration**: Docusaurus config in `book-source/` root
- **Examples**: Runnable code stored in `book-source/static/examples/` (keyed by chapter)

## Phase 0: Research & Clarification

**Status**: No NEEDS CLARIFICATION markers in technical context.

**Key Decisions Already Made**:
- ✅ Docusaurus v3.x for static site generation (standard for technical books)
- ✅ Markdown + YAML frontmatter for content (industry standard)
- ✅ Python 3.13+ for all code examples (Constitution Principle 3)
- ✅ Part-based folder hierarchy (pedagogically aligned with 7-part structure)
- ✅ Chapter filenames match chapter-index.md (single source of truth)

**No research phase required**: Technical choices are locked by Constitution and specification.

---

## Phase 1: Design & Implementation

### 1.1 Data Model (`data-model.md`)

**Core Entities**:
- **Chapter**: Markdown file with YAML frontmatter, content, code examples
  - Fields: title, number, part, learning_objectives, prerequisites, key_topics
  - Validations: filename must match chapter index, frontmatter required
- **Part**: Folder containing 3-8 chapters
  - Fields: part_number, part_name, description, chapters[], learning_outcomes
  - Validations: folder name format: `NN-Part-Name`
- **CodeExample**: Runnable Python code blocks within chapters
  - Fields: language, code, output, explanation, type_hints_required, tested
  - Validations: must have type hints, docstrings, passing tests

### 1.2 Docusaurus Configuration

**Key Files to Create/Update**:
1. **`book-source/sidebars.js`**: Navigation structure (7 parts → 32 chapters)
2. **`book-source/docusaurus.config.js`**: Site configuration, metadata, plugins
3. **`book-source/package.json`**: Dependencies (docusaurus, build scripts)

### 1.3 Directory Creation & Validation

**Tasks**:
1. Create 7 part folders in `book-source/docs/`
2. Create 32 chapter placeholder files (YAML frontmatter + stub content)
3. Create Docusaurus configuration files
4. Validate directory structure matches chapter-index.md

### 1.4 Chapter Template

All chapters follow `.claude/output-styles/chapters.md` template:
- Front matter (title, sidebar_position, description)
- Learning objectives (Bloom's aligned)
- Content sections (show-then-explain pedagogy)
- Code examples (Python 3.13+ with type hints)
- Exercises (aligned to objectives)
- Common mistakes
- AI Exercise (starting Ch 3)
- Assessment

---

## Phase 2: Quality Validation

### 2.1 Docusaurus Build

- ✅ Verify `npm run build` succeeds
- ✅ Check for broken links (both internal and external)
- ✅ Validate all images render
- ✅ Verify sidebar navigation is correct

### 2.2 Content Validation

- ✅ All 32 chapter files exist in correct locations
- ✅ All filenames match chapter-index.md exactly
- ✅ All frontmatter follows template
- ✅ No broken internal links (Chapter N references)
- ✅ All code examples have type hints (where applicable)

### 2.3 Navigation Validation

- ✅ Docusaurus sidebar correctly organizes 7 parts
- ✅ All 32 chapters appear in navigation
- ✅ Sidebar_position values ensure correct ordering
- ✅ Reading paths can be implemented via sidebar re-ordering

---

## Deliverables

| Artifact | Location | Status | Purpose |
|----------|----------|--------|---------|
| Implemented directory structure | `book-source/docs/01-*` through `07-*` | To Create | Physical book organization |
| Docusaurus configuration | `book-source/docusaurus.config.js` | To Create | Site metadata and build configuration |
| Sidebar navigation | `book-source/sidebars.js` | To Create | Chapter navigation and organization |
| Chapter placeholders | 32 `.md` files in part folders | To Create | Ready for writers to fill in |
| Validation report | `specs/002-book-structure/validation.md` | To Create | Quality assurance results |

---

## Success Criteria (Execution-Level)

- ✅ All 7 part folders created in `book-source/docs/`
- ✅ All 32 chapter files created with correct names and structure
- ✅ Docusaurus builds successfully with zero errors
- ✅ All 32 chapters appear in sidebar navigation
- ✅ Chapter filenames match chapter-index.md 100%
- ✅ Directory structure matches planned layout exactly
- ✅ Writers can immediately begin writing chapters without structural confusion
- ✅ Docusaurus preview renders correctly with all chapters visible

---

## Notes & Assumptions

1. **No modifications to Constitution**: All structure decisions align with v2.0.0
2. **Chapter order is final**: 32 chapters in 7 parts (5+4+4+8+5+3+3) is non-negotiable
3. **Markdown as content language**: Not planning alternatives (PDF, HTML, etc.)
4. **Docusaurus as build tool**: Already in use; not considering alternatives
5. **Writers will come later**: This phase creates empty structure only; /sp.implement will handle writing
6. **Chapter filenames are immutable**: Changing names requires constitutional amendment
