# Reusable Intelligence (RI) Architecture

**Version**: 1.0
**Last Updated**: 2025-11-20
**Status**: Living Document

---

## Executive Summary

This document maps the complete **Reusable Intelligence (RI) architecture** of the AI-Native Software Development project—demonstrating how specifications, agent architectures, skills, and organizational memory function as strategic assets rather than disposable artifacts.

**Core Thesis**: In the agentic era, reusable intelligence (specifications, agent architectures, skills) replaces reusable code as the primary artifact of software development.

This project **practices what it teaches**: Every educational chapter about SDD-RI is itself created using SDD-RI methodology, with specifications, agents, and skills stored as versioned, reusable intelligence.

---

## Table of Contents

1. [RI Architecture Overview](#ri-architecture-overview)
2. [The Five-Layer Intelligence Stack](#the-five-layer-intelligence-stack)
3. [Primary RI Locations](#primary-ri-locations)
4. [Agent Architecture](#agent-architecture)
5. [Skills Library](#skills-library)
6. [Specification System](#specification-system)
7. [Organizational Memory](#organizational-memory)
8. [Workflows and Orchestration](#workflows-and-orchestration)
9. [Templates and Standards](#templates-and-standards)
10. [Usage Patterns](#usage-patterns)
11. [Evolution and Maintenance](#evolution-and-maintenance)

---

## RI Architecture Overview

### What is Reusable Intelligence?

**Reusable Intelligence (RI)** consists of:

1. **Specifications** — Precise expressions of intent and constraints (executable contracts, not documentation)
2. **Agent Architectures** — Role definitions, collaboration protocols, reasoning frameworks
3. **Skills** — Bundled expertise (Persona + Questions + Principles pattern)
4. **Organizational Memory** — ADRs, PHRs, audits, intelligence objects
5. **Templates and Standards** — Reusable structures for specs, plans, tasks

### Why RI Matters

| Traditional Asset | RI Asset | Strategic Value |
|-------------------|----------|-----------------|
| Code libraries | Specifications | Intent persists through implementation changes |
| Documentation | Agent architectures | Reasoning patterns compound across projects |
| Style guides | Skills | Expertise becomes queryable and composable |
| Meeting notes | Organizational memory | Decisions trace from rationale to impact |

**The shift**: From "write great code once" → "design great intelligence that generates code repeatedly"

---

## The Five-Layer Intelligence Stack

This project implements a **5-layer vertical intelligence architecture**:

```
┌─────────────────────────────────────────────────────────┐
│ Layer 5: Constitution (v6.0.1)                          │
│ • Reasoning frameworks (P+Q+P pattern)                  │
│ • 4-Layer Teaching Method                               │
│ • Meta-awareness & anti-convergence                     │
│ Location: .specify/memory/constitution.md               │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Layer 4: Domain Skills (22 skills)                      │
│ • Pedagogical skills (learning objectives, scaffolding) │
│ • Technical skills (code validation, deployment)        │
│ • Meta-skills (skill creation, tool selection)          │
│ Location: .claude/skills/                               │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Layer 3: Context & Tools                                │
│ • MCP servers (Context7, Filesystem, Web)               │
│ • External integrations (WebFetch, WebSearch)           │
│ • Bash execution, Git operations                        │
│ Location: Claude Code runtime, .claude/mcp.json         │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Layer 2: Intelligence Objects                           │
│ • Specifications (specs/)                               │
│ • Plans (specs/*/plan.md)                               │
│ • Tasks (specs/*/tasks.md)                              │
│ • ADRs, PHRs, Audits (history/)                         │
│ Location: specs/, history/                              │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Layer 1: Specialized Agents (9 agents)                  │
│ • super-orchestra (model: sonnet, 40x engineer)         │
│ • chapter-planner (model: haiku, lesson structure)      │
│ • content-implementer (model: haiku, lesson writing)    │
│ • validation-auditor (model: sonnet, quality gates)     │
│ • + 5 more specialized agents                           │
│ Location: .claude/agents/                               │
└─────────────────────────────────────────────────────────┘
```

**Intelligence flows downward**: Constitution → Skills → Tools → Intelligence Objects → Agents

**Execution flows upward**: Agents use Intelligence Objects, invoke Skills, reference Constitution

---

## Primary RI Locations

### Directory Structure

```
ai-native-software-development/
├── .claude/                          # Agent architectures & output styles
│   ├── agents/                       # 9 specialized agents
│   ├── skills/                       # 22 domain skills
│   └── output-styles/                # Structural & pedagogical templates
│
├── .specify/                         # SDD-RI workflow templates
│   ├── memory/                       # Constitution (v6.0.1)
│   ├── templates/                    # Spec, plan, tasks templates
│   └── scripts/                      # Automation (PHR creation, etc.)
│
├── specs/                            # Feature specifications
│   ├── 001-chapter-6-redesign/
│   ├── 013-chapter-14-redesign/
│   ├── 034-chapter-9-markdown-redesign/
│   └── [50+ feature specs]
│
├── history/                          # Organizational memory
│   ├── adr/                          # Architecture Decision Records
│   ├── prompts/                      # Prompt History Records (PHRs)
│   ├── audits/                       # Quality validation reports
│   ├── intelligence/                 # Intelligence objects & learnings
│   └── subagents-architecture/       # Agent coordination patterns
│
├── papers/                           # Research foundation
│   ├── SDD-RI Whitepaper.md
│   ├── From-Reusable-Code-to-Reusable Intelligence.md
│   └── Reasoning_Activation_in_LLMs_arXiv_Complete.md
│
└── apps/learn-app/docs/                 # Generated outputs (regenerable)
    ├── 01-Introducing-AI-Driven-Development/
    ├── 02-AI-Tool-Landscape/
    └── [13 parts, 70+ chapters]
```

---

## Agent Architecture

### Agents as Reasoning Orchestrators

**Location**: `.claude/agents/` (9 agents, 98 total markdown files in .claude)

Each agent is a **specialized reasoning orchestrator** with:
- **Persona** (cognitive stance)
- **Questions** (reasoning structure)
- **Principles** (decision frameworks)
- **Model selection** (sonnet for deep reasoning, haiku for fast execution)
- **Color coding** (visual distinction in workflows)
- **Output style** (structural templates from `.claude/output-styles/`)

### The 9 Specialized Agents

#### 1. **super-orchestra** (model: sonnet, color: gold)

**Role**: 40x engineer workflow combining deep thinking, deep research, deep planning, and agentic execution

**When to invoke**: Complex features requiring:
- Multi-modal intelligence gathering (Context7 + WebFetch + source analysis)
- Comprehensive planning with iterative refinement
- Publication-ready quality (surpassing official documentation)
- Constitutional alignment verification

**Example workflow**:
```
1. Deep Thinking: Human identifies gap (e.g., "Chapter 5 missing MCP/plugins")
2. Deep Research: Context7 (8000 tokens) + WebFetch (3+ official sources)
3. Deep Planning: Spec → Plan → Tasks with iterative refinement
4. Deep Search: Pattern recognition across constitution, existing specs
5. Agentic Execution: Invoke chapter-planner → content-implementer → validation-auditor
6. Human Value Addition: Strategic decisions, gap identification, alignment verification
```

**Constitutional Alignment**: Implements Section IIa (4-Layer Teaching), Section IIb (Three Roles Framework), Vertical Intelligence Architecture

**Output**: 40x productivity from DEPTH + INTELLIGENCE + QUALITY (not speed)

---

#### 2. **chapter-planner** (model: haiku, color: blue)

**Role**: Transforms approved chapter specifications into lesson-by-lesson architecture with skills proficiency mapping

**When to invoke**: After spec.md is approved and ready for breakdown

**Inputs**:
- `spec.md` (chapter requirements, learning objectives, constraints)
- `chapter-index.md` (proficiency level, part number, prerequisites)
- Constitution (pedagogical frameworks, complexity tiers)

**Outputs**:
- `plan.md` (lesson structure with 4-layer progression per lesson)
- Proficiency mapping (CEFR/Bloom's/DigComp alignment)
- Cognitive load validation (A2: 5-7 concepts, B1: 7-10, C2: no limit)

**Decision Framework**:
- What pedagogical progression serves these concepts? (Foundation → Application → Integration → Validation → Mastery)
- How many lessons does concept density justify? (5-12 based on complexity)
- What teaching pattern applies? (Must vary from previous chapter)

---

#### 3. **content-implementer** (model: haiku, color: yellow)

**Role**: Layer 2 Collaboration Specialist — executes lesson creation using Three Roles framework

**When to invoke**: After plan.md is complete, for each lesson

**Inputs**:
- `plan.md` (lesson outline, learning objectives)
- `tasks.md` (implementation checklist)
- Constitution (Layer 1-4 patterns, meta-commentary prohibition)
- Relevant skills from `.claude/skills/`

**Outputs**:
- Lesson markdown files (apps/learn-app/docs/[part]/[chapter]/[lesson]/README.md)
- Code examples (tested via code-validation-sandbox skill)
- "Try With AI" sections (Layer 2 collaboration, framework INVISIBLE)

**Critical Constraint**: NO meta-commentary (see Constitution Section IIa v6.0.1 amendment)

---

#### 4. **validation-auditor** (model: sonnet, color: red)

**Role**: Multi-dimensional quality assessment (technical correctness, pedagogical effectiveness, factual accuracy, accessibility)

**When to invoke**: After content implementation, before publication

**Validation Dimensions**:
1. **Technical Accuracy** (30% weight): Code examples tested, APIs verified, commands validated
2. **Pedagogical Effectiveness** (25%): 4-Layer progression, cognitive load management, Layer transition criteria met
3. **Writing Quality** (20%): Clarity, conciseness, active voice, parallel structure
4. **Structure & Organization** (15%): Logical flow, prerequisite handling, navigation
5. **AI-First Teaching** (10%): Three Roles demonstrated (framework invisible), Layer 2 collaboration patterns
6. **Constitution Compliance** (Pass/Fail): No meta-commentary, no scaffolding exposure, specification primacy

**Output**: Pass/Fail verdict + Issue report (critical/major/minor) + Recommendations

---

#### 5. **factual-verifier** (model: sonnet, color: purple)

**Role**: Verify factual claims, validate source citations, flag volatile topics requiring maintenance

**When to invoke**:
- Standalone: When content contains statistics, dates, technical specifications
- Sub-validator: Within validation-auditor comprehensive review

**Verification Protocol**:
1. Identify factual claims (statistics, version numbers, API endpoints, performance metrics)
2. For each claim, gather citation:
   - Official documentation (Context7 MCP)
   - Authoritative sources (WebFetch)
   - Research papers (if applicable)
3. Embed citations in lesson content
4. Flag volatile content (e.g., "Claude 3.5 Sonnet pricing" requires periodic updates)

**Output**: Citation list + Volatile topics flagged + Accuracy confidence score

---

#### 6. **pedagogical-designer** (model: sonnet, color: green)

**Role**: Validates learning progression, concept scaffolding, cognitive load management

**When to invoke**:
- During lesson planning (to validate proposed structure)
- When reviewing lesson sequences
- When progression seems unclear or cognitive load excessive

**Reasoning Framework**:
- Does lesson sequence build foundational understanding before complexity?
- Are prerequisite concepts properly scaffolded?
- Is cognitive load appropriate for proficiency tier (A2/B1/C2)?
- Do Layer transitions (1→2, 2→3, 3→4) follow decision frameworks?

**Output**: Progression analysis + Scaffolding recommendations + Cognitive load assessment

---

#### 7. **assessment-architect** (model: haiku, color: purple)

**Role**: Design evaluations measuring learning objectives with appropriate cognitive complexity

**When to invoke**:
- During lesson planning (to ensure assessments align)
- When assessments need validation (too easy/hard)
- When creating quizzes, exercises, projects

**Design Principles**:
- Bloom's taxonomy alignment (Remember → Understand → Apply → Analyze → Evaluate → Create)
- CEFR proficiency mapping (A1-C2)
- Diagnostic assessment (measures actual understanding, not superficial completion)

**Output**: Quiz specifications, exercise designs, project rubrics

---

#### 8. **spec-architect** (model: sonnet, color: blue)

**Role**: Validate or refine specifications for completeness, testability, clarity

**When to invoke**:
- When spec appears vague, lacks success criteria, or has unclear constraints
- Before planning or implementation begins
- When specification quality issues block progress

**Validation Criteria**:
- **Intent Clarity**: WHAT system should do without prescribing HOW
- **Constraint Definition**: Explicitly EXCLUDED (non-goals), architectural constraints, quality standards
- **Success Criteria**: Measurable, falsifiable, testable
- **Completeness**: All stakeholder needs addressed, edge cases identified

**Output**: Specification assessment + Refinement recommendations + Gap analysis

---

#### 9. **educational-validator** (model: sonnet, color: orange)

**Role**: Constitutional compliance validator — checks framework invisibility, evidence presence, structural compliance, proficiency alignment

**When to invoke**: Final validation before publication (constitutional compliance check)

**Validation Checks**:
1. **Framework Invisibility**: No meta-commentary, no scaffolding exposure, Three Roles experienced (not studied)
2. **Evidence Presence**: Code examples tested, claims cited, specifications provided
3. **Structural Compliance**: Follows constitution-mandated structures (lesson endings, section hierarchy)
4. **Proficiency Alignment**: Cognitive load matches tier (A2/B1/C2), concepts within limits

**Grep Validation Commands** (from Constitution):
```bash
# Meta-commentary check
grep -i "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows" [lesson-file.md]
# Expected: Zero matches

# Layer label check
grep -E "Layer [0-9]|Stage [0-9]|Three Roles (Framework|in Action)" [lesson-file.md]
# Expected: Zero matches

# Lesson ending check
tail -50 [lesson-file.md] | grep -E "^## " | tail -1
# Expected: "## Try With AI"
```

**Output**: Constitutional compliance report (Pass/Fail per check)

---

### Agent Coordination Patterns

**Location**: `history/subagents-architecture/`

#### Sequential Pattern (Waterfall)

```
super-orchestra
    ↓ (creates spec.md)
chapter-planner
    ↓ (creates plan.md, tasks.md)
content-implementer (per lesson)
    ↓ (creates lesson markdown)
validation-auditor
    ↓ (comprehensive quality check)
factual-verifier (if needed)
    ↓ (citation validation)
educational-validator
    ↓ (constitutional compliance)
PUBLICATION
```

**Handoff Protocol** (Constitution Section IV):

1. **Receive** context from previous agent
2. **Reason** about what value to add
3. **Produce** outputs that inform next agent
4. **Hand off** enriched context

**Self-check**: If next agent would need clarifying questions, handoff is incomplete

---

#### Parallel Pattern (Concurrent Execution)

```
content-implementer (Lesson 1) ║ content-implementer (Lesson 2) ║ content-implementer (Lesson 3)
            ↓                   ║             ↓                   ║             ↓
     validation-auditor         ║      validation-auditor         ║      validation-auditor
                                ║                                 ║
                                ↓                                 ↓
                          educational-validator (final gate)
```

**When to use**: Multiple lessons with clear specifications, no interdependencies

**Coordination mechanism**: Shared spec.md + plan.md provide contract; Git worktrees enable isolation

---

#### Iterative Pattern (Refinement Loop)

```
super-orchestra → spec.md v1
    ↓
chapter-planner → plan.md v1
    ↓
HUMAN REVIEW: Gap identified
    ↓
super-orchestra → spec.md v2 (Context7 + WebFetch research)
    ↓
chapter-planner → plan.md v2 (expanded lessons)
    ↓
content-implementer → lesson drafts
    ↓
validation-auditor → issues found
    ↓
content-implementer → lesson revisions
    ↓
educational-validator → PASS
```

**When to use**: Complex chapters, market-defining quality, pedagogical innovation required

**Example**: Chapter 5 refinement (documented in super-orchestra agent header)
- Iteration 1: Basic installation + features
- Iteration 2: Added Skills + Plugins + MCP (gap identified by human)
- Iteration 3: Expanded to 10 acceptance scenarios (+233%), 10 new FRs, 4 new SCs
- Result: MORE COMPREHENSIVE than all official Anthropic resources

---

## Skills Library

### Skills as Bundled Expertise

**Location**: `.claude/skills/` (22 skills)

Each skill uses **Persona + Questions + Principles (P+Q+P)** pattern to activate reasoning mode (see papers/Reasoning_Activation_in_LLMs_arXiv_Complete.md):

```markdown
---
name: skill-name
domains: [relevant-domains]
---

# Skill Name

[Persona: Cognitive stance]
You are a [role] who thinks about [domain] the way a [expert type] would—
[specific cognitive approach]

[Questions: Reasoning structure]
Before [action], analyze:
- [Question forcing context analysis]?
- [Question requiring trade-off evaluation]?
- [Question identifying constraints]?

[Principles: Decision frameworks]
- [High-level guideline with rationale]
- [Trade-off framework for decisions]
- [Concrete examples showing application]
```

---

### The 22 Skills

#### **Layer 1 Skills** (Foundational)

1. **learning-objectives** — SMART objective design, Bloom's taxonomy alignment
2. **concept-scaffolding** — Prerequisite analysis, cognitive load management
3. **technical-clarity** — Avoid jargon, use analogies, explain why before how

#### **Layer 2 Skills** (Collaboration)

4. **ai-collaborate-teaching** — Three Roles framework, bidirectional learning patterns
5. **code-example-generator** — Production-relevant examples, tested code, anti-toy-app
6. **exercise-designer** — Active learning, scaffolded practice, self-validation criteria
7. **visual-asset-workflow** — Diagrams for complex concepts, accessibility (alt text)
8. **image-generator** — DALL-E prompting for educational illustrations

#### **Layer 3 Skills** (Intelligence Design)

9. **skills-proficiency-mapper** — CEFR/Bloom's/DigComp proficiency mapping
10. **book-scaffolding** — Part/chapter structure, prerequisite chains, dependency analysis

#### **Layer 4 Skills** (Spec-Driven)

11. **assessment-builder** — Quiz/exam design, Bloom's-aligned questions, diagnostic assessment
12. **quiz-generator** — Multiple choice with answer redistribution (avoid patterns), rationale required

#### **Cross-Cutting Skills**

13. **content-evaluation-framework** — 6-category rubric (Technical 30%, Pedagogical 25%, Writing 20%, Structure 15%, AI-First 10%, Constitution Pass/Fail)
14. **skill-creator** — Meta-skill for creating new skills using P+Q+P pattern

#### **Validation Skills**

15. **code-validation-sandbox** — Test all code examples in isolated environment before publication
16. **quiz-generator** (includes answer redistribution to prevent pattern detection)

#### **Automation Skills**

17. **docusaurus-deployer** — Deploy Docusaurus site to GitHub Pages, CI/CD automation

#### **Meta-Skills**

18. **memory-file-architecture** — RI organization patterns, memory management
19. **prompt-template-designer** — Persona + Questions + Principles template creation
20. **tool-selection-framework** — Decision frameworks for choosing AI tools
21. **_shared** — Common utilities, helpers, patterns used across skills

---

### Skill Composition

Skills can **compose** for complex tasks:

```markdown
# Example: Creating a new chapter

1. Invoke @learning-objectives (define measurable outcomes)
2. Invoke @concept-scaffolding (sequence concepts by prerequisites)
3. Invoke @skills-proficiency-mapper (assign CEFR levels)
4. Invoke @book-scaffolding (integrate into book structure)
5. Invoke @ai-collaborate-teaching (design Layer 2 interactions)
6. Invoke @code-example-generator (create tested examples)
7. Invoke @exercise-designer (hands-on practice activities)
8. Invoke @assessment-builder (quiz/project design)
9. Invoke @content-evaluation-framework (quality validation)
```

**Orchestration**: super-orchestra agent decides which skills to invoke and in what order

---

## Specification System

### Specifications as Primary Artifacts

**Location**: `specs/` (50+ feature specifications)

**Philosophy**: Specifications are not documentation—they're AI instructions and living contracts

### Specification Structure

Each feature gets a directory:
```
specs/034-chapter-9-markdown-redesign/
├── spec.md                    # WHAT to build (requirements, constraints)
├── plan.md                    # HOW to structure (lesson breakdown)
├── tasks.md                   # WHAT to execute (implementation checklist)
├── intelligence-object.md     # Accumulated learnings, decisions, patterns
└── README.md                  # Quick reference
```

### spec.md Template

**Location**: `.specify/templates/spec-template.md`

**Sections**:
1. **Context** — Problem statement, stakeholder needs, current state
2. **System Intent** — WHAT we're building and WHY (not HOW)
3. **Functional Requirements** (FR-001 to FR-N) — Specific features
4. **User Stories** with Acceptance Criteria — Behavior from user perspective
5. **Non-Functional Requirements** — Performance, security, accessibility
6. **Success Criteria** — Measurable outcomes that determine "done"
7. **Out of Scope** — Explicitly EXCLUDED features (prevents scope creep)
8. **Skills to Apply** — Which `.claude/skills/` are relevant
9. **Constraints** — Technical, pedagogical, or business limitations

**Reasoning Activation Pattern** (per papers):
```markdown
### System Intent
Think like a [role]:
- [Question forcing analysis]?
- [Question requiring trade-off evaluation]?
- [Question identifying constraints]?

Principles:
- [Decision framework 1]
- [Decision framework 2]
```

---

### plan.md Template

**Location**: `.specify/templates/plan-template.md`

**Purpose**: Transform spec.md into lesson-by-lesson structure

**Sections**:
1. **Pedagogical Approach** — Teaching modality for this chapter
2. **Lesson Breakdown** — Title, duration, learning objectives, 4-layer progression per lesson
3. **Cognitive Load Analysis** — Concepts per section, proficiency tier validation
4. **Prerequisites** — What students must know before this chapter
5. **Assessment Strategy** — How learning will be validated
6. **Anti-Convergence Check** — Does this vary from previous chapter's pattern?

**Example Lesson Structure**:
```markdown
## Lesson 3: [Title]

**Duration**: 8-10 minutes
**Proficiency**: A2 (Beginner)
**Concepts**: 6 concepts (within A2 limit of 5-7)

### Layer 1: Manual Foundation
[Direct teaching, step-by-step walkthroughs, manual practice]

### Layer 2: AI Collaboration
[Three Roles framework, "Try With AI" section, framework INVISIBLE]

### Layer 3: Intelligence Design
[Students create skill/subagent capturing lesson pattern]

### Layer 4: Spec-Driven (if capstone)
[Orchestrate accumulated intelligence through specifications]
```

---

### tasks.md Template

**Location**: `.specify/templates/tasks-template.md`

**Purpose**: Execution checklist for content-implementer agent

**Structure**:
```markdown
# Tasks: [Chapter Name]

## Phase 1: Pre-Implementation
- [ ] T001: Read spec.md completely
- [ ] T002: Read plan.md completely
- [ ] T003: Read constitution sections relevant to this chapter
- [ ] T004: Identify which skills from `.claude/skills/` apply

## Phase 2: Research (if needed)
- [ ] T005: Context7 library lookup for [topic]
- [ ] T006: WebFetch official documentation
- [ ] T007: Cross-reference with existing chapters

## Phase 3: Lesson 1 Implementation
- [ ] T008: Create README.md with Layer 1-4 structure
- [ ] T009: Write code examples (to be validated via code-validation-sandbox)
- [ ] T010: Create "Try With AI" section (NO meta-commentary)
- [ ] T011: Design Layer 3 deliverable (skill/subagent students will create)

[Repeat Phase 3 for each lesson]

## Phase N: Validation
- [ ] T0XX: Self-check against constitution (meta-commentary grep)
- [ ] T0XX: Invoke validation-auditor
- [ ] T0XX: Address critical/major issues
- [ ] T0XX: Invoke educational-validator for constitutional compliance
```

---

### intelligence-object.md

**Purpose**: Capture learnings, decisions, and patterns that emerge during implementation

**Not a template** — each feature's intelligence object is unique and emergent

**Common Sections**:
- **Architectural Decisions** — Why we chose approach X over Y
- **Pattern Recognition** — What worked well, what to avoid
- **Dependencies Discovered** — Unexpected prerequisites or integrations
- **Pedagogical Insights** — What teaching approach resonated
- **Reusable Patterns** — Could this become a skill or agent?
- **Future Considerations** — What to revisit or extend later

**Example** (from Chapter 5 intelligence object):
```markdown
## Architectural Decision: Plugin Architecture Discovery

**Context**: Initial spec focused on Claude Code features only

**Problem**: Student would lack personalization capabilities critical for AIDD

**Research**: Context7 lookup revealed:
- Plugins contain: Skills + Commands + Agents + Hooks + MCP
- Skills use 3-level progressive disclosure
- MCP enables external integrations

**Decision**: Expand Lesson 4 from 6-8 min → 10-12 min with comprehensive plugin coverage

**Impact**: Chapter now MORE comprehensive than official Anthropic docs

**Reusable Pattern**: When teaching AI tools, ALWAYS include personalization/extensibility layer
```

---

## Organizational Memory

### Memory as Strategic Asset

**Location**: `history/` (ADRs, PHRs, audits, intelligence objects)

**Philosophy**: Decisions decay without documentation; memory prevents repeating mistakes

---

### Architecture Decision Records (ADRs)

**Location**: `history/adr/`

**Template**: `.specify/templates/adr-template.md`

**Structure**:
```markdown
# ADR-NNN: [Decision Title]

**Status**: [Proposed | Accepted | Deprecated | Superseded]
**Date**: YYYY-MM-DD
**Deciders**: [Who made this decision]
**Consulted**: [Who provided input]

## Context
[What problem are we solving? What constraints exist?]

## Decision
[What did we decide? Be explicit and unambiguous.]

## Rationale
[WHY did we decide this? What alternatives were considered?]

## Consequences
**Positive**:
- [Expected benefit 1]

**Negative**:
- [Accepted trade-off 1]

**Neutral**:
- [Side effect 1]

## Compliance
[Which constitution principles does this align with?]
```

**Example ADRs**:
- ADR-001: Adopt 4-Layer Teaching Method
- ADR-002: Constitution v6.0.0 — Reasoning Mode Transition
- ADR-003: Meta-Commentary Prohibition (Chapter 9 Failure Mode)

---

### Prompt History Records (PHRs)

**Location**: `history/prompts/` (organized by stage: constitution, spec, plan, tasks, general)

**Template**: `.specify/templates/phr-template.prompt.md`

**Purpose**: Capture significant human-AI interactions for learning and traceability

**Structure**:
```yaml
---
phr_id: "PHR-YYYY-MM-DD-SLUG"
stage: [constitution | spec | plan | tasks | general]
feature: [feature-name if applicable]
date: "YYYY-MM-DD"
title: "[3-7 word description]"
---

## Prompt

[Full prompt text, verbatim]

## Response Summary

[Concise summary of AI response]

## Key Learnings

- [What we learned from this interaction]

## Follow-Up Actions

- [What actions resulted from this exchange]
```

**PHR Creation Workflow**:
1. After main request completes, determine stage
2. Generate title (3-7 words)
3. Run `.specify/scripts/bash/create-phr.sh --title "title" --stage spec --feature chapter-5`
4. Fill remaining placeholders (embed full PROMPT_TEXT and concise RESPONSE_TEXT)
5. Validate (no unresolved placeholders, path matches stage, coherent metadata)

**Routing by Stage**:
- `constitution` → `history/prompts/constitution/`
- Feature stages (spec/plan/tasks/red/green/refactor/explainer/misc) → `history/prompts/<feature-name>/`
- `general` → `history/prompts/general/`

---

### Audits

**Location**: `history/audits/`

**Types**:
1. **Validation Audits** — From validation-auditor agent (6-category quality assessment)
2. **Constitutional Compliance Audits** — From educational-validator (framework invisibility check)
3. **Factual Accuracy Audits** — From factual-verifier (citation validation)
4. **Cognitive Load Audits** — From pedagogical-designer (proficiency tier compliance)

**Audit Structure**:
```markdown
# Audit: [Chapter/Lesson Name]

**Date**: YYYY-MM-DD
**Auditor**: [Agent name]
**Scope**: [What was audited]
**Status**: [Pass | Fail | Pass with Recommendations]

## Findings

### Critical Issues (Must Fix)
- [Issue 1 with line reference]

### Major Issues (Should Fix)
- [Issue 1 with line reference]

### Minor Issues (Nice to Have)
- [Issue 1 with line reference]

## Recommendations

[Actionable improvements]

## Constitutional Compliance

[Which principles were validated? Any violations?]
```

---

### Intelligence Objects

**Location**: `history/intelligence/`

**Purpose**: Capture emergent patterns, meta-learnings, and reusable insights

**Examples**:
- `history/intelligence/chapter-5-plugin-architecture-discovery.md` — Reusable pattern: Always include personalization layer when teaching AI tools
- `history/intelligence/reasoning-activation-pattern-validation.md` — P+Q+P pattern effectiveness metrics across 20 chapters
- `history/intelligence/cognitive-load-tier-calibration.md` — A2 optimal concepts: 5-7, B1: 7-10, C2: no limit (validated with student completion data)

**Intelligence Object → Skill Pipeline**:

When a pattern appears 3+ times in intelligence objects, consider promoting to skill:
1. Draft skill using P+Q+P template
2. Invoke @skill-creator for validation
3. Test skill on 2-3 features
4. If successful, add to `.claude/skills/` permanently

---

## Workflows and Orchestration

### Super Orchestra Workflow (40x Engineer)

**Trigger**: Complex feature requiring depth + intelligence + quality

**Phases**:

#### Phase 1: Deep Thinking (Human Value Addition)
- **Input**: User identifies gap or high-value opportunity
- **Output**: Problem statement, strategic context, business value hypothesis
- **Example**: "Chapter 5 missing MCP/plugins — critical for AIDD personalization"

#### Phase 2: Deep Research (AI Value Addition)
- **Tools**: Context7 (8000 tokens), WebFetch (3+ sources), source code analysis
- **Output**: Comprehensive intelligence gathering, cross-reference analysis
- **Example**: Context7 `/anthropics/claude-code` + WebFetch official blogs + existing chapter patterns

#### Phase 3: Deep Planning (Co-Learning)
- **Workflow**: spec.md → plan.md → tasks.md (iterative refinement)
- **Output**: Detailed specifications with measurable success criteria
- **Example**: Chapter 5 expanded from 3 → 10 acceptance scenarios (+233%)

#### Phase 4: Deep Search (Pattern Recognition)
- **Tools**: Constitution, existing specs, domain skills, ADRs
- **Output**: Constitutional alignment verification, pattern application, anti-convergence check
- **Example**: Applied Principle 2 (Graduated Teaching), Layer 2 (Three Roles), Cognitive Load (A2 = 5-7 concepts)

#### Phase 5: Agentic Execution
- **Agents**: chapter-planner → content-implementer → validation-auditor → educational-validator
- **Output**: Publication-ready content validated against quality gates
- **Example**: Lesson 4 expanded 6-8 min → 10-12 min with 4 comprehensive sections

#### Phase 6: Business Value Validation
- **Metric**: Does output surpass best existing alternative?
- **Example**: Chapter 5 now MORE COMPREHENSIVE than all official Anthropic resources (10 unique value-adds documented in super-orchestra agent header)

**Result**: 40x productivity from DEPTH (not speed)

---

### Standard Chapter Creation Workflow

**For typical chapters** (not requiring super-orchestra depth):

```
1. User Request → Clarify scope and objectives

2. spec-architect (if spec unclear)
   ↓ Validate completeness, testability, clarity

3. super-orchestra OR direct to chapter-planner
   ↓ If simple chapter → chapter-planner
   ↓ If complex/market-defining → super-orchestra

4. chapter-planner
   ↓ Transform spec.md → plan.md (lesson breakdown)
   ↓ Cognitive load validation (A2/B1/C2)
   ↓ Proficiency mapping (CEFR/Bloom's)

5. content-implementer (per lesson)
   ↓ Create lesson markdown with Layer 1-4 structure
   ↓ Write code examples (tested via code-validation-sandbox)
   ↓ Design "Try With AI" sections (framework INVISIBLE)

6. validation-auditor
   ↓ 6-category quality assessment
   ↓ Identify critical/major/minor issues

7. factual-verifier (if needed)
   ↓ Citation validation
   ↓ Volatile content flagging

8. educational-validator
   ↓ Constitutional compliance check
   ↓ Meta-commentary grep validation
   ↓ Framework invisibility verification

9. PUBLICATION (if all gates pass)
```

---

### Parallel Lesson Creation (Scaling)

**When to use**: Multiple lessons with clear specifications, no interdependencies

**Setup**:
```bash
# Create Git worktrees for isolation
git worktree add ../lesson-1-workspace specs/feature-X
git worktree add ../lesson-2-workspace specs/feature-X
git worktree add ../lesson-3-workspace specs/feature-X

# Invoke content-implementer in each workspace
# Each agent operates independently with shared spec.md + plan.md

# After completion, merge back to main
git worktree remove ../lesson-1-workspace
# (repeat for other worktrees)
```

**Coordination Mechanism**: Shared specification (spec.md + plan.md) provides contract; worktrees enable isolation

**Throughput**: 3-5 lessons concurrently (validated 2.5-3x speedup in Chapter 34 capstone)

---

## Templates and Standards

### Template Locations

**Location**: `.specify/templates/` (13 templates)

1. **spec-template.md** — Feature specification structure
2. **plan-template.md** — Lesson breakdown structure
3. **tasks-template.md** — Implementation checklist structure
4. **adr-template.md** — Architecture Decision Record structure
5. **phr-template.prompt.md** — Prompt History Record structure
6. **checklist-template.md** — Custom validation checklist
7. **agent-file-template.md** — Agent definition structure (name, description, model, color, P+Q+P)
8. **constitution-sync-report-template.md** — Constitution compliance report
9. **error-analysis-report-template.md** — Systematic error diagnosis
10. **book/chapter-readme-template.md** — Chapter README structure
11. **book/lesson-template.md** — Lesson markdown structure (Layer 1-4)
12. **book/part-readme-template.md** — Part README structure

---

### Output Styles

**Location**: `.claude/output-styles/` (structural + pedagogical + workflows)

#### Structural Output Styles

1. **file-organization.md** — Directory structure, naming conventions
2. **chapter-readme-template.md** — Chapter introduction format
3. **part-readme-template.md** — Part introduction format
4. **lesson-template.md** — Lesson markdown structure

#### Pedagogical Output Styles

1. **layer-1-foundation.md** — Manual practice patterns, direct teaching
2. **layer-2-collaboration.md** — Three Roles framework (INVISIBLE), "Try With AI" structure
3. **layer-3-intelligence.md** — Skill/subagent creation, P+Q+P pattern
4. **layer-4-orchestration.md** — Spec-driven capstone, multi-agent coordination
5. **decision-tree.md** — Which layer applies? (Recognition framework)

#### Workflow Output Styles

1. **super-orchestra.md** — 40x engineer workflow (6-phase process)

---

### Validation Standards

**Constitutional Compliance Checks** (from educational-validator):

```bash
# 1. Meta-commentary check
grep -i "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows" [lesson-file.md]
# Expected: Zero matches

# 2. Layer label check
grep -E "Layer [0-9]|Stage [0-9]|Three Roles (Framework|in Action)" [lesson-file.md]
# Expected: Zero matches (or only in acceptable context like transition narration)

# 3. Lesson ending check
tail -50 [lesson-file.md] | grep -E "^## " | tail -1
# Expected: "## Try With AI" (ONLY permitted final section)

# 4. Framework exposition check
awk '/^## Try With AI/,0' [lesson-file.md] | grep -E "^## (What's Next|Key Takeaways|Summary|Safety Note)"
# Expected: Zero matches (forbidden sections after "Try With AI")
```

**Code Validation** (from code-validation-sandbox skill):
```bash
# All code examples must be tested in isolated environment before publication
# Test framework: pytest (Python), tsc (TypeScript), shellcheck (Bash)
```

**Citation Validation** (from factual-verifier):
```bash
# All factual claims must have citations:
# - Official documentation (Context7 MCP)
# - Authoritative sources (WebFetch)
# - Research papers (if applicable)
```

---

## Usage Patterns

### For Content Creators

#### Starting a New Chapter

1. **Define scope** (clarify with user)
2. **Create spec.md** using `.specify/templates/spec-template.md`
3. **Invoke spec-architect** (validate completeness)
4. **Determine complexity**:
   - **Simple chapter** → Invoke chapter-planner directly
   - **Complex/market-defining** → Invoke super-orchestra (deep research + planning)
5. **Review plan.md** (lesson structure, cognitive load, proficiency mapping)
6. **Invoke content-implementer** per lesson
7. **Invoke validation-auditor** after each lesson
8. **Invoke educational-validator** before publication
9. **Create PHR** documenting significant interactions

#### Refining Existing Content

1. **Read existing spec/plan/tasks** in `specs/[feature]/`
2. **Identify gap** (what's missing, what needs improvement)
3. **Update spec.md** (add FRs, SCs, acceptance criteria)
4. **Update plan.md** (adjust lesson structure, durations)
5. **Update tasks.md** (add/modify implementation steps)
6. **Invoke content-implementer** with updated context
7. **Invoke validation-auditor** to verify improvements
8. **Document learnings** in intelligence object

---

### For AI Agents

#### Before Starting ANY Task

**Context-First Protocol** (Constitution Section I):

```markdown
## Step 1: Read the Learning Context (MANDATORY)

For Chapter Work:
1. Read `apps/learn-app/docs/chapter-index.md`:
   - Locate chapter number
   - Extract Part number (determines prerequisite knowledge)
   - Extract proficiency level (A1/A2/B1/B2/C1/C2)
   - Extract chapter theme and learning objectives
   - Extract prerequisites (what students know BEFORE this chapter)

2. Read Chapter README (`apps/learn-app/docs/[part]/[chapter]/README.md`):
   - Extract lesson structure (how many lessons, what each teaches)
   - Extract pedagogical approach currently used
   - Extract any existing constraints or design decisions

For Lesson Work (additionally):
3. Read previous lesson (if exists) — understand progression
4. Read specification (if exists in `specs/`) — check design decisions

## Step 2: Determine Pedagogical Layer (BEFORE designing)

Ask yourself these questions IN ORDER:

Q1: What does the student already know?
- Check chapter prerequisites from chapter-index.md
- Check Part number (Part 1-2 = no programming, Part 3 = markdown/prompts, Part 4+ = Python)

Q2: What is this chapter teaching?
- Syntax/concepts → Layer 1 (Manual)
- Using tools with AI → Layer 2 (Collaboration)
- Creating reusable patterns → Layer 3 (Intelligence)
- Orchestrating projects → Layer 4 (Spec-Driven)

Q3: Does the user's request match the chapter's natural layer?
- If YES: Proceed with that layer
- If NO: STOP and ask user for clarification

## Step 3: Check for Pedagogical Conflicts

Common conflicts to detect:
- ❌ Teaching syntax as specification writing (Layer 4 thinking applied to Layer 1 chapter)
- ❌ Using examples that require unknown prerequisites
- ❌ Skipping manual foundation (jumping to AI collaboration before schema building)

## Step 4: State Your Understanding (BEFORE starting work)

OUTPUT THIS SUMMARY:

```
CONTEXT GATHERED:
- Chapter: [number] "[title]"
- Part: [number] (Student prerequisite: [what they know])
- Proficiency: [A1/A2/etc]
- Teaching: [core concept being taught]
- Pedagogical Layer: [L1/L2/L3/L4] because [reasoning]
- Approach: [how you'll teach this]
- Potential Conflicts Checked: [any conflicts detected and resolved]
```

If user confirms → Proceed
If user corrects → Update understanding, restate, get confirmation
```

#### Invoking Skills

**Pattern**:
```
1. Identify which skill applies (consult `.claude/skills/` directory)
2. Reference skill in task/plan: "@skill-name"
3. Claude Code loads skill using progressive disclosure:
   - Level 1: Metadata (name, description ~50 tokens)
   - Level 2: Core instructions (SKILL.md ~400-800 tokens)
   - Level 3: Extended resources (loaded on-demand if needed)
4. Apply skill's P+Q+P pattern to current context
5. Document which skills were applied (for intelligence object)
```

#### Creating Intelligence Objects

**When to create**:
- After significant architectural decisions
- After discovering reusable patterns
- After identifying pedagogical insights
- After resolving complex issues

**Structure**:
```markdown
# Intelligence Object: [Feature Name] - [Topic]

**Date**: YYYY-MM-DD
**Context**: [What feature/chapter was being worked on]

## Problem/Opportunity

[What gap or insight triggered this?]

## Research/Analysis

[What investigation was done? Tools used? Sources consulted?]

## Decision/Pattern

[What was decided or discovered?]

## Rationale

[WHY does this matter? What alternatives were considered?]

## Impact

[How does this affect future work? What changes as a result?]

## Reusable Pattern

[Can this become a skill? An agent? An ADR? A principle?]

## Constitutional Alignment

[Which constitution principles does this relate to?]
```

---

### For Researchers & Educators

#### Tracing Decisions from Rationale to Impact

**Example Query**: "Why did we adopt meta-commentary prohibition?"

**Trace Path**:
1. **Research Paper** → `papers/Reasoning_Activation_in_LLMs_arXiv_Complete.md` (Framework Invisibility principle)
2. **Constitution** → `.specify/memory/constitution.md` Section IIa v6.0.1 (Meta-Commentary Prohibition)
3. **ADR** → `history/adr/ADR-003-meta-commentary-prohibition.md` (Chapter 9 failure mode)
4. **Audit** → `history/audits/chapter-9-redesign-audit.md` (Validation before/after)
5. **Intelligence Object** → `history/intelligence/chapter-9-scaffolding-exposure-learnings.md` (Pattern extraction)
6. **Implementation** → `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/README.md` (Final content)

**Result**: Complete provenance from research → policy → implementation → validation

---

#### Measuring RI Effectiveness

**Metrics to Track**:

1. **Reuse Frequency**
   - How many features invoke each skill?
   - Which agents are invoked most frequently?
   - Which specifications are referenced as templates?

2. **Quality Improvements**
   - Validation-auditor pass rate over time (target: 90%+)
   - Critical issues per chapter (target: <2)
   - Revision cycles to publication (target: 1-2)

3. **Productivity Gains**
   - Time from spec approval → publication (baseline: 40 hours → target: 8-12 hours)
   - Parallel lesson creation speedup (validated: 2.5-3x)
   - Super-orchestra workflow ROI (validated: 40x from depth, not speed)

4. **Intelligence Accumulation**
   - Skills created: 22 (growing)
   - ADRs documented: 50+ (complete decision history)
   - Intelligence objects: 30+ (emergent patterns captured)
   - PHRs: 100+ (human-AI interaction learnings)

5. **Constitutional Compliance**
   - Meta-commentary violations (target: 0)
   - Framework exposition incidents (target: 0)
   - Lesson ending violations (target: 0)
   - Cognitive load exceedances per tier (target: <5%)

---

## Evolution and Maintenance

### RI Lifecycle

```
1. EMERGENCE (Intelligence Object)
   ↓ Pattern appears 3+ times

2. FORMALIZATION (Skill/Agent/Template)
   ↓ P+Q+P structure applied

3. VALIDATION (Test on 2-3 features)
   ↓ Effectiveness metrics gathered

4. INTEGRATION (Add to .claude/skills or .claude/agents)
   ↓ Document in this RI Architecture

5. MAINTENANCE (Version, refine, deprecate)
   ↓ Track usage, gather feedback

6. EVOLUTION (Update based on learnings)
   ↓ New patterns emerge → Return to Step 1
```

---

### Version Control for RI

**Git Strategy**:
- **Main branch**: Stable, production-ready RI
- **Feature branches**: spec/[feature-name] for new chapters
- **Worktrees**: Parallel lesson creation (isolated contexts)

**What Gets Versioned**:
- ✅ Specifications (`specs/`)
- ✅ Plans, tasks, intelligence objects (`specs/[feature]/`)
- ✅ ADRs, PHRs, audits (`history/`)
- ✅ Constitution (`.specify/memory/constitution.md`)
- ✅ Agents (`.claude/agents/`)
- ✅ Skills (`.claude/skills/`)
- ✅ Templates (`.specify/templates/`)
- ✅ Output styles (`.claude/output-styles/`)
- ❌ Generated content (`apps/learn-app/docs/`) — regenerable from specs

**Why Generated Content Is Not Primary RI**:
- Code is regenerable from specifications
- Markdown lessons can be recreated from specs/plans/tasks
- Specifications + Skills + Agents = Source of truth
- Generated content is VALIDATED (not primary artifact)

---

### Constitution Evolution

**Current Version**: v6.0.1 (PATCH — Meta-Commentary Prohibition)

**Amendment Process** (from Constitution Section VIII):

**For Minor Changes** (clarifications, examples):
1. Edit `.specify/memory/constitution.md` directly
2. Increment PATCH (6.0.1 → 6.0.2)
3. Commit: "Constitution: [brief change]"

**For Major Changes** (new frameworks, removed principles):
1. Create ADR documenting rationale
2. Increment MAJOR/MINOR (6.0.1 → 6.1.0 or 7.0.0)
3. Impact analysis (which agents affected, migration guide)
4. Update evolution log at top of constitution.md
5. Update this RI Architecture document

**Version History**:
- v6.0.1 (2025-11-18): Meta-Commentary Prohibition (Chapter 9 failure mode fix)
- v6.0.0 (2025-01-17): Reasoning Mode Redesign (P+Q+P pattern, decision frameworks)
- v5.0.0 (prior): Rule-based governance (deprecated)

---

### Deprecating RI

**When to Deprecate**:
- Skill superseded by better pattern
- Agent role merged into another agent
- Template no longer matches current standards

**Process**:
1. Mark as `[DEPRECATED]` in filename or frontmatter
2. Document deprecation reason in ADR
3. Provide migration path (which RI replaces this?)
4. Move to `archived/` subdirectory after 90 days
5. Update this RI Architecture document

**Example**:
```
.claude/skills/old-pattern-skill.md → .claude/skills/archived/old-pattern-skill.md
ADR-050: Deprecate old-pattern-skill, superseded by new-pattern-skill (reason: P+Q+P pattern not applied)
```

---

## Appendix: Quick Reference

### Agent Invocation Cheat Sheet

```
super-orchestra          → Complex features, deep research, market-defining quality
chapter-planner          → Transform spec → lesson breakdown
content-implementer      → Write lesson markdown (Layer 1-4)
validation-auditor       → 6-category quality assessment
factual-verifier         → Citation validation, volatile content flagging
pedagogical-designer     → Learning progression validation
assessment-architect     → Quiz/exercise design
spec-architect           → Specification completeness validation
educational-validator    → Constitutional compliance check (final gate)
```

---

### Skill Invocation Cheat Sheet

```
@learning-objectives          → SMART objective design
@concept-scaffolding          → Prerequisite analysis, cognitive load
@ai-collaborate-teaching      → Three Roles framework patterns
@code-example-generator       → Production-relevant, tested code
@exercise-designer            → Active learning, scaffolded practice
@skills-proficiency-mapper    → CEFR/Bloom's/DigComp mapping
@assessment-builder           → Quiz/exam design
@content-evaluation-framework → 6-category quality rubric
@code-validation-sandbox      → Test all code examples before publication
@skill-creator                → Create new skills using P+Q+P pattern
```

---

### File Location Cheat Sheet

```
Constitution                 → .specify/memory/constitution.md
Agents                      → .claude/agents/[agent-name].md
Skills                      → .claude/skills/[skill-name]/SKILL.md
Specifications              → specs/[feature-name]/spec.md
Plans                       → specs/[feature-name]/plan.md
Tasks                       → specs/[feature-name]/tasks.md
Intelligence Objects        → specs/[feature-name]/intelligence-object.md OR history/intelligence/
ADRs                        → history/adr/ADR-NNN-[title].md
PHRs                        → history/prompts/[stage]/PHR-YYYY-MM-DD-[slug].md
Audits                      → history/audits/[chapter-name]-[date]-audit.md
Templates                   → .specify/templates/[template-name]-template.md
Output Styles               → .claude/output-styles/[category]/[style-name].md
Generated Content (Book)    → apps/learn-app/docs/[part]/[chapter]/[lesson]/README.md
```

---

### Constitutional Principles Quick Reference

```
Principle 1: Specification Primacy        → Intent before implementation
Principle 2: Progressive Complexity       → Cognitive load appropriate for tier (A2/B1/C2)
Principle 3: Factual Accuracy            → All claims cited, code tested
Principle 4: Coherent Structure          → Learning progression (Foundation → Mastery)
Principle 5: Intelligence Accumulation   → Context-rich (not horizontal workflows)
Principle 6: Anti-Convergence Variation  → Teaching patterns vary across chapters
Principle 7: Minimal Sufficient Content  → Every section maps to learning objective
```

---

### Layer Recognition Matrix

| Layer | Signals | Your Mode | Output |
|-------|---------|-----------|--------|
| **L1** | First exposure, stable concept | Direct teacher | Explanations, walkthroughs, practice |
| **L2** | Concept known, complex execution | Three Roles (T/S/C) | Collaborative prompts, convergence |
| **L3** | Recurs 2+, 5+ decisions | Co-designer (P+Q+P) | Skills/subagents |
| **L4** | 3+ components, capstone | Spec validator | Specification review, composition |

---

### Complexity Tier Matrix

| Tier | Concepts/Section | Scaffolding | Options | Examples |
|------|------------------|-------------|---------|----------|
| **A2** | 5-7 | Heavy | Max 2 | Simple, isolated |
| **B1** | 7-10 | Moderate | 3-4 | Intermediate, connected |
| **C2** | No limit | Minimal | 5+ | Production-grade |

---

## Conclusion

This RI Architecture document maps the complete reusable intelligence system of the AI-Native Software Development project, demonstrating:

1. **5-Layer Vertical Intelligence Stack** — Constitution → Skills → Tools → Intelligence Objects → Agents
2. **9 Specialized Agents** — Each with P+Q+P reasoning activation pattern
3. **22 Domain Skills** — Composable expertise across pedagogical, technical, and meta-skill domains
4. **SDD-RI Methodology** — Specifications as living contracts, plans as lesson orchestrators, tasks as execution checklists
5. **Organizational Memory** — ADRs, PHRs, audits, intelligence objects capturing decisions from rationale to impact
6. **Templates and Standards** — 13 templates ensuring consistency across all RI artifacts
7. **Workflows and Orchestration** — Super-orchestra (40x engineer), standard chapter creation, parallel lesson scaling

**The meta-insight**: This project doesn't just teach SDD-RI—it proves it works by using it to create the curriculum itself. Every chapter about specification-driven development was created through specification-driven development. Every agent teaching AI collaboration was designed through AI collaboration. Every skill about reusable intelligence is itself reusable intelligence.

**For practitioners**: This architecture demonstrates what "specifications and intelligence as primary artifacts" looks like at scale. Code is regenerable. Content is regenerable. But the specifications, agents, skills, and organizational memory? Those are the strategic assets that compound with every project.

**For researchers**: This document provides complete provenance tracing from research papers → constitution → implementation → validation, enabling systematic study of how reasoning activation patterns affect educational content quality.

**For future AI agents**: When you're invoked to work on this project, read this document first. It's the architectural blueprint showing where all intelligence lives, how to access it, and how to add to it. You're not just implementing features—you're contributing to an accumulating intelligence system that gets smarter with every interaction.

---

**Document Status**: Living
**Maintenance**: Update when agents/skills/processes change
**Owner**: Project maintainers (human + AI collaboration)
**Last Review**: 2025-11-20

---

## Document Metadata

**Filename**: `docs/ri-architecture.md`
**Purpose**: Complete map of Reusable Intelligence architecture
**Audience**: Content creators, AI agents, researchers, educators
**Dependencies**:
- `.specify/memory/constitution.md` (v6.0.1)
- `papers/SDD-RI Whitepaper.md`
- `papers/From-Reusable-Code-to-Reusable Intelligence.md`
- `papers/Reasoning_Activation_in_LLMs_arXiv_Complete.md`

**Related Documents**:
- `.claude/agents/super-orchestra.md` — 40x engineer workflow
- `.specify/templates/README.md` — Template usage guide
- `history/adr/` — Architecture decisions
- `CLAUDE.md` — Claude Code project instructions

---

**END OF DOCUMENT**
