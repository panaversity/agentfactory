# Implementation Plan: Expand Book Structure (7 Parts → 13 Parts, 32 Chapters → 46 Chapters)

**Branch**: `001-expand-book-structure` | **Date**: 2025-10-29 | **Spec**: [spec.md](spec.md)  
**Input**: Feature specification from `/specs/001-expand-book-structure/spec.md`

**Note**: This plan is tailored for documentation and content structure updates, not traditional code implementation.

## Summary

This feature expands the book "CoLearning Programming & Agentic AI with Python and TypeScript: The AI-Driven Way" from 7 parts/32 chapters to 13 parts/46 chapters. The expansion adds advanced topics including TypeScript, Realtime/Voice Agents, Docker/Kubernetes, Databases, Kafka/Dapr, and Stateful Agents while restructuring existing parts for better coherence.

**Technical Approach**:
1. Update constitution and authoritative documentation (chapter-index, directory-structure) to reflect new structure
2. Create detailed chapter mapping document to track all restructuring changes
3. Reorganize existing content (Parts 1-5) to match new chapter counts
4. Create comprehensive part specs for 8 new parts (Parts 6-13)
5. Update Docusaurus physical directory structure
6. Update README and CLAUDE.md references

**Priority**: P1 (Foundation) → P2 (Restructuring) → P3 (New Content Scaffolding)

## Technical Context

**Project Type**: Educational book content (Markdown + Docusaurus)  
**Content Format**: Markdown with Docusaurus 3.9.2 frontmatter  
**Primary Tools**: Docusaurus 3.9.2, Markdown, Git  
**Target Platform**: Web (Docusaurus static site), PDF, Kindle/Leanpub  
**Version Control**: Git with branch-per-feature workflow  
**Quality Standards**: Constitution v2.1.0, 8 CoLearning Skills, Output Styles  

**Current Structure**:
- 7 parts, 32 chapters
- Parts 1-5 with existing content (some partially complete)
- Parts 6-7 with scaffolding only

**Target Structure**:
- 13 parts, 46 chapters
- Parts 1-5 restructured with different chapter counts
- Parts 6-13 fully scaffolded with comprehensive specs

**Content Standards**:
- Python 3.13+ with type hints (for code examples)
- TypeScript (for Parts 8-9)
- AI-first pedagogy
- Progressive complexity management
- Docusaurus-compatible structure

**Performance Goals**: N/A (documentation project)  
**Constraints**: 
- Must maintain Docusaurus compatibility
- Zero broken links after restructuring
- All existing content preserved (no data loss)
- Sequential chapter numbering (1-46 globally)

**Scale/Scope**: 
- 13 parts, 46 chapters total
- 8 new part specs required
- ~14 new chapter specs required
- 100+ file moves/renames expected

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Project Constitution v2.1.0

✅ **Aligned**:
- Progressive complexity maintained (new advanced topics in later parts)
- Consistent structure across all parts (using book-scaffolding skill)
- Clear prerequisite chains (foundational → intermediate → advanced)
- Specification-first methodology applied (creating specs before content)
- Multiple AI tools covered (Part 2 expanded to 4 chapters)
- Real-world project integration (new deployment parts: Docker, Kubernetes, Kafka)

✅ **Enhanced**:
- Tool diversity improved (Part 2: AI Tool Landscape expanded from 4 to 4 chapters, maintaining coverage)
- Real-world deployment skills added (Parts 10-13)
- TypeScript added for full-stack AI development (Parts 8-9)
- Advanced agentic patterns emphasized (Parts 6-7 remain, new stateful agents in Part 13)

⚠️ **Requires Update**:
- Constitution references "7 parts" and "32 chapters" — must be updated to "13 parts" and "46 chapters"
- Scaffolding ranges need adjustment (currently 1-10, 11-20, 21-32 → should be 1-10, 11-25, 26-46)
- Section III (Book Structure) must reference new part count

**Gate Result**: ✅ **PASS** — No violations. Constitution updates required as part of P1 work.

## Project Structure

### Documentation (this feature)

```text
specs/001-expand-book-structure/
├── spec.md                          # Feature specification (COMPLETE)
├── plan.md                          # This file (IN PROGRESS)
├── checklists/
│   └── requirements.md              # Spec quality checklist (COMPLETE)
├── research.md                      # Phase 0: Research findings (TO BE CREATED)
├── chapter-mapping.md               # Phase 1: Old → New chapter mapping (TO BE CREATED)
├── part-templates/                  # Phase 1: Part spec templates for Parts 6-13 (TO BE CREATED)
│   ├── part-6-outline.md
│   ├── part-7-outline.md
│   ├── part-8-outline.md
│   ├── part-9-outline.md
│   ├── part-10-outline.md
│   ├── part-11-outline.md
│   ├── part-12-outline.md
│   └── part-13-outline.md
└── tasks.md                         # Phase 2: Detailed task breakdown (NEXT COMMAND)
```

### Content Structure (repository root)

**Current State** (`book-source/docs/`):
```text
docs/
├── 01-Introducing-AI-Driven-Development/     (5 chapters → needs consolidation to 3)
├── 02-AI-Tool-Landscape/                     (4 chapters → stays 4)
├── 03-Prompt-and-Context-Engineering/        (4 chapters → needs consolidation to 2)
├── 04-Modern-Python-with-Type-Hints/         (8 chapters → needs expansion to 12)
├── 05-Spec-Kit-Methodology/                  (5 chapters → needs consolidation to 3)
├── 06-Agentic-AI-Fundamentals/               (3 chapters → stays 3, may need renaming)
└── 07-MCP-Fundamentals/                      (3 chapters → stays 3, may need renaming)
```

**Target State** (`book-source/docs/`):
```text
docs/
├── 01-Introducing-AI-Driven-Development/     (3 chapters)
├── 02-AI-Tool-Landscape/                     (4 chapters)
├── 03-Prompt-and-Context-Engineering/        (2 chapters)
├── 04-Python-The-Language-of-AI-Agents/      (12 chapters)
├── 05-Spec-Kit-Plus-Methodology/             (3 chapters)
├── 06-Agentic-AI-with-OpenAI-Agents-SDK/     (3 chapters - NEW/RENAMED)
├── 07-MCP-with-FastMCP/                      (3 chapters - NEW/RENAMED)
├── 08-TypeScript-Realtime-Interaction/       (3 chapters - NEW)
├── 09-Realtime-Voice-Agents/                 (3 chapters - NEW)
├── 10-Docker-Kubernetes/                     (3 chapters - NEW)
├── 11-Databases-State-Memory/                (3 chapters - NEW)
├── 12-Kafka-Dapr-Event-Architecture/         (2 chapters - NEW)
└── 13-Dapr-Stateful-Agents/                  (2 chapters - NEW)
```

**Spec Structure** (`specs/`):
```text
specs/
├── book/
│   ├── chapter-index.md              (UPDATE: 32 → 46 chapters)
│   └── directory-structure.md        (UPDATE: Add examples for new parts)
├── part-1/
│   └── part-1-spec.md                (UPDATE: 5 → 3 chapters)
├── part-2/                           (CREATE if missing)
├── part-3/                           (CREATE if missing)
├── part-4/                           (CREATE if missing)
├── part-5/                           (CREATE if missing)
├── part-6/                           (CREATE - NEW)
├── part-7/                           (CREATE - NEW)
├── part-8/                           (CREATE - NEW)
├── part-9/                           (CREATE - NEW)
├── part-10/                          (CREATE - NEW)
├── part-11/                          (CREATE - NEW)
├── part-12/                          (CREATE - NEW)
└── part-13/                          (CREATE - NEW)
```

**Structure Decision**: This is a **documentation restructuring project**, not a traditional software project. The "source code" is the book content itself (Markdown files). We use a dual structure:
1. **Specs** (`specs/`) — Authoritative planning documents
2. **Published Content** (`book-source/docs/`) — Docusaurus-compatible Markdown

## Complexity Tracking

> This feature has no Constitution violations. This section documents the inherent complexity of restructuring 32 → 46 chapters.

| Complexity Factor | Description | Mitigation Strategy |
|-------------------|-------------|---------------------|
| **Scope of Change** | 46 total chapters (14 new, 32 existing with potential remapping) | Phased approach: P1 (docs) → P2 (restructure) → P3 (new scaffolding) |
| **Content Consolidation** | Part 1 (5→3), Part 3 (4→2), Part 5 (5→3) require merging existing content | Create detailed chapter mapping document; preserve all content in git history |
| **Content Expansion** | Part 4 (8→12) requires identifying 4 new chapters | Use book-scaffolding skill to identify natural topic splits |
| **Naming Consistency** | Parts 6-7 may need renaming for clarity (e.g., "with OpenAI Agents SDK", "with FastMCP") | Research best naming conventions; align with industry standards |
| **Cross-Reference Updates** | All internal links and chapter references must be validated | Create automated validation script; manual review as fallback |
| **Docusaurus Compatibility** | Structure changes must not break Docusaurus build | Test build after each phase; use `_category_.json` where needed |

**Risk Mitigation**: Git branch isolation (`001-expand-book-structure`) allows rollback if issues arise. Each phase has clear acceptance criteria.

---

## Phase 0: Research & Analysis

**Objective**: Resolve open questions, establish naming conventions, and create detailed chapter mapping.

### Research Tasks

1. **Part Naming Conventions** (Priority: HIGH)
   - **Question**: Should Parts 6-7 be renamed to include technology specifics (e.g., "Agentic AI Fundamentals with OpenAI Agents SDK in Python", "MCP Fundamentals with FastMCP")?
   - **Research**: Review industry naming standards for educational content; check Docusaurus sidebar length limits
   - **Deliverable**: Naming decision with rationale

2. **Chapter Consolidation Strategy** (Priority: HIGH)
   - **Question**: How should existing chapters be merged for Part 1 (5→3), Part 3 (4→2), Part 5 (5→3)?
   - **Research**: Analyze existing chapter content; identify natural groupings; ensure no critical content loss
   - **Deliverable**: Chapter consolidation mapping with content preservation plan

3. **Chapter Expansion Strategy** (Priority: HIGH)
   - **Question**: What 4 new chapters should be added to Part 4 (Python) to expand from 8→12 chapters?
   - **Research**: Review Python curriculum best practices; identify gaps in current coverage; align with AI-driven development focus
   - **Deliverable**: List of 4 new Python chapters with topics and rationale

4. **New Part Topics** (Priority: MEDIUM)
   - **Question**: What specific topics should each of the 8 new parts cover?
   - **Research**: Industry standards for TypeScript, Realtime AI, Docker/Kubernetes, Databases (SQL/Graph/Vector), Kafka/Dapr, Stateful Agents
   - **Deliverable**: Topic outline for each new part (high-level)

5. **Scaffolding Range Adjustment** (Priority: MEDIUM)
   - **Question**: How should scaffolding ranges be adjusted for 46 chapters (currently 1-10 heavy, 11-20 moderate, 21-32 minimal)?
   - **Research**: Apply cognitive load principles; consider natural breakpoints in content progression
   - **Deliverable**: New scaffolding ranges with pedagogical rationale

6. **Docusaurus Build Validation** (Priority: LOW)
   - **Question**: Are there any Docusaurus 3.9.2 limits or best practices for 13-part, 46-chapter structures?
   - **Research**: Review Docusaurus documentation; test sidebar generation with large structures
   - **Deliverable**: Validation checklist and any configuration adjustments needed

### Research Output

**Deliverable**: `research.md` containing:
- All research findings with decisions and rationale
- Chapter consolidation mapping (old → new)
- Chapter expansion topics (new chapters for Part 4)
- Part naming conventions finalized
- Scaffolding range recommendations
- Docusaurus validation results

**Acceptance Criteria**:
- All NEEDS CLARIFICATION items from spec resolved
- Every decision has documented rationale and alternatives considered
- Chapter mapping is complete and reversible (can trace any old chapter to new location)

---

## Phase 1: Design & Structural Planning

**Prerequisites**: `research.md` complete

**Objective**: Create comprehensive structural documentation and part templates.

### Deliverables

1. **Chapter Mapping Document** (`chapter-mapping.md`)
   - **Purpose**: Definitive old → new chapter mapping
   - **Contents**:
     - Full 32→46 chapter mapping table
     - Part reassignments with rationale
     - Content consolidation strategy for merged chapters
     - Content expansion strategy for new chapters
     - Cross-reference update plan
   - **Format**: Markdown table with columns: Old Part, Old Ch#, Old Title, New Part, New Ch#, New Title, Action (Keep/Merge/Split/New), Notes

2. **Part Templates** (`part-templates/part-N-outline.md` for N=6 to 13)
   - **Purpose**: High-level outlines for 8 new parts
   - **Contents** (per part):
     - Part title and subtitle
     - Part philosophy and learning outcomes
     - Target audience and prerequisites
     - Chapter breakdown (titles and key topics)
     - Cognitive load strategy
     - Connections to prior/subsequent parts
   - **Format**: Simplified version of part-spec template (outline only, not full spec)

3. **Updated Authoritative Documents**:
   - `specs/book/chapter-index.md` — Updated to 46 chapters across 13 parts
   - `specs/book/directory-structure.md` — Add examples for new part naming (Parts 8-13)
   - `.specify/memory/constitution.md` — Update all references from "7 parts, 32 chapters" to "13 parts, 46 chapters"; adjust scaffolding ranges

4. **README and CLAUDE.md Updates**:
   - `README.md` — Update "Structure Overview" section to list all 13 parts
   - `CLAUDE.md` — Verify it defers to constitution/chapter-index (should already be decoupled from v2.1.0)

### Design Decisions

**Part Naming** (from research.md):
- Part 6: "Agentic AI Fundamentals with OpenAI Agents SDK in Python"
- Part 7: "MCP Fundamentals with FastMCP"
- Part 8: "TypeScript: The Language of Realtime and Interaction"
- Part 9: "Building Realtime and Voice Agents"
- Part 10: "Containerization & Orchestration using Docker and Kubernetes"
- Part 11: "Data, State, and Memory using PostgreSQL, Graph, and Vector Databases"
- Part 12: "Event-Driven Architecture using Kafka and Dapr"
- Part 13: "Stateful Agents using Dapr Actors and Dapr Workflows"

**Scaffolding Ranges** (from research.md):
- Chapters 1-10: Heavy scaffolding (foundational)
- Chapters 11-25: Moderate scaffolding (skill building)
- Chapters 26-46: Minimal scaffolding (advanced, assumes independence)

**Chapter Consolidation** (from research.md):
- Part 1: 5→3 (merge Chapters 1-2, 4-5; keep Chapter 3)
- Part 3: 4→2 (merge Chapters 10-11, 12-13)
- Part 5: 5→3 (merge Chapters 22-23, 25-26; keep Chapter 24)

**Chapter Expansion** (from research.md):
- Part 4: 8→12 (add 4 new chapters on advanced Python topics: decorators/generators, async/await, metaprogramming, package management)

### Phase 1 Output Structure

```text
specs/001-expand-book-structure/
├── research.md                      ✅ Phase 0 output
├── chapter-mapping.md               🎯 Phase 1 deliverable
├── part-templates/                  🎯 Phase 1 deliverable
│   ├── part-6-outline.md
│   ├── part-7-outline.md
│   ├── part-8-outline.md
│   ├── part-9-outline.md
│   ├── part-10-outline.md
│   ├── part-11-outline.md
│   ├── part-12-outline.md
│   └── part-13-outline.md
└── [Updated files listed above]     🎯 Phase 1 deliverable
```

**Acceptance Criteria**:
- Chapter mapping complete and bidirectional (can go old→new and new→old)
- All 8 part templates follow consistent structure
- Constitution, chapter-index, directory-structure updated and internally consistent
- Docusaurus build succeeds with updated chapter-index (test with placeholder content)
- No TODO or PLACEHOLDER markers remain in updated files

---

## Phase 2: Task Decomposition

**Command**: `/sp.tasks` (separate command, not part of `/sp.plan`)

**Objective**: Break down Phase 1 design into actionable tasks with clear acceptance criteria.

**Output**: `tasks.md` with prioritized task list:
- P1: Update core documentation (constitution, chapter-index, directory-structure, README, CLAUDE.md)
- P2: Restructure existing parts (1, 3, 4, 5)
- P2: Update Docusaurus physical directories
- P3: Create part specs for new parts (6-13)
- P3: Create chapter specs for all new chapters

Each task will have:
- Clear description
- Acceptance criteria
- Files to create/modify
- Dependencies on other tasks
- Estimated complexity

**This phase is NOT created by `/sp.plan`** — it requires the separate `/sp.tasks` command.

---

## Post-Planning: Next Steps

After this plan is complete, execute in order:

1. **Research Phase** (immediate):
   - Answer all research questions
   - Create `research.md`
   - Review with stakeholders

2. **Design Phase** (after research approved):
   - Create `chapter-mapping.md`
   - Create 8 part outline templates
   - Update all authoritative documents
   - Test Docusaurus build

3. **Task Breakdown** (after design approved):
   - Run `/sp.tasks` to generate detailed task list
   - Review and prioritize tasks
   - Begin implementation

4. **Implementation** (iterative):
   - Execute P1 tasks first (documentation updates)
   - Execute P2 tasks second (restructuring)
   - Execute P3 tasks third (new scaffolding)
   - Validate after each phase

5. **Validation** (continuous):
   - Docusaurus build check after each phase
   - Cross-reference validation
   - Constitution compliance check
   - Human review before merge

---

## Risks & Mitigation

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| **Content loss during consolidation** | HIGH | MEDIUM | Git history preservation; detailed chapter mapping; human review of all merges |
| **Broken internal links** | MEDIUM | HIGH | Automated link validation script; manual review; systematic cross-reference update |
| **Docusaurus build failure** | MEDIUM | LOW | Test build after each phase; use `_category_.json` fallbacks; Docusaurus 3.9.2 docs review |
| **Scope creep** (writing new content vs. scaffolding)** | MEDIUM | MEDIUM | Strict adherence to "scaffolding only" for new parts; defer content writing to future features |
| **Inconsistent naming** | LOW | MEDIUM | Follow research.md naming conventions; use templates; peer review |
| **Cognitive load miscalculation** | LOW | LOW | Leverage book-scaffolding skill; follow constitution principles; iterative feedback |

**Overall Risk Level**: **MEDIUM** — Manageable with phased approach and clear validation gates.

---

## Success Criteria (Plan Phase Complete)

This implementation plan is complete when:

- ✅ All sections filled with concrete details (no placeholders)
- ✅ Research tasks clearly defined with deliverables
- ✅ Phase 1 design deliverables specified
- ✅ Chapter mapping approach documented
- ✅ Part template structure defined
- ✅ Constitution updates identified
- ✅ Risks identified and mitigation strategies documented
- ✅ Next steps clearly outlined

**Plan Status**: ✅ COMPLETE

**Ready for**: Phase 0 (Research) → Create `research.md` by answering all research questions

---

**Notes**:
- This plan is intentionally high-level; `/sp.tasks` will create granular task breakdown
- Focus is on structural planning, not content writing
- All new parts start as "scaffolding" (specs/outlines only, not full lessons)
- Existing content preservation is paramount (no content deletion without explicit review)
