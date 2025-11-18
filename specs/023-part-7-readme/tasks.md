# Tasks: Part 7 README — AI Cloud Native Development

**Input**: Design documents from `/specs/023-part-7-readme/`
**Prerequisites**: spec.md (approved), plan.md (approved)

**Organization**: Tasks organized by documentation sections to enable incremental content development and validation.

## Format: `[ID] [P?] Description`

- **[P]**: Can write in parallel (different sections, no dependencies)
- Include exact file paths in descriptions

## Path Conventions

- **Target file**: `book-source/docs/07-AI-Cloud-Native-Development/README.md`
- **Reference files**:
  - `specs/book/chapter-index.md` (chapter descriptions)
  - `book-source/docs/06-AI-Native-Software-Development/README.md` (structural template)
  - `.specify/memory/constitution.md` (guidelines)

---

## Phase 1: Setup (Prerequisites)

**Purpose**: Ensure directory structure and reference materials are accessible

- [ ] T001 Create Part 7 directory at `book-source/docs/07-AI-Cloud-Native-Development/` if not exists
- [ ] T002 Verify chapter-index.md contains Part 7 chapter descriptions (50-61)
- [ ] T003 Read Part 6 README structure for pattern reference

---

## Phase 2: Content Drafting (Core Sections)

**Purpose**: Draft the 6 required README sections based on plan.md specifications

### Overview Section (8-10 lines)

- [ ] T004 [P] Draft Overview section in `book-source/docs/07-AI-Cloud-Native-Development/README.md`
  - Position Part 7 as bridge from local development (Parts 1-6) to production deployment
  - Identify target audience: Part 6 graduates (B1-B2 tier, can build agents locally)
  - State value proposition: deployment knowledge necessary for production agent systems
  - Clarify scope boundary: Part 7 = infrastructure/deployment, Part 8 = custom model training

**Validation**: Overview answers "What is Part 7?" and "Why does it matter?" in first paragraph

### Goals Section (6-8 lines)

- [ ] T005 [P] Draft Goals section in `book-source/docs/07-AI-Cloud-Native-Development/README.md`
  - List 4 deployment capabilities developers will gain:
    * Understand agent deployment architectures
    * Implement containerization strategies (Docker)
    * Apply orchestration patterns (Kubernetes, Dapr)
    * Design operational excellence (observability, cost engineering, security)

**Validation**: Goals are specific capabilities (verbs: understand, implement, apply, design), not vague outcomes

### Chapter Progression Section (25-30 lines)

- [ ] T006 Draft Chapter Progression section in `book-source/docs/07-AI-Cloud-Native-Development/README.md`
  - Intro paragraph: 12 chapters organized in 4 capability stages
  - **Foundation (Chapters 50-52)**: FastAPI deep-dive, Docker, Kafka
    * Extract chapter descriptions from `specs/book/chapter-index.md` lines 125-127
  - **Orchestration (Chapters 53-54)**: Kubernetes, CI/CD
    * Extract chapter descriptions from `specs/book/chapter-index.md` lines 128-129
  - **Dapr Framework (Chapters 55-58)**: Microservices, Actors, Workflows, Agents
    * Extract chapter descriptions from `specs/book/chapter-index.md` lines 130-133
  - **Operations Excellence (Chapters 59-61)**: Observability, API gateway, Security
    * Extract chapter descriptions from `specs/book/chapter-index.md` lines 134-136
  - Closing rationale: "Why this sequence?" → Foundation → Orchestration → Framework → Operations

**Validation**: All 12 chapters (50-61) mentioned with brief descriptions, organized in 4 logical groups

### Extended Methodology Note Section (20-25 lines)

- [ ] T007 Draft Extended Methodology Note section in `book-source/docs/07-AI-Cloud-Native-Development/README.md`
  - Positioning statement: "Infrastructure tooling requires an extended methodology beyond the 4-layer teaching method you've used in Parts 1-6."
  - **Layer comparison table** (directly from spec.md FR-003 Expanded):
    ```markdown
    | 4-Layer Method (Parts 1-6) | 7-Layer Framework (Part 7 Kubernetes) | Rationale |
    |----------------------------|----------------------------------------|-----------|
    | Layer 1: Manual Foundation | Layers 1-2: kubectl manual + kubectl-ai | CLI tools need both manual practice and AI-assisted variations |
    | Layer 2: AI Collaboration  | Layers 3-4: kagent + Claude Code | Agents operate at different abstraction levels (imperative vs declarative) |
    | Layer 3: Intelligence Design | Layers 5-6: Helm charts + subagents | Infrastructure has two intelligence types: configs (Helm) and autonomous agents |
    | Layer 4: Spec-Driven Integration | Layer 7: SDD for infrastructure | Spec-driven methodology applies to infrastructure orchestration |
    ```
  - Rationale paragraph: "Infrastructure tooling has more abstraction layers (raw CLI → AI-assisted CLI → orchestrating agents → reusable configurations) than application code, requiring finer-grained teaching progression to build competence at each level."
  - Application guidance:
    * "Kubernetes chapters (53-54) use the full 7-layer framework due to infrastructure complexity"
    * "Dapr chapters (55-58) simplify to 4 layers since Dapr operates at application level, not infrastructure level"
    * "FastAPI and Docker chapters use standard 4-layer method with infrastructure adaptations"
  - Student self-assessment question: "When working through Kubernetes chapter, ask yourself: 'Which layer am I on? Manual kubectl (L1), kubectl-ai (L2), kagent (L3), Claude Code (L4), Helm (L5), subagents (L6), or SDD (L7)?'"

**Validation**: Table has 3 columns (4-layer, 7-layer, rationale), all elements from spec.md FR-003 Expanded present

### Capstone Expectations Section (8-10 lines)

- [ ] T008 [P] Draft Capstone Expectations section in `book-source/docs/07-AI-Cloud-Native-Development/README.md`
  - State production operations focus (not just basic deployment)
  - List operational concerns: observability configurations, cost analysis, security setups
  - Contrast with "Docker tutorials only" approach
  - Clarify: local learning projects (not production deployments to live users)
  - Reference spec-driven development from Part 5 (capstones write specs first)

**Validation**: Capstone description includes at least 3 operational concerns (observability, cost, security)

### Getting Started Section (5-7 lines)

- [ ] T009 [P] Draft Getting Started section in `book-source/docs/07-AI-Cloud-Native-Development/README.md`
  - Direct students to begin with Chapter 50: FastAPI for AI Cloud-Native Services (Deep Dive)
  - State prerequisites explicitly:
    * Parts 4-5 REQUIRED (Python fundamentals + spec-driven development)
    * Part 6 STRONGLY RECOMMENDED (agent building experience)
  - Optional note for experienced DevOps professionals: novel content in Dapr chapters (55-58) and agent-specific patterns

**Validation**: Prerequisites clearly marked as REQUIRED vs RECOMMENDED

---

## Phase 3: Refinement (Tone & Length)

**Purpose**: Ensure tone matches "bridge from development to DevOps/SRE" and length is within 75-90 lines

- [ ] T010 Review tone across all sections for approachability
  - Check: uses developer-friendly language (not SRE jargon)
  - Check: introduces operations concepts as new material (doesn't assume ops expertise)
  - Apply tone examples from spec.md (approachable bridge tone, not ops-centric)

- [ ] T011 Verify length constraint: README should be 75-90 lines total
  - Count current line count
  - If <75 lines: expand chapter descriptions or methodology explanation
  - If >90 lines: condense without losing essential information
  - Adjust while maintaining all 8 functional requirements (FR-001 through FR-008)

- [ ] T012 Validate anti-convergence: compare structure to Part 6 README
  - Part 6 used: thematic stages (Agent Frameworks → Integration → Quality → Patterns → Data)
  - Part 7 uses: capability progression (Foundation → Orchestration → Framework → Operations)
  - Confirm: different organizational principle, both valid, variation prevents convergence

---

## Phase 4: Validation (Completeness & Quality)

**Purpose**: Ensure all functional requirements satisfied and success criteria met

- [ ] T013 Validate completeness against spec.md requirements (FR-001 through FR-008)
  - [ ] FR-001: README positions Part 7 as bridge from local dev to production deployment ✓
  - [ ] FR-002: 12-chapter progression previewed with 4 thematic groups ✓
  - [ ] FR-003: 7-layer framework explained with comparison table, rationale, application guidance ✓
  - [ ] FR-004: Production operations expectations set (observability, cost, security) ✓
  - [ ] FR-005: Prerequisites identified (Parts 4-5 required, Part 6 recommended) ✓
  - [ ] FR-006: Scope boundaries clarified (Part 7 vs Part 8) ✓
  - [ ] FR-007: Tone is "bridge from dev to ops" (approachable, not assuming expertise) ✓
  - [ ] FR-008: Follows Part 6 README pattern (6-8 sections, ~75-90 lines) ✓

- [ ] T014 Validate success criteria from spec.md (SC-001 through SC-006)
  - [ ] SC-001: Developer can identify Part 7 focus in <3 min (Overview clarity) ✓
  - [ ] SC-002: 90% correctly sequence prerequisites (Chapter Progression detail with 4 groups) ✓
  - [ ] SC-003: Developer explains 7-layer difference (Methodology table + rationale present) ✓
  - [ ] SC-004: Capstone plans include ops concerns (Capstone Expectations lists 3+ concerns) ✓
  - [ ] SC-005: Aligns with Part 6 structure (6 sections, 75-90 lines) ✓
  - [ ] SC-006: Zero skip prerequisites (Getting Started emphasis on Parts 4-5 required) ✓

- [ ] T015 Self-check against plan.md validation criteria
  - All 8 functional requirements addressed ✓
  - 7-layer framework table has 3 columns ✓
  - All 4 chapter groups described (Foundation, Orchestration, Dapr, Operations) ✓
  - Prerequisites stated, scope boundaries clarified ✓
  - Length within 75-90 lines ✓
  - 6 required sections present (Overview, Goals, Progression, Methodology, Capstone, Getting Started) ✓

---

## Phase 5: Polish & Finalization

**Purpose**: Final formatting and constitutional compliance

- [ ] T016 Add Docusaurus frontmatter (if required by book structure)
  - Check existing Part READMEs for frontmatter format
  - Add metadata if needed (title, sidebar label, etc.)

- [ ] T017 Final markdown formatting check
  - Headers properly nested (# Part 7, ## Section headings)
  - Lists formatted consistently (bullet points for goals, chapters)
  - Table formatted correctly (7-layer comparison table)
  - No markdown syntax errors

- [ ] T018 Constitutional principle compliance final check
  - Principle 1 (Specification Primacy): WHAT before HOW ✓
  - Principle 2 (Progressive Complexity): B1-B2 tier appropriate ✓
  - Principle 3 (Factual Accuracy): Infrastructure claims verifiable ✓
  - Principle 4 (Coherent Structure): Pedagogical progression maintained ✓
  - Principle 5 (Intelligence Accumulation): Builds on Part 6 pattern ✓
  - Principle 6 (Anti-Convergence): Capability progression vs thematic stages ✓
  - Principle 7 (Minimal Content): Orientation only, no exhaustive tutorials ✓

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Content Drafting (Phase 2)**: Depends on Setup (T001-T003) - sections T004-T009 can be written in parallel
- **Refinement (Phase 3)**: Depends on Content Drafting completion (all 6 sections drafted)
- **Validation (Phase 4)**: Depends on Refinement completion
- **Polish (Phase 5)**: Depends on Validation passing

### Section Dependencies

- **Overview, Goals, Capstone, Getting Started** (T004, T005, T008, T009): Can be written in parallel [P]
- **Chapter Progression** (T006): Depends on T002 (chapter-index.md access)
- **Extended Methodology Note** (T007): Can be written independently (spec.md has full table)

### Parallel Opportunities

**Phase 2 (Content Drafting)** - After Setup complete:
```bash
# These 4 sections can be drafted simultaneously by different writers:
Task T004: Draft Overview section
Task T005: Draft Goals section
Task T008: Draft Capstone Expectations section
Task T009: Draft Getting Started section

# These 2 sections require sequential attention:
Task T006: Draft Chapter Progression (needs chapter-index.md)
Task T007: Draft Extended Methodology Note (can write independently)
```

---

## Implementation Strategy

### Sequential Approach (Single Writer)

1. **Phase 1**: Setup (T001-T003) — ~10 minutes
2. **Phase 2**: Content Drafting (T004-T009) — ~60-90 minutes
   - Draft all 6 sections sequentially or in parallel
3. **Phase 3**: Refinement (T010-T012) — ~20 minutes
   - Tone review, length adjustment, anti-convergence check
4. **Phase 4**: Validation (T013-T015) — ~15 minutes
   - Checklist validation against spec + plan
5. **Phase 5**: Polish (T016-T018) — ~10 minutes
   - Formatting, constitutional compliance

**Total Estimated Time**: 2-2.5 hours for complete README

### Parallel Approach (Multiple Writers)

1. **Phase 1**: Setup (single coordinator) — 10 minutes
2. **Phase 2**: Content Drafting (4 parallel writers)
   - Writer A: Overview + Getting Started (T004, T009) — 30 minutes
   - Writer B: Goals + Capstone (T005, T008) — 30 minutes
   - Writer C: Chapter Progression (T006) — 40 minutes
   - Writer D: Extended Methodology Note (T007) — 40 minutes
3. **Phase 3**: Refinement (single editor) — 20 minutes
4. **Phase 4**: Validation (single reviewer) — 15 minutes
5. **Phase 5**: Polish (single finalizer) — 10 minutes

**Total Elapsed Time**: ~90 minutes with parallel execution

---

## Validation Checkpoints

### After Phase 2 (Content Drafting)

**STOP and CHECK**:
- [ ] All 6 sections drafted (Overview, Goals, Chapter Progression, Methodology, Capstone, Getting Started)
- [ ] 7-layer framework table complete with 3 columns
- [ ] All 12 chapters (50-61) mentioned in Chapter Progression
- [ ] Length approximately 75-90 lines

### After Phase 3 (Refinement)

**STOP and CHECK**:
- [ ] Tone is approachable "bridge from dev to ops" (not SRE jargon)
- [ ] Length within 75-90 lines
- [ ] Structure differs from Part 6 (capability progression vs thematic stages)

### After Phase 4 (Validation)

**STOP and CHECK**:
- [ ] All 8 functional requirements (FR-001 through FR-008) satisfied
- [ ] All 6 success criteria (SC-001 through SC-006) met
- [ ] All validation criteria from plan.md checked

### After Phase 5 (Polish)

**READY FOR PUBLICATION**:
- [ ] Markdown formatted correctly
- [ ] Constitutional principles compliant
- [ ] File ready at `book-source/docs/07-AI-Cloud-Native-Development/README.md`

---

## Notes

- [P] tasks = different sections, can write in parallel
- This is documentation, not code - no tests required
- Validation is against spec.md requirements and plan.md validation criteria
- Length target: 75-90 lines (CON-002 allows +15 lines for methodology explanation)
- Commit after each phase completion for incremental progress
- If implementing alone: work sequentially through phases
- If team available: parallelize Phase 2 content drafting across sections
