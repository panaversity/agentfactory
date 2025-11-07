# Tasks: Cloud Native to Agent Native Cloud - Parts and Chapters READMEs

**Input**: Design documents from `/specs/001-cloud-native-chapters/`
**Prerequisites**: plan.md ✅, spec.md ✅

**Tests**: No tests required (content creation, not code implementation)

**Organization**: Tasks are grouped by user story (3 Parts = 3 user stories) to enable independent implementation and validation of each Part's content.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Part 11, US2=Part 12, US3=Part 13)
- Include exact file paths in descriptions

## Path Conventions

All content files are created in `book-source/docs/` following Docusaurus structure:
- Part READMEs: `book-source/docs/[##-Part-Name]/README.md` (uppercase)
- Chapter readmes: `book-source/docs/[##-Part-Name]/[##-chapter-name]/readme.md` (lowercase)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Validate prerequisites and create directory structure

- [ ] T001 Validate chapter structure against context/cloud/readme.md
- [ ] T002 Validate chapter numbering: Part 11 (50-53), Part 12 (54-58), Part 13 (59-67)
- [ ] T003 Extract learning outcomes from spec.md User Stories 1-3
- [ ] T004 Identify technology stacks per part from spec.md Key Entities
- [ ] T005 Create directory structure: book-source/docs/11-Part-11-Cloud-Native-Infrastructure/
- [ ] T006 Create directory structure: book-source/docs/12-Part-12-Distributed-Agent-Runtime/
- [ ] T007 Create directory structure: book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/
- [ ] T008 Create chapter subdirectories for Part 11 (50-docker-fundamentals, 51-kubernetes-basics, 52-dapr-core, 53-production-kubernetes)
- [ ] T009 Create chapter subdirectories for Part 12 (54-kafka, 55-dapr-actors, 56-dapr-workflows, 57-agent-homes, 58-multi-agent-coordination)
- [ ] T010 Create chapter subdirectories for Part 13 (59-llmops, 60-agentops, 61-agentic-mesh, 62-multi-agent-orchestration, 63-agent-scaling, 64-cost-optimization, 65-compliance-governance, 66-model-governance, 67-daca-synthesis)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Research and validation that MUST be complete before ANY README creation

**⚠️ CRITICAL**: No README writing can begin until this phase is complete

- [ ] T011 Read and extract chapter details from context/cloud/readme.md (authoritative source)
- [ ] T012 Read and extract prerequisite information from context/cloud/prereq.md
- [ ] T013 Read reference Part README: book-source/docs/04-Part-4-Python-Fundamentals/README.md
- [ ] T014 Read reference Chapter readme: book-source/docs/04-Part-4-Python-Fundamentals/12-python-uv-package-manager/readme.md
- [ ] T015 Extract Professional Tier language patterns from constitution (.specify/memory/constitution.md)
- [ ] T016 Document research findings in specs/001-cloud-native-chapters/research.md

**Checkpoint**: Research complete - README creation can now begin in parallel

---

## Phase 3: User Story 1 - Part 11 README + Chapters 50-53 (Priority: P1) 🎯 MVP

**Goal**: Create Part 11 (Cloud Native Infrastructure) overview and 4 chapter readmes establishing "agents as workloads" foundation

**Independent Test**: Part 11 README exists with introduction, "What You'll Learn" section (6-8 outcomes), and "What's Next" section signaling paradigm shift. All 4 chapter readmes exist with proper frontmatter and Professional Tier language.

### Implementation for User Story 1 (Part 11)

- [ ] T017 [US1] Create Part 11 README.md introduction (4 paragraphs: prerequisites connection, part value, chapter preview, AIDD emphasis) in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/README.md
- [ ] T018 [US1] Write Part 11 "What You'll Learn" section (6-8 outcomes: Docker, Kubernetes, DAPR Core, OpenTelemetry, AIDD integration) in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/README.md
- [ ] T019 [US1] Write Part 11 "What's Next" section with explicit paradigm shift signal ("agents as workloads" → "agents as primitives") in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/README.md
- [ ] T020 [P] [US1] Create Chapter 50 (Docker Fundamentals) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with AIDD integration) in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/50-docker-fundamentals/readme.md
- [ ] T021 [P] [US1] Create Chapter 51 (Kubernetes Basics) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with AIDD integration) in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/51-kubernetes-basics/readme.md
- [ ] T022 [P] [US1] Create Chapter 52 (DAPR Core) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with AIDD integration) in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/52-dapr-core/readme.md
- [ ] T023 [P] [US1] Create Chapter 53 (Production Kubernetes) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with AIDD integration) in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/53-production-kubernetes/readme.md
- [ ] T024 [US1] Validate Part 11 README against plan.md checklist (frontmatter, Professional Tier, AIDD, paradigm shift signal)
- [ ] T025 [US1] Validate all Part 11 chapter readmes against plan.md checklist (frontmatter, Professional Tier, AIDD integration explicit)

**Checkpoint**: Part 11 (5 files) complete and independently reviewable. "Agents as workloads" paradigm established.

---

## Phase 4: User Story 2 - Part 12 README + Chapters 54-58 (Priority: P2)

**Goal**: Create Part 12 (Distributed Agent Runtime) overview and 5 chapter readmes establishing "agents as primitives" paradigm

**Independent Test**: Part 12 README exists with introduction acknowledging paradigm shift from Part 11, "What You'll Learn" section (6-8 outcomes), and "What's Next" section. All 5 chapter readmes exist with proper frontmatter and Professional Tier language emphasizing agent autonomy.

### Implementation for User Story 2 (Part 12)

- [ ] T026 [US2] Create Part 12 README.md introduction (4 paragraphs: Part 11 recap, paradigm shift explanation, chapter preview, AIDD emphasis) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/README.md
- [ ] T027 [US2] Write Part 12 "What You'll Learn" section (6-8 outcomes: Kafka, DAPR Actors, DAPR Workflows, Agent Homes, Multi-Agent Coordination with "agents as primitives" framing) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/README.md
- [ ] T028 [US2] Write Part 12 "What's Next" section linking to Part 13 (enterprise operations) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/README.md
- [ ] T029 [P] [US2] Create Chapter 54 (Kafka) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with event-driven agent communication patterns) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/54-kafka/readme.md
- [ ] T030 [P] [US2] Create Chapter 55 (DAPR Actors) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with agent identity and state persistence) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/55-dapr-actors/readme.md
- [ ] T031 [P] [US2] Create Chapter 56 (DAPR Workflows) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with durable execution patterns) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/56-dapr-workflows/readme.md
- [ ] T032 [P] [US2] Create Chapter 57 (Agent Homes) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with complete agent runtime integration) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/57-agent-homes/readme.md
- [ ] T033 [P] [US2] Create Chapter 58 (Multi-Agent Coordination) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with coordination patterns and agent discovery) in book-source/docs/12-Part-12-Distributed-Agent-Runtime/58-multi-agent-coordination/readme.md
- [ ] T034 [US2] Validate Part 12 README against plan.md checklist (frontmatter, Professional Tier, paradigm shift teaching, AIDD)
- [ ] T035 [US2] Validate all Part 12 chapter readmes against plan.md checklist (frontmatter, Professional Tier, "agents as primitives" framing)

**Checkpoint**: Part 12 (6 files) complete and independently reviewable. Paradigm shift from "agents as workloads" to "agents as primitives" explicitly taught.

---

## Phase 5: User Story 3 - Part 13 README + Chapters 59-67 (Priority: P3)

**Goal**: Create Part 13 (Agent Native Cloud & DACA) overview and 9 chapter readmes covering enterprise operations and DACA synthesis

**Independent Test**: Part 13 README exists with introduction building on Parts 11-12, "What You'll Learn" section (6-8 outcomes), and "What's Next" section. All 9 chapter readmes exist with proper frontmatter, Professional Tier language, and enterprise/production focus.

### Implementation for User Story 3 (Part 13)

- [ ] T036 [US3] Create Part 13 README.md introduction (4 paragraphs: Parts 11-12 foundation, enterprise operations focus, chapter preview, AIDD emphasis) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/README.md
- [ ] T037 [US3] Write Part 13 "What You'll Learn" section (6-8 outcomes: LLMOps, AgentOps, Agentic Mesh, orchestration, governance, DACA synthesis) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/README.md
- [ ] T038 [US3] Write Part 13 "What's Next" section (final section - link to book conclusion or next major part) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/README.md
- [ ] T039 [P] [US3] Create Chapter 59 (LLMOps) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with LLM observability and evaluation) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/59-llmops/readme.md
- [ ] T040 [P] [US3] Create Chapter 60 (AgentOps) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with agent evaluation and deployment pipelines) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/60-agentops/readme.md
- [ ] T041 [P] [US3] Create Chapter 61 (Agentic Mesh) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with mesh architecture and service discovery) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/61-agentic-mesh/readme.md
- [ ] T042 [P] [US3] Create Chapter 62 (Multi-Agent Orchestration) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with orchestration at scale) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/62-multi-agent-orchestration/readme.md
- [ ] T043 [P] [US3] Create Chapter 63 (Agent Scaling) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with scaling agent societies) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/63-agent-scaling/readme.md
- [ ] T044 [P] [US3] Create Chapter 64 (Cost Optimization) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with budget management and cost tracking) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/64-cost-optimization/readme.md
- [ ] T045 [P] [US3] Create Chapter 65 (Compliance & Governance) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with compliance and audit trails) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/65-compliance-governance/readme.md
- [ ] T046 [P] [US3] Create Chapter 66 (Model Governance) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes with model lifecycle and governance) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/66-model-governance/readme.md
- [ ] T047 [P] [US3] Create Chapter 67 (DACA Synthesis) readme with frontmatter, intro (4 paragraphs), "What You'll Learn" (6-8 outcomes synthesizing all patterns, case studies, production best practices) in book-source/docs/13-Part-13-Agent-Native-Cloud-DACA/67-daca-synthesis/readme.md
- [ ] T048 [US3] Validate Part 13 README against plan.md checklist (frontmatter, Professional Tier, enterprise focus, AIDD)
- [ ] T049 [US3] Validate all Part 13 chapter readmes against plan.md checklist (frontmatter, Professional Tier, production readiness focus)

**Checkpoint**: Part 13 (10 files) complete and independently reviewable. Enterprise operations and DACA synthesis established.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and improvements across all 21 files

- [ ] T050 [P] Validate all 3 Part READMEs follow consistent structure (intro, "What You'll Learn", "What's Next")
- [ ] T051 [P] Validate all 18 Chapter readmes follow consistent structure (intro, "What You'll Learn")
- [ ] T052 Verify paradigm shift teaching is explicit in Part 11 README "What's Next" section
- [ ] T053 Verify Professional Tier language used consistently (no scaffolding, business context, production concerns)
- [ ] T054 Verify AIDD methodology integration is explicit in all READMEs (spec → AI generates → validate pattern)
- [ ] T055 [P] Cross-check chapter titles and numbers against specs/book/chapter-index.md (official naming)
- [ ] T056 [P] Validate frontmatter metadata (sidebar_position, title) for all 21 files
- [ ] T057 Run Docusaurus build test to verify all files render correctly
- [ ] T058 Spell check and grammar review across all 21 files
- [ ] T059 Validate constitution alignment (spec-first, validation-first principles) across all content
- [ ] T060 Create pull request with all 21 files for final review

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all README creation
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (Part 11): Can start after Foundational - RECOMMENDED FIRST (MVP)
  - User Story 2 (Part 12): Can start after Foundational - Independent but builds on Part 11 conceptually
  - User Story 3 (Part 13): Can start after Foundational - Independent but builds on Parts 11-12 conceptually
- **Polish (Phase 6)**: Depends on all 3 user stories being complete

### User Story Dependencies

- **User Story 1 (Part 11 - P1)**: No dependencies on other stories - Can start after Foundational
- **User Story 2 (Part 12 - P2)**: No technical dependencies on US1, but conceptually builds on "agents as workloads" to teach paradigm shift
- **User Story 3 (Part 13 - P3)**: No technical dependencies on US1/US2, but conceptually requires both paradigms as foundation

**Note**: While user stories are technically independent (different files), they have **conceptual dependencies** (Part 12 teaches paradigm shift FROM Part 11 concepts). Recommended sequential implementation for pedagogical coherence, but parallel implementation is possible if authors coordinate on paradigm shift framing.

### Within Each User Story

- Part README before Chapter readmes (provides context)
- Chapter readmes within a Part can be created in parallel (marked [P])
- Validation tasks after all content creation

### Parallel Opportunities

- All Setup directory creation tasks (T005-T010) can run in parallel
- All Foundational research tasks (T011-T015) can run in parallel
- Within User Story 1: All 4 chapter readmes (T020-T023) can be created in parallel
- Within User Story 2: All 5 chapter readmes (T029-T033) can be created in parallel
- Within User Story 3: All 9 chapter readmes (T039-T047) can be created in parallel
- Polish validation tasks (T050-T051, T055-T056, T058) can run in parallel

---

## Parallel Example: User Story 1 (Part 11)

```bash
# After T019 completes (Part 11 README done), launch all 4 chapter readmes together:

Task T020: "Create Chapter 50 (Docker Fundamentals) readme in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/50-docker-fundamentals/readme.md"

Task T021: "Create Chapter 51 (Kubernetes Basics) readme in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/51-kubernetes-basics/readme.md"

Task T022: "Create Chapter 52 (DAPR Core) readme in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/52-dapr-core/readme.md"

Task T023: "Create Chapter 53 (Production Kubernetes) readme in book-source/docs/11-Part-11-Cloud-Native-Infrastructure/53-production-kubernetes/readme.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T016) - CRITICAL
3. Complete Phase 3: User Story 1 / Part 11 (T017-T025)
4. **STOP and VALIDATE**: Review Part 11 README and all 4 chapter readmes independently
5. Merge to main if ready (5 files = minimal viable documentation)

### Incremental Delivery

1. Complete Setup + Foundational → Research complete, structure ready
2. Add Part 11 (US1) → Review independently → Merge (MVP = 5 files)
3. Add Part 12 (US2) → Review independently → Merge (11 files total)
4. Add Part 13 (US3) → Review independently → Merge (21 files total)
5. Polish → Final validation → Complete feature

### Parallel Team Strategy

With multiple content authors:

1. Team completes Setup + Foundational together (research phase)
2. Once Foundational is done:
   - Author A: Part 11 README + Chapters 50-53
   - Author B: Part 12 README + Chapters 54-58
   - Author C: Part 13 README + Chapters 59-67
3. Coordinate on paradigm shift framing between Authors A and B
4. Cross-review each other's work for consistency

---

## Policy Note for Content Authors

**Lesson Structure Policy**: Within chapters (when lessons are created later), each lesson MUST end with a single final section titled **"Try With AI"** (no "Key Takeaways" or "What's Next" sections).

- **Before AI tools are taught** (Parts 1-3): Use ChatGPT web in that section
- **After tool onboarding** (Parts 4+): Instruct learners to use their preferred AI companion tool (e.g., Gemini CLI, Claude CLI), optionally providing both CLI and web variants

This policy applies to lesson content, NOT to these overview READMEs. Part READMEs include "What's Next" sections as navigational aids between major book sections.

---

## Notes

- **Total files to create**: 21 (3 Part READMEs + 18 Chapter readmes)
- **File naming convention**: Part READMEs use uppercase `README.md`, Chapter readmes use lowercase `readme.md`
- **[P] tasks**: Different files, can be created in parallel by multiple authors or AI agents
- **[Story] labels**: Map tasks to user stories for traceability (US1=Part 11, US2=Part 12, US3=Part 13)
- **Professional Tier enforcement**: All content must use Professional Tier language (no scaffolding, business context, production concerns)
- **Paradigm shift teaching**: Part 11 → Part 12 transition explicitly teaches "agents as workloads" → "agents as primitives"
- **AIDD integration**: Every README demonstrates spec → AI generates → validate workflow
- **Constitution alignment**: All content reinforces evals-first, spec-first, validation-first principles
- Commit after each Part completion (US1, US2, US3) for incremental delivery
- Stop at any checkpoint to validate independently before proceeding
