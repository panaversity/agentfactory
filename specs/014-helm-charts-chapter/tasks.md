# Tasks: Chapter 51 — Helm Charts for AI Services

**Input**: Design documents from `/specs/014-helm-charts-chapter/`
**Prerequisites**: plan.md (11 lessons), spec.md (8 user stories)

**Organization**: Tasks are organized by lesson (educational content) rather than user stories, since this is chapter content implementation.

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (e.g., L01, L02, L03)
- Include exact file paths in descriptions

## Path Conventions

- Chapter root: `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/`
- Lessons: `01-advanced-go-templating.md`, `02-named-templates-helpers.md`, etc.
- Assets: `assets/` subdirectory for images/diagrams

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Chapter structure initialization and README update

- [x] T001 Update README.md at `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/README.md` with lesson outline, prerequisites from Chapter 50, and learning objectives
- [x] T002 [P] Create chapter assets directory at `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/assets/`
- [x] T003 [P] Verify Chapter 50 Lesson 20 exists at `apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/20-helm-charts-ai-agent-packaging.md` for differentiation reference

**Checkpoint**: Chapter structure ready for lesson implementation

---

## Phase 2: Foundation Phase — Lessons 1-3 (Templating Vocabulary)

**Goal**: Students can create sophisticated, maintainable templates

---

### Lesson 1: Advanced Go Templating (Layer 1)

**Goal**: Master Go template syntax for dynamic Kubernetes manifests
**Independent Test**: Student creates templates with conditionals, ranges, and pipelines verified via `helm template --debug`

- [x] T004 [L01] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/01-advanced-go-templating.md` with YAML frontmatter (sidebar_position: 1, chapter: 51, lesson: 1, duration_minutes: 50, proficiency_level: B1, teaching_stage: 1)
- [x] T005 [L01] Write introduction explaining Go templating vs static manifests with JavaScript template literal analogy
- [x] T006 [L01] Document 9 key concepts: template variables, pipelines, variable assignment, with blocks, range iteration, if/else, comparison operators, template functions, built-in variables
- [x] T007 [L01] Create Exercise 1.1: Deployment with dynamic replica count using `{{.Values.replicaCount}}`
- [x] T008 [L01] Create Exercise 1.2: Pipeline syntax with filters `{{ .Values.env | upper | quote }}`
- [x] T009 [L01] Create Exercise 1.3: Conditional Ingress (include only if `ingress.enabled: true`)
- [x] T010 [L01] Create Exercise 1.4: Range iteration over environment variables from values
- [x] T011 [L01] Create Exercise 1.5: Debug template error with `helm template --debug` (intentional syntax error)
- [x] T012 [L01] Write "Try With AI" section with Initial Request, Critical Evaluation, Constraint Teaching, Refinement, Final Check structure
- [x] T013 [L01] Validate lesson against constitution (no meta-commentary, no "Key Takeaways")

**Checkpoint**: Lesson 1 complete and validated

---

### Lesson 2: Named Templates and Helpers (Layer 1)

**Goal**: Create reusable template partials using define and include
**Independent Test**: Student creates `_helpers.tpl` with labels template, includes it across multiple resources

- [x] T014 [L02] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/02-named-templates-helpers.md` with appropriate frontmatter
- [x] T015 [L02] Write introduction explaining DRY principle for templates
- [x] T016 [L02] Document 7 key concepts: named templates with define, helper files, include function, template function (deprecated), scope rules, common patterns, naming conventions
- [x] T017 [L02] Create Exercise 2.1: Create `_helpers.tpl` with common labels template
- [x] T018 [L02] Create Exercise 2.2: Include labels helper in Deployment, Service, and ConfigMap
- [x] T019 [L02] Create Exercise 2.3: Create image pull secrets helper
- [x] T020 [L02] Create Exercise 2.4: Verify `include` works vs show why `template` would fail
- [x] T021 [L02] Create Exercise 2.5: Refactor 3 duplicate template sections into single include
- [x] T022 [L02] Write "Try With AI" section
- [x] T023 [L02] Validate lesson against constitution

**Checkpoint**: Lesson 2 complete and validated

---

### Lesson 3: Values Deep Dive (Layer 1)

**Goal**: Master values hierarchy, schema validation, and multi-environment configuration
**Independent Test**: Student creates values-dev.yaml, values-staging.yaml, values-prod.yaml with schema validation

- [x] T024 [L03] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/03-values-deep-dive.md` with appropriate frontmatter
- [x] T025 [L03] Write introduction explaining values as configuration parameters
- [x] T026 [L03] Document 8 key concepts: values hierarchy, values.yaml structure, environment-specific files, command-line overrides, global values, schema validation, nested values, values.schema.json
- [x] T027 [L03] Create Exercise 3.1: Create values.yaml with realistic defaults
- [x] T028 [L03] Create Exercise 3.2: Create values-dev.yaml, values-staging.yaml, values-prod.yaml
- [x] T029 [L03] Create Exercise 3.3: Verify override precedence with `helm template -f values-prod.yaml --set image.tag=v2.0`
- [x] T030 [L03] Create Exercise 3.4: Create values.schema.json with validation rules
- [x] T031 [L03] Create Exercise 3.5: Pass invalid value, verify schema catches error
- [x] T032 [L03] Write "Try With AI" section
- [x] T033 [L03] Validate lesson against constitution

**Checkpoint**: Foundation Phase complete — students can create sophisticated templates

---

## Phase 3: Application Phase — Lessons 4-6 (Real-World Patterns)

**Goal**: Students can architect multi-component deployments with CI/CD integration

---

### Lesson 4: Chart Dependencies (Layer 1)

**Goal**: Compose charts with community subcharts (PostgreSQL, Redis)
**Independent Test**: Student adds Bitnami PostgreSQL/Redis as dependencies, configures via parent values

- [x] T034 [L04] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/04-chart-dependencies.md` with appropriate frontmatter
- [x] T035 [L04] Write introduction explaining subcharts as reusable application packages
- [x] T036 [L04] Document 9 key concepts: Chart.yaml dependencies, subchart resolution, conditions, tags, import-values, helm dependency update, charts/ directory, version constraints, aliases
- [x] T037 [L04] Create Exercise 4.1: Add Bitnami PostgreSQL as dependency with version constraint
- [x] T038 [L04] Create Exercise 4.2: Add Redis dependency with condition (redis.enabled)
- [x] T039 [L04] Create Exercise 4.3: Configure PostgreSQL via parent values: `postgresql.auth.database: mydb`
- [x] T040 [L04] Create Exercise 4.4: Use import-values to expose PostgreSQL host
- [x] T041 [L04] Create Exercise 4.5: Run `helm dependency update`, verify charts/ populated
- [x] T042 [L04] Create Exercise 4.6: Create values-nodedb.yaml disabling Redis but enabling PostgreSQL
- [x] T043 [L04] Write "Try With AI" section
- [x] T044 [L04] Validate lesson against constitution

**Checkpoint**: Lesson 4 complete and validated

---

### Lesson 5: Helm Hooks and Lifecycle Management (Layer 1)

**Goal**: Implement zero-downtime deployments with pre/post hooks
**Independent Test**: Student creates pre-upgrade hook for database migrations

- [x] T045 [L05] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/05-helm-hooks-lifecycle.md` with appropriate frontmatter
- [x] T046 [L05] Write introduction explaining hooks as lifecycle-aware Jobs
- [x] T047 [L05] Document all 10 key concepts: 9 hook types, hook resources, weights, delete policies, success/failure semantics, annotations, execution phases, test hooks, rollback hooks, pod cleanup
- [x] T048 [L05] Create execution sequence diagram showing hook timing relative to deployment
- [x] T049 [L05] Create Exercise 5.1: Pre-install hook Job for DB schema initialization
- [x] T050 [L05] Create Exercise 5.2: Pre-upgrade hook for migrations with weight ordering
- [x] T051 [L05] Create Exercise 5.3: Post-upgrade hook for cache invalidation
- [x] T052 [L05] Create Exercise 5.4: Make hook fail intentionally, verify deployment blocks
- [x] T053 [L05] Create Exercise 5.5: Test hook-delete-policy: hook-succeeded cleanup
- [x] T054 [L05] Create Exercise 5.6: Use weights to ensure migration runs before rollback
- [x] T055 [L05] Create Exercise 5.7: Check hook execution with kubectl
- [x] T056 [L05] Write "Try With AI" section
- [x] T057 [L05] Validate lesson against constitution

**Checkpoint**: Lesson 5 complete and validated

---

### Lesson 6: Testing Your Charts (Layer 1)

**Goal**: Validate charts before production with lint, template, and test commands
**Independent Test**: Student writes and executes chart tests verifying connectivity

- [x] T058 [L06] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/06-testing-your-charts.md` with appropriate frontmatter
- [x] T059 [L06] Write introduction explaining testing as safety net before prod
- [x] T060 [L06] Document 8 key concepts: helm lint, structure validation, helm template, template --debug, test pods, test semantics, helm test command, unit testing frameworks
- [x] T061 [L06] Create Exercise 6.1: Create chart with intentional error, catch with helm lint
- [x] T062 [L06] Create Exercise 6.2: Fix error, verify helm lint passes
- [x] T063 [L06] Create Exercise 6.3: Use helm template --debug to inspect variable resolution
- [x] T064 [L06] Create Exercise 6.4: Create test pod that connects to service
- [x] T065 [L06] Create Exercise 6.5: Deploy chart, run helm test, verify pass
- [x] T066 [L06] Create Exercise 6.6: Make test pod fail, verify helm test reports failure
- [x] T067 [L06] Write "Try With AI" section
- [x] T068 [L06] Validate lesson against constitution

**Checkpoint**: Application Phase complete — students can architect multi-component deployments

---

## Phase 4: Distribution Phase — Lessons 7-8 (Enterprise Scaling)

**Goal**: Students can share charts across teams with organizational patterns

---

### Lesson 7: OCI Registries and Distribution (Layer 1)

**Goal**: Publish and consume charts via OCI-compliant registries
**Independent Test**: Student pushes chart to Docker Hub, pulls and installs from OCI URL

- [x] T069 [L07] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/07-oci-registries-distribution.md` with appropriate frontmatter
- [x] T070 [L07] Write introduction explaining OCI as unified distribution platform
- [x] T071 [L07] Document 9 key concepts: OCI basics, helm package, helm push, helm pull, OCI URL format, registry auth, versioning strategy, dependency URLs, access control
- [x] T072 [L07] Create Exercise 7.1: Package chart with helm package
- [x] T073 [L07] Create Exercise 7.2: Create Docker Hub account (public registry for exercise)
- [x] T074 [L07] Create Exercise 7.3: Push chart to oci://docker.io/username/agent-chart
- [x] T075 [L07] Create Exercise 7.4: Pull chart from registry
- [x] T076 [L07] Create Exercise 7.5: Install directly from OCI URL
- [x] T077 [L07] Create Exercise 7.6: Add chart as dependency using OCI URL in Chart.yaml
- [x] T078 [L07] Write "Try With AI" section
- [x] T079 [L07] Validate lesson against constitution

**Checkpoint**: Lesson 7 complete and validated

---

### Lesson 8: Library Charts and Organizational Standardization (Layer 1)

**Goal**: Create library charts that enforce organizational patterns
**Independent Test**: Student creates library chart, uses it as dependency in application chart

- [x] T080 [L08] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/08-library-charts-standardization.md` with appropriate frontmatter
- [x] T081 [L08] Write introduction explaining library charts for organizational consistency
- [x] T082 [L08] Document 8 key concepts: library chart type, Chart.yaml type: library, cannot install directly, common templates, consuming library charts, override patterns, enterprise use cases, template inheritance
- [x] T083 [L08] Create Exercise 8.1: Create library chart with type: library
- [x] T084 [L08] Create Exercise 8.2: Verify helm install refuses (expected behavior)
- [x] T085 [L08] Create Exercise 8.3: Create common labels template in library
- [x] T086 [L08] Create Exercise 8.4: Create application chart with library dependency
- [x] T087 [L08] Create Exercise 8.5: Use include to pull labels from library
- [x] T088 [L08] Create Exercise 8.6: Override library defaults in application chart
- [x] T089 [L08] Write "Try With AI" section
- [x] T090 [L08] Validate lesson against constitution

**Checkpoint**: Distribution Phase complete — students can share charts across teams

---

## Phase 5: Synthesis Phase — Lessons 9-11 (Intelligence & Mastery)

**Goal**: Students orchestrate complete AI deployment strategies with reusable intelligence

---

### Lesson 9: AI-Assisted Chart Development (Layer 2)

**Goal**: Collaborate with AI to create sophisticated charts using Three Roles (invisible framework)
**Independent Test**: Student completes AI collaboration cycle producing refined chart component

**CRITICAL**: Three Roles framework must be INVISIBLE — no role labels, no meta-commentary

- [x] T091 [L09] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/09-ai-assisted-chart-development.md` with Layer 2 frontmatter
- [x] T092 [L09] Write introduction explaining AI as collaboration partner (NO framework labels)
- [x] T093 [L09] Document collaborative approach through action (NO "AI as Teacher/Student/Co-Worker" labels)
- [x] T094 [L09] Create Scenario 9.1: Complex ingress configuration with AI assistance
- [x] T095 [L09] Create Scenario 9.2: Multi-tier resource limits via AI dialogue
- [x] T096 [L09] Create Scenario 9.3: Custom hook design with AI iteration
- [x] T097 [L09] Create Scenario 9.4: Chart optimization through AI feedback cycles
- [x] T098 [L09] Create Scenario 9.5: Debugging template issues with AI help
- [x] T099 [L09] Write reflection prompts that reveal Three Roles through experience (NOT exposition)
- [x] T100 [L09] Validate lesson against constitution meta-commentary prohibition with grep: `grep -i "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows"`

**Checkpoint**: Lesson 9 complete — Layer 2 collaboration validated

---

### Lesson 10: Capstone — Production AI Agent Chart (Layer 4)

**Goal**: Create complete production-ready Helm chart for Part 6 AI agent
**Independent Test**: Single helm install deploys agent + PostgreSQL + Redis + migrations + tests

- [x] T101 [L10] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/10-capstone-production-agent-chart.md` with Layer 4 frontmatter
- [x] T102 [L10] Write capstone specification (SPEC FIRST, before implementation)
- [x] T103 [L10] Define acceptance criteria: successful install, helm test passes, multi-env deployment
- [x] T104 [L10] Create capstone Chart.yaml with PostgreSQL and Redis dependencies
- [x] T105 [L10] Create capstone values.yaml with production defaults
- [x] T106 [L10] Create capstone values-dev.yaml, values-staging.yaml, values-prod.yaml
- [x] T107 [L10] Create capstone templates: deployment.yaml, service.yaml, configmap.yaml, secrets.yaml
- [x] T108 [L10] Create capstone _helpers.tpl with standard labels and selectors
- [x] T109 [L10] Create capstone pre-upgrade hook for database migrations
- [x] T110 [L10] Create capstone test pod for connectivity verification
- [x] T111 [L10] Create capstone README.md documenting all configuration options
- [x] T112 [L10] Write validation section: helm lint, helm template, helm install, helm test
- [x] T113 [L10] Write "Try With AI" section for capstone refinement
- [x] T114 [L10] Validate capstone against all success criteria (SC-001 through SC-010)

**Checkpoint**: Capstone complete — production-ready chart deployable

---

### Lesson 11: Building a Helm Chart Skill (Layer 3)

**Goal**: Create reusable Helm Chart Skill using Persona + Questions + Principles format
**Independent Test**: Skill file created at `.claude/skills/engineering/helm-chart-architect/SKILL.md`

- [x] T115 [L11] Create lesson file `apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/11-building-helm-chart-skill.md` with Layer 3 frontmatter
- [x] T116 [L11] Write introduction explaining reusable intelligence (skills compound across projects)
- [x] T117 [L11] Document Persona + Questions + Principles (P+Q+P) skill structure
- [x] T118 [L11] Guide students through skill creation: define Persona (Helm Chart Architect)
- [x] T119 [L11] Guide students through Questions section (5-10 decision-forcing questions)
- [x] T120 [L11] Guide students through Principles section (8-12 guiding principles)
- [x] T121 [L11] Create sample skill file: `.claude/skills/helm-chart-architect/SKILL.md`
- [x] T122 [L11] Create skill validation exercise: invoke skill, verify chart architecture quality
- [x] T123 [L11] Write "Try With AI" section for skill testing
- [x] T124 [L11] Validate lesson against constitution

**Checkpoint**: Synthesis Phase complete — reusable Helm skill created

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and chapter-wide consistency

- [x] T125 [P] Run validation-auditor on all 11 lessons for constitution compliance
- [ ] T126 [P] Verify all code examples execute on Minikube 1.32+ with Helm 3.14+
- [ ] T127 [P] Create chapter summary asset (visual lesson progression diagram) in assets/
- [x] T128 Review README.md reflects final lesson structure
- [x] T129 Run grep validation for meta-commentary prohibition across all lessons
- [x] T130 Final review: differentiation from Chapter 50 Lesson 20 confirmed

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately
- **Foundation (Phase 2)**: Depends on Setup — Lessons 1-3 sequential (each builds on previous)
- **Application (Phase 3)**: Depends on Foundation — Lessons 4-6 can be parallel after Lesson 3
- **Distribution (Phase 4)**: Depends on Application — Lessons 7-8 can be parallel
- **Synthesis (Phase 5)**: Depends on Distribution — Lessons 9, 10, 11 sequential
- **Polish (Phase 6)**: Depends on all lessons complete

### Within Each Phase

- Tasks within a lesson must be completed in order (T004 before T005)
- Different lessons can be worked in parallel where indicated
- All lessons end with validation (final task per lesson)

### Parallel Opportunities

- Phase 1: T002, T003 can run parallel
- Phase 3: Lessons 4, 5, 6 can be developed in parallel (all L1)
- Phase 4: Lessons 7, 8 can be developed in parallel (all L1)
- Phase 6: T125, T126, T127 can run parallel

---

## Parallel Example: Application Phase

```bash
# After Foundation Phase complete, launch Application lessons in parallel:
Agent 1: "Implement Lesson 4: Chart Dependencies (T034-T044)"
Agent 2: "Implement Lesson 5: Helm Hooks (T045-T057)"
Agent 3: "Implement Lesson 6: Testing Charts (T058-T068)"
```

---

## Implementation Strategy

### MVP First (Lessons 1-3 + Capstone)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundation (Lessons 1-3)
3. Skip to Lesson 10: Capstone (validates core learning)
4. **STOP and VALIDATE**: Test capstone chart deploys successfully
5. Backfill Lessons 4-9, 11

### Incremental Delivery (Recommended)

1. Complete Setup → Foundation ready
2. Complete Lessons 1-3 → Templating mastery
3. Complete Lessons 4-6 → Application patterns
4. Complete Lessons 7-8 → Distribution ready
5. Complete Lessons 9-11 → Full synthesis
6. Polish phase → Chapter ready

### Quality Gates

| Gate | Validation |
|------|------------|
| After Lesson 3 | `helm template` renders all templates correctly |
| After Lesson 6 | `helm test` passes on deployed chart |
| After Lesson 7 | Chart pushed/pulled from OCI registry |
| After Lesson 10 | Capstone chart deploys complete stack |
| After Lesson 11 | Skill file validates in skill runner |
| Final | validation-auditor passes with 0 violations |

---

## Summary

**Total Tasks**: 130
**By Phase**:
- Setup: 3 tasks
- Foundation (L1-L3): 30 tasks
- Application (L4-L6): 35 tasks
- Distribution (L7-L8): 22 tasks
- Synthesis (L9-L11): 34 tasks
- Polish: 6 tasks

**Parallel Opportunities**: 15+ tasks can run in parallel
**Estimated Implementation**: 5-7 days for market-defining quality
**MVP Scope**: Lessons 1-3 + Lesson 10 (Capstone)
