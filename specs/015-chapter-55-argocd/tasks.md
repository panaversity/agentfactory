# Tasks: Chapter 55 - CI/CD Pipelines & GitOps with ArgoCD

**Input**: Design documents from `/specs/015-chapter-55-argocd/`
**Prerequisites**: plan.md (required), spec.md (required)

**Content Type**: Educational chapter with 18 lessons
**Output Directory**: `apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/`

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (e.g., L01, L02, L03)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Create chapter directory structure and README

- [x] T001 Create chapter directory at `apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/`
- [x] T002 Create README.md with chapter overview, learning objectives, and lesson structure per plan.md
- [x] T003 [P] Create `_category_.json` for Docusaurus sidebar configuration

**Checkpoint**: Chapter structure ready for lesson implementation

---

## Phase 2: Foundational (CI/CD Concepts - Lessons 1-4)

**Purpose**: Build mental models of CI/CD before ArgoCD specifics

### Lesson 1: CI/CD Concepts - The Automated Pipeline (30 min)

**Goal**: Students can diagram CI/CD stages and explain value proposition

**Independent Test**: Student draws pipeline with trigger → build → test → push → deploy

- [x] T004 [L01] Create `01-cicd-concepts-automated-pipeline.md` with:
  - Pipeline stages (trigger, build, test, push, deploy)
  - CI vs CD distinction (integration vs deployment)
  - Artifacts and their lifecycle
  - Quality gates and why they matter
  - Value proposition (automation, consistency, speed)
- [x] T005 [L01] Add architectural diagram showing pipeline flow
- [x] T006 [L01] Add "Try With AI" section for pipeline design exercise

### Lesson 2: GitHub Actions Fundamentals (45 min)

**Goal**: Students create basic GitHub Actions workflow

**Independent Test**: Working workflow that triggers on push and runs a job

- [x] T007 [P] [L02] Create `02-github-actions-fundamentals.md` with:
  - Workflow YAML structure (name, on, jobs, steps)
  - Triggers (push, pull_request, schedule, workflow_dispatch)
  - Jobs and steps execution model
  - Secrets and environment variables
  - Matrix builds for multi-version testing
- [x] T008 [L02] Add complete workflow example for FastAPI agent
- [x] T009 [L02] Add "Try With AI" section for workflow generation

### Lesson 3: Building Docker Images in CI (45 min)

**Goal**: Students build multi-platform Docker images in GitHub Actions

**Independent Test**: Workflow pushes image to registry with commit SHA tag

- [x] T010 [P] [L03] Create `03-building-docker-images-ci.md` with:
  - Docker buildx for multi-platform (amd64, arm64)
  - Registry authentication (Docker Hub, GHCR)
  - Image tagging strategies (SHA, semver, latest)
  - Build caching for faster builds
  - Build args and environment variables
- [x] T011 [L03] Add workflow example with buildx and registry push
- [x] T012 [L03] Add "Try With AI" section for Dockerfile optimization

### Lesson 4: Testing and Quality Gates (40 min)

**Goal**: Students implement test stages that block deployment on failure

**Independent Test**: Workflow fails fast when tests fail, doesn't push image

- [x] T013 [P] [L04] Create `04-testing-quality-gates.md` with:
  - Unit tests in CI (pytest, coverage)
  - Integration tests (service dependencies)
  - Quality gates (coverage thresholds, linting)
  - Fail-fast behavior and job dependencies
  - Test reporting and artifacts
- [x] T014 [L04] Add workflow with test matrix and coverage reporting
- [x] T015 [L04] Add "Try With AI" section for test configuration

**Checkpoint**: Students can create complete CI pipeline with GitHub Actions

---

## Phase 3: GitOps Foundation (Lesson 5)

**Purpose**: Establish GitOps mental model before ArgoCD

### Lesson 5: GitOps Principles - Git as Truth (35 min)

**Goal**: Students articulate declarative vs imperative and reconciliation loop

**Independent Test**: Student explains drift detection and why Git-as-truth matters

- [x] T016 [L05] Create `05-gitops-principles-git-as-truth.md` with:
  - Declarative vs imperative infrastructure
  - Git as single source of truth
  - Reconciliation loops (observe → diff → act)
  - Drift detection and correction
  - Benefits: auditability, rollback, collaboration
- [x] T017 [L05] Add diagram comparing kubectl apply vs GitOps
- [x] T018 [L05] Add "Try With AI" section for GitOps design discussion

**Checkpoint**: Students understand WHY GitOps before learning ArgoCD

---

## Phase 4: ArgoCD Core (Lessons 6-9)

**Purpose**: ArgoCD installation, Application CRD, sync fundamentals

### Lesson 6: ArgoCD Architecture & Installation (40 min)

**Goal**: Students install ArgoCD on Minikube and access UI

**Independent Test**: ArgoCD pods running, UI accessible, CLI authenticated

- [x] T019 [L06] Create `06-argocd-architecture-installation.md` with:
  - ArgoCD components (API Server, Repo Server, Controller, Redis, Dex)
  - CRDs introduced (Application, AppProject, ApplicationSet)
  - Helm installation on Minikube
  - UI access and initial admin password
  - CLI installation and authentication
- [x] T020 [L06] Add architecture diagram with component interactions
- [x] T021 [L06] Add "Try With AI" section for installation troubleshooting

### Lesson 7: Your First ArgoCD Application (50 min)

**Goal**: Students create Application that syncs Helm chart from Git

**Independent Test**: Application shows Synced and Healthy status

- [x] T022 [P] [L07] Create `07-your-first-argocd-application.md` with:
  - Application CRD structure (source, destination, syncPolicy)
  - Creating via UI, CLI, and declarative YAML
  - Source types (Helm, Kustomize, plain YAML)
  - Destination (cluster, namespace)
  - Sync status (OutOfSync, Synced, Unknown)
  - Health status (Healthy, Progressing, Degraded, Missing)
- [x] T023 [L07] Add complete Application YAML for Part 6 agent
- [x] T024 [L07] Add "Try With AI" section for Application creation

### Lesson 8: Sync Strategies and Policies (40 min)

**Goal**: Students configure auto-sync with appropriate policies

**Independent Test**: Application auto-syncs on Git push with prune enabled

- [x] T025 [P] [L08] Create `08-sync-strategies-policies.md` with:
  - Manual sync (when and why)
  - Auto-sync (automated: enabled: true)
  - Auto-prune (removing deleted resources)
  - Self-heal (reverting manual changes)
  - Replace vs Apply (force updates)
  - Sync windows (time-based restrictions)
- [x] T026 [L08] Add policy comparison table with use cases
- [x] T027 [L08] Add "Try With AI" section for policy selection

### Lesson 9: Sync Waves and Resource Hooks (50 min)

**Goal**: Students implement PreSync hook for database migration

**Independent Test**: PreSync Job completes before Deployment starts

- [x] T028 [P] [L09] Create `09-sync-waves-resource-hooks.md` with:
  - Sync wave ordering (argocd.argoproj.io/sync-wave annotation)
  - Hook types (PreSync, Sync, PostSync, SyncFail, PostDelete)
  - Hook deletion policies (HookSucceeded, HookFailed, BeforeHookCreation)
  - Common patterns (migrations, notifications, cleanup)
  - Troubleshooting failed hooks
- [x] T029 [L09] Add migration Job example with PreSync hook
- [x] T030 [L09] Add "Try With AI" section for hook design

**Checkpoint**: Students can deploy applications with ArgoCD and control sync behavior

---

## Phase 5: ArgoCD Advanced (Lessons 10-15)

**Purpose**: Production patterns for multi-env, RBAC, secrets, multi-cluster

### Lesson 10: ApplicationSets - Scaling Deployments (45 min)

**Goal**: Students create ApplicationSet for 3 environments

**Independent Test**: One ApplicationSet generates dev, staging, prod Applications

- [x] T031 [P] [L10] Create `10-applicationsets-scaling-deployments.md` with:
  - ApplicationSet CRD structure
  - List generator (explicit environments)
  - Cluster generator (multi-cluster)
  - Matrix generator (combinations)
  - Git generator (directory/file-based)
  - Template overrides per environment
- [x] T032 [L10] Add ApplicationSet example for Part 6 agent (3 envs)
- [x] T033 [L10] Add "Try With AI" section for generator selection

### Lesson 11: ArgoCD Projects and RBAC (40 min)

**Goal**: Students configure project with resource restrictions

**Independent Test**: Project limits allowed clusters, namespaces, and sources

- [x] T034 [P] [L11] Create `11-argocd-projects-rbac.md` with:
  - AppProject CRD structure
  - Source repository restrictions
  - Destination cluster/namespace restrictions
  - Resource whitelists/blacklists
  - Role-based access (p, g policies)
  - Project scoped repositories
  - v3.x fine-grained RBAC enhancements
- [x] T035 [L11] Add project example for multi-tenant setup
- [x] T036 [L11] Add "Try With AI" section for RBAC policy design

### Lesson 12: Health Status and Notifications (35 min)

**Goal**: Students configure custom health check and notification

**Independent Test**: Slack notification fires on sync failure

- [x] T037 [P] [L12] Create `12-health-status-notifications.md` with:
  - Built-in health assessments
  - Custom health checks (Lua scripts)
  - Degraded and Unknown states
  - Notification triggers (on sync, on health change)
  - Notification services (Slack, webhook, email)
  - argocd-notifications-cm configuration
- [x] T038 [L12] Add webhook notification example
- [x] T039 [L12] Add "Try With AI" section for health check design

### Lesson 13: Progressive Delivery Overview (40 min)

**Goal**: Students understand canary/blue-green concepts with Argo Rollouts

**Independent Test**: Student explains when to use canary vs blue-green

- [x] T040 [P] [L13] Create `13-progressive-delivery-overview.md` with:
  - Why progressive delivery for AI agents
  - Canary deployments (percentage traffic shift)
  - Blue-green deployments (instant switch)
  - Argo Rollouts introduction
  - Rollout CRD basics (strategy, steps, analysis)
  - Integration with ArgoCD
- [x] T041 [L13] Add Rollout example with canary steps
- [x] T042 [L13] Add "Try With AI" section for strategy selection

### Lesson 14: Secrets Management for GitOps (45 min)

**Goal**: Students implement secrets without storing in Git

**Independent Test**: Secret available in cluster, not visible in Git repo

- [x] T043 [P] [L14] Create `14-secrets-management-gitops.md` with:
  - Problem: secrets in Git are insecure
  - Sealed Secrets (Bitnami) approach
  - External Secrets Operator pattern
  - Vault integration overview
  - ArgoCD Vault Plugin
  - Best practices for API keys (LLM providers)
- [x] T044 [L14] Add Sealed Secrets example for API key
- [x] T045 [L14] Add "Try With AI" section for secrets strategy

### Lesson 15: Multi-Cluster Deployments (40 min)

**Goal**: Students register external cluster and deploy with ApplicationSet

**Independent Test**: ApplicationSet deploys to 2+ clusters

- [x] T046 [P] [L15] Create `15-multi-cluster-deployments.md` with:
  - Hub-spoke architecture
  - Cluster registration (argocd cluster add)
  - Cluster secrets and authentication
  - ApplicationSet with Cluster generator
  - Cross-cluster networking considerations
  - Disaster recovery patterns
- [x] T047 [L15] Add multi-cluster ApplicationSet example
- [x] T048 [L15] Add "Try With AI" section for cluster architecture

**Checkpoint**: Students can implement production ArgoCD patterns

---

## Phase 6: AI Collaboration (Lesson 16 - Layer 2)

**Purpose**: Three Roles pattern for manifest generation (framework invisible)

### Lesson 16: AI-Assisted GitOps Workflows (60 min)

**Goal**: Students collaborate with Claude to generate and refine ArgoCD manifests

**Independent Test**: Student iterates with AI to produce working ApplicationSet

- [x] T049 [L16] Create `16-ai-assisted-gitops-workflows.md` with:
  - When to use AI for GitOps (complex configurations)
  - Prompt patterns for manifest generation
  - Critical evaluation of AI output
  - Constraint teaching (telling AI your environment)
  - Iterative refinement cycle
  - Validation and testing
- [x] T050 [L16] Add 5-part "Try With AI" template from plan.md (invisible Three Roles)
- [x] T051 [L16] Validate: No meta-commentary (grep for forbidden patterns)

**Validation Command**:
```bash
grep -i "What to notice\|AI.*teach\|AI.*learn\|Three Roles\|AI as\|What to expect" \
  apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/16-*.md
# Expected: ZERO matches
```

**Checkpoint**: Students experience AI collaboration without exposed framework

---

## Phase 7: Capstone Integration (Lesson 17 - Layer 4)

**Purpose**: Spec-driven end-to-end CI/CD pipeline for Part 6 agent

### Lesson 17: Capstone - End-to-End Agent Pipeline (90 min)

**Goal**: Complete CI/CD pipeline from code push to running agent

**Independent Test**: Push code → GitHub Actions → ArgoCD sync → agent accessible

- [x] T052 [L17] Create `17-capstone-end-to-end-agent-pipeline.md` with:
  - Specification-first approach (write spec BEFORE implementation)
  - Pipeline specification template
  - GitHub Actions workflow (build, test, push)
  - GitOps repository structure
  - ArgoCD Application for Part 6 agent
  - Image update automation (image tag in GitOps repo)
  - End-to-end validation steps
  - Rollback demonstration (git revert → ArgoCD syncs old version)
- [x] T053 [L17] Add complete GitHub Actions workflow file
- [x] T054 [L17] Add complete ArgoCD Application YAML
- [x] T055 [L17] Add validation checklist for capstone completion

**Checkpoint**: Students demonstrate integrated CI/CD understanding

---

## Phase 8: Intelligence Design (Lesson 18 - Layer 3)

**Purpose**: Create reusable GitOps Deployment Skill

### Lesson 18: Building the GitOps Deployment Skill (120 min)

**Goal**: Students create reusable skill with Persona + Questions + Principles

**Independent Test**: Another student uses skill to deploy different application

- [x] T056 [L18] Create `18-building-gitops-deployment-skill.md` with:
  - Skill structure (Persona + Questions + Principles)
  - Persona: Think like a DevOps architect
  - Contextual Questions:
    - Deployment context (single/multi-cluster, environments)
    - GitOps architecture (repo structure, sync strategy)
    - Production patterns (secrets, progressive delivery)
  - Decision Principles:
    - Declarative > Imperative
    - Progressive Complexity (start simple, add patterns)
    - Git Audit Trail (all changes through Git)
    - Sync Strategy Selection (when auto vs manual)
    - Secrets Never in Git
  - Skill validation criteria
- [x] T057 [L18] Add complete skill file template
- [x] T058 [L18] Add "Try With AI" section for skill refinement

**Checkpoint**: Students have reusable intelligence for future GitOps work

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Chapter-level quality and validation

- [x] T059 [P] Update README.md with final lesson count and time estimates
- [ ] T060 [P] Validate all code examples in Minikube (attach execution logs)
- [x] T061 [P] Run constitutional compliance check (all 7 principles)
- [x] T062 [P] Run meta-commentary grep on all lessons (forbidden patterns)
- [x] T063 [P] Verify cognitive load per lesson (≤7 new concepts)
- [ ] T064 Create lesson summaries using summary-generator skill
- [x] T065 Final review: All 14 success evals mapped to lessons

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) ──────────────────────────────────────┐
                                                       │
Phase 2 (CI Concepts L1-4) ───────────────────────────┤
                                                       │
Phase 3 (GitOps L5) ──────────────────────────────────┤
                                                       ▼
Phase 4 (ArgoCD Core L6-9) ───► Phase 5 (ArgoCD Adv L10-15)
                                           │
                                           ▼
                            Phase 6 (AI Collab L16)
                                           │
                                           ▼
                            Phase 7 (Capstone L17)
                                           │
                                           ▼
                            Phase 8 (Skill L18)
                                           │
                                           ▼
                            Phase 9 (Polish)
```

### Parallel Opportunities

**Within Phase 2** (CI Concepts):
```
L01 ──► L02 ──► L03 ──► L04  (sequential, concepts build)
```

**Within Phase 4** (ArgoCD Core):
```
L06 ──► L07 ──► L08 ──► L09  (sequential, installation first)
```

**Within Phase 5** (ArgoCD Advanced):
```
L10, L11, L12, L13, L14, L15  (can run in parallel - different topics)
```

**Parallel Lesson Implementation**:
```bash
# These lessons can be written simultaneously (different files, no concept dependencies):
Task: "L10: ApplicationSets"
Task: "L11: Projects & RBAC"
Task: "L12: Health & Notifications"
Task: "L13: Progressive Delivery"
Task: "L14: Secrets Management"
Task: "L15: Multi-Cluster"
```

---

## Implementation Strategy

### MVP First (Lessons 1-7 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: CI Concepts (L1-4)
3. Complete Phase 3: GitOps (L5)
4. Complete Phase 4: ArgoCD Core (L6-7)
5. **STOP and VALIDATE**: Student can create CI pipeline + first ArgoCD Application
6. Deploy/demo if ready

### Full Chapter Delivery

1. Complete Phases 1-4 → Students have CI + basic ArgoCD
2. Complete Phase 5 → Production patterns
3. Complete Phase 6 → AI collaboration experience
4. Complete Phase 7 → End-to-end integration
5. Complete Phase 8 → Reusable intelligence
6. Complete Phase 9 → Quality validation

---

## Summary

**Total Tasks**: 65
**Tasks per Phase**:
- Phase 1 (Setup): 3 tasks
- Phase 2 (CI L1-4): 12 tasks
- Phase 3 (GitOps L5): 3 tasks
- Phase 4 (ArgoCD Core L6-9): 12 tasks
- Phase 5 (ArgoCD Advanced L10-15): 18 tasks
- Phase 6 (AI Collab L16): 3 tasks
- Phase 7 (Capstone L17): 4 tasks
- Phase 8 (Skill L18): 3 tasks
- Phase 9 (Polish): 7 tasks

**Parallel Opportunities**:
- Phase 5 lessons (L10-15) can run in parallel (6 simultaneous)
- Polish tasks can run in parallel (5 simultaneous)

**MVP Scope**: Phases 1-4 (Lessons 1-7) = 30 tasks

**Output**: 18 lesson markdown files + README + summaries
