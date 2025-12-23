# Feature Specification: Chapter 55 - CI/CD Pipelines & GitOps with ArgoCD

**Feature Branch**: `015-chapter-55-argocd`
**Created**: 2025-12-23
**Status**: Draft
**Input**: User description: "Chapter 55: CI/CD Pipelines and GitOps with ArgoCD - Teaching automated deployment pipelines with GitHub Actions and GitOps with ArgoCD for AI agent services"

---

## Assumed Knowledge

**What students know BEFORE this chapter** (from Parts 1-6 and Chapters 49-51):

- **Docker fundamentals** (Chapter 49): Container images, Dockerfiles, multi-stage builds, networking, volumes, Docker Compose
- **Kubernetes mastery** (Chapter 50): Pods, Deployments, Services, Ingress, ConfigMaps, Secrets, RBAC, HPA, StatefulSets, health probes
- **Helm charts** (Chapter 51): Chart structure, templating, values files, releases, dependencies
- **FastAPI agent service** (Part 6): A working AI agent with MCP integration, ChatKit, and database persistence
- **Git and GitHub** (Chapter 9): Branching, commits, pull requests, GitHub repositories
- **Specification-driven development** (Part 4): Writing specifications before implementation, SDD-RI methodology

**What this chapter must explain from scratch**:

- CI/CD pipeline concepts (continuous integration vs continuous deployment)
- GitHub Actions workflow syntax and architecture
- GitOps principles (declarative deployments, Git as source of truth)
- ArgoCD architecture (Application Controller, CRDs, reconciliation loop)
- ArgoCD Application and ApplicationSet resources
- Sync strategies, hooks, and progressive delivery patterns
- Multi-cluster deployment patterns

**Proficiency Level**: B1-B2 (Intermediate to Upper-Intermediate)

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learns CI Pipeline Fundamentals (Priority: P1)

A student who has completed Chapter 51 (Helm) wants to understand how code changes automatically flow from commit to container image. They need to build mental models of CI/CD stages before using any tools.

**Why this priority**: Without understanding CI/CD concepts, students cannot appreciate why automation matters or evaluate AI-generated pipeline configurations. This is the Layer 1 foundation.

**Independent Test**: Can be fully tested by student explaining the build-test-push flow in their own words and identifying what triggers each stage.

**Acceptance Scenarios**:

1. **Given** a student with no CI/CD knowledge, **When** they complete Lesson 1, **Then** they can diagram the stages of a typical CI pipeline (trigger → build → test → push → deploy)
2. **Given** a student who understands the concepts, **When** they see a GitHub Actions workflow file, **Then** they can identify which jobs correspond to which CI stages
3. **Given** a broken pipeline, **When** a student reads the error output, **Then** they can identify which stage failed and suggest a fix direction

---

### User Story 2 - Student Implements GitHub Actions Pipeline (Priority: P1)

A student needs to create a GitHub Actions workflow that builds their Docker image, runs tests, and pushes to a container registry on every commit to main.

**Why this priority**: Hands-on GitHub Actions is the first half of the chapter's core value proposition. Students must be able to automate builds before learning GitOps.

**Independent Test**: Can be tested by pushing a code change and observing the automated build/test/push complete successfully in GitHub's Actions tab.

**Acceptance Scenarios**:

1. **Given** a student with a GitHub repository containing their Part 6 agent, **When** they push to main, **Then** a workflow runs that builds and tests the code
2. **Given** a workflow with test failures, **When** tests fail, **Then** the workflow stops before pushing the image and reports the failure clearly
3. **Given** tests pass, **When** the build completes, **Then** the Docker image is pushed to a registry with a version tag matching the commit SHA
4. **Given** a student needs secrets (registry credentials), **When** they configure GitHub Secrets, **Then** the workflow can access them securely without exposing values in logs

---

### User Story 3 - Student Understands GitOps Principles (Priority: P1)

A student needs to understand why Git should be the source of truth for deployments and how declarative infrastructure differs from imperative scripts.

**Why this priority**: GitOps principles are the philosophical foundation of ArgoCD. Without this mental model, students will use ArgoCD as "another deployment tool" rather than understanding its reconciliation power.

**Independent Test**: Can be tested by student explaining the difference between "kubectl apply" and GitOps reconciliation, and why drift detection matters.

**Acceptance Scenarios**:

1. **Given** a student who deploys with kubectl apply, **When** they learn GitOps principles, **Then** they can articulate why Git-as-truth provides auditability, rollback, and collaboration benefits
2. **Given** a scenario where someone manually edits a Deployment in the cluster, **When** a student understands drift, **Then** they can explain how ArgoCD would detect and optionally correct this
3. **Given** the concept of reconciliation loops, **When** a student completes the GitOps lesson, **Then** they can describe the observe-diff-act cycle that ArgoCD performs

---

### User Story 4 - Student Installs and Configures ArgoCD (Priority: P2)

A student needs to install ArgoCD on their Minikube cluster and access the UI/CLI to understand the platform before creating applications.

**Why this priority**: Installation is prerequisite for all ArgoCD hands-on work. Priority P2 because conceptual foundation (P1 stories) must come first.

**Independent Test**: Can be tested by successfully logging into ArgoCD UI and listing clusters with the CLI.

**Acceptance Scenarios**:

1. **Given** a student with a running Minikube cluster, **When** they follow installation steps, **Then** ArgoCD pods are running in the argocd namespace
2. **Given** ArgoCD is installed, **When** the student accesses the UI, **Then** they can log in with the initial admin password
3. **Given** the argocd CLI is installed, **When** the student runs `argocd cluster list`, **Then** they see the in-cluster Kubernetes API server

---

### User Story 5 - Student Creates Their First ArgoCD Application (Priority: P2)

A student needs to create an ArgoCD Application that syncs their Helm chart from a Git repository to the Kubernetes cluster.

**Why this priority**: The Application CRD is the core abstraction of ArgoCD. Students must master this before advanced features.

**Independent Test**: Can be tested by observing the Application sync and seeing the agent pods running in the target namespace.

**Acceptance Scenarios**:

1. **Given** a Git repository with a Helm chart, **When** a student creates an Application resource, **Then** ArgoCD detects the chart and shows it as OutOfSync
2. **Given** an OutOfSync application, **When** the student triggers a sync, **Then** ArgoCD applies the manifests and the application becomes Synced and Healthy
3. **Given** a synced application, **When** the student changes a value in Git, **Then** ArgoCD detects the drift and shows OutOfSync status
4. **Given** the student enables auto-sync, **When** they push a change to Git, **Then** ArgoCD automatically applies the update without manual intervention

---

### User Story 6 - Student Masters Sync Strategies and Hooks (Priority: P2)

A student needs to understand how to control the order of resource application using sync waves and hooks (PreSync, Sync, PostSync).

**Why this priority**: Production deployments require careful sequencing (e.g., database migrations before app rollout). This is essential for reliable AI agent deployments.

**Independent Test**: Can be tested by deploying an application with a PreSync Job that must complete before the main Deployment starts.

**Acceptance Scenarios**:

1. **Given** an application with multiple resources, **When** the student applies sync waves, **Then** ArgoCD applies resources in wave order (lower numbers first)
2. **Given** a database schema migration requirement, **When** the student creates a PreSync hook Job, **Then** the migration runs and completes before the application Deployment starts
3. **Given** a PostSync notification requirement, **When** the student creates a PostSync hook, **Then** it executes only after all sync wave resources are healthy
4. **Given** a PreSync hook that fails, **When** ArgoCD attempts sync, **Then** the main application resources are NOT applied and the sync shows Failed status

---

### User Story 7 - Student Deploys Multiple Applications with ApplicationSets (Priority: P3)

A student needs to deploy their agent to multiple environments (dev, staging, prod) or multiple clusters using a single ApplicationSet definition.

**Why this priority**: ApplicationSets are the production pattern for multi-environment and multi-cluster deployments. Priority P3 because it builds on Application mastery.

**Independent Test**: Can be tested by creating one ApplicationSet that generates three Applications (dev, staging, prod) with different configurations.

**Acceptance Scenarios**:

1. **Given** a student with multiple environments, **When** they create an ApplicationSet with a List generator, **Then** ArgoCD generates one Application per environment
2. **Given** an ApplicationSet targeting multiple clusters, **When** using the Cluster generator, **Then** ArgoCD creates Applications for each registered cluster
3. **Given** a matrix of environments and regions, **When** using the Matrix generator, **Then** ArgoCD creates Applications for every combination
4. **Given** a new environment is added to the generator list, **When** the ApplicationSet syncs, **Then** a new Application is automatically created without manual intervention

---

### User Story 8 - Student Implements Progressive Delivery (Priority: P3)

A student needs to deploy their AI agent using a canary or blue-green strategy with Argo Rollouts integration.

**Why this priority**: AI agents require careful rollouts because behavior changes can be subtle. Priority P3 because it's an advanced production pattern.

**Independent Test**: Can be tested by deploying a new agent version to 10% of traffic, validating behavior, then promoting to 100%.

**Acceptance Scenarios**:

1. **Given** a student using Argo Rollouts, **When** they deploy a new version, **Then** traffic shifts gradually according to the canary strategy steps
2. **Given** a canary analysis fails, **When** metrics indicate problems, **Then** the rollout automatically aborts and traffic returns to the stable version
3. **Given** a successful canary, **When** all steps pass, **Then** the new version becomes the stable version and old pods are scaled down

---

### User Story 9 - Student Uses AI to Generate and Debug GitOps Configurations (Priority: P2)

A student collaborates with Claude Code to generate ArgoCD Application manifests, troubleshoot sync failures, and optimize configurations.

**Why this priority**: Layer 2 (AI Collaboration) is core to AI-native learning. Students should experience the Three Roles pattern with GitOps tools.

**Independent Test**: Can be tested by student prompting Claude to generate an ApplicationSet, evaluating the output, providing corrections, and achieving a working configuration.

**Acceptance Scenarios**:

1. **Given** a student needs an ApplicationSet for 3 environments, **When** they prompt Claude with requirements, **Then** Claude generates a valid ApplicationSet manifest with proper generator configuration
2. **Given** an ArgoCD sync failure, **When** the student asks Claude for debugging help, **Then** Claude guides them to check resource status, events, and logs to identify the root cause
3. **Given** Claude's initial output has issues, **When** the student provides domain constraints (e.g., "our clusters use private registries"), **Then** Claude refines the configuration to include imagePullSecrets
4. **Given** iterative refinement, **When** the student and Claude collaborate, **Then** the final configuration works correctly and the student can explain each section

---

### User Story 10 - Student Completes End-to-End Capstone (Priority: P1)

A student implements a complete CI/CD pipeline from GitHub Actions through ArgoCD that deploys their Part 6 FastAPI agent with automated builds, tests, and GitOps deployment.

**Why this priority**: The capstone validates all learning. Students must demonstrate integrated understanding, not just isolated concepts.

**Independent Test**: Can be tested by pushing a code change and observing the complete flow: GitHub Actions builds/tests/pushes, ArgoCD detects the new image tag, syncs, and the agent is accessible.

**Acceptance Scenarios**:

1. **Given** a student's complete pipeline, **When** they push a code change to main, **Then** GitHub Actions builds, tests, and pushes a new image within 10 minutes
2. **Given** the new image is pushed, **When** the image tag is updated in the GitOps repo, **Then** ArgoCD syncs the new version within 3 minutes
3. **Given** the sync completes, **When** the student accesses the agent endpoint, **Then** they see the new version running with updated functionality
4. **Given** a breaking change, **When** the student rolls back via Git revert, **Then** ArgoCD syncs the previous version and the agent returns to working state

---

### User Story 11 - Student Creates Reusable GitOps Skill (Priority: P3)

A student designs a reusable skill (Persona + Questions + Principles) for GitOps deployments that can be used across future projects.

**Why this priority**: Layer 3 (Intelligence Design) creates organizational value. This skill compounds across the student's career.

**Independent Test**: Can be tested by another student using the skill to deploy a different application successfully.

**Acceptance Scenarios**:

1. **Given** a student who completed the chapter, **When** they create a GitOps Deployment Skill, **Then** the skill includes a clear Persona, contextual Questions, and decision Principles
2. **Given** the skill is applied to a new project, **When** following the skill's guidance, **Then** the user can deploy an application to ArgoCD without referring back to the chapter
3. **Given** edge cases arise, **When** consulting the skill's Principles, **Then** the user can make informed decisions about sync strategies, RBAC, and multi-tenancy

---

### Edge Cases

- What happens when ArgoCD loses connection to the Git repository?
- How does the system handle when a sync takes longer than expected (resource-heavy deployments)?
- What happens when the target namespace doesn't exist?
- How are secrets handled when the Git repository is public?
- What happens when two ApplicationSets generate Applications with the same name?
- How does ArgoCD behave when the cluster runs out of resources during sync?
- What happens when a PreSync hook fails repeatedly (retry limits)?
- How are merge conflicts in the GitOps repository handled?

---

## Requirements *(mandatory)*

### Functional Requirements

**CI Pipeline Requirements**:

- **FR-001**: Lesson content MUST explain CI/CD concepts (continuous integration, continuous deployment, pipelines, stages) before any tool usage
- **FR-002**: Lessons MUST teach GitHub Actions workflow syntax (triggers, jobs, steps, secrets) with practical examples
- **FR-003**: Lessons MUST demonstrate building Docker images in CI with multi-platform support and registry push
- **FR-004**: Lessons MUST cover testing in CI (unit tests, integration tests, quality gates that block deployment)

**GitOps Requirements**:

- **FR-005**: Lesson content MUST explain GitOps principles (declarative, versioned, immutable, continuously reconciled) as mental models before ArgoCD specifics
- **FR-006**: Lessons MUST cover the relationship between imperative (kubectl apply) and declarative (GitOps) deployment styles

**ArgoCD Core Requirements**:

- **FR-007**: Lessons MUST cover ArgoCD installation on Minikube using Helm (aligned with Chapter 51 knowledge)
- **FR-008**: Lessons MUST explain ArgoCD architecture (API Server, Repo Server, Application Controller, Redis, Dex)
- **FR-009**: Lessons MUST teach the Application CRD (sources, destinations, sync policies, health status)
- **FR-010**: Lessons MUST demonstrate creating Applications via UI, CLI, and declarative YAML

**Advanced ArgoCD Requirements**:

- **FR-011**: Lessons MUST cover sync strategies (manual, auto-sync, auto-prune, self-heal)
- **FR-012**: Lessons MUST teach sync waves and resource hooks (PreSync, Sync, PostSync, SyncFail)
- **FR-013**: Lessons MUST cover ApplicationSets with multiple generators (List, Cluster, Matrix, Git)
- **FR-014**: Lessons MUST explain ArgoCD Projects for multi-tenancy and RBAC

**Production Pattern Requirements**:

- **FR-015**: Lessons MUST cover secrets management patterns (External Secrets Operator, Sealed Secrets, or Vault integration)
- **FR-016**: Lessons MUST address progressive delivery concepts (canary, blue-green) with Argo Rollouts overview
- **FR-017**: Lessons MUST demonstrate health checks and custom health assessments for AI agent readiness

**AI Collaboration Requirements**:

- **FR-018**: At least one lesson MUST demonstrate AI collaboration for generating ArgoCD manifests (Layer 2, Three Roles invisible)
- **FR-019**: AI collaboration MUST include student evaluation of AI output, refinement based on domain knowledge, and iterative improvement

**Capstone Requirements**:

- **FR-020**: Capstone MUST integrate GitHub Actions CI with ArgoCD CD for the Part 6 agent
- **FR-021**: Capstone MUST follow specification-first approach (Layer 4)
- **FR-022**: Capstone MUST result in a working end-to-end pipeline that students can demonstrate

**Skill Design Requirements**:

- **FR-023**: Final lesson MUST guide students in creating a GitOps Deployment Skill using Persona + Questions + Principles pattern
- **FR-024**: The skill MUST be reusable across different applications and clusters

### Key Entities

- **CI Pipeline**: A sequence of automated stages (build, test, push) triggered by code changes
- **GitOps Repository**: A Git repository containing declarative Kubernetes manifests that represent desired cluster state
- **ArgoCD Application**: A CRD that defines the relationship between a Git source and a Kubernetes destination
- **ApplicationSet**: A CRD that generates multiple Applications from templates and generators
- **Sync Wave**: An ordering mechanism for resource application within a single sync operation
- **Resource Hook**: A Job or other resource that executes at specific points in the sync lifecycle
- **ArgoCD Project**: A logical grouping of Applications with shared RBAC and resource restrictions

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Conceptual Understanding**:

- **SC-001**: Students can diagram a CI/CD pipeline and explain each stage's purpose within 5 minutes
- **SC-002**: Students can articulate the difference between imperative and declarative deployments
- **SC-003**: Students can explain ArgoCD's reconciliation loop in their own words

**Practical Skills**:

- **SC-004**: Students can create a GitHub Actions workflow that builds and pushes a Docker image within 30 minutes
- **SC-005**: Students can install ArgoCD and create their first Application within 20 minutes
- **SC-006**: Students can implement sync waves and hooks to control deployment ordering
- **SC-007**: Students can create an ApplicationSet that generates Applications for 3+ environments

**AI Collaboration**:

- **SC-008**: Students can prompt AI to generate ArgoCD manifests and critically evaluate the output
- **SC-009**: Students can identify errors in AI-generated configurations and provide corrections
- **SC-010**: Students demonstrate the iterative refinement pattern (request → evaluate → refine → validate)

**Integration**:

- **SC-011**: Students complete a working CI/CD pipeline for their Part 6 agent within the capstone time allocation
- **SC-012**: Students can rollback a deployment via Git revert and observe ArgoCD sync the previous state

**Reusable Intelligence**:

- **SC-013**: Students produce a GitOps Deployment Skill that another person could use to deploy an application
- **SC-014**: The skill captures decision-making principles, not just procedural steps

---

## Chapter Structure *(for educational content)*

### Lesson Breakdown

| Lesson | Title                                   | Layer | Focus                                              |
| ------ | --------------------------------------- | ----- | -------------------------------------------------- |
| 1      | CI/CD Concepts: The Automated Pipeline  | L1    | Stages, triggers, artifacts, value proposition     |
| 2      | GitHub Actions Fundamentals             | L1    | Workflows, jobs, steps, triggers, YAML syntax      |
| 3      | Building Docker Images in CI            | L1    | Multi-platform builds, registry push, build args   |
| 4      | Testing and Quality Gates               | L1    | Unit/integration tests, coverage, blocking failures|
| 5      | GitOps Principles: Git as Truth         | L1    | Declarative, versioned, reconciled, drift detection|
| 6      | ArgoCD Architecture & Installation      | L1    | Components, CRDs, Helm installation on Minikube    |
| 7      | Your First ArgoCD Application           | L1    | Source, destination, sync policies, UI/CLI creation|
| 8      | Sync Strategies and Policies            | L1    | Manual, auto-sync, auto-prune, self-heal, replace  |
| 9      | Sync Waves and Resource Hooks           | L1    | Wave ordering, PreSync, PostSync, SyncFail hooks   |
| 10     | ApplicationSets: Scaling Deployments    | L1    | List, Cluster, Matrix, Git generators              |
| 11     | ArgoCD Projects and RBAC                | L1    | Multi-tenancy, resource restrictions, roles        |
| 12     | Health Status and Notifications         | L1    | Health checks, degraded states, Slack/webhook alerts|
| 13     | Progressive Delivery Overview           | L1    | Canary, blue-green, Argo Rollouts concepts         |
| 14     | Secrets Management for GitOps           | L1    | External Secrets, Sealed Secrets, Vault patterns   |
| 15     | Multi-Cluster Deployments               | L1    | Hub-spoke, cluster registration, ApplicationSets   |
| 16     | AI-Assisted GitOps Workflows            | L2    | Claude collaboration for manifests and debugging   |
| 17     | Capstone: End-to-End Agent Pipeline     | L4    | Spec-driven CI/CD for Part 6 agent                 |
| 18     | Building the GitOps Deployment Skill    | L3    | Persona + Questions + Principles skill design      |

### 4-Layer Teaching Progression

- **Lessons 1-15 (Layer 1 - Manual Foundation)**: Build comprehensive mental models of CI/CD and GitOps concepts. Students execute commands, observe results, and understand mechanisms before AI assistance.

- **Lesson 16 (Layer 2 - AI Collaboration)**: Students collaborate with Claude Code to generate ArgoCD configurations. They experience the Three Roles (Teacher, Student, Co-Worker) without the framework being exposed.

- **Lesson 17 (Layer 4 - Spec-Driven Integration)**: Students write a specification for their complete CI/CD pipeline and implement it using accumulated knowledge plus AI assistance.

- **Lesson 18 (Layer 3 - Intelligence Design)**: Students create a reusable GitOps Deployment Skill that captures their learning as organizational intelligence.

---

## Assumptions

1. Students have completed Chapters 49-51 and have Docker, Kubernetes, and Helm proficiency
2. Students have a GitHub account and can create repositories
3. Students have access to a container registry (Docker Hub, GitHub Container Registry, or similar)
4. Minikube is already installed and configured from Chapter 50
5. The Part 6 FastAPI agent is available in a Git repository for the capstone
6. ArgoCD 3.x (current stable version) is used throughout the chapter
7. GitHub Actions is the CI tool (not Jenkins, GitLab CI, etc.) to maintain focus
8. Students have Claude Code access for the AI collaboration lesson

---

## Dependencies

- **Chapter 49 (Docker)**: Container image building fundamentals
- **Chapter 50 (Kubernetes)**: Cluster operations, RBAC, resource management
- **Chapter 51 (Helm)**: Chart creation and release management
- **Part 6 (AI Agent)**: The FastAPI agent that will be deployed via the pipeline
- **GitHub repository**: Students need a repo for their agent code
- **Container registry**: For storing built images

---

## Non-Goals (What This Chapter Does NOT Cover)

- Jenkins, GitLab CI, CircleCI, or other CI tools (focus is GitHub Actions)
- Flux CD or other GitOps tools (focus is ArgoCD exclusively)
- Argo Workflows (separate from Argo Rollouts and ArgoCD)
- Deep Kubernetes networking beyond what's needed for ArgoCD
- Cloud provider-specific CI/CD services (AWS CodePipeline, GCP Cloud Build)
- Container registry setup (assumed students have access from Chapter 49)
- Advanced Kustomize overlays (basic Helm focus aligns with Chapter 51)
