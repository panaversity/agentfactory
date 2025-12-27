# Feature Specification: Chapter 51 — Helm Charts for AI Services

**Feature Branch**: `014-helm-charts-chapter`
**Created**: 2025-12-23
**Status**: Draft
**Input**: User description: "Chapter 51: Helm Charts for AI Services - Comprehensive deep-dive on Helm packaging for Kubernetes"

---

## Overview

Chapter 51 is a **comprehensive deep-dive** into Helm, the package manager for Kubernetes. Starting from fundamentals in Lesson 1 (moved from Chapter 50), this chapter transforms students from Helm beginners into Helm chart architects who can design, template, test, and distribute production-grade charts for AI services.

**Why This Chapter Matters**: In production, AI services require multi-environment deployments (dev/staging/prod), complex dependencies, lifecycle hooks for migrations, and organizational chart sharing. Single-lesson Helm coverage is insufficient. This chapter produces students who can architect Helm-based deployment strategies for enterprise AI systems.

---

## Assumed Knowledge

**What students know BEFORE this chapter** (from Chapter 50):
- Kubernetes architecture (control plane, workers, declarative model)
- Core primitives: Pods, Deployments, Services, ConfigMaps, Secrets
- Advanced patterns: Init containers, sidecars, StatefulSets
- Security: RBAC, SecurityContext, NetworkPolicy
- Resource management: Requests/limits, HPA
- Docker Desktop Kubernetes cluster and kubectl operations

**What this chapter must explain from scratch**:
- Helm fundamentals (installation, chart structure, releases) - Lesson 1
- Go templating (flow control, variables, pipelines, named templates)
- Chart dependencies and subchart composition
- Helm hooks (pre-install, post-upgrade, test hooks)
- Chart testing with helm test and unit testing frameworks
- OCI registry integration for chart distribution
- Chart signing and provenance verification
- Library charts for organizational standardization
- Production patterns: umbrella charts, GitOps integration points

**Proficiency Level**: B1-B2 (Intermediate to Upper-Intermediate)
- Students have solid Kubernetes foundation
- Ready for advanced templating and enterprise patterns
- Can reason about deployment strategy tradeoffs

---

## Clarifications

### Session 2025-12-23

- Q: Coverage scan performed → A: All taxonomy categories Clear - no critical ambiguities detected

---

## User Scenarios & Testing

### User Story 1 - Master Advanced Templating (Priority: P1)

A student who completed Chapter 50's basic Helm intro needs to create sophisticated templates that handle complex conditional logic, iterate over dynamic data structures, and create reusable template partials.

**Why this priority**: Advanced templating is the foundation for all subsequent Helm skills. Without mastering Go templates, students cannot create maintainable, DRY charts.

**Independent Test**: Student can create a chart template that conditionally includes Ingress based on environment, iterates over a list of environment variables, and uses named templates for consistent labeling—all verified via `helm template --debug`.

**Acceptance Scenarios**:

1. **Given** a values.yaml with `ingress.enabled: true`, **When** student runs `helm template`, **Then** Ingress manifest is included in output
2. **Given** a list of environment variables in values.yaml, **When** student uses `range` in deployment template, **Then** all env vars appear correctly in rendered output
3. **Given** a helper template in `_helpers.tpl`, **When** student uses `include` in multiple templates, **Then** consistent labels appear across all resources

---

### User Story 2 - Manage Multi-Environment Deployments (Priority: P1)

A student needs to deploy their AI agent to dev, staging, and production environments with different configurations (replicas, resources, model versions, feature flags).

**Why this priority**: Real-world deployments require environment-specific configurations. This is the primary use case for Helm in enterprise settings.

**Independent Test**: Student can deploy the same chart to three environments using different values files, and verify each deployment has correct environment-specific configuration.

**Acceptance Scenarios**:

1. **Given** values-dev.yaml with 1 replica and DEBUG logging, **When** student runs `helm install -f values-dev.yaml`, **Then** deployment has 1 replica and DEBUG log level
2. **Given** values-prod.yaml with 5 replicas and autoscaling enabled, **When** student runs `helm install -f values-prod.yaml`, **Then** deployment has HPA configured with min 3, max 10 replicas
3. **Given** command-line override `--set image.tag=v2.0.0`, **When** combined with values file, **Then** command-line value takes precedence

---

### User Story 3 - Implement Chart Dependencies (Priority: P2)

A student's AI agent requires PostgreSQL for state and Redis for caching. Instead of creating charts from scratch, they need to compose their chart with community charts as dependencies.

**Why this priority**: Enterprise AI systems rarely exist in isolation. Understanding subchart composition enables rapid, reliable infrastructure setup using battle-tested community charts.

**Independent Test**: Student can add Bitnami PostgreSQL and Redis as dependencies, configure them via parent values, and deploy the complete stack with a single `helm install`.

**Acceptance Scenarios**:

1. **Given** Chart.yaml with postgresql dependency, **When** student runs `helm dependency update`, **Then** postgresql chart is downloaded to charts/ directory
2. **Given** parent values.yaml with `postgresql.auth.database: mydb`, **When** chart is deployed, **Then** PostgreSQL pod runs with database "mydb" created
3. **Given** dependency with `condition: redis.enabled`, **When** `redis.enabled: false` in values, **Then** Redis resources are not created

---

### User Story 4 - Use Helm Hooks for Lifecycle Management (Priority: P2)

A student needs to run database migrations before their agent starts and send notifications after successful deployment.

**Why this priority**: Production deployments require lifecycle orchestration. Hooks enable pre/post actions that are critical for zero-downtime deployments.

**Independent Test**: Student can create a pre-upgrade hook that runs migrations and verify it executes before the main deployment updates.

**Acceptance Scenarios**:

1. **Given** a Job with `helm.sh/hook: pre-upgrade`, **When** student runs `helm upgrade`, **Then** migration job completes before deployment pods update
2. **Given** hook with `helm.sh/hook-weight: "-5"`, **When** multiple hooks exist, **Then** lower weight hooks run first
3. **Given** hook with `helm.sh/hook-delete-policy: hook-succeeded`, **When** hook job succeeds, **Then** job resource is automatically deleted

---

### User Story 5 - Test Charts Before Deployment (Priority: P2)

A student needs to validate their chart works correctly before deploying to production, including testing connectivity, configuration, and integration.

**Why this priority**: Untested charts cause production incidents. Testing provides confidence and catches issues early.

**Independent Test**: Student can write and execute chart tests that verify service connectivity and configuration correctness.

**Acceptance Scenarios**:

1. **Given** a test pod with `helm.sh/hook: test`, **When** student runs `helm test`, **Then** test pod executes and reports pass/fail
2. **Given** `helm lint` command, **When** chart has missing required value, **Then** lint fails with specific error message
3. **Given** `helm template --debug`, **When** template has syntax error, **Then** detailed error with line number is shown

---

### User Story 6 - Distribute Charts via OCI Registry (Priority: P3)

A student needs to share their AI agent chart with their team by publishing to an OCI-compliant container registry.

**Why this priority**: Team collaboration requires chart sharing. OCI registries provide standard distribution with access control.

**Independent Test**: Student can push chart to an OCI registry, pull it in a different environment, and install from the registry URL.

**Acceptance Scenarios**:

1. **Given** packaged chart `my-agent-1.0.0.tgz`, **When** student runs `helm push` to OCI registry, **Then** chart is accessible at `oci://registry/charts/my-agent`
2. **Given** chart in OCI registry, **When** team member runs `helm install oci://registry/charts/my-agent`, **Then** chart installs successfully
3. **Given** Chart.yaml dependency with `repository: oci://registry/charts`, **When** `helm dependency update` runs, **Then** dependency is pulled from OCI registry

---

### User Story 7 - Create Library Charts for Standardization (Priority: P3)

A student's organization has 10+ microservices that need consistent deployment patterns (labels, probes, resource defaults). They need to create a library chart that other charts inherit.

**Why this priority**: Enterprise standardization reduces drift and enforces best practices across teams.

**Independent Test**: Student can create a library chart with common templates, use it as a dependency in an application chart, and verify inherited patterns.

**Acceptance Scenarios**:

1. **Given** library chart with `type: library` in Chart.yaml, **When** student runs `helm install`, **Then** Helm refuses (library charts cannot be installed directly)
2. **Given** application chart with library dependency, **When** using `include "libchart.labels"`, **Then** standard labels appear in all resources
3. **Given** library template with configurable defaults, **When** application chart overrides values, **Then** application-specific values take precedence

---

### User Story 8 - Capstone: Complete AI Agent Chart (Priority: P1)

A student creates a production-ready Helm chart for their Part 6 AI agent with all enterprise patterns: multi-environment values, dependencies, hooks, tests, and documentation.

**Why this priority**: Capstone synthesizes all learning. Students demonstrate mastery by producing deployable, maintainable artifacts.

**Independent Test**: Student can deploy their complete AI agent to Docker Desktop Kubernetes with one command, including database, cache, migrations, and health verification.

**Acceptance Scenarios**:

1. **Given** complete agent chart with PostgreSQL dependency, **When** `helm install my-agent ./agent-chart -f values-prod.yaml`, **Then** agent, database, and all supporting resources deploy successfully
2. **Given** deployed release, **When** `helm test my-agent` runs, **Then** connectivity and health tests pass
3. **Given** chart with README.md, **When** colleague reviews chart, **Then** they can understand configuration options and deploy without additional guidance

---

### Edge Cases

- What happens when a hook job fails? (deployment should not proceed)
- How does Helm handle circular dependencies between subcharts?
- What happens when OCI registry is unreachable during `helm dependency update`?
- How do students debug template rendering errors with complex nested values?
- What happens when two charts define the same named template?

---

## Requirements

### Functional Requirements

#### Content Structure Requirements

- **FR-001**: Chapter MUST contain 12 lessons following 4-Layer Teaching Method progression
- **FR-002**: Lessons 1-9 MUST be Layer 1 (Manual Foundation) establishing Helm fundamentals and templating mastery before AI collaboration
- **FR-003**: Lesson 10 MUST be Layer 2 (AI Collaboration) demonstrating AI-assisted chart creation
- **FR-004**: Lesson 11 MUST be Layer 4 capstone project producing deployable AI agent chart
- **FR-005**: Lesson 12 MUST be Layer 3 lesson creating reusable Helm intelligence (skill)

#### Knowledge Requirements

- **FR-006**: Chapter MUST teach Go template syntax: variables, pipelines, control flow (if/else, range, with)
- **FR-007**: Chapter MUST teach named templates and the `include` function (not deprecated `template`)
- **FR-008**: Chapter MUST teach Chart.yaml dependency management including conditions, tags, and aliases
- **FR-009**: Chapter MUST teach all 9 Helm hook types and hook delete policies
- **FR-010**: Chapter MUST teach chart testing with `helm test`, `helm lint`, and `helm template --debug`
- **FR-011**: Chapter MUST teach OCI registry operations: push, pull, install from oci:// URLs
- **FR-012**: Chapter MUST teach library charts with `type: library` and organizational patterns

#### Practical Requirements

- **FR-013**: Every lesson MUST include hands-on exercises using the student's Part 6 AI agent
- **FR-014**: Chapter MUST produce values files for at least 3 environments (dev, staging, prod)
- **FR-015**: Capstone MUST integrate chart with PostgreSQL and/or Redis dependencies
- **FR-016**: All code examples MUST be executable on Docker Desktop Kubernetes without cloud dependencies

#### Quality Requirements

- **FR-017**: Chapter MUST include Lesson 1 (Introduction to Helm) as foundation before advanced topics
- **FR-018**: Chapter MUST reference Chapter 50 prerequisites explicitly in README
- **FR-019**: Each lesson MUST end with "Try With AI" section (no "Key Takeaways" or "What's Next")
- **FR-020**: Chapter MUST follow constitution v6.0.1 meta-commentary prohibition

### Key Entities

- **Chart**: Packaged Kubernetes application with metadata, templates, and default values
- **Release**: Installed instance of a chart with specific configuration and revision history
- **Values**: Hierarchical configuration that customizes chart behavior per environment
- **Template**: Go-templated Kubernetes manifest that renders to valid YAML
- **Dependency**: Subchart that the parent chart requires (e.g., PostgreSQL, Redis)
- **Hook**: Special resource that executes at specific lifecycle points (pre-install, post-upgrade, etc.)
- **Repository**: Server hosting chart packages (traditional HTTP or OCI registry)

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can create a Helm chart from scratch in under 30 minutes (measured by capstone completion time)
- **SC-002**: Students can deploy their AI agent to 3 environments with a single chart (dev/staging/prod via values files)
- **SC-003**: 100% of code examples in chapter execute successfully on Docker Desktop Kubernetes
- **SC-004**: Students can explain the difference between `template` and `include` functions (conceptual understanding test)
- **SC-005**: Students can add a community chart as dependency and configure it via parent values (dependency integration test)
- **SC-006**: Students can write a pre-upgrade hook that runs before deployment updates (lifecycle management test)
- **SC-007**: Students can push/pull charts from an OCI registry (distribution test)
- **SC-008**: Chapter passes validation-auditor with zero constitution violations
- **SC-009**: All lessons follow 4-Layer Method with appropriate layer for content complexity
- **SC-010**: Chapter content is superior to official Helm documentation in pedagogical clarity (qualitative review)

---

## Scope & Constraints

### In Scope

- Advanced Go templating (building on Chapter 50's basics)
- Multi-environment deployment strategies
- Chart dependencies and subchart patterns
- Helm hooks for lifecycle management
- Chart testing and linting
- OCI registry distribution
- Library charts for standardization
- Capstone: Production-ready AI agent chart

### Out of Scope (Non-Goals)

- Helm plugin development (advanced topic beyond B1-B2)
- Custom Helm storage backends (SQL, memory)
- Chart signing with GPG (security topic, would require separate lesson)
- Helm SDK programming (Go development, not deployment)
- Helmfile or other Helm orchestration tools (separate tooling chapter)
- Cloud-specific chart repositories (Azure ACR, AWS ECR specifics)

### Assumptions

- Students have completed Chapter 50 (Kubernetes fundamentals)
- Students have Docker Desktop Kubernetes running with metrics-server enabled
- Students have their Part 6 AI agent containerized and pushed to a registry
- Students have access to Docker Hub or similar public registry for OCI exercises
- Helm 3.14+ is installed (OCI support is stable)

### Dependencies

- **Chapter 50**: Kubernetes fundamentals and basic Helm (prerequisite)
- **Chapter 49**: Containerized AI agent (provides the application to package)
- **Part 6**: AI agent development (provides the service being deployed)

---

## Lesson Outline (Final - 12 Lessons)

| #  | Title                               | Layer | Focus                                        |
|----|-------------------------------------|-------|----------------------------------------------|
| 1  | Introduction to Helm                | L1    | Installation, chart structure, public charts, releases (moved from Ch50) |
| 2  | Advanced Go Templating              | L1    | Variables, pipelines, flow control           |
| 3  | Named Templates and Helpers         | L1    | _helpers.tpl, include vs template            |
| 4  | Values Deep Dive                    | L1    | Hierarchy, schema validation, globals        |
| 5  | Chart Dependencies                  | L1    | Subcharts, conditions, import-values         |
| 6  | Helm Hooks and Lifecycle            | L1    | Lifecycle events, weights, delete policies   |
| 7  | Testing Your Charts                 | L1    | helm test, lint, template debugging          |
| 8  | OCI Registries and Distribution     | L1    | Push, pull, install from OCI                 |
| 9  | Library Charts and Standardization  | L1    | Organizational standardization               |
| 10 | AI-Assisted Chart Development       | L2    | Three Roles collaboration for chart creation |
| 11 | Capstone: Production AI Agent Chart | L4    | Spec-driven complete chart                   |
| 12 | Building a Helm Chart Skill         | L3    | Reusable intelligence for future projects    |

**Note**: Lesson 1 (Introduction to Helm) was moved from Chapter 50 to provide proper foundation before advanced topics.

---

## Running Example Requirement

**IMPORTANT**: All lessons MUST use the **Task API** from Chapters 40/49/50 as the running example:
- The containerized Task API image from Chapter 49
- The Kubernetes deployment patterns from Chapter 50
- Chart examples should reference `task-api` not generic `my-agent`

This maintains continuity across the cloud-native track (Docker → Kubernetes → Helm).

---

## Differentiation from Chapter 50

Chapter 50 no longer includes Helm content (moved to Chapter 51). Students come to Chapter 51 with:
- Kubernetes architecture understanding
- kubectl operations experience
- Deployed Task API on Kubernetes

Chapter 51 teaches Helm from scratch:
- Lesson 1: Helm fundamentals (installation, chart structure, public charts, releases)
- Lessons 2-9: Advanced patterns (templating, dependencies, hooks, testing, OCI, library charts)
- Lessons 10-12: AI collaboration, capstone, and skill creation
