# Feature Specification: Chapter 50 - Kubernetes for AI Services

**Feature Branch**: `001-chapter-50-kubernetes`
**Created**: 2025-12-22
**Status**: Draft
**Input**: Chapter 50: Kubernetes for AI Services - Teaching container orchestration for deploying AI agents

## Assumed Knowledge

**What students know BEFORE this chapter**:
- Docker fundamentals (Chapter 49): images, containers, Dockerfiles, multi-stage builds
- Container lifecycle: build, run, stop, remove
- Docker Compose for multi-container local development
- Their Part 6 FastAPI agent is containerized and ready for deployment
- Basic YAML syntax (from Chapter 10 Markdown and throughout book)
- Command-line proficiency (Chapter 7 Bash Essentials)

**What this chapter must explain from scratch**:
- Container orchestration concepts (why Docker alone isn't enough for production)
- The declarative model: desired state vs observed state
- Kubernetes architecture: control plane, worker nodes, API server
- Core resources: Pods, Deployments, Services, ConfigMaps, Secrets
- Local Kubernetes with Minikube
- kubectl CLI for cluster interaction
- AI-assisted Kubernetes workflows (kubectl-ai)

## Chapter Overview

**Proficiency Level**: B1 (Intermediate)

**Chapter Position**: Part 7 - AI Cloud Native Development, Chapter 50

**Thread**: Students deploy their Part 6 FastAPI agent to a Kubernetes cluster, transforming "works in Docker" into "runs in production-grade orchestration."

**Teaching Progression**: 4-Layer Method
- Lessons 1-7: Layer 1 (Manual Foundation) - Build mental models through hands-on practice
- Lesson 8: Layer 2 (AI Collaboration) - kubectl-ai for manifest generation and debugging
- Lesson 9: Layer 4 (Spec-Driven) - Capstone project deploying their actual agent
- Lesson 10: Layer 3 (Intelligence Design) - Create reusable kubernetes-deployment skill

---

## User Scenarios & Testing

### User Story 1 - Learn Kubernetes Architecture (Priority: P1)

A student who has containerized their AI agent wants to understand WHY container orchestration exists and HOW Kubernetes solves production deployment challenges that Docker alone cannot address.

**Why this priority**: Without understanding the declarative model and architecture, students cannot reason about any Kubernetes operation. This is the foundational mental model.

**Independent Test**: Student can explain desired state vs observed state, identify control plane vs worker node roles, and describe what happens when a container crashes.

**Acceptance Scenarios**:

1. **Given** a student has completed Chapter 49 Docker, **When** they complete Lesson 1, **Then** they can explain why "docker run" is insufficient for production and what problems Kubernetes solves
2. **Given** a student sees a Kubernetes cluster diagram, **When** asked to identify components, **Then** they correctly label API server, scheduler, etcd, kubelet, and kube-proxy
3. **Given** a scenario where a container crashes, **When** asked what happens, **Then** they explain the reconciliation loop (controller detects mismatch, schedules new Pod)

---

### User Story 2 - Deploy Containers to Kubernetes (Priority: P1)

A student wants to deploy their containerized application to a local Kubernetes cluster and understand the Pod → Deployment → Service abstraction hierarchy.

**Why this priority**: Core operational skill - students must be able to run workloads on Kubernetes before any advanced features.

**Independent Test**: Student can write a Deployment YAML, apply it with kubectl, expose it via Service, and access the running application.

**Acceptance Scenarios**:

1. **Given** Minikube is running, **When** student applies a Deployment manifest, **Then** Pods are created and reach Running status within 60 seconds
2. **Given** a running Deployment, **When** student creates a NodePort Service, **Then** they can access the application via browser or curl
3. **Given** a running Pod, **When** student deletes it manually, **Then** the Deployment controller creates a replacement Pod automatically

---

### User Story 3 - Configure Applications in Kubernetes (Priority: P2)

A student needs to inject configuration (API keys, environment variables, feature flags) into their containerized agent without rebuilding the image.

**Why this priority**: Essential for production patterns - separating configuration from code. Enables environment-specific deployments.

**Independent Test**: Student can create ConfigMaps and Secrets, mount them as environment variables or files, and verify the application reads them correctly.

**Acceptance Scenarios**:

1. **Given** an application that reads OPENAI_API_KEY from environment, **When** student creates a Secret and references it in Deployment, **Then** the application successfully authenticates with the API
2. **Given** a ConfigMap with application settings, **When** student updates the ConfigMap, **Then** they understand that Pods must be restarted to pick up changes (unless using volume mounts with auto-reload)
3. **Given** sensitive credentials, **When** student examines the Secret, **Then** they understand base64 encoding is NOT encryption and learn about sealed secrets for production

---

### User Story 4 - Debug Kubernetes Deployments (Priority: P2)

A student encounters a failing deployment and needs to diagnose the issue using kubectl commands and understand common failure modes.

**Why this priority**: Debugging skills are essential for real-world use. Students will encounter CrashLoopBackOff, ImagePullBackOff, and other issues.

**Independent Test**: Student can use kubectl describe, logs, events, and exec to diagnose and fix a broken deployment.

**Acceptance Scenarios**:

1. **Given** a Pod in CrashLoopBackOff state, **When** student runs kubectl logs, **Then** they identify the application error causing the crash
2. **Given** a Pod stuck in Pending state, **When** student runs kubectl describe, **Then** they identify resource constraints or scheduling issues
3. **Given** a Service not routing traffic, **When** student checks label selectors, **Then** they identify and fix the mismatch between Service selector and Pod labels

---

### User Story 5 - Use AI to Accelerate Kubernetes Workflows (Priority: P2)

A student wants to leverage AI assistance to generate Kubernetes manifests, debug cluster issues, and optimize configurations without memorizing YAML syntax.

**Why this priority**: Demonstrates AI-native development approach - specs and intent over syntax memorization.

**Independent Test**: Student can use kubectl-ai (or equivalent) to generate a complete Deployment from natural language, debug an issue by describing symptoms, and iterate on configurations through dialogue.

**Acceptance Scenarios**:

1. **Given** a natural language description "Deploy my-app with 3 replicas, 256MB memory limit, and health check on /health", **When** student uses kubectl-ai, **Then** a valid Deployment YAML is generated that matches the specification
2. **Given** a failing deployment, **When** student describes symptoms to AI assistant, **Then** they receive actionable debugging steps specific to their issue
3. **Given** an AI-generated manifest, **When** student reviews it, **Then** they can evaluate correctness because they understand the underlying concepts (L1 foundation)

---

### User Story 6 - Complete Capstone: Deploy Part 6 Agent (Priority: P1)

A student deploys their actual Part 6 FastAPI agent to Minikube with proper configuration, resource limits, health checks, and external access.

**Why this priority**: Integration of all learning - proves students can apply knowledge to real projects, not just tutorial examples.

**Independent Test**: The student's Part 6 agent runs on Minikube, accepts HTTP requests, and demonstrates self-healing when the Pod is deleted.

**Acceptance Scenarios**:

1. **Given** a containerized Part 6 agent, **When** student writes a deployment specification, **Then** AI generates manifests that include Deployment, Service, ConfigMap, and Secret
2. **Given** the deployed agent, **When** student sends a request to the agent endpoint, **Then** they receive a valid response through the Kubernetes Service
3. **Given** the running agent Pod, **When** student deletes it, **Then** Kubernetes automatically creates a new Pod and service is restored within 30 seconds

---

### User Story 7 - Create Reusable Kubernetes Skill (Priority: P3)

A student extracts their Kubernetes deployment knowledge into a reusable skill that can be applied to future projects.

**Why this priority**: Intelligence accumulation - transforms one-time learning into organizational capability.

**Independent Test**: The created skill successfully guides deployment of a NEW application (not the Part 6 agent) with appropriate questions and principles.

**Acceptance Scenarios**:

1. **Given** the student has completed the capstone, **When** they create a kubernetes-deployment skill, **Then** it includes Persona, Questions, and Principles sections
2. **Given** a new containerized application, **When** the skill is invoked, **Then** it asks relevant questions (workload type, scaling needs, health checks) before generating manifests
3. **Given** the skill output, **When** reviewed against best practices, **Then** it includes resource limits, health probes, and appropriate Service type

---

### Edge Cases

- What happens when Minikube runs out of resources (CPU/memory)?
- How does system handle image pull failures (private registry, missing tag)?
- What happens when a Pod exceeds its memory limit (OOMKilled)?
- How do students recover from a corrupted Minikube cluster?
- What if kubectl-ai is unavailable or produces incorrect output?

---

## Requirements

### Functional Requirements

**Lesson Structure**:
- **FR-001**: Chapter MUST contain exactly 10 lessons following the 4-Layer Teaching Method progression
- **FR-002**: Lessons 1-7 MUST provide Layer 1 (Manual Foundation) without AI assistance
- **FR-003**: Lesson 8 MUST demonstrate Layer 2 (AI Collaboration) using kubectl-ai or equivalent
- **FR-004**: Lesson 9 MUST be a Layer 4 (Spec-Driven) capstone deploying the Part 6 agent
- **FR-005**: Lesson 10 MUST guide creation of a Layer 3 (Intelligence Design) kubernetes-deployment skill

**Content Requirements**:
- **FR-006**: Chapter MUST explain the declarative model (desired state vs observed state) before any kubectl commands
- **FR-007**: Chapter MUST cover Minikube installation for macOS, Windows, and Linux
- **FR-008**: Each lesson MUST include working code examples with expected output shown
- **FR-009**: Chapter MUST provide "Try With AI" sections using action prompts (not meta-commentary about frameworks)
- **FR-010**: All YAML manifests MUST follow Kubernetes best practices (resource limits, labels, health probes)

**Pedagogical Requirements**:
- **FR-011**: Chapter MUST connect every concept to the "deploy your AI agent" thread
- **FR-012**: Chapter MUST build vocabulary before introducing kubectl commands
- **FR-013**: Debugging lessons MUST use intentional failures to build troubleshooting intuition
- **FR-014**: AI collaboration lesson MUST demonstrate Three Roles (invisible framework) through action

**Technical Accuracy**:
- **FR-015**: All examples MUST be tested against Kubernetes v1.28+ and Minikube v1.32+
- **FR-016**: Chapter MUST clarify that Secrets are base64-encoded, not encrypted
- **FR-017**: Chapter MUST explain that ConfigMap changes require Pod restart (unless using volume watch)

### Key Entities

- **Pod**: Smallest deployable unit; wraps one or more containers; mortal with ephemeral IP
- **Deployment**: Controller managing Pod replicas; provides self-healing, scaling, rolling updates
- **Service**: Stable networking abstraction; routes traffic to Pods via label selectors
- **ConfigMap**: Non-sensitive configuration data; injected as env vars or mounted as files
- **Secret**: Sensitive configuration (credentials); base64-encoded, injected similarly to ConfigMap
- **Namespace**: Virtual cluster for resource isolation and organization
- **Node**: Worker machine (physical or virtual) running Pods; managed by kubelet

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: 90% of students can explain the declarative model (desired vs observed state) in their own words after Lesson 1
- **SC-002**: Students can deploy a basic application to Minikube within 15 minutes of completing Lesson 3
- **SC-003**: Students successfully diagnose and fix 3 intentional deployment failures in Lesson 7
- **SC-004**: Students generate a valid Deployment manifest using AI assistance in under 5 minutes (Lesson 8)
- **SC-005**: 100% of students have their Part 6 agent running on Minikube by end of Lesson 9
- **SC-006**: The kubernetes-deployment skill (Lesson 10) successfully guides deployment of a new application without modification
- **SC-007**: All code examples compile and run on Kubernetes v1.28+ without modification
- **SC-008**: Chapter maintains framework invisibility - no meta-commentary about "AI as Teacher" or pedagogical scaffolding

### Quality Gates

- **QG-001**: Every lesson ends with "Try With AI" section using action prompts
- **QG-002**: All YAML examples include expected `kubectl get` output
- **QG-003**: Constitutional compliance validated (framework invisibility, evidence presence, structural compliance)
- **QG-004**: MDX safety validated (no angle brackets before letters/numbers)

---

## Lesson Breakdown

| # | Title | Layer | Duration | Key Concepts |
|---|-------|-------|----------|--------------|
| 1 | Kubernetes Architecture and the Declarative Model | L1 | 35 min | Why orchestration, control plane, workers, desired vs observed state |
| 2 | Setting Up Minikube | L1 | 25 min | Installation (mac/win/linux), cluster creation, kubectl context |
| 3 | Pods: The Atomic Unit | L1 | 40 min | Pod anatomy, YAML structure, lifecycle, multi-container patterns |
| 4 | Deployments: Self-Healing at Scale | L1 | 45 min | ReplicaSets, replicas, rolling updates, rollbacks |
| 5 | Services and Networking | L1 | 40 min | ClusterIP, NodePort, LoadBalancer, label selectors, DNS |
| 6 | ConfigMaps and Secrets | L1 | 35 min | Configuration injection, env vars, volume mounts, security notes |
| 7 | Resource Management and Debugging | L1 | 40 min | Requests/limits, QoS, kubectl debug commands, common failures |
| 8 | AI-Assisted Kubernetes with kubectl-ai | L2 | 45 min | Natural language to YAML, debugging dialogue, iterative refinement |
| 9 | Capstone: Deploy Your Part 6 Agent | L4 | 50 min | Spec-first deployment, full manifest suite, validation |
| 10 | Building the Kubernetes Deployment Skill | L3 | 35 min | Persona + Questions + Principles, cross-project reuse |

---

## Assumptions

- Students have completed Chapter 49 (Docker) and have a working containerized Part 6 agent
- Students have access to a machine capable of running Minikube (8GB RAM minimum recommended)
- kubectl-ai or equivalent AI assistant is available for Lesson 8
- Students have basic familiarity with YAML from earlier chapters
- Minikube provides sufficient local Kubernetes functionality without cloud costs
- The Part 6 agent is a stateless FastAPI application suitable for Deployment (not StatefulSet)

---

## Out of Scope

- Cloud-managed Kubernetes (EKS, GKE, AKS) - covered in later chapters
- Helm charts - Chapter 51 topic
- StatefulSets and persistent storage - separate chapter
- Kubernetes operators and CRDs
- Service mesh (Istio, Linkerd)
- Multi-cluster deployments
- Production security hardening (network policies, pod security standards) beyond basics
- CI/CD integration - Chapter 53 topic

---

## Dependencies

- **Chapter 49**: Docker fundamentals and containerized Part 6 agent
- **Part 6**: FastAPI agent that will be deployed in capstone
- **Chapter 7**: Bash/CLI proficiency for kubectl commands
- **Chapter 10**: YAML syntax understanding
