# Feature Specification: Chapter 50 Kubernetes Expansion

**Feature Branch**: `002-chapter-50-kubernetes-expansion`
**Created**: 2025-12-22
**Status**: Draft
**Input**: Expand Chapter 50 from 10 lessons to 22 comprehensive lessons covering all essential Kubernetes concepts for AI-native development.

## Executive Summary

The current Chapter 50 covers approximately 50-60% of essential Kubernetes concepts from Nigel Poulton's "The Kubernetes Book." This expansion will transform it into a comprehensive Kubernetes education equivalent to the full book, but with an AI-native focus applied to every topic.

### Gap Analysis Summary

| Priority | Missing Topic | AI-Native Application |
|----------|--------------|----------------------|
| P1 | Namespaces | Isolate dev/staging/prod agent environments |
| P1 | Persistent Storage (PV/PVC) | Store model weights, vector DBs, agent logs |
| P1 | Ingress | Expose agent APIs to external traffic |
| P1 | Autoscaling (HPA) | Handle variable AI inference load |
| P1 | Init Containers | Download models before agent starts |
| P2 | StatefulSets | Agents requiring persistent identity |
| P2 | RBAC | Secure production agent deployments |
| P2 | Service Discovery Deep Dive | Debug agent-to-agent communication |
| P2 | Security Best Practices | Production hardening for AI workloads |
| P3 | Helm Charts | Package agent deployments |

---

## User Scenarios & Testing

### User Story 1 - Complete K8s Foundation (Priority: P1)

A student completing Chapter 49 (Docker) wants to learn Kubernetes comprehensively to deploy AI agents in production. They need to understand not just basic concepts but the full production stack including networking, storage, security, and scaling.

**Why this priority**: Foundation knowledge is prerequisite for all advanced topics. Without understanding core K8s concepts, students cannot progress to production deployment.

**Independent Test**: Student can deploy a multi-container AI agent to Minikube with proper resource limits, health checks, and labels after completing Part 1 (Lessons 1-5).

**Acceptance Scenarios**:

1. **Given** student has completed Docker chapter, **When** they complete Lessons 1-5, **Then** they can create Pods, Deployments, and Services from scratch without AI assistance
2. **Given** student understands declarative model, **When** a Pod crashes, **Then** they can explain why K8s auto-recovers it using reconciliation loop concept

---

### User Story 2 - Production Networking (Priority: P1)

A student needs to expose their AI agent API to the internet and understand how agents discover and communicate with each other within the cluster.

**Why this priority**: AI agents are useless if users can't access them. Networking is fundamental to production deployment.

**Independent Test**: Student can expose an agent API via Ingress with TLS and debug DNS resolution issues between services.

**Acceptance Scenarios**:

1. **Given** a deployed agent Service, **When** student creates an Ingress resource, **Then** the agent API is accessible via HTTP(S) from outside the cluster
2. **Given** two agents in different namespaces, **When** Agent A needs to call Agent B, **Then** student can configure cross-namespace DNS resolution

---

### User Story 3 - Stateful AI Workloads (Priority: P1)

A student needs to deploy an AI agent that requires persistent storage for model weights, embeddings, or conversation history.

**Why this priority**: Most production AI agents need persistent data. Without storage knowledge, agents lose state on restart.

**Independent Test**: Student can deploy an agent with a PersistentVolumeClaim that survives Pod restarts and node failures.

**Acceptance Scenarios**:

1. **Given** an agent that saves embeddings to disk, **When** the Pod is deleted and recreated, **Then** the embeddings persist and are available to the new Pod
2. **Given** a vector database container, **When** student configures a StorageClass, **Then** dynamic volume provisioning automatically creates storage

---

### User Story 4 - Auto-Scaling AI Inference (Priority: P1)

A student's AI agent experiences variable load - quiet during night, high traffic during business hours. They need automatic scaling.

**Why this priority**: AI inference is resource-intensive. Manual scaling is impractical for production workloads.

**Independent Test**: Student can configure HPA that scales agent replicas from 2 to 10 based on CPU utilization during load test.

**Acceptance Scenarios**:

1. **Given** an agent Deployment with HPA configured, **When** CPU exceeds 70%, **Then** K8s automatically adds replicas
2. **Given** traffic drops, **When** CPU falls below 50%, **Then** K8s gracefully scales down replicas

---

### User Story 5 - Secure Production Deployment (Priority: P2)

A student needs to deploy agents to production with proper security: least-privilege access, secrets management, and network policies.

**Why this priority**: Security is mandatory for production but can be learned after basic deployment skills.

**Independent Test**: Student can configure ServiceAccount with minimal RBAC permissions and deploy agent that cannot access other namespaces.

**Acceptance Scenarios**:

1. **Given** a production namespace, **When** student applies RBAC rules, **Then** agent can only access secrets in its own namespace
2. **Given** an agent Pod, **When** student runs security audit, **Then** Pod runs as non-root with read-only filesystem

---

### User Story 6 - AI-Assisted Operations (Priority: P2)

A student wants to use AI tools (kubectl-ai) to accelerate Kubernetes operations while understanding what's happening under the hood.

**Why this priority**: AI collaboration is the Layer 2 teaching method, building on manual foundation.

**Independent Test**: Student can use kubectl-ai to generate manifests, then explain and modify the generated YAML.

**Acceptance Scenarios**:

1. **Given** student understands manual K8s operations, **When** they use kubectl-ai, **Then** they can validate and improve AI-generated manifests
2. **Given** a debugging scenario, **When** student asks AI for help, **Then** they understand the suggested commands before running them

---

### User Story 7 - Capstone Integration (Priority: P1)

A student deploys their complete Part 6 FastAPI agent to Kubernetes with all production concerns: Ingress, storage, autoscaling, secrets, health checks.

**Why this priority**: Capstone validates all learning objectives and produces a real deployable artifact.

**Independent Test**: Student's Part 6 agent runs in Minikube with external access, survives restarts, and auto-scales.

**Acceptance Scenarios**:

1. **Given** completed Part 6 agent, **When** student completes capstone, **Then** agent is accessible via Ingress URL
2. **Given** capstone deployment, **When** student deletes a Pod, **Then** agent recovers automatically with no data loss

---

### Edge Cases

- What happens when PersistentVolume runs out of space?
- How does HPA behave when cluster has no resources for new Pods?
- What happens when Ingress controller is not installed?
- How to handle Secrets rotation without downtime?
- What happens when DNS resolution fails between services?

---

## Requirements

### Functional Requirements - Content Structure

- **FR-001**: Chapter MUST contain minimum 22 lessons covering all major K8s concepts
- **FR-002**: Each lesson MUST follow 4-Layer Teaching Method progression
- **FR-003**: All lessons MUST have AI-native context (apply concepts to AI agent deployment)
- **FR-004**: All code examples MUST include expected outputs
- **FR-005**: Each lesson MUST end with "Try With AI" section for Layer 2 practice
- **FR-006**: Lessons MUST build progressively - no forward references to unexplained concepts

### Functional Requirements - Topic Coverage

- **FR-010**: Chapter MUST cover Kubernetes architecture and declarative model
- **FR-011**: Chapter MUST cover Pods (single, multi-container, init containers, sidecars)
- **FR-012**: Chapter MUST cover Deployments (ReplicaSets, rolling updates, rollbacks)
- **FR-013**: Chapter MUST cover Services (ClusterIP, NodePort, LoadBalancer)
- **FR-014**: Chapter MUST cover Namespaces (virtual clusters, resource quotas)
- **FR-015**: Chapter MUST cover Ingress (external access, TLS termination)
- **FR-016**: Chapter MUST cover Persistent Storage (PV, PVC, StorageClass)
- **FR-017**: Chapter MUST cover ConfigMaps and Secrets
- **FR-018**: Chapter MUST cover StatefulSets for stateful workloads
- **FR-019**: Chapter MUST cover Horizontal Pod Autoscaler (HPA)
- **FR-020**: Chapter MUST cover RBAC (ServiceAccounts, Roles, RoleBindings)
- **FR-021**: Chapter MUST cover Service Discovery and DNS deep dive
- **FR-022**: Chapter MUST cover Security best practices for AI workloads
- **FR-023**: Chapter MUST cover kubectl-ai for AI-assisted operations
- **FR-024**: Chapter MUST include capstone deploying Part 6 agent
- **FR-025**: Chapter MUST produce a reusable kubernetes-deployment skill

### Functional Requirements - Quality Standards

- **FR-030**: All YAML manifests MUST be syntactically valid
- **FR-031**: All kubectl commands MUST include realistic output examples
- **FR-032**: Each lesson MUST have complete YAML frontmatter (proficiency, learning_objectives, digcomp_mapping)
- **FR-033**: Evidence presence MUST be 70%+ (code blocks have outputs)
- **FR-034**: Framework invisibility MUST be maintained (no meta-commentary)

---

## Proposed Lesson Structure (22 Lessons)

### Part 1: Foundation (Lessons 1-5) - EXISTING, KEEP

| # | Title | Status | Layer |
|---|-------|--------|-------|
| 1 | Kubernetes Architecture and Declarative Model | Exists | L1 |
| 2 | Setting Up Minikube | Exists | L1 |
| 3 | Pods: The Atomic Unit | Exists | L1 |
| 4 | Deployments: Self-Healing at Scale | Exists | L1 |
| 5 | Services and Networking | Exists | L1 |

### Part 2: Multi-Container Patterns (Lessons 6-7) - NEW

| # | Title | Status | Layer |
|---|-------|--------|-------|
| 6 | Init Containers: Preparing the Environment | NEW | L1 |
| 7 | Sidecar Containers: The Agent's Best Friend | NEW | L1 |

### Part 3: Organization & Access (Lessons 8-10) - NEW

| # | Title | Status | Layer |
|---|-------|--------|-------|
| 8 | Namespaces: Virtual Clusters for AI Workloads | NEW | L1 |
| 9 | Ingress: Exposing Your Agent to the World | NEW | L1 |
| 10 | Service Discovery Deep Dive | NEW | L1 |

### Part 4: Configuration & State (Lessons 11-14) - MIXED

| # | Title | Status | Layer |
|---|-------|--------|-------|
| 11 | ConfigMaps and Secrets | Exists (renumber from L6) | L1 |
| 12 | Persistent Storage: PV, PVC, StorageClass | NEW | L1 |
| 13 | StatefulSets: When Your Agent Needs Identity | NEW | L1 |
| 14 | Resource Management and Debugging | Exists (renumber from L7) | L1 |

### Part 5: Production Readiness (Lessons 15-18) - NEW

| # | Title | Status | Layer |
|---|-------|--------|-------|
| 15 | Horizontal Pod Autoscaler: Scaling AI Inference | NEW | L1 |
| 16 | RBAC: Securing Your Agent Deployments | NEW | L1 |
| 17 | Kubernetes Security for AI Services | NEW | L1 |
| 18 | Health Checks: Liveness, Readiness, Startup Probes | NEW | L1 |

### Part 6: AI Collaboration & Integration (Lessons 19-22) - MIXED

| # | Title | Status | Layer |
|---|-------|--------|-------|
| 19 | AI-Assisted Kubernetes: kubectl-ai | Exists (renumber from L8) | L2 |
| 20 | Helm Charts for AI Agent Packaging | NEW | L1 |
| 21 | Capstone: Production-Ready Part 6 Agent | Exists (renumber from L9) | L4 |
| 22 | Building a Kubernetes Deployment Skill | Exists (renumber from L10) | L3 |

---

## Detailed New Lesson Specifications

### Lesson 6: Init Containers - Preparing the Environment

**Learning Objectives**:
- LO1: Explain when and why to use init containers (Understand)
- LO2: Create init containers that run before main containers (Apply)
- LO3: Use init containers to download ML models before agent starts (Apply)
- LO4: Debug init container failures using kubectl logs and describe (Apply)

**AI-Native Context**: Download model weights from S3/GCS before agent container starts. Wait for database to be ready before starting agent.

**Key Concepts**: Init container lifecycle, sequential execution, failure handling, shared volumes between init and app containers

**Duration**: 35 minutes

---

### Lesson 7: Sidecar Containers - The Agent's Best Friend

**Learning Objectives**:
- LO1: Explain the sidecar pattern and native sidecar support in K8s 1.28+ (Understand)
- LO2: Create native sidecars with restartPolicy: Always (Apply)
- LO3: Configure sidecars for logging, monitoring, and proxying (Apply)
- LO4: Share volumes between main and sidecar containers (Apply)
- LO5: Understand sidecar startup and shutdown ordering (Understand)

**AI-Native Context**: Add logging sidecar to capture agent interactions, metrics sidecar for inference latency tracking, proxy sidecar for external API calls.

**Key Concepts**: Native sidecars (spec.initContainers with restartPolicy: Always), container ordering guarantees, shared volumes, lifecycle management

**Duration**: 40 minutes

---

### Lesson 8: Namespaces - Virtual Clusters for AI Workloads

**Learning Objectives**:
- LO1: Explain Namespaces as virtual cluster partitions (Understand)
- LO2: Create and manage Namespaces with kubectl (Apply)
- LO3: Deploy resources to specific Namespaces (Apply)
- LO4: Configure ResourceQuotas to limit namespace resource usage (Apply)
- LO5: Configure LimitRanges for default container limits (Apply)
- LO6: Implement multi-environment strategy (dev/staging/prod) (Apply)

**AI-Native Context**: Isolate development agents from production agents. Set GPU quotas per environment. Prevent dev workloads from consuming prod resources.

**Key Concepts**: Namespace isolation, resource quotas, limit ranges, cross-namespace communication, default namespace behavior

**Duration**: 40 minutes

---

### Lesson 9: Ingress - Exposing Your Agent to the World

**Learning Objectives**:
- LO1: Explain difference between Ingress and LoadBalancer Services (Understand)
- LO2: Install an Ingress controller (nginx-ingress on Minikube) (Apply)
- LO3: Create Ingress resources with path-based routing (Apply)
- LO4: Create Ingress resources with host-based routing (Apply)
- LO5: Configure TLS termination for HTTPS (Apply)
- LO6: Route traffic to multiple agent versions for A/B testing (Apply)

**AI-Native Context**: Expose agent API at api.myagent.com with HTTPS. Route /v1 to stable agent, /v2 to experimental agent.

**Key Concepts**: Ingress controller, Ingress resource, IngressClass, TLS secrets, path routing, host routing, annotations

**Duration**: 45 minutes

---

### Lesson 10: Service Discovery Deep Dive

**Learning Objectives**:
- LO1: Explain Kubernetes DNS architecture (CoreDNS) (Understand)
- LO2: Understand DNS naming convention: service.namespace.svc.cluster.local (Understand)
- LO3: Debug DNS resolution failures with nslookup and dig (Apply)
- LO4: Configure headless Services for direct Pod discovery (Apply)
- LO5: Troubleshoot endpoint mismatches and selector issues (Apply)
- LO6: Understand cross-namespace service discovery (Understand)

**AI-Native Context**: Agent A calling Agent B's API in different namespace. Debugging "connection refused" and "host not found" errors.

**Key Concepts**: CoreDNS, FQDN, A records, SRV records, headless Services, endpoint slices, DNS debugging

**Duration**: 40 minutes

---

### Lesson 12: Persistent Storage

**Learning Objectives**:
- LO1: Explain the PersistentVolume (PV) and PersistentVolumeClaim (PVC) abstraction (Understand)
- LO2: Create static PersistentVolumes (Apply)
- LO3: Create PersistentVolumeClaims to request storage (Apply)
- LO4: Use StorageClasses for dynamic provisioning (Apply)
- LO5: Configure access modes (ReadWriteOnce, ReadOnlyMany, ReadWriteMany) (Apply)
- LO6: Mount volumes into agent containers (Apply)

**AI-Native Context**: Store vector embeddings that persist across Pod restarts. Save model checkpoints. Store conversation logs for analysis.

**Key Concepts**: PV, PVC, StorageClass, access modes, volume mounts, Minikube hostpath provisioner, reclaim policies

**Duration**: 45 minutes

---

### Lesson 13: StatefulSets

**Learning Objectives**:
- LO1: Explain when to use StatefulSets vs Deployments (Understand)
- LO2: Understand stable network identity (pod-0, pod-1, pod-2) (Understand)
- LO3: Understand stable storage with volumeClaimTemplates (Understand)
- LO4: Create StatefulSets with ordered Pod creation/deletion (Apply)
- LO5: Configure headless Services for StatefulSet DNS (Apply)
- LO6: Perform rolling updates on StatefulSets (Apply)

**AI-Native Context**: Deploy a vector database (Qdrant, Milvus, Weaviate) that requires persistent identity and storage. Deploy a multi-replica agent where each replica needs unique identity.

**Key Concepts**: Stable hostname, ordered deployment/scaling, volumeClaimTemplates, headless Service, partition updates

**Duration**: 45 minutes

---

### Lesson 15: Horizontal Pod Autoscaler (HPA)

**Learning Objectives**:
- LO1: Explain HPA and metrics-based autoscaling (Understand)
- LO2: Install and configure metrics-server on Minikube (Apply)
- LO3: Configure HPA based on CPU utilization (Apply)
- LO4: Configure HPA based on memory utilization (Apply)
- LO5: Set min/max replicas and scaling behavior (Apply)
- LO6: Test autoscaling with load generation (Apply)
- LO7: Understand scaling cooldown and stabilization windows (Understand)

**AI-Native Context**: Scale inference Pods when CPU exceeds 70%. Handle traffic spikes during peak hours. Gracefully scale down during quiet periods.

**Key Concepts**: HPA, metrics-server, target utilization, scaleUp/scaleDown behavior, stabilization window, resource metrics

**Duration**: 45 minutes

---

### Lesson 16: RBAC - Securing Agent Deployments

**Learning Objectives**:
- LO1: Explain RBAC components: ServiceAccount, Role, ClusterRole, RoleBinding, ClusterRoleBinding (Understand)
- LO2: Create ServiceAccounts for agent Pods (Apply)
- LO3: Define Roles with minimal required permissions (Apply)
- LO4: Bind Roles to ServiceAccounts (Apply)
- LO5: Audit RBAC with kubectl auth can-i (Apply)
- LO6: Understand the principle of least privilege (Understand)

**AI-Native Context**: Agent should read ConfigMaps but not Secrets from other namespaces. Restrict which APIs the agent can call. Audit what permissions a compromised agent would have.

**Key Concepts**: ServiceAccount, Role, ClusterRole, RoleBinding, ClusterRoleBinding, verbs, resources, API groups

**Duration**: 45 minutes

---

### Lesson 17: Kubernetes Security for AI Services

**Learning Objectives**:
- LO1: Configure Pods to run as non-root users (Apply)
- LO2: Set up SecurityContext for containers (Apply)
- LO3: Configure read-only root filesystem (Apply)
- LO4: Use Network Policies to isolate agent traffic (Apply)
- LO5: Understand Pod Security Standards (Restricted, Baseline, Privileged) (Understand)
- LO6: Scan container images for vulnerabilities (awareness)

**AI-Native Context**: Secure agent handling sensitive user data. Prevent agent from accessing other agents' network traffic. Harden production deployment.

**Key Concepts**: SecurityContext, runAsNonRoot, readOnlyRootFilesystem, allowPrivilegeEscalation, NetworkPolicy, Pod Security Standards

**Duration**: 45 minutes

---

### Lesson 18: Health Checks - Probes

**Learning Objectives**:
- LO1: Explain liveness, readiness, and startup probes (Understand)
- LO2: Configure HTTP GET probes (Apply)
- LO3: Configure TCP socket probes (Apply)
- LO4: Configure exec command probes (Apply)
- LO5: Set appropriate timeouts, periods, and thresholds (Apply)
- LO6: Debug probe failures (Apply)
- LO7: Design health endpoints for AI agents (Apply)

**AI-Native Context**: Agent readiness depends on model loaded in memory. Liveness check ensures agent isn't stuck. Startup probe gives slow-loading models time to initialize.

**Key Concepts**: livenessProbe, readinessProbe, startupProbe, initialDelaySeconds, periodSeconds, timeoutSeconds, failureThreshold

**Duration**: 40 minutes

---

### Lesson 20: Helm Charts for AI Agents

**Learning Objectives**:
- LO1: Explain Helm as Kubernetes package manager (Understand)
- LO2: Install applications from public Helm charts (Apply)
- LO3: Create a basic Helm chart structure for your agent (Apply)
- LO4: Use values.yaml for environment-specific configuration (Apply)
- LO5: Template Kubernetes manifests with Go templates (Apply)
- LO6: Manage releases: install, upgrade, rollback, uninstall (Apply)

**AI-Native Context**: Package agent with configurable replicas, resources, model version, and environment variables. Deploy same agent to dev/staging/prod with different values files.

**Key Concepts**: Chart, values.yaml, templates, release, helm install/upgrade/rollback, chart dependencies

**Duration**: 45 minutes

---

## Key Entities

- **Lesson**: Educational content unit with learning objectives, theory, hands-on exercises, and Try With AI section
- **Learning Objective**: Measurable outcome tied to Bloom's taxonomy level (Remember, Understand, Apply, Analyze, Evaluate, Create)
- **Code Example**: YAML manifest or kubectl command with expected output block
- **Try With AI Section**: Layer 2 practice prompts for AI collaboration using kubectl-ai or Claude

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: Chapter contains 22 lessons covering 100% of essential K8s concepts from Poulton's book
- **SC-002**: Student completing chapter can demonstrate knowledge equivalent to CKAD exam topics
- **SC-003**: All 12 new lessons have 70%+ evidence presence (code with outputs)
- **SC-004**: Capstone produces a working production-ready agent deployment with Ingress, storage, autoscaling
- **SC-005**: kubernetes-deployment skill is reusable across different agent projects
- **SC-006**: Each lesson is completable in 30-45 minutes (measured by word count and exercise complexity)

### Quality Gates

- **QG-001**: All YAML frontmatter passes schema validation (proficiency_level, learning_objectives with id/description, digcomp_mapping)
- **QG-002**: No forward references to unexplained concepts in any lesson
- **QG-003**: Framework invisibility maintained - grep finds no meta-commentary patterns
- **QG-004**: All kubectl commands tested on Minikube v1.32+
- **QG-005**: All YAML manifests are syntactically valid (kubectl apply --dry-run=client)

---

## Assumptions

1. Students have completed Chapter 49 (Docker) before starting this chapter
2. Minikube with 4GB+ RAM and 2+ CPUs is the standard local cluster
3. kubectl-ai is available and configured for AI collaboration lessons
4. Part 6 FastAPI agent is containerized and pushed to a registry
5. B1 proficiency level is appropriate for all lessons (intermediate)
6. Students have basic terminal/command-line proficiency

## Dependencies

- Chapter 49 (Docker) must be complete
- Part 6 FastAPI agent must exist and be containerized
- Minikube installation instructions must be current (v1.32+)
- kubectl-ai must be documented and installable

## Non-Goals

- Cloud-specific deployments (EKS, GKE, AKS) - covered in future cloud chapter
- Service mesh (Istio, Linkerd) - advanced topic for future
- Custom Resource Definitions (CRDs) and Operators - advanced topic
- Multi-cluster federation - advanced topic
- GPU scheduling for ML training - specialized topic
- Kubernetes API programming - developer-focused topic

---

## Implementation Notes

### Existing Lessons to Keep (renumber)

| Current # | New # | File | Action |
|-----------|-------|------|--------|
| 1 | 1 | 01-kubernetes-architecture-declarative-model.md | Keep as-is |
| 2 | 2 | 02-setting-up-minikube.md | Keep as-is |
| 3 | 3 | 03-pods-the-atomic-unit.md | Keep as-is |
| 4 | 4 | 04-deployments-self-healing-at-scale.md | Keep as-is |
| 5 | 5 | 05-services-and-networking.md | Keep as-is |
| 6 | 11 | 06-configmaps-and-secrets.md | Renumber to 11 |
| 7 | 14 | 07-resource-management-and-debugging.md | Renumber to 14 |
| 8 | 19 | 08-ai-assisted-kubernetes-kubectl-ai.md | Renumber to 19 |
| 9 | 21 | 09-capstone-deploy-your-part-6-agent.md | Renumber to 21, expand |
| 10 | 22 | 10-building-kubernetes-deployment-skill.md | Renumber to 22 |

### New Lessons to Create (12 total)

| # | Title | Priority |
|---|-------|----------|
| 6 | Init Containers: Preparing the Environment | P1 |
| 7 | Sidecar Containers: The Agent's Best Friend | P1 |
| 8 | Namespaces: Virtual Clusters for AI Workloads | P1 |
| 9 | Ingress: Exposing Your Agent to the World | P1 |
| 10 | Service Discovery Deep Dive | P2 |
| 12 | Persistent Storage: PV, PVC, StorageClass | P1 |
| 13 | StatefulSets: When Your Agent Needs Identity | P2 |
| 15 | Horizontal Pod Autoscaler: Scaling AI Inference | P1 |
| 16 | RBAC: Securing Your Agent Deployments | P2 |
| 17 | Kubernetes Security for AI Services | P2 |
| 18 | Health Checks: Liveness, Readiness, Startup Probes | P1 |
| 20 | Helm Charts for AI Agent Packaging | P3 |

### Capstone Expansion

The existing capstone (Lesson 9 -> 21) needs expansion to incorporate:
- Ingress configuration for external access
- PVC for persistent storage
- HPA for autoscaling
- Health probes for reliability
- RBAC for security
- Multiple environment deployment (dev namespace, prod namespace)
