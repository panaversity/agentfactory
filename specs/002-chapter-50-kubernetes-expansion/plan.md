# Implementation Plan: Chapter 50 Kubernetes Expansion

**Branch**: `002-chapter-50-kubernetes-expansion` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Expand Chapter 50 from 10 to 22 comprehensive lessons covering all essential Kubernetes concepts for AI-native development.

## Summary

Transform Chapter 50 into a comprehensive Kubernetes education equivalent to Nigel Poulton's "The Kubernetes Book" with an AI-native focus. The expansion adds 12 new lessons covering missing topics (Namespaces, Ingress, Persistent Storage, StatefulSets, HPA, RBAC, Security, Health Probes, Init Containers, Sidecars, Service Discovery, Helm) while preserving and renumbering 5 existing lessons.

**Pedagogical Arc**: Foundation (Lessons 1-5) → Multi-Container Patterns (6-7) → Organization & Access (8-10) → Configuration & State (11-14) → Production Readiness (15-18) → AI Collaboration & Integration (19-22)

## Technical Context

**Language/Version**: Markdown (MDX), YAML for Kubernetes manifests (v1 API)
**Primary Dependencies**: Minikube v1.32+, kubectl v1.28+, kubectl-ai, Helm v3
**Storage**: N/A (documentation project)
**Testing**: Manual validation via `kubectl apply --dry-run=client`, Minikube testing
**Target Platform**: Local development (Minikube on macOS/Linux/Windows)
**Project Type**: Educational content (22 lessons + README)
**Performance Goals**: Each lesson completable in 30-45 minutes
**Constraints**: B1 proficiency level (~7-10 concepts per lesson), all code with outputs
**Scale/Scope**: 22 lessons, ~800+ lines each, ~18,000 lines total content

## Constitution Check

*GATE: Must pass before content creation.*

| Principle | Status | Notes |
|-----------|--------|-------|
| P1: Specification Primacy | PASS | Spec exists, defines all 22 lessons |
| P2: Progressive Complexity | PASS | B1 tier, 7-10 concepts/lesson |
| P3: Factual Accuracy | TBD | All K8s commands verified against Minikube |
| P4: Coherent Structure | PASS | Foundation → Mastery arc defined |
| P5: Intelligence Accumulation | PASS | Builds on Chapter 49 Docker |
| P6: Anti-Convergence | PASS | 6 teaching modalities varied across parts |
| P7: Minimal Content | PASS | Each section maps to learning objective |
| Framework Invisibility | TBD | Validation required post-implementation |
| Evidence Presence | TBD | 70%+ code with outputs required |

## Project Structure

### Documentation (this feature)

```text
specs/002-chapter-50-kubernetes-expansion/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file
├── tasks.md             # Task breakdown (next step)
└── checklists/
    └── requirements.md  # Quality checklist (complete)
```

### Source Code (lesson files)

```text
apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/
├── README.md                                        # Chapter overview (exists)
├── 01-kubernetes-architecture-declarative-model.md  # KEEP (exists)
├── 02-setting-up-minikube.md                       # KEEP (exists)
├── 03-pods-the-atomic-unit.md                      # KEEP (exists)
├── 04-deployments-self-healing-at-scale.md         # KEEP (exists)
├── 05-services-and-networking.md                   # KEEP (exists)
├── 06-init-containers-preparing-environment.md     # NEW
├── 07-sidecar-containers-agents-best-friend.md     # NEW
├── 08-namespaces-virtual-clusters.md               # NEW
├── 09-ingress-exposing-agent-to-world.md           # NEW
├── 10-service-discovery-deep-dive.md               # NEW
├── 11-configmaps-and-secrets.md                    # RENUMBER from 06
├── 12-persistent-storage-pv-pvc.md                 # NEW
├── 13-statefulsets-agent-identity.md               # NEW
├── 14-resource-management-and-debugging.md         # RENUMBER from 07
├── 15-horizontal-pod-autoscaler.md                 # NEW
├── 16-rbac-securing-agent-deployments.md           # NEW
├── 17-kubernetes-security-ai-services.md           # NEW
├── 18-health-checks-probes.md                      # NEW
├── 19-ai-assisted-kubernetes-kubectl-ai.md         # RENUMBER from 08
├── 20-helm-charts-ai-agent-packaging.md            # NEW
├── 21-capstone-production-ready-agent.md           # RENUMBER from 09, EXPAND
└── 22-building-kubernetes-deployment-skill.md      # RENUMBER from 10
```

**Structure Decision**: Educational content structure with 22 sequential lesson files following Docusaurus conventions.

---

## Lesson-by-Lesson Plan

### Part 1: Foundation (Lessons 1-5) - EXISTING, KEEP AS-IS

These lessons already exist and provide the foundational K8s knowledge. No changes needed.

| # | Title | Stage | Modality | Duration | Status |
|---|-------|-------|----------|----------|--------|
| 1 | Kubernetes Architecture and Declarative Model | L1 | Direct Teaching | 40min | EXISTS |
| 2 | Setting Up Minikube | L1 | Hands-On | 30min | EXISTS |
| 3 | Pods: The Atomic Unit | L1 | Direct Teaching | 35min | EXISTS |
| 4 | Deployments: Self-Healing at Scale | L1 | Direct Teaching | 45min | EXISTS |
| 5 | Services and Networking | L1 | Direct Teaching | 40min | EXISTS |

---

### Part 2: Multi-Container Patterns (Lessons 6-7) - NEW

**Teaching Modality**: Hands-On Discovery
**Rationale**: Students learn container orchestration patterns by building, failing, and understanding why patterns exist.

#### Lesson 6: Init Containers - Preparing the Environment

**File**: `06-init-containers-preparing-environment.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual init container creation builds understanding of Pod startup sequences"
**Proficiency Level**: B1
**Duration**: 35 minutes
**Cognitive Load**: 7 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain when and why to use init containers vs regular containers | Understand |
| LO2 | Create init containers that run sequentially before main containers | Apply |
| LO3 | Use init containers to download ML models before agent starts | Apply |
| LO4 | Configure shared volumes between init and app containers | Apply |
| LO5 | Debug init container failures using kubectl logs and describe | Apply |

**AI-Native Context**: Download model weights from S3/GCS before agent container starts. Wait for database readiness before starting agent.

**Key Concepts**: Init container lifecycle, sequential execution, failure handling, shared volumes, initContainers spec, restartPolicy differences

**Teaching Pattern**:
1. Show problem: Agent crashes because model not downloaded
2. Introduce init containers as solution
3. Build init container step-by-step
4. Demonstrate failure scenarios and recovery
5. Try With AI: Design init container for custom use case

---

#### Lesson 7: Sidecar Containers - The Agent's Best Friend

**File**: `07-sidecar-containers-agents-best-friend.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual sidecar configuration builds understanding of multi-container Pod patterns"
**Proficiency Level**: B1
**Duration**: 40 minutes
**Cognitive Load**: 8 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain the sidecar pattern and native sidecar support in K8s 1.28+ | Understand |
| LO2 | Create native sidecars with restartPolicy: Always | Apply |
| LO3 | Configure sidecars for logging, monitoring, and proxying | Apply |
| LO4 | Share volumes between main and sidecar containers | Apply |
| LO5 | Understand sidecar startup and shutdown ordering | Understand |

**AI-Native Context**: Add logging sidecar to capture agent interactions. Metrics sidecar for inference latency tracking. Proxy sidecar for external API calls.

**Key Concepts**: Native sidecars (K8s 1.28+), spec.initContainers with restartPolicy: Always, container ordering guarantees, shared volumes, lifecycle management

**Teaching Pattern**:
1. Show problem: Agent logs scattered, hard to collect
2. Introduce sidecar pattern
3. Build logging sidecar step-by-step
4. Add metrics sidecar
5. Try With AI: Design sidecar for monitoring use case

---

### Part 3: Organization & Access (Lessons 8-10) - NEW

**Teaching Modality**: Socratic Dialogue
**Rationale**: These concepts involve decision-making (when to use namespaces? when Ingress vs LoadBalancer?). Socratic method activates reasoning.

#### Lesson 8: Namespaces - Virtual Clusters for AI Workloads

**File**: `08-namespaces-virtual-clusters.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual namespace management builds understanding of cluster organization"
**Proficiency Level**: B1
**Duration**: 40 minutes
**Cognitive Load**: 9 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain Namespaces as virtual cluster partitions | Understand |
| LO2 | Create and manage Namespaces with kubectl | Apply |
| LO3 | Deploy resources to specific Namespaces | Apply |
| LO4 | Configure ResourceQuotas to limit namespace resource usage | Apply |
| LO5 | Configure LimitRanges for default container limits | Apply |
| LO6 | Implement multi-environment strategy (dev/staging/prod) | Apply |

**AI-Native Context**: Isolate development agents from production agents. Set GPU quotas per environment. Prevent dev workloads from consuming prod resources.

**Key Concepts**: Namespace isolation, resource quotas, limit ranges, cross-namespace communication, default namespace behavior, kube-system namespace

**Teaching Pattern**:
1. Question: "What happens when dev agent consumes all cluster resources?"
2. Discovery: Namespaces as isolation mechanism
3. Question: "How do we prevent resource starvation?"
4. Discovery: ResourceQuotas and LimitRanges
5. Try With AI: Design namespace strategy for multi-team setup

---

#### Lesson 9: Ingress - Exposing Your Agent to the World

**File**: `09-ingress-exposing-agent-to-world.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual Ingress configuration builds understanding of external access patterns"
**Proficiency Level**: B1
**Duration**: 45 minutes
**Cognitive Load**: 10 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain difference between Ingress and LoadBalancer Services | Understand |
| LO2 | Install an Ingress controller (nginx-ingress on Minikube) | Apply |
| LO3 | Create Ingress resources with path-based routing | Apply |
| LO4 | Create Ingress resources with host-based routing | Apply |
| LO5 | Configure TLS termination for HTTPS | Apply |
| LO6 | Route traffic to multiple agent versions for A/B testing | Apply |

**AI-Native Context**: Expose agent API at api.myagent.com with HTTPS. Route /v1 to stable agent, /v2 to experimental agent.

**Key Concepts**: Ingress controller, Ingress resource, IngressClass, TLS secrets, path routing, host routing, annotations, nginx-ingress

**Teaching Pattern**:
1. Question: "Why can't we just use LoadBalancer for everything?"
2. Discovery: Cost, efficiency, feature limitations
3. Install Ingress controller hands-on
4. Build routing rules incrementally
5. Try With AI: Design Ingress for multi-version deployment

---

#### Lesson 10: Service Discovery Deep Dive

**File**: `10-service-discovery-deep-dive.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual DNS debugging builds understanding of service networking"
**Proficiency Level**: B1
**Duration**: 40 minutes
**Cognitive Load**: 9 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain Kubernetes DNS architecture (CoreDNS) | Understand |
| LO2 | Understand DNS naming convention: service.namespace.svc.cluster.local | Understand |
| LO3 | Debug DNS resolution failures with nslookup and dig | Apply |
| LO4 | Configure headless Services for direct Pod discovery | Apply |
| LO5 | Troubleshoot endpoint mismatches and selector issues | Apply |
| LO6 | Understand cross-namespace service discovery | Understand |

**AI-Native Context**: Agent A calling Agent B's API in different namespace. Debugging "connection refused" and "host not found" errors.

**Key Concepts**: CoreDNS, FQDN, A records, SRV records, headless Services, endpoint slices, DNS debugging, cross-namespace resolution

**Teaching Pattern**:
1. Question: "Why does Agent A get 'host not found' when calling Agent B?"
2. Hands-on DNS debugging with nslookup
3. Explore DNS naming hierarchy
4. Build headless service for direct Pod access
5. Try With AI: Troubleshoot DNS resolution scenario

---

### Part 4: Configuration & State (Lessons 11-14) - MIXED

**Teaching Modality**: Error Analysis
**Rationale**: Configuration and state management often fails. Learning through debugging builds intuition.

#### Lesson 11: ConfigMaps and Secrets - RENUMBER FROM 06

**File**: `11-configmaps-and-secrets.md`
**Teaching Stage**: 1 (Manual Foundation)
**Current File**: `06-configmaps-and-secrets.md`
**Action**: Rename file from 06 to 11, update sidebar_position to 11

---

#### Lesson 12: Persistent Storage - PV, PVC, StorageClass

**File**: `12-persistent-storage-pv-pvc.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual storage configuration builds understanding of persistent data patterns"
**Proficiency Level**: B1
**Duration**: 45 minutes
**Cognitive Load**: 10 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain the PersistentVolume (PV) and PersistentVolumeClaim (PVC) abstraction | Understand |
| LO2 | Create static PersistentVolumes | Apply |
| LO3 | Create PersistentVolumeClaims to request storage | Apply |
| LO4 | Use StorageClasses for dynamic provisioning | Apply |
| LO5 | Configure access modes (ReadWriteOnce, ReadOnlyMany, ReadWriteMany) | Apply |
| LO6 | Mount volumes into agent containers | Apply |

**AI-Native Context**: Store vector embeddings that persist across Pod restarts. Save model checkpoints. Store conversation logs for analysis.

**Key Concepts**: PV, PVC, StorageClass, access modes, volume mounts, Minikube hostpath provisioner, reclaim policies, capacity planning

**Teaching Pattern**:
1. Show problem: Agent loses embeddings on restart
2. Introduce PV/PVC abstraction
3. Create static PV, then PVC
4. Demonstrate dynamic provisioning with StorageClass
5. Try With AI: Design storage for vector database

---

#### Lesson 13: StatefulSets - When Your Agent Needs Identity

**File**: `13-statefulsets-agent-identity.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual StatefulSet configuration builds understanding of stateful workload patterns"
**Proficiency Level**: B1
**Duration**: 45 minutes
**Cognitive Load**: 9 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain when to use StatefulSets vs Deployments | Understand |
| LO2 | Understand stable network identity (pod-0, pod-1, pod-2) | Understand |
| LO3 | Understand stable storage with volumeClaimTemplates | Understand |
| LO4 | Create StatefulSets with ordered Pod creation/deletion | Apply |
| LO5 | Configure headless Services for StatefulSet DNS | Apply |
| LO6 | Perform rolling updates on StatefulSets | Apply |

**AI-Native Context**: Deploy a vector database (Qdrant, Milvus) that requires persistent identity and storage. Deploy multi-replica agent with unique identity per replica.

**Key Concepts**: Stable hostname, ordered deployment/scaling, volumeClaimTemplates, headless Service, partition updates, pod management policy

**Teaching Pattern**:
1. Question: "Why can't we use Deployment for Qdrant?"
2. Discovery: Need stable identity + storage
3. Build StatefulSet step-by-step
4. Demonstrate ordered scaling
5. Try With AI: Design StatefulSet for distributed cache

---

#### Lesson 14: Resource Management and Debugging - RENUMBER FROM 07

**File**: `14-resource-management-and-debugging.md`
**Teaching Stage**: 1 (Manual Foundation)
**Current File**: `07-resource-management-and-debugging.md`
**Action**: Rename file from 07 to 14, update sidebar_position to 14

---

### Part 5: Production Readiness (Lessons 15-18) - NEW

**Teaching Modality**: Specification-First
**Rationale**: Production patterns require thinking about requirements BEFORE implementation. Spec-first approach.

#### Lesson 15: Horizontal Pod Autoscaler - Scaling AI Inference

**File**: `15-horizontal-pod-autoscaler.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual HPA configuration builds understanding of autoscaling patterns"
**Proficiency Level**: B1
**Duration**: 45 minutes
**Cognitive Load**: 10 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain HPA and metrics-based autoscaling | Understand |
| LO2 | Install and configure metrics-server on Minikube | Apply |
| LO3 | Configure HPA based on CPU utilization | Apply |
| LO4 | Configure HPA based on memory utilization | Apply |
| LO5 | Set min/max replicas and scaling behavior | Apply |
| LO6 | Test autoscaling with load generation | Apply |
| LO7 | Understand scaling cooldown and stabilization windows | Understand |

**AI-Native Context**: Scale inference Pods when CPU exceeds 70%. Handle traffic spikes during peak hours. Gracefully scale down during quiet periods.

**Key Concepts**: HPA, metrics-server, target utilization, scaleUp/scaleDown behavior, stabilization window, resource metrics, custom metrics

**Teaching Pattern**:
1. Specify: "Agent needs 2 replicas at rest, up to 10 under load"
2. Design HPA configuration from spec
3. Implement and test with load generator
4. Observe scaling behavior
5. Try With AI: Design HPA for memory-bound workload

---

#### Lesson 16: RBAC - Securing Your Agent Deployments

**File**: `16-rbac-securing-agent-deployments.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual RBAC configuration builds understanding of access control patterns"
**Proficiency Level**: B1
**Duration**: 45 minutes
**Cognitive Load**: 10 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain RBAC components: ServiceAccount, Role, ClusterRole, RoleBinding, ClusterRoleBinding | Understand |
| LO2 | Create ServiceAccounts for agent Pods | Apply |
| LO3 | Define Roles with minimal required permissions | Apply |
| LO4 | Bind Roles to ServiceAccounts | Apply |
| LO5 | Audit RBAC with kubectl auth can-i | Apply |
| LO6 | Understand the principle of least privilege | Understand |

**AI-Native Context**: Agent should read ConfigMaps but not Secrets from other namespaces. Restrict which APIs the agent can call. Audit what permissions a compromised agent would have.

**Key Concepts**: ServiceAccount, Role, ClusterRole, RoleBinding, ClusterRoleBinding, verbs, resources, API groups, least privilege

**Teaching Pattern**:
1. Specify: "Agent needs ConfigMap read, Secret read (own namespace only)"
2. Design RBAC from security requirements
3. Implement ServiceAccount + Role + RoleBinding
4. Audit with can-i
5. Try With AI: Design RBAC for multi-tenant scenario

---

#### Lesson 17: Kubernetes Security for AI Services

**File**: `17-kubernetes-security-ai-services.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual security configuration builds understanding of production hardening"
**Proficiency Level**: B1
**Duration**: 45 minutes
**Cognitive Load**: 9 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Configure Pods to run as non-root users | Apply |
| LO2 | Set up SecurityContext for containers | Apply |
| LO3 | Configure read-only root filesystem | Apply |
| LO4 | Use Network Policies to isolate agent traffic | Apply |
| LO5 | Understand Pod Security Standards (Restricted, Baseline, Privileged) | Understand |
| LO6 | Scan container images for vulnerabilities (awareness) | Understand |

**AI-Native Context**: Secure agent handling sensitive user data. Prevent agent from accessing other agents' network traffic. Harden production deployment.

**Key Concepts**: SecurityContext, runAsNonRoot, readOnlyRootFilesystem, allowPrivilegeEscalation, NetworkPolicy, Pod Security Standards

**Teaching Pattern**:
1. Specify: "Agent must run non-root, read-only fs, isolated network"
2. Design SecurityContext from requirements
3. Implement step-by-step
4. Add NetworkPolicy
5. Try With AI: Audit security of existing deployment

---

#### Lesson 18: Health Checks - Liveness, Readiness, Startup Probes

**File**: `18-health-checks-probes.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual probe configuration builds understanding of container health patterns"
**Proficiency Level**: B1
**Duration**: 40 minutes
**Cognitive Load**: 10 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain liveness, readiness, and startup probes | Understand |
| LO2 | Configure HTTP GET probes | Apply |
| LO3 | Configure TCP socket probes | Apply |
| LO4 | Configure exec command probes | Apply |
| LO5 | Set appropriate timeouts, periods, and thresholds | Apply |
| LO6 | Debug probe failures | Apply |
| LO7 | Design health endpoints for AI agents | Apply |

**AI-Native Context**: Agent readiness depends on model loaded in memory. Liveness check ensures agent isn't stuck. Startup probe gives slow-loading models time to initialize.

**Key Concepts**: livenessProbe, readinessProbe, startupProbe, initialDelaySeconds, periodSeconds, timeoutSeconds, failureThreshold, successThreshold

**Teaching Pattern**:
1. Show problem: Agent receives traffic before model loads
2. Design probes for AI agent lifecycle
3. Implement all three probe types
4. Test failure scenarios
5. Try With AI: Design probes for custom agent

---

### Part 6: AI Collaboration & Integration (Lessons 19-22) - MIXED

**Teaching Modality**: Collaborative Debugging + Spec-Driven Integration
**Rationale**: Final section brings together all concepts with AI collaboration and capstone project.

#### Lesson 19: AI-Assisted Kubernetes - kubectl-ai - RENUMBER FROM 08

**File**: `19-ai-assisted-kubernetes-kubectl-ai.md`
**Teaching Stage**: 2 (AI Collaboration)
**Current File**: `08-ai-assisted-kubernetes-kubectl-ai.md`
**Action**: Rename file from 08 to 19, update sidebar_position to 19

---

#### Lesson 20: Helm Charts for AI Agent Packaging

**File**: `20-helm-charts-ai-agent-packaging.md`
**Teaching Stage**: 1 (Manual Foundation)
**Stage Name**: "Manual Foundation"
**Stage Description**: "Manual Helm chart creation builds understanding of package management"
**Proficiency Level**: B1
**Duration**: 45 minutes
**Cognitive Load**: 10 concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Explain Helm as Kubernetes package manager | Understand |
| LO2 | Install applications from public Helm charts | Apply |
| LO3 | Create a basic Helm chart structure for your agent | Apply |
| LO4 | Use values.yaml for environment-specific configuration | Apply |
| LO5 | Template Kubernetes manifests with Go templates | Apply |
| LO6 | Manage releases: install, upgrade, rollback, uninstall | Apply |

**AI-Native Context**: Package agent with configurable replicas, resources, model version, and environment variables. Deploy same agent to dev/staging/prod with different values files.

**Key Concepts**: Chart, values.yaml, templates, release, helm install/upgrade/rollback, chart dependencies, named templates

**Teaching Pattern**:
1. Show problem: Deploying same agent to 3 environments manually
2. Introduce Helm as solution
3. Install public chart (redis)
4. Create custom agent chart step-by-step
5. Try With AI: Extend chart with new parameters

---

#### Lesson 21: Capstone - Production-Ready Part 6 Agent - RENUMBER FROM 09, EXPAND

**File**: `21-capstone-production-ready-agent.md`
**Teaching Stage**: 4 (Spec-Driven Integration)
**Current File**: `09-capstone-deploy-your-part-6-agent.md`
**Action**: Rename file from 09 to 21, update sidebar_position to 21, SIGNIFICANT EXPANSION

**Expansion Requirements**:
The capstone must now incorporate ALL new concepts:
- Namespace isolation (dev/prod)
- Init containers (model download)
- Sidecar containers (logging)
- Ingress (external access)
- Persistent Storage (embeddings)
- HPA (autoscaling)
- Health Probes (liveness, readiness, startup)
- RBAC (secure deployment)
- Security hardening (non-root, read-only fs)

**Proficiency Level**: B1
**Duration**: 90 minutes
**Cognitive Load**: Synthesis of all concepts

**Learning Objectives**:
| ID | Description | Bloom Level |
|----|-------------|-------------|
| LO1 | Write specification for production K8s deployment | Create |
| LO2 | Deploy agent to isolated namespace with resource quotas | Apply |
| LO3 | Configure init container for model download | Apply |
| LO4 | Add logging sidecar container | Apply |
| LO5 | Create Ingress for external API access | Apply |
| LO6 | Configure PVC for persistent embeddings | Apply |
| LO7 | Set up HPA for autoscaling | Apply |
| LO8 | Configure all three health probes | Apply |
| LO9 | Implement RBAC with least privilege | Apply |
| LO10 | Apply security hardening | Apply |
| LO11 | Validate complete deployment | Evaluate |

---

#### Lesson 22: Building a Kubernetes Deployment Skill - RENUMBER FROM 10

**File**: `22-building-kubernetes-deployment-skill.md`
**Teaching Stage**: 3 (Intelligence Design)
**Current File**: `10-building-kubernetes-deployment-skill.md`
**Action**: Rename file from 10 to 22, update sidebar_position to 22

---

## Intelligence Creation Opportunities

### Lesson 22: kubernetes-deployment Skill

**Skill Name**: kubernetes-deployment
**Location**: `.claude/skills/kubernetes-deployment/SKILL.md`

**Persona**: "Think like a Kubernetes platform engineer designing production-ready deployments for AI services."

**Questions to Ask**:
1. What resources does this agent need (CPU, memory, GPU)?
2. Does the agent need persistent storage? What access mode?
3. How should the agent scale? What metrics trigger scaling?
4. What external access is required (Ingress, LoadBalancer, ClusterIP only)?
5. What secrets/configs does the agent need?
6. What security constraints apply (non-root, network isolation)?

**Principles**:
- Start with least privilege, add permissions as needed
- Every container must have resource limits
- Every deployment must have health probes
- Storage class should match durability requirements
- Ingress annotations depend on controller (nginx, traefik, etc.)

---

## Capstone Composition Strategy

### Accumulated Intelligence for Lesson 21

The capstone composes skills and patterns from ALL previous lessons:

| Lesson | Pattern/Skill Applied in Capstone |
|--------|----------------------------------|
| 1-2 | Declarative model, Minikube cluster |
| 3 | Pod structure with labels |
| 4 | Deployment with replicas |
| 5 | ClusterIP Service |
| 6 | Init container for model download |
| 7 | Sidecar for logging |
| 8 | Namespace isolation |
| 9 | Ingress for external access |
| 10 | DNS for inter-service calls |
| 11 | ConfigMap for config, Secret for API keys |
| 12 | PVC for embeddings storage |
| 13 | (Optional) StatefulSet if agent needs identity |
| 14 | Resource limits, debugging |
| 15 | HPA for autoscaling |
| 16 | RBAC for security |
| 17 | SecurityContext hardening |
| 18 | Health probes |
| 19 | kubectl-ai for troubleshooting |
| 20 | (Optional) Helm chart packaging |

### Success Criteria for Capstone

1. **Specification First**: Student writes spec.md BEFORE any YAML
2. **All Components Present**: Namespace, Deployment, Service, Ingress, ConfigMap, Secret, PVC, HPA, RBAC, SecurityContext, Probes
3. **External Access Works**: Agent API accessible via Ingress URL
4. **Self-Healing Works**: Delete Pod, agent recovers automatically
5. **Storage Persists**: Delete Pod, data survives in PVC
6. **Scaling Works**: HPA scales under load
7. **Security Audit Passes**: Non-root, read-only fs, least privilege RBAC

---

## Task Summary

### Phase 1: File Operations (Renumbering)
- Rename 5 existing files (06→11, 07→14, 08→19, 09→21, 10→22)
- Update sidebar_position in frontmatter

### Phase 2: New Lesson Creation (12 lessons)
- Create lessons 6, 7, 8, 9, 10, 12, 13, 15, 16, 17, 18, 20
- Each ~800 lines with full YAML frontmatter
- Each ends with "Try With AI" section
- Evidence presence: 70%+ code with outputs

### Phase 3: Capstone Expansion
- Significantly expand lesson 21 with all new concepts
- Ensure spec-driven approach demonstrated
- Validate all components work together

### Phase 4: Skill Creation
- Create kubernetes-deployment skill
- Following Persona + Questions + Principles pattern

### Phase 5: Validation
- Constitutional compliance check
- Evidence presence validation
- MDX safety check (angle brackets)
- Framework invisibility check

---

## Complexity Tracking

No constitution violations requiring justification. All content follows B1 complexity tier with appropriate scaffolding.
