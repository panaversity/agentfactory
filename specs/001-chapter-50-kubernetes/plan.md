# Implementation Plan: Chapter 50 - Kubernetes for AI Services

**Branch**: `001-chapter-50-kubernetes` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-chapter-50-kubernetes/spec.md`

## Summary

Create Chapter 50 teaching Kubernetes fundamentals for deploying AI agents. The chapter follows the 4-Layer Teaching Method with 10 lessons progressing from manual foundation (architecture, pods, deployments, services, configmaps, debugging) through AI-assisted workflows to a spec-driven capstone deploying the student's Part 6 FastAPI agent. The chapter concludes with creating a reusable kubernetes-deployment skill.

## Technical Context

**Content Type**: Educational chapter (MDX/Markdown lessons)
**Target Platform**: Docusaurus 3.x documentation site
**Prerequisites**: Chapter 49 (Docker), Part 6 (FastAPI agent)
**Testing**: Manual validation, MDX compilation, constitutional compliance
**Kubernetes Version**: v1.28+ (current stable)
**Minikube Version**: v1.32+ (current stable)
**kubectl-ai**: Latest available version for AI-assisted workflows

## Constitution Check

*GATE: Educational content must comply with constitution v6.0.1*

| Principle | Status | Notes |
|-----------|--------|-------|
| 4-Layer Teaching Method | ✅ PASS | L1 (1-7), L2 (8), L4 (9), L3 (10) |
| Framework Invisibility | ✅ PASS | No meta-commentary about "AI as Teacher" |
| Evidence Presence | ✅ PLAN | 70%+ code examples with output |
| Structural Compliance | ✅ PLAN | Each lesson ends with "Try With AI" |
| MDX Safety | ✅ PLAN | Validate no angle brackets before letters |
| Assumed Knowledge | ✅ PASS | Section in spec defines prereqs |

## Project Structure

### Documentation (this feature)

```text
specs/001-chapter-50-kubernetes/
├── spec.md              # Feature specification
├── plan.md              # This file
├── tasks.md             # Task breakdown (created by /sp.tasks)
└── checklists/
    └── requirements.md  # Specification checklist
```

### Content Output (repository docs)

```text
apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/
├── README.md                                    # Chapter overview
├── 01-kubernetes-architecture-declarative-model.md  # L1: Architecture
├── 02-setting-up-minikube.md                   # L1: Setup
├── 03-pods-the-atomic-unit.md                  # L1: Pods
├── 04-deployments-self-healing-at-scale.md     # L1: Deployments
├── 05-services-and-networking.md               # L1: Services
├── 06-configmaps-and-secrets.md                # L1: Configuration
├── 07-resource-management-and-debugging.md     # L1: Resources & Debug
├── 08-ai-assisted-kubernetes-kubectl-ai.md     # L2: AI Collaboration
├── 09-capstone-deploy-your-part-6-agent.md     # L4: Spec-Driven
└── 10-building-kubernetes-deployment-skill.md  # L3: Intelligence
```

**Structure Decision**: Educational chapter with 10 lesson files plus README in Docusaurus docs structure.

---

## Lesson Implementation Plans

### Lesson 1: Kubernetes Architecture and the Declarative Model

**Layer**: L1 (Manual Foundation)
**Duration**: 35 minutes
**File**: `01-kubernetes-architecture-declarative-model.md`

**Learning Objectives**:
- Explain why container orchestration exists (Docker limitations)
- Describe the declarative model (desired vs observed state)
- Identify control plane components (API server, scheduler, etcd, controllers)
- Identify worker node components (kubelet, kube-proxy, container runtime)
- Explain the reconciliation loop

**Key Concepts**:
1. From Docker to orchestration: Why "docker run" isn't enough
2. The declarative model: You describe WHAT, Kubernetes figures out HOW
3. Control plane architecture: The "brains" of the cluster
4. Worker nodes: The "muscles" running workloads
5. The watch-loop pattern: Continuous reconciliation

**Teaching Approach**:
- Analogy: Kubernetes as an operating system for containers
- Diagram: Control plane vs worker nodes
- Scenario: What happens when a container crashes?

---

### Lesson 2: Setting Up Minikube

**Layer**: L1 (Manual Foundation)
**Duration**: 25 minutes
**File**: `02-setting-up-minikube.md`

**Learning Objectives**:
- Install Minikube on macOS, Windows, or Linux
- Start a local Kubernetes cluster
- Verify cluster health with kubectl
- Access the Kubernetes dashboard
- Understand kubectl context and configuration

**Key Concepts**:
1. What is Minikube? (Local K8s for learning)
2. Installation by platform
3. Starting the cluster: `minikube start`
4. kubectl basics: `kubectl cluster-info`, `kubectl get nodes`
5. Dashboard access: `minikube dashboard`

**Teaching Approach**:
- Platform-specific installation tabs
- Verification commands with expected output
- Troubleshooting common issues

---

### Lesson 3: Pods - The Atomic Unit

**Layer**: L1 (Manual Foundation)
**Duration**: 40 minutes
**File**: `03-pods-the-atomic-unit.md`

**Learning Objectives**:
- Define what a Pod is and why it exists
- Write a Pod manifest in YAML
- Deploy a Pod with kubectl apply
- Inspect Pod status and logs
- Understand Pod lifecycle and networking

**Key Concepts**:
1. Pod abstraction: Why wrap containers?
2. Pod YAML anatomy: kind, apiVersion, metadata, spec
3. Deploying: `kubectl apply -f pod.yaml`
4. Inspection: `kubectl get pods`, `kubectl describe pod`
5. Logs: `kubectl logs <pod>`
6. Pod networking: Shared localhost, ephemeral IPs

**Teaching Approach**:
- Analogy: Pod as apartment, containers as roommates
- Hands-on: Deploy nginx Pod, inspect, view logs
- Key insight: Pods are mortal (ephemeral)

---

### Lesson 4: Deployments - Self-Healing at Scale

**Layer**: L1 (Manual Foundation)
**Duration**: 45 minutes
**File**: `04-deployments-self-healing-at-scale.md`

**Learning Objectives**:
- Explain why direct Pod deployment is rare
- Create a Deployment manifest
- Understand ReplicaSets and replica management
- Perform rolling updates and rollbacks
- Demonstrate self-healing behavior

**Key Concepts**:
1. Controllers and the declarative model
2. Deployment YAML: replicas, selector, template
3. ReplicaSets: The controller behind Deployments
4. Rolling updates: `kubectl set image`
5. Rollbacks: `kubectl rollout undo`
6. Self-healing demo: Delete a Pod, watch it return

**Teaching Approach**:
- Analogy: Deployment as manager ensuring worker count
- Hands-on: Create Deployment, scale, update, rollback
- Intentional failure: Delete Pod, observe recreation

---

### Lesson 5: Services and Networking

**Layer**: L1 (Manual Foundation)
**Duration**: 40 minutes
**File**: `05-services-and-networking.md`

**Learning Objectives**:
- Explain why Services exist (Pod IP ephemerality)
- Create ClusterIP, NodePort, and LoadBalancer Services
- Understand label selectors for Pod targeting
- Access applications through Services
- Explain Kubernetes DNS

**Key Concepts**:
1. The networking problem: Pods die, IPs change
2. Service types: ClusterIP, NodePort, LoadBalancer
3. Label selectors: How Services find Pods
4. Service YAML: selector, ports, type
5. DNS discovery: `<service>.<namespace>.svc.cluster.local`

**Teaching Approach**:
- Analogy: Service as phone number routing to on-duty worker
- Hands-on: Expose Deployment via NodePort, access in browser
- Debugging: Service selector mismatch scenario

---

### Lesson 6: ConfigMaps and Secrets

**Layer**: L1 (Manual Foundation)
**Duration**: 35 minutes
**File**: `06-configmaps-and-secrets.md`

**Learning Objectives**:
- Create ConfigMaps for application configuration
- Create Secrets for sensitive data
- Inject configuration as environment variables
- Mount configuration as files
- Understand Secret encoding (base64 != encryption)

**Key Concepts**:
1. Configuration vs code separation
2. ConfigMap creation: `kubectl create configmap`
3. Secret creation: `kubectl create secret generic`
4. Environment variable injection: `envFrom`, `valueFrom`
5. Volume mounts for file-based config
6. Security note: base64 is encoding, not encryption

**Teaching Approach**:
- Scenario: Inject API key without rebuilding image
- Hands-on: Create ConfigMap, reference in Deployment
- Security awareness: Examine Secret, see base64

---

### Lesson 7: Resource Management and Debugging

**Layer**: L1 (Manual Foundation)
**Duration**: 40 minutes
**File**: `07-resource-management-and-debugging.md`

**Learning Objectives**:
- Set resource requests and limits
- Understand QoS classes (Guaranteed, Burstable, BestEffort)
- Debug common Pod failures (CrashLoopBackOff, ImagePullBackOff, Pending)
- Use kubectl exec for interactive debugging
- Read cluster events

**Key Concepts**:
1. Resource requests: Guaranteed minimums for scheduling
2. Resource limits: Maximum allowed consumption
3. QoS classes: How Kubernetes prioritizes Pods
4. Debugging commands: `kubectl describe`, `kubectl logs`, `kubectl exec`
5. Common failures: CrashLoopBackOff, ImagePullBackOff, Pending, OOMKilled
6. Events: `kubectl get events`

**Teaching Approach**:
- Intentional failures: Deploy broken manifests, diagnose
- Hands-on: Fix 3 different failure scenarios
- Pattern: Describe → Logs → Events → Exec

---

### Lesson 8: AI-Assisted Kubernetes with kubectl-ai

**Layer**: L2 (AI Collaboration)
**Duration**: 45 minutes
**File**: `08-ai-assisted-kubernetes-kubectl-ai.md`

**Learning Objectives**:
- Use kubectl-ai for natural language to manifest generation
- Debug cluster issues through AI dialogue
- Iterate on configurations with AI assistance
- Evaluate AI-generated manifests using L1 knowledge
- Apply Three Roles through action (invisible framework)

**Key Concepts**:
1. kubectl-ai installation and setup
2. Natural language to YAML: "Deploy my-app with 3 replicas"
3. Debugging dialogue: Describe symptoms, get suggestions
4. Iterative refinement: "Add health checks", "Increase memory"
5. Critical evaluation: Why L1 foundation matters

**Teaching Approach**:
- Action prompts (not meta-commentary)
- AI suggests patterns → Student evaluates → Together refine
- Example dialogue showing iteration
- "Try With AI" section with specific prompts

---

### Lesson 9: Capstone - Deploy Your Part 6 Agent

**Layer**: L4 (Spec-Driven)
**Duration**: 50 minutes
**File**: `09-capstone-deploy-your-part-6-agent.md`

**Learning Objectives**:
- Write a deployment specification for their agent
- Generate complete manifest suite from spec
- Configure environment (ConfigMaps, Secrets)
- Deploy and validate on Minikube
- Demonstrate self-healing and external access

**Key Concepts**:
1. Specification-first: Define WHAT before generating HOW
2. Full manifest suite: Deployment + Service + ConfigMap + Secret
3. Health checks: Liveness and readiness probes
4. Validation: Send requests through Service
5. Self-healing proof: Delete Pod, verify recovery

**Teaching Approach**:
- Students write spec first (natural language requirements)
- AI generates manifests from spec
- Deploy their actual Part 6 agent
- Validation checklist: Can access? Self-heals? Logs work?

---

### Lesson 10: Building the Kubernetes Deployment Skill

**Layer**: L3 (Intelligence Design)
**Duration**: 35 minutes
**File**: `10-building-kubernetes-deployment-skill.md`

**Learning Objectives**:
- Extract deployment knowledge into reusable skill
- Structure skill with Persona + Questions + Principles
- Include best practices as principles
- Test skill on a different application
- Understand cross-project skill value

**Key Concepts**:
1. From one-time knowledge to reusable intelligence
2. Skill structure: Persona, Questions, Principles
3. Kubernetes deployment principles (resource limits, probes, labels)
4. Testing the skill on new applications
5. Intelligence accumulation across projects

**Teaching Approach**:
- Guide skill creation step-by-step
- Include chapter's best practices as principles
- Test on hypothetical new application
- Reflect on value of accumulated intelligence

---

## Quality Gates

| Gate | Validation Method | Status |
|------|------------------|--------|
| MDX Compilation | `docusaurus build` | Pending |
| Framework Invisibility | `grep -i "What to notice\|AI.*teach\|AI.*learn"` | Pending |
| Evidence Presence | Manual review: 70%+ code has output | Pending |
| Structural Compliance | Each lesson ends with "Try With AI" | Pending |
| Constitutional Compliance | educational-validator agent | Pending |

## Dependencies

| Dependency | Type | Status |
|------------|------|--------|
| Chapter 49 (Docker) | Content prereq | Exists |
| Part 6 FastAPI Agent | Student artifact | Assumed |
| Minikube v1.32+ | Tool | Documented |
| kubectl-ai | Tool | Documented |
| Kubernetes Book knowledge | Domain source | Read |

## Implementation Approach

1. **Content-first**: Write lessons focusing on teaching effectiveness
2. **Evidence-driven**: Include output for every command
3. **Thread-continuity**: Reference Part 6 agent throughout
4. **Validation-gated**: Run educational-validator before filesystem write
5. **MDX-safe**: Avoid angle brackets before alphanumerics
