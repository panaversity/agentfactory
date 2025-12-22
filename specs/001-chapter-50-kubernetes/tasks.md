# Task Breakdown: Chapter 50 - Kubernetes for AI Services

**Feature**: `001-chapter-50-kubernetes`
**Date**: 2025-12-22
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Phase 0: Setup

- [x] **T0.1**: Create chapter directory structure at `apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/`
- [x] **T0.2**: Read 2-3 existing Part 7 chapters for voice/level calibration (Chapter 49 Docker required)

## Phase 1: Layer 1 Lessons (Manual Foundation)

### Lesson 1: Kubernetes Architecture and the Declarative Model

- [x] **T1.1**: Write lesson 1 covering:
  - Why container orchestration exists (Docker limitations in production)
  - The declarative model (desired state vs observed state)
  - Control plane components (API server, scheduler, etcd, controllers)
  - Worker node components (kubelet, kube-proxy, container runtime)
  - The reconciliation loop
  - Include architecture diagram description
  - End with "Try With AI" section
- [x] **T1.2**: Validate lesson 1 (constitutional compliance, MDX safety)

### Lesson 2: Setting Up Minikube

- [x] **T2.1**: Write lesson 2 covering:
  - What is Minikube (local K8s for learning)
  - Platform-specific installation (macOS, Windows, Linux)
  - Starting cluster: `minikube start`
  - kubectl basics: `cluster-info`, `get nodes`
  - Dashboard access: `minikube dashboard`
  - Include expected output for all commands
  - End with "Try With AI" section
- [x] **T2.2**: Validate lesson 2

### Lesson 3: Pods - The Atomic Unit

- [x] **T3.1**: Write lesson 3 covering:
  - Pod abstraction: Why wrap containers?
  - Pod YAML anatomy: kind, apiVersion, metadata, spec
  - Deploying: `kubectl apply -f pod.yaml`
  - Inspection: `kubectl get pods`, `kubectl describe pod`
  - Logs: `kubectl logs <pod>`
  - Pod networking: Shared localhost, ephemeral IPs
  - Analogy: Pod as apartment, containers as roommates
  - End with "Try With AI" section
- [x] **T3.2**: Validate lesson 3

### Lesson 4: Deployments - Self-Healing at Scale

- [x] **T4.1**: Write lesson 4 covering:
  - Controllers and the declarative model
  - Deployment YAML: replicas, selector, template
  - ReplicaSets: The controller behind Deployments
  - Rolling updates: `kubectl set image`
  - Rollbacks: `kubectl rollout undo`
  - Self-healing demo: Delete Pod, watch recreation
  - Analogy: Deployment as manager ensuring worker count
  - End with "Try With AI" section
- [x] **T4.2**: Validate lesson 4

### Lesson 5: Services and Networking

- [x] **T5.1**: Write lesson 5 covering:
  - The networking problem: Pods die, IPs change
  - Service types: ClusterIP, NodePort, LoadBalancer
  - Label selectors: How Services find Pods
  - Service YAML: selector, ports, type
  - DNS discovery: `<service>.<namespace>.svc.cluster.local`
  - Analogy: Service as phone number routing to on-duty worker
  - End with "Try With AI" section
- [x] **T5.2**: Validate lesson 5

### Lesson 6: ConfigMaps and Secrets

- [x] **T6.1**: Write lesson 6 covering:
  - Configuration vs code separation
  - ConfigMap creation: `kubectl create configmap`
  - Secret creation: `kubectl create secret generic`
  - Environment variable injection: `envFrom`, `valueFrom`
  - Volume mounts for file-based config
  - Security note: base64 is encoding, NOT encryption
  - End with "Try With AI" section
- [x] **T6.2**: Validate lesson 6

### Lesson 7: Resource Management and Debugging

- [x] **T7.1**: Write lesson 7 covering:
  - Resource requests: Guaranteed minimums for scheduling
  - Resource limits: Maximum allowed consumption
  - QoS classes: Guaranteed, Burstable, BestEffort
  - Debugging commands: describe, logs, exec, events
  - Common failures: CrashLoopBackOff, ImagePullBackOff, Pending, OOMKilled
  - Intentional failure exercises (3 scenarios to fix)
  - Debugging pattern: Describe → Logs → Events → Exec
  - End with "Try With AI" section
- [x] **T7.2**: Validate lesson 7

## Phase 2: Layer 2 Lesson (AI Collaboration)

### Lesson 8: AI-Assisted Kubernetes with kubectl-ai

- [x] **T8.1**: Write lesson 8 covering:
  - kubectl-ai installation and setup
  - Natural language to YAML generation
  - Debugging dialogue: Describe symptoms, get suggestions
  - Iterative refinement through conversation
  - Critical evaluation: Why L1 foundation enables quality judgment
  - Three Roles demonstrated through ACTION (invisible framework)
  - Specific action prompts for student practice
  - End with "Try With AI" section
- [x] **T8.2**: Validate lesson 8 (especially framework invisibility)

## Phase 3: Layer 4 Lesson (Spec-Driven Capstone)

### Lesson 9: Capstone - Deploy Your Part 6 Agent

- [x] **T9.1**: Write lesson 9 covering:
  - Specification-first approach: Define WHAT before generating HOW
  - Writing deployment specification in natural language
  - Full manifest suite: Deployment + Service + ConfigMap + Secret
  - Health checks: Liveness and readiness probes
  - Deploying to Minikube
  - Validation checklist: Access, self-healing, logs
  - End with "Try With AI" section
- [x] **T9.2**: Validate lesson 9

## Phase 4: Layer 3 Lesson (Intelligence Design)

### Lesson 10: Building the Kubernetes Deployment Skill

- [x] **T10.1**: Write lesson 10 covering:
  - From one-time knowledge to reusable intelligence
  - Skill structure: Persona + Questions + Principles
  - Kubernetes deployment principles as skill content
  - Testing skill on new application
  - Cross-project value of accumulated intelligence
  - End with "Try With AI" section
- [x] **T10.2**: Validate lesson 10

## Phase 5: Chapter Finalization

- [x] **T11.1**: Write README.md chapter overview
- [x] **T11.2**: Run full chapter validation (educational-validator on all lessons)
- [x] **T11.3**: Verify MDX compilation (`grep -E '<[a-zA-Z0-9]' *.md` returns empty) - Note: Angle brackets found are within code blocks (valid MDX)
- [x] **T11.4**: Update chapter-index.md if needed - No chapter-index.md found in repo

## Task Summary

| Phase | Tasks | Type |
|-------|-------|------|
| Phase 0 | T0.1-T0.2 | Setup |
| Phase 1 | T1.1-T7.2 | L1 Content (14 tasks) |
| Phase 2 | T8.1-T8.2 | L2 Content (2 tasks) |
| Phase 3 | T9.1-T9.2 | L4 Content (2 tasks) |
| Phase 4 | T10.1-T10.2 | L3 Content (2 tasks) |
| Phase 5 | T11.1-T11.4 | Finalization (4 tasks) |
| **Total** | **26 tasks** | |

## Execution Notes

1. **Parallel Execution**: Lessons 1-7 can be parallelized after T0.2 (reading existing chapters)
2. **Sequential Dependency**: Lesson 8 requires L1 foundation concepts from 1-7
3. **Sequential Dependency**: Lesson 9 (capstone) requires all prior lessons
4. **Sequential Dependency**: Lesson 10 (skill) requires capstone completion
5. **Validation Gate**: Each lesson must pass educational-validator before filesystem write
6. **MDX Safety**: Run `grep -E '<[a-zA-Z0-9]'` on each lesson to catch angle bracket issues
