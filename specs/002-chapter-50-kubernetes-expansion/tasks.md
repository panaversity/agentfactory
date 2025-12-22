# Tasks: Chapter 50 Kubernetes Expansion

**Input**: Design documents from `/specs/002-chapter-50-kubernetes-expansion/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Organization**: Tasks grouped by implementation phase. Educational content project - no tests needed.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different lesson files, no dependencies)
- Include exact file paths in descriptions

## Path Conventions

- **Lesson files**: `apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/`
- **Skill files**: `.claude/skills/kubernetes-deployment/`

---

## Phase 1: Setup (File Renumbering)

**Purpose**: Renumber existing lessons to make room for new lessons

- [ ] T001 Rename `06-configmaps-and-secrets.md` to `11-configmaps-and-secrets.md` in apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/
- [ ] T002 Update sidebar_position from 6 to 11 in 11-configmaps-and-secrets.md frontmatter
- [ ] T003 Rename `07-resource-management-and-debugging.md` to `14-resource-management-and-debugging.md`
- [ ] T004 Update sidebar_position from 7 to 14 in 14-resource-management-and-debugging.md frontmatter
- [ ] T005 Rename `08-ai-assisted-kubernetes-kubectl-ai.md` to `19-ai-assisted-kubernetes-kubectl-ai.md`
- [ ] T006 Update sidebar_position from 8 to 19 in 19-ai-assisted-kubernetes-kubectl-ai.md frontmatter
- [ ] T007 Rename `09-capstone-deploy-your-part-6-agent.md` to `21-capstone-production-ready-agent.md`
- [ ] T008 Update sidebar_position from 9 to 21, update title in 21-capstone-production-ready-agent.md frontmatter
- [ ] T009 Rename `10-building-kubernetes-deployment-skill.md` to `22-building-kubernetes-deployment-skill.md`
- [ ] T010 Update sidebar_position from 10 to 22 in 22-building-kubernetes-deployment-skill.md frontmatter

**Checkpoint**: All existing lessons renumbered. Directory should have files 01-05 and 11, 14, 19, 21, 22.

---

## Phase 2: Part 2 - Multi-Container Patterns (Lessons 6-7)

**Purpose**: Teach init containers and sidecar patterns for AI agents

### Lesson 6: Init Containers

- [ ] T011 [P] Create `06-init-containers-preparing-environment.md` with full YAML frontmatter (sidebar_position: 6, chapter: 50, lesson: 6, duration_minutes: 35, title, proficiency_level: B1, teaching_stage: 1, stage_name, stage_description, cognitive_load, learning_objectives with 5 LOs, digcomp_mapping)
- [ ] T012 [P] Write Lesson 6 introduction: Problem statement (agent crashes without model), init container as solution
- [ ] T013 [P] Write Lesson 6 theory: Init container lifecycle, sequential execution, failure handling
- [ ] T014 [P] Write Lesson 6 hands-on: Create Pod with init container that downloads model, include kubectl apply with output
- [ ] T015 [P] Write Lesson 6 debugging section: kubectl logs -c init-container-name, kubectl describe pod with init container status
- [ ] T016 [P] Write Lesson 6 "Try With AI" section: Design init container for custom model download scenario

### Lesson 7: Sidecar Containers

- [ ] T017 [P] Create `07-sidecar-containers-agents-best-friend.md` with full YAML frontmatter (sidebar_position: 7, lesson: 7, duration_minutes: 40, 5 LOs, digcomp_mapping)
- [ ] T018 [P] Write Lesson 7 introduction: Problem (scattered logs), sidecar pattern as solution
- [ ] T019 [P] Write Lesson 7 theory: Native sidecars (K8s 1.28+), restartPolicy: Always, startup/shutdown ordering
- [ ] T020 [P] Write Lesson 7 hands-on: Create Pod with logging sidecar, shared volume, include kubectl apply with output
- [ ] T021 [P] Write Lesson 7 metrics sidecar example: Add prometheus metrics sidecar
- [ ] T022 [P] Write Lesson 7 "Try With AI" section: Design sidecar for monitoring agent inference latency

**Checkpoint**: Lessons 6-7 complete. Multi-container patterns taught.

---

## Phase 3: Part 3 - Organization & Access (Lessons 8-10)

**Purpose**: Teach namespaces, Ingress, and service discovery

### Lesson 8: Namespaces

- [ ] T023 [P] Create `08-namespaces-virtual-clusters.md` with full YAML frontmatter (sidebar_position: 8, lesson: 8, duration_minutes: 40, 6 LOs, digcomp_mapping)
- [ ] T024 [P] Write Lesson 8 introduction: Socratic question "What happens when dev agent consumes all resources?"
- [ ] T025 [P] Write Lesson 8 theory: Namespace isolation, resource quotas, limit ranges
- [ ] T026 [P] Write Lesson 8 hands-on: Create dev/staging/prod namespaces with quotas, include kubectl output
- [ ] T027 [P] Write Lesson 8 multi-environment strategy: Deploy same agent to different namespaces
- [ ] T028 [P] Write Lesson 8 "Try With AI" section: Design namespace strategy for multi-team setup

### Lesson 9: Ingress

- [ ] T029 [P] Create `09-ingress-exposing-agent-to-world.md` with full YAML frontmatter (sidebar_position: 9, lesson: 9, duration_minutes: 45, 6 LOs, digcomp_mapping)
- [ ] T030 [P] Write Lesson 9 introduction: Socratic question "Why not use LoadBalancer for everything?"
- [ ] T031 [P] Write Lesson 9 theory: Ingress controller vs Ingress resource, IngressClass, annotations
- [ ] T032 [P] Write Lesson 9 hands-on: Install nginx-ingress on Minikube with minikube addons enable ingress
- [ ] T033 [P] Write Lesson 9 path-based routing example: /v1 → stable, /v2 → experimental
- [ ] T034 [P] Write Lesson 9 TLS configuration: Create TLS secret, configure Ingress for HTTPS
- [ ] T035 [P] Write Lesson 9 "Try With AI" section: Design Ingress for multi-version deployment

### Lesson 10: Service Discovery Deep Dive

- [ ] T036 [P] Create `10-service-discovery-deep-dive.md` with full YAML frontmatter (sidebar_position: 10, lesson: 10, duration_minutes: 40, 6 LOs, digcomp_mapping)
- [ ] T037 [P] Write Lesson 10 introduction: Socratic question "Why 'host not found' when calling Agent B?"
- [ ] T038 [P] Write Lesson 10 theory: CoreDNS architecture, FQDN naming convention (service.namespace.svc.cluster.local)
- [ ] T039 [P] Write Lesson 10 hands-on: DNS debugging with nslookup from debug pod
- [ ] T040 [P] Write Lesson 10 headless services: Configure headless service for direct Pod discovery
- [ ] T041 [P] Write Lesson 10 cross-namespace resolution: Agent A calling Agent B in different namespace
- [ ] T042 [P] Write Lesson 10 "Try With AI" section: Troubleshoot DNS resolution scenario

**Checkpoint**: Lessons 8-10 complete. Organization and access patterns taught.

---

## Phase 4: Part 4 - Configuration & State (Lesson 12-13)

**Purpose**: Teach persistent storage and StatefulSets (Lessons 11, 14 already exist)

### Lesson 12: Persistent Storage

- [ ] T043 [P] Create `12-persistent-storage-pv-pvc.md` with full YAML frontmatter (sidebar_position: 12, lesson: 12, duration_minutes: 45, 6 LOs, digcomp_mapping)
- [ ] T044 [P] Write Lesson 12 introduction: Problem (agent loses embeddings on restart)
- [ ] T045 [P] Write Lesson 12 theory: PV/PVC abstraction, access modes, reclaim policies
- [ ] T046 [P] Write Lesson 12 hands-on: Create static PV, then PVC, mount into Pod
- [ ] T047 [P] Write Lesson 12 dynamic provisioning: Use StorageClass with Minikube hostpath provisioner
- [ ] T048 [P] Write Lesson 12 "Try With AI" section: Design storage for vector database

### Lesson 13: StatefulSets

- [ ] T049 [P] Create `13-statefulsets-agent-identity.md` with full YAML frontmatter (sidebar_position: 13, lesson: 13, duration_minutes: 45, 6 LOs, digcomp_mapping)
- [ ] T050 [P] Write Lesson 13 introduction: Socratic question "Why can't we use Deployment for Qdrant?"
- [ ] T051 [P] Write Lesson 13 theory: Stable hostname (pod-0, pod-1), volumeClaimTemplates, ordered scaling
- [ ] T052 [P] Write Lesson 13 hands-on: Create StatefulSet with headless Service, include kubectl output
- [ ] T053 [P] Write Lesson 13 rolling update example: Partition updates for StatefulSets
- [ ] T054 [P] Write Lesson 13 "Try With AI" section: Design StatefulSet for distributed vector database

**Checkpoint**: Lessons 12-13 complete. State management taught.

---

## Phase 5: Part 5 - Production Readiness (Lessons 15-18)

**Purpose**: Teach autoscaling, RBAC, security, and health probes

### Lesson 15: Horizontal Pod Autoscaler

- [ ] T055 [P] Create `15-horizontal-pod-autoscaler.md` with full YAML frontmatter (sidebar_position: 15, lesson: 15, duration_minutes: 45, 7 LOs, digcomp_mapping)
- [ ] T056 [P] Write Lesson 15 introduction: Spec-first approach "Agent needs 2-10 replicas based on load"
- [ ] T057 [P] Write Lesson 15 theory: HPA, metrics-server, target utilization, stabilization windows
- [ ] T058 [P] Write Lesson 15 hands-on: Enable metrics-server on Minikube, create HPA for CPU
- [ ] T059 [P] Write Lesson 15 load testing: Generate load with kubectl run, observe scaling
- [ ] T060 [P] Write Lesson 15 "Try With AI" section: Design HPA for memory-bound AI inference workload

### Lesson 16: RBAC

- [ ] T061 [P] Create `16-rbac-securing-agent-deployments.md` with full YAML frontmatter (sidebar_position: 16, lesson: 16, duration_minutes: 45, 6 LOs, digcomp_mapping)
- [ ] T062 [P] Write Lesson 16 introduction: Spec-first "Agent needs ConfigMap read, Secret read own namespace only"
- [ ] T063 [P] Write Lesson 16 theory: ServiceAccount, Role, ClusterRole, RoleBinding, ClusterRoleBinding
- [ ] T064 [P] Write Lesson 16 hands-on: Create ServiceAccount, Role, RoleBinding for agent
- [ ] T065 [P] Write Lesson 16 audit example: Use kubectl auth can-i to verify permissions
- [ ] T066 [P] Write Lesson 16 "Try With AI" section: Design RBAC for multi-tenant agent deployment

### Lesson 17: Kubernetes Security

- [ ] T067 [P] Create `17-kubernetes-security-ai-services.md` with full YAML frontmatter (sidebar_position: 17, lesson: 17, duration_minutes: 45, 6 LOs, digcomp_mapping)
- [ ] T068 [P] Write Lesson 17 introduction: Spec-first "Agent must run non-root, read-only fs, isolated network"
- [ ] T069 [P] Write Lesson 17 theory: SecurityContext, runAsNonRoot, readOnlyRootFilesystem, NetworkPolicy
- [ ] T070 [P] Write Lesson 17 hands-on: Configure SecurityContext for agent Pod
- [ ] T071 [P] Write Lesson 17 NetworkPolicy example: Isolate agent from other namespaces
- [ ] T072 [P] Write Lesson 17 "Try With AI" section: Audit security of existing deployment

### Lesson 18: Health Probes

- [ ] T073 [P] Create `18-health-checks-probes.md` with full YAML frontmatter (sidebar_position: 18, lesson: 18, duration_minutes: 40, 7 LOs, digcomp_mapping)
- [ ] T074 [P] Write Lesson 18 introduction: Problem (agent receives traffic before model loads)
- [ ] T075 [P] Write Lesson 18 theory: livenessProbe, readinessProbe, startupProbe, timing parameters
- [ ] T076 [P] Write Lesson 18 hands-on: Configure all three probes for AI agent with slow model loading
- [ ] T077 [P] Write Lesson 18 debugging section: Troubleshoot probe failures with kubectl describe
- [ ] T078 [P] Write Lesson 18 "Try With AI" section: Design probes for custom agent health requirements

**Checkpoint**: Lessons 15-18 complete. Production readiness patterns taught.

---

## Phase 6: Part 6 - AI Collaboration & Integration (Lesson 20)

**Purpose**: Teach Helm charts (Lesson 19 already exists, Lessons 21-22 need expansion)

### Lesson 20: Helm Charts

- [ ] T079 [P] Create `20-helm-charts-ai-agent-packaging.md` with full YAML frontmatter (sidebar_position: 20, lesson: 20, duration_minutes: 45, 6 LOs, digcomp_mapping)
- [ ] T080 [P] Write Lesson 20 introduction: Problem (deploying same agent to 3 environments manually)
- [ ] T081 [P] Write Lesson 20 theory: Helm chart structure, values.yaml, Go templates
- [ ] T082 [P] Write Lesson 20 hands-on: Install Redis from public chart, explore chart structure
- [ ] T083 [P] Write Lesson 20 custom chart: Create Helm chart for AI agent step-by-step
- [ ] T084 [P] Write Lesson 20 release management: helm install, upgrade, rollback, uninstall
- [ ] T085 [P] Write Lesson 20 "Try With AI" section: Extend chart with environment-specific values

**Checkpoint**: Lesson 20 complete. Helm charts taught.

---

## Phase 7: Capstone Expansion (Lesson 21)

**Purpose**: Significantly expand capstone to incorporate ALL new concepts

- [ ] T086 Read existing 21-capstone-production-ready-agent.md to understand current structure
- [ ] T087 Update Lesson 21 frontmatter: Add new learning objectives (11 total LOs covering all new concepts)
- [ ] T088 Write Lesson 21 specification section: Student writes spec.md BEFORE any YAML
- [ ] T089 Write Lesson 21 namespace setup: Create dev/prod namespaces with quotas
- [ ] T090 Write Lesson 21 init container section: Add model download init container
- [ ] T091 Write Lesson 21 sidecar section: Add logging sidecar
- [ ] T092 Write Lesson 21 Ingress section: Configure external access
- [ ] T093 Write Lesson 21 storage section: Add PVC for embeddings
- [ ] T094 Write Lesson 21 HPA section: Configure autoscaling
- [ ] T095 Write Lesson 21 probes section: Add all three health probes
- [ ] T096 Write Lesson 21 RBAC section: Configure ServiceAccount and Role
- [ ] T097 Write Lesson 21 security section: Add SecurityContext hardening
- [ ] T098 Write Lesson 21 validation section: Complete deployment checklist and verification steps
- [ ] T099 Write Lesson 21 "Try With AI" section: Deploy alternative agent using accumulated knowledge

**Checkpoint**: Capstone expanded with ALL new K8s concepts.

---

## Phase 8: README Update

**Purpose**: Update chapter README to reflect new 22-lesson structure

- [ ] T100 Update README.md lesson list from 10 to 22 lessons
- [ ] T101 Update README.md learning objectives to include new topics
- [ ] T102 Update README.md duration estimate (was ~6 hours, now ~15 hours)
- [ ] T103 Update README.md prerequisites section if needed

**Checkpoint**: README reflects complete 22-lesson chapter.

---

## Phase 9: Validation & Polish

**Purpose**: Ensure all lessons meet quality standards

- [ ] T104 Run MDX angle bracket safety check on all new lessons: `grep -n '<[^/!]' *.md | grep -v '```'`
- [ ] T105 Verify all lessons end with "Try With AI" section only (no "What's Next", "Summary", etc.)
- [ ] T106 Verify evidence presence: 70%+ code blocks have output blocks
- [ ] T107 Verify framework invisibility: No "Three Roles", "AI as Teacher/Student", "What to notice" patterns
- [ ] T108 Verify all YAML frontmatter is complete and valid
- [ ] T109 Verify sidebar_position values are sequential (1-22)
- [ ] T110 Cross-reference: Ensure capstone references concepts from earlier lessons correctly

**Checkpoint**: All 22 lessons validated and ready for publication.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately
- **Phase 2-6 (New Lessons)**: Depend on Phase 1 completion, can run in parallel
- **Phase 7 (Capstone)**: Should run after Phases 2-6 to reference new concepts
- **Phase 8 (README)**: Can run after Phase 1
- **Phase 9 (Validation)**: Depends on all content phases complete

### Parallel Opportunities

**Maximum parallelism**: All [P] tasks within a lesson can run in parallel. Different lessons can be written in parallel by different agents.

Example parallel execution:
```
Agent 1: Lessons 6-7 (Multi-Container Patterns)
Agent 2: Lessons 8-10 (Organization & Access)
Agent 3: Lessons 12-13 (State)
Agent 4: Lessons 15-18 (Production Readiness)
Agent 5: Lesson 20 (Helm)
```

After all new lessons complete:
- Single agent: Capstone expansion (needs to reference all)
- Single agent: Validation pass

---

## Implementation Strategy

### Phase-by-Phase Approach

1. Complete Phase 1 (Renumbering) - 10 tasks
2. Launch Phases 2-6 in parallel (12 new lessons) - 75 tasks
3. Complete Phase 7 (Capstone) after lessons ready - 14 tasks
4. Complete Phase 8 (README) - 4 tasks
5. Complete Phase 9 (Validation) - 7 tasks

**Total**: 110 tasks

### Task Distribution by Type

| Type | Tasks | Notes |
|------|-------|-------|
| File renaming | 10 | Phase 1 |
| Lesson frontmatter | 12 | One per new lesson |
| Lesson content | 63 | ~5-6 sections per lesson |
| Capstone expansion | 14 | Comprehensive update |
| README update | 4 | Reflect new structure |
| Validation | 7 | Quality checks |

---

## Notes

- [P] tasks = different files, no dependencies
- Each lesson should be 700-900 lines with complete coverage
- All code blocks MUST include realistic kubectl output
- Minikube v1.32+ is the target environment
- kubectl-ai is referenced in Lesson 19 for AI collaboration
- Capstone (Lesson 21) synthesizes ALL concepts - most critical lesson
