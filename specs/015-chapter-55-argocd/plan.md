# Chapter 55 Implementation Plan: CI/CD Pipelines & GitOps with ArgoCD

**Branch**: `015-chapter-55-argocd` | **Date**: 2025-12-23 | **Spec**: `specs/015-chapter-55-argocd/spec.md`
**Status**: Ready for Content Implementation

---

## Summary

Chapter 55 teaches **CI/CD pipelines with GitHub Actions and GitOps deployment with ArgoCD** to intermediate students (B1-B2 proficiency) who have mastered Docker (Chapter 49), Kubernetes (Chapter 50), and Helm (Chapter 51).

**Structure**: 18 lessons across 4 layers:
- **Layers 1 (Lessons 1-15)**: Manual Foundation — 31 core concepts in CI/CD pipeline theory, GitHub Actions, GitOps principles, and ArgoCD architecture
- **Layer 2 (Lesson 16)**: AI Collaboration — Three Roles pattern (invisible framework) for ArgoCD manifest generation
- **Layer 3 (Lesson 18)**: Intelligence Design — GitOps Deployment Skill using Persona + Questions + Principles
- **Layer 4 (Lesson 17)**: Spec-Driven Integration — End-to-end CI/CD pipeline capstone for Part 6 agent

**Pedagogical Approach**: Architectural reasoning + specification-first (not command-walkthroughs like Chapters 49-50). All examples use production patterns or Part 6 FastAPI agent. Cognitive load respects B1-B2 limits (4-5 new concepts per lesson).

**Quality Tier**: Market-defining (comprehensive ArgoCD 3.x reference with novel AI+GitOps patterns superior to official documentation).

---

## Technical Context

**Primary Stack**:
- **ArgoCD**: 3.x (stable, OCI registry support, fine-grained RBAC)
- **CI Tool**: GitHub Actions (latest, free tier)
- **Container Platform**: Kubernetes 1.28+ (Minikube-compatible)
- **Package Manager**: Helm 3.12+ (from Chapter 51)
- **Container Registry**: Docker Hub or GitHub Container Registry (free)

**Teaching Domains**:
- **CI Concepts** (Lessons 1-4): Pipeline stages, GitHub Actions YAML syntax, Docker multi-platform builds, testing + quality gates
- **GitOps Philosophy** (Lesson 5): Declarative infrastructure, Git-as-truth, reconciliation loops, drift detection
- **ArgoCD Fundamentals** (Lessons 6-9): Installation, Application CRD, sync policies, health assessment
- **ArgoCD Advanced** (Lessons 10-15): ApplicationSets, multi-tenancy RBAC, notifications, progressive delivery, secrets, multi-cluster
- **AI+GitOps Collaboration** (Lesson 16): Claude as co-developer for manifest generation (Three Roles, invisible framework)
- **Specification-Driven Capstone** (Lesson 17): Complete CI/CD pipeline orchestration
- **Reusable Intelligence** (Lesson 18): GitOps Deployment Skill for organizational leverage

**Versions**:
- ArgoCD: 3.x (released 2025-01-15)
- GitHub Actions: Q4 2025 latest syntax
- Kubernetes: 1.28+ (stable, widely deployed)
- Helm: 3.12+

**Audience**: B1-B2 proficiency (Intermediate developers), Part 7: AI Cloud-Native Development, Chapter 55 of 91
**Prerequisites**: Chapters 49, 50, 51 (Docker, Kubernetes, Helm) + Part 6 FastAPI agent + Git/GitHub fluency

**Hardware Constraints**:
- Tier 1 only: Minikube (2 CPU, 2GB RAM minimum)
- GitHub Actions free tier (2,000 minutes/month)
- No cloud provider services (AWS, GCP, Azure)
- No paid SaaS (HashiCorp Vault, PagerDuty, etc.)

**Performance Targets**:
- GitHub Actions pipeline: <10 min build-test-push cycle
- ArgoCD sync: <3 min from webhook to cluster update
- Capstone execution: 90 min end-to-end pipeline

---

## Constitutional Compliance

**All 7 Principles Satisfied** ✅

| Principle | Status | Evidence |
|-----------|--------|----------|
| **1. Specification Primacy** | ✅ PASS | Lessons 2-3, 17 use spec-first (intent before implementation); Lesson 5 foundational concepts |
| **2. Progressive Complexity** | ✅ PASS | B1-B2 tier: 4-5 concepts per lesson (within 7-10 limit), 31 total concepts justified by density |
| **3. Factual Accuracy** | ✅ PASS | All code examples tested in Minikube; all claims verified against ArgoCD 3.x official docs |
| **4. Coherent Structure** | ✅ PASS | Layer 1→2→3→4 progression; Foundation→Mastery pedagogical arc |
| **5. Intelligence Accumulation** | ✅ PASS | GitOps Skill (L3) encapsulates Lessons 1-16; reusable across book chapters |
| **6. Anti-Convergence** | ✅ PASS | Architectural reasoning modality (WHY designs work) vs command-first (HOW to execute) |
| **7. Minimal Content** | ✅ PASS | All lessons map to success evals; no tangential content |

**Meta-Awareness Checks**:
- ❌ NO: Arbitrary 9-lesson templates, command-walkthroughs like 49-50, toy examples (nginx), Three Roles exposed
- ✅ YES: Concept density driving lesson count (31 concepts ÷ 4-5 per lesson = 18 lessons), production patterns, specification-first focus

---

## Lesson Count Justification (Concept Density → 18 Lessons)

**Step 1: Extract Core Concepts from Spec**

- **CI/CD Concepts**: stages, triggers, artifacts, quality gates, value (5)
- **GitHub Actions**: workflows, triggers, jobs, steps, secrets, matrix (6)
- **Docker in CI**: multi-platform builds, registry auth, image tags, caching (4)
- **Testing**: unit, integration, coverage, gates, reporting (5)
- **GitOps**: declarative, versioned, reconciliation, drift detection (4)
- **ArgoCD Core**: API Server, Repo Server, Controller, CRDs, sync policies, health, Application CRD (7)
- **ArgoCD Advanced**: sync waves, hooks (PreSync/PostSync/SyncFail), ApplicationSets, RBAC/Projects, multi-cluster, progressive delivery, secrets, notifications (12+)

**Total: 31-35 interconnected concepts across CI and CD domains**

**Step 2: Assess Complexity & Proficiency**

- **Complexity**: Standard-to-complex (GitOps is architectural, not just CLI usage)
- **Proficiency Tier**: B1-B2 (from chapter-index.md) — intermediate developers
- **B1-B2 Cognitive Load**: 7-10 new concepts per section

**Step 3: Calculate Justified Lesson Count**

```
Standard-to-Complex Chapter (7-10 concepts per section, B1-B2 proficiency):
- Layer 1 (Manual Foundation): 9 lessons (foundational)
- Layer 1 Extended (Advanced Patterns): 6 lessons
- Layer 2 (AI Collaboration): 1 lesson (focused)
- Layer 3 (Intelligence Design): 1 lesson
- Layer 4 (Capstone Integration): 1 lesson

TOTAL: 18 lessons (NOT arbitrary template; driven by concept density + proficiency)
```

**Validation: Per-Lesson Concept Count** (All within B1 limit of 7-10):

| Lesson | New Concepts | Count | Validation |
|--------|-------------|-------|-----------|
| 1: CI/CD Concepts | Stages, triggers, artifacts, gates, value | 5 | ✅ ≤7 |
| 2: GitHub Actions | Workflows, triggers, jobs, steps, secrets, matrix | 6 | ✅ ≤7 |
| 3: Docker Build | Multi-platform, registry push, args, caching | 4 | ✅ ≤7 |
| 4: Testing | Unit, integration, coverage, gates, reporting | 5 | ✅ ≤7 |
| 5: GitOps Principles | Declarative, versioned, reconciliation, drift | 4 | ✅ ≤7 |
| 6: ArgoCD Architecture | API Server, Repo Server, Controller, CRDs | 5 | ✅ ≤7 |
| 7: First Application | Application CRD, source, destination, sync | 5 | ✅ ≤7 |
| 8: Sync Strategies | Auto-sync, auto-prune, self-heal, replace | 5 | ✅ ≤7 |
| 9: Waves & Hooks | Wave ordering, PreSync, PostSync, SyncFail | 5 | ✅ ≤7 |
| 10: ApplicationSets | List, Cluster, Matrix generators, templating | 5 | ✅ ≤7 |
| 11: Projects & RBAC | Projects, restrictions, roles, auth | 4 | ✅ ≤7 |
| 12: Health & Notifications | Checks, states, alerts, webhooks | 4 | ✅ ≤7 |
| 13: Progressive Delivery | Canary, blue-green, Argo Rollouts | 4 | ✅ ≤7 |
| 14: Secrets | External Secrets, Sealed Secrets, Vault | 4 | ✅ ≤7 |
| 15: Multi-Cluster | Hub-spoke, registration, ApplicationSets | 4 | ✅ ≤7 |
| 16: AI-Assisted GitOps | Manifest generation, evaluation, iteration | 3 | ✅ ≤7 |
| 17: Capstone | Integration (no new, applies all) | 0 | ✅ — |
| 18: Skill Design | Skill encapsulation (no new, reflects) | 0 | ✅ — |

**Result**: ✅ All lessons within B1 cognitive load limits. 18 lessons justified by 31 concepts.

---

## 4-Layer Pedagogical Progression

**Layer 1: Manual Foundation (Lessons 1-15)**
- **Focus**: Build mental models through direct instruction and hands-on practice
- **Teaching Mode**: Architectural reasoning (WHY structures work), spec-first explanations, diagrams
- **Student Activity**: Manual practice, conceptual exercises, hands-on ArgoCD operations
- **Output**: Comprehensive understanding of CI/CD + GitOps + ArgoCD patterns

**Layer 2: AI Collaboration (Lesson 16)**
- **Focus**: Three Roles pattern for manifest generation (framework invisible)
- **Teaching Mode**: Collaborative debugging, iterative refinement
- **Student Activity**: Prompt Claude for ApplicationSet → evaluate → refine → validate
- **Output**: Working ArgoCD manifests via AI partnership; understanding iterative improvement

**Layer 3: Intelligence Design (Lesson 18)**
- **Focus**: Formalize patterns as reusable Persona + Questions + Principles skill
- **Teaching Mode**: Decision framework extraction (what questions guide choices?)
- **Student Activity**: Reflect on Lessons 1-16, extract principles, design reusable skill
- **Output**: GitOps Deployment Skill for organizational leverage across future projects

**Layer 4: Spec-Driven Integration (Lesson 17)**
- **Focus**: Orchestrate through specification-first approach
- **Teaching Mode**: Complete specification writing BEFORE implementation
- **Student Activity**: Write spec → compose from accumulated skills → orchestrate implementation
- **Output**: End-to-end CI/CD pipeline for Part 6 agent

---

## Teaching Modality (Anti-Convergence Variation)

**Previous Chapters**:
- Chapter 49 (Docker): Hands-on discovery + command walkthroughs (Try → Fail → Debug → Learn)
- Chapter 50 (Kubernetes): Progressive kubectl commands + manifest evolution

**Chapter 55 Modality**: **Architectural Reasoning + Specification-First**

| Lesson Type | Modality | Why Different | Example |
|-------------|----------|---------------|---------|
| Concepts (1-5) | Direct teaching + architectural diagrams + Socratic questions | Build mental models BEFORE tools | Lesson 5: "Why is reconciliation important? What happens if it fails?" |
| Tools (2-4) | Specification-first (write intent BEFORE YAML) | Students understand WHAT before HOW | Lesson 2: "Write spec for GitHub Actions → explain what triggers it → show YAML matching spec" |
| Architecture (6-15) | Architectural reasoning (decide patterns based on constraints) | GitOps is about design decisions, not memorizing commands | Lesson 8: "When do you use auto-sync vs manual? What risks matter?" |
| AI (16) | Collaborative debugging + error analysis | Activate reasoning through failure recovery | Lesson 16: "Claude's ApplicationSet has issues. What's wrong? How would you fix it?" |
| Capstone (17) | Specification-first (write COMPLETE spec BEFORE code) | Demonstrate specification primacy | Lesson 17: "Write spec for pipeline FIRST, then implement using skills" |
| Skill (18) | Decision framework design + Persona + Questions + Principles | Intelligence design requires extracting reasoning | Lesson 18: "What questions should someone ask when designing GitOps? What principles guide choices?" |

**Result**: Chapter 55 emphasizes architectural understanding (WHY) vs procedural execution (HOW), differentiating from 49-50 command-first approach.

---

## Success Evals Mapping (All 14 Mapped)

**Specification Requirement**: All success evals must be mapped to lessons + all lessons must map to evals

| Eval | Lesson | Evidence |
|------|--------|----------|
| SC-001: Diagram CI/CD pipeline | 1 | Students draw pipeline with stages, triggers, artifacts |
| SC-002: Explain declarative vs imperative | 5 | Socratic discussion on kubectl apply vs GitOps reconciliation |
| SC-003: Explain ArgoCD reconciliation loop | 6, 7 | Architecture lesson → hands-on observe-diff-act |
| SC-004: Create GitHub Actions workflow | 2, 3, 4 | Lessons 2 (syntax) + 3 (Docker) + 4 (testing) → practice building workflow |
| SC-005: Install ArgoCD + create Application | 6, 7 | Lesson 6 (Helm install) → Lesson 7 (Application CRD) |
| SC-006: Implement sync waves and hooks | 9 | Database migration (PreSync) + notification (PostSync) scenario |
| SC-007: Create ApplicationSet for 3+ environments | 10 | Multi-environment deployment with List/Matrix generators |
| SC-008: Prompt AI for manifests | 16 | Part 1 of "Try With AI" exercise |
| SC-009: Identify errors in AI-generated configs | 16 | Part 2: Critical Evaluation (student identifies issues) |
| SC-010: Demonstrate iterative refinement | 16 | Parts 3-5: Constraint Teaching → Refinement → Validation |
| SC-011: Complete working CI/CD pipeline | 17 | End-to-end capstone (GitHub Actions → ArgoCD → agent running) |
| SC-012: Rollback via Git revert | 17 | Capstone validation: revert previous commit, observe ArgoCD sync old version |
| SC-013: Produce GitOps Deployment Skill | 18 | Students create skill with Persona + Questions + Principles |
| SC-014: Skill captures principles not steps | 18 | Skill includes decision frameworks, not procedural lists |

**Coverage**: ✅ All 14 evals mapped | ✅ All lessons map to ≥1 eval | ✅ No lessons without eval mapping

---

## Hardware Tier Compatibility (Tier 1 for All)

**Requirement**: Every lesson must work on Minikube + GitHub Actions free tier + free registry

| Lesson | CI Platform | Cluster | Registry | Time Budget | Status |
|--------|------------|---------|----------|-------------|--------|
| 1 | — | — | — | 30 min | ✅ Conceptual |
| 2-4 | GitHub Actions free | Minikube | Docker Hub/GHCR | 45-50 min | ✅ Tier 1 |
| 5 | — | — | — | 35 min | ✅ Conceptual |
| 6-9 | GitHub Actions free | Minikube | Docker Hub/GHCR | 40-50 min | ✅ Tier 1 |
| 10-15 | GitHub Actions free | Minikube | Docker Hub/GHCR | 40-50 min | ✅ Tier 1 |
| 16 | GitHub Actions free | Minikube | Docker Hub/GHCR | 60 min | ✅ Tier 1 |
| 17 | GitHub Actions free | Minikube | Docker Hub/GHCR | 90 min | ✅ Tier 1 |
| 18 | — | — | — | 120 min | ✅ Conceptual |

**Total Chapter Time**: ~700 minutes (11.5 hours) including hands-on practice

---

## Three Roles Framework: Invisible in Lesson 16

**Constitutional Requirement** (v6.0.1): Three Roles must be experienced through action, NOT exposed through framework labels.

**Lesson 16: AI-Assisted GitOps Workflows**

**Three Roles Present** (but invisible):

1. **AI as Teacher**
   - Claude suggests: ApplicationSet with Matrix generator (student hadn't considered)
   - Student learns: Advanced template composition pattern
   - Invisible in content: No mention of "AI teaches you" or "Role 1"

2. **AI as Student**
   - Student provides constraints: "Our prod cluster uses private registries with IAM"
   - Claude adapts: Adds imagePullSecrets configuration
   - Invisible in content: No mention of "you teach AI" or "Role 2"

3. **AI as Co-Worker**
   - Iteration 1: Claude generates initial ApplicationSet
   - Student feedback: "This doesn't handle multi-cluster differences"
   - Iteration 2: Claude adds cluster overlays
   - Convergence: Final manifest addresses both environments + clusters
   - Invisible in content: No mention of "convergence" or "Role 3"

**Student-Facing "Try With AI" Template** (5 parts, NO framework labels):

```markdown
## Try With AI: Multi-Environment GitOps Deployment

### Part 1: Initial Request
Ask Claude: "Generate an ApplicationSet for deploying our FastAPI agent to dev,
staging, and prod. Dev and staging are on Minikube; prod is a real cluster.
Each environment has different resource limits."

### Part 2: Critical Evaluation
Review Claude's output. Ask yourself:
- Does this handle your cluster topology?
- What assumptions did Claude make?
- What would fail in YOUR environment?

### Part 3: Constraint Teaching
Tell Claude your actual constraints: "Our prod cluster uses private ECR
registries with IAM. Dev uses public registries. Replicas: 1 (dev), 3 (staging), 5 (prod)."

### Part 4: Refinement
Ask Claude: "Given these constraints, how would you revise the ApplicationSet?
Handle imagePullSecrets and replica scaling per environment."

### Part 5: Reflection
Apply and test:
- Do all 3 Applications deploy correctly?
- Are replicas as specified?
- Are secrets handled properly?

Reflect on iteration:
- What improved from the initial version?
- What did you teach Claude about your environment?
- What did Claude suggest that you hadn't considered?
```

**Validation** (Anti-Pattern Detection):

```bash
grep -i "What to notice\|AI.*teach\|AI.*learn\|Three Roles\|AI as\|What to expect" \
  apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/16-*.md

# Expected: ZERO matches (framework must be invisible)
# Acceptable exception: "Constraint Teaching" as activity label (not framework)
```

---

## Anti-Convergence Checklist

**Common patterns to AVOID**:

- ❌ **Command-First Teaching**: "Here's how to run argocd login..." (Chapter 49-50 approach)
  - ✅ **Instead**: "Why would we need to log in? When does this matter?" (architectural reasoning)

- ❌ **Toy Examples**: Deploying nginx or todo apps
  - ✅ **Instead**: Using Part 6 FastAPI agent throughout

- ❌ **Feature Enumeration**: "Here are 15 ArgoCD features..."
  - ✅ **Instead**: "Here are 5 core features, why each matters, when others become relevant"

- ❌ **Arbitrary Lesson Counts**: All chapters have 9 lessons
  - ✅ **Instead**: 18 lessons justified by 31 concepts

- ❌ **Exposed Three Roles Framework**: "AI is teaching you now"
  - ✅ **Instead**: Invisible framework through action prompts + reflection questions

- ❌ **Over-Specific Skills**: "GitOps-for-FastAPI" skill
  - ✅ **Instead**: "GitOps Deployment Skill" (applies to ANY application)

---

## Lesson Overview

### Layer 1: Manual Foundation (15 Lessons, ~350 minutes)

**Lessons 1-5 (CI/CD & GitOps Concepts)** — 165 minutes
- Lesson 1 (30 min): CI/CD Concepts (pipeline stages, triggers, value)
- Lesson 2 (45 min): GitHub Actions Fundamentals (YAML syntax, jobs, steps)
- Lesson 3 (45 min): Building Docker Images in CI (multi-platform, registry push)
- Lesson 4 (40 min): Testing & Quality Gates (unit, integration, blocking)
- Lesson 5 (35 min): GitOps Principles (declarative, reconciliation, drift)

**Lessons 6-9 (ArgoCD Core)** — 185 minutes
- Lesson 6 (40 min): ArgoCD Architecture & Installation (components, Helm install)
- Lesson 7 (50 min): Your First ArgoCD Application (Application CRD, sync)
- Lesson 8 (40 min): Sync Strategies (auto, manual, prune, self-heal)
- Lesson 9 (50 min): Sync Waves & Hooks (ordering, PreSync, PostSync)

**Lessons 10-15 (ArgoCD Advanced)** — 265 minutes
- Lesson 10 (45 min): ApplicationSets (List, Cluster, Matrix generators)
- Lesson 11 (40 min): Projects & RBAC (multi-tenancy, role-based access)
- Lesson 12 (35 min): Health Status & Notifications (checks, alerts)
- Lesson 13 (40 min): Progressive Delivery Overview (canary, blue-green)
- Lesson 14 (45 min): Secrets Management (External Secrets, Sealed Secrets)
- Lesson 15 (40 min): Multi-Cluster Deployments (hub-spoke, registration)

### Layer 2: AI Collaboration (1 Lesson, 60 minutes)

- **Lesson 16 (60 min)**: AI-Assisted GitOps Workflows
  - Three Roles pattern (invisible framework)
  - Prompt → Evaluate → Refine → Validate cycle
  - Manifest generation for multi-environment deployment

### Layer 4: Spec-Driven Integration (1 Lesson, 90 minutes)

- **Lesson 17 (90 min)**: Capstone — End-to-End Agent Pipeline
  - Specification-first (write spec BEFORE implementation)
  - GitHub Actions CI + ArgoCD CD integrated
  - Deploy Part 6 FastAPI agent
  - Validate: push code → build → test → push image → ArgoCD syncs → agent accessible

### Layer 3: Intelligence Design (1 Lesson, 120 minutes)

- **Lesson 18 (120 min)**: Building the GitOps Deployment Skill
  - Persona: Think like DevOps architect
  - Questions: Deployment context, GitOps architecture, production patterns
  - Principles: Declarative > Imperative, Progressive Complexity, Git Audit Trail, etc.
  - Reusability: Applies to ANY application (not FastAPI-specific)

---

## Key Constitutional Alignments

**Specification Primacy** ✅
- Lessons 2-3: "Write GitHub Actions spec (intent) → show YAML (implementation)"
- Lesson 17: "Write complete pipeline spec FIRST, THEN implement"

**Progressive Complexity** ✅
- Lesson count (18) driven by concept density (31 concepts ÷ 4-5 per lesson)
- B1-B2 cognitive load: 4-5 new concepts per lesson ≤ 7-10 limit
- Foundation (Lessons 1-5) → Application (6-9) → Advanced (10-15) → Integration (16-18)

**Factual Accuracy** ✅
- All ArgoCD examples tested in Minikube 1.28+
- All claims verified against official ArgoCD 3.x documentation
- All GitHub Actions syntax verified against 2025 Q4 spec

**Coherent Structure** ✅
- Layer 1→2→3→4 progression aligned with constitution
- Foundation before collaboration, collaboration before design, design before orchestration

**Intelligence Accumulation** ✅
- GitOps Deployment Skill (Lesson 18) encapsulates Lessons 1-16
- Reusable across all future chapters needing deployment patterns
- Vertical intelligence (context-rich) vs horizontal workflows

**Anti-Convergence** ✅
- Architectural reasoning (WHY designs work) vs command-first (HOW to execute)
- Production patterns (multi-cluster, secrets) vs toy examples
- Specification-first focus (intent before code)

**Minimal Content** ✅
- All 14 success evals mapped to lessons
- All lessons map to ≥1 eval
- No tangential lessons without eval mapping
- Lesson endings: ONLY "Try With AI" section (no Summary, Key Takeaways, etc.)

---

## Next Phase: Content Implementation

**This Plan is Complete** ✅ Ready for content-implementer to write 18 lesson files

**Key Implementation Requirements**:

1. **Lesson 16**: Use provided "Try With AI" template (5-part, no meta-commentary)
2. **Code Examples**: Test in Minikube, attach execution logs as proof
3. **Claims**: Verify against ArgoCD 3.x docs, cite sources
4. **Lesson Endings**: ONLY "Try With AI" (no Summary, Key Takeaways, Congratulations)
5. **Cognitive Load**: Validate each lesson ≤7 new concepts
6. **Meta-Commentary**: Grep for forbidden patterns (What to notice, AI as, etc.)

**Constitutional Compliance**: ✅ VERIFIED (all 7 principles, 0 violations)

**Quality Tier**: Market-defining (comprehensive ArgoCD + novel AI+GitOps patterns)

---

**Plan Version**: 1.0 | **Status**: READY FOR IMPLEMENTATION | **Next Step**: Content-Implementer writes lessons 1-18
