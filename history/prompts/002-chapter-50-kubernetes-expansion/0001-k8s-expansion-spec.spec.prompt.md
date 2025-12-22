---
id: "0001"
title: "K8s Chapter Expansion Specification"
stage: spec
date: 2025-12-22
surface: cli
model: claude-opus-4-5
feature: 002-chapter-50-kubernetes-expansion
branch: main
user: mjs
command: /sp.specify
labels: [kubernetes, chapter-50, expansion, specification, comprehensive]
links:
  spec: specs/002-chapter-50-kubernetes-expansion/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-chapter-50-kubernetes-expansion/spec.md
  - specs/002-chapter-50-kubernetes-expansion/checklists/requirements.md
tests:
  - name: Coverage validation against K8s Book
    result: PASS (all 15 major topics covered)
  - name: 4-Layer Teaching Method progression
    result: PASS (L1→L2→L4→L3 maintained)
  - name: AI-native focus
    result: PASS (every concept mapped to AI agent use case)
---

## Prompt

User questioned whether existing 10-lesson Chapter 50 comprehensively teaches Kubernetes: "do we really learn everything from base including what is kubernetes, serves, deployment, pods etc. from basical like you just made 10 lessons from a full book really"

User then provided a hands-on Kubernetes tutorial and asked to compare against The Kubernetes Book: "Is everything in this covered and here from this book @/Users/mjs/.../The_Kubernetes_Book_3rd_Edition.md"

After gap analysis showing ~50-60% coverage, user approved expansion: "Yes"

## Response snapshot

Performed comprehensive gap analysis comparing current Chapter 50 (10 lessons) against:
1. User-provided hands-on Kubernetes tutorial
2. Nigel Poulton's "The Kubernetes Book" (17 chapters)

Identified missing topics:
- Namespaces (environment isolation)
- Init Containers (model preloading)
- Native Sidecars (K8s 1.28+, logging agents)
- Ingress Controllers (external API exposure)
- Persistent Storage (PV/PVC for model weights, vector DBs)
- StatefulSets (agents with persistent identity)
- HPA (scaling AI inference load)
- RBAC (secure production deployments)
- Security best practices (NetworkPolicy, PodSecurity)
- Helm Charts (packaging agent deployments)
- Health Probes (liveness, readiness, startup)

Created comprehensive specification with 22 lessons:
- Parts 1-6 structure covering all K8s concepts
- 7 user stories with acceptance scenarios
- 34 functional requirements
- Renumbering strategy for existing lessons
- AI-native focus throughout (every concept applied to agent deployment)

## Outcome

- ✅ Impact: Comprehensive spec expanding Chapter 50 from 10 to 22 lessons with full K8s coverage
- 🧪 Tests: Coverage PASS (15 major K8s topics), Teaching Method PASS (L1-L4), AI-native PASS
- 📁 Files: spec.md (~400 lines), checklists/requirements.md (all items PASS)
- 🔁 Next prompts: /sp.plan, /sp.tasks, /sp.implement
- 🧠 Reflection: Gap analysis against authoritative source (The Kubernetes Book) revealed significant coverage gaps; expansion necessary for comprehensive K8s education

## Evaluation notes (flywheel)

- Failure modes observed: None - user question prompted thorough gap analysis
- Graders run and results (PASS/FAIL): Requirements checklist ALL PASS (25 items)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider automated coverage validation tool comparing lesson objectives against book TOC
