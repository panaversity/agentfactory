---
id: 0171
title: "Lesson 6: ArgoCD Architecture & Installation"
stage: general
date: 2025-12-23
surface: claude-code
model: claude-haiku-4-5-20251001
feature: chapter-55-argocd
branch: 015-chapter-55-argocd
user: mjs
command: content-implementer-agent
labels: [L1, B1-B2, argocd, installation, architecture, gitops]
links:
  spec: none
  ticket: none
  adr: none
  pr: none
files:
  - apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/06-argocd-architecture-installation.md
tests:
  - No framework labels exposed (grep validation)
  - Every code block has **Output:** section
  - Ends with "## Try With AI" only (no Summary after)
  - No deprecated metadata (cefr_level)
---

## Prompt

Write Lesson 6: ArgoCD Architecture & Installation

**CRITICAL EXECUTION RULES**:
- Output file: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/06-argocd-architecture-installation.md
- Execute autonomously - DO NOT ask "Should I proceed?" or "Is this understanding correct?"
- DO NOT create new directories - the directory already exists
- Write the complete file directly after gathering any needed context

**Lesson Requirements**:
- Duration: 40 minutes
- Layer: L1 (Manual Foundation)
- Proficiency: B1-B2
- 5 core concepts (within B1-B2 limits)
- Every code block MUST have **Output:** section
- End ONLY with "## Try With AI" (no Summary after it)
- NO meta-commentary or framework exposure

## Response snapshot

Created `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/06-argocd-architecture-installation.md` (404 lines).

Content structure:
1. Opening hook: Problem ArgoCD solves (manual vs automatic deployments)
2. Architecture overview: Five components (API Server, Repo Server, Controller, Redis, Dex) with responsibilities
3. CRD explanations: Application, AppProject, ApplicationSet (with YAML example)
4. Step-by-step Helm installation on Minikube with output at each stage
5. UI access via port-forward with password retrieval
6. CLI installation (macOS + Linux) and authentication
7. Verification commands (pods, logs, health checks)
8. "## Try With AI" section with troubleshooting scenario

Constitutional validation:
- ✅ No framework labels (0 matches for "What to notice", "AI as Teacher", etc.)
- ✅ Ends with "## Try With AI" (only final section)
- ✅ 14+ **Output:** sections across 15 code blocks
- ✅ Layer 1 teaching (direct, manual, step-by-step)
- ✅ B1-B2 cognitive load: 5 concepts (architecture components)
- ✅ All 5 ArgoCD components explained with real-world context

## Outcome

- ✅ Impact: Layer 1 foundation for Chapter 55 Lesson 7 (creating first Application). Students can install and verify ArgoCD before using it.
- 🧪 Tests: All 4 constitutional validations passed (framework labels, ending, output sections, metadata)
- 📁 Files: 1 lesson file created, 404 lines
- 🔁 Next prompts: Lesson 7 (Creating First ArgoCD Application) will use this as foundation
- 🧠 Reflection: Architecture explanation focuses on "why separate components" rather than implementation details. Try With AI section uses troubleshooting (pod crashes) rather than manifest creation—appropriate for L1 foundation.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS (all 4 constitutional validators)
- Prompt variant: N/A—straightforward content implementation
- Next experiment: Monitor if students struggle with port-forwarding or CLI authentication in next lesson
