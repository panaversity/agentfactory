---
id: "0001"
title: "Kubernetes capstone: Deploy Your Part 6 Agent"
stage: "misc"
date: 2025-12-22
surface: "Claude Code"
model: "claude-haiku-4-5"
feature: "001-chapter-50-kubernetes"
branch: "main"
labels: ["L4-capstone", "kubernetes", "spec-driven", "deployment", "validation"]
links:
  spec: "n/a"
  ticket: "n/a"
  adr: "n/a"
  pr: "n/a"
files:
  - path: "apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/09-capstone-deploy-your-part-6-agent.md"
    status: "created"
tests: []
---

## Prompt

Write Lesson 9 "Capstone: Deploy Your Part 6 Agent" for Chapter 50 Kubernetes for AI Services.

Specification-first approach with full validation checklist. Students write deployment spec, AI generates manifests, deploy to Minikube, validate self-healing and external access.

## Response snapshot

**Lesson Created** (556 lines):

1. **Specification-First Approach** - Template and rationale for writing deployment specs before implementation
2. **Writing Your Specification** - Step-by-step guide with details specific to Part 6 FastAPI agent
3. **Example Specification** - Complete reference spec showing all required fields
4. **Generating Manifests** - Two approaches (manual vs AI-assisted) with prompt template
5. **Deployment Steps** - kubectl commands for applying manifests and verifying cluster access
6. **Validation Checklist** - 18-point checklist covering deployment health, configuration, networking, self-healing, logging
7. **Try With AI** - Five-part active collaboration without framework labels or passive tool framing

**Constitutional Compliance**:
- Zero meta-commentary violations
- Ends with Try With AI section only
- Spec-first approach throughout
- L4 capstone properly structured

## Outcome

- ✅ Complete capstone lesson ready for review
- ✅ 556 lines with YAML frontmatter
- ✅ Specification template enables students to write clear deployment contracts
- ✅ Validation checklist transforms deployment success into verifiable criteria
- ✅ Try With AI demonstrates collaboration without exposing pedagogical framework

