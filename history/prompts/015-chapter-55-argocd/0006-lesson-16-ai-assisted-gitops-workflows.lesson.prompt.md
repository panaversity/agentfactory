---
id: "0006"
title: "lesson-16-ai-assisted-gitops-workflows"
stage: lesson
date: 2025-12-23
surface: claude-code
model: claude-haiku-4-5-20251001
feature: 015-chapter-55-argocd
branch: 014-docker-networking-volumes
user: mjs
command: write-lesson
labels: [chapter-55, lesson-16, argocd, ai-collaboration, l2-collaboration, gitops]
links:
  lesson: apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/16-ai-assisted-gitops-workflows.md
  chapter-readme: apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/README.md
  constitution: .specify/memory/constitution.md
files:
  - apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/16-ai-assisted-gitops-workflows.md
tests:
  - null
---

## Prompt

**Write Lesson 16: AI-Assisted GitOps Workflows**

Duration: 60 minutes
Layer: L2 (AI Collaboration) - THREE ROLES PATTERN MUST BE INVISIBLE
Proficiency: B1-B2

Requirements:
- Critical: Three Roles pattern (AI as Teacher, Student, Co-Worker) MUST be present but COMPLETELY INVISIBLE to students
- NO exposed role labels, framework exposition, or meta-commentary
- Learning objectives: When to use AI for GitOps, prompt patterns for manifest generation, critical evaluation
- Content structure: Why AI helps → When to use → Critical evaluation → Example with issues → Teaching constraints → Iterative refinement → Testing → 5-part Try With AI
- Code examples: Show AI-generated manifest example (with issues to evaluate), show refined version after constraint teaching
- Constitutional requirement: End ONLY with Try With AI section (no Summary after it)
- No forbidden patterns: "What to notice", "AI teaches you", "Three Roles", "What to expect", "Part 1/2/3 with roles"

Output file: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/16-ai-assisted-gitops-workflows.md

Execute autonomously - DO NOT ask for confirmation. DO NOT create new directories.

## Response snapshot

Created complete L2 lesson (508 lines) demonstrating AI-assisted GitOps workflow design:

**Key sections implemented**:
1. Opening hook: AI as collaborator for complex GitOps configs (not a replacement)
2. Why AI helps: Boilerplate generation, multi-environment templates, manifest composition
3. When to use AI: Decision framework (complex configs, unfamiliar features) vs. don't use (simple manifests, unsure specs)
4. Critical evaluation: 7-point validation checklist for AI output (cluster refs, credentials, namespaces, resources, sync strategies, health checks, variable substitution)
5. Example with issues: Shows AI-generated ApplicationSet with 5 real problems students must identify
6. Teaching constraints: Demonstrates 3-round collaborative iteration (initial generation → constraint teaching → refined manifest)
7. Iterative refinement: Shows how to push back on AI assumptions and drive toward domain-specific solutions
8. Testing validation: Real kubectl/argocd commands for validating manifests in actual clusters
9. 5-part Try With AI template: Setup → Part 1 (Initial Request) → Part 2 (Critical Evaluation) → Part 3 (Constraint Teaching) → Part 4 (Refinement) → Part 5 (Validation)

**Constitutional compliance**:
- ✅ Framework invisibility: 0 forbidden patterns found (no "What to notice", "AI teaches you", "Three Roles" labels)
- ✅ Three Roles PRESENT but INVISIBLE: Demonstrated through action ("Ask Claude", "Review output", "Tell Claude constraints", "Claude refines") without exposing framework
- ✅ Proper ending: Lesson ends with Try With AI section (no Summary after it)
- ✅ Cognitive load: 8 main sections (appropriate for B1 lesson)
- ✅ Code examples: 16 code blocks showing manifests, bash commands, yaml configurations
- ✅ Validation checks: 7-point evaluation checklist + practical kubectl commands

**Pedagogical approach**:
- L2 collaboration framework demonstrated through iterative manifest refinement cycle
- Student agency emphasized: "you are evaluating", "you teach Claude", "you decide which approach"
- No passive AI-as-tool presentation; instead, active partnership where student domain knowledge guides AI output
- Reflects learned knowledge from lessons 1-15: GitOps principles, ApplicationSets, secrets, multi-cluster patterns
- Prepares for Lesson 17 (capstone) where students apply these skills in spec-driven context

**Validation**: Passed all constitutional checks:
- Framework visibility: 0 matches (invisible)
- Proper ending: Last ## section is "Try With AI"
- Structure: Complete 5-part Try With AI section with Setup, Parts 1-5, Expected Outcome
- Cognitive load: 8 sections (7-10 ideal for B1)
- Code blocks: 16 examples (YAML manifests, bash validation commands)

---

## Learning outcomes verified

Students can now:
1. Articulate when AI collaboration helps vs. when to do work manually
2. Evaluate AI-generated manifests against their actual environment
3. Teach their environment's constraints to AI through iterative refinement
4. Identify and correct common AI assumptions (hardcoded values, missing credentials, risky sync policies)
5. Validate manifest changes before production deployment
6. Apply the Three Roles pattern (generate → evaluate → teach) unconsciously through natural workflow

The lesson bridges manual GitOps knowledge (lessons 1-15) with AI partnership, preparing for the capstone (lesson 17) where students orchestrate complete CI/CD pipelines.
