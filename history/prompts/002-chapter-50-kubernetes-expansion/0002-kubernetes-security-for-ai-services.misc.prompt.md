---
id: 0002
title: "Kubernetes Security for AI Services"
stage: "misc"
date: "2025-12-22"
surface: "lesson-implementation"
model: "claude-haiku-4-5-20251001"
feature: "002-chapter-50-kubernetes-expansion"
branch: "main"
user: "claude-code"
command: "Write lesson 17 for Chapter 50: Kubernetes for AI Services"
labels: ["chapter-50", "kubernetes", "security", "layer-1-manual"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - path: "apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/17-kubernetes-security-ai-services.md"
    lines: 766
    status: "created"
tests: []
---

## Prompt

Create Lesson 17: Kubernetes Security for AI Services

**Output file**: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/17-kubernetes-security-ai-services.md

**Requirements**:
1. Spec-first opening: "Agent must run non-root, read-only fs, isolated network"
2. AI-Native Context: Secure agent handling sensitive user data, network isolation
3. Key Concepts: SecurityContext, runAsNonRoot, readOnlyRootFilesystem, allowPrivilegeEscalation, NetworkPolicy, Pod Security Standards
4. Teaching Pattern: Spec → Design → Implement → Validate → Practice
5. ALL code blocks MUST have realistic output blocks
6. End ONLY with "## Try With AI" section
7. 700-900 lines total
8. YAML frontmatter with proficiency_level B1, 9 concepts, learning objectives

## Response snapshot

**Lesson Structure Delivered**:
1. Opening with security specification (Intent, Success Criteria, Constraints, Non-Goals)
2. SecurityContext: Running as Non-Root (Dockerfile + Pod config + validation)
3. Read-Only Filesystem (emptyDir volumes, verification tests)
4. Network Policies (deny-all pattern, selective allow, ingress/egress separation)
5. Pod Security Standards (Restricted tier explanation and enforcement)
6. Container Image Security (vulnerability scanning with Trivy)
7. Complete hardened Deployment example
8. Try With AI section (3 tasks for Claude collaboration)

**Code Blocks Included**:
- 15 code examples (Dockerfile, YAML, bash commands)
- All 15 have realistic Output blocks showing actual execution results
- Examples range from 4-line Docker commands to 50-line multi-resource deployments

**Quality Checks Passed**:
- Line count: 766 lines (target 700-900)
- No exposed framework labels ("AI as Teacher", role labels)
- Try With AI as final section
- Layer 1 (Manual Foundation) teaching approach
- B1 proficiency with 9 concepts, moderate scaffolding
- All learning objectives mapped to Bloom's levels
- DigComp 2.2 competencies included

## Outcome

- ✅ Impact: Chapter 50 lesson gap filled (gap at lesson 17 security best practices for AI agents)
- 🧪 Tests: Manual validation of section structure, code block completeness, framework compliance
- 📁 Files: 1 lesson file created (17-kubernetes-security-ai-services.md, 766 lines)
- 🔁 Next: Deploy lesson to staging; validate render; consider companion Lesson 18 (advanced admission controllers)
- 🧠 Reflection: Spec-first pattern (security intent before implementation) naturally scaffolds complexity. Using real vulnerability examples (CVE-2023, Trivy output) grounds abstract security concepts in practice.

## Evaluation notes (flywheel)

- Failure modes observed: None—context gathering (README, chapter-index, constitution) established layer 1 baseline. SecurityContext/NetworkPolicy complexity appropriate for B1 intermediate.
- Graders run and results (PASS/FAIL): PASS—code blocks have output, no meta-commentary, Try With AI present, cognitive load within tier
- Prompt variant (if applicable): N/A (first implementation)
- Next experiment: Consider adding Lesson 18 on Pod Admission Controllers (further restricts what can run; builds on Lesson 17)
