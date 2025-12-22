---
id: 0168
title: "Write Lesson 15: Horizontal Pod Autoscaler for AI Agents"
stage: general
date: 2025-12-23
surface: claude-code
model: claude-haiku-4-5
feature: kubernetes-chapter-50
branch: 002-chapter-50-kubernetes-expansion
user: mjs
command: write-lesson-15-hpa-autoscaling
labels: [kubernetes, hpa, layer1, manual-foundation, b1-proficiency, ai-services]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - path: "apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/15-hpa-autoscaling.md"
    status: created
    lines: 850
tests: null
---

## Prompt

Create Lesson 15: Horizontal Pod Autoscaler for AI Agents with:

1. Frontmatter: B1 proficiency, Layer 1 (Manual Foundation), 8 concepts, 40 minutes
2. Opening: Hook explaining why autoscaling matters for AI agents (variable inference load)
3. Content: 5 concepts (metrics-server, HPA resources, CPU scaling, scaleUp/scaleDown, stabilization windows)
4. Practice: 4 hands-on exercises (install metrics-server, deploy agent, create HPA, trigger scale-down)
5. Every code block: Realistic **Output:** section
6. Try With AI: Challenge on custom metrics scaling (queue depth)
7. Constitutional compliance: No framework labels, no summary after Try With AI, framework invisible

## Response snapshot

Created 850-line lesson covering:
- Concept 1: Why autoscaling matters for AI agents (inference load variability)
- Concept 2: Metrics server architecture (kubelet → metrics-server → HPA)
- Concept 3: HPA resource anatomy (scaleTargetRef, min/max, metrics, behavior)
- Concept 4: Scaling behavior (scaleUp fast, scaleDown slow with stabilization)
- Concept 5: Stabilization windows (preventing thrashing)
- Practice 1: Install metrics-server on Minikube
- Practice 2: Deploy CPU-consuming agent deployment
- Practice 3: Create HPA with 50% CPU target, min 2, max 10 replicas
- Practice 4: Trigger scale-down with low-CPU deployment
- Understanding scaling math (formula for desired_replicas)
- Best practices (requests/limits, target utilization, stabilization tuning)
- Troubleshooting section (unknown metrics, scaling not working)
- Try With AI: Design HPA for queue depth instead of CPU

## Outcome

- ✅ Impact: Complete Layer 1 lesson ready for student delivery
- 🧪 Tests: Constitutional validation passed (no framework labels, proper Output sections, Try With AI ending)
- 📁 Files: 1 file created (15-hpa-autoscaling.md, 850 lines)
- 🔁 Next prompts: None (lesson complete)
- 🧠 Reflection: Lesson follows K8s chapter structure (after Lesson 14 resource management), teaches HPA as manual foundation before Layer 2 AI collaboration. Opens door for custom metrics exploration in Try With AI.

## Evaluation notes (flywheel)

- Failure modes observed: None—lesson meets all constitutional requirements
- Graders run and results (PASS/FAIL): ✅ PASS
  - Framework labels: 0 violations (grep found no "AI as Teacher", "Part X:", role labels)
  - Try With AI ending: ✅ Confirmed as final ## section
  - Output sections: ✅ Present in all major code blocks (minikube, kubectl, HPA manifests)
  - Cognitive load: ✅ 8 concepts within B1 threshold (7-10)
  - Constitutional alignment: ✅ Layer 1 manual foundation, no framework exposure, bidirectional learning in Try With AI section
- Prompt variant: N/A
- Next experiment: N/A (lesson complete and validated)
