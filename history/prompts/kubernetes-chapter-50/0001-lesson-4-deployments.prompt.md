---
id: "0001"
date: "2025-12-22"
stage: "content"
feature: "kubernetes-chapter-50"
title: "Lesson 4: Deployments - Self-Healing at Scale"
model: "claude-haiku-4-5"
tokens:
  input: 18500
  output: 7200
---

# Prompt History Record: Lesson 4 Creation

## Summary

Created complete Layer 1 (Manual Foundation) lesson teaching Kubernetes Deployments with focus on self-healing, scaling, and rolling updates. Lesson targets B1 proficiency with 10 concepts and moderate scaffolding. All 17 code blocks include expected output. Constitutional validation passed all checks.

## Context

- **Chapter**: 50 (Kubernetes for AI Services)
- **Lesson**: 4 of 10
- **Duration**: 45 minutes
- **Proficiency**: B1 (Intermediate)
- **Layer**: Layer 1 (Manual Foundation)
- **Stage Name**: Manual Foundation - declarative workload management
- **Cognitive Load**: 10 concepts, moderate scaffolding

## Design Decisions

### 1. Layer 1 Manual Foundation Structure

**Decision**: Teach all concepts through direct instruction and CLI examples, not through AI collaboration initially.

**Rationale**: Students need mental models of Kubernetes' control loop and declarative paradigm before AI collaboration becomes valuable. Layer 1 builds intuition for evaluating AI-generated manifests.

**Implementation**:
- Each concept explained with analogies (Deployment as manager)
- Step-by-step CLI commands with all outputs shown
- Multiple command variations (scale, update, rollback)
- Progressive complexity (create → verify → delete → recreate → scale → update → rollback)

### 2. Hierarchy Visualization

**Decision**: Introduce the Deployment → ReplicaSet → Pod hierarchy early and reinforce it throughout.

**Rationale**: Understanding resource ownership and controller reconciliation is foundational to Kubernetes reasoning. Students need to see WHY ReplicaSets exist, not just that they do.

**Implementation**:
```
Deployment (what you create)
    ↓ manages
ReplicaSet (intermediate controller)
    ↓ manages
Pods (actual running containers)
```

### 3. Self-Healing as Centerpiece

**Decision**: Make self-healing the emotional arc of the lesson (why Deployments matter).

**Rationale**: "Delete a Pod, watch it auto-recover" is memorable and demonstrates Kubernetes' power. This justifies why we use Deployments instead of manual Pod management.

**Implementation**:
- Problem setup: "Pods are ephemeral, direct deployment fails"
- Solution preview: "Deployments = manager that ensures desired state"
- Live demonstration: Delete a Pod, observe automatic recreation
- Concept reinforcement: "This is the reconciliation loop"

### 4. Rolling Updates as Advanced Application

**Decision**: Show rolling updates after students understand replicas, not as separate concept.

**Rationale**: Rolling updates are Deployments' killer feature for production. Understanding happens naturally as extension of replica management.

**Implementation**:
- Prerequisite: 3 replicas running
- Change: Update image version
- Observation: kubectl reports progress (1/3 updated, 2/3 updated, 3/3 done)
- Extension: Observe old ReplicaSet retained for rollback
- Rollback: Instantly undo the update

### 5. "Try With AI" as Scenario-Based Practice

**Decision**: Three scenarios with increasing complexity, not passive Q&A.

**Rationale**: Active prompts + critical evaluation + reflection deepens learning better than "ask AI about X".

**Implementation**:
- Scenario 1: Create a new Deployment (AI as code generator, student as validator)
- Scenario 2: Troubleshoot a failed Deployment (AI as troubleshooting guide)
- Scenario 3: Plan zero-downtime updates (AI as explainer, student as evaluator)

Each scenario includes specific prompts, AI's expected suggestions, and reflection questions.

### 6. All Evidence Included

**Decision**: Every bash/yaml block includes actual expected output.

**Rationale**: Constitutional requirement for code evidence. Students see what success looks like before running commands themselves.

**Implementation**: 17 code blocks, 17 output sections covering:
- YAML manifest creation and explanation
- kubectl apply and verification
- kubectl get commands showing resource hierarchy
- kubectl scale operations
- kubectl set image for updates
- kubectl rollout status and history
- Self-healing demonstration

### 7. Cognitive Load: 10 Concepts for B1

**Decision**: Balance 10 distinct concepts without overwhelming B1 learners.

**Rationale**: B1 proficiency (7-10 concepts) with moderate scaffolding.

**Concepts**:
1. Pod ephemerality problem
2. Deployment as manager abstraction
3. Deployment → ReplicaSet → Pod hierarchy
4. Deployment YAML structure
5. Replicas and desired state
6. Selector and label matching
7. Self-healing through reconciliation
8. Scaling operations
9. Rolling updates and image versioning
10. Rollbacks and revision history

### 8. Constitutional Compliance

**Decision**: Zero meta-commentary, no "What to notice" or "AI as Teacher" labels.

**Validation Results**:
- ✓ Framework invisibility (0 meta-commentary patterns)
- ✓ Evidence completeness (17/17 outputs)
- ✓ Proper ending (Try With AI is final ## section)
- ✓ YAML frontmatter complete
- ✓ Learning objectives aligned with content
- ✓ Cognitive load appropriate for B1
- ✓ Layer 1 pedagogy correct (manual foundation)
- ✓ Teaching modality varied (analogies, tables, diagrams, prose, practice)

## Artifacts Created

**File**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/04-deployments-self-healing-at-scale.md`

**Length**: ~1,100 lines (consistent with other chapter lessons)

**Sections**:
1. Opening hook (why Deployments solve Pod ephemeralitys)
2. The Problem (Pod lifecycle, manager analogy)
3. The Abstraction (hierarchy, responsibilities)
4. Creating First Deployment (YAML + kubectl apply)
5. Self-Healing (delete, observe, explain)
6. Scaling (scale up, scale down)
7. Rolling Updates (image update, watch progress)
8. Rollback (undo, history)
9. Declarative Model (reflect on what happened)
10. Key Concepts (vocabulary)
11. Try With AI (3 scenarios with prompts + reflection)

## Lessons Learned

### Pattern: Abstraction Hierarchy Teaching

The Deployment → ReplicaSet → Pod hierarchy introduces a pattern that repeats throughout Kubernetes:
- Higher abstractions declare WHAT
- Lower abstractions handle HOW
- Students reason about declaration, not implementation

This same pattern applies to Service → Endpoints → Pods, StatefulSet → Pods, Job → Pods in future lessons.

### Pattern: Declarative Model as Core Principle

Three Kubernetes commands demonstrate the declarative model:
1. `kubectl apply` - declare desired state
2. `kubectl delete` - remove and auto-reconcile
3. `kubectl rollout undo` - previous state still exists, activate it

Students who internalize the declarative model struggle less with Kubernetes complexity.

### Pattern: Self-Healing Justifies Abstraction

Why do students accept complexity of Deployments instead of simple `kubectl run`?

**Because they see self-healing in action**: "I deleted the Pod, and Kubernetes created a new one automatically."

This visceral experience is more compelling than explaining benefits theoretically.

## Next in Progression

Lesson 5: **Services** - Makes Deployments discoverable. Students will apply their Deployment knowledge while learning service discovery.

The progression builds naturally:
- Lesson 4 (Deployments): Get your application running stably
- Lesson 5 (Services): Make it discoverable
- Lesson 6-7: Configuration and persistence
- Later: Layer 2 AI collaboration, Layer 3 skills creation, Layer 4 capstone

## Status

✓ Ready for delivery
✓ Constitutional compliant
✓ All evidence included
✓ Layer 1 pedagogy correct
✓ B1 proficiency appropriate
✓ Validation passed all checks
