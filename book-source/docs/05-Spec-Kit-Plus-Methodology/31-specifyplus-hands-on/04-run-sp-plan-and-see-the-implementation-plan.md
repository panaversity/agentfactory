---
title: "Run /sp.plan and See the Implementation Plan"
chapter: 31
lesson: 4
duration: "1.5 hours"
skills:
  - name: "Planning Automation"
    proficiency: "A2"
    category: "Technical"
  - name: "Implementation Planning"
    proficiency: "B1"
    category: "Technical"
  - name: "Plan Evaluation"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Use /sp.plan CLI command to generate implementation plans from specifications (A2)"
  - "Understand plan structure and breakdown of work units (B1)"
  - "Evaluate plan feasibility and dependencies (B1)"
---

# Run /sp.plan and See the Implementation Plan

## What You're About to Do

Generate the implementation plan from your spec, then record a PHR and (if needed) an ADR to capture key decisions.

---

## Part 1: Run /sp.plan

```
cd grading-system
/sp.plan <prompt>
```

Examine `specs/grading-system/plan.md` (phases, dependencies, risks, lesson breakdown).

---


## PHR & ADRs

Use PHRs (Prompt History Records) to log major prompts/decisions, and ADRs (Architecture Decision Records) to capture design trade‑offs.

### 1. Record a Prompt History Record (PHR)

Mostly PHRs are auto recored. You can manually log the planning prompt/summary as well so the decision trail is auditable.

Example (adapt for your environment):
```
/sp.phr <prompt>
```

---

### 3. Capture ADRs for Trade‑offs (Optional but Recommended)

When the plan introduces a material trade‑off, add an ADR.

Example:
```
/sp.adr <prompt>
```

---

## Part 4: Gate Readiness and Next Steps

- Validate plan against checklist (phases logical, dependencies clear, risks addressed)
- If blockers exist, refine spec and re‑run /sp.plan
- Proceed to Lesson 5 to decompose into atomic tasks and pick your vertical slice
