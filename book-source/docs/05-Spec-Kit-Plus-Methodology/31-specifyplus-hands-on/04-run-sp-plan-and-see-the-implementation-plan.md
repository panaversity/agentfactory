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
/sp.plan
# or
uvx specifyplus plan
```

Examine `specs/grading-system/plan.md` (phases, dependencies, risks, lesson breakdown).

---

## Part 2: Record a Prompt History Record (PHR)

Log the planning prompt/summary so the decision trail is auditable.

Example (adapt for your environment):
```
phr create --title "plan-grader-vertical-slice" --stage plan --feature grading-system \
  --body-file history/prompts/tmp_plan_prompt.md
```

PHR Checklist:
- [ ] Title communicates decision context (e.g., vertical slice scope)
- [ ] Stage = plan; feature string set
- [ ] Prompt text and concise summary saved
- [ ] File placed under history/prompts/<feature>/

---

## Part 3: Capture ADRs for Trade‑offs (Optional but Recommended)

When the plan introduces a material trade‑off, add an ADR.

Example:
```
adr new "Choose JSON storage for MVP over database" \
  --context "MVP simplicity, no concurrency" \
  --decision "Use JSON files for rubrics/grades" \
  --consequences "Easy to start; migration needed later"
```

ADR Checklist:
- [ ] Context, Decision, Consequences filled
- [ ] Links back to spec sections and plan assumptions
- [ ] Stored under docs/adr/

---

## Part 4: Gate Readiness and Next Steps

- Validate plan against checklist (phases logical, dependencies clear, risks addressed)
- If blockers exist, refine spec and re‑run /sp.plan
- Proceed to Lesson 5 to decompose into atomic tasks and pick your vertical slice

References:
- Plan: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/04_plan/readme.md`
- PHR: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/01_phr/readme.md`
- ADR: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/05_adr/readme.md`
