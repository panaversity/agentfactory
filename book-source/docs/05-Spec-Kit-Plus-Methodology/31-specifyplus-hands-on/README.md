---
sidebar_position: 2
title: "Chapter 31: Spec-Kit Plus Hands-On"
---

# Chapter 31: Spec-Kit Plus Hands-On

Welcome to hands-on specification-driven development. This chapter transforms understanding (from Chapter 30) into practice. You won't read about specifications—you'll write them. You won't learn about AI collaboration—you'll build projects with your AI companion from specifications to implementation.

Through seven interconnected lessons, you'll master the Spec-Kit Plus workflow: /sp.specify → /sp.plan → /sp.tasks → implementation. You'll experience the cascade effect where specification quality determines everything downstream. In this chapter, you will also implement a minimal, spec‑faithful vertical slice of the grading system to validate your artifacts end‑to‑end. Chapter 32 then scales this to a full MVP with team guardrails.

Whether you're eager to start coding or skeptical that specs are worth the time, this chapter proves the value through direct experience: building projects specification-first is faster, more reliable, and more scalable than code-first development.

## The SpecifyPlus Loop and Core Commands

SpecifyPlus is an opinionated SDD loop:

1) /sp.specify — analyze and refine your spec (gaps, ambiguities, assumptions)
2) /sp.plan — produce an implementation plan (phases, dependencies, risks)
3) /sp.tasks — decompose into atomic, testable tasks
4) Implementation — build to spec; validate against acceptance criteria

Key command references:
- Setup: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/00_setup/readme.md`
- PHR: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/01_phr/readme.md`
- Spec: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/03_spec/readme.md`
- Plan: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/04_plan/readme.md`
- ADR: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/05_adr/readme.md`
- Tasks: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/06_tasks/readme.md`
- Implementation: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/07_implementation/readme.md`
- Analyze & Clarify: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/08_analyze_and_clarify/readme.md`
- Git commit & PR: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/09_git_commit_pr/readme.md`

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Operate the Spec‑Kit Plus loop**: Use /sp.specify, /sp.plan, and /sp.tasks to move from intent to shippable tasks
- **Master the Spec-Kit Plus workflow**: Use /sp.specify to generate structured specifications, /sp.plan to create implementation plans, and /sp.tasks to decompose work—understanding how each tool amplifies your clarity
- **Write SMART acceptance criteria**: Create testable, unambiguous success criteria that prevent misinterpretation—learning the difference between "works correctly" (vague) and "returns 12 when given 3 \* 4" (SMART)
- **Recognize quality through cascade effects**: Understand how specification quality determines plan quality, which determines task quality, which determines code quality—and how to diagnose problems at the right level
- **Build from specifications, not code**: Implement a Python calculator by starting with specifications and letting AI agents handle implementation—experiencing the paradigm shift from code-first to spec-first
- **Validate implementations against specs**: Use specifications as the source of truth for testing—writing test scenarios that verify code matches intent, not just "runs without errors"
- **Write production-grade specifications**: Create a real-world grading system specification with edge cases, error handling, and extensibility considerations—building the foundation for Chapter 32's capstone
- **Collaborate specification-first with AI**: Develop fluency in the back-and-forth dialogue with AI companions where you provide intent and critique output while AI handles structure and completeness

## What Makes This Chapter Different

**You'll practice every concept immediately.** Unlike Chapter 30's philosophical exploration, Chapter 31 is hands-on from the first lesson. You'll write specifications, generate plans, decompose tasks, and implement code—learning by doing, with your AI companion as partner.

**You'll experience the cascade effect directly.** When a vague specification produces a weak plan, you won't be told "specifications matter"—you'll diagnose the problem yourself, improve the specification, and watch the plan quality improve. This experiential learning builds intuition that lectures can't provide.

**You'll build two artifacts and one slice.** First, a Python calculator (simple domain, focus on workflow; pure Python examples). Second, a grading system specification (production-quality spec; Python-only for MVP). Third, a minimal vertical slice of the grader (rubric → submission → grade + feedback) implemented and tested to the spec.

## Vertical Slice in Chapter 31 (What you implement now)

- Scope: pure Python, stdlib, JSON. No external services.
- Slice includes:
  - Rubric CRUD (3–5 criteria) persisted to JSON
  - Submission validation (essay/code size and type limits)
  - Grade compute + 50–200 word template feedback referencing ≥2 criteria
  - Tests mapped directly to acceptance criteria
- Goal: validate spec → plan → tasks by building one cohesive flow before Chapter 32.

---

## Lesson Index

| # | Lesson | Purpose |
|---|--------|---------|
| 1 | Set Up Your Project With SpecifyPlus | Initialize project; understand structure/templates |
| 2 | Make Your Acceptance Criteria Clear | Write SMART, testable criteria |
| 3 | Run /sp.specify | Analyze and refine spec (gaps, assumptions) |
| 4 | Run /sp.plan | Produce plan (phases, dependencies, risks) + record PHR |
| 5 | Decompose Into Atomic Tasks | Create tasks with traceability; choose vertical slice |
| 6 | Build Spec End-to-End | Implement and test the vertical slice |
| 7 | Mini‑Project 1: Calculator | Spec → code → tests for a CLI calculator |
| 8 | Mini‑Project 2: Production Spec | Write a production‑grade grading‑system spec |

---

## PHR & ADR Mini‑Section

Use PHRs (Prompt History Records) to log major prompts/decisions, and ADRs (Architecture Decision Records) to capture design trade‑offs.

Example commands (adapt for your environment):

```
# Record a prompt history entry (PHR)
phr create --title "plan-grader-vertical-slice" --stage plan --feature grading-system \
  --body-file history/prompts/tmp_plan_prompt.md

# Create an ADR for storage choice
adr new "Choose JSON storage for MVP over database" \
  --context "MVP simplicity, no concurrency" \
  --decision "Use JSON files for rubrics/grades" \
  --consequences "Easy to start; migration needed later"

# Open a PR gated by checklists
sp git_commit_pr --branch feature/grader-slice --title "Grader slice: rubric→grade" \
  --include specs/grading-system/spec.md specs/grading-system/plan.md specs/grading-system/tasks.md \
  --checklists spec plan tasks
```

PHR/ADR Acceptance Checklist:
- [ ] PHR recorded for /sp.plan (title reflects decision context)
- [ ] ADR written for any significant trade‑off (e.g., JSON vs DB)
- [ ] PR description links to spec sections and task IDs
- [ ] Checklists attached/passed before merge
- [ ] Links added to README or history index for traceability

## Ready?

The next eight lessons will transform you from code-first to specification-first developer. You'll stop opening VS Code as the first step and start writing specifications. You'll stop debugging mysterious behaviors and start validating against explicit acceptance criteria. You'll stop explaining projects verbally and start pointing to specifications that capture exact intent.

Let's build this together.

---
