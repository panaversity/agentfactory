---
id: "0004"
title: "Lesson 06 Plan Phase Rewrite Research Paper"
stage: "green"
date: "2025-11-26"
surface: "claude-code"
model: "claude-haiku-4-5"
feature: "037-chapter-14-research-paper-pivot"
branch: "main"
user: "MJS"
command: "/sp.implement"
labels: ["lesson-rewrite", "chapter-14", "plan-phase", "content-generation"]
links:
  spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
  ticket: "Rewrite Lesson 06 with research paper focus (remove video generation)"
  adr: "n/a"
  pr: "pending"
files:
  - path: "apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/06-plan-phase.md"
    status: "rewritten"
    lines: 504
tests:
  - name: "Content validation"
    status: "PASS"
  - name: "No prohibited content (Playwright, Gemini, MCP, video)"
    status: "PASS"
  - name: "Four key concepts within cognitive load"
    status: "PASS"
  - name: "Plan bridges spec to tasks"
    status: "PASS"
  - name: "Research paper as practice vehicle"
    status: "PASS"
  - name: "Try With AI section present"
    status: "PASS"
---

## Prompt

You are rewriting Lesson 06: Plan Phase for Chapter 14.

**CRITICAL FOCUS**: Teach the `/sp.plan` command. Research paper is just the practice example.

**Task**: Rewrite 06-plan-phase.md

**File Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/06-plan-phase.md`

**Learning Objective**: Use `/sp.plan` to create implementation strategy.

**Concepts to Teach** (max 4):

1. What is a plan (HOW to build WHAT - connects spec to implementation)
2. `/sp.plan` command syntax and usage
3. Plan sections: approach, components, dependencies, milestones
4. How plan connects specification to tasks

**Key Teaching Points**:

- Plan = HOW you'll build what the spec defines
- Breaks the work into logical components
- Identifies dependencies between parts
- Creates the bridge to atomic tasks

**REMOVE ALL**:

- Playwright MCP architecture
- Video generation approach
- Gemini.google.com references
- MCP references

**Practice Vehicle**: Create plan for research paper

- Break paper into sections (intro, lit review, methodology, etc.)
- Identify dependencies (lit review before methodology)
- Define component relationships

**END WITH**: "Try With AI" section running `/sp.plan`

**Frontmatter**:

- title: "Plan Phase"
- chapter: 14, lesson: 6
- duration_minutes: 45
- proficiency_level: "B1"
- cognitive_load: new_concepts: 4

## Response snapshot

**Complete lesson rewritten** (504 lines):

**Structure**:

1. Opening: Hook on plan as bridge between spec and execution
2. The Role of a Plan: Distinguishes spec/plan/tasks progression
3. Cascade Effect: Clear specs → clear plans principle
4. Understanding `/sp.plan` Command: What it does, how it works
5. Plan Structure for Research Paper: Concrete example with 4 components, dependencies, phases
6. Four Key Plan Concepts: Approach, Components, Dependencies, Milestones
7. Executing `/sp.plan`: Command syntax and example output
8. Common Mistakes: Three error patterns
9. Connecting Spec to Plan to Tasks: Shows full chain
10. Try With AI: Four progressive prompts (generate plan, analyze quality, trace requirements, identify decisions)

**Content Highlights**:

- `/sp.plan` command is PRIMARY focus (taught as core skill)
- Research paper used as CONCRETE PRACTICE VEHICLE (not teaching paper writing)
- No prohibited content (zero Playwright, Gemini, MCP, video references)
- Four new concepts taught naturally (not as checklist)
- Cascade effect explicitly taught (vague spec → vague plan problem)
- Try With AI section: 4 prompts with expected outcomes

**Cognitive Load**: 4-5 concepts within B1 limit (7-10 total possible)

## Outcome

- ✅ Impact: Lesson 06 now focuses exclusively on `/sp.plan` command with research paper as simple practice vehicle
- 🧪 Tests: All validation checks pass (content, prohibited terms, structure, learning objectives)
- 📁 Files: Single file modified: 06-plan-phase.md (504 lines, complete rewrite)
- 🔁 Next prompts: Validate Lesson 07 (Tasks Phase) for alignment; review all Chapter 14 lessons for cascade flow
- 🧠 Reflection: Removed 297 lines of video generation content, replaced with clear SDD-RI workflow teaching. Lesson now teachable in 45 minutes.

## Evaluation notes (flywheel)

- Failure modes observed: Previous version had "research-first planning" focused on tool capabilities instead of `/sp.plan` command pedagogy
- Graders run and results (PASS/FAIL): Validation report PASS (all 20+ checks pass)
- Prompt variant: N/A (first attempt successful)
- Next experiment: Cross-validate Lesson 06 connections with Lessons 05 (Clarify) and 07 (Tasks) for coherence
