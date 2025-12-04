---
id: 0002
title: ux-evaluator-skill-creation
stage: misc
date: 2025-12-04
surface: claude-code
model: claude-opus-4-5-20251101
feature: 040-navbar-auth-ux
branch: 040-navbar-auth-ux
user: mjs
command: session-intelligence-harvester + skill-creator
labels: [skill, ux, design-system, harvesting]
links:
  spec: specs/040-navbar-auth-ux/spec.md
  ticket: null
  adr: null
  pr: https://github.com/panaversity/ai-native-software-development/pull/323
files:
  - .claude/skills/ux-evaluator/SKILL.md
  - .claude/skills/ux-evaluator/references/balsamiq-button-principles.md
tests:
  - skill-creator validation: PASS
---

## Prompt

User observed during navbar auth UX evaluation that a repeatable pattern emerged:
1. Gather user feedback + external best practice reference
2. Analyze component against 3 dimensions: Position, Visual Weight, Spacing
3. Compare to industry conventions (GitHub, Stripe, etc.)
4. Produce verdict with prioritized recommendations

User requested: "Use harvester and creator skill as I feel like a subagent or skill getting born for deciding and adding actions on platforms."

## Response snapshot

Created `ux-evaluator` skill using the skill-creator workflow:

1. **Analyzed session** to identify the recurring UX evaluation pattern
2. **Planned skill contents**: SKILL.md with 3-dimension framework, references/ with Balsamiq principles
3. **Initialized skill** using `init_skill.py`
4. **Wrote SKILL.md** with:
   - 3-dimension evaluation framework (Position, Visual Weight, Spacing)
   - Component-specific guidelines (Buttons, Navigation, Forms)
   - Industry conventions reference table
   - Structured output format
   - Self-monitoring checklist
5. **Created reference** `balsamiq-button-principles.md` with extracted principles
6. **Validated skill** - passed skill-creator validation

## Outcome

- ✅ Impact: New reusable skill for systematic UX evaluation
- 🧪 Tests: skill-creator validation PASS
- 📁 Files: 2 created (SKILL.md, balsamiq-button-principles.md)
- 🔁 Next prompts: Invoke `ux-evaluator` skill when evaluating UI components
- 🧠 Reflection: Session intelligence successfully transformed into permanent organizational capability

## Evaluation notes (flywheel)

- Failure modes observed: Initially misread navbar layout (theme toggle position) - corrected by user
- Graders run and results (PASS/FAIL): skill-creator package_skill.py validation PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add Nielsen heuristics reference file for broader UX coverage
