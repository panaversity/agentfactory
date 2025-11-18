---
id: 001
title: Refine colearning skills spec with focused architecture
stage: spec
date: 2025-10-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-colearning-skills
branch: 001-colearning-skills
user: user
command: null
labels: ["spec", "skills", "architecture", "claude-code", "pedagogy"]
links:
  spec: specs/001-colearning-skills/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-colearning-skills/spec.md
tests:
  - Manual review of spec completeness and consistency
  - Validation against Claude Code skills best practices
---

## Prompt

The spec looks to abstract @specs/001-colearning-skills/spec.md We are building specialized skills for claude to equip it for writing books in this AI disruption era for the new future. First understand the concept of skills: https://docs.claude.com/en/history/claude-code/skills and how to actually build effective skills https://www.anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills and reiterate. you can suggest new skills or edit them as well. These 4 are just a high level.

## Response snapshot

After researching Claude Code skills documentation and Anthropic's engineering blog on agent skills, I identified key issues with the original 4-skill architecture (planning, technical-writing, pedagogy, ai-collaboration):

**Problems Identified:**
- Skills were too broad, covering multiple capabilities each
- Not optimized for model invocation (unclear activation triggers)
- Missing progressive disclosure architecture
- ai-collaboration felt meta (teaching AI usage within AI)

**Refined Architecture - 6 Core Skills:**

1. **learning-objectives.md**: Define measurable outcomes with Bloom's taxonomy alignment
2. **concept-scaffolding.md**: Break down complex concepts with cognitive load management
3. **code-example-generator.md**: Create runnable, pedagogically sound code examples
4. **exercise-designer.md**: Design deliberate practice with evidence-based strategies
5. **technical-clarity.md**: Review explanations for accessibility and completeness
6. **assessment-builder.md**: Create assessments measuring deep understanding

**Optional Enhancement Skills (P2):**
7. **book-architecture.md**: Overall book structure and chapter flow
8. **ai-augmented-teaching.md**: Prepare students for AI-assisted development

**Spec Updates Completed:**
- Rewrote 6 user scenarios (all P1, independently testable)
- Expanded functional requirements from 18 to 33 FRs
- Updated key entities to match skill outputs
- Enhanced success criteria with 15 measurable outcomes
- Clarified scope with progressive disclosure and model-invocation architecture
- Added comprehensive edge cases for varied teaching contexts

## Outcome

- ✅ Impact: Spec now reflects focused, model-invoked skills following Claude Code best practices (progressive disclosure, one capability per skill, unbounded context through layered files)
- 🧪 Tests: Spec validated against Claude Code documentation and Anthropic engineering guidelines
- 📁 Files: Updated specs/001-colearning-skills/spec.md (all sections: user scenarios, requirements, entities, success criteria, scope)
- 🔁 Next prompts: Run /sp.plan to create architectural plan for implementing the 6 core skills, or begin creating actual SKILL.md files with proper frontmatter
- 🧠 Reflection: The refined architecture is significantly more aligned with how Claude Code skills actually work. Moving from broad capabilities (planning, pedagogy) to focused, single-purpose skills (learning-objectives, scaffolding) will improve activation accuracy and context efficiency.

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec assumed user-invoked commands rather than model-invoked skills; did not leverage progressive disclosure for supporting files
- Graders run and results (PASS/FAIL): PASS - Spec now aligns with documented Claude Code skills architecture and Anthropic's design principles
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Implement one complete skill (learning-objectives.md) with supporting files to validate the progressive disclosure pattern works as expected before scaling to all 6 skills
