---
id: 0003
title: "Rewrite Lesson 04 Specify Phase"
stage: "misc"
date: 2025-11-26
feature: "037-chapter-14-research-paper-pivot"
branch: "main"
model: "claude-haiku-4-5"
user: "tutoring-system"
labels: [lesson-implementation, spec-kit-plus, chapter-14]
links:
  spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
files:
  - path: "book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/04-specify-phase.md"
    status: "rewritten"
---

## Prompt

**Task**: Rewrite Lesson 04: Specify Phase for Chapter 14

**Critical Focus**: Teach the `/sp.specify` command. Research paper is just the practice example.

**Learning Objective**: Use `/sp.specify` to write clear specifications with success criteria.

**Concepts to Teach** (max 4):
1. What is a specification (WHAT not HOW)
2. `/sp.specify` command syntax and usage
3. Required sections: Intent, Constraints, Success Evals, Non-Goals
4. SMART criteria for success evals

**Remove ALL**:
- Video specification examples
- Gemini prompts
- MCP references
- Playwright references

**Practice Vehicle**: Write specification for research paper with measurable success criteria, explicit boundaries, and scope definitions.

**END WITH**: "Try With AI" section running `/sp.specify` command

**Frontmatter**:
- title: "Specify Phase"
- chapter: 14, lesson: 4
- duration_minutes: 45
- proficiency_level: "A2-B1"
- cognitive_load: new_concepts: 4

## Response snapshot

Complete rewrite of Lesson 04 focusing on `/sp.specify` command mastery. Key changes:

1. **Removed all problematic content**: video specs, Gemini, MCP, Playwright, YouTube references
2. **Shifted practice vehicle**: from video generation to research paper (zero external dependencies)
3. **Centralized command teaching**: `/sp.specify` command is primary focus, paper is practice example
4. **Added 4-section framework**: Intent, Constraints, Success Evals, Non-Goals with detailed examples
5. **Included SMART criteria**: Framework for writing measurable success evals
6. **3 common mistakes section**: Error patterns with fixes (implementation-leaking, subjective criteria, vague intent)
7. **Specification vs Plan vs Implementation table**: Clarifies WHAT vs HOW vs DO
8. **Progressive practice prompts**: 4 AI prompts in "Try With AI" section with increasing complexity
9. **Transference section**: Explains why research paper teaches specification skills for any domain

## Outcome

- ✅ Impact: Lesson now teaches Spec-Kit Plus `/sp.specify` command without external dependencies; research paper provides concrete, dependency-free practice
- ✅ Files: Complete lesson rewritten to 444 lines
- ✅ Structure: Maintains 45-minute duration, A2-B1 cognitive load (4 new concepts max)
- ✅ Compliance: All requirements met (no video/Gemini/MCP/Playwright content; `/sp.specify` primary focus)
- ✅ Pedagogical: Clear intent→constraints→success evals→non-goals progression with examples and common mistakes
- 🔁 Next prompt: Validate against spec.md requirements and pedagogical frameworks
- 🧠 Reflection: Successfully transformed from tool-specific (video generation) to framework-specific (specification writing) practice. Research paper vehicle eliminates cognitive load from external dependencies while maintaining specification writing skill practice.

## Evaluation notes (flywheel)

- Failure modes avoided:
  - ❌ NOT teaching video generation (requires MCP + Gemini + external APIs)
  - ❌ NOT teaching research writing (shifts focus from Spec-Kit Plus)
  - ❌ NOT mixing tool learning with command learning
- ✅ Constitutional compliance: No pedagogical framework exposed, no meta-commentary, focused on command execution
- ✅ Cognitive load: 4 new concepts (within A2-B1 limit of 7)
- ✅ Lesson independence: Can be completed with only markdown + AI companion (prerequisites met from earlier chapters)
