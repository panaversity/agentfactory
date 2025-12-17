---
id: "0001"
title: "Lesson 04 Skills Introduction Implementation"
stage: "green"
feature: "043-lesson-04-skills-introduction"
timestamp: "2025-12-17T12:00:00Z"
model: "claude-opus-4-5-20250114"
tokens_in: ~8000
tokens_out: ~12000
---

# Prompt

Use the SDD loop to work on new lesson 04 in chapter 05. This lesson will introduce skills in claude code.

Title: Teach Claude your way of working

Main Source: https://claude.com/skills

Additional Helpful Material: https://dev.to/yooi/beyond-coding-your-accountability-buddy-with-claude-code-skill-4omh

Create a detailed outline optimized for clarity, logical flow, and narrative momentum. Each section may introduce only ONE new core idea. If additional ideas arise, defer them to later sections.

Assume the reader starts confused and skeptical. By the end, they should feel clarity, confidence, and momentum. Maintain this emotional progression throughout the piece. And "Do not use summarizing phrases such as: "in conclusion", "to summarize", "overall", "in short". End sections by opening curiosity, not closing it."

After writing each section, internally rewrite it to:
- Remove redundancy
- Increase conceptual density
- Preserve length and tone

Finally explicitly challenge the most common belief about this topic before presenting the correct framing.

Before finalizing, internally critique the piece for:
- Generic phrasing
- Shallow explanations
- Missed nuance

Fix all issues before presenting the final version.

# Response Summary

## SDD Artifacts Created

1. **Specification** (`specs/043-lesson-04-skills-introduction/spec.md`)
   - Problem statement: Gap between setup lessons (01-03) and technical skills lesson (06)
   - Learning objectives: 5 objectives at A2-B1 proficiency
   - Cognitive load: 6 concepts (within limit)
   - Key constraint: NO SKILL.md syntax (deferred to Lesson 06)

2. **Plan** (`specs/043-lesson-04-skills-introduction/plan.md`)
   - 8-section outline with one core idea per section
   - Emotional arc: skeptical → curious → confident → momentum
   - Belief challenge: "Skills are just saved prompts" challenged first

3. **Tasks** (`specs/043-lesson-04-skills-introduction/tasks.md`)
   - T1: Main lesson file
   - T2: Summary file
   - T3: Validation
   - T4: PHR

## Lesson Content Created

**Main lesson** (`04-teach-claude-your-way.md`):
- 363 lines
- 10 sections
- No SKILL.md syntax (only preview mention for Lesson 06)
- 26 non-coding example references
- 0 summarizing phrases

**Summary** (`04-teach-claude-your-way.summary.md`):
- Key concepts
- Mental models
- Common mistakes
- Preparation checklist for Lesson 06

## Key Design Decisions

1. **Conceptual focus over technical** - Lesson 04 answers WHY skills matter; Lesson 06 answers HOW to create them
2. **Non-coding prominence** - Accountability buddy, meeting notes, writing, research examples featured prominently
3. **Procedure vs. Prompt distinction** - Core mental model introduced early
4. **Comparison table** - Skills vs. prompts vs. CLAUDE.md vs. subagents
5. **Preparation exercise** - Reader maps their first procedure before Lesson 06

## Validation Results

- [x] No SKILL.md syntax in content (only preview mention)
- [x] 26 non-coding example references
- [x] 0 summarizing phrases
- [x] Belief challenged in opening section (line 82)
- [x] Section endings open curiosity

# Learnings

1. **Positioning matters**: Lesson 04 fills the conceptual gap before Lesson 06's technical content
2. **Non-coding examples sell the concept**: Accountability buddy and meeting notes resonate beyond developers
3. **"Prompts encode WHAT, procedures encode HOW"**: This framing makes the distinction concrete
