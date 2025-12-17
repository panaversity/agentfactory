# Specification: Lesson 04 — Teach Claude Your Way of Working

**Feature ID**: 043-lesson-04-skills-introduction
**Version**: 1.0.0
**Status**: Draft
**Created**: 2025-12-17
**Author**: Claude Code

---

## 1. Problem Statement

### Current Gap

Chapter 5 currently jumps from setup (Lessons 01-03) directly to technical skill implementation (Lesson 06 "Agent Skills"). Students encounter SKILL.md syntax and three-level architecture without first understanding:

1. **WHY** they should care about teaching Claude their way of working
2. **WHAT** problems skills solve that prompts and CLAUDE.md don't
3. **HOW** to think about encoding expertise (conceptual model before technical syntax)

### User Need

Students completing Lessons 01-03 have Claude Code installed and working. They've experienced basic interactions. They need a conceptual bridge that answers:

- "Why would I create a skill instead of just asking Claude each time?"
- "What makes my way of working worth encoding?"
- "How is this different from just writing better prompts?"

### Common Misconception to Challenge

**The belief**: "Skills are just saved prompts" or "Skills are for developers who write code."

**The correct framing**: Skills encode *reasoning patterns*—the HOW you think about a task, not just WHAT you want done. They capture expertise that would take 20 minutes to explain each time. And they work for any repeated task: writing, research, accountability, learning—not just coding.

---

## 2. Solution Overview

### Lesson Title
**"Teach Claude Your Way of Working"**

### Position in Chapter
Lesson 04 (between "Free Claude Setup" [03] and future lesson [05], before "Agent Skills" [06])

### Pedagogical Layer
**Layer 1 (Manual Foundation)** transitioning to **Layer 2 (AI Collaboration)**

- L1: Understand the conceptual model (why skills exist, what they encode)
- L2: Experience skill activation through conversation (not creation yet)

### Core Insight
You have procedures, preferences, and patterns that make your work distinctive. Every time you explain these to Claude, you're burning time and context. Skills let you explain once and apply forever—across Claude.ai, Claude Code, and API.

---

## 3. Learning Objectives

| # | Objective | Proficiency | Bloom Level | Assessment |
|---|-----------|-------------|-------------|------------|
| 1 | Identify tasks in your workflow where you repeatedly explain the same preferences or procedures | A2 | Analyze | List 3 personal tasks with repeated explanation patterns |
| 2 | Distinguish skills from prompts, CLAUDE.md context, and subagents | A2 | Understand | Explain when to use each mechanism |
| 3 | Understand that skills encode reasoning patterns, not just commands | B1 | Understand | Articulate the difference with an example |
| 4 | Recognize skill activation when Claude applies a skill automatically | A2 | Recognize | Identify when Claude uses a skill vs. base knowledge |
| 5 | Prepare mentally for skill creation by mapping a personal procedure | B1 | Apply | Document a procedure you'd want to encode |

---

## 4. Cognitive Load Analysis

**New Concepts (Target: ≤7 for A2-B1)**:

1. **Skills as encoded expertise** — not just prompts
2. **Reasoning patterns vs. commands** — the WHY matters
3. **Cross-platform portability** — same skill everywhere
4. **Automatic activation** — Claude decides when relevant
5. **Skills vs. alternatives** — prompts, CLAUDE.md, subagents
6. **Institutional knowledge capture** — team-wide benefit

**Count**: 6 concepts ✓ (within limit)

**Scaffolding Strategy**:
- Start with relatable frustration (repeating yourself)
- Use concrete non-coding examples first (writing, notes, accountability)
- Defer SKILL.md syntax entirely (that's Lesson 06)

---

## 5. Narrative Structure

### Emotional Progression
- **Start**: Confused, skeptical ("Skills sound like fancy prompts")
- **Middle**: Curious ("Wait, this solves a real problem I have")
- **End**: Clarity, confidence, momentum ("I know exactly what procedure I'd encode")

### Belief Challenge
**Challenge first**: "You might think skills are just saved prompts—a way to avoid retyping. That's underselling the idea by 90%."

**Correct framing**: Skills encode how you THINK about a task. They capture the decision logic, preferences, and patterns that make your work distinctive. Prompts say "do this." Skills say "here's how to reason about this type of problem."

### Section Flow (One Core Idea Per Section)

1. **The Repetition Problem** — You explain the same things over and over
2. **What Skills Actually Are** — Encoded reasoning patterns, not saved prompts
3. **When Skills Beat Alternatives** — Prompts, CLAUDE.md, subagents comparison
4. **Cross-Platform Portability** — Write once, use everywhere
5. **Skills Beyond Coding** — Accountability, writing, research examples
6. **Preparing to Create** — Map your first procedure (setup for Lesson 06)

---

## 6. Content Constraints

### DO
- Use non-coding examples prominently (writing style, note-taking, accountability)
- Make the reader identify their own repeated procedures
- Create curiosity about HOW skills work (deferred to Lesson 06)
- Reference the dev.to accountability buddy example as real-world proof

### DON'T
- Show SKILL.md syntax (deferred to Lesson 06)
- Explain three-level loading architecture (Lesson 06)
- Create hands-on skill creation exercise (Lesson 06)
- Use summarizing phrases ("in conclusion", "to summarize")

### Section Endings
End each section by opening curiosity, not closing it. Point toward what's next.

---

## 7. Key Differentiator from Lesson 06

| Aspect | Lesson 04 (This Lesson) | Lesson 06 (Agent Skills) |
|--------|-------------------------|--------------------------|
| Focus | WHY skills exist, WHAT they encode | HOW to create skills |
| Examples | Conceptual (accountability, writing) | Technical (blog-planner, study-notes) |
| Syntax | None | SKILL.md format, YAML frontmatter |
| Architecture | Mentioned conceptually | Three-level loading explained |
| Hands-on | Identify your procedures | Create working skill |
| Layer | L1 → L2 (understand, recognize) | L2 (create, refine) |

---

## 8. Source Material

### Primary
- https://claude.com/skills (official landing page)
- User-provided context about Skills

### Secondary
- https://dev.to/yooi/beyond-coding-your-accountability-buddy-with-claude-code-skill-4omh (accountability use case)

### Internal Reference
- Lesson 06 (agent-skills.md) — ensure no duplication, proper handoff

---

## 9. Success Criteria

### Content Quality
- [ ] Each section introduces exactly ONE new core idea
- [ ] No SKILL.md syntax appears in the lesson
- [ ] At least 2 non-coding examples prominently featured
- [ ] Common belief challenged explicitly before correct framing
- [ ] Emotional progression from skeptical → confident → momentum
- [ ] No summarizing phrases ("in conclusion", etc.)
- [ ] Section endings open curiosity

### Learning Validation
- [ ] Reader can list 3 personal procedures they'd want to encode
- [ ] Reader can explain why skills differ from prompts
- [ ] Reader feels prepared and motivated for Lesson 06

---

## 10. Technical Metadata

```yaml
title: "Teach Claude Your Way of Working"
sidebar_position: 4
chapter: 5
lesson: 4
duration_minutes: 15

primary_layer: "Layer 1"
layer_progression: "L1 (Conceptual Foundation) → L2 (Recognition)"

skills:
  - name: "Understanding Skill-Based AI Customization"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Digital Content Creation"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (skills as expertise, reasoning patterns, cross-platform, automatic activation, comparison to alternatives, institutional capture) - within A2-B1 limit ✓"
```

---

## 11. Outline Preview

See `plan.md` for detailed section-by-section outline following these specifications.
