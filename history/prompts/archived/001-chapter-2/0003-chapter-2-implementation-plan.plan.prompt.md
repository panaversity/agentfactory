---
id: 0003
title: Chapter 2 Implementation Plan
stage: plan
date: 2025-10-29
surface: agent
model: claude-sonnet-4-5
feature: 001-chapter-2
branch: 001-chapter-2
user: panaversity
command: /sp.plan
labels: ["chapter-2", "lesson-planning", "pedagogical-design", "nine-revolutions", "ai-tools"]
links:
  spec: specs/001-chapter-2/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-chapter-2/plan.md
tests:
  - null
---

## Prompt

Plan using chapter-writer subagent and the domain skills and all the ground context available. We will use simple english and keep the lessons concise to keep readers engaged.

## Response snapshot

Created comprehensive lesson-by-lesson implementation plan for Chapter 2 with **4 lessons** totaling 4,500-5,400 words:

**Lesson Structure**:
1. **Lesson 1: "The AI Inflection Point"** (1,000-1,200 words) - Hook with ICPC World Finals, mainstream adoption evidence, capability milestones
2. **Lesson 2: "The Nine Technological Revolutions"** (1,500-1,800 words) - 9 revolutions grouped by layer (3+3+3), three-layer stack, Snakes and Ladders pattern
3. **Lesson 3: "The New Wave of AI Coding Agents"** (1,200-1,400 words) - Claude Code, Gemini CLI, OpenAI Codex, Qwen Code comparison with tool selection framework
4. **Lesson 4: "Vibe Coding vs. Spec-Driven Development"** (800-1,000 words) - Methodological distinction, DORA perspective, Team A vs. Team B example

**Constitution Check**: ✅ PASS on all 11 principles (9 applicable, 2 N/A for conceptual chapter)

**Pedagogical Approach**:
- User guidance integrated: Simple English, concise lessons, engaging content
- Maximum 3-4 key concepts per lesson (cognitive load management)
- 15 learning objectives across 4 lessons (mapped to 8 domain skills)
- 9 visual assets specified with alt text requirements
- Progressive disclosure with "Why This Matters" boxes and Quick Checks

**Source Material Integration**:
- Lesson 1: readme1.md sections 1-2 (evidence-based framing)
- Lesson 2: readme1.md sections 2-3 + readme.md Part I (9 revolutions)
- Lesson 3: readme2.md (tool-specific sections)
- Lesson 4: readme1.md section 4 + 6 (vibe coding vs. SDD, DORA)

**Content Quality Standards**:
- Language & Clarity: Zero gatekeeping, jargon defined, evidence-based tone
- Cognitive Load: 3-4 concepts max, visual aids, clear section headings
- Pedagogical Consistency: Show-then-explain, evidence before claims
- Accessibility: Alt text for ALL images, high contrast, text fallbacks
- Assessment: Quick Checks, reflection prompts, 3 end-of-chapter exercises

**Success Metrics**: 10 measurable outcomes targeting 80-90% reader comprehension

**Stakeholder Decisions Implemented** (from spec Q1-Q5):
- Hybrid approach (accessible narrative + "Deep Dive" boxes)
- Focus on Claude Code & Gemini CLI (reduce cognitive load)
- Enforce 5,000-word limit (respect Part 1 philosophy)
- All three optional exercises (maximum learning, reader autonomy)
- Brief vibe coding mention (prevent bad habits, full treatment in Part 5)

## Outcome

- ✅ Impact: Comprehensive 4-lesson plan ready for task generation and lesson writing; clear roadmap from specification to implementation with constitution alignment validated
- 🧪 Tests: 10 success criteria defined with validation methods; Quick Checks and assessments integrated into each lesson
- 📁 Files: Created specs/001-chapter-2/plan.md (approximately 15,000 words of detailed planning guidance)
- 🔁 Next prompts: (1) /sp.tasks to generate implementation checklist, (2) Invoke content-implementer subagent with plan, (3) Invoke validation-auditor after completion
- 🧠 Reflection: Plan successfully adapts software implementation template to educational content planning; lesson structure balances depth (9 revolutions + 4 tools) with accessibility (simple English, concise, engaging per user guidance); domain skills application matrix ensures all 8 skills are systematically applied

## Evaluation notes (flywheel)

- Failure modes observed: None; plan structure comprehensively addresses spec requirements and user guidance
- Graders run and results (PASS/FAIL): Constitution Check passed all 11 principles; content quality standards validated against Part 1 spec requirements
- Prompt variant (if applicable): Standard /sp.plan adapted for educational content (lessons vs. code implementation)
- Next experiment (smallest change to try): Consider whether 4 lessons could be condensed to 3 if word count targets prove challenging, or whether visual assets should be created before or during lesson writing
