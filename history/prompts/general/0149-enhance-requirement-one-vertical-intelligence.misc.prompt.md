---
id: 0149
title: Enhance Requirement One Vertical Intelligence
stage: misc
date: 2025-11-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: claude/enhance-requirement-one-011CUtAgUMKpFTRTXvK6TYYc
user: Claude
command: none
labels: ["book-content", "chapter-3", "requirement-1", "vertical-intelligence", "sub-agents", "agent-skills"]
links:
  spec: null
  ticket: https://github.com/panaversity/ai-native-software-development/issues/85
  adr: null
  pr: null
files:
  - book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.md
tests:
  - No tests (content enhancement)
---

## Prompt

Carefully review and plan how can we incorporate this

https://github.com/panaversity/ai-native-software-development/issues/85

Ehance first requirement #85

https://ai-native.panaversity.org/docs/Introducing-AI-Driven-Development/billion-dollar-ai/three-requirements#requirement-1-fine-tuned-models-with-domain-expertise

Update this section to: Requirement 1: Increase Domain Expertise with Fine-Tuned Models and/or with Vertical Resuable Intelegence with Sub-agent and Agent Skills

Expand on this in the requirement 1 details

## Response snapshot

Successfully enhanced Requirement 1 in Chapter 3, Lesson 6 to present TWO paths for achieving domain expertise:

**Path 1: Fine-Tuned Models**
- Existing content refined with clear strengths/challenges
- Example: Healthcare AI trained on 100,000 clinical papers
- Best for: Large datasets, ambiguous domains requiring pattern recognition

**Path 2: Vertical Reusable Intelligence with Sub-agents and Agent Skills** (NEW)
- Comprehensive section explaining how domain expertise can be encoded in prompts, workflows, and integration logic
- Sub-agents: Specialized AI assistants for domain-specific tasks
- Agent skills: Reusable capabilities (referenced this book's `.claude/skills/` directory as real example)
- Example: Healthcare AI built with diagnosis sub-agent + treatment planning skill + integration workflows
- Best for: Faster iteration, procedural expertise, limited training data

**Key enhancements:**
1. Updated section title to reflect both paths
2. Added decision criteria framework ("Choosing Your Path")
3. Presented both paths as equally valid (max 2 options rule for beginner content)
4. Updated OpenAI Study Mode analysis to show both approaches could have worked
5. Added new "Try With AI" Prompt 3: "Choose Between Two Paths" for personalized guidance
6. Updated metadata: cognitive load (3→5 concepts, within A2 limit of 7)
7. Updated learning objectives to include "two paths for domain expertise"

**Constitution alignment:**
- ✅ Beginner-friendly language (A1-A2 tier, no jargon without explanation)
- ✅ Cognitive load managed (5 concepts within A2 max of 7)
- ✅ Max 2 options rule followed (fine-tuning OR vertical intelligence)
- ✅ Real examples provided (this book's skills directory, healthcare scenarios)
- ✅ No forward references without context

**File changes:**
- Total additions: 97 lines
- Total deletions: 18 lines
- Net change: +79 lines

## Outcome

- ✅ Impact: Enhanced Chapter 3, Lesson 6 with two paths for domain expertise. Students now understand they can achieve 99% domain expertise through fine-tuning OR vertical intelligence (sub-agents/skills).
- 🧪 Tests: No automated tests (content enhancement). Manual validation: cognitive load check (5 concepts ≤ 7 max), beginner-friendly language verified, examples provided, decision criteria clear.
- 📁 Files: Modified 1 file: `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.md`
- 🔁 Next prompts: Push to remote branch, create pull request with summary of enhancements
- 🧠 Reflection: Successfully expanded Requirement 1 while maintaining beginner-appropriate cognitive load. The "two paths" framing fits the constitution's "max 2 options" rule perfectly. Real-world example (this book's skills directory) provides concrete reference students can explore.

## Evaluation notes (flywheel)

- Failure modes observed: None. Content flowed naturally from existing structure. Both paths are defensible and commonly used in industry.
- Graders run and results (PASS/FAIL): Manual constitution alignment check PASSED (beginner-friendly ✓, cognitive load ✓, max 2 options ✓, real examples ✓)
- Prompt variant (if applicable): N/A (single iteration successful)
- Next experiment (smallest change to try): Consider adding visual diagram showing two paths side-by-side in future iteration. For now, textual explanation with clear examples is sufficient for A1-A2 learners.
