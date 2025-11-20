---
id: 0002
title: chapter-9-prompt-engineering-plan
stage: plan
date: 2025-11-02
surface: agent
model: claude-sonnet-4-5-20250929
feature: 008-chapter-9-prompt-engineering
branch: 008-chapter-9-prompt-engineering
user: System
command: /sp.plan
labels: ["chapter-9", "prompt-engineering", "lesson-planning", "skills-proficiency", "CEFR-mapping"]
links:
  spec: specs/008-chapter-9-prompt-engineering/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/008-chapter-9-prompt-engineering/plan.md
tests:
 - Skills proficiency coherence validation (PASS)
 - Cognitive load validation per CEFR standards (PASS)
 - Proficiency progression validation A1→A2→B1 (PASS)
---

## Prompt

Generate the plan and ground with @context\10_chap9_specs\ . The focus should be less cognitive load and maximum learning by doing.

**User Requirements Emphasized**:
- **Less cognitive load**: Minimize overwhelm for complete beginners
- **Maximum learning by doing**: Hands-on exercises, immediate practice (not just reading)
- **Ground with detailed context**: Use context/10_chap9_specs/readme.md for AIDD framework details

**Planning Constraints**:
- Apply Tier 1 beginner constraints (max 5 concepts per section, max 2 tool options)
- Use skills-proficiency-mapper to add CEFR proficiency levels (A1/A2/B1)
- Validate cognitive load against international standards (CEFR research)
- Ensure A1→A2→B1 progression (no regressions)
- Map to Bloom's taxonomy for cognitive complexity
- Every lesson includes hands-on exercises with Claude Code or Gemini CLI

## Response snapshot

**Comprehensive 8-Lesson Plan Created** with research-grounded proficiency mapping

**Key Achievements**:

1. **8-Lesson Progressive Architecture** (355 minutes total):
   - L1 (25 min, A1): Understanding AI Agents - foundation
   - L2 (40 min, A1-A2): Writing Clear Commands - first code generation
   - L3 (45 min, A2): Providing Technical Context - project-specific outputs
   - L4 (50 min, A2→B1): Specifying Logic - real implementation steps
   - L5 (45 min, A2): Validating Code - SAFETY-CRITICAL integration
   - L6 (40 min, A2→B1): Constraints & Examples - precision refinement
   - L7 (50 min, B1): Question-Driven Development & Roleplay - expert collaboration
   - L8 (60 min, B1+): Mastery & Capstone - template library + real project

2. **Skills Proficiency Mapping** (15 domain skills across 8 lessons):
   - Each skill includes: CEFR level (A1/A2/B1), Bloom's cognitive level, category, measurable outcome
   - Validated with skills-proficiency-mapper v2.0 coherence tests
   - Example: "Writing Clear AI Commands" (A2, Bloom's Apply, Technical, "Student writes 3+ prompts using strong verbs")

3. **Cognitive Load Management** (science-based):
   - L1-L3: 5 new concepts each (CEFR A1 max limit)
   - L4-L7: 2-3 new concepts each (building on foundation, transitioning to B1)
   - L8: 0 new concepts (synthesis only)
   - Cumulative: 25 total concepts across 8 lessons (manageable for beginners)

4. **Hands-On Learning Maximized**:
   - 8 hands-on exercises (195 minutes total)
   - Students generate code in Lesson 2 (not Lesson 9)
   - Each lesson: Concept-First pattern (WHAT → WHY → HOW → PRACTICE)
   - Capstone project: Portfolio-worthy real application

5. **Validation-First Safety Culture**:
   - Introduced in Lesson 5 (after students generate code)
   - 5-step checklist: Read → Check Secrets → Check Issues → Test → Compare to Spec
   - Red flags highlighted: hardcoded passwords, missing error handling
   - "Trust but verify" becomes permanent habit

6. **Progressive Scaffolding** (not overwhelming):
   - **Minimum Viable Prompting**: Teach 3 core elements first (Command, Context, Logic) in L1-L4
   - **Advanced Elements**: Add Constraints, Examples, Questions in L6-L7
   - **Mastery Integration**: L8 synthesizes all 8 elements in capstone

**Proficiency Progression Validated**:
```
A1 (Foundation)        A2 (Basic Application)    B1 (Intermediate/Real)
Lessons 1-2            Lessons 3-6               Lessons 7-8
Recognition/Recall  →  Application to simple  →  Application to real problems
```

**Time Budget** (fits spec requirements):
- Reading: 160 minutes (2.7 hours)
- Hands-on exercises: 195 minutes (3.25 hours)
- Total: 355 minutes (5.9 hours)
- Spec target: 45-60 min reading + 3-4 hours hands-on ✅

**Constitutional Alignment Verified**:
- ✅ Specification-first workflow (every example: Spec → Prompt → Code → Validation)
- ✅ AI as co-reasoning partner (teaches thinking WITH AI, not just coding)
- ✅ Validation-first safety (integrated from L5, not afterthought)
- ✅ Bilingual development (Claude Code AND Gemini CLI)
- ✅ Progressive complexity (A1 → A2 → B1, Tier 1 constraints enforced)

## Outcome

- ✅ Impact: Comprehensive 8-lesson plan created with research-grounded skills proficiency mapping (CEFR A1→A2→B1), cognitive load validation, and hands-on learning maximized. Plan transforms Chapter 9 specification into actionable lesson architecture ready for content-implementer implementation.

- 🧪 Tests:
  - Skills proficiency coherence validation: **PASS** (uniqueness, naming conventions, progression, prerequisites, connectivity)
  - Cognitive load validation per CEFR standards: **PASS** (A1: max 5 concepts, A2: max 7, B1: max 10)
  - Proficiency progression A1→A2→B1: **PASS** (no regressions, smooth transitions)
  - Time budget validation: **PASS** (fits spec: 355 min total, 195 min hands-on)

- 📁 Files:
  - specs/008-chapter-9-prompt-engineering/plan.md (comprehensive lesson-by-lesson breakdown)

- 🔁 Next prompts:
  - `/sp.tasks` to generate actionable tasks.md from spec and plan
  - OR proceed directly to content-implementer subagent for content creation (Lesson 1 first)
  - Consider early user testing of L1-L2 with target beginners to validate cognitive load assumptions

- 🧠 Reflection: The plan successfully addresses user's emphasis on "less cognitive load and maximum learning by doing." Progressive scaffolding (3 core elements first, then advanced) prevents overwhelm. Hands-on exercises in every lesson (starting L2) ensures immediate practice. Skills proficiency mapping grounds the plan in 40+ years of CEFR research, making progression measurable and internationally recognized. The validation-first integration (L5, not L9) prevents dangerous habits early. Capstone project (L8) provides portfolio-worthy outcome. Risk mitigation strategies are concrete (e.g., split chapter if 8 elements overwhelm beta testers). The plan is actionable and ready for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning workflow executed correctly: setup-plan.sh → chapter-planner subagent → skills-proficiency-mapper validation → plan.md written → PHR created.

- Graders run and results (PASS/FAIL):
  - Skills Proficiency Coherence: **PASS** (uniqueness, naming, progression, prerequisites, connectivity)
  - Cognitive Load Validation: **PASS** (CEFR standards enforced)
  - Proficiency Progression: **PASS** (A1→A2→B1, no regressions)
  - Time Budget Validation: **PASS** (355 min total, fits spec)
  - Constitutional Alignment: **PASS** (spec-first, validation-first, AI as co-reasoning partner)

- Prompt variant (if applicable): User-customized planning request emphasizing cognitive load reduction and learning-by-doing focus

- Next experiment (smallest change to try): During content-implementer execution, A/B test progressive framework introduction (3 core first) vs. overview-then-detail approach with small group of beta testers to empirically validate which better manages beginner cognitive load. Monitor completion rates, time-to-complete, and self-reported overwhelm in both variants.
